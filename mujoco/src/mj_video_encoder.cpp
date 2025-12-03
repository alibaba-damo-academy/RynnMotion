#include "mj_video_encoder.hpp"

#include <stdexcept>

namespace mujoco {

VideoEncoder::VideoEncoder(VideoCodec codec, int fps, int crf)
    : codec_(codec), fps_(fps), crf_(crf) {}

VideoEncoder::~VideoEncoder() {
  close();
}

std::string VideoEncoder::codecName(VideoCodec codec) {
  switch (codec) {
    case VideoCodec::AV1:
      if (avcodec_find_encoder_by_name("libsvtav1")) {
        return "libsvtav1";
      }
      return "libaom-av1";
    case VideoCodec::H264:
      return "libx264";
    case VideoCodec::PNG:
      return "png";
    default:
      return "libaom-av1";
  }
}

bool VideoEncoder::isCodecAvailable(VideoCodec codec) {
  const AVCodec *avCodec = avcodec_find_encoder_by_name(codecName(codec).c_str());
  return avCodec != nullptr;
}

void VideoEncoder::open(const std::filesystem::path &outputPath, int width, int height) {
  close();

  av_log_set_level(AV_LOG_ERROR);

  if (codec_ == VideoCodec::PNG) {
    return;  // PNG stub - not implemented
  }

  width_ = width;
  height_ = height;

  std::filesystem::create_directories(outputPath.parent_path());

  int ret = avformat_alloc_output_context2(&formatCtx_, nullptr, nullptr, outputPath.c_str());
  if (ret < 0 || !formatCtx_) return;

  const AVCodec *codec = avcodec_find_encoder_by_name(codecName(codec_).c_str());
  if (!codec) {
    cleanup();
    return;
  }

  stream_ = avformat_new_stream(formatCtx_, nullptr);
  if (!stream_) {
    cleanup();
    return;
  }

  codecCtx_ = avcodec_alloc_context3(codec);
  if (!codecCtx_) {
    cleanup();
    return;
  }

  codecCtx_->width = width;
  codecCtx_->height = height;
  codecCtx_->time_base = AVRational{1, fps_};
  codecCtx_->framerate = AVRational{fps_, 1};
  codecCtx_->pix_fmt = AV_PIX_FMT_YUV420P;
  codecCtx_->gop_size = 2;

  if (codec_ == VideoCodec::AV1) {
    av_opt_set(codecCtx_->priv_data, "crf", std::to_string(crf_).c_str(), 0);
    av_opt_set(codecCtx_->priv_data, "preset", "8", 0);
  } else if (codec_ == VideoCodec::H264) {
    av_opt_set(codecCtx_->priv_data, "crf", std::to_string(crf_).c_str(), 0);
    av_opt_set(codecCtx_->priv_data, "preset", "ultrafast", 0);
    av_opt_set(codecCtx_->priv_data, "tune", "zerolatency", 0);
  }

  if (formatCtx_->oformat->flags & AVFMT_GLOBALHEADER) {
    codecCtx_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
  }

  ret = avcodec_open2(codecCtx_, codec, nullptr);
  if (ret < 0) {
    cleanup();
    return;
  }

  ret = avcodec_parameters_from_context(stream_->codecpar, codecCtx_);
  if (ret < 0) {
    cleanup();
    return;
  }

  stream_->time_base = codecCtx_->time_base;

  frame_ = av_frame_alloc();
  if (!frame_) {
    cleanup();
    return;
  }

  frame_->format = codecCtx_->pix_fmt;
  frame_->width = width;
  frame_->height = height;

  ret = av_frame_get_buffer(frame_, 0);
  if (ret < 0) {
    cleanup();
    return;
  }

  packet_ = av_packet_alloc();
  if (!packet_) {
    cleanup();
    return;
  }

  swsCtx_ = sws_getContext(width, height, AV_PIX_FMT_RGB24,
                           width, height, AV_PIX_FMT_YUV420P,
                           SWS_BILINEAR, nullptr, nullptr, nullptr);
  if (!swsCtx_) {
    cleanup();
    return;
  }

  ret = avio_open(&formatCtx_->pb, outputPath.c_str(), AVIO_FLAG_WRITE);
  if (ret < 0) {
    cleanup();
    return;
  }

  ret = avformat_write_header(formatCtx_, nullptr);
  if (ret < 0) {
    cleanup();
    return;
  }

  isOpen_ = true;
  pts_ = 0;
}

void VideoEncoder::writeFrame(const uint8_t *rgbData, int width, int height) {
  if (!isOpen_ || !rgbData) return;
  if (width != width_ || height != height_) return;

  int ret = av_frame_make_writable(frame_);
  if (ret < 0) return;

  const uint8_t *srcSlice[1] = {rgbData};
  int srcStride[1] = {3 * width};

  sws_scale(swsCtx_, srcSlice, srcStride, 0, height,
            frame_->data, frame_->linesize);

  frame_->pts = pts_++;

  ret = avcodec_send_frame(codecCtx_, frame_);
  if (ret < 0) return;

  while (ret >= 0) {
    ret = avcodec_receive_packet(codecCtx_, packet_);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) break;
    if (ret < 0) return;

    av_packet_rescale_ts(packet_, codecCtx_->time_base, stream_->time_base);
    packet_->stream_index = stream_->index;

    av_interleaved_write_frame(formatCtx_, packet_);
    av_packet_unref(packet_);
  }
}

void VideoEncoder::close() {
  if (!isOpen_) return;

  int ret = avcodec_send_frame(codecCtx_, nullptr);
  while (ret >= 0) {
    ret = avcodec_receive_packet(codecCtx_, packet_);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) break;
    if (ret < 0) break;

    av_packet_rescale_ts(packet_, codecCtx_->time_base, stream_->time_base);
    packet_->stream_index = stream_->index;

    av_interleaved_write_frame(formatCtx_, packet_);
    av_packet_unref(packet_);
  }

  if (formatCtx_ && formatCtx_->pb) {
    av_write_trailer(formatCtx_);
  }

  cleanup();
  isOpen_ = false;
}

bool VideoEncoder::isOpen() const {
  return isOpen_;
}

void VideoEncoder::cleanup() {
  if (swsCtx_) {
    sws_freeContext(swsCtx_);
    swsCtx_ = nullptr;
  }
  if (packet_) {
    av_packet_free(&packet_);
    packet_ = nullptr;
  }
  if (frame_) {
    av_frame_free(&frame_);
    frame_ = nullptr;
  }
  if (codecCtx_) {
    avcodec_free_context(&codecCtx_);
    codecCtx_ = nullptr;
  }
  if (formatCtx_) {
    if (formatCtx_->pb) {
      avio_closep(&formatCtx_->pb);
    }
    avformat_free_context(formatCtx_);
    formatCtx_ = nullptr;
  }
  stream_ = nullptr;
}

} // namespace mujoco
