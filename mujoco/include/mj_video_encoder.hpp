#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

namespace mujoco {

enum class VideoCodec {
  AV1,
  H264,
  PNG
};

/**
 * @class VideoEncoder
 * @brief Encodes RGB frames to video files using FFmpeg
 */
class VideoEncoder {
public:
  explicit VideoEncoder(VideoCodec codec = VideoCodec::AV1, int fps = 30, int crf = 30);
  ~VideoEncoder();

  VideoEncoder(const VideoEncoder &) = delete;
  VideoEncoder &operator=(const VideoEncoder &) = delete;

  void open(const std::filesystem::path &outputPath, int width, int height);
  void writeFrame(const uint8_t *rgbData, int width, int height);
  void close();

  bool isOpen() const;
  static bool isCodecAvailable(VideoCodec codec);
  static std::string codecName(VideoCodec codec);

private:
  void cleanup();

  VideoCodec codec_;
  int fps_;
  int crf_;
  int width_{0};
  int height_{0};
  bool isOpen_{false};
  int64_t pts_{0};

  AVFormatContext *formatCtx_{nullptr};
  AVCodecContext *codecCtx_{nullptr};
  AVStream *stream_{nullptr};
  AVFrame *frame_{nullptr};
  AVPacket *packet_{nullptr};
  SwsContext *swsCtx_{nullptr};
};

} // namespace mujoco
