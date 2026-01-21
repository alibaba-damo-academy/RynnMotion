import os
import subprocess
import platform
import sys
import argparse
import re

from PIL import Image
from enum import Enum


class platform_type(Enum):
    JETSON = 1
    RK = 2
    RDK = 3
    UNKNOWN = 4


def run_cmd(cmd):
    """执行系统命令并返回输出内容"""
    try:
        result = subprocess.run(
            cmd, shell=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        return result.stdout.strip()
    except Exception as e:
        return f"Error: {str(e)}"


def check_gstreamer_installed():
    """检查 GStreamer 是否已安装"""
    try:
        subprocess.run(
            ["gst-launch-1.0", "--version"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return True
    except FileNotFoundError:
        return False


def is_jetson():
    """判断是否为 NVIDIA Jetson 平台"""
    if not os.path.exists("/sys/firmware/devicetree/base/model"):
        return False
    try:
        devtree = run_cmd("cat /sys/firmware/devicetree/base/model")
        if "NVIDIA" in devtree or "Jetson" in devtree:
            return True
    except Exception:
        pass

    return False


def is_rockchip():
    """判断是否为 瑞芯微(Rockchip)平台"""
    if not os.path.exists("/sys/firmware/devicetree/base/model"):
        return False
    try:
        devtree = run_cmd("cat /sys/firmware/devicetree/base/model")
        if "Rockchip" in devtree or "RK" in devtree:
            return True
    except Exception:
        pass

    return False


def is_RDK():
    """判断是否为 地瓜(RDK)平台"""
    if not os.path.exists("/sys/firmware/devicetree/base/model"):
        return False
    try:
        tree = open("/sys/firmware/devicetree/base/model", "r", encoding="utf-8").read()
        if "X5" in tree or "S100" in tree:
            return True
    except Exception:
        pass

    return False


def detect_platform():
    """综合判断平台类型"""
    jetson = is_jetson()
    rk = is_rockchip()
    rdk = is_RDK()

    if jetson:
        print("NVIDIA Jetson")
        return platform_type.JETSON
    elif rk:
        print("Rockchip (RK)")
        return platform_type.RK
    elif rdk:
        print("RDK")
        return platform_type.RDK
    else:
        print("Unknown or mixed platform")
        return platform_type.UNKNOWN


def get_png_resolution(file_path):
    try:
        with Image.open(file_path) as img:
            # 获取图片尺寸（宽度, 高度）
            width, height = img.size
            return width, height
    except Exception as e:
        print(f"读取图片失败: {e}")
        return None


def get_match_files(file_path, image_pattern):
    base_pattern = os.path.splitext(image_pattern)[0].replace("%06d", "")
    matching_files = []

    # 1. 查找所有匹配的文件
    for fname in os.listdir(file_path):
        if fname.startswith(base_pattern) and fname.endswith(".png"):
            matching_files.append(fname)

    if not matching_files:
        print(f"⚠️ 警告: 目录 '{file_path}' 中未找到匹配的图片: {image_pattern}")
        return None

    return matching_files


def create_video_from_images(file_path, image_pattern, output_file, framerate=30):
    """将图片序列转换为 MP4 视频"""
    # 构建完整的图片路径模式
    full_image_pattern = os.path.join(file_path, image_pattern)

    # 检查目录是否存在
    if not os.path.exists(file_path):
        print(f"❌ 错误: 目录 '{file_path}' 不存在")
        return False

    # 检查目录中是否有匹配的图片
    matching_files = get_match_files(file_path, image_pattern)
    if matching_files is None:
        return False

    # 检查图片分辨率
    image_w, image_h = get_png_resolution(os.path.join(file_path, matching_files[0]))
    resolution = image_w * image_h
    bps = 4000000
    if resolution >= 3840 * 2160:
        bps = 16000000
    elif resolution >= 1920 * 1080:
        bps = 8000000
    elif resolution >= 1080 * 720:
        bps = 6000000
    elif resolution >= 640 * 360:
        bps = 4000000
    else:
        bps = 2000000

    # 检查部署平台
    device_type = detect_platform()

    # 确定编码管道
    if device_type is platform_type.JETSON:
        print("✅ 检测到当前为 NVIDIA 平台，使用硬件编码")
        pipeline = (
            f'multifilesrc location="{full_image_pattern}" caps="image/png,framerate={framerate}/1" ! '
            "pngdec ! "
            "videoconvert ! "
            "nvvidconv ! "
            f"nvv4l2h264enc bitrate={bps} ! "
            "h264parse config-interval=1 ! "
            f"mp4mux streamable=true ! filesink location={output_file} -e"
        )
    elif device_type is platform_type.RK:
        print("⚠️ 检测到当前为 RK 平台，使用硬件编码")
        pipeline = (
            f'multifilesrc location="{full_image_pattern}" caps="image/png,framerate={framerate}/1" ! '
            "pngdec ! queue max-size-buffers=20 ! "
            "videoconvert ! queue max-size-buffers=20 ! video/x-raw,format=NV12 ! "
            f"mpph264enc bps={bps} ! "
            "h264parse ! "
            f"mp4mux streamable=true ! filesink location={output_file} -e"
        )
    elif device_type is platform_type.RDK:
        print("⚠️ 检测到当前为 RDK 平台，使用硬件编码")
        pipeline = (
            f'multifilesrc location="{full_image_pattern}" caps="image/png,framerate={framerate}/1" ! '
            "pngdec ! queue max-size-buffers=20 ! "
            "videoconvert n-threads=6 ! queue max-size-buffers=20 ! video/x-raw,format=I420 ! "
            f'v4l2h264enc extra-controls="encode,frame_level_rate_control_enable=1,video_bitrate={bps};" ! '
            "h264parse ! "
            f"mp4mux streamable=true ! filesink location={output_file} -e"
        )
    elif device_type is platform_type.UNKNOWN:
        print("⚠️ 检测到当前为非脚本支持平台，默认使用软件编码")
        return False
        # pipeline = (
        #     f'multifilesrc location="{full_image_pattern}" caps="image/png,framerate={framerate}/1" ! '
        #     "pngdec ! "
        #     "videoconvert ! "
        #     "x264enc ! "  # 使用软件 H.264 编码器
        #     "h264parse ! "
        #     f"mp4mux streamable=true ! filesink location={output_file} -e"
        # )

    # 构建完整的 gst-launch 命令
    command = f"gst-launch-1.0 -v {pipeline}"
    print("执行命令:", command)

    # 运行命令
    try:
        subprocess.run(command, shell=True, check=True)
        print(f"✅ 视频创建成功: {output_file}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ 视频创建失败: {e}")
        return False
    except Exception as e:
        print(f"❌ 发生错误: {str(e)}")
        return False


if __name__ == "__main__":
    if not check_gstreamer_installed():
        print("错误: gst-launch-1.0 未找到，请确保已安装 GStreamer")
        sys.exit(1)

    # 设置命令行参数解析
    parser = argparse.ArgumentParser(description="将图片序列转换为 MP4 视频")
    parser.add_argument("--file_path", required=True, help="包含图片的文件夹路径")
    parser.add_argument(
        "--image_pattern",
        default="frame_%06d.png",
        help="图片命名模式 (例如: frame_%%06d.png)",
    )
    parser.add_argument("--output_file", required=True, help="输出视频文件路径")
    parser.add_argument("--framerate", type=int, default=30, help="视频帧率 (默认: 30)")

    args = parser.parse_args()

    # 创建视频
    create_video_from_images(
        file_path=args.file_path,
        image_pattern=args.image_pattern,
        output_file=args.output_file,
        framerate=args.framerate,
    )

