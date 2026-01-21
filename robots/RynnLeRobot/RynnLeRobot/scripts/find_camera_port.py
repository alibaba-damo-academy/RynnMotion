#!/usr/bin/env python
# Copyright 2025 Alibaba Damo Academy. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Find available camera devices.

Usage:
    $ find-camera-port
"""

import argparse
import concurrent.futures
import logging
import time
import sys
import platform
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

# Add parent directory to Python path
sys.path.append(str(Path(__file__).parent.parent))

from RynnLeRobot.hardware.cameras.configs import ColorMode
from RynnLeRobot.hardware.cameras.opencv.camera_opencv import OpenCVCamera
from RynnLeRobot.hardware.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from RynnLeRobot.scripts.lang import select_language, t

logger = logging.getLogger(__name__)


class I18nMessages:
    def __init__(self, language='EN'):
        self.lang = language
        self.messages = {
            'EN': {
                'hardcoded_detection_title': "🔍 Hardcoded camera detection - using platform-specific camera assignment...",
                'found_ubuntu_cameras': "Found {count} external cameras on Ubuntu",
                'assigned_front_camera': "✅ Assigned front camera: {port}",
                'assigned_wrist_camera': "✅ Assigned wrist camera: {port}",
                'not_enough_ubuntu_cameras': "⚠️ Only found {count} external cameras, need at least 2",
                'found_macos_cameras': "Found {count} cameras on macOS",
                'hardcoded_failed': "⚠️ Hardcoded detection failed, falling back to manual detection...",
                'using_manual_detection': "📋 Using manual plug/unplug detection method...",
                'unplug_prompt': "Please make sure the {camera_name} camera is UNPLUGGED and press Enter to continue.",
                'plug_prompt': "Please connect the {camera_name} camera and press Enter to continue.",
                'detected_camera_id': "The camera ID of this {camera_name} camera is '{port}'",
                'detected_camera_env': "DETECTED_{camera_name}_CAMERA={port}",
                'camera_detection_title': "🎯 Camera Detection",
                'detection_summary': "Port Detection Summary:",
                'detection_completed': "✅ Camera detection completed using hardcoded assignment",
                'detected_cameras_title': "--- Detected Cameras ---",
                'camera_number': "Camera #{index}:",
                'camera_property': "  {property}:",
                'camera_sub_property': "    {property}: {value}",
                'property_separator': "-" * 20,
                'error_detecting_camera': "Error detecting {camera_name} camera: {error}",
                'camera_not_found': "DETECTED_{camera_name}_CAMERA=NOT_FOUND",
                'saving_test_images': "Saving test images from detected cameras",
                'finalizing_saving': "\nFinalizing image saving...",
                'capture_finished': "Image capture finished. Images saved to {path}",
                'detection_title': "Camera Detection Summary:",
                'front_camera': "Front camera port: ",
                'wrist_camera': "Wrist camera port: ",
                'not_detected_skipped': "Not detected or skipped",
                'detection_separator': "=" * 50,
                'no_cameras_detected': "No cameras were detected.",
                'camera_detection_header': "Camera Detection",
                'press_enter': "Press Enter to continue...",
                'saving_images_to': "Saving images to {path}",
                'record_time': "Recording for {time:.1f} seconds",
                'camera_connection': "Connecting to camera: {id}",
                'camera_disconnection': "Disconnecting {count} cameras...",
                'camera_capture_started': "Starting image capture for {time:.1f} seconds from {count} cameras.",
                'camera_test': "Testing camera: {id}",
                'camera_test_success': "✓ Camera {id} test successful",
                'camera_test_failed': "✗ Camera {id} test failed: {error}",
                'camera_listing': "Listing available cameras...",
                'camera_listing_complete': "Camera listing completed.",
                'camera_listing_none': "No cameras found.",
            },
            'CN': {
                'hardcoded_detection_title': "🔍 硬编码相机检测 - 使用平台特定的相机分配...",
                'found_ubuntu_cameras': "在 Ubuntu 上找到 {count} 个外部相机",
                'assigned_front_camera': "✅ 已分配正面相机: {port}",
                'assigned_wrist_camera': "✅ 已分配腕部相机: {port}",
                'not_enough_ubuntu_cameras': "⚠️ 仅找到 {count} 个外部相机，至少需要 2 个",
                'found_macos_cameras': "在 macOS 上找到 {count} 个相机",
                'hardcoded_failed': "⚠️ 硬编码检测失败，回退到手动检测...",
                'using_manual_detection': "📋 使用手动插拔检测方法...",
                'unplug_prompt': "请确保 {camera_name} 相机已断开连接，然后按 Enter 继续。",
                'plug_prompt': "请连接 {camera_name} 相机，然后按 Enter 继续。",
                'detected_camera_id': "{camera_name} 相机的 ID 为: '{port}'",
                'detected_camera_env': "DETECTED_{camera_name}_CAMERA={port}",
                'camera_detection_title': "🎯 相机检测",
                'detection_summary': "相机检测摘要:",
                'detection_completed': "✅ 已使用硬编码分配完成相机检测",
                'detected_cameras_title': "--- 已检测到的相机 ---",
                'camera_number': "相机 #{index}:",
                'camera_property': "  {property}:",
                'camera_sub_property': "    {property}: {value}",
                'property_separator': "-" * 20,
                'error_detecting_camera': "检测 {camera_name} 相机时出错: {error}",
                'camera_not_found': "DETECTED_{camera_name}_CAMERA=未找到",
                'saving_test_images': "正在从检测到的相机保存测试图像",
                'finalizing_saving': "\n正在完成图像保存...",
                'capture_finished': "图像捕获已完成。图像已保存至 {path}",
                'detection_title': "相机检测摘要:",
                'front_camera': "正面相机端口: ",
                'wrist_camera': "腕部相机端口: ",
                'not_detected_skipped': "未检测到或已跳过",
                'detection_separator': "=" * 50,
                'no_cameras_detected': "未检测到相机。",
                'camera_detection_header': "相机检测",
                'press_enter': "按 Enter 继续...",
                'saving_images_to': "正在将图像保存至 {path}",
                'record_time': "正在录制 {time:.1f} 秒",
                'camera_connection': "正在连接相机: {id}",
                'camera_disconnection': "正在断开 {count} 个相机的连接...",
                'camera_capture_started': "开始从 {count} 个相机捕获图像，持续 {time:.1f} 秒。",
                'camera_test': "正在测试相机: {id}",
                'camera_test_success': "✓ 相机 {id} 测试成功",
                'camera_test_failed': "✗ 相机 {id} 测试失败: {error}",
                'camera_listing': "正在列出可用相机...",
                'camera_listing_complete': "相机列表已完成。",
                'camera_listing_none': "未找到相机。",
            }
        }

    def get(self, key, **kwargs):
        """获取指定键的翻译消息"""
        message = self.messages[self.lang].get(key, key)
        return message.format(**kwargs)

    def get_with_separator(self, key, **kwargs):
        """获取带分隔线的消息"""
        return f"{self.get(key, **kwargs)}\n{self.get('detection_separator')}"


def filter_ubuntu_external_cameras(camera_list: list[dict[str, Any]], i18n: I18nMessages) -> list[dict[str, Any]]:
    """
    Filter cameras for Ubuntu laptops to get only external cameras.
    Ubuntu laptops typically have:
    - /dev/video0, /dev/video2: Built-in cameras (webcam, infrared)
    - /dev/video4, /dev/video6: External USB cameras (front, wrist)

    Returns the external cameras (higher numbered video devices).
    """
    if platform.system() != "Linux":
        return camera_list

    external_cameras = []
    for cam in camera_list:
        cam_id = str(cam["id"])
        if "/dev/video" in cam_id:
            # Extract the video device number
            try:
                video_num = int(cam_id.replace("/dev/video", ""))
                # Consider cameras with video4+ as external cameras
                if video_num >= 4:
                    external_cameras.append(cam)
            except ValueError:
                # If we can't parse the number, include it anyway
                external_cameras.append(cam)
        else:
            # Not a /dev/video device, include it
            external_cameras.append(cam)

    logger.info(t("filtered_cameras", filtered=len(external_cameras), total=len(camera_list)))
    return external_cameras


def find_all_opencv_cameras(filter_ubuntu_external: bool = False, i18n: I18nMessages = None) -> list[dict[str, Any]]:
    """
    Finds all available OpenCV cameras plugged into the system.

    Args:
        filter_ubuntu_external: If True and on Ubuntu, filter to only external cameras (video4+)

    Returns:
        A list of all available OpenCV cameras with their metadata.
    """
    if i18n is None:
        i18n = I18nMessages()
        
    all_opencv_cameras_info: list[dict[str, Any]] = []
    logger.info(t("searching_opencv"))
    try:
        opencv_cameras = OpenCVCamera.find_cameras()
        for cam_info in opencv_cameras:
            all_opencv_cameras_info.append(cam_info)
        logger.info(t("found_opencv", count=len(opencv_cameras)))

        if filter_ubuntu_external:
            all_opencv_cameras_info = filter_ubuntu_external_cameras(all_opencv_cameras_info)

    except Exception as e:
        logger.error(f"Error finding OpenCV cameras: {e}")

    return all_opencv_cameras_info


def update_camera_config_yaml(front_camera_id=None, wrist_camera_id=None, config_path=None):
    """Update so101.yaml with detected camera ports."""
    import os
    import yaml

    # Use absolute package path to find configs at same level as RynnLeRobot package
    if config_path is None:
        import RynnLeRobot
        package_dir = Path(RynnLeRobot.__file__).parent  # RynnLeRobot/RynnLeRobot/
        project_root = package_dir.parent  # RynnLeRobot/
        config_path = project_root / "configs" / "so101.yaml"

    try:
        if Path(config_path).exists():
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
        else:
            config = {}

        # Ensure cameras section exists
        if "cameras" not in config or not isinstance(config["cameras"], dict):
            config["cameras"] = {}

        # Update front camera configuration
        if front_camera_id is not None:
            if "front" not in config["cameras"]:
                config["cameras"]["front"] = {
                    "color_mode": "rgb",
                    "fps": 30,
                    "height": 480,
                    "type": "opencv",
                    "width": 640
                }
            config["cameras"]["front"]["index_or_path"] = front_camera_id
            print(t("updated_front_camera", camera_id=front_camera_id))

        # Update wrist camera configuration
        if wrist_camera_id is not None:
            if "wrist" not in config["cameras"]:
                config["cameras"]["wrist"] = {
                    "color_mode": "rgb",
                    "fps": 30,
                    "height": 480,
                    "type": "opencv",
                    "width": 640
                }
            config["cameras"]["wrist"]["index_or_path"] = wrist_camera_id
            print(t("updated_wrist_camera", camera_id=wrist_camera_id))

        with open(config_path, "w") as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=False)

        print(t("camera_config_saved", config_path=config_path))

    except Exception as e:
        print(t("camera_config_failed", error=e))
        print(t("manual_set_cameras", config_path=config_path))


def find_camera_by_difference(camera_name: str, i18n: I18nMessages = None) -> int | None:
    """
    Find a camera by unplugging and plugging it back in.
    Following the same pattern as find_motor_port.py
    """
    if i18n is None:
        i18n = I18nMessages()
        
    print(i18n.get('unplug_prompt', camera_name=camera_name))
    input()
    cameras_before = find_all_opencv_cameras(i18n=i18n)
    camera_ids_before = {cam["id"] for cam in cameras_before}

    time.sleep(0.5)

    print(i18n.get('plug_prompt', camera_name=camera_name))
    input()
    cameras_after = find_all_opencv_cameras(i18n=i18n)
    camera_ids_after = {cam["id"] for cam in cameras_after}
    camera_diff = list(camera_ids_after - camera_ids_before)

    if len(camera_diff) == 1:
        camera_id = camera_diff[0]
        print(i18n.get('detected_camera_id', camera_name=camera_name, port=camera_id))
        print(i18n.get('detected_camera_env', camera_name=camera_name.upper(), port=camera_id))
        return camera_id
    elif len(camera_diff) == 0:
        raise OSError(
            i18n.get('no_cameras_detected')
        )
    else:
        raise OSError(
            i18n.get('not_enough_ubuntu_cameras', count=len(camera_diff))
        )


def find_cameras_hardcoded(i18n: I18nMessages = None):
    """
    Hardcoded camera detection - platform-specific assignment:
    - Ubuntu: Use external cameras (video4+ devices) - typically /dev/video4 (front) and /dev/video6 (wrist)
    - macOS: Use first two available cameras (camera 0 to front, camera 1 to wrist)
    """
    print(t("hardcoded_detection"))

    if platform.system() == "Linux":
        # Ubuntu: Filter for external cameras and use the last two
        all_cameras = find_all_opencv_cameras(filter_ubuntu_external=True)
        print(t("found_cameras_ubuntu", count=len(all_cameras)))

        if len(all_cameras) >= 2:
            # Sort by camera ID and take the last two (highest numbered external cameras)
            all_cameras.sort(key=lambda cam: str(cam["id"]))
            front_camera_id = all_cameras[-2]["id"]   # Second to last (typically /dev/video4)
            wrist_camera_id = all_cameras[-1]["id"]   # Last camera (typically /dev/video6)

            print(t("assigned_front", camera_id=front_camera_id))
            print(t("assigned_wrist", camera_id=wrist_camera_id))

            # Update config file
            update_camera_config_yaml(front_camera_id, wrist_camera_id)

            return {"front": front_camera_id, "wrist": wrist_camera_id}
        else:
            print(t("not_enough_cameras", count=len(all_cameras)))
            return {"front": None, "wrist": None}
    else:
        # macOS: Use first two available cameras
        all_cameras = find_all_opencv_cameras()
        print(t("found_cameras_macos", count=len(all_cameras)))

        if len(all_cameras) >= 2:
            # Sort by camera ID and take the first two (lowest numbered)
            all_cameras.sort(key=lambda cam: str(cam["id"]))
            front_camera_id = all_cameras[0]["id"]   # First camera (typically 0)
            wrist_camera_id = all_cameras[1]["id"]   # Second camera (typically 1)

            print(t("assigned_front", camera_id=front_camera_id))
            print(t("assigned_wrist", camera_id=wrist_camera_id))

            # Update config file
            update_camera_config_yaml(front_camera_id, wrist_camera_id)

            return {"front": front_camera_id, "wrist": wrist_camera_id}
        else:
            print(t("not_enough_cameras", count=len(all_cameras)))
            return {"front": None, "wrist": None}


def find_cameras(i18n: I18nMessages = None):
    """
    Find FRONT and WRIST cameras using hardcoded assignment (last two cameras).
    Similar to motor port detection pattern but without plug/unplug.
    """
    print(t("camera_detection"))
    print("=" * 50)

    # First try hardcoded detection (works for most cases)
    result = find_cameras_hardcoded(i18n=i18n)
    if result["front"] and result["wrist"]:
        #print(i18n.get('detection_separator'))
        return result

    # Commented out manual detection - may be needed in the future
    # print("⚠️ Hardcoded detection failed, falling back to manual detection...")
    # print("📋 Using manual plug/unplug detection method...")
    #
    # # Find FRONT camera
    # try:
    #     front_camera = find_camera_by_difference("FRONT")
    # except OSError as e:
    #     print(f"Error detecting FRONT camera: {e}")
    #     print("DETECTED_FRONT_CAMERA=NOT_FOUND")
    #     front_camera = None
    #
    # print()
    #
    # # Find WRIST camera
    # try:
    #     wrist_camera = find_camera_by_difference("WRIST")
    # except OSError as e:
    #     print(f"Error detecting WRIST camera: {e}")
    #     print("DETECTED_WRIST_CAMERA=NOT_FOUND")
    #     wrist_camera = None
    #
    # # Update config if cameras were detected
    # if front_camera or wrist_camera:
    #     update_camera_config_yaml(front_camera, wrist_camera)

    print(t("camera_detection_done"))
    print("=" * 50)
    return result


def find_and_print_cameras(camera_type_filter: str | None = None, i18n: I18nMessages = None) -> list[dict[str, Any]]:
    """
    Finds available cameras based on an optional filter and prints their information.

    Args:
        camera_type_filter: Optional string to filter cameras ("opencv").
                            If None, lists all opencv cameras.

    Returns:
        A list of all available cameras matching the filter, with their metadata.
    """
    if i18n is None:
        i18n = I18nMessages()
        
    all_cameras_info: list[dict[str, Any]] = []

    if camera_type_filter:
        camera_type_filter = camera_type_filter.lower()

    if camera_type_filter is None or camera_type_filter == "opencv":
        all_cameras_info.extend(find_all_opencv_cameras(i18n=i18n))

    if not all_cameras_info:
        if camera_type_filter:
            logger.warning(f"No {camera_type_filter} cameras were detected.")
        else:
            logger.warning(t("no_cameras_detected"))
    else:
        print(t("detected_cameras"))
        for i, cam_info in enumerate(all_cameras_info):
            print(t("camera_num", num=i))
            for key, value in cam_info.items():
                if key == "default_stream_profile" and isinstance(value, dict):
                    print(i18n.get('camera_property', property=key.replace('_', ' ').capitalize()))
                    for sub_key, sub_value in value.items():
                        print(i18n.get('camera_sub_property', property=sub_key.capitalize(), value=sub_value))
                else:
                    print(i18n.get('camera_property', property=key.replace('_', ' ').capitalize(), value=value))
            print(i18n.get('property_separator'))
    return all_cameras_info


def save_image(
    img_array: np.ndarray,
    camera_identifier: str | int,
    images_dir: Path,
    camera_type: str,
    sequence_number: int = 1,
    i18n: I18nMessages = None,
):
    """
    Saves a single image to disk using Pillow. Handles color conversion if necessary.
    """
    if i18n is None:
        i18n = I18nMessages()
        
    try:
        img = Image.fromarray(img_array, mode="RGB")

        # Extract camera port number from identifier
        # Linux: /dev/video0 -> 0, macOS: 0 -> 0, Windows: 1 -> 1
        port_number = "unknown"
        if "/dev/video" in str(camera_identifier):
            # Linux format: /dev/video0 -> 0
            port_number = str(camera_identifier).split("video")[-1]
        else:
            # macOS/Windows format: just use the identifier as is
            port_number = str(camera_identifier)

        filename = f"image{port_number}_{sequence_number:02d}.png"

        path = images_dir / filename
        path.parent.mkdir(parents=True, exist_ok=True)
        img.save(str(path))
        logger.info(f"Saved image: {path}")
    except Exception as e:
        logger.error(f"Failed to save image for camera {camera_identifier} (type {camera_type}): {e}")


def create_camera_instance(cam_meta: dict[str, Any], i18n: I18nMessages = None) -> dict[str, Any] | None:
    """Create and connect to a camera instance based on metadata."""
    if i18n is None:
        i18n = I18nMessages()
        
    cam_type = cam_meta.get("type")
    cam_id = cam_meta.get("id")
    instance = None

    logger.info(i18n.get('camera_connection', id=cam_id))

    try:
        if cam_type == "OpenCV":
            cv_config = OpenCVCameraConfig(
                index_or_path=cam_id,
                color_mode=ColorMode.RGB,
            )
            instance = OpenCVCamera(cv_config)
        else:
            logger.warning(f"Unknown camera type: {cam_type} for ID {cam_id}. Skipping.")
            return None

        if instance:
            logger.info(i18n.get('camera_test', id=cam_id))
            instance.connect(warmup=False)
            return {"instance": instance, "meta": cam_meta}
    except Exception as e:
        logger.error(i18n.get('camera_test_failed', id=cam_id, error=e))
        if instance and instance.is_connected:
            instance.disconnect()
        return None


def process_camera_image(
    cam_dict: dict[str, Any], 
    output_dir: Path, 
    current_time: float, 
    sequence_number: int = 1,
    i18n: I18nMessages = None,
) -> concurrent.futures.Future | None:
    """Capture and process an image from a single camera."""
    if i18n is None:
        i18n = I18nMessages()
        
    cam = cam_dict["instance"]
    meta = cam_dict["meta"]
    cam_type_str = str(meta.get("type", "unknown"))
    cam_id_str = str(meta.get("id", "unknown"))

    try:
        image_data = cam.read()

        return save_image(
            image_data,
            cam_id_str,
            output_dir,
            cam_type_str,
            sequence_number,
            i18n=i18n,
        )
    except TimeoutError:
        logger.warning(
            f"Timeout reading from {cam_type_str} camera {cam_id_str} at time {current_time:.2f}s."
        )
    except Exception as e:
        logger.error(f"Error reading from {cam_type_str} camera {cam_id_str}: {e}")
    return None


def cleanup_cameras(cameras_to_use: list[dict[str, Any]], i18n: I18nMessages = None):
    """Disconnect all cameras."""
    if i18n is None:
        i18n = I18nMessages()
        
    logger.info(i18n.get('camera_disconnection', count=len(cameras_to_use)))
    for cam_dict in cameras_to_use:
        try:
            if cam_dict["instance"] and cam_dict["instance"].is_connected:
                cam_dict["instance"].disconnect()
        except Exception as e:
            logger.error(f"Error disconnecting camera {cam_dict['meta'].get('id')}: {e}")


def save_images_from_all_cameras(
    output_dir: Path,
    record_time_s: float = 2.0,
    camera_type: str | None = None,
    i18n: I18nMessages = None,
):
    """
    Connects to detected cameras (optionally filtered by type) and saves images from each.
    Uses default stream profiles for width, height, and FPS.

    Args:
        output_dir: Directory to save images.
        record_time_s: Duration in seconds to record images.
        camera_type: Optional string to filter cameras ("realsense" or "opencv").
                            If None, uses all detected cameras.
    """
    if i18n is None:
        i18n = I18nMessages()
        
    output_dir.mkdir(parents=True, exist_ok=True)
    logger.info(i18n.get('saving_images_to', path=output_dir))
    all_camera_metadata = find_and_print_cameras(camera_type_filter=camera_type, i18n=i18n)

    if not all_camera_metadata:
        logger.warning(i18n.get('no_cameras_detected'))
        return

    cameras_to_use = []
    for cam_meta in all_camera_metadata:
        camera_instance = create_camera_instance(cam_meta, i18n=i18n)
        if camera_instance:
            cameras_to_use.append(camera_instance)

    if not cameras_to_use:
        logger.warning("No cameras could be connected. Aborting image save.")
        return

    logger.info(i18n.get('camera_capture_started', time=record_time_s, count=len(cameras_to_use)))
    start_time = time.perf_counter()

    # Initialize sequence counters for each camera
    camera_counters = {}
    for cam_dict in cameras_to_use:
        cam_id = cam_dict["meta"]["id"]
        camera_counters[cam_id] = 1

    with concurrent.futures.ThreadPoolExecutor(max_workers=len(cameras_to_use) * 2) as executor:
        try:
            while time.perf_counter() - start_time < record_time_s:
                futures = []
                current_capture_time = time.perf_counter()

                for cam_dict in cameras_to_use:
                    cam_id = cam_dict["meta"]["id"]
                    sequence_num = camera_counters[cam_id]

                    # Increment counter for this camera BEFORE submitting the task
                    camera_counters[cam_id] += 1

                    future = process_camera_image(cam_dict, output_dir, current_capture_time, sequence_num)
                    if future:
                        futures.append(future)

                if futures:
                    concurrent.futures.wait(futures)

                # Add a small delay to avoid overloading the cameras
                time.sleep(0.1)

        except KeyboardInterrupt:
            logger.info("Capture interrupted by user.")
        finally:
            print(i18n.get('finalizing_saving'))
            executor.shutdown(wait=True)
            cleanup_cameras(cameras_to_use, i18n=i18n)
            print(i18n.get('capture_finished', path=output_dir))


def main():
    import os

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s"
    )

    parser = argparse.ArgumentParser(
        description="Camera detection and listing utility."
    )

    parser.add_argument(
        "--detect",
        action="store_true",
        help="Detect FRONT and WRIST cameras using plug/unplug method"
    )
    parser.add_argument(
        "--save-images",
        action="store_true",
        help="Save test images from detected cameras"
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default="outputs/camera_images",
        help="Directory to save images. Default: outputs/camera_images",
    )
    parser.add_argument(
        "--record-time-s",
        type=float,
        default=2.0,
        help="Time duration to attempt capturing frames. Default: 2 seconds.",
    )

    args = parser.parse_args()

    # Select language if not set via environment
    select_language()

    if args.detect:
        find_cameras()
        # Force clean exit to avoid memory corruption during cleanup
        os._exit(0)
    elif args.save_images:
        print(i18n.get('saving_test_images'))
        save_images_from_all_cameras(
            output_dir=args.output_dir,
            record_time_s=args.record_time_s,
            camera_type="opencv",
            i18n=i18n
        )
        # Force clean exit to avoid memory corruption during cleanup
        os._exit(0)
    else:
        # Default behavior: find and assign cameras, then print results
        find_and_print_cameras()
        print("\n" + "=" * 50)
        find_cameras()
        # Force clean exit to avoid memory corruption during cleanup
        os._exit(0)


if __name__ == "__main__":
    main()
