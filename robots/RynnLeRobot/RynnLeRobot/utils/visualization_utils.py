"""Rerun visualization utilities for robot data."""

import os

# import rerun as rr
import cv2
import logging
import queue
import platform
import numpy as np
from typing import Any
from RynnLeRobot.tools.web_streamer import frame_queue
from RynnLeRobot.tools.interpolate import interpolate_tensor
from RynnLeRobot.tools.policy_interpolator import PolicyInterpolator


def _init_rerun(session_name: str = "lerobot_control_loop") -> None:
    pass
    # """Initializes the Rerun SDK for visualizing the control loop."""
    # batch_size = os.getenv("RERUN_FLUSH_NUM_BYTES", "8000")
    # os.environ["RERUN_FLUSH_NUM_BYTES"] = batch_size
    # rr.init(session_name)
    # memory_limit = os.getenv("LEROBOT_RERUN_MEMORY_LIMIT", "10%")
    # rr.spawn(memory_limit=memory_limit)


def log_rerun_data(observation: dict[str | Any], action: dict[str | Any]):
    pass
    # for obs, val in observation.items():
    #     if isinstance(val, float):
    #         rr.log(f"observation.{obs}", rr.Scalar(val))
    #     elif isinstance(val, np.ndarray):
    #         if val.ndim == 1:
    #             for i, v in enumerate(val):
    #                 rr.log(f"observation.{obs}_{i}", rr.Scalar(float(v)))
    #         else:
    #             rr.log(f"observation.{obs}", rr.Image(val), static=True)
    # for act, val in action.items():
    #     if isinstance(val, float):
    #         rr.log(f"action.{act}", rr.Scalar(val))
    #     elif isinstance(val, np.ndarray):
    #         for i, v in enumerate(val):
    #             rr.log(f"action.{act}_{i}", rr.Scalar(float(v)))


def is_headless():
    """
    Checks if the current environment is 'headless' (lacks a GUI).

    This function is useful for determining if GUI operations, like displaying
    an image with `cv2.imshow()`, are likely to succeed.

    It checks for:
    1. The presence of common environment variables used in GUI sessions (e.g., DISPLAY).
    2. Specific environment variables set by Docker or other container runtimes.
    3. The ability to import and initialize GUI-related libraries.

    Returns:
        bool: True if the environment is likely headless, False otherwise.
    """
    # --- 1. 检查环境变量 (Check Environment Variables) ---

    # 在 Linux/Unix 上，DISPLAY 环境变量是 X11 图形系统的关键。
    # On Linux/Unix, the DISPLAY variable is crucial for the X11 graphics system.
    if platform.system() in ["Linux", "Darwin"]:
        # 如果 DISPLAY 未设置或为空，几乎可以肯定是无头环境。
        # If DISPLAY is not set or is empty, it's almost certainly a headless environment.
        if not os.environ.get("DISPLAY"):
            logging.warning(
                "is_headless: 'DISPLAY' environment variable not set. Assuming headless."
            )
            return True

    # --- 2. 检查特定于容器/CI/CD 的环境变量 (Check for Container/CI/CD specific variables) ---

    # 很多持续集成 (CI) 工具会设置特定变量。
    # Many Continuous Integration (CI) tools set specific variables.
    if "CI" in os.environ:
        logging.warning(
            "is_headless: 'CI' environment variable found. Assuming headless."
        )
        return True

    # --- 3. 尝试初始化一个GUI后端 (Attempt to initialize a GUI backend) ---

    # 这是最可靠的方法：直接尝试创建一个窗口。如果失败，就是无头环境。
    # This is the most robust method: try to actually create a window. If it fails, it's headless.
    # try:
    #     import cv2

    #     # 创建一个非常小的、不可见的临时窗口。
    #     # Create a tiny, invisible temporary window.
    #     window_name = "headless_test_window"
    #     cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    #     cv2.resizeWindow(window_name, 1, 1)

    #     # 在某些系统上，即使 namedWindow 成功，imshow 也可能失败。
    #     # 为了更可靠，可以尝试显示一个1x1的黑色像素，但通常 namedWindow 就足够了。
    #     # 重要的是立即销毁它，不让它闪现。
    #     # Importantly, destroy it immediately to prevent it from flashing.
    #     cv2.destroyWindow(window_name)

    # except Exception as e:
    #     # 如果 cv2.namedWindow() 失败，通常会抛出一个带有 "could not connect to display"
    #     # 或类似信息的异常。
    #     # If cv2.namedWindow() fails, it usually throws an exception with a message like
    #     # "could not connect to display" or similar.
    #     logging.warning(
    #         f"is_headless: OpenCV GUI operation failed with error: {e}. Assuming headless."
    #     )
    #     return True

    # 如果所有检查都通过了，那么我们大概率在一个有图形界面的环境中。
    # If all checks pass, we are likely in an environment with a GUI.
    return False


def display_imgs(obs, show_display=False, show_webcam=False):
    """
    显示图像数据
    """
    if not (show_display or show_webcam):  # 如果没有显示选项，则返回
        return
    image_keys = [x for x in obs if "image" in x]
    # 处理numpy数组，不需要调用.cpu()
    image_data = []
    for x in image_keys:
        if hasattr(obs[x], "cpu"):  # 如果是PyTorch张量
            img = cv2.cvtColor(obs[x].cpu().numpy(), cv2.COLOR_RGB2BGR)
        else:  # 如果是numpy数组
            img = cv2.cvtColor(obs[x], cv2.COLOR_RGB2BGR)
        image_data.append(img)

    if len(image_data) > 0:
        # 水平拼接两张图片
        combined_img = np.hstack(image_data)
        # 添加标题文字
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(combined_img, "Front Camera", (10, 30), font, 1, (255, 255, 255), 2)
        if len(image_data) > 1:
            cv2.putText(
                combined_img,
                "Wrist Camera",
                (image_data[0].shape[1] + 10, 30),
                font,
                1,
                (255, 255, 255),
                2,
            )
        if show_display:
            # 显示拼接后的图片
            cv2.imshow("Cameras", combined_img)
            # 非阻塞等待1ms
            cv2.waitKey(1)
        if show_webcam:
            try:  # 图像传到 web 服务器的视频流线程中，远程显示
                frame_queue.put_nowait(combined_img)
            except queue.Full:
                pass
