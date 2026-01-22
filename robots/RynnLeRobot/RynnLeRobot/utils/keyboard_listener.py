#!/usr/bin/env python3
"""Keyboard listener for recording control."""

import logging
import threading
import platform
import os

# ==============================================================================
# --- 语言配置 (Language Configuration) ---
# ==============================================================================

# 定义所有需要翻译的文本
# Define all translatable text strings
LANG_STRINGS = {
    "EN": {
        "doc_module": "Keyboard listener utilities for recording control.",
        "doc_init": """Initialize keyboard listener for recording controls.

    Keyboard controls:
    - Right Arrow (→): Exit current episode early
    - Left Arrow (←): Exit and re-record current episode
    - Enter: Continue data recording
    - q or Esc: Stop entire recording session

    Returns:
        tuple: (listener_thread, events_dict)
    """,
        "doc_cleanup": "Stop the keyboard listener thread and clean up resources.",
        "warn_sshkeyboard_unavailable": "sshkeyboard not available. Keyboard controls disabled.",
        "info_right_arrow": "\n→ Right arrow pressed: Ending episode early and saving data...",
        "info_left_arrow": "\n← Left arrow pressed: Discarding episode and re-record...",
        "info_enter": "Enter key pressed. Continuing data recording...",
        "info_stop": "\n⏹️ q or Esc pressed: Stopping entire recording session...",
        "error_key_press": "Error handling key press: {e}",
        "info_os_detected": "🔍 OS Detected",
        "tip_linux": "💡 Tip: On some Linux terminals, listening for arrow/function keys may require running with normal user privileges (no sudo).\n   If keys are not responsive, try running in GNOME Terminal, Konsole, or tmux.",
        "info_windows": "✅ Windows system is generally well-supported.",
        "warn_macos": "⚠️ macOS may require one-time authorization:\n   System Settings → Privacy & Security → Input Monitoring → Check your terminal (e.g., Terminal or iTerm2).",
        "info_unknown_os": "❓ Unknown system, attempting to run...",
        "info_starting_listener": "Starting keyboard listener...",
        "info_skipping_listener": "Unsupported OS. Skipping keyboard listener.",
        "header_controls": "\n🎮 Keyboard Controls (Active During Recording):",
        "control_right": "   → (Right Arrow) or N: End episode early and SAVE partial data",
        "control_left": "   ← (Left Arrow) or R:  Discard episode and re-record from start",
        "control_enter": "   Enter:           Continue when paused",
        "control_esc": "   q or Esc:        Stop entire recording session",
        "note_right": "   💡 Note: Press right arrow if you want to end episode early but keep the data",
        "note_left": "           Press left arrow if you made a mistake and want to redo completely",
        "error_stop_listener": "Error stopping keyboard listener: {e}",
    },
    "CN": {
        "doc_module": "用于录制控制的键盘监听工具。",
        "doc_init": """初始化用于录制控制的键盘监听器。

    键盘控制:
    - 右方向键 (→): 提前结束当前片段
    - 左方向键 (←): 放弃并重新录制当前片段
    - 回车键 (Enter): 继续数据录制
    - q 或 Esc: 停止整个录制会话

    返回:
        元组: (监听器线程, 事件字典)
    """,
        "doc_cleanup": "停止键盘监听线程并清理资源。",
        "warn_sshkeyboard_unavailable": "sshkeyboard 库不可用。键盘控制功能已禁用。",
        "info_right_arrow": "\n→ 已按下右方向键: 提前结束片段并保存数据...",
        "info_left_arrow": "\n← 已按下左方向键: 放弃当前片段并准备重录...",
        "info_enter": "已按下回车键。继续数据录制...",
        "info_stop": "\n⏹️ 已按下 q 或 Esc: 正在停止整个录制会话...",
        "error_key_press": "处理按键时出错: {e}",
        "info_os_detected": "🔍 检测到操作系统",
        "tip_linux": "💡 提示：在部分 Linux 终端中，监听方向键/功能键可能需要以普通用户权限运行（无需 sudo）。\n   如果按键无响应，请尝试在 GNOME Terminal、Konsole 或 tmux 中运行。",
        "info_windows": "✅ Windows 系统通常支持良好。",
        "warn_macos": "⚠️ macOS 首次运行可能需要授权：\n   系统设置 → 隐私与安全性 → 输入监控 → 勾选你的终端（如 Terminal 或 iTerm2）。",
        "info_unknown_os": "❓ 未知系统，尝试运行...",
        "info_starting_listener": "正在启动键盘监听器...",
        "info_skipping_listener": "不支持的操作系统。跳过键盘监听器。",
        "header_controls": "\n🎮 键盘控制 (录制期间有效):",
        "control_right": "   → (右方向键) 或 N 键: 提前结束片段并保存部分数据",
        "control_left": "   ← (左方向键) 或 R 键:  放弃当前片段并从头开始重录",
        "control_enter": "   回车键:        暂停后继续",
        "control_esc": "   q 或 Esc:        停止整个录制会话",
        "note_right": "   💡 提示: 如果想提前结束但保留已录制的数据，请按右方向键",
        "note_left": "           如果操作失误想完全重做，请按左方向键",
        "error_stop_listener": "停止键盘监听器时出错: {e}",
    },
}

# 从环境变量获取语言设置，默认为英文 'EN'
# Get language setting from environment variable, default to 'EN'
CURRENT_LANG_CODE = os.getenv("LANGUAGE", "EN").upper()
if CURRENT_LANG_CODE not in LANG_STRINGS:
    CURRENT_LANG_CODE = "EN"
TEXT = LANG_STRINGS[CURRENT_LANG_CODE]

# ==============================================================================
# --- 核心代码 (Core Code) ---
# ==============================================================================


def init_keyboard_listener():
    # 动态设置文档字符串
    # Dynamically set the docstring
    init_keyboard_listener.__doc__ = TEXT["doc_init"]

    events = {}
    events["exit_early"] = False
    events["rerecord_episode"] = False
    events["stop_recording"] = False
    events["continue"] = False

    try:
        from sshkeyboard import listen_keyboard
    except ImportError:
        logging.warning(TEXT["warn_sshkeyboard_unavailable"])
        listener = None
        return listener, events

    def on_press(key):
        try:
            if key == "right" or key == "n":
                print(TEXT["info_right_arrow"])
                events["exit_early"] = True
            elif key == "left" or key == "r":
                print(TEXT["info_left_arrow"])
                events["rerecord_episode"] = True
            elif key == "enter":
                print(TEXT["info_enter"])
                events["continue"] = True
            elif key == "q" or key == "esc":
                print(TEXT["info_stop"])
                events["stop_recording"] = True
        except Exception as e:
            print(TEXT["error_key_press"].format(e=e))

    def keyboard_listener_thread():
        # listen_keyboard 是一个阻塞函数，所以我们在线程中运行它
        # listen_keyboard is a blocking function, so we run it in a thread.
        listen_keyboard(
            on_press=on_press,
            until=None,  # until=None 使其持续运行直到被 stop_listening() 停止
            lower=True,  # 将字符键转为小写 (e.g., 'Q' becomes 'q')
        )

    system = platform.system()
    print(f"{TEXT['info_os_detected']}: {system}")
    if system == "Linux":
        print(TEXT["tip_linux"])
    elif system == "Windows":
        print(TEXT["info_windows"])
    elif system == "Darwin":
        print(TEXT["warn_macos"])
    else:
        print(TEXT["info_unknown_os"])

    # 仅在支持的系统上启动监听器
    # Only start the listener on supported systems
    if system in ["Windows", "Linux", "Darwin"]:
        print(TEXT["info_starting_listener"])
        # 使用非守护线程，确保程序退出前有机会清理
        # Use a non-daemon thread to ensure cleanup opportunities before program exit
        listener = threading.Thread(target=keyboard_listener_thread, daemon=False)
        listener.start()
    else:
        print(TEXT["info_skipping_listener"])
        listener = None

    print(TEXT["header_controls"])
    print(TEXT["control_right"])
    print(TEXT["control_left"])
    print(TEXT["control_enter"])
    print(TEXT["control_esc"])
    print(TEXT["note_right"])
    print(TEXT["note_left"])
    print()

    return listener, events


def cleanup_keyboard_listener(listener):
    # 动态设置文档字符串
    # Dynamically set the docstring
    cleanup_keyboard_listener.__doc__ = TEXT["doc_cleanup"]

    if listener and listener.is_alive():
        try:
            from sshkeyboard import stop_listening

            # stop_listening() 会中断 listen_keyboard() 的阻塞
            # stop_listening() will interrupt the blocking listen_keyboard()
            stop_listening()
            # 等待线程自然结束
            # Wait for the thread to finish naturally
            listener.join(timeout=1.0)
        except Exception as e:
            logging.warning(TEXT["error_stop_listener"].format(e=e))


# 动态设置模块的文档字符串
# Dynamically set the module's docstring
__doc__ = TEXT["doc_module"]
