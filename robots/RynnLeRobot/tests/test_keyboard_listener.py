#!/usr/bin/env python3
"""
键盘监听测试工具（修复版）
用于验证 sshkeyboard 是否能在当前系统正常工作。
按 Esc 或 'q' 退出。
"""

import sys
import platform

try:
    from sshkeyboard import listen_keyboard
except ImportError:
    print("❌ 未安装 sshkeyboard，请运行: pip install sshkeyboard")
    sys.exit(1)


def on_press(key):
    """处理按键按下事件"""
    try:
        # 我们在这里打印，这样用户可以看到反馈
        print(f"👉 按下键: '{key}'", end="\r\n", flush=True)

        if key in ("esc", "q"):
            print("🛑 检测到退出指令，正在关闭监听器...")
            # 返回 False 会优雅地终止 listen_keyboard 循环
            return False

    except Exception as e:
        print(f"⚠️ 按键处理异常: {e}")


def print_instructions():
    """打印引导信息"""
    system = platform.system()
    print(f"🔍 检测到操作系统: {system}")
    if system == "Linux":
        print(
            "💡 提示：在部分 Linux 终端中，监听方向键/功能键可能需要以普通用户权限运行（无需 sudo）"
        )
        print("   如果按键无响应，请尝试在 GNOME Terminal、Konsole 或 tmux 中运行。")
    elif system == "Windows":
        print("✅ Windows 系统通常支持良好。")
    elif system == "Darwin":
        print("⚠️ macOS 首次运行可能需要授权：")
        print(
            "   系统设置 → 隐私与安全性 → 输入监控 → 勾选你的终端（如 Terminal 或 iTerm2）"
        )

    print("\n🟢 正在启动键盘监听器...")
    print("⌨️  请随意按键（支持字母、数字、方向键、Enter、Esc 等）")
    print("🛑 按 'Esc' 或 'q' 退出本程序\n")


def main():
    print_instructions()

    try:
        # 直接在主线程中运行 listen_keyboard。它会阻塞程序，直到 on_press 返回 False。
        # 这种方式可以确保 listen_keyboard 在退出时能够执行其内部的 try...finally 清理块。
        listen_keyboard(
            on_press=on_press,
            lower=True,
        )
    except Exception as e:
        # 捕获其他可能的异常，例如设备未找到等
        print(f"\n❌ 键盘监听器启动失败或发生意外错误: {e}")
    finally:
        # 虽然 listen_keyboard 内部有清理，但在这里打印信息可以确认程序流程
        print("✅ 键盘监听测试已结束。终端设置应已恢复。")
        # 程序正常退出，无需 sys.exit(0)


if __name__ == "__main__":
    main()
