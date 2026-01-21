import asyncio
import queue
import threading
import cv2
import time
import uvicorn
from fastapi import FastAPI, Response
from fastapi.responses import StreamingResponse

# 关键修复：禁用 threading 模块的 atexit 处理
try:
    import threading

    threading._shutdown = lambda: None  # 禁止 threading 的 atexit 处理
except:
    pass

# --- 创建一个线程安全的队列来共享图像帧 ---
# maxsize=1 保证队列中只存放最新的一帧，旧的会被自动丢弃
frame_queue = queue.Queue(maxsize=1)

# --- FastAPI 应用定义 ---
app = FastAPI()


async def generate_frames():
    """
    一个异步生成器，从共享队列中获取帧并进行流式传输。
    """
    while True:
        try:
            # 从队列中获取帧。这是一个阻塞操作，但可以设置超时。
            # 我们在一个异步函数中调用它，需要用 run_in_executor 来避免阻塞事件循环。
            loop = asyncio.get_running_loop()
            frame = await loop.run_in_executor(None, frame_queue.get)

            if frame is None:
                # 可以约定用None作为结束信号
                break

            # 将帧编码为JPEG
            ret, buffer = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            )
            if not ret:
                continue
            yield (
                b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                + buffer.tobytes()
                + b"\r\n"
            )
        except Exception as e:
            print(f"Error in frame generation: {e}")
            break


@app.get("/")
def read_root():
    html_content = """
    <html>
        <head>
            <title>摄像头实时视频流</title>
        </head>
        <body>
            <h1>摄像头实时视频流</h1>
            <img src="/video_feed">
        </body>
    </html>
    """
    return Response(content=html_content, media_type="text/html")


@app.get("/video_feed")
async def video_feed():
    return StreamingResponse(
        generate_frames(), media_type="multipart/x-mixed-replace; boundary=frame"
    )


class WebServer:
    def __init__(self, host="0.0.0.0", port=5001, log_level="info"):
        self.config = uvicorn.Config(
            app,
            host=host,
            port=port,
            log_level=log_level,
            timeout_keep_alive=5,
            lifespan="on",
        )
        self.server = uvicorn.Server(self.config)
        self.thread = None
        self.running = False
        self._stop_requested = False

    def start(self):
        """启动服务器（守护线程）"""
        if self.running:
            return

        self.running = True
        self._stop_requested = False
        self.thread = threading.Thread(target=self._server_runner)
        self.thread.daemon = True  # 关键：必须设为守护线程
        self.thread.start()

    def _server_runner(self):
        try:
            self.server.run()
        except Exception as e:
            print(f"Server error: {e}")
        finally:
            self.running = False

    def stop(self, timeout=0.1):
        """立即停止服务器（非阻塞）"""
        if not self.running or self._stop_requested:
            return

        self._stop_requested = True
        print(f"{time.strftime('%H:%M:%S')} 🛑 请求停止服务器...")
        self.server.should_exit = True

        # 尝试快速加入（非常短的超时）
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=timeout)

        # 不等待线程结束 - 主程序将继续执行
        self.running = False
        print(f"{time.strftime('%H:%M:%S')} ✅ 停止请求已发送")


# ================== 使用示例 ==================
if __name__ == "__main__":
    # 关键：移除所有 atexit 注册
    try:
        import atexit

        atexit._clear()
    except:
        pass

    server = WebServer()

    # 1. 启动服务器（守护线程）
    server.start()
    print(f"{time.strftime('%H:%M:%S')} 🌐 服务器已启动，访问: http://localhost:5001")

    try:
        # 2. 主程序核心逻辑
        print("主程序正在运行... (按 Ctrl+C 停止)")
        while True:
            time.sleep(0.1)  # 避免CPU空转

    except KeyboardInterrupt:
        print(
            "\n"
            + f"{time.strftime('%H:%M:%S')} 🚨 检测到 Ctrl+C，停止服务器请求已发送..."
        )
        server.stop()  # 发送停止请求（不等待）

    # 3. 继续执行其他关闭操作（立即执行！）
    print(f"{time.strftime('%H:%M:%S')} 🛠️ 开始执行后续清理操作...")

    # 示例：保存数据
    print(f"  - 保存最后处理的数据")
    # save_data()

    # 示例：关闭设备
    print(f"  - 关闭摄像头设备")
    # camera.close()

    # 示例：释放资源
    print(f"  - 释放共享内存资源")
    # shared_memory.close()

    print(f"{time.strftime('%H:%M:%S')} ✅ 所有资源已释放，主程序继续执行后续操作")

    # ======================
    # 主程序将继续执行到这里
    # ======================
    print("✅ 主程序后续代码执行中...")
    time.sleep(2)

    # 当主程序结束时，守护线程会自动终止
    print("主程序结束 - 服务器守护线程将自动终止")
