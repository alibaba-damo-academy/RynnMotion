#!/usr/bin/env python3
"""
MuJoCo 多相机分屏显示模块
基于 GLFW 实现多个相机视角的同步显示
"""

import os
import numpy as np
import logging
from typing import List, Optional, Tuple
import time

try:
    import glfw
    import OpenGL.GL as gl
except ImportError as e:
    print(f"GLFW 或 OpenGL 导入失败: {e}")
    print("请安装必要的依赖: pip install glfw PyOpenGL")
    glfw = None
    gl = None

import mujoco


class MultiCameraViewer:
    """
    多相机分屏显示类
    支持在单个 GLFW 窗口中显示多个相机视角
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        camera_names: Optional[List[str]] = None,
        window_width: int = 1200,
        window_height: int = 800,
        layout: str = "2x3"
    ):
        """
        初始化多相机查看器

        Args:
            model: MuJoCo 模型
            data: MuJoCo 数据
            camera_names: 相机名称列表，如果为 None 则使用模型中所有相机
            window_width: 窗口宽度
            window_height: 窗口高度
            layout: 布局类型 ("2x3", "1x5", "grid")
        """
        self.model = model
        self.data = data
        self.window_width = window_width
        self.window_height = window_height
        self.layout = layout

        # 初始化日志
        self.logger = logging.getLogger(__name__)

        # 获取相机列表
        if camera_names is None:
            self.camera_names = self._get_all_camera_names()
        else:
            self.camera_names = camera_names

        self.logger.info(f"发现 {len(self.camera_names)} 个相机: {self.camera_names}")

        # 初始化 GLFW
        if not self._init_glfw():
            raise RuntimeError("GLFW 初始化失败")

        # 初始化渲染器
        self._init_renderers()

        # 初始化视口管理
        self.viewports = self._setup_viewports()

        self.is_running = False

    def _get_all_camera_names(self) -> List[str]:
        """获取模型中所有相机名称"""
        camera_names = []
        for i in range(self.model.ncam):
            camera_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_CAMERA, i)
            if camera_name:
                camera_names.append(camera_name.decode('utf-8') if isinstance(camera_name, bytes) else camera_name)
        return camera_names

    def _init_glfw(self) -> bool:
        """初始化 GLFW 窗口"""
        if glfw is None:
            self.logger.error("GLFW 未安装")
            return False

        # 设置错误回调以获取更多信息
        glfw.set_error_callback(lambda error, description: self.logger.error(f"GLFW错误 {error}: {description}"))

        # 检查GLFW是否已经初始化
        try:
            # 尝试创建一个测试窗口来检查GLFW状态
            test_window = glfw.create_window(1, 1, "test", None, None)
            if test_window:
                glfw.destroy_window(test_window)
        except:
            # GLFW未初始化，需要初始化
            if not glfw.init():
                self.logger.error("GLFW 初始化失败")
                return False

        # 设置窗口提示
        glfw.window_hint(glfw.VISIBLE, glfw.TRUE)
        glfw.window_hint(glfw.RESIZABLE, glfw.TRUE)

        # 创建窗口
        self.window = glfw.create_window(
            self.window_width,
            self.window_height,
            "MuJoCo 多相机查看器",
            None,
            None
        )

        if not self.window:
            self.logger.error("创建窗口失败")
            return False

        glfw.make_context_current(self.window)

        # 设置回调函数
        glfw.set_window_size_callback(self.window, self._window_size_callback)

        return True

    def _init_renderers(self):
        """初始化相机渲染器"""
        self.renderers = []
        for camera_name in self.camera_names:
            try:
                renderer = mujoco.Renderer(self.model, 480, 480)
                self.renderers.append(renderer)
                self.logger.debug(f"创建相机 {camera_name} 的渲染器")
            except Exception as e:
                self.logger.error(f"创建相机 {camera_name} 渲染器失败: {e}")
                raise

    def _setup_viewports(self) -> List:
        """设置视口布局"""
        if self.layout == "2x3":
            return self._setup_2x3_layout()
        elif self.layout == "1x5":
            return self._setup_1x5_layout()
        else:
            return self._setup_grid_layout()

    def _setup_2x3_layout(self) -> List:
        """设置 2行3列 布局"""
        rows, cols = 2, 3
        viewport_width = self.window_width // cols
        viewport_height = self.window_height // rows

        viewports = []
        for i in range(min(len(self.camera_names), 6)):  # 最多6个视口
            if i >= len(self.camera_names):
                break
            row = i // cols
            col = i % cols
            # OpenGL 坐标系：左下角为原点
            viewport = (
                col * viewport_width,                    # left
                (rows - 1 - row) * viewport_height,      # bottom
                viewport_width,                          # width
                viewport_height                          # height
            )
            viewports.append(viewport)
        return viewports

    def _setup_1x5_layout(self) -> List:
        """设置 1行5列 布局"""
        cols = len(self.camera_names)
        viewport_width = self.window_width // cols
        viewport_height = self.window_height

        viewports = []
        for i in range(len(self.camera_names)):
            viewport = (
                i * viewport_width,    # left
                0,                     # bottom
                viewport_width,        # width
                viewport_height        # height
            )
            viewports.append(viewport)
        return viewports

    def _setup_grid_layout(self) -> List:
        """设置网格布局"""
        # 计算最佳行列数
        num_cameras = len(self.camera_names)
        cols = int(np.ceil(np.sqrt(num_cameras)))
        rows = int(np.ceil(num_cameras / cols))

        viewport_width = self.window_width // cols
        viewport_height = self.window_height // rows

        viewports = []
        for i in range(num_cameras):
            row = i // cols
            col = i % cols
            # OpenGL 坐标系：左下角为原点
            viewport = (
                col * viewport_width,
                (rows - 1 - row) * viewport_height,
                viewport_width,
                viewport_height
            )
            viewports.append(viewport)
        return viewports

    def _window_size_callback(self, window, width, height):
        """窗口大小改变回调"""
        self.window_width = width
        self.window_height = height
        self.viewports = self._setup_viewports()

    def render(self, external_images=None):
        """渲染所有相机视角

        Args:
            external_images: 外部提供的图像字典 {camera_name: image_array}
        """
        if not self.is_running:
            return

        # 清空缓冲区
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

        # 渲染每个相机视角
        for i, camera_name in enumerate(self.camera_names):
            if i >= len(self.viewports):
                break

            try:
                if external_images and camera_name in external_images:
                    # 使用外部提供的图像（零拷贝）
                    image = external_images[camera_name]
                else:
                    # 使用内部渲染器渲染
                    if i < len(self.renderers):
                        renderer = self.renderers[i]
                        renderer.update_scene(self.data, camera=camera_name)
                        image = renderer.render()
                    else:
                        # 如果没有渲染器，创建一个黑色图像
                        image = np.zeros((480, 640, 3), dtype=np.uint8)

                # 在指定视口中显示图像
                self._render_image_to_viewport(image, self.viewports[i], camera_name)

            except Exception as e:
                self.logger.error(f"渲染相机 {camera_name} 失败: {e}")

    def _render_image_to_viewport(self, image: np.ndarray, viewport: Tuple, camera_name: str):
        """将图像渲染到指定视口"""
        left, bottom, width, height = viewport

        # 设置视口
        gl.glViewport(left, bottom, width, height)

        # 确保图像是正确的格式
        if image.dtype != np.uint8:
            image = (image * 255).astype(np.uint8)

        # 翻转图像（OpenGL 坐标系差异）
        image = np.flipud(image)

        # 上传纹理并渲染
        gl.glEnable(gl.GL_TEXTURE_2D)
        texture_id = gl.glGenTextures(1)
        gl.glBindTexture(gl.GL_TEXTURE_2D, texture_id)

        # 设置纹理参数
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)

        # 上传图像数据
        gl.glTexImage2D(
            gl.GL_TEXTURE_2D, 0, gl.GL_RGB,
            image.shape[1], image.shape[0], 0,
            gl.GL_RGB, gl.GL_UNSIGNED_BYTE,
            image.tobytes()
        )

        # 渲染四边形
        gl.glBegin(gl.GL_QUADS)
        gl.glTexCoord2f(0, 0)
        gl.glVertex2f(-1, -1)
        gl.glTexCoord2f(1, 0)
        gl.glVertex2f(1, -1)
        gl.glTexCoord2f(1, 1)
        gl.glVertex2f(1, 1)
        gl.glTexCoord2f(0, 1)
        gl.glVertex2f(-1, 1)
        gl.glEnd()

        # 清理纹理
        gl.glDeleteTextures([texture_id])
        gl.glDisable(gl.GL_TEXTURE_2D)

        # 渲染相机名称标签
        # self._render_text_overlay(camera_name, left, bottom, width, height)

    def _render_text_overlay(self, text: str, left: int, bottom: int, width: int, height: int):
        """渲染文本覆盖层"""
        # 简单的文本显示实现
        # 使用纯 OpenGL 不依赖 GLUT 的方式
        try:
            # 设置视口用于文本渲染
            gl.glViewport(left, bottom, width, height)

            # 设置正交投影
            gl.glMatrixMode(gl.GL_PROJECTION)
            gl.glPushMatrix()
            gl.glLoadIdentity()
            gl.glOrtho(0, width, 0, height, -1, 1)

            gl.glMatrixMode(gl.GL_MODELVIEW)
            gl.glPushMatrix()
            gl.glLoadIdentity()

            # 禁用深度测试以确保文本在最前面
            gl.glDisable(gl.GL_DEPTH_TEST)

            # 设置颜色（白色半透明背景）
            gl.glColor4f(0.0, 0.0, 0.0, 0.5)  # 黑色半透明背景

            # 绘制背景矩形
            gl.glBegin(gl.GL_QUADS)
            gl.glVertex2f(5, height - 25)
            gl.glVertex2f(len(text) * 8 + 15, height - 25)
            gl.glVertex2f(len(text) * 8 + 15, height - 5)
            gl.glVertex2f(5, height - 5)
            gl.glEnd()

            # 设置文本颜色（白色）
            gl.glColor3f(1.0, 1.0, 1.0)

            # 简单的字符渲染（使用线段绘制基本字符）
            # 这里只是一个占位实现，实际项目中可以使用 freetype-py 等库
            gl.glRasterPos2f(10, height - 20)

            # 恢复状态
            gl.glEnable(gl.GL_DEPTH_TEST)
            gl.glPopMatrix()  # MODELVIEW
            gl.glMatrixMode(gl.GL_PROJECTION)
            gl.glPopMatrix()
            gl.glMatrixMode(gl.GL_MODELVIEW)

        except Exception as e:
            # 如果文本渲染失败，忽略错误
            self.logger.debug(f"文本渲染失败: {e}")
            pass

    def run(self, max_frames: Optional[int] = None):
        """
        运行多相机显示

        Args:
            max_frames: 最大帧数，None 表示无限运行
        """
        self.is_running = True
        frame_count = 0

        self.logger.info("开始多相机显示，按 ESC 键退出")

        while self.is_running and not glfw.window_should_close(self.window):
            # 处理用户输入
            if glfw.get_key(self.window, glfw.KEY_ESCAPE) == glfw.PRESS:
                self.is_running = False

            # 渲染
            self.render()

            # 交换缓冲区
            glfw.swap_buffers(self.window)
            glfw.poll_events()

            frame_count += 1
            if max_frames and frame_count >= max_frames:
                break

            # 控制帧率
            time.sleep(0.016)  # ~60 FPS

    def close(self):
        """关闭查看器"""
        self.is_running = False
        if hasattr(self, 'window') and self.window:
            try:
                glfw.destroy_window(self.window)
                self.window = None
            except Exception as e:
                self.logger.debug(f"销毁窗口时出错: {e}")

        # 清理渲染器
        for renderer in self.renderers:
            try:
                renderer.close()
            except Exception as e:
                self.logger.debug(f"关闭渲染器时出错: {e}")

        # 清空渲染器列表
        self.renderers = []

        # 清除GLFW错误回调以避免关闭后的错误
        try:
            glfw.set_error_callback(None)
        except:
            pass

        self.logger.info("多相机查看器已关闭")


def create_multi_camera_viewer(
    xml_path: str,
    camera_names: Optional[List[str]] = None,
    window_width: int = 1200,
    window_height: int = 800,
    layout: str = "2x3"
) -> MultiCameraViewer:
    """
    创建多相机查看器的便捷函数

    Args:
        xml_path: MuJoCo XML 模型文件路径
        camera_names: 相机名称列表
        window_width: 窗口宽度
        window_height: 窗口高度
        layout: 布局类型

    Returns:
        MultiCameraViewer 实例
    """
    # 加载模型
    if not os.path.exists(xml_path):
        raise FileNotFoundError(f"模型文件不存在: {xml_path}")

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # 初始化仿真
    mujoco.mj_forward(model, data)

    # 创建查看器
    viewer = MultiCameraViewer(
        model, data, camera_names, window_width, window_height, layout
    )

    return viewer


if __name__ == "__main__":
    # 示例用法
    logging.basicConfig(level=logging.INFO)

    # 测试代码（需要根据实际路径调整）
    xml_path = "../../../models/3.robot_arm/24.so101/scene/multi_robots.xml"

    try:
        viewer = create_multi_camera_viewer(
            xml_path=xml_path,
            layout="2x3"
        )
        viewer.run(max_frames=1000)
    except Exception as e:
        print(f"运行失败: {e}")
    finally:
        if 'viewer' in locals():
            viewer.close()
