# robot_monitor.py

import cv2
import numpy as np
import time
from threading import Thread, Lock
from queue import Queue
from collections import deque
from RynnMotion.common.data.robot_state import RobotState
import math


class DataMonitor:
    def __init__(
        self,
        data_dim=1,
        data_dim_names=None,
        control_freq=100,
        data_name="joint position",
        history_len=1000,
    ):
        self.data_dim = data_dim
        if data_dim_names is None:
            self.data_dim_names = self._set_default_data_names()
        self.data_name = data_name
        self.running = False
        self.paused = False
        self.data_queue = Queue(maxsize=history_len)
        self.lock = Lock()
        self.dt = 1 / control_freq  # s
        self.prev_time = time.time()
        self.control_delta_time = 0

        # sub image size
        self.img_width = 500  # 800 px
        self.img_height = 720  # 1080 px
        self.jpg_tag_height = 80
        self.jpg_print_massage_height = 50
        self.x_offset = 20
        self.y_offset = 20
        self.img_gap = 30
        self.text_gap = 5
        self.width_img_num = 3

        self.action_color = (0, 0, 255)  # red
        self.state_color = (255, 0, 0)  # blue

        self.k_t = int(self.img_width / (history_len * self.dt))

        self.title_text_size = self.jpg_tag_height * 0.5 / 30 * 0.9
        self.print_massage_text_size = self.jpg_print_massage_height * 0.5 / 30 * 0.9
        self.tag_text_size = self.img_height * 0.03 / 30 * 0.8
        # image size
        self.width = (
            self.img_width + self.img_gap
        ) * self.width_img_num + self.y_offset
        self.height = (
            (self.img_height + self.img_gap)
            * math.ceil(self.data_dim / self.width_img_num)
            + self.x_offset
            + self.jpg_tag_height
            + self.jpg_print_massage_height
        )
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        # history data
        self.history_len = history_len
        self.timestamps = deque(maxlen=history_len)
        self.state_history = [
            deque(maxlen=history_len) for _ in range(data_dim)
        ]  # state
        self.action_history = [
            deque(maxlen=history_len) for _ in range(data_dim)
        ]  # action

        # start time of beginning of the episode
        self.start_time_ref = time.time()
        print("data monitor init, data_name:", data_name)

    def _set_default_data_names(self):
        return [f"dim{i}" for i in range(self.data_dim)]

    def _img_background_render(self, img, cor_x, cor_y, img_height, img_width):
        # set sub image background
        cv2.rectangle(
            img,
            (cor_x, cor_y),
            (cor_x + img_width, cor_y + img_height),
            (255, 255, 255),
            -1,
        )

    def _monitor_miss_data_warning(self, delta_time, img, img_height):
        # if the delta time is greater than 1.5 times the control frequency
        if delta_time > self.dt * 1.5:
            cv2.putText(
                img,
                f"WARNING: Miss data! delta_time: {delta_time:.3f}, control_delta_time: {self.control_delta_time:.3f}",
                (self.text_gap, int(img_height - self.jpg_print_massage_height * 0.5)),
                self.font,
                self.print_massage_text_size,
                (0, 0, 255),
                1,
                cv2.LINE_AA,
            )

    def _resize_y_data(self, cor_y, img_height, state_data, action_data):
        ys_state = np.array(state_data)
        ys_action = np.array(action_data)
        max_state = np.max(ys_state)
        min_state = np.min(ys_state)
        max_action = np.max(ys_action)
        min_action = np.min(ys_action)
        max_data = min(max(max_state, max_action), 100000)
        min_data = max(min(min_state, min_action), -100000)
        mid_data = (max_data + min_data) / 2
        delta_data = max((max_data - min_data) * 1.1, 0.1)

        y_coords_state = (
            (mid_data - ys_state) / delta_data * img_height + cor_y + 0.5 * img_height
        ).astype(int)
        y_coords_action = (
            (mid_data - ys_action) / delta_data * img_height + cor_y + 0.5 * img_height
        ).astype(int)

        return y_coords_state, y_coords_action, mid_data, delta_data

    def _resize_x_data(self, cor_x, img_width, timestamps):
        xs = np.array(timestamps) - timestamps[-1]
        xs = xs * self.k_t + cor_x + img_width
        x_coords = np.clip(xs, cor_x, cor_x + img_width).astype(int)

        return x_coords

    def _plot_x_ref_line(
        self,
        img,
        cor_x,
        cor_y,
        img_width,
        img_height,
        max_x,
        delta_x,
        data_line_flag=[0.25, 0.5, 0.75],
    ):
        for data_line in data_line_flag:
            line_x = int(data_line * img_width) + cor_x
            cv2.line(
                img,
                (line_x, cor_y),
                (line_x, cor_y + img_height),
                (80, 80, 80),
                1,
            )
            # print x tag
            cv2.putText(
                img,
                "%.3f" % (max_x - (1 - data_line) * delta_x),
                (line_x + self.text_gap, cor_y + img_height - self.text_gap),
                self.font,
                self.tag_text_size,
                (0, 0, 0),
                1,
                cv2.LINE_AA,
            )

    def _plot_y_ref_line(
        self,
        img,
        cor_x,
        cor_y,
        img_width,
        img_height,
        mid_y,
        delta_y,
        data_line_flag=[0.25, 0.5, 0.75],
    ):
        for data_line in data_line_flag:
            line_y = int(data_line * img_height) + cor_y
            cv2.line(
                img,
                (cor_x, line_y),
                (cor_x + img_width, line_y),
                (80, 80, 80),
                1,
            )
            # print y tag
            cv2.putText(
                img,
                "%.3f" % (mid_y + (0.5 - data_line) * delta_y),
                (cor_x + self.text_gap, line_y - self.text_gap),
                self.font,
                self.tag_text_size,
                (0, 0, 0),
                1,
                cv2.LINE_AA,
            )

    def start(self):
        """Start the monitor thread"""
        if self.running:
            return
        self.running = True
        self.thread = Thread(target=self._run, daemon=True)
        self.thread.start()
        print("robot monitor Started and you MUST stop it by keyboard - q")

    def stop(self):
        """Stop the monitor thread"""
        self.running = False
        self.thread.join(timeout=1)

    def update(self, action, state, log="OK"):
        """
        Update the monitor with the latest data.

        action: the target data
        state: the state data
        log: str, status message
        """

        data = {
            "action": action,
            "state": state,
            "log": log,
            "timestamp": time.time(),
        }

        # update data_dim（if needed）
        if len(action) != self.data_dim:
            self.data_dim = len(action)
            self.data_dim_names = self._set_default_data_names()
        if len(state) != self.data_dim:
            raise ValueError("data monitor: state dimension mismatch")

        self.control_delta_time = data["timestamp"] - self.prev_time
        self.prev_time = data["timestamp"]

        try:
            self.data_queue.put_nowait(data)
        except:  # queue full
            pass  # drop old frame

    def _run(self):
        """GUI update loop with joint trajectory curves"""
        try:
            self.init_window()

            while self.running:
                not_update = self.run_once()
                if not_update:
                    time.sleep(self.dt)
                key = cv2.waitKey(1)
                if key == ord("q"):
                    self.running = False
                if key == ord(" "):
                    self.paused = not self.paused

            cv2.destroyAllWindows()
        except Exception as e:
            print(
                "data monitor thread stopped! Can't use cv2 in this thead, Error:",
                e,
            )
        finally:
            cv2.destroyAllWindows()

    def init_window(self):
        # create window
        cv2.namedWindow(self.data_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.data_name, self.width, self.height)
        # initial time
        self.start_time_ref = time.time()

    def run_once(self):
        """GUI update loop with joint trajectory curves"""

        # update data
        data = None
        while not self.data_queue.empty():
            try:
                data = self.data_queue.get_nowait()
            except:
                break
        if data is None:
            # time.sleep(self.dt)
            return True

        # print("data: ", data)
        # image initial
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        img[:] = (200, 200, 200)
        # timestamp ref
        elapsed = data["timestamp"] - self.start_time_ref
        # update history data
        for i in range(self.data_dim):
            if i >= len(self.state_history):
                self.state_history.append(deque(maxlen=self.history_len))
                self.action_history.append(deque(maxlen=self.history_len))
            self.state_history[i].append(data["state"][i])
            self.action_history[i].append(data["action"][i])
        self.timestamps.append(elapsed)
        y_offset = int(self.jpg_tag_height / 2)
        # text
        cv2.putText(
            img,
            "Robot State Monitor: " + self.data_name,
            (20, y_offset + self.y_offset),
            self.font,
            self.title_text_size,
            (0, 0, 0),
            2,
            cv2.LINE_AA,
        )
        delta_time = 0
        if len(self.timestamps) > 1:
            delta_time = self.timestamps[-1] - self.timestamps[-2]
        self._monitor_miss_data_warning(delta_time, img, self.height)
        y_offset += self.jpg_tag_height
        cols = self.width_img_num
        for i in range(self.data_dim):
            col_idx = i % cols
            row_idx = i // cols
            x_start = self.x_offset + col_idx * (self.img_width + self.img_gap)
            y_start = y_offset + row_idx * (self.img_height + self.img_gap)
            self._img_background_render(
                img, x_start, y_start, self.img_height, self.img_width
            )
            x_coords = self._resize_x_data(x_start, self.img_width, self.timestamps)
            y_coords_state, y_coords_action, mid_y, delta_y = self._resize_y_data(
                y_start,
                self.img_height,
                self.state_history[i],
                self.action_history[i],
            )
            self._plot_x_ref_line(
                img=img,
                cor_x=x_start,
                cor_y=y_start,
                img_width=self.img_width,
                img_height=self.img_height,
                max_x=self.timestamps[-1],
                delta_x=self.history_len * self.dt,
                data_line_flag=[0.0, 0.25, 0.5, 0.75],
            )
            self._plot_y_ref_line(
                img=img,
                cor_x=x_start,
                cor_y=y_start,
                img_width=self.img_width,
                img_height=self.img_height,
                mid_y=mid_y,
                delta_y=delta_y,
                data_line_flag=[0.25, 0.5, 0.75],
            )
            # valid x mask
            valid_mask = (x_coords >= x_start) & (x_coords <= x_start + self.img_width)
            pts_state = np.array(
                [
                    [x, y]
                    for x, y, valid in zip(x_coords, y_coords_state, valid_mask)
                    if valid
                ],
                dtype=np.int32,
            )
            pts_action = np.array(
                [
                    [x, y]
                    for x, y, valid in zip(x_coords, y_coords_action, valid_mask)
                    if valid
                ],
                dtype=np.int32,
            )
            # plot action and state
            if len(pts_state) > 1:
                cv2.polylines(img, [pts_state], False, self.state_color, 2)
            if len(pts_action) > 1:
                cv2.polylines(img, [pts_action], False, self.action_color, 1)
            # print joint state
            current_state = data["state"][i]
            current_action = data["action"][i]
            cv2.putText(
                img,
                f"state {self.data_dim_names[i]}: {current_state:+6.3f}",
                (x_start + self.text_gap, y_start + int(self.tag_text_size * 30)),
                self.font,
                self.tag_text_size,
                self.state_color,
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                img,
                f"action {self.data_dim_names[i]}: {current_action:+6.3f}",
                (x_start + self.text_gap, y_start + int(self.tag_text_size * 60)),
                self.font,
                self.tag_text_size,
                self.action_color,
                1,
                cv2.LINE_AA,
            )
        # show image
        if not self.paused:
            cv2.imshow(self.data_name, img)

        return False
