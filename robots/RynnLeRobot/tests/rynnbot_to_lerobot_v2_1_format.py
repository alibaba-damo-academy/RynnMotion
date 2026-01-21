#  Rynnbot数据格式转LerobotV2.1数据格式
import os
import json
import numpy as np
import argparse
import pandas as pd
import shutil
import logging
import sys
from pathlib import Path
from tqdm import tqdm
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

# 可选列名映射，直接在此处配置，如 {"xxx": "yyy"}
COLUMN_RENAMES = {
    "action.joint": "action",
    "observation.state.joint": "observation.state",
}


# Configure logger
logger = logging.getLogger(__name__)


# 自定义异常类，用于数据验证失败时的优雅退出
class DataValidationError(Exception):
    """数据验证失败异常"""

    def __init__(self, message, exit_code=2):
        super().__init__(message)
        self.exit_code = exit_code


def setup_logging():
    """Set log level and format"""
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s: %(message)s",
        handlers=[logging.StreamHandler()],
    )


setup_logging()

# 默认的features配置
default_features_string = """
{
    "action": {
        "dtype": "float32",
        "shape": [
            6
        ],
        "names": [
            "main_shoulder_pan",
            "main_shoulder_lift",
            "main_elbow_flex",
            "main_wrist_flex",
            "main_wrist_roll",
            "main_gripper"
        ]
    },
    "observation.state": {
        "dtype": "float32",
        "shape": [
            6
        ],
        "names": [
            "main_shoulder_pan",
            "main_shoulder_lift",
            "main_elbow_flex",
            "main_wrist_flex",
            "main_wrist_roll",
            "main_gripper"
        ]
    },
    "observation.images.wrist": {
        "dtype": "video",
        "shape": [
            480,
            640,
            3
        ],
        "names": [
            "height",
            "width",
            "channels"
        ],
        "info": {
            "video.height": 480,
            "video.width": 640,
            "video.codec": "av1",
            "video.pix_fmt": "yuv420p",
            "video.is_depth_map": false,
            "video.fps": 30,
            "video.channels": 3,
            "has_audio": false
        }
    },
    "observation.images.front": {
        "dtype": "video",
        "shape": [
            480,
            640,
            3
        ],
        "names": [
            "height",
            "width",
            "channels"
        ],
        "info": {
            "video.height": 480,
            "video.width": 640,
            "video.codec": "av1",
            "video.pix_fmt": "yuv420p",
            "video.is_depth_map": false,
            "video.fps": 30,
            "video.channels": 3,
            "has_audio": false
        }
    },
    "timestamp": {
        "dtype": "float32",
        "shape": [
            1
        ],
        "names": null
    }
}
"""


# 将features中的shape从列表格式转换为元组格式
def convert_shape_to_tuple(features_dict):
    """递归地将features字典中的shape字段从列表转换为元组"""
    for key, value in features_dict.items():
        if isinstance(value, dict):
            if "shape" in value and isinstance(value["shape"], list):
                value["shape"] = tuple(value["shape"])
            # 递归处理嵌套字典
            convert_shape_to_tuple(value)
    return features_dict


def parse_metadata_json(json_path):
    """
    解析metadata.json文件，返回所有字段和视频列表
    """
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    # 直接访问顶层字段
    task_description = data.get("task_description")
    task_prompt = data.get("task_prompt")
    total_frames = data.get("total_frames")
    fps = data.get("fps")
    collection_time = data.get("collection_time")

    return {
        "task_description": task_description,
        "task_prompt": task_prompt,
        "total_frames": total_frames,
        "fps": fps,
        "collection_time": collection_time,
    }


def convert_rynnbot_to_lerobot(output_dir, collection_dir, features=None):
    """
    Convert rynnbot-format files to LeRobot format

    Args:
        output_dir: LeRobot dataset output directory
        collection_dir: Root directory containing collections of rynnbot-format files
        features: Optional features configuration dict. If None, uses default features.
    """
    # 如果没有提供features，使用默认配置
    if features is None:
        features = json.loads(default_features_string)
        features = convert_shape_to_tuple(features)
        logger.info("使用默认的features配置")
    else:
        logger.info("使用提供的features配置")
    # Ensure output directory is an absolute path
    output_path = Path(output_dir).absolute()
    logger.info(f"Output directory: {output_path}")

    fps = 30

    # Check if output directory exists
    if output_path.exists():
        # 在自动化环境中自动删除已存在的目录
        try:
            shutil.rmtree(output_path)
            logger.info(f"Deleted existing directory: {output_path}")
        except Exception as e:
            logger.error(f"Failed to delete existing directory {output_path}: {e}")
            sys.exit(1)

    # Create LeRobot dataset
    logger.info(f"Creating LeRobot dataset FPS: {fps}")
    repo_id = os.path.basename(output_dir)
    lerobot_dataset = LeRobotDataset.create(
        repo_id=repo_id,
        root=output_path,
        fps=fps,
        features=features,
        use_videos=True,  # Use video mode
    )

    collection_dirs = [
        os.path.join(collection_dir, d)
        for d in os.listdir(collection_dir)
        if os.path.isdir(os.path.join(collection_dir, d))
    ]

    logger.info("Converting collection files to LeRobot format...")
    episode_idx_in_lerobot = 0  # Track the episode index in LeRobotDataset

    for _, collection_dir in enumerate(
        tqdm(
            collection_dirs, desc="Processing rynnbot files", total=len(collection_dirs)
        )
    ):
        logger.info(f"Processing file: {collection_dir}")

        metadata_json_path = os.path.join(collection_dir, "metadata.json")
        timeseries_path = os.path.join(collection_dir, "timeseries.parquet")
        metadata = parse_metadata_json(metadata_json_path)

        num_frames = metadata["total_frames"]
        if num_frames == 0:
            timeseries_df = pd.read_parquet(timeseries_path)
            num_frames = len(timeseries_df)

        if num_frames == 0:
            error_msg = f"数据验证失败: File {collection_dir} has 0 frames, 无法生成有效的数据集"
            logger.error(error_msg)
            raise DataValidationError(error_msg, exit_code=3)

        tasks = [metadata["task_description"]]

        written_video_files = {}

        for full_camera_key in lerobot_dataset.meta.video_keys:

            lerobot_target_video_path = lerobot_dataset.meta.get_video_file_path(
                episode_idx_in_lerobot, full_camera_key
            )
            # Ensure the target path is absolute
            if not lerobot_target_video_path.is_absolute():
                lerobot_target_video_path = (
                    lerobot_dataset.root / lerobot_target_video_path
                )

            origin_video_path = os.path.join(collection_dir, f"{full_camera_key}.mp4")

            if not os.path.exists(origin_video_path):
                error_msg = f"数据验证失败: Video file {origin_video_path} does not exist, 视频文件缺失"
                logger.error(error_msg)
                raise DataValidationError(error_msg, exit_code=5)

            lerobot_target_video_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(origin_video_path, lerobot_target_video_path)
            written_video_files[full_camera_key] = str(lerobot_target_video_path)

        if len(written_video_files) != len(lerobot_dataset.meta.video_keys):
            error_msg = f"数据验证失败: Not all videos were written for episode {episode_idx_in_lerobot}. 期望 {len(lerobot_dataset.meta.video_keys)} 个视频，实际写入 {len(written_video_files)} 个"
            logger.error(error_msg)
            raise DataValidationError(error_msg, exit_code=6)

        # 检查帧数与parquet行数是否一致
        timeseries_df = pd.read_parquet(timeseries_path)
        if COLUMN_RENAMES:
            timeseries_df = timeseries_df.rename(columns=COLUMN_RENAMES)
        if num_frames != len(timeseries_df):
            error_msg = f"数据验证失败: 帧数与parquet行数不一致，num_frames={num_frames}, parquet行数={len(timeseries_df)}，数据不一致: {collection_dir}"
            logger.error(error_msg)
            raise DataValidationError(error_msg, exit_code=7)
        # Prepare data for add_frame (including video placeholders)
        for row_dict in timeseries_df.to_dict(orient="records"):
            frame_data_for_lerobot = {}
            for key in features.keys():
                # 跳过视频特征，视频由下方占位图像或已拷贝的视频文件处理
                if features[key].get("dtype") == "video":
                    continue
                if key in row_dict:
                    value = row_dict[key]
                    # 转成np.array并reshape
                    arr = np.array(value, dtype=np.float32).reshape(
                        tuple(features[key]["shape"])
                    )
                else:
                    # 用0补齐，shape和dtype按features定义
                    shape = features[key]["shape"]
                    arr = np.zeros(shape, dtype=np.float32)
                frame_data_for_lerobot[key] = arr
            # 补充task字段
            frame_data_for_lerobot["task"] = tasks[0]
            # 不写入 timestamp，交由 LeRobotDataset 按帧索引/ fps 自动计算
            if "timestamp" in row_dict:
                frame_data_for_lerobot["timestamp"] = np.array(
                    [float(row_dict["timestamp"])], dtype=np.float32
                )

            # 为视频键提供虚拟图像数据，但标记这些键已有视频文件
            for full_video_key in lerobot_dataset.meta.video_keys:
                if (
                    full_video_key in features
                    and features[full_video_key]["dtype"] == "video"
                ):
                    # 如果已经成功写入了视频文件，我们仍然需要提供虚拟图像数据
                    # 但这些数据不会被用于编码，因为视频文件已存在
                    img_shape = features[full_video_key]["shape"]  # 例如 (H, W, C)
                    # 图像数据通常是uint8类型
                    dummy_image = np.zeros(img_shape, dtype=np.uint8)
                    frame_data_for_lerobot[full_video_key] = dummy_image

            lerobot_dataset.add_frame(frame_data_for_lerobot)

        # 手动修改episode_buffer中的timestamp值，确保它们是标量值
        if "timestamp" in lerobot_dataset.episode_buffer:
            # 如果timestamp是2D数组，将其转换为1D数组
            timestamps = lerobot_dataset.episode_buffer["timestamp"]
            if (
                isinstance(timestamps, list)
                and len(timestamps) > 0
                and hasattr(timestamps[0], "shape")
                and timestamps[0].shape == (1,)
            ):
                # 将每个元素从形状(1,)转换为标量值
                lerobot_dataset.episode_buffer["timestamp"] = [
                    float(ts[0]) for ts in timestamps
                ]
                logger.debug(f"已将episode_buffer timestamp从形状(1,)转换为标量值")

        lerobot_dataset.save_episode()
        logger.info(f"Successfully saved episode {episode_idx_in_lerobot}")
        episode_idx_in_lerobot += 1

    logger.info("All Episode Collection files converted successfully.")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Rynnbot数据集转LeRobot格式批量转换工具"
    )
    parser.add_argument(
        "--output_dir", type=str, required=True, help="LeRobot数据集输出目录"
    )
    parser.add_argument(
        "--collection_dir", type=str, required=True, help="so100rynnbot格式数据集根目录"
    )
    parser.add_argument(
        "--features",
        type=str,
        required=False,
        help="DatasetFeature JSON字符串，用于覆盖默认的features配置",
    )
    args = parser.parse_args()

    setup_logging()

    # 解析features参数
    parsed_features = None
    if args.features:
        try:
            logger.info("解析提供的features配置")
            parsed_features = json.loads(args.features)
            parsed_features = convert_shape_to_tuple(parsed_features)
            logger.info(f"成功解析features配置，包含 {len(parsed_features)} 个特征")
        except json.JSONDecodeError as e:
            logger.error(f"解析features JSON失败: {e}")
            sys.exit(1)

    try:
        convert_rynnbot_to_lerobot(
            args.output_dir, args.collection_dir, parsed_features
        )
        logger.info("数据转换成功完成")
    except DataValidationError as e:
        logger.error(f"数据转换失败: {e}")
        sys.exit(e.exit_code)
    except Exception as e:
        logger.error(f"数据转换过程中发生未预期的错误: {e}")
        sys.exit(1)
