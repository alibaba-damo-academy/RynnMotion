import os
import pandas as pd
import pyarrow.parquet as pq
import json
import math
from pathlib import Path
import shutil
import numpy as np
import logging

logging.basicConfig(
    level=logging.INFO, force=True, format="[%(asctime)s](%(levelname)s) %(message)s"
)

transform_keys = [
    "observation.state",
    "action",
    "observation.state.joint",
    "action.joint",
]
stats_fields = ["min", "max", "mean", "std"]


def transform_joint_values(joint_list, skip_angle_offset=False):
    """
    对关节值进行指定转换
    Args:
        joint_list: 关节值列表
        skip_angle_offset: 是否跳过第2、3个值的角度偏移变换（用于std等统计量）
    """
    if not isinstance(joint_list, (list, np.ndarray)):
        return joint_list

    # 如果是numpy数组，转换为列表进行处理
    if isinstance(joint_list, np.ndarray):
        joint_list = joint_list.tolist()

    if len(joint_list) < 6:
        # 如果列表长度不足6，则扩展到至少6个元素
        extended_list = joint_list + [0.0] * (6 - len(joint_list))
    else:
        extended_list = joint_list.copy()

    result = extended_list.copy()

    # 第2个值：* (-1) + 90 (索引为1) - 对于标准差等统计量跳过此操作
    if not skip_angle_offset and len(result) >= 2:
        try:
            result[1] = float(result[1]) * -1 + 90
        except (ValueError, TypeError):
            pass

    # 第3个值：- 90 (索引为2) - 对于标准差等统计量跳过此操作
    if not skip_angle_offset and len(result) >= 3:
        try:
            result[2] = float(result[2]) - 90
        except (ValueError, TypeError):
            pass

    # 前5个值：角度转弧度
    for i in range(min(5, len(result))):
        try:
            result[i] = math.radians(float(result[i]))
        except (ValueError, TypeError):
            # 如果转换失败，保持原值
            pass

    # 第6个值：/ 100 (索引为5)
    if len(result) >= 6:
        try:
            result[5] = float(result[5]) / 100
        except (ValueError, TypeError):
            pass

    # 截断回原来的长度
    result = result[: len(joint_list)]

    # 如果原始输入是numpy数组，返回numpy数组
    if isinstance(joint_list, np.ndarray):
        return np.array(result)

    return result


def transform_stats_values(stats_dict):
    """
    对统计信息中的min, max, mean, std值进行转换
    """
    for field in stats_fields:
        if field in stats_dict and isinstance(stats_dict[field], (list, np.ndarray)):
            # std字段需要跳过角度偏移变换
            stats_dict[field] = transform_joint_values(
                stats_dict[field], skip_angle_offset=bool(field == "std")
            )

    return stats_dict


def transform_data(data_dict):
    """
    递归地转换字典中的joint数据
    """
    if isinstance(data_dict, dict):
        new_dict = {}
        for key, value in data_dict.items():
            if key in transform_keys and isinstance(value, (list, np.ndarray)):
                # 应用转换函数
                new_dict[key] = transform_joint_values(value)
            elif key in transform_keys and isinstance(value, dict):
                # 检查是否包含统计信息（min, max, mean）
                if any(stat_key in value for stat_key in stats_fields):
                    new_dict[key] = transform_stats_values(value)
                else:
                    # 递归处理action和observation中的其他嵌套结构
                    new_value = transform_data(value)
                    new_dict[key] = new_value
            elif key in ["action", "observation"] and isinstance(value, dict):
                # 递归处理action和observation中的state
                new_value = transform_data(value)
                new_dict[key] = new_value
            elif isinstance(value, (dict, list, np.ndarray)):
                # 递归处理嵌套结构
                new_dict[key] = transform_data(value)
            else:
                new_dict[key] = value
        return new_dict
    elif isinstance(data_dict, list):
        # 处理列表中的每个元素
        return [transform_data(item) for item in data_dict]
    elif isinstance(data_dict, np.ndarray):
        # 处理numpy数组
        return transform_joint_values(data_dict)
    else:
        # 基本数据类型直接返回
        return data_dict


def process_field_value(field_value):
    """
    处理字段值，根据其类型应用适当的转换
    """

    if isinstance(field_value, str):
        try:
            # 尝试解析JSON字符串
            parsed_data = json.loads(field_value)
            transformed_data = transform_data(parsed_data)
            return json.dumps(transformed_data)
        except (json.JSONDecodeError, TypeError):
            # 如果不是有效的JSON，直接返回原值
            return field_value
    elif isinstance(field_value, dict):
        return transform_data(field_value)
    elif isinstance(field_value, (list, np.ndarray)):
        return transform_joint_values(field_value)
    else:
        return field_value


def process_parquet_file(file_path, output_file_path):
    """
    处理单个parquet文件
    """
    logging.info(f"Processing: {file_path}")

    # 读取parquet文件
    table = pq.read_table(file_path)
    df = table.to_pandas()

    # 处理每一行数据
    processed_rows = []
    for index, row in df.iterrows():
        # 将每一行转换为字典
        row_dict = row.to_dict()

        # 遍历所有需要转换的键
        for key in transform_keys:
            if key in row_dict:
                original_value = row_dict[key]
                row_dict[key] = process_field_value(original_value)

        processed_rows.append(row_dict)

    # 创建新的DataFrame
    processed_df = pd.DataFrame(processed_rows)

    # 保存到新文件
    processed_df.to_parquet(output_file_path, engine="pyarrow")
    logging.info(f"Saved to:   {output_file_path}")


def is_episode_timeseries_file(rel_path, file_name):
    """
    判断文件是否为 episode_数字/timeseries.parquet 格式的文件
    """
    logging.debug(f"{rel_path=} {len(rel_path.parts)=}")
    if len(rel_path.parts) >= 2:
        # 检查是否是 episode_数字/timeseries.parquet 格式
        episode_part = rel_path.parts[-2]  # 倒数第二个部分应该是episode_数字
        filename = rel_path.parts[-1]  # 最后一个部分应该是文件名

        if (
            episode_part.startswith("episode_")
            and filename.endswith(".parquet")
            and episode_part[8:].isdigit()
        ) or filename == "timeseries.parquet":  # 检查episode_后面的部分是否为数字
            return True

    return False


def copy_and_process_directory(input_dir, output_dir):
    """
    复制目录并处理特定的parquet文件
    """
    input_path = Path(input_dir)
    output_path = Path(output_dir)

    # 创建输出目录
    output_path.mkdir(parents=True, exist_ok=True)

    # 遍历输入目录
    for root, dirs, files in os.walk(input_path):
        # 计算相对于输入目录的路径
        rel_path = Path(root).relative_to(input_path)
        current_output_dir = output_path / rel_path

        # 创建对应输出目录
        current_output_dir.mkdir(parents=True, exist_ok=True)

        for file_name in files:
            input_file_path = Path(root) / file_name
            output_file_path = current_output_dir / file_name
            logging.debug(f"{file_name=} {input_file_path=} {output_file_path=}")
            # 检查是否是需要特殊处理的文件
            if is_episode_timeseries_file(rel_path / file_name, file_name):
                logging.info(f"Processing timeseries.parquet: {input_file_path}")
                process_parquet_file(input_file_path, output_file_path)
            else:
                # 直接复制其他文件
                shutil.copy2(input_file_path, output_file_path)


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print(f"Usage: python {__file__} <input_directory>")
        sys.exit(1)
    elif len(sys.argv) == 2:
        input_directory = sys.argv[1]
        output_directory = input_directory.rstrip("/") + "_rad"
    elif len(sys.argv) == 3:
        input_directory = sys.argv[1]
        output_directory = sys.argv[2]

    logging.info(f"Converting data from {input_directory} to {output_directory}")
    copy_and_process_directory(input_directory, output_directory)
    logging.info("Conversion completed!")
