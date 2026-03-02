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


def process_meta_episodes_stats(meta_input_path, meta_output_path):
    """
    处理 meta/episodes_stats.jsonl 文件，对其中的统计信息进行转换
    """
    if not meta_input_path.exists():
        logging.warning(f"Meta episodes stats file does not exist: {meta_input_path}")
        return

    logging.info(f"Processing meta episodes stats: {meta_input_path}")

    with open(meta_input_path, "r", encoding="utf-8") as input_file:
        with open(meta_output_path, "w", encoding="utf-8") as output_file:
            for line_num, line in enumerate(input_file, 1):
                line = line.strip()
                if not line:
                    continue

                try:
                    # 解析JSON行
                    json_obj = json.loads(line)

                    # 检查是否包含stats字段
                    if "stats" in json_obj and isinstance(json_obj["stats"], dict):
                        stats_dict = json_obj["stats"]

                        # 检查stats字典中是否有需要转换的键
                        for key in transform_keys:
                            if key in stats_dict and isinstance(stats_dict[key], dict):
                                # 检查是否包含统计信息（min, max, mean）
                                stat_values = stats_dict[key]
                                if any(
                                    stat_key in stat_values for stat_key in stats_fields
                                ):
                                    # logging.info(
                                    #     f"Transforming {key} stats in line {line_num}"
                                    # )
                                    json_obj["stats"][key] = transform_stats_values(
                                        stat_values
                                    )

                    # 写入转换后的行
                    output_file.write(json.dumps(json_obj) + "\n")

                except json.JSONDecodeError as e:
                    logging.error(f"Error parsing JSON in line {line_num}: {e}")
                    # 如果解析失败，直接写入原行
                    output_file.write(line + "\n")


def copy_other_directories(input_dir, output_dir):
    """
    复制input_directory中除了data/chunk-*之外的其他目录到output_directory
    """
    input_path = Path(input_dir)
    output_path = Path(output_dir)

    # 获取input_dir下的所有项目
    for item in input_path.iterdir():
        if item.is_dir():
            # 检查是否是data/chunk-*目录
            if item.name == "data":
                # 检查data目录下是否有chunk-*子目录
                chunk_dirs = list(item.glob("chunk-*"))
                if chunk_dirs:
                    # 如果是data目录且包含chunk-*子目录，跳过（因为我们已经处理了这些）
                    continue

            # 检查是否是meta目录，特殊处理episodes_stats.jsonl
            if item.name == "meta":
                meta_dir_path = output_path / item.name
                meta_dir_path.mkdir(parents=True, exist_ok=True)

                # 复制meta目录下所有内容到目标目录
                for sub_item in item.iterdir():
                    dest_sub_path = meta_dir_path / sub_item.name

                    if sub_item.is_file():
                        # 如果是文件，直接复制
                        shutil.copy2(sub_item, dest_sub_path)
                        logging.info(f"Copying file: {sub_item} -> {dest_sub_path}")

                        # 如果是episodes_stats.jsonl文件，需要特殊处理
                        if sub_item.name == "episodes_stats.jsonl":
                            # 先删除刚复制的文件，用处理后的文件替代
                            dest_sub_path.unlink()
                            process_meta_episodes_stats(sub_item, dest_sub_path)
                    elif sub_item.is_dir():
                        # 如果是子目录，递归复制
                        if dest_sub_path.exists():
                            shutil.rmtree(dest_sub_path)
                        shutil.copytree(sub_item, dest_sub_path)
                        logging.info(
                            f"Copying subdirectory: {sub_item} -> {dest_sub_path}"
                        )
            else:
                # 复制其他目录
                dest_path = output_path / item.name
                logging.info(f"Copying directory: {item} -> {dest_path}")

                if dest_path.exists():
                    shutil.rmtree(dest_path)  # 删除已存在的目录

                shutil.copytree(item, dest_path)


def process_directory(input_dir, output_dir):
    """
    处理输入目录中的所有parquet文件
    """
    input_path = Path(input_dir)
    output_path = Path(output_dir)

    # 创建输出目录
    output_path.mkdir(parents=True, exist_ok=True)

    # 查找data/chunk-*目录
    chunk_dirs = list(input_path.glob("data/chunk-*"))

    if not chunk_dirs:
        logging.warning(f"No chunk directories found in {input_path}/data/")
    else:
        for chunk_dir in chunk_dirs:
            logging.info(f"Processing chunk directory: {chunk_dir}")

            # 为每个chunk目录创建输出目录
            relative_chunk_path = chunk_dir.relative_to(input_path)
            output_chunk_dir = output_path / relative_chunk_path
            output_chunk_dir.mkdir(parents=True, exist_ok=True)

            # 查找episode_*.parquet文件
            parquet_files = list(chunk_dir.glob("episode_*.parquet"))

            for parquet_file in parquet_files:
                # 构建输出文件路径
                output_file = output_chunk_dir / parquet_file.name

                try:
                    process_parquet_file(parquet_file, output_file)
                except Exception as e:
                    logging.error(f"Error processing {parquet_file}: {str(e)}")

    # 复制其他目录到输出目录
    copy_other_directories(input_dir, output_dir)


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print(f"Usage: python {__file__}.py <input_directory>")
        sys.exit(1)
    elif len(sys.argv) == 2:
        input_directory = sys.argv[1]
        output_directory = input_directory.rstrip("/") + "_rad"
    elif len(sys.argv) == 3:
        input_directory = sys.argv[1]
        output_directory = sys.argv[2]

    logging.info(f"Converting data from {input_directory} to {output_directory}")
    process_directory(input_directory, output_directory)
    logging.info("Conversion completed!")
