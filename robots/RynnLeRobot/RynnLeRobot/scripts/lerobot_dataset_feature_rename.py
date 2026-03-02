import os
import pandas as pd
import pyarrow.parquet as pq
import json
from pathlib import Path
import shutil
import re
import logging

logging.basicConfig(
    level=logging.INFO, force=True, format="[%(asctime)s](%(levelname)s) %(message)s"
)

# 定义需要替换的键
replace_keys = {
    "observation.state.joint": "observation.state",
    "action.joint": "action",
}


def replace_keys_in_dict(obj):
    """
    递归地在字典中替换指定的键
    """
    if isinstance(obj, dict):
        new_dict = {}
        for key, value in obj.items():
            # 替换键名
            new_key = replace_keys.get(key, key)
            # 递归处理值
            new_dict[new_key] = replace_keys_in_dict(value)
        return new_dict
    elif isinstance(obj, list):
        return [replace_keys_in_dict(item) for item in obj]
    else:
        return obj


def replace_keys_in_json_string(json_str):
    """
    在JSON字符串中替换键名
    """
    try:
        parsed_data = json.loads(json_str)
        replaced_data = replace_keys_in_dict(parsed_data)
        return json.dumps(replaced_data)
    except (json.JSONDecodeError, TypeError):
        # 如果不是有效的JSON，直接返回原值
        return json_str


def process_parquet_file(file_path, output_file_path):
    """
    处理单个parquet文件，替换其中的键名
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

        # 替换字典中的键
        processed_row = replace_keys_in_dict(row_dict)
        processed_rows.append(processed_row)

    # 创建新的DataFrame
    processed_df = pd.DataFrame(processed_rows)

    # 保存到新文件
    processed_df.to_parquet(output_file_path, engine="pyarrow")
    logging.info(f"Saved to:   {output_file_path}")


def process_json_file(file_path, output_file_path):
    """
    处理JSON文件，替换其中的键名
    """
    logging.info(f"Processing: {file_path}")

    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read().strip()

    # 检查文件扩展名以确定处理方式
    if file_path.suffix == ".jsonl" or (file_path.name.endswith(".jsonl")):
        # 处理JSONL文件（每行一个JSON对象）
        lines = content.split("\n")
        processed_lines = []

        for line in lines:
            line = line.strip()
            if not line:
                continue

            try:
                # 解析JSON行
                json_obj = json.loads(line)
                # 替换键
                replaced_obj = replace_keys_in_dict(json_obj)
                # 添加到结果
                processed_lines.append(json.dumps(replaced_obj))
            except json.JSONDecodeError:
                # 如果不是有效的JSON行，保持原样
                processed_lines.append(line)

        # 写入处理后的内容
        with open(output_file_path, "w", encoding="utf-8") as f:
            f.write("\n".join(processed_lines))
    else:
        # 处理普通JSON文件
        try:
            json_obj = json.loads(content)
            # 替换键
            replaced_obj = replace_keys_in_dict(json_obj)
            # 保存时使用4个空格缩进
            with open(output_file_path, "w", encoding="utf-8") as f:
                json.dump(replaced_obj, f, indent=4, ensure_ascii=False)
        except json.JSONDecodeError:
            # 如果整个文件不是有效的JSON，尝试按行处理
            lines = content.split("\n")
            processed_lines = []

            for line in lines:
                line = line.strip()
                if not line:
                    continue

                try:
                    # 解析JSON行
                    json_obj = json.loads(line)
                    # 替换键
                    replaced_obj = replace_keys_in_dict(json_obj)
                    # 添加到结果
                    processed_lines.append(json.dumps(replaced_obj))
                except json.JSONDecodeError:
                    # 如果不是有效的JSON行，保持原样
                    processed_lines.append(line)

            # 写入处理后的内容
            with open(output_file_path, "w", encoding="utf-8") as f:
                f.write("\n".join(processed_lines))

    logging.info(f"Saved to:   {output_file_path}")


def copy_and_replace_keys(input_dir, output_dir):
    """
    复制目录并替换特定文件中的键名
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

            # 检查是否是需要特殊处理的文件
            if is_target_file(rel_path, file_name):
                logging.info(f"Replacing keys in: {input_file_path}")

                if file_name.endswith(".parquet"):
                    process_parquet_file(input_file_path, output_file_path)
                elif file_name.endswith(".json") or file_name.endswith(".jsonl"):
                    process_json_file(input_file_path, output_file_path)
                else:
                    # 对于其他类型的文件，直接复制
                    shutil.copy2(input_file_path, output_file_path)
            else:
                # 直接复制其他文件
                shutil.copy2(input_file_path, output_file_path)


def is_target_file(rel_path, file_name):
    """
    判断文件是否为目标文件（需要替换键名的文件）
    """
    full_path = rel_path / file_name

    # 检查是否是目标文件路径
    target_patterns = [Path("meta/episodes_stats.jsonl"), Path("meta/info.json")]

    # 检查是否是 data/chunk-数字/episode_数字.parquet 格式的文件
    if (
        rel_path.parts
        and len(rel_path.parts) >= 2
        and rel_path.parts[0] == "data"
        and rel_path.parts[1].startswith("chunk-")
        and file_name.startswith("episode_")
        and file_name.endswith(".parquet")
    ):

        # 验证chunk部分和episode部分是否为数字
        chunk_part = rel_path.parts[1][6:]  # 移除"chunk-"前缀
        episode_part = file_name[8:-8]  # 移除"episode_"前缀和".parquet"后缀

        if chunk_part.isdigit() and episode_part.isdigit():
            return True

    # 检查是否是meta目录下的特定文件
    if (
        len(rel_path.parts) == 1
        and rel_path.parts[0] == "meta"
        and file_name in ["episodes_stats.jsonl", "info.json"]
    ):
        return True

    return False


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print(f"Usage: python {__file__}.py <input_directory>")
        sys.exit(1)
    elif len(sys.argv) == 2:
        input_directory = sys.argv[1]
        output_directory = (
            input_directory.rstrip("/") + "_rename"
        )  # 使用_deg作为后缀表示角度单位
    elif len(sys.argv) == 3:
        input_directory = sys.argv[1]
        output_directory = sys.argv[2]

    logging.info(f"Replacing keys in data from {input_directory} to {output_directory}")
    copy_and_replace_keys(input_directory, output_directory)
    logging.info("Key replacement completed!")
