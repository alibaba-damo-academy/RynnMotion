import argparse
import json
import sys
from typing import List


def get_dataset_feature_names(
    json_file_path: str, feature: str, name_substring: str
) -> List[str]:
    """
    从给定的 JSON 文件中，查找 'features' 字典下所有包含特定子字符串的 key。

    Args:
        json_file_path (str): info.json 文件的路径。
        name_substring (str): 用于匹配 key 的部分字符串。

    Returns:
        List[str]: 包含所有匹配 key 的字符串列表。如果找不到匹配项或 'features' 不存在，则返回空列表。

    Raises:
        FileNotFoundError: 如果指定的 json_file_path 不存在。
        json.JSONDecodeError: 如果文件内容不是有效的 JSON 格式。
        KeyError: 如果 JSON 数据中不存在 'features' 这个顶层 key。
        TypeError: 如果 'features' 对应的值不是一个字典。
    """
    # 1. 读取并打开 JSON 文件
    with open(json_file_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    # 2. 检查 'features' key 是否存在且为字典
    if feature not in data:
        raise KeyError(f"JSON 文件 '{json_file_path}' 中缺少顶层 key='{feature}'。")

    features_dict = data[feature]

    if not isinstance(features_dict, dict):
        raise TypeError(
            f"在 '{json_file_path}' 中, 'features' 对应的值必须是一个字典，但实际类型为 {type(features_dict)}。"
        )

    # 3. 遍历 'features' 字典的 key 并进行匹配
    matching_keys = [key for key in features_dict.keys() if name_substring in key]

    # 4. 返回结果列表
    return matching_keys


def main():
    """
    命令行入口函数。
    解析参数，调用核心逻辑，并处理输出和错误。
    """
    # 1. 设置命令行参数解析
    parser = argparse.ArgumentParser(
        description="在数据文件的 info.json 中查找匹配的 'features' key。"
    )
    parser.add_argument("json_path", type=str, help="要检查的 info.json 文件路径。")
    parser.add_argument("feature", type=str, help="要检查 feature 名字")
    parser.add_argument(
        "name_substring", type=str, help="用于在 feature 下层搜索的子字符串。"
    )
    args = parser.parse_args()

    try:
        # 2. 调用核心逻辑函数
        result = get_dataset_feature_names(
            args.json_path, args.feature, args.name_substring
        )

        # 3. 打印结果到标准输出
        # print(result) 会打印出 Python 列表的表示形式，例如 ['item1', 'item2']
        # 这对于其他脚本解析非常方便
        # print(result)
        print(", ".join(result))

    except (FileNotFoundError, json.JSONDecodeError, KeyError, TypeError) as e:
        # 4. 如果发生任何预期的错误，打印到标准错误流并退出
        print(f"错误: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        # 捕获其他意外错误
        print(f"发生未知错误: {e}", file=sys.stderr)
        sys.exit(1)


# --- 标准的 Python 脚本入口 ---
if __name__ == "__main__":
    main()
