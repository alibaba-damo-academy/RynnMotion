"""将 LeRobotDataset 样式的数据集转换为 RynnBot episode 目录结构。"""

import argparse
import glob
import json
import logging
import os
import shutil
import time
from typing import Dict, List, Optional

try:
    import pyarrow as pa  # noqa: F401
    import pyarrow.parquet as pq

    _HAS_PYARROW = True
except Exception:  # pragma: no cover - 容错，无 PyArrow 时回退 Pandas
    _HAS_PYARROW = False
    pq = None  # type: ignore

import pandas as pd

rename_map = {
    "action": "action.joint",
    "observation.state": "observation.state.joint",
}

# ---------------------------------------------------------------------------
# 基础工具
# ---------------------------------------------------------------------------


def setup_logger(debug: bool = False) -> logging.Logger:
    level = logging.DEBUG if debug else logging.INFO
    logging.basicConfig(level=level, format="%(levelname)s: %(message)s")
    return logging.getLogger(__name__)


def ensure_dir(path: str) -> str:
    os.makedirs(path, exist_ok=True)
    return path


# ---------------------------------------------------------------------------
# LeRobot 元数据读取
# ---------------------------------------------------------------------------


def load_info(dataset_path: str) -> Dict:
    info_path = os.path.join(dataset_path, "meta", "info.json")
    if not os.path.exists(info_path):
        raise FileNotFoundError(f"未找到 info.json: {info_path}")
    with open(info_path, "r") as f:
        try:
            info = json.load(f)
        except json.JSONDecodeError as exc:
            raise SystemExit(f"解析 info.json 失败: {exc}") from exc

    version = info.get("codebase_version")
    if version != "v2.1":
        raise SystemExit(f"仅支持 codebase_version=v2.1，当前为: {version!r}")
    return info


def load_episodes_meta(dataset_path: str, logger: logging.Logger) -> Dict[int, Dict]:
    meta_path = os.path.join(dataset_path, "meta", "episodes.jsonl")
    result: Dict[int, Dict] = {}
    if not os.path.exists(meta_path):
        logger.warning(f"episodes.jsonl 不存在: {meta_path}")
        return result
    with open(meta_path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError as exc:  # pragma: no cover - 避免异常
                logger.error(f"解析 episodes.jsonl 行失败: {exc}")
                raise SystemExit(1) from exc
            if "episode_index" in obj:
                result[obj["episode_index"]] = obj
    return result


def infer_camera_keys(
    info: Dict, dataset_path: str, logger: logging.Logger
) -> List[str]:
    features = info.get("features", {})
    keys = [
        k
        for k, v in features.items()
        if isinstance(v, dict) and v.get("dtype") == "video"
    ]
    if keys:
        return keys

    videos_root = os.path.join(dataset_path, "videos")
    if not os.path.isdir(videos_root):
        logger.warning("未找到 videos 目录，无法推断相机列表")
        return []

    pattern = os.path.join(videos_root, "**", "episode_*.mp4")
    matches = glob.glob(pattern, recursive=True)
    fallback = sorted({os.path.basename(os.path.dirname(p)) for p in matches})
    if not fallback:
        logger.warning("未能从 videos 目录推断相机列表")
    return fallback


# ---------------------------------------------------------------------------
# 数据文件解析
# ---------------------------------------------------------------------------


def compute_chunk_index(episode_index: int, chunks_size: Optional[int]) -> int:
    if not chunks_size or chunks_size <= 0:
        return 0
    return episode_index // chunks_size


def resolve_path_from_template(
    dataset_path: str,
    template: Optional[str],
    episode_index: int,
    chunk_index: int,
    **kwargs,
) -> Optional[str]:
    if not template:
        return None
    try:
        rel_path = template.format(
            episode_chunk=chunk_index,
            episode_index=episode_index,
            **kwargs,
        )
    except KeyError:
        return None
    abs_path = os.path.join(dataset_path, rel_path)
    return abs_path if os.path.exists(abs_path) else None


def find_episode_parquet(dataset_path: str, episode_index: int) -> Optional[str]:
    pattern = os.path.join(
        dataset_path, "data", "**", f"episode_{episode_index:06d}.parquet"
    )
    matches = glob.glob(pattern, recursive=True)
    return matches[0] if matches else None


def find_episode_video(
    dataset_path: str, camera_key: str, episode_index: int
) -> Optional[str]:
    pattern = os.path.join(
        dataset_path,
        "videos",
        "**",
        camera_key,
        f"episode_{episode_index:06d}.*",
    )
    matches = glob.glob(pattern, recursive=True)
    return matches[0] if matches else None


def rewrite_parquet_with_renamed_columns(
    parquet_src: str,
    parquet_dst: str,
    logger: logging.Logger,
) -> int:
    """重写 parquet 列名并返回帧数。优先使用 PyArrow，失败回退 Pandas。"""

    if _HAS_PYARROW and pq is not None:
        try:
            table = pq.read_table(parquet_src)
            num_rows = table.num_rows
            new_cols = [rename_map.get(name, name) for name in table.column_names]
            if new_cols != table.column_names:
                table = table.rename_columns(new_cols)
                pq.write_table(table, parquet_dst)
            else:
                shutil.copy2(parquet_src, parquet_dst)
            return num_rows
        except Exception as exc:  # pragma: no cover - 容错
            logger.warning(f"PyArrow 重写 parquet 失败，回退到 Pandas：{exc}")

    try:
        df = pd.read_parquet(parquet_src)
        num_rows = len(df)
        existing_map = {
            src: dst for src, dst in rename_map.items() if src in df.columns
        }
        if existing_map:
            df = df.rename(columns=existing_map)
            df.to_parquet(parquet_dst, index=False)
        else:
            shutil.copy2(parquet_src, parquet_dst)
        return num_rows
    except Exception as exc:  # pragma: no cover - 容错
        logger.error(f"Pandas 重写 parquet 失败，直接复制：{exc}")
        shutil.copy2(parquet_src, parquet_dst)
        return 0


# ---------------------------------------------------------------------------
# 主转换逻辑
# ---------------------------------------------------------------------------


def convert_lerobot_to_rynnbot_format(
    dataset_path: str,
    output_dir: str,
    collection_time_now: bool = False,
    debug: bool = False,
):
    logger = setup_logger(debug)
    dataset_path = os.path.abspath(dataset_path)
    output_dir = os.path.abspath(output_dir)
    ensure_dir(output_dir)

    if not os.path.exists(dataset_path):
        raise FileNotFoundError(f"数据集路径不存在: {dataset_path}")

    info = load_info(dataset_path)
    episodes_meta = load_episodes_meta(dataset_path, logger)

    total_episodes = info.get("total_episodes")
    if total_episodes is None and episodes_meta:
        total_episodes = max(episodes_meta.keys()) + 1
    if total_episodes is None:
        raise ValueError("无法确定总 episode 数，请检查 meta 信息")

    chunks_size = (
        info.get("chunks_size") or info.get("chunk_size") or info.get("chunks_size", 0)
    )
    data_template = info.get("data_path")
    video_template = info.get("video_path")
    fps = info.get("fps")

    camera_keys = infer_camera_keys(info, dataset_path, logger)
    if not camera_keys:
        logger.warning("未发现相机信息，将跳过视频复制")

    # 确定需要处理的 episode 列表
    indices = list(range(total_episodes))

    logger.info(
        "开始转换，共 %d 个 episode，实际处理 %d 个",
        total_episodes,
        len(indices),
    )

    for idx in indices:
        chunk_index = compute_chunk_index(idx, chunks_size)

        parquet_path = resolve_path_from_template(
            dataset_path,
            data_template,
            episode_index=idx,
            chunk_index=chunk_index,
        )
        if parquet_path is None:
            parquet_path = find_episode_parquet(dataset_path, idx)
        if parquet_path is None:
            logger.warning(f"episode_{idx:06d} 未找到 parquet 文件，跳过")
            continue

        ep_dir = ensure_dir(os.path.join(output_dir, f"episode_{idx:06d}"))
        logger.info("处理 episode_%06d -> %s", idx, ep_dir)

        parquet_dst = os.path.join(ep_dir, "timeseries.parquet")
        total_frames = rewrite_parquet_with_renamed_columns(
            parquet_path, parquet_dst, logger
        )

        meta = episodes_meta.get(idx, {})
        if not total_frames:
            total_frames = int(meta.get("length", 0))
        else:
            total_frames = int(total_frames)

        for camera_key in camera_keys:
            video_src = resolve_path_from_template(
                dataset_path,
                video_template,
                episode_index=idx,
                chunk_index=chunk_index,
                video_key=camera_key,
            )
            if video_src is None:
                video_src = find_episode_video(dataset_path, camera_key, idx)

            if not video_src or not os.path.exists(video_src):
                logger.warning(
                    "未找到视频: episode=%s, camera=%s",
                    f"{idx:06d}",
                    camera_key,
                )
                continue

            ext = os.path.splitext(video_src)[1] or ".mp4"
            dst_name = f"{camera_key}{ext}"
            dst_path = os.path.join(ep_dir, dst_name)
            ensure_dir(os.path.dirname(dst_path))
            shutil.copy2(video_src, dst_path)
            logger.debug("已复制视频: %s -> %s", video_src, dst_path)

        if collection_time_now:
            collection_time = int(time.time())
        else:
            collection_time = meta.get("timestamp")

        if total_frames == 0 and meta.get("length"):
            total_frames = int(meta["length"])

        tasks = meta.get("tasks") or ["Unknown"]
        if isinstance(tasks, list) and tasks:
            task_name = tasks[0]
        else:
            task_name = str(tasks) if tasks else "Unknown"

        metadata = {
            "task_description": task_name,
            "task_prompt": task_name,
            "total_frames": total_frames,
            "fps": fps,
            "collection_time": collection_time,
        }

        metadata_path = os.path.join(ep_dir, "metadata.json")
        with open(metadata_path, "w") as f:
            json.dump(metadata, f, indent=2, ensure_ascii=False)
        logger.info("已写入 metadata: %s", metadata_path)

    logger.info("完成转换，输出目录: %s", output_dir)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "将 LeRobot 风格的数据集导出为按 episode 组织的 RynnBot 格式"
            "（metadata.json、相机视频、timeseries.parquet）"
        )
    )
    parser.add_argument("dataset_path", help="LeRobot 数据集根目录")
    parser.add_argument("output_dir", help="导出根目录")
    parser.add_argument(
        "--collection_time_now",
        action="store_true",
        help="使用当前时间(Unix timestamp)作为 collection_time",
    )
    parser.add_argument("--debug", action="store_true", help="启用调试日志")
    args = parser.parse_args()

    convert_lerobot_to_rynnbot_format(
        dataset_path=args.dataset_path,
        output_dir=args.output_dir,
        collection_time_now=args.collection_time_now,
        debug=args.debug,
    )


if __name__ == "__main__":  # pragma: no cover - CLI 入口
    main()
