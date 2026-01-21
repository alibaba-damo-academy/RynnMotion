import torch
import numpy as np
from scipy.interpolate import interp1d


def interpolate_tensor(
    input_tensor: torch.Tensor, k: int, kind: str = "linear", device: str = "cpu"
) -> torch.Tensor:
    """
    对一个 PyTorch Tensor 的每一列进行插值。

    Args:
        input_tensor (torch.Tensor): 输入的张量，shape 为 (n, m)，例如 (20, 6)。
        k (int): 插值系数。每个原始间隔中插入 k-1 个点。
        kind (str, optional): 插值的类型。默认为 'linear'（线性插值）。
            其他常用选项包括: 'quadratic' (二次), 'cubic' (三次样条).
        device (str, optional): 指定输出 Tensor 所在的设备 ('cpu' 或 'cuda')。
                                默认为 'cpu'。

    Returns:
        torch.Tensor: 插值后的张量，shape 为 (n*k, m)。

    Raises:
        ValueError: 如果 k 不是正整数。
    """
    if not isinstance(k, int) or k <= 0:
        raise ValueError("插值系数 k 必须是一个正整数。")

    if k == 1:
        return input_tensor.clone()  # 如果 k=1，无需插值，直接返回副本

    # 将输入的 torch.Tensor 转换为 numpy.ndarray 以便使用 scipy
    # .cpu() 确保数据在CPU上，因为scipy不能直接操作CUDA张量
    input_array = input_tensor.cpu().numpy()

    # 原始数据的行数和列数
    n_rows, n_cols = input_array.shape

    # 1. 创建原始的 "时间点" 序列 (0, 1, 2, ..., n-1)
    original_indices = np.arange(n_rows)

    # 2. 创建新的、更密集的 "时间点" 序列
    new_length = n_rows * k
    interpolated_indices = np.linspace(0, n_rows - 1, new_length)

    # 3. 准备一个空的 numpy 数组来存放结果
    interpolated_array = np.zeros((new_length, n_cols))

    # 4. 对每一列进行独立的插值
    for i in range(n_cols):
        y = input_array[:, i]
        f = interp1d(original_indices, y, kind=kind)
        interpolated_array[:, i] = f(interpolated_indices)

    # 5. 将插值结果从 numpy.ndarray 转换回 torch.Tensor
    output_tensor = torch.from_numpy(interpolated_array).to(input_tensor.dtype)

    # 将最终的 tensor 移动到指定的设备
    return output_tensor.to(device)


def interpolate_array(
    input_array: np.ndarray, k: int, kind: str = "linear"
) -> np.ndarray:
    """
    对一个 numpy 数组的每一列进行插值。

    Args:
        input_array (np.ndarray): 输入的数组，shape 为 (n, m)，其中 n=20, m=6。
        k (int): 插值系数。每个原始间隔中插入 k-1 个点。
        kind (str, optional): 插值的类型。默认为 'linear'（线性插值）。
            其他常用选项包括: 'quadratic' (二次), 'cubic' (三次样条).

    Returns:
        np.ndarray: 插值后的数组，shape 为 (n*k, m)。

    Raises:
        ValueError: 如果 k 不是正整数。
    """
    if not isinstance(k, int) or k <= 0:
        raise ValueError("插值系数 k 必须是一个正整数。")

    if k == 1:
        return input_array.copy()  # 如果 k=1，无需插值，直接返回副本

    # 原始数据的行数和列数
    n_rows, n_cols = input_array.shape

    # 1. 创建原始的 "时间点" 序列 (0, 1, 2, ..., n-1)
    original_indices = np.arange(n_rows)

    # 2. 创建新的、更密集的 "时间点" 序列
    # 我们希望在 n_rows * k 的总长度内插值，但要小心终点。
    # np.linspace 在包含终点的情况下，能更好地控制点数。
    new_length = n_rows * k
    # 终点是 n_rows - 1，因为原始索引是从0到n_rows-1
    interpolated_indices = np.linspace(0, n_rows - 1, new_length)

    # 3. 准备一个空的数组来存放结果
    interpolated_array = np.zeros((new_length, n_cols))

    # 4. 对每一列进行独立的插值
    for i in range(n_cols):
        # 获取当前列的数据
        y = input_array[:, i]

        # 创建一个插值函数
        f = interp1d(original_indices, y, kind=kind)

        # 使用插值函数在新时间点上求值
        interpolated_y = f(interpolated_indices)

        # 将结果存入输出数组
        interpolated_array[:, i] = interpolated_y

    return interpolated_array
