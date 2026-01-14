#!/usr/bin/env python3
"""
Feetech Servo Tool
支持 ping 扫描、设置 ID、读写配置等功能
所有输出同步写入带时间戳的日志文件

测试说明:
========
以下是一些测试命令示例，用于验证tele和robot设备的各种功能:

Robot设备测试:
-------------
1. Ping扫描:
   python3 feetech_tool.py --port /dev/ttyACM0 --dev robot --job ping

2. 设置舵机ID:
   python3 feetech_tool.py --port /dev/ttyACM0 --dev robot --job set_id --cur_id 1 --new_id 10

3. 设置舵机PID参数:
   python3 feetech_tool.py --port /dev/ttyACM0 --dev robot --job set_p --servo_id 1 --p_value 16
   python3 feetech_tool.py --port /dev/ttyACM0 --dev robot --job set_d --servo_id 1 --d_value 0
   python3 feetech_tool.py --port /dev/ttyACM0 --dev robot --job set_i --servo_id 1 --i_value 0

4. 设置舵机扭矩:
   python3 feetech_tool.py --port /dev/ttyACM0 --job set_torque --servo_id 1 --torque_value 1000

5. 读取配置:
   feetech-tool --port /dev/ttyACM0 --job read_config

6. 写入配置:
   python3 feetech_tool.py --port /dev/ttyACM0 --dev robot --job write_config

7. 工厂模式:
   feetech-tool --port /dev/ttyACM0 --dev robot --job factory_mode

Tele设备测试:
-------------
1. Ping扫描:
   python3 feetech_tool.py --port /dev/ttyACM0 --dev tele --job ping

2. 设置舵机ID:
   python3 feetech_tool.py --port /dev/ttyACM0 --dev tele --job set_id --cur_id 1 --new_id 10

3. 设置舵机PID参数:
   python3 feetech_tool.py --port /dev/ttyACM0 --dev tele --job set_p --servo_id 1 --p_value 1
   python3 feetech_tool.py --port /dev/ttyACM0 --dev tele --job set_d --servo_id 1 --d_value 0
   python3 feetech_tool.py --port /dev/ttyACM0 --dev tele --job set_i --servo_id 1 --i_value 0

4. 设置舵机扭矩:
   python3 feetech_tool.py --port /dev/ttyACM0 --job set_torque --servo_id 1 --torque_value 10

5. 读取配置:
   python3 feetech_tool.py --port /dev/ttyACM0 --dev tele --job read_config

6. 写入配置:
   python3 feetech_tool.py --port /dev/ttyACM0 --dev tele --job write_config

7. 工厂模式:
   python3 feetech_tool.py --port /dev/ttyACM0 --dev tele --job factory_mode

注意事项:
--------
1. 请根据实际连接的串口设备修改--port参数，例如/dev/ttyUSB0
2. 在执行write_config和factory_mode之前，请确保所有舵机(1-6)都已连接并上电
3. 执行write_config和factory_mode时，写入配置后会提示断电重启，请按照提示操作
4. 不同设备类型(robot/tele)使用不同的PID参数配置
5. 设置PID参数时需要指定舵机ID (--servo_id) 和对应的参数值
6. 在设置参数前确保舵机已连接并上电
"""

import argparse
import logging
import sys
import os
import signal
import time
from collections import namedtuple
from datetime import datetime
import scservo_sdk as scs               # Uses SCServo SDK library

# 定义地址信息结构
AddressInfo = namedtuple('AddressInfo', ['head', 'size'])

# Control table address
SCS_MAJ_FW_VER        = AddressInfo(0, 1)      
SCS_MIN_FW_VER        = AddressInfo(1, 1)   
SCS_MAJ_HW_VER        = AddressInfo(3, 1)      
SCS_MIN_HW_VER        = AddressInfo(4, 1)   
SCS_ID                = AddressInfo(5, 1)
SCS_MIN_ANGLE         = AddressInfo(9, 2)
SCS_MAX_ANGLE         = AddressInfo(11, 2)
SCS_MAX_TEMPE         = AddressInfo(13, 1)
SCS_MAX_V             = AddressInfo(14, 1)
SCS_MIN_V             = AddressInfo(15, 1)        
SCS_MAX_TORQUE        = AddressInfo(16, 2)
SCS_POSITION_P        = AddressInfo(21, 1)
SCS_POSITION_D        = AddressInfo(22, 1)
SCS_POSITION_I        = AddressInfo(23, 1)
SCS_TORQUE_ENABLE     = AddressInfo(40, 1)
SCS_GOAL_ACC          = AddressInfo(41, 1)
SCS_GOAL_POSITION     = AddressInfo(42, 2)
SCS_GOAL_SPEED        = AddressInfo(46, 2)
SCS_TORQUE_LIMIT      = AddressInfo(48, 2)
SCS_RES_4             = AddressInfo(54, 1)
SCS_LOCK_BIT          = AddressInfo(55, 1)
SCS_PRESENT_POSITION  = AddressInfo(56, 2)
SCS_PRESENT_VOLTAGE   = AddressInfo(62, 1)

SCS_ADDRESS_LIST = [
    ("SCS_MAJ_FW_VER", SCS_MAJ_FW_VER),
    ("SCS_MIN_FW_VER", SCS_MIN_FW_VER),
    ("SCS_MAJ_HW_VER", SCS_MAJ_HW_VER),
    ("SCS_MIN_HW_VER", SCS_MIN_HW_VER),
    ("SCS_ID", SCS_ID),
    ("SCS_MIN_ANGLE", SCS_MIN_ANGLE),
    ("SCS_MAX_ANGLE", SCS_MAX_ANGLE),
    ("SCS_MAX_TEMPE", SCS_MAX_TEMPE),
    ("SCS_MAX_V", SCS_MAX_V),
    ("SCS_MIN_V", SCS_MIN_V),
    ("SCS_MAX_TORQUE", SCS_MAX_TORQUE),
    ("SCS_TORQUE_LIMIT", SCS_TORQUE_LIMIT),
    ("SCS_POSITION_P", SCS_POSITION_P),
    ("SCS_POSITION_D", SCS_POSITION_D),
    ("SCS_POSITION_I", SCS_POSITION_I),
    ("SCS_TORQUE_ENABLE", SCS_TORQUE_ENABLE),
    ("SCS_GOAL_ACC", SCS_GOAL_ACC),
    ("SCS_GOAL_POSITION", SCS_GOAL_POSITION),
    ("SCS_GOAL_SPEED", SCS_GOAL_SPEED),
    ("SCS_RES_4", SCS_RES_4),
    ("SCS_LOCK_BIT", SCS_LOCK_BIT),
    ("SCS_PRESENT_POSITION", SCS_PRESENT_POSITION),
    ("SCS_PRESENT_VOLTAGE", SCS_PRESENT_VOLTAGE)
]

def create_config_template(max_torque=1000, p_value=16, d_value=0, i_value=0, servo_specific=None):
    """
    创建配置模板函数
    
    Args:
        max_torque (int): 默认最大扭矩值
        p_value (int): 默认P值
        d_value (int): 默认D值
        i_value (int): 默认I值
        servo_specific (dict): 特定舵机配置，如果为None则不配置
        
    Returns:
        dict: 配置字典
    """
    config = {
        "default_params": {
            "SCS_MAX_TORQUE": {
                "address": SCS_MAX_TORQUE,
                "default_value": max_torque
            },
            "SCS_POSITION_P": {
                "address": SCS_POSITION_P,
                "default_value": p_value
            },
            "SCS_POSITION_D": {
                "address": SCS_POSITION_D,
                "default_value": d_value
            },
            "SCS_POSITION_I": {
                "address": SCS_POSITION_I,
                "default_value": i_value
            }
        }
    }
    
    # 如果提供了特定舵机配置，则添加到配置中
    if servo_specific is not None:
        config["servo_specific"] = servo_specific
    else:
        config["servo_specific"] = {}
        
    return config

# 使用模板创建robot_config和tele_config
ROBOT_CONFIG = create_config_template(
    max_torque=1000, 
    p_value=16, 
    d_value=0, 
    i_value=0,
    servo_specific={
        2: {  # 舵机2的特殊配置
            "SCS_POSITION_P": {
                "value": 32
            }
        },
        3: {  # 舵机3的特殊配置
            "SCS_POSITION_P": {
                "value": 32
            }
        },
        6: {  # 舵机6的特殊配置
            "SCS_MAX_TORQUE": {
                "value": 500
            }
        }
    }
)

TELE_CONFIG = create_config_template(
    max_torque=10, 
    p_value=1, 
    d_value=0, 
    i_value=0,
    servo_specific=None  # 不配置特定舵机参数
)


# Default setting
BAUDRATE                    = 1000000           # SCServo default baudrate : 1000000
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

SCS_MINIMUM_POSITION_VALUE  = 100         # SCServo will rotate between this value
SCS_MAXIMUM_POSITION_VALUE  = 4000        # and this value (note that the SCServo would not move when the position value is out of movable range. Check e-manual about the range of the SCServo you use.)
SCS_MOVING_STATUS_THRESHOLD = 20          # SCServo moving status threshold
SCS_MOVING_SPEED            = 0           # SCServo moving speed
SCS_MOVING_ACC              = 0           # SCServo moving acc
protocol_end                = 0           # SCServo bit end(STS/SMS=0, SCS=1)

index = 0
scs_goal_position = [SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE]         # Goal position

# 全局标志，用于 Ctrl+C 退出
stop_ping = False

def signal_handler(sig, frame):
    """处理 Ctrl+C 信号"""
    global stop_ping
    stop_ping = True
    logging.getLogger("FeetechTool").info("Received Ctrl+C, stopping ping scan...")

def setup_logger():
    """配置日志：同时输出到终端和文件（文件与脚本同目录）"""
    # 获取脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    log_file = os.path.join(
        script_dir,
        f"feetech_tool_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    )
    
    logger = logging.getLogger("FeetechTool")
    logger.setLevel(logging.INFO)
    
    # 避免重复添加 handler
    if not logger.handlers:
        # 文件 handler
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(logging.INFO)
        
        # 终端 handler
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(logging.INFO)
        
        # 统一格式
        formatter = logging.Formatter('[%(asctime)s] %(levelname)s: %(message)s')
        file_handler.setFormatter(formatter)
        console_handler.setFormatter(formatter)
        
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)
    
    return logger

def send_ping_command(portHandler, packetHandler, servo_id):
    """
    发送 ping 命令并返回响应
    返回: (success, error_code, version)
    """
    logger = logging.getLogger("FeetechTool")

    try:
        # Try to ping the SCServo
        # Get SCServo model number
        scs_model_number, scs_comm_result, scs_error = packetHandler.ping(portHandler, servo_id)
        if scs_comm_result != scs.COMM_SUCCESS:
            return False, None, None
        else:
            return True, scs_error, scs_model_number
            
    except Exception as e:
        # 记录异常日志
        logger = logging.getLogger("FeetechTool")
        logger.debug(f"Error communicating with servo ID {servo_id}: {e}")
        return False, None, None

def ping_scan(portHandler, packetHandler, dev, port, baudrate):
    """执行 ping 扫描 ID 1-255"""
    logger = logging.getLogger("FeetechTool")
    logger.info(f"Starting ping scan on {port} (baudrate={baudrate}, dev={dev})")
    
    global stop_ping
    stop_ping = False
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        for servo_id in range(1, 256):
            if stop_ping:
                break
                
            # 打印当前尝试的 ID（覆盖同一行）
            sys.stdout.write(f"\rping id {servo_id:3d}")
            sys.stdout.flush()
            
            success, error_code, version = send_ping_command(portHandler, packetHandler, servo_id)
            
            if success:
                # 清除当前行并打印结果
                sys.stdout.write("\r" + " " * 20 + "\r")  # 清除行
                logger.info(f"Servo ID {servo_id:3d}: OK | Error Code: {error_code} | Version: {version}")
                sys.stdout.flush()  # 确保立即输出
            else:
                # 无响应：保持 "ping id xx" 显示（但会被下一次覆盖）
                pass
            
            time.sleep(0.01)  # 避免总线过载
        
        if not stop_ping:
            sys.stdout.write("\n")
            sys.stdout.flush()  # 确保换行立即输出
            logger.info("Ping scan completed.")
        else:
            logger.info("Ping scan stopped by user.")
            
    except Exception as e:
        logger.error(f"Ping scan failed: {e}")
        return False
    
    return True

def unlock(portHandler, packetHandler, servo_id):
    """
    解锁舵机(向SCS_LOCK_BIT写入0)
    
    Args:
        portHandler: 端口处理器
        packetHandler: 数据包处理器
        servo_id (int): 舵机ID
        
    Returns:
        bool: 解锁是否成功
    """
    logger = logging.getLogger("FeetechTool")
    
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, servo_id, SCS_LOCK_BIT.head, 0)
    if scs_comm_result != scs.COMM_SUCCESS:
        logger.error("Failed to unlock servo: %s" % packetHandler.getTxRxResult(scs_comm_result))
        return False
    elif scs_error != 0:
        logger.error("Error unlocking servo: %s" % packetHandler.getRxPacketError(scs_error))
        return False
        
    logger.info(f"Servo {servo_id} unlocked successfully.")
    time.sleep(0.1)
    return True

def lock(portHandler, packetHandler, servo_id):
    """
    锁定舵机(向SCS_LOCK_BIT写入1)
    
    Args:
        portHandler: 端口处理器
        packetHandler: 数据包处理器
        servo_id (int): 舵机ID
        
    Returns:
        bool: 锁定是否成功
    """
    logger = logging.getLogger("FeetechTool")
    
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, servo_id, SCS_LOCK_BIT.head, 1)
    if scs_comm_result != scs.COMM_SUCCESS:
        logger.error("Failed to lock servo: %s" % packetHandler.getTxRxResult(scs_comm_result))
        return False
    elif scs_error != 0:
        logger.error("Error locking servo: %s" % packetHandler.getRxPacketError(scs_error))
        return False
        
    logger.info(f"Servo {servo_id} locked successfully.")
    time.sleep(0.1)
    return True

def set_id(portHandler, packetHandler, dev, port, baudrate, cur_id, new_id):
    """设置伺服 ID"""
    logger = logging.getLogger("FeetechTool")
    unlock(portHandler, packetHandler, cur_id)
    logger.info(f"Setting ID: current={cur_id} -> new={new_id} on {port}")
    # 实际应发送 ID 修改指令
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, cur_id, SCS_ID.head, new_id)
    if scs_comm_result != scs.COMM_SUCCESS:
        logger.error("%s" % packetHandler.getTxRxResult(scs_comm_result))
        return False
    elif scs_error != 0:
        logger.error("%s" % packetHandler.getRxPacketError(scs_error))
        return False

    logger.info("ID successfully changed.")
    lock(portHandler, packetHandler, new_id)
    return True

def set_p(portHandler, packetHandler, dev, port, baudrate, servo_id, p_value):
    """设置伺服P参数"""
    logger = logging.getLogger("FeetechTool")
    logger.info(f"Setting P parameter for servo {servo_id}: {p_value}")
    
    # 先解锁舵机
    if not unlock(portHandler, packetHandler, servo_id):
        logger.error(f"Failed to unlock servo {servo_id}")
        return False
    
    # 写入P参数
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(
        portHandler, servo_id, SCS_POSITION_P.head, p_value)
    
    if scs_comm_result != scs.COMM_SUCCESS:
        logger.error("Failed to set P parameter: %s" % packetHandler.getTxRxResult(scs_comm_result))
        lock(portHandler, packetHandler, servo_id)  # 重新锁定舵机
        return False
    elif scs_error != 0:
        logger.error("Error setting P parameter: %s" % packetHandler.getRxPacketError(scs_error))
        lock(portHandler, packetHandler, servo_id)  # 重新锁定舵机
        return False
    
    logger.info(f"P parameter successfully set to {p_value}")
    
    # 重新锁定舵机
    if not lock(portHandler, packetHandler, servo_id):
        logger.warning(f"Failed to lock servo {servo_id}")
    
    return True

def set_d(portHandler, packetHandler, dev, port, baudrate, servo_id, d_value):
    """设置伺服D参数"""
    logger = logging.getLogger("FeetechTool")
    logger.info(f"Setting D parameter for servo {servo_id}: {d_value}")
    
    # 先解锁舵机
    if not unlock(portHandler, packetHandler, servo_id):
        logger.error(f"Failed to unlock servo {servo_id}")
        return False
    
    # 写入D参数
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(
        portHandler, servo_id, SCS_POSITION_D.head, d_value)
    
    if scs_comm_result != scs.COMM_SUCCESS:
        logger.error("Failed to set D parameter: %s" % packetHandler.getTxRxResult(scs_comm_result))
        lock(portHandler, packetHandler, servo_id)  # 重新锁定舵机
        return False
    elif scs_error != 0:
        logger.error("Error setting D parameter: %s" % packetHandler.getRxPacketError(scs_error))
        lock(portHandler, packetHandler, servo_id)  # 重新锁定舵机
        return False
    
    logger.info(f"D parameter successfully set to {d_value}")
    
    # 重新锁定舵机
    if not lock(portHandler, packetHandler, servo_id):
        logger.warning(f"Failed to lock servo {servo_id}")
    
    return True

def set_i(portHandler, packetHandler, dev, port, baudrate, servo_id, i_value):
    """设置伺服I参数"""
    logger = logging.getLogger("FeetechTool")
    logger.info(f"Setting I parameter for servo {servo_id}: {i_value}")
    
    # 先解锁舵机
    if not unlock(portHandler, packetHandler, servo_id):
        logger.error(f"Failed to unlock servo {servo_id}")
        return False
    
    # 写入I参数
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(
        portHandler, servo_id, SCS_POSITION_I.head, i_value)
    
    if scs_comm_result != scs.COMM_SUCCESS:
        logger.error("Failed to set I parameter: %s" % packetHandler.getTxRxResult(scs_comm_result))
        lock(portHandler, packetHandler, servo_id)  # 重新锁定舵机
        return False
    elif scs_error != 0:
        logger.error("Error setting I parameter: %s" % packetHandler.getRxPacketError(scs_error))
        lock(portHandler, packetHandler, servo_id)  # 重新锁定舵机
        return False
    
    logger.info(f"I parameter successfully set to {i_value}")
    
    # 重新锁定舵机
    if not lock(portHandler, packetHandler, servo_id):
        logger.warning(f"Failed to lock servo {servo_id}")
    
    return True


def read_config(portHandler, packetHandler, dev, port, baudrate):
    """读取配置"""
    logger = logging.getLogger("FeetechTool")
    logger.info(f"Reading config from {port} (dev={dev}, baudrate={baudrate})")
    
    # 先打印表头
    header = ["Parameter"] + [f"Servo{i}" for i in range(1, 7)]
    logger.info("Configuration Table:")
    
    # 打印表头（增加参数名列宽到16）
    header_str = "{:<16}".format(header[0])  # 增加到16个字符宽度
    for h in header[1:]:
        header_str += "{:>8}".format(h)
    logger.info(header_str)
    logger.info("-" * len(header_str))
    
    # 逐行读取并输出数据
    for name, address in SCS_ADDRESS_LIST:
        row_str = "{:<16}".format(name[:16])  # 增加到16个字符宽度
        for servo_id in range(1, 7):
            raw_data = None
            if address.size == 1:
                raw_data, scs_comm_result, scs_error = packetHandler.read1ByteTxRx(
                    portHandler, servo_id, address.head)
                if scs_comm_result != scs.COMM_SUCCESS:
                    logger.debug("%s" % packetHandler.getTxRxResult(scs_comm_result))
                    raw_data = "N/A"
                elif scs_error != 0:
                    logger.debug("%s" % packetHandler.getRxPacketError(scs_error))
                    raw_data = "N/A"
            elif address.size == 2:
                raw_data, scs_comm_result, scs_error = packetHandler.read2ByteTxRx(
                    portHandler, servo_id, address.head)
                if scs_comm_result != scs.COMM_SUCCESS:
                    logger.debug("%s" % packetHandler.getTxRxResult(scs_comm_result))
                    raw_data = "N/A"
                elif scs_error != 0:
                    logger.debug("%s" % packetHandler.getRxPacketError(scs_error))
                    raw_data = "N/A"
            else:
                raw_data = "N/A"
                
            row_str += "{:>8}".format(raw_data if raw_data is not None else "N/A")
        logger.info(row_str)



    return True

def set_torque(portHandler, packetHandler, port, servo_id, torque_value):
    """设置伺服最大扭矩"""
    logger = logging.getLogger("FeetechTool")
    logger.info(f"Setting MAX_TORQUE for servo {servo_id}: {torque_value}")
    
    # 先解锁舵机
    if not unlock(portHandler, packetHandler, servo_id):
        logger.error(f"Failed to unlock servo {servo_id}")
        return False
    
    # 写入MAX_TORQUE参数
    scs_comm_result, scs_error = packetHandler.write2ByteTxRx(
        portHandler, servo_id, SCS_MAX_TORQUE.head, torque_value)
    
    if scs_comm_result != scs.COMM_SUCCESS:
        logger.error("Failed to set MAX_TORQUE parameter: %s" % packetHandler.getTxRxResult(scs_comm_result))
        lock(portHandler, packetHandler, servo_id)  # 重新锁定舵机
        return False
    elif scs_error != 0:
        logger.error("Error setting MAX_TORQUE parameter: %s" % packetHandler.getRxPacketError(scs_error))
        lock(portHandler, packetHandler, servo_id)  # 重新锁定舵机
        return False
    
    logger.info(f"MAX_TORQUE parameter successfully set to {torque_value}")
    
    # 重新锁定舵机
    if not lock(portHandler, packetHandler, servo_id):
        logger.warning(f"Failed to lock servo {servo_id}")
    
    # 提示用户断电重启
    input("Please remove DC plug in servo board and waiting at least 3s then replug it if all completed. Press Enter to continue...")
    
    # 用户按下Enter后重新执行一次read_config
    logger.info("Re-reading configuration after power cycle...")
    read_config(portHandler, packetHandler, None, port, BAUDRATE)
    
    return True

def write_config(portHandler, packetHandler, dev, port, baudrate):
    """写入配置"""
    logger = logging.getLogger("FeetechTool")
    logger.info(f"Writing config to {port} (dev={dev}, baudrate={baudrate})")
    
    # 根据dev参数选择配置
    if dev == 'robot':
        config = ROBOT_CONFIG
        logger.info("Using ROBOT_CONFIG for writing")
    elif dev == 'tele':
        config = TELE_CONFIG
        logger.info("Using TELE_CONFIG for writing")
    else:
        logger.error(f"Unknown device type: {dev}")
        return False
    
    # 写入配置到舵机1-6
    success = True
    for servo_id in range(1, 7):
        logger.info(f"Writing config for Servo {servo_id}")
        unlock(portHandler, packetHandler, servo_id)
        # 获取该舵机的所有参数
        for param_name, param_info in config["default_params"].items():
            address = param_info["address"]
            # 检查是否有该舵机的特定配置
            value = None
            if "servo_specific" in config and servo_id in config["servo_specific"]:
                if param_name in config["servo_specific"][servo_id]:
                    value = config["servo_specific"][servo_id][param_name]["value"]
            
            # 如果没有特定配置，使用默认值
            if value is None:
                value = param_info["default_value"]
            
            # 写入参数到舵机
            try:
                if address.size == 1:
                    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(
                        portHandler, servo_id, address.head, value)
                elif address.size == 2:
                    scs_comm_result, scs_error = packetHandler.write2ByteTxRx(
                        portHandler, servo_id, address.head, value)
                else:
                    logger.warning(f"Unsupported address size {address.size} for {param_name}")
                    continue
                
                if scs_comm_result != scs.COMM_SUCCESS:
                    logger.error(f"Failed to write {param_name} to servo {servo_id}: {packetHandler.getTxRxResult(scs_comm_result)}")
                    success = False
                elif scs_error != 0:
                    logger.error(f"Error writing {param_name} to servo {servo_id}: {packetHandler.getRxPacketError(scs_error)}")
                    success = False
                else:
                    logger.info(f"Successfully wrote {param_name}={value}")
                    
            except Exception as e:
                logger.error(f"Exception while writing {param_name} to servo {servo_id}: {e}")
                success = False
        time.sleep(0.1)
        lock(portHandler, packetHandler, servo_id)
    
    if success:
        logger.info("Configuration written successfully.")
        
        # 提示用户断电重启
        input("Please remove DC plug in servo board and waiting at least 3s then replug it if all completed. Press Enter to continue...")
        
        # 用户按下Enter后重新执行一次read_config
        logger.info("Re-reading configuration after power cycle...")
        read_config(portHandler, packetHandler, dev, port, baudrate)
        
    else:
        logger.error("Configuration writing completed with errors.")
        
    return success

def factory_mode(portHandler, packetHandler, dev, port, baudrate):
    """工厂模式：检查ID 1-6是否存在，写入配置，然后读取确认"""
    logger = logging.getLogger("FeetechTool")
    logger.info(f"Running factory mode on {port} (dev={dev}, baudrate={baudrate})")
    
    # 第一步：检查ID 1-6是否存在
    logger.info("Step 1: Checking if servos 1-6 exist...")
    missing_servos = []
    for servo_id in range(1, 7):
        success, error_code, version = send_ping_command(portHandler, packetHandler, servo_id)
        if success:
            logger.info(f"Servo ID {servo_id}: OK | Error Code: {error_code} | Version: {version}")
        else:
            logger.error(f"Servo ID {servo_id}: Not found or not responding")
            missing_servos.append(servo_id)
    
    # 如果有缺失的舵机，报错并退出
    if missing_servos:
        logger.error(f"Missing servos: {missing_servos}. Factory mode aborted.")
        return False
    
    logger.info("All servos 1-6 are present.")
    
    # 第二步：执行写入配置
    logger.info("Step 2: Writing configuration...")
    if not write_config(portHandler, packetHandler, dev, port, baudrate):
        logger.error("Failed to write configuration. Factory mode aborted.")
        return False
        
    logger.info("Factory mode completed successfully.")
    return True

def print_key(portHandler, packetHandler, port, servo_id):
    """以4Hz频率读取并打印SCS_RES_4的值"""
    logger = logging.getLogger("FeetechTool")
    logger.info(f"Printing SCS_RES_4 for servo {servo_id} at 4Hz on {port}")
    
    try:
        while True:
            # 读取SCS_RES_4值
            res_value, scs_comm_result, scs_error = packetHandler.read1ByteTxRx(
                portHandler, servo_id, SCS_RES_4.head)
            
            if scs_comm_result != scs.COMM_SUCCESS:
                logger.error("Communication error: %s" % packetHandler.getTxRxResult(scs_comm_result))
                break
            elif scs_error != 0:
                logger.error("Packet error: %s" % packetHandler.getRxPacketError(scs_error))
                break
            else:
                # 打印值到控制台（不记录到日志文件）
                print(f"Servo {servo_id} SCS_RES_4: {res_value}")
            
            # 休眠0.25秒以达到4Hz频率
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        logger.info("Stopped printing SCS_RES_4 values.")
        return True
    except Exception as e:
        logger.error(f"Error reading SCS_RES_4: {e}")
        return False

# ... existing code ...

def main():
    parser = argparse.ArgumentParser(
        description="Feetech Servo Tool - Control Feetech/SCSCL servos via serial port",
        formatter_class=argparse.RawTextHelpFormatter,
        epilog="""
Examples:
  %(prog)s --port /dev/ttyACM0                    # Ping scan for servos
  %(prog)s --port /dev/ttyACM0 --job set_id --cur_id 1 --new_id 10
  %(prog)s --port /dev/ttyACM0 --job set_p --servo_id 1 --p_value 16
  %(prog)s --port /dev/ttyACM0 --job set_d --servo_id 1 --d_value 0
  %(prog)s --port /dev/ttyACM0 --job set_i --servo_id 1 --i_value 0
  %(prog)s --port /dev/ttyACM0 --job set_torque --servo_id 1 --torque_value 1000
  %(prog)s --port /dev/ttyACM0 --job read_config
  %(prog)s --dev tele --port /dev/ttyUSB0 --baudrate 115200 --job write_config
  %(prog)s --port /dev/ttyACM0 --dev robot --job factory_mode
  %(prog)s --port /dev/ttyACM0 --job print_key --servo_id 9
        """
    )
    
    parser.add_argument('--dev', choices=['robot', 'tele'], default='robot',
                        help="Device type: 'robot' or 'tele' (default: robot)")
    parser.add_argument('--port', required=True,
                        help="Serial port name (e.g., /dev/ttyACM0). Must be specified.")
    parser.add_argument('--baudrate', type=int, default=1000000,
                        help="Baud rate (default: 1000000)")
    parser.add_argument('--job', choices=['ping', 'set_id', 'set_p', 'set_d', 'set_i', 'set_torque', 
                                          'read_config', 'write_config', 'factory_mode', 'print_key'], default='ping',
                        help="Operation to perform (default: ping)")
    
    # set_id 专用参数
    parser.add_argument('--cur_id', type=int, default=1,
                        help="Current servo ID (used only for job=set_id, default: 1)")
    parser.add_argument('--new_id', type=int,
                        help="New servo ID (required when job=set_id)")
    # PID设置专用参数
    parser.add_argument('--servo_id', type=int, default=1,
                        help="Servo ID (used for set_p, set_d, set_i, set_torque, default: 1)")
    parser.add_argument('--p_value', type=int,
                        help="P value (required when job=set_p)")
    parser.add_argument('--d_value', type=int,
                        help="D value (required when job=set_d)")
    parser.add_argument('--i_value', type=int,
                        help="I value (required when job=set_i)")
    # Torque设置专用参数
    parser.add_argument('--torque_value', type=int,
                        help="Torque value (required when job=set_torque)")

    args = parser.parse_args()
    
    # 设置日志
    setup_logger()
    logger = logging.getLogger("FeetechTool")
    
    # 校验：job=set_id 时 new_id 必须提供
    if args.job == 'set_id' and args.new_id is None:
        logger.error("--new_id is required when job=set_id")
        sys.exit(1)
    
    # 校验：job=set_torque 时 torque_value 必须提供
    if args.job == 'set_torque' and args.torque_value is None:
        logger.error("--torque_value is required when job=set_torque")
        sys.exit(1)
    
    # 初始化端口和协议处理器
    portHandler = scs.PortHandler(args.port)
    packetHandler = scs.PacketHandler(protocol_end)
    
    # Open port
    if portHandler.openPort():
        pass
    else:
        logger.error("Failed to open the port")
        logger.info("Terminating program...")
        sys.exit(1)
        
    # Set port baudrate
    if portHandler.setBaudRate(args.baudrate):
        pass
    else:
        logger.error("Failed to change the baudrate")
        portHandler.closePort()
        sys.exit(1)
    
    # 执行对应操作
    try:
        if args.job == 'ping':
            success = ping_scan(portHandler, packetHandler, args.dev, args.port, args.baudrate)
        elif args.job == 'set_id':
            success = set_id(portHandler, packetHandler, args.dev, args.port, args.baudrate, args.cur_id, args.new_id)
        elif args.job == 'set_p':
            if args.p_value is None:
                logger.error("--p_value is required when job=set_p")
                sys.exit(1)
            success = set_p(portHandler, packetHandler, args.dev, args.port, args.baudrate, args.servo_id, args.p_value)
        elif args.job == 'set_d':
            if args.d_value is None:
                logger.error("--d_value is required when job=set_d")
                sys.exit(1)
            success = set_d(portHandler, packetHandler, args.dev, args.port, args.baudrate, args.servo_id, args.d_value)
        elif args.job == 'set_i':
            if args.i_value is None:
                logger.error("--i_value is required when job=set_i")
                sys.exit(1)
            success = set_i(portHandler, packetHandler, args.dev, args.port, args.baudrate, args.servo_id, args.i_value)
        elif args.job == 'set_torque':
            success = set_torque(portHandler, packetHandler, args.port, args.servo_id, args.torque_value)
        elif args.job == 'read_config':
            success = read_config(portHandler, packetHandler, args.dev, args.port, args.baudrate)
        elif args.job == 'write_config':
            success = write_config(portHandler, packetHandler, args.dev, args.port, args.baudrate)
        elif args.job == 'factory_mode':
            success = factory_mode(portHandler, packetHandler, args.dev, args.port, args.baudrate)
        elif args.job == 'print_key':
            success = print_key(portHandler, packetHandler, args.port, args.servo_id)
        else:
            logger.error(f"Unknown job: {args.job}")
            success = False
        
        # 关闭端口
        portHandler.closePort()
        
        if not success:
            sys.exit(1)
            
    except KeyboardInterrupt:
        logger.info("Program interrupted by user.")
        portHandler.closePort()
        sys.exit(0)
    except Exception as e:
        logger.error(f"Runtime error: {e}")
        portHandler.closePort()
        sys.exit(1)

if __name__ == '__main__':
    main()
