#!/usr/bin/env python3
"""从配置文件读取串口参数，并通过 Modbus RTU 控制继电器。"""

import argparse
import configparser
from pathlib import Path
import sys

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

try:
    import serial
except ImportError:
    serial = None


def _default_config():
    if get_package_share_directory is not None:
        try:
            return Path(get_package_share_directory('controller_pkg')) / 'config' / 'fault.ini'
        except Exception:
            pass
    return Path(__file__).parents[1] / 'config' / 'fault.ini'


DEFAULT_CONFIG = _default_config()


def crc16_modbus(data: bytes) -> bytes:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc.to_bytes(2, "little")


def build_write_coil(address: int, channel: int, energized: bool) -> bytes:
    """生成 05 功能码命令；channel 从 1 开始。"""
    if channel < 1:
        raise ValueError("通道号必须从 1 开始")
    coil = channel - 1
    value = 0xFF00 if energized else 0x0000
    payload = bytes([
        address,
        0x05,
        (coil >> 8) & 0xFF,
        coil & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF,
    ])
    return payload + crc16_modbus(payload)


class RelayController:
    """继电器控制器，串口参数来自 fault.ini。"""

    def __init__(self, config_file=DEFAULT_CONFIG):
        if serial is None:
            raise RuntimeError("缺少 pyserial，请先运行: python -m pip install pyserial")

        config = configparser.ConfigParser()
        if not config.read(config_file, encoding="utf-8"):
            raise FileNotFoundError(f"找不到配置文件: {config_file}")

        section = config["serial"]
        self.port = section.get("PortName", "COM3")
        self.baud = section.getint("BaudRate", 38400)
        self.address = section.getint("Address", 1)
        self.timeout = section.getfloat("Timeout", 1.0)

        if not 1 <= self.address <= 255:
            raise ValueError("配置项 Address 必须在 1 到 255 之间")

    def set_channel(self, channel: int, energized: bool) -> bool:
        """
        设置指定通道。

        channel: 通道号，从 1 开始。
        energized: True=吸合，False=断开（故障）。
        """
        request = build_write_coil(self.address, channel, energized)
        state = "吸合" if energized else "断开（故障）"
        print(f"发送（第{channel}路{state}）:", request.hex(" ").upper())

        try:
            with serial.Serial(
                port=self.port,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
            ) as bus:
                bus.reset_input_buffer()
                bus.write(request)
                bus.flush()
                response = bus.read(8)
        except serial.SerialException as exc:
            print(f"串口通信失败: {exc}", file=sys.stderr)
            return False

        print("接收:", response.hex(" ").upper() if response else "<超时，无应答>")
        if len(response) != 8:
            print(f"应答长度错误：期望 8 字节，实际 {len(response)} 字节", file=sys.stderr)
            return False
        if crc16_modbus(response[:-2]) != response[-2:]:
            print("应答 CRC 校验失败", file=sys.stderr)
            return False
        if response != request:
            print("应答与请求不匹配", file=sys.stderr)
            return False

        print(f"成功：第{channel}路已{state}。")
        return True


def main() -> int:
    parser = argparse.ArgumentParser(description="设置指定继电器通道的状态")
    parser.add_argument("channel", type=int, help="通道号，从 1 开始")
    parser.add_argument("state", choices=("on", "off"),
                        help="on=吸合，off=断开（故障）")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG),
                        help="配置文件路径，默认使用程序目录下的 fault.ini")
    args = parser.parse_args()

    try:
        controller = RelayController(args.config)
        return 0 if controller.set_channel(args.channel, args.state == "on") else 1
    except (FileNotFoundError, KeyError, ValueError, RuntimeError) as exc:
        print(f"配置错误: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
