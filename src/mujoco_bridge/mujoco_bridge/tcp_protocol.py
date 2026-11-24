"""
Windows ↔ WSL2 TCP Bridge Protocol (跨系统高速确定性帧协议)
Structure:
[Magic: 4B 'MJCO'] [Ver: 1B] [Type: 1B] [Seq: 4B] [Timestamp: 8B] [Length: 4B] [Payload: NB] [CRC16: 2B]
"""

import struct
import time

MAGIC = b'MJCO'
PROTOCOL_VERSION = 1

TYPE_CMD_MOTOR = 0x01
TYPE_FEEDBACK_MOTOR = 0x02
TYPE_HEARTBEAT = 0x03
TYPE_ESTOP = 0x04

HEADER_PREFIX_FORMAT = '>4sBBIdI'  # magic(4), ver(1), type(1), seq(4), timestamp(8), length(4) -> 22 bytes
HEADER_PREFIX_LEN = struct.calcsize(HEADER_PREFIX_FORMAT)  # 22 bytes
CRC_LEN = 2


def get_default_gateway():
    """动态获取 WSL2 默认网关 IP（即 Windows 宿主机虚拟网卡地址）"""
    import subprocess
    try:
        res = subprocess.check_output(
            ["ip", "route", "show", "default"],
            stderr=subprocess.DEVNULL,
            text=True
        )
        parts = res.strip().split()
        if len(parts) >= 3 and parts[1] == 'via':
            return parts[2]
    except Exception:
        pass

    try:
        with open("/etc/resolv.conf", "r") as f:
            for line in f:
                if line.startswith("nameserver"):
                    return line.split()[1]
    except Exception:
        pass

    return "172.17.240.1"


def calc_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def pack_command_frame(seq: int, timestamp: float, mode: int, positions: list, velocities: list = None, efforts: list = None) -> bytes:
    pos = (list(positions)[:6] + [0.0] * 6)[:6]
    vel = (list(velocities)[:6] + [0.0] * 6)[:6] if velocities else [0.0] * 6
    eff = (list(efforts)[:6] + [0.0] * 6)[:6] if efforts else [0.0] * 6

    # Payload: mode(1B) + 6 pos(48B) + 6 vel(48B) + 6 eff(48B) = 145B
    payload = struct.pack('>B6d6d6d', mode, *pos, *vel, *eff)
    length = len(payload)

    header = struct.pack(HEADER_PREFIX_FORMAT, MAGIC, PROTOCOL_VERSION, TYPE_CMD_MOTOR, seq, timestamp, length)
    core = header[4:] + payload  # ver + type + seq + timestamp + length + payload
    crc = calc_crc16(core)
    return header + payload + struct.pack('>H', crc)


def pack_feedback_frame(seq: int, timestamp: float, echo_seq: int, echo_timestamp: float,
                        positions: list, velocities: list, efforts: list = None,
                        temperatures: list = None, fault: int = 0) -> bytes:
    pos = (list(positions)[:6] + [0.0] * 6)[:6]
    vel = (list(velocities)[:6] + [0.0] * 6)[:6]
    eff = (list(efforts)[:6] + [0.0] * 6)[:6] if efforts else [0.0] * 6
    temp = (list(temperatures)[:6] + [25.0] * 6)[:6] if temperatures else [25.0] * 6

    # Payload: echo_seq(4B) + echo_ts(8B) + 6 pos(48B) + 6 vel(48B) + 6 eff(48B) + 6 temp(24B) + fault(2B) = 182B
    payload = struct.pack('>Id6d6d6d6fH', echo_seq, echo_timestamp, *pos, *vel, *eff, *temp, fault)
    length = len(payload)

    header = struct.pack(HEADER_PREFIX_FORMAT, MAGIC, PROTOCOL_VERSION, TYPE_FEEDBACK_MOTOR, seq, timestamp, length)
    core = header[4:] + payload
    crc = calc_crc16(core)
    return header + payload + struct.pack('>H', crc)


def unpack_command_payload(payload: bytes):
    if len(payload) < 145:
        return None
    data = struct.unpack('>B6d6d6d', payload[:145])
    mode = data[0]
    pos = list(data[1:7])
    vel = list(data[7:13])
    eff = list(data[13:19])
    return mode, pos, vel, eff


def unpack_feedback_payload(payload: bytes):
    if len(payload) < 182:
        return None
    data = struct.unpack('>Id6d6d6d6fH', payload[:182])
    echo_seq = data[0]
    echo_ts = data[1]
    pos = list(data[2:8])
    vel = list(data[8:14])
    eff = list(data[14:20])
    temp = list(data[20:26])
    fault = data[26]
    return echo_seq, echo_ts, pos, vel, eff, temp, fault


class StreamParser:
    """流式解析器：处理 TCP 粘包、半包及拆包"""
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)

    def parse_next(self):
        while len(self.buf) >= HEADER_PREFIX_LEN + CRC_LEN:
            idx = self.buf.find(MAGIC)
            if idx == -1:
                self.buf.clear()
                return None
            if idx > 0:
                del self.buf[:idx]

            if len(self.buf) < HEADER_PREFIX_LEN:
                return None

            magic, ver, msg_type, seq, timestamp, length = struct.unpack(
                HEADER_PREFIX_FORMAT, self.buf[:HEADER_PREFIX_LEN]
            )

            total_frame_len = HEADER_PREFIX_LEN + length + CRC_LEN
            if len(self.buf) < total_frame_len:
                return None  # 等待半包数据接收完整

            frame_data = bytes(self.buf[:total_frame_len])
            core = frame_data[4 : total_frame_len - CRC_LEN]
            expected_crc = struct.unpack('>H', frame_data[total_frame_len - CRC_LEN : total_frame_len])[0]
            actual_crc = calc_crc16(core)

            if actual_crc == expected_crc:
                payload = frame_data[HEADER_PREFIX_LEN : HEADER_PREFIX_LEN + length]
                del self.buf[:total_frame_len]
                return {
                    'ver': ver,
                    'type': msg_type,
                    'seq': seq,
                    'timestamp': timestamp,
                    'payload': payload
                }
            else:
                # CRC 错误，丢弃坏帧头
                del self.buf[:4]

        return None
