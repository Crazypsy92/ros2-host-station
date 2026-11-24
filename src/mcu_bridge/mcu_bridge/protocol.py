"""
Embedded Binary Frame Protocol (嵌入式单片机二进制通信协议)
Header: 0xAA 0x55 | MsgID: 1B | Length: 1B | Payload: NB | Checksum: 2B | Tail: 0x0D 0x0A
"""

import struct

FRAME_HEADER = bytes([0xAA, 0x55])
FRAME_TAIL = bytes([0x0D, 0x0A])

MSG_CMD_MOTOR = 0x01      # 上位机下发电机控制 (模式 + 6关节位置)
MSG_FEEDBACK_MOTOR = 0x02 # 下位机反馈状态 (6关节位置 + 6速度 + 6温度 + 故障码)
MSG_CMD_ESTOP = 0x03      # 急停指令
MSG_HEARTBEAT = 0x04      # 心跳包


def calc_checksum(data: bytes) -> int:
    """计算 16位 校验和 (XOR / Sum)"""
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_frame(msg_id: int, payload: bytes) -> bytes:
    """组装协议数据帧"""
    length = len(payload)
    core = bytes([msg_id, length]) + payload
    crc = calc_checksum(core)
    crc_bytes = struct.pack('>H', crc)
    return FRAME_HEADER + core + crc_bytes + FRAME_TAIL


def pack_motor_command(mode: int, positions: list) -> bytes:
    """编码电机控制帧"""
    # 截断或填充至 6 关节
    pos_data = list(positions)[:6] + [0.0] * max(0, 6 - len(positions))
    payload = struct.pack('<B6f', mode, *pos_data)
    return encode_frame(MSG_CMD_MOTOR, payload)


def unpack_motor_command(payload: bytes):
    """解析电机控制帧"""
    if len(payload) < 25:
        return None, []
    mode, *pos = struct.unpack('<B6f', payload[:25])
    return mode, list(pos)


def pack_motor_feedback(positions: list, velocities: list, temps: list, fault: int) -> bytes:
    """编码下位机反馈帧"""
    pos_data = list(positions)[:6] + [0.0] * max(0, 6 - len(positions))
    vel_data = [int(v * 100) for v in (list(velocities)[:6] + [0.0] * max(0, 6 - len(velocities)))]
    temp_data = [int(t) for t in (list(temps)[:6] + [25.0] * max(0, 6 - len(temps)))]
    payload = struct.pack('<6f6h6BH', *pos_data, *vel_data, *temp_data, fault)
    return encode_frame(MSG_FEEDBACK_MOTOR, payload)


def unpack_motor_feedback(payload: bytes):
    """解析下位机反馈帧"""
    if len(payload) < 44:
        return None
    data = struct.unpack('<6f6h6BH', payload[:44])
    positions = list(data[0:6])
    velocities = [v / 100.0 for v in data[6:12]]
    temperatures = [float(t) for t in data[12:18]]
    fault_code = data[18]
    return positions, velocities, temperatures, fault_code


def parse_stream(buffer: bytearray):
    """从字节流中提取完整帧并校验"""
    frames = []
    while len(buffer) >= 6:
        # 寻找帧头 0xAA 0x55
        idx = buffer.find(FRAME_HEADER)
        if idx == -1:
            buffer.clear()
            break
        if idx > 0:
            del buffer[:idx]

        if len(buffer) < 4:
            break

        msg_id = buffer[2]
        length = buffer[3]
        total_len = 2 + 1 + 1 + length + 2 + 2  # header(2) + id(1) + len(1) + payload(N) + crc(2) + tail(2)

        if len(buffer) < total_len:
            break  # 等待更多字节

        # 检查尾部
        tail = buffer[total_len - 2 : total_len]
        if tail != FRAME_TAIL:
            # 帧尾不对，移过当前帧头继续找
            del buffer[:2]
            continue

        core = bytes(buffer[2 : 2 + 2 + length])
        expected_crc = struct.unpack('>H', buffer[total_len - 4 : total_len - 2])[0]
        actual_crc = calc_checksum(core)

        if actual_crc == expected_crc:
            payload = bytes(buffer[4 : 4 + length])
            frames.append((msg_id, payload))
            del buffer[:total_len]
        else:
            del buffer[:2]

    return frames
