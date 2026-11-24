"""
Unit Tests for Windows ↔ WSL2 TCP Protocol & Framing (粘包/半包/校验测试)
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'mujoco_bridge'))
from mujoco_bridge.tcp_protocol import (
    StreamParser,
    pack_command_frame,
    pack_feedback_frame,
    unpack_command_payload,
    unpack_feedback_payload,
    TYPE_CMD_MOTOR,
    TYPE_FEEDBACK_MOTOR
)


class TestTcpFraming(unittest.TestCase):
    def test_single_command_frame(self):
        cmd_frame = pack_command_frame(
            seq=101,
            timestamp=12345.678,
            mode=1,
            positions=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
            velocities=[0.01, 0.02, 0.03, 0.04, 0.05, 0.06]
        )
        parser = StreamParser()
        parser.feed(cmd_frame)
        res = parser.parse_next()
        self.assertIsNotNone(res)
        self.assertEqual(res['type'], TYPE_CMD_MOTOR)
        self.assertEqual(res['seq'], 101)

        mode, pos, vel, eff = unpack_command_payload(res['payload'])
        self.assertEqual(mode, 1)
        self.assertAlmostEqual(pos[0], 0.1)
        self.assertAlmostEqual(pos[5], 0.6)

    def test_sticky_packets_multiple_frames(self):
        """测试粘包：一次性喂入 3 帧完整数据"""
        f1 = pack_command_frame(seq=1, timestamp=1.0, mode=1, positions=[0.1] * 6)
        f2 = pack_command_frame(seq=2, timestamp=2.0, mode=1, positions=[0.2] * 6)
        f3 = pack_command_frame(seq=3, timestamp=3.0, mode=1, positions=[0.3] * 6)

        parser = StreamParser()
        parser.feed(f1 + f2 + f3)

        r1 = parser.parse_next()
        self.assertIsNotNone(r1)
        self.assertEqual(r1['seq'], 1)

        r2 = parser.parse_next()
        self.assertIsNotNone(r2)
        self.assertEqual(r2['seq'], 2)

        r3 = parser.parse_next()
        self.assertIsNotNone(r3)
        self.assertEqual(r3['seq'], 3)

        # 缓冲区应已清空
        self.assertIsNone(parser.parse_next())

    def test_half_packets_fragmentation(self):
        """测试半包/分片：单帧拆为 1 字节逐次喂入"""
        f = pack_feedback_frame(
            seq=42, timestamp=100.0, echo_seq=10, echo_timestamp=99.5,
            positions=[0.5] * 6, velocities=[0.0] * 6
        )
        parser = StreamParser()
        parsed = None

        # 逐字节喂入
        for b in f:
            parser.feed(bytes([b]))
            res = parser.parse_next()
            if res:
                parsed = res

        self.assertIsNotNone(parsed)
        self.assertEqual(parsed['seq'], 42)
        echo_seq, echo_ts, pos, vel, eff, temp, fault = unpack_feedback_payload(parsed['payload'])
        self.assertEqual(echo_seq, 10)
        self.assertAlmostEqual(echo_ts, 99.5)
        self.assertAlmostEqual(pos[0], 0.5)

    def test_corrupted_data_recovery(self):
        """测试脏数据自愈恢复：在正常帧前加入垃圾字节"""
        garbage = b'GARBAGE_NOISE_1234567890'
        good_frame = pack_command_frame(seq=999, timestamp=50.0, mode=1, positions=[0.9] * 6)

        parser = StreamParser()
        parser.feed(garbage + good_frame)

        res = parser.parse_next()
        self.assertIsNotNone(res)
        self.assertEqual(res['seq'], 999)


if __name__ == '__main__':
    unittest.main()
