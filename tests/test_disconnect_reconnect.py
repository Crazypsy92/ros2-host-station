"""
Automated Disconnect & Reconnect Integration Test
"""
import socket
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'mujoco_bridge'))
from mujoco_bridge.tcp_protocol import pack_command_frame, StreamParser, get_default_gateway, TYPE_FEEDBACK_MOTOR

host = get_default_gateway()
port = 9088

def test_session(session_id):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(4.0)
    try:
        s.connect((host, port))
        cmd = pack_command_frame(seq=session_id * 100, timestamp=time.time(), mode=1, positions=[0.1 * session_id] * 6)
        s.sendall(cmd)

        parser = StreamParser()
        data = s.recv(4096)
        parser.feed(data)
        frame = parser.parse_next()
        if not frame:
            return False, "No frame parsed"
        return True, f"Session {session_id} OK, seq={frame['seq']}"
    except Exception as e:
        return False, str(e)
    finally:
        s.close()

if __name__ == '__main__':
    print("Testing session 1 (Initial connect)...")
    ok1, msg1 = test_session(1)
    print(f"Session 1 result: {ok1} ({msg1})")

    print("Simulating disconnect and pause of 2s...")
    time.sleep(2)

    print("Testing session 2 (Reconnect after disconnect)...")
    ok2, msg2 = test_session(2)
    print(f"Session 2 result: {ok2} ({msg2})")

    print("Simulating rapid reconnect session 3...")
    ok3, msg3 = test_session(3)
    print(f"Session 3 result: {ok3} ({msg3})")

    if ok1 and ok2 and ok3:
        print("RECONNECT_VERIFICATION_PASSED")
        sys.exit(0)
    else:
        print("RECONNECT_VERIFICATION_FAILED")
        sys.exit(1)
