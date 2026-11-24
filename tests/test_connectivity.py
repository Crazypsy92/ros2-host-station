"""
Test socket connectivity from WSL to Windows host
"""
import socket
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'mujoco_bridge'))
from mujoco_bridge.tcp_protocol import pack_command_frame, StreamParser, get_default_gateway, TYPE_FEEDBACK_MOTOR

host = get_default_gateway()
port = 9088
print(f"Connecting to Windows TCP server at {host}:{port}...")

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(5.0)
try:
    s.connect((host, port))
    print("SUCCESS: TCP connection established!")

    # Send a motor command frame
    cmd = pack_command_frame(seq=1, timestamp=1.0, mode=1, positions=[0.2] * 6)
    s.sendall(cmd)
    print("SUCCESS: Command frame sent!")

    # Receive feedback frame
    parser = StreamParser()
    data = s.recv(4096)
    parser.feed(data)
    frame = parser.parse_next()
    if frame and frame['type'] == TYPE_FEEDBACK_MOTOR:
        print(f"SUCCESS: Feedback frame received from Windows! Seq={frame['seq']}")
        print("ALL_CONNECTIVITY_CHECKS_PASSED")
    else:
        print(f"ERROR: Unexpected frame or no frame received: {frame}")
        sys.exit(1)
except Exception as e:
    print(f"ERROR: {e}")
    sys.exit(2)
finally:
    s.close()
