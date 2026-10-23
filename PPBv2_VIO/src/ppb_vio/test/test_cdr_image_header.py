import struct

import pytest

from ppb_vio.lib.cdr_image_header import CdrError, parse_image_header


def cdr_string(s, pos):
    """Encode a CDR string starting at body offset `pos`; returns (bytes, new_pos)."""
    pad = (-pos) % 4
    raw = s.encode() + b'\x00'
    out = b'\x00' * pad + struct.pack('<I', len(raw)) + raw
    return out, pos + len(out)


def make_image(sec, nanosec, frame_id, height, width, encoding):
    body = struct.pack('<iI', sec, nanosec)
    pos = len(body)
    chunk, pos = cdr_string(frame_id, pos)
    body += chunk
    pad = (-pos) % 4
    body += b'\x00' * pad + struct.pack('<II', height, width)
    pos += pad + 8
    chunk, pos = cdr_string(encoding, pos)
    body += chunk + b'\x00' * 64          # rest of the message is ignored
    return b'\x00\x01\x00\x00' + body


@pytest.mark.parametrize('frame_id', ['', 'a', 'cam0_optical_frame', 'abc'])
def test_parses_header_with_any_frame_id_alignment(frame_id):
    h = parse_image_header(make_image(1520530308, 199447626, frame_id, 512, 752, 'mono8'))
    assert (h.sec, h.nanosec, h.frame_id) == (1520530308, 199447626, frame_id)
    assert (h.height, h.width, h.encoding) == (512, 752, 'mono8')


def test_rejects_garbage_and_truncation():
    with pytest.raises(CdrError):
        parse_image_header(b'\x07\x07\x00\x00' + b'\x00' * 40)
    with pytest.raises(CdrError):
        parse_image_header(make_image(1, 2, 'cam0', 4, 4, 'mono8')[:20])


def test_matches_ros_serialization():
    rclpy_serialization = pytest.importorskip('rclpy.serialization')
    sensor_msgs = pytest.importorskip('sensor_msgs.msg')
    msg = sensor_msgs.Image()
    msg.header.stamp.sec, msg.header.stamp.nanosec = 1403715273, 262142976
    msg.header.frame_id = 'cam1_optical_frame'
    msg.height, msg.width, msg.encoding, msg.step = 480, 752, 'mono8', 752
    msg.data = bytes(480 * 752)
    h = parse_image_header(rclpy_serialization.serialize_message(msg))
    assert (h.sec, h.nanosec, h.frame_id) == (1403715273, 262142976, 'cam1_optical_frame')
    assert (h.height, h.width, h.encoding) == (480, 752, 'mono8')
