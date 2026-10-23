"""Read the header fields of a serialized sensor_msgs/Image without the pixels.

The stream monitor only needs stamp, frame_id, size and encoding. Deserializing
full images in Python makes the monitor itself drop frames under load, which
would then be misreported as input problems. CDR layout (after the 4-byte
encapsulation header; alignment is relative to the end of that header):

    int32 stamp.sec, uint32 stamp.nanosec,
    string frame_id (uint32 length incl. NUL, bytes),
    uint32 height, uint32 width,
    string encoding, ...
"""

from dataclasses import dataclass
import struct

_ENCAPSULATION = {b'\x00\x00': '>', b'\x00\x01': '<'}   # CDR_BE, CDR_LE


class CdrError(ValueError):
    pass


@dataclass(frozen=True)
class ImageHeader:
    sec: int
    nanosec: int
    frame_id: str
    height: int
    width: int
    encoding: str


class _Reader:

    def __init__(self, data):
        if len(data) < 4 or bytes(data[0:2]) not in _ENCAPSULATION:
            raise CdrError('not a plain CDR payload')
        self.endian = _ENCAPSULATION[bytes(data[0:2])]
        self.data = memoryview(data)[4:]
        self.pos = 0

    def _align(self, n):
        self.pos += (-self.pos) % n

    def u32(self, signed=False):
        self._align(4)
        if self.pos + 4 > len(self.data):
            raise CdrError('truncated message')
        value = struct.unpack_from(self.endian + ('i' if signed else 'I'), self.data, self.pos)[0]
        self.pos += 4
        return value

    def string(self):
        length = self.u32()
        if length == 0 or self.pos + length > len(self.data):
            raise CdrError('bad string length')
        raw = bytes(self.data[self.pos:self.pos + length])
        self.pos += length
        if raw[-1:] != b'\x00':
            raise CdrError('string not NUL terminated')
        return raw[:-1].decode('utf-8')


def parse_image_header(serialized):
    r = _Reader(serialized)
    sec = r.u32(signed=True)
    nanosec = r.u32()
    frame_id = r.string()
    height = r.u32()
    width = r.u32()
    encoding = r.string()
    return ImageHeader(sec, nanosec, frame_id, height, width, encoding)
