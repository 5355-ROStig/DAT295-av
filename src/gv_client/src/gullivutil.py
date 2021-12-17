import logging
import struct
from collections import namedtuple
from typing import NamedTuple, List


# Model GulliView packet components
Header = namedtuple('Header', ['type', 'subtype', 'unused1', 'timestamp', 'unused2', 'length'])
Detection = namedtuple('Detection', ['tag_id', 'x', 'y', 'camera_id'])


class Packet(NamedTuple):
    header: Header
    detections: List[Detection]


HEADER_FORMAT = '>IIIQQI'
DETECTION_FORMAT = '>IIII'


def parse_packet(binary_data: bytearray) -> Packet:
    """
    Populate a datastructure from unpacking a GulliView position packet

    :param binary_data: Bytearray of data, should be 256 bytes in size
    :return: Packet tuple containing parsed headers and detections from packet
    """

    if len(binary_data) != 256:
        raise ValueError("GulliView packet must contain 256 bytes of data")

    header = Header._make(struct.unpack_from(HEADER_FORMAT, binary_data))

    logging.debug(f"Received packet with header: {header}")

    if header.type != 1:
        raise ValueError(f"Received unexpected GulliView packet type '{header.type}'")
    if header.subtype != 2:
        raise ValueError(f"Received unexpected GulliView packet subtype '{header.subtype}'")

    detections: List[Detection] = []
    offset = 32
    for _ in range(header.length):
        detection = Detection._make(struct.unpack_from(DETECTION_FORMAT, binary_data, offset=offset))
        detections.append(detection)
        offset += 16

    return Packet(header=header, detections=detections)


if __name__ == '__main__':
    header_data = struct.pack(HEADER_FORMAT, 1, 2, 0, 123456789, 0, 2)
    d1 = struct.pack(DETECTION_FORMAT, 1, 2, 3, 4)
    d2 = struct.pack(DETECTION_FORMAT, 5, 6, 7, 8)

    data = bytearray(header_data + d1 + d2)
    data = data.ljust(256, b'\x00')  # Pad with zero-bytes to length 256

    print(parse_packet(data))
