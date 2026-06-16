"""CRC-16 single-bit error correction using backward-walk algorithm.

Usage:
  1. Compute CRC over data via crc16_xmodem(data)
  2. Append CRC to data as raw bytes (MSB first)
  3. Receiver runs crc16_verify(full_packet) — should be 0
  4. If non-zero, crc16_correct_single_bit(full_packet) finds and flips the error

Two CRC directions are used intentionally:
  - crc16_xmodem() for checksum generation (standard XMODEM CRC-16)
  - crc16_verify() for verification + error correction via backward walk
"""


def crc16_xmodem(data: bytes) -> int:
    """Compute CRC-16/XMODEM (poly=0x1021, init=0, reflected=False)."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) & 0xFFFF) ^ 0x1021 if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc & 0xFFFF


def _crc_forward(crc: int, bit: int) -> int:
    """One CRC step: consume *bit* at the LSB of the register."""
    if crc & 0x8000:
        return ((crc ^ 0x8000) << 1) ^ 0x1021 ^ bit
    return (crc << 1) ^ bit


def _crc_reverse(crc: int) -> int:
    """Undo one _crc_forward step."""
    if crc & 1:
        return (crc >> 1) ^ 0x8810
    return crc >> 1


def crc16_verify(data: bytes) -> int:
    """Run backward-walk-compatible CRC over *data*.

    Returns the residual — 0 means packet passed CRC check.
    """
    crc = 0
    for byte in data:
        for i in range(8):
            crc = _crc_forward(crc, (byte >> (7 - i)) & 1)
    return crc & 0xFFFF


def crc16_correct_single_bit(packet: bytearray) -> bytearray | None:
    """Attempt single-bit CRC error correction.

    Assumes *packet* = data_bytes + 2-byte XMODEM CRC (MSB first).
    Returns corrected bytearray, or None if correction impossible.
    """
    residual = crc16_verify(bytes(packet))
    if residual == 0:
        return packet

    steps = 0
    crc = residual
    while crc != 1:
        crc = _crc_reverse(crc)
        steps += 1
        if steps > len(packet) * 8 + 32:
            return None

    bit_pos = len(packet) * 8 - 1 - steps
    if bit_pos < 0:
        return None

    byte_idx = bit_pos // 8
    bit_idx = 7 - (bit_pos % 8)
    packet[byte_idx] ^= 1 << bit_idx

    if crc16_verify(bytes(packet)) == 0:
        return packet
    return None


def crc16_append(data: bytes) -> bytes:
    """Return *data* + 2-byte XMODEM CRC, ready for transmission."""
    crc = crc16_xmodem(data)
    return data + bytes([(crc >> 8) & 0xFF, crc & 0xFF])


def majority_vote(packets: list[bytes]) -> bytes | None:
    """Bit-wise majority vote across >=3 copies of the same packet.

    Returns the consensus bytes, or None if fewer than 3 copies
    or lengths differ.
    """
    if len(packets) < 3:
        return None
    length = len(packets[0])
    if not all(len(p) == length for p in packets):
        return None

    result = bytearray(length)
    for bit_pos in range(length * 8):
        byte_idx = bit_pos // 8
        bit_mask = 1 << (7 - (bit_pos % 8))
        votes = sum(1 for p in packets if p[byte_idx] & bit_mask)
        if votes > len(packets) // 2:
            result[byte_idx] |= bit_mask
    return bytes(result)
