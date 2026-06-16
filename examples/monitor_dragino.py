#!/usr/bin/env python3
"""Dragino SX1276 packet monitor with CRC error correction."""
import sys, asyncio, time, logging
from collections import defaultdict

sys.path.insert(0, "/home/maxious/pyMC_core/examples")
logging.basicConfig(level=logging.WARNING)

from common import create_radio
from pymc_core.protocol.crc_error_correction import (
    crc16_verify, crc16_correct_single_bit, majority_vote,
)

OUT_PATH = "/tmp/dragino_packets.txt"
packet_history: dict[int, list[bytes]] = defaultdict(list)
good_count = 0
fixed_count = 0
crc_fail_count = 0


def try_recovery(data: bytes) -> bytes | None:
    global fixed_count
    result = crc16_correct_single_bit(bytearray(data))
    if result is not None and crc16_verify(bytes(result)) == 0:
        fixed_count += 1
        return bytes(result)

    key = hash(data[:8])
    packet_history[key].append(data)
    if len(packet_history[key]) >= 3:
        voted = majority_vote(packet_history[key])
        if voted is not None and crc16_verify(voted) == 0:
            packet_history[key].clear()
            fixed_count += 1
            return voted
    return None


async def main():
    global good_count, crc_fail_count

    r = create_radio("dragino-lora-gps", None)
    r.begin()
    lo = r.lora
    lo._spi_write(0x33, 0x27)
    lo._spi_write(0x0B, 0x23)

    with open(OUT_PATH, "a") as f:
        f.write(f"\n=== {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
        f.flush()

    loop = asyncio.get_running_loop()
    r._event_loop = loop
    task = loop.create_task(r._rx_irq_background_task())

    def on_packet(data: bytes):
        global good_count, crc_fail_count
        ts = time.strftime("%H:%M:%S")
        crc_ok = crc16_verify(data) == 0

        if crc_ok:
            good_count += 1
            line = f"[{ts}] OK #{good_count} {len(data)}b: {data[:40].hex()}"
        else:
            crc_fail_count += 1
            fixed = try_recovery(data)
            if fixed:
                line = f"[{ts}] FIX #{crc_fail_count}→#{fixed_count} {len(fixed)}b: {fixed[:40].hex()}"
                data = fixed  # Log corrected version below
            else:
                line = f"[{ts}] BAD #{crc_fail_count} {len(data)}b: {data.hex()}"

        with open(OUT_PATH, "a") as f:
            f.write(line + "\n")
            f.flush()

    r.rx_callback = on_packet

    for i in range(1800):
        irq = lo._spi_read(0x12) & 0xFF
        if irq:
            with open(OUT_PATH, "a") as f:
                f.write(f"  IRQ=0x{irq:02X} at {(i*0.2):.0f}s\n")
                f.flush()
        await asyncio.sleep(0.2)


if __name__ == "__main__":
    asyncio.run(main())
