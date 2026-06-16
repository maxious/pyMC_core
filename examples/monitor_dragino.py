#!/usr/bin/env python3
"""Dragino SX1276 packet monitor — background-safe with file logging."""
import sys, asyncio, time, logging
sys.path.insert(0, "/home/maxious/pyMC_core/examples")
logging.basicConfig(level=logging.WARNING)

from common import create_radio

OUT_PATH = "/tmp/dragino_packets.txt"

async def main():
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

    count = 0
    def on_packet(data):
        nonlocal count
        count += 1
        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] #{count} {len(data)}b: {data[:40].hex()}"
        with open(OUT_PATH, "a") as f:
            f.write(line + "\n")
            f.flush()

    r.rx_callback = on_packet

    for i in range(3000):  # ~10 minutes
        irq = lo._spi_read(0x12) & 0xFF
        if irq:
            with open(OUT_PATH, "a") as f:
                f.write(f"  IRQ=0x{irq:02X} at {(i*0.2):.0f}s\n")
                f.flush()
        await asyncio.sleep(0.2)

if __name__ == "__main__":
    asyncio.run(main())
