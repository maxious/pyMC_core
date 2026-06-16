#!/usr/bin/env python3
"""Quick RX test for Dragino HAT — run directly on Pi."""
import sys, asyncio, time, logging
sys.path.insert(0, "/home/maxious/pyMC_core/examples")
logging.basicConfig(level=logging.WARNING)
from common import create_radio

async def main():
    r = create_radio("dragino-lora-gps", None)
    r.begin()
    lo = r.lora
    lo._spi_write(0x33, 0x27)
    lo._spi_write(0x0B, 0x23)

    results = []
    results.append(f"IRQ=0x{lo._spi_read(0x12):02X}")

    loop = asyncio.get_running_loop()
    r._event_loop = loop
    task = loop.create_task(r._rx_irq_background_task())
    r.rx_callback = lambda data: results.append(f"RX:{len(data)}b:{data[:20].hex()}")

    for i in range(150):
        irq = lo._spi_read(0x12) & 0xFF
        if irq:
            results.append(f"IRQ=0x{irq:02X}@{i*0.2:.0f}s")
        await asyncio.sleep(0.2)

    return "\n".join(results)

if __name__ == "__main__":
    result = asyncio.run(main())
    with open("/tmp/rx_local_result.txt", "w") as f:
        f.write(result + "\nDONE\n")
