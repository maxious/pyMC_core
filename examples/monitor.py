#!/usr/bin/env python3
"""Monitor for MeshCore packets - low-level sniffer."""
import argparse
import asyncio
import logging
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
logging.basicConfig(level=logging.WARNING)

from common import create_radio
from pymc_core.node.dispatcher import Dispatcher


class MonitorHandler:
    @staticmethod
    def payload_type() -> int:
        return 0xFF

    async def __call__(self, packet, metadata=None):
        ts = asyncio.get_event_loop().time()
        rssi = metadata.get("rssi", "?") if metadata else "?"
        snr = metadata.get("snr", "?") if metadata else "?"
        raw = (
            packet.hex()
            if isinstance(packet, bytes)
            else packet.write_to().hex()
            if hasattr(packet, "write_to")
            else bytes(packet).hex()
        )
        print(f"[{ts:.1f}] RSSI={rssi} SNR={snr}  {raw[:64]}")


async def main():
    parser = argparse.ArgumentParser(description="MeshCore packet monitor")
    parser.add_argument(
        "--radio-type",
        default="waveshare",
        choices=[
            "waveshare",
            "uconsole",
            "meshadv-mini",
            "dragino-lora-gps",
            "kiss-tnc",
            "kiss-modem",
            "ch341",
            "pymc_usb",
            "pymc_tcp",
        ],
    )
    parser.add_argument("--serial-port", default="/dev/ttyUSB0")
    args = parser.parse_args()

    print(f"Starting monitor on {args.radio_type}...")
    print("Waiting for packets (Ctrl+C to stop)")
    print("-" * 80)

    radio = create_radio(args.radio_type, args.serial_port)
    if hasattr(radio, "begin"):
        radio.begin()
    elif hasattr(radio, "connect"):
        radio.connect()

    dispatcher = Dispatcher(radio)
    dispatcher.register_fallback_handler(MonitorHandler())
    print("Listening...")

    try:
        await dispatcher.run_forever()
    except KeyboardInterrupt:
        print("\nStopped")


if __name__ == "__main__":
    asyncio.run(main())
