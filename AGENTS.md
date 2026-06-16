# AGENTS.md — pyMC_core Development Guide

## Project Overview

**pyMC_core** is a Python reimplementation of [MeshCore](https://github.com/meshcore-dev/meshcore) — a lightweight multi-hop mesh networking library using LoRa radios (SX1262/SX1276) over SPI. Target hardware: Raspberry Pi and similar SBCs.

## Architecture (Layered)

```
Application Layer  →  companion/         (CompanionRadio: contacts, channels, messages)
                          ↓
Mesh Runtime       →  node/              (MeshNode → Dispatcher → Handlers)
                          ↓
Protocol Layer     →  protocol/          (Packet, PacketBuilder, Crypto, Identity)
                          ↓
Hardware Layer     →  hardware/          (LoRaRadio ABC → SX1262/SX1276/USB/KISS/TCP/WS)
                          ↓
Transport Layer    →  hardware/transports/ (SPI: spidev or CH341 USB)
                          ↓
GPIO Layer         →  hardware/          (GPIOPinManager: python-periphery or gpiod)
```

## Directory Map

```
pyMC_core/
├── src/pymc_core/
│   ├── __init__.py              # Exports MeshNode, LocalIdentity, Packet, CryptoUtils
│   ├── protocol/                # Wire protocol: packets, crypto, identity, routing
│   │   ├── constants.py         # All protocol constants (payload types, route types, sizes)
│   │   ├── packet.py            # Packet class: 256-byte mesh packet structure
│   │   ├── packet_builder.py    # Factory: text/advert/trace/path/group/discovery packets
│   │   ├── packet_filter.py     # Duplicate packet detection
│   │   ├── packet_utils.py      # Validation, data, header, hashing, timing, path utilities
│   │   ├── crypto.py            # AES-128-ECB, HMAC-SHA256, Ed25519→X25519, ECDH
│   │   ├── identity.py          # Identity (public) + LocalIdentity (keypair: Ed25519+X25519)
│   │   ├── modem_identity.py    # KISS modem cryptographic identity wrapper
│   │   ├── region_map.py        # Region registry with flood/direct deny rules
│   │   ├── transport_keys.py    # Transport key derivation for region-scoped flooding
│   │   └── utils.py             # Advert parsing, appdata decoding
│   ├── node/                    # Mesh node runtime
│   │   ├── node.py              # MeshNode: thin transport owning radio + dispatcher
│   │   ├── dispatcher.py        # Dispatcher: TX/RX state machine, ACK mgmt, handler dispatch
│   │   ├── handlers/            # Per-packet-type handlers (advert, text, ack, path, trace, etc.)
│   │   └── events/              # EventService + MeshEvents pub/sub system
│   ├── hardware/                # Hardware abstraction
│   │   ├── base.py              # LoRaRadio ABC: begin(), send(), wait_for_rx(), sleep()
│   │   ├── sx1262_wrapper.py    # SX1262Radio (~1155 lines): full LoRa driver, LBT/CAD, IRQ RX
│   │   ├── sx1276_wrapper.py    # SX1276Radio (~901 lines): SX1276/77/78/79 driver
│   │   ├── kiss_serial_wrapper.py    # KISS serial protocol over UART
│   │   ├── kiss_modem_wrapper.py     # MeshCore KISS modem protocol (crypto ops)
│   │   ├── usb_radio.py         # USB-CDC LoRa radio (pymc_usb firmware)
│   │   ├── tcp_radio.py         # TCP LoRa radio (pymc_usb WiFi/TCP mode)
│   │   ├── wsradio.py           # WebSocket LoRa radio driver
│   │   ├── gpio_manager.py      # GPIO abstraction (python-periphery or gpiod backend)
│   │   ├── signal_utils.py      # SNR register-to-dB conversion
│   │   ├── protocol_constants.py # Wire protocol constants for USB/TCP radios
│   │   ├── transports/          # SPI transport abstraction
│   │   │   ├── spi_transport.py      # SPITransport ABC
│   │   │   ├── spidev_transport.py   # Linux spidev backend
│   │   │   └── ch341_spi_transport.py # CH341 USB-to-SPI adapter
│   │   ├── ch341/               # CH341 USB device driver + GPIO manager
│   │   └── lora/LoRaRF/         # Vendored LoRaRF-Python v1.3.0 (SX126x/SX127x low-level)
│   └── companion/               # High-level companion radio API
│       ├── companion_radio.py   # CompanionRadio: full feature set wrapper
│       ├── companion_base.py    # CompanionBase (~2056 lines): shared logic
│       ├── companion_bridge.py  # Companion bridge variant
│       ├── contact_store.py     # Contact management
│       ├── channel_store.py     # Channel management
│       ├── message_queue.py     # Offline message queue
│       ├── path_cache.py        # Routing path cache
│       ├── stats_collector.py   # Telemetry statistics
│       └── models.py            # Data models (Contact, Channel, NodePrefs, etc.)
├── examples/                    # Working examples (all support --radio-type)
│   ├── common.py                # Shared: create_radio(), create_mesh_node() for all hardware
│   ├── send_flood_advert.py     # Flood advertisement broadcast
│   ├── send_direct_advert.py    # Direct advertisement to contact
│   ├── send_tracked_advert.py   # Location-tracked advert with repeats
│   ├── send_text_message.py     # Encrypted text message with ACK
│   ├── send_channel_message.py  # Group/channel message
│   ├── discover_nodes.py        # Discovery request/response
│   ├── ping_repeater_trace.py   # Trace packet ping to repeater
│   ├── login_server.py          # Authentication server
│   ├── monitor.py               # Packet sniffer/monitor
│   ├── wireshark_stream.py      # Stream to Wireshark via UDP/PCAP
│   └── calibrate_cad.py         # CAD calibration tool
├── tests/                       # pytest suite (asyncio_mode=auto)
│   ├── test_crypto.py, test_identity.py, test_packet.py  # Protocol tests
│   ├── test_dispatcher.py, test_mesh_node.py, test_handlers.py  # Node tests
│   ├── test_companion_*.py      # Companion layer tests
│   └── hardware/                # Hardware-related tests
├── docs/                        # MkDocs Material documentation
├── scripts/                     # Standalone scripts (test_modem_crypto.py)
├── 99-ch341.rules               # udev rules for CH341 USB-to-SPI adapter
└── pyproject.toml               # Project config, deps, tool settings
```

## Raspberry Pi / SBC Development Workflow

### Hardware: Dragino LoRa/GPS HAT (SX1276)

On Raspberry Pi Zero 2 W with Dragino HAT:
```
CS=BCM25, RESET=BCM17, DIO0=BCM4, DIO1=BCM23
SPI bus 0, CS 0 (chip select managed by driver GPIO, not hardware SPI CS)
Frequency: 915.075 MHz (AU915)
Radio type: dragino-lora-gps
```

**Persistent Identity**: Derived from Pi serial (`00000000c8b0ba13`), so the node
keeps the same cryptographic identity across restarts. Use `_get_pi_serial_seed()`
from `examples/common.py`.

| Field | Value |
|---|---|
| **Node name** | DraginoPi |
| **Public key** | `f672d94d2c3c3671...` |
| **Serial → Seed** | `00000000c8b0ba13` → SHA-256 |
| **TX Power** | 17 dBm (PA_BOOST) |

### Running on Pi via sshfs

The Pi mounts this machine's filesystem, so local edits are instantly visible:

```bash
# On the Pi (one-time setup):
ssh-keyscan -H 192.168.1.201 >> ~/.ssh/known_hosts
mkdir -p ~/pyMC_core
sshfs maxious@192.168.1.201:/home/maxious/pyMC_core ~/pyMC_core -o reconnect,idmap=user

# Install deps (on Pi):
cd ~/pyMC_core
UV_PROJECT_ENVIRONMENT=$HOME/pymc_uv uv sync --extra hardware --extra dev

# Run examples (on Pi):
cd ~/pyMC_core
UV_PROJECT_ENVIRONMENT=$HOME/pymc_uv uv run python examples/common.py --radio-type dragino-lora-gps
```

Do NOT let `uv` create `.venv` inside the sshfs mount — it's slow. Always set `UV_PROJECT_ENVIRONMENT` to a local path.

### Running tests on Pi:

```bash
cd ~/pyMC_core
UV_PROJECT_ENVIRONMENT=$HOME/pymc_uv uv run pytest tests/ -v
```

## MQTT Packet Monitor

An MQTT broker at `192.168.1.20` (user `mqtt` / pass `mqtt`) relays packets from a nearby MeshCore node on the same AU915 region. Useful for comparing pyMC_core behavior against a real MeshCore device on the same frequency.

### Known Nodes

**🫪 maxious - waterloo** (MQTT relay):
| Field | Value |
|---|---|
| **Node ID** | `58DB978DE3C64793DC4D75BE16E230F232FB91301B94276B06CC479FB699673D` |
| **Hardware** | Heltec V3 (ESP32-S3 + SX1262) |
| **Firmware** | MeshCore v1.16.0 / EastMesh v2026.6.3 |
| **Radio** | 915.075 MHz, 125 kHz BW, SF9, CR 4/5 |
| **Repeater** | Enabled |
| **Region** | SYD (AU915) |

**🫪 max mobile** (local USB, `/dev/ttyUSB0`):
| Field | Value |
|---|---|
| **Node ID** | `25c88f4f...` |
| **Hardware** | Elecrow ThinkNode M2 |
| **Firmware** | MeshCore v1.16.0 |
| **Radio** | 915.075 MHz, 125 kHz BW, SF9, CR 4/5 |
| **TX Power** | 22 dBm |
| **Interaction** | `meshcore-cli -s /dev/ttyUSB0 -b 115200` |

### Topic Structure

```
meshcore/{REGION}/{NODE_ID}/status   — Online/offline, radio config, stats (battery, uptime, noise floor, airtime)
meshcore/{REGION}/{NODE_ID}/packet   — Raw mesh packets received (hex or JSON)
```

### Monitoring Commands

```bash
# Watch all topics from the known node:
mosquitto_sub -h 192.168.1.20 -u mqtt -P mqtt -t "meshcore/SYD/+/#" -v

# Watch status only:
mosquitto_sub -h 192.168.1.20 -u mqtt -P mqtt -t "meshcore/SYD/+/status" -v

# Watch packets only:
mosquitto_sub -h 192.168.1.20 -u mqtt -P mqtt -t "meshcore/SYD/+/packet" -v
```

The node is within range of the Dragino Pi — when pyMC_core transmits on 915.075 MHz, the Heltec should see it and the packet will appear on MQTT. This provides an independent validator for TX success.

## Code Conventions

| Tool | Config |
|---|---|
| **Black** | line-length=100 |
| **isort** | profile=black |
| **flake8** | max-line-length=100, ignore E203, W503 |
| **pytest** | asyncio_mode=auto, testpaths=tests |
| **Docstrings** | Google-style (used by mkdocstrings) |
| **Type hints** | Used throughout (Python ≥3.9) |
| **Pre-commit** | trailing-whitespace, end-of-file-fixer, check-yaml, black, isort, flake8 |

### Excluded from linting
- `src/pymc_core/hardware/lora/` — vendored LoRaRF library
- `examples/` — example scripts

## Optional Dependencies

| Extra | Packages | Purpose |
|---|---|---|
| `[hardware]` | python-periphery, spidev, pyserial, pyusb | SPI/GPIO for SX1262/SX1276 radios |
| `[gpiod]` | gpiod | Alternative GPIO backend |
| `[websocket]` | websockets | WebSocket radio driver |
| `[dev]` | pytest, pytest-cov, pytest-asyncio, black, isort, flake8, mypy, pre-commit | Development tools |
| `[all]` | hardware + websocket + dev + docs | Everything |

## CI/CD

- **GitHub Actions**: Tests on Python 3.9–3.12 matrix
- **Docs**: Auto-deploy MkDocs site to GitHub Pages on push to main
- **PyPI**: Manual publish to TestPyPI, auto-publish to PyPI on GitHub release

## Radio Driver Matrix

| Driver | Transport | Hardware | Use Case |
|---|---|---|---|
| `SX1262Radio` | SPI (spidev/CH341) | SX1262-based HATs (Waveshare, uConsole) | Direct SPI LoRa |
| `SX1276Radio` | SPI (spidev) | SX1276-based HATs (Dragino, RFM95) | Direct SPI LoRa |
| `KissSerialWrapper` | Serial UART | KISS TNC devices | Serial-attached radios |
| `KissModemWrapper` | Serial UART | MeshCore KISS modems | Full-modem devices |
| `USBLoRaRadio` | USB-CDC | pymc_usb firmware devices | USB-attached radios |
| `TCPLoRaRadio` | TCP socket | pymc_usb WiFi/TCP mode | Network-attached radios |
| `WsRadio` | WebSocket | WebSocket radio servers | Remote/cloud radios |

## Key Design Patterns

1. **Radio ABC**: All radios implement `LoRaRadio` (begin, send, wait_for_rx, sleep, get_last_rssi, get_last_snr)
2. **Dispatcher state machine**: IDLE → TRANSMIT → WAIT (for ACK) → IDLE
3. **Handler registration**: Handlers register for payload types; Dispatcher routes incoming packets
4. **Event pub/sub**: MeshEvents propagated through EventService to EventSubscribers
5. **PacketBuilder factory**: Static methods create well-formed packets for each protocol operation
6. **Transport keys**: Derived keys enable region-scoped flooding without full crypto per-hop
