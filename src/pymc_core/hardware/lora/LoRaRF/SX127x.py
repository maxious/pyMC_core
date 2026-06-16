"""
Low-level SX1276/77/78/79 LoRa chip driver.

Provides register-level access to Semtech SX127x family transceivers
(HopeRF RFM92/95/96/98 compatible). Follows the same global-GPIO-manager
pattern as the bundled SX126x driver so the high-level wrapper can share
the same wiring / interrupt setup.
"""

import time
from typing import Optional

try:
    import spidev

    spi = spidev.SpiDev()
except ImportError:
    spi = None

try:
    from periphery import GPIO as _PeripheryGPIO
except ImportError:
    _PeripheryGPIO = None

from ...signal_utils import snr_register_to_db
from .base import BaseLoRa

_gpio_manager = None


def set_gpio_manager(gpio_manager):
    """Set the GPIO manager instance to be used by this module"""
    global _gpio_manager
    _gpio_manager = gpio_manager


def set_spi_transport(spi_transport):
    """Set the SPI transport instance to be used by this module"""
    global spi
    spi = spi_transport


def _get_output(pin):
    if _gpio_manager is None:
        raise RuntimeError("GPIO manager not initialised. Call set_gpio_manager() first.")
    if pin not in _gpio_manager._pins:
        _gpio_manager.setup_output_pin(pin, initial_value=True)
    return _gpio_manager._pins[pin]


def _get_input(pin):
    if _gpio_manager is None:
        raise RuntimeError("GPIO manager not initialised. Call set_gpio_manager() first.")
    if pin not in _gpio_manager._pins:
        _gpio_manager.setup_input_pin(pin)
    return _gpio_manager._pins[pin]


class SX127x(BaseLoRa):
    """Class for SX1276/77/78/79 LoRa chipsets from Semtech (also HopeRF RFM92/95/96/98)."""

    # Set to True for RadioLib-style SPI transaction logging
    DEBUG_SPI = False

    # ── Register addresses (LoRa mode) ──────────────────────────────────────
    REG_FIFO = 0x00
    REG_OP_MODE = 0x01
    REG_FRF_MSB = 0x06
    REG_FRF_MID = 0x07
    REG_FRF_LSB = 0x08
    REG_PA_CONFIG = 0x09
    REG_PA_RAMP = 0x0A
    REG_OCP = 0x0B
    REG_LNA = 0x0C
    REG_FIFO_ADDR_PTR = 0x0D
    REG_FIFO_TX_BASE_ADDR = 0x0E
    REG_FIFO_RX_BASE_ADDR = 0x0F
    REG_FIFO_RX_CURRENT_ADDR = 0x10
    REG_IRQ_FLAGS_MASK = 0x11
    REG_IRQ_FLAGS = 0x12
    REG_RX_NB_BYTES = 0x13
    REG_PKT_SNR_VALUE = 0x19
    REG_PKT_RSSI_VALUE = 0x1A
    REG_RSSI_VALUE = 0x1B
    REG_MODEM_CONFIG_1 = 0x1D
    REG_MODEM_CONFIG_2 = 0x1E
    REG_SYMB_TIMEOUT_LSB = 0x1F
    REG_PREAMBLE_MSB = 0x20
    REG_PREAMBLE_LSB = 0x21
    REG_PAYLOAD_LENGTH = 0x22
    REG_MAX_PAYLOAD_LENGTH = 0x23
    REG_MODEM_CONFIG_3 = 0x26
    REG_FREQ_ERROR_MSB = 0x28
    REG_FREQ_ERROR_MID = 0x29
    REG_FREQ_ERROR_LSB = 0x2A
    REG_DETECTION_OPTIMIZE = 0x31
    REG_INVERTIQ = 0x33
    REG_HIGHBWOPTIMIZE1 = 0x36  # REG_LRTEST36 (ERRATA 2.1)
    REG_DETECTION_THRESHOLD = 0x37
    REG_SYNC_WORD = 0x39
    REG_HIGHBWOPTIMIZE2 = 0x3A  # REG_LRTEST3A (ERRATA 2.1)
    REG_INVERTIQ2 = 0x3B        # also REG_IMAGECAL in FSK mode
    REG_DIO_MAPPING_1 = 0x40
    REG_DIO_MAPPING_2 = 0x41
    REG_VERSION = 0x42
    REG_TCXO = 0x4B
    REG_PA_DAC = 0x4D
    REG_FORMER_TEMP = 0x5B
    REG_IFFREQ1 = 0x2F          # REG_LRTEST2F (ERRATA 2.3)
    REG_IFFREQ2 = 0x30          # REG_LRTEST30 (ERRATA 2.3)

    # Image calibration bits (REG_IMAGECAL at 0x3B in FSK mode)
    IMAGECAL_START = 0x40       # bit 6
    IMAGECAL_RUNNING = 0x20     # bit 5

    # ── Operational modes (written to REG_OP_MODE bits 2-0) ─────────────────
    MODE_SLEEP = 0x00
    MODE_STDBY = 0x01
    MODE_TX = 0x03
    MODE_RX_CONTINUOUS = 0x05
    MODE_RX_SINGLE = 0x06
    MODE_CAD = 0x07

    # LongRangeMode bit (bit 7 of REG_OP_MODE) - must be set for LoRa operation
    LORA_MODEM = 0x80

    # ── RX timeout values ────────────────────────────────────────────────────
    RX_SINGLE = 0x000000
    RX_CONTINUOUS = 0xFFFFFF

    # ── TX power options ────────────────────────────────────────────────────
    TX_POWER_RFO = 0x00
    TX_POWER_PA_BOOST = 0x80

    # ── RX gain / AGC ────────────────────────────────────────────────────────
    RX_GAIN_POWER_SAVING = 0x00
    RX_GAIN_BOOSTED = 0x01

    # ── Header types ────────────────────────────────────────────────────────
    HEADER_EXPLICIT = 0x00
    HEADER_IMPLICIT = 0x01

    # ── CRC ──────────────────────────────────────────────────────────────────
    CRC_ON = 0x01
    CRC_OFF = 0x00

    # ── IQ ───────────────────────────────────────────────────────────────────
    IQ_STANDARD = 0x00
    IQ_INVERTED = 0x01

    # ── DIO0 mappings ────────────────────────────────────────────────────────
    DIO0_RX_DONE = 0x00
    DIO0_TX_DONE = 0x40
    DIO0_CAD_DONE = 0x80

    # ── IRQ flag bits ────────────────────────────────────────────────────────
    IRQ_CAD_DETECTED = 0x01
    IRQ_FHSS_CHANGE_CH = 0x02
    IRQ_CAD_DONE = 0x04
    IRQ_TX_DONE = 0x08
    IRQ_VALID_HEADER = 0x10
    IRQ_CRC_ERR = 0x20
    IRQ_RX_DONE = 0x40
    IRQ_RX_TIMEOUT = 0x80
    IRQ_ALL = 0xFF
    IRQ_NONE = 0x00

    # ── Status codes (returned by status()) ──────────────────────────────────
    STATUS_DEFAULT = 0
    STATUS_TX_WAIT = 1
    STATUS_TX_TIMEOUT = 2
    STATUS_TX_DONE = 3
    STATUS_RX_WAIT = 4
    STATUS_RX_CONTINUOUS_ST = 5
    STATUS_RX_TIMEOUT = 6
    STATUS_RX_DONE = 7
    STATUS_HEADER_ERR = 8
    STATUS_CRC_ERR = 9
    STATUS_CAD_WAIT = 10
    STATUS_CAD_DETECTED = 11
    STATUS_CAD_DONE = 12

    # ── CAD params ───────────────────────────────────────────────────────────
    CAD_ON_1_SYMB = 0x00
    CAD_ON_2_SYMB = 0x01
    CAD_ON_4_SYMB = 0x02
    CAD_ON_8_SYMB = 0x03
    CAD_ON_16_SYMB = 0x04

    CAD_EXIT_STDBY = 0x00
    CAD_EXIT_RX = 0x10

    # ── RSSI offsets ─────────────────────────────────────────────────────────
    RSSI_OFFSET_LF = 164  # low-band (< 525 MHz)
    RSSI_OFFSET_HF = 157  # high-band (>= 525 MHz)

    # ── SPI & GPIO pin storage (set by wrapper before begin()) ──────────────
    _reset = -1
    _irq = -1
    _txen = -1
    _rxen = -1
    _cs_pin = -1  # manual chip-select GPIO (e.g. Dragino HAT uses GPIO6)
    _raw_gpios = {}  # fallback periphery GPIO objects (used when _gpio_manager is None)

    # Internal state
    _spiSpeed = 8000000
    _payloadTxRx = 0
    _transmitTime = 0.0
    _statusWait = STATUS_DEFAULT
    _statusIrq = 0x00
    _frequency = 0           # last set frequency in Hz (for errata/diagnostics)
    _bw_cfg = 7              # last bandwidth config value (0-9, default 125kHz)
    _mid_band_thresh = 525000000  # boundary between LF/HF bands (Hz)

    # ────────────────────────────────────────────────────────────────────────
    # SPI helpers
    # ────────────────────────────────────────────────────────────────────────

    def _init_raw_gpio(self):
        if _PeripheryGPIO is None:
            return
        if self._raw_gpios:
            return
        for pin_name in ("_reset", "_cs_pin", "_txen", "_rxen"):
            pin = getattr(self, pin_name, -1)
            if pin >= 0:
                try:
                    self._raw_gpios[pin] = _PeripheryGPIO(pin, "out")
                except Exception:
                    pass  # best-effort for fallback

    def _gpio_set_low(self, pin: int):
        if _gpio_manager is not None:
            _gpio_manager.set_pin_low(pin)
        elif pin in self._raw_gpios:
            self._raw_gpios[pin].write(False)

    def _gpio_set_high(self, pin: int):
        if _gpio_manager is not None:
            _gpio_manager.set_pin_high(pin)
        elif pin in self._raw_gpios:
            self._raw_gpios[pin].write(True)

    def _cs_acquire(self):
        if self._cs_pin >= 0:
            self._gpio_set_low(self._cs_pin)

    def _cs_release(self):
        if self._cs_pin >= 0:
            self._gpio_set_high(self._cs_pin)

    def _spi_write(self, address: int, data: int):
        global spi
        if self.DEBUG_SPI:
            print(f"[SPI W] 0x{address:02X} ← 0x{data:02X}")
        self._cs_acquire()
        spi.xfer2([address | 0x80, data])
        self._cs_release()

    def _spi_read(self, address: int) -> int:
        global spi
        self._cs_acquire()
        result = spi.xfer2([address & 0x7F, 0x00])[1]
        self._cs_release()
        if self.DEBUG_SPI:
            print(f"[SPI R] 0x{address:02X} → 0x{result:02X}")
        return result

    def _spi_write_burst(self, address: int, data: list):
        global spi
        if self.DEBUG_SPI:
            print(f"[SPI WB] 0x{address:02X} ← {len(data)} bytes: {[hex(b) for b in data[:8]]}")
        self._cs_acquire()
        spi.xfer2([address | 0x80] + list(data))
        self._cs_release()

    def _spi_read_burst(self, address: int, length: int) -> list:
        global spi
        self._cs_acquire()
        result = spi.xfer2([address & 0x7F] + [0x00] * length)[1:]
        self._cs_release()
        if self.DEBUG_SPI:
            print(f"[SPI RB] 0x{address:02X} → {[hex(b) for b in result[:8]]}")
        return result

    def _write_bits(self, address: int, data: int, position: int, width: int):
        current = self._spi_read(address)
        mask = ((1 << width) - 1) << position
        value = (current & ~mask) | ((data << position) & mask)
        if self.DEBUG_SPI:
            print(f"[SPI B] 0x{address:02X} [{position+width-1}:{position}] ← 0x{data:X} (0x{current:02X} → 0x{value:02X})")
        self._spi_write(address, value)

    # ────────────────────────────────────────────────────────────────────────
    # begin / end / reset / sleep / standby
    # ────────────────────────────────────────────────────────────────────────

    def begin(self) -> bool:
        """Initialise the chip: reset, RX chain calibration, verify version.

        RadioLib erratum: after a cold power-up the SX127x crystal oscillator
        needs ~2-5 ms to stabilise.  If NRESET is not pulsed long enough (or
        is tied to board power-enable) the first SPI transactions race the
        crystal and produce silent register-write failures — OpMode, PaConfig,
        ModemConfig1/2 are common victims.

        Workaround: pre-warm the crystal by stepping FSK SLEEP -> LoRa SLEEP
        (5 ms) -> LoRa STDBY BEFORE any other register writes.  Also, the
        SX1276 datasheet §3.1.1 requires going through SLEEP when switching
        between FSK and LoRa modem — direct STDBY->STDBY switch is illegal.
        (RadioLib PR #1760 / SX1278 datasheet §5.4.8 / AN1200.24)
        """
        if not self.reset():
            return False
        # ── Crystal pre-warm + FSK→LoRa transition through SLEEP ──────
        # After reset the chip is in FSK SLEEP (OpMode ≈ 0x01 or 0x09).
        # Step via LoRa SLEEP to warm the crystal:
        self._spi_write(self.REG_OP_MODE, self.LORA_MODEM | self.MODE_SLEEP)  # LoRa SLEEP
        time.sleep(0.005)   # crystal warm-up (5 ms)
        self._spi_write(self.REG_OP_MODE, self.LORA_MODEM | self.MODE_STDBY)  # LoRa STDBY
        time.sleep(0.001)
        # ── RX chain calibration (must run in FSK STDBY) ──────────────
        # Switch back to FSK through SLEEP (datasheet §3.1.1)
        self._spi_write(self.REG_OP_MODE, self.MODE_SLEEP)  # FSK SLEEP
        time.sleep(0.001)
        self._spi_write(self.REG_OP_MODE, self.MODE_STDBY)  # FSK STDBY
        time.sleep(0.001)
        self._rx_chain_calibration()
        # ── Enter LoRa standby for normal operation ───────────────────
        self._spi_write(self.REG_OP_MODE, self.MODE_SLEEP)  # FSK SLEEP
        time.sleep(0.001)
        self._spi_write(self.REG_OP_MODE, self.LORA_MODEM | self.MODE_SLEEP)  # LoRa SLEEP
        time.sleep(0.001)
        self._spi_write(self.REG_OP_MODE, self.LORA_MODEM | self.MODE_STDBY)  # LoRa STDBY
        time.sleep(0.001)
        # ── Verify the chip is alive by reading version ───────────────
        t0 = time.time()
        while time.time() - t0 < 1.0:
            v = self._spi_read(self.REG_VERSION)
            if v in (0x12, 0x22):
                self._is_lora = True
                return True
            time.sleep(0.01)
        return False

    def _rx_chain_calibration(self):
        """Rx chain calibration for LF and HF bands (Semtech reference).

        Must be called soon after reset while still in FSK standby mode
        so that REG_IMAGECAL (0x3B) is accessible.
        """
        # Calibrate LF band at current frequency
        self._spi_write(self.REG_INVERTIQ2, self.IMAGECAL_START)
        t0 = time.time()
        while time.time() - t0 < 0.1:
            if not (self._spi_read(self.REG_INVERTIQ2) & self.IMAGECAL_RUNNING):
                break
            time.sleep(0.001)
        # Set a known HF frequency and calibrate HF band
        self._write_frf(868000000)
        self._spi_write(self.REG_INVERTIQ2, self.IMAGECAL_START)
        t0 = time.time()
        while time.time() - t0 < 0.1:
            if not (self._spi_read(self.REG_INVERTIQ2) & self.IMAGECAL_RUNNING):
                break
            time.sleep(0.001)

    def _write_frf(self, freq_hz: int):
        """Write frequency registers without storing the value."""
        frf = int((freq_hz << 19) / 32000000)
        self._spi_write(self.REG_FRF_MSB, (frf >> 16) & 0xFF)
        self._spi_write(self.REG_FRF_MID, (frf >> 8) & 0xFF)
        self._spi_write(self.REG_FRF_LSB, frf & 0xFF)

    def end(self):
        """Put chip to sleep."""
        self.sleep()

    def reset(self) -> bool:
        if self._reset < 0:
            return False
        if _gpio_manager is not None:
            _get_output(self._reset)
        else:
            self._init_raw_gpio()
        self._gpio_set_low(self._reset)
        time.sleep(0.001)
        self._gpio_set_high(self._reset)
        time.sleep(0.005)
        # Wait for version register to stabilise
        t0 = time.time()
        while time.time() - t0 < 1.0:
            v = self._spi_read(self.REG_VERSION)
            if v in (0x12, 0x22):
                return True
            time.sleep(0.01)
        return False

    def sleep(self):
        """Enter sleep mode (lowest power) — preserves LongRangeMode bit."""
        current = self._spi_read(self.REG_OP_MODE)
        self._spi_write(self.REG_OP_MODE, (current & 0xF8) | self.MODE_SLEEP)

    def wake(self):
        """Wake from sleep by entering standby — preserves LongRangeMode bit."""
        current = self._spi_read(self.REG_OP_MODE)
        self._spi_write(self.REG_OP_MODE, (current & 0xF8) | self.MODE_STDBY)

    def setStandby(self, mode: int = 0x00):
        """Enter standby mode — preserves LongRangeMode bit. 'mode' ignored."""
        current = self._spi_read(self.REG_OP_MODE)
        self._spi_write(self.REG_OP_MODE, (current & 0xF8) | self.MODE_STDBY)

    # ────────────────────────────────────────────────────────────────────────
    # SPI / pin config (called by wrapper before begin())
    # ────────────────────────────────────────────────────────────────────────

    def setSpi(self, bus: int, cs: int, speed: int = 8000000):
        """Configure SPI bus / chip-select."""
        global spi
        spi = spidev.SpiDev()
        spi.open(bus, cs)
        spi.max_speed_hz = speed
        spi.mode = 0

    def setManualCsPin(self, cs_pin: int):
        self._cs_pin = cs_pin
        if cs_pin >= 0:
            if _gpio_manager is not None:
                _gpio_manager.setup_output_pin(cs_pin, initial_value=True)
            else:
                self._init_raw_gpio()

    # ────────────────────────────────────────────────────────────────────────
    # Frequency
    # ────────────────────────────────────────────────────────────────────────

    def setFrequency(self, freq_hz: int):
        """Set centre frequency in Hz."""
        self._frequency = freq_hz
        self._write_frf(freq_hz)

    def setRfFrequency(self, rfFreq: int):
        """Compatibility alias for setFrequency (SX126x naming)."""
        # On SX126x rfFreq = freq * 33554432 / 32000000
        # Reconstruct Hz value
        freq_hz = int(rfFreq * 32000000 / 33554432)
        self.setFrequency(freq_hz)

    # ────────────────────────────────────────────────────────────────────────
    # TX power / PA / OCP
    # ────────────────────────────────────────────────────────────────────────

    def setTxPower(self, txPower: int, paPin: int = TX_POWER_PA_BOOST):
        """Set TX power and PA pin selection.

        Args:
            txPower: Desired power in dBm (2-20 for PA_BOOST, up to 14 for RFO).
            paPin:   TX_POWER_RFO or TX_POWER_PA_BOOST.
        """
        if txPower > 20:
            txPower = 20
        elif txPower > 14 and paPin == self.TX_POWER_RFO:
            txPower = 14

        paConfig = 0x00
        outputPower = 0x00
        if paPin == self.TX_POWER_RFO:
            if txPower == 14:
                paConfig = 0x60
                outputPower = txPower + 1
            else:
                paConfig = 0x40
                outputPower = txPower + 2
        else:
            paConfig = 0xC0
            paDac = 0x04
            if txPower > 17:
                outputPower = 15
                paDac = 0x07
                self._set_current_protection(100)
            else:
                if txPower < 2:
                    txPower = 2
                outputPower = txPower - 2
                self._set_current_protection(140)
            self._spi_write(self.REG_PA_DAC, paDac)

        self._spi_write(self.REG_PA_CONFIG, paConfig | outputPower)

    def _set_current_protection(self, current_mA: int):
        """Set over-current protection (45-240 mA)."""
        ocpTrim = 27
        if current_mA <= 120:
            ocpTrim = int((current_mA - 45) / 5)
        elif current_mA <= 240:
            ocpTrim = int((current_mA + 30) / 10)
        self._spi_write(self.REG_OCP, 0x20 | ocpTrim)

    # ────────────────────────────────────────────────────────────────────────
    # LNA / RX gain
    # ────────────────────────────────────────────────────────────────────────

    def setRxGain(self, rxGain: int):
        """Set LNA gain and enable/disable AGC.

        Args:
            rxGain: 0 = power saving (AGC on), 1 = boosted gain (manual).
        """
        if rxGain == self.RX_GAIN_BOOSTED:
            # Boosted LNA + AGC off
            self._spi_write(self.REG_LNA, 0x23)  # G1 + LNA boost HF
            self._write_bits(self.REG_MODEM_CONFIG_3, 0, 2, 1)  # AGC off
        else:
            # Power saving + AGC on
            self._spi_write(self.REG_LNA, 0x00)  # max gain
            self._write_bits(self.REG_MODEM_CONFIG_3, 1, 2, 1)  # AGC on

    # ────────────────────────────────────────────────────────────────────────
    # Modulation parameters
    # ────────────────────────────────────────────────────────────────────────

    def setLoRaModulation(self, sf: int, bw: int, cr: int, ldro: Optional[bool] = None):
        """Configure LoRa modulation.

        Args:
            sf:  Spreading factor (6-12).
            bw:  Bandwidth in Hz (7800 - 500000).
            cr:  Coding rate denominator (5-8).
            ldro: Low data rate optimisation. None = auto-calculate.
        """
        self._set_spreading_factor(sf)
        self._set_bandwidth(bw)
        self._set_coding_rate(cr)
        if ldro is None:
            ldro = self._auto_ldro(sf)
        self._set_ldro(ldro)

    @staticmethod
    def _auto_ldro(sf: int, bw_cfg: int = 7) -> bool:
        """Auto-calculate LDRO based on SF and BW (Semtech reference)."""
        if bw_cfg == 7 and sf >= 11:   # 125 kHz + SF11/12
            return True
        if bw_cfg == 8 and sf == 12:   # 250 kHz + SF12
            return True
        return False

    def _set_spreading_factor(self, sf: int):
        if sf < 6:
            sf = 6
        elif sf > 12:
            sf = 12
        optimize = 0x03
        threshold = 0x0A
        if sf == 6:
            optimize = 0x05
            threshold = 0x0C
        self._spi_write(self.REG_DETECTION_OPTIMIZE, optimize)
        self._spi_write(self.REG_DETECTION_THRESHOLD, threshold)
        self._write_bits(self.REG_MODEM_CONFIG_2, sf, 4, 4)

    def _set_bandwidth(self, bw_hz: int):
        bw_map = {
            7800: 0, 10400: 1, 15600: 2, 20800: 3,
            31250: 4, 41700: 5, 62500: 6, 125000: 7,
            250000: 8, 500000: 9,
        }
        self._bw_cfg = 9  # default 500 kHz
        for key in sorted(bw_map.keys()):
            if bw_hz <= key:
                self._bw_cfg = bw_map[key]
                break
        self._write_bits(self.REG_MODEM_CONFIG_1, self._bw_cfg, 4, 4)
        # ERRATA 2.1: Sensitivity optimisation for 500 kHz bandwidth
        self._apply_sensitivity_optimization()

    def _apply_sensitivity_optimization(self):
        """ERRATA 2.1: Sensitivity optimisation with a 500 kHz bandwidth.

        Adjusts REG_HIGHBWOPTIMIZE1 (0x36) and REG_HIGHBWOPTIMIZE2 (0x3A)
        based on bandwidth and frequency band.
        """
        if self._bw_cfg == 9:  # 500 kHz
            if self._frequency >= self._mid_band_thresh:
                self._spi_write(self.REG_HIGHBWOPTIMIZE1, 0x02)
                self._spi_write(self.REG_HIGHBWOPTIMIZE2, 0x64)
            else:
                self._spi_write(self.REG_HIGHBWOPTIMIZE1, 0x02)
                self._spi_write(self.REG_HIGHBWOPTIMIZE2, 0x7F)
        else:
            self._spi_write(self.REG_HIGHBWOPTIMIZE1, 0x03)

    def _set_coding_rate(self, cr: int):
        cr_cfg = max(4, min(cr, 8)) - 4
        self._write_bits(self.REG_MODEM_CONFIG_1, cr_cfg, 1, 3)

    def _set_ldro(self, enable: bool):
        self._write_bits(self.REG_MODEM_CONFIG_3, 1 if enable else 0, 3, 1)

    # ────────────────────────────────────────────────────────────────────────
    # Packet parameters
    # ────────────────────────────────────────────────────────────────────────

    def setPacketParamsLoRa(
        self,
        preambleLength: int = 12,
        headerType: int = HEADER_EXPLICIT,
        payloadLength: int = 64,
        crcType: int = CRC_ON,
        invertIq: int = IQ_STANDARD,
    ):
        """Configure LoRa packet parameters."""
        self._set_preamble(preambleLength)
        self._set_header_mode(headerType)
        self._set_payload_length(payloadLength)
        self._set_crc(crcType)
        self._set_invert_iq(invertIq)

    def _set_preamble(self, length: int):
        self._spi_write(self.REG_PREAMBLE_MSB, (length >> 8) & 0xFF)
        self._spi_write(self.REG_PREAMBLE_LSB, length & 0xFF)

    def _set_header_mode(self, mode: int):
        self._write_bits(self.REG_MODEM_CONFIG_1, mode, 0, 1)

    def _set_payload_length(self, length: int):
        self._spi_write(self.REG_PAYLOAD_LENGTH, length)

    def _set_crc(self, crc_on: int):
        self._write_bits(self.REG_MODEM_CONFIG_2, 1 if crc_on else 0, 2, 1)

    def _set_invert_iq(self, invert: int):
        if invert:
            self._write_bits(self.REG_INVERTIQ, 0x01, 0, 1)
            self._write_bits(self.REG_INVERTIQ, 0x00, 6, 1)  # TX off
            self._spi_write(self.REG_INVERTIQ2, 0x19)         # IQ2 ON
        else:
            self._write_bits(self.REG_INVERTIQ, 0x00, 0, 1)
            self._write_bits(self.REG_INVERTIQ, 0x01, 6, 1)  # TX off (default)
            self._spi_write(self.REG_INVERTIQ2, 0x1D)         # IQ2 OFF (default)

    def setSyncWord(self, syncWord: int):
        """Set sync word (1 byte on SX127x)."""
        if syncWord > 0xFF:
            # Collapse 2-byte sync word to 1 byte (common convention)
            sw = ((syncWord >> 8) & 0xF0) | (syncWord & 0x0F)
        else:
            sw = syncWord
        self._spi_write(self.REG_SYNC_WORD, sw)

    # ────────────────────────────────────────────────────────────────────────
    # FIFO buffer operations
    # ────────────────────────────────────────────────────────────────────────

    def setBufferBaseAddress(self, txBaseAddr: int = 0x00, rxBaseAddr: int = 0x00):
        """Set TX and RX base addresses in FIFO."""
        self._spi_write(self.REG_FIFO_TX_BASE_ADDR, txBaseAddr)
        self._spi_write(self.REG_FIFO_RX_BASE_ADDR, rxBaseAddr)

    def writeBuffer(self, offset: int, data: tuple, length: int):
        """Write bytes to FIFO starting at offset."""
        self._spi_write(self.REG_FIFO_ADDR_PTR, offset)
        for i in range(length):
            self._spi_write(self.REG_FIFO, data[i])

    def readBuffer(self, offset: int, length: int) -> tuple:
        """Read bytes from FIFO starting at offset."""
        self._spi_write(self.REG_FIFO_ADDR_PTR, offset)
        return tuple(self._spi_read_burst(self.REG_FIFO, length))

    # ────────────────────────────────────────────────────────────────────────
    # DIO / IRQ configuration
    # ────────────────────────────────────────────────────────────────────────

    def setDioIrqParams(self, irqMask: int, dio1Mask: int, dio2Mask: int, dio3Mask: int):
        """Configure DIO mapping.

        On SX127x this maps to REG_DIO_MAPPING_1 and REG_DIO_MAPPING_2.
        irqMask is used to set IRQ_FLAGS_MASK.
        """
        # IRQ mask (which IRQs are enabled to fire DIO pins)
        self._spi_write(self.REG_IRQ_FLAGS_MASK, (~irqMask) & 0xFF)

        # DIO0 mapping is set dynamically before TX / RX / CAD by wrapper
        # Default: DIO0=RxDone, DIO1=RxTimeout, DIO2=FhssChangeChannel, DIO3=CadDone
        mapping1 = 0x00  # all default
        self._spi_write(self.REG_DIO_MAPPING_1, mapping1)
        self._spi_write(self.REG_DIO_MAPPING_2, 0x00)

    # ────────────────────────────────────────────────────────────────────────
    # IRQ status
    # ────────────────────────────────────────────────────────────────────────

    def getIrqStatus(self) -> int:
        """Read IRQ flags register."""
        return self._spi_read(self.REG_IRQ_FLAGS)

    def clearIrqStatus(self, clearMask: int = 0xFF):
        """Clear IRQ flags (write bits to clear)."""
        self._spi_write(self.REG_IRQ_FLAGS, clearMask & 0xFF)

    # ────────────────────────────────────────────────────────────────────────
    # TX operations
    # ────────────────────────────────────────────────────────────────────────

    def beginPacket(self):
        """Prepare FIFO for TX. Set base addr and reset pointer."""
        tx_base = self._spi_read(self.REG_FIFO_TX_BASE_ADDR)
        self._spi_write(self.REG_FIFO_ADDR_PTR, tx_base)
        self._payloadTxRx = 0
        if self._txen >= 0 and self._rxen >= 0:
            self._gpio_set_high(self._txen)
            self._gpio_set_low(self._rxen)

    def endPacket(self, timeout: int = 0) -> bool:
        """Finalise and start TX.

        Returns True if TX started successfully.
        """
        if (self._spi_read(self.REG_OP_MODE) & 0x07) == self.MODE_TX:
            return False

        # Clear IRQ flags
        self._spi_write(self.REG_IRQ_FLAGS, 0xFF)

        # Set payload length
        self._spi_write(self.REG_PAYLOAD_LENGTH, self._payloadTxRx)

        self._statusWait = self.STATUS_TX_WAIT
        self._statusIrq = 0x00

        # Map DIO0 to TX_DONE
        self._spi_write(self.REG_DIO_MAPPING_1, self.DIO0_TX_DONE)

        # Enter TX mode
        current = self._spi_read(self.REG_OP_MODE)
        self._spi_write(self.REG_OP_MODE, (current & 0xF8) | self.MODE_TX)
        self._transmitTime = time.time()
        return True

    def setTx(self, timeout: int = 0):
        """Direct TX entry (alias used by wrapper)."""
        self.beginPacket()
        self.endPacket(timeout)

    def write(self, data, length: int = 0):
        """Write bytes to TX FIFO.

        Args:
            data: list, tuple, int, or float of data bytes.
            length: number of bytes to write (0 = all).
        """
        if isinstance(data, (list, tuple)):
            if length == 0 or length > len(data):
                length = len(data)
        elif isinstance(data, (int, float)):
            length = 1
            data = (int(data),)
        else:
            raise TypeError("data must be list, tuple, int or float")

        for i in range(length):
            self._spi_write(self.REG_FIFO, int(data[i]))
        self._payloadTxRx += length

    def transmitTime(self) -> float:
        """Return last TX duration in ms."""
        return self._transmitTime * 1000

    def dataRate(self) -> float:
        """Return last TX data rate in bytes/sec."""
        if self._transmitTime <= 0:
            return 0.0
        return self._payloadTxRx / self._transmitTime

    # ────────────────────────────────────────────────────────────────────────
    # RX operations
    # ────────────────────────────────────────────────────────────────────────

    def request(self, timeout: int = RX_CONTINUOUS) -> bool:
        """Enter RX mode.

        Args:
            timeout: RX_SINGLE, RX_CONTINUOUS, or duration in symbols.

        Returns:
            True if RX mode entered.
        """
        rx_mode = self._spi_read(self.REG_OP_MODE) & 0x07
        if rx_mode in (self.MODE_RX_SINGLE, self.MODE_RX_CONTINUOUS):
            return False

        # Clear IRQ flags
        self._spi_write(self.REG_IRQ_FLAGS, 0xFF)

        # Handle TXEN/RXEN pins
        if self._txen >= 0 and self._rxen >= 0:
            self._gpio_set_low(self._txen)
            self._gpio_set_high(self._rxen)

        self._statusWait = self.STATUS_RX_WAIT
        self._statusIrq = 0x00

        rxMode = self.MODE_RX_CONTINUOUS
        if timeout == self.RX_CONTINUOUS:
            self._statusWait = self.STATUS_RX_CONTINUOUS_ST
        elif timeout > 0:
            rxMode = self.MODE_RX_SINGLE
            # Set symbol timeout (modem config 2 bits 1-0 + REG_SYMB_TIMEOUT_LSB)
            bw_hz_reg = (self._spi_read(self.REG_MODEM_CONFIG_1) >> 4) & 0x0F
            bw_table = [7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000]
            bw = bw_table[bw_hz_reg] if bw_hz_reg < len(bw_table) else 125000
            sf_val = (self._spi_read(self.REG_MODEM_CONFIG_2) >> 4) & 0x0F
            symbTimeout = int(timeout * bw / 1000) >> sf_val if sf_val > 0 else 0
            self._write_bits(self.REG_MODEM_CONFIG_2, (symbTimeout >> 8) & 0x03, 0, 2)
            self._spi_write(self.REG_SYMB_TIMEOUT_LSB, symbTimeout & 0xFF)

        # Map DIO0 to RX_DONE
        self._spi_write(self.REG_DIO_MAPPING_1, self.DIO0_RX_DONE)

        # Enter RX mode (preserve LongRangeMode bit)
        current = self._spi_read(self.REG_OP_MODE)
        self._spi_write(self.REG_OP_MODE, (current & 0xF8) | rxMode)
        return True

    def available(self) -> int:
        """Return remaining payload bytes to read."""
        return self._payloadTxRx

    def read(self, length: int = 0):
        """Read bytes from RX FIFO.

        Args:
            length: number of bytes (0 = single byte).

        Returns:
            Single int or tuple of ints.
        """
        single = False
        if length == 0:
            length = 1
            single = True

        if self._payloadTxRx > length:
            self._payloadTxRx -= length
        else:
            self._payloadTxRx = 0

        data = tuple(self._spi_read_burst(self.REG_FIFO, length))
        return data[0] if single else data

    # ────────────────────────────────────────────────────────────────────────
    # RX buffer status
    # ────────────────────────────────────────────────────────────────────────

    def getRxBufferStatus(self) -> tuple:
        """Return (payloadLength, rxStartBufferPointer)."""
        ptr = self._spi_read(self.REG_FIFO_RX_CURRENT_ADDR)
        length = self._spi_read(self.REG_RX_NB_BYTES)
        return (length, ptr)

    # ────────────────────────────────────────────────────────────────────────
    # RSSI / SNR
    # ────────────────────────────────────────────────────────────────────────

    def getRssiInst(self) -> int:
        """Instantaneous RSSI in raw units."""
        return self._spi_read(self.REG_RSSI_VALUE)

    def getSignalMetrics(self) -> tuple:
        """Return (rssi_dbm, snr_db, signal_rssi_dbm)."""
        packet_rssi = self._spi_read(self.REG_PKT_RSSI_VALUE)
        snr_raw = self._spi_read(self.REG_PKT_SNR_VALUE)
        # RSSI offset depends on frequency band
        # Assume high-band (> 525 MHz) for default 868/915 MHz
        rssi_offset = self.RSSI_OFFSET_HF
        rssi_dbm = packet_rssi - rssi_offset
        snr_db = (snr_raw if snr_raw < 128 else snr_raw - 256) / 4.0
        # Signal RSSI accounts for SNR
        if snr_raw < 128:
            signal_rssi_dbm = rssi_dbm
        else:
            signal_rssi_dbm = rssi_dbm + snr_db
        return (rssi_dbm, snr_db, signal_rssi_dbm)

    # ────────────────────────────────────────────────────────────────────────
    # CAD
    # ────────────────────────────────────────────────────────────────────────

    def setCadParams(self, cadSymbolNum: int, cadDetPeak: int, cadDetMin: int, cadExitMode: int, cadTimeout: int):
        """Configure CAD parameters."""
        # Write CAD symbol number to REG_DETECTION_OPTIMIZE bits 7-5
        sym_map = {1: 0x00, 2: 0x20, 4: 0x40, 8: 0x60, 16: 0x80}
        sym_bits = sym_map.get(cadSymbolNum, 0x00)
        if cadExitMode:
            sym_bits |= 0x10
        self._write_bits(self.REG_DETECTION_OPTIMIZE, sym_bits >> 5, 5, 3)
        self._write_bits(self.REG_DETECTION_OPTIMIZE, (sym_bits >> 4) & 1, 4, 1)
        # Detection thresholds and timeout stored for wrapper use

    def setCad(self):
        """Start Channel Activity Detection.

        Maps DIO0 to CAD_DONE and enables CAD_DETECTED on DIO3.
        """
        # DIO0 = CAD_DONE (0x80), DIO3 = CAD_DETECTED (0x01)
        mapping = self.DIO0_CAD_DONE | 0x01  # DIO3=01 (CadDetected)
        self._spi_write(self.REG_DIO_MAPPING_1, mapping)
        self._statusWait = self.STATUS_CAD_WAIT
        self._statusIrq = 0x00
        current = self._spi_read(self.REG_OP_MODE)
        self._spi_write(self.REG_OP_MODE, (current & 0xF8) | self.MODE_CAD)

    def applyRfErrata(self):
        """ERRATA 2.3: Spurious reception fix for LoRa RX.

        Call before entering RX mode to reduce spurious signal detection.
        Adjusts IFFREQ1/IFFREQ2 registers and detection optimise per BW.
        """
        if self._bw_cfg < 9:
            self._write_bits(self.REG_DETECTION_OPTIMIZE, 0, 7, 1)  # clear bit 7
            self._spi_write(self.REG_IFFREQ2, 0x00)
            bw_fixes = {
                0: 0x48, 1: 0x44, 2: 0x44, 3: 0x44,
                4: 0x44, 5: 0x44, 6: 0x40, 7: 0x40, 8: 0x40,
            }
            self._spi_write(self.REG_IFFREQ1, bw_fixes.get(self._bw_cfg, 0x40))
        else:
            self._write_bits(self.REG_DETECTION_OPTIMIZE, 1, 7, 1)  # set bit 7

    # ────────────────────────────────────────────────────────────────────────
    # Misc compatibility stubs (wrappers may call these)
    # ────────────────────────────────────────────────────────────────────────

    def setPacketType(self, packetType: int):
        """Set modem to LoRa (SX126x compat). SX127x sets this via REG_OP_MODE."""
        if packetType == self.LORA_MODEM:
            current_mode = self._spi_read(self.REG_OP_MODE) & 0x07
            self._spi_write(self.REG_OP_MODE, self.LORA_MODEM | current_mode)

    def busyCheck(self, timeout: int = 50) -> bool:
        """SX127x has no BUSY pin; always return False (not busy)."""
        return False

    def getMode(self) -> int:
        """Return current opmode (SX126x compat)."""
        return self._spi_read(self.REG_OP_MODE) & 0x07
