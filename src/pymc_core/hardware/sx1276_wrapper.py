"""
SX1276/77/78/79 (RFM92/95/96/98) LoRa Radio Driver for Raspberry Pi
Implements the LoRaRadio interface via the bundled LoRaRF SX127x driver.
"""

import asyncio
import logging
import math
import random
import time
from typing import Optional, Union

from .base import LoRaRadio
from .gpio_manager import GPIOPinManager
from .lora.LoRaRF.SX127x import SX127x, set_gpio_manager

logger = logging.getLogger("SX1276_wrapper")


class SX1276Radio(LoRaRadio):
    """SX1276/77/78/79 LoRa Radio implementation (HopeRF RFM92/95/96/98)."""

    # Singleton-like active instance tracking
    _active_instance = None

    RADIO_TIMING_DELAY = 0.01

    def __init__(
        self,
        bus_id: int = 0,
        cs_id: int = 0,
        cs_pin: int = -1,
        gpio_chip: int = 0,
        use_gpiod_backend: bool = False,
        reset_pin: int = 22,
        dio0_pin: int = 7,
        dio1_pin: int = -1,
        txen_pin: int = -1,
        rxen_pin: int = -1,
        txled_pin: int = -1,
        rxled_pin: int = -1,
        en_pin: int = -1,
        en_pins: Optional[list[int]] = None,
        frequency: int = 868000000,
        tx_power: int = 17,
        spreading_factor: int = 7,
        bandwidth: int = 125000,
        coding_rate: int = 5,
        preamble_length: int = 12,
        sync_word: int = 0x34,
        radio_timing_delay: float = RADIO_TIMING_DELAY,
    ):
        if SX1276Radio._active_instance is not None:
            logger.warning("Another SX1276Radio instance is active - cleaning it up")
            try:
                SX1276Radio._active_instance.cleanup()
            except Exception as e:
                logger.error(f"Error cleaning up previous instance: {e}")
            SX1276Radio._active_instance = None

        self.en_pins = self._normalize_en_pins(en_pin=en_pin, en_pins=en_pins)

        self.bus_id = bus_id
        self.cs_id = cs_id
        self.cs_pin = cs_pin
        self.gpio_chip = gpio_chip
        self.use_gpiod_backend = use_gpiod_backend
        self.reset_pin = reset_pin
        self.dio0_pin = dio0_pin
        self.dio1_pin = dio1_pin
        self.txen_pin = txen_pin
        self.rxen_pin = rxen_pin
        self.txled_pin = txled_pin
        self.rxled_pin = rxled_pin
        self.en_pin = self.en_pins[0] if self.en_pins else -1
        self._RADIO_TIMING_DELAY = radio_timing_delay

        self.frequency = frequency
        self.tx_power = tx_power
        self.spreading_factor = spreading_factor
        self.bandwidth = bandwidth
        self.coding_rate = coding_rate
        self.preamble_length = preamble_length
        self.sync_word = sync_word

        self.lora: Optional[SX127x] = None
        self.last_rssi: int = -99
        self.last_snr: float = 0.0
        self.last_signal_rssi: int = -99
        self._initialized = False
        self._rx_lock = asyncio.Lock()
        self._tx_lock = asyncio.Lock()

        from .lora.LoRaRF.SX127x import _gpio_manager as existing_gpio_manager

        if existing_gpio_manager is not None:
            self._gpio_manager = existing_gpio_manager
            logger.info("Using externally configured GPIO manager")
        else:
            backend = "gpiod" if self.use_gpiod_backend else "auto"
            self._gpio_manager = GPIOPinManager(
                backend=backend, gpio_chip=f"/dev/gpiochip{self.gpio_chip}"
            )
            set_gpio_manager(self._gpio_manager)

        self._interrupt_setup = False
        self._txen_pin_setup = False
        self._txled_pin_setup = False
        self._rxled_pin_setup = False
        self._en_pins_setup = False

        self._tx_done_event = asyncio.Event()
        self._rx_done_event = asyncio.Event()
        self._cad_event = asyncio.Event()

        self._last_irq_status = 0
        self._event_loop = None
        self._last_cad_detected = False
        self._last_cad_irq_status = 0

        self._custom_cad_peak = None
        self._custom_cad_min = None

        self._noise_floor = -99.0
        self._num_floor_samples = 0
        self._floor_sample_sum = 0.0
        self._last_packet_activity = 0.0
        self._is_receiving_packet = False
        self.NUM_NOISE_FLOOR_SAMPLES = 20
        self.SAMPLING_THRESHOLD = 10

        self.crc_error_count = 0

        logger.info(
            f"SX1276Radio configured: freq={frequency/1e6:.1f}MHz, "
            f"power={tx_power}dBm, sf={spreading_factor}, "
            f"bw={bandwidth/1000:.1f}kHz, pre={preamble_length}"
        )
        SX1276Radio._active_instance = self
        self.rx_callback = None

    # ── Helpers ──────────────────────────────────────────────────────────────

    @staticmethod
    def _normalize_en_pins(en_pin=-1, en_pins=None):
        normalized = []
        if en_pins:
            normalized.extend(en_pins)
        elif en_pin != -1:
            normalized.append(en_pin)
        deduped = []
        for pin in normalized:
            if pin == -1 or pin in deduped:
                continue
            deduped.append(pin)
        return deduped

    def _irq_trampoline(self):
        try:
            if self._event_loop is not None:
                self._event_loop.call_soon_threadsafe(self._handle_interrupt)
            else:
                logger.warning("IRQ received before event loop initialized")
        except Exception as e:
            logger.error(f"IRQ trampoline error: {e}", exc_info=True)

    def _safe_radio_operation(self, name, op_func, success_msg=None):
        if not self._initialized or self.lora is None:
            return False
        try:
            op_func()
            if success_msg:
                logger.debug(success_msg)
            return True
        except Exception as e:
            logger.error(f"Failed to {name}: {e}")
            return False

    # ── Interrupt handling ───────────────────────────────────────────────────

    def _handle_interrupt(self):
        try:
            if not self._initialized or not self.lora:
                return

            irqStat = self.lora.getIrqStatus() & 0xFF
            if irqStat == 0:
                return

            self.lora.clearIrqStatus(0xFF)
            self._last_irq_status = irqStat

            if irqStat & self.lora.IRQ_TX_DONE:
                logger.debug("[TX] TX_DONE interrupt")
                self._tx_done_event.set()

            if irqStat & (self.lora.IRQ_CAD_DETECTED | self.lora.IRQ_CAD_DONE):
                detected = bool(irqStat & self.lora.IRQ_CAD_DETECTED)
                self._last_cad_detected = detected
                self._last_cad_irq_status = irqStat
                self._cad_event.set()

            terminal_rx = self.lora.IRQ_RX_DONE | self.lora.IRQ_CRC_ERR | self.lora.IRQ_RX_TIMEOUT
            if irqStat & terminal_rx:
                if irqStat & self.lora.IRQ_RX_DONE:
                    logger.debug("[RX] RX_DONE interrupt")
                if irqStat & self.lora.IRQ_CRC_ERR:
                    logger.debug("[RX] CRC_ERR interrupt")
                if irqStat & self.lora.IRQ_RX_TIMEOUT:
                    logger.debug("[RX] RX_TIMEOUT interrupt")
                if not self._tx_lock.locked():
                    self._rx_done_event.set()

        except Exception as e:
            logger.error(f"IRQ handler error: {e}")
            self._tx_done_event.set()
            self._rx_done_event.set()

    def set_rx_callback(self, callback):
        self.rx_callback = callback
        if self._interrupt_setup and self._initialized:
            if (
                not hasattr(self, "_rx_irq_task")
                or self._rx_irq_task is None
                or self._rx_irq_task.done()
            ):
                try:
                    loop = asyncio.get_running_loop()
                    self._event_loop = loop
                    self._rx_irq_task = loop.create_task(self._rx_irq_background_task())
                except RuntimeError:
                    pass
                except Exception as e:
                    logger.warning(f"Failed to start delayed RX task: {e}")

    async def _rx_irq_background_task(self):
        logger.debug("[RX] Starting RX IRQ background task")
        rx_check_count = 0

        while self._initialized:
            try:
                if self._interrupt_setup:
                    try:
                        await asyncio.wait_for(
                            self._rx_done_event.wait(), timeout=self.RADIO_TIMING_DELAY
                        )
                        self._rx_done_event.clear()

                        self._is_receiving_packet = True
                        self._last_packet_activity = time.time()

                        try:
                            irqStat = self._last_irq_status

                            if irqStat & self.lora.IRQ_CRC_ERR:
                                self.crc_error_count += 1
                                try:
                                    rssi_dbm, snr_db, signal_rssi_dbm = self.lora.getSignalMetrics()
                                    length, ptr = self.lora.getRxBufferStatus()
                                    logger.warning(
                                        "[RX] CRC error #%d - RSSI=%ddBm, SNR=%.1fdB, Length=%d",
                                        self.crc_error_count, int(rssi_dbm), snr_db, length,
                                    )
                                except Exception:
                                    logger.warning("[RX] CRC error #%d", self.crc_error_count)

                            elif irqStat & self.lora.IRQ_RX_DONE:
                                length, ptr = self.lora.getRxBufferStatus()
                                rssi_dbm, snr_db, signal_rssi_dbm = self.lora.getSignalMetrics()
                                self.last_rssi = int(rssi_dbm)
                                self.last_snr = snr_db
                                self.last_signal_rssi = int(signal_rssi_dbm)

                                logger.debug(
                                    f"[RX] Packet: len={length}, RSSI={self.last_rssi}dBm, "
                                    f"SNR={self.last_snr}dB"
                                )

                                self._gpio_manager.blink_led(self.rxled_pin)

                                if length > 0:
                                    buffer = self.lora.readBuffer(ptr, length)
                                    packet_data = bytes(buffer)
                                    logger.debug(
                                        f"[RX] Data: {packet_data.hex()[:32]}... "
                                        f"({len(packet_data)} bytes)"
                                    )
                                    if self.rx_callback:
                                        try:
                                            self.rx_callback(packet_data)
                                        except Exception as cb_exc:
                                            logger.error(f"RX callback error: {cb_exc}")
                                    else:
                                        logger.warning("[RX] No RX callback registered!")
                                else:
                                    logger.warning("[RX] Empty packet received")

                            elif irqStat & self.lora.IRQ_RX_TIMEOUT:
                                logger.warning("[RX] RX timeout")

                            # Restore RX continuous mode
                            try:
                                self.lora.applyRfErrata()
                                self.lora.request(self.lora.RX_CONTINUOUS)
                                await asyncio.sleep(self.RADIO_TIMING_DELAY)
                            except Exception as e:
                                logger.error(f"Failed to restore RX mode: {e}")

                        except Exception as e:
                            logger.error(f"[IRQ RX] Error processing packet: {e}")
                        finally:
                            self._is_receiving_packet = False

                    except asyncio.TimeoutError:
                        rx_check_count += 1
                        # SPI polling fallback: DIO0 edge detection can fail if
                        # the pin is stuck HIGH from a spurious init assertion.
                        # Poll IRQ flags directly via SPI as a safety net.
                        try:
                            if self.lora and self._initialized:
                                irq = self.lora.getIrqStatus()
                                if irq:
                                    logger.debug(f"[RX] RX via SPI poll (IRQ=0x{irq:02X})")
                                    self._last_irq_status = irq
                                    self._rx_done_event.set()
                        except Exception:
                            pass
                        self._sample_noise_floor()
                        if rx_check_count % 500 == 0:
                            logger.debug(
                                f"[RX Task] alive check #{rx_check_count}, "
                                f"noise_floor={self._noise_floor:.1f}dBm"
                            )
                else:
                    await asyncio.sleep(0.1)

            except Exception as e:
                logger.error(f"[RX Task] Unexpected error: {e}")
                await asyncio.sleep(1.0)

        logger.warning("[RX] RX IRQ background task exiting")

    # ── Initialisation ───────────────────────────────────────────────────────

    def begin(self) -> bool:
        if self._initialized:
            logger.debug("SX1276 radio already initialised")
            return True

        try:
            logger.debug("Initialising SX1276 radio...")
            self.lora = SX127x()

            # Setup DIO0 interrupt pin
            self.irq_pin = self._gpio_manager.setup_interrupt_pin(
                self.dio0_pin, pull_up=False, callback=self._irq_trampoline
            )
            if self.irq_pin is not None:
                self._interrupt_setup = True
            else:
                logger.error(f"Failed to setup DIO0 pin {self.dio0_pin}")
                raise RuntimeError(f"Could not setup DIO0 pin {self.dio0_pin}")

            # SPI and GPIO pins
            self.lora.setSpi(self.bus_id, self.cs_id)
            if self.cs_pin != -1:
                self.lora.setManualCsPin(self.cs_pin)

            self.lora._reset = self.reset_pin
            self.lora._irq = self.dio0_pin
            self.lora._txen = -1  # Managed by wrapper if needed
            self.lora._rxen = -1

            # Setup TXEN/RXEN pins if configured
            if self.txen_pin != -1 and not self._txen_pin_setup:
                if self._gpio_manager.setup_output_pin(self.txen_pin, initial_value=False):
                    self._txen_pin_setup = True
                    self.lora._txen = self.txen_pin
            if self.rxen_pin != -1:
                self._gpio_manager.setup_output_pin(self.rxen_pin, initial_value=False)
                self.lora._rxen = self.rxen_pin

            # LEDs
            if self.txled_pin != -1 and not self._txled_pin_setup:
                if self._gpio_manager.setup_output_pin(self.txled_pin, initial_value=False):
                    self._txled_pin_setup = True
            if self.rxled_pin != -1 and not self._rxled_pin_setup:
                if self._gpio_manager.setup_output_pin(self.rxled_pin, initial_value=False):
                    self._rxled_pin_setup = True

            # Enable pins
            if self.en_pins and not self._en_pins_setup:
                all_ok = True
                for en in self.en_pins:
                    if self._gpio_manager.setup_output_pin(en, initial_value=True):
                        logger.debug(f"EN pin {en} configured HIGH")
                    else:
                        all_ok = False
                self._en_pins_setup = all_ok

            # Reset and initialise chip
            if not self.lora.begin():
                raise RuntimeError("SX127x begin() failed")

            # Set frequency
            self.lora.setFrequency(self.frequency)

            # Configure modulation parameters
            symbol_duration_ms = (2**self.spreading_factor) / (self.bandwidth / 1000)
            ldro = symbol_duration_ms > 16.0
            logger.info(f"LDRO {'enabled' if ldro else 'disabled'} ({symbol_duration_ms:.3f}ms)")
            self.lora.setLoRaModulation(
                self.spreading_factor, self.bandwidth, self.coding_rate, ldro
            )

            # Configure packet parameters
            self.lora.setPacketParamsLoRa(
                self.preamble_length,
                self.lora.HEADER_EXPLICIT,
                64,
                self.lora.CRC_ON,
                self.lora.IQ_STANDARD,
            )

            # Set sync word
            self.lora.setSyncWord(self.sync_word)

            # Set TX power
            logger.info(f"Setting TX power to {self.tx_power} dBm")
            self.lora.setTxPower(self.tx_power, self.lora.TX_POWER_PA_BOOST)

            # Set RX gain (boosted)
            self.lora.setRxGain(self.lora.RX_GAIN_BOOSTED)

            # Configure IRQ mask and DIO mapping for RX
            self.lora.setDioIrqParams(
                self.lora.IRQ_RX_DONE | self.lora.IRQ_CRC_ERR | self.lora.IRQ_RX_TIMEOUT
                | self.lora.IRQ_TX_DONE | self.lora.IRQ_CAD_DONE | self.lora.IRQ_CAD_DETECTED,
                0, 0, 0
            )

            # Snapshot IRQ state BEFORE clearing — preserves any packet already
            # received and waiting in the FIFO (DIO0 solidly lit = packet ready).
            _pre_clear_irq = self.lora.getIrqStatus()

            # Clear any pending IRQs
            self.lora.clearIrqStatus(0xFF)

            # Apply ERRATA 2.3 and start RX continuous
            self.lora.applyRfErrata()
            self.lora.request(self.lora.RX_CONTINUOUS)
            time.sleep(self._RADIO_TIMING_DELAY)

            # Ensure DIO0 is LOW — spurious assertion during init can leave
            # it HIGH permanently, breaking edge-triggered interrupt detection.
            # An extra IRQ clear + RX re-entry flushes any stuck state.
            self.lora.clearIrqStatus(0xFF)
            self.lora.request(self.lora.RX_CONTINUOUS)
            time.sleep(self._RADIO_TIMING_DELAY)

            self._initialized = True
            logger.info("SX1276 radio initialised successfully")

            # Start interrupt polling
            if self._gpio_manager:
                try:
                    if (self.dio0_pin is not None
                            and hasattr(self._gpio_manager, "_pins")
                            and self.dio0_pin in self._gpio_manager._pins):
                        pin_obj = self._gpio_manager._pins[self.dio0_pin]
                        if hasattr(pin_obj, "start_polling"):
                            pin_obj.start_polling()
                            logger.info("Started IRQ polling")
                except Exception as e:
                    logger.warning(f"Failed to start IRQ polling: {e}")

            # Start RX background task
            try:
                if self._interrupt_setup:
                    if (not hasattr(self, "_rx_irq_task")
                            or self._rx_irq_task is None
                            or self._rx_irq_task.done()):
                        try:
                            loop = asyncio.get_running_loop()
                            self._event_loop = loop
                        except RuntimeError:
                            return True
                        self._rx_irq_task = loop.create_task(self._rx_irq_background_task())
                        logger.debug("[RX] Background task started")

                        # If a packet was already waiting in the FIFO before we
                        # cleared IRQ flags, feed it to the background task now.
                        if _pre_clear_irq & self.lora.IRQ_RX_DONE:
                            logger.info(
                                "[RX] Packet already in FIFO at init (IRQ=0x%02X)",
                                _pre_clear_irq,
                            )
                            self._last_irq_status = _pre_clear_irq
                            self._rx_done_event.set()
            except Exception as e:
                logger.warning(f"Failed to start RX task: {e}")

            return True

        except Exception as e:
            logger.error(f"Failed to initialise SX1276 radio: {e}")
            self._initialized = False
            raise RuntimeError(f"Failed to initialise SX1276 radio: {e}") from e

    # ── TX ───────────────────────────────────────────────────────────────────

    def _calculate_tx_timeout(self, packet_length: int) -> tuple:
        sf = self.spreading_factor
        bw_hz = int(self.bandwidth)
        cr = self.coding_rate
        preamble = self.preamble_length
        low_dr_opt = 1 if (sf >= 11 and bw_hz <= 125000) else 0
        symbol_time = (1 << sf) / float(bw_hz)
        preamble_time = (preamble + 4.25) * symbol_time
        tmp = 8 * packet_length - 4 * sf + 28 + 16 * 1
        denom = 4 * (sf - 2 * low_dr_opt)
        if tmp > 0:
            payload_symbols = 8 + max(math.ceil(tmp / denom) * (cr + 4), 0)
        else:
            payload_symbols = 8
        payload_time = payload_symbols * symbol_time
        air_time_ms = (preamble_time + payload_time) * 1000.0
        timeout_ms = math.ceil(air_time_ms) + 1000
        driver_timeout = timeout_ms * 64
        return timeout_ms, driver_timeout

    async def _prepare_radio_for_tx(self) -> tuple:
        self._tx_done_event.clear()
        self.lora.setStandby()
        await asyncio.sleep(self.RADIO_TIMING_DELAY)

        # Listen Before Talk (LBT) using CAD
        lbt_backoff_delays = []
        lbt_attempts = 0
        max_lbt_attempts = 5

        while lbt_attempts < max_lbt_attempts:
            try:
                channel_busy = await self.perform_cad(timeout=0.5)
                if not channel_busy:
                    logger.debug(f"Channel clear after {lbt_attempts + 1} attempts")
                    break
                lbt_attempts += 1
                if lbt_attempts < max_lbt_attempts:
                    base_delay = random.randint(50, 200)
                    backoff_ms = base_delay * (2 ** (lbt_attempts - 1))
                    backoff_ms = min(backoff_ms, 5000)
                    lbt_backoff_delays.append(float(backoff_ms))
                    await asyncio.sleep(backoff_ms / 1000.0)
                else:
                    logger.warning("Channel still busy, transmitting anyway")
            except Exception as e:
                logger.warning(f"CAD failed: {e}, proceeding")
                break

        return True, lbt_backoff_delays

    def _setup_tx_interrupts(self):
        # Map DIO0 to TX_DONE via setDioIrqParams (wrapper handles the mapping)
        self.lora.clearIrqStatus(0xFF)

    async def _execute_transmission(self, driver_timeout: int) -> bool:
        self.lora.setStandby()
        await asyncio.sleep(self.RADIO_TIMING_DELAY)
        self.lora.setTx(driver_timeout)
        return True

    async def _wait_for_transmission_complete(self, timeout_seconds: float) -> bool:
        logger.debug(f"[TX] Waiting for TX completion (timeout: {timeout_seconds}s)")
        start_time = time.time()
        poll_interval = 0.05
        next_poll = start_time

        while True:
            elapsed = time.time() - start_time
            remaining = timeout_seconds - elapsed
            if remaining <= 0:
                logger.error("[TX] TX completion timeout")
                return False

            wait_for = min(remaining, max(0.0, next_poll - time.time()))
            try:
                await asyncio.wait_for(self._tx_done_event.wait(), timeout=wait_for)
                logger.debug("[TX] TX_DONE interrupt received")
                return True
            except asyncio.TimeoutError:
                pass

            now = time.time()
            if now >= next_poll:
                next_poll = now + poll_interval
                try:
                    irq = self.lora.getIrqStatus()
                    if irq & self.lora.IRQ_TX_DONE:
                        logger.debug("[TX] TX_DONE via poll")
                        return True
                    if irq & self.lora.IRQ_RX_TIMEOUT:
                        logger.error("[TX] TX_TIMEOUT via poll")
                        self.lora.clearIrqStatus(irq)
                        return False
                except Exception:
                    pass

    def _finalize_transmission(self):
        irq = self.lora.getIrqStatus()
        self.lora.clearIrqStatus(irq)
        logger.debug(f"TX complete, final IRQ: 0x{irq:02X}")

    async def _restore_rx_mode(self):
        logger.debug("[TX->RX] Restoring RX mode")
        try:
            if self.lora:
                self.lora.clearIrqStatus(0xFF)
                self.lora.setStandby()
                await asyncio.sleep(self.RADIO_TIMING_DELAY)
                self.lora.applyRfErrata()
                self.lora.request(self.lora.RX_CONTINUOUS)
                await asyncio.sleep(self.RADIO_TIMING_DELAY)
                self.lora.clearIrqStatus(0xFF)
        except Exception as e:
            logger.warning(f"[TX->RX] Failed: {e}")

    async def send(self, data: bytes) -> dict:
        if not self._initialized or self.lora is None:
            raise RuntimeError("Radio not initialised")

        async with self._tx_lock:
            try:
                data_list = list(data)
                length = len(data_list)
                final_timeout_ms, driver_timeout = self._calculate_tx_timeout(length)
                timeout_seconds = (final_timeout_ms / 1000.0) + 0.5
                airtime_ms = final_timeout_ms - 1000

                # Write packet to FIFO
                self.lora.beginPacket()
                self.lora.write(data_list, length)
                self.lora.clearIrqStatus(0xFF)

                # LBT
                tx_ready, lbt_backoff_delays = await self._prepare_radio_for_tx()
                if not tx_ready:
                    raise RuntimeError("Radio not ready for TX")

                # Start TX
                if not await self._execute_transmission(driver_timeout):
                    raise RuntimeError("Failed to start TX")

                # Wait for completion
                tx_ok = await self._wait_for_transmission_complete(timeout_seconds)
                if not tx_ok:
                    raise RuntimeError("TX timeout")

                self._finalize_transmission()
                self._gpio_manager.blink_led(self.txled_pin)

                return {
                    "airtime_ms": airtime_ms,
                    "lbt_attempts": len(lbt_backoff_delays),
                    "lbt_backoff_delays_ms": lbt_backoff_delays,
                    "lbt_channel_busy": len(lbt_backoff_delays) > 0,
                }

            except Exception as e:
                logger.error(f"Failed to send packet: {e}")
                raise
            finally:
                await self._restore_rx_mode()

    async def wait_for_rx(self) -> bytes:
        raise NotImplementedError("Use set_rx_callback(callback) for async RX")

    def sleep(self) -> None:
        if self._initialized and self.lora:
            try:
                self.lora.sleep()
                logger.debug("Radio in sleep mode")
            except Exception as e:
                logger.error(f"Failed to sleep: {e}")

    def get_last_rssi(self) -> int:
        return self.last_rssi

    def get_last_snr(self) -> float:
        return self.last_snr

    def get_last_signal_rssi(self) -> int:
        return self.last_signal_rssi

    # ── Noise floor sampling ─────────────────────────────────────────────────

    def _sample_noise_floor(self):
        if not self._initialized or self.lora is None:
            return
        if self._tx_lock.locked():
            return
        if time.time() - self._last_packet_activity < 0.5:
            return
        if self._is_receiving_packet:
            return

        if self._num_floor_samples < self.NUM_NOISE_FLOOR_SAMPLES:
            try:
                raw_rssi = self.lora.getRssiInst()
                if raw_rssi is not None:
                    # SX127x RSSI raw is in dBm with offset
                    rssi_offset = 157  # HF band default
                    current_rssi = -(raw_rssi - rssi_offset)

                    if self._noise_floor == -99.0:
                        accept_sample = -150 < current_rssi < -30
                    else:
                        accept_sample = current_rssi < (self._noise_floor + self.SAMPLING_THRESHOLD)

                    if accept_sample:
                        self._num_floor_samples += 1
                        self._floor_sample_sum += current_rssi
            except Exception as e:
                logger.debug(f"Noise floor sample failed: {e}")

        elif self._num_floor_samples >= self.NUM_NOISE_FLOOR_SAMPLES and self._floor_sample_sum != 0:
            new_nf = self._floor_sample_sum / self.NUM_NOISE_FLOOR_SAMPLES
            if new_nf < -150:
                new_nf = -150
            elif new_nf > -50:
                new_nf = -50
            self._noise_floor = new_nf
            self._floor_sample_sum = 0.0
            self._num_floor_samples = 0

    def get_noise_floor(self) -> Optional[float]:
        if not self._initialized or self.lora is None:
            return 0.0
        if self._tx_lock.locked():
            return 0.0
        return self._noise_floor

    # ── Frequency / power / modem setters ────────────────────────────────────

    def set_frequency(self, frequency: int) -> bool:
        def op():
            self.frequency = frequency
            self.lora.setFrequency(frequency)
        return self._safe_radio_operation("set frequency", op, f"Freq={frequency/1e6:.1f}MHz")

    def set_tx_power(self, power: int) -> bool:
        def op():
            self.tx_power = power
            self.lora.setTxPower(power, self.lora.TX_POWER_PA_BOOST)
        return self._safe_radio_operation("set TX power", op, f"Power={power}dBm")

    def set_spreading_factor(self, sf: int) -> bool:
        def op():
            self.spreading_factor = sf
            self.lora.setLoRaModulation(sf, self.bandwidth, self.coding_rate)
        return self._safe_radio_operation("set SF", op, f"SF={sf}")

    def set_bandwidth(self, bw: int) -> bool:
        def op():
            self.bandwidth = bw
            self.lora.setLoRaModulation(self.spreading_factor, bw, self.coding_rate)
        return self._safe_radio_operation("set BW", op, f"BW={bw/1000:.0f}kHz")

    def get_status(self) -> dict:
        return {
            "initialized": self._initialized,
            "frequency": self.frequency,
            "tx_power": self.tx_power,
            "spreading_factor": self.spreading_factor,
            "bandwidth": self.bandwidth,
            "coding_rate": self.coding_rate,
            "last_rssi": self.last_rssi,
            "last_snr": self.last_snr,
            "last_signal_rssi": self.last_signal_rssi,
            "crc_error_count": self.crc_error_count,
        }

    # ── CAD ──────────────────────────────────────────────────────────────────

    def set_custom_cad_thresholds(self, peak: int, min_val: int):
        if not (0 <= peak <= 31) or not (0 <= min_val <= 31):
            raise ValueError("CAD thresholds must be 0-31")
        self._custom_cad_peak = peak
        self._custom_cad_min = min_val
        logger.info(f"Custom CAD thresholds: peak={peak}, min={min_val}")

    def clear_custom_cad_thresholds(self):
        self._custom_cad_peak = None
        self._custom_cad_min = None

    def _get_thresholds_for_current_settings(self):
        if self._custom_cad_peak is not None and self._custom_cad_min is not None:
            return (self._custom_cad_peak, self._custom_cad_min)
        thresholds = {
            7: (22, 10), 8: (22, 10), 9: (24, 10),
            10: (25, 10), 11: (26, 10), 12: (30, 10),
        }
        return thresholds.get(self.spreading_factor, (22, 10))

    async def perform_cad(
        self, det_peak=None, det_min=None, timeout=1.0, calibration=False
    ) -> Union[bool, dict]:
        if not self._initialized:
            raise RuntimeError("Radio not initialised")
        if not self.lora:
            raise RuntimeError("LoRa object not available")

        if det_peak is None or det_min is None:
            det_peak, det_min = self._get_thresholds_for_current_settings()

        try:
            self.lora.setStandby()
            await asyncio.sleep(self.RADIO_TIMING_DELAY)

            self.lora.clearIrqStatus(0xFF)
            await asyncio.sleep(0.01)

            self._cad_event.clear()

            # SX127x CAD: map DIO0 to CAD_DONE
            self.lora.setDioIrqParams(
                self.lora.IRQ_CAD_DONE | self.lora.IRQ_CAD_DETECTED,
                0, 0, 0
            )

            # Configure CAD (use default 2-symbol CAD)
            self.lora.setCadParams(1, det_peak, det_min, 0, 0)

            _ = self._gpio_manager.read_pin(self.dio0_pin)
            await asyncio.sleep(0.02)

            self.lora.setCad()
            await asyncio.sleep(0.01)

            try:
                await asyncio.wait_for(self._cad_event.wait(), timeout=timeout)
                self._cad_event.clear()

                irq = self._last_cad_irq_status
                detected = self._last_cad_detected
                cad_done = bool(irq & self.lora.IRQ_CAD_DONE)

                self.lora.clearIrqStatus(0xFF)

                if calibration:
                    return {
                        "sf": self.spreading_factor,
                        "bw": self.bandwidth,
                        "det_peak": det_peak,
                        "det_min": det_min,
                        "detected": detected,
                        "cad_done": cad_done,
                        "timestamp": time.time(),
                        "irq_status": irq,
                    }
                return detected

            except asyncio.TimeoutError:
                logger.debug("CAD timeout - assuming clear")
                self.lora.clearIrqStatus(0xFF)
                if calibration:
                    return {"detected": False, "timeout": True, "sf": self.spreading_factor,
                            "bw": self.bandwidth, "det_peak": det_peak, "det_min": det_min,
                            "timestamp": time.time()}
                return False

        except Exception as e:
            logger.error(f"CAD failed: {e}")
            if calibration:
                return {"detected": False, "error": str(e), "sf": self.spreading_factor,
                        "bw": self.bandwidth, "det_peak": det_peak, "det_min": det_min}
            return False
        finally:
            try:
                self.lora.clearIrqStatus(0xFF)
                self.lora.setStandby()
                await asyncio.sleep(self.RADIO_TIMING_DELAY)
                self.lora.setDioIrqParams(
                    self.lora.IRQ_RX_DONE | self.lora.IRQ_CRC_ERR | self.lora.IRQ_RX_TIMEOUT
                    | self.lora.IRQ_TX_DONE | self.lora.IRQ_CAD_DONE | self.lora.IRQ_CAD_DETECTED,
                    0, 0, 0
                )
                self.lora.clearIrqStatus(0xFF)
                self.lora.applyRfErrata()
                self.lora.request(self.lora.RX_CONTINUOUS)
                await asyncio.sleep(self.RADIO_TIMING_DELAY)
            except Exception as e:
                logger.warning(f"CAD restore RX failed: {e}")

    def check_radio_health(self) -> bool:
        if not self._initialized:
            return False
        if (not hasattr(self, "_rx_irq_task")
                or self._rx_irq_task is None
                or self._rx_irq_task.done()):
            try:
                loop = asyncio.get_running_loop()
                self._rx_irq_task = loop.create_task(self._rx_irq_background_task())
                logger.warning("[RX] Restarted dead RX task")
                return False
            except Exception:
                return False
        return True

    def cleanup(self) -> None:
        if self.lora:
            try:
                self.lora.end()
            except Exception as e:
                logger.error(f"Cleanup error: {e}")
        if hasattr(self, "_gpio_manager"):
            self._gpio_manager.cleanup_all()
        self._interrupt_setup = False
        self._initialized = False
        if SX1276Radio._active_instance is self:
            SX1276Radio._active_instance = None

    @classmethod
    def get_instance(cls, **kwargs):
        if cls._active_instance is not None:
            return cls._active_instance
        return cls(**kwargs)


def create_sx1276_radio(**kwargs) -> SX1276Radio:
    radio = SX1276Radio(**kwargs)
    if radio.begin():
        return radio
    raise RuntimeError("Failed to initialise SX1276 radio")
