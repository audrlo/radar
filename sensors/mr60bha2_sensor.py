"""Seeed MR60BHA2 60GHz vitals radar driver.

Connects via GPIO UART at 115200 baud (/dev/ttyAMA0).
Provides heart rate, breathing rate, sleep staging, presence, and on-chip fall detection.

Protocol format:
  Header:     0x53 0x59 (2 bytes)
  Control:    2 bytes
  Command:    2 bytes (message type)
  Data len:   2 bytes (little-endian uint16)
  Payload:    N bytes
  Checksum:   1 byte (XOR of all preceding bytes)
  End marker: 0x54 0x43 (2 bytes)
"""

import struct
import sys
import time
from typing import Any, Dict, List, Optional

import serial

sys.path.insert(0, __file__.rsplit("/", 2)[0])
from common.radar_interface import (
    ActivityState,
    Alert,
    AlertType,
    FallEvent,
    PresenceData,
    RadarSensor,
    VitalSigns,
)

# Protocol constants
FRAME_HEADER = bytes([0x53, 0x59])
FRAME_END = bytes([0x54, 0x43])

# Command word identifiers
CMD_HEARTBEAT = 0x0102
CMD_PRESENCE = 0x0301
CMD_MOVEMENT = 0x0302
CMD_BODY_ENERGY = 0x0305
CMD_BREATHING_RATE = 0x0501
CMD_HEART_RATE = 0x0502
CMD_BREATHING_WAVEFORM = 0x0503
CMD_SLEEP_STATUS = 0x0601
CMD_FALL_DETECTION = 0x8001

# Vitals sanity bounds
HR_MIN, HR_MAX = 30.0, 200.0
BR_MIN, BR_MAX = 5.0, 40.0

# Minimum interval between vitals emissions (seconds)
VITALS_MIN_INTERVAL = 1.0

# Bed exit cooldown (seconds)
BED_EXIT_COOLDOWN = 30.0


class MR60BHA2Sensor(RadarSensor):
    """Driver for Seeed MR60BHA2 60GHz breathing and heartbeat radar."""

    def __init__(self, port: str = "/dev/ttyAMA0", baud: int = 115200):
        self.port = port
        self.baud = baud
        self._serial: Optional[serial.Serial] = None
        self._buffer = bytearray()

        # Latest parsed state
        self.is_present: bool = False
        self.movement_status: int = 0  # 0=none, 1=stationary, 2=active
        self.movement_energy: float = 0.0
        self.heart_rate: float = 0.0
        self.breathing_rate: float = 0.0
        self.sleep_status: int = 0  # 0=awake, 1=light, 2=deep
        self.fall_detected: bool = False

        # Timing
        self._last_vitals_time: float = 0.0
        self._last_bed_exit_time: float = 0.0
        self._prev_sleep_status: int = 0

        # Accumulated events for read_frame
        self._pending_alerts: List[Alert] = []
        self._pending_falls: List[FallEvent] = []

    def start(self) -> bool:
        try:
            self._serial = serial.Serial(
                self.port,
                self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
            )
            self._buffer.clear()
            print(f"[MR60BHA2] Connected on {self.port} at {self.baud} baud")
            return True
        except serial.SerialException as e:
            print(f"[MR60BHA2] Failed to open {self.port}: {e}")
            return False

    def stop(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()
            print("[MR60BHA2] Port closed")

    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def read_frame(self) -> Optional[Dict[str, Any]]:
        """Read bytes from UART, parse any complete frames, return latest state."""
        if not self.is_connected():
            return None

        try:
            incoming = self._serial.read(256)
            if incoming:
                self._buffer.extend(incoming)
        except serial.SerialException:
            return None

        # Parse all complete frames in buffer
        frames_parsed = 0
        while True:
            frame = self._extract_frame()
            if frame is None:
                break
            self._parse_frame(frame)
            frames_parsed += 1

        if frames_parsed == 0:
            return None

        # Build result
        now = time.time()
        result: Dict[str, Any] = {
            "source": "mr60bha2",
            "timestamp": now,
            "presence": PresenceData(
                is_present=self.is_present,
                activity_state=self._get_activity_state(),
                movement_energy=self.movement_energy,
                timestamp=now,
            ),
        }

        # Emit vitals at most once per second
        if (
            self.heart_rate > 0 or self.breathing_rate > 0
        ) and (now - self._last_vitals_time) >= VITALS_MIN_INTERVAL:
            result["vitals"] = VitalSigns(
                heart_rate_bpm=self.heart_rate,
                breathing_rate_bpm=self.breathing_rate,
                hr_confidence=0.8 if HR_MIN <= self.heart_rate <= HR_MAX else 0.0,
                br_confidence=0.8 if BR_MIN <= self.breathing_rate <= BR_MAX else 0.0,
                timestamp=now,
            )
            self._last_vitals_time = now

        # Emit pending alerts
        if self._pending_alerts:
            result["alerts"] = list(self._pending_alerts)
            self._pending_alerts.clear()

        # Emit pending fall events
        if self._pending_falls:
            result["falls"] = list(self._pending_falls)
            self._pending_falls.clear()

        return result

    def _extract_frame(self) -> Optional[bytes]:
        """Extract one complete frame from the buffer, or None if incomplete."""
        # Find header
        header_idx = self._buffer.find(FRAME_HEADER)
        if header_idx < 0:
            # No header found — discard buffer up to last byte (could be partial header)
            if len(self._buffer) > 1:
                self._buffer = self._buffer[-1:]
            return None

        # Discard any bytes before header
        if header_idx > 0:
            self._buffer = self._buffer[header_idx:]

        # Minimum frame size: header(2) + control(2) + command(2) + length(2) + checksum(1) + end(2) = 11
        if len(self._buffer) < 11:
            return None

        # Read data length from bytes 6-7 (little-endian uint16)
        data_len = struct.unpack_from("<H", self._buffer, 6)[0]
        total_len = 2 + 2 + 2 + 2 + data_len + 1 + 2  # header + ctrl + cmd + len + payload + cksum + end

        if len(self._buffer) < total_len:
            return None

        frame = bytes(self._buffer[:total_len])
        self._buffer = self._buffer[total_len:]

        # Verify end marker
        if frame[-2:] != FRAME_END:
            return None

        # Verify checksum — XOR of all bytes except checksum and end marker
        check_data = frame[:-3]  # everything before checksum byte and end marker
        computed = 0
        for b in check_data:
            computed ^= b
        if computed != frame[-3]:
            return None

        return frame

    def _parse_frame(self, frame: bytes) -> None:
        """Parse a validated frame and update internal state."""
        # Command word at bytes 4-5 (big-endian for the command identifier)
        cmd = struct.unpack_from(">H", frame, 4)[0]
        payload = frame[8:-3]  # data between length field and checksum

        if cmd == CMD_HEARTBEAT:
            # Module keep-alive, ignore
            pass

        elif cmd == CMD_PRESENCE:
            if len(payload) >= 1:
                self.is_present = payload[0] == 1
                if not self.is_present:
                    self.fall_detected = False

        elif cmd == CMD_MOVEMENT:
            if len(payload) >= 1:
                self.movement_status = payload[0]
                if self.movement_status in (1, 2):
                    self.fall_detected = False

        elif cmd == CMD_BODY_ENERGY:
            if len(payload) >= 4:
                self.movement_energy = struct.unpack_from("<f", payload, 0)[0]

        elif cmd == CMD_BREATHING_RATE:
            if len(payload) >= 4:
                br = struct.unpack_from("<f", payload, 0)[0]
                if BR_MIN <= br <= BR_MAX:
                    self.breathing_rate = br

        elif cmd == CMD_HEART_RATE:
            if len(payload) >= 4:
                hr = struct.unpack_from("<f", payload, 0)[0]
                if HR_MIN <= hr <= HR_MAX:
                    self.heart_rate = hr

        elif cmd == CMD_BREATHING_WAVEFORM:
            # Raw waveform data — not used in health tracking
            pass

        elif cmd == CMD_SLEEP_STATUS:
            if len(payload) >= 1:
                new_status = payload[0]
                self._check_bed_exit(new_status)
                self._prev_sleep_status = self.sleep_status
                self.sleep_status = new_status

        elif cmd == CMD_FALL_DETECTION:
            if len(payload) >= 1 and payload[0] == 1:
                self.fall_detected = True
                fall = FallEvent(
                    timestamp=time.time(),
                    confidence=0.80,  # MR60BHA2 on-chip fall ~80% accuracy
                )
                self._pending_falls.append(fall)
                self._pending_alerts.append(
                    Alert(
                        alert_type=AlertType.FALL_DETECTED,
                        message="Fall detected by MR60BHA2 on-chip algorithm",
                        severity=3,
                        data={"source": "mr60bha2", "confidence": 0.80},
                    )
                )

    def _check_bed_exit(self, new_sleep_status: int) -> None:
        """Detect bed exit: sleep (light/deep) → awake transition."""
        now = time.time()
        # Was sleeping (1=light or 2=deep), now awake (0)
        if self.sleep_status in (1, 2) and new_sleep_status == 0:
            if (now - self._last_bed_exit_time) >= BED_EXIT_COOLDOWN:
                self._last_bed_exit_time = now
                self._pending_alerts.append(
                    Alert(
                        alert_type=AlertType.BED_EXIT,
                        message="Bed exit detected — transitioned from sleep to awake",
                        severity=2,
                        data={
                            "previous_sleep_status": self.sleep_status,
                            "source": "mr60bha2",
                        },
                    )
                )

    def _get_activity_state(self) -> ActivityState:
        """Map movement status + sleep status to ActivityState."""
        if not self.is_present:
            return ActivityState.EMPTY
        if self.fall_detected:
            return ActivityState.FALLEN
        if self.sleep_status in (1, 2):
            return ActivityState.LYING
        if self.movement_status == 2:
            return ActivityState.WALKING
        if self.movement_status == 1:
            return ActivityState.SITTING
        return ActivityState.UNKNOWN


# --- Standalone test mode ---
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="MR60BHA2 vitals radar test")
    parser.add_argument("--port", default="/dev/ttyAMA0", help="Serial port")
    args = parser.parse_args()

    sensor = MR60BHA2Sensor(port=args.port)
    if not sensor.start():
        sys.exit(1)

    print("[MR60BHA2] Reading data... (Ctrl+C to stop)\n")
    try:
        while True:
            data = sensor.read_frame()
            if data:
                presence = data.get("presence")
                if presence:
                    print(
                        f"  Present: {presence.is_present}  "
                        f"Activity: {presence.activity_state.value}  "
                        f"Energy: {presence.movement_energy:.1f}"
                    )
                vitals = data.get("vitals")
                if vitals:
                    print(
                        f"  HR: {vitals.heart_rate_bpm:.1f} bpm  "
                        f"BR: {vitals.breathing_rate_bpm:.1f} bpm"
                    )
                alerts = data.get("alerts", [])
                for alert in alerts:
                    print(f"  ** ALERT [{alert.severity}]: {alert.message}")
                falls = data.get("falls", [])
                for fall in falls:
                    print(f"  ** FALL DETECTED confidence={fall.confidence:.0%}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[MR60BHA2] Stopping...")
    finally:
        sensor.stop()
