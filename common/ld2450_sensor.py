"""HLK-LD2450 24GHz multi-target radar driver for bathroom zone monitoring.

Connects via USB-UART adapter (CP2102/CH340) at 256000 baud → /dev/ttyUSB0.

Protocol: Fixed 30-byte frames.
  Header:  0xAA 0xFF 0x03 0x00 (4 bytes)
  Target 1: x (int16 LE mm), y (int16 LE mm), speed (int16 LE mm/s), resolution (int16) — 8 bytes
  Target 2: same 8 bytes
  Target 3: same 8 bytes
  Footer:  0x55 0xCC (2 bytes)
  Check:   2 bytes

Tracks up to 3 targets simultaneously. Used for zone occupancy monitoring
(bathroom, hallway) with configurable alert on prolonged occupancy.
"""

import struct
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

import serial

sys.path.insert(0, __file__.rsplit("/", 2)[0])
from common.radar_interface import (
    ActivityState,
    Alert,
    AlertType,
    PresenceData,
    RadarSensor,
)

# Protocol constants
FRAME_HEADER = bytes([0xAA, 0xFF, 0x03, 0x00])
FRAME_FOOTER = bytes([0x55, 0xCC])
FRAME_SIZE = 30  # Total frame size in bytes

# Target slot size (x, y, speed, resolution — 4x int16 = 8 bytes)
TARGET_SLOT_SIZE = 8
NUM_TARGET_SLOTS = 3


class LD2450Target:
    """A single detected target from the LD2450."""

    def __init__(self, x_mm: int, y_mm: int, speed_mm_s: int):
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.speed_mm_s = speed_mm_s

    @property
    def x_m(self) -> float:
        return self.x_mm / 1000.0

    @property
    def y_m(self) -> float:
        return self.y_mm / 1000.0

    @property
    def speed_m_s(self) -> float:
        return self.speed_mm_s / 1000.0

    @property
    def is_valid(self) -> bool:
        """Target is valid if x or y is non-zero."""
        return self.x_mm != 0 or self.y_mm != 0

    def __repr__(self) -> str:
        return f"Target(x={self.x_m:.2f}m, y={self.y_m:.2f}m, speed={self.speed_m_s:.2f}m/s)"


class LD2450Sensor(RadarSensor):
    """Driver for HLK-LD2450 24GHz multi-target radar."""

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 256000):
        self.port = port
        self.baud = baud
        self._serial: Optional[serial.Serial] = None
        self._buffer = bytearray()

        # Target state
        self.targets: List[LD2450Target] = []
        self.is_occupied: bool = False

        # Occupancy tracking
        self._occupancy_start: Optional[float] = None
        self._occupancy_duration: float = 0.0

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
            print(f"[LD2450] Connected on {self.port} at {self.baud} baud")
            return True
        except serial.SerialException as e:
            print(f"[LD2450] Failed to open {self.port}: {e}")
            return False

    def stop(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()
            print("[LD2450] Port closed")

    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def read_frame(self) -> Optional[Dict[str, Any]]:
        """Read and parse one frame from the LD2450."""
        if not self.is_connected():
            return None

        try:
            incoming = self._serial.read(128)
            if incoming:
                self._buffer.extend(incoming)
        except serial.SerialException:
            return None

        frame = self._extract_frame()
        if frame is None:
            return None

        self._parse_frame(frame)
        self._update_occupancy()

        now = time.time()
        return {
            "source": "ld2450",
            "timestamp": now,
            "targets": list(self.targets),
            "is_occupied": self.is_occupied,
            "occupancy_duration": self._occupancy_duration,
            "presence": PresenceData(
                is_present=self.is_occupied,
                activity_state=ActivityState.UNKNOWN,
                distance=(
                    min(
                        (t.x_m ** 2 + t.y_m ** 2) ** 0.5
                        for t in self.targets
                        if t.is_valid
                    )
                    if any(t.is_valid for t in self.targets)
                    else 0.0
                ),
                movement_energy=(
                    max(abs(t.speed_m_s) for t in self.targets if t.is_valid) * 10
                    if any(t.is_valid for t in self.targets)
                    else 0.0
                ),
                timestamp=now,
            ),
        }

    def _extract_frame(self) -> Optional[bytes]:
        """Extract one complete 30-byte frame from the buffer."""
        # Find header
        idx = self._buffer.find(FRAME_HEADER)
        if idx < 0:
            if len(self._buffer) > 3:
                self._buffer = self._buffer[-3:]
            return None

        if idx > 0:
            self._buffer = self._buffer[idx:]

        if len(self._buffer) < FRAME_SIZE:
            return None

        frame = bytes(self._buffer[:FRAME_SIZE])
        self._buffer = self._buffer[FRAME_SIZE:]

        # Verify footer at expected position (bytes 28-29)
        if frame[28:30] != FRAME_FOOTER:
            return None

        return frame

    def _parse_frame(self, frame: bytes) -> None:
        """Parse a 30-byte frame into up to 3 target slots."""
        self.targets = []
        offset = 4  # skip header

        for _ in range(NUM_TARGET_SLOTS):
            if offset + TARGET_SLOT_SIZE > len(frame):
                break

            x, y, speed, _resolution = struct.unpack_from("<hhhh", frame, offset)
            self.targets.append(LD2450Target(x_mm=x, y_mm=y, speed_mm_s=speed))
            offset += TARGET_SLOT_SIZE

    def _update_occupancy(self) -> None:
        """Update zone occupancy state and duration tracking."""
        now = time.time()
        any_target = any(t.is_valid for t in self.targets)

        if any_target:
            if not self.is_occupied:
                # Zone just became occupied
                self._occupancy_start = now
            self.is_occupied = True
            self._occupancy_duration = (
                now - self._occupancy_start if self._occupancy_start else 0.0
            )
        else:
            if self.is_occupied:
                # Zone just became empty
                self.is_occupied = False
                self._occupancy_duration = 0.0
                self._occupancy_start = None

    def get_occupancy_minutes(self) -> float:
        """Get current continuous occupancy duration in minutes."""
        return self._occupancy_duration / 60.0


# --- Standalone test mode ---
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="LD2450 bathroom zone radar test")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port")
    args = parser.parse_args()

    sensor = LD2450Sensor(port=args.port)
    if not sensor.start():
        sys.exit(1)

    print("[LD2450] Reading data... (Ctrl+C to stop)\n")
    try:
        while True:
            data = sensor.read_frame()
            if data:
                valid = [t for t in data["targets"] if t.is_valid]
                occ_str = f"OCCUPIED ({data['occupancy_duration']:.0f}s)" if data["is_occupied"] else "empty"
                print(f"  Zone: {occ_str}  Targets: {len(valid)}")
                for i, t in enumerate(valid):
                    print(f"    T{i}: x={t.x_m:.2f}m y={t.y_m:.2f}m speed={t.speed_m_s:.2f}m/s")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[LD2450] Stopping...")
    finally:
        sensor.stop()
