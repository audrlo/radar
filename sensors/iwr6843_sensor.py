"""TI IWR6843AOPEVM 60GHz FMCW radar driver.

Connects via USB dual UART through MMWAVEICBOOST carrier board:
  /dev/ttyACM0 — data port at 921600 baud (TLV binary frames)
  /dev/ttyACM1 — CLI port at 115200 baud (chirp configuration)

Provides 3D point cloud, fall detection, activity classification, and optional
firmware-extracted vital signs.

TLV frame format:
  Magic word: 02 01 04 03 06 05 08 07 (8 bytes)
  Header: 40 bytes total (includes magic) — version, totalPacketLen, platform,
          frameNumber, timeCpuCycles, numDetectedObj, numTLVs, subFrameNumber
  TLV payloads: type (4B) + length (4B) + data (variable)
    Type 1 — DETECTED_POINTS: array of (x, y, z, doppler) float32
    Type 6 — VITAL_SIGNS: heart rate + breathing rate (last 8 bytes of payload)
    Type 7 — SIDE_INFO: SNR + noise per point (int16 pairs)
"""

import struct
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
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

# TLV magic word
MAGIC_WORD = bytes([0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07])

# TLV type IDs
TLV_DETECTED_POINTS = 1
TLV_VITAL_SIGNS = 6
TLV_SIDE_INFO = 7

# Header size in bytes (including magic word)
HEADER_SIZE = 40

# Fall detection thresholds
FALL_HEIGHT_DROP = 0.5        # meters — minimum rapid drop to trigger
FALL_FLOOR_THRESHOLD = 0.3    # meters — below this = on floor
FALL_CONFIRM_TIME = 3.0       # seconds at floor level to confirm
FALL_COOLDOWN_TIME = 30.0     # seconds between fall alerts
FALL_MAX_DOPPLER = 0.2        # m/s — max movement to confirm "stayed down"
FALL_DROP_WINDOW = 1.5        # seconds — time window for height drop

# Activity classification thresholds (assumes ~2.5m ceiling mount)
WALKING_HEIGHT = 1.0
WALKING_DOPPLER = 0.3
STANDING_HEIGHT = 0.8
SITTING_HEIGHT = 0.4
SITTING_SPREAD = 0.3
FALLEN_HEIGHT = 0.2

# ─── Chirp configurations ──────────────────────────────────────────────────────
# These are sent line-by-line to the CLI UART port on boot.
# Each line gets echoed back with a "Done" response.

PEOPLE_COUNTING_CFG = """\
sensorStop
flushCfg
dfeDataOutputMode 1
channelCfg 15 7 0
adcCfg 2 1
adcbufCfg -1 0 1 1 1
profileCfg 0 60 7 7 57.14 0 0 70 1 256 5209 0 0 158
chirpCfg 0 0 0 0 0 0 0 1
chirpCfg 1 1 0 0 0 0 0 2
chirpCfg 2 2 0 0 0 0 0 4
frameCfg 0 2 16 0 100 1 0
lowPower 0 0
guiMonitor -1 1 0 0 0 0 1
cfarCfg -1 0 2 8 4 3 0 15 1
cfarCfg -1 1 0 4 2 3 1 15 1
multiObjBeamForming -1 1 0.5
clutterRemoval -1 1
calibDcRangeSig -1 0 -5 8 256
extendedMaxVelocity -1 0
compRangeBiasAndRxChanPhase 0.0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0
measureRangeBiasAndRxChanPhase 0 1.5 0.2
CQRxSatMonitor 0 3 5 121 0
CQSigImgMonitor 0 127 4
analogMonitor 0 0
aoaFovCfg -1 -90 90 -90 90
cfarFovCfg -1 0 0.5 6.0
cfarFovCfg -1 1 -3.0 3.0
sensorStart"""

VITAL_SIGNS_CFG = """\
sensorStop
flushCfg
dfeDataOutputMode 1
channelCfg 15 7 0
adcCfg 2 1
adcbufCfg -1 0 1 1 1
profileCfg 0 60 7 7 170 0 0 70 1 256 5209 0 0 158
chirpCfg 0 0 0 0 0 0 0 1
chirpCfg 1 1 0 0 0 0 0 2
chirpCfg 2 2 0 0 0 0 0 4
frameCfg 0 2 16 0 50 1 0
lowPower 0 0
guiMonitor -1 1 0 0 0 1 1
cfarCfg -1 0 2 8 4 3 0 15 1
cfarCfg -1 1 0 4 2 3 1 15 1
multiObjBeamForming -1 1 0.5
clutterRemoval -1 1
calibDcRangeSig -1 0 -5 8 256
extendedMaxVelocity -1 0
compRangeBiasAndRxChanPhase 0.0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0
measureRangeBiasAndRxChanPhase 0 1.5 0.2
CQRxSatMonitor 0 3 5 121 0
CQSigImgMonitor 0 127 4
analogMonitor 0 0
aoaFovCfg -1 -90 90 -90 90
cfarFovCfg -1 0 0.25 2.0
cfarFovCfg -1 1 -1.0 1.0
sensorStart"""

CONFIGS = {
    "people_counting": PEOPLE_COUNTING_CFG,
    "vital_signs": VITAL_SIGNS_CFG,
}


class IWR6843Sensor(RadarSensor):
    """Driver for TI IWR6843AOPEVM with MMWAVEICBOOST carrier board."""

    def __init__(
        self,
        data_port: str = "/dev/ttyACM0",
        cli_port: str = "/dev/ttyACM1",
        config_name: str = "people_counting",
    ):
        self.data_port_path = data_port
        self.cli_port_path = cli_port
        self.config_name = config_name

        self._data_serial: Optional[serial.Serial] = None
        self._cli_serial: Optional[serial.Serial] = None
        self._buffer = bytearray()

        # Point cloud state
        self.points: np.ndarray = np.empty((0, 4), dtype=np.float32)  # (x,y,z,doppler)
        self.snr: np.ndarray = np.empty(0, dtype=np.int16)
        self.noise: np.ndarray = np.empty(0, dtype=np.int16)

        # Derived state
        self.centroid: Optional[Tuple[float, float, float]] = None  # (x, y, z)
        self.avg_doppler: float = 0.0
        self.num_points: int = 0

        # Fall detection state machine
        self._fall_state = "monitoring"  # monitoring | falling | fallen | cooldown
        self._fall_trigger_time: float = 0.0
        self._fall_cooldown_start: float = 0.0
        self._prev_centroid_z: float = 1.0
        self._prev_centroid_time: float = 0.0

        # Activity state
        self.activity_state = ActivityState.UNKNOWN

        # Firmware vitals (if available from TLV type 6)
        self.fw_heart_rate: float = 0.0
        self.fw_breathing_rate: float = 0.0

        # Events
        self._pending_alerts: List[Alert] = []
        self._pending_falls: List[FallEvent] = []

    def start(self) -> bool:
        try:
            self._data_serial = serial.Serial(
                self.data_port_path, 921600, timeout=0.1
            )
            self._cli_serial = serial.Serial(
                self.cli_port_path, 115200, timeout=0.5
            )
        except serial.SerialException as e:
            print(f"[IWR6843] Failed to open ports: {e}")
            return False

        print(f"[IWR6843] Data port: {self.data_port_path} (921600)")
        print(f"[IWR6843] CLI port:  {self.cli_port_path} (115200)")

        # Send chirp configuration
        config = CONFIGS.get(self.config_name)
        if config is None:
            print(f"[IWR6843] Unknown config '{self.config_name}', using people_counting")
            config = PEOPLE_COUNTING_CFG

        print(f"[IWR6843] Sending '{self.config_name}' config...")
        if not self._send_config(config):
            print("[IWR6843] WARNING: Config may not have been fully accepted")

        self._buffer.clear()
        print("[IWR6843] Sensor started, streaming data")
        return True

    def stop(self) -> None:
        # Try to send sensorStop
        if self._cli_serial and self._cli_serial.is_open:
            try:
                self._cli_serial.write(b"sensorStop\n")
                time.sleep(0.1)
            except serial.SerialException:
                pass
            self._cli_serial.close()

        if self._data_serial and self._data_serial.is_open:
            self._data_serial.close()

        print("[IWR6843] Ports closed")

    def is_connected(self) -> bool:
        return (
            self._data_serial is not None
            and self._data_serial.is_open
            and self._cli_serial is not None
            and self._cli_serial.is_open
        )

    def _send_config(self, config: str) -> bool:
        """Send chirp config line-by-line to CLI port."""
        if not self._cli_serial or not self._cli_serial.is_open:
            return False

        # Flush any pending data
        self._cli_serial.reset_input_buffer()

        for line in config.strip().splitlines():
            line = line.strip()
            if not line or line.startswith("%"):
                continue

            self._cli_serial.write((line + "\n").encode())
            time.sleep(0.05)

            # Read echo/response
            response = b""
            deadline = time.time() + 1.0
            while time.time() < deadline:
                chunk = self._cli_serial.read(256)
                if chunk:
                    response += chunk
                    if b"Done" in response or b"sensorStart" in response:
                        break
                else:
                    break

            resp_text = response.decode("ascii", errors="replace").strip()
            if "Error" in resp_text:
                print(f"[IWR6843] Config error on '{line}': {resp_text}")

        return True

    def read_frame(self) -> Optional[Dict[str, Any]]:
        """Read and parse one TLV frame from the data port."""
        if not self.is_connected():
            return None

        try:
            incoming = self._data_serial.read(4096)
            if incoming:
                self._buffer.extend(incoming)
        except serial.SerialException:
            return None

        frame = self._extract_tlv_frame()
        if frame is None:
            return None

        self._parse_tlv_frame(frame)
        self._run_fall_detection()
        self._classify_activity()

        now = time.time()
        result: Dict[str, Any] = {
            "source": "iwr6843",
            "timestamp": now,
            "num_points": self.num_points,
            "points": self.points.copy() if self.num_points > 0 else None,
            "centroid": self.centroid,
            "presence": PresenceData(
                is_present=self.num_points > 0,
                activity_state=self.activity_state,
                distance=(
                    np.sqrt(self.centroid[0] ** 2 + self.centroid[1] ** 2 + self.centroid[2] ** 2)
                    if self.centroid
                    else 0.0
                ),
                movement_energy=min(abs(self.avg_doppler) * 100, 100.0),
                timestamp=now,
            ),
        }

        # Include firmware vitals if available
        if self.fw_heart_rate > 0 or self.fw_breathing_rate > 0:
            result["vitals"] = VitalSigns(
                heart_rate_bpm=self.fw_heart_rate,
                breathing_rate_bpm=self.fw_breathing_rate,
                hr_confidence=0.5,  # firmware vitals are secondary to MR60BHA2
                br_confidence=0.5,
                timestamp=now,
            )

        if self._pending_alerts:
            result["alerts"] = list(self._pending_alerts)
            self._pending_alerts.clear()

        if self._pending_falls:
            result["falls"] = list(self._pending_falls)
            self._pending_falls.clear()

        return result

    def _extract_tlv_frame(self) -> Optional[bytes]:
        """Find and extract one complete TLV frame from the buffer."""
        # Find magic word
        idx = self._buffer.find(MAGIC_WORD)
        if idx < 0:
            # Keep last 7 bytes (could be partial magic word)
            if len(self._buffer) > 7:
                self._buffer = self._buffer[-7:]
            return None

        # Discard bytes before magic word
        if idx > 0:
            self._buffer = self._buffer[idx:]

        # Need at least a full header
        if len(self._buffer) < HEADER_SIZE:
            return None

        # Read total packet length from header (bytes 12-15, uint32 LE)
        total_len = struct.unpack_from("<I", self._buffer, 12)[0]

        # Sanity check — frames shouldn't exceed 64KB
        if total_len > 65536:
            self._buffer = self._buffer[8:]  # skip past this magic word
            return None

        if len(self._buffer) < total_len:
            return None

        frame = bytes(self._buffer[:total_len])
        self._buffer = self._buffer[total_len:]
        return frame

    def _parse_tlv_frame(self, frame: bytes) -> None:
        """Parse the header and TLV payloads from a complete frame."""
        if len(frame) < HEADER_SIZE:
            return

        # Header fields
        # Bytes 0-7:   magic word
        # Bytes 8-11:  version (uint32)
        # Bytes 12-15: totalPacketLen (uint32)
        # Bytes 16-19: platform (uint32)
        # Bytes 20-23: frameNumber (uint32)
        # Bytes 24-27: timeCpuCycles (uint32)
        # Bytes 28-31: numDetectedObj (uint32)
        # Bytes 32-35: numTLVs (uint32)
        # Bytes 36-39: subFrameNumber (uint32)

        num_detected = struct.unpack_from("<I", frame, 28)[0]
        num_tlvs = struct.unpack_from("<I", frame, 32)[0]

        # Reset point cloud
        self.points = np.empty((0, 4), dtype=np.float32)
        self.snr = np.empty(0, dtype=np.int16)
        self.noise = np.empty(0, dtype=np.int16)
        self.num_points = 0
        self.fw_heart_rate = 0.0
        self.fw_breathing_rate = 0.0

        # Parse TLVs
        offset = HEADER_SIZE
        for _ in range(min(num_tlvs, 20)):  # cap at 20 to avoid runaway
            if offset + 8 > len(frame):
                break

            tlv_type = struct.unpack_from("<I", frame, offset)[0]
            tlv_len = struct.unpack_from("<I", frame, offset + 4)[0]
            offset += 8

            if offset + tlv_len - 8 > len(frame):
                break

            payload = frame[offset : offset + tlv_len - 8]
            offset += tlv_len - 8

            if tlv_type == TLV_DETECTED_POINTS:
                self._parse_point_cloud(payload, num_detected)
            elif tlv_type == TLV_SIDE_INFO:
                self._parse_side_info(payload, num_detected)
            elif tlv_type == TLV_VITAL_SIGNS:
                self._parse_vital_signs(payload)

        # Compute centroid from point cloud
        if self.num_points > 0:
            self.centroid = (
                float(np.mean(self.points[:, 0])),
                float(np.mean(self.points[:, 1])),
                float(np.mean(self.points[:, 2])),
            )
            self.avg_doppler = float(np.mean(np.abs(self.points[:, 3])))
        else:
            self.centroid = None
            self.avg_doppler = 0.0

    def _parse_point_cloud(self, payload: bytes, expected_points: int) -> None:
        """Parse TLV Type 1 — array of (x, y, z, doppler) float32."""
        point_size = 16  # 4 floats * 4 bytes
        n_points = min(len(payload) // point_size, expected_points)
        if n_points == 0:
            return

        self.points = np.frombuffer(
            payload[: n_points * point_size], dtype=np.float32
        ).reshape(n_points, 4)
        self.num_points = n_points

    def _parse_side_info(self, payload: bytes, expected_points: int) -> None:
        """Parse TLV Type 7 — SNR and noise per point (int16 pairs)."""
        pair_size = 4  # 2 int16s
        n_points = min(len(payload) // pair_size, expected_points)
        if n_points == 0:
            return

        data = np.frombuffer(payload[: n_points * pair_size], dtype=np.int16).reshape(
            n_points, 2
        )
        self.snr = data[:, 0]
        self.noise = data[:, 1]

    def _parse_vital_signs(self, payload: bytes) -> None:
        """Parse TLV Type 6 — heart rate + breathing rate (last 8 bytes)."""
        if len(payload) >= 8:
            # Last 8 bytes: breathing_rate (float32) + heart_rate (float32)
            self.fw_breathing_rate = struct.unpack_from("<f", payload, len(payload) - 8)[0]
            self.fw_heart_rate = struct.unpack_from("<f", payload, len(payload) - 4)[0]

    def _classify_activity(self) -> None:
        """Classify activity based on point cloud centroid height and doppler."""
        if self.num_points == 0:
            self.activity_state = ActivityState.EMPTY
            return

        if self.centroid is None:
            self.activity_state = ActivityState.UNKNOWN
            return

        z = self.centroid[2]

        if z < FALLEN_HEIGHT:
            self.activity_state = ActivityState.FALLEN
        elif z > WALKING_HEIGHT and self.avg_doppler > WALKING_DOPPLER:
            self.activity_state = ActivityState.WALKING
        elif z > STANDING_HEIGHT:
            self.activity_state = ActivityState.STANDING
        elif z > SITTING_HEIGHT:
            # Distinguish sitting vs lying by point cloud spread
            if self.num_points > 1:
                spread = float(np.std(self.points[:, 2]))
                if spread > SITTING_SPREAD:
                    self.activity_state = ActivityState.SITTING
                else:
                    self.activity_state = ActivityState.LYING
            else:
                self.activity_state = ActivityState.SITTING
        else:
            self.activity_state = ActivityState.LYING

    def _run_fall_detection(self) -> None:
        """Fall detection state machine.

        States: monitoring → falling → fallen → cooldown → monitoring
        """
        now = time.time()

        if self._fall_state == "cooldown":
            if now - self._fall_cooldown_start >= FALL_COOLDOWN_TIME:
                self._fall_state = "monitoring"
            return

        if self.centroid is None:
            return

        current_z = self.centroid[2]

        if self._fall_state == "monitoring":
            # Check for rapid height drop
            if self._prev_centroid_z > 0:
                dt = now - self._prev_centroid_time
                if dt > 0 and dt < FALL_DROP_WINDOW:
                    z_drop = self._prev_centroid_z - current_z
                    if z_drop > FALL_HEIGHT_DROP and current_z < FALL_FLOOR_THRESHOLD:
                        self._fall_state = "falling"
                        self._fall_trigger_time = now

            self._prev_centroid_z = current_z
            self._prev_centroid_time = now

        elif self._fall_state == "falling":
            # Check if person recovered (false alarm — sat down quickly)
            if current_z > FALL_FLOOR_THRESHOLD:
                self._fall_state = "monitoring"
                return

            # Check for confirmation: stayed at floor level for FALL_CONFIRM_TIME
            if now - self._fall_trigger_time >= FALL_CONFIRM_TIME:
                if current_z < FALL_FLOOR_THRESHOLD and self.avg_doppler < FALL_MAX_DOPPLER:
                    # Confirmed fall
                    self._fall_state = "fallen"
                    fall = FallEvent(
                        timestamp=now,
                        confidence=0.85,
                        x=self.centroid[0],
                        y=self.centroid[1],
                        z=self.centroid[2],
                    )
                    self._pending_falls.append(fall)
                    self._pending_alerts.append(
                        Alert(
                            alert_type=AlertType.FALL_DETECTED,
                            message=(
                                f"Fall detected at ({fall.x:.1f}, {fall.y:.1f}, {fall.z:.1f})m "
                                f"— person on floor for {FALL_CONFIRM_TIME:.0f}s"
                            ),
                            severity=3,
                            data={
                                "source": "iwr6843",
                                "confidence": 0.85,
                                "x": fall.x,
                                "y": fall.y,
                                "z": fall.z,
                            },
                        )
                    )
                    # Enter cooldown
                    self._fall_state = "cooldown"
                    self._fall_cooldown_start = now
                else:
                    # Not confirmed — person moved or recovered
                    self._fall_state = "monitoring"

        elif self._fall_state == "fallen":
            # Shouldn't reach here normally — transitions directly to cooldown
            self._fall_state = "cooldown"
            self._fall_cooldown_start = now


# --- Standalone test mode ---
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="IWR6843 radar test")
    parser.add_argument("--data-port", default="/dev/ttyACM0", help="Data UART port")
    parser.add_argument("--cli-port", default="/dev/ttyACM1", help="CLI UART port")
    parser.add_argument(
        "--config",
        default="people_counting",
        choices=list(CONFIGS.keys()),
        help="Chirp configuration",
    )
    args = parser.parse_args()

    sensor = IWR6843Sensor(
        data_port=args.data_port,
        cli_port=args.cli_port,
        config_name=args.config,
    )
    if not sensor.start():
        sys.exit(1)

    print("[IWR6843] Reading data... (Ctrl+C to stop)\n")
    try:
        while True:
            data = sensor.read_frame()
            if data and data.get("num_points", 0) > 0:
                p = data["presence"]
                c = data.get("centroid")
                centroid_str = (
                    f"({c[0]:.2f}, {c[1]:.2f}, {c[2]:.2f})" if c else "N/A"
                )
                print(
                    f"  Points: {data['num_points']:3d}  "
                    f"Centroid: {centroid_str}  "
                    f"Activity: {p.activity_state.value}"
                )
                vitals = data.get("vitals")
                if vitals and (vitals.heart_rate_bpm > 0 or vitals.breathing_rate_bpm > 0):
                    print(
                        f"  FW Vitals — HR: {vitals.heart_rate_bpm:.1f}  "
                        f"BR: {vitals.breathing_rate_bpm:.1f}"
                    )
                alerts = data.get("alerts", [])
                for alert in alerts:
                    print(f"  ** ALERT [{alert.severity}]: {alert.message}")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n[IWR6843] Stopping...")
    finally:
        sensor.stop()
