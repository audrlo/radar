"""Radar-Based Ambient Health Monitor — Main Application.

Fuses data from TI IWR6843AOPEVM (point cloud, fall detection, activity),
Seeed MR60BHA2 (heart rate, breathing, sleep), and HLK-LD2450 (bathroom zone)
into a unified health tracking engine with SQLite persistence, rolling baselines,
z-score anomaly detection, and alert generation.

Usage:
    python main.py \\
        --ti-data-port /dev/ttyACM0 \\
        --ti-cli-port /dev/ttyACM1 \\
        --ti-config people_counting \\
        --vitals-port /dev/ttyAMA0 \\
        --ld2450 /dev/ttyUSB0

    # Without bathroom sensor:
    python main.py \\
        --ti-data-port /dev/ttyACM0 \\
        --ti-cli-port /dev/ttyACM1 \\
        --vitals-port /dev/ttyAMA0
"""

import argparse
import collections
import json
import signal
import sqlite3
import sys
import threading
import time
from datetime import datetime, timezone
from typing import Any, Deque, Dict, List, Optional

import numpy as np

from common.ld2450_sensor import LD2450Sensor
from common.radar_interface import (
    ActivityState,
    Alert,
    AlertType,
    FallEvent,
    PresenceData,
    VitalSigns,
)
from sensors.iwr6843_sensor import IWR6843Sensor
from sensors.mr60bha2_sensor import MR60BHA2Sensor

# ─── Configuration Constants ──────────────────────────────────────────────────

# Baseline computation
BASELINE_WINDOW = 3600       # 1 hour of 1Hz readings
BASELINE_RECOMPUTE = 100     # recompute every N readings
BASELINE_MIN_SAMPLES = 30    # minimum samples before computing baselines

# Anomaly detection
ANOMALY_Z_WARNING = 2.5      # z-score threshold for warning
ANOMALY_Z_CRITICAL = 3.5     # z-score threshold for critical

# Activity monitoring
INACTIVITY_THRESHOLD = 0.05  # movement energy below this = inactive
INACTIVITY_TIMEOUT = 1800.0  # 30 minutes in seconds

# Bathroom monitoring
BATHROOM_TIMEOUT = 1200.0    # 20 minutes in seconds

# Room empty monitoring
ROOM_EMPTY_TIMEOUT = 300.0   # 5 minutes before flagging room empty

# Display refresh
DISPLAY_INTERVAL = 2.0       # seconds between terminal status updates

# Daily summary
SUMMARY_HOUR = 6             # generate daily summary at 6 AM


# ─── SQLite Schema ─────────────────────────────────────────────────────────────

SCHEMA_SQL = """
CREATE TABLE IF NOT EXISTS vitals (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp REAL,
    heart_rate REAL,
    breathing_rate REAL,
    hr_confidence REAL,
    br_confidence REAL
);

CREATE TABLE IF NOT EXISTS activity (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp REAL,
    state TEXT,
    duration_s REAL
);

CREATE TABLE IF NOT EXISTS alerts (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp REAL,
    alert_type TEXT,
    severity INTEGER,
    message TEXT,
    data TEXT
);

CREATE TABLE IF NOT EXISTS daily_summary (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    date TEXT UNIQUE,
    avg_heart_rate REAL,
    avg_breathing_rate REAL,
    total_sleep_hours REAL,
    total_active_hours REAL,
    fall_count INTEGER,
    alert_count INTEGER
);

CREATE TABLE IF NOT EXISTS baselines (
    key TEXT PRIMARY KEY,
    value REAL,
    updated_at REAL
);

CREATE INDEX IF NOT EXISTS idx_vitals_ts ON vitals(timestamp);
CREATE INDEX IF NOT EXISTS idx_activity_ts ON activity(timestamp);
CREATE INDEX IF NOT EXISTS idx_alerts_ts ON alerts(timestamp);
"""


# ─── HealthTracker ─────────────────────────────────────────────────────────────

class HealthTracker:
    """Core health tracking engine — fuses sensor data, computes baselines,
    detects anomalies, tracks activity, and generates alerts."""

    def __init__(self, db_path: str = "health_data.db"):
        self.db_path = db_path
        self._db: Optional[sqlite3.Connection] = None
        self._lock = threading.Lock()

        # Rolling baseline windows
        self._hr_window: Deque[float] = collections.deque(maxlen=BASELINE_WINDOW)
        self._br_window: Deque[float] = collections.deque(maxlen=BASELINE_WINDOW)
        self._readings_since_recompute = 0

        # Current baselines
        self.hr_mean: float = 0.0
        self.hr_std: float = 0.0
        self.br_mean: float = 0.0
        self.br_std: float = 0.0

        # Activity tracking
        self.current_activity = ActivityState.UNKNOWN
        self._activity_start: float = time.time()
        self._last_movement_time: float = time.time()
        self._is_in_bed: bool = False
        self._inactivity_alerted: bool = False

        # Latest state
        self.latest_vitals: Optional[VitalSigns] = None
        self.latest_presence: Optional[PresenceData] = None
        self.latest_ti_presence: Optional[PresenceData] = None

        # Room occupancy
        self._last_presence_time: float = time.time()
        self._room_empty_alerted: bool = False

        # Bathroom
        self._bathroom_alerted: bool = False

        # Alert callback
        self._alert_callbacks: List = []

        # Daily summary
        self._last_summary_date: Optional[str] = None

        # Stats counters for current day
        self._day_start = time.time()
        self._day_falls = 0
        self._day_alerts = 0

    def init_db(self) -> None:
        """Initialize SQLite database and create tables."""
        self._db = sqlite3.connect(self.db_path, check_same_thread=False)
        self._db.execute("PRAGMA journal_mode=WAL")
        self._db.executescript(SCHEMA_SQL)
        self._db.commit()
        self._load_baselines()
        print(f"[HealthTracker] Database initialized: {self.db_path}")

    def _load_baselines(self) -> None:
        """Load persisted baselines from database."""
        if not self._db:
            return
        try:
            cursor = self._db.execute("SELECT key, value FROM baselines")
            for key, value in cursor.fetchall():
                if key == "hr_mean":
                    self.hr_mean = value
                elif key == "hr_std":
                    self.hr_std = value
                elif key == "br_mean":
                    self.br_mean = value
                elif key == "br_std":
                    self.br_std = value

            if self.hr_mean > 0:
                print(
                    f"[HealthTracker] Loaded baselines — "
                    f"HR: {self.hr_mean:.1f} +/- {self.hr_std:.1f}, "
                    f"BR: {self.br_mean:.1f} +/- {self.br_std:.1f}"
                )
        except sqlite3.Error:
            pass

    def _save_baselines(self) -> None:
        """Persist current baselines to database."""
        if not self._db:
            return
        now = time.time()
        with self._lock:
            try:
                for key, value in [
                    ("hr_mean", self.hr_mean),
                    ("hr_std", self.hr_std),
                    ("br_mean", self.br_mean),
                    ("br_std", self.br_std),
                ]:
                    self._db.execute(
                        "INSERT OR REPLACE INTO baselines (key, value, updated_at) VALUES (?, ?, ?)",
                        (key, value, now),
                    )
                self._db.commit()
            except sqlite3.Error as e:
                print(f"[HealthTracker] DB error saving baselines: {e}")

    def process_vitals(self, vitals: VitalSigns) -> None:
        """Process a vitals reading: log, update baselines, check anomalies."""
        self.latest_vitals = vitals

        # Log to database
        self._log_vitals(vitals)

        # Update rolling windows
        if vitals.heart_rate_bpm > 0:
            self._hr_window.append(vitals.heart_rate_bpm)
        if vitals.breathing_rate_bpm > 0:
            self._br_window.append(vitals.breathing_rate_bpm)

        self._readings_since_recompute += 1

        # Recompute baselines periodically
        if self._readings_since_recompute >= BASELINE_RECOMPUTE:
            self._recompute_baselines()
            self._readings_since_recompute = 0

        # Check for anomalies
        self._check_vitals_anomaly(vitals)

    def _log_vitals(self, vitals: VitalSigns) -> None:
        """Write vitals to SQLite."""
        if not self._db:
            return
        with self._lock:
            try:
                self._db.execute(
                    "INSERT INTO vitals (timestamp, heart_rate, breathing_rate, hr_confidence, br_confidence) VALUES (?, ?, ?, ?, ?)",
                    (
                        vitals.timestamp,
                        vitals.heart_rate_bpm,
                        vitals.breathing_rate_bpm,
                        vitals.hr_confidence,
                        vitals.br_confidence,
                    ),
                )
                self._db.commit()
            except sqlite3.Error as e:
                print(f"[HealthTracker] DB error logging vitals: {e}")

    def _recompute_baselines(self) -> None:
        """Recompute mean/std from rolling windows."""
        if len(self._hr_window) >= BASELINE_MIN_SAMPLES:
            hr_arr = np.array(self._hr_window)
            self.hr_mean = float(np.mean(hr_arr))
            self.hr_std = float(np.std(hr_arr))

        if len(self._br_window) >= BASELINE_MIN_SAMPLES:
            br_arr = np.array(self._br_window)
            self.br_mean = float(np.mean(br_arr))
            self.br_std = float(np.std(br_arr))

        self._save_baselines()

    def _check_vitals_anomaly(self, vitals: VitalSigns) -> None:
        """Z-score anomaly detection against patient baselines."""
        if self.hr_std > 0 and vitals.heart_rate_bpm > 0:
            z_hr = abs(vitals.heart_rate_bpm - self.hr_mean) / self.hr_std
            if z_hr > ANOMALY_Z_CRITICAL:
                self._emit_alert(Alert(
                    alert_type=AlertType.VITAL_ANOMALY,
                    message=f"Heart rate critically anomalous: {vitals.heart_rate_bpm:.0f} bpm (z={z_hr:.1f}, baseline {self.hr_mean:.0f}+/-{self.hr_std:.0f})",
                    severity=3,
                    data={"metric": "heart_rate", "value": vitals.heart_rate_bpm, "z_score": z_hr, "baseline_mean": self.hr_mean, "baseline_std": self.hr_std},
                ))
            elif z_hr > ANOMALY_Z_WARNING:
                self._emit_alert(Alert(
                    alert_type=AlertType.VITAL_ANOMALY,
                    message=f"Heart rate anomalous: {vitals.heart_rate_bpm:.0f} bpm (z={z_hr:.1f}, baseline {self.hr_mean:.0f}+/-{self.hr_std:.0f})",
                    severity=2,
                    data={"metric": "heart_rate", "value": vitals.heart_rate_bpm, "z_score": z_hr, "baseline_mean": self.hr_mean, "baseline_std": self.hr_std},
                ))

        if self.br_std > 0 and vitals.breathing_rate_bpm > 0:
            z_br = abs(vitals.breathing_rate_bpm - self.br_mean) / self.br_std
            if z_br > ANOMALY_Z_CRITICAL:
                self._emit_alert(Alert(
                    alert_type=AlertType.VITAL_ANOMALY,
                    message=f"Breathing rate critically anomalous: {vitals.breathing_rate_bpm:.0f} bpm (z={z_br:.1f}, baseline {self.br_mean:.0f}+/-{self.br_std:.0f})",
                    severity=3,
                    data={"metric": "breathing_rate", "value": vitals.breathing_rate_bpm, "z_score": z_br, "baseline_mean": self.br_mean, "baseline_std": self.br_std},
                ))
            elif z_br > ANOMALY_Z_WARNING:
                self._emit_alert(Alert(
                    alert_type=AlertType.VITAL_ANOMALY,
                    message=f"Breathing rate anomalous: {vitals.breathing_rate_bpm:.0f} bpm (z={z_br:.1f}, baseline {self.br_mean:.0f}+/-{self.br_std:.0f})",
                    severity=2,
                    data={"metric": "breathing_rate", "value": vitals.breathing_rate_bpm, "z_score": z_br, "baseline_mean": self.br_mean, "baseline_std": self.br_std},
                ))

    def process_presence(self, presence: PresenceData, source: str = "iwr6843") -> None:
        """Process presence/activity data from a sensor."""
        now = time.time()

        if source == "iwr6843":
            self.latest_ti_presence = presence
        else:
            self.latest_presence = presence

        # Update room occupancy tracking
        if presence.is_present:
            self._last_presence_time = now
            self._room_empty_alerted = False

        # Track movement for inactivity monitoring
        if presence.movement_energy > INACTIVITY_THRESHOLD:
            self._last_movement_time = now
            self._inactivity_alerted = False

        # Activity state change tracking (from TI sensor)
        if source == "iwr6843" and presence.activity_state != self.current_activity:
            self._log_activity_change(presence.activity_state, now)

    def _log_activity_change(self, new_state: ActivityState, now: float) -> None:
        """Log activity state transition with duration of previous state."""
        duration = now - self._activity_start
        prev_state = self.current_activity

        # Log previous state duration to DB
        if self._db and prev_state != ActivityState.UNKNOWN:
            with self._lock:
                try:
                    self._db.execute(
                        "INSERT INTO activity (timestamp, state, duration_s) VALUES (?, ?, ?)",
                        (self._activity_start, prev_state.value, duration),
                    )
                    self._db.commit()
                except sqlite3.Error as e:
                    print(f"[HealthTracker] DB error logging activity: {e}")

        # Bed entry/exit detection
        if new_state == ActivityState.LYING and prev_state != ActivityState.LYING:
            self._is_in_bed = True
        elif prev_state == ActivityState.LYING and new_state != ActivityState.LYING:
            self._is_in_bed = False

        self.current_activity = new_state
        self._activity_start = now

    def process_fall(self, fall: FallEvent, source: str = "iwr6843") -> None:
        """Process a fall event from a sensor."""
        self._day_falls += 1

    def process_alert(self, alert: Alert) -> None:
        """Process an alert from any sensor — log and invoke callbacks."""
        self._emit_alert(alert)

    def check_timed_alerts(self) -> None:
        """Check for time-based alerts — call periodically from main loop."""
        now = time.time()

        # Room empty check
        if (now - self._last_presence_time) > ROOM_EMPTY_TIMEOUT and not self._room_empty_alerted:
            self._room_empty_alerted = True
            self._emit_alert(Alert(
                alert_type=AlertType.ROOM_EMPTY,
                message=f"Room appears empty for {ROOM_EMPTY_TIMEOUT / 60:.0f} minutes",
                severity=1,
                data={"duration_s": now - self._last_presence_time},
            ))

        # Inactivity check (only when present and not in bed)
        if (
            not self._is_in_bed
            and not self._inactivity_alerted
            and (now - self._last_movement_time) > INACTIVITY_TIMEOUT
            and (now - self._last_presence_time) < ROOM_EMPTY_TIMEOUT  # only if room is occupied
        ):
            self._inactivity_alerted = True
            self._emit_alert(Alert(
                alert_type=AlertType.PROLONGED_INACTIVITY,
                message=f"Very low activity for {INACTIVITY_TIMEOUT / 60:.0f} minutes while not in bed",
                severity=2,
                data={"duration_s": now - self._last_movement_time},
            ))

    def check_bathroom(self, ld2450: LD2450Sensor) -> None:
        """Check bathroom zone occupancy duration."""
        if ld2450.is_occupied:
            minutes = ld2450.get_occupancy_minutes()
            if minutes > (BATHROOM_TIMEOUT / 60.0) and not self._bathroom_alerted:
                self._bathroom_alerted = True
                self._emit_alert(Alert(
                    alert_type=AlertType.BATHROOM_LONG,
                    message=f"Bathroom occupied for {minutes:.0f} minutes (threshold: {BATHROOM_TIMEOUT / 60:.0f} min)",
                    severity=2,
                    data={"duration_min": minutes},
                ))
        else:
            self._bathroom_alerted = False

    def _emit_alert(self, alert: Alert) -> None:
        """Log alert to database and invoke handle_alert."""
        self._day_alerts += 1

        # Log to database
        if self._db:
            with self._lock:
                try:
                    self._db.execute(
                        "INSERT INTO alerts (timestamp, alert_type, severity, message, data) VALUES (?, ?, ?, ?, ?)",
                        (
                            alert.timestamp,
                            alert.alert_type.value,
                            alert.severity,
                            alert.message,
                            json.dumps(alert.data),
                        ),
                    )
                    self._db.commit()
                except sqlite3.Error as e:
                    print(f"[HealthTracker] DB error logging alert: {e}")

        handle_alert(alert)

    def get_daily_summary(self) -> Dict[str, Any]:
        """Generate a daily summary JSON payload for care reports."""
        now = time.time()
        today = datetime.now(timezone.utc).strftime("%Y-%m-%d")

        # Compute averages from today's data
        avg_hr = self.hr_mean
        avg_br = self.br_mean

        # Activity breakdown from DB
        activity_breakdown = {}
        total_sleep_hours = 0.0
        total_active_hours = 0.0

        if self._db:
            day_start = datetime.now(timezone.utc).replace(
                hour=0, minute=0, second=0, microsecond=0
            ).timestamp()
            try:
                cursor = self._db.execute(
                    "SELECT state, SUM(duration_s) FROM activity WHERE timestamp >= ? GROUP BY state",
                    (day_start,),
                )
                for state, total_s in cursor.fetchall():
                    hours = (total_s or 0) / 3600.0
                    activity_breakdown[state] = round(hours, 2)
                    if state == ActivityState.LYING.value:
                        total_sleep_hours = hours
                    elif state in (ActivityState.WALKING.value, ActivityState.STANDING.value):
                        total_active_hours += hours
            except sqlite3.Error:
                pass

        summary = {
            "date": today,
            "avg_heart_rate": round(avg_hr, 1),
            "avg_breathing_rate": round(avg_br, 1),
            "total_sleep_hours": round(total_sleep_hours, 2),
            "total_active_hours": round(total_active_hours, 2),
            "activity_breakdown": activity_breakdown,
            "fall_count": self._day_falls,
            "alert_count": self._day_alerts,
            "baselines": {
                "hr_mean": round(self.hr_mean, 1),
                "hr_std": round(self.hr_std, 1),
                "br_mean": round(self.br_mean, 1),
                "br_std": round(self.br_std, 1),
            },
        }

        # Save to daily_summary table
        if self._db:
            with self._lock:
                try:
                    self._db.execute(
                        """INSERT OR REPLACE INTO daily_summary
                           (date, avg_heart_rate, avg_breathing_rate, total_sleep_hours,
                            total_active_hours, fall_count, alert_count)
                           VALUES (?, ?, ?, ?, ?, ?, ?)""",
                        (
                            today,
                            avg_hr,
                            avg_br,
                            total_sleep_hours,
                            total_active_hours,
                            self._day_falls,
                            self._day_alerts,
                        ),
                    )
                    self._db.commit()
                except sqlite3.Error:
                    pass

        return summary

    def close(self) -> None:
        """Close database connection."""
        if self._db:
            self._db.close()


# ─── Alert Handler Hook ──────────────────────────────────────────────────────

def handle_alert(alert: Alert) -> None:
    """Central alert handler — extend with external integrations.

    This is the hook point for integrating with external systems:
      - Sam voice alert to the senior
      - Push notification to family companion app
      - Nurse call system integration
      - Daily summary → cloud LLM → care report
    """
    severity_labels = {1: "INFO", 2: "WARNING", 3: "CRITICAL"}
    label = severity_labels.get(alert.severity, "UNKNOWN")
    ts = datetime.fromtimestamp(alert.timestamp).strftime("%H:%M:%S")

    print(f"\n{'='*60}")
    print(f"  ALERT [{label}] at {ts}")
    print(f"  Type: {alert.alert_type.value}")
    print(f"  {alert.message}")
    if alert.data:
        print(f"  Data: {json.dumps(alert.data, indent=2)}")
    print(f"{'='*60}\n")

    # --- Add your integrations below ---
    # sam_voice.speak(f"Alert: {alert.message}")
    # api.post("/alerts", json=alert.to_dict())
    # if alert.severity >= 3:
    #     nurse_call.trigger(room_id, alert)


# ─── Sensor Reading Threads ──────────────────────────────────────────────────

def ti_sensor_loop(
    sensor: IWR6843Sensor, tracker: HealthTracker, stop_event: threading.Event
) -> None:
    """Background thread: read TI IWR6843 data and feed to tracker."""
    while not stop_event.is_set():
        try:
            data = sensor.read_frame()
            if data:
                presence = data.get("presence")
                if presence:
                    tracker.process_presence(presence, source="iwr6843")

                vitals = data.get("vitals")
                if vitals:
                    tracker.process_vitals(vitals)

                for fall in data.get("falls", []):
                    tracker.process_fall(fall, source="iwr6843")

                for alert in data.get("alerts", []):
                    tracker.process_alert(alert)
        except Exception as e:
            print(f"[TI Loop] Error: {e}")
            time.sleep(1)

        time.sleep(0.01)


def vitals_sensor_loop(
    sensor: MR60BHA2Sensor, tracker: HealthTracker, stop_event: threading.Event
) -> None:
    """Background thread: read MR60BHA2 vitals data and feed to tracker."""
    while not stop_event.is_set():
        try:
            data = sensor.read_frame()
            if data:
                presence = data.get("presence")
                if presence:
                    tracker.process_presence(presence, source="mr60bha2")

                vitals = data.get("vitals")
                if vitals:
                    tracker.process_vitals(vitals)

                for fall in data.get("falls", []):
                    tracker.process_fall(fall, source="mr60bha2")

                for alert in data.get("alerts", []):
                    tracker.process_alert(alert)
        except Exception as e:
            print(f"[Vitals Loop] Error: {e}")
            time.sleep(1)

        time.sleep(0.05)


def ld2450_sensor_loop(
    sensor: LD2450Sensor, tracker: HealthTracker, stop_event: threading.Event
) -> None:
    """Background thread: read LD2450 bathroom zone data."""
    while not stop_event.is_set():
        try:
            data = sensor.read_frame()
            if data:
                tracker.check_bathroom(sensor)
        except Exception as e:
            print(f"[LD2450 Loop] Error: {e}")
            time.sleep(1)

        time.sleep(0.05)


# ─── Terminal Display ─────────────────────────────────────────────────────────

def print_status(tracker: HealthTracker) -> None:
    """Print current system status to terminal."""
    now_str = datetime.now().strftime("%H:%M:%S")
    lines = [f"\n--- Health Monitor Status [{now_str}] ---"]

    # Vitals
    v = tracker.latest_vitals
    if v and (v.heart_rate_bpm > 0 or v.breathing_rate_bpm > 0):
        lines.append(
            f"  Vitals:   HR {v.heart_rate_bpm:.0f} bpm  |  BR {v.breathing_rate_bpm:.0f} bpm"
        )
    else:
        lines.append("  Vitals:   waiting for data...")

    # Baselines
    if tracker.hr_mean > 0:
        lines.append(
            f"  Baseline: HR {tracker.hr_mean:.0f}+/-{tracker.hr_std:.1f}  |  "
            f"BR {tracker.br_mean:.0f}+/-{tracker.br_std:.1f}"
        )

    # Activity (from TI)
    ti_p = tracker.latest_ti_presence
    if ti_p:
        lines.append(
            f"  Activity: {tracker.current_activity.value}  |  "
            f"Present: {ti_p.is_present}  |  "
            f"In bed: {tracker._is_in_bed}"
        )

    # MR60BHA2 presence
    mr_p = tracker.latest_presence
    if mr_p:
        lines.append(
            f"  MR60BHA2: Present={mr_p.is_present}  Energy={mr_p.movement_energy:.1f}"
        )

    lines.append("-" * 44)
    print("\n".join(lines))


# ─── Main Entry Point ─────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Radar-Based Ambient Health Monitor",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--ti-data-port", default="/dev/ttyACM0", help="TI IWR6843 data port (default: /dev/ttyACM0)"
    )
    parser.add_argument(
        "--ti-cli-port", default="/dev/ttyACM1", help="TI IWR6843 CLI port (default: /dev/ttyACM1)"
    )
    parser.add_argument(
        "--ti-config",
        default="people_counting",
        choices=["people_counting", "vital_signs"],
        help="Chirp configuration (default: people_counting)",
    )
    parser.add_argument(
        "--vitals-port", default="/dev/ttyAMA0", help="MR60BHA2 UART port (default: /dev/ttyAMA0)"
    )
    parser.add_argument(
        "--ld2450", default=None, help="LD2450 serial port (default: disabled, use /dev/ttyUSB0)"
    )
    parser.add_argument(
        "--db-path", default="health_data.db", help="SQLite database path (default: health_data.db)"
    )
    args = parser.parse_args()

    # Initialize health tracker
    tracker = HealthTracker(db_path=args.db_path)
    tracker.init_db()

    # Initialize sensors
    ti_sensor = IWR6843Sensor(
        data_port=args.ti_data_port,
        cli_port=args.ti_cli_port,
        config_name=args.ti_config,
    )
    vitals_sensor = MR60BHA2Sensor(port=args.vitals_port)
    ld2450_sensor: Optional[LD2450Sensor] = None
    if args.ld2450:
        ld2450_sensor = LD2450Sensor(port=args.ld2450)

    # Start sensors
    sensors_started = []
    if ti_sensor.start():
        sensors_started.append("TI IWR6843")
    else:
        print("[Main] WARNING: TI IWR6843 failed to start — continuing without spatial data")

    if vitals_sensor.start():
        sensors_started.append("Seeed MR60BHA2")
    else:
        print("[Main] WARNING: MR60BHA2 failed to start — continuing without vitals")

    if ld2450_sensor:
        if ld2450_sensor.start():
            sensors_started.append("HLK-LD2450")
        else:
            print("[Main] WARNING: LD2450 failed to start — continuing without bathroom monitoring")
            ld2450_sensor = None

    if not sensors_started:
        print("[Main] ERROR: No sensors started. Check connections and try again.")
        sys.exit(1)

    print(f"\n[Main] Active sensors: {', '.join(sensors_started)}")
    print("[Main] Health monitoring started. Press Ctrl+C to stop.\n")

    # Set up stop event and threads
    stop_event = threading.Event()
    threads: List[threading.Thread] = []

    if ti_sensor.is_connected():
        t = threading.Thread(
            target=ti_sensor_loop, args=(ti_sensor, tracker, stop_event), daemon=True
        )
        t.start()
        threads.append(t)

    if vitals_sensor.is_connected():
        t = threading.Thread(
            target=vitals_sensor_loop, args=(vitals_sensor, tracker, stop_event), daemon=True
        )
        t.start()
        threads.append(t)

    if ld2450_sensor and ld2450_sensor.is_connected():
        t = threading.Thread(
            target=ld2450_sensor_loop, args=(ld2450_sensor, tracker, stop_event), daemon=True
        )
        t.start()
        threads.append(t)

    # Graceful shutdown handler
    def shutdown(signum, frame):
        print("\n[Main] Shutting down...")
        stop_event.set()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # Main loop — display status and check timed alerts
    last_display = 0.0
    last_summary_check = 0.0

    try:
        while not stop_event.is_set():
            now = time.time()

            # Periodic status display
            if now - last_display >= DISPLAY_INTERVAL:
                print_status(tracker)
                last_display = now

            # Timed alert checks
            tracker.check_timed_alerts()

            # Daily summary generation (check every 5 minutes)
            if now - last_summary_check >= 300:
                current_hour = datetime.now().hour
                today = datetime.now().strftime("%Y-%m-%d")
                if current_hour == SUMMARY_HOUR and tracker._last_summary_date != today:
                    summary = tracker.get_daily_summary()
                    tracker._last_summary_date = today
                    print(f"\n[Daily Summary] {json.dumps(summary, indent=2)}\n")
                last_summary_check = now

            time.sleep(1.0)
    except KeyboardInterrupt:
        pass

    # Cleanup
    print("[Main] Stopping sensors...")
    stop_event.set()

    for t in threads:
        t.join(timeout=3.0)

    ti_sensor.stop()
    vitals_sensor.stop()
    if ld2450_sensor:
        ld2450_sensor.stop()

    # Final daily summary
    summary = tracker.get_daily_summary()
    print(f"\n[Final Summary] {json.dumps(summary, indent=2)}")

    tracker.close()
    print("[Main] Shutdown complete.")


if __name__ == "__main__":
    main()
