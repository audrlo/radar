"""Shared data types, enums, and abstract base class for all radar sensor drivers."""

import json
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional


class ActivityState(str, Enum):
    UNKNOWN = "unknown"
    EMPTY = "empty"
    LYING = "lying"
    SITTING = "sitting"
    STANDING = "standing"
    WALKING = "walking"
    FALLEN = "fallen"


class AlertType(str, Enum):
    FALL_DETECTED = "fall_detected"
    BED_EXIT = "bed_exit"
    VITAL_ANOMALY = "vital_anomaly"
    PROLONGED_INACTIVITY = "prolonged_inactivity"
    ROOM_EMPTY = "room_empty"
    BATHROOM_LONG = "bathroom_long"


@dataclass
class VitalSigns:
    heart_rate_bpm: float
    breathing_rate_bpm: float
    hr_confidence: float = 0.0
    br_confidence: float = 0.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class PresenceData:
    is_present: bool
    activity_state: ActivityState = ActivityState.UNKNOWN
    distance: float = 0.0
    movement_energy: float = 0.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class FallEvent:
    timestamp: float = field(default_factory=time.time)
    confidence: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Alert:
    alert_type: AlertType
    message: str
    severity: int = 1  # 1=info, 2=warning, 3=critical
    data: Dict[str, Any] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "alert_type": self.alert_type.value,
            "message": self.message,
            "severity": self.severity,
            "data": self.data,
            "timestamp": self.timestamp,
        }


class RadarSensor(ABC):
    """Abstract base class for all radar sensor drivers."""

    @abstractmethod
    def start(self) -> bool:
        """Initialize and start the sensor. Returns True on success."""
        ...

    @abstractmethod
    def stop(self) -> None:
        """Stop the sensor and close serial ports."""
        ...

    @abstractmethod
    def read_frame(self) -> Optional[Dict[str, Any]]:
        """Read and parse one frame of data. Returns None on timeout/error."""
        ...

    @abstractmethod
    def is_connected(self) -> bool:
        """Check if the sensor serial port is open and responsive."""
        ...
