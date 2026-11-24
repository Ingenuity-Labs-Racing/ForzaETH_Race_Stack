"""Shared, ROS-independent contract between the manager and controller adapters."""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Optional
import logging


def normalize_behavior(value):
    """Accept old state strings at the boundary; publish behavior names only."""
    name = str(value).split('.')[-1]
    return {'NMPC_TRACK': 'GB_TRACK'}.get(name, name)


@dataclass
class ControllerInputs:
    pose: Any = None
    speed: Optional[float] = None
    waypoints: Any = None
    behavior: str = 'GB_TRACK'
    control_dt: float = .025
    pose_age_s: float = 0.
    frenet: Any = None
    acceleration: Any = None
    opponent: Any = None
    obstacles: Any = None
    track_length: Optional[float] = None
    scan: Any = None


@dataclass
class ControllerResult:
    speed: float = 0.
    steering: float = 0.
    valid: bool = False
    reason: str = ''
    steering_rate: Optional[float] = None
    diagnostics: dict = field(default_factory=dict)


@dataclass
class ControllerContext:
    rate: float = 40.
    wheelbase: float = .3302
    lookup_table: str = 'NUC2_hangar_pacejka'
    mapping: bool = False
    logger_info: Any = logging.info
    logger_warn: Any = logging.warning


class ControllerAdapter(ABC):
    error = ''

    @abstractmethod
    def step(self, inputs: ControllerInputs) -> ControllerResult:
        pass

    @abstractmethod
    def reset(self, steering=0.):
        pass

    def parameter_descriptors(self):
        return {}

    def update_parameters(self, parameters):
        self.config.update(parameters)

    def close(self):
        pass


class PredictiveAdapter(ControllerAdapter):
    """Common contract for independent geometric and nonlinear trackers."""
    def __init__(self, tracker):
        self.tracker = tracker
        self.config = tracker.config

    @property
    def error(self):
        return self.tracker.error

    def reset(self, steering=0.):
        self.tracker.reset(steering)

    def step(self, inputs):
        result = self.tracker.step(inputs.pose, inputs.speed, inputs.waypoints,
                                   control_dt=inputs.control_dt, pose_age_s=inputs.pose_age_s)
        return ControllerResult(result.speed, result.steering, result.valid, result.reason,
                                result.steering_rate, dict(self.tracker.last_diagnostics))
