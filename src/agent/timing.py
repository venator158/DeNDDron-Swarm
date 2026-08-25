"""Explicit simulation-time and wall-clock timing for the swarm agent.

Simulation time is used for physical integration.  Monotonic wall time is used
for communication freshness and scheduling.  The class deliberately keeps
sensor-time validation separate from control-step timing so delayed/out-of-
order sensor frames cannot corrupt the controller's integration clock.
"""

from dataclasses import dataclass
from enum import Enum
import math
import time
from typing import Optional


class TimingState(str, Enum):
    FIRST_FRAME = "FIRST_FRAME"
    NORMAL = "NORMAL"
    PAUSED_ZERO_DT = "PAUSED_ZERO_DT"
    OUT_OF_ORDER = "OUT_OF_ORDER"
    TIME_RESET = "TIME_RESET"
    LARGE_DT = "LARGE_DT"
    MISSING_TIME = "MISSING_TIME"


@dataclass(frozen=True)
class TimingSnapshot:
    state: TimingState
    sim_time: Optional[float]
    sim_dt: float
    wall_time: float
    wall_dt: float
    sensor_wall_age: float
    sensor_sim_time: Optional[float]
    sensor_sim_age: Optional[float]

    @property
    def is_sim_time_valid(self) -> bool:
        return self.sim_time is not None and math.isfinite(self.sim_time)


class TimingManager:
    """Own all timing semantics needed by the agent and future consensus code."""

    def __init__(
        self,
        max_control_dt: float = 0.2,
        sensor_timeout_s: float = 0.5,
        control_period_s: float = 0.02,
        reset_threshold_s: float = 0.5,
        clock=time.monotonic,
    ):
        if max_control_dt <= 0:
            raise ValueError("max_control_dt must be > 0")
        if sensor_timeout_s < 0:
            raise ValueError("sensor_timeout_s must be >= 0")
        if control_period_s <= 0:
            raise ValueError("control_period_s must be > 0")
        if reset_threshold_s < 0:
            raise ValueError("reset_threshold_s must be >= 0")

        self.max_control_dt = float(max_control_dt)
        self.sensor_timeout_s = float(sensor_timeout_s)
        self.control_period_s = float(control_period_s)
        self.reset_threshold_s = float(reset_threshold_s)
        self._clock = clock

        self._control_prev_sim_time: Optional[float] = None
        self._last_wall_time: Optional[float] = None
        self._last_sensor_wall_time: Optional[float] = None
        self._last_sensor_sim_time: Optional[float] = None
        self._last_sensor_state = TimingState.MISSING_TIME
        self._pending_reset = False

    @staticmethod
    def _valid_sim_time(sim_time) -> bool:
        return sim_time is not None and isinstance(sim_time, (int, float)) and math.isfinite(float(sim_time))

    def record_sensor(self, sim_time, wall_time: Optional[float] = None) -> tuple[bool, TimingState]:
        """Record a sensor arrival and decide whether its frame is acceptable.

        Returns ``(accepted, state)``.  Backwards frames are rejected unless
        they look like a simulator reset (new time below reset_threshold_s).
        Wall arrival time is always updated because it represents communication
        freshness, independently of simulation progress.
        """
        now = self._clock() if wall_time is None else float(wall_time)
        self._last_sensor_wall_time = now

        if not self._valid_sim_time(sim_time):
            self._last_sensor_state = TimingState.MISSING_TIME
            return True, TimingState.MISSING_TIME

        sim_time = float(sim_time)
        previous = self._last_sensor_sim_time

        if previous is None:
            state = TimingState.FIRST_FRAME
            accepted = True
        elif sim_time < previous:
            if sim_time < self.reset_threshold_s:
                state = TimingState.TIME_RESET
                accepted = True
                # The next control step must establish a new simulation epoch.
                self._control_prev_sim_time = None
                self._pending_reset = True
            else:
                state = TimingState.OUT_OF_ORDER
                accepted = False
        elif sim_time == previous:
            state = TimingState.PAUSED_ZERO_DT
            accepted = True
        else:
            state = TimingState.NORMAL
            accepted = True

        if accepted:
            self._last_sensor_sim_time = sim_time
        self._last_sensor_state = state
        return accepted, state

    def step(self, sim_time, wall_time: Optional[float] = None) -> TimingSnapshot:
        """Advance the control timing state and return an immutable snapshot."""
        now = self._clock() if wall_time is None else float(wall_time)
        if self._last_wall_time is None:
            wall_dt = 0.0
        else:
            wall_dt = max(0.0, now - self._last_wall_time)
        self._last_wall_time = now

        if not self._valid_sim_time(sim_time):
            state = TimingState.MISSING_TIME
            dt = self.control_period_s
            normalized_sim = None
        else:
            normalized_sim = float(sim_time)
            previous = self._control_prev_sim_time

            if self._pending_reset:
                state = TimingState.TIME_RESET
                dt = self.control_period_s
                self._control_prev_sim_time = normalized_sim
                self._pending_reset = False
            elif previous is None:
                state = TimingState.FIRST_FRAME
                dt = self.control_period_s
                self._control_prev_sim_time = normalized_sim
            elif normalized_sim == previous:
                state = TimingState.PAUSED_ZERO_DT
                dt = 0.0
            elif normalized_sim < previous:
                if normalized_sim < self.reset_threshold_s:
                    state = TimingState.TIME_RESET
                    dt = self.control_period_s
                    self._control_prev_sim_time = normalized_sim
                else:
                    state = TimingState.OUT_OF_ORDER
                    dt = 0.0
            else:
                raw_dt = normalized_sim - previous
                self._control_prev_sim_time = normalized_sim
                if raw_dt > self.max_control_dt:
                    state = TimingState.LARGE_DT
                    dt = self.max_control_dt
                else:
                    state = TimingState.NORMAL
                    dt = raw_dt

        sensor_age = self.sensor_wall_age(now)
        sensor_sim_age = self.sensor_sim_age(normalized_sim)
        return TimingSnapshot(
            state=state,
            sim_time=normalized_sim,
            sim_dt=dt,
            wall_time=now,
            wall_dt=wall_dt,
            sensor_wall_age=sensor_age,
            sensor_sim_time=self._last_sensor_sim_time,
            sensor_sim_age=sensor_sim_age,
        )

    def sensor_wall_age(self, wall_time: Optional[float] = None) -> float:
        now = self._clock() if wall_time is None else float(wall_time)
        if self._last_sensor_wall_time is None:
            return float("inf")
        return max(0.0, now - self._last_sensor_wall_time)

    def sensor_sim_age(self, current_sim_time) -> Optional[float]:
        if not self._valid_sim_time(current_sim_time) or self._last_sensor_sim_time is None:
            return None
        age = float(current_sim_time) - self._last_sensor_sim_time
        return max(0.0, age)

    def sensor_is_fresh(self, wall_time: Optional[float] = None) -> bool:
        return self.sensor_wall_age(wall_time) <= self.sensor_timeout_s

    def reset(self) -> None:
        """Clear all timing epochs, primarily for controlled shutdown/restart."""
        self._control_prev_sim_time = None
        self._last_wall_time = None
        self._last_sensor_wall_time = None
        self._last_sensor_sim_time = None
        self._last_sensor_state = TimingState.MISSING_TIME
