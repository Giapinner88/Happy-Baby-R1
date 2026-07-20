"""Robot state data structures"""
from dataclasses import dataclass, field
from typing import List
import math


@dataclass
class MotorState:
    """State of a single motor"""
    q: float = 0.0       # Position (radians)
    dq: float = 0.0      # Velocity (rad/s)
    ddq: float = 0.0     # Acceleration (rad/s^2)
    tau: float = 0.0     # Torque (Nm)
    vol: float = 0.0     # Voltage (V)
    temp: float = 0.0    # Coil Temperature (Celsius)
    temp_inv: float = 0.0# Inverter Temperature (Celsius)
    mode: int = 0        # Internal Mode


@dataclass
class RobotState:
    """Complete state of the robot"""
    mode_machine: int = 0
    soc: int = 100
    bms_current: float = 0.0
    bms_voltage: float = 0.0
    bms_temp: float = 0.0
    motor_states: List[MotorState] = field(default_factory=lambda: [MotorState() for _ in range(35)])

    # IMU data
    imu_roll: float = 0.0
    imu_pitch: float = 0.0
    imu_yaw: float = 0.0
    imu_gyro: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    imu_accel: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    imu_quat: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    imu_temp: float = 0.0

    # Timestamps
    timestamp: float = 0.0

    def get_joint_angle_deg(self, sdk_idx: int) -> float:
        """Get joint angle in degrees"""
        return math.degrees(self.motor_states[sdk_idx].q)

    def get_joint_velocity(self, sdk_idx: int) -> float:
        """Get joint velocity"""
        return self.motor_states[sdk_idx].dq

    def get_joint_torque(self, sdk_idx: int) -> float:
        """Get joint torque"""
        return self.motor_states[sdk_idx].tau

    def get_joint_temp(self, sdk_idx: int) -> float:
        """Get joint temperature (coil)"""
        return self.motor_states[sdk_idx].temp

    @property
    def imu_roll_deg(self) -> float:
        return math.degrees(self.imu_roll)

    @property
    def imu_pitch_deg(self) -> float:
        return math.degrees(self.imu_pitch)

    @property
    def imu_yaw_deg(self) -> float:
        return math.degrees(self.imu_yaw)
