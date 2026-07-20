"""UDP client for communicating with C++ bridge"""
import socket
import struct
import threading
import time
from typing import Optional, Callable
import numpy as np

from .robot_state import RobotState, MotorState
from utils.joint_names import JOINT_IDX


# UDP configuration
UDP_IP = "127.0.0.1"
PORT_SEND = 12346  # Sending commands to C++
PORT_RECV = 12345  # Receiving state from C++

# Struct formats
# C++ BridgeState:
# uint8_t mode_machine;
# uint8_t soc;
# uint8_t bms_status;
# float bms_current;
# float bms_voltage;
# float bms_temp;
# JointState joints[26]; -> each is 8 floats/uint8 (float q, dq, ddq, tau, vol, temp_coil, temp_inv; uint8_t mode)
# float roll, pitch, yaw;
# float gyro_x, gyro_y, gyro_z;
# float acc_x, acc_y, acc_z;
# float quat_w, quat_x, quat_y, quat_z;
# float imu_temp;

STRUCT_STATE_FMT = '<BBB 3f ' + ('7fB' * 26) + ' 14f'

# C++ BridgeCommand: uint8_t motors_enabled; float target_q[26]; float target_kp[26]; float target_kd[26];
STRUCT_CMD_FMT = '<B 78f'

# Default Kp and Kd arrays matching C++ to ensure safe defaults
DEFAULT_KP = [
    200.0, 200.0, 200.0, 200.0, 200.0, 200.0,  # Left leg
    200.0, 200.0, 200.0, 200.0, 200.0, 200.0,  # Right leg
    300.0, 300.0,                              # Waist
    100.0, 100.0, 100.0, 100.0, 50.0,          # Left arm
    100.0, 100.0, 100.0, 100.0, 50.0,          # Right arm
    50.0,  10.0                                # Head
]

DEFAULT_KD = [
    3.0, 3.0, 3.0, 3.0, 3.0, 3.0,  # Left leg
    3.0, 3.0, 3.0, 3.0, 3.0, 3.0,  # Right leg
    5.0, 5.0,                      # Waist
    2.0, 2.0, 2.0, 2.0, 2.0,       # Left arm
    2.0, 2.0, 2.0, 2.0, 2.0,       # Right arm
    2.0, 0.1                       # Head
]


class UDPClient:
    """UDP client for robot communication"""

    def __init__(self, recv_callback: Optional[Callable] = None):
        self.recv_callback = recv_callback
        self.running = False

        # State data
        self.robot_state: Optional[RobotState] = None
        self.target_q: Optional[np.ndarray] = None
        self.target_kp: np.ndarray = np.array(DEFAULT_KP, dtype=np.float32)
        self.target_kd: np.ndarray = np.array(DEFAULT_KD, dtype=np.float32)
        self.motors_enabled = False
        self.packet_count = 0

        # Locks
        self.state_lock = threading.Lock()
        self.cmd_lock = threading.Lock()

        # Sockets
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_recv.settimeout(0.1)

        # Threads
        self.recv_thread: Optional[threading.Thread] = None
        self.send_thread: Optional[threading.Thread] = None

    def start(self):
        """Start the UDP client"""
        if self.running:
            return

        try:
            self.sock_recv.bind((UDP_IP, PORT_RECV))
        except OSError as e:
            raise RuntimeError(
                f"Failed to bind to UDP port {PORT_RECV}: {e}.\n"
                f"Another Python GUI process may already be running. "
                f"Please check and terminate any running python3 processes."
            )

        self.running = True

        # Start receive thread
        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.recv_thread.start()

        # Start send thread
        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.send_thread.start()

        print(f"UDP Client started on {UDP_IP}:{PORT_RECV}")

    def stop(self):
        """Stop the UDP client"""
        self.running = False

        if self.send_thread:
            self.send_thread.join(timeout=1.0)
        if self.recv_thread:
            self.recv_thread.join(timeout=1.0)

        self.sock_send.close()
        self.sock_recv.close()

        print("UDP Client stopped")

    def _recv_loop(self):
        """Receive loop - runs in separate thread"""
        expected_size = struct.calcsize(STRUCT_STATE_FMT)
        while self.running:
            try:
                data, addr = self.sock_recv.recvfrom(2048)
                if len(data) >= expected_size:
                    self._parse_state(data[:expected_size])
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"Recv error: {e}")

    def _parse_state(self, data: bytes):
        """Parse state data from C++ bridge"""
        unpacked = struct.unpack(STRUCT_STATE_FMT, data)
        mode_machine = unpacked[0]
        soc = unpacked[1]
        bms_status = unpacked[2]
        bms_current = unpacked[3]
        bms_voltage = unpacked[4]
        bms_temp = unpacked[5]

        offset = 6
        motor_states = []
        for i in range(26):
            q = unpacked[offset]
            dq = unpacked[offset + 1]
            ddq = unpacked[offset + 2]
            tau = unpacked[offset + 3]
            vol = unpacked[offset + 4]
            temp_coil = unpacked[offset + 5]
            temp_inv = unpacked[offset + 6]
            mode = unpacked[offset + 7]
            
            # Using existing fields, maybe monkey-patching new ones since RobotState/MotorState class might need updates
            ms = MotorState(q=q, dq=dq, tau=tau, temp=temp_coil)
            ms.ddq = ddq
            ms.vol = vol
            ms.temp_inv = temp_inv
            ms.mode = mode
            motor_states.append(ms)
            offset += 8

        # IMU Data
        roll, pitch, yaw = unpacked[offset], unpacked[offset+1], unpacked[offset+2]
        gyro_x, gyro_y, gyro_z = unpacked[offset+3], unpacked[offset+4], unpacked[offset+5]
        acc_x, acc_y, acc_z = unpacked[offset+6], unpacked[offset+7], unpacked[offset+8]
        quat_w, quat_x, quat_y, quat_z = unpacked[offset+9], unpacked[offset+10], unpacked[offset+11], unpacked[offset+12]
        imu_temp = unpacked[offset+13]

        # Fill in empty motors for indices not in our 26
        all_motors = [MotorState() for _ in range(35)]
        for i, motor in enumerate(motor_states):
            all_motors[JOINT_IDX[i]] = motor

        state = RobotState(
            mode_machine=mode_machine,
            soc=soc,
            motor_states=all_motors,
            imu_roll=roll,
            imu_pitch=pitch,
            imu_yaw=yaw,
            imu_gyro=[gyro_x, gyro_y, gyro_z],
            imu_accel=[acc_x, acc_y, acc_z],
            timestamp=time.time()
        )
        
        # Monkey patch new bms/imu fields into state
        state.bms_current = bms_current
        state.bms_voltage = bms_voltage
        state.bms_temp = bms_temp
        state.imu_quat = [quat_w, quat_x, quat_y, quat_z]
        state.imu_temp = imu_temp

        with self.state_lock:
            self.robot_state = state
            self.packet_count += 1
            # Initialize target_q on first state
            if self.target_q is None:
                self.target_q = np.array([m.q for m in motor_states])

        # Call callback if set
        if self.recv_callback:
            self.recv_callback(state)

    def _send_loop(self):
        """Send loop - runs in separate thread at ~200Hz"""
        while self.running:
            if self.target_q is not None:
                with self.cmd_lock:
                    enabled_val = 1 if self.motors_enabled else 0
                    
                    # Pack 1 byte + 26 target_q + 26 target_kp + 26 target_kd
                    cmd_data = [enabled_val]
                    cmd_data.extend(self.target_q.tolist())
                    cmd_data.extend(self.target_kp.tolist())
                    cmd_data.extend(self.target_kd.tolist())
                    
                    data = struct.pack(STRUCT_CMD_FMT, *cmd_data)
                    
                try:
                    self.sock_send.sendto(data, (UDP_IP, PORT_SEND))
                except Exception:
                    pass
            time.sleep(0.005)  # 200 Hz

    def set_target_angle(self, joint_idx: int, angle_rad: float):
        """Set target angle for a joint"""
        if self.target_q is not None:
            with self.cmd_lock:
                self.target_q[joint_idx] = angle_rad

    def set_target_kp(self, joint_idx: int, kp: float):
        """Set target kp for a joint"""
        with self.cmd_lock:
            self.target_kp[joint_idx] = kp

    def set_target_kd(self, joint_idx: int, kd: float):
        """Set target kd for a joint"""
        with self.cmd_lock:
            self.target_kd[joint_idx] = kd

    def set_all_target_angles(self, angles: np.ndarray):
        """Set all target angles"""
        with self.cmd_lock:
            self.target_q = angles.copy() if angles is not None else None

    def enable_motors(self):
        """Enable motors (snap to current position first)"""
        with self.state_lock:
            if self.robot_state:
                # Snap to current angles
                motor_states = self.robot_state.motor_states
                new_targets = []
                for i in range(26):
                    new_targets.append(motor_states[JOINT_IDX[i]].q)
                with self.cmd_lock:
                    self.target_q = np.array(new_targets)
                self.motors_enabled = True
                return True
        return False

    def disable_motors(self):
        """Disable all motors"""
        self.motors_enabled = False

    def get_state(self) -> Optional[RobotState]:
        """Get current robot state (thread-safe)"""
        with self.state_lock:
            return self.robot_state

    @property
    def is_connected(self) -> bool:
        """Check if receiving data"""
        with self.state_lock:
            return self.robot_state is not None
