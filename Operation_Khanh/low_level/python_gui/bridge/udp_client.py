"""UDP client for communicating with C++ bridge"""
import socket
import struct
import threading
import time
from typing import Optional, Callable
import numpy as np

from .robot_state import RobotState, MotorState
from utils.joint_names import JOINT_IDX

UDP_IP = "127.0.0.1"
PORT_SEND = 12346
PORT_RECV = 12345

STRUCT_STATE_FMT = '<BBB 3f ' + ('7fB' * 26) + ' 14f'
STRUCT_CMD_FMT = '<B 78f'

DEFAULT_KP = [
    200.0, 200.0, 200.0, 200.0, 200.0, 200.0,
    200.0, 200.0, 200.0, 200.0, 200.0, 200.0,
    300.0, 300.0,
    100.0, 100.0, 100.0, 100.0, 50.0,
    100.0, 100.0, 100.0, 100.0, 50.0,
    50.0,  10.0
]

DEFAULT_KD = [
    3.0, 3.0, 3.0, 3.0, 3.0, 3.0,
    3.0, 3.0, 3.0, 3.0, 3.0, 3.0,
    5.0, 5.0,
    2.0, 2.0, 2.0, 2.0, 2.0,
    2.0, 2.0, 2.0, 2.0, 2.0,
    2.0, 0.1
]


class UDPClient:
    def __init__(self, recv_callback: Optional[Callable] = None):
        self.recv_callback = recv_callback
        self.running = False

        self.robot_state: Optional[RobotState] = None
        self.target_q: Optional[np.ndarray] = None
        self.target_kp: np.ndarray = np.array(DEFAULT_KP, dtype=np.float32)
        self.target_kd: np.ndarray = np.array(DEFAULT_KD, dtype=np.float32)
        self.motors_enabled = False
        self.packet_count = 0

        self.state_lock = threading.Lock()
        self.cmd_lock = threading.Lock()

        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_recv.settimeout(0.1)

        self.recv_thread: Optional[threading.Thread] = None
        self.send_thread: Optional[threading.Thread] = None

    def start(self):
        if self.running:
            return

        try:
            self.sock_recv.bind((UDP_IP, PORT_RECV))
        except OSError as e:
            raise RuntimeError(
                f"Failed to bind to UDP port {PORT_RECV}: {e}.\n"
                f"Another Python GUI process may already be running."
            )

        self.running = True

        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.recv_thread.start()

        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.send_thread.start()

        print(f"UDP Client started on {UDP_IP}:{PORT_RECV}")

    def stop(self):
        self.running = False

        if self.send_thread:
            self.send_thread.join(timeout=1.0)
        if self.recv_thread:
            self.recv_thread.join(timeout=1.0)

        self.sock_send.close()
        self.sock_recv.close()

        print("UDP Client stopped")

    def _recv_loop(self):
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
            
            ms = MotorState(q=q, dq=dq, tau=tau, temp=temp_coil)
            ms.ddq = ddq
            ms.vol = vol
            ms.temp_inv = temp_inv
            ms.mode = mode
            motor_states.append(ms)
            offset += 8

        roll, pitch, yaw = unpacked[offset], unpacked[offset+1], unpacked[offset+2]
        gyro_x, gyro_y, gyro_z = unpacked[offset+3], unpacked[offset+4], unpacked[offset+5]
        acc_x, acc_y, acc_z = unpacked[offset+6], unpacked[offset+7], unpacked[offset+8]
        quat_w, quat_x, quat_y, quat_z = unpacked[offset+9], unpacked[offset+10], unpacked[offset+11], unpacked[offset+12]
        imu_temp = unpacked[offset+13]

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
        
        state.bms_current = bms_current
        state.bms_voltage = bms_voltage
        state.bms_temp = bms_temp
        state.imu_quat = [quat_w, quat_x, quat_y, quat_z]
        state.imu_temp = imu_temp

        with self.state_lock:
            self.robot_state = state
            self.packet_count += 1
            if self.target_q is None:
                self.target_q = np.array([m.q for m in motor_states])

        if self.recv_callback:
            self.recv_callback(state)

    def _send_loop(self):
        while self.running:
            if self.target_q is not None:
                with self.cmd_lock:
                    enabled_val = 1 if self.motors_enabled else 0
                    
                    cmd_data = [enabled_val]
                    cmd_data.extend(self.target_q.tolist())
                    cmd_data.extend(self.target_kp.tolist())
                    cmd_data.extend(self.target_kd.tolist())
                    
                    data = struct.pack(STRUCT_CMD_FMT, *cmd_data)
                    
                try:
                    self.sock_send.sendto(data, (UDP_IP, PORT_SEND))
                except Exception:
                    pass
            time.sleep(0.005)

    def set_target_angle(self, joint_idx: int, angle_rad: float):
        if self.target_q is not None:
            with self.cmd_lock:
                self.target_q[joint_idx] = angle_rad

    def set_target_kp(self, joint_idx: int, kp: float):
        with self.cmd_lock:
            self.target_kp[joint_idx] = kp

    def set_target_kd(self, joint_idx: int, kd: float):
        with self.cmd_lock:
            self.target_kd[joint_idx] = kd

    def set_all_target_angles(self, angles: np.ndarray):
        with self.cmd_lock:
            self.target_q = angles.copy() if angles is not None else None

    def enable_motors(self):
        with self.state_lock:
            if self.robot_state:
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
        self.motors_enabled = False

    def get_state(self) -> Optional[RobotState]:
        with self.state_lock:
            return self.robot_state

    @property
    def is_connected(self) -> bool:
        with self.state_lock:
            return self.robot_state is not None
