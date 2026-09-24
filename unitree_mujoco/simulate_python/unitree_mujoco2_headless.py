import argparse
import sys
import time
from pathlib import Path
import threading
import mujoco

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge
import config

locker = threading.Lock()

mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)

mj_model.opt.timestep = config.SIMULATE_DT
num_motor_ = mj_model.nu
dim_motor_sensor_ = 3 * num_motor_

def SimulationThread():
    global mj_data, mj_model

    print("[Simulator] Initializing ChannelFactory...")
    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    
    print("[Simulator] Initializing UnitreeSdk2Bridge...")
    unitree = UnitreeSdk2Bridge(mj_model, mj_data, data_lock=locker)

    if config.PRINT_SCENE_INFORMATION:
        unitree.PrintSceneInformation()

    print("[Simulator] Headless simulation running at 500 Hz...")
    try:
        while True:
            step_start = time.perf_counter()

            locker.acquire()
            mujoco.mj_step(mj_model, mj_data)
            locker.release()

            time_until_next_step = mj_model.opt.timestep - (
                time.perf_counter() - step_start
            )
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
    except KeyboardInterrupt:
        print("[Simulator] Headless simulation stopped by user.")

if __name__ == "__main__":
    SimulationThread()
