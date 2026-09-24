import argparse
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read Unitree HG LowState over DDS without sending motor commands."
    )
    parser.add_argument(
        "interface",
        nargs="?",
        default=None,
        help="Network interface connected to the robot, for example enp2s0. Omit for default DDS interface.",
    )
    parser.add_argument(
        "--domain",
        type=int,
        default=0,
        help="DDS domain id. Use 0 for real Unitree robot, 1 for local MuJoCo simulator.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for the first LowState before reporting no data.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    latest = {"msg": None, "count": 0, "first_time": None}

    def on_lowstate(msg: LowState_):
        latest["msg"] = msg
        latest["count"] += 1
        if latest["first_time"] is None:
            latest["first_time"] = time.time()
            print("DDS OK: received first rt/lowstate")

    if args.interface:
        ChannelFactoryInitialize(args.domain, args.interface)
        print(f"DDS initialized: domain={args.domain}, interface={args.interface}")
    else:
        ChannelFactoryInitialize(args.domain)
        print(f"DDS initialized: domain={args.domain}, default interface")

    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(on_lowstate, 10)

    start = time.time()
    last_print = 0.0
    while True:
        now = time.time()
        msg = latest["msg"]
        if msg is not None and now - last_print >= 1.0:
            last_print = now
            imu = msg.imu_state
            m0 = msg.motor_state[0]
            print(
                "lowstate "
                f"count={latest['count']} "
                f"tick={msg.tick} "
                f"imu_rpy={[round(v, 4) for v in imu.rpy]} "
                f"motor0(q={m0.q:.4f}, dq={m0.dq:.4f}, tau={m0.tau_est:.4f})"
            )
        elif msg is None and now - start > args.timeout:
            print("No rt/lowstate received. Check interface, robot IP, DDS domain, and firewall.")
            return 1

        time.sleep(0.02)


if __name__ == "__main__":
    raise SystemExit(main())
