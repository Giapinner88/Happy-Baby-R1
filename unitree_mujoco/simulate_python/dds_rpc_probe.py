import argparse

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.r1.audio.r1_audio_client import AudioClient
from unitree_sdk2py.r1.loco.r1_loco_client import LocoClient
from unitree_sdk2py.r1.loco.r1_loco_api import (
    ROBOT_API_ID_LOCO_GET_BALANCE_MODE,
    ROBOT_API_ID_LOCO_GET_FSM_ID,
    ROBOT_API_ID_LOCO_GET_STAND_HEIGHT,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Probe Unitree R1 DDS RPC services without sending motion commands."
    )
    parser.add_argument(
        "interface",
        nargs="?",
        default=None,
        help="Network interface connected to the robot, for example wlp111s0.",
    )
    parser.add_argument(
        "--domain",
        type=int,
        default=0,
        help="DDS domain id. Use 0 for the real robot.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=3.0,
        help="RPC timeout in seconds.",
    )
    return parser.parse_args()


def print_result(name, result):
    print(f"{name}: {result}")


def call_get(client, api_id):
    code, data = client._Call(api_id, "{}")
    return code, data


def main():
    args = parse_args()
    if args.interface:
        ChannelFactoryInitialize(args.domain, args.interface)
        print(f"DDS initialized: domain={args.domain}, interface={args.interface}")
    else:
        ChannelFactoryInitialize(args.domain)
        print(f"DDS initialized: domain={args.domain}, default interface")

    audio = AudioClient()
    audio.SetTimeout(args.timeout)
    audio.Init()
    print_result("voice.GetVolume", audio.GetVolume())

    loco = LocoClient()
    loco.SetTimeout(args.timeout)
    loco.Init()
    print_result("sport.GetFsmId", call_get(loco, ROBOT_API_ID_LOCO_GET_FSM_ID))
    print_result(
        "sport.GetBalanceMode",
        call_get(loco, ROBOT_API_ID_LOCO_GET_BALANCE_MODE),
    )
    print_result(
        "sport.GetStandHeight",
        call_get(loco, ROBOT_API_ID_LOCO_GET_STAND_HEIGHT),
    )


if __name__ == "__main__":
    raise SystemExit(main())
