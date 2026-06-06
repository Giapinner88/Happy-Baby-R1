#!/usr/bin/env bash
set -euo pipefail

# DDS WiFi compatibility test endpoint for the Ubuntu 22.04 laptop.
# Run this at the same time as test/dds_wifi_workstation_ubuntu20.sh on the workstation.

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
DURATION_SEC="${DURATION_SEC:-60}"
WIFI_INTERFACE="${WIFI_INTERFACE:-}"
CYCLONEDDS_TMP="${CYCLONEDDS_TMP:-/tmp/hb_cyclonedds_wifi_ubuntu22.xml}"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup file not found: $ROS_SETUP" >&2
  echo "Install/source ROS 2 Humble or set ROS_SETUP=/path/to/setup.bash" >&2
  exit 2
fi

if [[ -z "$WIFI_INTERFACE" ]]; then
  WIFI_INTERFACE="$(ip route get 1.1.1.1 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i == "dev") {print $(i+1); exit}}')"
fi

if [[ -z "$WIFI_INTERFACE" ]]; then
  echo "Could not auto-detect WiFi/default interface. Set WIFI_INTERFACE=wlan0 or similar." >&2
  exit 2
fi

cat >"$CYCLONEDDS_TMP" <<XML
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <NetworkInterfaceAddress>${WIFI_INTERFACE}</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
XML

source "$ROS_SETUP"
export ROS_DOMAIN_ID
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${CYCLONEDDS_TMP}"
export HB_DDS_ROLE="ubuntu22_laptop"
export HB_DDS_PUB_TOPIC="/hb/dds_wifi/ubuntu22_to_ubuntu20"
export HB_DDS_SUB_TOPIC="/hb/dds_wifi/ubuntu20_to_ubuntu22"
export HB_DDS_DURATION_SEC="$DURATION_SEC"

echo "=== Happy Baby DDS WiFi endpoint: Ubuntu 22.04 laptop ==="
echo "ROS_DISTRO=${ROS_DISTRO:-unknown}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "WIFI_INTERFACE=${WIFI_INTERFACE}"
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI}"
echo "Publish: ${HB_DDS_PUB_TOPIC}"
echo "Listen:  ${HB_DDS_SUB_TOPIC}"
echo

python3 - <<'PY'
import os
import socket
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class WifiDdsProbe(Node):
    def __init__(self) -> None:
        self.role = os.environ["HB_DDS_ROLE"]
        self.pub_topic = os.environ["HB_DDS_PUB_TOPIC"]
        self.sub_topic = os.environ["HB_DDS_SUB_TOPIC"]
        self.duration_sec = float(os.environ["HB_DDS_DURATION_SEC"])
        super().__init__(f"hb_dds_wifi_{self.role}")
        self.publisher = self.create_publisher(String, self.pub_topic, 10)
        self.subscription = self.create_subscription(String, self.sub_topic, self.on_msg, 10)
        self.start = time.monotonic()
        self.sent = 0
        self.received = 0
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_msg(self, msg: String) -> None:
        self.received += 1
        self.get_logger().info(f"RX {self.received}: {msg.data}")

    def on_timer(self) -> None:
        self.sent += 1
        msg = String()
        msg.data = (
            f"from={self.role} host={socket.gethostname()} seq={self.sent} "
            f"t={time.time():.3f}"
        )
        self.publisher.publish(msg)
        self.get_logger().info(f"TX {self.sent}: {msg.data}")


def main() -> int:
    rclpy.init()
    node = WifiDdsProbe()
    deadline = time.monotonic() + node.duration_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        sent = node.sent
        received = node.received
        node.destroy_node()
        rclpy.shutdown()

    print(f"SUMMARY role=ubuntu22_laptop sent={sent} received={received}")
    return 0 if received > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
PY
