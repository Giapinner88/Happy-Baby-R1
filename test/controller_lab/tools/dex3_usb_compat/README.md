# Dex3 USB interface compatibility test

This is a reversible compatibility test for the exact R1 installation observed
on 2026-08-18. It does not publish `HandCmd` and does not move the hands.

## Evidence

- Dex3 left is `34b7:1010`; Dex3 right is `34b7:1011`.
- Both devices expose bulk endpoints `0x01` and `0x81` on USB interface 1.
- `dex3_service_pc4` version `2.1.0.1` has service binary SHA256
  `b5b62f1aa45b76ea53b2b8dc8c86afc93cf06f41e93934600ae4f0dfb7adf50f`.
- Static disassembly shows that `LinuxSendMcu::init(int)` calls both
  `libusb_detach_kernel_driver(handle, 0)` and
  `libusb_claim_interface(handle, 0)`.
- The kernel reports `Dex3 Thread did not claim interface 1 before use` for
  both hands.
- DDS topic/type/QoS checks pass, but all four state topics deliver zero
  samples.

The preload library preserves the vendor binary, additionally claims USB
interface 1, and assembles a requested 1270-byte input frame across at most
four successful short bulk reads. The vendor service still checks the original
frame header and CRC before publishing state. The installer backs up both
vendor startup scripts, restarts only the two Dex3 services, and runs read-only
state probes. It automatically rolls back if either hand still produces no
valid state.

## Robot-side execution

The folder is staged at `/home/unitree/dex3_usb_compat` on the robot.

```bash
cd /home/unitree/dex3_usb_compat
./build_on_robot.sh
sudo ./install_and_test.sh
```

If the test succeeds, uninstall later with:

```bash
sudo /home/unitree/dex3_usb_compat/uninstall.sh
```

Do not run Unitree's interactive Dex3 example for validation because it creates
a command publisher and can move the finger motors.

`test_disable_port9.sh` is a separate reversible diagnostic for the observed
unmounted `1a86:80b5` USB card reader at hub port `1-1.9`. The kernel repeatedly
reports descriptor error `-71` and power-cycles that device. The script refuses
to continue if the USB identity differs or `/dev/sda` is mounted. It restores
the port and restarts the original Dex3 services automatically unless both
read-only hand-state probes pass.
