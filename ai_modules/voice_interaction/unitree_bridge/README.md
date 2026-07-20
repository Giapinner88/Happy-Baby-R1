# Unitree R1 Bridge

Small SDK2 bridge programs for connecting `voice_r1` to Robot Hanh Phuc R1 hardware.

This bridge deliberately uses only high-level R1 services:

- `voice`: volume, LED, built-in TTS, speaker PCM stream, ASR text topic.
- `sport`: safe high-level state commands such as `stand_up`, `stop_move`, `damp`.

It does not publish low-level motor commands. That keeps it from fighting the robot's
motion controller while the voice app is running.

## Build On The Robot

```bash
cd /home/unitree/HappyBaby/ai_modules/voice_interaction/unitree_bridge
mkdir -p build
cd build
cmake .. -DUNITREE_SDK2_ROOT=/home/unitree/unitree_sdk2-main
make -j$(nproc)
```

If CMake cannot find `unitree_sdk2`, install or build the SDK first from the
robot's `unitree_sdk2-main` folder:

```bash
cd /home/unitree/unitree_sdk2-main
mkdir -p build
cd build
cmake .. -DUNITREE_SDK2_ROOT=/home/unitree/unitree_sdk2-main
make -j$(nproc)
sudo make install
```

## Find Network Interface

```bash
ip -br a
```

Use the interface whose IP is in the robot DDS/audio network. The R1 audio
example looks for `192.168.123.x` for microphone multicast.

## Test Speaker

```bash
./r1_bridge tts eth0 "Xin chao, toi la Robot Hanh Phuc R1" 1
./r1_bridge volume eth0 90
```

Play a raw 16 kHz mono 16-bit PCM stream from stdin:

```bash
ffmpeg -i test.wav -f s16le -acodec pcm_s16le -ac 1 -ar 16000 - \
  | ./r1_bridge speaker eth0 pipecat
```

## Test Microphone

Write 5 seconds from the R1 microphone multicast into a WAV file:

```bash
./r1_bridge mic eth0 5 > /tmp/r1_mic.raw
ffmpeg -f s16le -ar 16000 -ac 1 -i /tmp/r1_mic.raw /tmp/r1_mic.wav
aplay /tmp/r1_mic.wav
```

Subscribe to the robot's built-in ASR topic:

```bash
./r1_bridge asr eth0
```

## Test High-Level Actions

```bash
./r1_bridge action eth0 stand_up
./r1_bridge action eth0 stop_move
./r1_bridge action eth0 damp
```

Avoid running low-level SDK examples while this bridge or another motion service
is controlling the robot.
