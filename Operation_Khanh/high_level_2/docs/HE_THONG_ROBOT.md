# Kiến trúc hệ thống R1 — khảo sát trực tiếp trên robot (2026-07-15)

> Mọi số liệu **đo trực tiếp trên robot đang bật**, bằng công cụ **chỉ nghe DDS / chỉ đọc**
> (không gửi lệnh động cơ, không sửa gì). Không phỏng đoán. Robot lúc khảo sát **đã vào dev mode**.
> Đăng nhập PC2: `ssh unitree@unitree-r1` (hoặc `@100.82.165.36`, hoặc `@192.168.1.33`), mật khẩu `123`.

---

## 0. TÓM TẮT NHANH

- Robot có **2 máy tính** nối qua mạng nội bộ `192.168.123.0/24` (card `eth10`, Realtek GbE).
- **PC2 = 192.168.123.164** — Jetson **Orin NX 16GB**, Ubuntu 20.04, nơi deploy `run_r1`. Vào được bằng SSH.
- **Built-in = 192.168.123.161** — bo điều khiển tầng thấp, Linux headless, **không đăng nhập được** (chỉ DDS/UDP).
- **Luồng động cơ:** bất kỳ ai publish `rt/lowcmd` → **.161 đọc và chạy motor**. Đây là gốc xung đột.
- Có **23 node DDS** trên mạng. Loa/mic đi qua `master_service` (API riêng), KHÔNG qua `rt/lowcmd`.

---

## 1. PC2 — máy phát triển (192.168.123.164, hostname `ubuntu`)

Đây là nơi `run_r1` chạy. Vào được bằng SSH nên khảo sát được đầy đủ.

### 1.1 Phần cứng
| | |
|---|---|
| Board | **NVIDIA Jetson Orin NX Developer Kit** (`t186ref`) |
| SoC | 8× ARMv8 (Cortex, v8l), 115 MHz–1.98 GHz |
| RAM | **15 GiB** (dùng ~1.2 GiB lúc rảnh) |
| Swap | 7.5 GiB — **8× zram** (nén RAM), không dùng ổ |
| Ổ cứng | **NVMe 477 GB**, dùng **6% (23 GB)** — còn 427 GB trống |
| JetPack/L4T | **R35.3.1** (2023-03), kernel `5.10.104-tegra` |
| HĐH | Ubuntu 20.04.5 LTS |

### 1.2 Nhiệt độ (lúc rảnh, robot đứng yên)
CPU **56 °C**, GPU 53 °C, SoC 54–57 °C, tj 57 °C — mát, có `nvfancontrol`. GPU load 0%.

### 1.3 Mạng (4 interface)
| Interface | Driver | Địa chỉ | Vai trò |
|---|---|---|---|
| **`eth10`** | **r8169** (Realtek GbE PCIe) | **192.168.123.164/24** | 🔴 **Mạng điều khiển nội bộ → nối .161**. MAC `4c:bb:47:ab:de:24` |
| `wlan0` | rtl88x2bu (Realtek USB ac) | 192.168.12.2/24 | WiFi, default route qua 192.168.12.1 |
| `tailscale0` | wireguard | 100.82.165.36 | VPN — đường mình vào từ xa |
| `eth0` | — | DOWN | không dùng |

> ⚠ `run_r1` phải chạy trên **`eth10`** (đã đặt trong `tuning.yaml: network_interface: eth10`).
> File `cyclonedds.xml` hệ thống ghi `eth0` nhưng KHÔNG ảnh hưởng vì `run_r1` truyền interface riêng.

### 1.4 USB (giải mã tên)
| Thiết bị | Chip | Driver | Ghi chú |
|---|---|---|---|
| Dual serial | CH343 (1a86) | `usb_ch343` | → `/dev/ttyCH343USB0,1` |
| UART+SPI+I2C+JTAG | CH347 (1a86) | `usb_ch343` | → `/dev/ttyCH343USB2` — cổng debug/nạp |
| USB 10/100 LAN | 1a86 | `cdc_ether` | LAN phụ qua USB (hiện không rõ nối gì) |
| 802.11ac NIC | TP-Link (2357) | — | chính là `wlan0` |

3 cổng `ttyCH343USB*` hiện **không tiến trình nào mở** (rảnh). Là serial/debug phụ, không phải đường motor chính (motor đi qua DDS→.161).

### 1.5 Phần mềm đang chạy
- `master_service` (PID 1145) — **API server của hãng** (xem §3). Chỉ ăn ~17 MB.
- `ota_pipe_service` — cập nhật OTA.
- **NoMachine** (`nxserver/nxnode/nxrunner`) — remote desktop, cổng **4000**.
- `dex3_service` — dịch vụ bàn tay Dex3 (nếu lắp tay).
- **ROS Noetic** + workspace **`unitree_vo`** = **visual odometry/SLAM** (SVO Pro) — robot có thị giác.
- **KHÔNG có** `hb_high_level.service` → **auto-start chưa tồn tại**, `run_r1` không tự chạy.

### 1.6 Âm thanh (liên quan phần voice)
- `card 0 HDA` = **chỉ HDMI** (4 cổng HDMI out).
- `card 1 APE` = Tegra audio (XBAR-ADMAIF).
- PulseAudio có `alsa_output.platform-sound.analog-stereo` (loa) + `alsa_input...analog-stereo` (mic) — **jack analog của board dev**, đang SUSPENDED.

> 🔊 **Loa/mic của ROBOT KHÔNG đi qua ALSA/PulseAudio này.** Chúng đi qua **`a2::AudioClient`
> của Unitree (DDS)** — xem `[[hb-mimic-music-audio]]`. Jack analog trên đây là của board Jetson,
> không phải loa robot. Đừng nhầm khi làm voice.

### 1.7 Deploy `run_r1`
- Ở `~/HB/high_level_2/` — **KHÔNG phải git repo** (chỉ là bản copy). Build lúc `2026-07-15 00:52`.
- Link động: `libonnxruntime.so.1` (thirdparty/onnxruntime_aarch64), `libddsc.so.0` + `libddscxx.so.0` (/usr/local). SDK Unitree link **tĩnh** (`libunitree_sdk2.a`).
- SDK nguồn ở `~/unitree_sdk2` (KHÁC máy dev — máy dev dùng `/opt/unitree_robotics`).

---

## 2. Built-in — bo điều khiển tầng thấp (192.168.123.161)

Máy bạn "không có thông tin". Không đăng nhập được nên khảo sát qua mạng + DDS.

| | |
|---|---|
| HĐH | **Linux** (TTL=64) |
| MAC | `7e:1d:75:60:f5:89` (locally-administered — bo nhúng nội bộ) |
| Kết nối | thẳng vào `eth10` qua switch, ping **0.13 ms** (cực nhanh) |
| TCP | **MỌI cổng ĐÓNG** — quét 30 cổng phổ biến (22/80/443/554/1883/6379/11311…): **không SSH, web, telnet** |
| UDP | DDS mở (7400–7411) — **chỉ nói chuyện bằng DDS** |
| DDS GUID | `01109c07…` |

### Vai trò (suy ra từ luồng DDS — xem §4)
- **Publish (luôn luôn):** `rt/lowstate` (~1000 Hz, khớp + IMU), `rt/lf/bmsstate` (~20 Hz, pin).
- **Subscribe `rt/lowcmd`** → **đây là bên THỰC THI lệnh động cơ.** Ai ghi `rt/lowcmd` thì .161 lái motor theo.
- **Publish `rt/lowcmd` ~621 Hz KHI CHƯA dev mode** → built-in controller. Vào dev mode (L2+R2) thì **ngừng ghi**.

> 🔒 **Vì .161 không đăng nhập được** → KHÔNG thể `systemctl disable` built-in (cách A của
> PLAN_standalone_P0 **bất khả thi**). Cách **duy nhất** làm nó im là **bắt tay dev-mode L2+R2**.
> ⇒ P0-1 **bắt buộc đi cách B (passive-until-armed)**.

---

## 3. `master_service` (chạy trên PC2)

- Binary `/unitree/module/master_service/master_service`, khởi động kiểu **SysV** (`/etc/init.d/master_service`), nên KHÔNG hiện trong `systemctl list-unit-files --enabled`.
- **Là API server tầng cao của hãng**, chở các topic: `rt/api/loco/*`, `rt/api/sport/*`,
  `rt/api/motion_switcher/*`, `rt/api/gpt/*`, `rt/api/voice/*`, `rt/api/vui/*`, `rr/locoReply`,
  `rr/voiceReply`, và publish `rt/arm_sdk` (GUID `01105591c4eb`).
- **Loa/mic/voice/gpt đi qua service NÀY**, tách biệt hoàn toàn `rt/lowcmd` → **tắt built-in
  locomotion không mất loa/mic**.
- Config `master_service.json` **mã hoá nhị phân**, không đọc được.

### Phiên bản phần mềm hãng
```
master_service_pc4 : 1.0.0.2      dex3_service_pc4 : 2.1.0.1
unitree_patch_pc4  : 1.0.0.1  (patch: gỡ unitree-upgrade.service)
```
Hậu tố **`_pc4`** ⇒ trong sơ đồ hãng, máy Jetson này là khối **"PC4"** (bạn quen gọi "PC2").

---

## 4. LUỒNG DỮ LIỆU ĐỘNG CƠ (đo bằng DDS discovery — 23 participant)

Ai ghi / ai đọc các topic điều khiển (GUID rút gọn):

```
        ┌─────────────────────────── rt/lowcmd (lệnh động cơ) ───────────────────────────┐
        │ GHI (publish):                          ĐỌC (subscribe):                        │
        │   • built-in .161  (621Hz, TẮT khi dev)   • .161  guid=01109c07  ← THỰC THI MOTOR │
        │   • run_r1 (khi chạy)                                                             │
        └──────────────────────────────────────────────────────────────────────────────────┘

  rt/lowstate  (khớp+IMU, ~1000Hz)   GHI: .161 (01109c07)   ĐỌC: master_service, VIO, run_r1…
  rt/lf/bmsstate (pin, 20Hz)         GHI: .161 (01109c07)   ĐỌC: ≥4 node (nhiều thứ theo dõi pin)
  rt/arm_sdk   (điều khiển tay)      GHI: master_service (5591c4eb)   ĐỌC: master_service
```

**Điểm mấu chốt:** `rt/lowcmd` **không có "chủ"** — DDS cho **nhiều writer**. `.161` đọc *tất cả*
và lái motor theo. Nên built-in (621 Hz) + `run_r1` cùng ghi = motor nhận **2 luồng mâu thuẫn**
→ giật dữ dội (đúng sự cố đã gặp). **Dev mode = bảo built-in ngừng ghi**, để `run_r1` là writer duy nhất.

---

## 5. XUNG ĐỘT "2 NGUỒN LỆNH" — đo tận số

| Tình huống | `rt/lowcmd` trên mạng | Nguồn |
|---|---|---|
| Mới bật, **CHƯA** dev mode, `run_r1` tắt | **~621 Hz** | built-in (.161) |
| **Giữ L2+R2** vào dev mode | **→ 0 Hz** | built-in đã nhả |
| Bật `run_r1` khi built-in còn 621 Hz | **2 writer cùng ghi** | 💥 xung đột |

---

## 6. NÚT TAY CẦM R3-1 (đo thật)

- Trạng thái nút nằm trong `rt/lowstate` → `wireless_remote[40]`, byte **[2..3]** = bitmask.
- **Giữ L2+R2 → `0x0030`** (bit4=R2, bit5=L2). Đọc được **liên tục 7 giây**, đúng struct trong `GamepadR3.hpp`.
- ✅ Factory/dev mode **CÓ** tuồn nút sang PC2 → **lớp 1 của cổng P0-1 chạy được**.

---

## 7. NGUỒN DỮ LIỆU CHO CODE (đã xác minh chạy)

| Cần | Topic | Kiểu | Tần số | Giá trị đo được |
|---|---|---|---|---|
| Khớp + IMU | `rt/lowstate` | `hg::LowState_` | ~1000 Hz | đang dùng |
| Lệnh động cơ | `rt/lowcmd` | `hg::LowCmd_` | ghi 500 Hz | đang dùng |
| **Pin** | **`rt/lf/bmsstate`** | `hg::BmsState_` | **20 Hz** | soc **90%**, soh **99%**, 36.5 V, dòng −2352 |
| Pin phụ | `rt/lf/secondary_bmsstate` | `BmsState_` | — | chưa dùng |
| Cảnh báo pin (chữ) | `rt/lf/battery_alarm` | `std_msgs/String` | — | text |

*(93 topic tổng: còn `rt/sportmodestate`, `rt/odommodestate`, `rt/arm_sdk`, `rt/frontvideostream`,
`rt/audio_msg`, cụm `rt/api/*` của master_service…)*

---

## 8. RỦI RO / LƯU Ý

- ⚠ **Không backup được .161** — bo khoá, không đăng nhập. Hỏng là mất, ngoài tầm phần mềm.
- ⚠ **Auto-start chưa có** — an toàn hiện tại, nhưng khi bật phải qua cổng P0-1 trước.
- ⚠ **2 default route** (wlan0 metric 600, eth10 metric 20100) — internet đi wlan0, điều khiển đi eth10. Đừng gỡ route eth10.
- ✅ Ổ còn 427 GB, RAM 15 GB — thừa sức chạy run_r1 + VIO + log.

---

## Phụ lục — công cụ khảo sát (chỉ đọc, để lại ở /tmp trên robot)
| File | Việc |
|---|---|
| `/tmp/cmdcount` | đếm tần số `rt/lowcmd` (phát hiện built-in) |
| `/tmp/topics` | liệt kê 93 topic + kiểu dữ liệu |
| `/tmp/bms` | đọc pin `rt/lf/bmsstate` |
| `/tmp/armtest` | kiểm 2 lớp cổng P0-1 (nút + im lặng built-in) |
| `/tmp/whopub`, `/tmp/whosub` | ai publish/subscribe lowcmd/lowstate/bms (+ đếm participant) |

Tất cả **chỉ nghe/chỉ đọc**, chạy lại được. Mã nguồn ở scratchpad phiên làm việc.
