import sys
import os
import time
import math
import socket
import struct
import threading
import numpy as np
import pygame

class Theme:
    BG_MAIN = (11, 14, 20)
    BG_PANEL = (19, 26, 36)
    BDR_DEFAULT = (35, 43, 58)
    BDR_SELECTED = (242, 169, 59)
    TXT_TITLE = (230, 234, 242)
    TXT_LABEL = (107, 118, 134)
    VAL_REAL = (95, 227, 139)
    VAL_TGT = (79, 195, 242)
    ALERT = (242, 85, 90)
    ACCENT = (242, 169, 59)
    BDR_ALERT = (193, 58, 62)
    BDR_ACCENT_DK = (196, 133, 15)

def lighten(color, amt):
    return tuple(min(255, int(c + (255 - c) * amt)) for c in color)

def darken(color, amt):
    return tuple(max(0, int(c * (1 - amt))) for c in color)

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

JOINT_NAMES = [
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "waist_roll", "waist_yaw",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow", "left_wrist_roll",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow", "right_wrist_roll",
    "head_pitch", "head_yaw"
]

JOINT_IDX = [
    0, 1, 2, 3, 4, 5,
    6, 7, 8, 9, 10, 11,
    12, 13,
    15, 16, 17, 18, 19,
    22, 23, 24, 25, 26,
    29, 30
]

SAFE_LIMITS = [
    [-152.0, 130.0],
    [-52.0, 92.0],
    [-141.0, 141.0],
    [-2.0, 131.0],
    [-45.0, 28.0],
    [-13.0, 13.0],
    [-152.0, 130.0],
    [-92.0, 52.0],
    [-141.0, 141.0],
    [-2.0, 131.0],
    [-45.0, 28.0],
    [-13.0, 13.0],
    [-27.0, 27.0],
    [-135.0, 135.0],
    [-165.0, 105.0],
    [5.0, 134.0],
    [-99.0, 99.0],
    [-46.0, 116.0],
    [-99.0, 99.0],
    [-165.0, 105.0],
    [-134.0, -5.0],
    [-99.0, 99.0],
    [-46.0, 116.0],
    [-99.0, 99.0],
    [-20.0, 20.0],
    [-34.0, 34.0]
]

robot_state = None
state_lock = threading.Lock()
cmd_lock = threading.Lock()
motors_enabled = False
target_q = None
running = True

UDP_IP = "127.0.0.1"
PORT_SEND = 12346
PORT_RECV = 12345

STRUCT_STATE_FMT = '<B104f6f'
STRUCT_CMD_FMT = '<B26f'

sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_recv.bind((UDP_IP, PORT_RECV))
sock_recv.settimeout(0.1)

class DummyMotorState:
    def __init__(self, q_val, dq_val=0.0, tau_val=0.0, temp_val=0.0):
        self.q = q_val
        self.dq = dq_val
        self.tau = tau_val
        self.temp = temp_val

class DummyState:
    def __init__(self, mode, joint_data, imu_data):
        self.mode_machine = mode
        self.motor_state = [DummyMotorState(0.0)] * 35
        for i in range(26):
            q, dq, tau, temp = joint_data[i]
            self.motor_state[JOINT_IDX[i]] = DummyMotorState(q, dq, tau, temp)
        self.imu_roll = imu_data[0]
        self.imu_pitch = imu_data[1]
        self.imu_yaw = imu_data[2]
        self.imu_gyro = imu_data[3:6]

def udp_receiver_loop():
    global robot_state, target_q, running
    while running:
        try:
            data, addr = sock_recv.recvfrom(2048)
            if len(data) == 441:
                mode_val, *floats = struct.unpack(STRUCT_STATE_FMT, data)
                joint_data = []
                for i in range(26):
                    offset = i * 4
                    q = floats[offset]
                    dq = floats[offset+1]
                    tau = floats[offset+2]
                    temp = floats[offset+3]
                    joint_data.append((q, dq, tau, temp))
                imu_data = floats[104:110]
                with state_lock:
                    robot_state = DummyState(mode_val, joint_data, imu_data)
                    if target_q is None:
                        target_q = [jd[0] for jd in joint_data]
                        print(">>> Synced targets to physical joint angles.")
        except socket.timeout:
            continue
        except Exception as e:
            print(f"Error receiving UDP: {e}")

def udp_publisher_loop():
    global target_q, motors_enabled, running
    while running:
        if target_q is not None:
            with cmd_lock:
                enabled_val = 1 if motors_enabled else 0
                data = struct.pack(STRUCT_CMD_FMT, enabled_val, *target_q)
            try:
                sock_send.sendto(data, (UDP_IP, PORT_SEND))
            except Exception as e:
                pass
        time.sleep(0.005)

REFERENCE_COL_W = 245.0
FONT_MIN_SCALE = 0.62
FONT_MAX_SCALE = 1.25

BASE_SIZES = {
    "joint_name": 14,
    "label": 12,
    "mono_bold": 13,
    "mono": 12,
}

UI_SIZES = {
    "status": 16,
    "header": 14,
    "small": 13,
    "button_title": 18,
    "button_sub": 13,
}

def build_ui_fonts():
    return {
        "status": pygame.font.SysFont("Arial", UI_SIZES["status"], bold=True),
        "header": pygame.font.SysFont("Arial", UI_SIZES["header"], bold=True),
        "small": pygame.font.SysFont("Arial", UI_SIZES["small"]),
        "button_title": pygame.font.SysFont("Arial", UI_SIZES["button_title"], bold=True),
        "button_sub": pygame.font.SysFont("Arial", UI_SIZES["button_sub"], bold=True),
    }

def build_fonts(scale):
    def s(base):
        return max(9, int(round(base * scale)))
    return {
        "joint_name": pygame.font.SysFont("Arial", s(BASE_SIZES["joint_name"]), bold=True),
        "label": pygame.font.SysFont("Arial", s(BASE_SIZES["label"])),
        "mono_bold": pygame.font.SysFont("Courier New", s(BASE_SIZES["mono_bold"]), bold=True),
        "mono": pygame.font.SysFont("Courier New", s(BASE_SIZES["mono"]), bold=True),
    }

def draw_button(screen, rect, base_color, border_color, label_lines, label_colors, fonts, hover=False, pressed=False):
    fill = lighten(base_color, 0.12) if hover else base_color
    border = lighten(border_color, 0.25) if hover else border_color
    border_w = 4 if pressed else 2

    pygame.draw.rect(screen, fill, rect, border_radius=8)
    pygame.draw.rect(screen, border, rect, width=border_w, border_radius=8)

    tri_cx, tri_cy = rect.right - 18, rect.top + 16
    tri_color = darken(fill, 0.35)
    pygame.draw.polygon(
        screen, tri_color,
        [(tri_cx - 5, tri_cy - 6), (tri_cx - 5, tri_cy + 6), (tri_cx + 6, tri_cy)]
    )

    line_fonts = [fonts["button_title"], fonts["button_sub"]]
    line_hs = [line_fonts[i].get_height() for i in range(len(label_lines))]
    total_h = sum(line_hs) + 4 * (len(label_lines) - 1)
    y_cursor = rect.centery - total_h // 2 + line_hs[0] // 2
    for i, text in enumerate(label_lines):
        font = line_fonts[i]
        color = label_colors[i]
        lbl = font.render(text, True, color)
        lbl_rect = lbl.get_rect(center=(rect.centerx, y_cursor))
        screen.blit(lbl, lbl_rect)
        y_cursor += line_hs[i] // 2 + (line_hs[i + 1] // 2 if i + 1 < len(line_hs) else 0) + 4

def main():
    global motors_enabled, target_q, running

    interface = "eno1"
    if len(sys.argv) >= 2:
        interface = sys.argv[1]

    print(f"📡 Connecting to C++ bridge...")
    print(f">>> Network interface: {interface}")

    pub_thread = threading.Thread(target=udp_publisher_loop, daemon=True)
    pub_thread.start()

    recv_thread = threading.Thread(target=udp_receiver_loop, daemon=True)
    recv_thread.start()

    pygame.init()
    screen_w, screen_h = 1600, 1200
    screen = pygame.display.set_mode((screen_w, screen_h), pygame.RESIZABLE)
    pygame.display.set_caption("R1 Low-Level Joint Tuning GUI")
    clock = clock = pygame.time.Clock()

    fonts = build_fonts(1.0)
    last_font_col_w = None
    fonts_ui = build_ui_fonts()

    groups = [
        {"name": "LEFT LEG", "joints": list(range(0, 6))},
        {"name": "RIGHT LEG", "joints": list(range(6, 12))},
        {"name": "WAIST & HEAD", "joints": [12, 13, 24, 25]},
        {"name": "LEFT ARM", "joints": list(range(14, 19))},
        {"name": "RIGHT ARM", "joints": list(range(19, 24))}
    ]

    current_w, current_h = screen_w, screen_h

    selected_joint = 24
    step_size = 3.0
    speed_dps = 5.0

    btn_steps = [1.0, 3.0, 5.0, 7.0]
    btn_speeds = [3.0, 5.0, 10.0, 15.0]

    while running:
        dt = clock.tick(60) / 1000.0

        curr_q_deg = [0.0] * 26
        with state_lock:
            if robot_state is not None:
                for i in range(26):
                    sdk_idx = JOINT_IDX[i]
                    curr_q_deg[i] = math.degrees(robot_state.motor_state[sdk_idx].q)

        margin_x = 20
        margin_y = 15
        spacing_x = 15
        spacing_y = 10
        header_font_h = fonts_ui["header"].get_height()
        bar_bottom = 50
        header_gap_top = 12
        header_gap_bottom = 10
        grid_y_start = bar_bottom + header_gap_top + header_font_h + header_gap_bottom
        grid_y_end = current_h - 190
        grid_h = grid_y_end - grid_y_start
        col_w = (current_w - 2 * margin_x - 5 * spacing_x) / 6.0
        card_h = (grid_h - 5 * spacing_y) / 6.0

        rounded_col_w = round(col_w)
        if rounded_col_w != last_font_col_w:
            scale = clamp(col_w / REFERENCE_COL_W, FONT_MIN_SCALE, FONT_MAX_SCALE)
            fonts = build_fonts(scale)
            last_font_col_w = rounded_col_w

        font_sans_bold = fonts["joint_name"]
        font_sans_normal = fonts["label"]
        font_mono_bold = fonts["mono_bold"]
        font_mono_normal = fonts["mono"]

        _speed_title_w = fonts_ui["header"].size("CONTINUOUS MOVE (W/S, speed 1-4):")[0]
        _step_title_w = fonts_ui["header"].size("STEP MOVE (A/D, step 5-8):")[0]
        btns_start_x = 20 + max(_speed_title_w, _step_title_w) + 30

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.VIDEORESIZE:
                current_w, current_h = event.w, event.h
                screen = pygame.display.set_mode((current_w, current_h), pygame.RESIZABLE)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    motors_enabled = False
                    print("🛑 EMERGENCY STOP: Motors Disabled!")
                elif event.key == pygame.K_d:
                    if target_q is not None and motors_enabled:
                        with cmd_lock:
                            lim_min, lim_max = SAFE_LIMITS[selected_joint]
                            val_deg = math.degrees(target_q[selected_joint]) + step_size
                            val_deg = np.clip(val_deg, lim_min, lim_max)
                            target_q[selected_joint] = math.radians(val_deg)
                elif event.key == pygame.K_a:
                    if target_q is not None and motors_enabled:
                        with cmd_lock:
                            lim_min, lim_max = SAFE_LIMITS[selected_joint]
                            val_deg = math.degrees(target_q[selected_joint]) - step_size
                            val_deg = np.clip(val_deg, lim_min, lim_max)
                            target_q[selected_joint] = math.radians(val_deg)
                elif event.key == pygame.K_RETURN or event.key == pygame.K_e:
                    if robot_state is not None:
                        with cmd_lock:
                            target_q = [0.0] * 26
                            for i in range(26):
                                sdk_idx = JOINT_IDX[i]
                                target_q[i] = robot_state.motor_state[sdk_idx].q
                            motors_enabled = True
                            print("🟢 Motors Enabled! Locked at current position.")
                    else:
                        print("⚠️ Cannot enable motors: No State packet received yet!")
                elif event.key == pygame.K_1:
                    speed_dps = 3.0
                elif event.key == pygame.K_2:
                    speed_dps = 5.0
                elif event.key == pygame.K_3:
                    speed_dps = 10.0
                elif event.key == pygame.K_4:
                    speed_dps = 15.0
                elif event.key == pygame.K_5:
                    step_size = 1.0
                elif event.key == pygame.K_6:
                    step_size = 3.0
                elif event.key == pygame.K_7:
                    step_size = 5.0
                elif event.key == pygame.K_8:
                    step_size = 7.0
                elif event.key == pygame.K_LEFT:
                    for gi, g in enumerate(groups):
                        if selected_joint in g["joints"]:
                            if gi > 0:
                                prev_group = groups[gi-1]
                                idx_in_curr = g["joints"].index(selected_joint)
                                next_idx = min(idx_in_curr, len(prev_group["joints"])-1)
                                selected_joint = prev_group["joints"][next_idx]
                            break
                elif event.key == pygame.K_RIGHT:
                    for gi, g in enumerate(groups):
                        if selected_joint in g["joints"]:
                            if gi < len(groups) - 1:
                                next_group = groups[gi+1]
                                idx_in_curr = g["joints"].index(selected_joint)
                                next_idx = min(idx_in_curr, len(next_group["joints"])-1)
                                selected_joint = next_group["joints"][next_idx]
                            break
                elif event.key == pygame.K_UP:
                    for g in groups:
                        if selected_joint in g["joints"]:
                            idx = g["joints"].index(selected_joint)
                            if idx > 0:
                                selected_joint = g["joints"][idx-1]
                            break
                elif event.key == pygame.K_DOWN:
                    for g in groups:
                        if selected_joint in g["joints"]:
                            idx = g["joints"].index(selected_joint)
                            if idx < len(g["joints"]) - 1:
                                selected_joint = g["joints"][idx+1]
                            break

            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = event.pos
                for col_idx, g in enumerate(groups):
                    x = margin_x + col_idx * (col_w + spacing_x)
                    for row_idx, j_id in enumerate(g["joints"]):
                        y = grid_y_start + row_idx * (card_h + spacing_y)
                        card_rect = pygame.Rect(x, y, col_w, card_h)
                        if card_rect.collidepoint(mx, my):
                            selected_joint = j_id

                for bi, val in enumerate(btn_speeds):
                    btn_rect = pygame.Rect(btns_start_x + bi * 85, current_h - 150, 75, 35)
                    if btn_rect.collidepoint(mx, my):
                        speed_dps = val

                for bi, val in enumerate(btn_steps):
                    btn_rect = pygame.Rect(btns_start_x + bi * 85, current_h - 100, 75, 35)
                    if btn_rect.collidepoint(mx, my):
                        step_size = val

                btn_stop = pygame.Rect(current_w * 0.42, current_h - 150, current_w * 0.13, 100)
                if btn_stop.collidepoint(mx, my):
                    motors_enabled = False
                    print("🛑 EMERGENCY STOP: Motors Disabled!")

                btn_enable = pygame.Rect(current_w * 0.56, current_h - 150, current_w * 0.13, 100)
                if btn_enable.collidepoint(mx, my):
                    if robot_state is not None:
                        with cmd_lock:
                            target_q = [0.0] * 26
                            for i in range(26):
                                sdk_idx = JOINT_IDX[i]
                                target_q[i] = robot_state.motor_state[sdk_idx].q
                            motors_enabled = True
                            print("🟢 Motors Enabled! Locked at current position.")
                    else:
                        print("⚠️ Cannot enable motors: No State packet received yet!")

        keys_pressed = pygame.key.get_pressed()
        if target_q is not None and motors_enabled:
            adjust_dir = 0.0
            if keys_pressed[pygame.K_w]:
                adjust_dir = 1.0
            elif keys_pressed[pygame.K_s]:
                adjust_dir = -1.0

            if adjust_dir != 0.0:
                with cmd_lock:
                    lim_min, lim_max = SAFE_LIMITS[selected_joint]
                    val_deg = math.degrees(target_q[selected_joint]) + adjust_dir * speed_dps * dt
                    val_deg = np.clip(val_deg, lim_min, lim_max)
                    target_q[selected_joint] = math.radians(val_deg)

        screen.fill(Theme.BG_MAIN)
        mx, my = pygame.mouse.get_pos()

        bar_rect = pygame.Rect(0, 0, current_w, 50)
        pygame.draw.rect(screen, Theme.BG_PANEL, bar_rect)
        pygame.draw.line(screen, Theme.BDR_DEFAULT, (0, 50), (current_w, 50), 1)

        status_text = "STATUS: "
        if robot_state is None:
            status_text += "WAITING FOR C++ BRIDGE..."
            status_color = Theme.ALERT
        elif not motors_enabled:
            status_text += "DAMPING (MOTORS OFF)"
            status_color = Theme.ACCENT
        else:
            status_text += "ACTIVE (LOW LEVEL CONTROL)"
            status_color = Theme.VAL_REAL

        lbl_status = fonts_ui["status"].render(status_text, True, status_color)
        screen.blit(lbl_status, (20, 16))

        selected_text = f"SELECTED JOINT (Arrows): [{selected_joint}] {JOINT_NAMES[selected_joint].upper()}"
        lbl_sel = fonts_ui["status"].render(selected_text, True, Theme.TXT_TITLE)
        rect_sel = lbl_sel.get_rect(center=(current_w // 2, 25))
        screen.blit(lbl_sel, rect_sel)

        fps = clock.get_fps()
        net_text = f"FPS: {fps:.0f} | INTERFACE: {interface.upper()}"
        lbl_net = fonts_ui["small"].render(net_text, True, Theme.TXT_LABEL)
        screen.blit(lbl_net, (current_w - lbl_net.get_width() - 20, 17))

        card_top_pad = 8
        name_block_h = font_sans_bold.get_height() + 6
        bar_h = 4
        bar_bottom_pad = 10
        bar_top_gap = 6
        content_top = card_top_pad + name_block_h
        content_bottom = card_h - bar_bottom_pad - bar_h - bar_top_gap
        content_h = max(20, content_bottom - content_top)
        row_h = content_h / 4.0
        bar_y_offset = card_h - bar_bottom_pad - bar_h

        header_y = bar_bottom + header_gap_top
        for col_idx, g in enumerate(groups):
            x = margin_x + col_idx * (col_w + spacing_x)
            header_text = fonts_ui["header"].render(g["name"], True, Theme.TXT_LABEL)
            header_rect = header_text.get_rect(centerx=x + col_w / 2.0, y=header_y)
            screen.blit(header_text, header_rect)

            for row_idx, j_id in enumerate(g["joints"]):
                y = grid_y_start + row_idx * (card_h + spacing_y)
                card_rect = pygame.Rect(x, y, col_w, card_h)

                hover = card_rect.collidepoint(mx, my)
                bg_color = Theme.BG_PANEL

                if j_id == selected_joint:
                    border_color = Theme.BDR_SELECTED
                    border_width = 2
                else:
                    border_color = Theme.BDR_DEFAULT
                    border_width = 1
                    if hover:
                        bg_color = lighten(bg_color, 0.06)

                pygame.draw.rect(screen, bg_color, card_rect, border_radius=6)
                pygame.draw.rect(screen, border_color, card_rect, width=border_width, border_radius=6)

                j_name_clean = f"[{j_id}] {JOINT_NAMES[j_id]}"
                text_name = font_sans_bold.render(j_name_clean, True, Theme.TXT_TITLE)
                screen.blit(text_name, (x + 12, y + card_top_pad))

                half_col = col_w / 2.0

                # Row 0: Limit
                lim_min, lim_max = SAFE_LIMITS[j_id]
                row_y = y + content_top + 0 * row_h
                text_lim_lbl = font_sans_normal.render("Limit:", True, Theme.TXT_LABEL)
                text_lim_val = font_mono_normal.render(f"{int(lim_min)}~{int(lim_max)}°", True, Theme.TXT_TITLE)
                screen.blit(text_lim_lbl, (x + 12, row_y))
                screen.blit(text_lim_val, (x + 55, row_y))

                # Row 1: Real / Tgt
                row_y = y + content_top + 1 * row_h
                curr_val = curr_q_deg[j_id]
                text_curr_lbl = font_sans_normal.render("Real:", True, Theme.TXT_LABEL)
                text_curr_val = font_mono_bold.render(f"{curr_val:5.1f}°", True, Theme.VAL_REAL)
                screen.blit(text_curr_lbl, (x + 12, row_y))
                screen.blit(text_curr_val, (x + 55, row_y))

                tgt_val = math.degrees(target_q[j_id]) if target_q is not None else 0.0
                text_tgt_lbl = font_sans_normal.render("Tgt:", True, Theme.TXT_LABEL)
                text_tgt_val = font_mono_bold.render(f"{tgt_val:5.1f}°", True, Theme.VAL_TGT if motors_enabled else Theme.TXT_LABEL)
                screen.blit(text_tgt_lbl, (x + half_col + 4, row_y))
                screen.blit(text_tgt_val, (x + half_col + 40, row_y))

                dq_val = 0.0
                tau_val = 0.0
                temp_val = 0.0
                with state_lock:
                    if robot_state is not None:
                        sdk_idx = JOINT_IDX[j_id]
                        dq_val = robot_state.motor_state[sdk_idx].dq
                        tau_val = robot_state.motor_state[sdk_idx].tau
                        temp_val = robot_state.motor_state[sdk_idx].temp

                temp_color = Theme.TXT_LABEL
                if temp_val > 65.0:
                    temp_color = Theme.ALERT
                elif temp_val > 50.0:
                    temp_color = Theme.ACCENT

                # Row 2: Vel
                row_y = y + content_top + 2 * row_h
                text_vel_lbl = font_sans_normal.render("Vel:", True, Theme.TXT_LABEL)
                text_vel_val = font_mono_normal.render(f"{dq_val:4.1f} r/s", True, Theme.TXT_TITLE)
                screen.blit(text_vel_lbl, (x + 12, row_y))
                screen.blit(text_vel_val, (x + 55, row_y))

                # Row 3: Trq / Temp
                row_y = y + content_top + 3 * row_h
                text_trq_lbl = font_sans_normal.render("Trq:", True, Theme.TXT_LABEL)
                text_trq_val = font_mono_normal.render(f"{tau_val:4.1f} Nm", True, Theme.TXT_TITLE)
                screen.blit(text_trq_lbl, (x + 12, row_y))
                screen.blit(text_trq_val, (x + 55, row_y))

                text_tmp_lbl = font_sans_normal.render("Temp:", True, Theme.TXT_LABEL)
                text_tmp_val = font_mono_normal.render(f"{temp_val:2.0f}°C", True, temp_color)
                screen.blit(text_tmp_lbl, (x + half_col + 4, row_y))
                screen.blit(text_tmp_val, (x + half_col + 44, row_y))

                range_span = lim_max - lim_min
                pct = 0.5
                if range_span > 0:
                    pct = (curr_val - lim_min) / range_span
                pct = max(0.0, min(1.0, pct))

                bar_rect = pygame.Rect(x + 12, y + bar_y_offset, col_w - 24, bar_h)
                pygame.draw.rect(screen, Theme.BDR_DEFAULT, bar_rect, border_radius=2)

                fill_w = int(pct * (col_w - 24))
                if fill_w > 0:
                    fill_rect = pygame.Rect(x + 12, y + bar_y_offset, fill_w, bar_h)
                    pygame.draw.rect(screen, Theme.VAL_REAL, fill_rect, border_radius=2)

        imu_x = margin_x + 5 * (col_w + spacing_x)
        imu_y = grid_y_start
        imu_h = 4 * card_h + 3 * spacing_y
        imu_rect = pygame.Rect(imu_x, imu_y, col_w, imu_h)

        imu_hover = imu_rect.collidepoint(mx, my)
        imu_bg = Theme.BG_PANEL
        if imu_hover:
            imu_bg = lighten(imu_bg, 0.06)

        pygame.draw.rect(screen, imu_bg, imu_rect, border_radius=8)
        pygame.draw.rect(screen, Theme.VAL_TGT, imu_rect, width=1, border_radius=8)

        col5_header = fonts_ui["header"].render("IMU TELEMETRY", True, Theme.TXT_LABEL)
        col5_header_rect = col5_header.get_rect(centerx=imu_x + col_w / 2.0, y=header_y)
        screen.blit(col5_header, col5_header_rect)

        lbl_imu_title = fonts_ui["header"].render("ROBOT IMU SENSOR", True, Theme.VAL_TGT)
        screen.blit(lbl_imu_title, (imu_x + 12, imu_y + 12))

        roll_deg, pitch_deg, yaw_deg = 0.0, 0.0, 0.0
        gx, gy, gz = 0.0, 0.0, 0.0
        roll_rad, pitch_rad, yaw_rad = 0.0, 0.0, 0.0
        with state_lock:
            if robot_state is not None:
                roll_rad = robot_state.imu_roll
                pitch_rad = robot_state.imu_pitch
                yaw_rad = robot_state.imu_yaw
                roll_deg = math.degrees(roll_rad)
                pitch_deg = math.degrees(pitch_rad)
                yaw_deg = math.degrees(yaw_rad)
                gx, gy, gz = robot_state.imu_gyro

        line_imu_h = (imu_h - 40) / 7.0

        lbl_roll_lbl = font_sans_normal.render("Roll:", True, Theme.TXT_LABEL)
        lbl_roll_val = font_mono_normal.render(f"{roll_deg:6.1f}° ({roll_rad:5.2f} rad)", True, Theme.TXT_TITLE)
        screen.blit(lbl_roll_lbl, (imu_x + 12, imu_y + 20 + 1 * line_imu_h))
        screen.blit(lbl_roll_val, (imu_x + 65, imu_y + 20 + 1 * line_imu_h))

        lbl_pitch_lbl = font_sans_normal.render("Pitch:", True, Theme.TXT_LABEL)
        lbl_pitch_val = font_mono_normal.render(f"{pitch_deg:6.1f}° ({pitch_rad:5.2f} rad)", True, Theme.TXT_TITLE)
        screen.blit(lbl_pitch_lbl, (imu_x + 12, imu_y + 20 + 2 * line_imu_h))
        screen.blit(lbl_pitch_val, (imu_x + 65, imu_y + 20 + 2 * line_imu_h))

        lbl_yaw_lbl = font_sans_normal.render("Yaw:", True, Theme.TXT_LABEL)
        lbl_yaw_val = font_mono_normal.render(f"{yaw_deg:6.1f}° ({yaw_rad:5.2f} rad)", True, Theme.TXT_TITLE)
        screen.blit(lbl_yaw_lbl, (imu_x + 12, imu_y + 20 + 3 * line_imu_h))
        screen.blit(lbl_yaw_val, (imu_x + 65, imu_y + 20 + 3 * line_imu_h))

        lbl_gx_lbl = font_sans_normal.render("GyroX:", True, Theme.TXT_LABEL)
        lbl_gx_val = font_mono_normal.render(f"{gx:5.2f} rad/s", True, Theme.VAL_REAL)
        screen.blit(lbl_gx_lbl, (imu_x + 12, imu_y + 20 + 4 * line_imu_h))
        screen.blit(lbl_gx_val, (imu_x + 65, imu_y + 20 + 4 * line_imu_h))

        lbl_gy_lbl = font_sans_normal.render("GyroY:", True, Theme.TXT_LABEL)
        lbl_gy_val = font_mono_normal.render(f"{gy:5.2f} rad/s", True, Theme.VAL_REAL)
        screen.blit(lbl_gy_lbl, (imu_x + 12, imu_y + 20 + 5 * line_imu_h))
        screen.blit(lbl_gy_val, (imu_x + 65, imu_y + 20 + 5 * line_imu_h))

        lbl_gz_lbl = font_sans_normal.render("GyroZ:", True, Theme.TXT_LABEL)
        lbl_gz_val = font_mono_normal.render(f"{gz:5.2f} rad/s", True, Theme.VAL_REAL)
        screen.blit(lbl_gz_lbl, (imu_x + 12, imu_y + 20 + 6 * line_imu_h))
        screen.blit(lbl_gz_val, (imu_x + 65, imu_y + 20 + 6 * line_imu_h))

        panel_rect = pygame.Rect(0, current_h - 180, current_w, 180)
        pygame.draw.rect(screen, Theme.BG_PANEL, panel_rect)
        pygame.draw.line(screen, Theme.BDR_DEFAULT, (0, current_h - 180), (current_w, current_h - 180), 1)

        text_speed_title = fonts_ui["header"].render("CONTINUOUS MOVE (W/S, speed 1-4):", True, Theme.TXT_TITLE)
        text_step_title = fonts_ui["header"].render("STEP MOVE (A/D, step 5-8):", True, Theme.TXT_TITLE)
        screen.blit(text_speed_title, (20, current_h - 142))
        screen.blit(text_step_title, (20, current_h - 92))

        for bi, val in enumerate(btn_speeds):
            btn_rect = pygame.Rect(btns_start_x + bi * 85, current_h - 150, 75, 35)
            is_active = (abs(speed_dps - val) < 0.1)
            hover_b = btn_rect.collidepoint(mx, my)
            btn_color = Theme.BG_MAIN if is_active else Theme.BDR_DEFAULT
            if hover_b and not is_active:
                btn_color = lighten(btn_color, 0.15)
            btn_bdr = Theme.BDR_SELECTED if is_active else Theme.BDR_DEFAULT
            pygame.draw.rect(screen, btn_color, btn_rect, border_radius=4)
            pygame.draw.rect(screen, btn_bdr, btn_rect, width=1, border_radius=4)

            lbl = fonts_ui["small"].render(f"{val}°/s", True, Theme.VAL_REAL if is_active else Theme.TXT_TITLE)
            lbl_rect = lbl.get_rect(center=btn_rect.center)
            screen.blit(lbl, lbl_rect)

        for bi, val in enumerate(btn_steps):
            btn_rect = pygame.Rect(btns_start_x + bi * 85, current_h - 100, 75, 35)
            is_active = (abs(step_size - val) < 0.1)
            hover_b = btn_rect.collidepoint(mx, my)
            btn_color = Theme.BG_MAIN if is_active else Theme.BDR_DEFAULT
            if hover_b and not is_active:
                btn_color = lighten(btn_color, 0.15)
            btn_bdr = Theme.BDR_SELECTED if is_active else Theme.BDR_DEFAULT
            pygame.draw.rect(screen, btn_color, btn_rect, border_radius=4)
            pygame.draw.rect(screen, btn_bdr, btn_rect, width=1, border_radius=4)

            lbl = fonts_ui["small"].render(f"{val}°", True, Theme.VAL_REAL if is_active else Theme.TXT_TITLE)
            lbl_rect = lbl.get_rect(center=btn_rect.center)
            screen.blit(lbl, lbl_rect)

        btn_stop = pygame.Rect(current_w * 0.42, current_h - 150, current_w * 0.13, 100)
        stop_hover = btn_stop.collidepoint(mx, my)
        stop_pressed = keys_pressed[pygame.K_SPACE]
        draw_button(
            screen, btn_stop, Theme.ALERT, Theme.BDR_ALERT,
            ["STOP (SPACE)", "DAMP MOTORS"],
            [(255, 255, 255), (255, 255, 255)],
            fonts_ui, hover=stop_hover, pressed=stop_pressed
        )

        btn_enable = pygame.Rect(current_w * 0.56, current_h - 150, current_w * 0.13, 100)
        enable_hover = btn_enable.collidepoint(mx, my)
        enable_pressed = keys_pressed[pygame.K_RETURN] or keys_pressed[pygame.K_e]
        if not motors_enabled:
            en_base = Theme.ACCENT
            en_border = Theme.BDR_ACCENT_DK
            en_text = (15, 15, 20)
        else:
            en_base = Theme.BDR_DEFAULT
            en_border = darken(Theme.BDR_DEFAULT, 0.25)
            en_text = Theme.TXT_LABEL
        draw_button(
            screen, btn_enable, en_base, en_border,
            ["ENABLE MOTORS", "LOCK POS (ENTER/E)"],
            [en_text, en_text],
            fonts_ui, hover=enable_hover, pressed=enable_pressed
        )

        pygame.display.flip()

    motors_enabled = False
    print("🛑 Exiting... Disabling all motors.")
    time.sleep(0.1)
    pygame.quit()

if __name__ == '__main__':
    main()