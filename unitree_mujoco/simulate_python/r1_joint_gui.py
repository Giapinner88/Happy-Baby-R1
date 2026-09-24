import sys
import os
import time
import math
import threading
import numpy as np
import pygame

# Initialize Unitree SDK2 Python bindings
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

# Joint definitions for R1 (26 joints)
JOINT_NAMES = [
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "waist_roll", "waist_yaw",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow", "left_wrist_roll",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow", "right_wrist_roll",
    "head_pitch", "head_yaw"
]

# Map from joint index (0-25) to Unitree SDK motor index
JOINT_IDX = [
    0, 1, 2, 3, 4, 5,       # Left leg (0-5)
    6, 7, 8, 9, 10, 11,     # Right leg (6-11)
    12, 13,                 # Waist (12-13)
    15, 16, 17, 18, 19,     # Left arm (14-18)
    22, 23, 24, 25, 26,     # Right arm (19-23)
    29, 30                  # Head (24-25)
]

# XML limits scaled to 90% for physical safety
SAFE_LIMITS = [
    [-152.0, 130.0],  # 0: left_hip_pitch
    [-52.0, 92.0],    # 1: left_hip_roll
    [-141.0, 141.0],  # 2: left_hip_yaw
    [-2.0, 131.0],    # 3: left_knee
    [-45.0, 28.0],    # 4: left_ankle_pitch
    [-13.0, 13.0],    # 5: left_ankle_roll
    [-152.0, 130.0],  # 6: right_hip_pitch
    [-92.0, 52.0],    # 7: right_hip_roll
    [-141.0, 141.0],  # 8: right_hip_yaw
    [-2.0, 131.0],    # 9: right_knee
    [-45.0, 28.0],    # 10: right_ankle_pitch
    [-13.0, 13.0],    # 11: right_ankle_roll
    [-27.0, 27.0],    # 12: waist_roll
    [-135.0, 135.0],  # 13: waist_yaw
    [-165.0, 105.0],  # 14: left_shoulder_pitch
    [-5.0, 134.0],    # 15: left_shoulder_roll
    [-99.0, 99.0],    # 16: left_shoulder_yaw
    [-46.0, 116.0],   # 17: left_elbow
    [-99.0, 99.0],    # 18: left_wrist_roll
    [-165.0, 105.0],  # 19: right_shoulder_pitch
    [-134.0, 5.0],    # 20: right_shoulder_roll
    [-99.0, 99.0],    # 21: right_shoulder_yaw
    [-46.0, 116.0],   # 22: right_elbow
    [-99.0, 99.0],    # 23: right_wrist_roll
    [-18.0, 18.0],    # 24: head_pitch
    [-30.0, 30.0]     # 25: head_yaw
]

KP_ARRAY = [
    100.0, 100.0, 100.0, 100.0, 40.0, 40.0,   # Left leg (0-5)
    100.0, 100.0, 100.0, 100.0, 40.0, 40.0,   # Right leg (6-11)
    100.0, 100.0,                              # Waist (12-13)
    40.0, 40.0, 20.0, 20.0, 20.0,             # Left arm (14-18)
    40.0, 40.0, 20.0, 20.0, 20.0,             # Right arm (19-23)
    50.0, 10.0                                # Head (24-25)
]

KD_ARRAY = [
    2.0, 2.0, 2.0, 2.0, 2.0, 2.0,             # Left leg
    2.0, 2.0, 2.0, 2.0, 2.0, 2.0,             # Right leg
    2.0, 2.0,                                 # Waist
    2.0, 2.0, 1.0, 1.0, 1.0,                  # Left arm
    2.0, 2.0, 1.0, 1.0, 1.0,                  # Right arm
    2.0, 0.1                                  # Head
]

# Global variables for communication
robot_state = None
state_lock = threading.Lock()
cmd_lock = threading.Lock()
motors_enabled = False
target_q = None # Will initialize to current angles upon first received state

cmd = unitree_hg_msg_dds__LowCmd_()

def state_handler(msg: LowState_):
    global robot_state, target_q
    with state_lock:
        robot_state = msg
        # Auto-initialize target_q to current physical angles on first packet
        if target_q is None:
            target_q = [0.0] * 26
            for i in range(26):
                sdk_idx = JOINT_IDX[i]
                target_q[i] = msg.motor_state[sdk_idx].q
            print(">>> Đã nhận dữ liệu khớp thực tế từ Robot. Khởi tạo vị trí đích an toàn!")

def dds_publisher_loop(pub):
    global cmd, motors_enabled, target_q
    crc_calc = CRC()
    while True:
        with cmd_lock:
            # 1. Zero out command struct
            for i in range(31):
                cmd.motor_cmd[i].mode = 0x00
                cmd.motor_cmd[i].q = 0.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kp = 0.0
                cmd.motor_cmd[i].kd = 0.0
                cmd.motor_cmd[i].tau = 0.0
            
            # 2. If motors are enabled and target_q is ready, populate commands
            if motors_enabled and target_q is not None:
                for i in range(26):
                    sdk_idx = JOINT_IDX[i]
                    cmd.motor_cmd[sdk_idx].mode = 0x01
                    cmd.motor_cmd[sdk_idx].q = float(target_q[i])
                    cmd.motor_cmd[sdk_idx].kp = float(KP_ARRAY[i])
                    cmd.motor_cmd[sdk_idx].kd = float(KD_ARRAY[i])
            
            # 3. Calculate CRC and write
            cmd.crc = crc_calc.Crc(cmd)
            pub.Write(cmd)
            
        time.sleep(0.005) # 200 Hz

def main():
    global motors_enabled, target_q
    
    # Parse interface argument
    interface = "eno1"
    if len(sys.argv) >= 2:
        interface = sys.argv[1]
        
    print(f"📡 Đang kết nối mạng DDS qua card mạng: {interface}")
    if interface == "lo":
        print(">>> Chế độ GIẢ LẬP (Domain 1)")
        ChannelFactoryInitialize(1, "lo")
    else:
        print(">>> Chế độ ROBOT THẬT (Domain 0)")
        ChannelFactoryInitialize(0, interface)
        
    # Set up publisher & subscriber
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(state_handler, 10)
    
    # Start background DDS publishing thread
    pub_thread = threading.Thread(target=dds_publisher_loop, args=(pub,), daemon=True)
    pub_thread.start()
    
    # Initialize Pygame GUI
    pygame.init()
    screen_w, screen_h = 1350, 850
    screen = pygame.display.set_mode((screen_w, screen_h))
    pygame.display.set_caption("R1 Low-Level Joint Tuning GUI")
    clock = pygame.time.Clock()
    
    # Fonts
    font_title = pygame.font.SysFont("Courier New", 22, bold=True)
    font_bold = pygame.font.SysFont("Courier New", 14, bold=True)
    font_normal = pygame.font.SysFont("Courier New", 13)
    
    # Palette
    C_BG = (15, 15, 20)           # Dark Slate background
    C_CARD = (22, 22, 30)         # Gray card panel
    C_CARD_HOVER = (30, 32, 44)   # Hover state
    C_CARD_SEL = (38, 42, 60)     # Selected joint card
    C_BORDER = (45, 45, 55)
    C_BORDER_SEL = (137, 180, 250) # Blue glow
    C_TEXT = (220, 225, 235)
    C_GREEN = (166, 227, 161)
    C_RED = (243, 139, 168)
    C_AMBER = (250, 179, 135)
    C_MUTED = (147, 153, 178)
    
    # Grouping columns layout
    groups = [
        {"name": "LEFT LEG", "joints": list(range(0, 6))},
        {"name": "RIGHT LEG", "joints": list(range(6, 12))},
        {"name": "WAIST & HEAD", "joints": [12, 13, 24, 25]},
        {"name": "LEFT ARM", "joints": list(range(14, 19))},
        {"name": "RIGHT ARM", "joints": list(range(19, 24))}
    ]
    
    col_w = 240
    col_spacing = 20
    start_x = 30
    start_y = 70
    card_h = 90
    card_w = 230
    
    # Navigation parameters
    selected_joint = 24 # Default to head_pitch
    step_size = 2.0     # Default step size in degrees
    speed_dps = 5.0     # Default continuous speed in degrees/second
    
    # UI Control bounds
    btn_steps = [0.5, 1.0, 2.0, 5.0]
    btn_speeds = [1.0, 2.0, 5.0, 10.0]
    
    running = True
    while running:
        dt = clock.tick(60) / 1000.0 # Delta time in seconds
        
        # Read current physical angles from State
        curr_q_deg = [0.0] * 26
        with state_lock:
            if robot_state is not None:
                for i in range(26):
                    sdk_idx = JOINT_IDX[i]
                    curr_q_deg[i] = math.degrees(robot_state.motor_state[sdk_idx].q)
        
        # Event loop
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    # EMERGENCY STOP
                    motors_enabled = False
                    print("🛑 EMERGENCY STOP: Motors Disabled!")
                
                # Single-press Step Adjustments using A/D
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
                            
                # Enable motors / Lock position using Enter or E key
                elif event.key == pygame.K_RETURN or event.key == pygame.K_e:
                    if robot_state is not None:
                        with cmd_lock:
                            target_q = [0.0] * 26
                            for i in range(26):
                                sdk_idx = JOINT_IDX[i]
                                target_q[i] = robot_state.motor_state[sdk_idx].q
                            motors_enabled = True
                            print("🟢 Motors Enabled! Posture locked at current position.")
                    else:
                        print("⚠️ Cannot enable motors: No State packet received yet!")

                # Speed settings keys 1-4
                elif event.key == pygame.K_1:
                    speed_dps = 1.0
                elif event.key == pygame.K_2:
                    speed_dps = 2.0
                elif event.key == pygame.K_3:
                    speed_dps = 5.0
                elif event.key == pygame.K_4:
                    speed_dps = 10.0

                # Step size settings keys 5-8
                elif event.key == pygame.K_5:
                    step_size = 0.5
                elif event.key == pygame.K_6:
                    step_size = 1.0
                elif event.key == pygame.K_7:
                    step_size = 2.0
                elif event.key == pygame.K_8:
                    step_size = 5.0

                # Column navigation keys
                elif event.key == pygame.K_LEFT:
                    # Move to left column
                    for gi, g in enumerate(groups):
                        if selected_joint in g["joints"]:
                            if gi > 0:
                                prev_group = groups[gi-1]
                                # Choose closest matching vertical index
                                idx_in_curr = g["joints"].index(selected_joint)
                                next_idx = min(idx_in_curr, len(prev_group["joints"])-1)
                                selected_joint = prev_group["joints"][next_idx]
                            break
                elif event.key == pygame.K_RIGHT:
                    # Move to right column
                    for gi, g in enumerate(groups):
                        if selected_joint in g["joints"]:
                            if gi < len(groups) - 1:
                                next_group = groups[gi+1]
                                idx_in_curr = g["joints"].index(selected_joint)
                                next_idx = min(idx_in_curr, len(next_group["joints"])-1)
                                selected_joint = next_group["joints"][next_idx]
                            break
                elif event.key == pygame.K_UP:
                    # Navigate up in the current column
                    for g in groups:
                        if selected_joint in g["joints"]:
                            idx = g["joints"].index(selected_joint)
                            if idx > 0:
                                selected_joint = g["joints"][idx-1]
                            break
                elif event.key == pygame.K_DOWN:
                    # Navigate down in the current column
                    for g in groups:
                        if selected_joint in g["joints"]:
                            idx = g["joints"].index(selected_joint)
                            if idx < len(g["joints"]) - 1:
                                selected_joint = g["joints"][idx+1]
                            break
            
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = event.pos
                
                # Check clicks on Joint Cards
                for col_idx, g in enumerate(groups):
                    x = start_x + col_idx * (col_w + col_spacing)
                    for row_idx, j_id in enumerate(g["joints"]):
                        y = start_y + row_idx * (card_h + 12)
                        card_rect = pygame.Rect(x, y, card_w, card_h)
                        if card_rect.collidepoint(mx, my):
                            selected_joint = j_id
                
                # Check clicks on Speed Preset buttons
                for bi, val in enumerate(btn_speeds):
                    btn_rect = pygame.Rect(300 + bi * 80, 680, 70, 30)
                    if btn_rect.collidepoint(mx, my):
                        speed_dps = val
                
                # Check clicks on Step Preset buttons
                for bi, val in enumerate(btn_steps):
                    btn_rect = pygame.Rect(300 + bi * 80, 730, 70, 30)
                    if btn_rect.collidepoint(mx, my):
                        step_size = val
                
                # Emergency Stop button click
                btn_stop = pygame.Rect(800, 680, 200, 80)
                if btn_stop.collidepoint(mx, my):
                    motors_enabled = False
                    print("🛑 EMERGENCY STOP: Motors Disabled!")
                
                # Enable button click
                btn_enable = pygame.Rect(1030, 680, 200, 80)
                if btn_enable.collidepoint(mx, my):
                    if robot_state is not None:
                        with cmd_lock:
                            # Re-read and snap targets to current angles before enabling
                            target_q = [0.0] * 26
                            for i in range(26):
                                sdk_idx = JOINT_IDX[i]
                                target_q[i] = robot_state.motor_state[sdk_idx].q
                            motors_enabled = True
                            print("🟢 Motors Enabled! Posture locked at current position.")
                    else:
                        print("⚠️ Cannot enable motors: No State packet received yet!")
 
        # Continuous adjustments (while keys are held down)
        keys_pressed = pygame.key.get_pressed()
        if target_q is not None and motors_enabled:
            # W key holds can increase continuous target angle
            # S key holds can decrease continuous target angle
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

        # Draw UI background
        screen.fill(C_BG)
        
        # Render Title
        title_text = font_title.render("UNITREE R1 LOW-LEVEL CALIBRATION / JOINT TUNER", True, C_TEXT)
        screen.blit(title_text, (30, 25))
        
        # Draw columns and cards
        mx, my = pygame.mouse.get_pos()
        for col_idx, g in enumerate(groups):
            # Render Column Header
            header_text = font_bold.render(g["name"], True, C_MUTED)
            x = start_x + col_idx * (col_w + col_spacing)
            screen.blit(header_text, (x, start_y - 25))
            
            for row_idx, j_id in enumerate(g["joints"]):
                y = start_y + row_idx * (card_h + 12)
                card_rect = pygame.Rect(x, y, card_w, card_h)
                
                # Check mouse hover
                hover = card_rect.collidepoint(mx, my)
                bg_color = C_CARD_SEL if j_id == selected_joint else (C_CARD_HOVER if hover else C_CARD)
                border_color = C_BORDER_SEL if j_id == selected_joint else C_BORDER
                
                # Draw Card Box
                pygame.draw.rect(screen, bg_color, card_rect, border_radius=6)
                pygame.draw.rect(screen, border_color, card_rect, width=2 if j_id == selected_joint else 1, border_radius=6)
                
                # Render Joint Name
                j_name_clean = f"[{j_id}] {JOINT_NAMES[j_id]}"
                text_name = font_bold.render(j_name_clean, True, C_TEXT)
                screen.blit(text_name, (x + 10, y + 10))
                
                # Render limits
                lim_min, lim_max = SAFE_LIMITS[j_id]
                text_lim = font_normal.render(f"Lim: {int(lim_min)}~{int(lim_max)}°", True, C_MUTED)
                screen.blit(text_lim, (x + 10, y + 30))
                
                # Render physical current and target angles
                curr_val = curr_q_deg[j_id]
                text_curr = font_normal.render(f"Real : {curr_val:6.1f}°", True, C_AMBER)
                screen.blit(text_curr, (x + 10, y + 50))
                
                tgt_val = math.degrees(target_q[j_id]) if target_q is not None else 0.0
                text_tgt = font_normal.render(f"Target: {tgt_val:6.1f}°", True, C_GREEN if motors_enabled else C_MUTED)
                screen.blit(text_tgt, (x + 10, y + 68))
        
        # Draw bottom control panel
        panel_rect = pygame.Rect(30, 660, screen_w - 60, 160)
        pygame.draw.rect(screen, C_CARD, panel_rect, border_radius=10)
        pygame.draw.rect(screen, C_BORDER, panel_rect, width=1, border_radius=10)
        
        # Render continuous speed buttons
        text_speed_title = font_bold.render("DI CHUYỂN LIÊN TỤC (Nhấn giữ W/S):", True, C_TEXT)
        screen.blit(text_speed_title, (50, 685))
        for bi, val in enumerate(btn_speeds):
            btn_rect = pygame.Rect(320 + bi * 85, 680, 75, 30)
            is_active = (abs(speed_dps - val) < 0.1)
            btn_color = C_CARD_SEL if is_active else C_CARD_HOVER
            btn_bdr = C_BORDER_SEL if is_active else C_BORDER
            pygame.draw.rect(screen, btn_color, btn_rect, border_radius=4)
            pygame.draw.rect(screen, btn_bdr, btn_rect, width=1, border_radius=4)
            
            lbl = font_bold.render(f"{val}°/s", True, C_GREEN if is_active else C_TEXT)
            screen.blit(lbl, (btn_rect.x + 15, btn_rect.y + 7))
            
        # Render single-press step size buttons
        text_step_title = font_bold.render("DI CHUYỂN TỪNG BƯỚC (Phím A/D):", True, C_TEXT)
        screen.blit(text_step_title, (50, 735))
        for bi, val in enumerate(btn_steps):
            btn_rect = pygame.Rect(320 + bi * 85, 730, 75, 30)
            is_active = (abs(step_size - val) < 0.1)
            btn_color = C_CARD_SEL if is_active else C_CARD_HOVER
            btn_bdr = C_BORDER_SEL if is_active else C_BORDER
            pygame.draw.rect(screen, btn_color, btn_rect, border_radius=4)
            pygame.draw.rect(screen, btn_bdr, btn_rect, width=1, border_radius=4)
            
            lbl = font_bold.render(f"{val}°", True, C_GREEN if is_active else C_TEXT)
            screen.blit(lbl, (btn_rect.x + 22, btn_rect.y + 7))
            
        # Draw Emergency STOP button (Spacebar)
        btn_stop = pygame.Rect(800, 680, 200, 80)
        pygame.draw.rect(screen, C_RED, btn_stop, border_radius=10)
        lbl_stop_1 = font_title.render("STOP (SPACE)", True, (255, 255, 255))
        lbl_stop_2 = font_bold.render("XẢ MÔ-MEN KHỚP", True, (255, 255, 255))
        screen.blit(lbl_stop_1, (btn_stop.x + 25, btn_stop.y + 20))
        screen.blit(lbl_stop_2, (btn_stop.x + 35, btn_stop.y + 45))
        
        # Draw Enable / Snap button
        btn_enable = pygame.Rect(1030, 680, 200, 80)
        pygame.draw.rect(screen, C_GREEN if not motors_enabled else C_MUTED, btn_enable, border_radius=10)
        lbl_en_1 = font_title.render("ENABLE MOTORS", True, (15, 15, 20))
        lbl_en_2 = font_bold.render("KHÓA VỊ TRÍ HIỆN TẠI", True, (15, 15, 20))
        screen.blit(lbl_en_1, (btn_enable.x + 18, btn_enable.y + 20))
        screen.blit(lbl_en_2, (btn_enable.x + 22, btn_enable.y + 45))
        
        # Status / Instructions text
        instr_1 = font_normal.render("KEYS: [Mũi tên]: Chọn khớp | [W/S]: Giữ phím xoay liên tục", True, C_MUTED)
        instr_2 = font_normal.render("      [A/D]: Xoay theo bước | [Space]: NGẮT ĐỘNG CƠ (DAMP)", True, C_MUTED)
        instr_3 = font_normal.render("      [Enter/E]: BẬT ĐỘNG CƠ | [1-4]: Tốc độ | [5-8]: Cỡ bước", True, C_MUTED)
        screen.blit(instr_1, (750, 775))
        screen.blit(instr_2, (750, 792))
        screen.blit(instr_3, (750, 809))
        
        # State information / network info
        status_text = "STATUS: "
        if robot_state is None:
            status_text += "WAITING FOR DDS STATE PACKET..."
            status_color = C_RED
        elif not motors_enabled:
            status_text += "DAMPING (MOTORS OFF)"
            status_color = C_AMBER
        else:
            status_text += "ACTIVE (LOW LEVEL CONTROL)"
            status_color = C_GREEN
            
        lbl_status = font_bold.render(status_text, True, status_color)
        screen.blit(lbl_status, (50, 785))
        
        net_info = f"INTERFACE: {interface} | SELECTED JOINT: [{selected_joint}] {JOINT_NAMES[selected_joint]}"
        lbl_net = font_normal.render(net_info, True, C_TEXT)
        screen.blit(lbl_net, (50, 805))
        
        pygame.display.flip()

    # Disable all motors on exit
    motors_enabled = False
    print("🛑 Exiting... Disabling all motors.")
    time.sleep(0.1)
    pygame.quit()

if __name__ == '__main__':
    main()
