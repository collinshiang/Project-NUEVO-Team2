# A.R.M.O.R. Full Script
# State Architecture Driven Logic.
# Full Mobility: Includes Traversal (Pure Pursuit), Obstacle Detection (LiDAR), and Detection (Camera Vision). No Sensing (GPS) Used.
# Also has collision handling.
# MAE 162E - Capstone Project
# Group 2, Spring 2026

from __future__ import annotations
import time
import math
import numpy as np

from robot.robot import FirmwareState, Robot
from robot.util import densify_polyline
from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    Motor,
    INITIAL_THETA_DEG,
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LIDAR_RANGE_MIN_MM,
    POSITION_UNIT,
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,
    WHEEL_BASE,
    WHEEL_DIAMETER,
    LEFT_WHEEL_MOTOR,
    LEFT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    RIGHT_WHEEL_DIR_INVERTED,
)

# ---------------------------------------------------------------------------
# Sensor Toggles
# ---------------------------------------------------------------------------
ENABLE_LIDAR     = True
ENABLE_GPS       = False    # currently only defined on ramp. can expand easily if needed
ENABLE_VISION    = False
ENABLE_STOP_SIGN = False  

# ---------------------------------------------------------------------------
# Vision Configuration
# ---------------------------------------------------------------------------
VISION_STALE_SEC = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.50  # higher = more skeptical   
MIN_STOP_SIGN_CONFIDENCE = 0.50  

# ---------------------------------------------------------------------------
# Pure Pursuit & GPS Configuration (Phase 1)
# ---------------------------------------------------------------------------
TAG_ID = 13                 
GPS_POSITION_ALPHA = 0.01   

PP_VELOCITY_MM_S = 220.0  # Higher speed for open track
PP_ANGULAR_VEL = 1.5
PP_LOOKAHEAD_MM = 100.0                                     
PP_TOLERANCE_MM = 20.0

# ---------------------------------------------------------------------------
# DWA Obstacle Avoidance Configuration (Phase 2)
# ---------------------------------------------------------------------------
# These limits ensure the planner doesn't demand impossible speeds/turns
DWA_MAX_VEL_MM_S = 160.0  # Slower, careful speed for cones
DWA_MAX_ACC_MM_S2 = 500.0  
DWA_MAX_ANGULAR_RAD_S = 1.5
DWA_MAX_ANGULAR_ACC_RAD_S2 = 5.0

# Trajectory generation parameters
DWA_PREDICT_TIME_S = 4.0        
DWA_VELOCITY_SAMPLES = [7, 15]  # [linear_samples, angular_samples] - num. of calculated paths

# Cost Weights: [heading_gain, clearance_gain, velocity_gain]
DWA_COST_GAINS = [1.0, 1.5, 2.0]
DWA_LOOKAHEAD_MM = 300.0        # Unique lookahead for DWA
DWA_TOLERANCE_MM = 600.0        # Goal point tolerance 

# Distance at which an obstacle starts generating a cost. 
DWA_OBSTACLES_RANGE_MM = 600.0  

# ---------------------------------------------------------------------------
# LASTLANE DWA Configuration (Phase 3)
# ---------------------------------------------------------------------------
LASTLANE_DWA_MAX_VEL_MM_S = 200.0  
LASTLANE_DWA_MAX_ACC_MM_S2 = 500.0  
LASTLANE_DWA_MAX_ANGULAR_RAD_S = 1.5
LASTLANE_DWA_MAX_ANGULAR_ACC_RAD_S2 = 5.0
LASTLANE_DWA_PREDICT_TIME_S = 1.5        
LASTLANE_DWA_VELOCITY_SAMPLES = [5, 7] 
LASTLANE_DWA_COST_GAINS = [2.5, 1.0, 2.5]
LASTLANE_DWA_LOOKAHEAD_MM = 300.0        
LASTLANE_DWA_TOLERANCE_MM = 150.0        
LASTLANE_DWA_OBSTACLES_RANGE_MM = 160.0

# ---------------------------------------------------------------------------
# Kinematic Footprint (LiDAR Bounding Box)
# ---------------------------------------------------------------------------
# Bounding box for LiDAR to know what the rover is (give slight buffer for each measurement)
ROBOT_FRONT_MM = 375.0       # Distance from rear axle to the front nose, actual 350
ROBOT_REAR_MM = 50.0         # Distance from rear axle to the back tail, actual 40
ROBOT_HALF_WIDTH_MM = 200.0  # Half of the total robot width, actual 165.0

# ---------------------------------------------------------------------------
# PATH CONFIGURATIONS - 3 Sections of Course
# ---------------------------------------------------------------------------
# Phase 1: Pure Pursuit Until Obstacles
PP_PATH_CONTROL_POINTS_1 = [    # all tuned to the lookahead of 100mm
    (0.0, 0.0),           # 1st point
    (0.0, 3520.0),        # 2nd point
    (530.0, 3520.0),      # 3rd point 
    (530.0, 610.0),       # 4th point
    (1370.0, 610.0),      # 5th point
    (1370.0, 750.0),      # 6th point, ends exactly where DWA lane starts
]
PP_DENSE_PATH_1 = densify_polyline(PP_PATH_CONTROL_POINTS_1, spacing=30.0)

# Phase 2: DWA Algorithm for Obstacles, separate so that DWA only sees the one lane
DWA_PATH_CONTROL_POINTS = [
    (1370.0, 750.0),      
    (1370.0, 3520.0),
]
DWA_DENSE_PATH = densify_polyline(DWA_PATH_CONTROL_POINTS, spacing=20.0)

# Phase 3: DWA to Finish Line
LASTLANE_DWA_PATH_CONTROL_POINTS = [
    (1370.0, 3520.0),     
    (2150.0, 3520.0),
    (2150.0, 610.0),    
]
LASTLANE_DWA_DENSE_PATH = densify_polyline(LASTLANE_DWA_PATH_CONTROL_POINTS, spacing=30.0)

# -----------  State Transition Thresholds  ---------------------------------
# GPS Activation Zone (between points 3 and 4)
GPS_ZONE_X_MIN = 300.0
GPS_ZONE_X_MAX = 1000.0
GPS_ZONE_Y_MIN = 1100.0
GPS_ZONE_Y_MAX = 3200.0

# DWA Handoff Zone (Activates when turning into the obstacle lane)
DWA_ZONE_X_MIN = 1100.0
DWA_ZONE_X_MAX = 2100.0
DWA_ZONE_Y_MIN = 700.0

# Phase 3 Handoff Zone (Activates when leaving the obstacle lane)
LASTLANE_ZONE_X_MIN = 1000.0
LASTLANE_ZONE_Y_MIN = 2900.0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )

    if ENABLE_LIDAR:
        robot.enable_lidar()
        robot.set_lidar_mount(
            x_mm=LIDAR_MOUNT_X_MM,
            y_mm=LIDAR_MOUNT_Y_MM,
            theta_deg=LIDAR_MOUNT_THETA_DEG,
        )
        robot.set_lidar_filter(
            range_min_mm=LIDAR_RANGE_MIN_MM,
            range_max_mm=LIDAR_RANGE_MAX_MM,
            fov_deg=LIDAR_FOV_DEG,
        )
        robot.start_lidar_world_publisher()
        print("[sensor] lidar enabled — subscribing to /scan")

    if ENABLE_GPS:
        robot.enable_gps()
        robot.set_tracked_tag_id(TAG_ID)
        robot.set_tag_body_offset(TAG_BODY_OFFSET_X_MM, TAG_BODY_OFFSET_Y_MM)
        robot.set_gps_offset(-20.7, -201.8) 
        
        # Start with GPS Fusion OFF
        robot.set_position_fusion_alpha(0.0)
        print(f"[sensor] GPS enabled — Tracking ArUco tag {TAG_ID} (Fusion starting OFF)")
        
    if ENABLE_VISION:
        robot.enable_vision()
        print("[sensor] Vision enabled — subscribing to YOLO detections")


def load_dwa_profile(robot: Robot, profile_type: str, period: float):
    """Injects either Phase 2 or Phase 3 DWA parameters into the Robot's planner."""
    if profile_type == "DWA":
        print("[CFG] Loading DWA Profile (Phase 2)")
        robot._init_dwa_planner(
            max_vel_mm=DWA_MAX_VEL_MM_S,
            max_acc_mm=DWA_MAX_ACC_MM_S2,
            max_angular_rad=DWA_MAX_ANGULAR_RAD_S,
            max_angular_acc_rad=DWA_MAX_ANGULAR_ACC_RAD_S2,
            lookahead_mm=DWA_LOOKAHEAD_MM,
            robot_front_mm=ROBOT_FRONT_MM,
            robot_rear_mm=ROBOT_REAR_MM,
            robot_half_width_mm=ROBOT_HALF_WIDTH_MM,
            tolerance_mm=DWA_TOLERANCE_MM,
            gains_of_costs=DWA_COST_GAINS,
            period=period,
            predict_time=DWA_PREDICT_TIME_S,
            predict_velocity_samples_resolution=DWA_VELOCITY_SAMPLES,
            obstacles_range_mm=DWA_OBSTACLES_RANGE_MM,
            ttc_weight=0.0
        )
    elif profile_type == "LASTLANE":
        print("[CFG] Loading LASTLANE DWA Profile (Phase 3)")
        robot._init_dwa_planner(
            max_vel_mm=LASTLANE_DWA_MAX_VEL_MM_S,
            max_acc_mm=LASTLANE_DWA_MAX_ACC_MM_S2,
            max_angular_rad=LASTLANE_DWA_MAX_ANGULAR_RAD_S,
            max_angular_acc_rad=LASTLANE_DWA_MAX_ANGULAR_ACC_RAD_S2,
            lookahead_mm=LASTLANE_DWA_LOOKAHEAD_MM,
            robot_front_mm=ROBOT_FRONT_MM,
            robot_rear_mm=ROBOT_REAR_MM,
            robot_half_width_mm=ROBOT_HALF_WIDTH_MM,
            tolerance_mm=LASTLANE_DWA_TOLERANCE_MM,
            gains_of_costs=LASTLANE_DWA_COST_GAINS,
            period=period,
            predict_time=LASTLANE_DWA_PREDICT_TIME_S,
            predict_velocity_samples_resolution=LASTLANE_DWA_VELOCITY_SAMPLES,
            obstacles_range_mm=LASTLANE_DWA_OBSTACLES_RANGE_MM,
            ttc_weight=0.0
        )


def start_pp_path_1(robot: Robot):
    """Starts the Phase 1 Pure Pursuit thread."""
    return robot.purepursuit_follow_path(
        waypoints=PP_DENSE_PATH_1,
        velocity=PP_VELOCITY_MM_S,
        lookahead=PP_LOOKAHEAD_MM,
        tolerance=PP_TOLERANCE_MM,
        advance_radius=PP_LOOKAHEAD_MM, 
        max_angular_rad_s=PP_ANGULAR_VEL,
        blocking=False, 
    )


def check_collision(robot: Robot) -> bool:
    """Watchdog: Manually scans LiDAR point cloud for obstacles in the forward driving path."""
    if not ENABLE_LIDAR:
        return False
        
    with robot._lock:
        obstacles = robot._obstacles_mm.copy()
        
    if obstacles.size > 0:
        obs_x = obstacles[:, 0] + LIDAR_MOUNT_X_MM
        obs_y = obstacles[:, 1] + LIDAR_MOUNT_Y_MM
        
        in_front_mask = (obs_x > 0) & (np.abs(obs_y) < (ROBOT_HALF_WIDTH_MM + 50.0))
        if np.any(in_front_mask):
            closest_fwd = np.min(obs_x[in_front_mask])
            if closest_fwd < (ROBOT_FRONT_MM + 120.0):
                return True
    return False


def find_traffic_light_color(robot: Robot) -> str | None:
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None
    best_color, best_confidence = None, -1.0
    for detection in robot.get_detections("traffic light"):
        confidence = float(detection["confidence"])
        if confidence < MIN_TRAFFIC_LIGHT_CONFIDENCE: continue
        color = detection.get("attributes", {}).get("color", {}).get("value")
        if color not in ("red", "green"): continue
        if confidence > best_confidence:
            best_confidence = confidence
            best_color = str(color)
    return best_color


def is_stop_sign_detected(robot: Robot) -> bool:
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC): return False
    for sign in robot.get_detections("stop sign"):
        if float(sign["confidence"]) >= MIN_STOP_SIGN_CONFIDENCE: return True
    return False


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


# ---------------------------------------------------------------------------
# FSM run() loop
# ---------------------------------------------------------------------------
def run(robot: Robot) -> None:
    time.sleep(0.5) 
    configure_robot(robot)

    state = "INIT"
    previous_state = ""
    recovery_ticks = 0
    
    drive_handle = None
    is_gps_fusion_active = False

    period = 1.0 / float(DEFAULT_FSM_HZ)
    print(f"FSM period: {period:.3f} seconds")
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        if state == "INIT":
            start_robot(robot)
            print("[FSM] INIT (odometry reset)")
            print("[FSM] IDLE - Press BTN_1 or show GREEN LIGHT to begin Phase 1.")
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)
            
            button_pressed = robot.get_button(Button.BTN_1)
            green_light_seen = ENABLE_VISION and find_traffic_light_color(robot) == "green"
            
            if button_pressed or green_light_seen:
                start_robot(robot) 
                show_running_leds(robot)
                if green_light_seen:
                    print("[VISION] Green light detected! Start Moving via Pure Pursuit (Phase 1)!")
                else:
                    print("[UI] BTN_1 pressed. Start Moving via Pure Pursuit (Phase 1)!")
                    
                drive_handle = start_pp_path_1(robot)
                state = "MOVING_PHASE_1"
            
            if robot.get_button(Button.BTN_2):
                print("BTN_2 pressed. Stopping robot.")
                robot.shutdown()


        # ==============================================================================
        # PHASE 1: PURE PURSUIT
        # ==============================================================================
        elif state == "MOVING_PHASE_1":
            show_running_leds(robot)
            
            # --- Location Tracking for Fencing ---
            pose = robot.get_fused_pose() if (ENABLE_GPS and robot.has_fused_pose()) else robot.get_odometry_pose()
            current_x = pose[0]
            current_y = pose[1]
            
            # --- CHECK IF WITHIN GPS RANGE FOR RAMP ---
            in_gps_zone = (GPS_ZONE_X_MIN < current_x < GPS_ZONE_X_MAX) and (GPS_ZONE_Y_MIN < current_y < GPS_ZONE_Y_MAX)
            if in_gps_zone and not is_gps_fusion_active and ENABLE_GPS:
                robot.set_position_fusion_alpha(GPS_POSITION_ALPHA)
                is_gps_fusion_active = True
                print(f"[GPS] Entering GPS Zone. Fusion ENABLED (alpha={GPS_POSITION_ALPHA})")
            elif not in_gps_zone and is_gps_fusion_active and ENABLE_GPS:
                robot.set_position_fusion_alpha(0.0)
                is_gps_fusion_active = False
                print("[GPS] Leaving GPS Zone. Fusion DISABLED (alpha=0.0)")
                
            # --- DWA TRANSITION (Phase 1 -> 2) ---
            if (DWA_ZONE_X_MIN < current_x < DWA_ZONE_X_MAX) and (current_y > DWA_ZONE_Y_MIN):
                if drive_handle is not None:
                    drive_handle.cancel()
                    drive_handle.wait(timeout=1.0)
                    drive_handle = None
                
                print("\n[FSM] ----------------------------------------------------")
                print("[FSM] OBSTACLE PORTION REACHED: Rover has turned into the DWA lane.")
                print("[FSM] Terminating Phase 1 and Engaging DWA Planner...")
                print("[FSM] ----------------------------------------------------\n")
                
                load_dwa_profile(robot, "DWA", period)
                state = "MOVING_PHASE_2"
                continue
            
            # --- Emergency Stop (Vision & UI) ---
            button_pressed = robot.get_button(Button.BTN_2)
            stop_sign_seen = False
            if ENABLE_VISION and ENABLE_STOP_SIGN:
                stop_sign_seen = is_stop_sign_detected(robot)
            
            if button_pressed or stop_sign_seen:
                if stop_sign_seen: print("[VISION] Stop sign detected! Stopping robot.")
                else: print("[UI] BTN_2 pressed. Stopping robot.")
                    
                if drive_handle is not None:
                    drive_handle.cancel()
                    drive_handle.wait(timeout=1.0)
                    drive_handle = None
                robot.stop()
                print("Path cancelled. Returning to IDLE.")
                state = "IDLE"
                
            # --- Perception Watchdog (Collision Check) ---
            # elif check_collision(robot):
            #     if drive_handle is not None:
            #         drive_handle.cancel()
            #         drive_handle.wait(timeout=1.0)
            #         drive_handle = None
            #     print("\n[FSM] Collision detected in Phase 1! Halting...")
            #     previous_state = state  
            #     state = "RECOVERY"
                
            else:
                if drive_handle is not None and drive_handle.is_finished():
                    # Failsafe: if PP1 finishes early but it's near the DWA zone, hand off.
                    if current_x > DWA_ZONE_X_MIN:
                        print("\n[FSM] PP_1 Target reached early! Failsafe Engaging DWA Planner...\n")
                        load_dwa_profile(robot, "DWA", period)
                        state = "MOVING_PHASE_2"
                    else:
                        print("MOVING_PHASE_1: Target reached unexpectedly. Stopping.")
                        drive_handle = None
                        robot.stop()
                        state = "IDLE"


        # ==============================================================================
        # PHASE 2: DWA
        # ==============================================================================
        elif state == "MOVING_PHASE_2":
            # --- Location Tracking for Fencing ---
            pose = robot.get_fused_pose() if (ENABLE_GPS and robot.has_fused_pose()) else robot.get_odometry_pose()
            current_x = pose[0]
            current_y = pose[1]

            # --- PHASE 3 HANDOFF LOGIC (Phase 2 -> 3) ---
            if current_y > LASTLANE_ZONE_Y_MIN and current_x > LASTLANE_ZONE_X_MIN:
                robot.stop()
                print("\n[FSM] ----------------------------------------------------")
                print(f"[FSM] Y > {LASTLANE_ZONE_Y_MIN} REACHED: End of Obstacle Lane.")
                print("[FSM] Terminating Phase 2 and Engaging Phase 3 LASTLANE DWA...")
                print("[FSM] ----------------------------------------------------\n")
                
                load_dwa_profile(robot, "LASTLANE", period)
                state = "MOVING_PHASE_3"
                continue

            # --- Emergency Stop (Vision & UI) ---
            button_pressed = robot.was_button_pressed(Button.BTN_2)
            stop_sign_seen = False
            if ENABLE_VISION and ENABLE_STOP_SIGN:
                stop_sign_seen = is_stop_sign_detected(robot)

            if button_pressed or stop_sign_seen:
                robot.stop()
                show_idle_leds(robot)
                if stop_sign_seen: print("[VISION] Stop sign detected! Stopping robot.")
                else: print("[FSM] IDLE — DWA navigation cancelled via UI")
                state = "IDLE"
            else:
                # Execute one tick of DWA navigation
                status = robot._nav_follow_path_loop(DWA_DENSE_PATH, period)

                if status == "IDLE":
                    # Failsafe if DWA finishes before Geofence
                    print("\n[FSM] DWA Reached target early! Failsafe Engaging Phase 3...\n")
                    load_dwa_profile(robot, "LASTLANE", period)
                    state = "MOVING_PHASE_3"
                elif status == "COLLISION":
                    print("\n[FSM] Collision detected in Phase 2! Halting...")
                    previous_state = state  
                    state = "RECOVERY"


        # ==============================================================================
        # PHASE 3: LASTLANE DWA
        # ==============================================================================
        elif state == "MOVING_PHASE_3":
            show_running_leds(robot)
            
            # --- Emergency Stop (Vision & UI) ---
            button_pressed = robot.was_button_pressed(Button.BTN_2)
            stop_sign_seen = False
            if ENABLE_VISION and ENABLE_STOP_SIGN:
                stop_sign_seen = is_stop_sign_detected(robot)
            
            if button_pressed or stop_sign_seen:
                robot.stop()
                show_idle_leds(robot)
                if stop_sign_seen: print("[VISION] Stop sign detected! Stopping robot.")
                else: print("[UI] BTN_2 pressed. Stopping robot.")
                state = "IDLE"
            else:
                status = robot._nav_follow_path_loop(LASTLANE_DWA_DENSE_PATH, period)
                
                if status == "IDLE":
                    print("[FSM] DONE — Final Phase 3 Goal complete!")
                    robot.stop()
                    show_idle_leds(robot)
                    print("[FSM] IDLE — Course finished.")
                    
                    if is_gps_fusion_active and ENABLE_GPS:
                        robot.set_position_fusion_alpha(0.0)
                        is_gps_fusion_active = False
                        
                    state = "IDLE"
                elif status == "COLLISION":
                    print("\n[FSM] Collision detected in Phase 3! Halting...")
                    previous_state = state  
                    state = "RECOVERY"


        # ==============================================================================
        # COLLISION RECOVERY
        # ==============================================================================
        elif state == "RECOVERY":
            show_running_leds(robot)
            
            if recovery_ticks == 0:
                print(f"\n[FSM] Executing Universal Backup Recovery...")

            # Back up in a straight line slowly. No turning.
            robot._send_body_velocity_mm(-100.0, 0.0)
            recovery_ticks += 1
            
            # Execute maneuver for exactly 1.5 seconds
            max_recovery_ticks = int(1.5 * float(DEFAULT_FSM_HZ))
            
            if recovery_ticks >= max_recovery_ticks:
                robot.stop()
                recovery_ticks = 0  
                
                print(f"[FSM] Recovery Complete! Resuming {previous_state}...\n")
                
                # If we crashed in Phase 1, we must re-spin the Pure Pursuit thread.
                # Since pure pursuit auto-snaps to nearest neighbor, it's perfectly safe to re-init.
                if previous_state == "MOVING_PHASE_1":
                    drive_handle = start_pp_path_1(robot)
                    
                state = previous_state

            # Retain UI control during recovery
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop()
                print("[UI] BTN_2 pressed during recovery. Stopping robot.")
                recovery_ticks = 0
                state = "IDLE"

        # FSM refresh rate control
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()