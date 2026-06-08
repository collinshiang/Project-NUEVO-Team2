# A.R.M.O.R. Full Script
# State Architecture Driven Logic.
# Full Mobility: Includes Traversal (Pure Pursuit), Obstacle Detection (LiDAR), and Detection (Camera Vision). No Sensing (GPS) Used.
# Also has dynamic masking for LiDAR (for ramp portion) & collision handling.
# MAE 162E - Capstone Project
# Group 2, Spring 2026

from __future__ import annotations
import time
import math

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

ENABLE_LIDAR  = True
ENABLE_VISION = True

# ---------------------------------------------------------------------------
# Vision Configuration
# ---------------------------------------------------------------------------

VISION_STALE_SEC = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.50  # higher = more skeptical     
MIN_STOP_SIGN_CONFIDENCE = 0.50  

# ---------------------------------------------------------------------------
# STATE-DEPENDENT DWA PROFILES
# ---------------------------------------------------------------------------

# Bounding box for LiDAR to know what the rover is (give slight buffer for each measurement)
ROBOT_FRONT_MM = 400.0       # Distance from rear axle to the front nose, actual 350
ROBOT_REAR_MM = 70.0         # Distance from rear axle to the back tail, actual 40
ROBOT_HALF_WIDTH_MM = 180.0  # Half of the total robot width, actual 165.0

# PROFILE 1: Normal Path Following (Phases 1 & 3)
STRAIGHT_MAX_VEL_MM_S = 180.0  
STRAIGHT_MAX_ACC_MM_S2 = 500.0  
STRAIGHT_MAX_ANGULAR_RAD_S = 1.8
STRAIGHT_MAX_ANGULAR_ACC_RAD_S2 = 5.0
STRAIGHT_PREDICT_TIME_S = 1.5           # shorter future vision
STRAIGHT_VELOCITY_SAMPLES = [7, 11]     # less trajectories to check [5, 7]
STRAIGHT_COST_GAINS = [3.0, 1.5, 2.5]   # higher heading weight [3.0, 1.0, 2.5]
STRAIGHT_LOOKAHEAD_MM = 300.0           # higher for proper tracking
STRAIGHT_TOLERANCE_MM = 100.0
STRAIGHT_OBSTACLES_RANGE_MM = 160.0     # lower for narrow walls

# PROFILE 2: OBS (Phase 2)
OBS_MAX_VEL_MM_S = 160.0  
OBS_MAX_ACC_MM_S2 = 500.0  
OBS_MAX_ANGULAR_RAD_S = 1.5
OBS_MAX_ANGULAR_ACC_RAD_S2 = 5.0
OBS_PREDICT_TIME_S = 4.0          
OBS_VELOCITY_SAMPLES = [7, 15]     # [linear_samples, angular_samples] - num. of calculated paths

# Cost Weights: [heading_gain, clearance_gain, velocity_gain]
OBS_COST_GAINS = [1.0, 1.5, 2.0]

OBS_LOOKAHEAD_MM = 200.0           # Unique lookahead for DWA (won't really matter later)
OBS_TOLERANCE_MM = 600.0           # large such that rover doesn't think wall at end is an obstacle and freaks out
OBS_OBSTACLES_RANGE_MM = 600.0     # Deep vision to plan dodges early 

# ---------------------------------------------------------------------------
# PATH CONFIGURATIONS - 3 Sections of Course
# ---------------------------------------------------------------------------

# Phase 1: Start to Obstacle Lane
PATH_CONTROL_POINTS_1 = [
    (0.0, 0.0),           # 1st point
    (0.0, 3530.0),        # 2nd point
    (490.0, 3530.0),      # 3rd point
    (490.0, 624.8),       # 4th point (.., 624.8)
    (1374.0, 624.8),      # 5th point
    (1374.0, 750.0),      # Ends exactly where DWA lane starts
]
DENSE_PATH_1 = densify_polyline(PATH_CONTROL_POINTS_1, spacing=20.0)

# Phase 2: Obstacle Lane
PATH_CONTROL_POINTS_2 = [
    (1374.0, 750.0),      
    (1374.0, 2700.0),
]
DENSE_PATH_2 = densify_polyline(PATH_CONTROL_POINTS_2, spacing=20.0)

# Phase 3: Finish Line
PATH_CONTROL_POINTS_3 = [
    (1374.0, 2700.0),     
    (1600.0, 2700.0),     
]
DENSE_PATH_3 = densify_polyline(PATH_CONTROL_POINTS_3, spacing=20.0)

# -----------  State Transition Thresholds  ---------------------------------

# Ramp Deadzone (Inside Phase 1)
RAMP_ZONE_X_MIN = 200.0
RAMP_ZONE_X_MAX = 1000.0
RAMP_ZONE_Y_MIN = 1300.0
RAMP_ZONE_Y_MAX = 3200.0

# Transition 1 (Phase 1 -> 2): Activates when turning into the obstacle lane
PHASE_2_ZONE_X_MIN = 1100.0
PHASE_2_ZONE_X_MAX = 2100.0
PHASE_2_ZONE_Y_MIN = 700.0

# Transition 2 (Phase 2 -> 3): Activates when leaving the obstacle lane
PHASE_3_ZONE_Y_MIN = 2630.0

STATUS_PRINT_INTERVAL_S = 0.5

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
        
    if ENABLE_VISION:
        robot.enable_vision()
        print("[sensor] Vision enabled — subscribing to YOLO detections")


def load_dwa_profile(robot: Robot, profile_type: str, period: float):
    if profile_type == "STRAIGHT":
        print("[CFG] Loading STRAIGHTAWAY DWA Profile")
        robot._init_dwa_planner(
            max_vel_mm=STRAIGHT_MAX_VEL_MM_S,
            max_acc_mm=STRAIGHT_MAX_ACC_MM_S2,
            max_angular_rad=STRAIGHT_MAX_ANGULAR_RAD_S,
            max_angular_acc_rad=STRAIGHT_MAX_ANGULAR_ACC_RAD_S2,
            lookahead_mm=STRAIGHT_LOOKAHEAD_MM,
            robot_front_mm=ROBOT_FRONT_MM,
            robot_rear_mm=ROBOT_REAR_MM,
            robot_half_width_mm=ROBOT_HALF_WIDTH_MM,
            tolerance_mm=STRAIGHT_TOLERANCE_MM,
            gains_of_costs=STRAIGHT_COST_GAINS,
            period=period,
            predict_time=STRAIGHT_PREDICT_TIME_S,
            predict_velocity_samples_resolution=STRAIGHT_VELOCITY_SAMPLES,
            obstacles_range_mm=STRAIGHT_OBSTACLES_RANGE_MM,
            ttc_weight=0.0
        )
    elif profile_type == "OBS":
        print("[CFG] Loading OBS DWA Profile")
        robot._init_dwa_planner(
            max_vel_mm=OBS_MAX_VEL_MM_S,
            max_acc_mm=OBS_MAX_ACC_MM_S2,
            max_angular_rad=OBS_MAX_ANGULAR_RAD_S,
            max_angular_acc_rad=OBS_MAX_ANGULAR_ACC_RAD_S2,
            lookahead_mm=OBS_LOOKAHEAD_MM,
            robot_front_mm=ROBOT_FRONT_MM,
            robot_rear_mm=ROBOT_REAR_MM,
            robot_half_width_mm=ROBOT_HALF_WIDTH_MM,
            tolerance_mm=OBS_TOLERANCE_MM,
            gains_of_costs=OBS_COST_GAINS,
            period=period,
            predict_time=OBS_PREDICT_TIME_S,
            predict_velocity_samples_resolution=OBS_VELOCITY_SAMPLES,
            obstacles_range_mm=OBS_OBSTACLES_RANGE_MM,
            ttc_weight=0.0
        )

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

def print_status(robot: Robot) -> None:
    pose = robot.get_pose()
    print(f"  odom=({pose[0]:6.0f}, {pose[1]:6.0f}) mm  θ={pose[2]:5.1f}°")

# ---------------------------------------------------------------------------
# FSM run() loop
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    time.sleep(0.5) 
    configure_robot(robot)

    state = "INIT"
    last_status_print_at = 0.0
    is_on_ramp = False
    
    # --- Recovery State Trackers ---
    previous_state = ""
    recovery_ticks = 0
    recovery_v = 0.0
    recovery_w = 0.0

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
                
                load_dwa_profile(robot, "STRAIGHT", period)

                if green_light_seen:
                    print("[VISION] Green light detected! Start Moving!")
                else:
                    print("[UI] BTN_1 pressed. Start Moving!")
                    
                state = "MOVING_PHASE_1"
            
            if robot.get_button(Button.BTN_2):
                print("BTN_2 pressed. Stopping robot.")
                robot.shutdown()

        elif state == "MOVING_PHASE_1":
            show_running_leds(robot)
            
            pose = robot.get_odometry_pose()
            current_x, current_y = pose[0], pose[1]
            

            # --- RAMP TRANSITION ---
            # 1. The Intercept Trigger - ramp bounding box
            if (RAMP_ZONE_X_MIN < current_x < RAMP_ZONE_X_MAX) and (RAMP_ZONE_Y_MIN < current_y < RAMP_ZONE_Y_MAX):
                print("\n[FSM] RAMP REACHED: Halting DWA for stationary alignment...")
                robot.stop()
                state = "RAMP_ALIGN"                

                
            # --- DWA TRANSITION (Phase 1 -> 2) ---
            if (current_x > PHASE_2_ZONE_X_MIN) and (current_x < PHASE_2_ZONE_X_MAX) and (current_y > PHASE_2_ZONE_Y_MIN):
                print("\n[FSM] ----------------------------------------------------")
                print("[FSM] OBSTACLE PORTION REACHED: Rover has turned into the DWA lane.")
                print("[FSM] Swapping to OBSTACLE Profile...")
                print("[FSM] ----------------------------------------------------\n")
                
                # Failsafe: Ensure mask is off before obstacle course
                if hasattr(robot, '_planner') and robot._planner is not None:
                    robot._planner.ramp_mask_active = False
                    
                load_dwa_profile(robot, "OBS", period)
                last_status_print_at = now
                state = "MOVING_PHASE_2"
                continue
            
            # --- Emergency Stop (Vision & UI) ---
            button_pressed = robot.get_button(Button.BTN_2)
            stop_sign_seen = ENABLE_VISION and is_stop_sign_detected(robot)
            
            if button_pressed or stop_sign_seen:
                if stop_sign_seen: print("[VISION] Stop sign detected! Stopping robot.")
                else: print("[UI] BTN_2 pressed. Stopping robot.")
                robot.stop()
                print("Path cancelled. Returning to IDLE.")
                state = "IDLE"
            else:
                # if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                #     print_status(robot)
                #     last_status_print_at = now
                
                status = robot._nav_follow_path_loop(DENSE_PATH_1, period)
                if status == "IDLE":
                    if current_x > PHASE_2_ZONE_X_MIN:
                        print("\n[FSM] Phase 1 Target reached early! Failsafe Engaging Phase 2...\n")
                        load_dwa_profile(robot, "OBS", period)
                        state = "MOVING_PHASE_2"
                    else:
                        print("MOVING_PHASE_1: Target reached unexpectedly. Stopping.")
                        robot.stop()
                        state = "IDLE"
                # --- COLLISION TRIGGER ---
                elif status == "COLLISION":
                    print(f"\n[FSM] Collision detected in Phase 1! Halting...")
                    previous_state = state  
                    state = "RECOVERY"


        elif state == "MOVING_PHASE_2":
            pose = robot.get_odometry_pose()
            current_y = pose[1]

            # --- TRANSITION LOGIC (Phase 2 -> 3) ---
            if current_y > PHASE_3_ZONE_Y_MIN:
                print("\n[FSM] ----------------------------------------------------")
                print(f"[FSM] Y > {PHASE_3_ZONE_Y_MIN} REACHED: End of Obstacle Lane.")
                print("[FSM] Swapping back to STRAIGHTAWAY Profile...")
                print("[FSM] ----------------------------------------------------\n")
                
                load_dwa_profile(robot, "STRAIGHT", period)
                last_status_print_at = now
                state = "MOVING_PHASE_3"
                continue

            # --- Emergency Stop (Vision & UI) ---
            button_pressed = robot.was_button_pressed(Button.BTN_2)
            stop_sign_seen = ENABLE_VISION and is_stop_sign_detected(robot)

            if button_pressed or stop_sign_seen:
                robot.stop()
                show_idle_leds(robot)
                if stop_sign_seen: print("[VISION] Stop sign detected! Stopping robot.")
                else: print("[FSM] IDLE — DWA navigation cancelled via UI")
                state = "IDLE"
            else:
                # if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                #     print_status(robot)
                #     last_status_print_at = now

                status = robot._nav_follow_path_loop(DENSE_PATH_2, period)
                if status == "IDLE":
                    print("\n[FSM] DWA Reached target early! Failsafe Engaging Phase 3...\n")
                    load_dwa_profile(robot, "STRAIGHT", period)
                    state = "MOVING_PHASE_3"
                # --- COLLISION TRIGGER ---
                elif status == "COLLISION":
                    print(f"\n[FSM] Collision detected in Phase 2! Halting...")
                    previous_state = state  
                    state = "RECOVERY"


        elif state == "MOVING_PHASE_3":
            show_running_leds(robot)
            
            # --- Emergency Stop (Vision & UI) ---
            button_pressed = robot.get_button(Button.BTN_2)
            stop_sign_seen = ENABLE_VISION and is_stop_sign_detected(robot)
            
            if button_pressed or stop_sign_seen:
                if stop_sign_seen: print("[VISION] Stop sign detected! Stopping robot.")
                else: print("[UI] BTN_2 pressed. Stopping robot.")
                robot.stop()
                print("Path cancelled. Returning to IDLE.")
                state = "IDLE"
            else:
                # if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                #     print_status(robot)
                #     last_status_print_at = now
                
                status = robot._nav_follow_path_loop(DENSE_PATH_3, period)
                if status == "IDLE":
                    print("[FSM] DONE — Final Phase 3 Goal complete!")
                    robot.stop()
                    show_idle_leds(robot)
                    print("[FSM] IDLE — Course finished.")
                    state = "IDLE"
                # --- COLLISION TRIGGER ---
                elif status == "COLLISION":
                    print(f"\n[FSM] Collision detected in Phase 3! Halting...")
                    previous_state = state  
                    state = "RECOVERY"

        elif state == "RAMP_ALIGN":
            """
            Rotates the rover in place until it is perfectly parallel with the ramp (-90 deg)
            """
            pose = robot.get_pose() # Returns [x, y, theta_deg]
            
            target_heading_rad = math.radians(-135.0)  # accounting for slip and hardcoding offset, otherwise would be 90deg
             
            # Convert the degree output from get_pose() to radians
            current_heading_rad = math.radians(pose[2])
            
            # Calculate the shortest angular distance to -90 degrees
            heading_error = target_heading_rad - current_heading_rad
            heading_error = (heading_error + math.pi) % (2.0 * math.pi) - math.pi
            
            # 3-Degree Tolerance (math.radians(3.0) approx 0.052 rad)
            if abs(heading_error) > 0.052:
                # Proportional controller for a smooth, stationary spin
                # Clamped to a max of 0.5 rad/s so it doesn't violently jerk
                w_cmd = max(min(heading_error * 1.5, 0.5), -0.5) 
                
                # Command 0 forward speed, only turning
                robot._send_body_velocity_mm(0.0, w_cmd)
            else:
                print("\n[FSM] ALIGNMENT COMPLETE: Locking heading and executing blind ramp traversal...")
                robot.stop()
                state = "RAMP_TRAVERSAL"


        elif state == "RAMP_TRAVERSAL":
            """
            Drives perfectly straight down the -Y axis. Bypasses DWA and LiDAR entirely.
            Uses a micro-correction P-controller to prevent slipping on the incline.
            """
            pose = robot.get_pose()
            current_y = pose[1]
            
            if current_y <= RAMP_ZONE_Y_MIN:
                # We have safely reached the bottom of the ramp
                print("\n[FSM] RAMP CLEARED: Waking up DWA for obstacle lane...")

                # --- MANUAL WAYPOINT FAST-FORWARD ---
                # # Deletes all waypoints physically behind the rover (higher up the ramp).
                # while DENSE_PATH_1:
                #     wp_x, wp_y = DENSE_PATH_1[0]
                #     if wp_y > current_y: 
                #         DENSE_PATH_1.pop(0)
                #     else:
                #         break

                state = "MOVING_PHASE_1"
            else:
                # Active Heading Hold
                # Continuously recalculates error so if one wheel slips on the 
                # ramp, the rover automatically corrects itself to stay at -90 deg.
                target_heading_rad = -math.pi / 2.0
                heading_error = target_heading_rad - pose[2]
                heading_error = (heading_error + math.pi) % (2.0 * math.pi) - math.pi
                
                # Drive forward at 150 mm/s, with tiny angular corrections
                v_cmd = 180.0 
                # w_cmd = max(min(heading_error * 1.5, 0.3), -0.3)
                w_cmd = 0.0 
                
                robot._send_body_velocity_mm(v_cmd, w_cmd)


        # --- RECOVERY STATE ---
        elif state == "RECOVERY":
            show_running_leds(robot)
            
            # Initialize the maneuver on the very first FSM tick
            if recovery_ticks == 0:
                # closest_dist, closest_angle = robot.get_closest_lidar_point()
                
                print(f"\n[FSM] COLLISION! Executing Backup Recovery...")

                # Back up in a straight line slowly
                recovery_v, recovery_w = -100.0, 0.0
            
            # Execute the stored raw maneuver
            robot._send_body_velocity_mm(recovery_v, recovery_w)
            recovery_ticks += 1
            
            # The Tick Counter (Execute maneuver for exactly 1.5 seconds)
            max_recovery_ticks = int(1.5 * float(DEFAULT_FSM_HZ))
            
            if recovery_ticks >= max_recovery_ticks:
                robot.stop()
                recovery_ticks = 0  # Reset counter for future collisions
                
                print(f"[FSM] Recovery Complete! Resuming {previous_state}...\n")
                
                # Throw the FSM back to the exact phase we crashed in
                last_status_print_at = now
                state = previous_state

            # 5. Emergency Stop (Always retain UI control during a recovery!)
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