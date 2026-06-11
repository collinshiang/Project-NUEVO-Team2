"""
auto_3d_debris_retrieval.py
===========================
3D stationary manipulation sequence using Stepper Motor for Yaw alignment.
Includes corrected Lateral Transformation, 100mm Pickup Z-Height, 
and dedicated 270-degree hardware commands.
"""

from __future__ import annotations
import math
import time

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    ServoChannel,
    Stepper,
    StepMoveType,
)
from robot.robot import FirmwareState, Robot

# ---------------------------------------------------------------------------
# Physical Geometry Configuration (Arm & Servos)
# ---------------------------------------------------------------------------
Z_LIDAR = 110.0 # tune this
Z_LTS = 65.0    # lidar to stepper
Z_SS = 8.5
L_1 = 54.2
L_J2J3 = 201.0
L_J3EE = 330.0        
X_0 = 0.0     # Forward offset from LiDAR to Stepper Pivot
Y_0 = -115.0  # Lateral offset from LiDAR to Stepper Pivot (Negative = Right)

Z_0 = Z_LIDAR + Z_LTS + Z_SS
SHOULDER_OFFSET = Z_0 + L_1  

SHOULDER_SERVO = ServoChannel.CH_1
ELBOW_SERVO = ServoChannel.CH_2
GRIPPER_SERVO = ServoChannel.CH_3

# --- PHYSICAL HARDWARE SAFETY LIMITS (IN 270 DEG SPACE) ---
SHOULDER_MIN = 2.0
SHOULDER_MAX = 268.0 
ELBOW_MIN = 2.0
ELBOW_MAX = 178.0

# --- MECHANICAL OFFSETS ---
# Subtracted from logical commands so the physical servo lags behind the math
SHOULDER_MOUNT_OFFSET = -10.0  

# --- TARGET ARM STATES (IN PHYSICAL DEGREES) ---
# 0 degrees = fully extended straight forward
SHOULDER_REST_DEG = 180.0    # Folded backward
ELBOW_REST_DEG = 10.0        # Folded inward

SHOULDER_MAST_DEG = 90.0     # Straight up
ELBOW_MAST_DEG = 178.0       # Straight out

# Gripper is a standard 180-degree servo, limits apply directly
GRIPPER_OPEN_DEG = 178.0    
GRIPPER_CLOSE_DEG = 50.0        

# ---------------------------------------------------------------------------
# Perception & Stepper Configuration
# ---------------------------------------------------------------------------
ENABLE_VISION = True
VISION_STALE_SEC = 3.0

SCAN_FOV_DEG = 60.0
DEBRIS_MIN_DIST_MM = 100.0
DEBRIS_MAX_DIST_MM = 600.0
CLUSTER_TOLERANCE_MM = 80.0  
ISOLATION_RADIUS_MM = 100.0  
PICK_Z_HEIGHT_MM = 150.0        # manual adjust later 

# --- STEPPER YAW CONFIGURATION ---
BASE_STEPPER = Stepper.STEPPER_1
STEPS_PER_REV = 200.0
BASE_STEPPER_VEL = 30           
BASE_STEPPER_ACCEL = 400        
BACKLASH_STEPS = 35             
DISCARD_ADDITIONAL_TURN_DEG = 90.0


class ArmKinematics3D:
    def calculate_ik(self, r_target_mm: float, z_target_mm: float) -> tuple[float, float]:
        r_arm = r_target_mm               
        z_arm = z_target_mm - SHOULDER_OFFSET   

        d = math.hypot(r_arm, z_arm)
        max_reach = L_J2J3 + L_J3EE
        if d > max_reach or d < abs(L_J2J3 - L_J3EE):
            raise ValueError(f"Target unreachable. Distance from shoulder is {d:.1f}mm (Max: {max_reach}mm)")
       
        D = (r_arm**2 + z_arm**2 - L_J2J3**2 - L_J3EE**2) / (2 * L_J2J3 * L_J3EE)
        D = max(min(D, 1.0), -1.0)
        
        theta3_rad = math.atan2(-math.sqrt(1 - D**2), D)
        theta2_rad = math.atan2(z_arm, r_arm) - math.atan2(
            L_J3EE * math.sin(theta3_rad), 
            L_J2J3 + L_J3EE * math.cos(theta3_rad)
        )

        return math.degrees(theta2_rad), math.degrees(theta3_rad)


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)

def command_shoulder_physical(robot: Robot, logical_deg: float) -> None:
    """Applies mechanical offsets and safety clamps before commanding hardware."""
    physical_deg = logical_deg + SHOULDER_MOUNT_OFFSET
    safe_phys = max(SHOULDER_MIN, min(SHOULDER_MAX, physical_deg))
    robot.set_servo_270(SHOULDER_SERVO, safe_phys)

def stow_arm(robot: Robot) -> None:
    print(f"[ACTUATE] Stowing Arm safely to {SHOULDER_REST_DEG}/{ELBOW_REST_DEG}...")
    # Shoulder goes through the offset wrapper
    command_shoulder_physical(robot, SHOULDER_REST_DEG)
    
    # Elbow goes directly to hardware
    safe_e = max(ELBOW_MIN, min(ELBOW_MAX, ELBOW_REST_DEG))
    robot.set_servo_180_wide(ELBOW_SERVO, safe_e)

def move_servos_slowly(robot: Robot, curr_s: float, curr_e: float, target_s: float, target_e: float, step_deg=2.0, delay_s=0.02) -> None:
    # We maintain logical math here, only clamping the elbow which lacks a wrapper
    safe_target_e = max(ELBOW_MIN, min(ELBOW_MAX, target_e))
    
    diff_s = target_s - curr_s
    diff_e = safe_target_e - curr_e
    
    max_diff = max(abs(diff_s), abs(diff_e))
    num_steps = int(math.ceil(max_diff / step_deg))
    
    if num_steps <= 0: return

    for i in range(1, num_steps + 1):
        fraction = i / float(num_steps)
        
        logical_s = curr_s + (diff_s * fraction)
        logical_e = curr_e + (diff_e * fraction)
        
        command_shoulder_physical(robot, logical_s)
        robot.set_servo_180_wide(ELBOW_SERVO, logical_e)
        time.sleep(delay_s)

def move_gripper_slowly(robot: Robot, curr_g: float, target_g: float, step_deg=1.0, delay_s=0.02) -> None:
    """Smoothly actuates the 180-degree gripper to prevent snapping and jerky momentum."""
    diff_g = target_g - curr_g
    num_steps = int(math.ceil(abs(diff_g) / step_deg))
    
    if num_steps <= 0: return
    
    for i in range(1, num_steps + 1):
        fraction = i / float(num_steps)
        pos = curr_g + (diff_g * fraction)
        robot.set_servo(GRIPPER_SERVO, pos)
        time.sleep(delay_s)

def command_yaw_stepper(robot: Robot, steps: int) -> None:
    if steps == 0: return
    robot.step_set_config(BASE_STEPPER, max_velocity=BASE_STEPPER_VEL, acceleration=BASE_STEPPER_ACCEL)
    robot.step_move(BASE_STEPPER, steps, StepMoveType.RELATIVE, blocking=True)

def find_traffic_light_color(robot: Robot) -> str | None:
    if not ENABLE_VISION or not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None
    for detection in robot.get_detections("traffic light"):
        color = detection.get("attributes", {}).get("color", {}).get("value")
        if color == "green" and float(detection["confidence"]) >= 0.50:
            return "green"
    return None

def scan_for_debris(robot: Robot, fov_limit_deg: float) -> tuple[float, float] | None:
    obstacles = robot.get_obstacles()
    if not obstacles: return None

    valid_points = []
    fov_rad = math.radians(fov_limit_deg)
    
    for ox, oy in obstacles:
        dist = math.hypot(ox, oy)
        angle = math.atan2(oy, ox)
        if DEBRIS_MIN_DIST_MM < dist < DEBRIS_MAX_DIST_MM and abs(angle) <= fov_rad:
            valid_points.append((ox, oy))

    if not valid_points: return None

    clusters = []
    for pt in valid_points:
        placed = False
        for cluster in clusters:
            if any(math.hypot(pt[0] - cp[0], pt[1] - cp[1]) < CLUSTER_TOLERANCE_MM for cp in cluster):
                cluster.append(pt)
                placed = True
                break
        if not placed:
            clusters.append([pt])

    centroids = []
    for cluster in clusters:
        cx = sum(p[0] for p in cluster) / len(cluster)
        cy = sum(p[1] for p in cluster) / len(cluster)
        centroids.append((cx, cy))

    best_target = None
    min_dist = float('inf')

    for i, (cx1, cy1) in enumerate(centroids):
        isolated = True
        for j, (cx2, cy2) in enumerate(centroids):
            if i != j and math.hypot(cx2 - cx1, cy2 - cy1) < ISOLATION_RADIUS_MM:
                isolated = False
                break
        
        if isolated:
            dist = math.hypot(cx1, cy1)
            if dist < min_dist:
                min_dist = dist
                best_target = (cx1, cy1)

    return best_target


def run(robot: Robot) -> None:
    state = "INIT"
    ik_solver = ArmKinematics3D()
    
    curr_shoulder = SHOULDER_REST_DEG
    curr_elbow = ELBOW_REST_DEG
    curr_gripper = GRIPPER_OPEN_DEG
    
    current_yaw_deg = 0.0
    last_move_direction = 1   
    
    target_dist_mm = 0.0
    target_yaw_deg = 0.0
    target_shoulder_deg = 0.0
    target_elbow_deg = 0.0
    
    idle_sleep_time = 1.0 / float(DEFAULT_FSM_HZ)

    while True:
        if state == "INIT":
            start_robot(robot)
            robot.enable_lidar()
            
            if hasattr(robot, 'start_lidar_world_publisher'):
                robot.start_lidar_world_publisher()
                
            if ENABLE_VISION: robot.enable_vision()
                
            robot.enable_servo(SHOULDER_SERVO)
            robot.enable_servo(ELBOW_SERVO)
            robot.enable_servo(GRIPPER_SERVO)
            robot.step_enable(BASE_STEPPER)
            
            stow_arm(robot)
            
            robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_DEG)
            curr_gripper = GRIPPER_OPEN_DEG
            time.sleep(2.0) 
            
            print("\n[FSM] READY. Press BTN_1 or show GREEN LIGHT to begin.")
            robot.set_led(LED.ORANGE, 200)
            robot.set_led(LED.GREEN, 0)
            state = "IDLE"

        elif state == "IDLE":
            button_pressed = robot.was_button_pressed(Button.BTN_1)
            green_light_seen = (find_traffic_light_color(robot) == "green")
            
            if button_pressed or green_light_seen:
                robot.set_led(LED.ORANGE, 0)
                robot.set_led(LED.GREEN, 200)
                print("\n[FSM] Triggered! Scanning for debris...")
                state = "SCAN_FOR_DEBRIS"
            else:
                time.sleep(idle_sleep_time)

        elif state == "SCAN_FOR_DEBRIS":
            target = scan_for_debris(robot, fov_limit_deg=SCAN_FOV_DEG)
            if target:
                tx, ty = target
                
                dx = tx - X_0   
                dy = ty - Y_0         
                
                target_yaw_deg = math.degrees(math.atan2(dy, dx))
                target_dist_mm = math.hypot(dx, dy)
                
                print(f"[PERCEPTION] Debris cluster found at LiDAR X:{tx:.1f}, Y:{ty:.1f}")
                
                try:
                    t2_raw, t3_raw = ik_solver.calculate_ik(r_target_mm=target_dist_mm, z_target_mm=PICK_Z_HEIGHT_MM)
                    target_shoulder_deg = t2_raw
                    target_elbow_deg = t3_raw + 180.0
                    
                    print(f"[IK] Transformed to Stepper-Relative R:{target_dist_mm:.1f}mm, Yaw:{target_yaw_deg:.1f}°")
                    print(f"[IK] Arm targets: Shoulder {target_shoulder_deg:.1f}°, Elbow {target_elbow_deg:.1f}°")
                    state = "ALIGN_BASE"
                except ValueError as e:
                    print(f"[WARNING] Debris spotted but unreachable: {e}. Rescanning...")
                    time.sleep(1.0)
            else:
                time.sleep(0.1)

        elif state == "ALIGN_BASE":
            delta_deg = target_yaw_deg - current_yaw_deg
            steps_to_move = int(delta_deg * (STEPS_PER_REV / 360.0))
            
            move_direction = 1 if steps_to_move >= 0 else -1
            
            if move_direction != last_move_direction and steps_to_move != 0:
                print(f"[STEPPER] Direction change! Adding {BACKLASH_STEPS} steps backlash compensation.")
                steps_to_command = steps_to_move + (move_direction * BACKLASH_STEPS)
            else:
                steps_to_command = steps_to_move
                
            last_move_direction = move_direction
            current_yaw_deg = target_yaw_deg  
            
            print(f"[STEPPER] Swiveling {delta_deg:.1f}° ({steps_to_command} steps) at {BASE_STEPPER_VEL} steps/s...")
            command_yaw_stepper(robot, steps_to_command)
            
            state = "REACH_ARM"

        elif state == "REACH_ARM":
            print(f"[ACTUATE] Reaching arm {target_dist_mm:.1f}mm outwards at {PICK_Z_HEIGHT_MM}mm height...")
            
            move_servos_slowly(robot, curr_shoulder, curr_elbow, target_shoulder_deg, target_elbow_deg)
            curr_shoulder = target_shoulder_deg
            curr_elbow = target_elbow_deg
            
            state = "GRAB_AND_MAST"

        elif state == "GRAB_AND_MAST":
            print("[ACTUATE] Gripping object slowly...")
            move_gripper_slowly(robot, curr_gripper, GRIPPER_CLOSE_DEG, step_deg=1.0, delay_s=0.03)
            curr_gripper = GRIPPER_CLOSE_DEG
            time.sleep(0.5)
            
            print(f"[ACTUATE] Pulling into vertical MAST position ({SHOULDER_MAST_DEG}/{ELBOW_MAST_DEG}) to reduce torque...")
            move_servos_slowly(robot, curr_shoulder, curr_elbow, SHOULDER_MAST_DEG, ELBOW_MAST_DEG)
            curr_shoulder, curr_elbow = SHOULDER_MAST_DEG, ELBOW_MAST_DEG
            
            state = "DISCARD_AND_STOW"

        elif state == "DISCARD_AND_STOW":
            discard_delta = DISCARD_ADDITIONAL_TURN_DEG * last_move_direction
            discard_steps = int(discard_delta * (STEPS_PER_REV / 360.0))
            
            print(f"[STEPPER] Swinging {discard_delta}° further to discard (Direction maintained).")
            command_yaw_stepper(robot, discard_steps)
            current_yaw_deg += discard_delta
            
            print("[ACTUATE] Lowering arm back to drop height...")
            move_servos_slowly(robot, curr_shoulder, curr_elbow, target_shoulder_deg, target_elbow_deg)
            curr_shoulder, curr_elbow = target_shoulder_deg, target_elbow_deg

            print("[ACTUATE] Releasing object slowly...")
            move_gripper_slowly(robot, curr_gripper, GRIPPER_OPEN_DEG, step_deg=1.0, delay_s=0.03)
            curr_gripper = GRIPPER_OPEN_DEG
            time.sleep(0.5)
            
            print("[ACTUATE] Retracting arm back to MAST to avoid collision...")
            move_servos_slowly(robot, curr_shoulder, curr_elbow, SHOULDER_MAST_DEG, ELBOW_MAST_DEG)
            curr_shoulder, curr_elbow = SHOULDER_MAST_DEG, ELBOW_MAST_DEG

            home_delta = 0.0 - current_yaw_deg
            home_steps = int(home_delta * (STEPS_PER_REV / 360.0))
            home_direction = 1 if home_steps >= 0 else -1
            
            if home_direction != last_move_direction and home_steps != 0:
                home_command = home_steps + (home_direction * BACKLASH_STEPS)
            else:
                home_command = home_steps
                
            last_move_direction = home_direction
            current_yaw_deg = 0.0
            
            print("[STEPPER] Returning to 0° Home...")
            command_yaw_stepper(robot, home_command)
            
            move_servos_slowly(robot, curr_shoulder, curr_elbow, SHOULDER_REST_DEG, ELBOW_REST_DEG)
            curr_shoulder, curr_elbow = SHOULDER_REST_DEG, ELBOW_REST_DEG
            
            print("\n[FSM] Sequence complete. Scanning for next object...")
            state = "SCAN_FOR_DEBRIS"