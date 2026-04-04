import cv2
import time
import math
from enum import Enum
from dataclasses import dataclass
from typing import Tuple, Optional

class TurnType(Enum):
    """Different turning strategies for various scenarios"""
    POINT_TURN = "point"      # Zero-radius turn (spin in place)
    PIVOT_TURN = "pivot"      # One wheel stationary, other moves
    ARC_TURN = "arc"          # Both wheels move at different speeds
    GENTLE_ARC = "gentle"     # Wide arc for following


@dataclass
class TurnProfile:
    """Configuration for a specific turn behavior"""
    turn_type: TurnType
    speed_ratio: float        # Ratio between inner/outer wheel (0.0 to 1.0)
    base_speed: float         # Base speed for the turn (0.0 to 1.0)
    min_turn_angle: float     # Minimum angle threshold to trigger this turn


class ImprovedTurningController:
    """
    Enhanced turning controller with adaptive strategies.
    Provides smooth, context-aware turning for differential drive robots.
    """
    
    def __init__(self, 
                 wheel_base: float = 0.25,  # Distance between wheels in meters
                 max_speed: float = 1.0):
        """
        Initialize the turning controller
        
        Args:
            wheel_base: Distance between left and right wheels (meters)
            max_speed: Maximum motor speed (0.0 to 1.0)
        """
        self.wheel_base = wheel_base
        self.max_speed = max_speed
        
        # Define turn profiles for different scenarios
        self.profiles = {
            'precise': TurnProfile(TurnType.POINT_TURN, 0.0, 0.4, 45),
            'normal': TurnProfile(TurnType.ARC_TURN, 0.3, 0.6, 20),
            'gentle': TurnProfile(TurnType.GENTLE_ARC, 0.7, 0.8, 5),
            'obstacle': TurnProfile(TurnType.PIVOT_TURN, 0.0, 0.5, 30)
        }
        
        # Smoothing parameters
        self.last_left_speed = 0.0
        self.last_right_speed = 0.0
        self.smoothing_factor = 0.3  # Lower = smoother but slower response
        
    def calculate_turn_speeds(self, 
                            error_angle: float,
                            forward_speed: float = 0.0,
                            profile_name: str = 'normal') -> Tuple[float, float]:
        """
        Calculate motor speeds for turning based on angular error
        
        Args:
            error_angle: Desired turning angle in degrees (+ = turn right, - = turn left)
            forward_speed: Base forward velocity component (0.0 to 1.0)
            profile_name: Which turn profile to use ('precise', 'normal', 'gentle', 'obstacle')
            
        Returns:
            Tuple of (left_motor_speed, right_motor_speed)
        """
        # Get the appropriate turn profile
        profile = self.profiles.get(profile_name, self.profiles['normal'])
        
        # Normalize angle to -180 to 180
        error_angle = self._normalize_angle(error_angle)
        
        # Select turn type based on angle magnitude
        if abs(error_angle) < profile.min_turn_angle and forward_speed > 0:
            # Small correction - use gentle arc
            return self._gentle_arc_turn(error_angle, forward_speed)
        
        # Apply the selected turn strategy
        if profile.turn_type == TurnType.POINT_TURN:
            left_speed, right_speed = self._point_turn(error_angle, profile.base_speed)
        elif profile.turn_type == TurnType.PIVOT_TURN:
            left_speed, right_speed = self._pivot_turn(error_angle, profile.base_speed)
        elif profile.turn_type == TurnType.ARC_TURN:
            left_speed, right_speed = self._arc_turn(error_angle, forward_speed, 
                                                     profile.speed_ratio, profile.base_speed)
        else:  # GENTLE_ARC
            left_speed, right_speed = self._gentle_arc_turn(error_angle, forward_speed)
        
        # Apply smoothing to prevent jerky movements
        left_speed = self._smooth_speed(left_speed, self.last_left_speed)
        right_speed = self._smooth_speed(right_speed, self.last_right_speed)
        
        # Store for next iteration
        self.last_left_speed = left_speed
        self.last_right_speed = right_speed
        
        # Clamp to valid range
        left_speed = max(min(left_speed, self.max_speed), -self.max_speed)
        right_speed = max(min(right_speed, self.max_speed), -self.max_speed)
        
        return left_speed, right_speed
    
    def _point_turn(self, angle: float, speed: float) -> Tuple[float, float]:
        """Point turn: Rotate in place (zero turning radius)"""
        direction = 1 if angle > 0 else -1
        return -direction * speed, direction * speed
    
    def _pivot_turn(self, angle: float, speed: float) -> Tuple[float, float]:
        """Pivot turn: One wheel stationary, other wheel moves"""
        direction = 1 if angle > 0 else -1
        if direction > 0:  # Turn right
            return 0.0, speed
        else:  # Turn left
            return speed, 0.0
    
    def _arc_turn(self, angle: float, forward_speed: float, 
                  speed_ratio: float, base_speed: float) -> Tuple[float, float]:
        """Arc turn: Both wheels move forward but at different speeds"""
        # Calculate turn intensity (0 to 1) based on angle
        turn_intensity = min(abs(angle) / 90.0, 1.0)
        
        # Determine outer and inner wheel speeds
        outer_speed = base_speed + (forward_speed * (1 - turn_intensity))
        inner_speed = outer_speed * (1 - turn_intensity * (1 - speed_ratio))
        
        if angle > 0:  # Turn right
            return inner_speed, outer_speed
        else:  # Turn left
            return outer_speed, inner_speed
    
    def _gentle_arc_turn(self, angle: float, forward_speed: float) -> Tuple[float, float]:
        """Gentle arc: Minimal speed differential for smooth path following"""
        # Very gradual turn adjustment
        turn_factor = angle / 180.0  # Normalized to -1 to 1
        adjustment = turn_factor * 0.3  # Max 30% speed differential
        
        left_speed = forward_speed * (1 - adjustment)
        right_speed = forward_speed * (1 + adjustment)
        
        return left_speed, right_speed
    
    def _smooth_speed(self, target: float, current: float) -> float:
        """Apply exponential smoothing to prevent jerky movements"""
        return current + self.smoothing_factor * (target - current)
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to -180 to 180 degrees"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def calculate_turn_radius(self, left_speed: float, right_speed: float) -> Optional[float]:
        """Calculate the turning radius given wheel speeds (in meters)"""
        if abs(left_speed - right_speed) < 0.01:
            return None  # Going straight
        
        if abs(left_speed + right_speed) < 0.01:
            return 0.0  # Point turn
        
        # Calculate radius using differential drive kinematics
        avg_speed = (left_speed + right_speed) / 2
        speed_diff = right_speed - left_speed
        
        radius = (self.wheel_base / 2) * (avg_speed / speed_diff)
        return abs(radius)
    
    def reset_smoothing(self):
        """Reset smoothing state (call when changing direction dramatically)"""
        self.last_left_speed = 0.0
        self.last_right_speed = 0.0


# ==========================================
# 1. CONFIGURATION & MODE SETUP
# ==========================================

SIMULATION_MODE = True  # Set to False when running on the actual Raspberry Pi

if not SIMULATION_MODE:
    from gpiozero import Motor
    import serial
    
    # Initialize Motors
    left_motor = Motor(forward=17, backward=18)
    right_motor = Motor(forward=22, backward=23)
    
    # Initialize TF-Luna LiDAR via UART (Serial0 on Pi)
    try:
        lidar = serial.Serial("/dev/serial0", 115200, timeout=0)
    except Exception as e:
        print(f"LiDAR Error: {e}")
        lidar = None
else:
    print("=" * 60)
    print("   AGROBOT - IMPROVED TURNING ALGORITHM")
    print("   Running in PC Simulation Mode")
    print("=" * 60)
    print("\nIMPROVEMENTS:")
    print("  ✓ Multiple turn types (point, pivot, arc, gentle)")
    print("  ✓ Adaptive profile selection")
    print("  ✓ Motion smoothing for stability")
    print("  ✓ Proper differential drive kinematics")
    print("\nPress 'q' to quit\n")
    lidar = None

# Open Webcam (0 is usually the default laptop/Pi camera)
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# ==========================================
# 2. ALGORITHM VARIABLES
# ==========================================

# OpenCV ArUco Setup
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

CENTER_X = 320
TARGET_AREA = 15000

# IMPROVED: Initialize the turning controller
# This replaces the simple Kp_rot PID gain
turn_controller = ImprovedTurningController(wheel_base=0.25, max_speed=1.0)

# Keep distance control PID gain
Kp_dist = 0.0001

state = "SEARCH"
last_known_x = CENTER_X
time_target_lost = 0

# ==========================================
# 3. HELPER FUNCTIONS
# ==========================================

def read_tfluna_distance(ser):
    """Reads distance from TF-Luna in meters. Returns 9.99 if safe/no reading."""
    if ser is None or SIMULATION_MODE: 
        return 9.99
    try:
        if ser.in_waiting > 8:
            bytes_serial = ser.read(9)
            # TF-Luna frame always starts with 0x59 0x59
            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
                distance_cm = bytes_serial[2] + bytes_serial[3] * 256
                return distance_cm / 100.0  # Convert to meters
    except:
        pass
    return 9.99


def execute_motors(ls, rs, action_text, frame, turn_radius=None):
    """Sends commands to motors or displays them on screen in simulation."""
    if SIMULATION_MODE:
        # Draw status on the computer screen
        cv2.putText(frame, f"ACTION: {action_text}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, f"L_Motor: {ls:+.2f} | R_Motor: {rs:+.2f}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Show turn radius if available
        if turn_radius is not None:
            if turn_radius == 0:
                radius_text = "Turn: Point (0.0m)"
            elif turn_radius < 10:
                radius_text = f"Turn Radius: {turn_radius:.2f}m"
            else:
                radius_text = "Turn: Straight"
            cv2.putText(frame, radius_text, (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
    else:
        # Actually drive the hardware
        if ls > 0: 
            left_motor.forward(ls)
        else: 
            left_motor.backward(abs(ls))
        if rs > 0: 
            right_motor.forward(rs)
        else: 
            right_motor.backward(abs(rs))


# ==========================================
# 4. MAIN CONTROL LOOP
# ==========================================

try:
    while True:
        ret, frame = camera.read()
        if not ret: 
            continue
        
        # Detect Marker
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        
        # Read LiDAR
        front_distance = read_tfluna_distance(lidar)
        
        # --- STATE MACHINE LOGIC ---
        if front_distance < 0.20:
            state = "AVOID"
        elif ids is not None and 0 in ids:
            state = "FOLLOW"
        else:
            if state == "FOLLOW":
                state = "LOST_RECENTLY"
                time_target_lost = time.time()
            elif state == "LOST_RECENTLY" and (time.time() - time_target_lost > 3):
                state = "SEARCH"
        
        # --- VISUALIZE STATE IN SIMULATION ---
        if SIMULATION_MODE:
            cv2.putText(frame, f"STATE: {state}", (10, 450),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # --- EXECUTE STATE ACTIONS ---
        if state == "AVOID":
            # IMPROVED: Use obstacle avoidance turn profile
            ls, rs = turn_controller.calculate_turn_speeds(
                error_angle=-90,  # Turn left to avoid
                forward_speed=0.0,
                profile_name='obstacle'
            )
            # Add backward component for backing away
            ls -= 0.3
            rs -= 0.3
            
            turn_radius = turn_controller.calculate_turn_radius(ls, rs)
            execute_motors(ls, rs, "AVOIDING WALL (Improved Turn)", frame, turn_radius)
            
        elif state == "FOLLOW":
            idx = list(ids.flatten()).index(0)
            marker_corners = corners[idx][0]
            
            # Draw box around marker in simulation
            if SIMULATION_MODE:
                cv2.polylines(frame, [marker_corners.astype(int)], True, (0, 255, 255), 2)
            
            x_coords = [pt[0] for pt in marker_corners]
            y_coords = [pt[1] for pt in marker_corners]
            cx = sum(x_coords) / 4
            area = (max(x_coords) - min(x_coords)) * (max(y_coords) - min(y_coords))
            
            last_known_x = cx
            
            # IMPROVED: Calculate turning angle from pixel error
            # Positive error_x = target is right of center
            error_x = cx - CENTER_X
            
            # Convert pixel error to approximate angle (calibrate based on camera FOV)
            # Assuming ~60° horizontal FOV and 640px width
            # This means each pixel represents ~0.094 degrees
            error_angle = (error_x / 640) * 60  
            
            # Calculate forward/backward speed from area error
            error_dist = TARGET_AREA - area
            forward_speed = error_dist * Kp_dist
            forward_speed = max(min(forward_speed, 0.8), -0.8)  # Clamp
            
            # IMPROVED: Use the turning controller with appropriate profile
            # Select profile based on angle magnitude
            if abs(error_angle) > 30:
                profile = 'normal'  # Moderate turn for larger corrections
            else:
                profile = 'gentle'  # Small correction for tracking
            
            ls, rs = turn_controller.calculate_turn_speeds(
                error_angle=error_angle,
                forward_speed=forward_speed,
                profile_name=profile
            )
            
            # Determine action text for display
            if forward_speed > 0.1:
                action = "FORWARD"
            elif forward_speed < -0.1:
                action = "BACKWARD"
            else:
                action = "MAINTAINING DISTANCE"
                
            if abs(error_angle) > 5:
                direction = "RIGHT" if error_angle > 0 else "LEFT"
                action += f" + TURNING {direction}"
            
            turn_radius = turn_controller.calculate_turn_radius(ls, rs)
            execute_motors(ls, rs, action, frame, turn_radius)
            
        elif state == "LOST_RECENTLY":
            # IMPROVED: Use normal turn profile for searching
            # Search in the direction where target was last seen
            if last_known_x < CENTER_X:
                angle = -45  # Search left
                direction = "LEFT"
            else:
                angle = 45   # Search right
                direction = "RIGHT"
                
            ls, rs = turn_controller.calculate_turn_speeds(
                error_angle=angle,
                forward_speed=0.0,
                profile_name='normal'
            )
            
            turn_radius = turn_controller.calculate_turn_radius(ls, rs)
            execute_motors(ls, rs, f"SEARCHING {direction} (Memory)", frame, turn_radius)
            
        elif state == "SEARCH":
            execute_motors(0.0, 0.0, "STOPPED - WAITING FOR TARGET", frame, None)
        
        # Show the video window if in simulation
        if SIMULATION_MODE:
            cv2.imshow("Agrobot - Improved Turning Algorithm", frame)
            
            # Press 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

except KeyboardInterrupt:
    print("\nShutting down safely...")
    
finally:
    if not SIMULATION_MODE:
        left_motor.stop()
        right_motor.stop()
        if lidar: 
            lidar.close()
    camera.release()
    cv2.destroyAllWindows()
    print("\nAgrobot stopped successfully!")

