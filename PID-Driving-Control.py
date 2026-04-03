import cv2
import time
import math

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
    print("--- RUNNING IN PC SIMULATION MODE ---")
    lidar = None

# Open Webcam (0 is usually the default laptop/Pi camera)
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# ==========================================
# 2. ALGORITHM VARIABLES
# ==========================================
# OpenCV 4.7+ ArUco Setup
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

CENTER_X = 320         
TARGET_AREA = 15000    
Kp_rot = 0.003         
Kp_dist = 0.0001       

state = "SEARCH"
last_known_x = CENTER_X 
time_target_lost = 0

# ==========================================
# 3. HELPER FUNCTIONS
# ==========================================
def read_tfluna_distance(ser):
    """Reads distance from TF-Luna in meters. Returns 9.99 if safe/no reading."""
    if ser is None or SIMULATION_MODE: return 9.99 
    try:
        if ser.in_waiting > 8:
            bytes_serial = ser.read(9)
            # TF-Luna frame always starts with 0x59 0x59
            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
                distance_cm = bytes_serial[2] + bytes_serial[3] * 256
                return distance_cm / 100.0 # Convert to meters
    except:
        pass
    return 9.99

def execute_motors(ls, rs, action_text, frame):
    """Sends commands to motors or displays them on screen in simulation."""
    if SIMULATION_MODE:
        # Draw status on the computer screen
        cv2.putText(frame, f"ACTION: {action_text}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, f"L_Motor: {ls:.2f} | R_Motor: {rs:.2f}", (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    else:
        # Actually drive the hardware
        if ls > 0: left_motor.forward(ls) 
        else: left_motor.backward(abs(ls))
        
        if rs > 0: right_motor.forward(rs)
        else: right_motor.backward(abs(rs))

# ==========================================
# 4. MAIN CONTROL LOOP
# ==========================================
try:
    while True:
        ret, frame = camera.read()
        if not ret: continue
        
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
            execute_motors(-0.5, 0.5, "AVOIDING WALL (Back & Turn)", frame)
            
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
            
            # PID Proportional Error Calculation
            error_x = CENTER_X - cx
            error_dist = TARGET_AREA - area
            
            rot_adj = error_x * Kp_rot
            dist_adj = error_dist * Kp_dist
            
            # Mix speeds and clamp (-1.0 to 1.0)
            ls = max(min(dist_adj - rot_adj, 1.0), -1.0)
            rs = max(min(dist_adj + rot_adj, 1.0), -1.0)
            
            # Determine text for simulation
            if ls > 0 and rs > 0: action = "MOVING FORWARD"
            elif ls < 0 and rs < 0: action = "MOVING BACKWARD"
            elif ls < rs: action = "TURNING LEFT"
            else: action = "TURNING RIGHT"
            
            execute_motors(ls, rs, action, frame)
            
        elif state == "LOST_RECENTLY":
            if last_known_x < CENTER_X:
                execute_motors(-0.4, 0.4, "SEARCHING LEFT (Memory)", frame)
            else:
                execute_motors(0.4, -0.4, "SEARCHING RIGHT (Memory)", frame)
                
        elif state == "SEARCH":
            execute_motors(0.0, 0.0, "STOPPED - WAITING FOR TARGET", frame)

        # Show the video window if in simulation
        if SIMULATION_MODE:
            cv2.imshow("Robot Follow-Me Simulation", frame)
            # Press 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

except KeyboardInterrupt:
    print("\nShutting down safely...")

finally:
    if not SIMULATION_MODE:
        left_motor.stop()
        right_motor.stop()
        if lidar: lidar.close()
    camera.release()
    cv2.destroyAllWindows()
