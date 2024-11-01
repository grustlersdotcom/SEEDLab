import cv2
import numpy as np
import math
import queue
import threading
import time

# TODO later we need to add a thread that will reset the turn_commanded flag

arrow_check = True  # Set this to true in the second demonstration
stop_flag = False
turn_commanded = False

# Define the ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()

detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Camera matrix and distortion coefficients (from calibration)
mtx = np.array([[667.2844, 0, 342.1226],
                [0, 666.966, 233.012],
                [0, 0, 1]])
dist = np.array([[0.149, -1.001, -0.002, 0.003, 1.576]])

newcameramtx = np.array([[664.206, 0, 343.452],
                         [0, 663.913, 232.271],
                         [0, 0, 1]])

currentAngle = 100  # This is a placeholder because we know this can't be true

# Function to send the current angle over the I2C bus every 0.25 seconds
def send_angle_over_i2c():
    while True:
        if currentAngle is not None:
            angle_data = f"{currentAngle:.2f}"
            # Placeholder for I2C send code
            # print(f"Sending angle over I2C: {angle_data}") # TODO fix rq
        
        time.sleep(0.25)

# Start the I2C sending thread
i2c_thread = threading.Thread(target=send_angle_over_i2c)
i2c_thread.start()

def detect_aruco_marker(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    
    if ids is not None and len(ids) > 0:
        center = np.mean(corners[0][0], axis=0)
        image_center_x = frame.shape[1] / 2
        dx = center[0] - image_center_x
        focal_length = frame.shape[1] / (2 * math.tan(math.radians(48.4 / 2)))
        angle = round(math.degrees(math.atan2(dx, focal_length)), 2)
        
        marker_area = cv2.contourArea(np.array(corners[0][0], dtype=np.int32))
        frame_area = frame.shape[0] * frame.shape[1]
        percentage_visible = (marker_area / frame_area) * 100

        return angle, percentage_visible
    
    return None, None

stop_queue = queue.Queue()

def process_visible_percentage(percentage_visible):
    global stop_flag
    if stop_flag:
        return
    
    cutoff_percentage = 20  # Define a threshold for visibility
    if percentage_visible > cutoff_percentage:
        stop_queue.put(1)
        stop_flag = True
        print("STOP FLAG RAISED")
    else:
        return
    return

def process_green_percentage(frame):
    lower_green = np.array([35, 100, 100])
    upper_green = np.array([85, 255, 255])
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    green_area = cv2.countNonZero(green_mask)
    total_area = frame.shape[0] * frame.shape[1]
    green_percentage = (green_area / total_area) * 100
    
    return green_percentage

def process_red_percentage(frame):
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.add(red_mask1, red_mask2)
    
    red_area = cv2.countNonZero(red_mask)
    total_area = frame.shape[0] * frame.shape[1]
    red_percentage = (red_area / total_area) * 100
    
    return red_percentage

# Queue for I2C messages
i2c_flag_queue = queue.Queue()

def compare_green_red_percentage(green_percentage, red_percentage):
    margin_of_error = 5.0  # Define an acceptable margin of error between green and red percentages
    global turn_commanded

    if turn_commanded:
        return

    # Determine which color percentage is higher, accounting for margin of error
    if (green_percentage - red_percentage) > margin_of_error:
        higher_color = "Green"
    elif (red_percentage - green_percentage) > margin_of_error:
        higher_color = "Red"
    else:
        # If the difference is within the margin of error, do not raise any flags
        return

    # Raise the flag only if stop_flag is True
    if stop_flag:
        flags = {"Red": "RED", "Green": "GREEN"}
        # Raise the flag on the I2C bus by putting it into the queue
        i2c_flag_queue.put(flags[higher_color])
        turn_commanded = True  # Set the flag to indicate a command has been issued


# I2C flag handling function to be used in a separate thread
def handle_red_green_flag():
    global turn_commanded
    while True:
        if not i2c_flag_queue.empty():
            flag = i2c_flag_queue.get()
            if flag == "RED":
                # TODO Send the RED flag over I2C (or perform appropriate action)
                print("I2C FLAG RAISED: Red percentage is within margin.")
            elif flag == "GREEN":
                # TODO Send the GREEN flag over I2C (or perform appropriate action)
                print("I2C FLAG RAISED: Green percentage is within margin.")
        time.sleep(0.1)

# Start the updated I2C flag handling thread
i2c_flag_thread = threading.Thread(target=handle_red_green_flag)
i2c_flag_thread.start()

# Initialize camera
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera not available")
        break
    
    angle, percentage_visible = detect_aruco_marker(frame)
    
    # Check if the angle has changed significantly and update currentAngle
    if angle is not None and (currentAngle + 0.2 < angle or currentAngle - 0.2 > angle):
        currentAngle = angle
    
    # Process visibility percentage if ArUco marker is detected
    if percentage_visible is not None:
        process_visible_percentage(percentage_visible)
    
    # Only process green and red percentages if stop_flag is True
    if stop_flag and not turn_commanded:
        # Process and display green and red percentages on the frame
        green_percentage = process_green_percentage(frame)
        red_percentage = process_red_percentage(frame)
        
        # Compare green and red percentages
        compare_green_red_percentage(green_percentage, red_percentage)
        
        # Display information on the frame
        cv2.putText(frame, f"Green: {green_percentage:.2f}%", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Red: {red_percentage:.2f}%", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    if angle is not None:
        cv2.putText(frame, f"Angle: {angle:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Visible: {percentage_visible:.2f}%", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    cv2.imshow('ArUco Marker Detection', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

#TODO all we need to do is check our calibration, check how fast we can spin, and find the margin of error on the comparison,
# and then send the flags to the I2C bus in the best way we can.
