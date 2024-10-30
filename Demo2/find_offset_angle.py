# Add imports for additional functionality
import cv2
import numpy as np
import math
# import board
# import adafruit_character_lcd.character_lcd_rgb_i2c as lcd
import queue
import threading

# Define the ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()

detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Camera matrix and distortion coefficients (from calibration)
mtx = np.array([[667.2844,0,342.1226],
               [0,  666.966,233.012],
               [0,  0,     1]])
dist = np.array([[0.149,-1.001,-0.002,0.003,1.576]])

newcameramtx = np.array([[664.206,0,343.452],
                        [0,663.913,232.271],
                        [0,0,1]])

q = queue.Queue()
currentAngle = 0
def detect_aruco_marker(frame):
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect ArUco markers
    corners, ids, _ = detector.detectMarkers(gray)
    
    if ids is not None and len(ids) > 0:
        # Get the center of the first detected marker
        center = np.mean(corners[0][0], axis=0)
        
        # Calculate angle
        image_center_x = frame.shape[1] / 2
        dx = center[0] - image_center_x
        focal_length = frame.shape[1] / (2 * math.tan(math.radians(48.4 / 2)))
        angle = round(math.degrees(math.atan2(dx, focal_length)), 2)
        
        # Calculate marker area and percentage
        marker_area = cv2.contourArea(np.array(corners[0][0], dtype=np.int32))
        frame_area = frame.shape[0] * frame.shape[1]
        percentage_visible = (marker_area / frame_area) * 100

        return angle, percentage_visible
    
    return None, None

# Communicate with the LCD display
# def write_lcd():
#     bus = board.I2C()
#     rows = 2
#     columns = 16
#     display = lcd.Character_LCD_RGB_I2C(bus, columns, rows)
#     display.color = [100, 100, 100]
    
#     while True:
#         if not q.empty():
#             message = q.get()``
#             display.message = message
#     return

# lcdThread = threading.Thread(target=write_lcd, args=())
# lcdThread.start()

def process_visible_percentage(percentage_visible):
    cutoff_percentage = 20 #TODO this will update based on the function that we find
    def estimate_distance_to_marker(percentage_visible):
        #TODO find the funciton that relates percentage visible to distance
        return
    
    if percentage_visible > cutoff_percentage:
        return
    
    return

# Initialize camera
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera not available")
        break
    
    angle, percentage_visible = detect_aruco_marker(frame)

    #TODO add the processing of the visible percentage call here
    if angle is not None and (currentAngle + 0.2 < angle or currentAngle - 0.2 > angle):
        currentAngle = angle
        q.put(f"Angle: {angle}\nVisibility: {percentage_visible:.2f}%")
    
    if angle is not None:
        cv2.putText(frame, f"Angle: {angle:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Visible: {percentage_visible:.2f}%", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
    
    cv2.imshow('ArUco Marker Detection', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
