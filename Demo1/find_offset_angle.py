#Name: Find Offset Angle
#Authors: Brendan and Noah
#Description: This script is responsible for finding the offset angle from the front facing vector of the camera, it uses the new camera matrix generated from our calibration script.
#Notes: Write to LCD logic taken from Assignment 2-1B. Queue logic and threading taken from mini-project. Camera instantiation and operation taken from several previous assignments.
import cv2
import numpy as np
import math
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as lcd
import queue
import threading

# Define the ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()

detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Stolen from calibration script
mtx = np.array([[667.2844,0,342.1226],
               [0,  666.966,233.012],
               [0,  0,     1]])
#print("We made it")
dist = np.array([[0.149,-1.001,-0.002,0.003,1.576]])

newcameramtx = np.array([[664.206,0,343.452],
                        [0,663.913,232.271],
                        [0,0,1]])

q = queue.Queue()
currentAngle = 0
def detect_aruco_marker(frame):
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # undistort the image
    #gray = cv2.undistort(gray,mtx,dist,None,newcameramtx)
    
    # Detect ArUco markers
    corners, ids, _ = detector.detectMarkers(gray)
    
    if ids is not None and len(ids) > 0:
        # Get the center of the first detected marker
        center = np.mean(corners[0][0], axis=0)
        
        # Calculate angle
        image_center_x = frame.shape[1] / 2
        dx = center[0] - image_center_x
<<<<<<< HEAD
        focal_length = frame.shape[1] / (2 * math.tan(math.radians(68.5 / 2)))  # Using 68.5-degree FOV
        angle = math.degrees(math.atan2(dx, focal_length))
=======
        focal_length = frame.shape[1] / (2 * math.tan(math.radians(60 / 2)))
        #Assuming 60-degree FOV
        angle = round(math.degrees(math.atan2(dx, focal_length)),2)
>>>>>>> deed2df9cea48a64a1a60d2072d0c922d5dbff30
        
        return angle
    
    return None
#Communicate with the lcd display
def write_lcd():
    bus = board.I2C()
    rows = 2
    columns = 16
    display = lcd.Character_LCD_RGB_I2C(bus,columns,rows)
                    #display.clear()
    display.color = [100,100,100]
                    
    while(True):
        if not q.empty():
            print("LCD Updated")
            #with board.I2C() as bus:
            display.message = str(q.get())
    return

lcdThread = threading.Thread(target = write_lcd, args=())
lcdThread.start()
# Initialize camera
cap = cv2.VideoCapture(0)
#print(cap.isOpened())

while True:
    ret, frame = cap.read()
    #print(ret)
    if not ret:
        print("Broken")
        break
    
    angle = detect_aruco_marker(frame)
    if angle!=None and (currentAngle+0.2<angle or currentAngle-0.2>angle):
        currentAngle=angle
        q.put(angle)
    if angle is not None:
        #print(f"Angle: {angle:.2f} degrees")
        cv2.putText(frame, f"Angle: {angle:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    cv2.imshow('ArUco Marker Detection', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
