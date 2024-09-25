import cv2
import numpy as np
from smbus2 import SMBus as i2c
from time import sleep
import queue
import threading
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as lcd

# Define arduino i2c bus
ARD_ADDR = 8

# Define screen size (you can set this based on your display resolution)
SCREEN_WIDTH = 640
SCREEN_HEIGHT = 480

# Initialize the ARUCO dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# Initialize the ARUCO detector parameters
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Define quadrant labels
def get_quadrant(x, y):
    if x < SCREEN_WIDTH // 2 and y < SCREEN_HEIGHT // 2:
        return "NW"  # North-West
    elif x >= SCREEN_WIDTH // 2 and y < SCREEN_HEIGHT // 2:
        return "NE"  # North-East
    elif x < SCREEN_WIDTH // 2 and y >= SCREEN_HEIGHT // 2:
        return "SW"  # South-West
    else:
        return "SE"  # South-East

# Function to draw quadrants
def draw_quadrants(frame):
    # Vertical and horizontal midpoint lines
    cv2.line(frame, (SCREEN_WIDTH // 2, 0), (SCREEN_WIDTH // 2, SCREEN_HEIGHT), (0, 255, 0), 2)
    cv2.line(frame, (0, SCREEN_HEIGHT // 2), (SCREEN_WIDTH, SCREEN_HEIGHT // 2), (0, 255, 0), 2)

q = queue.Queue()

# Function to send to Arduino over i2c
def write_to_i2c():
    coordDict = {"NE": [1, "[0, 0]"],"NW": [2, "[0, 1]"],"SW": [3, "[1, 1]"], "SE": [4, "[1, 0]"]}
    current = ""
    while True:
        if not q.empty():
            print("Here")
            dictResult = coordDict[q.get()]
            if current == "":
                current = dictResult
            elif dictResult != current:
                current = dictResult
                quadrant = dictResult[0]
                #Send info to arduino
                with i2c(1) as bus:
                    bus.write_byte_data(ARD_ADDR,0,quadrant)
                #Send info to LCD Display
                
                with board.I2C() as bus:
                    rows = 2
                    columns = 16
                    display = lcd.Character_LCD_RGB_I2C(bus,columns,rows)
                    display.clear()
                    display.color = [100,0,0]
                    display.message = "Desired location\n" + dictResult[1]
    
busThread = threading.Thread(target = write_to_i2c,args=())
busThread.start()
# Capture video from webcam
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, SCREEN_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, SCREEN_HEIGHT)

while True:
    ret, frame = cap.read()  # Capture each frame from the webcam
    if not ret:
        break

    # Convert the captured frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers on the grayscale image
    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is not None:
        # Draw markers on the original (color) frame for better visualization
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Iterate over detected markers
        for corner, marker_id in zip(corners, ids):
            # Get the center point of the marker
            center_x = int(corner[0][:, 0].mean())
            center_y = int(corner[0][:, 1].mean())

            # Determine the quadrant
            quadrant = get_quadrant(center_x, center_y)
            q.put(quadrant)
            
            
                

            # Print marker ID and quadrant information to console
            print(f"Marker ID: {marker_id[0]}, Quadrant: {quadrant}")

            # Draw the center point and display the quadrant on the color frame
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

            # Display marker ID and quadrant information on frame
            text = f"ID: {marker_id[0]}, Quadrant: {quadrant}"
            cv2.putText(frame, text, (center_x + 10, center_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Draw quadrant lines on the color frame
    draw_quadrants(frame)

    # Display the color frame with quadrant lines and detected markers
    cv2.imshow("ArUco Marker Detection with Quadrants", frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close the windows
cap.release()
cv2.destroyAllWindows()
