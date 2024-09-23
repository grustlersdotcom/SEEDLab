import cv2
import numpy as np

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
