import cv2
import numpy as np
import math

# Define the ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()

detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

def detect_aruco_marker(frame):
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect ArUco markers
    corners, ids, _ = detector.detectMarkers(gray)
    
    if ids is not None and len(ids) > 0:
        # Calculate area of the first detected marker
        marker_area = cv2.contourArea(corners[0])
        frame_area = frame.shape[0] * frame.shape[1]
        
        # Calculate percentage of frame area occupied by marker
        marker_percentage = (marker_area / frame_area) * 100
        
        # Get the center of the first detected marker
        center = np.mean(corners[0][0], axis=0)
        
        # Calculate angle
        image_center_x = frame.shape[1] / 2
        dx = center[0] - image_center_x
        focal_length = frame.shape[1] / (2 * math.tan(math.radians(48.4 / 2)))  # Assuming 60-degree FOV
        angle = math.degrees(math.atan2(dx, focal_length))
        
        return angle, marker_percentage
    
    return None, None

# Initialize camera
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    angle, marker_percentage = detect_aruco_marker(frame)
    
    if angle is not None:
        print(f"Angle: {angle:.2f} degrees")
        cv2.putText(frame, f"Angle: {angle:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Check if marker occupies more than 30% of the screen
        if marker_percentage > 30:
            warning_text = "TOO CLOSE!"
            print(warning_text)
            cv2.putText(frame, warning_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    cv2.imshow('ArUco Marker Detection', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
