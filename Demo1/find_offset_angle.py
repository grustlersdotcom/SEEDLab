import cv2
import numpy as np
import math

# Define the ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()

detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Stolen from calibration script
mtx = np.array([[667.2844,0,342.1226],
               [0,  666.966,233.012],
               [0,  0,     1]])

dist = np.array([[0.149,-1.001,-0.002,0.003,1.576]])

newcameramtx = np.array([[664.206,0,343.452],
                        [0,663.913,232.271],
                        [0,0,1]])

def detect_aruco_marker(frame):
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # undistort the imate
    gray = cv2.undistort(gray,mtx,dist,None,newcameramtx)
    
    # Detect ArUco markers
    corners, ids, _ = detector.detectMarkers(gray)
    
    if ids is not None and len(ids) > 0:
        # Get the center of the first detected marker
        center = np.mean(corners[0][0], axis=0)
        
        # Calculate angle
        image_center_x = frame.shape[1] / 2
        dx = center[0] - image_center_x
        focal_length = frame.shape[1] / (2 * math.tan(math.radians(68.5 / 2)))  # Using 68.5-degree FOV
        angle = math.degrees(math.atan2(dx, focal_length))
        
        return angle
    
    return None

# Initialize camera
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    angle = detect_aruco_marker(frame)
    
    if angle is not None:
        print(f"Angle: {angle:.2f} degrees")
        cv2.putText(frame, f"Angle: {angle:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    cv2.imshow('ArUco Marker Detection', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
