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
        focal_length = frame.shape[1] / (2 * math.tan(math.radians(48.43 / 2)))  # Assuming 60-degree FOV
        angle = math.degrees(math.atan2(dx, focal_length))
        
        return angle, marker_percentage
    
    return None, None

def detect_arrow(frame):
    # Convert the frame to grayscale and use edge detection to find arrow contours
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray_frame, (5, 5), 0)
    edged = cv2.Canny(blurred, 50, 150)
    
    # Find contours in the entire frame
    contours, _ = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        # Approximate the contour to identify arrow-like shapes
        approx = cv2.approxPolyDP(cnt, 0.05 * cv2.arcLength(cnt, True), True)
        
        # Arrow should have around 7 vertices (triangle + tail shape)
        if len(approx) == 7:
            # Calculate the bounding rectangle of the contour
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = w / float(h)

            if 0.5 < aspect_ratio < 2.0:  # Filtering to get arrows with balanced width and height
                # Determine direction based on the relative position of points
                if approx[0][0][0] < approx[3][0][0]:  # Check direction by comparing specific points
                    color = (0, 255, 0)  # Green for left arrow
                    direction_text = "Left Arrow"
                else:
                    color = (0, 0, 255)  # Red for right arrow
                    direction_text = "Right Arrow"
                
                # Draw detected arrow and label it
                cv2.drawContours(frame, [approx], -1, color, 3)
                cv2.putText(frame, direction_text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                return direction_text

    # Return "No Arrow Detected" if no arrow is found
    return "No Arrow Detected"

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
        
        # Detect arrow direction in the whole frame
        direction = detect_arrow(frame)
        print(f"Detected Arrow Direction: {direction}")
        cv2.putText(frame, direction, (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

    else:
        # Display "No ArUco Marker Detected" if no ArUco markers are found in the frame
        cv2.putText(frame, "No ArUco Marker Detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    cv2.imshow('ArUco Marker and Arrow Detection', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
