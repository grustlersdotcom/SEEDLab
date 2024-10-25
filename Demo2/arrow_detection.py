import cv2
import numpy as np

# Load left and right arrow templates
left_arrow_template = cv2.imread('left_arrow.png', cv2.IMREAD_GRAYSCALE)
right_arrow_template = cv2.imread('right_arrow.png', cv2.IMREAD_GRAYSCALE)

# Get template dimensions
left_w, left_h = left_arrow_template.shape[::-1]
right_w, right_h = right_arrow_template.shape[::-1]

def detect_arrows_with_template(frame):
    # Convert the frame to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Match left arrow template
    left_match = cv2.matchTemplate(gray_frame, left_arrow_template, cv2.TM_CCOEFF_NORMED)
    left_locations = np.where(left_match >= 0.8)  # Threshold for left arrow match quality

    # Draw bounding boxes for left arrow matches
    for pt in zip(*left_locations[::-1]):
        cv2.rectangle(frame, pt, (pt[0] + left_w, pt[1] + left_h), (0, 255, 0), 2)
        cv2.putText(frame, "Left Arrow", (pt[0], pt[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Match right arrow template
    right_match = cv2.matchTemplate(gray_frame, right_arrow_template, cv2.TM_CCOEFF_NORMED)
    right_locations = np.where(right_match >= 0.8)  # Threshold for right arrow match quality

    # Draw bounding boxes for right arrow matches
    for pt in zip(*right_locations[::-1]):
        cv2.rectangle(frame, pt, (pt[0] + right_w, pt[1] + right_h), (0, 0, 255), 2)
        cv2.putText(frame, "Right Arrow", (pt[0], pt[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    return frame

# Initialize camera
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera not available")
        break

    # Remove the flip; display the original camera feed
    # frame = cv2.flip(frame, 1)  # This line has been removed

    # Detect arrows using template matching
    processed_frame = detect_arrows_with_template(frame)

    # Display the result
    cv2.imshow('Arrow Detection', processed_frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
