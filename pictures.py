import cv2
import os
from datetime import datetime

# Create a directory to store captured images
output_dir = 'captured_images'
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Initialize the webcam (default camera is index 0)
cap = cv2.VideoCapture(0)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

print("Press spacebar to capture an image. Press 'q' to quit.")

while True:
    # Capture frame-by-frame from the webcam
    ret, frame = cap.read()
    
    # If frame was captured successfully
    if ret:
        # Display the live frame
        cv2.imshow('Press Spacebar to Capture', frame)
        
        # Wait for a key press for 1ms
        key = cv2.waitKey(1) & 0xFF
        
        # Check if the spacebar (ASCII code 32) is pressed
        if key == 32:
            # Generate a unique file name using the current timestamp
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            file_path = os.path.join(output_dir, f'photo_{timestamp}.png')
            
            # Save the captured frame as an image
            cv2.imwrite(file_path, frame)
            print(f'Image saved: {file_path}')
        
        # Break the loop if 'q' is pressed to quit
        elif key == ord('q'):
            print("Quitting program.")
            break

# Release the webcam and close all windows
cap.release()
cv2.destroyAllWindows()
