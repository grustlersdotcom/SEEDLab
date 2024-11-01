# ArUco Marker Detection with Color Comparison

## Overview

This script uses OpenCV to detect ArUco markers and analyze color visibility percentages in a camera feed. It communicates results via an I2C bus based on the detected conditions.

## Script Flow

1. **Initialization**:
   - Import necessary libraries: `cv2`, `numpy`, `math`, `queue`, `threading`, and `time`.
   - Define global flags: `arrow_check`, `stop_flag`, and `turn_commanded`.
   - Set up the ArUco dictionary and parameters for detection.

2. **Camera Calibration**:
   - Define the camera matrix (`mtx`) and distortion coefficients (`dist`) obtained from calibration.
   - Create a new camera matrix for undistortion.

3. **Thread for I2C Communication**:
   - Define a function `send_angle_over_i2c()` that sends the current angle over I2C every 0.25 seconds.
   - Start a separate thread to handle I2C communication.

4. **Aruco Marker Detection**:
   - Define the `detect_aruco_marker(frame)` function:
     - Convert the frame to grayscale and detect markers.
     - Calculate the center of the detected marker.
     - Compute the angle based on the offset from the frame center.
     - Calculate the area of the marker and determine the percentage visible in the frame.

5. **Process Visibility**:
   - Define the `process_visible_percentage(percentage_visible)` function:
     - If the visibility percentage exceeds a defined threshold (20%), raise a stop flag.
     - Use a queue to manage the stop signal.

6. **Color Processing Functions**:
   - Define `process_green_percentage(frame)` and `process_red_percentage(frame)` functions:
     - Convert the frame to HSV color space.
     - Create masks for green and red colors and calculate their respective visibility percentages.

7. **Compare Color Percentages**:
   - Define `compare_green_red_percentage(green_percentage, red_percentage)` function:
     - Compare the green and red percentages with a defined margin of error (5%).
     - If one color is significantly higher, raise a flag for I2C communication.
     - If the difference is within the margin of error, do nothing.

8. **Handle I2C Flags**:
   - Define `handle_red_green_flag()` to manage the flags raised based on color comparisons:
     - Check the I2C queue and perform actions based on received flags (e.g., print messages).

9. **Main Loop**:
   - Capture frames from the camera.
   - Call `detect_aruco_marker()` to retrieve the angle and visibility percentage of the marker.
   - Update `currentAngle` if a significant change is detected.
   - Call `process_visible_percentage()` to manage the stop flag based on visibility.
   - If the stop flag is raised, calculate green and red percentages and compare them.
   - Display relevant information (angle, green percentage, red percentage) on the video feed.
   - Allow the user to exit by pressing 'q'.

10. **Cleanup**:
    - Release the camera and destroy all OpenCV windows upon exit.

## Dependencies
- Python 3.x
- OpenCV (`opencv-python`, `opencv-contrib-python`)
- NumPy

## Future Improvements
- Verify camera calibration.
- Test responsiveness to varying speeds of marker detection.
- Refine margin of error values based on testing.
- Implement full I2C communication logic for flag handling.
