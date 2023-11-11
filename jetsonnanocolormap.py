import cv2
import numpy as np

# GStreamer pipeline for the left camera
left_pipeline = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480, format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"

# GStreamer pipeline for the right camera
right_pipeline = "nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480, format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"

# Open the stereo cameras
cap_left = cv2.VideoCapture(left_pipeline, cv2.CAP_GSTREAMER)
cap_right = cv2.VideoCapture(right_pipeline, cv2.CAP_GSTREAMER)

if not cap_left.isOpened() or not cap_right.isOpened():
    print("Error: Could not open cameras.")
    exit()

while True:
    # Read a frame from each camera
    ret_left, frame_left = cap_left.read()
    ret_right, frame_right = cap_right.read()

    if not ret_left or not ret_right:
        break

    # Convert the frames to grayscale
    gray_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)

    # Apply a color map to the grayscale images
    colormap_left = cv2.applyColorMap(gray_left, cv2.COLORMAP_TURBO)
    colormap_right = cv2.applyColorMap(gray_right, cv2.COLORMAP_TURBO)

    # Display the original frames and the colormap frames side by side
    combined_frame = np.hstack((frame_left, colormap_left, frame_right, colormap_right))

    cv2.imshow('Stereo: Original vs. Colormap', combined_frame)

    # Break the loop when the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the cameras and close all OpenCV windows
cap_left.release()
cap_right.release()
cv2.destroyAllWindows()
