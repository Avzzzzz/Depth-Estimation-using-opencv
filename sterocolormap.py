import cv2
import numpy as np

# Open the stereo camera
cap_left = cv2.VideoCapture(0)
cap_right = cv2.VideoCapture(1)  # Assuming the second camera is used as the right camera

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
