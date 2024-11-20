## not mine, GPT created:

import pyrealsense2 as rs
import numpy as np
import cv2
from pupil_apriltags import Detector

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Initialize AprilTag detector
at_detector = Detector(families='tag36h11')

try:
    while True:
        # Capture frames
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert RealSense image to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Convert to grayscale
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        tags = at_detector.detect(gray_image, estimate_tag_pose=False)

        # Draw detected tags
        for tag in tags:
            (ptA, ptB, ptC, ptD) = tag.corners
            ptA = (int(ptA[0]), int(ptA[1]))
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))

            # Draw the bounding box of the AprilTag detection
            cv2.line(color_image, ptA, ptB, (0, 255, 0), 2)
            cv2.line(color_image, ptB, ptC, (0, 255, 0), 2)
            cv2.line(color_image, ptC, ptD, (0, 255, 0), 2)
            cv2.line(color_image, ptD, ptA, (0, 255, 0), 2)

            # Draw the center (cx, cy) of the tag
            (cX, cY) = (int(tag.center[0]), int(tag.center[1]))
            cv2.circle(color_image, (cX, cY), 5, (0, 0, 255), -1)

            # Display tag ID
            cv2.putText(color_image, str(tag.tag_id),
                        (ptA[0], ptA[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)

        # Display the resulting image
        cv2.imshow('AprilTag Detection', color_image)

        # Break loop with 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
