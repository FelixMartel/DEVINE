import cv2
import numpy as np
from enum import Enum

# The idea: 
# The player zone will be defined by the body_tracking lib
# Where a player close enough from the robot will be selected
# While Idle state should just try and look for humans
# The object area should be pre-found on boot
# The object area will be recognized by a set of colors
# The colors will come from post-its/papers
# The object area should be setable via the dashboard in case of errors
# The object area will default at (0, 0) motor position

# https://www.sciencedirect.com/science/article/pii/S0898122112002787 comparable

# Tracked colors
class TrackingColor(Enum):
    FUCHSIA = 1
    #GREEN = 2

#Hue range of tracked colors
TRACKED_COLORS_HUE = {
   # TrackingColor.GREEN: [25, 85],
    TrackingColor.FUCHSIA: [150, 180]
}

# Todo: We could use more corners
#UPPER_LEFT = [TrackingColor.FUCHSIA, TrackingColor.GREEN]
#BOTTOM_RIGHT = [TrackingColor.GREEN, TrackingColor.FUCHSIA]

video_capture = cv2.VideoCapture(0)

while True:
    _, frame = video_capture.read()  # Read the next image from the camera
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Blur
    hsv = cv2.blur(hsv, (3, 3), (-1, -1))

    # Filter out low saturation pixels
    mask = cv2.inRange(hsv, (0, 40, 0), (255, 255, 255))
    hsv = cv2.bitwise_and(frame, frame, mask=mask)
    
    hue = hsv[:, :, 0] # Convert to single channel hue representation

    cv2.imshow("Hue representation", hue)

    # Noise reduction equivalent to erosion + dilatation filters
    hue = cv2.morphologyEx(hue, cv2.MORPH_OPEN, None, iterations=2)

    hsv[:, :, 0] = hue
    cv2.imshow("Filtered HSV", hsv)
    test = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    cv2.imshow("Filtered HSV 2", hsv)

    found_colors = {}

    for tracked_color in TrackingColor:
        [color_min, color_max] = TRACKED_COLORS_HUE[tracked_color]
        masked_image=cv2.inRange(hue, color_min, color_max)
        cv2.imshow("Masked image", masked_image)
        _, contours, _ = cv2.findContours(
            masked_image,
            cv2.RETR_LIST,
            cv2.CHAIN_APPROX_SIMPLE
        )

        def filter_contour(contour):
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
            return len(approx) == 4

        filtered_contours = filter(filter_contour, contours)
        
        found_colors[tracked_color] = map(cv2.boundingRect, filtered_contours)

    (image_width, image_height) = hue.shape
    for found_color in found_colors:
        for (x, y, w, h) in found_colors[found_color]:
            if w > 20 and h > 20 and w < 0.4*image_width and h < 0.4*image_height: #todo, find appropriate thresholds for the area
                cv2.rectangle(frame, (x,y), (x+w,y+h), (255, 0, 0), 2)


    #cv2.imshow('Hue1', tracked_hues)
    cv2.imshow('Result', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video_capture.release()
cv2.destroyAllWindows()
