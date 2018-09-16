#!/usr/bin/env python2
'''Zone detection using bright colored squares'''

import rospy
from std_msgs.msg import String

from ros_image_processor import ImageProcessor, ROSImageProcessingWrapper

import cv2
import numpy as np
from enum import Enum

IMAGE_TOPIC = '/camera/rgb/image_color/compressed' #'/devine/image/zone_detection'
ZONE_DETECTION_TOPIC = '/zone_detection'

# Tracked colors
class TrackingColor(Enum):
    FUCHSIA = 1
    #GREEN = 2

#Hue range of tracked colors
TRACKED_COLORS_HUE = {
   # TrackingColor.GREEN: [25, 85],
    TrackingColor.FUCHSIA: [150, 180]
}

f = open('HSVDump.txt', 'w')

class ZoneDetection(ImageProcessor):
    '''Zone detection'''
    
    def process(self, image):
        bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        r = cv2.selectROI(bgr)

        important = hsv[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]

        f.write("\n\n")
        for item in important:
            print >> f, item

        # # Blur
        # hsv = cv2.blur(hsv, (3, 3), (-1, -1))

        # # Filter out low saturation pixels
        # mask = cv2.inRange(hsv, (0, 40, 0), (255, 255, 255))
        # hsv = cv2.bitwise_and(image, image, mask=mask)

        # hue = hsv[:, :, 0] # Convert to single channel hue representation

        # cv2.imshow("Hue representation", hue)

        # # Noise reduction equivalent to erosion + dilatation filters
        # hue = cv2.morphologyEx(hue, cv2.MORPH_OPEN, None, iterations=2)

        # hsv[:, :, 0] = hue
        # cv2.imshow("Filtered HSV", hsv)

        # found_colors = {}

        # for tracked_color in TrackingColor:
        #     [color_min, color_max] = TRACKED_COLORS_HUE[tracked_color]
        #     masked_image=cv2.inRange(hue, color_min, color_max)
        #     cv2.imshow("Masked image", masked_image)
        #     _, contours, _ = cv2.findContours(
        #         masked_image,
        #         cv2.RETR_LIST,
        #         cv2.CHAIN_APPROX_SIMPLE
        #     )

        #     def filter_contour(contour):
        #         perimeter = cv2.arcLength(contour, True)
        #         approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
        #         return len(approx) == 4

        #     filtered_contours = filter(filter_contour, contours)
            
        #     found_colors[tracked_color] = map(cv2.boundingRect, filtered_contours)

        # (image_width, image_height) = hue.shape
        # for found_color in found_colors:
        #     for (x, y, w, h) in found_colors[found_color]:
        #         if w > 20 and h > 20 and w < 0.4*image_width and h < 0.4*image_height: #todo, find appropriate thresholds for the area
        #             cv2.rectangle(bgr, (x,y), (x+w,y+h), (255, 0, 0), 2)
        
        
        # cv2.imshow("Original image", bgr)


        if cv2.waitKey(0) & 0xFF == ord('q'):
            exit()

        return "Hello, World !"


def main():
    '''Entry point of this file'''
    processor = ROSImageProcessingWrapper(ZoneDetection, IMAGE_TOPIC)
    publisher = rospy.Publisher(ZONE_DETECTION_TOPIC, String, queue_size=10, latch=True)
    processor.loop(lambda processor_output : publisher.publish(processor_output))

if __name__ == '__main__':
    main()
