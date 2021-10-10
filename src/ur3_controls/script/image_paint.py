#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import imutils
from collections import deque

from dynamic_reconfigure.server import Server
from ur3_controls.cfg import DHConfig

class Paint:
    def __init__(self):
        rospy.loginfo("Init")

        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(10)

        # Publishers
        self.pub = rospy.Publisher('image_paint', Image,queue_size=1)

        # Subscribers
        rospy.Subscriber("/camera/rgb/image_color",Image,self.callback)

        self.red_lower = np.array([25, 146, 190], np.uint8)
        self.red_upper = np.array([62, 174, 250], np.uint8)
        self.kernal = np.ones((7,7),np.uint8)

        self.mask = None
        self.output = None

        self.pts_j = [deque(maxlen=2000),deque(maxlen=2000),deque(maxlen=2000),deque(maxlen=2000),deque(maxlen=2000), deque(maxlen=2000)]

        self.srv = Server(DHConfig, self.dyn_callback)

        self.color_list = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (255, 255, 0), (0, 255, 255), (255, 0, 255)]
        self.color = 0


    def dyn_callback(self, config, level):

        self.red_lower = np.array([config.B_lower, config.G_lower, config.R_lower], np.uint8)
        self.red_upper = np.array([config.B_upper, config.G_upper, config.R_upper], np.uint8)

        self.color = config.Color_paint
        self.paint = config.Paint_enable

        if config.Paint_clear:
            self.pts_j = [deque(maxlen=2000),deque(maxlen=2000),deque(maxlen=2000),deque(maxlen=2000),deque(maxlen=2000), deque(maxlen=2000)]

        return config


    def callback(self, msg):
        # rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)
        
        blurred = cv2.GaussianBlur(self.image, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        self.mask = cv2.inRange(hsv, self.red_lower , self.red_upper)
        self.mask = cv2.erode(self.mask, None, iterations=2)
        self.mask = cv2.dilate(self.mask, None, iterations=2)

            # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(self.mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
        if len(cnts) > 0:

            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        if self.paint:

            self.pts_j[self.color].appendleft(center)

        

        # ------------------------------------------------------------------------#
    	# loop over the set of tracked points
        if self.paint:

            for i in range(1, 2000):
                # if either of the tracked points are None, ignore
                # them
                if self.pts_j[0] is None:
                    pass
                else:
                    try:
                        cv2.circle(self.image, self.pts_j[0][i], 3, self.color_list[0], 5)
                    except:
                        pass

                if self.pts_j[1] is None:
                    pass
                else:
                    try:
                        cv2.circle(self.image, self.pts_j[1][i], 2, self.color_list[1], 3)
                    except:
                        pass

                if self.pts_j[2] is None:
                    pass
                else:
                    try:
                        cv2.circle(self.image, self.pts_j[2][i], 2, self.color_list[2], 3)
                    except:
                        pass

                if self.pts_j[3] is None:
                    pass
                else:
                    try:
                        cv2.circle(self.image, self.pts_j[3][i], 2, self.color_list[3], 3)
                    except:
                        pass

                if self.pts_j[4] is None:
                    pass
                else:
                    try:
                        cv2.circle(self.image, self.pts_j[4][i], 2, self.color_list[4], 3)
                    except:
                        pass

                if self.pts_j[3] is None:
                    pass
                else:
                    try:
                        cv2.circle(self.image, self.pts_j[5][i], 2, self.color_list[5], 5)
                    except:
                        pass

            
        self.pub.publish(self.br.cv2_to_imgmsg(self.image))

    def start(self):
        rospy.loginfo("Timing images")
        rospy.spin()
        # while not rospy.is_shutdown():
        #     # rospy.loginfo('publishing image')
        #     #br = CvBridge()
        #     if self.image is not None:
        #         self.pub.publish(self.br.cv2_to_imgmsg(self.image))
        #     self.loop_rate.sleep()

if __name__ == '__main__':
    
    rospy.init_node('Paint')

    my_node = Paint()
    my_node.start()