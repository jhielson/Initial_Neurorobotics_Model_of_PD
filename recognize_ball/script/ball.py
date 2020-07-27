#!/usr/bin/env python
import time
import rospy
import cv2
import numpy as np

from std_srvs.srv import Empty
from std_msgs.msg import String
from sensor_msgs.msg import Image
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from cv_bridge import CvBridge, CvBridgeError

class Ball(object):
    def __init__(self):
        #Params
        self.loop_rate = rospy.Rate(3)
        self.bridge = CvBridge()
        self.image = Image()
        self.pixel_hue = -1
        self.flag_new_image = 0
    
        #Pub
        self.pub_colour = rospy.Publisher('ball_colour', String, queue_size=10)
        #Sub
        rospy.Subscriber('nao_robot/camera/bottom/camera/image_raw', Image, self.image_callback) 

    def show_image(self,img):
        cv2.imshow('Image Window', img)
        cv2.waitKey(3)

    def image_callback(self,data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data,'passthrough')
            self.flag_new_image = 1
        except CvBridgeError, e:
            rospy.logerr('CvBridge Error:{0}'.format(e))
     
        #Copy Image        
        copy_image = self.image.copy()
        #Convert BGR to HSV
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        
        pixels = []
        for x in range(180,210):
            for y in range(140,170):
                    if hsv[x,y][1] > 0 and hsv[x,y][2] < 255:  
                        #print hsv[x,y]
                        if hsv[x,y][0] > 120:
                            pixels.append(1)
                        elif hsv[x,y][0] > 75 and hsv[x,y][0] < 95:
                            pixels.append(2)
                        elif hsv[x,y][0] > 25 and hsv[x,y][0] < 55:
                            pixels.append(3)
                        copy_image[x][y] = [0,0,0]

        #Get Pixel
        if len(pixels) > 0:
            self.pixel_hue = np.bincount(pixels).argmax()
        else:
            self.pixel_hue = 0
        
        #Show
        self.show_image(copy_image)

    def start(self):
        #Initialize the CvBridge class
        self.bridge = CvBridge()
        while not rospy.is_shutdown():
            #Check colour
            if self.flag_new_image == 1:
                if self.pixel_hue == 1:
                    print 'Purple'
                    self.pub_colour.publish('Purple')
                    self.loop_rate.sleep()
                elif self.pixel_hue == 2:
                    print 'Green'
                    self.pub_colour.publish('Green')
                    self.loop_rate.sleep()
                elif self.pixel_hue == 3:
                    print 'Yellow'
                    self.pub_colour.publish('Yellow')
                    self.loop_rate.sleep()
                else:
                    print 'Not sure'
                self.flag_new_image = 0
            self.loop_rate.sleep()
    

if __name__ == '__main__':
    try:
        #IniT
        rospy.init_node('recognize_ball', anonymous=True)
        recognize_ball = Ball()
        recognize_ball.start()
    except rospy.ROSInterruptException:
        pass
