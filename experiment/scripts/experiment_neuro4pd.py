#!/usr/bin/env python

import roslib
import rospy
from std_msgs.msg import String

class NodeExp():
    def __init__(self):
        # ROS
        self.loop_rate = rospy.Rate(1.5)
        # Pub
        self.pub_colour = rospy.Publisher('ball_colour',String,queue_size=10)
        # Variables
        self.flag = 0
        self.time = 1

    def simulate(self):    
        while not rospy.is_shutdown():
            if self.time == 30:
                self.pub_colour.publish('Not Green')
            elif self.time == 55:
                self.pub_colour.publish('Green')
            elif self.time == 75:
                self.pub_colour.publish('Not Green')
            
            print self.time           
            self.loop_rate.sleep()
            self.time = self.time+1
            if self.time > 90:
                break;

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('experiment_neuro4pd')
    try:
        exp = NodeExp()
        exp.simulate()
    except rospy.ROSInterruptException: 
        pass
