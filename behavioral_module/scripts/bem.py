#!/usr/bin/env python
import os, math, random, time
import numpy as np
import rospy
from std_srvs.srv import Empty
from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed, HeadTouch

class Robot(object):
    def __init__(self):
        #Param        
        self.spikes_rate = 0
        self.loop_rate = rospy.Rate(1)
        self.trials = 3

        #Variables
        self.counter_wait = 10
        self.state = 0
        self.colour = 'Not sure!'
        self.button = 0
        self.send_wait = 0
        self.joint_names = []
        self.joint_angles = []
        self.awake = 0
        self.tremor = 1
        
        #Flags
        self.speak_once = False
        self.flag_stimuli = False
        self.flag_stimuli_number = 0

        #Joints
        self.joints = [0,0,0,0,0,0]
        self.noise = 0
    
        #Pub Robot
        self.pub_joints = rospy.Publisher('joint_angles', JointAnglesWithSpeed, queue_size=10)
        self.pub_speak = rospy.Publisher('speech', String, queue_size=10)
        
        #Sub General
        rospy.Subscriber('visualCue', String, self.callback_colour)        
        rospy.Subscriber('motorStimulation',Float64,self.callback_perturbation)
        #Sub Robot    
        rospy.Subscriber('tactile_touch', HeadTouch, self.callback_headTouch) 
        rospy.Subscriber('joint_states',JointState,self.callback_jointState)

    def callback_perturbation(self,data):
        if not math.isnan(data.data):
            self.noise = data.data*self.tremor
            if self.tremor == 1:
                self.tremor = -1
            else:
                self.tremor = 1
            #print self.noise

        #Move right arm
        if self.awake == 1:
            msg = JointAnglesWithSpeed()
            msg.joint_names = self.joint_names
            msg.joint_angles = self.joint_angles
            print self.joint_angles
            msg.speed = 0.08
            msg.relative = 0
            self.pub_joints.publish(msg)
            time.sleep(0.1)

    def callback_jointState(self,data):
        self.joint = [data.position[8],data.position[9],data.position[10],data.position[11],data.position[12],data.position[13]]

    def callback_headTouch(self,data):
        self.button = data.button
        if data.state == 1 and self.state == 0:
            self.state = 1    
            self.speak_once = False

    def callback_colour(self,data):
        if data.data == 'Green':
            self.flag_stimuli = 1
        else:
            self.flag_stimuli = 0
        self.colour = data.data

    def wakeUp(self):
        rospy.wait_for_service('wakeup')
        try:
            #WakeUp
            wakeup = rospy.ServiceProxy('wakeup', Empty)
            wakeup()
            time.sleep(1)
            #Speak: Hi                
            self.pub_speak.publish("Hi!")
            time.sleep(1)

            #Move right arm to the initial pose
            self.joint_names = ['RShoulderPitch','RShoulderRoll','RWristYaw','RElbowRoll','RElbowYaw','RHand']
            self.joint_angles = [0.61,-0.80,1.44,0.83,0.25,0.0]
            #Move left arm to the initial pose
            msg = JointAnglesWithSpeed()
            msg.joint_names = ['LShoulderPitch','LShoulderRoll','LWristYaw','LElbowRoll','LElbowYaw']
            msg.joint_angles = [1.35,1.00,-0.53,-1.10,-1.24]
            msg.speed = 0.1
            msg.relative = 0
            self.pub_joints.publish(msg)
            #Sleep
            time.sleep(1)

            #Fix head pose
            msg = JointAnglesWithSpeed()
            msg.joint_names = ['HeadPitch','HeadYaw']
            msg.joint_angles = [0.4,0.0]
            msg.speed = 0.2
            msg.relative = 0
            self.pub_joints.publish(msg)
            #Sleep 
            time.sleep(1)
            self.awake = 1

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def open_hand(self):
        msg = JointAnglesWithSpeed()
        msg.joint_names = ['RHand']
        msg.joint_angles = [0.6]
        msg.speed = 0.5
        msg.relative = 0
        self.pub_joints.publish(msg)
        time.sleep(0.1)

    def close_hand(self):
        msg = JointAnglesWithSpeed()
        msg.joint_names = ['RHand']
        msg.joint_angles = [0.0]
        msg.speed = 0.5
        msg.relative = 0
        self.pub_joints.publish(msg)
        time.sleep(0.1)

    def start_nao(self):
        #Wake Up
        print 'waking up...'
        self.wakeUp() 
        time.sleep(3) 
        print 'awake'
        #Speak  
        self.pub_speak.publish("Please put the ping pong ball in front of me.")
        time.sleep(1)
        #Loop
        count_trials = 0
        #Joints
        rShoulderRoll = []
        rElbowRoll = []
        rElbowYaw =[]
        rHand = []

        while not rospy.is_shutdown():
            #print "State: "+str(self.state)
            #print "Counter: "+str(self.counter_wait)
            
            #PrintJoints
            if len(rShoulderRoll) == 0 or self.joint_angles[1] != rShoulderRoll[len(rShoulderRoll)-1]:  
                rShoulderRoll.append(self.joint_angles[1])
            if len(rElbowRoll) == 0 or self.joint_angles[3] != rElbowRoll[len(rElbowRoll)-1]:
                rElbowRoll.append(self.joint_angles[3])
            if len(rElbowYaw) == 0 or self.joint_angles[4] != rElbowYaw[len(rElbowYaw)-1]:
                rElbowYaw.append(self.joint_angles[4])
            if len(rHand) == 0 or self.joint_angles[5] != rHand[len(rHand)-1]:
                rHand.append(self.joint_angles[5])
            
            #End interaction
            if self.button == 3:
                self.pub_speak.publish("Oh, okay. Bye bye!")
                time.sleep(0.1)
                #Adjust head pose
                msg = JointAnglesWithSpeed()
                msg.joint_names = ['HeadPitch','HeadYaw']
                msg.joint_angles = [0.0,0.0]
                msg.speed = 0.03
                msg.relative = 0
                self.pub_joints.publish(msg)
                time.sleep(0.1)
                #Move right arm to the rest pose
                self.joint_names = ['RShoulderPitch','RShoulderRoll','RWristYaw','RElbowRoll','RElbowYaw','RHand']
                self.joint_angles = [0.61,0.55+self.noise,1.44+self.noise,0.83+self.noise,0.25+self.noise,0.0+self.noise]
                #Move left arm to the rest pose
                msg = JointAnglesWithSpeed()
                msg.joint_names = ['LShoulderPitch','LShoulderRoll','LWristYaw','LElbowRoll','LElbowYaw']
                msg.joint_angles = [1.35,1.00,-0.53,-1.10,-1.24]
                msg.speed = 0.1
                msg.relative = 0
                self.pub_joints.publish(msg)
                #Sleep
                time.sleep(0.1)
                break
   
            #Initial State
            if self.state == 0:
                    #Move right arm to the initial pose
                    self.joint_names = ['RShoulderPitch','RShoulderRoll','RWristYaw','RElbowRoll','RElbowYaw','RHand']
                    self.joint_angles = [0.61,-0.80+self.noise,1.44+self.noise,0.83+self.noise,0.25+self.noise,0.0+self.noise]
                    #self.state = 1 #delete later
                    if self.counter_wait == 50:
                        #Speak       
                        self.speak_once = True
                        self.pub_speak.publish("Is it ready?")
                        time.sleep(0.1)   
                        self.counter_wait = 0
                    else:
                        time.sleep(0.1)
                        self.counter_wait += 1
            elif self.state == 1:
                #Move right arm to the initial pose
                self.joint_names = ['RShoulderPitch','RShoulderRoll','RWristYaw','RElbowRoll','RElbowYaw','RHand']
                self.joint_angles = [0.61,-0.80+self.noise,1.44+self.noise,0.83+self.noise,0.25+self.noise,0.0+self.noise]
                #Speak
                if self.speak_once == False:
                    self.pub_speak.publish("Okay, let me see.")
                    self.speak_once = True
                #Check colour
                if not self.colour == 'Not sure!':
                    count_trials += 1
                    # Speak
                    self.pub_speak.publish("I see "+self.colour)
                    self.state = 2
                    self.speak_once = False
                    self.send_wait = 0
                    self.counter_wait = 0
                    time.sleep(0.1)
                else:
                    self.pub_speak.publish("Not sure! Where is the ball?")
                    time.sleep(0.1)
                    self.state = 0         
                    self.speak_once = False         
                    self.send_wait = 0
            elif self.state == 2:
                if self.flag_stimuli_number == 1 or self.flag_stimuli:
                    self.flag_stimuli_number = 1
                    if self.speak_once == False:
                        #Ask to put the ball in its hand
                        self.pub_speak.publish("I like it. Can you put it on my hand?")
                        self.speak_once = True
                    
                    if self.counter_wait <= 50:    
                        #Set arm
                        self.joint_names = ['RShoulderPitch','RShoulderRoll','RWristYaw','RElbowRoll','RElbowYaw','RHand']
                        self.joint_angles = [0.61,-0.80+self.noise,1.44+self.noise,0.83+self.noise,0.25+self.noise,0.0+self.noise]
                        time.sleep(0.1)

                    if self.counter_wait > 50 and self.counter_wait < 160:
                        self.counter_wait +=1
                        #Move arm
                        self.joint_names = ['RShoulderPitch','RShoulderRoll','RWristYaw','RElbowRoll','RElbowYaw','RHand']
                        self.joint_angles = [0.61,-0.05+self.noise,1.47+self.noise,0.96+self.noise,1.69+self.noise,0.8+self.noise]
                        time.sleep(0.1)
                        
                    elif self.counter_wait >= 160:
                        self.state = 3
                        self.speak_once = False
                        self.counter_wait = 0
                        self.flag_stimuli_number = 0
                    else:
                        time.sleep(0.1)
                        self.counter_wait += 1
                else:
                    if self.speak_once == False:
                        self.pub_speak.publish("I do not want this ball.")
                        self.speak_once = True
                    
                    if self.counter_wait <= 30:    
                        #Set arm
                        self.joint_names = ['RShoulderPitch','RShoulderRoll','RWristYaw','RElbowRoll','RElbowYaw','RHand']
                        self.joint_angles = [0.61,-0.80+self.noise,1.44+self.noise,0.83+self.noise,0.25+self.noise,0.0+self.noise]
                        time.sleep(0.1)

                    if self.counter_wait > 30 and self.counter_wait < 160:
                        self.counter_wait +=1
                        #Move right  arm
                        self.joint_names = ['RShoulderPitch','RShoulderRoll','RWristYaw','RElbowRoll','RElbowYaw','RHand']
                        self.joint_angles = [0.61,1.23+self.noise,1.44+self.noise,1.43+self.noise,0.45+self.noise,0.0+self.noise]
                        time.sleep(0.1)
                        
                    elif self.counter_wait >= 160:
                        if count_trials == self.trials:
                            self.state = 4
                        else:
                            #Lets try again
                            self.pub_speak.publish("Lets try again.")
                            self.colour = 'Not sure!'
                            self.state = 0
                        self.counter_wait = 0
                        self.speak_once = False
                    else:
                        time.sleep(0.1)
                        self.counter_wait += 1
            elif self.state == 3:
                if self.speak_once == False:
                    #Speak
                    self.pub_speak.publish("Thank you!")
                    
                    self.speak_once = True
                #Move right arm to the box
                self.joint_names = ['RShoulderPitch','RShoulderRoll','RWristYaw','RElbowRoll','RElbowYaw','RHand']
                self.joint_angles = [0.61,0.10+self.noise,1.44+self.noise,0.83+self.noise,0.25+self.noise,0.8+self.noise]
                if self.counter_wait >= 60:
                    if count_trials == self.trials:
                        self.state = 4
                    else:
                        self.pub_speak.publish("I want more! Lets do it again.")
                        time.sleep(0.1)
                        self.state = 0
                    self.speak_once = False
                    self.counter_wait = 0
                else:
                    time.sleep(0.1)
                    self.counter_wait += 1
            elif self.state == 4:
                #Speak and move once
                if self.speak_once == False:
                    msg = JointAnglesWithSpeed()
                    msg.joint_names = ['HeadPitch','HeadYaw']
                    msg.joint_angles = [0.0,0.0]
                    msg.speed = 0.1
                    msg.relative = 0
                    self.pub_joints.publish(msg)
                    self.pub_speak.publish("Uhh. That is enough. Bye bye!")
                    self.speak_once = True

                #Move right arm to the box
                self.joint_names = ['RShoulderPitch','RShoulderRoll','RWristYaw','RElbowRoll','RElbowYaw','RHand']
                self.joint_angles = [0.61,0.10+self.noise,1.44+self.noise,0.83+self.noise,0.25+self.noise,0.8+self.noise]

                print 'rShoulderRoll:' 
                print rShoulderRoll
                print 'rElbowRoll:'
                print rElbowRoll
                print 'rElbowYaw:'
                print rElbowYaw
                print 'rHand:' 
                print rHand
    
if __name__ == '__main__':
    try:
        #Init
        rospy.init_node('nao', anonymous=True)
        my_robot = Robot()
        my_robot.start_nao()
    except rospy.ROSInterruptException:
        pass 
