#!/usr/bin/env python

import math, random, csv, rospy
import numpy as np
from std_msgs.msg import Float64MultiArray,Float64
import matplotlib.pyplot as plt


class MLP(object):
    def __init__(self):
        # Bias
        self.bias = []
        # Weights
        self.weights_input = []
        self.weights_hidden1 = []
        self.weights_hidden2 = []
        self.weights_output = []
        # Output
        self.output = []
        self.outputRaw = []
        self.outputDiff = []
        self.previous = -1
        self.averageRaw = []
        self.averageDiff = []
        self.calculateAvgRaw = []
        self.calculateAvgDiff = []

        # Pub 
        self.pub_perturbation = rospy.Publisher('motorStimulation', Float64, queue_size=10)

        # Sub
        rospy.Subscriber('motorNeuralSignal', Float64MultiArray, self.predict)

        # Rate
        self.rate = rospy.Rate(100)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    def loadMLP(self):        
        self.bias = np.loadtxt(open("bias.csv", "rb"), delimiter=",", skiprows=0)
        self.weights_input = np.loadtxt(open("input.csv", "rb"), delimiter=",", skiprows=0)
        self.weights_hidden1 = np.loadtxt(open("hidden1.csv", "rb"), delimiter=",", skiprows=0)
        self.weights_hidden2 = np.loadtxt(open("hidden2.csv", "rb"), delimiter=",", skiprows=0)
        self.weights_output = np.loadtxt(open("output.csv", "rb"), delimiter=",", skiprows=0)

    def testFile(self):
        self.dataH = np.loadtxt(open("H.csv", "rb"), delimiter=",", skiprows=0)
        self.dataPD = np.loadtxt(open("PD.csv", "rb"), delimiter=",", skiprows=0)
        print '_____________________________________'
        print 'PD'
   
        resultPD = []
        for i in range(0,len(self.dataPD)):
            mean_fr = self.dataPD[i][0]
            mean_isis = self.dataPD[i][1]
            sd_isis = self.dataPD[i][2]
            resultPD.append(self.network(mean_fr,mean_isis,sd_isis))

        fig, axs = plt.subplots(2, 2)
        axs[0,1].plot(resultPD)
        axs[0,1].set_title('PD')
        axs[0,1].set_xlim([0,4500])
        axs[0,1].set_ylim([-0.1,0.1])

        print '_____________________________________'
        print 'H'

        resultH = []
        for i in range(0,len(self.dataH)):
            mean_fr = self.dataH[i][0]
            mean_isis = self.dataH[i][1]
            sd_isis = self.dataH[i][2]
            resultH.append(self.network(mean_fr,mean_isis,sd_isis))

        axs[0,0].plot(resultH)
        axs[0,0].set_title('H')
        axs[0,0].set_xlim([0,4500])
        axs[0,0].set_ylim([-0.1,0.1])

        print '_____________________________________'
        print 'Diff'

        # Diff
        diff_H = []
        diff_PD = []

        for n in range(2,4500,1):
            diff_PD.append(abs(resultPD[n]-resultPD[n-1]))
            diff_H.append(abs(resultH[n]-resultH[n-1]))

        axs[1,1].plot(diff_PD)
        axs[1,1].set_title('PD')
        axs[1,1].set_xlim([0,4500])
        axs[1,1].set_ylim([0,0.1])

        axs[1,0].plot(diff_H)
        axs[1,0].set_title('H')
        axs[1,0].set_xlim([0,4500])
        axs[1,0].set_ylim([0,0.1])
        plt.show()


    def tansig(self,n):
        return  2/(1+math.exp(-2*n))-1

    def logsig(self,n):
        return 1/(1+math.exp(-n))

    def linear(self,n):
        return n

    def norm(self,value,max_v,min_v):
        return (value-min_v)/(max_v-min_v)

    def normI2(self,n):
	ymax = 1  #input_ymax;
	ymin = -1 #input_ymin;
	xmax = 462.0750  #input_xmax;
	xmin = 125.5192  #input_xmin;
	n_proc = (ymax-ymin) * (n-xmin) / (xmax-xmin) + ymin;
        return n_proc;

    def normI3(self,n):
        ymax = 1  #input_ymax;
        ymin = -1 #input_ymin;
        xmax = 110.0044  #input_xmax;
        xmin = 13.0019  #input_xmin;
        n_proc = (ymax-ymin) * (n-xmin) / (xmax-xmin) + ymin;
        return n_proc;

    def normI1(self,n):    
        ymax = 1  #input_ymax;
        ymin = -1 #input_ymin;
        xmax = 8.0  #input_xmax;
        xmin = 2.1  #input_xmin;
        n_proc = (ymax-ymin) * (n-xmin) / (xmax-xmin) + ymin;
        return n_proc;

    def normO(self,n):
	ymax = 1  #output_ymax;
	ymin = -1 #output_ymin;
	xmax = 1  #output_xmax;
	xmin = -1  #output_xmin;
	n_proc = (n-ymin) * (xmax-xmin) /(ymax-ymin) + xmin;
        return n_proc;       

    def predict(self,data):
        mean_fr = data.data[0]
        print mean_fr
        mean_isis = data.data[1]
        print mean_isis
        sd_isis = data.data[2]
        print sd_isis
 
        # Predict
        outputRaw = self.network(mean_fr,mean_isis,sd_isis)
        
        # Save
        self.outputRaw.append(outputRaw)
        print 'RAW'
        print self.outputRaw

        # Average
        outputAvgRaw = self.calcAverageRaw(outputRaw)
        self.averageRaw.append(outputAvgRaw)
        print 'Average RAW'
        print self.averageRaw

        if outputAvgRaw != -1 :
             # Publish
             msg = Float64()
             data = self.ampLinear(abs(outputAvgRaw))
             self.output.append(data)
             print 'Output'
             print self.output
             msg.data = data
             self.pub_perturbation.publish(msg)
            
        # Diff
        #if self.previous != -1:
        #    outputDiff = abs(outputRaw-self.previous)
        #    self.outputDiff.append(outputDiff)
        #    print 'DIFF'
        #    print self.outputDiff
        #    # Average
        #    outputAvgDiff = self.calcAverageDiff(outputDiff)
        #    self.averageDiff.append(outputAvgDiff)
        #    print 'Average DIFF'
        #    print self.averageDiff
        #    # Check output
        #    if outputAvgDiff != -1 :      
        #        # Publish
        #        msg = Float64()
        #        msg.data = self.ampl(abs(outputAvgDiff))
        #        self.pub_perturbation.publish(msg)
        #self.previous = outputRaw

    def ampNonLinear(self,value):
        temp = (1100*value-9.5)
        r = np.power(2,temp)
        # Constrain
        if r>0.5:
            r = 0.5
        return r

    def ampLinear(self,value):
        return 15*(value-0.0015)

    def calcAverageRaw(self,value):
        if len(self.calculateAvgRaw) < 4:
            self.calculateAvgRaw.append(value)
            return -1
        else:
            self.calculateAvgRaw.pop(0)
            self.calculateAvgRaw.append(value)
            return np.mean(self.calculateAvgRaw)

    def calcAverageDiff(self,value):
        if len(self.calculateAvgDiff) < 4:
            self.calculateAvgDiff.append(value)
            return -1
        else:
            self.calculateAvgDiff.pop(0)
            self.calculateAvgDiff.append(value)
            return np.mean(self.calculateAvgDiff)

    def network(self,I1,I2,I3):
        # First layer
        O_I1 = self.logsig(self.normI1(I1)*self.weights_input[0,0]+self.normI2(I2)*self.weights_input[0,1]+self.normI3(I3)*self.weights_input[0,2]+self.bias[0])
        O_I2 = self.logsig(self.normI1(I1)*self.weights_input[1,0]+self.normI2(I2)*self.weights_input[1,1]+self.normI3(I3)*self.weights_input[1,2]+self.bias[1])
        O_I3 = self.logsig(self.normI1(I1)*self.weights_input[2,0]+self.normI2(I2)*self.weights_input[2,1]+self.normI3(I3)*self.weights_input[2,2]+self.bias[2])
        
        # Second layer
        O_H1_1 = self.logsig(O_I1*self.weights_hidden1[0,0]+O_I2*self.weights_hidden1[0,1]+O_I3*self.weights_hidden1[0,2])
        O_H1_2 = self.logsig(O_I1*self.weights_hidden1[1,0]+O_I2*self.weights_hidden1[1,1]+O_I3*self.weights_hidden1[1,2])
        O_H1_3 = self.logsig(O_I1*self.weights_hidden1[2,0]+O_I2*self.weights_hidden1[2,1]+O_I3*self.weights_hidden1[2,2])
        O_H1_4 = self.logsig(O_I1*self.weights_hidden1[3,0]+O_I2*self.weights_hidden1[3,1]+O_I3*self.weights_hidden1[3,2])
        O_H1_5 = self.logsig(O_I1*self.weights_hidden1[4,0]+O_I2*self.weights_hidden1[4,1]+O_I3*self.weights_hidden1[4,2])
        O_H1_6 = self.logsig(O_I1*self.weights_hidden1[5,0]+O_I2*self.weights_hidden1[5,1]+O_I3*self.weights_hidden1[5,2])
        O_H1_7 = self.logsig(O_I1*self.weights_hidden1[6,0]+O_I2*self.weights_hidden1[6,1]+O_I3*self.weights_hidden1[6,2])
        O_H1_8 = self.logsig(O_I1*self.weights_hidden1[7,0]+O_I2*self.weights_hidden1[7,1]+O_I3*self.weights_hidden1[7,2])
        O_H1_9 = self.logsig(O_I1*self.weights_hidden1[8,0]+O_I2*self.weights_hidden1[8,1]+O_I3*self.weights_hidden1[8,2])

        # Third layer
        O_H2_1 = self.logsig(O_H1_1*self.weights_hidden2[0,0]+O_H1_2*self.weights_hidden2[0,1]+O_H1_3*self.weights_hidden2[0,2]+O_H1_4*self.weights_hidden2[0,3]+O_H1_5*self.weights_hidden2[0,4]+O_H1_6*self.weights_hidden2[0,5]+O_H1_7*self.weights_hidden2[0,6]+O_H1_8*self.weights_hidden2[0,7]+O_H1_9*self.weights_hidden2[0,8])
        O_H2_2 = self.logsig(O_H1_1*self.weights_hidden2[1,0]+O_H1_2*self.weights_hidden2[1,1]+O_H1_3*self.weights_hidden2[1,2]+O_H1_4*self.weights_hidden2[1,3]+O_H1_5*self.weights_hidden2[1,4]+O_H1_6*self.weights_hidden2[1,5]+O_H1_7*self.weights_hidden2[1,6]+O_H1_8*self.weights_hidden2[1,7]+O_H1_9*self.weights_hidden2[1,8])
        O_H2_3 = self.logsig(O_H1_1*self.weights_hidden2[2,0]+O_H1_2*self.weights_hidden2[2,1]+O_H1_3*self.weights_hidden2[2,2]+O_H1_4*self.weights_hidden2[2,3]+O_H1_5*self.weights_hidden2[2,4]+O_H1_6*self.weights_hidden2[2,5]+O_H1_7*self.weights_hidden2[2,6]+O_H1_8*self.weights_hidden2[2,7]+O_H1_9*self.weights_hidden2[2,8])
        O_H2_4 = self.logsig(O_H1_1*self.weights_hidden2[3,0]+O_H1_2*self.weights_hidden2[3,1]+O_H1_3*self.weights_hidden2[3,2]+O_H1_4*self.weights_hidden2[3,3]+O_H1_5*self.weights_hidden2[3,4]+O_H1_6*self.weights_hidden2[3,5]+O_H1_7*self.weights_hidden2[3,6]+O_H1_8*self.weights_hidden2[3,7]+O_H1_9*self.weights_hidden2[3,8])
        O_H2_5 = self.logsig(O_H1_1*self.weights_hidden2[4,0]+O_H1_2*self.weights_hidden2[4,1]+O_H1_3*self.weights_hidden2[4,2]+O_H1_4*self.weights_hidden2[4,3]+O_H1_5*self.weights_hidden2[4,4]+O_H1_6*self.weights_hidden2[4,5]+O_H1_7*self.weights_hidden2[4,6]+O_H1_8*self.weights_hidden2[4,7]+O_H1_9*self.weights_hidden2[4,8])
        O_H2_6 = self.logsig(O_H1_1*self.weights_hidden2[5,0]+O_H1_2*self.weights_hidden2[5,1]+O_H1_3*self.weights_hidden2[5,2]+O_H1_4*self.weights_hidden2[5,3]+O_H1_5*self.weights_hidden2[5,4]+O_H1_6*self.weights_hidden2[5,5]+O_H1_7*self.weights_hidden2[5,6]+O_H1_8*self.weights_hidden2[5,7]+O_H1_9*self.weights_hidden2[5,8])
        
        # Forth layer
        output = self.normO(self.tansig(O_H2_1*self.weights_output[0]+O_H2_2*self.weights_output[1]+O_H2_3*self.weights_output[2]+O_H2_4*self.weights_output[3]+O_H2_5*self.weights_output[4]+O_H2_6*self.weights_output[5]))

        return output

if __name__ == '__main__':
    try:
        rospy.init_node('mlp', anonymous=True)
        mlp = MLP()
        mlp.loadMLP()
        #print mlp.network(1,1,1)
        #print mlp.network(0.5,0.5,0.5)
        #print mlp.network(0.1,0.1,0.1)
        #print mlp.network(1,0.5,0.1)
        #print mlp.network(0.3,0.6,0.2)
        #print mlp.network(0.4,0.1,0.8)
        #mlp.testFile()
        mlp.run()
    except rospy.ROSInterruptException:
        pass

