
**This work represents the first steps towards the creation of an initial neurorobotics model of Parkinson’s Disease (PD). It is part of a multidisciplinary project, the Neuro4PD.**

# Neuro4PD
The Neurorobotics Model of Parkinson’s Disease is an interdisciplinary project to gain further insights into the mechanisms of Parkinson’s disease (PD) by combining the neuroscience expertise of the Brazilian partner with the data mining and robotics excellence of the UK partner. We envisage that the resultant model will contribute to inform new PD therapies and reduce and/or replace the use of live animals.

http://www.macs.hw.ac.uk/neuro4pd/

# Project
We advanced the state-of-art by embedding, for the first time, a computational composite model of PD in a real humanoid robot that is engaged on a simple behavioral task. 

The main contributions of this paper are: (i) an embedded computational composite model of PD in a real humanoid robot and (ii) the reproduction of abnormal PD motor stimulation based on cortical dynamics via the modulation of central oscillators.

We expect new insights into PD by future studies conducted using our proposed model.

## Requirements

In order to run this project, it will be necessary the following equipments:
1. NAO robot (we used the version T14)
2. Laptop/Computer
    * Ubuntu 18.04
    * NetPyNE 0.9.9
    * NEURON 7.6
    * ROS (Melodic)
    * Python 2.7
  
## Installation

1. Create a catkin workspace if you don't have one already
2. Git clone these packages into your ROS workspace.
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/jhielson/SML_Neuro4PD.git
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
```
3. Connect the robot to your computer using an ethernet cable. 
4. Access the network setting and change the IPV4 method to 'Link-Local Only'.

Now, you can turn the robot on. 

## Bring Up
In order to bring the robot up, you will need to obtain the IP address of your computer and the robot.

---

#### Roscore IP

Run the following command:
```
$ifconfig
```

#### Robot IP

Press the button located on the robot's chest and wait. He will say:
"Hello! I am NAO. My IP adress is ..."
You can use the IP address in your browser to access the robot's configuration. 

---

Once, the robot is running properly, you can execute the following command:
```
$roslaunch nao_bringup nao_full_py.launch nao_ip:=X roscore_ip:=X
```

## Experiment

Run the following packages:
```
$rosrun decoding_module dm.py
$rosrun encoding_module em.py
$rosrun brain_module bm.py
$rosrun behavioral_module bem.py
```





