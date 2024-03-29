#!/usr/bin/env python
import random
from netpyne import specs, sim
from scipy import signal
import numpy as np
import matplotlib.pyplot as plt
import scipy.io as sio
from scipy.signal import argrelmax
import pylab

import csv
import rospy
from std_msgs.msg import Float64MultiArray,String

# Configuration 1
# region AP ML DV
# M1 10 6.5 14.4
# GPi 8 3.5 7.8
# GPe 8 5.2 8.8
# Put 8.5 6.5 11.5
# VL 5.5 3.7 10.5
# VPL 4.5 4.3 9.2
# STN 5.5 3.7 7.6

# Configuration 2
# M1 10 6.5 14.4
# S1 8 5.2 15.6
# Put 8.5 6.5 11.5
# VL 5.5 3.7 10.5
# VPL 4.5 4.3 9.2
# STN 5.5 3.7 7.6

# Modify xRange and zRange for each population
# Change network size to accomodate all populations

###################### Health / Parkinson ################### 
# RS-> StrD1 connections
# GPe-< GPe connections
# Str.mod
#############################################################

class Network:
    class Spikes:
        def __init__(self):
            self.times = []

    def __init__(self,
                 has_pd=0,
                 it_num=1,
                 dbs=False,
                 th=1.2,
                 t_sim=1000,
                 n_neurons=10):

        # Parameters
        self.pd = has_pd
        self.it_num = it_num
        self.dbs = dbs
        self.t_sim = t_sim
        self.th = th#*1e-3

        # Build network
        self.netParams = self.buildNetParams()
        self.buildPopulationParameters()
        self.buildCellRules()
        self.buildSynMechParams()
        self.buildCellConnRules()
        self.buildStimParams()

        # Data: [meanFR, meanISIS, sdISIS]
        self.data = []

        #Necessary to count spikes
        self.previous_interval = 0

        #Stimulus-Time
        self.stimulus_moment = {'stim':[],'moment':[]}
        self.current_interval = 0

        # Plot PSD
        self.data_X_H = []
        self.data_X_PD = []
        self.data_Y_H = []
        self.data_Y_PD = []

        #ROS
        self.loop_rate = rospy.Rate(1)

        #Pub General
        self.pub_spikes = rospy.Publisher('motorNeuralSignal', Float64MultiArray, queue_size=10)
        self.pub_cortex_avg = rospy.Publisher('cortex', String, queue_size=10)  # used only for testing

        #Sub General
        rospy.Subscriber('sensoryNeuralSignal', String, self.neuralModel)

    def neuralModel(self,data):
        # Modify stimulus
        if data.data == "0.0035":
            sim.net.modifyStims({'conds':{'source':'Input_th'},'cellConds':{'pop':'TH'},'amp':0.0035})
            self.stimulus_moment['stim'].append('Green')
            self.stimulus_moment['moment'].append(self.current_interval)
        else:
            sim.net.modifyStims({'conds':{'source':'Input_th'},'cellConds':{'pop':'TH'},'amp':0.0012})
            self.stimulus_moment['stim'].append('NotGreen')
            self.stimulus_moment['moment'].append(self.current_interval)     

    def buildNetParams(self):
        return specs.NetParams()  # object of class NetParams to store the network parameters

    def buildPopulationParameters(self,
                                  n_strd1=10, n_strd2=10,
                                  n_th=10, n_gpi=10,
                                  n_gpe=10, n_rs=10,
                                  n_fsi=10, n_stn=10):

        self.netParams.sizeX = 7500  # x-dimension (horizontal length) size in um
        self.netParams.sizeY = 8800  # y-dimension (vertical height or cortical depth) size in um
        self.netParams.sizeZ = 5000  # z-dimension (horizontal length) size in um

        # volume occupied by each population can be customized (xRange, yRange and zRange) in um
        # xRange or xnormRange - Range of neuron positions in x-axis (horizontal length), specified 2-element list [min, max].
        # zRange or znormRange - Range of neuron positions in z-axis (horizontal depth)
        # establishing 2000 um as a standard coordinate span

        self.netParams.popParams['StrD1'] = {'cellModel': 'StrD1',
                                             'cellType': 'StrD1',
                                             'numCells': n_strd1,
                                             'xRange': [4000, 6000],
                                             'yRange': [3900, 5900],
                                             'zRange': [3000, 5000]}
        self.netParams.popParams['StrD2'] = {'cellModel': 'StrD2',
                                             'cellType': 'StrD2',
                                             'numCells': n_strd2,
                                             'xRange': [4000, 6000],
                                             'yRange': [3900, 5900],
                                             'zRange': [3000, 5000]}
        # considering VPL coordinates
        self.netParams.popParams['TH'] = {'cellModel': 'TH',
                                          'cellType': 'Thal',
                                          'numCells': n_th,
                                          'xRange': [0, 2000],
                                          'yRange': [1600, 3600],
                                          'zRange': [800, 2800]}
        self.netParams.popParams['GPi'] = {'cellModel': 'GPi',
                                           'cellType': 'GPi',
                                           'numCells': n_gpi,
                                           'xRange': [3500, 5500],
                                           'yRange': [200, 2200],
                                           'zRange': [0, 2000]}
        self.netParams.popParams['GPe'] = {'cellModel': 'GPe',
                                           'cellType': 'GPe',
                                           'numCells': n_gpe,
                                           'xRange': [3500, 5500],
                                           'yRange': [1200, 3200],
                                           'zRange': [1700, 3700]}
        # considering M1
        self.netParams.popParams['CTX_RS'] = {'cellModel': 'CTX_RS',
                                              'cellType': 'CTX_RS',
                                              'numCells': n_rs,
                                              'xRange': [5500, 7500],
                                              'yRange': [6800, 8800],
                                              'zRange': [3000, 5000]}
        self.netParams.popParams['CTX_FSI'] = {'cellModel': 'CTX_FSI',
                                               'cellType': 'CTX_FSI',
                                               'numCells': n_fsi,
                                               'xRange': [5500, 7500],
                                               'yRange': [6800, 8800],
                                               'zRange': [3000, 5000]}
        self.netParams.popParams['STN'] = {'cellModel': 'STN',
                                           'cellType': 'STN',
                                           'numCells': n_stn,
                                           'xRange': [1000, 3000],
                                           'yRange': [0, 2000],
                                           'zRange': [200, 2200]}

    def buildCellRules(self, **args):
        self.rsCellRules(**args)
        self.fsiCellRules(**args)
        self.strD1CellRules(**args)
        self.strD2CellRules(**args)
        self.thCellRules(**args)
        self.gpiCellRules(**args)
        self.gpeCellRules(**args)
        self.stnCellRules(**args)

    def rsCellRules(self):
        cellRule = {'conds': {'cellModel': 'CTX_RS', 'cellType': 'CTX_RS'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'pointps': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1,
                                            'cm': 1}
        cellRule['secs']['soma']['pointps']['Izhi'] = {'mod': 'Izhi2003b',
                                                       'a': 0.02,
                                                       'b': 0.2,
                                                       'c': -65,
                                                       'd': 8,
                                                       'f': 5,
                                                       'g': 140,
                                                       'thresh': 30}
        cellRule['secs']['soma']['vinit'] = -65
        cellRule['secs']['soma']['threshold'] = 30
        self.netParams.cellParams['CTX_RS'] = cellRule

    def fsiCellRules(self):
        cellRule = {'conds': {'cellModel': 'CTX_FSI', 'cellType': 'CTX_FSI'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'pointps': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1,
                                            'cm': 1}
        cellRule['secs']['soma']['pointps']['Izhi'] = {'mod': 'Izhi2003b',
                                                       'a': 0.1,
                                                       'b': 0.2,
                                                       'c': -65,
                                                       'd': 2,
                                                       'f': 5,
                                                       'g': 140,
                                                       'thresh': 30}
        cellRule['secs']['soma']['vinit'] = -65
        cellRule['secs']['soma']['threshold'] = 30
        self.netParams.cellParams['CTX_FSI'] = cellRule

    def strD1CellRules(self):
        cellRule = {'conds': {'cellModel': 'StrD1', 'cellType': 'StrD1'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1}
        cellRule['secs']['soma']['mechs']['Str'] = {'gmbar': (2.6e-3 - self.pd * 1.1e-3)}
        cellRule['secs']['soma']['vinit'] = random.gauss(-63.8, 5)
        cellRule['secs']['soma']['threshold'] = -10
        self.netParams.cellParams['StrD1'] = cellRule

    def strD2CellRules(self):
        cellRule = {'conds': {'cellModel': 'StrD2', 'cellType': 'StrD2'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1}
        cellRule['secs']['soma']['mechs']['Str'] = {'gmbar': (2.6e-3 - self.pd * 1.1e-3)}
        cellRule['secs']['soma']['vinit'] = random.gauss(-63.8, 5)
        cellRule['secs']['soma']['threshold'] = -10
        self.netParams.cellParams['StrD2'] = cellRule

    def thCellRules(self):
        cellRule = {'conds': {'cellModel': 'TH', 'cellType': 'Thal'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1}
        cellRule['secs']['soma']['mechs']['thalamus'] = {}
        cellRule['secs']['soma']['vinit'] = random.gauss(-62, 5)
        cellRule['secs']['soma']['threshold'] = -10
        self.netParams.cellParams['TH'] = cellRule

    def gpiCellRules(self, gahp=10e-3):
        cellRule = {'conds': {'cellModel': 'GPi', 'cellType': 'GPi'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1}
        cellRule['secs']['soma']['mechs']['GP'] = {'gahp': gahp}
        # cellRule['secs']['GPi']['mechs']['GP'] = {}
        cellRule['secs']['soma']['vinit'] = random.gauss(-62, 5)
        cellRule['secs']['soma']['threshold'] = -10
        self.netParams.cellParams['GPi'] = cellRule

    def gpeCellRules(self, gahp=10e-3):
        cellRule = {'conds': {'cellModel': 'GPe', 'cellType': 'GPe'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1}
        cellRule['secs']['soma']['mechs']['GP'] = {'gahp': gahp}
        # cellRule['secs']['GPe']['mechs']['GP'] = {}
        cellRule['secs']['soma']['vinit'] = random.gauss(-62, 5)
        cellRule['secs']['soma']['threshold'] = -10
        self.netParams.cellParams['GPe'] = cellRule

    def stnCellRules(self, gkcabar=5e-3):
        cellRule = {'conds': {'cellModel': 'STN', 'cellType': 'STN'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1}
        cellRule['secs']['soma']['mechs']['STN'] = {'dbs': self.dbs,
                                                    'gkcabar': gkcabar}
        cellRule['secs']['soma']['vinit'] = random.gauss(-62, 5)
        cellRule['secs']['soma']['threshold'] = -10
        self.netParams.cellParams['STN'] = cellRule

    def buildSynMechParams(self):
        # TH
        self.netParams.synMechParams['Igith'] = {'mod': 'Exp2Syn',
                                                 'tau1': 5,
                                                 'tau2': 5,
                                                 'e': -85}  # gpi -<th
        # GPe
        self.netParams.synMechParams['Insge,ampa'] = {'mod': 'Exp2Syn',
                                                      'tau1': 0.4,
                                                      'tau2': 2.5,
                                                      'e': 0}  # stn -> gpe
        self.netParams.synMechParams['Insge,nmda'] = {'mod': 'Exp2Syn',
                                                      'tau1': 2,
                                                      'tau2': 67,
                                                      'e': 0}  # stn -> gpe
        self.netParams.synMechParams['Igege'] = {'mod': 'Exp2Syn',
                                                 'tau1': 5,
                                                 'tau2': 5,
                                                 'e': -85}  # gpe -< gpe
        self.netParams.synMechParams['Istrgpe'] = {'mod': 'Exp2Syn',
                                                   'tau1': 5,
                                                   'tau2': 5,
                                                   'e': -85}  # D2 -> gpe
        # GPi
        self.netParams.synMechParams['Igegi'] = {'mod': 'Exp2Syn',
                                                 'tau1': 5,
                                                 'tau2': 5,
                                                 'e': -85}  # gpe -< gp
        self.netParams.synMechParams['Isngi'] = {'mod': 'Exp2Syn',
                                                 'tau1': 5,
                                                 'tau2': 5,
                                                 'e': 0}  # stn -> gpi
        self.netParams.synMechParams['Istrgpi'] = {'mod': 'Exp2Syn',
                                                   'tau1': 5,
                                                   'tau2': 5,
                                                   'e': -85}  # D1 -> gpi
        # STN
        self.netParams.synMechParams['Igesn'] = {'mod': 'Exp2Syn',
                                                 'tau1': 0.4,
                                                 'tau2': 7.7,
                                                 'e': -85}  # gpe -< stn
        self.netParams.synMechParams['Icosn,ampa'] = {'mod': 'Exp2Syn',
                                                      'tau1': 0.5,
                                                      'tau2': 2.49,
                                                      'e': 0}  # ctx -> gpe
        self.netParams.synMechParams['Icosn,nmda'] = {'mod': 'Exp2Syn',
                                                      'tau1': 2,
                                                      'tau2': 90,
                                                      'e': 0}  # ctx -> gpe
        # Str
        self.netParams.synMechParams['Igabadr'] = {'mod': 'Exp2Syn',
                                                   'tau1': 0.1,
                                                   'tau2': 13,
                                                   'e': -80}  # str -< str
        self.netParams.synMechParams['Igabaindr'] = {'mod': 'Exp2Syn',
                                                     'tau1': 0.1,
                                                     'tau2': 13,
                                                     'e': -80}  # str -< str
        self.netParams.synMechParams['Icostr'] = {'mod': 'Exp2Syn',
                                                  'tau1': 5,
                                                  'tau2': 5,
                                                  'e': 0}  # ctx -> str
        # CTX
        self.netParams.synMechParams['Iei'] = {'mod': 'Exp2Syn',
                                               'tau1': 5,
                                               'tau2': 5,
                                               'e': 0}  # rs->fsi
        self.netParams.synMechParams['Iie'] = {'mod': 'Exp2Syn',
                                               'tau1': 5,
                                               'tau2': 5,
                                               'e': -85}  # fsi<-rs
        self.netParams.synMechParams['Ithco'] = {'mod': 'Exp2Syn',
                                                 'tau1': 5,
                                                 'tau2': 5,
                                                 'e': 0}  # th->rs

    def buildCellConnRules(self, **args):
        self.thConnRules(**args)
        self.gpeConnRules(**args)
        self.gpiConnRules(**args)
        self.stnConnRules(**args)
        self.strConnRules(**args)
        self.ctxConnRules(**args)

    def thConnRules(self, **args):
        # GPi-> Th connections 
        n_neurons = min(self.netParams.popParams['TH']['numCells'],
                        self.netParams.popParams['GPi']['numCells'])
        self.netParams.connParams['GPi->th'] = {
            'preConds': {'pop': 'GPi'}, 'postConds': {'pop': 'TH'},  # GPi-> th
            'connList': [[i, i] for i in range(n_neurons)],
            'weight': 0.0336e-3,  # synaptic weight (conductance)
            'delay': 5,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Igith'}  # target synaptic mechanism

    def gpeConnRules(self,
                     stn_gpe=2,
                     gpe_gpe=2,
                     **args):
        # STN->GPe connections
        # Two aleatory GPe cells (index i) receive synapse from cells i and i - 1
        n_neurons = min(self.netParams.popParams['STN']['numCells'],
                        self.netParams.popParams['GPe']['numCells'])
        aux = random.sample(range(n_neurons), stn_gpe)
        connList = [[x - c, x] for x in aux for c in [1, 0]]
        weight = [random.uniform(0, 0.3) * 0.43e-3 for k in range(len(connList))]
        self.netParams.connParams['STN->GPe'] = {
            'preConds': {'pop': 'STN'}, 'postConds': {'pop': 'GPe'},  # STN-> GPe
            'connList': connList,  # AMPA
            'weight': weight,  # synaptic weight (conductance)
            'delay': 2,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Insge,ampa'}  # target synaptic mechanism
        # STN->GPe connections
        # Two aleatory GPe cells (index i) receive synapse from cells i and i - 1
        aux = random.sample(range(n_neurons), stn_gpe)
        connList = [[x - c, x] for x in aux for c in [1, 0]]
        weight = [random.uniform(0, 0.002) * 0.43e-3 for k in range(len(connList))]
        self.netParams.connParams['STN->GPe2'] = {
            'preConds': {'pop': 'STN'}, 'postConds': {'pop': 'GPe'},  # STN-> GPe
            'connList': connList,  # NMDA
            'weight': weight,  # synaptic weight (conductance)
            'delay': 2,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Insge,nmda'}  # target synaptic mechanism

        # GPe-< GPe connections
        n_neurons = self.netParams.popParams['GPe']['numCells']
        connList = [[(idx + ncn) % n_neurons, idx] for ncn in range(1, gpe_gpe + 1, 2)
                    for idx in range(n_neurons)] + \
                   [[idx, (idx + ncn) % n_neurons] for ncn in range(2, gpe_gpe + 1, 2)
                    for idx in range(n_neurons)]
        # connList = [[2,1],[3,2],[4,3],[5,4],[6,5],[7,6],[8,7],[9,8],[0,9],[1,0],
        #            [8,0],[9,1],[0,2],[1,3],[2,4],[3,5],[4,6],[5,7],[6,8],[7,9]]
        weight = [(0.25 + 0.75 * self.pd) * random.uniform(0, 1) * 0.3e-3 \
                  for k in range(len(connList))]
        self.netParams.connParams['GPe->GPe'] = {
            'preConds': {'pop': 'GPe'}, 'postConds': {'pop': 'GPe'},  # GPe-< GPe
            'connList': connList,
            'weight': weight,  # synaptic weight (conductance)
            'delay': 1,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Igege'}  # target synaptic mechanism

        # StrD2>GPe connections
        n_strd2 = self.netParams.popParams['StrD2']['numCells']
        n_gpe = self.netParams.popParams['GPe']['numCells']
        self.netParams.connParams['StrD2->GPe'] = {
            'preConds': {'pop': 'StrD2'}, 'postConds': {'pop': 'GPe'},  # StrD2-> GPe
            'connList': [[j, i] for i in range(n_gpe)
                         for j in range(n_strd2)],
            'weight': 0.15e-3,  # synaptic weight (conductance)
            'delay': 5,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Istrgpe'}  # target synaptic mechanism

    def gpiConnRules(self,
                     stn_gpi=5,
                     gpe_gpi=2,
                     **args):
        # STN-> GPi connections
        # Five aleatory GPi cells (index i) receive synapse from cells i and i - 1
        n_neurons = min(self.netParams.popParams['STN']['numCells'],
                        self.netParams.popParams['GPi']['numCells'])
        aux = random.sample(range(n_neurons), stn_gpi)
        connList = [[x - c, x] for x in aux for c in [1, 0]]
        self.netParams.connParams['STN->GPi'] = {
            'preConds': {'pop': 'STN'}, 'postConds': {'pop': 'GPi'},
            'connList': connList,
            'weight': 0.0645e-3,  # synaptic weight (conductance)
            'delay': 1.5,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Isngi'}  # target synaptic mechanism

        # GPe-< GPi connections 
        n_neurons = min(self.netParams.popParams['GPe']['numCells'],
                        self.netParams.popParams['GPi']['numCells'])
        self.netParams.connParams['GPe->GPi'] = {
            'preConds': {'pop': 'GPe'}, 'postConds': {'pop': 'GPi'},
            'connList':
                [[idx, (idx + ncn) % n_neurons] for ncn in range(2, gpe_gpi + 1, 2)
                 for idx in range(n_neurons)] + \
                [[(idx + ncn) % n_neurons, idx] for ncn in range(1, gpe_gpi + 1, 2)
                 for idx in range(n_neurons)],
            # [ [ idx, (idx+2) % n_neurons ] for idx in range( n_neurons ) ] + \
            # [ [ (idx+1) % n_neurons, idx ] for idx in range( n_neurons ) ],
            'weight': 0.15e-3,  # synaptic weight (conductance)
            'delay': 3,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Igegi'}  # target synaptic mechanism

        # StrD1>GPi connections
        n_strd1 = self.netParams.popParams['StrD1']['numCells']
        n_gpi = self.netParams.popParams['GPi']['numCells']
        self.netParams.connParams['StrD1->GPe'] = {
            'preConds': {'pop': 'StrD1'}, 'postConds': {'pop': 'GPi'},  # StrD1-> GPi
            'connList': [[j, i] for i in range(n_gpi)
                         for j in range(n_strd1)],
            'weight': 0.15e-3,  # synaptic weight (conductance)
            'delay': 4,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Istrgpi'}  # target synaptic mechanism

    def stnConnRules(self, **args):
        # GPe-> STN connections 
        n_neurons = min(self.netParams.popParams['GPe']['numCells'],
                        self.netParams.popParams['STN']['numCells'])
        self.netParams.connParams['GPe->STN'] = {
            'preConds': {'pop': 'GPe'}, 'postConds': {'pop': 'STN'},  # GPe-< STN
            'connList': [[(i + c) % n_neurons, i] for c in [1, 0] for i in range(n_neurons)],
            'weight': 0.15e-3,  # synaptic weight (conductance)
            'delay': 4,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Igesn'}  # target synaptic mechanism

        # CTX-> STN connections
        n_neurons = min(self.netParams.popParams['CTX_RS']['numCells'],
                        self.netParams.popParams['STN']['numCells'])
        connList = [[(i + c) % n_neurons, i] for c in [1, 0] for i in range(n_neurons)]
        weight = [random.uniform(0, 0.3) * 0.43e-3 for k in range(len(connList))]
        self.netParams.connParams['CTX->STN'] = {
            'preConds': {'pop': 'CTX_RS'}, 'postConds': {'pop': 'STN'},  # CTX-> STN
            'connList': connList,
            'weight': weight,  # synaptic weight (conductance)
            'delay': 5.9,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Icosn,ampa'}  # target synaptic mechanism
        # CTX-> STN2 
        connList = [[(i + c) % n_neurons, i] for c in [1, 0] for i in range(n_neurons)]
        weight = [random.uniform(0, 0.003) * 0.43e-3 for k in range(len(connList))]
        self.netParams.connParams['CTX->STN2'] = {
            'preConds': {'pop': 'CTX_RS'}, 'postConds': {'pop': 'STN'},  # CTX-> STN
            'connList': connList,
            'weight': weight,  # synaptic weight (conductance)
            'delay': 5.9,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Icosn,nmda'}  # target synaptic mechanism

    def strConnRules(self,
                     strd2_strd2=4,
                     strd1_strd1=3,
                     gsynmod=1,
                     **args):
        # StrD2-< StrD2 connections
        # Each StrD2 cell receive synapse from 4 aleatory StrD2 cell (except from itself)
        n_neurons = self.netParams.popParams['StrD2']['numCells']
        connList = [[x, i] for i in range(n_neurons)
                    for x in random.sample([k for k in range(n_neurons) if k != i],
                                           strd2_strd2)]
        self.netParams.connParams['StrD2-> StrD2'] = {
            'preConds': {'pop': 'StrD2'}, 'postConds': {'pop': 'StrD2'},  # StrD2-< StrD2
            'connList': connList,
            'weight': 0.1 / 4 * 0.5e-3,  # synaptic weight (conductance) -> mudar essa maluquisse
            'delay': 0,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Igabaindr'}  # target synaptic mechanism

        # StrD1-< StrD1 connections
        # Each StrD1 cell receive synapse from 3 aleatory StrD1 cell (except from itself)
        n_neurons = self.netParams.popParams['StrD1']['numCells']
        connList = [[x, i] for i in range(n_neurons)
                    for x in random.sample([k for k in range(n_neurons) if k != i],
                                           strd1_strd1)]
        self.netParams.connParams['StrD1-> StrD1'] = {
            'preConds': {'pop': 'StrD1'}, 'postConds': {'pop': 'StrD1'},  # StrD1-< StrD1
            'connList': connList,
            'weight': 0.1 / 3 * 0.5e-3,  # synaptic weight (conductance) -> mudar aqui tb
            'delay': 0,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Igabadr'}  # target synaptic mechanism

        # RS-> StrD1 connections 
        n_neurons = min(self.netParams.popParams['CTX_RS']['numCells'],
                        self.netParams.popParams['StrD1']['numCells'])
        self.netParams.connParams['RS->StrD1'] = {
            'preConds': {'pop': 'CTX_RS'}, 'postConds': {'pop': 'StrD1'},  # RS-> StrD1
            'connList': [[i, i] for i in range(n_neurons)],
            'weight': (0.07 - 0.044 * self.pd) * 0.43e-3 * gsynmod,  # synaptic weight (conductance)
            'delay': 5.1,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Icostr'}  # target synaptic mechanism

        # RS-> StrD2 connections 
        n_neurons = min(self.netParams.popParams['CTX_RS']['numCells'],
                        self.netParams.popParams['StrD2']['numCells'])
        self.netParams.connParams['RS->StrD2'] = {
            'preConds': {'pop': 'CTX_RS'}, 'postConds': {'pop': 'StrD2'},  # RS-> StrD2 
            'connList': [[i, i] for i in range(n_neurons)],
            'weight': 0.07 * 0.43e-3 * gsynmod,  # synaptic weight (conductance)
            'delay': 5.1,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Icostr'}  # target synaptic mechanism

    def ctxConnRules(self,
                     rs_fsi=4,
                     fsi_rs=4,
                     **args):
        # RS -> FSI connections
        # Each FSI cell receive synapse from 4 aleatory RS cells
        n_rs = self.netParams.popParams['CTX_RS']['numCells']
        n_fsi = self.netParams.popParams['CTX_FSI']['numCells']
        connList = [[x, i] for i in range(n_fsi)
                    for x in random.sample([k for k in range(n_rs) if k != i],
                                           rs_fsi)]
        self.netParams.connParams['ctx_rs->ctx_fsi'] = {
            'preConds': {'pop': 'CTX_RS'}, 'postConds': {'pop': 'CTX_FSI'},  # ctx_rs -> ctx_fsi
            'connList': connList,
            'weight': 0.043e-3,  # synaptic weight (conductance)
            'delay': 1,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Iei'}  # target synaptic mechanism

        # FSI -> RS connections
        # Each RS cell receive synapse from 4 aleatory FSI cells
        connList = [[x, i] for i in range(n_rs)
                    for x in random.sample([k for k in range(n_fsi) if k != i],
                                           fsi_rs)]
        self.netParams.connParams['ctx_fsi->ctx_rs'] = {
            'preConds': {'pop': 'CTX_FSI'}, 'postConds': {'pop': 'CTX_RS'},  # ctx_fsi -< ctx_rs
            'connList': connList,
            'weight': 0.083e-3,  # synaptic weight (conductance)
            'delay': 1,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Iie'}  # target synaptic mechanism

        # Th -> RS connections
        n_neurons = min(self.netParams.popParams['TH']['numCells'],
                        self.netParams.popParams['CTX_RS']['numCells'])
        self.netParams.connParams['th->ctx_rs'] = {
            'preConds': {'pop': 'TH'}, 'postConds': {'pop': 'CTX_RS'},  # th -> ctx_rs
            'connList': [[i, i] for i in range(n_neurons)],
            'weight': 0.0645e-3,  # synaptic weight (conductance)
            'delay': 5,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Ithco'}  # target synaptic mechanism

    def buildStimParams(self,
                        amp_th=1.2e-3, amp_gpe=3e-3,
                        amp_gpi=3e-3, amp_stn=0,
                        amp_fs=0, amp_rs=0,
                        amp_dstr=0, amp_istr=0):
        bin_fs = 0;
        bin_rs = 0;
        bin_gpe = 0;
        bin_gpi = 0;
        bin_stn = 0;
        bin_dstr = 0;
        bin_istr = 0;
        bin_th = 0;

        # FS receve a constante 3 density current or 1 during cortical stimulation
        self.netParams.stimSourceParams['Input_FS'] = {'type': 'IClamp',
                                                       'delay': 0,
                                                       'dur': self.t_sim,
                                                       'amp': bin_fs * -1}
        self.netParams.stimTargetParams['Input_FS->FS'] = {'source': 'Input_FS',
                                                           'conds': {'pop': 'CTX_FSI'},
                                                           'sec': 'fsi',
                                                           'loc': 0}

        # RS receve a constante 3 density current or 1 during cortical stimulation
        self.netParams.stimSourceParams['Input_RS'] = {'type': 'IClamp',
                                                       'delay': 0,
                                                       'dur': self.t_sim,
                                                       'amp': bin_rs * -1 + amp_rs}
        self.netParams.stimTargetParams['Input_RS->RS'] = {'source': 'Input_RS',
                                                           'conds': {'pop': 'CTX_RS'},
                                                           'sec': 'rs',
                                                           'loc': 0}

        # GPe receve a constante 3 density current or 1 during cortical stimulation
        self.netParams.stimSourceParams['Input_GPe'] = {'type': 'IClamp',
                                                        'delay': 0,
                                                        'dur': self.t_sim,
                                                        'amp': bin_gpe * -1 + amp_gpe}
        self.netParams.stimTargetParams['Input_GPe->GPe'] = {'source': 'Input_GPe',
                                                             'conds': {'pop': 'GPe'},
                                                             'sec': 'GPe',
                                                             'loc': 0}

        # GPi receve a constante 3 density current
        self.netParams.stimSourceParams['Input_GPi'] = {'type': 'IClamp',
                                                        'delay': 0, 'dur': self.t_sim,
                                                        'amp': bin_gpi * -1 + amp_gpi}
        self.netParams.stimTargetParams['Input_GPi->GPi'] = {'source': 'Input_GPi',
                                                             'conds': {'pop': 'GPi'},
                                                             'sec': 'GPi',
                                                             'loc': 0}

        # STN receve a constante 3 density current or 1 during cortical stimulation
        self.netParams.stimSourceParams['Input_STN'] = {'type': 'IClamp',
                                                        'delay': 0,
                                                        'dur': self.t_sim,
                                                        'amp': bin_stn * -1 + amp_stn}
        self.netParams.stimTargetParams['Input_STN->STN'] = {'source': 'Input_STN',
                                                             'conds': {'pop': 'STN'},
                                                             'sec': 'STN',
                                                             'loc': 0}

        # dStr receve a constante 3 density current
        self.netParams.stimSourceParams['Input_StrD1'] = {'type': 'IClamp',
                                                          'delay': 0,
                                                          'dur': self.t_sim,
                                                          'amp': bin_dstr * -1 + amp_dstr}
        self.netParams.stimTargetParams['Input_StrD1->StrD1'] = {'source': 'Input_StrD1',
                                                                 'conds': {'pop': 'StrD1'},
                                                                 'sec': 'StrD1',
                                                                 'loc': 0}

        # iStr receve a constante 3 density current
        self.netParams.stimSourceParams['Input_StrD2'] = {'type': 'IClamp',
                                                          'delay': 0, 'dur': self.t_sim,
                                                          'amp': bin_istr * -1 + amp_istr}
        self.netParams.stimTargetParams['Input_StrD2->StrD2'] = {'source': 'Input_StrD2',
                                                                 'conds': {'pop': 'StrD2'},
                                                                 'sec': 'StrD2',
                                                                 'loc': 0}

        # Thalamus receve a constante 1.2 density current
        self.netParams.stimSourceParams['Input_th'] = {'type': 'IClamp',
                                                       'delay': 0,
                                                       'dur': self.t_sim,
                                                       'amp': self.th}  # 0.0012
        self.netParams.stimTargetParams['Input_th->TH'] = {'source': 'Input_th',
                                                           'conds': {'pop': 'TH'},
                                                           'sec': 'th',
                                                           'loc': 0}

    def extractLFP(self):

        lfp = sim.allSimData['LFP']
        lfp = np.transpose(lfp, [1, 0])

        # calculate LFP using Welch method
        lfp_f, lfp_dimensions = signal.welch(lfp[0], 1000, nperseg=1024, detrend=False)
        lfp_fft = np.zeros((8, lfp_dimensions.shape[0]))
        for i in range(lfp.shape[0]):
            lfp_f, lfp_fft[i, :] = signal.welch(lfp[i], 1000, nperseg=1024, detrend=False)

        return lfp_f, lfp_fft


    def extractSpikes(self):
        spikes = self.Spikes
        dStr_APs = [spikes() for k in range(10)]
        iStr_APs = [spikes() for k in range(10)]
        TH_APs = [spikes() for k in range(10)]
        GPi_APs = [spikes() for k in range(10)]
        GPe_APs = [spikes() for k in range(10)]
        Cor_APs = [spikes() for k in range(10 + 10)]  # rs + fs
        STN_APs = [spikes() for k in range(10)]

        for i in range(0, len(sim.allSimData.spkt)):
            if (sim.allSimData.spkid[i] >= 0 and sim.allSimData.spkid[i] <= 9):
                dStr_APs[int(sim.allSimData.spkid[i])].times = dStr_APs[
                                                                   int(sim.allSimData.spkid[i])].times + [
                                                                   sim.allSimData.spkt[i]]
            elif (sim.allSimData.spkid[i] >= 10 and sim.allSimData.spkid[i] <= 19):
                iStr_APs[int(sim.allSimData.spkid[i] - 10)].times = iStr_APs[
                                                                        int(sim.allSimData.spkid[i] - 10)].times + [
                                                                        sim.allSimData.spkt[i]]
            elif (sim.allSimData.spkid[i] >= 20 and sim.allSimData.spkid[i] <= 29):
                TH_APs[int(sim.allSimData.spkid[i] - 20)].times = TH_APs[
                                                                      int(sim.allSimData.spkid[i] - 20)].times + [
                                                                      sim.allSimData.spkt[i]]
            elif (sim.allSimData.spkid[i] >= 30 and sim.allSimData.spkid[i] <= 39):
                GPi_APs[int(sim.allSimData.spkid[i] - 30)].times = GPi_APs[
                                                                       int(sim.allSimData.spkid[i] - 30)].times + [
                                                                       sim.allSimData.spkt[i]]
            elif (sim.allSimData.spkid[i] >= 40 and sim.allSimData.spkid[i] <= 49):
                GPe_APs[int(sim.allSimData.spkid[i] - 40)].times = GPe_APs[
                                                                       int(sim.allSimData.spkid[i] - 40)].times + [
                                                                       sim.allSimData.spkt[i]]
            elif (sim.allSimData.spkid[i] >= 50 and sim.allSimData.spkid[i] <= 69):
                Cor_APs[int(sim.allSimData.spkid[i] - 50)].times = Cor_APs[
                                                                       int(sim.allSimData.spkid[i] - 50)].times + [
                                                                       sim.allSimData.spkt[i]]
            elif (sim.allSimData.spkid[i] >= 70 and sim.allSimData.spkid[i] <= 79):
                STN_APs[int(sim.allSimData.spkid[i] - 70)].times = STN_APs[
                                                                       int(sim.allSimData.spkid[i] - 70)].times + [
                                                                       sim.allSimData.spkt[i]]
        
    def buildSimConfig(self, dt=0.1, lfp=False):
        # Simulation parameters
        simConfig = specs.SimConfig()
        simConfig.duration = self.t_sim  # Duration of the simulation, in ms
        simConfig.dt = dt  # Internal integration timestep to use
        simConfig.verbose = False  # Show detailed messages
        simConfig.printPopAvgRates = True

        # Saving
        simConfig.filename = 'model_output'  # Set file output name
        simConfig.saveFileStep = 1 # step size in ms to save data to disk
        simConfig.savePickle = False # save to pickle file
        simConfig.saveJson = False # save to json file
        simConfig.saveMat = True # save to mat file
        simConfig.saveTxt = False # save to txt file
        simConfig.saveDpk = False # save to .dpk pickled file
        simConfig.saveHDF5 = False # save to HDF5 file

        # Recording
        simConfig.recordStep = 1  # Step size in ms to save data (eg. V traces, LFP, etc)
        simConfig.recordCells = ['allCells']
        simConfig.recordSpikesGids = True
  
        # Plot
        simConfig.recordTraces = {'V_soma':{'sec':'soma','loc':0.5,'var':'v'}}
        simConfig.analysis['plotRaster'] = {'syncLines': False,'saveData':'./ras/spikes','saveFig':'./ras/raster'}
        simConfig.analysis['plotTraces'] = {'include':[20,21,22,23,24,25,26,27,28,29,51,52,53,54,55,56,57,58,59],'showFig':False}
        simConfig.analysis['plotSpikeHist'] = {'include':['TH','CTX_RS'],'timeRange':[0,self.t_sim],'binSize':1000,'overlay':False,'graphType':'bar','yaxis':'rate','saveFig':'./stim/hist','showFig':False}
        simConfig.analysis['plotRatePSD'] = {'include':['CTX_RS'],'timeRange':[0,self.t_sim],'binSize':5,'NFFT':256,'noverlap':128,'smooth':0,'maxFreq':100,'overlay':True,'popColors':None,'ylim':None,'figSize':[10,8],'fontSize':12,'saveData':'./filename.pkl','saveFig':'./PSD','showFig':False}

        # lfp and plot
        #lfp = False
        #if lfp:
        #    simConfig.analysis['plotRaster'] = True
        #    simConfig.recordLFP = [[5000, 4900, 4000],  # StrD1
        #                           [5000, 4900, 4000],  # StrD2
        #                           [1000, 2600, 1800],  # TH
        #                           [4500, 1200, 1000],  # GPi
        #                           [4500, 2200, 2700],  # GPe
        #                           [6500, 7800, 4000],  # CtxRS
        #                           [6500, 7800, 4000],  # CtxFSI
        #                           [2000, 1200, 1200]]  # STN
        #    simConfig.saveLFPCells = True
        
        return simConfig

    def simulate(self, experiment, has_pd=1, dt=0.1, lfp=False, interval = 1000):
        # Config
        simConfig = self.buildSimConfig(dt=dt, lfp=lfp)
        if has_pd == 1:
            simConfig.filename = 'model_output_PD_'+str(self.th)+'('+str(experiment)+')'
        else:
            simConfig.filename = 'model_output_H_'+str(self.th)+'('+str(experiment)+')'

        # Create
        sim.initialize(simConfig=simConfig,netParams=self.netParams)
        sim.net.createPops()
        sim.net.createCells()
        sim.net.connectCells()
        sim.net.addStims()
        sim.setupRecording()
        # Run
        sim.updateInterval = interval
        sim.runSimWithIntervalFunc(sim.updateInterval, self.communicateToROS)
        # Analyze
        sim.gatherData()
        sim.saveData()

        # Plot default
        #sim.analysis.plotData()

        # Plot PSD
        #(f,d) = sim.analysis.plotRatePSD(include=['CTX_RS'],saveFig='./PSD',showFig=False)

        #data = d.get(u'allPower')
        #if has_pd == 0:
        #    self.data_X_H = data[0][1]
        #else:
        #    self.data_X_PD = data[0][1]

        #data = d.get(u'allSignal')
        #if has_pd == 0:
        #    self.data_Y_H = data[0]
        #else:
        #    self.data_Y_PD = data[0]

        # Population rates
        #freq_disp = [sim.allSimData.popRates['CTX_FSI'],
        #             sim.allSimData.popRates['CTX_RS'],
        #             sim.allSimData.popRates['GPe'],
        #             sim.allSimData.popRates['GPi'],
        #             sim.allSimData.popRates['STN'],
        #             sim.allSimData.popRates['StrD1'],
        #             sim.allSimData.popRates['StrD2'],
        #             sim.allSimData.popRates['TH']] 
        #for i in range(0, 8):
        #    freq_disp[i] = round(freq_disp[i], 2)
        #    print freq_disp[i]

        # Plot window
        #window = 6000
        #print len(self.stimulus_moment['moment'])
        #for i in range(0,5):
        #    if i < len(self.stimulus_moment['moment']):
        #        time_bef = int(self.stimulus_moment['moment'][i])-window
        #        time_aft = int(self.stimulus_moment['moment'][i])+window
        #        if time_bef < 0:
        #            time_bef = 0
        #        if time_aft > self.t_sim:
        #            time_aft = self.t_sim
                #Plot: 
        #        sim.cfg.analysis.plotSpikeHist['saveFig'] = './stim/hist_'+str(i)
        #        sim.cfg.analysis.plotSpikeHist['timeRange'] = [time_bef,time_aft]
        #        sim.cfg.analysis.plotSpikeHist['showFig'] = False
        #        sim.cfg.analysis.plotTraces['saveFig'] = str(i)
        #        sim.cfg.analysis.plotTraces['timeRange'] = [time_bef,time_aft]
        #        sim.cfg.analysis.plotTraces['showFig'] = False
        #        sim.analysis.plotData()

        return self.data    

    def communicateToROS(self,time):
        # Check time
        self.current_interval = time

        # Collect Data 
        sim.gatherData()

        # Variables to calculate ISIs
        one_second_data = [[],[],[],[],[],[],[],[],[],[]]

        # Count spikes of cortex neurons        
        countSpikes = 0
        for i in range(self.previous_interval,len(sim.allSimData['spkid'])):
            spike_id = sim.allSimData['spkid'][i]
            if spike_id >= 50 and spike_id < 60:
                countSpikes += 1
                pos = int(spike_id%50)
                one_second_data[pos].append(sim.allSimData['spkt'][i])                
        
        # Save position for the next second          
        self.previous_interval = len(sim.allSimData['spkid'])
        
        # Mean firing rate on Cortex neurons
        mean_FR = countSpikes/10.0
        
        # Calculate ISIs
        isis_mean = np.zeros(10)
        isis_std  = np.zeros(10)
        isis = [[],[],[],[],[],[],[],[],[],[]]
        for i in range(0,10):
            for t in range(0,len(one_second_data[i])):
                if t>0:
                    diff = one_second_data[i][t]-one_second_data[i][t-1]
                    isis[i].append(diff)
            #print isis[i]
            if len(isis[i]) > 0:
                isis_mean[i] = np.mean(isis[i])
                #print isis_mean[i]
                isis_std[i] = np.std(isis[i])
                #print isis_std[i]
            else:
                isis_mean[i] = 0
                isis_std[i] = 0

        # Calculate std and mean of ISIs
        isis_mean_ctx = np.mean(isis_mean)
        isis_std_ctx = np.mean(isis_std)

        # Publish: mean firing rate only
        msg = String()
        msg.data = str(mean_FR)
        self.pub_cortex_avg.publish(msg)

        # Publish: mean firing rate and std and mean of ISIs of Cortex neurons
        msg = Float64MultiArray()
        msg.data = [mean_FR,isis_mean_ctx,isis_std_ctx]
        self.pub_spikes.publish(msg)

        # Save data
        self.data.append([mean_FR,isis_mean_ctx,isis_std_ctx])

    def saveCSV(self,filename,data):
        # Data to CSV 
        np.savetxt(file_name, data, delimiter=",")

        
if __name__ == '__main__':
    try:
        # ROS
        rospy.init_node('neural_model_rat', anonymous=False)

        # Data
        data = np.empty(shape=[0, 3])

        # Thalamic current
        current = 0.0012 #0.0012 #0.0035
        
        # Healthy/PD
        pd = 0
        data = np.empty(shape=[0, 3]) 
        for exp in range(1,2):
            network1 = Network(th = current, t_sim = 120000, has_pd=pd)
	    output = network1.simulate(exp, pd, dt = 0.1, lfp = False, interval = 1000)
            data = np.concatenate([data,output])
        
        # Save output in csv file
        #file_name = 'H.csv'
        #np.savetxt(file_name, data, delimiter=",")

        # Data
        #data = np.empty(shape=[0, 3])

        # PD
        #pd = 1
        #data = np.empty(shape=[0, 3])
        #for exp in range(1,2):
        #    network2 = Network(th = current, t_sim = 90000, has_pd=pd)
        #    output = network2.simulate(exp, pd, dt = 0.1, lfp = False, interval = 1000)
        #    data = np.concatenate([data,output])
      
        # Save output in csv file
        #file_name = 'PD.csv'
        #np.savetxt(file_name, data, delimiter=",")

        # Plot
        #plt.figure(figsize=(15,7))
        #plt.rcParams.update({'font.size': 22})
        #plt.title('PSD (0.0035 mA)')
        #plt.plot(network1.data_X_H,network1.data_Y_H, label='Healthy State', linewidth=2)
        #plt.plot(network2.data_X_PD,network2.data_Y_PD, '--', label='Parkinsonian State', linewidth=2)
        #plt.legend(loc="upper right")
        #plt.xlabel('Frequency (Hz)')
        #plt.ylim(-8,17)
        #plt.xlim(0,50)
        #plt.ylabel('Power Spectral Density (db/Hz)')
        #plt.show()
        #plt.fill_betweenx(range(-8,20,1), 12, 30, color='green', alpha='0.2')
        #plt.savefig('foo.png')

    except rospy.ROSInterruptException:
        pass
              
