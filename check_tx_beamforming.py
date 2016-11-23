#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 15 14:23:52 2016

@author: mguski
"""
# %%
import PySPT
import numpy as np
import matplotlib.pyplot as plt
import configparser
import pickle
import sys

# %%

if len(sys.argv) == 1:
    import glob
    import os
    print('No input. Looking for newest file in /data/diagnostic_samples/ ...')
    fileName = max(glob.iglob('/data/diagnostic_samples/cuda_dump_tx_*.pkl'), key=os.path.getctime)
    print('found :{}'.format(fileName[25:]))
else:
    fileName = sys.argv[1]
    if fileName.find("/") == -1: # no path defined => assume /data/diagnostic_samples/
        fileName = "/data/diagnostic_samples/" + fileName



# %%
#fileName = '../PySPT/cuda_dump_tx_2016-11-15_141617.pkl'
nPulses = 8
tx_rf_samplingRate = 10e6
# %%
#del sys.modules['PySPT'] 
#import PySPT

# %%

usrpconfig = configparser.ConfigParser()
usrpconfig.read('usrp_config.ini')

usrp_names = usrpconfig.sections()
usrp_arrayIdx = []
usrp_xPos     = []
usrp_tdelay = []
usrp_phaseOffset = []

for usrpName in usrp_names:
 usrp_arrayIdx.append(int(usrpconfig[usrpName]['array_idx']))
 usrp_xPos.append(float(usrpconfig[usrpName]['x_position']))
 usrp_tdelay.append(float(usrpconfig[usrpName]['tdelay']))
 usrp_phaseOffset.append(float(usrpconfig[usrpName]['phase_offset']))
 
# overwrite wrong inifile
# usrp_xPos = [15.24*arrayIdx for arrayIdx in usrp_arrayIdx]
print(usrp_xPos)

arrayconfig = configparser.ConfigParser()
arrayconfig.read('array_config.ini')

x_spacing = float(arrayconfig['array_info']['x_spacing'])
nBeams = float(arrayconfig['array_info']['nbeams'])
beamSep = float(arrayconfig['array_info']['beam_sep'])

# %% read file 

with open(fileName, 'rb') as fh:
    tx_rf_outdata = pickle.load(fh)
    
nAntennas, nSamplesRF = tx_rf_outdata.shape

tx_sig = PySPT.giSignal(tx_rf_outdata,tx_rf_samplingRate, iqInterleaved=True)
tx_sig.length /= nPulses # take only first pulse


# %% up-mixing

usrp_mixing_freq = 13e6

tmp = PySPT.resample(tx_sig, tx_sig.samplingRate + usrp_mixing_freq*2)
tx_sig = PySPT.frequencyMixer(tmp, usrp_mixing_freq)






# %%
# array on x-axis, y axis in facing direction of array
# origin in middle of array
# simulate field in distace r and at nSimulationPoints on half circle

r_simulation = 3e3
nSimulationPoints = 100
freq = 10e6
c = 3e8

simulation_points_phi = np.linspace(-np.pi/2, np.pi/2, num=nSimulationPoints)
#simulation_points_phi = np.linspace(0, np.pi*2, num=nSimulationPoints)
simulation_points_x  = r_simulation * np.sin(simulation_points_phi)
simulation_points_y  = r_simulation * np.cos(simulation_points_phi)



antenna_pos_x = np.array(usrp_xPos)
antenna_pos_y = np.zeros_like(antenna_pos_x)

if len(usrp_xPos) != nAntennas:
    print("just {} usrp entries found for {} antennas".format(len(usrp_xPos), nAntennas))
    raise VauleError("error")
"""
# plot to check
plt.figure()
plt.scatter(antenna_pos_x,antenna_pos_y )
plt.scatter(simulation_points_x,simulation_points_y )

# %% plot all distance
allDists = np.zeros((nSimulationPoints, nAntennas))
for iSimulationPoint in range(nSimulationPoints):
    for iAntenna in range(nAntennas):
        allDists[iSimulationPoint, iAntenna] =np.sqrt((antenna_pos_x[iAntenna] - simulation_points_x[iSimulationPoint])**2 + (0 - simulation_points_y[iSimulationPoint])**2) 
plt.figure()
plt.pcolor(range(8), simulation_points_phi/np.pi*180, allDists)    
plt.colorbar()
"""
# %% sim signals
freqVec = tx_sig.freqVector
receivedSig_list = []
for iSimulationPoint in range(nSimulationPoints):
    dist_vector = np.array([np.sqrt((antenna_pos_x[iAntenna] - simulation_points_x[iSimulationPoint])**2 + (0 - simulation_points_y[iSimulationPoint])**2) for iAntenna in range(nAntennas)])
    timeDiff_vector = dist_vector / c
    
    
    sigAtPoint = tx_sig.copy
    freqDataRef = sigAtPoint.freqData_reference
    for iAntenna in range(nAntennas):
        freqDataRef[iAntenna] = np.multiply(sigAtPoint.freqData[iAntenna], np.exp(-1j*2*np.pi*freqVec*timeDiff_vector[iAntenna]))
    
    sigAtPoint.sum()
    receivedSig_list.append(sigAtPoint.copy)

# %%
t = PySPT.merge(receivedSig_list)
t.comment = "beamforming simulation"
t.channelNames = ["Sim point {}".format(iSimulationPoint) for iSimulationPoint in range(nSimulationPoints)]


# %%
iBeam = 2
plt.figure()
plt.subplot(111)
plt.title("tx beamformin beam {}".format(iBeam))
ax = plt.subplot(221,projection='polar')

rmsVec = t.rms()
idxMax= np.argmax(rmsVec)
ax.plot(simulation_points_phi, rmsVec)
ax.plot([0, simulation_points_phi[idxMax]], [0, rmsVec[idxMax]], color='r', lineWidth=2)

plt.title('linear (max at {} degree)'.format(simulation_points_phi[idxMax]/np.pi*180))



ax = plt.subplot(222,projection='polar')
ax.plot(simulation_points_phi, 20*np.log10(t.rms()))
plt.title('dB')


#- %% plot time vs angle 
#plt.figure()
plt.subplot(212)
plt.pcolor(t.timeVector*1e6, simulation_points_phi*180/np.pi, np.array(np.absolute(t.timeData)))
#plt.pcolor(t.timeVector*1e6, simulation_points_phi*180/np.pi, 20*np.log10(np.array(np.absolute(t.timeData))))
plt.xlabel('time in us')
plt.ylabel('azimuth in degree')
plt.axis([0, t.timeVector[-1]*1e6, simulation_points_phi[0]*180/np.pi, simulation_points_phi[-1]*180/np.pi ])
plt.colorbar()
plt.title(fileName)

# %% plot freq vs angle
if False: 
    plt.figure()
    nSamples_short = 1000
    t_part = PySPT.sample_shift(t, int(t.nSamples/2-nSamples_short/2))
    t_part.nSamples = nSamples_short
    t_part = t

    plt.pcolor(t_part.freqVector, simulation_points_phi*180/np.pi, np.array(np.absolute(t_part.freqData)))
    #plt.pcolor(t.timeVector*1e6, simulation_points_phi*180/np.pi, 20*np.log10(np.array(np.absolute(t.timeData))))
    plt.xlabel('freq in Hz')
    plt.ylabel('azimuth in degree')
    plt.axis([t_part.freqVector[0], t_part.freqVector[-1], simulation_points_phi[0]*180/np.pi, simulation_points_phi[-1]*180/np.pi ])
    plt.colorbar()
    plt.title("truncated signal to {} samples".format(nSamples_short))


plt.show()

