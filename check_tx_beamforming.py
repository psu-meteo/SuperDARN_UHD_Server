#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 15 14:23:52 2016

@author: mguski
"""
# %%
import pyspt
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



plotTimeVsAngle = False
plotFreqVsAngle = False

# %%
#fileName = '../PySPT/cuda_dump_tx_2016-11-15_141617.pkl'
nPulses = 8
tx_rf_samplingRate = 10e6
usrp_mixing_freq = 13e6
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

tx_sig = pyspt.Signal(tx_rf_outdata,tx_rf_samplingRate, iqInterleaved=True)
tx_sig.length /= nPulses # take only first pulse


# %% up-mixing

tmp = pyspt.dsp.resample(tx_sig, tx_sig.samplingRate + usrp_mixing_freq*2)
tx_sig = pyspt.dsp.frequency_mixer(tmp, usrp_mixing_freq)



# %% check phase in time signals
tData = tx_sig.timeData
iSample = 2000
print("angles in deg in time data: {}".format(np.angle(tData[1:,iSample] / tData[0,iSample]) /np.pi*180))




# %%
# array on x-axis, y axis in facing direction of array
# origin in middle of array
# simulate field in distace r and at nSimulationPoints on half circle

r_simulation = 3e3
nSimulationPoints = 180
freq = 10e6
c = 3e8 # 299792458.0

simulation_points_phi = np.linspace(-np.pi/2, np.pi/2, num=nSimulationPoints)
#simulation_points_phi = np.linspace(0, np.pi*2, num=nSimulationPoints)
simulation_points_x  = r_simulation * np.sin(simulation_points_phi)
simulation_points_y  = r_simulation * np.cos(simulation_points_phi)



antenna_pos_x = np.array(usrp_xPos)
antenna_pos_y = np.zeros_like(antenna_pos_x)

# set antenna in center of coordinate system
antenna_pos_x -=  np.mean(antenna_pos_x)

if len(usrp_xPos) != nAntennas:
    raise ValueError("just {} usrp entries found for {} antennas".format(len(usrp_xPos), nAntennas))


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
    print('   simulation point {} / {} : {}'.format(iSimulationPoint, nSimulationPoints, ("[{:=>"+str(int(iSimulationPoint/nSimulationPoints*20))+ "}{:>"+str(int((1-iSimulationPoint/nSimulationPoints)*20))+ "}").format(">", "]") ))
    dist_vector = np.array([np.sqrt((antenna_pos_x[iAntenna] - simulation_points_x[iSimulationPoint])**2 + (0 - simulation_points_y[iSimulationPoint])**2) for iAntenna in range(nAntennas)])
    timeDiff_vector = dist_vector / c
    
    
    sigAtPoint = tx_sig.copy
    freqDataRef = sigAtPoint.freqData_reference
    for iAntenna in range(nAntennas):
        freqDataRef[iAntenna] = np.multiply(sigAtPoint.freqData[iAntenna], np.exp(-1j*2*np.pi*freqVec*timeDiff_vector[iAntenna]))
    sigAtPoint.sum()
    receivedSig_list.append(sigAtPoint.copy)

# %%
t = pyspt.other_functions.merge(receivedSig_list)
t.comment = "beamforming simulation"
t.channelNames = ["Sim point {}".format(iSimulationPoint) for iSimulationPoint in range(nSimulationPoints)]

print("Plotting, this might take a long time.....")
# %%
iBeam = 'nan'
plt.figure()
plt.subplot(111)
plt.title("tx beamformin beam {}".format(iBeam))
ax = plt.subplot(121,projection='polar')

rmsVec = t.rms()
idxMax= np.argmax(rmsVec)
ax.plot(simulation_points_phi, rmsVec)
ax.plot([0, simulation_points_phi[idxMax]], [0, rmsVec[idxMax]], color='r', lineWidth=2)
print(simulation_points_phi[idxMax-1:idxMax+2]*180/np.pi)
plt.title('linear (max at {} degree)'.format(simulation_points_phi[idxMax]/np.pi*180))



ax = plt.subplot(122,projection='polar')
ax.plot(simulation_points_phi, 20*np.log10(rmsVec/max(rmsVec)))
ax.set_yticks([-50, -40, -30, -20, -10, -3, 0, 3])
ax.plot([simulation_points_phi[idxMax], simulation_points_phi[idxMax]], [-30, 0], color='r', lineWidth=2)
plt.title('dB')


#- %% plot time vs angle 
if plotTimeVsAngle:
    plt.figure()
#    plt.subplot(212)
    nSamples2plot = int(t.nSamples /4)
    plt.pcolor(t.timeVector[:nSamples2plot]*1e6, simulation_points_phi*180/np.pi, np.array(np.absolute(t.timeData[:,:nSamples2plot])))
    #plt.pcolor(t.timeVector*1e6, simulation_points_phi*180/np.pi, 20*np.log10(np.array(np.absolute(t.timeData))))
    plt.xlabel('time in us')
    plt.ylabel('azimuth in degree')
    plt.axis([0, t.timeVector[nSamples2plot]*1e6, simulation_points_phi[0]*180/np.pi, simulation_points_phi[-1]*180/np.pi ])
    plt.colorbar()
    plt.title(fileName)

# %% plot freq vs angle
if plotFreqVsAngle: 
    plt.figure()
    nSamples_short = 1000
    t_part = pyspt.dsp.sample_shift(t, int(t.nSamples/2-nSamples_short/2))
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

