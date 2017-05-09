#!/usr/bin/python3
# 
# function to 
# usage:
# 

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec
import pathlib

buffer_flag_file = "../usrp_server/bufferLiveData.flag"
data_file = '../usrp_server/liveRawData.pkl'
sys.path.insert(0, '../python_include')
from drivermsg_library import *


nAntennas = 8

def read_and_plot():
   with open(data_file, 'rb') as f:
      main_data, back_data, parDict = pickle.load(f)


   os.remove(data_file)
   with open(buffer_flag_file, "w+") as f:
      pass
   

   # put dataqueue in dict
   for item in parDict['ctrlprm_dataqueue']:
      parDict[item.name] = item.data
   
   #print(parDict)
   for key in parDict.keys():
      if key != 'ctrlprm_dataqueue':
         print(" {} = {}".format(key, parDict[key]))
   
   #plt.figure()
   #plt.pcolor(np.abs(main_data[0]))
   #plt.colorbar()
   #plt.show()
   
   #plt.figure()
   #plotData =  main_data[0][0]
   #timeVec = [iSample / parDict['baseband_samplerate'] for iSample in range(len(plotData))]
   #plt.plot(timeVec, np.abs(plotData))
   
   for iAnt in range(nAntennas):
      plt.subplot(821+iAnt) 
      plt.cla()
      plotData =  main_data[0][iAnt]
      timeVec = [iSample / parDict['baseband_samplerate'] for iSample in range(parDict['nbb_rx_samples_per_sequence'])]
      plt.plot([i /305 for i in range(len(plotData))], np.abs(plotData))
    #  plt.pcolor( np.abs(np.reshape(plotData, ( parDict['nSequences_per_period'],parDict['nbb_rx_samples_per_sequence']) )))
      #plt.pcolor( np.abs(np.reshape(plotData, (parDict['nbb_rx_samples_per_sequence'], parDict['nSequences_per_period']) )))
      plt.grid(True) 
    # plt.figure()
    # plt.plot( np.transpose(np.abs(np.reshape(plotData, ( parDict['nSequences_per_period'],parDict['nbb_rx_samples_per_sequence']) ))))
      #plt.plot( np.abs(plotData ))
      
      
      # start of sequences
      seqStartTimes = np.array(parDict['sequence_start_time_secs']) + np.array(parDict["sequence_start_time_usecs"]) / 1e6
      seqStartTimes = seqStartTimes - seqStartTimes[0]
      seqValueVec = [100000 for i in range(len(seqStartTimes))]
      #plt.plot( seqStartTimes, seqValueVec, 'o')
      if iAnt == 0:
         plt.title("nSequences_per_period={}, tbeam={}, rfreq={}".format(parDict["nSequences_per_period"], parDict["tbeam"] , parDict["rfreq"]), ) 
   #   plt.show()
      plt.ylabel("ant {} ".format(iAnt) )

   plt.subplot(413) 
   plt.cla()
   plotData = parDict['main_beamformed']
   plt.plot([i /305 for i in range(len(plotData))], np.abs(plotData))
   plt.grid(True)
   plt.title("Beamformed main samples")

 
   exampleTXsamples = [main_data[0][iAnt][305] for iAnt in range(nAntennas)]
   arrayAngle = np.angle(exampleTXsamples, deg=True)
   arrayAngle = arrayAngle - arrayAngle[0]

   plt.subplot(414)
   plt.cla()
   plt.scatter(np.arange(nAntennas), arrayAngle % 360, s=5*np.log(np.abs(exampleTXsamples)))
   
   beamsep = 3.24 / 180 * np.pi
   delta_x = 15.4
   nBeams = 16
   
   alpha = beamsep * (parDict["tbeam"] - (nBeams -1) /2 )
   phaseDiff_per_ant = - delta_x / 3e8 * parDict["rfreq"] *1000 * np.sin(alpha) / np.pi *180
   plt.plot(np.arange(nAntennas), [phaseDiff_per_ant*iAnt % 360  for iAnt in range(nAntennas)])
   plt.grid(True)
   plt.ylabel("phase difference in deg")
   plt.xlabel("antenna number")
   plt.axis([-0.5, nAntennas+0.5, 0, 360])
   plt.legend(["Theory", "Measured"])
   plt.pause(0.05)

plt.figure()
#plt.axis([0, 10, 0, 1])
plt.ion()
with open(buffer_flag_file, "w+") as f:
   pass
while True:
  
   while os.path.isfile(buffer_flag_file):
      plt.pause(0.1)
   print("Starting read and plot")
   read_and_plot() 


