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
   
   
   plt.cla()
   plotData =  main_data[0][0]
   timeVec = [iSample / parDict['baseband_samplerate'] for iSample in range(parDict['nbb_rx_samples_per_sequence'])]
   plt.plot( np.abs(plotData))
 #  plt.pcolor( np.abs(np.reshape(plotData, ( parDict['nSequences_per_period'],parDict['nbb_rx_samples_per_sequence']) )))
   #plt.pcolor( np.abs(np.reshape(plotData, (parDict['nbb_rx_samples_per_sequence'], parDict['nSequences_per_period']) )))
   
 # plt.figure()
 # plt.plot( np.transpose(np.abs(np.reshape(plotData, ( parDict['nSequences_per_period'],parDict['nbb_rx_samples_per_sequence']) ))))
   #plt.plot( np.abs(plotData ))
   
   
   # start of sequences
   seqStartTimes = np.array(parDict['sequence_start_time_secs']) + np.array(parDict["sequence_start_time_usecs"]) / 1e6
   seqStartTimes = seqStartTimes - seqStartTimes[0]
   seqValueVec = [100000 for i in range(len(seqStartTimes))]
   #plt.plot( seqStartTimes, seqValueVec, 'o')
   plt.title("nSequences_per_period={}, tbeam={}, rfreq={}".format(parDict["nSequences_per_period"], parDict["tbeam"] , parDict["rfreq"]), ) 
#   plt.show()
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


