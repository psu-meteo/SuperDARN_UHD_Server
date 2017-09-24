#!/usr/bin/python3
# function to plot raw bb samples exportet to disk by usrp_server

# -*- coding: utf-8 -*-
"""
Created on Sat Sep 23 17:13:42 2017

@author: mguski
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt




# %% used hardcoded file and path
#raw_path = "/data/image_samples/bb_data/"
#fileName = '201709231930.1.iraw.d'
#file_with_path = os.path.join(raw_path, fileName)

# %%


# %% try input arguments ...
raw_path = "/data/image_samples/bb_data/"

if len(sys.argv) == 1:
    import glob
    import os
    print('No input. Looking for newest file in {} ...'.format(raw_path))
    file_with_path = max(glob.iglob('{}*.iraw.*'.format(raw_path)), key=os.path.getctime)
    fileName = file_with_path[len(raw_path):]
    print('found :{}'.format(fileName))
else:
    fileName = sys.argv[1]
    if fileName.find("/") == -1: # no path defined => assume /data/diagnostic_samples/
        print('Only filename without path. looking for file in {} ...'.format(raw_path))
        file_with_path = os.path.join(raw_path, fileName)
    else:
        file_with_path = fileName


#%% read all data as int32
rawFile = open(file_with_path , "rb")
data = np.fromfile(rawFile, dtype=np.uint32)
rawFile.close()
# %% sort data into dictionary

# data structure:
# HEADER for complete integration period
# for each sequence:
#   start time of sequnece (sec and usec)
#   for each antenna:
#     samples of this antenna for this sequnece
#
# and then comes the next integration period staring with HEADER

all_periods = []
curr_idx = 0
while (curr_idx < len(data)):
    period_dict = {}
    period_dict['version'] = data[curr_idx]
    period_dict['year'] = data[curr_idx+1]
    period_dict['month'] = data[curr_idx+2]
    period_dict['day'] = data[curr_idx+3]
    period_dict['hour'] = data[curr_idx+4]
    period_dict['minute'] = data[curr_idx+5]
    period_dict['second'] = data[curr_idx+6]
    period_dict['microsecond'] = data[curr_idx+7]
    period_dict['nrang'] = data[curr_idx+8]
    period_dict['mpinc'] = data[curr_idx+9]
    period_dict['smsep'] = data[curr_idx+10]
    period_dict['lagfr'] = data[curr_idx+11]
    period_dict['pulseLength'] = data[curr_idx+12]
    period_dict['beam'] = data[curr_idx+13]
    
    period_dict['rfreq'] = data[curr_idx+14]
    period_dict['mppul'] = data[curr_idx+15]
    period_dict['ppat'] = data[curr_idx+16:curr_idx+16+period_dict['mppul']]
    curr_idx = curr_idx+16+period_dict['mppul']
    
    period_dict['nbaud'] = data[curr_idx]
    period_dict['pcode'] = data[curr_idx+1:curr_idx+period_dict['nbaud']+1]
    curr_idx += period_dict['nbaud']+1
    
    period_dict['nSamples'] = data[curr_idx]
    period_dict['nSeq'] = data[curr_idx+1]
    period_dict['nAntennas'] = data[curr_idx+2]
    period_dict['antenna_list']= data[curr_idx+3:curr_idx+3+period_dict['nAntennas']]
    curr_idx += period_dict['nAntennas']+3
    
    seq_list = []
    
    for iSeq in range(period_dict["nSeq"]):
        seq_dict = {}
        seq_dict["sequence_no_in_period"] = iSeq
        seq_dict["seq_start_time_sec"] = data[curr_idx]
        seq_dict["seq_start_time_usec"] = data[curr_idx+1]
        samples = []
        for iAntenna in range(period_dict['nAntennas']):
           packed_data = data[curr_idx+2+iAntenna*period_dict['nSamples']:curr_idx+2+period_dict['nSamples']*(iAntenna+1) ]
           samples.append( np.int16(packed_data >> 16) + 1j* np.int16(packed_data % 2**16))
    
        seq_dict["samples"] = samples
        seq_list.append(seq_dict)
        curr_idx += 2+period_dict['nSamples']*period_dict['nAntennas']
      
    period_dict['seq_list'] = seq_list
    all_periods.append(period_dict)

# %%
print("Read {} periods ".format(len(all_periods)))

# %% plot one sequence and all antennas
iPeriod = 0
iSequence = 8

nAntennas = all_periods[iPeriod]["nAntennas"]

subplot_size = int(np.ceil(nAntennas/2))
plt.clf()


for iAntenna, antenna_no in enumerate(all_periods[iPeriod]["antenna_list"]):
    plt.subplot(subplot_size,2,iAntenna+1)    
    plt.plot(np.real(all_periods[iPeriod]["seq_list"][iSequence]['samples'][iAntenna]))
    plt.plot(np.imag(all_periods[iPeriod]["seq_list"][iSequence]['samples'][iAntenna]))
    plt.ylabel("ant {}".format(antenna_no))
    plt.xlim([0, all_periods[iPeriod]["nSamples"]])
    plt.grid(True)

plt.subplot(subplot_size,2,1)
plt.title("{}".format(fileName))
plt.subplot(subplot_size,2,2)
plt.title("period {}, sequence {} / {}".format(iPeriod, iSequence, all_periods[iPeriod]["nSeq"]))

plt.show()
