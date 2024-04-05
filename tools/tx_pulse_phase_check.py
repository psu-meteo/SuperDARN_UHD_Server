#!/usr/bin/python3
# reads in raw sample dump from usrp_driver

filename = '../usrp_driver/tx_rf_raw.cint16_'

cfreq = 13e6
rfrate = 10e6

#from myPlotTools import *
#from pylab import *
import matplotlib.pyplot as plt
import numpy as np

with open(filename, 'r') as raw_file:
    samples = np.fromfile(raw_file, dtype=np.int16)
    samples = samples[0::2] + 1j * samples[1::2]
    samples=samples[200000:480000]
    fc = 10821*1000 - 13e6 
    timeVec = np.arange(len(samples)) /10e6
    
    osz = np.exp(1j*2*np.pi*fc*timeVec)
    samples =  osz * samples

    plt.plot(np.real(samples))
    plt.show()


