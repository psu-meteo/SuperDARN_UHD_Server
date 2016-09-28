import numpy as np
import matplotlib.pyplot as plt
import pdb

def plot_freq(timeData, samplingRate, iqInterleaved=False, show=True):

   if iqInterleaved:
      timeData = timeData[0::2] + 1j* timeData[1::2]


   tVec = [ t / samplingRate for t in range(timeData.size) ]
   
#   plt.figure(figsize=(10,7))
#   plt.figure()

   spk = np.fft.fft(timeData)*2 / timeData.size
   freqVec = np.fft.fftfreq(timeData.size, 1/samplingRate )
   sortIdx = np.argsort(freqVec)
   
##   pdb.set_trace()
 #  ax = plt.subplot(111)
   ax = plt.gca()
  # plt.plot(freqVec[sortIdx], np.abs(spk[sortIdx]))
   plt.plot(freqVec[sortIdx], 20*np.log10(np.abs(np.real(spk[sortIdx]))), label='real')
   plt.plot(freqVec[sortIdx], 20*np.log10(np.abs(np.imag(spk[sortIdx]))), label='imag')
   plt.grid(True)
   ax.set_xlim([freqVec[sortIdx[1]], freqVec[sortIdx[-1]]])
   #ax.set_xscale('log')
   plt.xlabel('freq in Hz')
   plt.legend()
  # plt.ylabel('abs spectrum in dBFS')	
   if show:
      plt.show()



def plot_time_freq(timeData, samplingRate):
   tVec = [ t / samplingRate for t in range(timeData.size) ]
   
#   plt.figure(figsize=(10,7))
   plt.figure()
   plt.subplot(121)
   plt.plot(tVec, np.real(timeData))

   plt.xlabel('time in s')
   plt.ylabel('real ')
  # plt.axis([0, tVec_bb[-1], -1.1, 1.1])
   plt.grid(True)

   spk = np.fft.fft(timeData)*2 / timeData.size
   freqVec = np.fft.fftfreq(timeData.size, 1/samplingRate )
   sortIdx = np.argsort(freqVec)
   
##   pdb.set_trace()
   ax = plt.subplot(122)
  # plt.plot(freqVec[sortIdx], np.abs(spk[sortIdx]))
   plt.plot(freqVec[sortIdx], 20*np.log10(np.abs(spk[sortIdx])))
   plt.grid(True)
   #plt.axis([freqVec[sortIdx[1]], freqVec[sortIdx[-1]], -140, 10])
   #ax.set_xscale('log')
   plt.xlabel('freq in Hz')
  # plt.ylabel('abs spectrum in dBFS')	
   plt.show()
