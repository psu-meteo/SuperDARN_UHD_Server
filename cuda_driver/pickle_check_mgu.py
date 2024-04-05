#!/usr/bin/python3
import pickle
from pylab import *
import pdb
import numpy as np

p = pickle.load(open('debug_export.pkl', 'rb'))

freq = p[2] - 13000000
time_range = p[1]
samples = p[0]
if_data = p[3][2][0]
if_data = if_data[0::2] + 1j * if_data[1::2]

full_rf  = p[4][2][0::2] + 1j * p[4][2][1::2]

#t = np.linspace(time_range[0], time_range[1], len(samples))
#t_if = np.linspace(time_range[0], time_range[1], len(if_data))

t = np.arange(len(full_rf)) / 10e6
plt.plot(t, np.real(full_rf))
plt.plot(t, np.imag(full_rf))
plt.show()
pdb.set_trace()

# check rf small part
if False:
   t = np.arange(len(samples)) / 10e6
   
   demod = samples * np.exp(-1j*2*np.pi * freq *t)
   
   plt.subplot(211)
   plt.plot(t, np.real(samples))
   plt.plot(t, np.imag(samples))
   
   plt.subplot(212)
   plt.plot(t, np.real(demod))
   plt.plot(t, np.imag(demod))
   plt.title(freq)
   plt.show()
   
   pdb.set_trace()


# look at if part 
if False:
   plt.subplot(211)
   plt.plot(np.real(if_data))
   plt.plot( np.imag(if_data))
   plt.subplot(212)
   plt.plot(np.angle(if_data)*(if_data>10))
   
   
   x1 = 52500
   y1 = 0.6
   x2 = 53500
   y2 = 1.9
   delta =  (y2-y1+2*np.pi) / (x2-x1)
   y0 = y1 - delta * x1
   t = np.arange(70000)
   ph = delta *t  + y0
   plt.plot(np.angle(if_data)*(np.abs(if_data)>10))
   
   plt.plot(t,mod(ph+np.pi,2*np.pi)-np.pi)
   
   
   plt.show()
   pdb.set_trace()
