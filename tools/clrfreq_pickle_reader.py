#!/usr/bin/python3

from pylab import *
import pickle
import pdb

#filename = './logs/clrfreq_dump.1506323040.518915.pickle'
filename = './logs/clrfreq_dump.1506323049.5147038.pickle'

clrfreq_data = pickle.load(open(filename, 'rb'))

clrfreq = clrfreq_data['clrfreq']
time = clrfreq_data['time']

raw_samples = clrfreq_data['raw_samples']

'''
ant = [20 * np.log10(fftshift(abs(fft(a)))) for a in raw_samples]
subplot(3,1,1)
plot(abs(ant[0]))
plot(abs(ant[1]))
plot(abs(ant[2]))
plot(abs(ant[3]))
subplot(3,1,2)
plot(abs(ant[4]))
plot(abs(ant[5]))
plot(abs(ant[6]))
plot(abs(ant[7]))
subplot(3,1,3)
plot(abs(ant[8]))
plot(abs(ant[9]))
plot(abs(ant[10]))
show()
'''

p = clrfreq_data['power']
f = clrfreq_data['freqs']

p[p > 1e11] = 0 # blank out restricted frequencies
p = 20 * log10(p) - 160

plot(f / 1e6, p)
plot([clrfreq/1000 - .015, clrfreq/1000 - .015, clrfreq/1000 + .015, clrfreq/1000 + .015], [-100, max(p), max(p), -100], linewidth=2)

xlabel('frequency (MHz)')
ylabel('amplitude, dB, 2 kHz FFT bins')
title('clear frequency search, {}, picked {}'.format(time, clrfreq))
show()


pdb.set_trace()
