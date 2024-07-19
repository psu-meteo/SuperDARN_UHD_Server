# -*- coding: utf-8 -*-
"""
Created on Tue Oct 10 10:53:41 2017

@author: mguski
"""

# %%
import numpy as np
import os
import sys
import pickle
import matplotlib.pyplot as plt

import phasing_utils

# %%


path = '/home/mguski/kodiak/clearfreq_log/'

onlyfiles = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
onlyfiles.sort()

#file = "clrfreq_dump.1507068741.1909587.pickle"
iFile = 7
file = onlyfiles[iFile]

with open(os.path.join(path, file), 'rb') as f:
    cf_dict = pickle.load(f)
    
cf_dict['sample_data']['antenna_list']
#locals().update(cf_dict)
#locals().update(sample_data)

antennas = cf_dict['sample_data']['antenna_list']
num_samples = cf_dict['sample_data']['number_of_samples']
# % %     
nBeams = 16

x_spacing = 15.24
beamsep = 3.24

iBeam = 16
#tfreq = 10.5e6
mean_freq = (cf_dict['clear_freq_range'][0]+cf_dict['clear_freq_range'][1])*1000/2

beam_angle = phasing_utils.calc_beam_azm_rad(nBeams, iBeam, beamsep)
print("beam {}".format(cf_dict['beam_angle']/np.pi*180))
phase_increment = phasing_utils.calc_phase_increment(beam_angle, mean_freq, x_spacing)
phasing_matrix = [phasing_utils.rad_to_rect(ant * phase_increment) for ant in antennas]

# mute back array antennas
antennas = np.array(antennas)
phasing_matrix = np.array(phasing_matrix)
phasing_matrix[np.logical_and(antennas > 15, antennas < 20)] = 0


# apply beamforming 
beamformed_samples = phasing_utils.beamform_uhd_samples(cf_dict['raw_samples'], phasing_matrix, num_samples, antennas, False)
beamformed_samples2 = phasing_matrix * np.matrix(cf_dict['raw_samples'])

# % %


plt.cla()    
spk_orig = np.fft.fftshift(np.abs(np.fft.fft(beamformed_samples, norm = 'ortho')) ** 2)    
spk_orig2 = np.fft.fftshift(np.abs(np.fft.fft(beamformed_samples2, norm = 'ortho')) ** 2)  
freq_vec_orig = np.fft.fftshift(np.fft.fftfreq(int(num_samples), 1/cf_dict['sample_data']['usrp_rf_rate'])) + cf_dict['sample_data']['usrp_fcenter']*1000
plt.plot(freq_vec_orig/1e6, 10*np.log10(spk_orig), label="my")
plt.plot(freq_vec_orig/1e6, 10*np.log10(spk_orig2[0]), label="m2")
spk = cf_dict['power']    
freq_vec = cf_dict["freq_vector"]
plt.plot(freq_vec/1e6, 10*np.log10(spk), label='pickle')
plt.grid(True)

plt.xlim([freq_vec_orig[0]/1e6, freq_vec_orig[-1]/1e6])
plt.xlim([cf_dict['clear_freq_range'][0]/1e3-0.3, cf_dict['clear_freq_range'][1]/1e3+0.3])
plt.plot([cf_dict['clear_freq_range'][0]/1e3, cf_dict['clear_freq_range'][1]/1e3], [20, 20], "g", linewidth=3)
plt.ylim([10, 136])
plt.title("{} antennas ({}), noise={:2.1f}, {} Hz".format(len(antennas), antennas, cf_dict['noise'], cf_dict['clrfreq']/1000))
plt.scatter(cf_dict['clrfreq']/1000, 10*np.log10(cf_dict['noise']), 100, 'k')
plt.legend()
#plt.plot(freq_vec_orig/1e6, 10*np.log10(np.fft.fftshift(np.abs(np.fft.fft(cf_dict['raw_samples'][9], norm = 'ortho'))) ** 2))
#plt.plot(np.abs(cf_dict['raw_samples'][1]))

# %%
plt.clf()    
freq_vec_orig = np.fft.fftshift(np.fft.fftfreq(int(num_samples), 1/usrp_rf_rate)) + usrp_fcenter*1000
nRows = np.ceil(len(antennas)/2)
iSubplot = 1
for ant, samples in zip(antennas, cf_dict['raw_samples']):
    ax = plt.subplot(nRows,2,iSubplot)
    spk_orig = np.fft.fftshift(np.abs(np.fft.fft(samples, norm = 'ortho')) ** 2)    
    plt.plot(freq_vec_orig/1e6, 10*np.log10(spk_orig), label="my")
    plt.xlim([freq_vec_orig[0]/1e6, freq_vec_orig[-1]/1e6])
    plt.ylim([0, 100])
    plt.grid()
    ax.set_ylabel("ant {}".format(ant))
    iSubplot += 1

# %%

import time
nSamples = 2500
nRuns = 100
x = np.random.randn(nSamples)

start_total = time.time()
for i in range(nRuns):
    start = time.time()
    np.fft.fft(x, norm = 'ortho')
    end = time.time()
    print("{} ms".format((end - start)*1000))
    
end_total = time.time()
print("\nAverage {} ms".format((end_total - start_total)/nRuns*1000))