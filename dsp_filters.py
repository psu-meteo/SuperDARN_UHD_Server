# all filters for RX downsampling and smooting TX pulses
import numpy as np
import scipy.ndimage

# filter for TX pulses
def gaussian_pulse(samples, trise, rate):
# samples - input samples of rate rate
# trise - rise time, in seconds
# rate - sampling rate, in Hz
    # generate filter coefficients
    gauss_sigma = rate * (trise)

    filt_real = scipy.ndimage.filters.gaussian_filter1d(np.real(samples), gauss_sigma)
    filt_imag = scipy.ndimage.filters.gaussian_filter1d(np.imag(samples), gauss_sigma)

    return filt_real + 1j * filt_imag

# filter also includes mixing frequencies
def kaiser_filter_s0(nTaps, channelFreqVec, samplingRate):
    gain = 3.5
    beta = 5
    filterData = np.zeros((len(channelFreqVec), nTaps,2), dtype=np.float32)
    m = nTaps - 1
    b = scipy.special.i0(beta)
    for iChannel, channelFreq in enumerate(channelFreqVec):
        if channelFreq != None:
           dbPrint('filter generation Channel {}: Freq : {} kHz'.format(iChannel, channelFreq/1e3))
           for iTap in range(nTaps):
               phi = 2 * np.pi * channelFreq * iTap / samplingRate # phase of downmixing frequency
               k = scipy.special.i0((2 * beta / m) * np.sqrt(iTap * (m - iTap)))
               filterData[iChannel,iTap,0] = gain * (k / b) * np.cos(phi)
               filterData[iChannel,iTap,1] = gain * (k / b) * np.sin(phi) # still real filter, imag part result from multiplication with oscillator
        else:
           dbPrint("filter generation: channel {}: skipping because undefined".format(iChannel))
    return filterData

#TODO: test thsi filter
def rect_filter_s0(nTaps, channelFreqVec, samplingRate):
#simple filter: real = 1, image = 0
    filterData = np.zeros((len(channelFreqVec), nTaps,2), dtype=np.float32)
    for iChannel, channelFreq in enumerate(channelFreqVec):
        if channelFreq != None:
           dbPrint('filter generation Channel {}: Freq : {} kHz'.format(iChannel, channelFreq/1e3))
           for iTap in range(nTaps):
               phi = 2 * np.pi * channelFreq * iTap / samplingRate # phase of downmixing frequency
               filterData[iChannel,iTap,0] = np.cos(phi)
               filterData[iChannel,iTap,1] = np.sin(phi) # still real filter, imag part result from multiplication with oscillator
        else:
           dbPrint("filter generation: channel {}: skipping because undefined".format(iChannel))
    return filterData



def raisedCosine_filter(nTaps, nChannels):
    # TODO: I changed this to be a complex filter. check if this makes sense for real radar signals! (mgu)
    alpha = 0.22
    filterData = np.zeros((nChannels, nTaps, 2), dtype=np.float32)
    for iTap in range(nTaps-1):
        t = 2 * iTap - (nTaps-1)
        if t == 0:
           filterData[:,iTap,:] = 1
        elif np.absolute(t) ==  (nTaps-1)/(2*alpha):
           filterData[:,iTap,:] = np.sin(np.pi/(2*alpha)) / (np.pi/(2*alpha)) * np.pi/4
        else:
           filterData[:,iTap,:] = np.sin(np.pi*t/(nTaps-1)) / (np.pi*t/(nTaps-1)) * np.cos(alpha*np.pi*t / (nTaps-1)) / (1-2*(alpha*t/(nTaps-1))**2)

    return filterData

def dbPrint(msg):
    print(' {}: {} '.format(__file__, msg) )




# OLD UNTESTED...
def matched_filter_s1(self, dmrate1):
    self.rx_filtertap_ifbb[:,:,:] = 0.

    for i in range(ntaps1/2-dmrate1/4, self.ntaps_ifbb/2+dmrate1/4):
        self.rx_filtertap_ifbb[:,i,0] = 4./dmrate1
# OLD UNTESTED...
def rolloff_filter_s1():
    self.rx_filtertap_ifbb[:,:,:] = 0
    for i in range(self.ntaps_ifbb):
        x = 8 * (2 * np.pi * (float(i) / self.ntaps_ifbb) - np.pi)
        self.rx_filtertap_ifbb[:,i,0] = 0.1 * (0.54 - 0.46 * np.cos((2 * np.pi * (float(i) + 0.5)) / self.ntaps_ifbb)) * np.sin(x) / x

    self.rx_filtertap_ifbb[:,self.ntaps_ifbb/2,0] = 0.1 * 1. # handle the divide-by-zero condition


