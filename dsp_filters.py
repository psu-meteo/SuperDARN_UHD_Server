import numpy as np
import scipy.ndimage

# samples - input samples of rate rate
# trise - rise time, in seconds
# rate - sampling rate, in Hz
def gaussian_pulse(samples, trise, rate):
    # generate filter coefficients
    gauss_sigma = rate * (trise)

    filt_real = scipy.ndimage.filters.gaussian_filter1d(np.real(samples), gauss_sigma)
    filt_imag = scipy.ndimage.filters.gaussian_filter1d(np.imag(samples), gauss_sigma)

    return filt_real + 1j * filt_imag
