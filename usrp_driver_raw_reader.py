# reads in raw sample dump from usrp_driver

filename = "raw_samples_ant_0.cint16"

cfreq = 13e6
rfrate = 10e6

from myPlotTools import *
from pylab import *


with open(filename, 'r') as raw_file:
    samples = np.fromfile(raw_file, dtype=np.int16)
    samples = samples[0::2] + 1j * samples[1::2]

    plot_freq(samples, rfrate)

