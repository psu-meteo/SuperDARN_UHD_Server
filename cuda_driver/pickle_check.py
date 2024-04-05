import pickle
from pylab import *
import pdb


p = pickle.load(open('debug_export.pkl', 'rb'))

freq = p[2] - 13000000
time_range = p[1]
samples = p[0]

samples = samples[0::2] + 1j * samples[1::2]

t = np.linspace(time_range[0], time_range[1], len(samples))
ref_clk = 20 * exp(1j * (2 * np.pi * freq * t))

plt.plot(np.angle(ref_clk) - np.angle(samples))

plt.show()
pdb.set_trace()
