# tools for clear frequency search, used by usrp_server.py

# timeline of clear frequency search:
# control program sends RequestClearFreqSearch command to a channel
#   channel enters CLR_FREQ state, waits for hardware manager to start a clear frequency search
#   
# control program sends RequestAssignedFreq command, channel responds with tfreq in kHz and noise
#   channel uses self.tfreq and self.noise 

# this replaces the following legacy qnx code:
# ros/server/reciever_handler.c receiver_assign_frequency
# ros/server/main.c:273, reading restrict file
# based on gc316_tcp_driver/main.c, fetching samples and signal processing

from drivermsg_library import *
from phasing_utils import calc_beam_azm_rad
from radar_config_constants import *
import numpy as np
import pdb

MIN_CLRFREQ_DELAY = .05 # TODO: lower this?
MAX_CLRFREQ_AVERAGE = 10
MAX_CLRFREQ_BANDWIDTH = 512
MAX_CLRFREQ_USABLE_BANDWIDTH = 300
CLRFREQ_RES = 1e3 # fft frequency resolution in kHz
RESTRICT_FILE = '/home/radar/repos/SuperDARN_MSI_ROS/linux/home/radar/ros.3.6/tables/superdarn/site/site.kod/restrict.dat.inst'

def read_restrict_file(restrict_file):
	restricted_frequencies = []
	with open(restrict_file, 'r') as f:
		for line in f:
			if line[0] == '#' or line[0] == 'd' or len(line) < 8:
				continue
			line = line.split(' ')
			restrict_start = int(line[0]) * 1e3 # convert kHz units in restrict to Hz
			restrict_end = int(line[1]) * 1e3 # convert kHz units in restrict to Hz
			restricted_frequencies.append([restrict_start, restrict_end])
			pdb.set_trace()	

	pdb.set_trace()			
	return restricted_frequencies; 

def clrfreq_search(clrfreq_struct, usrp_sockets, restricted_frequencies):
    # unpack clear frequency search parameters
    fstart = clrfreq_struct.payload['start']
    fstop = clrfreq_struct.payload['end']
    filter_bandwidth = clrfreq_struct.payload['filter_bandwidth'] # kHz (c/(2 * rsep))
    power_threshold = clrfreq_struct.payload['pwr_threshold'] # (typically .9, threshold before changing freq)
    nave = clrfreq_struct.payload['nave']

    # mimic behavior of gc316 drivers, cap nave at 10
    if nave > MAX_CLRFREQ_AVERAGE:
            nave = MAX_CLRFREQ_AVERAGE
   
    usable_bandwidth = fstop - fstart
    usable_bandwidth = np.floor(usable_bandwidth/2.0) * 2 # mimic behavior of gc316 driver, why does the old code do this?
    assert usable_bandwidth > 0, "usable bandwidth for clear frequency search must be greater than 0"
    cfreq = (fstart + (fstop-fstart/2.0)) # calculate center frequency of clrfreq search in KHz
    
    # mimic behavior of gc316 drivers, if requested search bandwidth is broader than 1024 KHz, force it to 512 KHz?
    # calculate the number of points in the FFT
    num_clrfreq_samples = np.int32(pow(2, np.ceil(np.log10(1.25*usable_bandwidth)/np.log10(2))))
    if num_clrfreq_samples > MAX_CLRFREQ_BANDWIDTH:
            num_clrfreq_samples = MAX_CLRFREQ_BANDWIDTH
            usable_bandwidth = MAX_CLRFREQ_USABLE_BANDWIDTH

    # recalculate start and stop frequency using actual search bandwidth
    fstart = np.ceil(cfreq - usable_bandwidth / 2.0)
    fstop = np.ceil(cfreq + usable_bandwidth / 2.0)
    unusable_sideband = np.int32((num_clrfreq_samples - usable_bandwidth)/2.0)
    clrfreq_samples = np.zeros(num_clrfreq_samples, dtype=np.complex64)

    # TODO: work with the sampling rate provided by the USRP 
    clrfreq_rate = 2.0 * clrfreq_struct.payload['filter_bandwidth'] * 1000 # twice the filter bandwidth, convert from kHz to Hz
	clrfreq_rate_actual = 0


    # calculate phasing
    tbeamwidth = ctrlprm_struct.payload['tbeamwidth']
    tbeam = ctrlprm_struct.payload['tbeam']
    bmazm = calc_beam_azm_rad(RADAR_NBEAMS, tbeam, tbeamwidth)
    pshift = calc_phase_increment(bmazm, cfreq)

    spectrum_power = np.zeros(num_clrfreq_samples)

    for ai in range(nave):
        cprint('gathering samples on avg {}'.format(ai), 'green')
        clrfreq_rate_actual, samples = grab_usrp_clrfreq_samples(usrp_sockets)
		spectrum_power += fft_clrfreq_samples(samples, clrfreq_rate_actual, cfreq)

	tfreq, noise = find_clrfreq_from_spectrum(samples, clrfreq_rate_actual, cfreq)

	return tfreq, noise

def find_clrfreq_from_spectrum(samples, clrfreq_rate_actual, cfreq):
	return cfreq, 10

def fft_clrfreq_samples(samples, rate, center_freq):
    fft_time_start = time.time()

    # return fft of width usable_bandwidth, kHz resolution
    # TODO: fft recentering, etc
    c_fft = np.fft.fft(combined_samples)
    # compute power..
    pc_fft = (np.real(c_fft) ** 2) + (np.imag(c_fft) ** 2) / (num_clrfreq_samples ** 2)
    pwr2 += pc_fft

    fft_time_end = time.time()
    cprint('fft time: {}'.format(fft_time_end - fft_time_start), 'yellow')
    clrfreq_cmd.client_return()

	return pc_fft

def grab_usrp_clrfreq_samples(usrp_sockets, num_clrfreq_samples, center_freq, clrfreq_rate_requested, pshift_per_antenna)
    # gather current UHD time
    combined_samples = np.zeros(num_clrfreq_samples, dtype=np.complex128)

    gettime_cmd = usrp_get_time_command(self.usrpsocks)
    gettime_cmd.transmit()

	clrfreq_rate_actual = 0

    usrptimes = []
    for usrpsock in self.usrpsocks:
            usrptimes.append(gettime_cmd.recv_time(usrpsock))

    gettime_cmd.client_return()

    # schedule clear frequency search in MIN_CLRFREQ_DELAY seconds
    clrfreq_time = np.max(usrptimes) + MIN_CLRFREQ_DELAY

    # TODO: what is the clear frequency rate? (c / (2 * rsep?))

    clrfreq_cmd = usrp_clrfreq_command(self.usrpsocks, num_clrfreq_samples, clrfreq_time, center_freq, clrfreq_rate_requested)
    clrfreq_cmd.transmit()

    # grab raw samples, apply beamforming
    for usrpsock in self.usrpsocks:
		antenna = recv_dtype(usrpsock, np.int32)
		ant_rotation = rad_to_rect(antenna * pshift_per_antenna)
		clrfreq_rate_actual = recv_dtype(usrpsock, np.float32)
		samples = recv_dtype(usrpsock, np.int16, 2 * num_clrfreq_samples)
		samples = samples[0::2] + 1j * samples[1::2]
		combined_samples += ant_rotation * samples
	
	return combined_samples, clrfreq_rate_actual


def test_clrfreq():
	pass

if __name__ == '__main__':
	test_clrfreq()

	

