# tools for clear frequency search, used by usrp_server.py
# timeline of clear frequency search:
# control program sends RequestClearFreqSearch command to a channel
#   channel enters CLR_FREQ state, waits for hardware manager to start a clear frequency search
#   
# control program sends RequestAssignedFreq command, channel responds with tfreq in kHz and noise
#   channel uses self.tfreq and snsmitelf.noise 

# this replaces the following legacy qnx code:
# ros/server/reciever_handler.c receiver_assign_frequency
# ros/server/main.c:273, reading restrict file
# based on gc316_tcp_driver/main.c, fetching samples and signal processing

from drivermsg_library import *
from rosmsg import *
from phasing_utils import calc_beam_azm_rad, calc_phase_increment, rad_to_rect, beamform_uhd_samples
from radar_config_constants import *
import matplotlib.pyplot as plt
import numpy as np
import scipy.signal

MIN_CLRFREQ_DELAY = .20 # TODO: lower this?
MAX_CLRFREQ_AVERAGE = 5 
MAX_CLRFREQ_BANDWIDTH = 512
MAX_CLRFREQ_USABLE_BANDWIDTH = 300
CLEAR_FREQUENCY_FILTER_FUDGE_FACTOR = 1.5
CLRFREQ_RES = 1e3 # fft frequency resolution in kHz
RESTRICTED_POWER = 1e12 # arbitrary high power for restricted frequency
RESTRICT_FILE = '/home/radar/repos/SuperDARN_MSI_ROS/linux/home/radar/ros.3.6/tables/superdarn/site/site.kod/restrict.dat.inst'
PLOT_CLEAR_FREQUENCY_SEARCH = False 
OBEY_RESTRICTED_FREQS = True 

DEBUG = 1
def dbPrint(msg):
   if DEBUG:
     print("clear_frequency_serch.py : " + msg)


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

    return restricted_frequencies; 

def calc_clear_freq_on_raw_samples(raw_samples, sample_meta_data, restricted_frequencies, clear_freq_range, beam_angle):
    # unpack meta data 
    antennas = sample_meta_data['antenna_list']
    num_samples = sample_meta_data['number_of_samples']
    tfreq = np.mean(clear_freq_range)
    x_spacing = sample_meta_data['x_spacing']

    usrp_center_freq = sample_meta_data['usrp_fcenter']
    usrp_sampling_rate = sample_meta_data['usrp_rf_rate']

    # calculate phasing matrix 
    phase_increment = calc_phase_increment(beam_angle, tfreq, x_spacing)
    phasing_matrix = [rad_to_rect(ant * phase_increment) for ant in antennas]

    # apply beamforming 
    beamformed_samples = beamform_uhd_samples(raw_samples, phasing_matrix, num_samples, antennas, False)

    # apply spectral estimation (takes about 20-40 ms) TODO: why [0]?
    spectrum_power = fft_clrfreq_samples(raw_samples)[0]
   
    # calculate spectrum range of rf samples given sampling rate and center frequency
    fstart_actual = usrp_center_freq - usrp_sampling_rate / 2.0 
    fstop_actual = usrp_center_freq + usrp_sampling_rate / 2.0 
    spectrum_freqs = np.arange(fstart_actual, fstop_actual, CLRFREQ_RES)
 
    # mask restricted frequencies
    if restricted_frequencies:
        spectrum_power = mask_spectrum_power_with_restricted_freqs(spectrum_power, spectrum_freqs, restricted_frequencies)
  
    # search for a clear frequency within the given frequency range
    fstart = clear_freq_range[0] * 1e3 
    fstop = (clear_freq_range[0] + clear_freq_range[1]) * 1e3
    tfreq, noise = find_clrfreq_from_spectrum(spectrum_power, spectrum_freqs, fstart, fstop)

    return tfreq, noise

# stale clear frequency search, used before we added in proper swing buffer support..
def clrfreq_search(clrfreq_struct, usrp_sockets, restricted_frequencies, tbeam_number, tbeam_width_deg, nBeams, x_spacing):
    dbPrint("enter clrfreq_search")
    # unpack clear frequency search parameters
    fstart = clrfreq_struct.payload['start'] * 1000 # convert kHz in struct to Hz
    fstop = clrfreq_struct.payload['end'] * 1000  # convert kHz in struct to Hz
    search_bandwidth_requested = clrfreq_struct.payload['filter_bandwidth'] * 1000 # kHz (c/(2 * rsep))
    power_threshold = clrfreq_struct.payload['pwr_threshold'] # (typically .9, threshold before changing freq)
    nave = clrfreq_struct.payload['nave']

    # mimic behavior of gc316 drivers, cap nave
    if nave > MAX_CLRFREQ_AVERAGE:
            nave = MAX_CLRFREQ_AVERAGE
    
    # pick reasonable sampling rate from USRP
    # so, master clock frequency divided by an even number

    # first, pick something at least twice the requested bandwidth, with some extra room for channelization filter rolloff
    search_rate_requested = search_bandwidth_requested * CLEAR_FREQUENCY_FILTER_FUDGE_FACTOR 
    
    # calculate some supported sampling rates that are cleanly divisble by 1 kHz, this should probably be moved out of clrfreq search, profile me to see if I'm fast enough..
    search_rate_supported = np.array([USRP_MASTER_CLOCK_FREQ / (4 * n) for n in range(4,200)]) / 1e3
    search_rate_supported_integer = np.uint32(search_rate_supported)
    search_rate_supported = search_rate_supported[search_rate_supported == search_rate_supported_integer] * 1e3
    
    # pick the lowested supported sampling rate that is equal to or greater than requested sampling rate 
    if search_rate_requested > min(search_rate_supported):
        search_rate_actual = search_rate_supported[np.argmax(np.diff(search_rate_supported > search_rate_requested))]
    else:
        search_rate_actual = min(search_rate_supported)

    assert search_rate_actual >= search_rate_requested, "actual clear frequency search sampling rate must be equal or faster to requested rate"
    
    # calculate center frequency of clrfreq search
    center_freq = (fstart + (fstop-fstart)/2.0)
    
    # calculate the number of points in the FFT
    num_clrfreq_samples = int(np.round(search_rate_actual / CLRFREQ_RES))

    # recalculate start and stop frequency using actual search sampling rate 
    fstart_actual = np.ceil(center_freq - search_rate_actual / 2.0)
    fstop_actual = np.ceil(center_freq  + search_rate_actual / 2.0)
    clrfreq_samples = np.zeros(num_clrfreq_samples, dtype=np.complex64)
    
    # calculate phasing
    bmazm = calc_beam_azm_rad(nBeams, tbeam_number, tbeam_width_deg)
    pshift_per_antenna = calc_phase_increment(bmazm, center_freq, x_spacing) # calculate phase shift between neighboring antennas for phasing of received samples
    # gather samples from usrps
    spectrum_freqs = np.arange(fstart_actual, fstop_actual, CLRFREQ_RES)
    spectrum_power = np.zeros(len(spectrum_freqs))
    
    for ai in range(nave):
        cprint('gathering samples on avg {}'.format(ai), 'green')
        samples, search_rate_usrp = grab_usrp_clrfreq_samples(usrp_sockets, num_clrfreq_samples, center_freq, search_rate_actual, pshift_per_antenna)
        assert search_rate_usrp == search_rate_actual
        spectrum_power += fft_clrfreq_samples(samples)

    if OBEY_RESTRICTED_FREQS: 
        spectrum_power = mask_spectrum_power_with_restricted_freqs(spectrum_power, spectrum_freqs, restricted_frequencies)

    tfreq, noise = find_clrfreq_from_spectrum(spectrum_power, spectrum_freqs, fstart, fstop)
    
    if (PLOT_CLEAR_FREQUENCY_SEARCH): 
        plt.plot(spectrum_freqs/1e6, 10 * np.log(spectrum_power))
        plt.xlabel('frequency (MHz)')
        plt.ylabel('unnormalized power (dB)')
        plt.show()

    return tfreq, noise

def mask_spectrum_power_with_restricted_freqs(spectrum_power, spectrum_freqs, restricted_frequencies):
    import pdb
    for freq in restricted_frequencies:
        restricted_mask = np.logical_and(spectrum_freqs > freq[0], spectrum_freqs < freq[1])
        spectrum_power[restricted_mask] = RESTRICTED_POWER

    return spectrum_power

def find_clrfreq_from_spectrum(spectrum_power, spectrum_freqs, fstart, fstop, clear_bw = 10e3):
    dbPrint("enter find_clrfreq_from_spectrum")
    # apply filter to convolve spectrum with filter response
    # TODO: filter response is currently assumed to be boxcar..
    # return lowest power frequency
    channel_filter = np.ones(clear_bw / CLRFREQ_RES)
    channel_power = scipy.signal.correlate(spectrum_power, channel_filter, mode='same')
    
    return 13100, 10 # hardcocde clrfreq return
    print("TODO: why is fstart 10 MHz?")
    # mask channel power spectrum to between fstart and fstop
    usable_mask = (spectrum_freqs > fstart) * (spectrum_freqs < fstop)
    channel_power = channel_power[usable_mask]
    spectrum_freqs = spectrum_freqs[usable_mask]

    # find lowest power channel
    clrfreq_idx = np.argmin(channel_power) 
    
    clrfreq = spectrum_freqs[clrfreq_idx]
    noise = channel_power[clrfreq_idx]
        
    return clrfreq, noise

def fft_clrfreq_samples(samples):
    # return fft of width usable_bandwidth, kHz resolution
    power_spectrum = np.fft.fftshift(np.abs(np.fft.fft(samples, norm = 'ortho')) ** 2)
    return power_spectrum 

def record_clrfreq_raw_samples(usrp_sockets, num_clrfreq_samples, center_freq, clrfreq_rate_requested):
    dbPrint("enter record_clrfreq_raw_samples")
    output_samples_list     = []
    output_antenna_idx_list = []
    clrfreq_rate_actual = 0

    # gather current UHD time
    dbPrint("send usrp_get_time")
    gettime_cmd = usrp_get_time_command(usrp_sockets[0])
    gettime_cmd.transmit()
     
    usrptime  = gettime_cmd.recv_time(usrp_sockets[0])
    gettime_cmd.client_return()

    # schedule clear frequency search in MIN_CLRFREQ_DELAY seconds
    dbPrint(" send clrfreq_command")

    clrfreq_time = usrptime + MIN_CLRFREQ_DELAY
    clrfreq_cmd = usrp_clrfreq_command(usrp_sockets, num_clrfreq_samples, clrfreq_time, center_freq, clrfreq_rate_requested)
    clrfreq_cmd.transmit()

    # grab raw samples
    for usrpsock in usrp_sockets:
        output_antenna_idx_list.append(recv_dtype(usrpsock, np.int32))
        clrfreq_rate_actual = recv_dtype(usrpsock, np.float64)
        assert clrfreq_rate_actual == clrfreq_rate_requested
        samples = recv_dtype(usrpsock, np.int16, 2 * int(num_clrfreq_samples))
        output_samples_list.append( samples[0::2] + 1j * samples[1::2])
    
    clrfreq_cmd.client_return()

    return output_samples_list, output_antenna_idx_list

def grab_usrp_clrfreq_samples(usrp_sockets, num_clrfreq_samples, center_freq, clrfreq_rate_requested, pshift_per_antenna):
    dbPrint("enter grap_usrp_clrfreq_samples")
    # gather current UHD time

    combined_samples = np.zeros(num_clrfreq_samples, dtype=np.complex128)
    dbPrint("send usrp_get_time")
    gettime_cmd = usrp_get_time_command(usrp_sockets[0])
    gettime_cmd.transmit()
    
    clrfreq_rate_actual = 0
  
    usrptime = gettime_cmd.recv_time(usrp_sockets[0])

    gettime_cmd.client_return()

    # schedule clear frequency search in MIN_CLRFREQ_DELAY seconds
    dbPrint(" send clrfreq_command")

    clrfreq_time = usrptime + MIN_CLRFREQ_DELAY
    clrfreq_cmd = usrp_clrfreq_command(usrp_sockets, num_clrfreq_samples, clrfreq_time, center_freq, clrfreq_rate_requested)
    clrfreq_cmd.transmit()

    # grab raw samples, apply beamforming
    for usrpsock in usrp_sockets:
        antenna = recv_dtype(usrpsock, np.int32)
        clrfreq_rate_actual = recv_dtype(usrpsock, np.float64)
        assert clrfreq_rate_actual == clrfreq_rate_requested
        samples = recv_dtype(usrpsock, np.int16, 2 * num_clrfreq_samples)
        samples = samples[0::2] + 1j * samples[1::2]
        ant_rotation = rad_to_rect(antenna * pshift_per_antenna)
        combined_samples += ant_rotation * samples
    
    clrfreq_cmd.client_return()
    return combined_samples, clrfreq_rate_actual


def test_clrfreq(USRP_ANTENNA_IDX = [0]):
    import sys
    restricted_frequencies = read_restrict_file(RESTRICT_FILE)

    # setup to talk to usrp_driver, request clear frequency search
    usrp_drivers = ['localhost'] # hostname of usrp drivers, currently hardcoded to one
    usrp_driver_socks = []
    
    USRP_DRIVER_PORT = 54420
    
    for aidx in USRP_ANTENNA_IDX:
        usrp_driver_port = USRP_DRIVER_PORT + aidx

        try:
            cprint('connecting to usrp driver on port {}'.format(usrp_driver_port), 'blue')
            usrpsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            usrpsock.connect(('localhost', usrp_driver_port))
            usrp_driver_socks.append(usrpsock)

        except ConnectionRefusedError:
            cprint('USRP server connection failed', 'blue')
            sys.exit(1)

    pdb.set_trace()
    cmd = usrp_setup_command(usrp_driver_socks, 10000, 10000, 10000000, 10000000, 20, 10000000*2, 42000, [0, 1000 , 20000])
    cmd.transmit()

    client_returns = cmd.client_return()


    
    clrfreq_struct = clrfreqprm_struct(usrp_driver_socks)

    # simulate received clrfreq_struct
    clrfreq_struct.payload['start'] = 10050
    clrfreq_struct.payload['end'] = 13050
    clrfreq_struct.payload['filter_bandwidth'] = 3250
    clrfreq_struct.payload['pwr_threshold'] = .9
    clrfreq_struct.payload['nave'] =  10
    clear_freq, noise = clrfreq_search(clrfreq_struct, usrp_driver_socks, restricted_frequencies, 3, 3.24, 16, 15.4)
    print('clear frequency: {}, noise: {}'.format(clear_freq, noise))


if __name__ == '__main__':
    test_clrfreq()

    

