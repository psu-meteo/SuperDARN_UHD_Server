#!/usr/bin/python3
# driver to process chunks of samples using CUDA
# uses shared memory to get samples from usrp_drivers 
# spawn one cude driver per computer
# sends processed samples to usrp_server using sockets
# requires python 3 (need mmap with buffer interface to move samples directly between shared memory and gpu)

import socket
import pdb
import numpy as np
import scipy.special
import mmap
import os
import sys
import argparse
import signal
import functools 
import configparser

import posix_ipc
import pycuda.driver as cuda
import pycuda.compiler
import pycuda.autoinit

from termcolor import cprint
from socket_utils import *
from drivermsg_library import *
import dsp_filters
from phasing_utils import *
# import pycuda stuff
SWING0 = 0
SWING1 = 1

SIDEA = 0
SIDEB = 1

RXDIR = 'rx'
TXDIR = 'tx'

DEBUG = True
C = 3e8

BASEBAND_RATE = 1e6 # TODO: don't hardcode this..

rx_shm_list = [[],[]]
tx_shm_list = []
semaphore_list = [[],[]] 
swings = [SWING0, SWING1]
sides = [SIDEA]

# list of all semaphores/shared memory paths for cleaning up
shm_list = []
sem_list = []

swing = SWING0

# TODO: write decorator for acquring/releasing semaphores

# python3 or greater is needed for direct transfers between shm and gpu memory
if sys.hexversion < 0x030300F0:
    print('this code requires python 3.3 or greater')
    sys.exit(0)


class cudamsg_handler(object):
    def __init__(self, serversock, command, gpu, antennas, array_info, hardware_limits):
        self.sock = serversock
        self.antennas = np.uint16(antennas)
        self.command = command
        self.status = 0
        self.gpu = gpu
        self.array_info = array_info
        self.hardware_limits = hardware_limits

    def respond(self):
        transmit_dtype(self.sock, self.command, np.uint8)

    def process(self):
        raise NotImplementedError('The process method for this driver message is unimplemented')

# allocate memory for cuda sample buffers
class cuda_setup_handler(cudamsg_handler):
    def process(self):
        cmd = cuda_setup_command([self.sock])
        cmd.receive(self.sock)

        print('entering cuda_setup_handler (currently blank!)')
        semaphore_list[SIDEA][SWING0].acquire()
        semaphore_list[SIDEA][SWING1].acquire()
        
        # release semaphores
        semaphore_list[SIDEA][SWING0].release()
        semaphore_list[SIDEA][SWING1].release()
    
# take copy and process data from shared memory, send to usrp_server via socks 
class cuda_generate_pulse_handler(cudamsg_handler):
    def process(self):
        print('enter cuda_generate_pulse_handler.process')
        cmd = cuda_generate_pulse_command([self.sock])
        cmd.receive(self.sock)

        print('entering generate_pulse_handler')
        semaphore_list[SIDEA][SWING0].acquire()
        semaphore_list[SIDEA][SWING1].acquire()

        # create empty baseband transmit waveform vector
        npulses = self.gpu.npulses
        nantennas = self.gpu.nAntennas
        nchannels = self.gpu.nChannels
        
        bbtx = [None for c in range(nchannels)] 
        bbrates_tx = [None for c in range(nchannels)] 
        # populate waveform table with channels 
        for channel in self.gpu.sequences:
            if channel != None:
                cnum = channel.ctrlprm['channel']
                bbtx[cnum], bbrates_tx[cnum] = self.generate_bbtx(channel, shapefilter = dsp_filters.gaussian_pulse)
        #pdb.set_trace() 
        if not (len(np.unique(bbrates_tx)) == 1 + (None in bbrates_tx)):
            print("all baseband sampling rates must be the same?..")# TODO: investigate this case :(
            pdb.set_trace()

        # synthesize rf waveform
        self.gpu.synth_channels(bbtx, bbrates_tx)
        
        # copy rf waveform to shared memory from GPU memory 
        self.gpu.txsamples_host_to_shm()

        semaphore_list[SIDEA][SWING0].release()
        semaphore_list[SIDEA][SWING1].release()

    # generate baseband sample vectors for a transmit pulse from sequence information
    def generate_bbtx(self, channel, shapefilter = None):
        print('entering generate_bbtx')
        # tpulse is the pulse time in seconds
        # bbrate is the transmit baseband sample rate 
        # tfreq is transmit freq in hz
        # tbuffer is guard time between tr gate and rf pulse in seconds
        chnum = channel.ctrlprm['channel']
        ctrlprm = channel.ctrlprm

        tpulse = channel.pulse_lens[0] / 1e6 # read in array of pulse lengths in seconds 
        tfreq = ctrlprm['tfreq'] * 1000 # read in tfreq, convert from kHz to Hz
    
        tbuffer = float(self.hardware_limits['min_tr_to_pulse']) / 1e6 # convert microseconds tbuffer in config file to seconds
        bbrate = 50 / tbuffer # so, uh.. 1 microsecond baseband rate? (1 msps..)
        # TODO: resolve conflict between pre-pulse buffer time and baseband sampling rate
        # I'll need to resample transmit pulse by (1/tbuffer) / bbrate?
        # ctrlprm['baseband_samplerate'] is expected baseband sampling rate in samples/second 

        trise = ctrlprm['trise'] / 1e9 # read in rise time in seconds (TODO: fix units..)
        assert tbuffer >= int(self.hardware_limits['min_tr_to_pulse']) / 1e6, 'time between TR gate and RF pulse too short for hardware'
        assert tpulse > int(self.hardware_limits['min_chip']) / 1e6, 'pulse length ({} micro s) is too short for hardware (min is {} micro s)'.format(tpulse*1e6, self.hardware_limits['min_chip'])
        assert tpulse < int(self.hardware_limits['max_tpulse']) / 1e6, 'pulse length is too long for hardware'

        if not (tfreq >= int(self.hardware_limits['minimum_tfreq'])):
            print('transmit frequency too low for hardware')
            #pdb.set_trace()

        if tfreq > int(self.hardware_limits['maximum_tfreq']):
            print('transmit frequency too high for hardware')
            #pdb.set_trace()

        #assert tfreq < int(self.hardware_limits['maximum_tfreq']), 'transmit frequency too high for hardware'

        assert sum(channel.pulse_lens) / (1e6 * ((channel.pulse_offsets_vector[-1] + tpulse))) < float(self.hardware_limits['max_dutycycle']), ' duty cycle of pulse sequence is too high'
        
        npulses = len(channel.pulse_lens)
        nantennas = len(self.antennas)
        
        # tbuffer is the time between tr gate and transmit pulse 
        nsamples = np.round((tpulse + 2 * tbuffer) * bbrate)
        padding = np.zeros(np.round(tbuffer * bbrate))
        pulse = np.ones(np.round(tpulse * bbrate))
        pulsesamps = np.complex64(np.concatenate([padding, pulse, padding]))

        beam_sep = float(self.array_info['beam_sep']) # degrees
        nbeams = int(self.array_info['nbeams'])
        x_spacing = float(self.array_info['x_spacing']) # meters
        beamnum = ctrlprm['tbeam']

        # convert beam number of radian angle
        bmazm = calc_beam_azm_rad(nbeams, beamnum, beam_sep)

        # calculate antenna-to-antenna phase shift for steering at a frequency
        pshift = calc_phase_increment(bmazm, tfreq)

        # calculate a complex number representing the phase shift for each antenna
        beamforming_shift = [rad_to_rect(a * pshift) for a in self.antennas]
        
        # construct baseband tx sample array
        bbtx = np.complex128(np.zeros((nantennas, npulses, len(pulsesamps))))

        for aidx in range(len(self.antennas)):
            for pulse in range(npulses):
                # compute pulse compression 
                # apply phase shifting to pulse using phase_mask

                psamp = pulsesamps
                psamp[len(padding):-len(padding)] *=  np.exp(1j * np.pi * channel.phase_masks[aidx])
                # TODO: support non-1us resolution phase masks
                # apply filtering function
                if shapefilter != None:
                    psamp = shapefilter(psamp, trise, bbrate)
                
                # apply phasing
                psamp *= beamforming_shift[aidx]

                # update baseband pulse sample array for antenna
                bbtx[aidx][pulse] = psamp

        return bbtx, bbrate
         
# add a new channel 
class cuda_add_channel_handler(cudamsg_handler):
    def process(self):
        print('entering cuda_add_channel_handler, waiting for swing semaphores')
        semaphore_list[SIDEA][SWING0].acquire()
        semaphore_list[SIDEA][SWING1].acquire()
        # get picked sequence information dict from usrp_server
        cmd = cuda_add_channel_command([self.sock])
        cmd.receive(self.sock)

        print('received command')
        sequence = cmd.sequence
        ctrlprm = sequence.ctrlprm
        self.gpu.sequences[ctrlprm['channel']] = sequence

        fc = ctrlprm['tfreq'] * 1000
        cnum = ctrlprm['channel']

        self.gpu._set_mixerfreq(cnum)
        self.gpu._set_phasedelay(cnum)
       
        # release semaphores
        semaphore_list[SIDEA][SWING0].release()
        semaphore_list[SIDEA][SWING1].release()
        #pdb.set_trace()
        print('semaphores released in cuda add channel handler')

# prepare for a refresh of sequences 
class cuda_pulse_init_handler(cudamsg_handler):
    def process(self):
        cmd = cuda_pulse_init_command([self.sock])
        cmd.receive(self.sock)
          
        print('entering cuda_pulse_init_handler')

        semaphore_list[SIDEA][SWING0].acquire()
        semaphore_list[SIDEA][SWING1].acquire()

        #self.gpu.sequences = [None for chan in range(self.gpu.nChannels)]

        semaphore_list[SIDEA][SWING0].release()
        semaphore_list[SIDEA][SWING1].release()

# remove a channel from the GPU
class cuda_remove_channel_handler(cudamsg_handler):
    def process(self):
        cmd = cuda_remove_channel_command([self.sock])
        cmd.receive(self.sock)
        # This is not implemented..
        # pdb.set_trace()
        pass



# take copy and process data from shared memory, send to usrp_server via socks 
class cuda_get_data_handler(cudamsg_handler):
    def process(self):
        print('entering cuda_get_data handler')
        cmd = cuda_get_data_command([self.sock])
        cmd.receive(self.sock)

        channel = recv_dtype(self.sock, np.int32) # TODO: use this infomation..
        dbPrint('received channel={}'.format(channel))
        dbPrint('pulling data...') 
        samples = self.gpu.pull_rxdata()
        dbPrint('finished pullind data')
        nants, nChannels, nsamps = samples.shape
        dbPrint('data format: {}'.format(samples.shape))
        dbPrint('transmitting nants {}'.format(nants)) 
        transmit_dtype(self.sock, nants, np.uint32)

        for aidx in range(nants):
            transmit_dtype(self.sock, self.antennas[aidx], np.uint16)
            dbPrint('transmitted antenna index')
            # TODO: calculate size of transmit samples for channel/ant
            transmit_dtype(self.sock, nsamps, np.uint32)
            dbPrint('transmitted number of samples ({})'.format(nsamps))
            self.sock.sendall(samples[aidx][channel-1].tobytes())

        # TODO: remove hardcoded swing/side
        semaphore_list[SIDEA][SWING0].release()

# cleanly exit.
class cuda_exit_handler(cudamsg_handler):
    def process(self):
        cmd = cuda_exit_command([self.sock])
        cmd.receive(self.sock)

        clean_exit() 

# copy data to gpu, start processing
class cuda_process_handler(cudamsg_handler):
    def process(self):
        print('enter cuda_process_handler:process')
        cmd = cuda_process_command([self.sock], SWING0)
        cmd.receive(self.sock)
        # TODO: remove hardcoded swing/side
        semaphore_list[SIDEA][SWING0].acquire()
        self.gpu.rxsamples_shm_to_gpu(rx_shm_list[SIDEA][swing])
        self.gpu.rxsamples_process() 

cudamsg_handlers = {\
        CUDA_SETUP: cuda_setup_handler, \
        CUDA_GET_DATA: cuda_get_data_handler, \
        CUDA_PROCESS: cuda_process_handler, \
        CUDA_ADD_CHANNEL: cuda_add_channel_handler, \
        CUDA_REMOVE_CHANNEL: cuda_remove_channel_handler, \
        CUDA_GENERATE_PULSE: cuda_generate_pulse_handler, \
        CUDA_PULSE_INIT : cuda_pulse_init_handler, \
        CUDA_EXIT: cuda_exit_handler}

cudamsg_handler_names = {\
        CUDA_SETUP: 'CUDA_SETUP', \
        CUDA_GET_DATA: 'CUDA_GET_DATA', \
        CUDA_PROCESS: 'CUDA_PROCESS', \
        CUDA_ADD_CHANNEL: 'CUDA_ADD_CHANNEL', \
        CUDA_REMOVE_CHANNEL: 'CUDA_REMOVE_CHANNEL', \
        CUDA_GENERATE_PULSE: 'CUDA_GENERATE_PULSE', \
        CUDA_PULSE_INIT : 'CUDA_PULSE_INIT', \
        CUDA_EXIT: 'CUDA_EXIT'}


def sem_namer(ant, swing):
    name = 'semaphore_ant_{}_swing_{}'.format(int(ant), int(swing))
    return name

def shm_namer(antenna, swing, side, direction = 'rx'):
    name = 'shm_{}_ant_{}_side_{}_swing_{}'.format(direction, int(antenna), int(side), int(swing))
    return name

def create_shm(antenna, swing, side, shm_size, direction = 'rx'):
    name = shm_namer(antenna, swing, side, direction)
    memory = posix_ipc.SharedMemory(name, posix_ipc.O_CREAT, size=int(shm_size))
    mapfile = mmap.mmap(memory.fd, memory.size)
    memory.close_fd()
    shm_list.append(name)
    return mapfile

def create_sem(ant, swing):
    name = sem_namer(ant, swing)
    sem = posix_ipc.Semaphore(name, posix_ipc.O_CREAT)
    sem.release()
    sem_list.append(sem)
    return sem

def clean_exit():
    for shm in shm_list:
        posix_ipc.unlink_shared_memory(shm)

    for sem in sem_list:
        sem.release()
        sem.unlink()
        
    sys.exit(0)

def sigint_handler(signum, frame):
   clean_exit()

# class to contain references to gpu-side information
# handle launching signal processing kernels
# and host/gpu communication and initialization
# bbtx is now [NANTS, NPULSES, NCHANNELS, NSAMPLES]
class ProcessingGPU(object):
    def __init__(self, antennas, maxchannels, maxpulses, ntapsrx_rfif, ntapsrx_ifbb, rfifrate, ifbbrate, fsamptx, fsamprx):
        self.antennas = np.int16(antennas)
        # maximum supported channels
        self.nChannels = int(maxchannels)
        self.nAntennas = len(antennas)
        self.npulses = int(maxpulses)

        # number of taps for baseband and if filters
        self.ntaps_rfif = int(ntapsrx_rfif)
        self.ntaps_ifbb = int(ntapsrx_ifbb)

        # rf to if downsampling ratio 
        self.rfifrate = int(rfifrate)
        self.ifbbrate = int(ifbbrate)

        # USRP rx/tx sampling rates
        self.fsamptx = int(fsamptx)
        self.fsamprx = int(fsamprx)
         
        # calibration tables for phase and time delay offsets
        self.tdelays = np.zeros(self.nAntennas) # table to account for constant time delay to antenna, e.g cable length difference
        self.phase_offsets = np.zeros(self.nAntennas) # table to account for constant phase offset, e.g 180 degree phase flip

        self.baseband_samples = [[None for i in range(self.nChannels)] for p in maxpulses] # table to store baseband samples of pulse for each frequency
        self.sequences = [None for i in range(self.nChannels)] # table to store sequence infomation

        # host side copy of channel transmit frequency array
        self.mixer_freqs = np.zeros(self.nChannels, dtype=np.float64)

        # host side copy of per-channel, per-antenna array with calibrated cable phase offset
        self.phase_delays = np.zeros((self.nChannels, self.nAntennas), dtype=np.float32)
        # dictionaries to map usrp array indexes and sequence channels to indexes 
        self.channel_to_idx = {}
        
        with open('rx_cuda.cu', 'r') as f:
            self.cu_rx = pycuda.compiler.SourceModule(f.read())
            self.cu_rx_multiply_and_add = self.cu_rx.get_function('multiply_and_add')
            self.cu_rx_multiply_mix_add = self.cu_rx.get_function('multiply_mix_add')

        
        with open('tx_cuda.cu', 'r') as f:
            self.cu_tx = pycuda.compiler.SourceModule(f.read())
            self.cu_tx_interpolate_and_multiply = self.cu_tx.get_function('interpolate_and_multiply')
            self.cu_txfreq_rads = self.cu_tx.get_global('txfreq_rads')[0]
            self.cu_txoffsets_rads = self.cu_tx.get_global('txphasedelay_rads')[0]
                
        self.streams = [cuda.Stream() for i in range(self.nChannels)]

    # add a USRP with some constant calibration time delay and phase offset (should be frequency dependant?)
    # instead, calibrate VNA on one path then measure S2P of other paths, use S2P file as calibration?
    def addUSRP(self, hostname = '', mainarray = True, array_idx = -1, x_position = None, tdelay = 0, side = 'a', phase_offset = None):
        self.tdelays[int(array_idx)] = tdelay
        self.phase_offsets[int(array_idx)] = phase_offset
    
    # generate tx rf samples from sequence
    def synth_tx_rf_pulses(self, bbtx, nbb_samples_per_pulse):
        for (aidx, ant) in enumerate(self.antennas):
            for chan in range(self.nChannels):
                for pulse in range(bbtx[chan].shape[1]):
                    # bbtx[channel][nantennas, npulses, len(pulsesamps)]
                    # create interleaved real/complex bb vector
                    bb_vec_interleaved = np.zeros(nbb_samples_per_pulse * 2)
                    try:
                        bb_vec_interleaved[0::2] = np.real(bbtx[chan][aidx][pulse][:])
                        bb_vec_interleaved[1::2] = np.imag(bbtx[chan][aidx][pulse][:])
                    except:
                        print('error while merging baseband tx vectors..')
                        pdb.set_trace()
                    self.tx_bb_indata[aidx][chan][pulse][:] = bb_vec_interleaved[:]
        
        # upsample baseband samples on GPU, write samples to shared memory
        self.interpolate_and_multiply()
        cuda.Context.synchronize()
    
    def tx_init(self, nbb_samples_per_pulse):
        bbrate = 1e6 # TODO: don't hardcode this this
        
        # calculate the number of rf samples per pulse 
        nrf_samples_per_pulse = int(np.ceil(self.fsamptx * nbb_samples_per_pulse / bbrate)) # number of rf samples for all pulses
        # TODO: add back in 2x factor for I/Q interleaving

        # calculate tx upsample rates
        tx_upsample_rate = int(nrf_samples_per_pulse / nbb_samples_per_pulse)
        npulses = self.npulses
        nrf_samples_total = nrf_samples_per_pulse * npulses

        print('nrf_samps: ' + str(nrf_samples_total))
        print('nants: ' + str(self.nAntennas))
 
        # allocate page-locked memory on host for rf samples to decrease transfer time
        # TODO: benchmark this, see if I should move this to init function..
        self.tx_rf_outdata = cuda.pagelocked_empty((self.nAntennas, 2 * nrf_samples_total), np.int16, mem_flags=cuda.host_alloc_flags.DEVICEMAP)
        self.tx_bb_indata = np.float32(np.zeros([self.nAntennas, self.nChannels, self.npulses, nbb_samples_per_pulse * 2])) # * 2 pulse samples for interleaved i/q

        # TODO: look into memorypool and freeing page locked memory?
        # https://stackoverflow.com/questions/7651450/how-to-create-page-locked-memory-from-a-existing-numpy-array-in-pycuda

        # point GPU to page-locked memory for rf rx and tx samples
        self.cu_tx_rf_outdata = np.intp(self.tx_rf_outdata.base.get_device_pointer())
        self.cu_tx_bb_indata = cuda.mem_alloc_like(self.tx_bb_indata)
       
        # compute grid/block sizes for cuda kernels
        self.tx_block = self._intify((tx_upsample_rate, self.nChannels, 1))
        self.tx_grid = self._intify((nbb_samples_per_pulse, self.nAntennas, self.npulses))

        print('tx block: ' + str(self.tx_block))
        print('tx grid: ' + str(self.tx_grid))
 
        max_blocksize = cuda.Device(0).get_attribute(pycuda._driver.device_attribute.MAX_THREADS_PER_BLOCK)
        assert self._blocksize(self.tx_block) <= max_blocksize, 'tx upsampling block size exceeds CUDA limits, reduce stage upsampling rate, number of pulses, or number of channels'
        #pdb.set_trace()

    def rx_init(self): 
        # build arrays based on first sequence..
        # TODO: support multiple sequences?
        ctrlprm = self.sequences[0].ctrlprm
        # TODO: get form ini file and calculate
        decimationRate_rf2if = self.rfifrate
        decimationRate_if2bb = 75

        bbrate_rx = ctrlprm['baseband_samplerate']
        nSamples_rx_bb = int(ctrlprm['number_of_samples']) # number of recv samples

        nSamples_rx_if = (nSamples_rx_bb-1) * decimationRate_if2bb + self.ntaps_ifbb
        self.nSamples_rx_rf = int((nSamples_rx_if-1) * decimationRate_rf2if + self.ntaps_rfif)

        # calculate rx sample decimation rates
        rx_time = nSamples_rx_bb / bbrate_rx
        

        # [NCHANNELS][NTAPS][I/Q]
        self.rx_filtertap_rfif = np.float32(np.zeros([self.nChannels, self.ntaps_rfif, 2]))
        self.rx_filtertap_ifbb = np.float32(np.zeros([self.nChannels, self.ntaps_ifbb, 2]))
    
        # generate filters
        
        self._kaiser_filter_s0()    
        # self._rolloff_filter_s1()
        self._raisedCosine_filter()
    
        #import matplotlib.pyplot as plt
        #plt.figure()
        #plt.plot(self.rx_filtertap_rfif[0])
        #plt.show()
        #pdb.set_trace()
        
        self.rx_samples_if = np.float32(np.zeros([self.nAntennas, self.nChannels, 2 * nSamples_rx_if]))
        self.rx_samples_bb = np.float32(np.zeros([self.nAntennas, self.nChannels, 2 * nSamples_rx_bb]))

        self.rx_samples_rf = cuda.pagelocked_empty((self.nAntennas, self.nChannels, self.nSamples_rx_rf*2), np.int16, mem_flags=cuda.host_alloc_flags.DEVICEMAP)

        self.cu_rx_samples_rf = np.intp(self.rx_samples_rf.base.get_device_pointer())

        # allocate memory on GPU
        self.cu_rx_filtertaps_rfif = cuda.mem_alloc_like(self.rx_filtertap_rfif)
        self.cu_rx_filtertaps_ifbb = cuda.mem_alloc_like(self.rx_filtertap_ifbb)

        self.cu_rx_samples_if = cuda.mem_alloc_like(self.rx_samples_if)
        self.cu_rx_samples_bb = cuda.mem_alloc_like(self.rx_samples_bb)
       
        cuda.memcpy_htod(self.cu_rx_filtertaps_rfif, self.rx_filtertap_rfif)
        cuda.memcpy_htod(self.cu_rx_filtertaps_ifbb, self.rx_filtertap_ifbb)
 
        self.rx_grid_if = self._intify((nSamples_rx_if, self.nAntennas, 1))
        self.rx_grid_bb = self._intify((nSamples_rx_bb, self.nAntennas, 1))
        self.rx_block_if = self._intify((self.ntaps_rfif / 2, self.nChannels, 1))
        self.rx_block_bb = self._intify((self.ntaps_ifbb / 2, self.nChannels, 1))
        
        # check if up/downsampling cuda kernels block sizes exceed hardware limits 
        max_blocksize = cuda.Device(0).get_attribute(pycuda._driver.device_attribute.MAX_THREADS_PER_BLOCK)

        assert self._blocksize(self.rx_block_if) <= max_blocksize, 'rf to if block size exceeds CUDA limits, reduce downsampling rate, number of pulses, or number of channels'
        assert self._blocksize(self.rx_block_bb) <= max_blocksize, 'if to bb block size exceeds CUDA limits, reduce downsampling rate, number of pulses, or number of channels'

    def synth_channels(self, bbtx, bbrates_tx):
        # prepare sample 
        self.rx_init()
        # TODO: this assumes all channels have the same number of samples 
        # TODO: why am I passing in antennas?
        txbb_samples_per_pulse = int(bbtx[0].shape[2]) # number of baseband samples per pulse
        self.tx_init(txbb_samples_per_pulse)

        self.synth_tx_rf_pulses(bbtx, txbb_samples_per_pulse)
        if False:
        # save plot of transmit pulse for debugging...
            import matplotlib
            #matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            txpulse = self.tx_rf_outdata[0]
            arp  = np.sqrt(np.float32(txpulse[0::2]) ** 2 + np.float32(txpulse[1::2]) ** 2)

            plt.subplot(3,1,1)
            plt.plot(txpulse)
            plt.subplot(3,1,2)
            plt.plot(arp)
            plt.subplot(3,1,3)
            plt.plot(txpulse[0:5000:2])
            plt.plot(txpulse[1:5000:2])
            plt.show()
            print('finished pulse generation, breakpoint..')
            import myPlotTools as mpt
            mpt.plot_freq(txpulse, 8e6, iqInterleaved=True)
            pdb.set_trace()
        
    # calculates the threads in a block from a block size tuple
    def _blocksize(self, block):
        return functools.reduce(np.multiply, block)

    # convert a tuple to ints for pycuda, blocks/grids must be passed as int tuples
    def _intify(self, tup):
        return tuple([int(v) for v in tup])

    # transfer rf samples from shm to memory pagelocked to gpu (TODO: make sure it is float32..)
    def rxsamples_shm_to_gpu(self, shm):
        shm.seek(0)
        for aidx in range(self.nAntennas):
            for ch in range(self.nChannels):
                self.rx_samples_rf[aidx][ch] = np.frombuffer(shm, dtype=np.int16, count = self.nSamples_rx_rf*2)
               
    # kick off async data processing
    def rxsamples_process(self):
        print('processing rf -> if')
        self.cu_rx_multiply_mix_add(self.cu_rx_samples_rf, self.cu_rx_samples_if, self.cu_rx_filtertaps_rfif, block = self.rx_block_if, grid = self.rx_grid_if, stream = self.streams[swing])

        print('processing if -> bb')
        self.cu_rx_multiply_and_add(self.cu_rx_samples_if, self.cu_rx_samples_bb, self.cu_rx_filtertaps_ifbb, block = self.rx_block_bb, grid = self.rx_grid_bb, stream = self.streams[swing])

        # for testing: plot RF, IF and BB
        if False:
            import myPlotTools as mpt
            import matplotlib.pyplot as plt
            #samplingRate_rx_bb =  self.gpu.sequence[0].ctrlprm['baseband_samplerate']
            plt.figure()        
            cuda.memcpy_dtoh(self.rx_samples_if, self.cu_rx_samples_if) 
            cuda.memcpy_dtoh(self.rx_samples_bb, self.cu_rx_samples_bb) 
            ax = plt.subplot(311)
            mpt.plot_freq(self.rx_samples_rf[0][0],  8e6, iqInterleaved=True, show=False)
            ax.set_ylim([-80, 20])
            plt.ylabel('RF')

            ax = plt.subplot(312)
            mpt.plot_freq(self.rx_samples_if[0][0], 3333.3333 * 75, iqInterleaved=True, show=False)
            ax.set_ylim([-50, 50])
            plt.ylabel('IF')

            ax =plt.subplot(313)
            mpt.plot_freq(self.rx_samples_bb[0][0], 3333.3333 , iqInterleaved=True, show=False)
            ax.set_ylim([0, 100])
            plt.ylabel('BB')

            plt.show()
            # pdb.set_trace()

    # pull baseband samples from GPU into host memory
    def pull_rxdata(self):
        self.streams[swing].synchronize()
        cuda.memcpy_dtoh(self.rx_samples_bb, self.cu_rx_samples_bb)
        return self.rx_samples_bb
    
    # upsample baseband data on gpu
    def interpolate_and_multiply(self):
        cuda.memcpy_htod(self.cu_tx_bb_indata, self.tx_bb_indata)
        self.cu_tx_interpolate_and_multiply(self.cu_tx_bb_indata, self.cu_tx_rf_outdata, block = self.tx_block, grid = self.tx_grid)
    
    # copy rf samples to shared memory for transmission by usrp driver
    def txsamples_host_to_shm(self):
        # TODO: assumes single polarization
        for aidx in range(self.nAntennas):
            tx_shm_list[aidx].seek(0)
            tx_shm_list[aidx].write(self.tx_rf_outdata[aidx].tobytes())
            tx_shm_list[aidx].flush()
    
    # update host-side mixer frequency table with current channel sequence, then refresh array on GPU
    def _set_mixerfreq(self, channel):
        # TODO: determine fc from channel
        fc = self.sequences[channel].ctrlprm['tfreq'] * 1000
        self.mixer_freqs[channel] = np.float64(2 * np.pi * (self.fsamptx - fc) / self.fsamptx)
        cuda.memcpy_htod(self.cu_txfreq_rads, self.mixer_freqs)
    

    # update host-side phase delay table with current channel sequence, then refresh array on GPU
    def _set_phasedelay(self, channel):
        fc = self.sequences[channel].ctrlprm['tfreq'] * 1000
        
        for ant in range(self.nAntennas):
            self.phase_delays[channel][ant] = np.float32(np.mod(2 * np.pi * 1e-9 * self.tdelays[ant] * fc, 2 * np.pi)) 

        cuda.memcpy_htod(self.cu_txoffsets_rads, self.phase_delays)
 
    def _rect_filter_s0(self):
        self.rx_filtertap_rfif[:,:,:] = 0
        self.rx_filtertap_rfif[:,:,0] = 1
    
    # filter also includes down mixing with LO       
    def _kaiser_filter_s0(self):
        gain = 3.5
        beta = 5

        self.rx_filtertap_rfif[:,:,:] = 0
        m = self.ntaps_rfif - 1 
        b = scipy.special.i0(beta)
        freq_LO = self.sequences[0].ctrlprm['tfreq'] * 1000
        print('freq_LO: {} Hz', freq_LO)
        for iTap in range(self.ntaps_rfif):
            phi = 2*np.pi*freq_LO*iTap/self.fsamprx # phase of LO frequency
            k = scipy.special.i0((2 * beta / m) * np.sqrt(iTap * (m - iTap)))
            self.rx_filtertap_rfif[:,iTap,0] = gain * (k / b) * np.cos(phi)
            self.rx_filtertap_rfif[:,iTap,1] = 0*np.sin(phi)
        ##  pdb.set_trace()

    def _rolloff_filter_s1(self):
        self.rx_filtertap_ifbb[:,:,:] = 0
        for i in range(self.ntaps_ifbb):
            x = 8 * (2 * np.pi * (float(i) / self.ntaps_ifbb) - np.pi)
            self.rx_filtertap_ifbb[:,i,0] = 0.1 * (0.54 - 0.46 * np.cos((2 * np.pi * (float(i) + 0.5)) / self.ntaps_ifbb)) * np.sin(x) / x
        
        self.rx_filtertap_ifbb[:,self.ntaps_ifbb/2,0] = 0.1 * 1. # handle the divide-by-zero condition

    def _raisedCosine_filter(self):
        alpha = 0.22
        self.rx_filtertap_ifbb[:,:,:] = 0
        nTaps = self.ntaps_ifbb-1
        for iTap in range(nTaps+1):
            t = 2*iTap - nTaps
            if t == 0:
               self.rx_filtertap_ifbb[:,iTap,0] = 1
            elif np.absolute(t) ==  nTaps/(2*alpha): 
               self.rx_filtertap_ifbb[:,iTap,0] = np.sin(np.pi/(2*alpha)) / (np.pi/(2*alpha)) * np.pi/4
            else: 
               self.rx_filtertap_ifbb[:,iTap,0] = np.sin(np.pi*t/nTaps) / (np.pi*t/nTaps) * np.cos(alpha*np.pi*t / nTaps) / (1-2*(alpha*t/nTaps)**2) 

    
    def _matched_filter_s1(self, dmrate1):
        self.rx_filtertap_ifbb[:,:,:] = 0.

        for i in range(ntaps1/2-dmrate1/4, self.ntaps_ifbb/2+dmrate1/4):
            self.rx_filtertap_ifbb[:,i,0] = 4./dmrate1

# returns a list of antennas indexes in the usrp_config.ini file
def parse_usrpconfig_antennas(usrpconfig):
    main_antenna_list = []
    back_antenna_list = []
    for usrp in usrpconfig.sections():
        if eval(usrpconfig[usrp]['mainarray']) == True: # probably a bad idea
            main_antenna_list.append(int(usrpconfig[usrp]['array_idx']))
        else:
            back_antenna_list.append(int(usrpconfig[usrp]['array_idx']))
    return main_antenna_list, back_antenna_list

def dbPrint(msg):
    print(' {}: {} '.format(__file__, msg) )


def main():
    # parse usrp config file, read in antennas list
    usrpconfig = configparser.ConfigParser()
    usrpconfig.read('usrp_config.ini')

    main_antennas, back_antennas = parse_usrpconfig_antennas(usrpconfig)
    antennas = main_antennas + back_antennas

    # parse gpu config file
    cudadriverconfig = configparser.ConfigParser()
    cudadriverconfig.read('driver_config.ini')
    shm_settings = cudadriverconfig['shm_settings']
    cuda_settings = cudadriverconfig['cuda_settings']
    network_settings = cudadriverconfig['network_settings']
   
    # parse array config file
    arrayconfig = configparser.ConfigParser()
    arrayconfig.read('array_config.ini')
    array_info = arrayconfig['array_info']
    hardware_limits = arrayconfig['hardware_limits']

    # initalize cuda stuff
    gpu = ProcessingGPU(antennas, **cuda_settings)
    for usrp in usrpconfig.sections():
        gpu.addUSRP(**dict(usrpconfig[usrp]))
    
    # create command socket server to communicate with usrp_server.py
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    cmd_sock.bind((network_settings.get('ServerHost'), network_settings.getint('CUDADriverPort')))
   
    # get size of shared memory buffer per-antenna in bytes from cudadriver_config.ini
    rxshm_size = shm_settings.getint('rxshm_size') 
    txshm_size = shm_settings.getint('txshm_size')
    
    # create shared memory buffers and semaphores for rx and tx
    for ant in antennas:
        #pdb.set_trace()
        rx_shm_list[SIDEA].append(create_shm(ant, SWING0, SIDEA, rxshm_size, direction = RXDIR))
        rx_shm_list[SIDEA].append(create_shm(ant, SWING1, SIDEA, rxshm_size, direction = RXDIR))
        tx_shm_list.append(create_shm(ant, SWING0, SIDEA, txshm_size, direction = TXDIR))
        tx_shm_list.append(create_shm(ant, SWING1, SIDEA, txshm_size, direction = TXDIR))

        semaphore_list[SIDEA].append(create_sem(ant, SWING0))
        semaphore_list[SIDEA].append(create_sem(ant, SWING1))
    
    # create sigin_handler for graceful-ish cleanup on exit
    signal.signal(signal.SIGINT, sigint_handler)

    if(DEBUG):
        print('cuda_driver waiting for socket connection')

    # TODO: make this more.. robust, add error recovery..
    cmd_sock.listen(1)
    server_conn, addr = cmd_sock.accept()
 
    if(DEBUG):
        print('cuda_driver waiting for command')
   
    # wait for commands from usrp_server,  
    while(True):
        cmd = recv_dtype(server_conn, np.uint8)
        cmdname = cudamsg_handler_names[cmd]
        cprint('received {} command'.format(cmdname), 'green')

        handler = cudamsg_handlers[cmd](server_conn, cmd, gpu, antennas, array_info, hardware_limits)
        handler.process()

        cprint('finished processing {} command, responding'.format(cmdname), 'green')
        handler.respond()

        cprint('responded to {},  waiting for next command'.format(cmdname), 'green')
    

if __name__ == '__main__':
    main()
