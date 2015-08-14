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

from socket_utils import *
from drivermsg_library import *

# import pycuda stuff
SWING0 = 0
SWING1 = 1

SIDEA = 0
SIDEB = 1

RXDIR = 'rx'
TXDIR = 'tx'

DEBUG = True
C = 3e8

rx_shm_list = [[],[]]
tx_shm_list = []
rx_semaphore_list = [[],[]] # [side][swing]
tx_semaphore_list = [] 
swings = [SWING0, SWING1]
sides = [SIDEA]

# list of all semaphores/shared memory paths for cleaning up
shm_list = []
sem_list = []

swing = SWING0

# python3 or greater is needed for direct transfers between shm and gpu memory
if sys.hexversion < 0x030300F0:
    print('this code requires python 3.3 or greater')
    sys.exit(0)


# returns a complex number from a phase in radians
def rad_to_rect(rad):
    return np.exp(1j * rad)

class cudamsg_handler(object):
    def __init__(self, serversock, gpu, antennas, array_info, hardware_limits):
        self.sock = serversock
        self.antennas = np.uint16(antennas)
        self.status = 0
        self.gpu = gpu
        self.array_info = array_info
        self.hardware_limits = hardware_limits

    def process(self):
        raise NotImplementedError('The process method for this driver message is unimplemented')

# get infomation about a transmit pulse sequence channel
# assemble tx pulse samples and setup shared memory for rx
# expect information about all pulse sequences..
class cuda_setup_handler(cudamsg_handler):
    def process(self):
        # acquire rx/tx semaphores
        # (don't drag the samples rug out from under an ongoing transmission)
        rx_semaphore_list[SIDEA][SWING0].acquire()
        rx_semaphore_list[SIDEA][SWING1].acquire()
        tx_semaphore_list[SIDEA].acquire()
        
        # get picked sequence information dict from usrp_server
        cmd = cuda_setup_command([self.sock])
        cmd.receive(self.sock)

        self.sequence = cmd.sequence
    
        # extract some information from the picked dict, generate baseband samples
        self.beam = self.sequence.ctrlprm['tbeam']
        self.fc = self.sequence.ctrlprm['tfreq'] * 1e3
        self.generate_bbtx()

        self.gpu.sequences_setup(self.sequence, self.fc, self.bb_vec)

        # copy upconverted rf samples to shared memory from GPU memory to shared memory 
        self.gpu.txsamples_host_to_shm()
                
        # release semaphores
        rx_semaphore_list[SIDEA][SWING0].release()
        rx_semaphore_list[SIDEA][SWING1].release()
        tx_semaphore_list[SIDEA].release()
 
    def generate_bbtx(self, shapefilter = None):
        # tpulse is the pulse time in seconds
        # bbrate is the transmit baseband sample rate 
        # tfreq is transmit freq in hz
        # tbuffer is guard time between tr gate and rf pulse in seconds

        bbrate = self.sequence.ctrlprm['baseband_samplerate'] 
        trise = self.sequence.ctrlprm['trise']
        tpulse = self.sequence.pulse_lens[0]
        tfreq = self.sequence.ctrlprm['tfreq'] * 1000
        tbuffer = int(self.hardware_limits['min_tr_to_pulse'])

        assert tbuffer >= int(self.hardware_limits['min_tr_to_pulse']) / 1e6, 'time between TR gate and RF pulse too short for hardware'
        assert tpulse > int(self.hardware_limits['min_chip']) / 1e6, 'pulse length is too short for hardware'
        assert tpulse < int(self.hardware_limits['max_tpulse']) / 1e6, 'pulse length is too long for hardware'
        assert tfreq >= int(self.hardware_limits['minimum_tfreq']), 'transmit frequency too low for hardware'
        assert tfreq <= int(self.hardware_limits['maximum_tfreq']), 'transmit frequency too high for hardware'
        assert sum(self.sequence.pulse_lens) / (self.sequence.pulse_offsets_vector[-1] + tpulse) < float(self.hardware_limits['max_dutycycle']), ' duty cycle of pulse sequence is too high'
        
        npulses = len(self.sequence.pulse_lens)
        nantennas = len(self.antennas)
        
        # tbuffer is the time between tr gate and transmit pulse 
        nsamples = (tpulse + 2 * tbuffer) * bbrate
        padding = np.zeros(tbuffer * bbrate)
        pulse = np.ones(tpulse * bbrate)
        pulsesamps = np.complex64(np.concatenate([padding, pulse, padding]))

        beam_sep = float(self.array_info['beam_sep']) # degrees
        nbeams = int(self.array_info['nbeams'])
        x_spacing = float(self.array_info['x_spacing']) # meters
        beamnum = self.sequence.ctrlprm['tbeam']

        # calculate beamforming shift..
        center_beam = (nbeams - 1.0) / 2.

        # calculate beam azimuth, in radians
        bmazm = np.deg2rad(90 + (beamnum - center_beam) * beam_sep)

        # translate to phase increment
        wavelength = C / tfreq
        pshift = (2 * np.pi * x_spacing * np.sin(bmazm)) / wavelength
        beamforming_shift = [a * pshift for a in self.antennas]
        
        # construct baseband tx sample array
        bbtx = np.complex128(np.zeros((nantennas, npulses, len(pulsesamps))))
        
        for ant in self.antennas:
            for pulse in range(npulses):
                # compute pulse compression 
                psamp = pulsesamps * self.sequence.phase_masks[ant][pulse]
                 
                # apply filtering function
                if shapefilter != None:
                    psamp = shapefilter(psamp)
                
                # apply phasing
                beamforming_phase = rad_to_rect(beamforming_shift[ant])
                psamp *= beamforming_phase 

                # update baseband pulse sample array for antenna
                bbtx[ant][pulse] = psamp
        
        self.bb_vec = bbtx 


# take copy and process data from shared memory, send to usrp_server via socks 
class cuda_get_data_handler(cudamsg_handler):
    def process(self):
        cmd = cuda_get_data_command([self.sock])
        cmd.receive(self.sock)

        samples = self.gpu.pull_rxdata()
        self.sock.sendall(samples.tobytes())
        semaphore.release()

# cleanly exit.
class cuda_exit_handler(cudamsg_handler):
    def process(self):
       clean_exit() 

# copy data to gpu, start processing
class cuda_process_handler(cudamsg_handler):
    def process(self):
        cmd = cuda_process_command()
        cmd.receive(serversock)

        rx_semaphore_list
        semaphore.acquire()
        
        self.gpu.rxsamples_shm_to_gpu(rx_shm_list[SIDEA][swing])
        self.gpu.rxsamples_process() 

cudamsg_handlers = {\
        CUDA_SETUP: cuda_setup_handler, \
        CUDA_GET_DATA: cuda_get_data_handler, \
        CUDA_PROCESS: cuda_process_handler, \
        CUDA_EXIT: cuda_exit_handler}


def sem_namer(swing, side, direction = 'rx'):
    name = 'semaphore_{}_side_{}_swing_{}'.format(direction, int(side), int(swing))
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

def create_sem(swing, side, direction):
    name = sem_namer(swing, side, direction)
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
    def __init__(self, maxfreqs = 8, maxants = 16, maxstreams = 2, maxchannels = 1, maxpulses = 16, ntapsrx0 = 50, ntapsrx1 = 200, rfifrate = 32, fsamptx = 10000000, fsamprx = 10000000):
        self.max_streams = int(maxstreams)
        self.max_freqs = int(maxfreqs)
        self.nchans = int(maxchannels)
        self.ntaps0 = int(ntapsrx0)
        self.ntaps1 = int(ntapsrx1)
        self.nants = int(maxants)
        self.npulses = int(maxpulses)
        self.rfifrate = int(rfifrate)
        self.fsamptx = int(fsamptx)
        self.fsamprx = int(fsamprx)
        self.tdelays = np.zeros(self.nants) # table to account for constant time delay to antenna, e.g cable length difference
        self.phase_offsets = np.zeros(self.nants) # table to account for constant phase offset, e.g 180 degree phase flip

        # host side copy of channel transmit frequency array
        self.mixer_freqs = np.float64(np.zeros(self.max_freqs))
        # host side copy of per-channel, per-antenna array with calibrated cable phase offset
        self.phase_delays = np.float32(np.zeros((self.max_freqs, self.nants)))
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
                
        self.streams = [cuda.Stream() for i in range(self.max_streams)]
    
    # add a USRP with some constant calibration time delay and phase offset (should be frequency dependant?)
    # instead, calibrate VNA on one path then measure S2P of other paths, use S2P file as calibration?
    def addUSRP(self, hostname = '', mainarray = True, array_idx = -1, x_position = None, tdelay = 0, side = 'a', phase_offset = None):
        self.tdelays[array_idx] = tdelay
        self.phase_offsets[array_idx] = phase_offset
    
    # TODO: Merge sequence generation code to GPU side,
    # sort out how to get samples to gpu class, then onto gpu.. 
    def sequences_setup(self, sequence, fc, bbtx):
        # TODO: set these.. look at sequences
        channel = sequence.ctrlprm['channel']
        bbrate = sequence.ctrlprm['baseband_samplerate']

        if not channel in self.channel_to_idx:
            self.channel_to_idx[channel] = len(self.channel_to_idx)
            # TODO: how are channels remove from mapping?
    
        cidx = self.channel_to_idx[channel] 

        # calculate rx sample decimation rates
        nbbsamps_rx = sequence.ctrlprm['number_of_samples'] # number of recv samples
        rx_time = nbbsamps_rx / bbrate
        nrfsamps_rx = rx_time * self.fsamprx
        # compute decimation rate for RF to IF, this is fixed..
        # so, current worst case is 16 antennas with 2 channels, 10 MSPS sampling rate
        # block size of 1024
        # so, (1024 / 16 / 2) = 32
        # unrolled pre-reduction of 2 in rx kernels
        # so, decimation rate of 64
        nifsamps_rx = nrfsamps_rx / self.rfifrate 

        # calculate tx upsample rates
        # bb based on tx sample stepping? (5e-6)
        nbbsamps_tx = sum(sequence.pulse_lens) / (sequence.txbbrate)
        nrfsamps_tx = self.fsamptx * sum(sequence.pulse_lens)
        nifsamps_tx = nrfsamps_tx / self.rfifrate
        
        # TODO: this is a workaround for testing.. replace with actual antenna indexes from config files?
        self.antennas = np.arange(self.nants, dtype=np.uint16)

        self.dmrate_rftoif = nrfsamps_rx / nifsamps_rx

        # allocate memory on cpu, compile functions
        # [NCHANNELS][NTAPS][I/Q]
        self.rx_filtertap_s0 = np.float32(np.zeros([self.nchans, self.ntaps0, 2]))
        self.rx_filtertap_s1 = np.float32(np.zeros([self.nchans, self.ntaps1, 2]))
    
        self.rx_samples_if = np.float32(np.zeros([self.nants, self.nchans, nifsamps_rx]))
        self.rx_samples_bb = np.float32(np.zeros([self.nants, self.nchans, nbbsamps_rx]))
    
        self.tx_bb_indata = np.float32(np.zeros([self.nants, self.nchans, self.npulses, nbbsamps_tx]))

        # allocate page-locked memory on host for rf samples to decrease transfer time
        self.tx_rf_outdata = cuda.pagelocked_empty((self.nants, nrfsamps_tx), np.uint16, mem_flags=cuda.host_alloc_flags.DEVICEMAP)
        self.rx_samples_rf = cuda.pagelocked_empty((self.nants, self.nchans, nrfsamps_rx), np.float32, mem_flags=cuda.host_alloc_flags.DEVICEMAP)
    
        # point GPU to page-locked memory for rf rx and tx samples
        self.cu_rx_samples_rf = np.intp(self.rx_samples_rf.base.get_device_pointer())
        self.cu_tx_rf_outdata = np.intp(self.tx_rf_outdata.base.get_device_pointer())

        # allocate memory on GPU
        self.cu_rx_filtertaps0 = cuda.mem_alloc_like(self.rx_filtertap_s0)
        self.cu_rx_filtertaps1 = cuda.mem_alloc_like(self.rx_filtertap_s1)

        self.cu_rx_samples_if = cuda.mem_alloc_like(self.rx_samples_if)
        self.cu_rx_samples_bb = cuda.mem_alloc_like(self.rx_samples_bb)
    
        self.cu_tx_bb_indata = cuda.mem_alloc_like(self.tx_bb_indata)
        
        self.tx_block = self._intify(((nrfsamps_tx / nbbsamps_tx), self.nchans, 1))
        self.tx_grid = self._intify((nbbsamps_tx, self.nants, self.npulses))

        # check that downsample rate doesn't create a block size that exceeds hardware limits 
        # with 4 channels and 8 pulses there is a maximum downsample rate of about 32..
        blocksize = functools.reduce(np.multiply, self.tx_block)
        assert blocksize <= cuda.Device(0).get_attribute(pycuda._driver.device_attribute.MAX_THREADS_PER_BLOCK), 'block size exceeds CUDA limits, reduce downsampling rate, number of pulses, or number of channels'

        self.rx_grid_if = self._intify((nifsamps_rx, self.nants, 1))
        self.rx_grid_bb = self._intify((nbbsamps_rx, self.nants, 1))
        self.rx_block_if = self._intify((self.ntaps0 / 2, self.nchans, 1))
        self.rx_block_bb = self._intify((1,nbbsamps_rx))

        # upsample baseband samples on GPU, write samples to shared memory
        samples = self.interpolate_and_multiply(fc, sequence.ctrlprm['channel'])

   
    # convert a tuple to ints for pycuda, blocks/grids must be passed as int tuples
    def _intify(self, tup):
        return tuple([int(v) for v in tup])

    # transfer samples from shared memory to gpu
    def rxsamples_shm_to_gpu(self, shm):
        shm.seek(0)
        cuda.memcpy_dtoh_async(self.cu_rx_samples_rf, shm, stream = self.streams[swing])
        shm.flush()
               
    # kick off async data processing
    def rxsamples_process(self):
        self.cu_rx_multiply_and_add(self.cu_rx_samples_rf, self.cu_rx_samples_if, self.rx_filtertap_s0, block = self.rx_block_0, grid = self.rx_grid_if, stream = self.streams[swing])
        self.cu_rx_multiply_mix_add(self.cu_rx_samples_if, self.cu_rx_samples_bb, self.rx_filtertap_s1, block = self.rx_block_1, grid = self.rx_grid_bb, stream = self.streams[swing])
    
    # pull baseband samples from GPU into host memory
    def pull_rxdata(self):
        self.streams[swing].synchronize()
        cuda.memcpy_dtoh(self.cu_rx_samples_bb, self.rx_samples_bb)
        return self.rx_samples_bb
  
    # upsample baseband data on gpu
    def interpolate_and_multiply(self, fc, channel = 0):
        self._set_mixerfreq(fc, channel)
        self._set_phasedelay(fc, channel)
        cuda.memcpy_htod(self.cu_tx_bb_indata, self.tx_bb_indata)
        self.cu_tx_interpolate_and_multiply(self.cu_tx_bb_indata, self.cu_tx_rf_outdata, block = self.tx_block, grid = self.tx_grid)
        # TODO: add stream.synchronize() before copying samples
    
    # copy rf samples to shared memory for transmission by usrp driver
    def txsamples_host_to_shm(self):
        # TODO: assumes single polarization
        for ant in self.antennas:
            tx_shm_list[ant].seek(0)
            tx_shm_list[ant].write(self.tx_rf_outdata[ant].tobytes())
            tx_shm_list[ant].flush()
    
    # update host-side mixer frequency table with current channel sequence, then refresh array on GPU
    def _set_mixerfreq(self, fc, channel):
        self.mixer_freqs[channel] = np.float64(2 * np.pi * (self.fsamptx - fc) / self.fsamptx)
        cuda.memcpy_htod(self.cu_txfreq_rads, self.mixer_freqs)
    

    # update host-side phase delay table with current channel sequence, then refresh array on GPU
    def _set_phasedelay(self, fc, channel):
        for ant in range(self.nants):
            self.phase_delays[channel][ant] = np.float32(np.mod(2 * np.pi * 1e-9 * self.tdelays[ant] * fc, 2 * np.pi)) 

        cuda.memcpy_htod(self.cu_txoffsets_rads, self.phase_delays)
 
    def _rect_filter_s0():
        self.rx_filtertaps0[:,:,:] = 0
        self.rx_filtertaps0[:,:,0] = 1
           
    def _kaiser_filter_s0(ntaps0):
        gain = 3.5
        beta = 5

        self.rx_filtertaps0[:,:,:] = 0
        m = ntaps0 - 1 
        b = i0(beta)
        for i in range(ntaps0):
            k = scipy.special.i0((2 * beta / m) * sqrt(i * (m - i)))
            self.rx_filtertaps0[:,i,0] = gain * (k / b)
            self.rx_filtertaps0[:,i,1] = 0

    def _rolloff_filter_s1(dmrate1):
        self.rx_filtertaps1[:,:,:] = 0
        for i in range(ntaps1):
            x = 8 * (2 * np.pi * (float(i) / ntaps1) - np.pi)
            self.rx_filtertaps1[:,i,0] = 0.1 * (0.54 - 0.46 * np.cos((2 * np.pi * (float(i) + 0.5)) / ntaps1)) * np.sin(x) / x
        
        self.rx_filtertaps1[:,ntaps1/2,0] = 0.1 * 1. # handle the divide-by-zero condition
        
    def _matched_filter_s1(dmrate1):
        self.rxfilter_taps1[:,:,:] = 0.

        for i in range(ntaps1/2-dmrate1/4, ntaps1/2+dmrate1/4):
            self.rx_filter_taps1[:,i,0] = 4./dmrate1

def main():
    # parse usrp config file, read in antennas list
    usrpconfig = configparser.ConfigParser()
    usrpconfig.read('usrp_config.ini')
    antennas = [int(usrpconfig[usrp]['array_idx']) for usrp in usrpconfig.sections()]  # TODO: fix for back array..
    
    # parse gpu config file
    cudadriverconfig = configparser.ConfigParser()
    cudadriverconfig.read('cudadriver_config.ini')
    shm_settings = cudadriverconfig['shm_settings']
    cuda_settings = cudadriverconfig['cuda_settings']
    network_settings = cudadriverconfig['network_settings']
   
    # parse array config file
    arrayconfig = configparser.ConfigParser()
    arrayconfig.read('array_config.ini')
    array_info = arrayconfig['array_info']
    hardware_limits = arrayconfig['hardware_limits']

    # initalize cuda stuff
    gpu = ProcessingGPU(**dict(cuda_settings))
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
        rx_shm_list[SIDEA].append(create_shm(ant, SWING0, SIDEA, rxshm_size, direction = RXDIR))
        rx_shm_list[SIDEA].append(create_shm(ant, SWING1, SIDEA, rxshm_size, direction = RXDIR))
        tx_shm_list.append(create_shm(ant, SWING0, SIDEA, txshm_size, direction = TXDIR))

    rx_semaphore_list[SIDEA].append(create_sem(SWING0, SIDEA, direction = RXDIR))
    rx_semaphore_list[SIDEA].append(create_sem(SWING1, SIDEA, direction = RXDIR))
    tx_semaphore_list.append(create_sem(SWING0, SIDEA, direction = TXDIR))
    
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
        handler = cudamsg_handlers[cmd](server_conn, gpu, antennas, array_info, hardware_limits)
        handler.process()
    

if __name__ == '__main__':
    main()
