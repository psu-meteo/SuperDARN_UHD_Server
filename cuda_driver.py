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


class cudamsg_handler(object):
    def __init__(self, serversock, gpu):
        self.sock = serversock
        self.status = 0
        self.gpu = gpu

    def process(self):
        raise NotImplementedError('The process method for this driver message is unimplemented')

# get infomation about a transmit pulse sequence channel
# assemble tx pulse samples and setup shared memory for rx
# expect information about all pulse sequences..
class cuda_setup_handler(cudamsg_handler):
    def process(self):
        rx_semaphore_list[SIDEA][SWING0].acquire()
        rx_semaphore_list[SIDEA][SWING1].acquire()
        tx_semaphore_list[SIDEA].acquire()
        
        cmd = cuda_setup_command([self.sock])
        cmd.receive(self.sock)

        sequence = cmd.sequence

        beam = sequence.ctrlprm['beam']
        fc = sequence.ctrlprm['txfreq'] * 1e3
        bbrate = sequence.ctrlprm['baseband_samplerate'] 
        trise = sequence.ctrlprm['trise']

        pdb.set_trace()

        # TODO: pass in tpulse

        self.generate_bbtx(trise)
        self.gpu.tx_bb_indata = self.bb_vec # TODO: check data type..
        samples = self.gpu.interpolate_and_multiply(fc, nchannels)

        tx_shm_list[SIDEA].seek(0)
        tx_shm_list[SIDEA].write(samples.tobytes())
        tx_shm_list[SIDEA].flush()

        rx_semaphore_list[SWING0].release()
        rx_semaphore_list[SWING1].release()
        tx_semaphore_list[SIDEA].release()
 
    def generate_bbtx(self, bbrate, npulses, trise, nbaud = 1, shapefilter = None):
        # calculate number of baseband samples per pulse
        # tpulse is the pulse time in seconds
        # tguard is the time between the T/R gate going high and the start of a rectangular pulse in seconds
        # txrate is the transmit baseband rate 
        nsamples = (tpulse + 2 * tbuffer) * txrate
        padding = np.zeros(tbuffer * txrate)
        pulse = np.ones(tpulse * txrate)
        pulsesamps = np.concatenate([padding, pulse, padding])

        # construct baseband tx sample array
        bbtx = np.zeros((NANTENNAS, NPULSES, NSAMPLES))
        
        
        for ant in NANTENNAS:
            for pulse in NPULSES:
                pdb.set_trace()
                # compute pulse, apply phase shift
                psamp = pulsesamps * phase_mask[ant][pulse]
                
                # apply filtering function
                if shapefilter != None:
                    psamp = shapefilter(psamp)
                
                # apply phasing here?
                # psamp = psamp * self.tx_time_delay_main * something
                # store..
                bbtx[ant][pulse] = psamp
        
        self.bb_vec = bb_vec

        '''
        filter code from c.. break out into functions or use numpy/scipy functions
    std::vector<float> taps((size_t)(25e3/trise), trise/25.e3/2);
    std::vector<std::complex<float> > rawsignal(seq_buf[old_index].size());

    std::complex<float> temp;
    size_t signal_len = rawsignal.size();
    size_t taps_len = taps.size();

    /*Calculate taps for Gaussian filter. This is reference code for future u
    //alpha = 32*(9.86/(2e-8*client.trise)) / (0.8328*usrp->get_rx_rate());
    ////std::cout << "alpha: " << alpha << std::endl;
    ////for (i=0; i<filter_table_len; i++){
    ////  filter_table[i] = pow(alpha/3.14,0.5)*pow(M_E, 
    ////      -1*(alpha)*pow((((float)i-(filter_table_len-1)/2)/filter_table_

    for (size_t i=0; i<taps_len/2; i++){
            temp = std::complex<float>(0,0);
            for(size_t j=0; j<taps_len/2+i; j++){
                    temp += rawsignal[i+j] * taps[taps_len/2-i+j];
            }
            //if (i %5 == 0 ) std::cout << i << " " << temp << std::endl;
            bb_vec[i] = temp;
    }

    for (size_t i=taps_len/2; i<signal_len-taps_len; i++){
            temp = std::complex<float>(0,0);
            for(size_t j=0; j<taps_len; j++){
                    temp += rawsignal[i+j] * taps[j];
            }
            bb_vec[i] = temp;
            //std::cout << i << " " << temp << std::endl;
    }

    for (size_t i=signal_len-taps_len; i<signal_len/2; i++){
            temp = std::complex<float>(0,0);
            for(size_t j=0; j<signal_len-i; j++){
                    temp += rawsignal[i+j] * taps[j];
            }
            bb_vec[i] = temp;
            //if (i 5 == 0 ) std::cout << i << " " << temp << std::endl;
    }

        '''



# take copy and process data from shared memory, send to usrp_server via socks 
class cuda_get_data_handler(cudamsg_handler):
    def process(self):
        cmd = cuda_get_data_command()
        cmd.receive(serversock)
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


def sem_namer(antenna, swing, side, direction = 'rx'):
    name = 'semaphore_{}_ant_{}_side_{}_swing_{}'.format(direction, int(antenna), int(side), int(swing))
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

def create_sem(antenna, swing, side, direction):
    name = sem_namer(antenna, swing, side, direction)
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
    def __init__(self, maxfreqs = 8, maxants = 16, maxstreams = 2, maxchannels = 4, maxpulses = 16, ntapsrx0 = 50, ntapsrx1 = 200, fsamptx = 10000000):
        self.max_streams = int(maxstreams)
        self.nchans = int(maxchannels)
        self.ntaps0 = int(ntapsrx0)
        self.ntaps1 = int(ntapsrx1)
        self.nants = int(maxants)
        self.npulses = int(maxpulses)
        self.fsamptx = int(fsamptx)
        
        self.tdelays = np.zeros(maxants) # table to account for constant time delay to antenna, e.g cable length difference
        self.phase_offsets = np.zeros(maxants) # table to account for constant phase offset, e.g 180 degree phase flip

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
   
    def addUSRP(self, hostname = '', mainarray = True, array_idx = -1, x_position = None, tdelay = 0, side = 'a', phase_offset = None):
        self.tdelays[array_idx] = tdelay
        self.phase_offsets[array_idx] = phase_offset

    def sequences_setup(self, sequence, usrps):
        # TODO: set these.. look at sequences
        channel = sequence.ctrlprm['channel']

        if not channel in self.channel_to_idx:
            self.channel_to_idx[channel] = len(self.channel_to_idx)
            # TODO: how are channels remove from mapping?
    
        cidx = self.channel_to_idx[channel] 

        NRFSAMPS_RX = 10000
        NIFSAMPS_RX = 1000
        NBBSAMPS_RX = 20
        NRFSAMPS_TX = 10000
        NBBSAMPS_TX = 200

        self.antennas = np.zeros(self.nants)
        
        self.dmrate0 = NRFSAMPS_RX / NIFSAMPS_RX
        self.dmrate1 = NIFSAMPS_RX / NBBSAMPS_RX # TODO: not used?

        # allocate memory on cpu, compile functions
        # [NCHANNELS][NTAPS][I/Q]
        self.rx_filtertap_s0 = np.float32(np.zeros([self.nchans, self.ntaps0, 2]))
        self.rx_filtertap_s1 = np.float32(np.zeros([self.nchans, self.ntaps1, 2]))
    
        self.rx_samples_if = np.float32(np.zeros([self.nants, self.nchans, NIFSAMPS_RX]))
        self.rx_samples_bb = np.float32(np.zeros([self.nants, self.nchans, NBBSAMPS_RX]))
    
        self.tx_bb_indata = np.float32(np.zeros([self.nants, self.nchans, self.npulses, NBBSAMPS_TX]))

        # allocate page-locked memory on host for rf samples to decrease transfer time
        self.tx_rf_outdata = cuda.pagelocked_empty((self.nants, NRFSAMPS_TX), np.uint16, mem_flags=cuda.host_alloc_flags.DEVICEMAP)
        self.rx_samples_rf = cuda.pagelocked_empty((self.nants, self.nchans, NRFSAMPS_RX), np.float32, mem_flags=cuda.host_alloc_flags.DEVICEMAP)
    
        # point GPU to page-locked memory for rf rx and tx samples
        self.cu_rx_samples_rf = np.intp(self.rx_samples_rf.base.get_device_pointer())
        self.cu_tx_rf_outdata = np.intp(self.tx_rf_outdata.base.get_device_pointer())

        # allocate memory on GPU
        self.cu_rx_filtertaps0 = cuda.mem_alloc_like(self.rx_filtertap_s0)
        self.cu_rx_filtertaps1 = cuda.mem_alloc_like(self.rx_filtertap_s1)

        self.cu_rx_samples_if = cuda.mem_alloc_like(self.rx_samples_if)
        self.cu_rx_samples_bb = cuda.mem_alloc_like(self.rx_samples_bb)
    
        self.cu_tx_bb_indata = cuda.mem_alloc_like(self.tx_bb_indata)

        self.tx_block = self._intify(((NRFSAMPS_TX / NBBSAMPS_TX), self.npulses, self.nchans))
        self.tx_grid = self._intify((NBBSAMPS_TX, self.nants, 1))

        self.rx_grid_0 = self._intify((NRFSAMPS_RX / self.dmrate0, self.nants, 1))
        self.rx_grid_1 = self._intify((NBBSAMPS_RX, self.nants, 1))
        self.rx_block_0 = self._intify((self.ntaps0 / 2, self.nchans, 1))
        self.rx_block_1 = self._intify((1,NBBSAMPS_RX))

    def _intify(self, tup):
        return tuple([int(v) for v in tup])

    # transfer samples from shared memory to gpu
    def rxsamples_shm_to_gpu(self, shm):
        shm.seek(0)
        cuda.memcpy_dtoh_async(self.cu_rx_samples_rf, shm, stream = self.streams[swing])
        shm.flush()
               
    # kick off async data processing
    def rxsamples_process(self):
        self.cu_rx_multiply_and_add(self.cu_rx_samples_rf, self.cu_rx_samples_if, self.rx_filtertap_s0, block = self.rx_block_0, grid = self.rx_grid_0, stream = self.streams[swing])
        self.cu_rx_multiply_mix_add(self.cu_rx_samples_if, self.cu_rx_samples_bb, self.rx_filtertap_s1, block = self.rx_block_1, grid = self.rx_grid_1, stream = self.streams[swing])

    def pull_rxdata(self):
        self.streams[swing].synchronize()
        cuda.memcpy_dtoh(self.cu_rx_samples_bb, self.rx_samples_bb)
        return self.rx_samples_bb

    def interpolate_and_multiply(self, fc, nchannels):
        self._set_mixerfreqs(fc, nchannels)
        self._set_phasedelays(fc, nchannels)

        cuda.memcpy_htod(self.cu_tx_bb_indata, self.tx_bb_indata)
        self.cu_tx_interpolate_and_multiply(self.cu_tx_bb_indata, self.cu_tx_rf_outdata, block = self.tx_block, grid = self.tx_grid)
        # TODO: add stream.synchronize() before copying samples

    def txsamples_host_to_shm(self, shm):
        shm.seek(0)
        shm.write(self.tx_rf_outdata)
        shm.flush()

    def _set_mixerfreqs(self, fc, nchannels):
        mixer_freqs = np.float64([2 * np.pi * (self.fsamptx - fc[i]) / self.fsamptx for i in range(nchannels)])
        cuda.memcpy_htod(self.cu_txfreq_rads, mixer_freqs)

    def _set_phasedelays(self, fc, nchannels):
        phase_delays = np.float32(np.mod([2 * np.pi * 1e-9 * self.tdelays[i] * fc[i] for i in range(nchannels)], 2 * np.pi)) 
        cuda.memcpy_htod(self.cu_txoffsets_rads, phase_delays)
 
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
    antennas = [usrpconfig[usrp]['array_idx'] for usrp in usrpconfig.sections()]  # TODO: fix for back array..
    
    # parse gpu config file
    cudadriverconfig = configparser.ConfigParser()
    cudadriverconfig.read('cudadriver_config.ini')
    shm_settings = cudadriverconfig['shm_settings']
    cuda_settings = cudadriverconfig['cuda_settings']
    network_settings = cudadriverconfig['network_settings']
    
    # initalize cuda stuff
    gpu = ProcessingGPU(**dict(cuda_settings))
    for usrp in usrpconfig.sections():
        gpu.addUSRP(**dict(usrpconfig[usrp]))
    
    # create command socket server to communicate with usrp_server.py
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    cmd_sock.bind((network_settings.get('ServerHost'), network_settings.get('CUDADriverPort')))   
    
    rxshm_size = shm_settings.getint('rxshm_size') 
    txshm_size = shm_settings.getint('txshm_size')

    for ant in args.antennas:
        rx_shm_list[SIDEA].append(create_shm(ant, SWING0, SIDEA, rxshm_size, direction = RXDIR))
        rx_shm_list[SIDEA].append(create_shm(ant, SWING1, SIDEA, rxshm_size, direction = RXDIR))
        tx_shm_list.append(create_shm(ant, SWING0, SIDEA, txshm_size, direction = TXDIR))

        rx_semaphore_list[SIDEA].append(create_sem(ant, SWING0, SIDEA, direction = RXDIR))
        rx_semaphore_list[SIDEA].append(create_sem(ant, SWING1, SIDEA, direction = RXDIR))
        tx_semaphore_list.append(create_sem(ant, SWING0, SIDEA, direction = TXDIR))

    signal.signal(signal.SIGINT, sigint_handler)       
    # TODO: make this more.. robust
    cmd_sock.listen(1)
    server_conn, addr = cmd_sock.accept()

    while(True):
        cmd = recv_dtype(server_conn, np.uint8)
        handler = cudamsg_handlers[cmd](server_conn, gpu)
        handler.process()
    

if __name__ == '__main__':
    main()
