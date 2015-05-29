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

HOST = '127.0.0.1'
PORT = 40500

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


class cudamsg_handler(obj):
    def __init__(self, serversock, gpu):
        self.sock = serversock
        self.status = 0
        self.gpu = gpu

    def process(self):
        raise NotImplementedError('The process method for this driver message is unimplemented')

# get infomation about transmit pulse sequences, assemble tx pulse samples and setup shared memory for rx
class cuda_setup_handler(cudamsg_handler):
    def process(self):
        rx_semaphore_list[SIDEA][SWING0].acquire()
        rx_semaphore_list[SIDEA][SWING1].acquire()
        tx_semaphore_list[SIDEA].acquire()

        cmd = cuda_setup_command()
        cmd.receive(self.sock)
 
        fc = cmd.payload['center_freq']
        fsamp = cmd.payload['rxrate']
        tdelay = cmd.payload['time_delay']
        trise = cmd.payload['trise']

        gpu.generate_bbtx(fc, fsamp, nchannels, tdelay, trise)
        samples = gpu.interpolate_and_multiply(fc, fsamp, nchannels, tdelay)

        tx_shm_list[SIDEA].seek(0)
        tx_shm_list[SIDEA].write(samples.tobytes())
        tx_shm_list[SIDEA].flush()

        rx_semaphore_list[SWING0].release()
        rx_semaphore_list[SWING1].release()
        tx_semaphore_list[SIDEA].release()


# take copy and process data from shared memory, send to usrp_server via socks 
class cuda_get_data_handler(cudamsg_handler):
    def process(self):
        cmd = cuda_get_data_command()
        cmd.receive(serversock)
        samples = gpu.pull_rxdata()
        self.sock.send(samples.tobytes())
        semaphore.release()

# copy data to gpu, start processing
class cuda_process_handler(cudamsg_handler):
    def process(self):
        cmd = cuda_process_command()
        cmd.receive(serversock)

        rx_semaphore_list
        semaphore.acquire()
        
        gpu.rxsamples_shm_to_gpu(rx_shm_list[SIDEA][swing])
        gpu.rxsamples_process() 

cudamsg_handlers = {\
        CUDA_SETUP: cuda_setup_handler, \
        CUDA_GET_DATA: cuda_get_data_handler, \
        CUDA_PROCESS: cuda_process_handler}


def semaphore_namer(antenna, side, swing, direction = 'rx'):
    name = 'semaphore_{}_ant_{}_side_{}_swing_{}'.format(direction, int(antenna), int(side), int(swing))
    return name

def shm_namer(antenna, side, swing, direction = 'rx'):
    name = 'shm_{}_ant_{}_side_{}_swing_{}'.format(direction, int(antenna), int(side), int(swing))
    return name

def create_shm(antenna, swing, side, direction = 'rx')
    memory = posix_ipc.SharedMemory(shm_namer(ant, side, swing, direction), posix_ipc.O_CREATE, size=shm_size])
    mapfile = mmap.mmap(memory.fd, memory.size)
    memory.close_fd()
    shm_list.append(name)
    return mapfile

def create_semaphore(antenna, side, swing, direction):
    sem_list.append(name)
    sem = posix_ipc.SharedMemory(shm_namer(antenna, side, swing, direction), posix_ipc.O_CREATE, size=shm_size])
    return sem

def sigint_handler(signum, frame):
    pdb.set_trace()
    for shm in shm_list:
        posix_ipc.unlink_shared_memory(shm)

    for sem in sem_list:
        sem.release()
        sem.unlink()
        
    sys.exit(0)

# class to contain references to gpu-side information
# handle launching signal processing kernels
# and host/gpu communication and initialization
class ProcessingGPU(obj):
    MAXSTREAMS = 2
    def __init__(self):
        # allocate memory on cpu, compile functions
        self.rx_filtertap_s0 = np.float32(np.zeros([NFREQS, NTAPS0, 2]))
        self.rx_filtertap_s1 = np.float32(np.zeros([NFREQS, NTAPS1, 2]))
    
        self.rx_samples_rf = np.float32(np.zeros([NANTS, NFREQS, NRFSAMPS_RX]))
        self.rx_samples_if = np.float32(np.zeros([NANTS, NFREQS, NIFSAMPS_RX]))
        self.rx_samples_bb = np.float32(np.zeros([NANTS, NFREQS, NBBSAMPS_RX]))
    
        self.tx_bb_indata = np.float32(np.zeros([NANTS, NFREQS, NBBSAMPS_TX])
        self.tx_rf_outdata = np.uint16(np.zeros([NANTS, NFREQS, NRFSAMPS_TX])

        self.cu_rx_filtertaps0 = cuda.mem_alloc_like(self.rx_filtertap_s0)
        self.cu_rx_filtertaps1 = cuda.mem_alloc_like(self.rx_filtertap_s1)

        self.cu_rx_samples_rf = cuda.mem_alloc_like(self.rx_samples_rf)
        self.cu_rx_samples_if = cuda.mem_alloc_like(self.rx_samples_if)
        self.cu_rx_samples_bb = cuda.mem_alloc_like(self.rx_samples_bb)
    
        self.cu_tx_bb_indata = cuda.mem_alloc_like(self.tx_bb_indata)
        self.cu_tx_rf_outdata = cuda.mem_alloc_like(self.tx_rf_outdata)
        
        with open('rx_cuda.cu', 'r') as f:
            self.cu_rx = pycuda.compiler.SourceModule(f.read())
            self.cu_rx_multiply_and_add = self.cu_rx.get_function('multiply_and_add')
            self.cu_rx_multiply_mix_add = self.cu_rx.get_function('multiply_mix_add')

        
        with open('tx_cuda.cu', 'r') as f:
            self.cu_tx = pycuda.compiler.SourceModule(f.read())
            self.cu_tx_interpolate_and_multiply = self.cu_rx.get_function('interpolate_and_multiply')
            self.cu_txfreq_rads = self.cu_tx.get_global('txfreq_rads')[0]
            self.cu_txoffsets_rads = self.cu_rx.get_global('txphasedelay_rads')[0]

        self.tx_block = (NRFSAMPLES / NBBSAMPLES, NFREQS, 1)
        self.tx_grid = (NBBSAMPLES, NANTS, 1)

        self.rx_grid_0 = (NRFSAMPS_RX / DMRATE0, NANTS, 1)
        self.rx_grid_1 = (NBBSAMPS, NANTS, 1)
        self.rx_block_0 = (NTAPS0 / 2, NFREQS, 1)
        self.rx_block_1 = (NBBSAMPS)
        
        self.streams = [drv.Stream() for i in range(MAXSTREAMS)]
    
    # transfer samples from shared memory to gpu
    def rxsamples_shm_to_gpu(self, shm):
        shm.seek(0)
        #shm.read(rx_samples_rf.nbytes())
        #sample_str = rx_shm_list[SIDEA][SWING0].

        #.seek(0)
        #c = mapfile.read_byte()

        cuda.memcpy_dtoh_async(self.cu_rx_samples_rf, shm, stream = self.streams[swing])

               
    # kick off async data processing
    def rxsamples_process(self):
        self.cu_rx_multiply_and_add(self.cu_rx_samples_rf, self.cu_rx_samples_if, self.rx_filtertap_s0, block = self.rx_block_0, grid = self.rx_grid_0, stream = self.streams[swing])
        self.cu_rx_multiply_mix_add(self.cu_rx_samples_if, self.cu_rx_samples_bb, self.rx_filtertap_s1, block = self.rx_block_1, grid = self.rx_grid_1, stream = self.streams[swing])

    def pull_rxdata(self):
        cuda.memcpy_dtoh(self.cu_rx_samples_bb, self.rx_samples_bb)
        return self.rx_samples_bb

    def interpolate_and_multiply(self, bbvec, fc, fsamp, nchannels, tdelay):
        self._set_mixerfreqs(fc, fsamp, nchannels)
        self._set_phasedelays(fc, fsamp, nchannels, tdelay)

        cuda.memcpy_htod(self.cu_tx_bb_indata, bbvec)
        self.cu_tx_interpolate_and_multiply(self.cu_tx_bb_indata, self.cu_tx_rf_outdata, self.cu_txfreqs_rads, self.cu_txoffsets_rads, block = self.tx_block, grid = self.tx_grid)
        cuda.memcpy_dtoh(self.cu_tx_rf_outdata, self.tx_rf_outdata)
        return self.tx_rf_outdata
   
    def generate_bbtx(self, seqbuf, trise):
        bb_vec = np.complex64(np.zeros(seqbuf.shape))
        pdb.set_trace() 
        bb_vec((seqbuf & X_BIT) & (seqbuf & P_BIT)) = -1
        bb_vec((seqbuf & X_BIT) & (seqbuf & P_BIT)) = 1 
        '''
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
        self.cu_tx_bb_indata = bb_vec

    def _set_mixerfreqs(fc, fsamp, nchannels):
        mixer_freqs = np.float64([2 * np.pi * (f - fc[i]) / fsamp for i in range(nchannels)])
        cuda.memcpy_htod(self.txfreq_rads, mixer_freqs)

    def _set_phasedelays(fc, fsamp, nchannels, tdelay):
        phase_delays = np.float32(np.mod([2 * np.pi * 1e-9 * tdelay[i] * fc[i]] for i in range(nchannels)], 2 * np.pi)) 
        cuda.memcpy_htod(self.txphasedelay_rads, phase_delays)
 
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
        filter_taps1[:,:,:] = 0.

        for i in range(ntaps1/2-dmrate1/4, ntaps1/2+dmrate1/4):
            filter_taps1[:,i,0] = 4./dmrate1

def main():
    # initalize cuda stuff
    parser = argparse.ArguementParser()
    parser.add_arguement('antennas', type=int, default=[0, 1], nargs='+')
    
    # create command socket server to communicate with usrp_server.py
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cmd_sock.bind((HOST, PORT))   

    shm_size = 0 # TODO: get shm size from usrp server, tx and rx
    
    # create shared memory space for each antenna (see demo in posix_ipc-1.0.0/demo/)
    # - create two shared memory spaces and semaphores for receive samples
    # - create one shared memory space and semaphore for transmit sample

    for ant in antennas:
        rx_shm_list[SIDEA].append(create_shm(ant, SIDEA, SWING0, direction = RXDIR))
        rx_shm_list[SIDEA].append(create_shm(ant, SIDEA, SWING1, direction = RXDIR))

        tx_shm_list.append(create_shm(shm_namer(ant, SIDEA, SWING0, direction = TXDIR))

        rx_semaphore_list[SIDEA].append(posix_ipc.Semaphore(semaphore_namer(ant, SIDEA, SWING0, direction = RXDIR), posix_ipc.O_CREX))
        rx_semaphore_list[SIDEA].append(posix_ipc.Semaphore(semaphore_namer(ant, SIDEA, SWING1, direction = RXDIR), posix_ipc.O_CREX))

        tx_semaphore_list.append(posix_ipc.Semaphore(semaphore_namer(ant, SIDEA, SWING0, direction = SWING0), posix_ipc.O_CREX))

    signal.signal(signal.SIGINT, sigint_handler)       

    while(True):
        cmd = recv_dtype(sock, np.uint8)
        handler = cudamsg_handlers[cmd](cmd_sock, gpu)
        handler.process()
    

if __name__ == '__main__':
    main()
