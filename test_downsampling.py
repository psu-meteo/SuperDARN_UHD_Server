#!/usr/bin/python3
# test the cuda driver..
#import unittest
import cuda_driver
import numpy as np
import sys
import posix_ipc
import pdb
import time
import subprocess
import pycuda.driver as cuda
import myPlotTools as mpt

from termcolor import cprint

from drivermsg_library import *
from socket_utils import *
from shm_library import *




def stop_cudaserver(sock):
    # transmit clean exit command
    exitcmd = cuda_exit_command([sock])
    exitcmd.transmit()
    time.sleep(1)
    
def tearDown(serversock):
    global rx_shm_list, tx_shm_list
    stop_cudaserver(serversock)
    serversock.close()
    rx_shm_list[0][0].close()
    rx_shm_list[0] = []
   # tx_shm_list[0].close()
   # tx_shm_list = []

def dbPrint(msg):
    print(' {}: {}'.format(__file__, msg))


serversock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

max_connect_attempts = 5 
for i in range(max_connect_attempts): 
    print('attempting connection to usrp_driver') 
    try: 
        serversock.connect(('localhost', CUDADRIVER_PORT))
        break
    except:
        print('connecting to cuda driver failed on attempt ' + str(i + 1))
        time.sleep(5)



   
        
cprint('testing cuda get data (downsampling)', 'red')
seq = create_testsequence_uafscan()
nSamplesBB = seq.ctrlprm['number_of_samples']
bbrate_rx = seq.ctrlprm['baseband_samplerate']
fsamprx = 10000000

nSamples_rx_rf = int(np.round(nSamplesBB / bbrate_rx * fsamprx))
rx_shm_size = 160000000 # from driver confi.ini

cudasocks = [] # TODO...
#rx_shm_list[0].append(shm_mmap(shm_namer(ANTENNA, SWING0, SIDEA, direction = 'rx')))
rx_shm_list[0].append(cuda_driver.create_shm(ANTENNA, SWING0, SIDEA, rx_shm_size, direction = 'rx'))
#tx_shm_list.append(shm_mmap(shm_namer(ANTENNA, SWING0, SIDEA, direction = 'tx')))


cmd = cuda_setup_command([serversock], seq) # cudas
cmd.transmit()
cmd.client_return()

cmd = cuda_add_channel_command([serversock], seq) 
cmd.transmit()
cmd.client_return()


cmd = cuda_generate_pulse_command(serversock)
cmd.transmit()
cmd.client_return()


nAnts = 1
rx_rf_data = cuda.pagelocked_empty((nAnts, 1, 2 * nSamples_rx_rf), np.int16, mem_flags=cuda.host_alloc_flags.DEVICEMAP)
 
rx_rf_data[:] = 0
timeVector = np.array(range(nSamples_rx_rf)) / fsamprx
rx_rf_data[0][0][0::2] = np.sin(2*np.pi*1e6*timeVector)*250


rx_shm_list[0][0].seek(0)
rx_shm_list[0][0].write(rx_rf_data[0].tobytes())
rx_shm_list[0][0].flush()


cmd = cuda_process_command([serversock])
cmd.transmit()
cmd.client_return()



# Copy from usrm_server.py
cmd = cuda_get_data_command(serversock)

cmd.transmit()

# TODO: setup through all sockets
# recv metadata from cuda drivers about antenna numbers and whatnot
# fill up provided main and back baseband sample buffers

# TODO: fill in sample buffer with baseband sample array
cudasock = serversock
    # TODO: recieve antenna number

transmit_dtype(cudasock, 0 , np.int32)
nants = recv_dtype(cudasock, np.uint32)
dbPrint(' received nants= {}'.format( nants))
print(seq.ctrlprm)
main_samples = np.complex64(np.zeros((nants, nSamplesBB)))


for ant in range(nants):
    ant = recv_dtype(cudasock, np.uint16)
    dbPrint('received idx ant = {}'.format(ant))
    num_samples = recv_dtype(cudasock, np.uint32)
    samples = recv_dtype(cudasock, np.float32, num_samples)
     
#    if ant < MAX_MAIN_ARRAY_TODO:
    main_samples[ant] = samples[0::2] + samples[1::2] *1j
 #   else:
 #      back_samples[ant - MAX_MAIN_ARRAY_TODO] = samples
		   
# samples is interleaved I/Q float32 [self.nants, self.nchans, nbbsamps_rx]

# TODO: currently assuming samples is main samples, no back array!


# pdb.set_trace()
cmd.client_return


mpt.plot_time_freq(main_samples[0], bbrate_rx)
#pdb.set_trace()

# setupcmd = cuda_get_data_command([self.serversock]) #seq 
# setupcmd.transmit()

# transmit_dtype(self.serversock, 0, np.int32)
# nAnts = recv_dtype(self.serversock, np.int32)

stop_cudaserver(serversock)
tearDown(serversock)
