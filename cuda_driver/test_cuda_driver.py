#!/usr/bin/python3
# test the cuda driver..
import unittest
import cuda_driver
import numpy as np
import sys
import posix_ipc
import pdb
import time
import subprocess
from pylab import *

from termcolor import cprint

from drivermsg_library import *
from socket_utils import *
from shm_library import *

START_DRIVER = False


if sys.hexversion < 0x030300F0:
    print('this code requires python 3.3 or greater')
    sys.exit(0)

def start_cudaserver():
    # open up ports..
    time.sleep(.5)
    if not START_DRIVER:
        return -1
    cuda_driver = subprocess.Popen(['python3', 'cuda_driver.py'])
    time.sleep(2)
    return cuda_driver.pid

def stop_cudaserver(sock, pid):
    # transmit clean exit command
    exitcmd = cuda_exit_command([sock])
    exitcmd.transmit()
    # kill the process just to be sure..
    if not START_DRIVER:
        return
    subprocess.Popen(['pkill', str(pid)])
    time.sleep(1)
    
class CUDA_ServerTestCases(unittest.TestCase):
    def setUp(self):
        time.sleep(1)
        self.pid = start_cudaserver()
        self.serversock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        max_connect_attempts = 5 
        for i in range(max_connect_attempts): 
            print('attempting connection to usrp_driver') 
            try: 
                self.serversock.connect(('localhost', 55420))
                break
            except:
                print('connecting to cuda driver failed on attempt ' + str(i + 1))
                time.sleep(5)

        rx_shm_list[0].append(shm_mmap(shm_namer(ANTENNA, SWING0, SIDEA, direction = 'rx')))
        tx_shm_list.append(shm_mmap(shm_namer(ANTENNA, SWING0, SIDEA, direction = 'tx')))

    def tearDown(self):
        global rx_shm_list, tx_shm_list
        stop_cudaserver(self.serversock, self.pid)
        self.serversock.close()
        rx_shm_list[0][0].close()
        rx_shm_list[0] = []
        tx_shm_list[0].close()
        tx_shm_list = []
   
    def test_cuda_downsample_and_filter(self):
        cprint('testing cuda get data (downsampling)', 'red')
        seq = create_testsequence()
	
        channel_number = 1
        nAntennas = 1
        nMainAntennas = 16
        rf_nSamples_per_antenna = 459060
        bb_nSamples_per_antenna = 305

        rf_sampling_rate = 5e6 
        bb_sampling_rate = 3.333e3
    
        usrp_center_frequency = 13e6
        usrp_channel_frequency = 12e6
        tone_frequency = 1e3

        signal_frequency = (usrp_channel_frequency - usrp_center_frequency) + tone_frequency
        downconverted_tone = tone_frequency 

        # create rf and baseband test signals, copy it into shared memory, gpu accepts complex int16
        rf_test_signal = np.zeros((nAntennas, 2 * rf_nSamples_per_antenna), dtype=np.int16)
        expected_bb_signal = np.zeros((nAntennas, bb_nSamples_per_antenna), dtype=np.complex64)

        rf_time_vector = np.arange(rf_nSamples_per_antenna, dtype=np.float64) / rf_sampling_rate 
        bb_time_vector = np.arange(bb_nSamples_per_antenna, dtype=np.float64) / bb_sampling_rate
        rf_test_signal[0][0::2] = 1000 * np.sin(rf_time_vector * 2 * np.pi * signal_frequency)

        expected_bb_signal[0][:] = 1000 * np.sin(bb_time_vector * 2 * np.pi * tone_frequency)

        # rx_shm is [swing][antenna][samples]
        # rf_test_signal is [antenna][samples]
        # we're only looking at swing a, antenna 0..
        # move samples into shared memory
        rx_shm_list[0][0].write(rf_test_signal[0].tobytes())

        # send channel information
        cmd = cuda_add_channel_command([self.serversock], seq) 
        cmd.transmit()
        cmd.client_return()

        cprint('finished add channel command', 'red')
        # TODO: populate samples in shared memory
        
        # run generate pulse command to initialize memory..
        cmd = cuda_generate_pulse_command([self.serversock])
        cmd.transmit()
        cmd.client_return()
        cprint('finished generate pulse command', 'red')


        # process samples from shared memory
        cmd = cuda_process_command([self.serversock])
        cmd.transmit()
        cmd.client_return()
        cprint('finished process command', 'red')

        # copy processed samples
        cmd = cuda_get_data_command(self.serversock)
        cmd.transmit()
        
        main_samples = None
        back_samples = None

        cudasock = self.serversock
        
        cprint('waiting for number of antennas from cuda_driver', 'red')
        nAntennas = recv_dtype(cudasock, np.uint32)
        cprint('collecting data from {} antennas'.format(nAntennas), 'red')
        transmit_dtype(cudasock, channel_number, np.int32)


        for iAntenna in range(nAntennas):
            antIdx = recv_dtype(cudasock, np.uint16)

            cprint('collecting samples from antenna {}'.format(antIdx), 'red')
            num_samples = recv_dtype(cudasock, np.uint32)
            samples = recv_dtype(cudasock, np.float32, num_samples)
            samples = samples[0::2] + 1j * samples[1::2] # unpacked interleaved i/q


            #... initialize main/back sample arrays once num_samples is known
            if main_samples == None:
                main_samples = np.zeros((4, 16, num_samples/2))	
                back_samples = np.zeros((4, 4, num_samples/2))

            if antIdx < nMainAntennas:
                main_samples[channel_number][antIdx] = samples[:]

            else:
                back_samples[channel_number][antIdx - nMainAntennas] = samples[:]


        transmit_dtype(cudasock, -1, np.int32) # send channel -1 to cuda driver to end transfer process

        cprint('finished collecting samples!', 'red')

        cmd.client_return()

        # compare processed samples with expected samples 
        processed_samples = main_samples[channel_number][0] # pull samples from antenna 0
        expected_samples = expected_bb_signal[0] 
        pdb.set_trace()

        subplot(2,1,1)
        plt.plot(bb_time_vector, processed_samples)
        title('cuda processed samples')

        subplot(2,1,2)
        plt.plot(bb_time_vector, expected_samples)
        title('expected signal')
        plt.show()

    '''
    
    def test_cuda_tx_shm(self):
        cprint('testing cuda transmit sample generation', 'red')
        seq = create_testsequence()
        setupcmd = cuda_setup_command([self.serversock], seq) # cudas
        setupcmd.transmit()
        
    # TODO: test removing channels
    def test_channel_remove(self):
        print('testing removing a channel from a gpu')
  
     # TODO: test registering multiple channels
    def test_cuda_multichannel(self):
        print('testing adding multiple channels')

    '''

def dbPrint(msg):
    print(' {}: {}'.format(__file__, msg))  
       
if __name__ == '__main__':
    unittest.main()
