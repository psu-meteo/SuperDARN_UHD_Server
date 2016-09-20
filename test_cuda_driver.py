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

from termcolor import cprint

from drivermsg_library import *
from socket_utils import *
from shm_library import *

START_DRIVER = True 


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
                self.serversock.connect(('localhost', CUDADRIVER_PORT))
                break
            except:
                print('connecting to cuda driver failed on attempt ' + str(i + 1))
                time.sleep(5)

        self.cudasocks = [] # TODO...
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
   
    ''' 
    def test_cuda_getdata(self):
        cprint('testing get data', 'red')

        seq = create_testsequence()
        setupcmd = cuda_setup_command([self.serversock], seq) # cudas
        setupcmd.transmit()

        print('running get data command')
        getdata = cuda_get_data_command([self.serversock])
        getdata.transmit()
    '''
    def test_cuda_downsample_and_filter(self):
        
        cprint('testing cuda get data (downsampling)', 'red')
        seq = create_testsequence()
        setupcmd = cuda_setup_command([self.serversock], seq) # cudas
        setupcmd.transmit()

        setupcmd = cuda_add_channel_command([self.serversock], seq) 
        setupcmd.transmit()


        cmd = cuda_process_command([self.serversock])
        cmd.transmit()

       # setupcmd = cuda_get_data_command([self.serversock]) #seq 
       # setupcmd.transmit()

       # transmit_dtype(self.serversock, 0, np.int32)
       # nAnts = recv_dtype(self.serversock, np.int32)
    '''
    def test_cuda_add_channel(self):
        cprint('testing cuda add channel', 'red')
        seq = create_testsequence()
        setupcmd = cuda_add_channel_command([self.serversock], seq) 
        setupcmd.transmit()


    def test_cuda_setup(self):
        cprint('testing cuda setup', 'red')
        seq = create_testsequence()
        setupcmd = cuda_setup_command([self.serversock], seq) # cudas
        setupcmd.transmit()

    
    def test_cuda_tx_shm(self):
        cprint('testing cuda transmit sample generation', 'red')
        seq = create_testsequence()
        setupcmd = cuda_setup_command([self.serversock], seq) # cudas
        setupcmd.transmit()
        




    
 
    # TODO: test removing channels
    def test_channel_remove(self):
        print('testing removing a channel from a gpu')
   
    # TODO: test shared memory
    def test_cuda_shm(self):
        print('testing tx sample shared memory')


    # TODO: test sample downconversion of rx pulse sequence
    def test_cuda_downsample(self):
        print('testing gpu downsampling')

    # TODO: test registering multiple channels
    def test_cuda_multichannel(self):
        print('testing adding multiple channels')


    '''
       
if __name__ == '__main__':
    unittest.main()
