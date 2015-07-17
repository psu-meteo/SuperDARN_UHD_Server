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

from drivermsg_library import *
from socket_utils import *
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
    time.sleep(1)
    return cuda_driver.pid

def stop_cudaserver(sock, pid):
    # transmit clean exit command
    exitcmd = cuda_exit_command([sock])
    exitcmd.transmit()
    # kill the process just to be sure..
    if not START_DRIVER:
        return
    subprocess.Popen(['pkill', str(pid)])
    

class CUDA_ServerTestCases(unittest.TestCase):
    def setUp(self):
        time.sleep(1)
        self.pid = start_cudaserver()
        self.serversock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serversock.connect(('localhost', CUDADRIVER_PORT))

    def tearDown(self):
        stop_cudaserver(self.serversock, self.pid)
    '''
    def test_cuda_exit(self):
        # setUp, tearDown..
        pass
    '''
    def test_cuda_setup(self):
        # try passing in parameters
        # testing cuda setup

        # TODO: construct sequence setup..
        bmnum = 0
        seq = create_testsequence()
        pdb.set_trace()
        setupcmd = cuda_setup_command([self.serversock], seq, bmnum)
        setupcmd.transmit()

    '''
    
    def test_cuda_getdata(self):
        # populate shm
        getdata = cuda_get_data_command([self.serversock])
        getdata.transmit()

    def test_cuda_process(self):
        # try copying samples then processing them..
        processcmd = cuda_process_command([self.serversock])
        processcmd.transmit()
    '''
        
if __name__ == '__main__':
    unittest.main()
