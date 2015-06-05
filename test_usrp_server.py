#!/usr/bin/python3
# test the cuda driver..
import unittest
import usrp_server 
import socket
import numpy as np
import subprocess 
import posix_ipc
import pdb
import time
import copy
import usrp_server

from drivermsg_library import *
from socket_utils import *
HOST = 'localhost'


ctrlprm_default = {\
    'radar' : 0, \
    'channel' : 0, \
    'local' : 0, \
    'priority' : 0, \
    'current_pulseseq_idx': 0, \
    'tbeam' : 0, \
    'tbeamcode' : 0, \
    'tbeamazm': 0, \
    'tbeamwidth': 0, \
    'number_of_samples' : 0, \
    'buffer_index' : 0, \
    'baseband_samplerate' : 0, \
    'filter_bandwidth' : 0, \
    'match_filter' : 0, \
    'rfreq' : 0, \
    'rbeam' : 0, \
    'rbeamcode' : 0, \
    'rbeamazm' : 0, \
    'rbeamwidth' : 0, \
    'status' : 0}

def start_uhdserver():
    # open up ports..
    subprocess.Popen(['python3', 'usrp_server.py'])
    time.sleep(.1)

def stop_uhdserver(sock):
    send_servercmd(sock, usrp_server.CLEAN_EXIT, status = 0)

def send_servercmd(sock, cmd, status = 0):
    transmit_dtype(sock, np.int32(ord(cmd)))
    transmit_dtype(sock, np.int32(status))
  
class ServerTestCases(unittest.TestCase):
    def setUp(self):
        time.sleep(1)
        self.arbysockserver = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.arbysockserver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
        self.arbysockserver.bind((HOST, ARBYSERVER_PORT))
        self.arbysockserver.listen(1)

        self.usrpsockserver = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.usrpsockserver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
        self.usrpsockserver.bind((HOST, USRPDRIVER_PORT))
        self.usrpsockserver.listen(1)

        self.cudasockserver= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.cudasockserver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
        self.cudasockserver.bind((HOST, CUDADRIVER_PORT))
        self.cudasockserver.listen(1)

        start_uhdserver()

        (self.arbysock, addr) = self.arbysockserver.accept()
        (self.usrpsock, addr) = self.usrpsockserver.accept()
        (self.cudasock, addr) = self.cudasockserver.accept()
    
    def tearDown(self):
        stop_uhdserver(self.arbysock)
        self.arbysock.close()
        self.usrpsock.close()
        self.cudasock.close()
        self.arbysockserver.close()
        self.usrpsockserver.close()
        self.cudasockserver.close()
        time.sleep(1)
   
    def test_wait(self):
        send_servercmd(self.arbysock, usrp_server.WAIT)
        status = recv_dtype(self.arbysock, np.int32)
        stop_uhdserver(self.arbysock)
        self.assertTrue(status == 0)

    def test_getstatus(self):
        send_servercmd(self.arbysock, usrp_server.GET_STATUS)
        status = recv_dtype(self.arbysock, np.int32)
        stop_uhdserver(self.arbysock)
        self.assertTrue(status == 0)

    def test_registerseq(self):
        send_servercmd(self.arbysock, usrp_server.REGISTER_SEQ)

        # send ctrlprm
        ctrlprm = copy.copy(ctrlprm_default)
        ctrlprm_struct = server_ctrlprm(self.arbysock, ctrlprm)
        ctrlprm_struct.transmit(self.arbysock)
    
        # send other data..
        transmit_dtype(self.arbysock, np.int32(0)) # seq_idx
        transmit_dtype(self.arbysock, np.int32(0)) # tsg_idx
        transmit_dtype(self.arbysock, np.int32(0)) # tsg_len
        transmit_dtype(self.arbysock, np.int32(0)) # tsg_step

        #transmit_dtype(self.arbysock, np.int8(0)) # tsg_rep
        #transmit_dtype(self.arbysock, np.int8(0)) # tsg_code

        status = recv_dtype(self.arbysock, np.int32)
        stop_uhdserver(self.arbysock)
        self.assertTrue(status == 0)

    '''
    def test_pretrigger(self):
        pass
    
    def test_trigger(self):
        pass

    def test_posttrigger(self):
        pass

    def test_clrfreq(self):
        pass

    def test_ctrlprgready(self):
        pass
    
 
    def test_rxfereset(self):
        pass
    ''' 

if __name__ == '__main__':
    unittest.main()
