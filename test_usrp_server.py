#!/usr/bin/python3
# test the cuda driver..
import unittest
import usrp_server 
import socket
import numpy as np
import os
import posix_ipc
import pdb

import usrp_server

from drivermsg_library import *
from socket_utils import *
HOST = 'localhost'

def start_uhdserver():
    # open up ports..
    os.system('python3 usrp_server.py')

def stop_uhdserver(sock):
    send_servercmd(usrp_server.CLEAN_EXIT, sock, status = 0)

def send_servercmd(cmd, sock, status = 0):
    transmit_dtype(sock, np.int32(ord(cmd)))
    transmit_dtype(sock, np.int32(status))

class ServerTestCases(unittest.TestCase):
    def setUp(self):
        self.arbysock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.arbysock.bind((HOST, ARBYSERVER_PORT))
        self.arbysock.listen(1)

        self.usrpsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.usrpsock.bind((HOST, USRPDRIVER_PORT))
        self.usrpsock.listen(1)

        self.cudasock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.cudasock.bind((HOST, CUDADRIVER_PORT))
        self.cudasock.listen(1)

        start_uhdserver()
    
    def tearDown(self):
        stop_uhdserver(self.arbysock)
        self.arbysock.close()
        self.usrpsock.close()
        self.cudasock.close()

   
    def test_wait(self):
        pdb.set_trace()
        send_servercmd(usrp_server.WAIT, self.arbysock)
        stop_uhdserver(self.arbysock)
        '''
     # if this fails.. 
    def test_syntax(self):
        self.assertTrue(True)

    def test_pretrigger(self):
        pass
    
    def test_trigger(self):
        pass

    def test_posttrigger(self):
        pass

    def test_clrfreq(self):
        pass

    def test_getstatus(self):
        pass

    def test_registerseq(self):
        pass

    def test_ctrlprgready(self):
        pass
    
 
    def test_rxfereset(self):
        pass
        '''
       
if __name__ == '__main__':
    unittest.main()
