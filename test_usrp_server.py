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
import usrp_server

from drivermsg_library import *
from socket_utils import *
HOST = 'localhost'

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
        print('setting up arbysock..')
        self.arbysockserver = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.arbysockserver.bind((HOST, ARBYSERVER_PORT))
        self.arbysockserver.listen(1)

        print('setting up usrpdriver sock..')
        self.usrpsockserver = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.usrpsockserver.bind((HOST, USRPDRIVER_PORT))
        self.usrpsockserver.listen(1)

        print('setting up cudadriver sock..')
        self.cudasockserver= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
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


   
    def test_wait(self):
        send_servercmd(self.arbysock, usrp_server.WAIT)
        status = recv_dtype(self.arbysock, np.int32)
        stop_uhdserver(self.arbysock)
        self.assertTrue(status == 0)
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
