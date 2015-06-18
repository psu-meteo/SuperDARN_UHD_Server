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
HOST = '127.0.0.1'
START_SERVER = True 


S_BIT = np.uint8(0x01) # sample impulses 
R_BIT = np.uint8(0x02) # tr gate, use for tx pulse times
X_BIT = np.uint8(0x04) # transmit path, use for bb 
A_BIT = np.uint8(0x08) # enable attenuator 
P_BIT = np.uint8(0x10) # phase code

CTRLPRM_DEFAULT = {\
    'radar' : 0, \
    'channel' : 0, \
    'local' : 0, \
    'priority' : 0, \
    'current_pulseseq_idx': 0, \
    'tbeam' : 0, \
    'tbeamcode' : 0, \
    'tbeamazm': 0, \
    'tbeamwidth': 0, \
    'tfreq': 0, \
    'trise': 0, \
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
        if START_SERVER:
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

        drivertype  = recv_dtype(self.arbysock, np.int32)
        status = recv_dtype(self.arbysock, np.int32)
        self.assertTrue(status == 0)

    def test_getstatus(self):
        send_servercmd(self.arbysock, usrp_server.GET_STATUS)

        drivertype  = recv_dtype(self.arbysock, np.int32)
        status = recv_dtype(self.arbysock, np.int32)
        self.assertTrue(status == 0)
    
    def test_registerseq(self):
        send_servercmd(self.arbysock, usrp_server.REGISTER_SEQ)

        # send ctrlprm
        ctrlprm = copy.copy(CTRLPRM_DEFAULT)
        ctrlprm_struct = server_ctrlprm(self.arbysock, ctrlprm)
        ctrlprm_struct.transmit()
    
        
        # create test sequence
        # ###_______###_______##%%______
        # ~20 ms long
        # step is 5 us
        # 300 us pulse width
        # 5 ms pulse to pulse
        # 60 us TR to pulse

        step = 5e-6
        seqlen = 20e-3
        smsep = 300e-6
        nsamples = 66
        pulses = [1.35e-3, 6.15e-3, 12.15e-3]
        trtotx = 50e-6
        pulselen = 300e-6
        t_seq = smsep * nsamples

        # create decompress vector, then run length encode..

        # create sample_mask, S_BIT every 60 bins
        sample_mask = np.zeros(int(np.round(smsep / step)))
        sample_mask[0] = S_BIT
        sample_mask = np.tile(sample_mask, nsamples)
        
        # create tr_mask
        tr_mask = np.zeros(len(sample_mask))
        for pulse in pulses:
            trstartidx = int(np.round((pulse - trtotx)  / step))
            trstopidx = int(np.round((pulse + trtotx + pulselen) / step))
            tr_mask[trstartidx:trstopidx] = R_BIT

        # create transmit mask  
        x_mask = np.zeros(len(sample_mask))
        for pulse in pulses:
            startidx = int(np.round((pulse)  / step))
            stopidx = int(np.round((pulse + pulselen) / step))
            x_mask[startidx:stopidx] = X_BIT
   
        
        # create phase mask  
        p_mask = np.zeros(len(sample_mask))
        startidx = int(np.round((pulses[-1] + pulselen / 2) / step))
        stopidx = int(np.round((pulses[-1] + pulselen) / step))
        p_mask[startidx:stopidx] = P_BIT

        seq_buf = sample_mask + tr_mask + x_mask + p_mask
        
        # compress seq_buf
        # rle, see http://mail.scipy.org/pipermail/numpy-discussion/2007-October/029378.html
        pos, = np.where(np.diff(seq_buf) != 0)
        pos = np.concatenate(([0],pos+1,[len(seq_buf)]))
        rle = [(a,b,seq_buf[a]) for (a,b) in zip(pos[:-1],pos[1:])]        
         
        tsg_rep = []
        tsg_code = []

        for c in rle:
            tsg_code.append(c[2])
            tsg_rep.append(c[1] - c[0])

        transmit_dtype(self.arbysock, np.int32(0)) # seq_idx
        transmit_dtype(self.arbysock, np.int32(0)) # tsg_idx
        transmit_dtype(self.arbysock, np.int32(len(tsg_code))) # tsg_len TODO: create this
        transmit_dtype(self.arbysock, np.int32(5)) # tsg_step TODO: what are the units on this?

        transmit_dtype(self.arbysock, np.int8(tsg_rep))
        transmit_dtype(self.arbysock, np.int8(tsg_code))

        drivertype  = recv_dtype(self.arbysock, np.int32)
        status = recv_dtype(self.arbysock, np.int32)
        self.assertTrue(status == 0)

    def test_ctrlprgready(self):
        send_servercmd(self.arbysock, usrp_server.CTRLPROG_READY)
        
        # send ctrlprm
        ctrlprm = copy.copy(CTRLPRM_DEFAULT)
        ctrlprm_struct = server_ctrlprm(self.arbysock, ctrlprm)
        ctrlprm_struct.transmit()

        # check status and close up ports

        drivertype  = recv_dtype(self.arbysock, np.int32)
        status = recv_dtype(self.arbysock, np.int32)
        self.assertTrue(status == 0)


    def test_ctrlprogend(self):
        send_servercmd(self.arbysock, usrp_server.CTRLPROG_END)
        
        # send ctrlprm
        ctrlprm = copy.copy(CTRLPRM_DEFAULT)
        ctrlprm_struct = server_ctrlprm(self.arbysock, ctrlprm)
        ctrlprm_struct.transmit()

        # check status and close up ports

        drivertype  = recv_dtype(self.arbysock, np.int32)
        status = recv_dtype(self.arbysock, np.int32)
        self.assertTrue(status == 0)
    
    def test_get_data(self):
        # first, register sequence
        print('testing get_data')
        self.test_registerseq()

        # next, request data for that channel
        send_servercmd(self.arbysock, usrp_server.RECV_GET_DATA)
        
        # send ctrlprm
        ctrlprm = copy.copy(CTRLPRM_DEFAULT)
        ctrlprm_struct = server_ctrlprm(self.arbysock, ctrlprm)
        ctrlprm_struct.transmit()

        # check status
        status = recv_dtype(self.arbysock, np.int32)
        self.assertTrue(status == 0)

        # pretend to be a usrp driver..
        # send nantennas (int16)
        # send antenna numbers (int16 list)
        # send main and back samples as complex floats64s for main/back

        # receive shm/socket config
        shm_config = recv_dtype(self.arbysock, np.int32)
        frame_header = recv_dtype(self.arbysock, np.int32)
        buffer_number = recv_dtype(self.arbysock, np.int32)
        nsamples = recv_dtype(self.arbysock, np.int32)
    
        # receive sample vector
        main_beamformed = recv_dtype(self.arbysock, np.uint32)
        back_beamformed = recv_dtype(self.arbysock, np.uint32)

        # TODO: is this expected to be sent again?

        drivertype  = recv_dtype(self.arbysock, np.int32)
        status = recv_dtype(self.arbysock, np.int32)
        print('get data test complete')

    def test_trigger(self):
        send_servercmd(self.arbysock, usrp_server.TRIGGER)
        status = recv_dtype(self.arbysock, np.int32)
        self.assertTrue(status == 0)


'''
    def test_pretrigger(self):
        pass
    

    def test_posttrigger(self):
        pass

    def test_clrfreq(self):
        pass

    
    def test_rxfereset(self):
        pass
    '''
if __name__ == '__main__':
    unittest.main()
