#!/usr/bin/python3
# test the usrp driver..
import unittest
import numpy as np
import sys
import posix_ipc
import pdb
import time
import subprocess

from drivermsg_library import *
from socket_utils import *

START_DRIVER = True 
ANTENNA_UNDER_TEST = 1
if sys.hexversion < 0x030300F0:
    print('this code requires python 3.3 or greater')
    sys.exit(0)

def start_usrpserver():
    # open up ports..
    time.sleep(.5)
    if not START_DRIVER:
        return -1
    usrp_driver = subprocess.Popen(['./usrp_driver', '--antenna', str(ANTENNA_UNDER_TEST), '--host', 'usrp' + str(ANTENNA_UNDER_TEST)])
    time.sleep(2)
    return usrp_driver.pid

def stop_usrpserver(sock, pid):
    # transmit clean exit command
    exitcmd = usrp_exit_command([sock])
    exitcmd.transmit()
    # kill the process just to be sure..
    if not START_DRIVER:
        return
    subprocess.Popen(['pkill', str(pid)])
    time.sleep(1)
    

class USRP_ServerTestCases(unittest.TestCase):
    def setUp(self):
        time.sleep(1)
        self.pid = start_usrpserver()
        self.serversock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serversock.connect(('localhost', USRPDRIVER_PORT))

    def tearDown(self):
        stop_usrpserver(self.serversock, self.pid)
        self.serversock.close()
    
    def test_usrp_setup(self):
        pass

    def test_usrp_shm(self):
        pass

    def test_usrp_rxfe(self):
        pass

    def test_usrp_clrfreq(self):
        pass

    def test_ready_data_process(self):
        pass
 
    def test_ready_data_busy(self):
        pass
  
    def test_trigger_pulse(self):
        pass
       
      
      
if __name__ == '__main__':
    unittest.main()
