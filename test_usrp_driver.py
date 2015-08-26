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
from cuda_driver import *

START_DRIVER = True 
ANTENNA_UNDER_TEST = 1

rx_shm_list = [[],[]]
tx_shm_list = []
rx_semaphore_list = [[],[]] # [side][swing]
swings = [SWING0, SWING1]
sides = [SIDEA]

# list of all semaphores/shared memory paths for cleaning up
shm_list = []
sem_list = []

rxshm_size = 160000000
txshm_size = 51200000



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
    print('stopping USRP server')

    # kill the process just to be sure..
    if not START_DRIVER:
        return
    subprocess.Popen(['pkill', str(pid)])
    time.sleep(1)
    

class USRP_ServerTestCases(unittest.TestCase):
    def setUp(self):
        antennas = [1]
        for ant in antennas:
            rx_shm_list[SIDEA].append(create_shm(ant, SWING0, SIDEA, rxshm_size, direction = RXDIR))
            rx_shm_list[SIDEA].append(create_shm(ant, SWING1, SIDEA, rxshm_size, direction = RXDIR))
            tx_shm_list.append(create_shm(ant, SWING0, SIDEA, txshm_size, direction = TXDIR))
            tx_shm_list.append(create_shm(ant, SWING1, SIDEA, txshm_size, direction = TXDIR))

        rx_semaphore_list[SIDEA].append(create_sem(ant, SWING0))
        rx_semaphore_list[SIDEA].append(create_sem(ant, SWING1))

        # TODO: setup shared memory
        time.sleep(1)

        self.pid = start_usrpserver()
        self.serversock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serversock.connect(('localhost', USRPDRIVER_PORT))


    def tearDown(self):
        for shm in shm_list:
            posix_ipc.unlink_shared_memory(shm)

        for sem in sem_list:
            sem.release()
            sem.unlink()

        stop_usrpserver(self.serversock, self.pid)
        self.serversock.close()
    
    def test_usrp_setup(self):
        pass
    '''
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
    ''' 
      
      
if __name__ == '__main__':
    unittest.main()
