#!/usr/bin/python3
# test the usrp driver..
import unittest
import numpy as np
import sys
import posix_ipc
import pdb
import time
import subprocess

from termcolor import cprint

from drivermsg_library import *
from socket_utils import *
from cuda_driver import *

START_DRIVER = False
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

RFRATE = 10000000

if sys.hexversion < 0x030300F0:
    print('this code requires python 3.3 or greater')
    sys.exit(0)

def start_usrpserver():
    # open up ports..
    if not START_DRIVER:
        return -1
    usrp_driver = subprocess.Popen(['./usrp_driver', '--intclk', '--antenna', str(ANTENNA_UNDER_TEST), '--host', 'usrp' + str(ANTENNA_UNDER_TEST)])
    time.sleep(8)
    return usrp_driver.pid

def stop_usrpserver(sock, pid):
    # transmit clean exit command
    exitcmd = usrp_exit_command([sock])
    exitcmd.transmit()
    print('stopping USRP server')

    # kill the process just to be sure..
    if not START_DRIVER:
        return
    
    print('killing usrp_driver')
    subprocess.Popen(['pkill', 'usrp_driver'])
    time.sleep(10) # delay to let the usrp_driver settle..
    #pdb.set_trace()
    
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

        time.sleep(1)
        self.pid = start_usrpserver()

        self.serversock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        max_connect_attempts = 5
        for i in range(max_connect_attempts):
            print('attempting connection to usrp_driver')
            try:
                self.serversock.connect(('localhost', USRPDRIVER_PORT + ANTENNA_UNDER_TEST))
                break;
            except:
                print('connecting to usrp driver failed on attempt ' + str(i + 1))
                time.sleep(3)

                if i == (max_connect_attempts - 1):
                    subprocess.Popen(['pkill', 'usrp_driver'])
                    print('connecting to usrp driver failed, exiting')
                    sys.exit(1)

    def tearDown(self):
        for shm in shm_list:
            posix_ipc.unlink_shared_memory(shm)

        for sem in sem_list:
            sem.release()
            sem.unlink()

        #stop_usrpserver(self.serversock, self.pid)
        self.serversock.close()
    '''
    # test setting up usrp.. 
    def test_usrp_setup(self):
        cprint('usrp initialization','red')
        seq = create_testsequence()
        cmd = usrp_setup_command([self.serversock], seq.ctrlprm, seq, RFRATE)
        client_returns = cmd.client_return()
        for r in client_returns:
            assert(r == UHD_SETUP)
    
    # test reading/writing to shm
    def test_usrp_shm(self):
        cprint('testing shared memory usrp rxfe setup', 'red')
        # TODO: INITIALIZE SHARED MEMORY IN USRP_DRIVER, READ BACK HERE
        # populated first 10 spots in shm with 0,1,2,3,4,5,6,7,8,9
        shm = tx_shm_list[0]
        shm.seek(0)
        for i in range(10):
            c = shm.read_byte()
            print("checking byte {} of shm, read {}".format(i, c))
            assert(i == c)

    # test rxfe setup
    def test_usrp_rxfe(self):
        cprint('testing usrp rxfe setup', 'red')
        cmd = usrp_rxfe_setup_command([self.serversock], 0, 0, 0)
        cmd.transmit()
        client_returns = cmd.client_return()
        for r in client_returns:
            assert(r == UHD_RXFE_SET)


        time.sleep(5)
        cmd = usrp_rxfe_setup_command([self.serversock], 1, 1, 31)
        cmd.transmit()
        client_returns = cmd.client_return()
        for r in client_returns:
            assert(r == UHD_RXFE_SET)

        time.sleep(5)
    '''
    # test trigger pulse read
    def test_trigger_pulse(self):
        cprint('testing usrp trigger','red')
        seq = create_testsequence()
        cprint('sending setup command', 'blue')
        cmd = usrp_setup_command([self.serversock], seq.ctrlprm, seq, RFRATE)
        cmd.transmit()
        client_returns = cmd.client_return()
        for r in client_returns:
            assert(r == UHD_SETUP)

        cprint('sending trigger pulse command', 'blue')
        cmd = usrp_trigger_pulse_command([self.serversock])
        cmd.transmit()

        client_returns = cmd.client_return()
        for r in client_returns:
            assert(r == UHD_TRIGGER_PULSE) 

        cprint('checking trigger pulse data', 'blue')
        # request pulse data
        transmit_dtype(self.serversock, np.uint8(UHD_READY_DATA)) 
        cmd = usrp_ready_data_command([self.serversock])

        print('sleeping for a few seconds..')
        print('finished sleeping, looking for pulse data')

        cmd.receive(self.serversock) # get pulse data
        client_returns = cmd.client_return()
        for r in client_returns:
            assert(r == UHD_READY_DATA) 

        cprint('finished test trigger pulse', 'green')
        
        # plot data
        rx_shm = rx_shm_list[0][0]
        rx_shm.seek(0)
        ar = np.frombuffer(rx_shm, dtype=np.int16, count=10000)
        import matplotlib.pyplot as plt

        tx_shm = tx_shm_list[0]
        tx_shm.seek(0)
        ar = np.frombuffer(tx_shm, dtype=np.int16, count=10000)
        import matplotlib.pyplot as plt
        pdb.set_trace()



        

    '''
    # test clear frequency
    def test_usrp_clrfreq(self):
        pass

    def test_ready_data_process(self):
        pass
         
    '''  
if __name__ == '__main__':
    make = subprocess.call(['make'])
    time.sleep(.5)

    unittest.main()
