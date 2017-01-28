#!/usr/bin/python3
# test the usrp driver..
import unittest
import numpy as np
import sys
import posix_ipc
import pdb
import time
import subprocess
import configparser


from termcolor import cprint

sys.path.insert(0, '../python_include')
sys.path.insert(0, '../cuda_driver')

from drivermsg_library import *
from socket_utils import *
from cuda_driver import *
from radar_config_constants import *


START_DRIVER = False
ANTENNA_UNDER_TEST = 0


# parse gpu config file
driverconfig = configparser.ConfigParser()
driverconfig.read('../driver_config.ini')
shm_settings = driverconfig['shm_settings']
cuda_settings = driverconfig['cuda_settings']
network_settings = driverconfig['network_settings']

rxshm_size = shm_settings.getint('rxshm_size')
txshm_size = shm_settings.getint('txshm_size')


USRPDRIVER_PORT = network_settings.getint('USRPDriverPort')

RFRATE = 10000000


rx_shm_list = [[],[]]
tx_shm_list = []
rx_semaphore_list = [[],[]] # [side][swing]
swings = [SWING0, SWING1]
sides = [SIDEA]

# list of all semaphores/shared memory paths for cleaning up
shm_list = []
sem_list = []
             

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
        antennas = [ANTENNA_UNDER_TEST]
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
            print('attempting connection to usrp_driver (at localhost:{}'.format(USRPDRIVER_PORT + ANTENNA_UNDER_TEST))
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

        cprint('populating tx shm sample buffer','red')
        tx_shm = tx_shm_list[0]
        tx_shm.seek(0)

        # create test tone
        tone = 50e3 # 50 khz tone..
        amplitude = np.iinfo(np.int16).max / 2 # 1/2 of max amplitude
        print("nPulses: {}, tx_time: {}, RFRATE: {}".format(seq.npulses, seq.tx_time, RFRATE))
        samplet = np.arange(0,seq.npulses * seq.tx_time/1e6 ,1/RFRATE)[:-1]
        sample_real = np.int16(amplitude * np.cos(2 * np.pi * tone * samplet))
        sample_imag = np.int16(amplitude * np.sin(2 * np.pi * tone * samplet))
        sample_tx = np.zeros(2 * len(samplet), dtype=np.int16)
        nSamples_per_pulse =  (seq.tx_time/1e6*RFRATE)

        sample_tx[0::2] = sample_real
        sample_tx[1::2] = sample_imag
        tx_shm.write(sample_tx.tobytes())
        num_requested_rx_samples = np.uint64(np.round((RFRATE) * (seq.ctrlprm['number_of_samples'] / seq.ctrlprm['baseband_samplerate'])))  # TODO: thiss has to changed for integration period 

        cprint('sending setup command', 'blue')
        offset_sample_list = [offset * RFRATE for offset in seq.pulse_offsets_vector]
        cmd = usrp_setup_command([self.serversock], seq.ctrlprm['tfreq'], seq.ctrlprm['rfreq'],RFRATE, RFRATE, seq.npulses, num_requested_rx_samples, nSamples_per_pulse, offset_sample_list)
        # cmd = usrp_setup_command([self.serversock], seq.ctrlprm, seq, RFRATE)
        cmd.transmit()
        client_returns = cmd.client_return()
        for r in client_returns:
            assert(r == UHD_SETUP)
    
        for i in range(1):
            # grab current usrp time from one usrp_driver
            cmd = usrp_get_time_command(self.serversock)
            cmd.transmit()
            usrp_time = cmd.recv_time(self.serversock)
            cmd.client_return()
            trigger_time = usrp_time +  INTEGRATION_PERIOD_SYNC_TIME

            cprint('sending trigger pulse command', 'blue')
            cmd = usrp_trigger_pulse_command([self.serversock], trigger_time)
            cmd.transmit()

            client_returns = cmd.client_return()
            for r in client_returns:
                assert(r == UHD_TRIGGER_PULSE) 

            cprint('checking trigger pulse data', 'blue')
            # request pulse data
#            transmit_dtype(self.serversock, np.uint8(UHD_READY_DATA)) 
            cmd = usrp_ready_data_command([self.serversock])
            cmd.transmit()

            print('looking for pulse data')
            # TODO: delete this
            #transmit_dtype(self.serversock, np.int32(ANTENNA_UNDER_TEST ))

            ret = cmd.recv_metadata(self.serversock)
            print("  recieved READY STATUS: status:{}, ant: {}, nSamples: {}, fault: {}".format(ret['status'], ret['antenna'], ret['nsamples'], ret['fault']))

#            cmd.receive(self.serversock) # get pulse data
            client_returns = cmd.client_return()
            for r in client_returns:
                assert(r == UHD_READY_DATA) 

            cprint('finished test trigger pulse', 'green')
            
        # plot data
        num_rx_samples = np.uint64(np.round((RFRATE) * (seq.ctrlprm['number_of_samples'] / seq.ctrlprm['baseband_samplerate'])))
        rx_shm = rx_shm_list[0][0]
        rx_shm.seek(0)
        ar = np.frombuffer(rx_shm, dtype=np.int16, count=num_rx_samples)
        arp = np.sqrt(np.float32(ar[0::2]) ** 2 + np.float32(ar[1::2]) ** 2)
        print('sampled power')
        print(arp[:200000:1000])

        print('sampled phase')
#        pdb.set_trace() 

        

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
