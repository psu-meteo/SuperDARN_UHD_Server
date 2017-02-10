#!/usr/bin/python3 
# test the swing buffers with usrp_driver and cuda_driver 
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
import clear_frequency_search 
import rosmsg 
import test_cuda_driver 
 
 
START_DRIVER = False 
ANTENNA_UNDER_TEST = [0, 1, 2, 3, 4 ,5, 6, 7] 
#ANTENNA_UNDER_TEST = [0] 
 


# parse gpu config file
driverconfig = configparser.ConfigParser()
driverconfig.read('../driver_config.ini')
shm_settings = driverconfig['shm_settings']
cuda_settings = driverconfig['cuda_settings']
network_settings = driverconfig['network_settings']

rxshm_size = shm_settings.getint('rxshm_size')
txshm_size = shm_settings.getint('txshm_size')


cuda_port = network_settings.getint('CUDADriverPort')
usrp_port = network_settings.getint('USRPDriverPort')

samplingRate_rx = cuda_settings['FSampTX']
samplingRate_tx = cuda_settings['FSampRX']



rx_shm_list = [[],[]]
tx_shm_list = []
rx_semaphore_list = [[],[]] # [side][swing]
swings = [SWING0, SWING1]
sides = [SIDEA]

# list of all semaphores/shared memory paths for cleaning up
shm_list = []
sem_list = []

def connect_to_cuda_driver():
  serversock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

  max_connect_attempts = 5
  for i in range(max_connect_attempts):
      print('attempting connection to usrp_driver')
      try:
          serversock.connect(('localhost', 55420))
          break
      except:
          print('connecting to cuda driver failed on attempt ' + str(i + 1))
          time.sleep(5)
   return serversock


def connect_to_cuda_driver():
   max_connect_attempts = 5

    serversock = []
    for iUSRP, antennaNumber in enumerate(ANTENNA_UNDER_TEST):
       serversock.append(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
       for i in range(max_connect_attempts):
           print('attempting connection to usrp_driver ( at localhost:{} )'.format(USRPDRIVER_PORT + antennaNumber))
           try:
               serversock[iUSRP].connect(('localhost', USRPDRIVER_PORT + antennaNumber))
               break;
           except:
               print('connecting to usrp driver failed on attempt ' + str(i + 1))
               time.sleep(3)

               if i == (max_connect_attempts - 1):
                   print('connecting to usrp driver failed, exiting')
                   sys.exit(1)
    return serversock


def test_trigger_pulse_one_period_and_swing(self):
      cprint('testing usrp trigger with one period','red')

      cmd = usrp_sync_time_command(self.serversock)
      cmd.transmit()
      ret = cmd.client_return()

      seq = test_cuda_driver.create_testsequence()


      nSamples_per_pulse =  self.fill_tx_shm_with_one_pulse(seq)

      # copied from usrp_driver and adjusted
      # determine the length of integration periods for all channels in seconds
      integration_period = 1.2
      PULSE_SEQUENCE_PADDING_TIME = 35e3 * 75 * 2 / 3e8 # without offset
      INTEGRATION_PERIOD_SYNC_TIME = 0.2  # todo: get from file

      tx_sample_rate = RFRATE

      nPulses_in_sequence = 8 # seq.npulses

      # to find out how much time is available in an integration period for pulse sequences, subtract out startup delay
      sampling_duration = integration_period - INTEGRATION_PERIOD_SYNC_TIME

      # calculate the pulse sequence period with padding
      time_sequence     = PULSE_SEQUENCE_PADDING_TIME + seq.pulse_offsets_vector[-1] + seq.pulse_lens[-1] /1e6
      nSamples_sequence = int(np.round(time_sequence * tx_sample_rate))

      # calculate the number of pulse sequences that fit in the available time within an integration period
      nSequences_in_period = int(np.floor(sampling_duration / time_sequence))

      # then calculate sample indicies at which pulse sequences start within a pulse sequence
      pulse_sequence_offsets_samples = [int(offset* tx_sample_rate) for offset in seq.pulse_offsets_vector]
      pulse_sequence_offsets_vector = seq.pulse_offsets_vector

      # then, calculate sample indicies at which pulses start within an integration period
      nPulses_per_period = int(nPulses_in_sequence * nSequences_in_period )
      integration_period_pulse_sample_offsets = np.zeros(nPulses_per_period, dtype=np.uint64)
      for iSequence in range(nSequences_in_period):
          for iPulse in range(nPulses_in_sequence):
              integration_period_pulse_sample_offsets[iSequence * nPulses_in_sequence + iPulse] = iSequence * nSamples_sequence + pulse_sequence_offsets_samples[iPulse]

      # calculate the number of RF transmit and receive samples 
      ctrlprm = seq.ctrlprm
      nSamples_rx = nSamples_sequence * nSequences_in_period

      cprint('sending setup command', 'blue')
      print("nSamples_rx:{}, nSamples_per_pulse:{}, integration_period_pulse_sample_offsets:".format(nSamples_rx, nSamples_per_pulse))
      print("nSequences_in_period:{}, nPulses_per_period:{}, ".format(nSequences_in_period, nPulses_per_period))
      print(integration_period_pulse_sample_offsets)
      swing = 1
      start_setup = time.time()
      cmd = usrp_setup_command(self.serversock, seq.ctrlprm['tfreq'], seq.ctrlprm['rfreq'],RFRATE, RFRATE, nPulses_per_period, nSamples_rx, nSamples_per_pulse, integration_period_pulse_sample_offsets, swing)
      cmd.transmit()
      client_returns = cmd.client_return()
      for r in client_returns:
          assert(r == UHD_SETUP)
      time_needed_for_setup = time.time() - start_setup

      for i in range(1):
       while True:
          # grab current usrp time from one usrp_driver
          start_sending = time.time()
          cmd = usrp_get_time_command(self.serversock[0])
           pdb.set_trace()
          cmd.transmit()
          usrp_time = cmd.recv_time(self.serversock[0])
          cmd.client_return()

          cprint('sending trigger pulse command', 'blue')
          trigger_time = usrp_time +  INTEGRATION_PERIOD_SYNC_TIME
          cmd = usrp_trigger_pulse_command(self.serversock, trigger_time, swing)
          cmd.transmit()
          client_returns = cmd.client_return()
          for r in client_returns:
              assert(r == UHD_TRIGGER_PULSE)

          time_needed_for_trigger = time.time() - start_sending

          start_ready_data = time.time()
          cprint('checking trigger pulse data', 'blue')
          # request pulse data
          cmd = usrp_ready_data_command(self.serversock, swing)
          cmd.transmit()
          for sock in self.serversock:
             ret = cmd.recv_metadata(sock)
             print("  recieved READY STATUS: status:{}, ant: {}, nSamples: {}, fault: {}".format(ret['status'], ret['antenna'], ret['nsamples'], ret['fault']))

          client_returns = cmd.client_return()
          for r in client_returns:
              assert(r == UHD_READY_DATA)
          time_needed_for_ready_data = time.time() - start_ready_data
          cprint('finished test trigger pulse', 'green')


      # plot data
      import matplotlib.pyplot as plt
      print("Reading data from shm:")
      for ant in ANTENNA_UNDER_TEST:
         rx_shm = rx_shm_list[0][ant*2]
         rx_shm.seek(0)
         ar = np.frombuffer(rx_shm, dtype=np.int16, count=nSamples_rx*2)
         arp = np.float32(ar[0::2]) ** 2 + np.float32(ar[1::2]) ** 2
         print("  ant {}: rms: {:5.3f}   max: {:5.3f}".format(ant, np.sqrt(np.mean(arp) ), np.sqrt(np.max(arp)) ))
         if True:
             plt.plot(ar[::2])
             plt.plot(ar[1::2])
             plt.plot(np.sqrt(arp))
             plt.show()

      print('sampled phase')
       pdb.set_trace() 


      print("Time for:  setup: {}\n  get time and trigger: {}\n  ready_data:{}".format(time_needed_for_setup, time_needed_for_trigger, time_needed_for_ready_data))


cuda_sock = connect_to_cuda_driver()
usrp_sock = connect_to_usrp_driver()


