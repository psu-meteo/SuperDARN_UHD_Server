# library containing constants and functions for passing messages over sockets between c/python and python/python
import numpy as np
from socket_utils import *
UHD_SETUP = ord('s')
UHD_RXFE_SET = ord('r')
UHD_READY_DATA = ord('d')
UHD_TRIGGER_PULSE = ord('t')

CUDA_SETUP = ord('s')
CUDA_PROCESS = ord('p')
CUDA_GET_DATA = ord('g')
CUDA_EXIT = ord('e')

NO_COMMAND = ord('n')

ARBYSERVER_PORT = 55421
CUDADRIVER_PORT = 55420
USRPDRIVER_PORT = 55422


class driver_command(object):
    # class to help manage sending data 
    class socket_data(object):
        def __init__(self, data, dtype, name, nitems = 1):
            # if data is numpy array, do something
            # otherwise
            self.data = dtype(data)
            self.dtype = dtype
            self.name = name
            self.nitems = nitems

        def transmit(self, sock):
            return transmit_dtype(sock, self.data)

        def receive(self, sock):
            return recv_dtype(sock, self.dtype, self.nitems)

    def __init__(self, clients, command):
        self.clients = clients
        self.dataqueue = [] # ordered list of variables to transmit/receive
        self.payload = {} # dictionary to store received values
        self.command = np.uint8(command)
    
    def queue(self, data, dtype, name = '', nitems = 1):
        self.dataqueue.append(self.socket_data(data, dtype, name, nitems = nitems))

    def transmit(self):
        for clientsock in self.clients:
            if self.command != NO_COMMAND:
                transmit_dtype(clientsock, np.uint8(self.command))

            for item in self.dataqueue:
                item.transmit(clientsock)

    def receive(self, sock):
        for item in self.dataqueue:
            self.payload[item.name] = item.receive(sock)

class server_ctrlprm(driver_command):
    def __init__(self, servers, ctrlprm_dict):
        driver_command.__init__(self, servers, NO_COMMAND)
        self.queue(ctrlprm_dict['radar'], np.int32, 'radar')
        self.queue(ctrlprm_dict['channel'], np.int32, 'channel')
        self.queue(ctrlprm_dict['local'], np.int32, 'local')
        self.queue(ctrlprm_dict['priority'], np.int32, 'priority')
        self.queue(ctrlprm_dict['current_pulseseq_idx'], np.int32, 'pulseseq_idx')
        self.queue(ctrlprm_dict['tbeam'], np.int32, 'tbeam')
        self.queue(ctrlprm_dict['tbeamcode'], np.uint32, 'tbeamcode')

        self.queue(ctrlprm_dict['tbeamazm'], np.float32, 'teambeamazm')
        self.queue(ctrlprm_dict['tbeamwidth'], np.float32, 'tbeamwidth')

        self.queue(ctrlprm_dict['number_of_samples'], np.int32, 'number_of_samples')
        self.queue(ctrlprm_dict['buffer_index'], np.int32, 'buffer_index')

        self.queue(ctrlprm_dict['baseband_samplerate'], np.float32, 'baseband_samplerate')
        self.queue(ctrlprm_dict['filter_bandwidth'], np.int32, 'filter_bandwidth')

        self.queue(ctrlprm_dict['match_filter'], np.int32, 'match_filter')
        self.queue(ctrlprm_dict['rfreq'], np.int32, 'rfreq')
        self.queue(ctrlprm_dict['rbeam'], np.int32, 'rbeam')
        self.queue(ctrlprm_dict['rbeamcode'], np.uint32, 'rbeamcode')
        self.queue(ctrlprm_dict['rbeamazm'], np.float32, 'rbeamazm')
        self.queue(ctrlprm_dict['rbeamwidth'], np.float32, 'rbeamwidth')

        self.queue(ctrlprm_dict['status'], np.int32, 'status')

        # TODO: set these properly 
        name_arr = np.uint8(np.zeros(80))
        self.queue(name_arr, np.uint8, nitems = 80)

        description_arr = np.uint8(np.zeros(120))
        self.queue(description_arr, np.uint8, nitems = 120)

class cuda_get_data_command(driver_command):
    def __init__(self, cudas, swing = 0):
        driver_command.__init__(self, cudas, CUDA_PROCESS)
        self.queue(swing, np.uint32, 'swing')

class cuda_exit_command(driver_command):
    def __init__(self, cudas, swing = 0):
        driver_command.__init__(self, cudas, CUDA_EXIT)


class cuda_process_command(driver_command):
    def __init__(self, cudas, swing = 0):
        driver_command.__init__(self, cudas, CUDA_PROCESS)
        self.queue(swing, np.uint32, 'swing')

class cuda_setup_command(driver_command):
    def __init__(self, cudas, txfreq = 0, rxfreq = 0, txrate = 0, rxrate = 0, npulses = 0, num_requested_samples = 0, pulse_offsets_vector = 0):
        driver_command.__init__(self, cudas, CUDA_SETUP)

        self.queue(txfreq, np.float64, 'txfreq')
        self.queue(rxfreq, np.float64, 'rxfreq')
        self.queue(txrate, np.float64, 'txrate')
        self.queue(rxrate, np.float64, 'rxrate')
        self.queue(npulses, np.uint32, 'npulses')
        self.queue(num_requested_samples, np.uint64, 'num_requested_samples')
        self.queue(pulse_offsets_vector, np.float64, 'pulse_offsets_vector') # vector.. TODO: figure out how this works

class usrp_setup_command(driver_command):
    def __init__(self, usrps, txfreq = 0, rxfreq = 0, txrate = 0, rxrate = 0, npulses = 0, num_requested_samples = 0, pulse_offsets_vector = 0):
        driver_command.__init__(self, usrps, UHD_SETUP)

        self.queue(txfreq, np.float64, 'txfreq')
        self.queue(rxfreq, np.float64, 'rxfreq')
        self.queue(txrate, np.float64, 'txrate')
        self.queue(rxrate, np.float64, 'rxrate')
        self.queue(npulses, np.uint32, 'npulses')
        self.queue(num_requested_samples, np.uint64, 'num_requested_samples')
        self.queue(pulse_offsets_vector, np.float64, 'pulse_offsets_vector') # vector..

class usrp_rxfe_setup_command(driver_command):
    def __init__(self, usrps, amp0 = 0, amp1 = 0, att = 0):
        driver_command.__init__(self, usrps, UHD_RXFE_SET)
        self.queue(amp0, np.uint8)
        self.queue(amp1, np.uint8)
        self.queue(att, np.uint8)

class usrp_trigger_pulse_command(driver_command):
    def __init__(self, usrps):
        driver_command.__init__(self, usrps, UHD_TRIGGER_PULSE)

class usrp_ready_data_command(driver_command):
    def __init__(self, usrps, channel):
        driver_command.__init__(self, usrps, UHD_READY_DATA)
        self.queue(channel, np.int32, 'channel')
