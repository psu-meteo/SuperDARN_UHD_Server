# library containing constants and functions for passing messages over sockets between c/python and python/python
import numpy as np
import uuid
import collections
import pdb
from socket_utils import *
from termcolor import cprint

UHD_SETUP = ord('s')
UHD_RXFE_SET = ord('r')
UHD_READY_DATA = ord('d')
UHD_TRIGGER_PULSE = ord('t')
UHD_TRIGGER_BUSY = ord('b')
UHD_TRIGGER_PROCESS = ord('p')
UHD_SETUP_READY = ord('y')
UHD_CLRFREQ = ord('c')
UHD_EXIT = ord('e')

CUDA_SETUP = ord('s')
CUDA_PROCESS = ord('p')
CUDA_GET_DATA = ord('g')
CUDA_EXIT = ord('e')

NO_COMMAND = ord('n')

ARBYSERVER_PORT = int(55421)
CUDADRIVER_PORT = int(55420)
USRPDRIVER_PORT = int(54420) # + ant

# parent class for driver socket messages
class driver_command(object):

    # class to help manage sending data 
    class socket_data(object):
        def __init__(self, data, dtype, name, nitems = 1):
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

    # ask all clients for a return value, compare against command
    # normally, client will indicate success by sending the command byte back to the server 
    def client_return(self, dtype = np.uint8, check_return = True):
        returns = []
        for client in self.clients:
            r = recv_dtype(client, dtype)
            if check_return:
                assert r == self.command, 'return {} does not match expected command of {}'.format(r, self.command)
            
            returns.append(r)
        cprint('command return success', 'yellow')
        return returns

    def receive(self, sock):
        for item in self.dataqueue:
            cprint('waiting for {}'.format(item.name), 'yellow')
            self.payload[item.name] = item.receive(sock)
            cprint('found {}'.format(item.name), 'yellow')

class server_ctrlprm(driver_command):
    def __init__(self, servers = None, ctrlprm_dict = None):
        if not (servers == None) and not isinstance(servers, collections.Iterable):
                servers = [servers]

        driver_command.__init__(self, servers, NO_COMMAND)
        if ctrlprm_dict == None:
            # stuff in default values to be over-written during rx
            ctrlprm_dict = {}
            ctrlprm_dict['radar'] = 0
            ctrlprm_dict['channel'] = 0
            ctrlprm_dict['local'] = 0
            ctrlprm_dict['priority'] = 0
            ctrlprm_dict['current_pulseseq_idx'] = 0
            ctrlprm_dict['tbeam'] = 0
            ctrlprm_dict['tbeamcode'] = 0
            ctrlprm_dict['tbeamazm'] = 0
            ctrlprm_dict['tbeamwidth'] = 0
            ctrlprm_dict['tfreq'] = 0
            ctrlprm_dict['trise'] = 0
            ctrlprm_dict['number_of_samples'] = 0
            ctrlprm_dict['buffer_index'] = 0
            ctrlprm_dict['baseband_samplerate'] = 0
            ctrlprm_dict['filter_bandwidth'] = 0
            ctrlprm_dict['match_filter'] = 0
            ctrlprm_dict['rfreq'] = 0
            ctrlprm_dict['rbeam'] = 0
            ctrlprm_dict['rbeamcode'] = 0
            ctrlprm_dict['rbeamazm'] = 0
            ctrlprm_dict['rbeamwidth'] = 0
            ctrlprm_dict['status'] = 0

        self.queue(ctrlprm_dict['radar'], np.int32, 'radar')
        self.queue(ctrlprm_dict['channel'], np.int32, 'channel')
        self.queue(ctrlprm_dict['local'], np.int32, 'local')
        self.queue(ctrlprm_dict['priority'], np.int32, 'priority')
        self.queue(ctrlprm_dict['current_pulseseq_idx'], np.int32, 'pulseseq_idx')
        self.queue(ctrlprm_dict['tbeam'], np.int32, 'tbeam')
        self.queue(ctrlprm_dict['tbeamcode'], np.uint32, 'tbeamcode')

        self.queue(ctrlprm_dict['tbeamazm'], np.float32, 'teambeamazm')
        self.queue(ctrlprm_dict['tbeamwidth'], np.float32, 'tbeamwidth')

        self.queue(ctrlprm_dict['tfreq'], np.int32, 'tfreq')
        self.queue(ctrlprm_dict['trise'], np.int32, 'trise')

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
    def __init__(self, cudas, sequence = None):
        driver_command.__init__(self, cudas, CUDA_SETUP)
        self.sequence = sequence
        
    def receive(self, sock):
        super().receive(sock)
        self.sequence = pickle_recv(sock)

    def transmit(self):
        super().transmit()
        for sock in self.clients:
            pickle_send(sock, self.sequence)

       
# re-initialize the usrp driver for a new pulse sequence
class usrp_setup_command(driver_command):
    def __init__(self, usrps, ctrlprm, sequence, rfrate):
        driver_command.__init__(self, usrps, UHD_SETUP)
        txfreq = ctrlprm['tfreq'] * 1000
        rxfreq = ctrlprm['rfreq'] * 1000
        txrate = rfrate
        rxrate = rfrate
        npulses = sequence.npulses
        num_requested_rx_samples = np.uint64(np.round((rfrate) * (sequence.ctrlprm['number_of_samples'] / sequence.ctrlprm['baseband_samplerate'])))
        self.queue(txfreq, np.float64, 'txfreq')
        self.queue(rxfreq, np.float64, 'rxfreq')
        self.queue(txrate, np.float64, 'txrate')
        self.queue(rxrate, np.float64, 'rxrate')
        self.queue(npulses, np.uint32, npulses)
        self.queue(num_requested_rx_samples, np.uint64, 'num_requested_rx_samples')
        self.queue(sequence.pulse_offsets_vector, np.float64, 'pulse_offsets_vector') # vector..

# set rxfe (amplifier and attenuator) settings 
class usrp_rxfe_setup_command(driver_command):
    def __init__(self, usrps, amp0 = 0, amp1 = 0, att = 0):
        driver_command.__init__(self, usrps, UHD_RXFE_SET)
        self.queue(amp0, np.uint8)
        self.queue(amp1, np.uint8)
        self.queue(att, np.uint8)

# start usrp trigger
class usrp_trigger_pulse_command(driver_command):
    def __init__(self, usrps):
        driver_command.__init__(self, usrps, UHD_TRIGGER_PULSE)

# command usrp drivers to ready rx sample data into shared memory
class usrp_ready_data_command(driver_command):
    def __init__(self, usrps, status = 0, antenna = 0, nsamples = 0, samples = None):
        driver_command.__init__(self, usrps, UHD_READY_DATA)

        self.queue(status, np.int32, 'status')
        self.queue(antenna, np.int32, 'antenna')
        self.queue(nsamples, np.int32, 'nsamples')

    def receive(self, sock, sock_samples = False):
        super().receive(sock)
        
        if sock_samples:
            self.samples = recv_dtype(sock, np.uint16, nitems = 2 * self.payload['nsamples'])



# command to prompt usrp drivers to run clear frequency sequence
class usrp_clrfreq_command(driver_command):
    def __init__(self, usrps):
        driver_command.__init__(self, usrps, UHD_CLRFREQ)
        # TODO: what do I need to send here?

# prompt the usrp driver to cleanly shut down, useful for testing
class usrp_exit_command(driver_command):
    def __init__(self, usrps):
        driver_command.__init__(self, usrps, UHD_EXIT)
        # TODO: what do I need to send here?


# class with pulse sequence infomation
class sequence(object):
    def __init__(self, usrp_config, npulses, tr_to_pulse_delay, pulse_offsets_vector, pulse_lens, phase_masks, pulse_masks, ctrlprm):
        self.ctrlprm = ctrlprm
        self.npulses = npulses
        self.pulse_offsets_vector = pulse_offsets_vector
        self.pulse_lens = pulse_lens
        self.phase_masks = phase_masks # phase masks are complex number to multiply phase by, so, 1 + j0 is no rotation
        self.pulse_masks = pulse_masks
        self.ready = True # TODO: what is ready flag for?
        
        # validate input sequence
        if self.ctrlprm['rfreq'] and self.ctrlprm['rfreq'] != self.ctrlprm['tfreq']:
            raise ValueError('rfreq != tfreq, this behavior is not yet supported')

        if self.ctrlprm['number_of_samples'] <= 0:
            raise ValueError('number of samples must be greater than zero!')

        if self.npulses == 0:
            raise ValueError('number of pulses must be greater than zero!')


        self.sequence_id = uuid.uuid1()

def create_testsequence():
    # fill ctrlprm with something reasonable, create test sequence

    import configparser

    ctrlprm = {\
    'radar' : 0, \
    'channel' : 0, \
    'local' : 0, \
    'priority' : 0, \
    'current_pulseseq_idx': 0, \
    'tbeam' : 0, \
    'tbeamcode' : 0, \
    'tbeamazm': 0, \
    'tbeamwidth': 0, \
    'tfreq': 10000, \
    'trise': 100, \
    'number_of_samples' : 100, \
    'buffer_index' : 0, \
    'baseband_samplerate' : 100000, \
    'filter_bandwidth' : 0, \
    'match_filter' : 0, \
    'rfreq' : 10000, \
    'rbeam' : 0, \
    'rbeamcode' : 0, \
    'rbeamazm' : 0, \
    'rbeamwidth' : 0, \
    'status' : 0}
        
    npulses = 3

    tr_to_pulse_delay = 50e-6
    pulse_offsets_vector = [1.35e-3, 6.15e-3, 12.15e-3]

    txbbrate = ctrlprm['baseband_samplerate']
    pulse_lens = [300e-6, 300e-6, 300e-6]
    phase_masks = [np.ones(int(p*txbbrate)) for p in pulse_lens] # 
    pulse_masks = [np.ones(int(p*txbbrate)) for p in pulse_lens]


    usrp_config = configparser.ConfigParser()
    usrp_config.read('usrp_config.ini')


    seq = sequence(usrp_config, npulses, tr_to_pulse_delay, pulse_offsets_vector, pulse_lens, phase_masks, pulse_masks, ctrlprm)
    return seq
