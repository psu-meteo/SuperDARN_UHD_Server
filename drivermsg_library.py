# library containing constants and functions for passing messages over sockets between c/python and python/python

UHD_SETUP = 's'
UHD_RXFE_SET = 'r'
UHD_READY_DATA = 'd'
UHD_TRIGGER_PULSE = 't'

CUDA_SETUP = 's'
CUDA_PROCESS = 'p'
CUDA_GET_DATA = 'g'

class driver_command(object):
    # class to help manage sending data 
    class socket_data(object):
        def __init__(self, data, dtype, name, nitems = 1):
            # if data is numpy array, do something
            # otherwise
            self.data = dtype(data)
            self.dtype = dtype
            self.name = name

        def transmit(self, sock):
            return transmit_dtype(sock, data)

        def receive(self, sock):
            return recv_dtype(sock, self.dtype)

    def __init__(self, clients, command):
        self.clients = clients
        self.queue = [] # ordered list of variables to transmit/receive
        self.payload = {} # dictionary to store received values
        self.command = np.uint8(command)
    
    def queue(self, data, dtype, name = ''):
        self.queue.append(socket_data(data, dtype, name))

    def transmit(self, sock):
        transmit_dtype(sock, np.uint8(self.command))
        for item in self.queue:
            item.transmit(sock)

    def receive(self, sock):
        for item in self.queue:
            self.payload[item.name] = item.receive(sock)

class cuda_get_data_command(driver_command):
    def __init__(self, cudas, swing = 0):
        driver_command.__init__(self, cudas, CUDA_PROCESS)
        self.queue(swing, np.uint32, 'swing')

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
    def __init__(self, usrps):
        driver_command.__init__(self, usrps, UHD_READY_DATA)
                                                    
