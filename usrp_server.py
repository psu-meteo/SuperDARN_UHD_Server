# usrp drivers may not be running on the computer which usrp_server is running on
# and will communicate with usrp_drivers using tcp ip sockets

# usrp_server takes arby_server messages and translates them for usrp_drivers
# usrp_server handles one or more usrp_drivers

import numpy as np
import pdb
import struct
import socket
import argparse
import mmap
import sys
import warnings
import time
import posix_ipc

from drivermsg_library import *
from socket_utils import *
# see posix_ipc python library
# http://semanchuk.com/philip/posix_ipc/

VERBOSE = 1
STATE_TIME = 5 # microseconds

# arby server commands
REGISTER_SEQ = '+'
CTRLPROG_READY = '1'
CTRLPROG_END = '@'
WAIT = 'W'
PRETRIGGER = '3'
TRIGGER = '4'
GPS_TRIGGER = 'G'
POST_TRIGGER = '5'
RECV_GET_DATA = 'd'
FULL_CLRFREQ = '-'
CLRFREQ = 'C'
DIO_RXFE_RESET = 'r'
GET_STATUS = 'S'
SETTINGS = 'R'
DIO_TABLE_SETTINGS = 'T'
GPS_GET_SOFT_TIME = 't'
GPS_GET_EVENT_TIME = 'e'
GPS_SCHEDULE_SINGLE_SCAN = 's'
GPS_MSG_ERROR = 'X'
CLEAN_EXIT = 'x'

# human readable descriptions of commands for status messages
ARBY_COMMAND_STRS = {
        REGISTER_SEQ : 'register sequence', \
        CTRLPROG_READY : 'control program ready', \
        CTRLPROG_END : 'control program end', \
        WAIT : 'wait', \
        PRETRIGGER : 'pretrigger', \
        TRIGGER : 'trigger', \
        GPS_TRIGGER : 'gps trigger', \
        POST_TRIGGER : 'post trigger', \
        RECV_GET_DATA : 'recv get data', \
        FULL_CLRFREQ : 'full clearfreq', \
        CLRFREQ : 'clearfreq', \
        DIO_RXFE_RESET : 'reset rxfe dio', \
        GET_STATUS : 'get status',\
        SETTINGS : 'settings',\
        DIO_TABLE_SETTINGS : 'dio table settings', \
        GPS_GET_SOFT_TIME : 'get gps soft time', \
        GPS_GET_EVENT_TIME : 'get gps event time', \
        GPS_SCHEDULE_SINGLE_SCAN : 'schedule gps single scan', \
        GPS_MSG_ERROR : 'gps message error', \
        CLEAN_EXIT : 'clean exit'}

#TODO: cleanup global variables
CMD_ERROR = 10 
sequence_list = []
old_seq_id = 0
old_beam = -1
iseq = 0

def hash_sequence_list(sequence_list):
    return ''.join([s.__hash__() for s in sequence_list])

def getDriverMsg(arbysock):
    msgtype = recv_dtype(arbysock, np.int32)
    msgstatus = recv_dtype(arbysock, np.int32)
    cmd = {'cmd':msgtype, 'status':msgstatus}
    return cmd


class sequence(object):
    def __init__(self, channel, npulses, pulse_offsets_vector, phase_masks, ctrlprm):
        self.ctrlprm = ctrlprm
        self.npulses = npulses
        self.num_requested_samples = num_requested_samples
        self.pulse_offsets_vector = pulse_offsets_vector
        self.priority = priority
        self.bb_vec = self._make_bb_vec()
        self.phase_mask = phase_mask
        self.ready = True # what is ready flag for?

    def __key(self):
        return (self.txfreq, self.rxfreq, self.txrate, self.rxrate, self.npulses, self.num_requested_samples, self.pulse_offsets_vector)

    def __eq__(self, x, y):
        return type(x) == type(y) and x.__key() == y.__key()

    def __hash__(self):
        return hash(self.__key())


class dmsg_handler(object):
    def __init__(self, arbysock, usrpsocks, cudasocks):
        self.arbysock = arbysock
        self.usrpsocks = usrpsocks 
        self.cudasocks = cudasocks 
        self.status = 0;

    def process(self):
        raise NotImplementedError('The process method for this driver message is unimplemented')

    def respond(self):
        self.arbysock.sendall(np.int32(self.status).tostring())

    def _recv_ctrlprm(self):
        ctrlprm = server_ctrlprm()
        ctrlprm.receive(self.arbysock)
        self.ctrlprm = ctrlprm.payload
        
class register_seq_handler(dmsg_handler):
    def process(self):
        self._recv_ctrlprm()
        
        seq_idx = recv_dtype(self.arbysock, np.int32) # what is this for? channel index..
        tx_tsg_idx = recv_dtype(self.arbysock, np.int32) # what is this for?
        tx_tsg_len = recv_dtype(self.arbysock, np.int32) # what is this for? ..
        tx_tsg_step = recv_dtype(self.arbysock, np.int32) # what is this for..
        
        # psuedo-run length encoded tsg
        tx_tsg_rep = recv_dtype(self.arbysock, np.uint8, tx_tsg_len) # number of iterations of code tx_tsg_code
        tx_tsg_code = recv_dtype(self.arbysock, np.uint8, tx_tsg_len) # code

        step = np.ceil(tx_tsg_step / STATE_TIME)
        seqbuf = []
        for i in range(tx_tsg_len):
            for j in range(0, step * tx_tsg_rep[i]):
                seq_buf.append(tx_tsg_code[i])
        
        # extract out pulse information...
        S_BIT = 0x01 # sample impulses 
        R_BIT = 0x02 # tr gate, use for tx pulse times
        X_BIT = 0x04 # transmit path, use for bb 
        A_BIT = 0x08 # enable attenuator 
        P_BIT = 0x10 # phase code

        # create masks
        samples = seq_buf & S_BIT
        tr_window = seq_buf & R_BIT
        rf_pulse = seq_buf & X_BIT
        atten = seq_buf & A_BIT
        phase_mask = seq_buf & P_BIT
    
        # extract smsep and number of samples
        samples_idx = np.nonzero(samples)
        smsep = (sample_idx[2] - sample_idx[1]) * STATE_TIME
        nbb_samples = sum(samples_idx)
        
        # extract pulse timing
        tr_window_idx = np.nonzero(tr_window)
        tr_rising_edge_idx = rising_edge_idx(tr_window_idx)
        pulse_offsets_vector = STEP * tr_window[tr_rising_edge_idx] 

        # extract tr window to rf pulse delay
        rf_pulse_idx = np.nonzero(rf_pulse)
        rf_pulse_edge_idx = rising_edge_idx(rf_pulse_idx)
        tr_to_pulse_delay = (rf_pulse_idx[rf_pulse_edge_idx[0]] - tr_window_idx[tr_rising_edge_idx[0]]) * STATE_TIME
        npulses = len(rf_pulse_edge_idx)

        # extract per-pulse phase coding masks
        phase_masks = []
        for i in range(npulses)
            pstart = rf_pulse_edge_idx[i] 
            pend = pstart + _pulse_len(rf_pulse, pstart)
            phase_masks.append(phase_mask[pstart:pend])
        
        # add sequence to sequence list..
        seq = sequence(channel, tr_to_pulse_delay, nbb_samples, pulse_offsets_vector, phase_masks, self.ctrlprm)
        sequence_list.append(seq)

def _rising_edge_idx(ar):
    return np.nonzero(np.diff(np.insert(ar, 0, -1)) - 1)

# returns the run length of a pulse in array ar starting at index idx
def _pulse_len(ar, idx):
    runlen = 0
    for element in ar[idx:]:
        if not element: 
            break
        runlen += 1
    return runlen

class ctrlprog_ready_handler(dmsg_handler):
    def process(self):
        # TODO: something here
        # when is 
        self._recv_ctrlprm()
        #r = client.radar - 1
        #c = client.channel - 1

        #rx.ready_client(&client);
        #tx.ready_client(&client);
        '''
        if ((ready_index[r][c]>=0) && (ready_index[r][c] <maxclients) ) {
            clients[ready_index[r][c]]=client;
        }
        else {
            clients[numclients]=client;
            ready_index[r][c]=numclients;
            numclients=(numclients+1);
        }
        '''
        #index=client.current_pulseseq_index;

        #tx.unpack_pulseseq(index);

        if numclients >= MAXCLIENTS:
            msg.status = -2;
            numclients = numclients % MAXCLIENTS;
        
class ctrlprog_end_handler(dmsg_handler):
    def process(self):
        self._recv_ctrlprm()
        

class wait_handler(dmsg_handler):
    def process(self):
        pass

class exit_handler(dmsg_handler):
    def process(self):
        # clean up and exit
        self.arbysock.close()
        for sock in self.usrpsocks:
            sock.close()
        for sock in self.cudasocks:
            sock.close()
        # TODO: clean up shared memory
        if(VERBOSE): 
            print('received exit command, cleaning up..')
        sys.exit(0)

class pretrigger_handler(dmsg_handler):
    def process(self):
        # setup for next trigger
        seq_id = hash_sequence_list(sequence_list)
        
        # provide fresh samples to usrp_driver.cpp shared memory if beam or sequence has changed
        if (seq_id != old_seq_id) or (self.beam != old_beam):
            iseq = 0
            cmd = usrp_setup_command(self.usrpsocks, self.channel) # txfreq = 0, rxfreq = 0, txrate = 0, rxrate = 0, npulses = 0, num_requested_samples = 0, pulse_offsets_vector = 0):
            cmd.transmit()

        old_seq_id = seq_id
        old_beam = new_beam

        '''
        send_data(msgsock, &bad_transmit_times.length, sizeof(bad_transmit_times.length));
        send_data(msgsock, bad_transmit_times.start_usec, sizeof(uint32_t)*bad_transmit_times.length
        send_data(msgsock, bad_transmit_times.duration_usec, sizeof(uint32_t)*bad_transmit_times.len
        send_data(msgsock, &msg, sizeof(struct DriverMsg));
        '''

class trigger_handler(dmsg_handler):
    def process(self):
        if len(sequences):
            cmd = usrp_trigger_pulse_command(self.usrpsocks)
            cmd.transmit()


class posttrigger_handler(dmsg_handler):
    def process(self):
        numclients = 0

        #tx.clear_channel_list(); clear list of tx_freqs and time_delays (time delay between adjacent antennas)
        #rx.clear_channel_list(); clear list of rx frequency offsets? 
        #ready_index[r][c]=-1;

class recv_get_data_handler(dmsg_handler):
    def dump_raw(self):
        pass


        # TODO: port to python..
        '''
        int32_t nants = rx.get_num_ants_per_radar();
        int32_t ctrlprmsize = sizeof(ControlPRM);
        int32_t seqbufsize = tx.get_seqbuf_len(client.current_pulseseq_index);
        int64_t usrp_intsec = start_time.get_full_secs();
        double usrp_fracsec = start_time.get_frac_secs();
        std::vector<uint8_t> seqbuf = tx.get_seq_buf(client.current_pulseseq_index);
        struct timeval tdumpstart, tdumpend;
        gettimeofday(&tdumpstart,NULL);
        struct sockaddr_in serv_addr;
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(55501);
        serv_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        rawsock=socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

        if(connect(rawsock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
            printf("raw data socket connection failed..\n");
        }
        // send number of antennas  
        send_data(rawsock, &nants, sizeof(int32_t));
        // send size of ctrlprm
        send_data(rawsock, &ctrlprmsize, sizeof(int32_t));
        // send ctrlprm struct
        send_data(rawsock, &client, ctrlprmsize);
        // send time when packets sent to usrp 
        send_data(rawsock, &t1, 2 * sizeof(int64_t));
        // send integer and fractional usrp time of start of pulse
        send_data(rawsock, &usrp_intsec, sizeof(int64_t));
        send_data(rawsock, &usrp_fracsec, sizeof(double));
        // send number of samples
        send_int32(rawsock, rx.get_num_bb_samples());
        // send samples
        for(int32_t iant = 0; iant < nants; iant++) {
            send_data(rawsock, bb_vec_ptrs[iant], client.number_of_samples  * 2 * sizeof(int
        }
        // send length of seq_buf
        send_data(rawsock, &seqbufsize, sizeof(int32_t));
        // send seq_buf
        send_data(rawsock, &(seqbuf[0]), seqbufsize * sizeof(uint8_t));
        send_int32(rawsock, STATE_TIME);
        send_int32(rawsock, RX_OFFSET);
        send_int32(rawsock, DDS_OFFSET);
        send_int32(rawsock, TX_OFFSET);

        close(rawsock);
        printf("RAW DATA DUMP COMPLETE..\n");
        gettimeofday(&tdumpend,NULL);
        int64_t tdump = 1000000 * (tdumpend.tv_sec - tdumpstart.tv_sec) + (tdumpend.tv_usec
        std::cout << "raw data transmission time (us) : " << tdump << "\n";
        '''

    def process(self):
        self._recv_ctrlprm()

        if not len(sequence_length):
            self.status = CMD_ERROR
        
        transmit_dtype(self.arbysock, self.status) 

        if not self.status: 
            cmd = usrp_ready_data_command(self.usrpsocks, self.channel)
            # TODO: check for off-by-one error in self.channel
            iseq += 1
            
            # TODO: for each channel/antenna?
            transmit_dtype(self.arbysock, np.int32(2)) # shared memory configuration flag
            transmit_dtype(self.arbysock, np.int32(0)) # frame header offset (no header)
            transmit_dtype(self.arbysock, np.int32(0)) # buffer number
            transmit_dtype(self.arbysock, np.int32(self.ctrlprm['number_of_samples'])) # number of baseband samples
            
            main_samples = np.complex64(np.zeros((NANTENNAS, NBBSAMPLES)))
            back_samples = np.complex64(np.zeros((NANTENNAS, NBBSAMPLES)))
            for usrp in self.usrpsocks:
                for ant in usrpantennas: # TODO: FIX THIS 
                main_samples = self.usrpsock.
                back_samples = 

                # TODO: grab samples over socket 
            

            # TODO: do rx beamforming

            # send samples over socket for self.channel on main and back array
            transmit_dtype(self.arbysock, np.uint32(samples), self.ctrlprm['number_of_samples']) # TODO: send main data for channel
            transmit_dtype(self.arbysock, np.uint32(samples), self.ctrlprm['number_of_samples']) # TODO: send back data for channel
           
            if (DUMP_RAW):
                self.dump_raw()
            

class clrfreq_handler(dmsg_handler):
    def process(self):
        self._recv_clrfrqprm()
        self._rect_ctrlprm()
        nave = 0
        usable_bandwidth = clrfreq_parameters.end - clrfreq_parameters.start
        usable_bandwidth = np.floor(usable_bandwidth/2) * 2


        N = int(pow(2,np.ceil(np.log10(1.25*usable_bandwidth) / log10(2))))

        if N > 1024:
            N = 512
            usable_bandwidth = 300
            start = int(center-usable_bandwidth/2+0.49999)
            end = int(center+usable_bandwidth/2+0.49999)
        '''
        for(int chan = 0; chan < nrx_antennas; chan++) {
            usrp->set_rx_freq(1e3*center, chan);
            usrp->set_rx_rate(1e6, chan);
            if(verbose>-1) std::cout << "Actual RX rate for clr freq search: " << N << " kHz\n";
        }

        /* set up search parameters search_bandwidth > usable_bandwidth */
        search_bandwidth=N;
        //search_bandwidth=800;            
        start=(int)(center-search_bandwidth/2.0+0.49999);
        end=(int)(center+search_bandwidth/2+0.49999);
        unusable_sideband=(search_bandwidth-usable_bandwidth)/2;
        clrfreq_parameters.start=start;
        clrfreq_parameters.end=end;

        pwr.clear();
        pwr2.clear();
        pwr.resize(N,0);
        pwr2.resize(usable_bandwidth,0);

        rx_clrfreq_rval= recv_clr_freq(
                usrp,
                rx_stream,
                center,
                usable_bandwidth,
                (int) client.filter_bandwidth/1e3,
                clrfreq_parameters.nave,
                10,
                &pwr2.front());


        send_data(msgsock, &clrfreq_parameters, sizeof(struct CLRFreqPRM));
        send_data(msgsock, &usable_bandwidth, sizeof(int));
        send_data(msgsock, &pwr2.front(), sizeof(double)*usable_bandwidth);
        send_data(msgsock, &msg, sizeof(struct DriverMsg));
        '''
        pass

class rxfe_reset_handler(dmsg_handler):
    def process(self):
        kodiak_set_rxfe(usrp, default_rf_settings);

class settings_handler(dmsg_handler):
    def process(self):
        ifmode = recv_dtype(self.arbysock, np.uint32)
        rf_settings = recv_dtype(self.arbysock, np.uint32, 8)
        if_settings = recv_dtype(self.arbysock, np.uint32, 8)

        if ifmode:
            raise NotImplementedError('IF mode is unimplemented')
        
        # set RF settings
        kodiak_set_rxfe(usrp, rf_settings);

def kodiak_set_rxfe(handler, rf_settings):
    pass

class full_clrfreq_handler(dmsg_handler):
    pass

class get_status_handler(dmsg_handler):
    def process(self):
        pass

class dio_table_settings_handler(dmsg_handler):
    pass

class gps_get_soft_time_handler(dmsg_handler):
    pass

class gps_get_event_time_handler(dmsg_handler):
    pass

class gps_schedule_single_scan_handler(dmsg_handler):
    pass

class gps_msg_error_handler(dmsg_handler):
    pass

class gpstrigger_handler(dmsg_handler):
    pass


dmsg_handlers = {\
    REGISTER_SEQ : register_seq_handler, \
    CTRLPROG_READY : ctrlprog_ready_handler, \
    CTRLPROG_END : ctrlprog_end_handler, \
    WAIT : wait_handler, \
    PRETRIGGER : pretrigger_handler, \
    TRIGGER : trigger_handler,\
    GPS_TRIGGER : gpstrigger_handler,\
    POST_TRIGGER: posttrigger_handler,\
    RECV_GET_DATA : recv_get_data_handler,\
    FULL_CLRFREQ : full_clrfreq_handler,\
    CLRFREQ : clrfreq_handler,\
    DIO_RXFE_RESET : rxfe_reset_handler,\
    GET_STATUS : get_status_handler,\
    SETTINGS : settings_handler,\
    DIO_TABLE_SETTINGS : dio_table_settings_handler,\
    GPS_GET_SOFT_TIME : gps_get_soft_time_handler,\
    GPS_GET_EVENT_TIME : gps_get_event_time_handler,\
    GPS_SCHEDULE_SINGLE_SCAN : gps_schedule_single_scan_handler,\
    GPS_MSG_ERROR : gps_msg_error_handler,\
    CLEAN_EXIT : exit_handler}

def main():
    # TODO: read in config information
    
    # open USRPs drvers and initialize them
    usrp_drivers = ['localhost'] # computers to talk to for cuda
    usrp_driver_socks = []

    cuda_drivers = ['localhost'] # computers with cuda drivers
    cuda_driver_socks = []

    arby_server = 'localhost' # hostname arby server..

    # open arby server socket
    try:
        arbysock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        arbysock.connect((arby_server, ARBYSERVER_PORT))
    except ConnectionRefusedError: 
        warnings.warn("Arby server connection failed")
        sys.exit(1)

    time.sleep(.05)
    # connect to usrp_driver servers
    try:
        for d in usrp_drivers:
            usrpsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            usrpsock.connect((d, USRPDRIVER_PORT))
            usrp_driver_socks.append(usrpsock)
    except ConnectionRefusedError: 
        warnings.warn("USRP server connection failed")
        sys.exit(1)


    time.sleep(.05)
    # connect cuda_driver servers
    try:
        for c in cuda_drivers:
            cudasock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            cudasock.connect((c, CUDADRIVER_PORT))
            cuda_driver_socks.append(cudasock)
    except ConnectionRefusedError: 
        warnings.warn("cuda server connection failed")
        sys.exit(1)

    
    while True:
        dmsg = getDriverMsg(arbysock)
        if(VERBOSE):
            print('received {} command from arbyserver, processing'.format(ARBY_COMMAND_STRS[chr(dmsg['cmd'])]))
        try:
            handler = dmsg_handlers[chr(dmsg['cmd'])](arbysock, usrp_driver_socks, cuda_driver_socks)
        except KeyError:
            # TODO: recover..
            warnings.warn("Warning, unrecognized arbyserver command: {}".format(dmsg['cmd']))
            continue

        handler.process()
        handler.respond()

if __name__ == '__main__':
    main()
