#!/usr/bin/python3
# usrp drivers may not be running on the computer which usrp_server is running on
# and will communicate with usrp_drivers using tcp ip sockets

# usrp_server takes arby_server messages and translates them for usrp_drivers
# usrp_server handles one or more usrp_drivers

import sys
# python3 or greater is needed to support direct transfers between shm and gpu memory
if sys.hexversion < 0x030300F0:
    print('This code requires Python 3.3 or greater, exiting..')
    sys.exit(0)



import numpy as np
import pdb
import struct
import socket
import argparse
import configparser
import mmap
import warnings
import time
import uuid
import posix_ipc

from drivermsg_library import *
from socket_utils import *
from phasing_utils import * 

VERBOSE = 0
STATE_TIME = 5 # microseconds
DUMP_RAW = False

MAXANTENNAS_MAIN = 32
MAXANTENNAS_BACK = 4
RADAR_NBEAMS = 16 # TODO: set this from config files..
RADAR_BEAMWIDTH = 3.24 # TODO... set this from a config file

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

TRIGGER_BUSY = 'b'
TRIGGER_PROCESS = 'p'

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

CMD_ERROR = np.int32(10)
USRP_DRIVER_ERROR = np.int32(11)


def getDriverMsg(arbysock):
    msgtype = recv_dtype(arbysock, np.int32)
    msgstatus = recv_dtype(arbysock, np.int32)
    cmd = {'cmd':msgtype, 'status':msgstatus}
    return cmd

class sequenceManager(object):
    def __init__(self):
        self.stored_seq_id = uuid.uuid1() # id of sequence stored 
        self.loaded_seq_id = uuid.uuid1() # id of sequence sent to cuda_driver 
        self.iseq = 0 # number of sequence checks since last sequence change
        self.sequences = [] # list of sequence objects
        self.beam = -1 

    def addSequence(self, sequence):
        self.sequences.append(sequence)
        self.stored_seq_id = uuid.uuid1()
        self.iseq = 0
    
    def getSequence(self, channel):
        for sequence in self.sequences:
            if sequence.ctrlprm['channel'] == channel:
                return sequence 

        warnings.warn('Sequence for channel {} not found!'.format(channel))
        return None

    # check if new samples need to be calculated 
    # sequence changed or beam changed
    def sequenceUpdateCheck(self, beam):
        return (self.stored_seq_id != self.loaded_seq_id) or (self.beam != beam)
        
    
    # update sequences loaded id
    def sequencesLoaded(self, beam = None):
        if beam:
            self.beam = beam
            self.iseq = 0
        self.loaded_seq_id = self.stored_seq_id

class dmsg_handler(object):
    def __init__(self, arbysock, usrpsocks, cudasocks, usrp_config, cuda_config, sequence_manager):
        self.arbysock = arbysock
        self.usrpsocks = usrpsocks 
        self.cudasocks = cudasocks 
        self.usrp_config = usrp_config
        self.cuda_config = cuda_config
        self.sequence_manager = sequence_manager 
        self.status = 0;

    def process(self):
        raise NotImplementedError('The process method for this driver message is unimplemented')

    def respond(self):
        transmit_dtype(self.arbysock,np.int32(0))
        transmit_dtype(self.arbysock,np.int32(self.status))

    def _recv_ctrlprm(self):
        ctrlprm = server_ctrlprm()
        ctrlprm.receive(self.arbysock)
        self.ctrlprm = ctrlprm.payload
        
class register_seq_handler(dmsg_handler):
    def process(self):
        self._recv_ctrlprm()
        
        seq_idx = recv_dtype(self.arbysock, np.int32)
        tx_tsg_idx = recv_dtype(self.arbysock, np.int32)
        tx_tsg_len = recv_dtype(self.arbysock, np.int32) 
        tx_tsg_step = recv_dtype(self.arbysock, np.int32)
        
        # psuedo-run length encoded tsg
        tx_tsg_rep = recv_dtype(self.arbysock, np.uint8, tx_tsg_len) # number of iterations of code tx_tsg_code
        tx_tsg_code = recv_dtype(self.arbysock, np.uint8, tx_tsg_len) # code
        
        # TODO: sort out step/tx_tsg_step/STATE_TIME conversions..
        step = np.ceil(tx_tsg_step / STATE_TIME)
        seq_buf = []
        for i in range(tx_tsg_len):
            for j in range(0, np.int32(step * tx_tsg_rep[i])):
                seq_buf.append(tx_tsg_code[i])
        seq_buf = np.uint8(np.array(seq_buf))

        # extract out pulse information...
        S_BIT = np.uint8(0x01) # sample impulses 
        R_BIT = np.uint8(0x02) # tr gate, use for tx pulse times
        X_BIT = np.uint8(0x04) # transmit path, use for bb 
        A_BIT = np.uint8(0x08) # enable attenuator 
        P_BIT = np.uint8(0x10) # phase code
       
        # create masks
        samples = seq_buf & S_BIT
        tr_window = seq_buf & R_BIT
        rf_pulse = seq_buf & X_BIT
        atten = seq_buf & A_BIT
        phase_mask = seq_buf & P_BIT
    
        # extract smsep and number of samples
        sample_idx = np.nonzero(samples)[0]
        if len(sample_idx) < 3:
            warnings.warn("Warning, cannot register empty sequence")
            return

        smsep = (sample_idx[2] - sample_idx[1] + 1) * STATE_TIME
        nbb_samples = len(sample_idx)
        
        # extract pulse start timing
        tr_window_idx = np.nonzero(tr_window)[0]
        tr_rising_edge_idx = _rising_edge_idx(tr_window_idx)
        pulse_offsets_vector = tx_tsg_step * tr_rising_edge_idx 
        
        # extract tr window to rf pulse delay
        rf_pulse_idx = np.nonzero(rf_pulse)[0]
        rf_pulse_edge_idx = _rising_edge_idx(rf_pulse_idx)
        tr_to_pulse_delay = (rf_pulse_edge_idx[0] - tr_rising_edge_idx[0]) * STATE_TIME
        npulses = len(rf_pulse_edge_idx)

        # extract per-pulse phase coding and transmit pulse masks
        phase_masks = []
        pulse_masks = []
        pulse_lens = []
        for i in range(npulses):
            pstart = rf_pulse_edge_idx[i] 
            pend = pstart + _pulse_len(rf_pulse, pstart)
            phase_masks.append(phase_mask[pstart:pend])
            pulse_masks.append(rf_pulse[pstart:pend])
            pulse_lens.append((pend - pstart) * STATE_TIME) 
        # add sequence to sequence list..
        seq = sequence(self.usrp_config, npulses, tr_to_pulse_delay, pulse_offsets_vector, pulse_lens, phase_masks, pulse_masks, self.ctrlprm)
        self.sequence_manager.addSequence(seq)

# function to get the indexes of rising edges going from zero to a nonzero value in array ar
def _rising_edge_idx(ar):
    ar = np.insert(ar, 0, -2)
    edges = np.array([ar[i+1] * (ar[i+1] - ar[i] > 1) for i in range(len(ar)-1)])
    return edges[edges > 0]

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
        self._recv_ctrlprm()
        # TODO: configure per-beam phase shifts and per-beam beamforming vectors
        # I handle all this setup when the sequences are registered..?
        #tx.ready_client(&client);
        # c code sets up time_delays and tx)freqs here
        # okay, so here I configure per-beam information 
        #     tx_freqs[client->radar-1].push_back(1e3*client->tfreq);
        # time_delays[client->radar-1].push_back(10*(16/2-client->tbeam)); //10 ns per antenna per beam

        '''
        if ((ready_index[r][c]>=0) && (ready_index[r][c] <maxclients) ) {
            clients[ready_index[r][c]]=client;
        }
        else {
            clients[numclients]=client;
            ready_index[r][c]=numclients;
            numclients=(numclients+1);
        }
        if numclients >= MAXCLIENTS:
            msg.status = -2;
            numclients = numclients % MAXCLIENTS;

        '''
        #index=client.current_pulseseq_index;

        #tx.unpack_pulseseq(index);

                
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

        if(VERBOSE): 
            print('received exit command, cleaning up..')

        sys.exit(0)

class pretrigger_handler(dmsg_handler):
    def process(self):
        self._recv_ctrlprm()
        beam = self.ctrlprm['tbeam'] 

        # extract sampling info from cuda driver
        rfrate = np.int32(self.cuda_config['FSampTX'])
        print('rf rate parsed from ini: ' + str(rfrate))

        sequence = self.sequence_manager.getSequence(self.ctrlprm['channel'])
        if (self.sequence_manager.sequenceUpdateCheck(beam)):
            cmd = usrp_setup_command(self.usrpsocks, self.ctrlprm, sequence, rfrate)
            # TODO: pretrigger test crashes with broken pipe on cmd.transmit()
            cmd.transmit()
            self.sequence_manager.sequencesLoaded(beam)

        sequence.iseq += 1

        bad_transmit_times_len = np.int32(sequence.npulses)
        bad_transmit_times_start = np.uint32(sequence.pulse_offsets_vector * 1e6 - 100) # units of usec
        bad_transmit_times_duration = np.uint32(np.array([600] * sequence.npulses)) # TODO.. don't hardcode.. units of usec

        transmit_dtype(self.arbysock, bad_transmit_times_len) 
        transmit_dtype(self.arbysock, bad_transmit_times_start)
        transmit_dtype(self.arbysock, bad_transmit_times_duration)

class trigger_handler(dmsg_handler):
    def process(self):
        if len(self.sequence_manager.sequences):
            cmd = usrp_trigger_pulse_command(self.usrpsocks)
            cmd.transmit()
            
            state = chr(recv_dtype(usrpsock, np.uint8))

            if state == TRIGGER_BUSY:
                print('could not trigger, usrp driver is busy')
                # TODO HANDLE THIS WELL...
           


class posttrigger_handler(dmsg_handler):
    def process(self):
        # todo: For each sequence, clear tx_time_delay, rx_time_delay, tx_freq, and rx_freq 
        pass

class recv_get_data_handler(dmsg_handler):
    def dump_raw(self):
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
            send_data(rawsock, bb_vec_ptrs[iant], client.number_of_baseband_samples  * 2 * sizeof(int
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
        pass

    # get_data, get samples for radar/channel 
    def process(self):
        self._recv_ctrlprm()
        nbb_samples = self.ctrlprm['number_of_baseband_samples']
        
        #  check if any sequences are registered
        if not len(self.sequence_manager.sequences):
            self.status = CMD_ERROR
        
        transmit_dtype(self.arbysock, np.int32(self.status)) 
        print('USRP_SERVER GET_DATA: sent status: ' + str(self.status))
        if not self.status: 
            cmd = usrp_ready_data_command(self.usrpsocks, self.ctrlprm['channel'])
            # TODO: iseq += 1
            # TODO: form main samples
            main_samples = np.complex64(np.zeros((MAXANTENNAS_MAIN, nbb_samples)))
            main_beamformed = np.uint32(np.zeros(nbb_samples))
            back_samples = np.complex64(np.zeros((MAXANTENNAS_BACK, nbb_samples)))
            main_beamformed = np.uint32(np.zeros(nbb_samples))
            
            print('USRP_SERVER GET_DATA: checking for USRP_DRIVERS, status: ' + str(self.status))
            # check status of usrp drivers
            for usrpsock in self.usrpsocks:
                rx_status = recv_dtype(usrpsock, np.int32)
                if rx_status != 1:
                    warnings.warn('USRP driver status {} in GET_DATA'.format(rx_status))
                    self.status = USRP_DRIVER_ERROR
            
            print('USRP_SERVER GET_DATA: waiting for samples from USRP_DRIVERS, status: ' + str(self.status))
            # grab samples
            for usrpsock in self.usrpsocks:
                # receive antennas controlled by usrp driver
                nantennas = recv_dtype(usrpsock, np.uint16)
                antennas = recv_dtype(usrpsock, np.uint16, nantennas)
                
                if isinstance(antennas, (np.ndarray, np.generic)):
                    antennas = np.array([antennas])

                for ant in antennas: 
                    main_samples[ant][:] = recv_dtype(usrpsock, np.float64, nbb_samples) # TODO: check data type!?
                    #back_samples[ant][:] = recv_dtype(usrpsock, np.float64, nbb_samples)
            
            print('USRP_SERVER GET_DATA: received samples from USRP_DRIVERS, applying beamforming: ' + str(self.status))
            
            bmazm = calc_beam_azm_rad(RADAR_NBEAMS, self.ctrlprm['tbeam'], RADAR_BEAMWIDTH)

            # calculate antenna-to-antenna phase shift for steering at a frequency
            pshift = calc_phase_increment(bmazm, self.ctrlprm['tfreq'] * 1000.)

            # calculate a complex number representing the phase shift for each antenna
            beamform_main = np.array([rad_to_rect(a * pshift) for a in self.antennas])

 
            #beamform_back = np.ones(len(MAIN_ANTENNAS))
            
            def _beamform_uhd_samples(samples, phasing_matrix, n_samples, antennas):
                beamform_samples = np.ones(len(antennas))

                for i in range(n_samples):
                    itemp = np.int16(0)
                    qtemp = np.int16(0) 
                    
                    for ant in range(antennas):
                        itemp += np.real(beamform_samples[ant][i]) * np.real(phasing_matrix[ant]) - \
                                 np.imag(beamform_samples[ant][i]) * np.imag(phasing_matrix[ant]) 
                        qtemp += np.real(beamform_samples[ant][i]) * np.imag(phasing_matrix[ant]) + \
                                 np.imag(beamform_samples[ant][i]) * np.real(phasing_matrix[ant])
                    
                    beamformed[i] = complex_ui32_pack(itemp, qtemp)

            main_beamformed = _beamform_uhd_samples(main_samples, beamform_main, nbb_samples, antennas)
            back_beamformed = _beamform_uhd_samples(back_samples, beamform_back, nbb_samples, antennas)


            
            # transmit status to arby_server    
            transmit_dtype(self.arbysock, np.int32(2)) # shared memory config flag - send data over socket
            transmit_dtype(self.arbysock, np.int32(0)) # frame header offset (no header)
            transmit_dtype(self.arbysock, np.int32(0)) # buffer number
            transmit_dtype(self.arbysock, np.int32(self.ctrlprm['number_of_baseband_samples'])) # number of baseband samples

            # send samples over socket for self.channel on main and back array
            transmit_dtype(self.arbysock, main_beamformed) # TODO: send main data for channel
            #transmit_dtype(self.arbysock, back_beamformed) # TODO: send back data for channel
            
            if (DUMP_RAW):
                self.dump_raw()
            

# TODO: port this..
class clrfreq_handler(dmsg_handler):
    def process(self):
        MIN_CLRFREQ_DELAY = .2
        MAX_CLRFREQ_AVERAGE = 10
        MAX_CLRFREQ_BANDWIDTH = 512
        MAX_CLRFREQ_USABLE_BANDWIDTH = 300
        CLRFREQ_RES = 1e3 # fft frequency resolution in kHz
        
        fstart = recv_dtype(self.arbysock, np.int32) # kHz
        fstop = recv_dtype(self.arbysock, np.int32) # kHz
        filter_bandwidth = recv_dtype(self.arbysock, np.float32) # kHz (c/(2 * rsep))
        power_threshold = recv_dtype(self.arbysock, np.float32) # (typically .9, threshold before changing freq)
        nave = recv_dtype(self.arbysock, np.int32) # (typically .9, threshold before changing freq)
        self._recv_ctrlprm()

        usable_bandwidth = fstop - fstart 
        usable_bandwidth = np.floor(usable_bandwidth/2.0) * 2 # mimic behavior of gc316 driver, why does the old code do this?
        cfreq = (fstart + (fstop-fstart/2.0)) # calculate center frequency of clrfreq search in KHz
        assert usable_bandwidth > 0, "usable bandwidth for clear frequency search must be greater than 0" 
        
        # mimic behavior of gc316 drivers, if requested search bandwidth is broader than 1024 KHz, force it to 512 KHz?
        # calculate the number of points in the FFT
        num_clrfreq_samples = np.int32(pow(2,np.ceil(np.log10(1.25 * usable_bandwidth)/np.log10(2))))
        if num_clrfreq_samples > MAX_CLRFREQ_BANDWIDTH:
            num_clrfreq_samples = MAX_CLRFREQ_BANDWIDTH 
            usable_bandwidth = MAX_CLRFREQ_USABLE_BANDWIDTH 
        
        # mimic behavior of gc316 drivers, cap nave at 10
        if nave > MAX_CLRFREQ_AVERAGE:
            nave = MAX_CLRFREQ_AVERAGE

        fstart = np.ceil(cfreq - usable_bandwidth / 2.0)
        fstop = np.ceil(cfreq + usable_bandwidth / 2.0)

        unusable_sideband = np.int32((num_clrfreq_samples - usable_bandwidth)/2.0)
        clrfreq_samples = np.zeros(num_clrfreq_samples, dtype=np.complex64)
 
        # calculate center frequency of beamforming, form 
        # apply narrowband beamforming around center frequency
        bmazm = calc_beam_azm_rad(RADAR_NBEAMS, self.ctrlprm['tbeam'], RADAR_BEAMWIDTH)
        pshift = calc_phase_increment(bmazm, cfreq)

        pwr2 = np.zeros(num_clrfreq_samples) 

        for ai in range(nave):
            # gather current UHD time
            gettime_cmd = usrp_get_time_command(self.usrpsocks)
            gettime_cmd.transmit()
            usrptimes = []
            for usrpsock in self.usrpsocks:
                usrptimes.append(gettime_cmd.recv_time(usrpsock))

            # schedule clear frequency search in MIN_CLRFREQ_DELAY seconds 
            clrfreq_time = np.max(usrptimes) + MIN_CLRFREQ_DELAY

            # request clrfreq sample dump
            # TODO: what is the clear frequency rate? (c / (2 * rsep?))
            clrfreq_rate = 1000
            clrfreq_cmd = usrp_clrfreq_command(self.usrpsocks, num_clrfreq_samples, clrfreq_time, cfreq, clrfreq_rate)
            clrfreq_cmd.transmit() 
            
            # grab raw samples, apply beamforming
            for usrpsock in self.usrpsocks:
                # receive 
                nantennas = recv_dtype(usrpsock, np.uint16)
                antennas = recv_dtype(usrpsock, np.uint16, nantennas)
                
                if isinstance(antennas, (np.ndarray, np.generic)):
                    antennas = np.array([antennas])

                for ant in antennas: 
                    phase_rotation = rad_to_rect(ant * pshift)
                    combined_samples += phase_rotation * recv_dtype(usrpsock, np.complex64, num_samples) 
            
            
            # return fft of width usable_bandwidth, kHz resolution
            c_fft = np.fft(combined_samples)
            # compute power.. 
            pc_fft = (np.real(c_fft) ** 2) + (np.imag(c_fft) ** 2) / (num_clrfreq_samples ** 2)
            pwr2 += pc_fft
            pdb.set_trace()
            # todo: convert fft to power spectrum
         
        pwr2 /= nave

        pdb.set_trace() # check sideband trimming
        pwr = pwr2[sideband-1:-sideband]
        
        transmit_dtype(self.arbysock, np.int32(fstart)) # kHz
        transmit_dtype(self.arbysock, np.int32(fstop)) # kHz
        transmit_dtype(self.arbysock, np.float32(filter_bandwidth)) # kHz (c/(2 * rsep))
        transmit_dtype(self.arbysock, np.float32(power_threshold)) # (typically .9, threshold before changing freq)
        transmit_dtype(self.arbysock, np.int32(nave)) # (typically .9, threshold before changing freq)
        transmit_dtype(self.arbysock, np.float64(usable_bandwidth)) # kHz?
        transmit_dtype(self.arbysock, np.float64(pwr)) # length usable_bandwidth array?

class rxfe_reset_handler(dmsg_handler):
    def process(self):
        kodiak_set_rxfe(self.usrpsocks, default_rf_settings);

class settings_handler(dmsg_handler):
    def process(self):
        ifmode = recv_dtype(self.arbysock, np.uint32)
        rf_settings = recv_dtype(self.arbysock, np.uint32, 8)
        if_settings = recv_dtype(self.arbysock, np.uint32, 8)

        if ifmode:
            raise NotImplementedError('IF mode is unimplemented')
        
        # set RF settings
        kodiak_set_rxfe(self.usrpsocks, rf_settings);

def kodiak_set_rxfe(usrpsocks, rf_settings):
    amp0 = rf_settings[1] # amp1 in RXFESettings struct
    amp1 = rf_settings[2] # amp2 in RXFESettings struct
    att_p5dB = np.uint8((rf_settings[4] > 0))
    att_1dB = np.uint8((rf_settings[5] > 0))
    att_2dB = np.uint8((rf_settings[6] > 0))
    att_4dB = np.uint8((rf_settings[7] > 0))
    att = (att_p5dB) | (att_1dB << 1) | (att_2dB << 2) | (att_4dB << 3)

    cmd = usrp_rxfe_setup_command(usrpsocks, amp0, amp1, att)

class full_clrfreq_handler(dmsg_handler):
    def process(self):
        pdb.set_trace()
        raise NotImplementedError('full clrfreq handler is unimplemented')

class get_status_handler(dmsg_handler):
    def process(self):
        # TODO, low priority, send back fault status read from usrps
        raise NotImplementedError('fault status handler is unimplemented')
        pdb.set_trace()
        pass

class dio_table_settings_handler(dmsg_handler):
    # not applicable to USRP setup
    def process(self):
        raise NotImplementedError('dio settings handler is unimplemented')
        pdb.set_trace()

class gps_get_soft_time_handler(dmsg_handler):
    def process(self):
        raise NotImplementedError('gps get soft time handler is unimplemented')
    
class gps_get_event_time_handler(dmsg_handler):
    def process(self):
        raise NotImplementedError('gps event handler is unimplemented')

class gps_schedule_single_scan_handler(dmsg_handler):
    def process(self):
        raise NotImplementedError('gps scan handler is unimplemented')

class gps_msg_error_handler(dmsg_handler):
    def process(self):
        raise NotImplementedError('gps is unimplemented')

class gpstrigger_handler(dmsg_handler):
    def process(self):
        raise NotImplementedError('gps trigger is unimplemented')


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
    # read in config information
    usrp_config = configparser.ConfigParser()
    usrp_config.read('usrp_config.ini')
    
    cuda_config = configparser.ConfigParser()
    cuda_config.read('driver_config.ini')
    cuda_config = cuda_config['cuda_settings']


    # TODO: list of registered sequences
    sequence_manager = sequenceManager()

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
            handler = dmsg_handlers[chr(dmsg['cmd'])](arbysock, usrp_driver_socks, cuda_driver_socks, usrp_config, cuda_config, sequence_manager)
        except KeyError:
            # TODO: recover..
            warnings.warn("Warning, unrecognized arbyserver command: {}".format(dmsg['cmd']))
            continue

        handler.process()
        handler.respond()

if __name__ == '__main__':
    main()