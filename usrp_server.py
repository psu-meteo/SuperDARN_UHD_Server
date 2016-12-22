#!/usr/bin/python3
# todo, write mock arbyserver that can handle these commands
# TODO: write mock arby server that can feed multiple normalscans with false data..

import sys
import numpy as np
import threading
import logging
import pdb
import socket
import time
import configparser
import copy

from termcolor import cprint
from phasing_utils import *
from socket_utils import *
from rosmsg import *
from drivermsg_library import *
from radar_config_constants import *
from clear_frequency_search import read_restrict_file, clrfreq_search
from profiling_tools import *

MAX_CHANNELS = 10
RMSG_FAILURE = -1
RMSG_SUCCESS = 0
RADAR_STATE_TIME = .0001
CHANNEL_STATE_TIMEOUT = 120
# TODO: move this out to a config file
RESTRICT_FILE = '/home/radar/repos/SuperDARN_MSI_ROS/linux/home/radar/ros.3.6/tables/superdarn/site/site.kod/restrict.dat.inst'

# TODO: pull these from config file
STATE_INIT = 'INIT'
STATE_RESET = 'RESET'
STATE_WAIT = 'WAIT'
STATE_PRETRIGGER = 'PRETRIGGER'
STATE_TRIGGER = 'TRIGGER'
STATE_CLR_FREQ = 'CLR_FREQ_WAIT'
STATE_GET_DATA = 'GET_DATA'

debug = True

# handle arbitration with multiple channels accessing the usrp hardware
# track state of a grouping of usrps
# merge information from multiple control programs, handle disparate settings
# e.g, ready flags and whatnot
class RadarHardwareManager:
    def __init__(self, port):
        self.client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client_sock.bind(('localhost', port))
        cprint('listening on port ' + str(port) + ' for control programs', 'blue')
        
        self.ini_file_init()
        self.usrp_init()
        #self.rxfe_init()
        self.cuda_init()

        self.restricted_frequencies = read_restrict_file(RESTRICT_FILE)

    def run(self):
        def spawn_channel(conn):
            # start new radar channel handler
            channel = RadarChannelHandler(conn, self)
            self.channels.append(channel)
            try:
                channel.run()
            except socket.error:
                print("RadarChannelHandler: Socket error => Deleting channel... ")
                self.deleteRadarChannel(channel)
   #         else:
   #             print("RadarChannelHandler: Other error => Deleting channel... ")
   #             self.deleteRadarChannel(channel)
   #             conn.close()
   #             print("RadarChannelHandler: Unexpected error is:", sys.exc_info()[0])
   #             raise

        # TODO: add lock support
        def radar_state_machine():
            cprint('starting radar hardware state machine', 'blue')
            self.state = STATE_INIT
            self.next_state = STATE_INIT

            while True:
                self.state = self.next_state
                self.next_state = STATE_RESET
                if self.state == STATE_INIT:
                    # TODO: write init code?
                    self.next_state = STATE_WAIT

                if self.state == STATE_WAIT:
                    self.next_state = STATE_WAIT

                    # TODO: think about radar state priority
                    # e.g, CLR_FREQ > PRETRIGGER > TRIGGER > GET_DATA?
                    for ch in self.channels:
                        if ch.state == STATE_CLR_FREQ:
                            self.next_state = STATE_CLR_FREQ
                            break
                        if ch.state == STATE_PRETRIGGER:
                            self.next_state = STATE_PRETRIGGER
                            break
                        if ch.state == STATE_TRIGGER:
                            self.next_state = STATE_TRIGGER
                            break
                        if ch.state == STATE_GET_DATA:
                            self.next_state = STATE_GET_DATA
                            break

                if self.state == STATE_CLR_FREQ:
                    # do a clear frequency search for channel requesting one
                    for ch in self.channels:
                        if ch.state == STATE_CLR_FREQ:
                            self.clrfreq(ch)
                            ch.state = STATE_WAIT
                    self.next_state = STATE_WAIT

                if self.state == STATE_PRETRIGGER:
                    self.next_state = STATE_WAIT
                    # loop through channels, see if anyone is not ready for pretrigger
                    # stay in pretrigger state until all channels are ready
                    # TODO: add timeout?
                    # (tuning the USRP, preparing combined rf samples on GPU)
                    for ch in self.channels:
                        if ch.state != STATE_PRETRIGGER:
                            print('STATE_MACHINE: remaining in PRETRIGGER because channel {} state is {}, '.format(ch.cnum, ch.state))
                            self.next_state = STATE_PRETRIGGER

                    # if everyone is ready, prepare for data collection
                    if self.next_state == STATE_WAIT:
                        cprint('running pretrigger!', 'green')
                        self.pretrigger()


                if self.state == STATE_TRIGGER:
                    self.next_state = STATE_WAIT
                    # wait for all channels to be in TRIGGER state
                    for ch in self.channels:
                        if ch.state != STATE_TRIGGER:
                            print('STATE_MACHINE: remaining in TRIGGER because channel {} state is {}, '.format(ch.cnum, ch.state))
                            self.next_state = STATE_TRIGGER

                    # if all channels are TRIGGER, then TRIGGER and return to STATE_WAIT
                    if self.next_state == STATE_WAIT:
                        self.trigger()

                if self.state == STATE_GET_DATA:
                    self.next_state = STATE_WAIT

                    for ch in self.channels:
                        self.get_data(ch)
                   
                if self.state == STATE_RESET:
                    cprint('stuck in STATE_RESET?!', 'yellow')
                    pdb.set_trace()
                    self.next_state = STATE_INIT

                time.sleep(RADAR_STATE_TIME)

        self.client_sock.listen(MAX_CHANNELS)
        client_threads = []
        self.channels = []

        ct = threading.Thread(target=radar_state_machine)
        ct.start()
        while True:
            cprint('waiting for control program', 'green')
            client_conn, addr = self.client_sock.accept()

            cprint('connection from control program, spawning channel handler thread', 'green')
            ct = threading.Thread(target=spawn_channel, args=(client_conn,))
            client_threads.append(ct)
            ct.start()
       
            # remove threads that are not alive
            # TODO: the client_sock.accept is probably blocking, this will only execute after connection of a new control program. is that the intended behavior? -jtk
            client_threads = [iThread for iThread in client_threads if iThread.is_alive()]

        self.client_sock.close()


    # read in ini config files..
    def ini_file_init(self):
        driver_config = configparser.ConfigParser()
        driver_config.read('driver_config.ini')
        self.ini_shm_settings = driver_config['shm_settings']
        self.ini_cuda_settings = driver_config['cuda_settings']
        self.ini_network_settings = driver_config['network_settings']

        usrp_config = configparser.ConfigParser()
        usrp_config.read('usrp_config.ini')
        usrp_configs = []
        for usrp in usrp_config.sections():
            usrp_configs.append(usrp_config[usrp])
        self.ini_usrp_configs = usrp_configs

    def usrp_init(self):
        usrp_drivers = [] # hostname of usrp drivers
        usrp_driver_base_port = int(self.ini_network_settings['USRPDriverPort'])

        for usrp in self.ini_usrp_configs:
            usrp_driver_hostname = usrp['driver_hostname'] 
            usrp_driver_port = int(usrp['array_idx']) + usrp_driver_base_port 
            usrp_drivers.append((usrp_driver_hostname, usrp_driver_port))
	
        usrp_driver_socks = []
        
        # TODO: read these from ini config file
        # currently pulled from radar_config_constants.py
        self.usrp_tx_cfreq = DEFAULT_USRP_CENTER_FREQ
        self.usrp_rx_cfreq = DEFAULT_USRP_CENTER_FREQ 
        self.usrp_rf_tx_rate = int(self.ini_cuda_settings['FSampTX'])
        self.usrp_rf_rx_rate = int(self.ini_cuda_settings['FSampRX'])

        # open each 
        try:
            for usrp in usrp_drivers:
                cprint('connecting to usrp driver on port {}'.format(usrp[1]), 'blue')
                usrpsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                usrpsock.connect(usrp)
                usrp_driver_socks.append(usrpsock)
        except ConnectionRefusedError:
            cprint('USRP server connection failed', 'blue')
            sys.exit(1)

        self.usrpsocks = usrp_driver_socks
        self._resync_usrps()


    def _resync_usrps(self):
        usrps_synced = False

        while not usrps_synced:
            cmd = usrp_sync_time_command(self.usrpsocks)
            cmd.transmit()
            cmd.client_return()

            # once USRPs are connected, synchronize clocks/timers 
            cmd = usrp_get_time_command(self.usrpsocks)
            cmd.transmit() 

            usrptimes = []
            for usrpsock in self.usrpsocks:
                usrptimes.append(cmd.recv_time(usrpsock))
           
            cmd.client_return()
     
            # check if sync succeeded..
            if max(np.abs(np.array(usrptimes) - usrptimes[0])) < .5:
                usrps_synced = True
                print('USRPs synchronized, approximate times: ' + str(usrptimes))
            else:
                # TODO: why does USRP synchronization fail?
                print('USRP syncronization failed, trying again..')
                time.sleep(1)

    @timeit
    def rxfe_init(self):
        # TODO: read rxfe parameters in from config file
        pdb.set_trace()

        cmd = usrp_rxfe_setup_command(self.usrpsocks, 0, 0, 0) 
        cmd.transmit()
        cmd.client_return()

    @timeit
    def cuda_init(self): 
        self.tx_upsample_rate = int(self.ini_cuda_settings['TXUpsampleRate'])

        # connect cuda_driver servers
        cuda_driver_socks = []

        cuda_driver_port = int(self.ini_network_settings['CUDADriverPort'])
        cuda_driver_hostnames = self.ini_network_settings['CUDADriverHostnames'].split(',')

        try:
            for c in cuda_driver_hostnames:
                cudasock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                cudasock.connect((c, cuda_driver_port))
                cuda_driver_socks.append(cudasock)
        except ConnectionRefusedError:
            logging.warnings.warn("cuda server connection failed")
            sys.exit(1)

        self.cudasocks = cuda_driver_socks
    
    @timeit
    def clrfreq(self, channel):
        # run clear frequency search on provided channel chan
        cprint('running clrfreq for channel {}'.format(channel.cnum), 'blue')
        tbeamnum = channel.ctrlprm_struct.get_data('tbeam')
        tbeamwidth = channel.ctrlprm_struct.get_data('tbeamwidth') 
        print('todo: verify that tbeamwidth is 3.24ish')
        pdb.set_trace()
        channel.tfreq, channel.noise = clrfreq_search(channel.clrfreq_struct, self.usrpsocks, self.restricted_frequencies, tbeamnum, tbeamwidth) 
        channel.tfreq /= 1000 # clear frequency search stored in kHz for compatibility with control programs..
    
    @timeit
    def get_data(self, channel): #XXX XXX
        # retrieve data over an integration period for a single channel
        ctrlprm = channel.ctrlprm_struct.payload

        cprint('running get_data', 'blue')
        # stall until all channels are ready
        print('todo: scale number of samples in get_data to an integration period')
        pdb.set_trace() # TODO 

        nbb_samples = ctrlprm['number_of_samples'] * PULSE_SEQUENCES_PER_INTEGRATION_PERIOD

        cprint('sending USRP_READY_DATA command ', 'blue')
        cmd = usrp_ready_data_command(self.usrpsocks)
        cmd.transmit()
        cprint('sending USRP_READY_DATA command sent, waiting on READY_DATA status', 'blue')

        # TODO: seperate tracking of number of antennas in main and back array from nants
        # the following line will fail once we start testing with the back array or test with spare arrays
        main_samples = np.complex64(np.zeros((len(self.usrpsocks), nbb_samples))) # TODO: support sparse array.., this assumes no missing elements
        main_beamformed = np.uint32(np.zeros(nbb_samples)) # samples for control program are packed 16 bit i/q pairs in a 32 bit array..
        back_samples = np.complex64(np.zeros((MAXANTENNAS_BACK, nbb_samples))) # TODO: don't hardcode back array size, support missing elements
        main_beamformed = np.uint32(np.zeros(nbb_samples))

        # check status of usrp drivers
        for usrpsock in self.usrpsocks:
            transmit_dtype(usrpsock, channel.cnum, np.int32) # WHY DO THIS?

            ready_return = cmd.recv_metadata(usrpsock)
            rx_status = ready_return['status']

            cprint('GET_DATA rx status {}'.format(rx_status), 'blue')
            if rx_status != 2:
                cprint('USRP driver status {} in GET_DATA'.format(rx_status), 'blue')
                #status = USRP_DRIVER_ERROR # TODO: understand what is an error here..

        cmd.client_return()
        cprint('GET_DATA: received samples from USRP_DRIVERS, status: ' + str(rx_status), 'blue')

        # DOWNSAMPLING
        cprint('GET_DATA: send CUDA_PROCESS', 'blue')
        cmd = cuda_process_command(self.cudasocks)
        cmd.transmit()
        cmd.client_return()
        cprint('GET_DATA: finished CUDA_PROCESS', 'blue')

        # RECEIVER RX BB SAMPLES 
        cmd = cuda_get_data_command(self.cudasocks)
        cmd.transmit()

        # recv metadata from cuda drivers about antenna numbers and whatnot
        # fill up provided main and back baseband sample buffers
        for cudasock in self.cudasocks:
            transmit_dtype(cudasock, channel.cnum, np.int32)
            nants = recv_dtype(cudasock, np.uint32)

            for ant in range(nants):
                ant = recv_dtype(cudasock, np.uint16)
                num_sample = recv_dtype(cudasock, np.uint32)
                samples = recv_dtype(cudasock, np.float32, num_samples)
                samples = samples[0::2] + 1j * samples[1::2] # unpacked interleaved i/q
                
                if ant < MAX_MAIN_ARRAY_TODO:
                    main_samples[ant] = samples[:]

                else:
                    back_samples[ant - MAX_MAIN_ARRAY_TODO] = samples[:]
                            
            # samples is interleaved I/Q float32 [self.nants, self.nchans, nbbsamps_rx]
        
        # TODO: currently assuming samples is main samples, no back array!
        cmd.client_return()
        
        # BEAMFORMING
        # calculate beam azimuth from transmit beam number 
        bmazm = calc_beam_azm_rad(RADAR_NBEAMS, ctrlprm['tbeam'], RADAR_BEAMWIDTH)

        # calculate antenna-to-antenna phase shift for steering at a frequency
        pshift = calc_phase_increment(bmazm, ctrlprm['tfreq'] * 1000.)

        # TODO: HARDCODED TO ONE ANTENNA
        antennas_list = [0]

        # calculate a complex number representing the phase shift for each antenna
        beamform_main = np.array([rad_to_rect(a * pshift) for a in antennas_list])
        #beamform_back = np.ones(len(MAIN_ANTENNAS))

        cprint('get_data complete!', 'blue')
    
        def _beamform_uhd_samples(samples, phasing_matrix, n_samples, antennas):
            beamformed_samples = np.ones(n_samples)

            for i in range(n_samples):
                itemp = np.int16(0)
                qtemp = np.int16(0)

                for aidx in range(len(antennas)):
                    itemp += np.real(samples[aidx][i]) * np.real(phasing_matrix[aidx]) - \
                             np.imag(samples[aidx][i]) * np.imag(phasing_matrix[aidx])
                    qtemp += np.real(samples[aidx][i]) * np.imag(phasing_matrix[aidx]) + \
                             np.imag(samples[aidx][i]) * np.real(phasing_matrix[aidx])
                beamformed_samples[i] = complex_ui32_pack(itemp, qtemp)

            return beamformed_samples
        
        # apply beamforming to all samples in an integration period for a channel at once
        channel.main_beamformed = _beamform_uhd_samples(main_samples, beamform_main, nbb_samples, antennas_list)
        # next, reshape the sample array to be (number of pulse sequences) by (number of samples per pulse sequence)
        channel.main_beamformed = channel.main_beamformed.reshape(num_pulse_sequences_per_integration_period, nsamples_per_sequence)
        
        #channel.back_beamformed = _beamform_uhd_samples(back_samples, beamform_back, nbb_samples, antennas)
        channel.back_beamformed = np.zeros_like(channel.main_beamformed) # TODO: actually process back samples
        channel.state = STATE_WAIT

    def deleteRadarChannel(self, channelObject):
        if channelObject in self.channels:
            # don't delete channel in middle of trigger, pretrigger, or ....
            channelObject._waitForState(STATE_WAIT) 
            print('RHM:deleteRadarChannel removing channel {} from HardwareManager'.format(self.channels.index(channelObject)))
            self.channels.remove(channelObject)
            print('RHM:deleteRadarChannel {} channels left'.format(len(self.channels)))
        else:
            print('RHM:deleteRadarChannel channel already deleted')

       # if len(self.channels) == 0:
       #     self.exit()

        

    def exit(self):
        # clean up and exit
        self.client_sock.close()
        cmd = usrp_exit_command(self.usrpsocks)
        cmd.transmit()

        cmd = cuda_exit_command(self.cudasocks)
        cmd.transmit()

        for sock in self.usrpsocks:
            sock.close()
        for sock in self.cudasocks:
            sock.close()

        cprint('received exit command, cleaning up..', 'red')

        sys.exit(0)
    
    @timeit
    def trigger(self):
        # trigger the start of an integration period
        cprint('running trigger, triggering usrp drivers', 'yellow')
        
        # so, schedule pulse start times for the integration period..
        if len(self.channels):

            # grab current usrp time from one usrp_driver
            cmd = usrp_get_time_command(self.usrpsocks[0])
            cmd.transmit()
            
            # todo: tag time using a better source? this will have a few hundred microseconds of uncertainty
            # maybe measure offset between usrp time and computer clock time somewhere, then calculate from there
            usrp_integration_period_start_clock_time = time.time() + INTEGRATION_PERIOD_SYNC_TIME

            usrp_time = cmd.recv_time(self.usrpsocks[0])
            cmd.client_return()
          
            # provide sequence times for 
            for channel in self.channels: 
                channel.pulse_start_time_secs = np.zeros(channel.pulse_sequences_per_integration_period, dtype=np.uint64)
                channel.pulse_start_time_usecs = np.zeros(channel.pulse_sequences_per_integration_period, dtype=np.uint32)

                for sidx in range(channel.pulse_sequences_per_integration_period):
                    pulse_start_time = usrp_integration_period_start_clock_time + sidx * self.nsamples_per_sequence
                    channel.pulse_start_time_secs[sidx] = int(pulse_start_time) 
                    channel.pulse_start_time_usecs[sidx] = pulse_start_time - int(pulse_start_time)

            # broadcast the start of the next integration period to all usrp
            trigger_time = usrp_time + INTEGRATION_PERIOD_SYNC_TIME 
            cmd = usrp_trigger_pulse_command(self.usrpsocks, trigger_time)
            cprint('sending trigger pulse command', 'yellow')
            cmd.transmit()
            cprint('waiting for trigger return', 'yellow')
            returns = cmd.client_return()
            cprint('trigger command return!', 'yellow')

            if TRIGGER_BUSY in returns:
                # I'm not aware of any case where we would hit this. set a breakpoint if we make it here..
                cprint('could not trigger, usrp driver is busy', 'yellow')
                pdb.set_trace()


        else:
            cprint('no channels loaded, trigger failed!', 'green')
            pdb.set_trace()

        for ch in self.channels:
            ch.state = STATE_WAIT

        cprint('trigger complete', 'yellow')

    # TODO: Merge channel infomation here!
    # TODO: pretrigger cuda client return fails
    # key error: 0?
    @timeit
    def pretrigger(self):
        cprint('running RadarHardwareManager.pretrigger()', 'blue')
        
        synth_pulse = False
        print('self.channels len : ' + str(len(self.channels)))
        
        for ch in self.channels:
            if ch.ctrlprm_struct.payload['tfreq'] != 0 and ch.update_channel:
                # check that we are actually able to transmit at that frequency given the USRP center frequency and sampling rate
                print('tfreq: ' + str(ch.ctrlprm_struct.payload['tfreq']))
                print('rfreq: ' + str(ch.ctrlprm_struct.payload['rfreq']))
                print('usrp tx cfreq: ' + str(self.usrp_tx_cfreq))
                print('usrp rf tx rate: ' + str(self.usrp_rf_tx_rate))
                
                assert np.abs((ch.ctrlprm_struct.payload['tfreq'] * 1e3) - self.usrp_tx_cfreq) < (self.usrp_rf_tx_rate / 2), 'transmit frequency outside range supported by sampling rate and center frequency'

                if not (ch.ctrlprm_struct.payload['tfreq'] == ch.ctrlprm_struct.payload['rfreq']):
                    cprint('tfreq != rfreq!', 'yellow')
                
                # TODO: check if the sequence has changed?
                cmd = cuda_add_channel_command(self.cudasocks, sequence=ch.getSequence())

                cprint('sending CUDA_ADD_CHANNEL', 'blue')
                cmd.transmit()

                cprint('waiting for CUDA_ADD_CHANNEL to return', 'blue')
                cmd.client_return()
                ch.update_channel = False
                synth_pulse = True


        if synth_pulse:
            cprint('sending CUDA_GENERATE_PULSE', 'blue')
            cmd = cuda_generate_pulse_command(self.cudasocks)
            cmd.transmit()
            cmd.client_return()
            cprint('finished CUDA_GENERATE_PULSE', 'blue')
        
        # calculate relative pulse times over the entire integration period 

        # determine the length of integration periods for all channels in seconds
        integration_periods = [(channel.ctrlprm_struct.payload['scansc'] + channel.ctrlprm_struct.payload['scanus'] / 1e6) for channel in self.channels]

        # for now, all integration periods must be the same length.. check if that is the case
        integration_period = integration_periods[0]
        for p in integration_periods:
            assert p == integration_period, 'integration periods for all channels must be equal.'
        
        # to find out how much time is available in an integration period for pulse sequences, subtract out startup delay
        sampling_duration = integration_period - INTEGRATION_PERIOD_SYNC_TIME 

        # determine length of each pulse sequence and inter-pulse-sequence padding 
        sequence_lengths = [self.channels[0].pulse_sequence_offsets_vector[-1] + channels[0].pulse_lens[-1] for chan in self.channels]
        sequence_length = sequence_length[0]
        for sl in sequence_lengths:
            assert sl = sequence_length, 'pulses must occur at the same time for all channels'

        # calculate the pulse sequence period with padding
        pulse_sequence_period = (PULSE_SEQUENCE_PADDING_TIME + sequence_length)

        # calculate the number of pulse sequences that fit in the available time within an integration period
        num_pulse_sequences_per_integration_period = np.floor(sampling_duration / pulse_sequence_period)
        # inform all channels with the number of pulses per integration period
        for ch in self.channels:
            ch.pulse_sequences_per_integration_period = num_pulse_sequences_per_integration_period

        # find transmit sample rate
        tx_sample_rate = self.channels[0].usrp_tx_sampling_rate

        # then calculate sample indicies at which pulse sequences start within a pulse sequence
        pulse_sequence_offsets_samples = self.channels[0].pulse_sequence_offsets_vector * tx_sample_rate  
        pulse_sequence_offsets_vector = self.channels[0].pulse_sequence_offsets_vector # TODO: verify this is uniform across channels
        nsamples_per_sequence = pulse_sequence_period * tx_sample_rate
        npulses_per_sequence = self.channels[0].npulses_per_sequence # TODO: verify this is uniform across channels

        # then, calculate sample indicies at which pulses start within an integration period
        integration_period_pulse_sample_offsets = np.zeros(npulses_per_sequence * num_pulse_sequences_per_integration_period, dtype=np.uint64)
        for sidx in range(num_pulse_sequences_per_integration_period):
            for pidx in range(npulses_per_sequence):
                integration_period_pulse_sample_offsets[sidx * npulses_per_sequence + pidx] = sidx * nsamples_per_sequence + pulse_sequence_offsets_samples[pidx]

        for ch in self.channels:
            ch.integration_period_pulse_sample_offsets = integration_period_pulse_sample_offsets

        # calculate the number of RF transmit and receive samples 
        ctrlprm = self.channels[0].ctrlprm_struct.payload
        num_requested_rx_samples = np.uint64(np.round((self.usrp_rf_rx_rate) * (ctrlprm['number_of_samples'] / ctrlprm['baseband_samplerate'])))
        tx_time = self.channels[0].tx_time
        num_requested_tx_samples = np.uint64(np.round((self.usrp_rf_tx_rate)  * tx_time / 1e6))

        # calculate pulse length in rf-rate samples
        # padded on either side with the length of pulse.., tx_worker.cpp needs padding because of how it sends transmit pulses.. 
        # currently assumes all channels have the same pulse length, and all pulses within an integration period are the same length
        padded_nsamples_per_pulse = 3 * self.channels[0].pulse_lens[0] * tx_sample_rate

        cmd = usrp_setup_command(self.usrpsocks, self.usrp_tx_cfreq, self.usrp_rx_cfreq, self.usrp_rf_tx_rate, self.usrp_rf_rx_rate, npulses_per_integration_period, num_requested_rx_samples, padded_nsamples_per_pulse, integration_period_pulse_sample_offsets)
        cmd.transmit()
        cmd.client_return()

        for ch in self.channels:
            ch.state = STATE_WAIT

        cprint('end of RadarHardwareManager.pretrigger()', 'blue')

class RadarChannelHandler:
    def __init__(self, conn, parent_RadarHardwareManager):
        self.active = False# TODO: eliminate
        self.ready = False # TODO: eliminate
        self.conn = conn
        self.update_channel = True # flag indicating a new beam or pulse sequence 
        self.parent_RadarHardwareManager = parent_RadarHardwareManager

        self.state = STATE_WAIT

        self.ctrlprm_struct = ctrlprm_struct(self.conn)
        self.seqprm_struct = seqprm_struct(self.conn)
        self.clrfreq_struct = clrfreqprm_struct(self.conn)
        self.rprm_struct = rprm_struct(self.conn)
        self.dataprm_struct = dataprm_struct(self.conn)

        self.clrfreq_start = 0
        self.clrfreq_stop = 0

        # TODO: initialize ctlrprm_struct with site infomation
        #self.ctrlprm_struct.set_data('rbeamwidth', 3)
        #self.ctrlprm_struct.set_data('tbeamwidth', 3)


    def run(self):
        rmsg_handlers = {\
            SET_RADAR_CHAN : self.SetRadarChanHandler,\
            SET_INACTIVE : self.SetInactiveHandler,\
            SET_ACTIVE : self.SetActiveHandler,\
            QUERY_INI_SETTINGS : self.QueryIniSettingsHandler,\
        #   GET_SITE_SETTINGS : self.GetSiteSettingsHandler, \
        #   UPDATE_SITE_SETTINGS : self.UpdateSiteSettingsHandler,\
            GET_PARAMETERS : self.GetParametersHandler,\
            SET_PARAMETERS : self.SetParametersHandler,\
            PING : self.PingHandler,\
        #   OKAY : self.OkayHandler,\
        #   NOOP : self.NoopHandler,\
            QUIT : self.QuitHandler,\
            REGISTER_SEQ: self.RegisterSeqHandler,\
        #   REMOVE_SEQ: self.RemoveSeqHandler,\
            REQUEST_ASSIGNED_FREQ: self.RequestAssignedFreqHandler,\
            REQUEST_CLEAR_FREQ_SEARCH: self.RequestClearFreqSearchHandler,\
            LINK_RADAR_CHAN : self.LinkRadarChanHandler,\
            SET_READY_FLAG: self.SetReadyFlagHandler,\
            UNSET_READY_FLAG: self.UnsetReadyFlagHandler,\
        #   SET_PROCESSING_FLAG: self.SetProcessingFlagHandler,\
        #   UNSET_PROCESSING_FLAG: self.UnsetProcessingFlagHandler,\
        #   WAIT_FOR_DATA: self.WaitForDataHandler,\
            GET_DATA: self.GetDataHandler}


        while True:
            rmsg = rosmsg_command(self.conn)
            status = RMSG_FAILURE

            print('waiting for command')
            rmsg.receive(self.conn)
            command = chr(rmsg.payload['type'] & 0xFF) # for some reason, libtst is sending out 4 byte commands with junk..
            try:
                cprint('received command (ROS=>USRP_Server): {}, {}'.format(command, RMSG_COMMAND_NAMES[command]), 'red')
            except KeyError:
                print('unrecognized command!')
                print(rmsg.payload)
                pdb.set_trace()

            if command in rmsg_handlers:
                status = rmsg_handlers[command](rmsg)
            else:
                status = self.DefaultHandler(rmsg)

            if status == 'exit': # output of QuitHandler
                break
 
            rmsg.set_data('status', status)
            rmsg.set_data('type', rmsg.payload['type'])
            rmsg.transmit()


    def close(self):
        print('todo: write close..:')
        pdb.set_trace()
        # TODO write this..


    # busy wait until state enters desired state
    # useful for waiting for
    def _waitForState(self, state):
        wait_start = time.time()

        while self.state != state:
            #cprint('channel {} waiting for {}, current state:'.format(self.cnum, self.state, state), 'green')
            time.sleep(RADAR_STATE_TIME)
            if time.time() - wait_start > CHANNEL_STATE_TIMEOUT:
                print('CHANNEL STATE TIMEOUT')
                pdb.set_trace()
                break

    # return a sequence object, used for passing pulse sequence and channel infomation over to the CUDA driver
    def getSequence(self):
        print('TODO: move sequence object over to per-integration-period')
        print('TODO: move phase mask over to singular..')
        pdb.set_trace()
        return sequence(self.npulses, self.tr_to_pulse_delay, self.pulse_offsets_vector, self.pulse_lens, self.phase_masks, self.pulse_masks, self.ctrlprm_struct.payload)

    def DefaultHandler(self, rmsg):
        print("Unexpected command: {}".format(chr(rmsg.payload['type'])))
        pdb.set_trace()
        return RMSG_FAILURE

    def QuitHandler(self, rmsg):
        # TODO: close down stuff cleanly
        #rmsg.set_data('status', RMSG_FAILURE)
        #rmsg.set_data('type', rmsg.payload['type'])
        #rmsg.transmit()
        self.conn.close()
        self.parent_RadarHardwareManager.deleteRadarChannel(self)
        del self # TODO close thread ?!?
        print('Deleted channel')
        # sys.exit() # TODO: set return value
        return 'exit'

    def PingHandler(self, rmsg):
        return RMSG_SUCCESS
    
    @timeit
    def RequestAssignedFreqHandler(self, rmsg):
        # wait for clear frequency search to end, hardware manager will set channel state to WAIT
        self._waitForState(STATE_WAIT) 

        transmit_dtype(self.conn, self.tfreq, np.int32)
        transmit_dtype(self.conn, self.noise, np.float32)

        self.clrfreq_stop = time.time()
        cprint('clr frequency search time: {}'.format(self.clrfreq_stop - self.clrfreq_start), 'yellow')
        return RMSG_SUCCESS

    @timeit
    def RequestClearFreqSearchHandler(self, rmsg):
        self._waitForState(STATE_WAIT)
        self.clrfreq_struct.receive(self.conn)
        self.clrfreq_start = time.time()

        # request clear frequency search time from hardware manager
        self.state = STATE_CLR_FREQ

        return RMSG_SUCCESS

    def UnsetReadyFlagHandler(self, rmsg):
        self.ready = False
        return RMSG_SUCCESS


    def SetReadyFlagHandler(self, rmsg):
        self._waitForState(STATE_WAIT)
        transmit_dtype(self.conn, self.pulse_sequences_per_integration_period, np.uint32)
        self.state = STATE_TRIGGER
        # send trigger command
        self.ready = True
        return RMSG_SUCCESS
    
    @timeit
    def RegisterSeqHandler(self, rmsg):
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
        # site libraries appear to not initialize the status, so a nonzero status here is normall.

        self.seqprm_struct.receive(self.conn)
        self.seq_rep = recv_dtype(self.conn, np.uint8, self.seqprm_struct.payload['len'])
        self.seq_code = recv_dtype(self.conn, np.uint8, self.seqprm_struct.payload['len'])

        tx_tsg_idx = self.seqprm_struct.get_data('index')
        tx_tsg_len = self.seqprm_struct.get_data('len')
        tx_tsg_step = self.seqprm_struct.get_data('step')
        
        # ratio between tsg step (units of microseconds) to baseband sampling period
        # TODO: calculate this from TXUpsampleRate, FSampTX in cuda_config.ini
        # however, it should always be 1..
        tsg_bb_per_step = 1

        # psuedo-run length encoded tsg
        tx_tsg_rep = self.seq_rep
        tx_tsg_code = self.seq_code

        seq_buf = []
        for i in range(tx_tsg_len):
            for j in range(0, np.int32(tsg_bb_per_step * tx_tsg_rep[i])):
                seq_buf.append(tx_tsg_code[i])
        seq_buf = np.uint8(np.array(seq_buf))

        # extract out pulse information...
        S_BIT = np.uint8(0x01) # sample impulses
        R_BIT = np.uint8(0x02) # tr gate, use for tx pulse times
        X_BIT = np.uint8(0x04) # transmit path, use for bb
        A_BIT = np.uint8(0x08) # enable attenuator
        P_BIT = np.uint8(0x10) # phase code (BPSK)

        # create masks
        samples = seq_buf & S_BIT
        tr_window = seq_buf & R_BIT
        rf_pulse = seq_buf & X_BIT
        atten = seq_buf & A_BIT
        phase_mask = seq_buf & P_BIT

        # extract and number of samples
        sample_idx = np.nonzero(samples)[0]
        assert len(sample_idx) > 3, 'register empty sequence'

        nbb_samples = len(sample_idx)

        # extract pulse start timing
        tr_window_idx = np.nonzero(tr_window)[0]
        tr_rising_edge_idx = _rising_edge_idx(tr_window_idx)
        pulse_sequence_offsets_vector = tr_rising_edge_idx * tx_tsg_step

        # extract tr window to rf pulse delay
        rf_pulse_idx = np.nonzero(rf_pulse)[0]
        rf_pulse_edge_idx = _rising_edge_idx(rf_pulse_idx)
        tr_to_pulse_delay = (rf_pulse_edge_idx[0] - tr_rising_edge_idx[0]) * tx_tsg_step
        npulses_per_sequence = len(rf_pulse_edge_idx)

        # extract per-pulse phase coding and transmit pulse masks
        # indexes are in microseconds from start of pulse
        phase_masks = []
        pulse_masks = []
        pulse_lens = []

        for i in range(npulses_per_sequence):
            pstart = rf_pulse_edge_idx[i]
            pend = pstart + _pulse_len(rf_pulse, pstart)
            phase_masks.append(phase_mask[pstart:pend])
            pulse_masks.append(rf_pulse[pstart:pend])
            pulse_lens.append((pend - pstart) * tx_tsg_step)

        self.npulses_per_sequence = npulses_per_sequence
        self.pulse_sequence_offsets_vector = pulse_sequence_offsets_vector / 1e6
        self.pulse_lens = pulse_lens # length of pulses, in seconds
        self.phase_masks = phase_masks # phase masks are complex number to multiply phase by, so
        self.pulse_masks = pulse_masks
        self.tr_to_pulse_delay = tr_to_pulse_delay
        # self.ready = True # TODO: what is ready flag for?
        self.tx_time = self.pulse_lens[0] + 2 * self.tr_to_pulse_delay

        # calculate the number of possible pulse sequences per integration period
        self.


        if debug:
           print("pulse0 length: {} us, tr_pulse_delay: {} us, tx_time: {} us".format(self.pulse_lens[0], tr_to_pulse_delay, self.tx_time))
        if npulses_per_sequence == 0:
            raise ValueError('number of pulses must be greater than zero!')
        if nbb_samples == 0:
            raise ValueError('number of samples in sequence must be nonzero!')

      #  pdb.set_trace()

        return RMSG_SUCCESS
    
    # receive a ctrlprm struct
    @timeit
    def SetParametersHandler(self, rmsg):
        self._waitForState(STATE_WAIT)
        cprint('channel {} starting PRETRIGGER'.format(self.cnum), 'green')
        # check if the sequence is new before diving into PRETRIGGER state?
        
        ctrlprm_old = copy.deepcopy(self.ctrlprm_struct.payload)
        self.ctrlprm_struct.receive(self.conn)

        # if we're not due for an update already, check and see if we need to update the transmit pulse sequence
        if not self.update_channel:
            for key in ctrlprm_old.keys():
                if key != '': # TODO: what is the blank key doing in the ctrlprm?..
                    if ctrlprm_old[key] != self.ctrlprm_struct.payload[key]:
                        self.update_channel = True
                        break
        print('usrp_server RadarChannelHandler.SetParametersHandler: tfreq: ' + str(self.ctrlprm_struct.payload['tfreq']))
        print('usrp_server RadarChannelHandler.SetParametersHandler: rfreq: ' + str(self.ctrlprm_struct.payload['rfreq']))

        self.state = STATE_PRETRIGGER
        cprint('channel {} going into PRETRIGGER state'.format(self.cnum), 'green')
        # TODO for a SetParametersHandler, prepare transmit and receive sample buffers
        if (self.rnum < 0 or self.cnum < 0):
            return RMSG_FAILURE
        return RMSG_SUCCESS

    # send ctrlprm struct
    @timeit
    def GetParametersHandler(self, rmsg):
        # TODO: return bad status if negative radar or channel
        self.ctrlprm_struct.transmit()
        return RMSG_SUCCESS
    
    @timeit
    def GetDataHandler(self, rmsg):
        cprint('entering hardware manager get data handler', 'blue')
        self.dataprm_struct.set_data('samples', self.ctrlprm_struct.payload['number_of_samples'])

        self.dataprm_struct.transmit()
        cprint('sending dprm struct', 'green')

        if not self.active or self.rnum < 0 or self.cnum < 0:
            pdb.set_trace()
            return RMSG_FAILURE

        cprint('waiting for channel to idle before GET_DATA', 'blue')
        self._waitForState(STATE_WAIT)

        cprint('entering GET_DATA state', 'blue')
        self.state = STATE_GET_DATA

        cprint('waiting to return to WAIT before returning samples', 'blue')
        self._waitForState(STATE_WAIT)

        cprint('GET_DATA complete, returning samples', 'blue')
        # interact with site library's SiteIntegrate loop
        # send back samples and metadata for each pulse sequence 
        for sidx in range(num_pulse_sequences_per_integration_period):
            cprint('GET_DATA returning samples from pulse {}'.format(sidx), 'blue')

            transmit_dtype(self.conn, sidx, np.uint32)
            transmit_dtype(self.conn, sequence_length_samples, np.uint32)

            transmit_dtype(self.conn, self.pulse_start_time_secs[sidx], np.uint64)
            transmit_dtype(self.conn, self.pulse_start_time_usecs[sidx], np.uint32)

            transmit_dtype(self.conn, self.main_beamformed[sidx], np.uint32)
            transmit_dtype(self.conn, self.back_beamformed[sidx], np.uint32)

            pdb.set_trace()
            print('TODO: verify that this is working properly with the site library..')

        # currently assuming pulse sequences are uniform within an integration period
        badtrdat_start_usec = self.pulse_sequence_offsets_vector * 1e6 # convert to us
        transmit_dtype(self.conn, self.npulses_per_sequence, np.uint32)
        transmit_dtype(self.conn, badtrdat_start_usec, np.uint32) # length badtrdat_len
        transmit_dtype(self.conn, self.pulse_lens, np.uint32) # length badtrdat_len

        # stuff these with junk, they don't seem to be used..
        num_transmitters = 16 
        txstatus_agc = np.zeros(num_transmitters)
        txstatus_lowpwr = np.zeros(num_transmitters)

        transmit_dtype(self.conn, num_transmitters, np.int32)
        transmit_dtype(self.conn, txstatus_agc, np.int32) # length num_transmitters
        transmit_dtype(self.conn, txstatus_lowpwr, np.int32) # length num_transmitters

        return RMSG_SUCCESS
    
    @timeit
    def SetRadarChanHandler(self, rmsg):
        self.rnum = recv_dtype(self.conn, np.int32)
        self.cnum = recv_dtype(self.conn, np.int32)
        
        self.ctrlprm_struct.set_data('channel', self.cnum)
        self.ctrlprm_struct.set_data('radar',  self.rnum)

        # TODO: how to handle channel contention?
        if(debug):
            print('radar num: {}, radar chan: {}'.format(self.rnum, self.cnum))

        # TODO: set RMSG_FAILURE if radar channel is unavailable
        # rmsg.set_data('status', RMSG_FAILURE)
        return RMSG_SUCCESS

    def LinkRadarChanHandler(self, rmsg):
        rnum = recv_dtype(self.conn, np.int32)
        cnum = recv_dtype(self.conn, np.int32)
        print('link radar chan is unimplemented!')
        pdb.set_trace()
        return RMSG_SUCCESS


    def QueryIniSettingsHandler(self, rmsg):
        # TODO: don't hardcode this if I find anything other than ifmode querying..
        data_length = recv_dtype(self.conn, np.int32)
        ini_name = recv_dtype(self.conn, str, nitems=data_length)
        requested_type = recv_dtype(self.conn, np.uint8)

        # hardcode to reply with ifmode is false
        assert ini_name == b'site_settings:ifmode\x00'

        payload = 0 # assume always false

        transmit_dtype(self.conn, requested_type, np.uint8)
        transmit_dtype(self.conn, data_length, np.int32) # appears to be unused by site library
        transmit_dtype(self.conn, payload, np.int32)

        return 1 # TODO: Why does the ini handler expect a nonzero response for success?

    def SetActiveHandler(self, rmsg):
        # TODO: return failure if rnum and cnum are not set
        # TODO: why is active only set if it is already set?
        if not self.active:
            self.active = True
            self.ready = False
        return RMSG_SUCCESS

    def SetInactiveHandler(self, rmsg):
        # TODO: return failure status if the radar or channel number is invalid

        if not self.active:
            self.active = False
            self.ready = False
            # TODO: what is coordination handler doing?

        return RMSG_SUCCESS


def main():
    # maybe switch to multiprocessing with manager process
    print('main!')
    logging.basicConfig(filename='client_server.log', level=logging.DEBUG, format='[%(levelname)s] (%(threadName)-10s) %(asctime)s %(message)s',)

    logging.debug('main()')
    rmsg_port = 45000

    radar = RadarHardwareManager(rmsg_port)
    radar.run()


if __name__ == '__main__':
    main()

