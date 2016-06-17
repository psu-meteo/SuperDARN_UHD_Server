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
from termcolor import cprint
from phasing_utils import *
from socket_utils import *
from rosmsg import *
from drivermsg_library import *

MAX_CHANNELS = 10
RMSG_FAILURE = -1
RMSG_SUCCESS = 0
RADAR_STATE_TIME = .0001#.0005
CHANNEL_STATE_TIMEOUT = 120000

debug = True

# TODO: pull these from config file
RADAR_NBEAMS = 16
ANTENNA = 1
STATE_INIT = 'INIT'
STATE_RESET = 'RESET'
STATE_WAIT = 'WAIT'
STATE_PRETRIGGER = 'PRETRIGGER'
STATE_TRIGGER = 'TRIGGER'
STATE_CLR_FREQ = 'CLR_FREQ_WAIT'
STATE_GET_DATA = 'GET_DATA'

TRIGGER_BUSY = 'b'

# TODO: move in code from usrp_server_uhd
# handle arbitration with multiple channels accessing the usrp hardware
# track state of a grouping of usrps
# merge information from multiple control programs, handle disparate settings
# e.g, ready flags and whatnot
class RadarHardwareManager:
    def __init__(self, port):
        self.client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client_sock.bind(('localhost', port))
        self.usrp_init()
        #self.rxfe_init()
        self.cuda_init()

    def run(self):
        def spawn_channel(conn):
            # start new radar channel handler
            channel = RadarChannelHandler(conn)
            self.channels.append(channel)
            channel.run()
            conn.close()

        # TODO: set state machine args
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

                    # TODO: set radar state priority
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
                    # TODO: add timeout
                    # (tuning the USRP, preparing combined rf samples on GPU)
                    for ch in self.channels:
                        if ch.state != STATE_PRETRIGGER:
                            print('channel {} state is {}, remaining in PRETRIGGER'.format(ch.cnum, ch.state))
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
                            print('channel state is {}, remaining in TRIGGER'.format(ch.state))
                            self.next_state = STATE_TRIGGER

                    # if all channels are TRIGGER, then TRIGGER and return to STATE_WAIT
                    if self.next_state == STATE_WAIT:
                        self.trigger()

                if self.state == STATE_GET_DATA:
                    self.next_state = STATE_WAIT

                    cmd = cuda_process_command(self.cudasocks)
                    cmd.transmit()
                    cmd.client_return()

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

        # TODO: write state machine..
        ct = threading.Thread(target=radar_state_machine)
        ct.start()
        while True:
            client_conn, addr = self.client_sock.accept()

            ct = threading.Thread(target=spawn_channel, args=(client_conn,))
            client_threads.append(ct)
            ct.start()

        self.client_sock.close()



    def usrp_init(self):
        usrp_drivers = ['localhost']
        usrp_driver_socks = []
        try:
            for d in usrp_drivers:
                usrpsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                usrpsock.connect((d, USRPDRIVER_PORT + ANTENNA))
                usrp_driver_socks.append(usrpsock)
        except ConnectionRefusedError:
            cprint('USRP server connection failed', 'blue')
            sys.exit(1)

        self.usrpsocks = usrp_driver_socks

    def rxfe_init(self):
        # TODO: fix this function
        pdb.set_trace()
        '''
        # TODO: read in rf_setting struct?
        amp0 = rf_settings[1] # amp1 in RXFESettings struct
        amp1 = rf_settings[2] # amp2 in RXFESettings struct
        att_p5dB = np.uint8((rf_settings[4] > 0))
        att_1dB = np.uint8((rf_settings[5] > 0))
        att_2dB = np.uint8((rf_settings[6] > 0))
        att_4dB = np.uint8((rf_settings[7] > 0))
        att = (att_p5dB) | (att_1dB << 1) | (att_2dB << 2) | (att_4dB << 3)
        '''
        cmd = usrp_rxfe_setup_command(self.usrpsocks, 0, 0, 0) #amp0, amp1, att)
        cmd.transmit()
        cmd.client_return()


    def cuda_init(self):
        time.sleep(.05)
        # connect cuda_driver servers
        cuda_drivers = ['localhost']
        cuda_driver_socks = []

        try:
            for c in cuda_drivers:
                cudasock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                cudasock.connect((c, CUDADRIVER_PORT))
                cuda_driver_socks.append(cudasock)
        except ConnectionRefusedError:
            logging.warnings.warn("cuda server connection failed")
            sys.exit(1)

        self.cudasocks = cuda_driver_socks


    def setup(self, chan):
        pass

    def clrfreq(self, chan):
        cprint('running clrfreq for channel {}'.format(chan.cnum), 'blue')
        # TODO: check if radar is free
        MIN_CLRFREQ_DELAY = .1 # TODO: lower this?
        MAX_CLRFREQ_AVERAGE = 10
        MAX_CLRFREQ_BANDWIDTH = 512
        MAX_CLRFREQ_USABLE_BANDWIDTH = 300
        CLRFREQ_RES = 1e3 # fft frequency resolution in kHz

        fstart = chan.clrfreq_struct.payload['start']
        fstop = chan.clrfreq_struct.payload['end']
        filter_bandwidth = chan.clrfreq_struct.payload['filter_bandwidth'] # kHz (c/(2 * rsep))
        power_threshold = chan.clrfreq_struct.payload['pwr_threshold'] # (typically .9, threshold before changing freq)
        nave = chan.clrfreq_struct.payload['nave']

        usable_bandwidth = fstop - fstart
        usable_bandwidth = np.floor(usable_bandwidth/2.0) * 2 # mimic behavior of gc316 driver, why does the old code do this?
        cfreq = (fstart + (fstop-fstart/2.0)) # calculate center frequency of clrfreq search in KHz
        assert usable_bandwidth > 0, "usable bandwidth for clear frequency search must be greater than 0"

        # mimic behavior of gc316 drivers, if requested search bandwidth is broader than 1024 KHz, force it to 512 KHz?
        # calculate the number of points in the FFT
        num_clrfreq_samples = np.int32(pow(2, np.ceil(np.log10(1.25*usable_bandwidth)/np.log10(2))))
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
        tbeamwidth = chan.ctrlprm_struct.payload['tbeamwidth']
        tbeam = chan.ctrlprm_struct.payload['tbeam']
        bmazm = calc_beam_azm_rad(RADAR_NBEAMS, tbeam, tbeamwidth)
        pshift = calc_phase_increment(bmazm, cfreq)
        clrfreq_rate = 2.0 * chan.clrfreq_struct.payload['filter_bandwidth'] * 1000 # twice the filter bandwidth, convert from kHz to Hz
        pwr2 = np.zeros(num_clrfreq_samples)
        combined_samples = np.zeros(num_clrfreq_samples, dtype=np.complex128)

        cprint('gathering samples for clrfreq on beam {} at {}'.format(tbeam, fstart), 'green')
        for ai in range(nave):
            cprint('gathering samples on avg {}'.format(ai), 'green')

            # gather current UHD time
            gettime_cmd = usrp_get_time_command(self.usrpsocks)
            gettime_cmd.transmit()
            usrptimes = []
            for usrpsock in self.usrpsocks:
                usrptimes.append(gettime_cmd.recv_time(usrpsock))

            gettime_cmd.client_return()

            # schedule clear frequency search in MIN_CLRFREQ_DELAY seconds
            clrfreq_time = np.max(usrptimes) + MIN_CLRFREQ_DELAY

            # request clrfreq sample dump
            # TODO: what is the clear frequency rate? (c / (2 * rsep?))

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
                    ant_rotation = rad_to_rect(ant * pshift)
                    samples = recv_dtype(usrpsock, np.int16, 2 * num_clrfreq_samples)
                    samples = samples[0::2] + 1j * samples[1::2]
                    combined_samples += ant_rotation * samples
            fft_time_start = time.time()

            # return fft of width usable_bandwidth, kHz resolution
            # TODO: fft recentering, etc
            c_fft = np.fft.fft(combined_samples)
            # compute power..
            pc_fft = (np.real(c_fft) ** 2) + (np.imag(c_fft) ** 2) / (num_clrfreq_samples ** 2)
            pwr2 += pc_fft

            fft_time_end = time.time()
            cprint('fft time: {}'.format(fft_time_end - fft_time_start), 'yellow')
            clrfreq_cmd.client_return()
            # todo: convert fft to power spectrum

        pwr2 /= nave

        #pwr = pwr2[sideband-1:-sideband]

        cprint('clrfreq complete for channel {}'.format(chan.cnum), 'blue')
        #self.tfreq = self.clrfreq_struct.payload['start'] + (self.clrfreq_struct.payload['start'] + self.clrfreq_struct.payload['end']) / 2.0
        self.noise = 100

    def get_data(self, channel):
        # TODO: parse from config files or structs
        MAXANTENNAS_BACK = 10
        MAXANTENNAS_MAIN = 40
        RADAR_BEAMWIDTH = 3.24
        RADAR_NBEAMS = 20



        status = True # TODO: what should this be..

        ctrlprm = channel.ctrlprm_struct.payload

        cprint('running get_data', 'blue')
        # stall until all channels are ready
        nbb_samples = ctrlprm['number_of_samples']

        cprint('sending GET_DATA command ', 'blue')
        cmd = usrp_ready_data_command(self.usrpsocks)
        cmd.transmit()


        cprint('sending GET_DATA command sent, waiting on GET_DATA status', 'blue')
        # TODO: iseq += 1
        # TODO: form main samples
        main_samples = np.complex64(np.zeros((MAXANTENNAS_MAIN, nbb_samples)))
        main_beamformed = np.uint32(np.zeros(nbb_samples))
        back_samples = np.complex64(np.zeros((MAXANTENNAS_BACK, nbb_samples)))
        main_beamformed = np.uint32(np.zeros(nbb_samples))

        # check status of usrp drivers
        for usrpsock in self.usrpsocks:
            ready_return = cmd.recv_metadata(usrpsock)
            rx_status = ready_return['status']

            cprint('GET_DATA rx status {}'.format(rx_status), 'blue')
            if rx_status != 1:
                cprint('USRP driver status {} in GET_DATA'.format(rx_status), 'blue')
                #status = USRP_DRIVER_ERROR # TODO: understand what is an error here..

        cmd.client_return()
        channel.state = STATE_WAIT

        cprint('USRP_SERVER GET_DATA: received samples from USRP_DRIVERS: ' + str(status), 'blue')

        cmd = cuda_get_data_command(self.cudasocks)
        cmd.transmit()

        # TODO: setup through all sockets
        # recv metadata from cuda drivers about antenna numbers and whatnot
        # fill up provided main and back baseband sample buffers

        # TODO: fill in sample buffer with baseband sample array
        # TODO: refactor this into drivermsg library
        for cudasock in self.cudasocks:
            num_samples = recv_dtype(cudasock, np.uint32)
            samples = recv_dtype(cudasock, np.float32, num_samples)

            #samples is np.float32(np.zeros([self.nants, self.nchans, nbbsamps_rx]))


        #cmd.populate_samplebuffer(main_samples, back_samples)

        cmd.client_return()
        # TODO: move samples from shared memory
        #bmazm = calc_beam_azm_rad(RADAR_NBEAMS, ctrlprm['tbeam'], RADAR_BEAMWIDTH)

        # calculate antenna-to-antenna phase shift for steering at a frequency
        #pshift = calc_phase_increment(bmazm, ctrlprm['tfreq'] * 1000.)

        # calculate a complex number representing the phase shift for each antenna
        #beamform_main = np.array([rad_to_rect(a * pshift) for a in self.antennas])
        #beamform_back = np.ones(len(MAIN_ANTENNAS))

        cprint('get_data complete!', 'blue')

        def _beamform_uhd_samples(samples, phasing_matrix, n_samples, antennas):
            beamformed_samples = np.ones(len(antennas))

            for i in range(n_samples):
                itemp = np.int16(0)
                qtemp = np.int16(0)

                for ant in range(antennas):
                    itemp += np.real(beamformed_samples[ant][i]) * np.real(phasing_matrix[ant]) - \
                             np.imag(beamformed_samples[ant][i]) * np.imag(phasing_matrix[ant])
                    qtemp += np.real(beamformed_samples[ant][i]) * np.imag(phasing_matrix[ant]) + \
                             np.imag(beamformed_samples[ant][i]) * np.real(phasing_matrix[ant])
                beamformed_samples[i] = complex_ui32_pack(itemp, qtemp)

            return beamformed_samples

        #main_beamformed = _beamform_uhd_samples(main_samples, beamform_main, nbb_samples, antennas)
        #back_beamformed = _beamform_uhd_samples(back_samples, beamform_back, nbb_samples, antennas)



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

    def trigger(self):
        cprint('running trigger, triggering usrp drivers', 'yellow')

        if len(self.channels):
            cmd = usrp_trigger_pulse_command(self.usrpsocks)
            cprint('sending trigger pulse command', 'yellow')
            cmd.transmit()
            # TODO: handle multiple usrps with trigger..
            cprint('waiting for trigger return', 'yellow')
            returns = cmd.client_return()

            if TRIGGER_BUSY in returns:
                cprint('could not trigger, usrp driver is busy', 'yellow')
                pdb.set_trace()

            cprint('trigger return!', 'yellow')

        else:
            cprint('no channels loaded, trigger failed!', 'green')
            pdb.set_trace()

        for ch in self.channels:
            ch.state = STATE_WAIT

        cprint('trigger complete', 'yellow')

    # TODO: Merge channel infomation here!
    # TODO: pretrigger cuda client return fails
    # key error: 0?
    def pretrigger(self):
        cprint('running pretrigger', 'blue')

        # TODO: handle channels with different pulse infomation..
        rfrate = 2e6
        # TODO: parse tx sample rate dynamocially
        # TODO: handle pretrigger with no channels
        # rfrate = np.int32(self.cuda_config['FSampTX'])
        # extract sampling info from cuda driver
        # print('rf rate parsed from ini: ' + str(rfrate))

        pulse_offsets_vector = self.channels[0].pulse_offsets_vector # TODO: merge pulses from multiple channels
        npulses = self.channels[0].npulses # TODO: merge..
        ctrlprm = self.channels[0].ctrlprm_struct.payload

        txfreq = 10e6 # TODO: read from config file
        rxfreq = 10e6 # TODO...
        txrate = 2e6
        rxrate = 2e6

        # TODO: calculate the number of RF transmit samples per-pulse
        num_requested_rx_samples = np.uint64(np.round((rfrate) * (ctrlprm['number_of_samples'] / ctrlprm['baseband_samplerate'])))
        tx_time = self.channels[0].tx_time
        num_requested_tx_samples = np.uint64(np.round((rfrate)  * tx_time / 1e6))

        cmd = usrp_setup_command(self.usrpsocks, txfreq, rxfreq, txrate, rxrate, npulses, num_requested_rx_samples, num_requested_tx_samples, pulse_offsets_vector)
        cmd.transmit()
        cmd.client_return()
        cprint('running cuda_pulse_init command', 'blue')

# TODO: untangle cuda pulse init handler
#        pdb.set_trace()
#        cmd = cuda_pulse_init_command(self.cudasocks)
#        cmd.transmit()
#        cprint('waiting for cuda driver return', 'blue')
#        cmd.client_return()
#        cprint('cuda return received', 'blue')

        synth_pulse = False

        for ch in self.channels:
            ch.state = STATE_WAIT
            #pdb.set_trace()
            if ch.ctrlprm_struct.payload['tfreq'] != 0:
                # load sequence to cuda driver if it has a nonzero transmit frequency..
                time.sleep(.05) # TODO: investigate race condition.. why does adding a sleep here help
                if not (ch.ctrlprm_struct.payload['tfreq'] == ch.ctrlprm_struct.payload['rfreq']):
                    cprint('tfreq != rfreq!', 'yellow')
                    pdb.set_trace()
                cmd = cuda_add_channel_command(self.cudasocks, sequence=ch.getSequence())
                # TODO: separate loading sequences from generating baseband samples..

                cprint('transmitting cuda channel handler', 'blue')
                cmd.transmit()

                cprint('waiting for cuda channel handler return', 'blue')
                cmd.client_return()
                synth_pulse = True

        if synth_pulse:
            cprint('transmitting generate pulse command', 'blue')
            cmd = cuda_generate_pulse_command(self.cudasocks)
            cmd.transmit()

            cmd.client_return()

            cprint('pulse generated, end of pretrigger', 'blue')


class RadarChannelHandler:
    def __init__(self, conn):
        self.active = False# TODO: eliminate
        self.ready = False # TODO: eliminate
        self.conn = conn

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
                cprint('received command: {}, {}'.format(command, RMSG_COMMAND_NAMES[command]), 'red')
            except KeyError:
                print('unrecognized command!')
                print(rmsg.payload)
                pdb.set_trace()

            if command in rmsg_handlers:
                status = rmsg_handlers[command](rmsg)
            else:
                status = self.DefaultHandler(rmsg)

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
        return sequence(self.npulses, self.tr_to_pulse_delay, self.pulse_offsets_vector, self.pulse_lens, self.phase_masks, self.pulse_masks, self.ctrlprm_struct.payload)

    def DefaultHandler(self, rmsg):
        print("Unexpected command: {}".format(chr(rmsg.payload['type'])))
        pdb.set_trace()
        return RMSG_FAILURE

    def QuitHandler(self, rmsg):
        # TODO: close down stuff cleanly
        rmsg.set_data('status', RMSG_FAILURE)
        rmsg.set_data('type', rmsg.payload['type'])
        rmsg.transmit()
        self.conn.close()
        sys.exit() # TODO: set return value

    def PingHandler(self, rmsg):
        return RMSG_SUCCESS

    def RequestAssignedFreqHandler(self, rmsg):
        self._waitForState(STATE_WAIT)

        transmit_dtype(self.conn, self.tfreq, np.int32)
        transmit_dtype(self.conn, self.noise, np.float32)

        self.clrfreq_stop = time.time()
        cprint('clr frequency search time: {}'.format(self.clrfreq_stop - self.clrfreq_start), 'yellow')
        return RMSG_SUCCESS

    def RequestClearFreqSearchHandler(self, rmsg):
        self._waitForState(STATE_WAIT)

        # start clear frequency search
        self.clrfreq_struct.receive(self.conn)
        self.tfreq = self.clrfreq_struct.payload['start'] + (self.clrfreq_struct.payload['start'] + self.clrfreq_struct.payload['end']) / 2.0
        self.noise = 10
        self.state = STATE_CLR_FREQ
        self.clrfreq_start = time.time()
        return RMSG_SUCCESS

    def UnsetReadyFlagHandler(self, rmsg):
        self.ready = False
        return RMSG_SUCCESS


    def SetReadyFlagHandler(self, rmsg):
        self._waitForState(STATE_WAIT)
        # TODO: check if samples are ready?
        self.state = STATE_TRIGGER
        # send trigger command
        self.ready = True
        return RMSG_SUCCESS

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


        # psuedo-run length encoded tsg
        tx_tsg_rep = self.seq_rep
        tx_tsg_code = self.seq_code

        seq_buf = []
        for i in range(tx_tsg_len):
            for j in range(0, np.int32(tx_tsg_step * tx_tsg_rep[i])):
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
        if len(sample_idx) < 3:
            logging.warnings.warn("Warning, cannot register empty sequence")
            return

        nbb_samples = len(sample_idx)

        # extract pulse start timing
        tr_window_idx = np.nonzero(tr_window)[0]
        tr_rising_edge_idx = _rising_edge_idx(tr_window_idx)
        pulse_offsets_vector = tr_rising_edge_idx

        # extract tr window to rf pulse delay
        rf_pulse_idx = np.nonzero(rf_pulse)[0]
        rf_pulse_edge_idx = _rising_edge_idx(rf_pulse_idx)
        tr_to_pulse_delay = (rf_pulse_edge_idx[0] - tr_rising_edge_idx[0])
        npulses = len(rf_pulse_edge_idx)

        # extract per-pulse phase coding and transmit pulse masks
        # indexes are in microseconds from start of pulse
        phase_masks = []
        pulse_masks = []
        pulse_lens = []
        for i in range(npulses):
            pstart = rf_pulse_edge_idx[i]
            pend = pstart + _pulse_len(rf_pulse, pstart)
            phase_masks.append(phase_mask[pstart:pend])
            pulse_masks.append(rf_pulse[pstart:pend])
            pulse_lens.append((pend - pstart))

        self.npulses = npulses
        self.pulse_offsets_vector = pulse_offsets_vector / 1e6
        self.pulse_lens = pulse_lens # length of pulses, in seconds
        self.phase_masks = phase_masks # phase masks are complex number to multiply phase by, so
        self.pulse_masks = pulse_masks
        self.tr_to_pulse_delay = tr_to_pulse_delay
        # self.ready = True # TODO: what is ready flag for?
        self.tx_time = self.pulse_lens[0] + 2 * self.tr_to_pulse_delay

        if npulses == 0:
            raise ValueError('number of pulses must be greater than zero!')
        if nbb_samples == 0:
            raise ValueError('number of samples in sequence must be nonzero!')

        return RMSG_SUCCESS

    # receive a ctrlprm struct
    def SetParametersHandler(self, rmsg):
        self._waitForState(STATE_WAIT)
        cprint('channel {} starting PRETRIGGER'.format(self.cnum), 'green')
        self.ctrlprm_struct.receive(self.conn)
        self.state = STATE_PRETRIGGER
        cprint('channel {} going into PRETRIGGER state'.format(self.cnum), 'green')
        # TODO for a SetParametersHandler, prepare transmit and receive sample buffers
        if (self.rnum < 0 or self.cnum < 0):
            return RMSG_FAILURE
        return RMSG_SUCCESS

    # send ctrlprm struct
    def GetParametersHandler(self, rmsg):
        # TODO: return bad status if negative radar or channel
        self.ctrlprm_struct.transmit()
        return RMSG_SUCCESS

    def GetDataHandler(self, rmsg):
        cprint('entering hardware manager get data handler', 'blue')
        # TODO: setup dataprm_struct
        # see self.ctrlprm_struct.payload['number_of_samples']
        # TODO investigate possible race conditions
        self.dataprm_struct.transmit()
        cprint('sending dprm struct', 'green')

        #pdb.set_trace()
        if not self.active or self.rnum < 0 or self.cnum < 0:
            pdb.set_trace()
            return RMSG_FAILURE

        cprint('waiting for channel to idle before GET_DATA', 'blue')
        self._waitForState(STATE_WAIT)

        cprint('entering GET_DATA state', 'blue')
        self.state = STATE_GET_DATA
        self._waitForState(STATE_WAIT)
        cprint('GET_DATA complete, returning samples', 'blue')


        # TODO: get data handler waits for control_program to set active flag in controlprg struct
        # need some sort of synchronizaion..
        # TODO: gather main/back samples..

        main_samples = np.zeros(self.dataprm_struct.get_data('samples'))
        back_samples = np.zeros(self.dataprm_struct.get_data('samples'))
        transmit_dtype(self.conn, main_samples, np.uint32)
        transmit_dtype(self.conn, back_samples, np.uint32)
        

        # TODO: what are these *actually*?
        badtrdat_len = 0
        badtrdat_start_usec = np.zeros(badtrdat_len)
        badtrdat_duration_usec = np.zeros(badtrdat_len)
        transmit_dtype(self.conn, badtrdat_len, np.uint32)
        transmit_dtype(self.conn, badtrdat_start_usec, np.uint32) # length badtrdat_len
        transmit_dtype(self.conn, badtrdat_duration_usec, np.uint32) # length badtrdat_len
        # TODO: what are these?.. really?
        num_transmitters = 16
        txstatus_agc = np.zeros(num_transmitters)
        txstatus_lowpwr = np.zeros(num_transmitters)

        transmit_dtype(self.conn, num_transmitters, np.int32)
        transmit_dtype(self.conn, txstatus_agc, np.int32) # length num_transmitters
        transmit_dtype(self.conn, txstatus_lowpwr, np.int32) # length num_transmitters

        return RMSG_SUCCESS

    def SetRadarChanHandler(self, rmsg):
        self.rnum = recv_dtype(self.conn, np.int32)
        self.cnum = recv_dtype(self.conn, np.int32)

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
    #man = RMsgManager(rmsg_port)
    #man.run()

    radar = RadarHardwareManager(rmsg_port)
    radar.run()


if __name__ == '__main__':
    main()

