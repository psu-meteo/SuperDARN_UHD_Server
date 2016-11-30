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

from phasing_utils import *
from socket_utils import *
from rosmsg import *
from drivermsg_library import *
from radar_config_constants import *
from clear_frequency_search import read_restrict_file, clrfreq_search
import logging_usrp

MAX_CHANNELS = 10
RMSG_FAILURE = -1
RMSG_SUCCESS = 0
RADAR_STATE_TIME = .0001
CHANNEL_STATE_TIMEOUT = 120000
# TODO: move this out to a config file
RESTRICT_FILE = '/home/radar/repos/SuperDARN_MSI_ROS/linux/home/radar/ros.3.6/tables/superdarn/site/site.kod/restrict.dat.inst'

debug = True

USRP_TRIGGER_START_DELAY = .5 # TODO: move this to a config file
# TODO: pull these from config file
STATE_INIT = 'INIT'
STATE_RESET = 'RESET'
STATE_WAIT = 'WAIT'
STATE_PRETRIGGER = 'PRETRIGGER'
STATE_TRIGGER = 'TRIGGER'
STATE_CLR_FREQ = 'CLR_FREQ_WAIT'
STATE_GET_DATA = 'GET_DATA'


# handle arbitration with multiple channels accessing the usrp hardware
# track state of a grouping of usrps
# merge information from multiple control programs, handle disparate settings
# e.g, ready flags and whatnot
class RadarHardwareManager:
    def __init__(self, port):
        self.client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client_sock.bind(('localhost', port))
        self.logger = logging.getLogger('HwManager')
        self.logger.info('listening on port ' + str(port) + ' for control programs')
        
        self.ini_file_init()
        self.usrp_init()
        self.rxfe_init()
     #   self.test_rxfe_control() # toggle all amp and att stages on and off for testing
        self.cuda_init()

        self.restricted_frequencies = read_restrict_file(RESTRICT_FILE)
        self.nRegisteredChannels = 0

    def run(self):
        def spawn_channel(conn):
            # start new radar channel handler
            channel = RadarChannelHandler(conn, self)
            try:
                channel.run()
            except socket.error:
                self.logger.error("RadarChannelHandler: Socket error => Deleting channel... ")
                self.deleteRadarChannel(channel)
   #         else:
   #             self.logger.error("RadarChannelHandler: Other error => Deleting channel... ")
   #             self.deleteRadarChannel(channel)
   #             conn.close()
   #             self.logger.error("RadarChannelHandler: Unexpected error is:", sys.exc_info()[0])
   #             raise

        # TODO: add lock support
        def radar_state_machine():
            sm_logger = logging.getLogger('StateMachine')
            sm_logger.info('starting radar hardware state machine')
            self.state = STATE_INIT
            self.next_state = STATE_INIT
            self.addingNewChannelsAllowed = True    # no new channel should be added between PRETRIGGER and GET_DATA
            statePriorityOrder = [ STATE_CLR_FREQ, STATE_PRETRIGGER, STATE_TRIGGER, STATE_GET_DATA]
          
            sleepTime = 0.01 # used if state machine waits for one channel

            while True:
                self.state = self.next_state
                self.next_state = STATE_RESET

                if self.state == STATE_INIT:
                    # TODO: write init code?
                    self.next_state = STATE_WAIT

                if self.state == STATE_WAIT:
                    self.next_state = STATE_WAIT

                    # 1st priority is stateOrder, 2nd is channel number
                    for iState in statePriorityOrder:
                        if self.next_state == STATE_WAIT:      # if next state still not found
                            for iChannel in self.channels:
                                if iChannel.state == iState:
                                    self.next_state = iState
                                    break
                        else:
                            break
                        

                if self.state == STATE_CLR_FREQ:
                    sm_logger.debug('start clearFreqSearch()')
                    # do a clear frequency search for channel requesting one
                    for ch in self.channels:
                        if ch.state == STATE_CLR_FREQ:
                            self.clrfreq(ch)
                            ch.state = STATE_WAIT
                    self.next_state = STATE_WAIT
                    sm_logger.debug('start clearFreqSearch()')

                if self.state == STATE_PRETRIGGER:
                    self.next_state = STATE_WAIT
                    # loop through channels, see if anyone is not ready for pretrigger
                    # stay in pretrigger state until all channels are ready
                    # TODO: add timeout?
                    # (tuning the USRP, preparing combined rf samples on GPU)
                    for ch in self.channels:
                        if not ch.state in [ STATE_PRETRIGGER, STATE_TRIGGER]:
                            if ch.state  == STATE_CLR_FREQ:
                                sm_logger.info('setting state back to {} from {}'.format(ch.state, self.state))
                                self.next_state = ch.state
                            else:
                                sm_logger.debug('remaining in PRETRIGGER because channel {} state is {}, '.format(ch.cnum, ch.state))
                                time.sleep(sleepTime)
                                self.next_state = STATE_PRETRIGGER

                    # if everyone is ready, prepare for data collection
                    if self.next_state == STATE_WAIT:
                        sm_logger.debug('start pretrigger()')
                        self.addingNewChannelsAllowed = False
                        self.pretrigger()
                        sm_logger.debug('end pretrigger()')


                if self.state == STATE_TRIGGER:
                    self.next_state = STATE_WAIT
                    # wait for all channels to be in TRIGGER state
                    for ch in self.channels:
                        if not ch.state in [ STATE_TRIGGER, STATE_GET_DATA]:
                            if ch.state in [STATE_CLR_FREQ, STATE_PRETRIGGER]:
                                sm_logger.info('setting state back to {} from {}'.format(ch.state, self.state))
                                self.next_state = ch.state
                            else:
                                sm_logger.debug('remaining in TRIGGER because channel {} state is {}, '.format(ch.cnum, ch.state))
                                time.sleep(sleepTime)
                                self.next_state = STATE_TRIGGER

                    # if all channels are TRIGGER, then TRIGGER and return to STATE_WAIT
                    if self.next_state == STATE_WAIT:
                        sm_logger.debug('start trigger()')
                        self.trigger()
                        sm_logger.debug('end trigger()')

                if self.state == STATE_GET_DATA:
                    sm_logger.debug('start get_data()')
                    self.next_state = STATE_WAIT

                    self.get_data(self.channels)

                    for ch in self.channels:
                        if ch.state == STATE_GET_DATA: 
                            ch.state = STATE_WAIT
                    self.addingNewChannelsAllowed = True
                    sm_logger.debug('end get_data()')


                if self.state == STATE_RESET:
                    sm_logger.error('stuck in STATE_RESET?!')
                    pdb.set_trace()
                    self.next_state = STATE_INIT

                time.sleep(RADAR_STATE_TIME)

        self.client_sock.listen(MAX_CHANNELS)
        client_threads = []
        self.channels = []
        usrp_server_logger = logging.getLogger('usrp_server')

        ct = threading.Thread(target=radar_state_machine)
        ct.start()
        while True:

            usrp_server_logger.info('waiting for control program')
            client_conn, addr = self.client_sock.accept()

            usrp_server_logger.info('connection from control program, spawning channel handler thread')
            ct = threading.Thread(target=spawn_channel, args=(client_conn,))
            client_threads.append(ct)
            ct.start()
       
            # remove threads that are not alive
            client_threads = [iThread for iThread in client_threads if iThread.is_alive()]

 

        self.client_sock.close()


    # read in ini config files..
    def ini_file_init(self):
        # READ driver_config.ini
        driver_config = configparser.ConfigParser()
        driver_config.read('driver_config.ini')
        self.ini_shm_settings = driver_config['shm_settings']
        self.ini_cuda_settings = driver_config['cuda_settings']
        self.ini_network_settings = driver_config['network_settings']

        # READ usrp_config.ini
        usrp_config = configparser.ConfigParser()
        usrp_config.read('usrp_config.ini')
        usrp_configs = []
        for usrp in usrp_config.sections():
            usrp_configs.append(usrp_config[usrp])
        self.ini_usrp_configs = usrp_configs


        # READ array_config.ini
        array_config = configparser.ConfigParser()
        array_config.read('array_config.ini')
        self.ini_array_settings = array_config['array_info']
        self.ini_rxfe_settings  = array_config['rxfe']



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
                self.logger.debug('connecting to usrp driver on port {}'.format(usrp[1]))
                usrpsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                usrpsock.connect(usrp)
                usrp_driver_socks.append(usrpsock)
        except ConnectionRefusedError:
            self.logger.error('USRP server connection failed on port {}'.format(usrp[1]))
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
            if max(np.array(usrptimes) - usrptimes[0]) < 1:
                usrps_synced = True
            else:
                # TODO: why does USRP synchronization fail?
                self.logger.warning('_resync_USRP USRP syncronization failed, trying again..')
                time.sleep(1)


    def rxfe_init(self):
        
        activeStrings = ['true', '1', 'on']
        amp1 = self.ini_rxfe_settings['enable_amp1'].lower() in activeStrings
        amp2 = self.ini_rxfe_settings['enable_amp2'].lower() in activeStrings
        att = float(self.ini_rxfe_settings['attenuation'])
        if att < 0:
           self.logger.warning('attenuation for rxfe in array.ini is defnined positive, but given value is negative ({} dB). correcting that to {} dB...'.format(att, att*(-1)))
           att *= -1

        if att > 31.5:
           self.logger.warning('attenuation ({}) for rxfe in array.ini is > 31.5 dB. using maximum atenuation of 31.5 dB'.format(att))
           att = 31.5

        self.logger.info("Setting RXFR: Amp1={}, Amp2={}, Attenuation={} dB".format(amp1, amp2, att)) 
        cmd = usrp_rxfe_setup_command(self.usrpsocks, amp1, amp2, att*2) # *2 since LSB is 0.5 dB 
        cmd.transmit()
        cmd.client_return()


    def test_rxfe_control(self):

        self.logger.warning("Starting RXFE test: walk through all bits:") 

        testParSet = [[False, False, 0], [True, False, 0], [True, True, 0], [False, True, 0], [True, True, 31.5]] + [[False, False, 2**i/2] for i in range(6) ]
        
        nSets = len(testParSet)
        for iSet in range(nSets):
            amp1 = testParSet[iSet][0]
            amp2 = testParSet[iSet][1]
            att  = testParSet[iSet][2]

            cmd = usrp_rxfe_setup_command(self.usrpsocks, amp1, amp2, att*2) # *2 since LSB is 0.5 dB 
            cmd.transmit()
            cmd.client_return()
            self.logger.warning("Current settings: Amp1={}, Amp2={}, Attenuation={} dB".format(amp1, amp2, att)) 
            input("  Press Enter for next chage...")

        print("Finished testing RXFE!")

    def cuda_init(self):
        #time.sleep(.05)

        self.tx_upsample_rate = int(self.ini_cuda_settings['TXUpsampleRate']) # TODO: delete because unused

        # connect cuda_driver servers
        cuda_driver_socks = []

        cuda_driver_port = int(self.ini_network_settings['CUDADriverPort'])
        cuda_driver_hostnames = self.ini_network_settings['CUDADriverHostnames'].split(',')

        try:
            for c in cuda_driver_hostnames:
                self.logger.debug('connecting to cuda driver on {}'.format(c))
                cudasock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                cudasock.connect((c, cuda_driver_port))
                cuda_driver_socks.append(cudasock)
        except ConnectionRefusedError:
            self.logger.error("cuda server connection failed on {}".format(c))
            sys.exit(1)

        self.cudasocks = cuda_driver_socks


    def setup(self, chan):
        pass

    def clrfreq(self, chan):
        self.logger.debug('running clrfreq for channel {}'.format(chan.cnum))
        tbeamnum = chan.ctrlprm_struct.get_data('tbeam')
        tbeamwidth = 3.24 # TODO: why is it zero, this should be chan.ctrlprm_struct.get_data('tbeamwidth')
        chan.tfreq, chan.noise = clrfreq_search(chan.clrfreq_struct, self.usrpsocks, self.restricted_frequencies, tbeamnum, tbeamwidth, self.ini_array_settings['nbeams']) 
        chan.tfreq /= 1000 # clear frequency search stored in kHz for compatibility with control programs..
        self.logger.debug('clrfreq for channel {} found {} kHz'.format(chan.cnum, chan.tfreq))

    def get_data(self, allChannels):
        nMainAntennas = int(self.ini_array_settings['main_ants'])
        nBackAntennas = int(self.ini_array_settings['back_ants'])
        self.logger.debug('running get_data()')

        nChannels = len(allChannels)

        # stall until all USRPs are ready
        self.logger.debug('sending USRP_READY_DATA command ')
        cmd = usrp_ready_data_command(self.usrpsocks)
        cmd.transmit()
        self.logger.debug('sending USRP_READY_DATA command sent, waiting on READY_DATA status')

        # alloc memory
        nSamples_bb  = self.commonChannelParameter['number_of_samples'] 
        main_samples = [np.complex64(np.zeros((nMainAntennas, nSamples_bb))) for iCh in range(nChannels) ]
        back_samples = [np.complex64(np.zeros((nBackAntennas, nSamples_bb))) for iCh in range(nChannels) ] 
        
        # check status of usrp drivers

        for usrpsock in self.usrpsocks:
            self.logger.debug('start receiving one USRP status')
            transmit_dtype(usrpsock, 11 , np.int32) # WHY DO THIS? (not used in usrp_driver) 

            ready_return = cmd.recv_metadata(usrpsock)
            rx_status = ready_return['status']

            self.logger.debug('GET_DATA rx status {}'.format(rx_status))
            if rx_status != 2:
                self.logger.error('USRP driver status {} in GET_DATA'.format(rx_status))
                #status = USRP_DRIVER_ERROR # TODO: understand what is an error here..
            self.logger.debug('end receiving one USRP status')

        self.logger.debug('start waiting for USRP_DATA return')
        cmd.client_return()
        self.logger.debug('end waiting for USRP_DATA return')

        self.logger.debug('GET_DATA: received samples from USRP_DRIVERS, status: ' + str(rx_status))

        # DOWNSAMPLING
        self.logger.debug('start get_data(): CUDA_PROCESS')
        cmd = cuda_process_command(self.cudasocks)
        cmd.transmit()
        cmd.client_return()
        self.logger.debug('end get_data(): CUDA_PROCESS')

        # RECEIVER RX BB SAMPLES 
        # recv metadata from cuda drivers about antenna numbers and whatnot
        # fill up provided main and back baseband sample buffers
        self.logger.debug('start get_data() transfering rx data via socket')
        cmd = cuda_get_data_command(self.cudasocks)
        cmd.transmit()

        for cudasock in self.cudasocks:
            nAntennas = recv_dtype(cudasock, np.uint32)
            for iChannel,channel in enumerate(allChannels):
                transmit_dtype(cudasock, channel.cnum, np.int32)

                for iAntenna in range(nAntennas):
                    antIdx = recv_dtype(cudasock, np.uint16)
                    num_samples = recv_dtype(cudasock, np.uint32)
                    samples = recv_dtype(cudasock, np.float32, num_samples)
                    samples = samples[0::2] + 1j * samples[1::2] # unpacked interleaved i/q
                    
                    if antIdx < nMainAntennas:
                        main_samples[iChannel][antIdx] = samples[:]

                    else:
                        back_samples[iChannel][antIdx - nMainAntennas] = samples[:]
                                
      
            transmit_dtype(cudasock, -1, np.int32) # to end transfer process
            
        cmd.client_return()
        self.logger.debug('end get_data() transfering rx data via socket')

        
        # BEAMFORMING
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

        for iChannel, channel in enumerate(allChannels):
            ctrlprm = channel.ctrlprm_struct.payload
            # TODO: where are RADAR_NBEAMS and RADAR_BEAMWIDTH is comming from?
#            bmazm         = calc_beam_azm_rad(RADAR_NBEAMS, ctrlprm['tbeam'], RADAR_BEAMWIDTH)    # calculate beam azimuth from transmit beam number
            # TODO: use data from config file
            beam_sep  = float(self.ini_array_settings['beam_sep'] ) # degrees
            nbeams    = int(  self.ini_array_settings['nbeams'] )
            x_spacing = float(self.ini_array_settings['x_spacing'] ) # meters 
            beamnum   = ctrlprm['tbeam']

            bmazm         = calc_beam_azm_rad(nbeams, beamnum, beam_sep)    # calculate beam azimuth from transmit beam number          
            pshift        = calc_phase_increment(bmazm, ctrlprm['tfreq'] * 1000., x_spacing)       # calculate antenna-to-antenna phase shift for steering at a frequency        
            antennas_list = [0]   # TODO: HARDCODED TO ONE ANTENNA
            phasingMatrix_main = np.array([rad_to_rect(a * pshift) for a in antennas_list])  # calculate a complex number representing the phase shift for each antenna
           #phasingMatrix_back = np.ones(len(MAIN_ANTENNAS))


            channel.main_beamformed = _beamform_uhd_samples(main_samples[iChannel], phasingMatrix_main, nSamples_bb, antennas_list)
           #channel.back_beamformed = _beamform_uhd_samples(back_samples, phasingMatrix_back, nSamples_bb, antennas)
        
        self.logger.debug('get_data complete!')



    def deleteRadarChannel(self, channelObject):
        if channelObject in self.channels:
            # don't delete channel in middle of trigger, pretrigger, or ....
            channelObject._waitForState(STATE_WAIT) 
            self.logger.info('deleteRadarChannel() removing channel {} from HardwareManager'.format(self.channels.index(channelObject)))
            self.channels.remove(channelObject)
            self.logger.debug('RHM:deleteRadarChannel {} channels left'.format(len(self.channels)))
            if (len(self.channels) == 0) and not self.addingNewChannelsAllowed:  # reset flag for adding new channels  if last channel has been deleted between PRETRIGGER and GET_DATA
                self.addingNewChannelsAllowed = True

            self.nRegisteredChannels -= 1
            if self.nRegisteredChannels == 0:  
                self.commonChannelParameter = {}

        else:
            self.logger.warning('deleteRadarChannel() channel already deleted')


        

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

        self.logger.info('received exit command, cleaning up..')

        sys.exit(0)

    def trigger(self):
        self.logger.debug('running trigger, triggering usrp drivers')

        if len(self.channels):
        
            # grab current time from a usrp
            cmd = usrp_get_time_command(self.usrpsocks[0])
            cmd.transmit() 
            usrp_time = cmd.recv_time(self.usrpsocks[0])
            cmd.client_return()
            trigger_time = usrp_time + USRP_TRIGGER_START_DELAY
            cmd = usrp_trigger_pulse_command(self.usrpsocks, trigger_time)
            self.logger.debug('sending trigger pulse command')
            cmd.transmit()
            # TODO: handle multiple usrps with trigger..
            self.logger.debug('waiting for trigger return')
            returns = cmd.client_return()

            if TRIGGER_BUSY in returns:
                self.logger.error('could not trigger, usrp driver is busy')
                pdb.set_trace()

            self.logger.debug('trigger return!')

        else:
            self.logger.error('no channels loaded, trigger failed!')
            pdb.set_trace()

        for ch in self.channels:
            ch.state = STATE_WAIT

        self.logger.debug('trigger complete')

    # TODO: Merge channel infomation here!
    # TODO: pretrigger cuda client return fails
    # key error: 0?
    def pretrigger(self):
        self.logger.debug('start RadarHardwareManager.pretrigger()')
        

       #  self.logger.debug('running cuda_pulse_init command')
        # TODO: untangle cuda pulse init handler
        #        pdb.set_trace()
        #        cmd = cuda_pulse_init_command(self.cudasocks)
        #        cmd.transmit()
        #        self.logger.debug('waiting for cuda driver return')
        #        cmd.client_return()
        #        self.logger.debug('cuda return received')

       
        synth_pulse = False
        
        for ch in self.channels:
            #pdb.set_trace()
            if ch.ctrlprm_struct.payload['tfreq'] != 0:
                # check that we are actually able to transmit at that frequency given the USRP center frequency and sampling rate TODO
                self.logger.debug('pretrigger() tfreq: {}, rfreq: {}. usrp_center: tx={} rx={} '.format( ch.ctrlprm_struct.payload['tfreq'], ch.ctrlprm_struct.payload['rfreq'], self.usrp_tx_cfreq, self.usrp_rf_tx_rate))
                
                assert np.abs((ch.ctrlprm_struct.payload['tfreq'] * 1e3) - self.usrp_tx_cfreq) < (self.usrp_rf_tx_rate / 2), 'transmit frequency outside range supported by sampling rate and center frequency'

                # load sequence to cuda driver if it has a nonzero transmit frequency..
                #time.sleep(.05) # TODO: investigate race condition.. why does adding a sleep here help
                #self.logger.debug('after 50 ms of delay..')

                if not (ch.ctrlprm_struct.payload['tfreq'] == ch.ctrlprm_struct.payload['rfreq']):
                    self.logger.warning('pretrigger() tfreq (={}) != rfreq (={}) !'.format( ch.ctrlprm_struct.payload['tfreq'], ch.ctrlprm_struct.payload['rfreq']))
                    #pdb.set_trace()
                cmd = cuda_add_channel_command(self.cudasocks, sequence=ch.getSequence())
                # TODO: separate loading sequences from generating baseband samples..

                self.logger.debug('send CUDA_ADD_CHANNEL')
                cmd.transmit()

                cmd.client_return()
                self.logger.debug('receiver response CUDA_ADD_CHANNEL')
                synth_pulse = True


        if synth_pulse:
            self.logger.debug('send CUDA_GENERATE_PULSE')
            cmd = cuda_generate_pulse_command(self.cudasocks)
            cmd.transmit()
            cmd.client_return()
            self.logger.debug('received respinse CUDA_GENERATE_PULSE')

        # TODO: parse tx sample rate dynamocially
        # TODO: handle pretrigger with no channels
        pulse_offsets_vector = self.commonChannelParameter['pulse_offsets_vector'] 
        npulses              = self.commonChannelParameter['npulses']

        # calculate exact number of RF samples for RX and TX (equal to cuda_driver)
        tx_rf_nSamples = np.uint64(int(self.ini_cuda_settings['TXUpsampleRate']) *  ( np.floor(self.usrp_rf_tx_rate * self.commonChannelParameter['pulseLength']/1e6 ) + 2 * np.floor(self.usrp_rf_tx_rate * self.commonChannelParameter['tr_to_pulse_delay']/1e6 )))
        rx_if_nSamples = np.floor((self.commonChannelParameter['number_of_samples'] -1) * int(self.ini_cuda_settings['IFBBRATE']) + int(self.ini_cuda_settings['NTapsRX_ifbb']))
        rx_rf_nSamples = np.uint64(np.floor((rx_if_nSamples -1) * int(self.ini_cuda_settings['RFIFRATE']) + int(self.ini_cuda_settings['NTapsRX_rfif'])))

        self.logger.debug("nSamples rx: {}   nSamples tx: {}".format(rx_rf_nSamples, tx_rf_nSamples))
        self.logger.debug('send USRP_SETUP')
        cmd = usrp_setup_command(self.usrpsocks, self.usrp_tx_cfreq, self.usrp_rx_cfreq, self.usrp_rf_tx_rate, self.usrp_rf_rx_rate, npulses, rx_rf_nSamples, tx_rf_nSamples, pulse_offsets_vector)
        cmd.transmit()
        cmd.client_return()
        self.logger.debug('received response USRP_SETUP')

        for ch in self.channels:
            if ch.state == STATE_PRETRIGGER: # only reset PRETRIGGER , not TRIGGER
                ch.state = STATE_WAIT

        self.logger.debug('end RadarHardwareManager.pretrigger()')


class RadarChannelHandler:
    def __init__(self, conn, parent_RadarHardwareManager):
        self.active = False# TODO: eliminate
        self.ready = False # TODO: eliminate
        self.conn = conn
        self.parent_RadarHardwareManager = parent_RadarHardwareManager
        self.logger = logging.getLogger("ChManager")
        self.state = STATE_WAIT

        self.ctrlprm_struct = ctrlprm_struct(self.conn)
        self.seqprm_struct = seqprm_struct(self.conn)
        self.clrfreq_struct = clrfreqprm_struct(self.conn)
        self.rprm_struct = rprm_struct(self.conn)
        self.dataprm_struct = dataprm_struct(self.conn)

        self.clrfreq_start = 0
        self.clrfreq_stop = 0
        self.cnum = 'unknown'

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

        while not self.parent_RadarHardwareManager.addingNewChannelsAllowed:
            self.logger.info("Waiting to finish GET_DATA before adding new channel")
            time.sleep(RADAR_STATE_TIME)

        # adding channel to HardwareManager
        self.parent_RadarHardwareManager.channels.append(self)  

        while True:
            rmsg = rosmsg_command(self.conn)
            status = RMSG_FAILURE

            self.logger.debug('ch {}: waiting for command'.format(self.cnum))
            rmsg.receive(self.conn)
            command = chr(rmsg.payload['type'] & 0xFF) # for some reason, libtst is sending out 4 byte commands with junk..
            try:
                self.logger.debug('ch {}: received command (ROS=>USRP_Server): {}, {}'.format(self.cnum, command, RMSG_COMMAND_NAMES[command]))
            except KeyError:
                self.logger.error('unrecognized command! {}'.format(rmsg.payload))
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
        self.logger.error('todo: write close function...')
        pdb.set_trace()
        # TODO write this..


    # busy wait until state enters desired state
    # useful for waiting for
    def _waitForState(self, state):
        wait_start = time.time()

        while self.state != state:
            time.sleep(RADAR_STATE_TIME)
            if time.time() - wait_start > CHANNEL_STATE_TIMEOUT:
                self.logger.error('CHANNEL STATE TIMEOUT')
                pdb.set_trace()
                break

    # return a sequence object, used for passing pulse sequence and channel infomation over to the CUDA driver
    def getSequence(self):
        return sequence(self.npulses, self.tr_to_pulse_delay, self.pulse_offsets_vector, self.pulse_lens, self.phase_masks, self.pulse_masks, self.ctrlprm_struct.payload)

    def DefaultHandler(self, rmsg):
        self.logger.error("Unexpected command: {}".format(chr(rmsg.payload['type'])))
        pdb.set_trace()
        return RMSG_FAILURE

    def QuitHandler(self, rmsg):
        # TODO: close down stuff cleanly
        rmsg.set_data('status', RMSG_FAILURE)
        rmsg.set_data('type', rmsg.payload['type'])
        rmsg.transmit()
        self.conn.close()
        self.logger.debug('Deleting channel...')
        hardwareManager = self.parent_RadarHardwareManager
        hardwareManager.deleteRadarChannel(self)
        del self # TODO close thread ?!?
        # sys.exit() # TODO: set return value
        hardwareManager.logger.info('Deleted channel.')
        return 'exit'

    def PingHandler(self, rmsg):
        return RMSG_SUCCESS

    def RequestAssignedFreqHandler(self, rmsg):
        # wait for clear frequency search to end, hardware manager will set channel state to WAIT
        self._waitForState(STATE_WAIT) 

        transmit_dtype(self.conn, self.tfreq, np.int32)
        transmit_dtype(self.conn, self.noise, np.float32)

        self.clrfreq_stop = time.time()
        self.logger.info('clr frequency search time: {}'.format(self.clrfreq_stop - self.clrfreq_start))
        return RMSG_SUCCESS

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
        # TODO: check if samples are ready?
        self.state = STATE_TRIGGER
        # send trigger command
        self.ready = True
        self._waitForState(STATE_WAIT)
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
        pulse_offsets_vector = tr_rising_edge_idx * tx_tsg_step

        # extract tr window to rf pulse delay
        rf_pulse_idx = np.nonzero(rf_pulse)[0]
        rf_pulse_edge_idx = _rising_edge_idx(rf_pulse_idx)
        tr_to_pulse_delay = (rf_pulse_edge_idx[0] - tr_rising_edge_idx[0]) * tx_tsg_step
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
            pulse_lens.append((pend - pstart) * tx_tsg_step)

        self.npulses = npulses
        self.pulse_offsets_vector = pulse_offsets_vector / 1e6
        self.pulse_lens = pulse_lens # length of pulses, in seconds
        self.phase_masks = phase_masks # phase masks are complex number to multiply phase by, so
        self.pulse_masks = pulse_masks
        self.tr_to_pulse_delay = tr_to_pulse_delay
        # self.ready = True # TODO: what is ready flag for?
        
        
        self.logger.debug("pulse0 length: {} us, tr_pulse_delay: {} us, tx_time: {} us".format(self.pulse_lens[0], tr_to_pulse_delay,  self.pulse_lens[0] + 2 * self.tr_to_pulse_delay))
        if npulses == 0:
            raise ValueError('number of pulses must be greater than zero!')
        if nbb_samples == 0:
            raise ValueError('number of samples in sequence must be nonzero!')

      #  pdb.set_trace()

        return RMSG_SUCCESS

    # receive a ctrlprm struct
    def SetParametersHandler(self, rmsg):
        self._waitForState(STATE_WAIT)
        self.logger.debug('channel {} starting PRETRIGGER'.format(self.cnum))
        self.ctrlprm_struct.receive(self.conn)
        self.logger.debug('tfreq: {}, rfreq: {}'.format(self.ctrlprm_struct.payload['tfreq'], self.ctrlprm_struct.payload['rfreq']))

        if not self.CheckChannelCompatibility():   # TODO: move this to point where sequence is received?
            self.logger.error("new channel is not compatible")
            return RMSG_FAILURE


        self.state = STATE_PRETRIGGER
        self.logger.debug('channel {} going into PRETRIGGER state'.format(self.cnum))
        self._waitForState(STATE_WAIT)
        # TODO for a SetParametersHandler, prepare transmit and receive sample buffers
        if (self.rnum < 0 or self.cnum < 0):
            return RMSG_FAILURE
        return RMSG_SUCCESS

    def CheckChannelCompatibility(self):
        hardwareManager = self.parent_RadarHardwareManager
        commonParList_ctrl = ['number_of_samples', 'baseband_samplerate' ]
        commonParList_seq  = [ 'npulses', 'pulse_offsets_vector',  'tr_to_pulse_delay' ]
        if all([self.pulse_lens[0]==self.pulse_lens[i] for i in range(1,len(self.pulse_lens))]):
            pulseLength = self.pulse_lens[0]
        else:
            self.logger.error("Pulse lengths has the be the same! ") # TODO raise error?
            return False

        if hardwareManager.nRegisteredChannels == 0:  # this is the first channel
            hardwareManager.commonChannelParameter = {key: getattr(self, key) for key in commonParList_seq}
            hardwareManager.commonChannelParameter.update( {key: self.ctrlprm_struct.payload[key] for key in commonParList_ctrl})
            hardwareManager.commonChannelParameter.update({'pulseLength':pulseLength})

            hardwareManager.nRegisteredChannels = 1
            return True

        else:   # not first channel => check if new parameters are compatible
            parCompatibleList_seq = [hardwareManager.commonChannelParameter[parameter] == getattr(self, parameter) for parameter in commonParList_seq]
            parCompatibleList_ctrl = [hardwareManager.commonChannelParameter[parameter] == self.ctrlprm_struct.payload[parameter] for parameter in commonParList_ctrl]
            idxOffsetVec = commonParList_seq.index('pulse_offsets_vector')  # convert vector of bool to scalar
            parCompatibleList_seq[idxOffsetVec] = parCompatibleList_seq[idxOffsetVec].all()
 
         #   pdb.set_trace()
            if (not all(parCompatibleList_seq)) or (not  all(parCompatibleList_ctrl)) or (pulseLength != hardwareManager.commonChannelParameter['pulseLength']):
                self.logger.error('Unable to add new channel. Parameters not compatible with active channels.')
                for iPar,isCompatible in enumerate(parCompatibleList_seq):
                     if not all(isCompatible):
                        self.logger.error(" Not compatible sequence parameter: {}   old channel(s): {} , new channel: {}".format(commonParameterList_seq[iPar], hardwareManager.commonChannelParameter[commonParameterList_seq[iPar]] , getattr(self, commonParameterList_seq[iPar])))
                for iPar,isCompatible in enumerate(parCompatibleList_ctrl):
                     if not isCompatible:
                        self.logger.error(" Not compatible ctrlprm: {}   old channel(s): {} , new channel: {}".format(commonParameterList_ctrl[iPar], hardwareManager.commonChannelParameter[commonParameterList_ctrl[iPar]] , self.ctrlprm_struct.payload[commonParameterList_ctrl[iPar]]))
                if pulseLength != hardwareManager.commonChannelParameter['pulseLength']:
                    self.logger.error(" Pulse length of new channel ({}) is not compatible to old channel(s) ({})".format(pulseLength, hardwareManager.commonChannelParameter['pulseLength'])) 
                return False
            else:
                hardwareManager.nRegisteredChannels += 1
                return True
                    

        
     


    # send ctrlprm struct
    def GetParametersHandler(self, rmsg):
        # TODO: return bad status if negative radar or channel
        self.ctrlprm_struct.transmit()
        return RMSG_SUCCESS

    def GetDataHandler(self, rmsg):
        self.logger.debug('entering hardware manager get data handler')
        self.dataprm_struct.set_data('samples', self.ctrlprm_struct.payload['number_of_samples'])

        self.dataprm_struct.transmit()
        self.logger.debug('sending dprm struct')

        if not self.active or self.rnum < 0 or self.cnum < 0:
            pdb.set_trace()
            return RMSG_FAILURE

        # TODO investigate possible race conditions
        self.logger.debug('waiting for channel to idle before GET_DATA')
        self._waitForState(STATE_WAIT)

        self.logger.debug('entering GET_DATA state')
        self.state = STATE_GET_DATA
        self.logger.debug('waiting to return to WAIT before returning samples')
        self._waitForState(STATE_WAIT)
        self.logger.debug('GET_DATA complete, returning samples')


        # TODO: get data handler waits for control_program to set active flag in controlprg struct
        # need some sort of synchronizaion..
        # TODO: back samples..
        main_samples = self.main_beamformed
        back_samples = np.zeros(self.dataprm_struct.get_data('samples'))

        transmit_dtype(self.conn, main_samples, np.uint32)
        transmit_dtype(self.conn, back_samples, np.uint32)
        

        badtrdat_start_usec = self.pulse_offsets_vector * 1e6 # convert to us
        transmit_dtype(self.conn, self.npulses, np.uint32)
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

    def SetRadarChanHandler(self, rmsg):
        self.rnum = recv_dtype(self.conn, np.int32)
        self.cnum = recv_dtype(self.conn, np.int32)
        
        self.ctrlprm_struct.set_data('channel', self.cnum)
        self.ctrlprm_struct.set_data('radar',  self.rnum)

        # TODO: how to handle channel contention?
        self.logger.debug('radar num: {}, radar chan: {}'.format(self.rnum, self.cnum))

        # TODO: set RMSG_FAILURE if radar channel is unavailable
        # rmsg.set_data('status', RMSG_FAILURE)
        return RMSG_SUCCESS

    def LinkRadarChanHandler(self, rmsg):
        rnum = recv_dtype(self.conn, np.int32)
        cnum = recv_dtype(self.conn, np.int32)
        self.logger.error('link radar chan is unimplemented!')
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
    
    logging_usrp.initLogging('server.log')

    # Now, we can log to the root logger, or any other logger. First the root...
    logging.info('Strating main() of usrp_server')

    rmsg_port = 45000

    radar = RadarHardwareManager(rmsg_port)
    radar.run()


if __name__ == '__main__':
    main()

