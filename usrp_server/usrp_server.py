#!/usr/bin/python3
# Interface between control program(s) and drivers (cuda and usrp)
#
# RadarChannelHandler class
# - one object for each control program
# - communicates with the control program and changes the state of the channel
#
# RadarHardwareManager class (RHM)
# - has a radar_main_control_loop() that executes clear freq search, adding new channels and triggering
#

# TODO: write mock arbyserver that can handle these commands
# TODO: write mock arby server that can feed multiple normalscans with false data..


import sys
import os
import numpy as np
import threading
import logging
import pdb
import socket
import time
import configparser
import copy
import posix_ipc
import pickle
sys.path.insert(0, '../python_include')

from phasing_utils import *
from socket_utils import *
from rosmsg import *
from drivermsg_library import *
from radar_config_constants import *
from clear_frequency_search import read_restrict_file, record_clrfreq_raw_samples, calc_clear_freq_on_raw_samples
from profiling_tools import *
import logging_usrp

MAX_CHANNELS = 10
RMSG_FAILURE = -1
CLRFREQ_RES_HZ = 1000
RMSG_SUCCESS = 0
RADAR_STATE_TIME = .0001
CHANNEL_STATE_TIMEOUT = 12000
# TODO: move this out to a config file
RESTRICT_FILE = '/home/radar/repos/SuperDARN_MSI_ROS/linux/home/radar/ros.3.6/tables/superdarn/site/site.kod/restrict.dat.inst'
nSwings = 2 

debug = True 

# channel states (CS) for each channel
CS_INACTIVE      = 'CS_INACTIVE'
CS_READY         = 'CS_READY'
CS_TRIGGER       = 'CS_TRIGGER'
CS_PROCESSING    = 'CS_PROCESSING'
CS_SAMPLES_READY = 'CS_SAMPLES_READY'
CS_LAST_SWING    = 'CS_LAST_SWING'

class usrpSockManager():
   def __init__(self, RHM):
      self.addressList_active     = [] # tuple of IP and port
      self.addressList_inactive   = []
      self.socks = []
      usrp_driver_base_port = int(RHM.ini_network_settings['USRPDriverPort'])
      self.RHM = RHM
      self.logger = logging.getLogger("usrpSockManager")      
      
      addressList = []
      for usrp in RHM.ini_usrp_configs:
          addressList.append((usrp['driver_hostname'], int(usrp['array_idx']) + usrp_driver_base_port))
      
      self.nUSRPs = len(addressList) # TODO should this be all USRPs or only active?
      self.fault_status = np.ones(self.nUSRPs)
      
      # open each 
      for usrp in addressList:
         try: 
            usrpsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            usrpsock.connect(usrp)
            self.socks.append(usrpsock)
            self.addressList_active.append(usrp)
            self.logger.debug('connected to usrp driver on port {}'.format(usrp[1]))
         except ConnectionRefusedError:
            self.logger.error('USRP server connection failed on port {}'.format(usrp[1]))
            self.addressList_inactive.append(usrp)


   def eval_client_return(self, cmd, fcn=None):
      if fcn is None: # default receive function
         client_return = cmd.client_return()
      else:
         client_return = fcn()

      if CONNECTION_ERROR in client_return:
         offset = 0
         for iSock, singleReturn in enumerate(client_return):
            if singleReturn == CONNECTION_ERROR:
               self.logger.error("Connection lost to usrp {}:{}. Removing it from sock list. ".format(self.addressList_active[iSock-offset][0], self.addressList_active[iSock-offset][1])) 
               self.addressList_inactive.append(self.addressList_active[iSock-offset] )
               del self.addressList_active[iSock-offset]
               del self.socks[iSock-offset]
               offset += 1 
      return client_return

   def restore_lost_connections(self):
      addressList = self.addressList_inactive
      self.addressList_inactive = [] 
      for usrp in addressList:
         self.logger.error("Skipping reconnecting usrps because resync causes problems. TODO fix it!")
         return
         try: 
            usrpsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            usrpsock.connect(usrp)
            # TODO : sync to other usrps
            self.RHM._resync_usrps
            # TODO : initialize shm with zeros
            self.socks.append(usrpsock)
            self.addressList_active.append(usrp)
            self.logger.info('reconnection to usrp driver on port {} successful'.format(usrp[1]))
         except ConnectionRefusedError:
            self.logger.error('usrp reconnection failed on port {}'.format(usrp[1]))
            self.addressList_inactive.append(usrp)

class usrpMixingFreqManager():
    """ Manages usrp mixing frequency based on channels. Ensures that only one channel
        at a time can call add_new_freq_band().  """
  
    def __init__(self, cFreq, bandWidth):
       self.current_mixing_freq = cFreq
       self.usrp_bandwidth      = bandWidth
       self.semaphore = posix_ipc.Semaphore('usrp_mixing_freq', posix_ipc.O_CREAT)
       self.semaphore.release()
       self.channelRangeList    = []
       self.channelList         = []

    def add_new_freq_band(self, channel):
       """ Checks if new channel is covered with current mixing frequency and bandwidth. 
           Retruns True/False if channel can/cannot be added. If changing the mixing frequency
           allows to add the new channel, the new mixing frequency will be output argument.
       """

       newLower, newUpper = self.get_range_of_channel(channel)

       channel.logger.debug("ch {}: waiting for semaphore of usrpMixingFreqManager")
       self.semaphore.acquire()
       channel.logger.debug("ch {}: acquired semaphore of usrpMixingFreqManager")
   
       if newLower > (self.current_mixing_freq - self.usrp_bandwidth/2) and newUpper < (self.current_mixing_freq + self.usrp_bandwidth/2):
          channel.logger.degug("channel range is within USRP bandwidth")
          self.channelRangeList.append([newLower, newUpper])
          self.channelList.append(channel)
          result = True
       else:
          #determine range of all channels
          allCh_lower = newLower
          allCh_upper = newUpper
   
          for otherChRange in self.channelRangeList:
             allCh_lower = min(allCh_lower, otherChRange[0])
             allCh_upper = max(allCh_upper, otherChRange[1])


          if (allCh_upper - allCh_lower) > self.usrp_bandwith:
             channel.logger.error("new channel can not be added. USPR bandwidth too small")
             result =  False
          else:
             newMixingFreq = (allCh_upper - allCh_lower)/2 + allCh_lower
             channel.logger.info("calculated new usrp mixing frequency: {} kHz (old was {} kHz)".format(newMixingFreq, self.current_mixing_freq))
             # adjust mixing freq that everything is in overall bandwidth
             RHM = channel.parent_RadarHardwareManager
             newMixingFreq = max(newMixingFreq, RHM.hardwareLimit_freqRange[0]+self.usrp_bandwidth/2)
             newMixingFreq = min(newMixingFreq, RHM.hardwareLimit_freqRange[1]-self.usrp_bandwidth/2)
             self.current_mixing_freq = newMixingFreq
             result = newMixingFreq
          
     
       self.semaphore.release()
       channel.logger.debug("ch {}: released semaphore of usrpMixingFreqManager")
       return result

    def get_range_of_channel(self, channel):
       rangeList = channel.scanManager.clear_freq_range_list
       lower = rangList[0][0] 
       upper = rangeList[0][1]
       for periodRange in rangeList[1:]:
          lower = min(lower, periodRange[0])
          upper = max(lower, periodRange[1])
       return lower, upper


class clearFrequencyRawDataManager():
    """ Buffers the raw clearfrequency data for all channels
    """
    def __init__(self, antenna_spacing):
        self.rawData    = None
        self.recordTime = None

        self.raw_data_available_from_this_period = False
        self.outstanding_request = False     # Flag set by the RadarChannelHandlers 
        self.repeat_request_for_2nd_period = False

        self.usrp_socks = None
        self.center_freq = None
        self.sampling_rate = None
        self.number_of_samples = None
         
        self.metaData = {}
        self.rawData = None
        
        self.metaData['x_spacing'] = antenna_spacing 

        self.logger = logging.getLogger('clearFrequency')
        self.logger.debug('clearFrequencyRawDataManager initialized')


    def set_usrp_driver_connections(self, usrp_driver_socks):
        self.usrp_socks = usrp_driver_socks
    
    def set_clrfreq_search_span(self, center_freq, clrfreq_sampling_rate, number_of_clrfreq_samples):
        self.center_freq = center_freq

        self.sampling_rate = clrfreq_sampling_rate
        self.number_of_samples = number_of_clrfreq_samples

        self.metaData['usrp_fcenter'] = self.center_freq 
        self.metaData['number_of_samples'] = self.number_of_samples 
        self.metaData['usrp_rf_rate'] = self.sampling_rate 

    def period_finished(self):
        self.raw_data_available_from_this_period = False
   
    def record_new_data(self):
        assert self.usrp_socks != None, "no usrp drivers assigned to clear frequency search data manager"
        assert self.center_freq != None, "no center frequency assigned to clear frequency search manager"

        self.logger.debug('grabbing new data')
        self.rawData, self.antennaList = record_clrfreq_raw_samples(self.usrp_socks, self.number_of_samples, self.center_freq, self.sampling_rate)

        self.metaData['antenna_list'] = self.antennaList

        self.logger.debug("recorded clear samples for clear frequency search, antenna list: {}".format(self.antennaList))

        # so, self.rawData is np.array(complex(nantennas, nsamples)
        self.recordTime = time.time()
        self.raw_data_available_from_this_period = True
        self.outstanding_request = False

        self.logger.debug("clrfreq record time: {}".format(self.recordTime))

    def get_raw_data(self):
        if self.rawData is None or not self.raw_data_available_from_this_period:
           self.record_new_data()
        else:
           print("clearFreqDataManager: provide raw data (age {}) ".format(time.time() - self.recordTime))
        return self.rawData, self.metaData, self.recordTime

class swingManager():
    """ Class to handle which swing is active and processing """

    def __init__(self):
        self.activeSwing       = 0
        self.processingSwing   = 1

        # async buffers for control program handlers
        # a/p swing status is switched in main loop async from crtl progam. this var is used for GetDataHandler to get the correct swing no matter when handler is called
        self.lastSwingWithData  = self.activeSwing  
        self.nextSwingToTrigger = self.activeSwing
    def reset(self):
        self.__init__() # for now reset is the same as init

    def switch_swings(self):
        self.activeSwing     = 1 - self.activeSwing
        self.processingSwing = 1 - self.processingSwing

class scanManager():
    """ Class to handle
        - last recorded clear frequency search raw data
        - keep track of beam numbers and transmit frequencies 
        created for each RadarChannelHandler """
        

    def __init__(self, restricted_frequency_list, beamSep, numBeams):
        self.scan_beam_list        = []
        self.clear_freq_range_list = []
        
        self.current_period = 0
        self.repeat_clrfreq_recording = False # 2nd period is triggered automatically before ROS finishes 1st. if CLR_FRQ was requested for 1st => also do record on 2nd
        
        self.beamSep = beamSep
        self.numBeams = numBeams

        self.current_clrFreq_result = None
        self.next_clrFreq_result    = None 
        self.isPrePeriod = True # is vert first trigger_next_period() call that just triggers first period but does not collect cuda data
        self.isPostLast = False # to handle last trigger_next_swing() call

        self.get_clr_freq_raw_data  = None # handle to RHM:ClearFrequencyRawDatamanager.get_raw_data()

        self.restricted_frequency_list = restricted_frequency_list
        self.logger = logging.getLogger('scanManager')

    def init_new_scan(self, freq_range_list, scan_beam_list):
  
        # list of [fstart, fstop] lists in Hz, desired frequency range for each period
        self.clear_freq_range_list = freq_range_list

        # list of [bmnum, bmnum..] 
        self.scan_beam_list = scan_beam_list

        # rest all other parameter
        self.current_period = 0
        self.current_clrFreq_result = None
        self.next_clrFreq_result    = None 
        self.isPrePeriod = True # is vert first trigger_next_period() call that just triggers first period but does not collect cuda data
        self.isPostLast = False # to handle last trigger_next_swing() call


    def switch_swings(self):
        # switch swings
        self.activeSwing     = 1 - self.activeSwing
        self.processingSwing = 1 - self.processingSwing
    
    # XXX store current freqencies in clrfreq manager,
    # add to restricted
    def period_finished(self):
      #  print("swing manager period finished... ")
        self.current_clrFreq_result = self.next_clrFreq_result
        self.next_clrFreq_result = None
        if self.isPrePeriod:
           self.isPrePeriod = False
           return

        if not self.isLastPeriod:
            self.current_period += 1
            self.logger.debug("Increasing current_period to {}".format(self.current_period))
        else:
            self.logger.debug("Last period current_period is still {}".format(self.current_period))
            self.isPostLast = True

        
    def status(self):
        print("current period: {: >2d}/{}, beam: {} ".format(self.current_period, len(self.scan_beam_list), self.current_beam))
    
    @property        
    def current_beam(self):
       # print("Requesting current beam for period {}".format(self.current_period))
        return self.scan_beam_list[self.current_period]
        
    @property        
    def next_beam(self):
        if self.current_period == len(self.scan_beam_list) -1:
            return None
        else:
            return self.scan_beam_list[self.current_period+1]

    def get_current_clearFreq_result(self):
        if self.current_clrFreq_result is None:
           # print("  calc current clr_freq (period {})".format(self.current_period))
            self.current_clrFreq_result = self.evaluate_clear_freq(self.current_period, self.current_beam)
        return self.current_clrFreq_result
        
    def get_next_clearFreq_result(self):
        if self.next_clrFreq_result is None:
           # print("  calc next clr_freq (period {})".format(self.current_period+1))
            self.next_clrFreq_result = self.evaluate_clear_freq(self.current_period+1,self.next_beam)
        return self.next_clrFreq_result        
        
    def evaluate_clear_freq(self, iPeriod, beamNo):
        rawData, metaData, recordTime = self.get_clr_freq_raw_data() 
        beam_angle = calc_beam_azm_rad(self.numBeams, beamNo, self.beamSep)
        

        self.logger.debug("clear_freq_range: {}".format(self.clear_freq_range_list[iPeriod]))

        clearFreq, noise = calc_clear_freq_on_raw_samples(rawData, metaData, self.restricted_frequency_list, self.clear_freq_range_list[iPeriod], beam_angle) 
        return (clearFreq, noise, recordTime)

    @property
    def isFirstPeriod(self): # TODO delete if unused
        return self.current_period == 0

    @property
    def isForelastPeriod(self):
        return self.current_period + 2 == len(self.scan_beam_list) or len(self.scan_beam_list) == 1 # for one beam scan: first is also forelast
        
    @property
    def isLastPeriod(self):
        return self.current_period + 1 == len(self.scan_beam_list) and len(self.scan_beam_list) != 1 # for one beam scan: first is not last
       
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
        self.mixingFreqManager       = None
        
        self.ini_file_init()
        self.usrp_init()
        self.rxfe_init()
     #   self.test_rxfe_control() # toggle all amp and att stages on and off for testing
        self.cuda_init()

        self.nRegisteredChannels = 0
        self.clearFreqRawDataManager = clearFrequencyRawDataManager(self.array_x_spacing)
        self.clearFreqRawDataManager.set_usrp_driver_connections(self.usrpManager.socks)

        self.logger.warning("RadarHardwareManager: hardcoded clear frequency search frequency!, modify code to use adaptive center freq")
        self.clearFreqRawDataManager.set_clrfreq_search_span(12.5e6, self.usrp_rf_rx_rate, self.usrp_rf_rx_rate / CLRFREQ_RES_HZ)
        self.newChannelList      = []
        self.record_new_data         = self.clearFreqRawDataManager.record_new_data
        self.swingManager            = swingManager()

        self.set_par_semaphore = posix_ipc.Semaphore('SET_PAR', posix_ipc.O_CREAT)
        self.set_par_semaphore.release()
        self.lastSwingInvalid = False
        self.trigger_next_function_running = False


    def run(self):
        def spawn_channel(conn):
            # start new radar channel handler
            channel = RadarChannelHandler(conn, self)
            try:
                channel.run()
            except socket.error:
                self.logger.error("RadarChannelHandler: Socket error => Deleting channel... ")
                self.unregister_channel_from_HardwareManager(channel)

        # TODO: add lock support
        def radar_main_control_loop():
            controlLoop_logger = logging.getLogger('Control Loop')
            controlLoop_logger.info('Starting RHM.radar_main_control_loop() ')
          
            sleepTime = 0.01 # used if control loop waits for one channel
           
            while True:

                # set start time of integration period (will be overwriten if not triggered)
                self.starttime_period = time.time()

                # check if there are any disconnected URSPs
                if len(self.usrpManager.addressList_inactive):
                   self.usrpManager.restore_lost_connections()

                # CLEAR FREQ SEARCH: recoring when ever requested (independent of swing, state or channel)
                if self.clearFreqRawDataManager.outstanding_request:
                    controlLoop_logger.debug('start self.clearFreqRawDataManager.record_new_data()')
                    self.clearFreqRawDataManager.record_new_data()

                    # check if CLR_FREQ has to be repeated
                    if CS_INACTIVE in [ch.active_state for ch in self.channels]:
                       controlLoop_logger.debug("Repeating CLR_FREQ for next integation period")
                       self.clearFreqRawDataManager.repeat_request_for_2nd_period = True              

                    controlLoop_logger.debug('end self.clearFreqRawDataManager.record_new_data()')


                # FRIST CUDA_ADD FOR NEW CHANNELS
                if len(self.newChannelList) != 0:
                   self.initialize_channel()
                           

                # TRIGGER if any channel is in CS_TRIGGER
                if CS_TRIGGER in [ch.active_state for ch in self.channels]:
                    # wait for all channels to be in TRIGGER state
                    executeTrigger = True
                    for ch in self.channels:
                        if  ch.active_state not in [CS_TRIGGER, CS_INACTIVE, CS_LAST_SWING]:
                            controlLoop_logger.debug('remaining in TRIGGER because channel {} state is {} (active swing is {})'.format(ch.cnum, ch.active_state, ch.swingManager.activeSwing))
                            time.sleep(sleepTime)
                            executeTrigger = False

                    # if all channels are TRIGGER, then TRIGGER
                    if executeTrigger:
                        controlLoop_logger.debug('start RHM.trigger_next_swing()')
                        self.trigger_next_swing()
                        controlLoop_logger.debug('end RHM.trigger_next_swing()')

                else:
                    time.sleep(RADAR_STATE_TIME) # sleep to reduce load of this while loop

        # end of radar_main_control_loop()


        self.client_sock.listen(MAX_CHANNELS)
        client_threads = []
        self.channels = []
        usrp_server_logger = logging.getLogger('usrp_server')

        ct = threading.Thread(target=radar_main_control_loop)
        ct.start()
        while True:
            usrp_server_logger.info('waiting for control program')
            client_conn, addr = self.client_sock.accept()

            usrp_server_logger.info('connection from control program, spawning channel handler thread')
            ct = threading.Thread(target=spawn_channel, args=(client_conn,))
            client_threads.append(ct)
            ct.start()
       
            # remove threads that are not alive
            # TODO: the client_sock.accept is probably blocking, this will only execute after connection of a new control program. is that the intended behavior? -jtk
            client_threads = [iThread for iThread in client_threads if iThread.is_alive()]

        self.client_sock.close()


    # read in ini config files..
    def ini_file_init(self):
        # READ driver_config.ini
        driver_config = configparser.ConfigParser()
        driver_config.read('../driver_config.ini')
        self.ini_shm_settings     = driver_config['shm_settings']
        self.ini_cuda_settings    = driver_config['cuda_settings']
        self.ini_network_settings = driver_config['network_settings']

        # READ usrp_config.ini
        usrp_config = configparser.ConfigParser()
        usrp_config.read('../usrp_config.ini')
        usrp_configs = []
        self.antenna_idx_list_main = []
        self.antenna_idx_list_back = []
        for usrp in usrp_config.sections():
            usrp_configs.append(usrp_config[usrp])
            if usrp_config[usrp]['mainarray'].lower() in ['true', 1]:
               self.antenna_idx_list_main.append(int(usrp_config[usrp]['array_idx']))
            else:
               self.antenna_idx_list_back.append(int(usrp_config[usrp]['array_idx']))
            
        self.ini_usrp_configs = usrp_configs

        # READ array_config.ini
        array_config = configparser.ConfigParser()
        array_config.read('../array_config.ini')
        self.ini_rxfe_settings  = array_config['rxfe']
        self.totalScalingFactor = float(array_config['gain_control']['total_scaling_factor'])

        self.ini_array_settings = array_config['array_info']
        self.array_beam_sep  = float(self.ini_array_settings['beam_sep'] ) # degrees
        self.array_nBeams    = int(  self.ini_array_settings['nbeams'] )
        self.array_x_spacing = float(self.ini_array_settings['x_spacing'] ) # meters 
        self.hardwareLimit_freqRange = [float(array_config['hardware_limits']['minimum_tfreq'] ) /1000, float(array_config['hardware_limits']['maximum_tfreq'] )/1000] # converted to kHz

    def usrp_init(self):
      self.usrpManager = usrpSockManager(self)
      self.usrp_rf_tx_rate   = int(self.ini_cuda_settings['FSampTX'])
      self.usrp_rf_rx_rate   = int(self.ini_cuda_settings['FSampRX'])
      self.mixingFreqManager = usrpMixingFreqManager(11500, self.usrp_rf_tx_rate/1000)

      self._resync_usrps()


    def _resync_usrps(self):
        usrps_synced = False

        while not usrps_synced:
            cmd = usrp_sync_time_command(self.usrpManager.socks)
            cmd.transmit()
            cmd.client_return()

            # once USRPs are connected, synchronize clocks/timers 
            cmd = usrp_get_time_command(self.usrpManager.socks)
            cmd.transmit() 

            usrptimes = []
            for usrpsock in self.usrpManager.socks:
                usrptimes.append(cmd.recv_time(usrpsock))
           
            cmd.client_return()
     
            # check if sync succeeded..
            if max(np.abs(np.array(usrptimes) - usrptimes[0])) < .5:
                usrps_synced = True
                print('USRPs synchronized, approximate times: ' + str(usrptimes))
            else:
                # TODO: why does USRP synchronization fail?
                self.logger.warning('_resync_USRP USRP syncronization failed, trying again..')
                time.sleep(1)

    #@timeit
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
        cmd = usrp_rxfe_setup_command(self.usrpManager.socks, amp1, amp2, att*2) # *2 since LSB is 0.5 dB 
        cmd.transmit()
        self.usrpManager.eval_client_return(cmd)


    def test_rxfe_control(self):
        """ Function that steps through all amp and att stages of the rxfe board """

        self.logger.warning("Starting RXFE test: walk through all bits:") 

        testParSet = [[False, False, 0], [True, False, 0], [True, True, 0], [False, True, 0], [True, True, 31.5]] + [[False, False, 2**i/2] for i in range(6) ]
        
        nSets = len(testParSet)
        for iSet in range(nSets):
            amp1 = testParSet[iSet][0]
            amp2 = testParSet[iSet][1]
            att  = testParSet[iSet][2]

            cmd = usrp_rxfe_setup_command(self.usrpManager.socks, amp1, amp2, att*2) # *2 since LSB is 0.5 dB 
            cmd.transmit()
            cmd.client_return()
            self.logger.warning("Current settings: Amp1={}, Amp2={}, Attenuation={} dB".format(amp1, amp2, att)) 
            input("  Press Enter for next chage...")

        print("Finished testing RXFE!")

    def cuda_init(self):
        #time.sleep(.05)

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
  
    def initialize_channel(RHM):
        """ Adds first period of channel for new channel or after CS_INACTIVE. Also appends channel to RHM.channels if not already done."""
        RHM.gain_control_divide_by_nChannels()

        RHM.set_par_semaphore.acquire()
        RHM.logger.debug("start initialize_channel")
        newChannelList = RHM.newChannelList.copy()  #  make a copy in case another channel is added during this function call
        RHM._calc_period_details(newChannels=newChannelList) # TODO only if this is first channel?
        for channel in newChannelList:
     
            # CUDA_ADD_CHANNEL in first period
            cmd = cuda_add_channel_command(RHM.cudasocks, sequence=channel.get_current_sequence(), swing = channel.swingManager.activeSwing)
            RHM.logger.debug('start CUDA_ADD_CHANNEL (cnum {}, swing {})'.format(channel.cnum, channel.swingManager.activeSwing))
            cmd.transmit()
            cmd.client_return()      
            if channel.active_state == CS_INACTIVE: 
               RHM.logger.debug("initialize_channel() is setting ch {} swing {} from CS_INACTIVE to CS_READY".format(channel.cnum,  channel.swingManager.activeSwing))
               channel.active_state = CS_READY # channel not really ready until CUDA_GENERATE, but there will be no trigger in parallel to this function
            else:
               RHM.logger.debug("initialize_channel() ch {} state stays {}".format(channel.cnum, channel.active_state))


            if channel not in RHM.channels:
               RHM.channels.append(channel)
               RHM.logger.debug("Adding channel {} to RHM.channels".format(channel.cnum))
            RHM.newChannelList.remove(channel)

        # CUDA_GENERATE for first period
        RHM.logger.debug('start CUDA_GENERATE_PULSE (1st period)')
        cmd = cuda_generate_pulse_command(RHM.cudasocks, RHM.swingManager.activeSwing, RHM.mixingFreqManager.current_mixing_freq*1000)
        cmd.transmit()
        cmd.client_return()
        RHM.logger.debug('end CUDA_GENERATE_PULSE (1st period)')
        RHM.logger.debug("end initialize_channel")

        RHM.set_par_semaphore.release()


    def unregister_channel_from_HardwareManager(self, channelObject):
        if channelObject in self.channels:
       # this is only called if something went wrong or crtl program quit => so don't care about channel states ? 
       #    # don't delete channel in middle of trigger, pretrigger, or ....
       #     channelObject._waitForState([CS_READY, CS_INACTIVE])  
            self.logger.info('unregister_channel_from_HardwareManager() removing channel {} from HardwareManager'.format(self.channels.index(channelObject)))
            self.channels.remove(channelObject)
            # remove channel from cuda
            self.logger.debug('send CUDA_REMOVE_CHANNEL')
            for iSwing in range(nSwings):
               try:
                   cmd = cuda_remove_channel_command(self.cudasocks, sequence=channelObject.get_current_sequence(), swing = iSwing)
                   cmd.transmit()
                   cmd.client_return()
               except AttributeError:
                   # catch errors where channel.getSequence() fails because npulses_per_sequence is uninitialized
                   # TODO: discover why this happens..
                   self.logger.error('unregister_channel_from_HardwareManager() failed to remove channel from HardwareManager')
                
            self.logger.debug('RHM:unregister_channel_from_HardwareManager {} channels left'.format(len(self.channels)))

            self.nRegisteredChannels -= 1
            if self.nRegisteredChannels == 0:  
                self.commonChannelParameter = {}

        else:
            self.logger.warning('unregister_channel_from_HardwareManager() channel already deleted')


        

    def exit(self):
        # clean up and exit
        self.client_sock.close()
        cmd = usrp_exit_command(self.usrpManager.socks)
        cmd.transmit()

        cmd = cuda_exit_command(self.cudasocks)
        cmd.transmit()

        for sock in self.usrpManager.socks:
            sock.close()
        for sock in self.cudasocks:
            sock.close()

        self.logger.info('received exit command, cleaning up..')

        sys.exit(0)

    def _calc_period_details(self, newChannels=[]):
        """ calculate details for integration period and save it in channel objects"""

         
#        sequence_length = self.commonChannelParameter['pulse_sequence_offsets_vector'][-1] 
#        pulse_sequence_period = (PULSE_SEQUENCE_PADDING_TIME + sequence_length)

        # calculate the pulse sequence period with padding
        nSamples_per_sequence = self.commonChannelParameter['number_of_samples'] + int(PULSE_SEQUENCE_PADDING_TIME / self.commonChannelParameter['baseband_samplerate'])
        pulse_sequence_period = nSamples_per_sequence / self.commonChannelParameter['baseband_samplerate']  

        # to find out how much time is available in an integration period for pulse sequences, subtract out startup delay
        transmitting_time_left = self.starttime_period + self.commonChannelParameter['integration_period_duration'] - time.time() - INTEGRATION_PERIOD_SYNC_TIME

        # calculate the number of pulse sequences that fit in the available time within an integration period
        nSequences_per_period = int(transmitting_time_left / pulse_sequence_period)
###        sampling_duration = pulse_sequence_period * nSequences_per_period   # just record full number of sequences


        # calculate the number of RF transmit and receive samples 
        nSamples_per_sequence_if =  int(self.ini_cuda_settings['IFBBRATE'])* ((nSamples_per_sequence*nSequences_per_period) - 1 ) +  int(self.ini_cuda_settings['NTapsRX_ifbb']) 
        num_requested_rx_samples =  int(self.ini_cuda_settings['RFIFRATE'])* (nSamples_per_sequence_if                      - 1 ) +  int(self.ini_cuda_settings['NTapsRX_rfif'])

        self.logger.debug("Effective integration time: {:0.3f}s = {} sequences ({}s) swing {}".format(num_requested_rx_samples /self.usrp_rf_tx_rate, nSequences_per_period,  self.commonChannelParameter['integration_period_duration'], self.swingManager.activeSwing))
        self.nsamples_per_sequence     = pulse_sequence_period * self.usrp_rf_tx_rate

        # XXX
        if num_requested_rx_samples < nSequences_per_period * self.nsamples_per_sequence:
            self.logger.error("number of requested samples is too low for integration period!")
            sys.exit(0)

        # then calculate sample indicies at which pulse sequences start within a pulse sequence
        nPulses_per_sequence           = self.commonChannelParameter['npulses_per_sequence']
        pulse_sequence_offsets_samples = self.commonChannelParameter['pulse_sequence_offsets_vector'] * self.usrp_rf_tx_rate

        # then, calculate sample indicies at which pulses start within an integration period
        integration_period_pulse_sample_offsets = np.zeros(nPulses_per_sequence *  nSequences_per_period, dtype=np.uint64)
        for iSequence in range(nSequences_per_period):
            for iPulse in range(nPulses_per_sequence):
                integration_period_pulse_sample_offsets[iSequence * nPulses_per_sequence + iPulse] = iSequence * self.nsamples_per_sequence + pulse_sequence_offsets_samples[iPulse]

        self.nPulses_per_integration_period = nPulses_per_sequence * nSequences_per_period
     
        if True:
           self.logger.debug(" > nSamples_per_sequence (bb) {} ".format(nSamples_per_sequence ))
           self.logger.debug(" > nSamples_per_sequence (if) {} ".format(nSamples_per_sequence_if ))
           self.logger.debug(" > num_requested_rx_samples {}".format(num_requested_rx_samples ))
           self.logger.debug(" > nSequences_per_period {} ".format(nSequences_per_period ))

        # inform all channels with the number of pulses per integration period        
        for ch in self.channels+newChannels:
            ch.nSequences_per_period                 = nSequences_per_period
            ch.nrf_rx_samples_per_integration_period = num_requested_rx_samples
            ch.nbb_rx_samples_per_sequence           = nSamples_per_sequence  
            print(ch.nbb_rx_samples_per_sequence)
            print(pulse_sequence_period)
            print(self.commonChannelParameter['baseband_samplerate'])
            assert abs(ch.nbb_rx_samples_per_sequence - pulse_sequence_period * self.commonChannelParameter['baseband_samplerate']) < 1e4 / self.commonChannelParameter['baseband_samplerate'], 'pulse sequences lengths must be a multiple of the baseband sampling rate'
            ch.integration_period_pulse_sample_offsets = integration_period_pulse_sample_offsets
        self.nSequences_per_period = nSequences_per_period
            
        

    #@timeit
    def trigger_next_swing(self):
        self.trigger_next_function_running = True
        self.logger.debug('running RHM.trigger_next_swing()')
        swingManager = self.swingManager
     
        self.logger.warning('TODO: where do we want to do gain control?')
        self.gain_control_divide_by_nChannels()
        
        self._calc_period_details()

        nSamples_per_pulse = int(self.commonChannelParameter['pulseLength'] / 1e6 * self.usrp_rf_tx_rate) + 2 * int(self.commonChannelParameter['tr_to_pulse_delay']/1e6 * self.usrp_rf_tx_rate)
        self.logger.debug("nSamples_per_pulse: ".format(nSamples_per_pulse))
        for ch in self.channels:
            if ch.ctrlprm_struct.payload['tfreq'] != 0: 
                self.logger.debug('pretrigger() tfreq: {}, rfreq: {}. usrp_center: tx={} rx={} '.format( ch.ctrlprm_struct.payload['tfreq'], ch.ctrlprm_struct.payload['rfreq'], self.mixingFreqManager.current_mixing_freq, self.usrp_rf_tx_rate))
                if not (ch.ctrlprm_struct.payload['tfreq'] == ch.ctrlprm_struct.payload['rfreq']):
                    self.logger.warning('pretrigger() tfreq (={}) != rfreq (={}) !'.format( ch.ctrlprm_struct.payload['tfreq'], ch.ctrlprm_struct.payload['rfreq']))
        
        # look for one active channel
        transmittingChannelAvailable = False
        for tmpChannel in self.channels:
           if (tmpChannel is not None) and (not tmpChannel.scanManager.isLastPeriod): 
              channel = tmpChannel
              transmittingChannelAvailable = True
              break

        self.swingManager.nextSwingToTrigger = self.swingManager.processingSwing
        self.logger.debug("setting nextSwingToTrigger to swing {}".format(self.swingManager.nextSwingToTrigger))

        if transmittingChannelAvailable:
           # USRP SETUP
           self.logger.debug('triggering period no {}'.format(channel.scanManager.current_period))
           self.logger.debug("start USRP_SETUP")
           cmd = usrp_setup_command(self.usrpManager.socks, self.mixingFreqManager.current_mixing_freq*1000, self.mixingFreqManager.current_mixing_freq*1000, self.usrp_rf_tx_rate, self.usrp_rf_rx_rate, \
                                    self.nPulses_per_integration_period,  channel.nrf_rx_samples_per_integration_period, nSamples_per_pulse, channel.integration_period_pulse_sample_offsets, swingManager.activeSwing)
           cmd.transmit()
           self.usrpManager.eval_client_return(cmd)
           self.logger.debug("end USRP_SETUP")


           # USRP_TRIGGER
           self.logger.debug("start USRP_GET_TIME")
           cmd = usrp_get_time_command(self.usrpManager.socks[0]) # grab current usrp time from one usrp_driver 
           cmd.transmit()
          
    ##       self.swingManager.nextSwingToTrigger = self.swingManager.processingSwing
    ##     self.logger.debug("setting nextSwingToTrigger to swing {}".format(self.swingManager.nextSwingToTrigger))

           # TODO: tag time using a better source? this will have a few hundred microseconds of uncertainty
           # maybe measure offset between usrp time and computer clock time somewhere, then calculate from there
           usrp_integration_period_start_clock_time = time.time() + INTEGRATION_PERIOD_SYNC_TIME
           usrp_time = cmd.recv_time(self.usrpManager.socks[0])
           cmd.client_return()
           self.logger.debug("end USRP_GET_TIME")


           # calculate sequence times for control program
           sequence_start_time_secs  = np.zeros(self.nSequences_per_period, dtype=np.uint64)
           sequence_start_time_usecs = np.zeros(self.nSequences_per_period, dtype=np.uint32)
           for iSequence in range(self.nSequences_per_period):
               pulse_start_time = usrp_integration_period_start_clock_time + iSequence * self.nsamples_per_sequence / self.usrp_rf_rx_rate
               sequence_start_time_secs[iSequence]  = int(pulse_start_time) 
               sequence_start_time_usecs[iSequence] = int(( pulse_start_time - int(pulse_start_time) ) *1e6)

           # save data for returning results to control program
           resultDict = {}
           resultDict['sequence_start_time_secs']      = sequence_start_time_secs
           resultDict['sequence_start_time_usecs']     = sequence_start_time_usecs
           resultDict['number_of_samples']             = self.commonChannelParameter['number_of_samples']
           resultDict['nSequences_per_period']         = self.nSequences_per_period       
           resultDict['pulse_sequence_offsets_vector'] = self.commonChannelParameter['pulse_sequence_offsets_vector'] 
           resultDict['npulses_per_sequence']          = self.commonChannelParameter['npulses_per_sequence']
           resultDict['results_are_valid']             = True
           for channel in self.channels: 
               resultDict['nbb_rx_samples_per_sequence'] = channel.nbb_rx_samples_per_sequence
               resultDict['pulse_lens']                   = channel.pulse_lens    
               channel.resultDict_list.insert(0,resultDict)
    
           # broadcast the start of the next integration period to all usrp
           self.logger.debug('start USRP_TRIGGER')
           trigger_time = usrp_time + INTEGRATION_PERIOD_SYNC_TIME 
           cmd = usrp_trigger_pulse_command(self.usrpManager.socks, trigger_time, self.commonChannelParameter['tr_to_pulse_delay'], swingManager.activeSwing) 
           self.logger.debug('sending trigger pulse command')
           cmd.transmit()

           self.logger.debug('current usrp time: {}, trigger time of: {}'.format(usrp_time, trigger_time))
           # set state of channel to CS_PROCESSING
           for ch in self.channels:
               if ch.active_state == CS_TRIGGER:
                  ch.active_state = CS_PROCESSING
                  ch.logger.debug("Changing active channel state from CS_TRIGGER to CS_PROCESSING (cnum: {}, swing {}, period {})".format(ch.cnum, self.swingManager.activeSwing, ch.scanManager.current_period))
           self.logger.debug('waiting for trigger return')
           returns = self.usrpManager.eval_client_return(cmd)

           if TRIGGER_BUSY in returns:
               self.logger.error('could not trigger, usrp driver is busy')
               pdb.set_trace()
           self.logger.debug('end USRP_TRIGGER')
        else:
           self.logger.debug('No tranmitting channles available. Skipping USRP_TRIGGER')

        allProcessingChannelStates = [ch.processing_state for ch in self.channels]
        if self.lastSwingInvalid: # TODO check if this works (in control program or data files)
           self.logger.warning("Last swin has been invalid. Preparing 0 sequences to transmit")
           for iChannel, channel in enumerate(self.channels):
              if channel.processing_state is CS_PROCESSING:
                 channel.update_ctrlprm_class("current")
                 channel.resultDict_list[-1]['ctrlprm_dataqueue'] = copy.deepcopy(channel.ctrlprm_struct.dataqueue)
                 channel.resultDict_list[-1]['results_are_valid'] = False
                 channel.resultDict_list[-1]['nSequences_per_period'] = 0


                 for item in channel.ctrlprm_struct.dataqueue:
                    if item.name == 'rbeam':
                       channel.logger.debug("saving dataqueue to resultDict (rbeam={})".format(item.data))
           self.lastSwingInvalid = False
        else:
           if CS_PROCESSING in allProcessingChannelStates:
               # CUDA_GET_DATA
               self.logger.debug('start CUDA_GET_DATA')
               cmd = cuda_get_data_command(self.cudasocks, swingManager.processingSwing)
               cmd.transmit()

               nMainAntennas = len(self.antenna_idx_list_main)
               nBackAntennas = len(self.antenna_idx_list_back)
               main_samples = None


               for cudasock in self.cudasocks:
                   nAntennas = recv_dtype(cudasock, np.uint32)
                   for iChannel,channel in enumerate(self.channels):

                       if channel.processing_state == CS_PROCESSING:
                           transmit_dtype(cudasock, channel.cnum, np.int32)

                           for iAntenna in range(nAntennas):
                               antIdx = recv_dtype(cudasock, np.uint16)
                               nSamples_bb = int(recv_dtype(cudasock, np.uint32) / 2)
                               self.logger.debug("Receiving {} bb samples.".format(nSamples_bb))
                               if main_samples is None:
                                  main_samples = np.zeros((len(self.channels), nMainAntennas, nSamples_bb), dtype=np.complex64)
                                  back_samples = np.zeros((len(self.channels), nBackAntennas, nSamples_bb), dtype=np.complex64)


                            #   self.logger.warning('CUDA_GET_DATA: stalling for 100 ms to avoid a race condition')
                            #   time.sleep(.1)
                               samples = recv_dtype(cudasock, np.float32, nSamples_bb * 2)
                               samples = samples[0::2] + 1j * samples[1::2] # unpacked interleaved i/q
                               
                               if antIdx in self.antenna_idx_list_main:
                                   iAntenna = self.antenna_idx_list_main.index(antIdx)
                                   main_samples[iChannel][iAntenna] = samples[:]

                               else:
                                   iAntenna = self.antenna_idx_list_back.index(antIdx)
                                   back_samples[iChannel][iAntenna] = samples[:]
                                           
             
                   transmit_dtype(cudasock, -1, np.int32) # to end transfer process
                   
               cmd.client_return()
               self.logger.debug('end CUDA_GET_DATA')


               # BEAMFORMING
               self.logger.debug('start rx beamforming')
               beamformed_main_samples, beamformed_back_samples = self.calc_beamforming( main_samples, back_samples)
               for iChannel, channel in enumerate(self.channels):
                  if channel.processing_state is CS_PROCESSING:
                     # copy samples and ctrlprm to transmit later to control program
                     if 'main_beamformed' in channel.resultDict_list[-1]:
                        channel.logger.error("Main beamformed data already exist. Overwriting it. This is not correct. GetDataHandler too slow??")
                     channel.resultDict_list[-1]['main_beamformed'] = beamformed_main_samples[iChannel]
                     channel.resultDict_list[-1]['back_beamformed'] = beamformed_back_samples[iChannel]
                     channel.update_ctrlprm_class("current")
                     channel.resultDict_list[-1]['ctrlprm_dataqueue'] = copy.deepcopy(channel.ctrlprm_struct.dataqueue)
                     for item in channel.ctrlprm_struct.dataqueue:
                        if item.name == 'rbeam':
                           channel.logger.debug("saving dataqueue to resultDict (rbeam={})".format(item.data))

               self.logger.debug('end rx beamforming')

               if os.path.isfile("./bufferLiveData.flag"):
                  self.logger.info("Buffering raw data to disk.")
                  with open('tmpRawData.pkl', 'wb') as f:
                     pickle.dump([main_samples, back_samples, channel.resultDict_list[-1]],f,  pickle.HIGHEST_PROTOCOL)
                  os.rename("tmpRawData.pkl", "liveRawData.pkl")
                  os.remove("./bufferLiveData.flag")
           else:
              self.logger.debug('No processing channles available. Skipping CUDA_GET_DATA and rx beamforming')


        # PERIOD FINISHED        
        self.next_period_RHM()

        # update (next) states
        for iChannel, channel in enumerate(self.channels):
           if channel.processing_state is CS_PROCESSING:

              # determine next state after samples are returned to control program
              if channel.scanManager.isPostLast: # or channel.scanManager.isForelastPeriod:
                 channel.next_processing_state = CS_INACTIVE
                 channel.active_state          = CS_INACTIVE
                 channel.next_active_state     = CS_INACTIVE
                 channel.logger.debug('last period finished, setting active and next processing state to CS_INACTIVE')
              elif channel.scanManager.isLastPeriod:
                 channel.next_processing_state = CS_LAST_SWING
              else:
                 channel.next_processing_state = CS_READY 
              channel.logger.debug("Switching next processing state (swing {}) of cnum {} to {}".format(self.swingManager.processingSwing, channel.cnum, channel.next_processing_state )) 

              channel.logger.debug("Switching processing state (swing {}) state of cnum {} from CS_PROCESSING to CS_SAMPLES_READY".format(self.swingManager.processingSwing, channel.cnum )) 
              channel.processing_state = CS_SAMPLES_READY
 
        # CUDA_ADD & CUDA_GENGERATE for processingSwing 
        for channel in self.channels:
            if channel.scanManager.isLastPeriod: # or channel.scanManager.isForelastPeriod:
               self.logger.debug("start CUDA_REMOVE_CHANNEL")
               cmd = cuda_remove_channel_command(self.cudasocks, sequence=channel.get_current_sequence(), swing = swingManager.processingSwing) 
               self.logger.debug('send CUDA_REMOVE_CHANNEL (cnum {}, swing {})'.format(channel.cnum, swingManager.processingSwing))
               cmd.transmit()
               cmd.client_return()      
               self.logger.debug("end CUDA_REMOVE_CHANNEL")
            else:
               if channel.active:
                  self.logger.debug("start CUDA_ADD_CHANNEL")
                  cmd = cuda_add_channel_command(self.cudasocks, sequence=channel.get_next_sequence(), swing = swingManager.processingSwing) 
                  self.logger.debug('send CUDA_ADD_CHANNEL (cnum {}, swing {})'.format(channel.cnum, swingManager.processingSwing))
                  cmd.transmit()
                  cmd.client_return()      
                  self.logger.debug("end CUDA_ADD_CHANNEL")
   
                  if channel.processing_state == CS_INACTIVE: # first use of swing 1
                      channel.processing_state = CS_READY
                      channel.logger.debug("Switching processing state (swing {}) state of cnum {} to CS_READY (first use of swing 1)".format(self.swingManager.processingSwing, channel.cnum )) 
               else:
                  self.logger.debug("ch {}: channel not active => not calling CUDA_ADD".format(channel.cnum))
                  self.logger.error("When is this happening and is this okay???")
 

        # CUDA_GENERATE for first period
        synthNewPulses = True # TODO keep track of changes to do this only if necessary
        if synthNewPulses:
           self.logger.debug('start CUDA_GENERATE_PULSE')
           cmd = cuda_generate_pulse_command(self.cudasocks, swingManager.processingSwing, self.mixingFreqManager.current_mixing_freq*1000)
           cmd.transmit()
           cmd.client_return()
           self.logger.debug('end CUDA_GENERATE_PULSE')

        if transmittingChannelAvailable:
           # USRP_READY_DATA for activeSwing 
           self.logger.debug('start USRP_READY_DATA')
           cmd = usrp_ready_data_command(self.usrpManager.socks, swingManager.activeSwing)
           cmd.transmit()
    
           # check status of usrp drivers
           self.logger.debug('start receiving all USRP status')
           payloadList = self.usrpManager.eval_client_return(cmd, fcn=cmd.receive_all_metadata)
           self.logger.debug('end receiving all USRP status')

           for iUSRP, ready_return in enumerate(payloadList):
               if ready_return == CONNECTION_ERROR:
                  self.usrpManager.fault_status[iUSRP] = True
                  self.logger.error('connection to USRP broke in GET_DATA')
               else: 
                  rx_status                = ready_return['status']
                  if rx_status == -1:
                     self.logger.error("Error occurred in rx_worker. TODO: delete data oder mute USRP? ") # TODO
                  self.usrpManager.fault_status[iUSRP] = ready_return["fault"]
   
                  self.logger.debug('GET_DATA rx status {}'.format(rx_status))
                  if rx_status != 2:
                      self.logger.error('USRP driver status {} in GET_DATA'.format(rx_status))
                      #status = USRP_DRIVER_ERROR # TODO: understand what is an error here..
   
           self.logger.debug('start waiting for USRP_DATA return')
           self.usrpManager.eval_client_return(cmd)
           self.logger.debug('end waiting for USRP_DATA return')

           self.logger.debug('end USRP_READY_DATA')


        # SWITCH SWINGS
        swingManager.switch_swings()
        self.logger.debug('switching swings to: active={}, processing={}'.format(self.swingManager.activeSwing, self.swingManager.processingSwing))
  
        if transmittingChannelAvailable:
           # CUDA_PROCESS for processingSwing
           self.logger.debug('start CUDA_PROCESS')
           cmd = cuda_process_command(self.cudasocks, swingManager.processingSwing)
           cmd.transmit()
           cmd.client_return()
           self.logger.debug('end CUDA_PROCESS')

           # repeat CLR_FREQ record for 2nd period (if executed for 1st)
           if self.clearFreqRawDataManager.repeat_request_for_2nd_period:
              self.logger.debug("Setting outstanding_request for CLR_FREQ for 2nd period.")
              self.clearFreqRawDataManager.repeat_request_for_2nd_period = False
              self.clearFreqRawDataManager.outstanding_request = True
              
           # automatic trigger of second period (without ROS:SET_READY)
           for channel in  self.channels:
              if channel.scanManager.current_period == 0:  # first period 
                 channel.logger.debug('setting active state (cnum {}, swing {}) to CS_TRIGGER to start second period'.format(channel.cnum, self.swingManager.activeSwing))
                 channel.active_state = CS_TRIGGER
 
        self.trigger_next_function_running = False
        

    def next_period_RHM(self):
        self.clearFreqRawDataManager.period_finished()
        for ch in self.channels:
            if ch is not None: 
          #    if (not ch.scanManager.isFirstPeriod):
          #    if (not ch.scanManager.isLastPeriod) and (not ch.scanManager.isFirstPeriod):
                  ch.scanManager.period_finished()


    def gain_control_divide_by_nChannels(self):
        nChannels = len(self.channels)
        for ch in self.channels:
            ch.channelScalingFactor = 1 / nChannels * self.totalScalingFactor 

    # BEAMFORMING
    def calc_beamforming(RHM, main_samples, back_samples):
        RHM.logger.warning("TODO process back array! where to split from main array??")
        nSamples = main_samples.shape[2]
        beamformed_main_samples = np.zeros((len(RHM.channels), nSamples), dtype=np.int32)
        beamformed_back_samples = np.zeros((len(RHM.channels), nSamples), dtype=np.int32)
        debugPlot = False
    
        for iChannel, channel in enumerate(RHM.channels):
            if channel.processing_state is CS_PROCESSING:
                bmazm         = calc_beam_azm_rad(RHM.array_nBeams, channel.scanManager.current_beam, RHM.array_beam_sep)    # calculate beam azimuth from transmit beam number          
                channel.logger.debug("rx beamforming: ch {}, beam {}".format(channel.cnum, channel.scanManager.current_beam))
                clrFreqResult = channel.scanManager.get_current_clearFreq_result()
                pshift        = calc_phase_increment(bmazm, clrFreqResult[0] * 1000., RHM.array_x_spacing)       # calculate antenna-to-antenna phase shift for steering at a frequency        
                
                # MAIN ARRAY
                phasing_matrix = np.matrix([rad_to_rect(ant_idx * pshift) for ant_idx in RHM.antenna_idx_list_main])  # calculate a complex number representing the phase shift for each antenna
                complex_float_samples = phasing_matrix * np.matrix(main_samples[iChannel]) 
                real_mat = np.real(complex_float_samples)
                imag_mat = np.imag(complex_float_samples)
                maxInt16value = np.iinfo(np.int16).max # 32767
                minInt16value = np.iinfo(np.int16).min # -32768

                if (real_mat > maxInt16value).any() or (real_mat < minInt16value).any() or (imag_mat > maxInt16value).any() or (imag_mat < minInt16value).any():
                   RHM.logger.error("Overflow error while casting beamformed rx samples to complex int16s.")
                   OverflowError("calc_beamforming: overflow error in casting data to complex int")
                   real_mat = np.clip(real_mat, minInt16value, maxInt16value)
                   imag_mat = np.clip(imag_mat, minInt16value, maxInt16value)
                complexInt32_pack_mat = (np.int32(np.int16(real_mat)) << 16) + np.int16(imag_mat) 
                beamformed_main_samples[iChannel] = complexInt32_pack_mat.tolist()[0]

                if debugPlot:
                   import matplotlib.pyplot as plt
                  # plt.figure()
                  # plt.plot(np.transpose(main_samples[iChannel]))
                  # plt.title("raw")
                   plt.figure()
                   plt.subplot(2,1,1)
                   plt.plot(real_mat.tolist()[0])
                   plt.plot(imag_mat.tolist()[0])
                   plt.title("Main array")

                # BACK ARRAY (same as middle of main array, ant 16 = ant 6, ...)
                phasing_matrix = np.matrix([rad_to_rect((ant_idx-10) * pshift) for ant_idx in RHM.antenna_idx_list_back])  # calculate a complex number representing the phase shift for each antenna
                complex_float_samples = phasing_matrix * np.matrix(back_samples[iChannel]) 
                real_mat = np.real(complex_float_samples)
                imag_mat = np.imag(complex_float_samples)
                if (real_mat > maxInt16value).any() or (real_mat < minInt16value).any() or (imag_mat > maxInt16value).any() or (imag_mat < minInt16value).any():
                   RHM.logger.error("Overflow error while casting beamformed rx samples to complex int16s.")
                   OverflowError("calc_beamforming: overflow error in casting data to complex int")
                   real_mat = np.clip(real_mat, minInt16value, maxInt16value)
                   imag_mat = np.clip(imag_mat, minInt16value, maxInt16value)
                complexInt32_pack_mat = (np.int32(np.int16(real_mat)) << 16) + np.int16(imag_mat) 
                beamformed_back_samples[iChannel] = complexInt32_pack_mat.tolist()[0]
                if debugPlot:
                   import matplotlib.pyplot as plt
                   plt.subplot(2,1,2)
                   plt.plot(real_mat.tolist()[0])
                   plt.plot(imag_mat.tolist()[0])
                   plt.title("Back array")
                   plt.show()


        return beamformed_main_samples, beamformed_back_samples


class RadarChannelHandler:
    def __init__(self, conn, parent_RadarHardwareManager):
        self.conn = conn
        self.update_channel = True # flag indicating a new beam or pulse sequence 
        self.parent_RadarHardwareManager = parent_RadarHardwareManager
        self.logger = logging.getLogger("ChManager")
        self.state      = [CS_INACTIVE, CS_INACTIVE]
        self.next_state = [CS_INACTIVE, CS_INACTIVE]

        self.ctrlprm_struct = ctrlprm_struct(self.conn)
        self.seqprm_struct  = seqprm_struct(self.conn)
        self.clrfreq_struct = clrfreqprm_struct(self.conn)
        self.rprm_struct    = rprm_struct(self.conn)
        self.dataprm_struct = dataprm_struct(self.conn)

        self.channelScalingFactor = 0
        self.cnum = 'unknown'
        self.resultDict_list = []

        self.scanManager  = scanManager(read_restrict_file(RESTRICT_FILE), self.parent_RadarHardwareManager.array_beam_sep, self.parent_RadarHardwareManager.array_nBeams)
        self.scanManager.get_clr_freq_raw_data = self.parent_RadarHardwareManager.clearFreqRawDataManager.get_raw_data
        self.swingManager = parent_RadarHardwareManager.swingManager # reference to global swingManager of RadarHardwareManager
        

# QUICK ACCESS TO CURRENT/NEXT ACTIVE/PROCESSING STATE
    @property
    def active_state(self):
        return self.state[self.swingManager.activeSwing]
    @active_state.setter
    def active_state(self,value):
        self.state[self.swingManager.activeSwing] = value
    @property
    def processing_state(self):
        return self.state[self.swingManager.processingSwing]
    @processing_state.setter
    def processing_state(self, value):
        self.state[self.swingManager.processingSwing] = value
    @property
    def next_active_state(self):
        return self.next_state[self.swingManager.activeSwing]
    @next_active_state.setter
    def next_active_state(self,value ):
        self.next_state[self.swingManager.activeSwing] = value
    @property
    def next_processing_state(self):
        return self.next_state[self.swingManager.processingSwing]
    @next_processing_state.setter
    def next_processing_state(self, value):
        self.next_state[self.swingManager.processingSwing] = value 
 

    def run(self):
        rmsg_handlers = {\
            SET_RADAR_CHAN       : self.SetRadarChanHandler,\
            SET_INACTIVE         : self.SetInactiveHandler,\
            SET_ACTIVE           : self.SetActiveHandler,\
            QUERY_INI_SETTINGS   : self.QueryIniSettingsHandler,\
        #   GET_SITE_SETTINGS    : self.GetSiteSettingsHandler, \
        #   UPDATE_SITE_SETTINGS : self.UpdateSiteSettingsHandler,\
            GET_PARAMETERS       : self.GetParametersHandler,\
            SET_PARAMETERS       : self.SetParametersHandler,\
            PING                 : self.PingHandler,\
        #   OKAY                 : self.OkayHandler,\
        #   NOOP                 : self.NoopHandler,\
            QUIT                 : self.QuitHandler,\
            REGISTER_SEQ         : self.RegisterSeqHandler,\
        #   REMOVE_SEQ           : self.RemoveSeqHandler,\
            REQUEST_ASSIGNED_FREQ: self.RequestAssignedFreqHandler,\
            REQUEST_CLEAR_FREQ_SEARCH: self.RequestClearFreqSearchHandler,\
            LINK_RADAR_CHAN      : self.LinkRadarChanHandler,\
            SET_READY_FLAG       : self.SetReadyFlagHandler,\
            UNSET_READY_FLAG     : self.UnsetReadyFlagHandler,\
        #   SET_PROCESSING_FLAG  : self.SetProcessingFlagHandler,\
        #   UNSET_PROCESSING_FLAG: self.UnsetProcessingFlagHandler,\
        #   WAIT_FOR_DATA        : self.WaitForDataHandler,\
            GET_DATA             : self.GetDataHandler}


        while True:
            rmsg = rosmsg_command(self.conn)
            status = RMSG_FAILURE

            self.logger.debug('ch {}: waiting for command'.format(self.cnum))
            rmsg.receive(self.conn)
            command = chr(rmsg.payload['type'] & 0xFF) # for some reason, libtst is sending out 4 byte commands with junk..
            try:
                self.logger.debug('ch {}: received command (ROS=>USRP_Server): {}, {}'.format(self.cnum, command, RMSG_COMMAND_NAMES[command]))
            except KeyError:
                self.logger.error(command)
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
    def _waitForState(self, swing, state):
        if type(state) is not list:
            state = [state]

        wait_start = time.time()
        counter = 0
        while self.state[swing] not in state:
            counter = (counter + 1) % 10000
#            if state[0] == CS_SAMPLES_READY and counter == 1:
            if counter == 1:
               self.logger.debug("ch {}:_waitForState {}. state is {} (swing {})".format(self.cnum, state, self.state[swing], swing))
            time.sleep(RADAR_STATE_TIME)
            if time.time() - wait_start > CHANNEL_STATE_TIMEOUT:
                self.logger.error('CHANNEL STATE TIMEOUT for channel {}'.format(self.cnum))
                pdb.set_trace()
                break
    
    def update_ctrlprm_class(self, period):
        if len(self.scanManager.scan_beam_list) == 0:
           self.logger.warning("scan_beam_list is empty. unable to update ctrlprm_class!")
           return

        if period == "current":
           beam = self.scanManager.current_beam
           freq = self.scanManager.get_current_clearFreq_result()[0]
        elif period == "next":
           beam = self.scanManager.next_beam
           freq = self.scanManager.get_next_clearFreq_result()[0]
        else:
            self.logger.error("unknown period specifier: {} (valid: current or next )".format(period)) 
       
        parNameList  = ['rbeam', 'tbeam', 'rfreq', 'tfreq'] 
        parValueList = [ beam  ,  beam  ,  freq  ,  freq  ]
     
        for iPar, par in enumerate(parNameList):
           self.ctrlprm_struct.set_data(par, parValueList[iPar])
           self.ctrlprm_struct.payload[par] = parValueList[iPar]

    # return a sequence object, used for passing pulse sequence and channel infomation over to the CUDA driver
    def get_current_sequence(self):
        self.update_ctrlprm_class('current')
        seq = sequence(self.npulses_per_sequence, self.nrf_rx_samples_per_integration_period, self.tr_to_pulse_delay, self.pulse_sequence_offsets_vector, self.pulse_lens, self.phase_masks, self.pulse_masks, self.channelScalingFactor,  self.ctrlprm_struct.payload )
        return seq

    def get_next_sequence(self):
        self.update_ctrlprm_class('next')
        seq = sequence(self.npulses_per_sequence, self.nrf_rx_samples_per_integration_period, self.tr_to_pulse_delay, self.pulse_sequence_offsets_vector, self.pulse_lens, self.phase_masks, self.pulse_masks, self.channelScalingFactor,  self.ctrlprm_struct.payload )
        return seq

    def DefaultHandler(self, rmsg):
        self.logger.error("Unexpected command: {}".format(chr(rmsg.payload['type'])))
        pdb.set_trace()
        return RMSG_FAILURE

    def QuitHandler(self, rmsg):
        # TODO: close down stuff cleanly
        #rmsg.set_data('status', RMSG_FAILURE)
        #rmsg.set_data('type', rmsg.payload['type'])
        #rmsg.transmit()
        self.conn.close()
        self.logger.debug('Deleting channel {}'.format(self.cnum))
        hardwareManager = self.parent_RadarHardwareManager
        hardwareManager.unregister_channel_from_HardwareManager(self)
        cnum = self.cnum
        del self # TODO close thread ?!?
        # sys.exit() # TODO: set return value
        hardwareManager.logger.info('Deleted channel {}.'.format(cnum))
        return 'exit'

    def PingHandler(self, rmsg):
        return RMSG_SUCCESS
    
    #@timeit
    def RequestAssignedFreqHandler(self, rmsg):
        # wait for clear frequency search to end, hardware manager will set channel state to WAIT
        # self._waitForState(STATE_WAIT) 
        clrFreqResult = self.scanManager.get_current_clearFreq_result()

        transmit_dtype(self.conn, clrFreqResult[0], np.int32)
        transmit_dtype(self.conn, clrFreqResult[1], np.float32)

        self.logger.info('ch {}: clr frequency search raw data age: {} s'.format(self.cnum, time.time() - clrFreqResult[2]))
        return RMSG_SUCCESS

    #@timeit
    def RequestClearFreqSearchHandler(self, rmsg):
        self.clrfreq_struct.receive(self.conn)

        # set request flat from RadarHardwareManager:clearFreqRawDatamanager
        self.parent_RadarHardwareManager.clearFreqRawDataManager.outstanding_request = True
        self.logger.debug("RequestClearFreqSearchHandler: setting request CLR_FREQ flag in clearFreqRawDataManager (caused by ch {})".format(self.cnum))

        return RMSG_SUCCESS

    def UnsetReadyFlagHandler(self, rmsg):
        return RMSG_SUCCESS


    def SetReadyFlagHandler(self, rmsg):
        # ROS calls it ready, we call it trigger
        self.logger.debug("ch {}: SetReadyFlagHandler: waiting for nextSwingToTrigger (swing {}) become  CS_READY".format(self.cnum, self.swingManager.nextSwingToTrigger))
        self._waitForState(self.swingManager.nextSwingToTrigger, [CS_READY, CS_LAST_SWING])
 #       transmit_dtype(self.conn, self.nSequences_per_period, np.uint32) # TODO mgu transmit here nSeq ?     
        self.logger.debug("ch {}: SetReadyFlagHandler: setting nextSwingToTrigger state (swing {}) to CS_TRIGGER".format(self.cnum, self.swingManager.nextSwingToTrigger))
        self.state[self.swingManager.nextSwingToTrigger] = CS_TRIGGER
        # send trigger command
        return RMSG_SUCCESS
    
    #@timeit
    def RegisterSeqHandler(self, rmsg):
        # function to get the indexes of rising edges going from zero to a nonzero value in array ar

        self.logger.debug('Entering RegisterSeqHandler for channel {}'.format(self.cnum))
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
    
        self.logger.debug('RegisterSeqHandler, received sequence data from control program')
        intsc = recv_dtype(self.conn, np.int32)
        intus = recv_dtype(self.conn, np.int32)

        self.logger.debug('RegisterSeqHandler, received intsc: {}, intus: {}'.format(intsc, intus))
        self.integration_period_duration = intsc + (intus / 1e6)

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
        samples    = seq_buf & S_BIT
        tr_window  = seq_buf & R_BIT
        rf_pulse   = seq_buf & X_BIT
        atten      = seq_buf & A_BIT
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
        self.tx_time = self.pulse_lens[0] + 2 * self.tr_to_pulse_delay

        self.logger.debug("pulse0 length: {} us, tr_pulse_delay: {} us, tx_time: {} us".format(self.pulse_lens[0], tr_to_pulse_delay,  self.pulse_lens[0] + 2 * self.tr_to_pulse_delay))
        if npulses_per_sequence == 0:
            raise ValueError('number of pulses per sequence must be greater than zero!')
        if nbb_samples == 0:
            raise ValueError('number of samples in sequence must be nonzero!')

        return RMSG_SUCCESS
    
    # receive a ctrlprm struct
    #@timeit
    def SetParametersHandler(self, rmsg):
        # TODO: check if new freq is possible with usrp_centerFreq
        # TODO divide compatibiliti check in sequence and ctrlprm check?
        # TODO add compatibility check in parameter prediction function

        RHM = self.parent_RadarHardwareManager


        # period not jet triggered
        if self.active_state == CS_INACTIVE or self.active_state == CS_READY: 
           RHM.set_par_semaphore.acquire()

           if self.active_state == CS_READY:
              self.logger.debug("Channel already initialized, but not triggered, Reinitializing it...")
              self.active_state = CS_INACTIVE

           self.ctrlprm_struct.receive(self.conn)
           self.logger.debug("ch {}: Received from ROS: tbeam={}, rbeam={}, tfreq={}, rfreq={}".format(self.cnum, self.ctrlprm_struct.payload['tbeam'], self.ctrlprm_struct.payload['rbeam'], self.ctrlprm_struct.payload['tfreq'], self.ctrlprm_struct.payload['rfreq']))

           if not self.CheckChannelCompatibility(): # TODO  for two swings and reset after transmit?
              return RSMG_FAILURE

           if self not in self.parent_RadarHardwareManager.newChannelList:
              self.parent_RadarHardwareManager.newChannelList.append(self)
              self.logger.debug("Adding ch {} to newChannelList ".format(self.cnum))
           else:
              self.logger.debug("Ch {} already in newChannelList ".format(self.cnum))
           RHM.set_par_semaphore.release()
 
        # in middle of scan, period already triggerd. only compare with prediction
        elif self.active_state == CS_PROCESSING or self.active_state == CS_LAST_SWING: 
         # TODO something here is wrong: uafscan with --onesec has CS_LAST_SWING but --fast not
           self.update_ctrlprm_class("current")
           ctrlprm_old = copy.deepcopy(self.ctrlprm_struct.payload)

           # compare received with predicted parameter
           self.ctrlprm_struct.receive(self.conn)
           for key in ctrlprm_old.keys():
              if np.any(ctrlprm_old[key] != self.ctrlprm_struct.payload[key]):
                 self.logger.error("ch {}: received ctrlprm_struct for {} ({}) is not equal with prediction ({})".format(self.cnum, key,self.ctrlprm_struct.payload[key], ctrlprm_old[key] ))
                 # TODO return RMSG_FAILURE
              #else:
               #  self.logger.debug("ch {}: received ctrlprm_struct for {} ({}) IS     equal with prediction ({})".format(self.cnum, key,self.ctrlprm_struct.payload[key], ctrlprm_old[key] ))
        else:
           self.logger.error("Active state is {}. Dont know what to do...".format(self.active_state))
           pdb.set_trace()

        
        if (self.rnum < 0 or self.cnum < 0):
            self.logger.error("SET_PARAMETER: Invalid radar or channel number: rnum={}, cnum={}".format(self.rnum, self.cnum))
            return RMSG_FAILURE

        return RMSG_SUCCESS

    def CheckChannelCompatibility(self):
        self.logger.debug('checking channel compatibility for channel {}'.format(self.cnum))
        hardwareManager = self.parent_RadarHardwareManager
        commonParList_ctrl = ['number_of_samples', 'baseband_samplerate' ]
        commonParList_seq  = [ 'npulses_per_sequence', 'pulse_sequence_offsets_vector',  'tr_to_pulse_delay', 'integration_period_duration', 'tx_time']
        if all([self.pulse_lens[0]==self.pulse_lens[i] for i in range(1,len(self.pulse_lens))]):
            pulseLength = self.pulse_lens[0]
        else:
            self.logger.error("Pulse lengths in one sequence have to be the equal! ") # TODO raise error?
            pdb.set_trace()
            return False
        if hardwareManager.nRegisteredChannels == 1 and hardwareManager.channels[0] == self:
           self.logger.info("Compatibility check: This channel is already registered at HardwareManager and is the only one. Renewing registration.")
           hardwareManager.nRegisteredChannels == 0

        if hardwareManager.nRegisteredChannels == 0:  # this is the first channel
            hardwareManager.commonChannelParameter = {key: getattr(self, key) for key in commonParList_seq}
            hardwareManager.commonChannelParameter.update( {key: self.ctrlprm_struct.payload[key] for key in commonParList_ctrl})
            hardwareManager.commonChannelParameter.update({'pulseLength':pulseLength})

            hardwareManager.nRegisteredChannels = 1
            return True

        else:   # not first channel => check if new parameters are compatible
            
            parCompatibleList_seq  = [hardwareManager.commonChannelParameter[parameter] == getattr(self, parameter) for parameter in commonParList_seq]
            parCompatibleList_ctrl = [hardwareManager.commonChannelParameter[parameter] == self.ctrlprm_struct.payload[parameter] for parameter in commonParList_ctrl]

            idxOffsetVec = commonParList_seq.index('pulse_sequence_offsets_vector')  # convert vector of bool to scalar
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
                    

        # TODO change usrp_xx_cfreq somewhere if possible        
        assert np.abs((ch.ctrlprm_struct.payload['tfreq'] * 1e3) - self.mixingFreqManager.current_mixing_freq*1e6) < (self.usrp_rf_tx_rate / 2), 'transmit frequency outside range supported by sampling rate and center frequency'


    # send ctrlprm struct
    #@timeit
    def GetParametersHandler(self, rmsg):
        # TODO: return bad status if negative radar or channel
        self.update_ctrlprm_class("current")
        self.ctrlprm_struct.transmit()
        self.logger.debug("ch {}: sending current ctrlprm_struct (tfreq={}, rfreq={},tbeam={},rbeam={})".format(self.cnum, self.ctrlprm_struct.get_data('tfreq'), self.ctrlprm_struct.get_data('rfreq'), self.ctrlprm_struct.get_data('tbeam'), self.ctrlprm_struct.get_data('rbeam') ))
        return RMSG_SUCCESS
    
    #@timeit
    def GetDataHandler(self, rmsg):
        self.logger.debug('start channelHanlder:GetDataHandler ch: {}'.format(self.cnum))
        self.update_ctrlprm_class("current")
        self.dataprm_struct.set_data('samples', self.ctrlprm_struct.payload['number_of_samples'])

        self.dataprm_struct.transmit() # only 'samples' of dataprm is ever changed TODO check other parameter such as event_secs....
        self.logger.debug('ch {}: sending dprm struct'.format(self.cnum))

        if self.rnum < 0 or self.cnum < 0:
            pdb.set_trace()
            return RMSG_FAILURE

        # TODO investigate possible race conditions

        finishedSwing = self.swingManager.lastSwingWithData 
        self.logger.debug('ch {}: channelHanlder:GetDataHandler waiting for channel to idle before GET_DATA (finished swing is {})'.format(self.cnum, finishedSwing))
        self.logger.debug("start waiting for CS_SAMPLES_READY")
        self._waitForState(finishedSwing, CS_SAMPLES_READY)
        self.logger.debug("end waiting for CS_SAMPLES_READY")

        self.logger.debug('ch {}: channelHanlder:GetDataHandler returning samples'.format(self.cnum))
#        transmit_dtype(self.conn, self.parent_RadarHardwareManager.resultData_nSequences_per_period, np.uint32)  
        self.send_results_to_control_program()

        self.logger.debug('ch {}: channelHanlder:GetDataHandler finished returning samples. setting state to {}  (swing {})'.format(self.cnum, self.next_state[finishedSwing], finishedSwing))
        self.state[finishedSwing] = self.next_state[finishedSwing]
        self.logger.debug('end channelHanlder:GetDataHandler ch: {}'.format(self.cnum))

        self.swingManager.lastSwingWithData = 1 - finishedSwing   # alternate swing

        return RMSG_SUCCESS

    def send_results_to_control_program(self):
        # interact with site library's SiteIntegrate loop
        # send metadata for integration period
        # currently assuming pulse sequences are uniform within an integration period

        rd_shallow = self.resultDict_list[-1]
        resultDict = copy.deepcopy(self.resultDict_list.pop())

        transmit_dtype(self.conn, resultDict['nSequences_per_period'], np.uint32)  
        self.logger.debug("transmitting number of sequences in period: {}".format(resultDict['nSequences_per_period']))

        badtrdat_start_usec = resultDict['pulse_sequence_offsets_vector'] * 1e6 # convert to us
        transmit_dtype(self.conn, resultDict['npulses_per_sequence'], np.uint32)
        transmit_dtype(self.conn, badtrdat_start_usec,                np.uint32) # length badtrdat_len
        transmit_dtype(self.conn, resultDict['pulse_lens'],           np.uint32) # length badtrdat_len

        # stuff these with junk, they don't seem to be used..
        num_transmitters = self.parent_RadarHardwareManager.usrpManager.nUSRPs   # TODO update for polarization?
        txstatus_agc = self.parent_RadarHardwareManager.usrpManager.fault_status # TODO is this the right way to return fault status????
        txstatus_lowpwr = np.zeros(num_transmitters)
        if txstatus_agc.any():
            self.logger.warning('Following USRPs report Fault:  {} (usrp index)'.format([k for k in range(txstatus_agc.size) if txstatus_agc[k] != 0]))

        transmit_dtype(self.conn, num_transmitters, np.int32)
        transmit_dtype(self.conn, txstatus_agc,     np.int32) # length num_transmitters
        transmit_dtype(self.conn, txstatus_lowpwr,  np.int32) # length num_transmitters
        
#        # send back ctrlprm
#        for par in ["rbeam", "tbeam", "tfreq", "rfreq"]:
#            for item in resultDict['ctrlprm_struct']:
#                if item.name == par:
#                   self.logger.debug("Sending to ROS: {}={}".format(par, item.data))

        for item in resultDict['ctrlprm_dataqueue']:
            item.transmit(self.ctrlprm_struct.clients[0])
            if item.name in ["rbeam", "tbeam", "tfreq", "rfreq"]:
                   self.logger.debug("Sending to ROS: {}={}".format(item.name, item.data))
        
##        self.resultData_ctrlprm = []

    
        # send back samples with pulse start times 
        for iSequence in range(resultDict['nSequences_per_period']):
            self.logger.debug('GET_DATA returning samples from pulse {}'.format(iSequence))
            
            self.logger.debug('GET_DATA sending sequence start time')
            transmit_dtype(self.conn, resultDict['sequence_start_time_secs'][iSequence],  np.uint32)
            transmit_dtype(self.conn, resultDict['sequence_start_time_usecs'][iSequence], np.uint32)

            # calculate the baseband sample index for the start and end of a pulse sequence
            # within a block of beamformed samples over the entire integration period
            # assuming that the first sample is aligned with the center of the first transmit pulse
            # and all sequences within the integration period have the same length

            pulse_sequence_start_index = iSequence * resultDict['nbb_rx_samples_per_sequence']
            pulse_sequence_end_index = pulse_sequence_start_index + resultDict['number_of_samples']
            self.logger.debug("Number of samples if {} (no deepcopy version is {}), main beamformed shape: {}".format(resultDict['number_of_samples'], rd_shallow['number_of_samples'], resultDict['main_beamformed'].shape))
            self.logger.debug("start index: {}, end index: {}".format(pulse_sequence_start_index, pulse_sequence_end_index))
        
            # send the packed complex int16 samples to the control program.. 
            self.logger.debug('GET_DATA sending main samples')
            transmit_dtype(self.conn, resultDict['main_beamformed'][pulse_sequence_start_index:pulse_sequence_end_index], np.uint32)

            self.logger.debug('GET_DATA sending back samples')
            transmit_dtype(self.conn, resultDict['back_beamformed'][pulse_sequence_start_index:pulse_sequence_end_index], np.uint32)
            
            # wait for confirmation before sending the next antenna..
            # if we start catching this assert or timing out, maybe add some more error handling here
            self.logger.debug('GET_DATA waiting on ack from site library')
            sample_send_status = recv_dtype(self.conn, np.int32)
            assert sample_send_status == iSequence 


        self.logger.warning('GET_DATA: sending main array samples twice instead of main then back array!')


    
    #@timeit
    def SetRadarChanHandler(self, rmsg):
        self.rnum = recv_dtype(self.conn, np.int32)
        self.cnum = recv_dtype(self.conn, np.int32)

        if self.cnum in [ch.cnum for ch in self.parent_RadarHardwareManager.channels if ch is not None and ch is not self]:
           self.logger.error("New channel (cnum {}) can not be added beause channel with this cnum already active.".format(self.cnum))
           return RMSG_FAILURE
        
        
        self.ctrlprm_struct.set_data('channel', self.cnum)
        self.ctrlprm_struct.set_data('radar',  self.rnum)

        # TODO: how to handle channel contention?
        # self.logger.name = "ChManager {}".format(self.cnum)
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
        # called by site library at the start of a scan 
        self.active = True

        self.logger.debug('SetActiveHandler starting')
        
        if self.parent_RadarHardwareManager.trigger_next_function_running:
           self.logger.debug('start SetActiveHandler: waiting for trigger_next() to finish')
           while self.parent_RadarHardwareManager.trigger_next_function_running:
              time.sleep(0.01)
           self.logger.debug('end SetActiveHandler: waiting for trigger_next() to finish')

        scan_num_beams = recv_dtype(self.conn, np.int32)
        self.logger.debug('SetActiveHandler number of beams per scan: {}'.format(scan_num_beams))

        clrfreq_start_list = recv_dtype(self.conn, np.int32, nitems = scan_num_beams)
        self.logger.debug('SetActiveHandler clear frequency search start frequencies: {}'.format(clrfreq_start_list))

        clrfreq_bandwidth_list = recv_dtype(self.conn, np.int32, nitems = scan_num_beams)
        self.logger.debug('SetActiveHandler clear frequency search bandwidths (Hz): {}'.format(clrfreq_bandwidth_list))

        scan_beam_list = recv_dtype(self.conn, np.int32, nitems = scan_num_beams)
        self.logger.debug('SetActiveHandler scan beam list: {}'.format(scan_beam_list))

        freq_range_list = [[clrfreq_start_list[i], clrfreq_start_list[i] + clrfreq_bandwidth_list[i]] for i in range(scan_num_beams)]

        self.logger.debug('SetActiveHandler updating swingManager with new freq/beam lists')
        self.scanManager.init_new_scan(freq_range_list, scan_beam_list)

        self.swingManager.reset()
        self.logger.debug("Resetting swing manager (active={}, processing={})".format(self.swingManager.activeSwing, self.swingManager.processingSwing ))
    

        return RMSG_SUCCESS

    def SetInactiveHandler(self, rmsg):
        self.active = False
        # TODO: return failure status if the radar or channel number is invalid?
        return RMSG_SUCCESS


def main():
    # maybe switch to multiprocessing with manager process
    
    logging_usrp.initLogging('server.log')
    logging.info('Strating main() of usrp_server')

    rmsg_port = 45000

    radar = RadarHardwareManager(rmsg_port)
    radar.run()


if __name__ == '__main__':
    main()

