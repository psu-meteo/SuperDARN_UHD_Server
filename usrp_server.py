# so, hook usrp_tcp_driver into site library
# todo, write mock arbyserver that can handle these commands
# TODO: write mock arby server that can feed multiple normalscans with false data..

import sys
import numpy as np
import threading
import logging
import pdb
import socket
from termcolor import cprint


from socket_utils import *
from rosmsg import *

MAX_CHANNELS = 10
RMSG_FAILURE = -1 
RMSG_SUCCESS = 0

debug = True

def rmsg_return_success(func):
    def success_wrapper(self, rmsg):
        func(self, rmsg)
        
        rmsg.set_data('status', RMSG_SUCCESS)
        rmsg.set_data('type', rmsg.payload['type'])
        rmsg.transmit()

    return success_wrapper

def rmsg_return_failure(func):
    def failure_wrapper(self, rmsg):
        func(self, rmsg)
        
        rmsg.set_data('status', RMSG_FAILURE)
        rmsg.set_data('type', rmsg.payload['type'])
        rmsg.transmit()

    return failure_wrapper



class RadarChannelHandler:
    def __init__(self, conn):
        self.active = False
        self.ready = False
        self.conn = conn

        self.ctrlprm_struct = ctrlprm_struct(self.conn)
        self.seqprm_struct = seqprm_struct(self.conn)
        self.clrfreq_struct = clrfreqprm_struct(self.conn)
        self.rprm_struct = rprm_struct(self.conn)
        self.dataprm_struct = dataprm_struct(self.conn)


    def run(self):

        rmsg_handlers = {\
            SET_RADAR_CHAN : self.SetRadarChanHandler,\
            SET_INACTIVE : self.SetInactiveHandler,\
            SET_ACTIVE : self.SetActiveHandler,\
            QUERY_INI_SETTINGS : self.QueryIniSettingsHandler,\
        #    GET_SITE_SETTINGS : self.GetSiteSettingsHandler, \
        #    UPDATE_SITE_SETTINGS : self.UpdateSiteSettingsHandler,\
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
                rmsg_handlers[command](rmsg)
            else:
                self.DefaultHandler(rmsg)


    def close(self):
        pdb.set_trace()
        # TODO
        pass

    @rmsg_return_success
    def DefaultHandler(self, rmsg):
        print("Unexpected command: {}".format(chr(rmsg.payload['type'])))
        pdb.set_trace()

    def QuitHandler(self, rmsg):
        # TODO: close down stuff cleanly
        rmsg.set_data('status', RMSG_FAILURE)
        rmsg.set_data('type', rmsg.payload['type'])
        rmsg.transmit()
        self.conn.close()
        sys.exit() # TODO: set return value

    @rmsg_return_success
    def PingHandler(self, rmsg):
        pass

    @rmsg_return_success
    def RequestAssignedFreqHandler(self, rmsg):
        # TODO: set these using clear frequency search
        self.tfreq = 12000
        self.noise = 1000
        transmit_dtype(self.conn, self.tfreq, np.int32)
        transmit_dtype(self.conn, self.noise, np.float32)

    @rmsg_return_success
    def RequestClearFreqSearchHandler(self, rmsg):
        # start clear frequency search
        self.clrfreq_struct.receive(self.conn)

        # TODO: start clear frequency search
        # set self.tfreq, self.noise

    @rmsg_return_success
    def UnsetReadyFlagHandler(self, rmsg):
        self.ready = False 


    @rmsg_return_success
    def SetReadyFlagHandler(self, rmsg):
        self.ready = True

    @rmsg_return_success
    def RegisterSeqHandler(self, rmsg):
        # site libraries appear to not initialize the status, so a nonzero status here is normall. 
        self.seqprm_struct.receive(self.conn)
        self.seq_rep = recv_dtype(self.conn, np.uint8, self.seqprm_struct.payload['len'])
        self.seq_code = recv_dtype(self.conn, np.uint8, self.seqprm_struct.payload['len'])
        
        # TODO: do something with this..

    # receive a ctrlprm struct
    @rmsg_return_success
    def SetParametersHandler(self, rmsg):
        self.ctrlprm_struct.receive(self.conn)

    # send ctrlprm struct
    @rmsg_return_success
    def GetParametersHandler(self, rmsg):
        # TODO: return bad status if negative radar or channel
        self.ctrlprm_struct.transmit()

    @rmsg_return_success
    def GetDataHandler(self, rmsg):
        # TODO: setup dataprm_struct
        pdb.set_trace()
        # rmsg.set_data('status', RMSG_FAILURE)
        self.dataprm_struct.set_data('samples', 294)
    
        # TODO: gather main/back samples..
        main_samples = np.zeros(30)
        back_samples = np.zeros(30)
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
        
    @rmsg_return_success
    def SetRadarChanHandler(self, rmsg):
        rnum = recv_dtype(self.conn, np.int32)
        cnum = recv_dtype(self.conn, np.int32)

        if(debug):
            print('radar num: {}, radar chan: {}'.format(rnum, cnum))

        # TODO: set RMSG_FAILURE if radar channel is unavailable
        # rmsg.set_data('status', RMSG_FAILURE)
 
    @rmsg_return_success
    def LinkRadarChanHandler(self, rmsg):
        rnum = recv_dtype(self.conn, np.int32)
        cnum = recv_dtype(self.conn, np.int32)
        print('link radar chan is unimplemented!')
        pdb.set_trace()


    def QueryIniSettingsHandler(self, rmsg):
        # TODO: don't hardcode this if I find anything other than ifmode querying..
        data_length = recv_dtype(self.conn, np.int32)
        ini_name = recv_dtype(self.conn, str, nitems = data_length)
        requested_type = recv_dtype(self.conn, np.uint8)
        
        # hardcode to reply with ifmode is false
        assert ini_name == b'site_settings:ifmode\x00'
       
        payload = 0 # assume always false

        transmit_dtype(self.conn, requested_type, np.uint8)
        transmit_dtype(self.conn, data_length, np.int32) # appears to be unused by site library
        transmit_dtype(self.conn, payload, np.int32) 
        
        # TODO: Why does the ini handler expect a nonzero response for success?
        rmsg.set_data('status', 1)
        rmsg.set_data('type', rmsg.payload['type'])
        rmsg.transmit()

    @rmsg_return_success
    def SetActiveHandler(self, rmsg):
        # TODO: return failure if rnum and cnum are not set
        # TODO: why is active only set if it is already set?
        if not self.active:
            self.active = True
            self.ready = False

    @rmsg_return_success
    def SetInactiveHandler(self, rmsg):
        # TODO: return failure status if the radar or channel number is invalid
            
        if not self.active:
            self.active = False
            self.ready = False
            # TODO: what is coordination handler doing?


class RMsgManager:
    def __init__(self, port):
        self.client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client_sock.bind(('localhost', port))
    

    def run(self):
        self.client_sock.listen(MAX_CHANNELS)
        client_threads = []

        def spawn_channel(conn):
            # start new radar channel handler
            channel = RadarChannelHandler(conn)
            channel.run()
            conn.close()

        while True:
            client_conn, addr = self.client_sock.accept()
            
            ct = threading.Thread(target = spawn_channel, args = (client_conn,))
            client_threads.append(ct)
            ct.start()
        
        client_sock.close() 


def main():
    # maybe switch to multiprocessing with manager process
    print('main!')
    logging.basicConfig(filename = 'client_server.log', level=logging.DEBUG, format='[%(levelname)s] (%(threadName)-10s) %(asctime)s %(message)s',)

    logging.debug('main()')
    rmsg_port = 45000 
    man = RMsgManager(rmsg_port)
    man.run()

    # so, open server and listen on port
    # capture rmsg commands, return bogus data
    # try running two control programs.. 

if __name__ == '__main__':
    main()
    

