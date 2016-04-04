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
        pdb.set_trace()
        # TODO write this..

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
        # TODO: set these using clear frequency search
        self.tfreq = 12000
        self.noise = 1000
        transmit_dtype(self.conn, self.tfreq, np.int32)
        transmit_dtype(self.conn, self.noise, np.float32)
        return RMSG_SUCCESS

    def RequestClearFreqSearchHandler(self, rmsg):
        # start clear frequency search
        self.clrfreq_struct.receive(self.conn)

        # TODO: start clear frequency search
        # set self.tfreq, self.noise
        return RMSG_SUCCESS

    def UnsetReadyFlagHandler(self, rmsg):
        self.ready = False 
        return RMSG_SUCCESS


    def SetReadyFlagHandler(self, rmsg):
        self.ready = True
        return RMSG_SUCCESS

    def RegisterSeqHandler(self, rmsg):
        # site libraries appear to not initialize the status, so a nonzero status here is normall. 
        self.seqprm_struct.receive(self.conn)
        self.seq_rep = recv_dtype(self.conn, np.uint8, self.seqprm_struct.payload['len'])
        self.seq_code = recv_dtype(self.conn, np.uint8, self.seqprm_struct.payload['len'])
        
        # TODO: do something with this..
        return RMSG_SUCCESS

    # receive a ctrlprm struct
    def SetParametersHandler(self, rmsg):
        self.ctrlprm_struct.receive(self.conn)
        return RMSG_SUCCESS

    # send ctrlprm struct
    def GetParametersHandler(self, rmsg):
        # TODO: return bad status if negative radar or channel
        self.ctrlprm_struct.transmit()
        return RMSG_SUCCESS

    def GetDataHandler(self, rmsg):
        # TODO: setup dataprm_struct
        # see self.ctrlprm_struct.payload['number_of_samples']
        self.dataprm_struct.transmit()

        if not self.active or self.rnum < 0 or self.cnum < 0:
            pdb.set_trace()
            return RMSG_FAILURE

        # TODO: get data handler waits for control_program to set active flag in controlprg struct
        # need some sort of synchronizaion..
        pdb.set_trace()
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

        return RMSG_SUCCESS
        
    def SetRadarChanHandler(self, rmsg):
        self.rnum = recv_dtype(self.conn, np.int32)
        self.cnum = recv_dtype(self.conn, np.int32)

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
        ini_name = recv_dtype(self.conn, str, nitems = data_length)
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
    

