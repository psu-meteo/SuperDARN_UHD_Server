# so, hook usrp_tcp_driver into site library
# todo, write mock arbyserver that can handle these commands
# TODO: write mock arby server that can feed multiple normalscans with false data..

import sys
import numpy as np
import threading
import logging
import pdb
import socket

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
        #   LINK_RADAR_CHAN : self.LinkRadarChanHandler,\
            SET_READY_FLAG: self.SetReadyFlagHandler,\
            UNSET_READY_FLAG: self.UnsetReadyFlagHandler,\
        #   SET_PROCESSING_FLAG: self.SetProcessingFlagHandler,\
        #   UNSET_PROCESSING_FLAG: self.UnsetProcessingFlagHandler,\
        #   WAIT_FOR_DATA: self.WaitForDataHandler,\
            GET_DATA: self.GetDataHandler}


        while True:
            rmsg = rosmsg_command(self.conn)
            rmsg.receive(self.conn)
            command = chr(rmsg.payload['type'])
            print('received command: {}'.format(command))

            
            if command in rmsg_handlers:	
                rmsg_handlers[command](rmsg)
            else:
                self.DefaultHandler(rmsg)


    def close(self):
        # TODO
        pass
   	
    def DefaultHandler(self, rmsg):
        print("Unexpected command: {}".format(chr(rmsg.payload['type'])))
        pdb.set_trace()
        rmsg.payload['status'] = RMSG_SUCCESS

    def QuitHandler(self, rmsg):
        # TODO: close down stuff cleanly
        rmsg.payload['status'] = RMSG_FAILURE
        rmsg.transmit()
        sys.exit()

    def PingHandler(self, rmsg):
        rmsg.payload['status'] = RMSG_SUCCESS
        rmsg.transmit()

    def RequestAssignedFreqHandler(self, rmsg):
        send_dtype(self.conn, np.int32(self.tfreq))
        send_dtype(self.conn, np.float32(self.noise))
        rmsg.payload['status'] = RMSG_SUCCESS
        rmsg.transmit()

    def RequestClearFreqSearchHandler(self, rmsg):
        # start clear frequency search
        self.clrfreq = clrfreqprm_struct(self.conn)
        self.clrfreq.receive()

        # TODO: start clear frequency search
        # set self.tfreq, self.noise
        rmsg.payload['status'] = RMSG_SUCCESS
        rmsg.transmit()
        pass

    def UnsetReadyFlagHandler(self, rmsg):
        self.ready = False 
        rmsg.payload['status'] = RMSG_SUCCESS
        rmsg.transmit()


    def SetReadyFlagHandler(self, rmsg):
        self.ready = True
        rmsg.payload['status'] = RMSG_SUCCESS
        rmsg.transmit()

    def RegisterSeqHandler(self, rmsg):
        self.seqprm = seqprm_struct(self.conn)
        self.seqprm.receive()
        self.seq_rep = recv_dtype(self.conn, np.uint8, seqprm.payload['len'])
        self.seq_code = recv_dtype(self.conn, np.uint8, seqprm.payload['len'])
        
        # TODO: do something with this..
        self.payload['status'] = RMSG_SUCCESS
        rmsg.transmit()

    # receive a ctrlprm struct
    def SetParametersHandler(self, rmsg):
        self.ctrlprm_struct.receive(self.conn)
        rmsg.payload['status'] = RMSG_SUCCESS
        rmsg.transmit()

    # send ctrlprm struct
    def GetParametersHandler(self, rmsg):
        self.ctrlprm_struct.transmit(self.conn)
        rmsg.payload['status'] = RMSG_SUCCESS
        rmsg.transmit()

    def GetDataHandler(self, rmsg):
        # TODO: write this
        self.dataprm.transmit(self.conn)
        # TODO: find main/back samples..
        self.transmit_dtype(self.conn, np.uint32(main_samples))
        self.transmit_dtype(self.conn, np.uint32(back_samples))
     
        send_dtype(self.conn, np.uint32(badtrdat_len))
        send_dtype(self.conn, np.uint32(badtrdat_start_usec)) # length badtrdat_len
        send_dtype(self.conn, np.uint32(badtrdat_duration_usec)) # length badtrdat_len
    
        send_dtype(self.conn, np.int32(num_transmitters))
        send_dtype(self.conn, np.int32(txstatus_agc)) # length num_transmitters
        send_dtype(self.conn, np.int32(txstatus_lowpwr)) # length num_transmitters
        
        rmsg.payload['status'] = RMSG_SUCCESS
        rmsg.transmit()

    def SetRadarChanHandler(self, rmsg):
        rnum = recv_dtype(self.conn, np.int32)
        cnum = recv_dtype(self.conn, np.int32)

        if(debug):
            print('radar num: {}, radar chan: {}'.format(rnum, cnum))

        if True: # TODO: if channel is unavailable
            rmsg.payload['status'] = RMSG_FAILURE
        else:
            rmsg.payload['status'] = RMSG_SUCCESS 

        rmsg.transmit()

    def QueryIniSettingsHandler(self, rmsg):
        data_length = recv_dtype(self.conn, np.int32)
        ini_name = recv_dtype(self.conn, str, nitems = data_length)
        requested_type = recv_dtype(self.conn, np.uint8)

        # TODO: look up ini name in ini files
        send_dtype(self.conn, requested_type)
        send_dtype(self.conn, requested_length) # appears to be unused by site library
        send_dtype(self.conn, payload) # always 4 bytes..

        rmsg.payload['status'] = RMSG_SUCCESS
        rmsg.transmit()

        
    def SetActiveHandler(rmsg):
        self.active = True
        rmsg.payload['status'] = RMSG_SUCCESS
        rmsg.transmit()

    def SetInactiveHandler(rmsg):
        self.active = False
        rmsg.payload['status'] = RMSG_SUCCESS
        rmsg.transmit()

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
    

