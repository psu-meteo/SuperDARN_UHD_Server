# so, hook usrp_tcp_driver into site library
# todo, write mock arbyserver that can handle these commands
# TODO: write mock arby server that can feed normalscan with false data..

import sys
from rosmsg import *

class RadarChannelManager:
    # create new channel
    def __init__(self):
        self.active = False
        self.ready = False
        pass


    def close(self):
        pass
    
    def __del__(self):
        pass

    def DefaultHandler(self, rmsg):
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
        send_dtype(sock, np.int32(self.tfreq))
        send_dtype(sock, np.float32(self.noise))
        rmsg.payload['status'] = RMSG_SUCCESS
        rmsg.transmit()

    def RequestClearFreqHandler(self, rmsg):
        # start clear frequency search
        self.clrfreq = clrfreqprm_struct(sock)
        self.clrfreq.receive()

        # TODO: start clear frequency search
        # set self.tfreq, self.noise
        rmsg.payload['status'] = RMSG_SUCCESS
        rmsg.transmit()

        pass

    def SetReadyFlagHandler(self, rmsg):
        self.ready = True
        rmsg.payload['status'] = RMSG_SUCCESS
        rmsg.transmit()

    def RegisterSeqHandler(self, rmsg):
        self.seqprm = seqprm_struct(sock)
        self.seqprm.receive()
        self.seq_rep = recv_dtype(sock, np.uint8, seqprm.payload['len'])
        self.seq_code = recv_dtype(sock, np.uint8, seqprm.payload['len'])
        
        # TODO: do something with this..
        self.payload['status'] = RMSG_SUCCESS
        rmsg.transmit()

    # receive a ctrlprm struct
    def SetParametersHandler(self, rmsg):
        self.ctrlprm_struct.receive(sock)
        rmsg.payload['status'] = RMSG_SUCCESS
        rmsg.transmit()

    # send ctrlprm struct
    def GetParametersHandler(self, rmsg):
        self.ctrlprm_struct.transmit(sock)
        rmsg.payload['status'] = RMSG_SUCCESS
        rmsg.transmit()

    def GetDataHandler(self, rmsg):
        # TODO: write this
        self.dataprm.transmit(sock)
        # TODO: find main/back samples..
        self.transmit_dtype(sock, np.uint32(main_samples))
        self.transmit_dtype(sock, np.uint32(back_samples))
     
        send_dtype(sock, np.uint32(badtrdat_len))
        send_dtype(sock, np.uint32(badtrdat_start_usec)) # length badtrdat_len
        send_dtype(sock, np.uint32(badtrdat_duration_usec)) # length badtrdat_len
    
        send_dtype(sock, np.int32(num_transmitters))
        send_dtype(sock, np.int32(txstatus_agc)) # length num_transmitters
        send_dtype(sock, np.int32(txstatus_lowpwr)) # length num_transmitters
        
        rmsg.payload['status'] = RMSG_SUCCESS
        rmsg.transmit()

    def SetRadarChanHandler(self, rmsg):
        rnum = recv_dtype(sock, np.int32)
        cnum = recv_dtype(sock, np.int32)

        if TODO_CHANNEL_IS_AVAILABLE:
            rmsg.payload['status'] = RMSG_FAILURE
        else:
            rmsg.payload['status'] = RMSG_SUCCESS 

        rmsg.transmit()

    def QueryIniSettingHandler(self, rmsg):
        data_length = recv_dtype(sock, np.int32)
        ini_name = recv_dtype(sock, str, nitems = data_length)
        requested_type = recv_dtype(sock, np.uint8)

        # TODO: look up ini name in ini files
        send_dtype(sock, requested_type)
        send_dtype(sock, requested_length) # appears to be unused by site library
        send_dtype(sock, payload) # always 4 bytes..

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
        cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        cmd_sock.bind(('localhost', port))
    

    def run(self):

        cmd_sock.listen(5)

        while(True):
            server_conn, addr = cmd_sock.accept()
            cmd = recv_dtype(server_conn, np.uint8)
            handler = cudamsg_handlers[cmd](server_conn, gpu, antennas, array_info, hardware_limits)
            handler.process()

    def start(self):
        rmsg = rosmsg_command(sock)
        rmsg.receive()
        if rmsg.payload['type'] in rmsg_handlers:	
            rmsg_handlers(rmsg.payload['type'])(rmsg)
        else:
            SiteDefaultHandler(rmsg)
        rmsg.transmit()

        # receive rosmsg
        # run handler
        # send rosmsg

    rmsg_handlers = {\
        SET_RADAR_CHAN : pass, \
        SET_INACTIVE : pass, \
        SET_ACTIVE : pass, \
        QUERY_INI_SETTINGS : pass, \
        GET_SITE_SETTINGS : pass, \
        UPDATE_SITE_SETTINGS : pass,\
        GET_PARAMETERS : pass,\
        SET_PARAMETERS : pass,\
        PING : pass,\
        OKAY : pass,\
        NOOP : pass,\
        QUIT : pass,\
        REGISTER_SEQ: pass,\
        REMOVE_SEQ: pass,\
        REQUEST_ASSIGNED_FREQ: pass,\
        REQUEST_CLEAR_FREQ_SEARCH: pass,\
        LINK_RADAR_CHAN : pass,\
        SET_READY_FLAG: pass,\
        UNSET_READY_FLAG: pass,\
        SET_PROCESSING_FLAG: pass,\
        UNSET_PROCESSING_FLAG: pass,\
        GET_DATA: pass,\
        WAIT_FOR_DATA: pass}

def main():
    # so, open server and listen on port
    # capture rmsg commands, return bogus data
    # try running two control programs.. 

if '__name__' == '__main__':
    main()
    rmsg_port = 45000 
    rmsg_manager(port)
    rmsg_manager.start()


