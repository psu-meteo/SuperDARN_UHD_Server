# usrp drivers may not be running on the computer which usrp_server is running on
# and will communicate with usrp_drivers using tcp ip sockets

# usrp_server takes arby_server messages and translates them for usrp_drivers
# usrp_server handles one or more usrp_drivers

import numpy as np
import pdb
import struct
import socket
from drivermsg_library import *
from socket_utils import *
# see posix_ipc python library
# http://semanchuk.com/philip/posix_ipc/

dmsg_handler = {}

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
#GPS_SCHEDULE_REPEAT_SCAN = 'r'
#GPS_TRIGGER_NOW = 'n'
#GPS_SET_TRIGGER_RATE = 'R'
GPS_MSG_ERROR = 'X'


sequence_list = []

class sequence(object):
    def __init__(self, txfreq, rxfreq, txrate, rxrate, npulses, num_requested_samples, pulse_offsets_vector, priority):
        self.txfreq = txfreq
        self.rxfreq = rxfreq
        self.txrate = txrate
        self.rxrate = rxrate
        self.npulses = npulses
        self.num_requested_samples = num_requested_samples
        self.pulse_offsets_vector = pulse_offsets_vector
        self.priority = priority
        # self.bb_vec = self._make_bb_vec()
        # self.tr_times = self._make_tr_times()


class dmsg_handler(object):
    def __init__(self, sock):
        self.sock = sock
        self.status = 0;

    def process(self):
        raise NotImplementedError('The process method for this driver message is unimplemented')

    def respond(self):
        self.sock.send(np.int32(self.status).tostring())

    def _recv_ctrlprm(self):
        self.radar = self._recv_dtype(np.int32)
        self.channel = self._recv_dtype(np.int32)
        self.local = self._recv_dtype(np.int32)
        self.priority = self._recv_dtype(np.int32)
        self.current_pulseseq_index = self._recv_dtype(np.int32)
        self.tbeam = self._recv_dtype(np.int32)
        self.tbeamcode = self._recv_dtype(np.uint32)

        self.tbeamazm = self._recv_dtype(np.float32)
        self.tbeamwidth = self._recv_dtype(np.float32)

        self.number_of_samples = self._recv_dtype(np.int32)
        self.buffer_index = self._recv_dtype(np.int32)

        self.baseband_samplerate = self._recv_dtype(np.float32)
        self.filter_bandwidth = self._recv_dtype(np.int32)

        self.match_filter = self._recv_dtype(np.int32)
        self.rfreq = self._recv_dtype(np.int32)
        self.rbeam = self._recv_dtype(np.int32)
        self.rbeamcode = self._recv_dtype(np.uint32)
        self.rbeamazm = self._recv_dtype(np.float32)
        self.rbeamwidth = self._recv_dtype(np.float32)

        self.status = self._recv_dtype(np.int32)
        self.name = self._recv_dtype(np.uint8, nitems = 80)
        self.description = self._recv_dtype(np.uint8, nitems = 120)

    def _recv_dtype(self, dtype, nitems = 1):
        return recv_dtype(sock, dtype, nitems) 

class register_seq_handler(dmsg_handler):
    def process(self):
        crtlprm = self._recv_ctrlprm()
        r = client.radar - 1
        c = client.channel - 1
        
        seq_idx = self._recv_dtype(np.int32)
        tx_tsg_idx = self._recv_dtype(np.int32)
        tx_tsg_len = self._recv_dtype(np.int32)
        tx_tsg_step = self._recv_dtype(np.int32)

        # tx.allocate_pulseseq_mem(index);
        
        tx_tsg_rep = self._recv_dtype(np.uint8, tx_tsg_len)
        tx_tsg_code = self._recv_dtype(np.uint8, tx_tsg_len)

        # old_seq_id=-10;
        # new_seq_id=-1;

class ctrlprog_ready_handler(dmsg_handler):
    def process(self):
        crtlprm = self._recv_ctrlprm()
        r = client.radar - 1
        c = client.channel - 1


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
        crtlprm = self._recv_ctrlprm()
        r = client.radar - 1
        c = client.channel - 1
        old_seq_id=-10;
        new_seq_id=-1;


class wait_handler(dmsg_handler):
    def process(self):
        pass

class pretrigger_handler(dmsg_handler):
    def process(self):
        # setup for next trigger\n";
        new_seq_id = -1;
        new_beam = client.tbeam;
        for i in range(numclients):
            r = clients[i].radar - 1;
            c = clients[i].channel - 1;
            new_seq_id += r*1000 + c*100 + clients[i].current_pulseseq_index+1
    
        #  tx.make_bb_vecs(clients[0].trise);
        
        # provide fresh samples to usrp_driver.cpp shared memory if beam or sequence has changed
        if (new_seq_id != old_seq_id) or (new_beam != old_beam):
            iseq = 0
            # send USRP_SETUP commands to all USRPs
            

            # tx.allocate_rf_vec_mem();
            # tx.zero_rf_vec(iradar);
            # tx_process_gpu new samples
            '''    tx_process_gpu(
                        tx.get_bb_vec_ptr(iradar),
                        tx.get_rf_vec_ptrs(iradar),
                        tx.get_num_bb_samples(),
                        tx.get_num_rf_samples(),
                        usrp->get_tx_freq(),
                        TXRATE,
                        tx.get_freqs(iradar), // List of center frequencies for this radar
                        tx.get_time_delays(iradar),
                        tx.get_num_channels(iradar), // Number of channels for this radar
                        tx.get_num_ants_per_radar()); //number of antennas per radar
            }'''

        if new_seq_id != old_seq_id:
            # setup usrp if new sequence
            # set rx/tx frequency, rate, 
            # tx.make_tr_times(&bad_transmit_times);
            pass

        if new_seq_id < 0:
            old_seq_id = -10
        else:
            old_seq_id = new_seq_id

        new_seq_id = -1
        old_beam = new_beam

        '''
                send_data(msgsock, &bad_transmit_times.length, sizeof(bad_transmit_times.length));
                send_data(msgsock, bad_transmit_times.start_usec, sizeof(uint32_t)*bad_transmit_times.length
                send_data(msgsock, bad_transmit_times.duration_usec, sizeof(uint32_t)*bad_transmit_times.len
                send_data(msgsock, &msg, sizeof(struct DriverMsg));
        '''
        pass

class trigger_handler(dmsg_handler):
    def process(self):
        '''
             gettimeofday(&t0,NULL);
                    if (verbose > 1 ) std::cout << "Setup for trigger\n";
                    if (verbose>1) std::cout << std::endl;
                    msg.status=0;

                    if(configured) {
                        //First make sure sure we're done receiving the last pulse sequence
                        if (verbose > 1) std::cout << "Waiting on thread.." << std::endl;
                        receive_threads.join_all();

                        if (verbose > 1){
                            std::cout << "client baseband sample rate: " << client.baseband_samplerate << std::e
                            std::cout << "nsamples, rx: " << rx.get_num_rf_samples() << std::endl;
                            std::cout << "nsamples, tx: " << tx.get_num_rf_samples() << std::endl;
                            std::cout << "Usrp sample rate: " << RXRATE << std::endl;

                        }

                        //Start the receive stream thread
                        if(verbose>1) std::cout << "About to start rx thread..\n";
                        rx_thread_status=0;
                        gettimeofday(&t0,NULL);

                        tx.set_rf_vec_ptrs(&tx_vec_ptrs);
                        rx.set_rf_vec_ptrs(&rx_vec_ptrs);
                        /* Toggle the swing buffer for the NEXT pulse sequence
                         * The rx_data::get_num_samples(), get_rf_dptr(), get_bb_dptr(), get_num_channels(), etc
                         * commands still point to the CURRENT pulse sequence */
                        rx.toggle_swing_buffer();

                        start_time = usrp->get_time_now() + 0.01;

                        gettimeofday(&t1,NULL);
                        receive_threads.create_thread(boost::bind(recv_and_hold,
                                    &bad_transmit_times,
                                    usrp,
                                    rx_stream,
                                    rx_vec_ptrs,
                                    rx.get_num_rf_samples(),
                                    start_time,
                                    &rx_thread_status));

                        if(verbose>1) std::cout << "About to start tx thread..\n";
                        //call function to start tx stream simultaneously with rx stream
                        tx_threads.create_thread(boost::bind(tx_worker,
                                    tx_stream,
                                    tx_vec_ptrs,
                                    tx.get_num_rf_samples(),
                                    start_time+20e-6));
                    }
                    /* 
                       if(verbose>1) std::cout << "About to join tx thread..\n";
                       tx_threads.join_all();
                       if(verbose>1) std::cout << "About to join rx thread..\n";
                       receive_threads.join_all();
                       */

                    send_data(msgsock, &msg, sizeof(struct DriverMsg));

                    gettimeofday(&t6,NULL);
                    elapsed=(t6.tv_sec-t0.tv_sec)*1E6;
                    elapsed+=(t6.tv_usec-t0.tv_usec);
        '''
        pass



class posttrigger_handler(dmsg_handler):
    def process(self):
        numclients = 0

        #tx.clear_channel_list(); clear list of tx_freqs and time_delays (time delay between adjacent antennas)
        #rx.clear_channel_list(); clear list of rx frequency offsets? 
        #ready_index[r][c]=-1;

class recv_get_data_handler(dmsg_handler):
    def process(self):
        '''
                        if (verbose>1) std::cout << "RECV_GET_DATA: Waiting on all threads.." << std::endl;

                        gettimeofday(&t6,NULL);
                        tx_threads.join_all();

                        rx_process_threads.join_all();

                        receive_threads.join_all();
                        gettimeofday(&t0,NULL);
                        //rx_thread_status=-1;

                        elapsed=(t6.tv_sec-t1.tv_sec)*1E6;
                        elapsed+=(t6.tv_usec-t1.tv_usec);
                        if (verbose)std::cout << "Rx thread elapsed: " <<
                            elapsed << " usec" << std::endl;
                        elapsed=(t6.tv_sec-t0.tv_sec)*1E6;



                        if (rx_thread_status!=0){
                            std::cerr << "Error, bad status from Rx thread!!\n";
                            rx_status_flag=1;
                        }
                        else {
                            rx_status_flag=0;
                            if (verbose>1) printf("Status okay!!\n");
                        }
                        //rx_status_flag=-5;

                        recv_data(msgsock, &client,sizeof(struct ControlPRM));
                        r=client.radar-1;
                        c=client.channel-1;

                        if (rx.get_num_channels(r) == 0){
                            std::cerr << "No channels for this radar.\n";
                            rx_status_flag=10;
                        }
                        //nclients = 0;
                        //for (r=0;r<MAX_RADARS;r++){
                        //    for (c=0;c<MAX_CHANNELS;c++){
                        //        if (ready_index[r][c] >=0){
                        //            nclients += 1;
                        //        }
                        //    }
                        //}
                        if (rx_status_flag != 0){
                            std::cerr << "Bad GET_DATA status!! " << rx_status_flag << "\t" << "radar: " << client.r
                        }
                        send_data(msgsock, &rx_status_flag, sizeof(rx_status_flag));
                        printf("Sent status flag\n");

                        if (verbose>1)std::cout << "Client asking for rx samples (Radar,Channel): " <<
                            client.radar << " " << client.channel << std::endl;
                        get_data_t0 = usrp->get_time_now();
                        r = client.radar-1; c = client.channel-1;

                        if (verbose > 2){
                            std::cout << "radar: " << r << " channel: " << c << "\n";
                            std::cout << "bb_dptr: " << rx.get_bb_dptr(r) << std::endl;
                            std::cout << "rf dptr: " << rx.get_rf_dptr(r) << std::endl;
                            std::cout << "nrf_samples: " << rx.get_num_rf_samples() << std::endl;
                            std::cout << "nbb_samples: " << rx.get_num_bb_samples() << std::endl;
                            std::cout << "nchannels: " << rx.get_num_channels(r) << std::endl;
                            std::cout << "nants per radar: " << rx.get_num_ants_per_radar() << std::endl;
                            std::cout << "high sample rate: " << (float) RXRATE << std::endl;
                            std::cout << "low sample rate: " << (float) client.baseband_samplerate << std::endl;
                            std::cout << "frequency offsets: \n";
                            for (size_t i=0; i<rx.get_num_channels(r); i++){
                                std::cout << "\t" << (rx.get_freqs(r))[i]/1e3 << " khz"<< std::endl;
                            }
                        }
                        if (verbose>1) std::cout << "About to enter rx_process_gpu()\n";
                        if (rx_status_flag==0){
                            rx_process_threads.create_thread(boost::bind(rx_process_gpu,
                                        rx.get_rf_dptr(r),
                                        rx.get_bb_dptr(r),
                                        rx.get_num_rf_samples(),
                                        rx.get_num_bb_samples(),
                                        rx.get_num_channels(r),
                                        rx.get_num_ants_per_radar(),
                                        (float) RXRATE,
                                        (float) client.baseband_samplerate,
                                        rx.get_freqs(r)));

                            /*if (1) {
                              rx_process_threads.create_thread(boost::bind(rx_txpulse_process_gpu,
                              rx.get_rf_dptr(r),
                              rx.get_num_rf_samples(),
                              rx.get_num_channels(r),
                              rx.get_num_ants_per_radar(),
                              (uint32_t) 2000,
                              (float) RXRATE,
                              rx.get_freqs(r)));
                              }*/
                            if (double_buf==0) rx_process_threads.join_all();

                            if (verbose>1) printf("double_buf: %d\n", double_buf);
                            if (verbose>1) printf("iseq: %i\n", iseq);
                            if (verbose>1)std::cout << "set_bb_vec_ptrs: " << r << " " << c << std::endl;

                            /* Set the pointers to vectors of baseband samples
                             * If double_buf flag is non-zero, then the function will create
                             * pointers to the samples created by the LAST pulse sequence.
                             * This allows the driver to collect samples for the current pulse
                             * sequence and process samples for the previous pulse sequence 
                             * simultaneously*/
                            rx.set_bb_vec_ptrs(r,c, &bb_vec_ptrs,double_buf);

                            beamform_main.resize(rx.get_num_ants_per_radar(), 1);
                            beamform_back.resize(rx.get_num_ants_per_radar(), 1);

                            // TODO: expand for number of channels
                            if(DUMP_RAW) {
                                printf("STARTING RAW DATA DUMP..\n");

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

                            }

                            rx_beamform(
                                    shared_main_addresses[r][c][0],
                                    shared_back_addresses[r][c][0],
                                    &bb_vec_ptrs,
                                    rx.get_num_ants_per_radar(),
                                    0,
                                    //1,
                                    //1,
                                    rx.get_num_bb_samples(),
                                    &beamform_main,
                                    &beamform_back);


                            gettimeofday(&t1,NULL);
                            gettimeofday(&t6,NULL);
                            elapsed=(t6.tv_sec-t1.tv_sec)*1E6;
                            elapsed+=(t6.tv_usec-t1.tv_usec);
                            if (verbose > 1) {
                                std::cout << "Data sent: Elapsed Microseconds: " << elapsed << "\n";
                            }

                            iseq++;

                            if(verbose > 1 ){
                                std::cout << "Radar: " << client.radar <<
                                    "\tChannel: " << client.channel << std::endl;
                                std::cout << "r: " << r << "\tc: " << c << std::endl;
                            }
                            shm_memory=1; // Flag used to indicate to client if shared memory (mmap()) is used.
                            shm_memory=2; // Flag used to indicate to client if shared memory (mmap()) is used.
                            send_data(msgsock, &shm_memory, sizeof(int32_t));
                            printf("GET_DATA: shm_mem: %i\n", shm_memory);
                            frame_offset=0;  // The GC316 cards normally produce rx data w/ a header of length 2 sam
                            send_data(msgsock, &frame_offset, sizeof(int32_t));
                            printf("GET_DATA: frame_offset: %i\n", frame_offset);
                            dma_buffer=0; // Flag used to indicate to client if DMA tranfer is used. 1 for yes.
                            send_data(msgsock, &dma_buffer, sizeof(int32_t));
                            printf("GET_DATA: dma_buffer: %i\n", dma_buffer);
                            //nrx_samples=client.number_of_samples;
                            send_data(msgsock, &client.number_of_samples, sizeof(int32_t));
                            printf("GET_DATA: num samples: %i\n", client.number_of_samples);


                            if (shm_memory==2){
                                printf("Sending data. %i\n", client.number_of_samples);
                                send_data(msgsock, shared_main_addresses[r][c][0], sizeof(uint32_t)*client.number_of
                                printf("Sent main data %lu\n", sizeof(uint32_t)*client.number_of_samples);
                                send_data(msgsock, shared_back_addresses[r][c][0], sizeof(uint32_t)*client.number_of
                                printf("Sent back data %lu\n", sizeof(uint32_t)*client.number_of_samples);
                            }


                            if(IMAGING==0){
                                if(verbose > 1 ) std::cout << "Using shared memory addresses..: " <<
                                    shared_main_addresses[r][c][0] << "\t" << shared_back_addresses[r][c][0] << std:
                            }
                            if (verbose>1)std::cout << "Send data to client successful" << std::endl;
                            msg.status = rx_status_flag;
                        }
                        send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        printf("Sent DriverMsg %lu\n", sizeof(struct DriverMsg));
                        rx_status_flag=0;
                        gettimeofday(&t6,NULL);
                        elapsed=(t6.tv_sec-t0.tv_sec)*1E6;
                        elapsed+=(t6.tv_usec-t0.tv_usec);
                        if (verbose > 1) {
                            std::cout << "Data sent: Elapsed Microseconds: " << elapsed << "\n";
                        }

                        if(verbose>1)std::cout << "Ending RECV_GET_DATA. Elapsed time: " << std::endl;
                        gettimeofday(&t6,NULL);
                        elapsed=(t6.tv_sec-t0.tv_sec)*1E6;
                        elapsed+=(t6.tv_usec-t0.tv_usec);
                        if (verbose > 1) {
                            std::cout << "RECV_GET_DATA Elapsed Microseconds: " << elapsed << "\n";
                        }

                        if(PROFILE_EXIT) {
                            profile_pulsecount++;
                            if (profile_pulsecount >= PROFILE_EXIT) {
                                return 0;
                            }
                        }

        '''
        pass


class clrfreq_handler(dmsg_handler):
    def process(self):
        '''
                       tx_threads.join_all();
                        receive_threads.join_all();
                        rx_process_threads.join_all();

                        if(verbose > 1) std::cout << "Doing clear frequency search!!!" << std::endl;
                        recv_data(msgsock, &clrfreq_parameters,sizeof(struct CLRFreqPRM));
                        recv_data(msgsock, &client,sizeof(struct ControlPRM));
                        //printf("RECV_CLRFREQ for radar %i channel %i",client.radar, client.channel);
                        if (verbose) printf("Doing clear frequency search for radar %d, channel %d\n",client.radar,c
                        nave=0;
                        usable_bandwidth=clrfreq_parameters.end-clrfreq_parameters.start;
                        center=(clrfreq_parameters.start+clrfreq_parameters.end)/2;
                        if(verbose > -1 ){
                            printf("  requested values\n");
                            printf("    start: %d\n",clrfreq_parameters.start);
                            printf("    end: %d\n",clrfreq_parameters.end);
                            printf("    center: %d\n",center);
                            printf("    bandwidth: %lf in Khz\n",(float)usable_bandwidth);
                            printf("    nave:  %d %d\n",nave,clrfreq_parameters.nave);
                        }
                        usable_bandwidth=floor(usable_bandwidth/2)*2;
                        /*
                         *  Set up fft variables
                         */
                        N=(int)pow(2,ceil(log10(1.25*(float)usable_bandwidth)/log10(2)));
                        if(N>1024){
                            N=512;
                            usable_bandwidth=300;
                            start=(int)(center-usable_bandwidth/2+0.49999);
                            end=(int)(center+usable_bandwidth/2+0.49999);
                        }
                        /* 1 kHz fft bins*/

                        //Set the rx center frequency
                        //Set the rx sample rate
                        for(int chan = 0; chan < nrx_antennas; chan++) {
                            usrp->set_rx_freq(1e3*center, chan);
                            usrp->set_rx_rate(1e6, chan);
                            //N = (int) (usrp->get_rx_rate() / 1000);
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
                        if(verbose > 1 ){
                            printf("  search values\n");
                            printf("  start: %d %d\n",start,clrfreq_parameters.start);
                            printf("  end: %d %d\n",end,clrfreq_parameters.end);
                            printf("  center: %d\n",center);
                            printf("  search_bandwidth: %lf in Khz\n",search_bandwidth);
                            printf("  usable_bandwidth: %d in Khz\n",usable_bandwidth);
                            printf("  unusable_sideband: %lf in Khz\n",unusable_sideband);
                            printf("  N: %d\n",N);
                            printf("Malloc fftw_complex arrays %d\n",N);
                        }

                        //if(pwr!=NULL) {free(pwr);pwr=NULL;}
                        //pwr = (double*) malloc(N*sizeof(double));
                        pwr.clear();
                        pwr2.clear();
                        pwr.resize(N,0);
                                                                    tx_threads.join_all();
                        receive_threads.join_all();
                        rx_process_threads.join_all();

                        if(verbose > 1) std::cout << "Doing clear frequency search!!!" << std::endl;
                        recv_data(msgsock, &clrfreq_parameters,sizeof(struct CLRFreqPRM));
                        recv_data(msgsock, &client,sizeof(struct ControlPRM));
                        //printf("RECV_CLRFREQ for radar %i channel %i",client.radar, client.channel);
                        if (verbose) printf("Doing clear frequency search for radar %d, channel %d\n",client.radar,c
                        nave=0;
                        usable_bandwidth=clrfreq_parameters.end-clrfreq_parameters.start;
                        center=(clrfreq_parameters.start+clrfreq_parameters.end)/2;
                        if(verbose > -1 ){
                            printf("  requested values\n");
                            printf("    start: %d\n",clrfreq_parameters.start);
                            printf("    end: %d\n",clrfreq_parameters.end);
                            printf("    center: %d\n",center);
                            printf("    bandwidth: %lf in Khz\n",(float)usable_bandwidth);
                            printf("    nave:  %d %d\n",nave,clrfreq_parameters.nave);
                        }
                        usable_bandwidth=floor(usable_bandwidth/2)*2;
                        /*
                         *  Set up fft variables
                         */
                        N=(int)pow(2,ceil(log10(1.25*(float)usable_bandwidth)/log10(2)));
                        if(N>1024){
                            N=512;
                            usable_bandwidth=300;
                            start=(int)(center-usable_bandwidth/2+0.49999);
                            end=(int)(center+usable_bandwidth/2+0.49999);
                        }
                        /* 1 kHz fft bins*/

                        //Set the rx center frequency
                        //Set the rx sample rate
                        for(int chan = 0; chan < nrx_antennas; chan++) {
                            usrp->set_rx_freq(1e3*center, chan);
                            usrp->set_rx_rate(1e6, chan);
                            //N = (int) (usrp->get_rx_rate() / 1000);
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
                        if(verbose > 1 ){
                            printf("  search values\n");
                            printf("  start: %d %d\n",start,clrfreq_parameters.start);
                            printf("  end: %d %d\n",end,clrfreq_parameters.end);
                            printf("  center: %d\n",center);
                            printf("  search_bandwidth: %lf in Khz\n",search_bandwidth);
                            printf("  usable_bandwidth: %d in Khz\n",usable_bandwidth);
                            printf("  unusable_sideband: %lf in Khz\n",unusable_sideband);
                            printf("  N: %d\n",N);
                            printf("Malloc fftw_complex arrays %d\n",N);
                        }

                        //if(pwr!=NULL) {free(pwr);pwr=NULL;}
                        //pwr = (double*) malloc(N*sizeof(double));
                        pwr.clear();
                        pwr2.clear();
                        pwr.resize(N,0);
                                             
                        pwr2.resize(usable_bandwidth,0);
                        //for(int i=0;i<N;i++)
                        //  pwr[i]=0.;
                        //if(pwr2!=NULL) {free(pwr2);pwr2=NULL;}
                        //pwr2 = (double*) malloc(N*sizeof(double));

                        if(verbose>1)std::cout << "starting clr freq search\n";
                        std::cout << "beam direction: " << client.tbeam << std::endl;
                        std::cout << "beam direction: " << client.filter_bandwidth << std::endl;

                        //usrp->set_rx_freq(1e3*center);
                        //usrp->set_rx_rate(1e3*center);
                        rx_clrfreq_rval= recv_clr_freq(
                                usrp,
                                rx_stream,
                                center,
                                usable_bandwidth,
                                (int) client.filter_bandwidth/1e3,
                                clrfreq_parameters.nave,
                                10,
                                &pwr2.front());

                        //pwr2 = &pwr[(int)unusable_sideband];

                        if(verbose > 0 ) printf("Send clrfreq data back\n");
                        send_data(msgsock, &clrfreq_parameters, sizeof(struct CLRFreqPRM));
                        send_data(msgsock, &usable_bandwidth, sizeof(int));
                        if(verbose > 1 ) {
                            printf("  final values\n");
                            printf("  start: %d\n",clrfreq_parameters.start);
                            printf("  end: %d\n",clrfreq_parameters.end);
                            printf("  nave: %d\n",clrfreq_parameters.nave);
                            printf("  usable_bandwidth: %d\n",usable_bandwidth);
                        }
                        //for (int i=0; i<usable_bandwidth; i++){
                        //    std::cout << pwr2[i] << std::endl;
                        //}
                        send_data(msgsock, &pwr2.front(), sizeof(double)*usable_bandwidth);
                        send_data(msgsock, &msg, sizeof(struct DriverMsg));

                        //clr_fd = fopen("/tmp/clr_data.txt","a+");
                        //for(int i=0;i<usable_bandwidth;i++){
                        //    printf("%4d: %8d %8.3lf\n",i,(int)(unusable_sideband+start+i),pwr2[i]);
                        //    fprintf(clr_fd,"%4d %8d %8.3lf\n",i,(int)(unusable_sideband+start+i),pwr2[i]);
                        //  }
                        //fclose(clr_fd);

                        //if(pwr!=NULL) {free(pwr);pwr=NULL;}
                        /* The following free causes crash because pwr2 is in use by the arby_server.
                           Does arby_server free() this pointer?  Or is this a memory leak? (AFM)*/
                        //if(pwr2!=NULL) {free(pwr2);pwr2=NULL;}
                        if (verbose > 1) printf("DIO clrfreq setup\n");

                        // TODO: configure RXFE for clear frequency search
                        //      - select beam based on &client 
                        //      - select for non-imaging:

                        if (verbose > 1) printf("DIO clrfreq end\n");
                        break;

        ''' 
        pass

class rxfe_reset_handler(dmsg_handler):
    def process(self):
        kodiak_set_rxfe(usrp, default_rf_settings);

class settings_handler(dmsg_handler):
    def process(self):
        ifmode = self._recv_dtype(np.uint32)
        rf_settings = self.recv_dtype(np.uint32, 8)
        if_settings = self.recv_dtype(np.uint32, 8)

        if ifmode:
            raise NotImplementedError('IF mode is unimplemented')
        
        # set RF settings
        kodiak_set_rxfe(usrp, rf_settings);

def kodiak_set_rxfe(handler, rf_settings):
    pass

class full_clrfreq_handler(dmsg_handler):
    pass

class get_status_handler(dmsg_handler):
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
    GPS_MSG_ERROR : gps_msg_error_handler}

def main():
    # read in config information
    # initialize shared memory buffers
    #   tx pulse
    #   rx samples

    
    # open USRPs drvers and initialize them
    usrps = [] # list of usrps
    usrp_computers = [] # computers to talk to for cuda

    # open arby server socket
    
    while (True):
        dmsg = getDriverMsg(sock)
        handler = dmsg_handlers[dmsg.cmd](sock)
        handler.process()
        handler.respond()



        

    # server to manage usrp drivers
    #  - initialize and synchronize drivers
    #  - handle and translate arbyserver commands
    #  - fetch downconverted samples from cuda_driver(s)
    #  - return shared memory baseband samples to arby_server

    # shared memory is with arby_server
    # populate with information from cuda_driver
    '''
    for(r=0;r<MAX_RADARS;r++){
        for(c=0;c<MAX_CHANNELS;c++){
            ready_index[r][c]=-1;

            sprintf(shm_device,"/receiver_main_%d_%d_%d",r,c,0);
            shm_unlink(shm_device);
            shm_fd=shm_open(shm_device,O_RDWR|O_CREAT,S_IRUSR | S_IWUSR);
            if (ftruncate(shm_fd,MAX_SAMPLES*4) != 0){
                std::cerr << "ftruncate error!!\n";
            }
            shared_main_addresses[r][c][0]=(uint32_t *)mmap(0,MAX_SAMPLES*4,PROT_READ|PROT_WRITE,MAP_SHARED,shm_fd,0);
            close(shm_fd);

            sprintf(shm_device,"/receiver_back_%d_%d_%d",r,c,0);
            shm_unlink(shm_device);
            shm_fd=shm_open(shm_device,O_RDWR|O_CREAT,S_IRUSR | S_IWUSR);
            if (ftruncate(shm_fd,MAX_SAMPLES*4) != 0){
                std::cerr << "ftruncate error!!\n";
            }
            shared_back_addresses[r][c][0]=(uint32_t *)mmap(0,MAX_SAMPLES*4,PROT_READ|PROT_WRITE,MAP_SHARED,shm_fd,0);
            close(shm_fd);
            /*
            // For testing..
            for (i=0;i<MAX_SAMPLES;i++) {
                shared_main_addresses[r][c][0][i]=i;
                shared_back_addresses[r][c][0][i]=i;
            }*/
        }
    }
    '''    

    pass

if __name__ == '__main__':
    main()
