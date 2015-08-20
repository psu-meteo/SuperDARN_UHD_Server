// usrp_driver
// connect to a USRP using UHD
// wait for connection from usrp_server
//
// look at boost property tree ini parser for usrp_config.ini..
// 
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <signal.h>
#include <iostream>
#include <complex>
#include <vector>
#include <semaphore.h>
#include <time.h>
#include <fcntl.h>

#include <argtable2.h>
#include <uhd/types/tune_request.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/transport/udp_simple.hpp>
#include <uhd/exception.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/cstdint.hpp>

#include "burst_worker.h"
#include "recv_and_hold.h"
#include "dio.h"

#define SETUP_WAIT 1
#define SWING0 0
#define SWING1 1
#define SIDEA 0
#define SIDEB 1
#define NSIDES 2 // sides are halves of the usrp x300, eg. rxa and txa slots form side a
#define NSWINGS 2 // swings are slots in the swing buffer

#define START_OFFSET .05

#define ARG_MAXERRORS 10

// these should be in a config file
#define TXSHM_SIZE 1280000000
#define RXSHM_SIZE 1024000
#define USRP_DRIVER_PORT 40041

#define USRP_SETUP 's'
#define RXFE_SET 'r'
#define CLRFREQ 'c'
#define READY_DATA 'd'
#define TRIGGER_PULSE 't'

enum driver_states
{
    ST_INIT,
    ST_READY,
    ST_PULSE
};

namespace po = boost::program_options;
int32_t driversock = 0;

void *open_sample_shm(int32_t ant, int32_t side, int32_t swing, size_t shm_size) {
    void *pshm = NULL;
    char shm_device[80];
    int32_t shm_fd;

    sprintf(shm_device,"/shm_ant_%d_side_%d_swing_%d", ant, side, swing);
    shm_fd = shm_open(shm_device, O_RDWR, S_IRUSR | S_IWUSR);
    if (shm_fd == -1) {
        fprintf(stderr, "Couldn't get a handle to the shared memory; errno is %d\n", errno);
    }
    
    if (ftruncate(shm_fd, shm_size) != 0){
        fprintf(stderr, "ftruncate error!! Shared memory buffer is undersized.\n");
    }

    pshm = mmap((void *)0, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (pshm == MAP_FAILED) {
        fprintf(stderr, "MMapping shared memory failed; errno is %d", errno);
    }
    
    close(shm_fd);    
    return pshm;
}

sem_t open_sample_semaphore(int32_t ant, int32_t swing) {
    char sem_name[80];
    sem_t *sem = NULL;
    sprintf(sem_name,"/semaphore_ant_%d_swing_%d", ant, swing);
    
    sem = sem_open(sem_name, 0);

    if (sem == SEM_FAILED) {
        sem = NULL;
        fprintf(stderr, "Getting a handle to the semaphore failed; errno is %d", errno);
    }

    return *sem;
}

void lock_semaphore(int32_t swing, sem_t sem)
{
    sem_wait(&sem);
}

void unlock_semaphore(int32_t swing, sem_t sem)
{
    sem_post(&sem);

}

uint32_t toggle_swing(uint32_t swing) {
    if (swing == SWING0) {
        return SWING1;
    }
    else if (swing == SWING1) {
        return SWING0;
    }
    else {
        fprintf(stderr, "Unknown swing value, reverting to SWING0");
    }
    return SWING0; 
}

double sock_get_float64(int32_t sock)
{
   double d;
   ssize_t status = recv(sock, &d, sizeof(double), 0);
   if(status != sizeof(double)) {
        fprintf(stderr, "error receiving float64");
   }
   return d;
}

uint32_t sock_get_uint32(int32_t sock)
{
   uint32_t d;
   ssize_t status = recv(sock, &d, sizeof(uint32_t), 0);
   if(status != sizeof(uint32_t)) {
        fprintf(stderr, "error receiving uint32_t");
   }
   return d;
}

uint32_t sock_send_int32(int32_t sock, int32_t d)
{
   ssize_t status = send(sock, &d, sizeof(uint32_t), 0);
   if(status != sizeof(uint32_t)) {
        fprintf(stderr, "error receiving uint32_t");
   }
   return d;
}


uint64_t sock_get_uint64(int32_t sock)
{
   uint64_t d;
   ssize_t status = recv(sock, &d, sizeof(uint64_t), 0);
   if(status != sizeof(uint64_t)) {
        fprintf(stderr, "error receiving uint64_t");
   }
   return d;
}

uint8_t sock_get_uint8(int32_t sock)
{
   uint8_t d;
   ssize_t status = recv(sock, &d, sizeof(uint8_t), 0);
   if(status != sizeof(uint8_t)) {
        fprintf(stderr, "error receiving uint8_t");
   }
   return d;
}


void siginthandler(int sigint)
{
    // rc = munmap(pSharedMemory, (size_t)params.size)
    // rc = sem_close(the_semaphore); 
    close(driversock);
    exit(1);
}



int UHD_SAFE_MAIN(int argc, char *argv[]){
    // example usage:
    // ./usrp_driver --antenna 1 --host usrp1 
    // spawn a usrp_server for each usrp
    
    uhd::set_thread_priority_safe(); 

    std::string txsubdev, rxsubdev, ref, usrpargs;
    
    size_t rxshm_size = RXSHM_SIZE; 
    size_t txshm_size = TXSHM_SIZE;

    int32_t verbose = 0; 
    int32_t rx_worker_status;

    // clean up to fix it later..
    void *shm_swingarx, *shm_swingbrx;
    void *shm_swingatx, *shm_swingbtx;
    sem_t sem_swinga, sem_swingb;

    uint32_t state = ST_INIT;
    uint32_t ant = 0;
    uint32_t swing = SWING0;
    size_t num_requested_samples = 0; 
    uint32_t npulses, nerrors;

    int32_t sockopt;
    struct sockaddr_in sockaddr;
    char *usrphost = NULL;

    uhd::time_spec_t get_data_t0;
    uhd::time_spec_t get_data_t1;
    uhd::time_spec_t start_time;
    
    std::vector<double> pulse_offsets;

    boost::thread_group uhd_threads;
   
    // process command line arguments
    struct arg_lit  *al_help   = arg_lit0(NULL, "help", "Prints help information and then exits");
    struct arg_int  *ai_ant    = arg_int0(NULL, "antenna", NULL,"Antenna position index for the USRP (0-19)"); 
    struct arg_str  *as_host   = arg_str0(NULL, "host", NULL,"Hostname or IP address of USRP to control (e.g usrp1)"); 
    struct arg_end  *ae_argend = arg_end(ARG_MAXERRORS);
    void* argtable[] = {al_help, ai_ant, as_host, ae_argend};

    nerrors = arg_parse(argc,argv,argtable);
    if (nerrors > 0) {
        arg_print_errors(stdout,ae_argend,"usrp_driver");
    }
    if (argc == 1) {
        printf("No arguments found, try running again with --help for more information.\n");
        exit(1);
    }
    if(al_help->count > 0) {
        printf("Usage: ");
        arg_print_syntax(stdout,argtable,"\n");
        arg_print_glossary(stdout,argtable,"  %-25s %s\n");
        arg_freetable(argtable, sizeof(argtable)/sizeof(argtable[0]));
        return 0;
    }
    
    if(ai_ant->count == 0 || ai_ant->ival[0] < 0 || ai_ant->ival[0] > 19) {
        printf("No or invalid antenna index, exiting...");
        return 0;
    }
    
    if(as_host->sval == NULL) {
        printf("Missing usrp host command line argument, exiting...");
        return 0;
    }
    ant = ai_ant->ival[0];
    std::string usrpargs(as_host->sval[1]);
    usrpargs = "addr0=" + usrpargs; 
    //usrphost = malloc((strlen(as_host->sval[0]) + 1) * sizeof(char));
     
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(usrpargs);
    boost::this_thread::sleep(boost::posix_time::seconds(SETUP_WAIT));

    uhd::stream_args_t stream_args("sc16", "sc16"); // TODO: expand for dual polarization
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);
    
    signal(SIGINT, siginthandler);
    
    // open shared rx sample shared memory buffers created by cuda_driver.py
    shm_swingarx = open_sample_shm(ant, SIDEA, SWING0, rxshm_size);
    shm_swingbrx = open_sample_shm(ant, SIDEA, SWING1, rxshm_size);

    shm_swingatx = open_sample_shm(ant, SIDEA, SWING0, txshm_size);
    shm_swingatx = open_sample_shm(ant, SIDEA, SWING0, txshm_size);

    // open shared rx sample shared memory buffer semaphores created by cuda_driver.py
    sem_swinga = open_sample_semaphore(ant, SWING0);
    sem_swingb = open_sample_semaphore(ant, SWING1);

    // init rxfe
    kodiak_init_rxfe(usrp);

    // open existing shared memory created by cuda_driver.py
    // swing a and swing b
    boost::this_thread::sleep(boost::posix_time::seconds(SETUP_WAIT));
    
    // bind to socket for communication with usrp_server.py
    driversock = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr.sin_family = AF_INET;
    // TODO: maybe limit addr to interface connected to usrp_server
    sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    sockaddr.sin_port = htonl(USRP_DRIVER_PORT);
    sockopt = 1;
    setsockopt(driversock, SOL_SOCKET, SO_REUSEADDR, &sockopt, sizeof(int32_t));

    if( bind(driversock, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0){
            perror("binding tx stream socket");
            exit(1);
    }

  
    while(true) {
        uint8_t command = sock_get_uint8(driversock);

        switch(command) {
            case USRP_SETUP: {
                double txrate, rxrate, txfreq, rxfreq;
                
                txfreq = sock_get_float64(driversock);
                rxfreq = sock_get_float64(driversock);
                txrate = sock_get_float64(driversock);
                rxrate = sock_get_float64(driversock);
                npulses = sock_get_uint32(driversock);
                num_requested_samples = sock_get_uint64(driversock);
                
                pulse_offsets.resize(npulses);
                for(uint32_t i = 0; i < npulses; i++) {
                    pulse_offsets[i] = sock_get_float64(driversock); 
                }
                
                usrp->set_rx_rate(rxrate);
                usrp->set_tx_rate(txrate);
                
                usrp->set_rx_freq(rxfreq);
                usrp->set_tx_freq(txfreq);

                if(verbose) {
                    std::cout << boost::format("Actual RX Freq: %f MHz...") % (usrp->get_rx_freq()/1e6) << std::endl << std::endl;
                    std::cout << boost::format("Actual RX Rate: %f Msps...") % (usrp->get_rx_rate()/1e6) << std::endl << std::endl;
                    std::cout << boost::format("Actual TX Freq: %f MHz...") % (usrp->get_tx_freq()/1e6) << std::endl << std::endl;
                    std::cout << boost::format("Actual TX Rate: %f Msps...") % (usrp->get_tx_rate()/1e6) << std::endl << std::endl;
                }
                
                // sync clock with external 10 MHz and PPS
                usrp->set_clock_source("external");
                usrp->set_time_source("external");
                const uhd::time_spec_t last_pps_time = usrp->get_time_last_pps();
                while (last_pps_time == usrp->get_time_last_pps()) {
                    usleep(5e4);
                }
                usrp->set_time_next_pps(uhd::time_spec_t(0.0), 0);
                boost::this_thread::sleep(boost::posix_time::milliseconds(1100));
                
                state = ST_READY; 
                break;
                }
            case RXFE_SET: {
                RXFESettings rf_settings;
                rf_settings.amp1 = sock_get_uint8(driversock);
                rf_settings.amp2 = sock_get_uint8(driversock);
                uint8_t att = sock_get_uint8(driversock);
                rf_settings.att1 = (att >> 0) & 0x01;
                rf_settings.att2 = (att >> 1) & 0x01;
                rf_settings.att3 = (att >> 2) & 0x01;
                rf_settings.att4 = (att >> 3) & 0x01;
                kodiak_set_rxfe(usrp, rf_settings);
                break;
                }

            case TRIGGER_PULSE: {
                std::vector<uhd::time_spec_t> pulse_times (pulse_offsets.size());
                if (state != ST_READY) {
                    ;
                    // TODO: return busy message
                }
                else {
                    state = ST_PULSE;
                    lock_semaphore(swing, sem_swinga);
                    // todo: num_requested_samples
                    start_time = usrp->get_time_now() + START_OFFSET;

                    for(uint32_t p_i = 0; p_i < pulse_offsets.size(); p_i++) {
                        pulse_times[p_i] = start_time + pulse_offsets[p_i];
                    }

                    // TODO: arbitrarily create arguements for testing.. these shouldn't be here
                    // create vector pointers to arrays of of complex int16_ts..
                    std::vector<std::complex<int16_t> *> pulse_seq_ptrs; // TODO: not intialized

                    uint32_t pulse_tx_samps = 0; 
                    // TODO: remove above...
                    
                    uhd_threads.create_thread(boost::bind(recv_and_hold,usrp, rx_stream, pulse_seq_ptrs, num_requested_samples, start_time, &rx_worker_status)); 
                    uhd_threads.create_thread(boost::bind(tx_worker, tx_stream, pulse_seq_ptrs, pulse_tx_samps, usrp->get_tx_rate(), pulse_times)); 

                    swing = toggle_swing(swing); 
                }

                break;
                }

            case READY_DATA: {
                uhd_threads.join_all(); // wait for transmit threads to finish, drawn from shared memory..

                sock_send_int32_t(driversock, status);
                sock_send_int32_t(driversock, nantennas);
                // TODO: send antenna information 
                sock_send_int32_t(driversock, status);
                sock_send_int32_t(driversock, status);
                // TODO: send int32_t status
                // TODO: send number of antennas
                // TODO: send antennas 
                // TODO: send back data..
                unlock_semaphore(swing, sem_swinga);
                state = ST_READY; 
                break;
                }

            case CLRFREQ: {
               // TODO: synchronize clr_freq scan.. 
               /*
               rx_clrfreq_rval= recv_clr_freq(
                                usrp,
                                rx_stream,
                                center,
                                usable_bandwidth,
                                (int) client.filter_bandwidth/1e3,
                                clrfreq_parameters.nave,
                                10,
                                &pwr2.front()); 
                // TODO: send back raw samples for beamforming on server
                */
                break;
                }

            default: {
                break;
            }
        }

        // create state machine to test triggering, simulating commands
    }
    
    return 0;
}

