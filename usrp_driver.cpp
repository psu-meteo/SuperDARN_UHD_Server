// usrp_driver
// connect to a USRP using UHD
// wait for connection from usrp_server
//
// look at boost property tree ini parser for usrp_config.ini..
// 
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <signal.h>
#include <iostream>
#include <complex>
#include <semaphore.h>
#include <time.h>
#include <fcntl.h>

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

#define SETUP_WAIT 1
#define SWING0 0
#define SWING1 1
#define SIDEA 0
#define SIDEB 1
#define NSIDES 2 // sides are halves of the usrp x300, eg. rxa and txa slots form side a
#define NSWINGS 2 // swings are slots in the swing buffer

#define START_OFFSET .05


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

void* open_sample_shm(int32_t ant, int32_t side, int32_t swing, size_t shm_size) {
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

sem_t* open_sample_semaphore(int32_t ant, int32_t swing) {
    char sem_name[80];
    sem_t *sem = NULL;
    sprintf(sem_name,"/semaphore_ant_%d_swing_%d", ant, swing);
    
    sem = sem_open(sem_name, 0);

    if (sem == SEM_FAILED) {
        sem = NULL;
        fprintf(stderr, "Getting a handle to the semaphore failed; errno is %d", errno);
    }
    return sem;
}

void lock_semaphore(int32_t swing, sem_t **sem_arr)
{
    sem_wait(sem_arr[swing]);
}

void unlock_semaphore(int32_t swing, sem_t **sem_arr)
{
    sem_post(sem_arr[swing]);

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

double sock_get_float64(int32_t serversock)
{
   double d;
   ssize_t status = recv(serversock, &d, sizeof(double), 0)
   if(status != sizeof(double)) {
        fprintf(stderr, "error receiving float64");
   }
   return d;
}
uint32_t sock_get_uint32(int32_t serversock)
{
   uint32_t d;
   ssize_t status = recv(int32_t serversock, &d, sizeof(uint32_t), 0)
   if(status != sizeof(uint32_t)) {
        fprintf(stderr, "error receiving uint32_t");
   }
   return d;
}

uint64_t sock_get_uint64(int32_t serversock)
{
   uint64_t d;
   ssize_t status = recv(serversock, &d, sizeof(uint64_t), 0)
   if(status != sizeof(uint64_t)) {
        fprintf(stderr, "error receiving uint64_t");
   }
   return d;
}

uint8_t sock_get_uint8(int32_t serversock)
{
   uint8_t d;
   ssize_t status = recv(serversock, &d, sizeof(uint8_t), 0)
   if(status != sizeof(uint8_t)) {
        fprintf(stderr, "error receiving uint8_t");
   }
   return d;
}





int UHD_SAFE_MAIN(int argc, char *argv[]){
    // connect to usrp_server
    //
    // wait for pulses
    //      get pulse sequence information, generate pulse sequence
    //      get pulse times
    //          line up pulse samples
    //          line up gpio
    //          line up rx requests
    
    // example usage:
    // ./usrp_server
    uhd::set_thread_priority_safe(); 

    std::string args, txsubdev, rxsubdev, ref;
    
    size_t shm_size, num_rx_samps, tx_pulse_length;
    int32_t verbose = 0; 
    int32_t rx_worker_status;

    void *shm_arr[NSWINGS][NSWINGS];
    sem_t *sem_arr[NSWINGS][NSWINGS];
    uint32_t state = ST_INIT;
    uint32_t ant = 0;
    uint32_t swing = SWING0;
    size_t num_requested_samples = 0; 
    uint32_t npulses = 0;

    uhd::time_spec_t get_data_t0;
    uhd::time_spec_t get_data_t1;
    uhd::time_spec_t start_time;
    
    std::vector<double> pulse_offsets;

    boost::thread_group rx_threads;
    boost::thread_group tx_threads;

    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    boost::this_thread::sleep(boost::posix_time::seconds(SETUP_WAIT));

    uhd::stream_args_t stream_args("sc16", "sc16"); // TODO: expand for dual polarization
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);
    
    signal(SIGINT, siginthandler);

    // open shared rx sample shared memory buffers created by cuda_driver.py
    shm_arr[SWING0][SIDEA] = open_sample_shm(ant, SIDEA, SWING0, shm_size);
    //shm_arr[SWING0][SIDEB] = open_sample_shm(ant, SIDEB, SWING0, shm_size);
    shm_arr[SWING1][SIDEA] = open_sample_shm(ant, SIDEA, SWING1, shm_size);
    //shm_arr[SWING1][SIDEB] = open_sample_shm(ant, SIDEB, SWING1, shm_size);


    
    // open shared rx sample shared memorby buffer semaphores created by cuda_driver.py
    sem_arr[SWING0] = open_sample_semaphore(ant, SWING0);
    //sem_arr[SWING0][SIDEB] = open_sample_semaphore(ant, SIDEB, SWING0);
    sem_arr[SWING1] = open_sample_semaphore(ant, SWING1);
    //sem_arr[SWING1][SIDEB] = open_sample_semaphore(ant, SIDEB, SWING1);

    // init rxfe
    kodiak_init_rxfe(usrp);

    // open existing shared memory created by cuda_driver.py
    // swing a and swing b
    boost::this_thread::sleep(boost::posix_time::seconds(SETUP_WAIT));
    
    while(true) {
        uint8_t command = sock_get_uint8(serversock);

        switch(command) {
            case USRP_SETUP:
                double txrate, rxrate, txfreq, rxfreq;
                
                txfreq = sock_get_float64(serversock);
                rxfreq = sock_get_float64(serversock);
                txrate = sock_get_float64(serversock);
                rxrate = sock_get_float64(serversock);
                npulses = sock_get_uint32(serversock);
                num_requested_samples = sock_get_uint64(serversock);
                
                pulse_offsets.resize(npulses);
                for(int i = 0; i < npulses; i++) {
                    pulse_offsets[i] = sock_get_float64(serversock); 
                }
                
                usrp->set_rx_rate(rxrate);
                usrp->set_rx_rate(txrate);
                
                uhd::tune_request_t tune_request(rxfreq);
                usrp->set_rx_freq(tune_request);
                uhd::tune_request_t tune_request(txfreq);
                usrp->set_rx_freq(tune_request);

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
           
            case RXFE_SET:
                RXFESettings rf_settings;
                rf_settings.amp1 = sock_get_uint8(serversock);
                rf_settings.amp2 = sock_get_uint8(serversock);
                uint8_t att = sock_get_uint8(serversock);
                rf_settings.att1 = (att >> 0) & 0x01;
                rf_settings.att2 = (att >> 1) & 0x01;
                rf_settings.att3 = (att >> 2) & 0x01;
                rf_settings.att4 = (att >> 3) & 0x01;
                kodiak_set_rxfe(usrp, rf_settings);
                break;

            case TRIGGER_PULSE:
                std::vector<uhd::time_spec_t> pulse_times (pulse_offsets.size());
                if (state != ST_READY) {
                    ;
                    // TODO: return busy message
                }
                else {
                    state = ST_PULSE;
                    lock_semaphore(swing, sem_arr);
                    // todo: num_requested_samples
                    start_time = usrp->get_time_now() + START_OFFSET;

                    for(uint32_t p_i = 0; p_i < pulse_offsets.size(); p_i++) {
                        pulse_times[p_i] = start_time + pulse_offsets[p_i];
                    }

                    rx_threads.create_thread(boost::bind(uhd_worker, usrp, rx_stream, shm_arr[swing], num_requested_samples, start_time, rx_worker_status); 
                    tx_threads.create_thread(boost::bind(uhd_worker, tx_stream, pulse_seq_ptr, pulse_tx_samps, usrp->get_tx_rate(), pulse_times)); 
                    swing = toggle_swing(swing); 
                }

                break;
            
            case READY_DATA:
                uhd_threads.join_all(); // wait for transmit threads to finish, drawn from shared memory..
                // TODO: send int32_t status
                // TODO: send number of antennas
                // TODO: send antennas 
                // TODO: send back data..
                unlock_semaphore(swing, sem_arr);
                state = ST_READY; 
                break;

            case CLRFREQ:
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

            default:
                break;
        }

        // create state machine to test triggering, simulating commands
    }
    
    return 0;
}

void siginthandler(int sigint)
{
    // rc = munmap(pSharedMemory, (size_t)params.size)
    // rc = sem_close(the_semaphore); 
    // close(msgsock);
    exit(1);
}
