#include <complex>
#include <vector>
#include <queue>
#include <iostream>
#include <iomanip>

//Added by Alex for usrp
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/exception.hpp>
#include <boost/thread.hpp>
#include <thread>
#include <math.h>
#include <control_program.h>

#include "recv_and_hold.h"

#define SYNC_PINS 0x02
#define TR_PINS 0x18
#define MIMIC_PINS 0x1800 
#define USEMIMIC 1 // add extra pulses with some offset from the transmit pulses to gate an external synthetic target

#define TR_TX 0x08 // pins on TX(A/B) connected to control board for TR
#define TR_RX 0x10 

#define MIMIC_TX 0x0800 
#define MIMIC_RX 0x1000
#define MIMIC_RANGE (5100) // microseconds  (765 km)
#define FAULT 0x0001

#define RX_OFFSET 290e-6 // microseconds, alex had 450-e6 set here

#define MANUAL_CONTROL 0x0

struct GPIOCommand {
    uhd::time_spec_t cmd_time;
    std::string port;
    std::string gpiocmd;
    uint32_t value;
    uint32_t mask;
};

class CompareTime {
public:
    bool operator()(GPIOCommand& t1, GPIOCommand& t2)
    {
       return (t1.cmd_time > t2.cmd_time);
    }
};


/***********************************************************************
 * recv_and_hold() function
 * A function to be used to receive samples from the USRP and
 * hold them in the network socket buffer. *client_buff_ptrs point to
 * the memory locations of each antenna's samples.   Meant to operate in its
 * own thread context so that it does not block the execution in main()
 * Also handles the former duties of the timing card...
 **********************************************************************/
void recv_and_hold(
    struct TRTimes* trtimes,
    uhd::usrp::multi_usrp::sptr usrp,
    uhd::rx_streamer::sptr rx_stream,
    std::vector<std::complex<int16_t> *> client_buff_ptrs,
    size_t num_requested_samples,
    uhd::time_spec_t start_time,
    int *return_status
){
    std::vector<std::vector<std::complex<short> > >  temp_buffs;
    std::vector<std::complex<short> *> temp_buff_ptrs;
    
    GPIOCommand c; // struct to hold command information so gpio commands can be created out of temporal order, sorted, and issued in order
    std::priority_queue<GPIOCommand, std::vector<GPIOCommand>, CompareTime> cmdq;

    //setup streaming
    uhd::rx_metadata_t md;
    float timeout = 0.1;
    //num_requested_samples=(int)num_requested_samples;
    uhd::stream_cmd_t stream_cmd = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE;
    stream_cmd.num_samps = num_requested_samples;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec = start_time+RX_OFFSET; // why 450 microseconds...?
    //stream_cmd.time_spec = start_time+010e-6;
    
    // setup gpio direction and control
    usrp->set_gpio_attr("TXA","CTRL",MANUAL_CONTROL,SYNC_PINS);
    usrp->set_gpio_attr("TXA","CTRL",MANUAL_CONTROL,TR_PINS);
    usrp->set_gpio_attr("TXA","CTRL",MANUAL_CONTROL,MIMIC_PINS);

    usrp->set_gpio_attr("TXA","DDR",SYNC_PINS,SYNC_PINS);
    usrp->set_gpio_attr("TXA","DDR",TR_PINS, TR_PINS);
    usrp->set_gpio_attr("TXA","DDR",MIMIC_PINS, MIMIC_PINS);
    
    usrp->issue_stream_cmd(stream_cmd);
    
    // set sync pin
    usrp->set_command_time(start_time);
    usrp->set_gpio_attr("TXA","OUT",SYNC_PINS, SYNC_PINS);

    // lower sync pin when rx streaming starts
    c.port = "TXA";
    c.gpiocmd = "OUT";
    c.mask = SYNC_PINS;
    c.value = 0;
    c.cmd_time = start_time.get_real_secs() + RX_OFFSET;
    cmdq.push(c);

    // formerly leave sync pin on for 1 us
    //usrp->set_command_time(start_time+1e-6);
    //usrp->set_gpio_attr("TXA","OUT",0x00,SYNC_PINS);
   
    //std::cout << "Printing out TR logic signals\n";
    //for (int i=0; i<npulses; i++){
    //    std::cout << i << " " << trtimes[i] << " " << trdurations[i] << std::endl;
    //}
    //std::cout << "Sending out TR logic signals\n";
    
    // queue up gpio commands for T/R signals
    for (int i=0; i<trtimes->length; i++){
        c.port = "TXA";
        c.gpiocmd = "OUT";
        c.mask = TR_PINS;
        c.value = TR_TX;
        c.cmd_time = start_time.get_real_secs()+1e-6*trtimes->start_usec[i];
        cmdq.push(c);
        
        c.value = TR_RX;
        c.cmd_time = start_time+1e-6*(trtimes->start_usec[i]+trtimes->duration_usec[i]);
        cmdq.push(c);
    }
    
    
    // queue up gpio commands for mimic/aux signals
    if(USEMIMIC) {
        for (int i=0; i<trtimes->length; i++){
            c.port = "TXA";
            c.gpiocmd = "OUT";
            c.mask = MIMIC_PINS;
            c.value = MIMIC_TX;
            c.cmd_time = start_time.get_real_secs()+1e-6*(trtimes->start_usec[i] + MIMIC_RANGE);
            cmdq.push(c);
            
            c.value = MIMIC_RX;
            c.cmd_time = start_time+1e-6*(trtimes->start_usec[i]+trtimes->duration_usec[i] + MIMIC_RANGE);
            cmdq.push(c);
        }
    }

    // issue gpio commands in time sorted order 
    // set_command_time must be sent in temporal order, they are sent into a fifo queue on the usrp and will block until executed
    // clear_command_time does not clear or bypass the buffer as of 10/2014..
    while (!cmdq.empty()) {
        c = cmdq.top();
        usrp->set_command_time(uhd::time_spec_t(c.cmd_time));
        usrp->set_gpio_attr(c.port,c.gpiocmd,c.value,c.mask);
        cmdq.pop();
    }        
    usrp->clear_command_time();

    // commands to the USRP are now blocked until the pulse sequence is completed
    // usrp_set_command_time will delay any future commands until the command queue is empty (once the pulse sequence including any diagnostic targets is completed)
    
    // the maximum size of the queue is 64
    // a dual polarization radar with timing on separate daughtercards and a 7 pulse sequence plus sync and two mimic targets
    // would have 2 * 2 * 2 * 7 + 2 = 60 entries in the buffer..
    // for more complicated pulse sequences, we may need to fill the buffer partway through the pulse sequence.. 
    
    md.error_code = uhd::rx_metadata_t::ERROR_CODE_NONE;
    size_t num_rx_samps = rx_stream->recv(client_buff_ptrs, num_requested_samples, md, timeout);
	if (num_rx_samps != num_requested_samples){
        *return_status=-1;
		uhd::time_spec_t rx_error_time = usrp->get_time_now();
		std::cerr << "Error in receiving samples..(" << rx_error_time.get_real_secs() << ")\t";;
		std::cerr << "Samples rx'ed: " << num_rx_samps << 
			" (expected " << num_requested_samples << ")" << std::endl;
	}

    if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
    uhd::time_spec_t rx_error_time = usrp->get_time_now();
        std::cerr << "Timeout encountered at " << rx_error_time.get_real_secs() << std::endl;
    *return_status=-1;
    }
    if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW){
    uhd::time_spec_t rx_error_time = usrp->get_time_now();
        std::cerr << "Overflow encountered at " << rx_error_time.get_real_secs() << std::endl;
    *return_status=-1;
    }
    if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
    uhd::time_spec_t rx_error_time = usrp->get_time_now();
    std::cerr << "start time: " << start_time.get_real_secs() << std::endl;
        std::cerr << "Unexpected error code " << md.error_code <<
    " encountered at " << rx_error_time.get_real_secs() << std::endl;
    *return_status=-1;
    }
    if (md.out_of_sequence){
    uhd::time_spec_t rx_error_time = usrp->get_time_now();
    std::cerr << "start time: " << start_time.get_real_secs() << std::endl;
        std::cerr << "Packets out of order " << 
    " encountered at " << rx_error_time.get_real_secs() << std::endl;
    *return_status=-1;
    }

    //for (size_t i=0;i<usrp->get_rx_num_channels();i++)
    //	client_buff_ptrs[i] = &temp_buffs[i].front();

    //gettimeofday(&rt1,NULL); 
    //if(verbose > -1){
    	//std::cout << "recv_and_hold() elapsed time: " << 1e6*(rt1.tv_sec-rt0.tv_sec)+(rt1.tv_usec-rt0.tv_usec) << " usec" << std::endl;
    	//std::cout << "Expected time: " << 300.*(float)num_requested_samples << std::endl;
    //for (size_t i=0;i<usrp->get_rx_num_channels();i++)
    //	client_buff_ptrs[i] = &temp_buffs[i].front();

    /*if(verbose > 1 && *return_status==0){
	    std::cout << "recv_and_hold() succesful\n";
    }*/
}

