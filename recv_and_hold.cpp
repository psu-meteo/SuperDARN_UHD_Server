#include <complex>
#include <vector>
#include <queue>
#include <iostream>
#include <iomanip>
#include <thread>
#include <math.h>

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/exception.hpp>
#include <boost/thread.hpp>

#include "recv_and_hold.h"
#include "usrp_utils.h"

#define DEBUG 1
#ifdef DEBUG
#define DEBUG_PRINT(...) do{ fprintf( stderr, __VA_ARGS__ ); } while( false )
#else
#define DEBUG_PRINT(...) do{ } while ( false )
#endif


#define TEST_RXWORKER 0

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
#define SYNC_OFFSET_START (-500e-6) // start of sync pulse
#define SYNC_OFFSET_END (-400e-6) // start of sync pulse
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
 * hold them in the network socket buffer. *rx_data_buffers points to
 * the memory locations of each antenna's samples.   Meant to operate in its
 * own thread context so that it does not block the execution in main()
 * Also handles the former duties of the timing card...
 **********************************************************************/
void recv_and_hold(
    uhd::usrp::multi_usrp::sptr usrp,
    uhd::rx_streamer::sptr rx_stream,
    std::vector<std::complex<int16_t>> *rx_data_buffer,
    size_t num_requested_samps,
    uhd::time_spec_t start_time,
    int32_t *return_status
){

    DEBUG_PRINT("entering RECV_AND_HOLD\n");
    GPIOCommand c; // struct to hold command information so gpio commands can be created out of temporal order, sorted, and issued in order
    std::priority_queue<GPIOCommand, std::vector<GPIOCommand>, CompareTime> cmdq;

    //setup streaming
    uhd::rx_metadata_t md;
    md.error_code = uhd::rx_metadata_t::ERROR_CODE_NONE;

    double timeout = 1.0;
    
    uhd::stream_cmd_t stream_cmd = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE;
    stream_cmd.num_samps = num_requested_samps;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec = offset_time_spec(start_time, RX_OFFSET);
   
    
    double debugt = usrp->get_time_now().get_real_secs();
    DEBUG_PRINT("RECV_AND_HOLD queing GPIO commands at usrp_time %2.4f\n", debugt);

    // setup gpio direction and control
    usrp->set_gpio_attr("TXA","CTRL",MANUAL_CONTROL,SYNC_PINS);
    usrp->set_gpio_attr("TXA","CTRL",MANUAL_CONTROL,TR_PINS);
    usrp->set_gpio_attr("TXA","CTRL",MANUAL_CONTROL,MIMIC_PINS);

    usrp->set_gpio_attr("TXA","DDR",SYNC_PINS,SYNC_PINS);
    usrp->set_gpio_attr("TXA","DDR",TR_PINS, TR_PINS);
    usrp->set_gpio_attr("TXA","DDR",MIMIC_PINS, MIMIC_PINS);
   
	debugt = usrp->get_time_now().get_real_secs();
    DEBUG_PRINT("RECV_AND_HOLD set gpio attrs at usrp_time %2.4f\n", debugt);
 
    usrp->issue_stream_cmd(stream_cmd);
    
	debugt = usrp->get_time_now().get_real_secs();
    DEBUG_PRINT("RECV_AND_HOLD issued stream command at usrp_time %2.4f\n", debugt);

    // TODO: fix sync?
    //set sync pin
    usrp->set_command_time(offset_time_spec(start_time, SYNC_OFFSET_START));
    usrp->set_gpio_attr("TXA","OUT",SYNC_PINS, SYNC_PINS);

    // lower sync pin when rx streaming starts
    c.port = "TXA";
    c.gpiocmd = "OUT";
    c.mask = SYNC_PINS;
    c.value = 0;
    c.cmd_time = offset_time_spec(start_time, SYNC_OFFSET_END).get_real_secs();
    cmdq.push(c);
	debugt = usrp->get_time_now().get_real_secs();
    DEBUG_PRINT("RECV_AND_HOLD pushed gpio commands at usrp_time %2.4f\n", debugt);
    // issue gpio commands in time sorted order 
    // set_command_time must be sent in temporal order, they are sent into a fifo queue on the usrp and will block until executed
    // clear_command_time does not clear or bypass the buffer as of 10/2014..
    while (!cmdq.empty()) {
        c = cmdq.top();
        usrp->set_command_time(uhd::time_spec_t(c.cmd_time));
        usrp->set_gpio_attr(c.port,c.gpiocmd,c.value,c.mask);
        cmdq.pop();
    }

    debugt = usrp->get_time_now().get_real_secs();
    DEBUG_PRINT("RECV_AND_HOLD set gpio commands at usrp_time %2.4f\n", debugt);

    usrp->clear_command_time();

    /*
    // commands to the USRP are now blocked until the pulse sequence is completed
    // usrp_set_command_time will delay any future commands until the command queue is empty (once the pulse sequence including any diagnostic targets is completed)
    
    // the maximum size of the queue is 64
    // a dual polarization radar with timing on separate daughtercards and a 7 pulse sequence plus sync and two mimic targets
    // would have 2 * 2 * 2 * 7 + 2 = 60 entries in the buffer..
    // for more complicated pulse sequences, we may need to fill the buffer partway through the pulse sequence.. 
    //
    // TODO: set timeout dynamically 

    */
    /*
	debugt = usrp->get_time_now().get_real_secs();

    DEBUG_PRINT("RECV_AND_HOLD recv samples, requesting %d samples at usrp time %.4f for time %.4f, timeout %2.4f\n", (int32_t) num_requested_samps, debugt, stream_cmd.time_spec.get_real_secs(), timeout);
    */
    // reform using /home/kleinjt/repos/uhd/host/examples/rx_samples_to_file.cpp?
    
    size_t num_acc_samps = 0;
    const size_t num_max_request_samps = rx_stream->get_max_num_samps();

    while(num_acc_samps < num_requested_samps) {
        size_t samp_request = std::min(num_max_request_samps, num_requested_samps - num_acc_samps);
        size_t num_rx_samps = rx_stream->recv(&((*rx_data_buffer)[num_acc_samps]), samp_request, md, timeout);
        
        timeout = 0.1;

        //handle the error codes
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) break;
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
            throw std::runtime_error(str(boost::format(
                "Receiver error %s"
            ) % md.strerror()));
        }
                
        //if (DEBUG) {
        //    std::cout << boost::format("Received packet: %u samples, %u full secs, %f frac secs") % num_rx_samps % md.time_spec.get_full_secs() % md.time_spec.get_frac_secs() << std::endl;
        //}
        num_acc_samps += num_rx_samps;
    }

    DEBUG_PRINT("RECV_AND_HOLD fetched samples!\n");
    if(DEBUG) std::cout << boost::format("RECV_AND_HOLD: %u full secs, %f frac secs") % md.time_spec.get_full_secs() % md.time_spec.get_frac_secs() << std::endl;

	if (num_acc_samps != num_requested_samps){
        *return_status=-1;
		uhd::time_spec_t rx_error_time = usrp->get_time_now();
		std::cerr << "Error in receiving samples..(" << rx_error_time.get_real_secs() << ")\t";

		std::cerr << "Error code: " << md.error_code << "\t";
		std::cerr << "Samples rx'ed: " << num_acc_samps << 
			" (expected " << num_requested_samps << ")" << std::endl;
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
        std::cerr << "Unexpected error code " << md.error_code << " encountered at " << rx_error_time.get_real_secs() << std::endl;
        *return_status=-1;
    }
    if (md.out_of_sequence){
        uhd::time_spec_t rx_error_time = usrp->get_time_now();
        std::cerr << "start time: " << start_time.get_real_secs() << std::endl;
        std::cerr << "Packets out of order " << " encountered at " << rx_error_time.get_real_secs() << std::endl;
        *return_status=-1;
    }
    DEBUG_PRINT("RECV_AND_HOLD finished\n");
    return;
}

