/***********************************************************************
 * recv_and_hold() function
 * A function to be used to receive samples from the USRP and
 * hold them in the network socket buffer. *rx_data_buffers points to
 * the memory locations of each antenna's samples.   Meant to operate in its
 * own thread context so that it does not block the execution in main()
 * Also handles the former duties of the timing card...
 **********************************************************************/
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

#include "rx_worker.h"
#include "usrp_utils.h"
#include "dio.h"

#define DEBUG 1
#ifdef DEBUG
#define DEBUG_PRINT(...) do{ fprintf( stderr, __VA_ARGS__ ); } while( false )
#else
#define DEBUG_PRINT(...) do{ } while ( false )
#endif

#define RX_OFFSET 0 // 290e-6 // microseconds, alex had 450-e6 set here

void usrp_rx_worker(
    uhd::usrp::multi_usrp::sptr usrp,
    uhd::rx_streamer::sptr rx_stream,
    std::vector<std::complex<int16_t>> *rx_data_buffer,
    size_t num_requested_samps,
    uhd::time_spec_t start_time,
    int32_t *return_status
){

    DEBUG_PRINT("entering RX_WORKER\n");

    //setup streaming
    uhd::rx_metadata_t md;
    md.error_code = uhd::rx_metadata_t::ERROR_CODE_NONE;

    double timeout = 5.0;
    
    uhd::stream_cmd_t stream_cmd = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE;
    stream_cmd.num_samps = num_requested_samps;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec = offset_time_spec(start_time, RX_OFFSET);
   
    usrp->issue_stream_cmd(stream_cmd);
    
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

    DEBUG_PRINT("RX_WORKER fetched samples!\n");
    if(DEBUG) std::cout << boost::format("RX_WORKER : %u full secs, %f frac secs") % md.time_spec.get_full_secs() % md.time_spec.get_frac_secs() << std::endl;

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
    DEBUG_PRINT("RX_WORKER finished\n");
    return;
}

