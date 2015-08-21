#include <complex>
#include <vector>

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/exception.hpp>
#include <boost/thread.hpp>
#include <thread>
#include <math.h>
#include <time.h>

#include "burst_worker.h"

#define TEST_TXWORKER 0

extern int verbose;
int debug=0;

/***********************************************************************
 * tx_worker function
 **********************************************************************/
void tx_worker(
    uhd::tx_streamer::sptr tx_stream,
    std::vector<std::complex<int16_t> *>& pulse_seq_ptrs,
    uint32_t pulse_length,
    double tx_rate,
    std::vector<uhd::time_spec_t> pulse_times 
){

    struct timeval t0,t1;
    gettimeofday(&t0,NULL);

    for(size_t i = 0; i < pulse_times.size(); i++) {
        //setup the metadata flags to send the first buffer's worth of data
        uhd::tx_metadata_t md;
        md.start_of_burst = true;
        md.end_of_burst   = false;
        md.has_time_spec  = true;
        md.time_spec = pulse_times[i];
        
        double pulse_duration = pulse_length / tx_rate;
        double timeout = pulse_duration + pulse_times[i].get_real_secs() + .0001;

        //Initialize the temporary pointers according to the argument passed to the function
        std::vector<std::complex<int16_t> *> temp_ptrs(pulse_seq_ptrs.size());
        // pulse_seq_ptrs is length 8
        if (debug) std::cout << "num tx channels: " << tx_stream->get_num_channels() << std::endl;
        if (debug) std::cout << "sequence length : " << pulse_length << std::endl;
        if (debug) std::cout << "number of pulses : " << pulse_times.size() << std::endl;
        if (debug) std::cout << "pointer values: " << std::endl;

        for (uint32_t t=0; t<pulse_seq_ptrs.size(); t++){
            temp_ptrs[t] = pulse_seq_ptrs.at(t);
        }

        size_t nacc_samps = 0;
        size_t spb = tx_stream->get_max_num_samps();

        //Now go for it!
        while(nacc_samps < pulse_length){
            size_t samps_to_send = std::min(pulse_length - nacc_samps, spb);
            size_t ntx_samps = tx_stream->send(temp_ptrs, samps_to_send, md, timeout);

            md.start_of_burst = false;
            md.has_time_spec = false;

            if(debug) std::cout << boost::format("Sent packet: %u samples") % ntx_samps << std::endl;
            nacc_samps += ntx_samps;
        }

        md.end_of_burst = true;
        tx_stream->send(temp_ptrs, 0, md, timeout);

        if (debug) std::cout << std::endl << "Waiting for async burst ACK... " << std::flush;
        uhd::async_metadata_t async_md;
        bool got_async_burst_ack = false;

        while (not got_async_burst_ack and tx_stream->recv_async_msg(async_md, timeout)){
            got_async_burst_ack = (async_md.event_code == uhd::async_metadata_t::EVENT_CODE_BURST_ACK);
        }
        
        if (debug) std::cout << (got_async_burst_ack? "success" : "fail") << std::endl;
    }

    gettimeofday(&t1,NULL);
    double elapsed=(t1.tv_sec-t0.tv_sec)*1E6;
    elapsed+=(t1.tv_usec-t0.tv_usec);
    std::cout << "finished transmitting pulse sequence, tx worker time (us): " << elapsed << "\n";

 
}

/*
#ifdef TEST_TXWORKER
int main(void) {
    typedef std::complex<int16_t> sc16;

    std::vector<sc16 *> tx_vec_ptrs;
    std::vector<std::vector<std::complex<int16_t> > > tx_rf_vecs;
    std::vector<uhd::time_spec_t> pulse_times;
    uhd::time_spec_t start_time;
    uhd::usrp::multi_usrp::sptr usrp;
    std::string args, txsubdev, rxsubdev;
    
    struct timeval t0,t1;
    float rxrate = 10e6;
    float txrate = 20e6;
    float txfreq = 10e6;
    float rxfreq = 10e6;
    uint32_t nrf_samples = (uint32_t) txfreq * 200e-6;
    uint32_t nants;

    // create usrp
    //args = "addr0=usrp1, addr2=usrp3, addr3=usrp4, addr4=usrp5, addr6=usrp7, addr8=usrp9, addr9=usrp10, addr7=usrp11, addr5=usrp12, addr1=usrp13";//addr12=usrp15, addr13=usrp16";
    args = "addr0=usrp1";//, addr1=usrp3, addr2=usrp4, addr3=usrp5";

    txsubdev = "A:A";
    rxsubdev = "A:A";

    usrp = uhd::usrp::multi_usrp::make(args);
    
    usrp->set_time_source("external");
    usrp->set_clock_source("external");

    const uhd::time_spec_t last_pps_time = usrp->get_time_last_pps();
    while (last_pps_time == usrp->get_time_last_pps()) {
        usleep(5e4);
    }
    
    usrp->set_time_next_pps(uhd::time_spec_t(0.0));
    boost::this_thread::sleep(boost::posix_time::milliseconds(1100));
    usrp->set_rx_rate(rxrate);
    usrp->set_rx_freq(rxfreq);
    usrp->set_tx_rate(txrate);
    usrp->set_tx_freq(txfreq);
    
    nants = usrp->get_tx_num_channels();

    uhd::stream_args_t tx_stream_args("sc16", "sc16");

    for (size_t tx_chan = 0; tx_chan < nants; tx_chan++) {
        tx_stream_args.channels.push_back(tx_chan); //linear mapping
    }
    
    (&tx_vec_ptrs)->resize(nants);
    (&tx_rf_vecs)->resize(nants);

    for (size_t i=0; i<nants; i++){
        tx_rf_vecs[i].resize(nrf_samples);
        (tx_vec_ptrs)[i] = &tx_rf_vecs[i].front();
    }

    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(tx_stream_args);
    for(int i = 0; i < 10000; i++) {
        gettimeofday(&t0,NULL);
        usleep(5000);
        start_time = usrp->get_time_now() + 0.01;
        pulse_times.resize(8);
        double mpinc = 200e-6;
        pulse_times[0] = start_time;
        pulse_times[1] = start_time + 14 * mpinc;
        pulse_times[2] = start_time + 22 * mpinc;
        pulse_times[3] = start_time + 24 * mpinc;
        pulse_times[4] = start_time + 27 * mpinc;
        pulse_times[5] = start_time + 31 * mpinc;
        pulse_times[6] = start_time + 42 * mpinc;
        pulse_times[7] = start_time + 43 * mpinc;
        

        tx_worker(tx_stream, tx_vec_ptrs, nrf_samples, txrate, pulse_times);

        gettimeofday(&t1,NULL);
        double elapsed=(t1.tv_sec-t0.tv_sec)*1E6;
        elapsed+=(t1.tv_usec-t0.tv_usec);
        std::cout << "tx worker loop time (us): " << elapsed << "\n";
    } 
    return 0;
}
#endif
*/
     
