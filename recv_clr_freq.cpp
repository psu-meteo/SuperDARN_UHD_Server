#include <complex>
#include <vector>

#include <fftw3.h>
//Added by Alex for usrp
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/exception.hpp>

#include <boost/thread.hpp>

void spectrum_worker(
    fftw_plan* plan,
    size_t ant,
    size_t nsamps,
    size_t fftsize,
    float time_delay,
    float center_freq,
    double* window,
    std::complex<int16_t>* samples,
    double* output);

/***********************************************************************
 * recv_clr_freq function
 * A function to be used to receive samples from the USRP and
 * calculate a periodogram
 **********************************************************************/
int recv_clr_freq(
    uhd::usrp::multi_usrp::sptr usrp,
    uhd::rx_streamer::sptr rx_stream,
    size_t center,
    size_t span_khz,
    size_t bandwidth_khz,
    size_t nave,
    double time_delay,
    double *return_pwr
){
    int debug = 0;

    size_t start_inx;
    size_t num_acc_samps = 0;
    size_t nsamps = usrp->get_rx_rate() / (1e3*bandwidth_khz);
    size_t fftsize = usrp->get_rx_rate() / 1e3;
    size_t num_usrp_samples = nave * nsamps;

    std::vector<std::vector<std::complex<int16_t> > > rx_short_vecs;
    std::vector<std::complex<int16_t> *> rx_vec_ptrs;
    fftw_complex *in=NULL, *out=NULL;
    fftw_plan plan;
    std::vector<double> tmp_pwr;
    std::vector<double> pwr;
    std::vector<double> hann_window(nsamps);

    struct timespec start, start_proc, stop;
    float elapsed, process_t;

    for(size_t i=0; i<nsamps; i++){
	    hann_window[i] = 0.5*(1-cos(6.28*i/(nsamps-1)));
    }

    //if (tmp_pwr!=NULL) free(tmp_pwr);
    //tmp_pwr = (double*) calloc(bandwidth,sizeof(double));
    tmp_pwr.resize(fftsize, 0);
    pwr.resize(fftsize, 0);
    for (size_t i=0; i<fftsize; i++){
        tmp_pwr[i] = 0;
        pwr[i] = 0;
    }
    for (size_t i=0; i<span_khz; i++){
        return_pwr[i] = 0;
    }

    for(size_t i=0;i<usrp->get_rx_num_channels();i++)
	rx_short_vecs.push_back(std::vector<std::complex<int16_t> >(nsamps,0));
    for(size_t i=0;i<usrp->get_rx_num_channels();i++)
	rx_vec_ptrs.push_back(&rx_short_vecs[i].front());

    uhd::rx_metadata_t md;
    float timeout = 0.1;

    clock_gettime(CLOCK_REALTIME, &start);
    //setup streaming
    usrp->set_rx_freq(1.e3*center);
    std::cout << "RX FREQ: " << usrp->get_rx_freq() << std::endl;
    //usrp->get_rx_rate(bandwidth);
    uhd::stream_cmd_t stream_cmd = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE;
    stream_cmd.num_samps = num_usrp_samples;
    stream_cmd.stream_now = false;
    uhd::time_spec_t start_time = usrp->get_time_now() + 0.02;
    stream_cmd.time_spec = start_time;
    usrp->issue_stream_cmd(stream_cmd);

    md.error_code = uhd::rx_metadata_t::ERROR_CODE_NONE;
    std::cout << "clr freq search: about to start receiving\n";
    std::cout << "nusrp samples: " << num_usrp_samples << std::endl;
    std::cout << "nsamps: " << nsamps << std::endl;
    std::cout << "start_time: " << start_time.get_real_secs() << std::endl;
    clock_gettime(CLOCK_REALTIME, &start_proc);
    boost::thread_group spect_threads;
    while(num_acc_samps < num_usrp_samples){
        uint32_t num_rx_samps = rx_stream->recv(rx_vec_ptrs, nsamps, md, timeout);

	    //Check for errors
	    if (num_rx_samps != nsamps){
	    	uhd::time_spec_t rx_error_time = usrp->get_time_now();
	    	std::cerr << "Error in receiving samples..(" << rx_error_time.get_real_secs() << ")\t";;
	    	std::cerr << "Samples rx'ed: " << num_rx_samps << 
	    		" (expected " << nsamps << ")" << std::endl;
	    	std::cerr << "Total Samples rx'ed: " << num_acc_samps << std::endl;
	    	num_rx_samps=nsamps;
	    	for (size_t i=0;i<usrp->get_rx_num_channels();i++){
	    		for (size_t j=0;j<nsamps;j++){
	    			rx_short_vecs[i][j]=std::complex<int16_t>(0,0);
	    		}
	    	}
	    }
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
	    uhd::time_spec_t rx_error_time = usrp->get_time_now();
            std::cerr << "Timeout encountered at " << rx_error_time.get_real_secs() << std::endl;
	    return -1;
        }
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW){
	    uhd::time_spec_t rx_error_time = usrp->get_time_now();
            std::cerr << "Overflow encountered at " << rx_error_time.get_real_secs() << std::endl;
	    return -1;
        }
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
	    uhd::time_spec_t rx_error_time = usrp->get_time_now();
            std::cerr << "Unexpected error code " << md.error_code <<
		" encountered at " << rx_error_time.get_real_secs() << std::endl;
	    return -1;
        }
	    //Done checking for errors
	    	
	    plan = fftw_plan_dft_1d(fftsize, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
        for (size_t iant=0; iant<usrp->get_rx_num_channels(); iant++){
            if(debug) std::cout << "Spawning thread " << iant << std::endl;
            spect_threads.create_thread(boost::bind(spectrum_worker,
                &plan,
                iant,
                nsamps,
                fftsize,
                time_delay,
                usrp->get_rx_freq(),
                &hann_window.front(),
                &rx_short_vecs[iant].front(),
                &tmp_pwr.front()));
        }
        spect_threads.join_all();
	    fftw_destroy_plan(plan);
                
        /*
	    //Execute fft for each sample buffer
	    if (in!=NULL) {free(in);in=NULL;}
	    in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex)*fftsize);
	    if (out!=NULL) {free(out);out=NULL;}
	    out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex)*fftsize);

	    plan = fftw_plan_dft_1d(fftsize, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

        for (int iant=0; iant<usrp->get_rx_num_channels(); iant++){
	        for (int i=0;i<nsamps;i++){
                	in[i][0] = (hann_window[i]*rx_short_vecs[iant][i].real());
                	in[i][1] = (hann_window[i]*rx_short_vecs[iant][i].imag());
                    //std::cout << "ini: " << in[i][0] << " inq: " << in[i][1] << std::endl;
                	//in[i][0] = rx_short_vecs[0][i].real();
                	//in[i][1] = rx_short_vecs[0][i].imag();
	        }
	        for (int i=nsamps;i<fftsize;i++){
                	in[i][0] = 0;
                	in[i][1] = 0;
            }

	        fftw_execute(plan);
	        //Done executing fft
            
            //Perform some beamforming. The relative bandwidth is assumed large, so
            //put a different phase shift on each FFT bin so we have constant
            //time delay
            double start_pshift = 2*M_PI*iant*1e-9*time_delay*(usrp->get_rx_freq() - 1.e3*span_khz/2.);
            double end_pshift = 2*M_PI*iant*1e-9*time_delay*(usrp->get_rx_freq() + 1.e3*span_khz/2.);
            double inc_pshift = (end_pshift - start_pshift) / span_khz;
            if (debug && num_acc_samps == 0) std::cout << "start phase: " << 
                iant << " " << start_pshift << " " << end_pshift << " " << 
                inc_pshift << std::endl;
            for (int i=0; i<fftsize; i++){
                out[i][0] = out[i][0] * cos(start_pshift + i*inc_pshift) -
                    out[i][1] * sin(start_pshift*i*inc_pshift);
                out[i][1] = out[i][0] * sin(start_pshift + i*inc_pshift) +
                    out[i][1] * cos(start_pshift*i*inc_pshift);
            }
            //Done beamforming
	    
	        //Add the current spectrum to the running total
	        for (int i=0;i<fftsize;i++){
                	double temp = out[i][0]*out[i][0] + out[i][1]*out[i][1];
                    temp /= double(fftsize*fftsize*nave);
                	tmp_pwr[i] += temp;
	        }
	        //Done adding spectrum to running total
        }
        */

        num_acc_samps += num_rx_samps;
	    //fftw_destroy_plan(plan);
	    //if (in!=NULL) {free(in); in=NULL;}
	    //if (out!=NULL) {free(out); out=NULL;}
    }
    
    //Center the fft (fftshift)
    for(size_t i=0;i<(fftsize/2);i++){
    	pwr[fftsize/2+i]=tmp_pwr[i];
    	pwr[i]=tmp_pwr[fftsize/2+i];
    }   
    if (debug){
        for(size_t i=0;i<(fftsize);i++){
            std::cout << "Pwr " << i+(usrp->get_rx_freq()/1.e3)-fftsize/2 << " " << pwr[i] << std::endl;
        }
     }
    //Done centering

    start_inx = (fftsize - span_khz) / 2;
    if (debug) std::cout << "start_inx: " << start_inx << std::endl;
    if (debug) std::cout << "fftsize: " << fftsize << std::endl;
    if (debug) std::cout << "span_khz: " << span_khz << std::endl;
    if (debug) std::cout << "pwr size: " << pwr.size() << std::endl;
    for (size_t i=0; i<span_khz; i++){
        return_pwr[i] = pwr[start_inx+i];
        if (debug) std::cout << 
            1e-3*usrp->get_rx_freq() + i << " " << return_pwr[i] << std::endl;
        if (debug) std::cout << 
            1e-3*usrp->get_rx_freq() + i << " " << pwr[start_inx+i] << std::endl;
    }
    clock_gettime(CLOCK_REALTIME, &stop);

    elapsed = (1e9*stop.tv_sec+stop.tv_nsec) - (1e9*start.tv_sec+start.tv_nsec);
    process_t = (1e9*stop.tv_sec+stop.tv_nsec) - (1e9*start_proc.tv_sec+start.tv_nsec);
    if (debug) std::cout << "clear freq function total time: " <<
        elapsed / 1e6 << " ms" << std::endl;
    if (debug) std::cout << "clear freq function processing time: " <<
        process_t / 1e6 << " ms" << std::endl;
    if (debug) std::cout << "clear freq function listening time: " <<
        1e3*num_usrp_samples / usrp->get_rx_rate() << " ms" << std::endl;

    //if (tmp_pwr!=NULL) free(tmp_pwr);

    return 0;
}

void spectrum_worker(
    fftw_plan* plan,
    size_t ant,
    size_t nsamps,
    size_t fftsize,
    float time_delay,
    float center_freq,
    double* window,
    std::complex<int16_t>* samples,
    double* output)
{
    int debug =0;
    if (debug) std::cout << "Entering worker function " << ant << std::endl;
    //fftw_plan plan;
    fftw_complex *in=NULL, *out=NULL;
    if (debug) std::cout << "Allocating FFTW memory " << ant << std::endl;
    if (debug) std::cout << "fftsize: " << fftsize << std::endl;
    if (in!=NULL) {free(in);in=NULL;}
    in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex)*fftsize);
    if (out!=NULL) {free(out);out=NULL;}
    out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex)*fftsize);
    
    //plan = fftw_plan_dft_1d(fftsize, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
    
    if (debug) std::cout << "Copying samples to fftw input\n";
    //for (int iant=0; iant<usrp->get_rx_num_channels(); iant++){
	for (unsigned int i=0;i<nsamps;i++){
        	in[i][0] = (window[i]*samples[i].real());
        	in[i][1] = (window[i]*samples[i].imag());
	}
	for (unsigned int i=nsamps;i<fftsize;i++){
        	in[i][0] = 0;
        	in[i][1] = 0;
    }
    if (debug) std::cout << "About to exectute plan\n";

	//fftw_execute(plan);
	fftw_execute_dft(*plan, in, out);
	//Done executing fft
    if (debug) std::cout << "Exectuted plan; about to perform beamforming\n";
    
    //Perform some beamforming. The relative bandwidth is assumed large, so
    //put a different phase shift on each FFT bin so we have constant
    //time delay
    double start_pshift = 2*M_PI*ant*1e-9*time_delay*(center_freq - 1.e3*fftsize/2.);
    double end_pshift = 2*M_PI*ant*1e-9*time_delay*(center_freq + 1.e3*fftsize/2.);
    double inc_pshift = (end_pshift - start_pshift) / fftsize;
    //if (debug && num_acc_samps == 0) std::cout << "start phase: " << 
    //    ant << " " << start_pshift << " " << end_pshift << " " << 
    //    inc_pshift << std::endl;
    for (size_t i=0; i<fftsize; i++){
        out[i][0] = out[i][0] * cos(start_pshift + i*inc_pshift) -
            out[i][1] * sin(start_pshift*i*inc_pshift);
        out[i][1] = out[i][0] * sin(start_pshift + i*inc_pshift) +
            out[i][1] * cos(start_pshift*i*inc_pshift);
    }
    //Done beamforming
    if (debug) std::cout << "Done beamforming; About to add samples to shared memory\n";
	
	//Add the current spectrum to the running total
	for (unsigned int i=0;i<fftsize;i++){
        	double temp = out[i][0]*out[i][0] + out[i][1]*out[i][1];
            temp /= double(fftsize*fftsize);
        	output[i] += temp;
	}
    if (debug) std::cout << "Freeing memory, destroying plan\n";
	//Done adding spectrum to running total
	if (in!=NULL) {free(in); in=NULL;}
	if (out!=NULL) {free(out); out=NULL;}
	//fftw_destroy_plan(plan);
}
