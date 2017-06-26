void usrp_tx_worker(
    uhd::tx_streamer::sptr tx_stream, 
    std::vector<std::vector<std::complex<int16_t>>> pulse_samples, 
    uhd::time_spec_t burst_start_time,
    std::vector<size_t> pulse_sample_idx_offsets);
