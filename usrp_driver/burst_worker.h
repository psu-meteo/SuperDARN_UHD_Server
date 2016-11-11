void tx_worker(uhd::tx_streamer::sptr tx_stream, std::vector<std::complex<int16_t> *> pulse_seq_ptrs, uint32_t pulse_length, double tx_rate, std::vector<uhd::time_spec_t> pulse_times);
