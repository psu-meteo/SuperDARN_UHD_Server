void recv_and_hold(
    struct TRTimes* trtimes,
    uhd::usrp::multi_usrp::sptr usrp,
    uhd::rx_streamer::sptr rx_stream,
    std::vector<std::complex<int16_t> *> client_buff_ptrs,
    size_t num_requested_samples,
    uhd::time_spec_t start_time,
    int *return_status
);

