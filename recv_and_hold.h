void recv_and_hold(
    uhd::usrp::multi_usrp::sptr usrp,
    uhd::rx_streamer::sptr rx_stream,
    std::vector<void *> client_buff_ptrs,
    size_t num_requested_samples,
    uhd::time_spec_t start_time,
    int32_t *return_status
);

