void kodiak_set_rxfe(uhd::usrp::multi_usrp::sptr usrp, struct RXFESettings rf_settings);
void kodiak_init_rxfe(uhd::usrp::multi_usrp::sptr usrp);
void send_timing_for_sequence(uhd::usrp::multi_usrp::sptr usrp, uhd::time_spec_t start_time, std::vector<uhd::time_spec_t> pulse_times);
void init_timing_signals( uhd::usrp::multi_usrp::sptr usrp);


struct RXFESettings {
     uint32_t ifmode;  /* IF Enabled */
     uint32_t amp1;    /* Stage 1 Amp 20 db before IF mixer */
     uint32_t amp2;    /* Stage 2 Amp 10 db after IF mixer */
     uint32_t amp3;    /* Stage 3 Amp 10 db after IF mixer */
     uint32_t att1;    /* 1/2 db Attenuator */
     uint32_t att2;    /*  1  db Attenuator */
     uint32_t att3;    /*  2  db Attenuator */
     uint32_t att4;    /*  4  db Attenuator */
};

