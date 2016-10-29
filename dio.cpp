// Functions to handle timing card/dio card on USRP
// broken out and expanded from Alex's recv_and_hold.cpp timing card code
// added rxfe dio and command issue time sorting
// JTK
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

#include "dio.h"
#include "usrp_utils.h"

#define MANUAL_CONTROL 0x00

// timing signals
#define SYNC_PINS 0x02
#define TR_PINS 0x18
#define TR_TX 0x08
#define TR_RX 0x10

#define FAULT 0x0001

#define RX_OFFSET 290e-6 // microseconds, alex had 450-e6 set here
#define SYNC_OFFSET_START (-500e-6) // start of sync pulse
#define SYNC_OFFSET_END (-400e-6) // start of sync pulse



// MIMIC
#define USEMIMIC 1
#define MIMIC_PINS 0x1800 
#define MIMIC_TX 0x0800
#define MIMIC_RX 0x1000
#define MIMIC_RANGE 6600 // microseconds


// RXFE pins
#define RXFE_PINS 0xFF 

#define RXFE_AMP_SHIFT 0
#define RXFE_ATT_SHIFT 2

#define RXFE_MASK 0xFF
#define RXFE_AMP_MASK 0xD0
#define RXFE_ATT_MASK 0x3F
#define ATT_D1 4
#define ATT_D2 3
#define ATT_D3 2
#define ATT_D4 1
#define ATT_D5 0
#define AMP_1 7
#define AMP_2 6


#define DEBUG 1
#ifdef DEBUG
#define DEBUG_PRINT(...) do{ fprintf( stderr, __VA_ARGS__ ); } while( false )
#else
#define DEBUG_PRINT(...) do{ } while ( false )
#endif

// delete? (mgu)
#define RXDIO 1
//#include "rtypes.h"
//#include "rosmsg.h"
//#define RXFE_CONTROL 0x00 // manual control

#define PULSE_LENGTH 420e-6 // TODO: get from usrp_driver


void init_timing_signals(
    uhd::usrp::multi_usrp::sptr usrp
) {

    double debugt = usrp->get_time_now().get_real_secs();
    DEBUG_PRINT("DIO queing GPIO commands at usrp_time %2.4f\n", debugt);

    // setup gpio direction and control
    usrp->set_gpio_attr("TXA","CTRL",MANUAL_CONTROL,SYNC_PINS);
    usrp->set_gpio_attr("TXA","CTRL",MANUAL_CONTROL,TR_PINS);
    usrp->set_gpio_attr("TXA","CTRL",MANUAL_CONTROL,MIMIC_PINS);

    usrp->set_gpio_attr("TXA","DDR",SYNC_PINS,SYNC_PINS);
    usrp->set_gpio_attr("TXA","DDR",TR_PINS, TR_PINS);
    usrp->set_gpio_attr("TXA","DDR",MIMIC_PINS, MIMIC_PINS);

    debugt = usrp->get_time_now().get_real_secs();
    DEBUG_PRINT("DIO: set gpio attrs at usrp_time %2.4f\n", debugt);

}


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




void send_timing_for_sequence(
    uhd::usrp::multi_usrp::sptr usrp,
    uhd::time_spec_t start_time,
    std::vector<uhd::time_spec_t> pulse_times
) {

    GPIOCommand c; // struct to hold command information so gpio commands can be created out of temporal order, sorted, and issued in order

    std::priority_queue<GPIOCommand, std::vector<GPIOCommand>, CompareTime> cmdq;


    // TODO: fix sync?
    //set sync pin
    usrp->set_command_time(offset_time_spec(start_time, SYNC_OFFSET_START));
    usrp->set_gpio_attr("TXA","OUT",SYNC_PINS, SYNC_PINS);

    // lower sync pin when rx streaming starts
    c.port = "TXA";
    c.gpiocmd = "OUT";
    c.mask = SYNC_PINS;
    c.value = 0;
  //  c.cmd_time = offset_time_spec(start_time, SYNC_OFFSET_END).get_real_secs();
    c.cmd_time = offset_time_spec(start_time, SYNC_OFFSET_END);
    cmdq.push(c);
    
    fprintf( stderr, "DIO setting start of SYNC to %2.11f\n", offset_time_spec(start_time, SYNC_OFFSET_START).get_real_secs() ); 
    fprintf( stderr, "DIO setting end   of SYNC to %2.11f\n", offset_time_spec(start_time, SYNC_OFFSET_END).get_real_secs() ); 


    // set TX line

    for(size_t iPulse = 0; iPulse < pulse_times.size() ; iPulse++) { //pulse_times.size()
        // bock here to tets if this call by reference error
        GPIOCommand c;
        c.port = "TXA";
        c.gpiocmd = "OUT";
        c.mask = TR_PINS;
    
        c.value = TR_TX; 
      //  c.cmd_time = offset_time_spec(start_time, SYNC_OFFSET_END).get_real_secs();
        c.cmd_time = pulse_times[iPulse];
        cmdq.push(c);
        c.value = TR_RX;
        c.cmd_time = offset_time_spec(pulse_times[iPulse], PULSE_LENGTH);
        cmdq.push(c);

    }




    float debugt = usrp->get_time_now().get_real_secs();
    DEBUG_PRINT("DIO: pushed gpio commands at usrp_time %2.4f\n", debugt);
    // issue gpio commands in time sorted order 
    // set_command_time must be sent in temporal order, they are sent into a fifo queue on the usrp and will block until executed
    // clear_command_time does not clear or bypass the buffer as of 10/2014..
    while (!cmdq.empty()) {
        c = cmdq.top();
     //   usrp->set_command_time(uhd::time_spec_t(c.cmd_time));
        usrp->set_command_time(c.cmd_time);
        usrp->set_gpio_attr(c.port,c.gpiocmd,c.value,c.mask);
        DEBUG_PRINT("DIO: sending queue: val:%2.6u mask: %2.0u at usrp_time %2.6f\n", c.value, c.mask,  c.cmd_time.get_real_secs());


        cmdq.pop();
    }

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
 

}


// TODO: add function for settings beams
// TODO: add function for reading transmitter status
// GPIO allocation..
//
// TXA - 
// TXB - 
// RXA - 
// RXB - 
//
// so, 3 SCSI cables..
// each one is 25ish pins
/*
TR - 2 pins
SYNC - 2 pins
RXFE - 16 pins
MINIC - 2 pins
total of 80 GPIO pins
 */


void kodiak_set_rxfe(
    uhd::usrp::multi_usrp::sptr usrp,
    struct RXFESettings rf_settings
) {
    // load up rxfe_dio with rf_setting struct
    uint16_t rxfe_dio = 0;
    rxfe_dio |= AMP_1 * (rf_settings.amp1 != 0);
    rxfe_dio |= AMP_2 * (rf_settings.amp2 != 0); 
    rxfe_dio |= (rf_settings.att1 != 0) * ATT_D1;
    rxfe_dio |= (rf_settings.att2 != 0) * ATT_D2;
    rxfe_dio |= (rf_settings.att3 != 0) * ATT_D3;
    rxfe_dio |= (rf_settings.att4 != 0) * ATT_D4;
    //rxfe_dio |= (rf_settings.att5 != 0) * ATT_5;

    usrp->set_gpio_attr("RXA", "OUT", rxfe_dio, RXFE_MASK);
}

void kodiak_init_rxfe(uhd::usrp::multi_usrp::sptr usrp)
{
    // hardcode RXFE settings...
    usrp->set_gpio_attr("RXA", "CTRL", MANUAL_CONTROL, RXFE_MASK); // set GPIO to manual control
    usrp->set_gpio_attr("RXA", "DDR", RXFE_MASK, RXFE_MASK); // set everybody as outputs

    // start up with rxfe disabled, rx mode
    usrp->set_gpio_attr("RXA", "OUT", 0x00, RXFE_AMP_MASK);
    usrp->set_gpio_attr("RXA", "OUT", 0xFF, RXFE_ATT_MASK);

    // set to 1/2 full attenuation, both amps online 
    usrp->set_gpio_attr("RXA", "OUT", 63, RXFE_ATT_MASK);
    usrp->set_gpio_attr("RXA", "OUT", 3 << 6, RXFE_AMP_MASK);


}

