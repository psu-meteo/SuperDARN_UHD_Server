// Function uses GPIO of USRP to generate timing signals and control RXFE
//  Outputs are:
//     
//  To control board
//     io_tx[00] 
//     io_tx[01] NOT_FAULT
//     io_tx[02]
//     io_tx[03]   TR_TX
//     io_tx[04]   TR_RX
//     io_tx[05]
//     io_tx[06]
//     Io_tx[07]
//      
//  Not connected
//     io_tx[08]   SYNC
//     io_tx[09]
//     io_tx[10]
//     io_tx[11]  MIMIC_TX
//     io_tx[12]  MIMIC_RX
//     io_tx[13]
//     io_tx[14]
//     io_tx[15]
//      
//  To RXFE
//     io_rx[00] ATT_D5    // -16  dB
//     io_rx[01] ATT_D4    // - 8  dB
//     io_rx[02] ATT_D3    // - 4  dB
//     io_rx[03] ATT_D2    // - 2  dB
//     io_rx[04] ATT_D1    // - 1  dB
//     io_rx[05] ATT_D0    // -0.5 dB
//     io_rx[06] AMP_2     // + 15 dB
//     io_rx[07] AMP_1     // + 15 dB
//     
//     
//     
// Functions to handle timing card/dio card on USRP
// broken out and expanded from Alex's recv_and_hold.cpp timing card code
// added rxfe dio and command issue time sorting
// JTK

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
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <thread>
#include <math.h>

#include "dio.h"
#include "usrp_utils.h"
 
// dio pin names to binary
#define IO_PIN_00 0x0001
#define IO_PIN_01 0x0002
#define IO_PIN_02 0x0004
#define IO_PIN_03 0x0008
#define IO_PIN_04 0x0010
#define IO_PIN_05 0x0020 
#define IO_PIN_06 0x0040
#define IO_PIN_07 0x0080

#define IO_PIN_08 0x0100
#define IO_PIN_09 0x0200
#define IO_PIN_10 0x0400
#define IO_PIN_11 0x0800
#define IO_PIN_12 0x1000
#define IO_PIN_13 0x2000
#define IO_PIN_14 0x4000
#define IO_PIN_15 0x8000 



#define MANUAL_CONTROL 0x00


// TIMING: LFTX to control board (agc) 
// mapping timing TX RX signals pins
#define TR_TX     IO_PIN_03
#define TR_RX     IO_PIN_04
#define TR_PINS   (TR_TX +TR_RX) 

#define FAULT_PIN  IO_PIN_01

// SYNC
#define SYNC_PINS IO_PIN_08 
#define SYNC_OFFSET_START (-500e-6) // start of sync pulse
#define SYNC_OFFSET_END   (-400e-6) // start of sync pulse
#define SYNC_FOR_EACH_SEQUENCE 1

//  MIMIC (LFTX)
#define MIMIC_TX    IO_PIN_11
#define MIMIC_RX    IO_PIN_12
#define MIMIC_PINS  (MIMIC_TX + MIMIC_RX)


// RXFE CONTROL: LFRX to rxfe board
// map rxfe pins to USRP dio pins
#define AMP_1  IO_PIN_07   // + 15 dB
#define AMP_2  IO_PIN_06   // + 15 dB
#define ATT_D0 IO_PIN_05   // -0.5 dB
#define ATT_D1 IO_PIN_04   // - 1  dB
#define ATT_D2 IO_PIN_03   // - 2  dB
#define ATT_D3 IO_PIN_02   // - 4  dB
#define ATT_D4 IO_PIN_01   // - 8  dB
#define ATT_D5 IO_PIN_00   // -16  dB

// define rxfe masks
#define RXFE_AMP_MASK  (AMP_1 + AMP_2)
#define RXFE_ATT_MASK  (ATT_D0 + ATT_D1 + ATT_D2 + ATT_D3 + ATT_D4 + ATT_D5)
#define RXFE_MASK      (RXFE_ATT_MASK + RXFE_AMP_MASK)


//#define DEBUG 1
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
//#include <complex>

// #define RX_OFFSET          0 //  290e-6  // microseconds, alex had 450-e6 set here


void init_timing_signals(
    uhd::usrp::multi_usrp::sptr usrp,
    bool mimic_active,
    int nSides
) {
    char bank_name[4];
    int iSide;

    double debugt = usrp->get_time_now().get_real_secs();
    DEBUG_PRINT("DIO queing GPIO commands at usrp_time %2.4f\n", debugt);

    // setup gpio to manual control and direction (output)
    for (iSide=0; iSide<nSides; iSide++) {
       sprintf(bank_name, "TX%c",65 + iSide);
       fprintf( stderr, "Bank name: %s\n",bank_name ); 
       
       usrp->set_gpio_attr(bank_name,"CTRL",MANUAL_CONTROL, SYNC_PINS);
       usrp->set_gpio_attr(bank_name,"CTRL",MANUAL_CONTROL, TR_PINS);
       usrp->set_gpio_attr(bank_name,"CTRL",MANUAL_CONTROL, FAULT_PIN);
       // TODO: set in one step
       //  usrp->set_gpio_attr(side_name_list[iSide],"CTRL",MANUAL_CONTROL, (SYNC_PINS + TR_PINS + FAULT_PIN)); // + or | should be the same here

       usrp->set_gpio_attr(bank_name,"DDR" ,SYNC_PINS, SYNC_PINS);
       usrp->set_gpio_attr(bank_name,"DDR" ,TR_PINS,   TR_PINS);
       usrp->set_gpio_attr(bank_name,"DDR" ,      0,   FAULT_PIN); // FAULT as input
    }

   if (mimic_active) {
       usrp->set_gpio_attr("TXA","CTRL",MANUAL_CONTROL,MIMIC_PINS);
       usrp->set_gpio_attr("TXA","DDR" ,MIMIC_PINS, MIMIC_PINS);
       DEBUG_PRINT("DIO.cpp: init MIMIC target\n");
   }


    debugt = usrp->get_time_now().get_real_secs();
    DEBUG_PRINT("DIO: set gpio attrs at usrp_time %2.4f\n", debugt);

}

bool read_FAULT_status_from_control_board(
    uhd::usrp::multi_usrp::sptr usrp,
    int iSide
) {
    char bank_name[4];
    sprintf(bank_name, "TX%c",65 + iSide);
    uint32_t input = usrp->get_gpio_attr(bank_name, "READBACK");
    DEBUG_PRINT("Readback from control board: %d\n", input);
    bool fault = (input & FAULT_PIN) != 0;
    DEBUG_PRINT("Returning FAULT = %d\n", fault);
    return fault;
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



// CREATE QUEUE WITH ALL SIGNAL CHANGES
void send_timing_for_sequence(
    uhd::usrp::multi_usrp::sptr usrp,
    uhd::time_spec_t start_time,
    std::vector<uhd::time_spec_t> pulse_times,
    double pulseLength,
    bool mimic_active,
    float mimic_delay,
    int nSides
) {

    char bank_name[4];
    int iSide;


    GPIOCommand c; // struct to hold command information so gpio commands can be created out of temporal order, sorted, and issued in order
    std::priority_queue<GPIOCommand, std::vector<GPIOCommand>, CompareTime> cmdq;
    
    // set SYNC to high
    c.gpiocmd  = "OUT";
    c.mask     = SYNC_PINS;
    c.value    = SYNC_PINS;
    c.cmd_time = offset_time_spec(start_time, SYNC_OFFSET_START);
    for (iSide=0; iSide<nSides; iSide++) {
       sprintf(bank_name, "TX%c",65 + iSide);
       c.port     = bank_name;
       cmdq.push(c);
    }

    // lower SYNC pin
    c.value = 0;
    c.cmd_time = offset_time_spec(start_time, SYNC_OFFSET_END);
    for (iSide=0; iSide<nSides; iSide++) {
       sprintf(bank_name, "TX%c",65 + iSide);
       c.port     = bank_name;
       cmdq.push(c);
    }
    
    //debug
    fprintf( stderr, "DIO setting start of SYNC to %2.11f\n", offset_time_spec(start_time, SYNC_OFFSET_START).get_real_secs() ); 
    fprintf( stderr, "DIO setting end   of SYNC to %2.11f\n", offset_time_spec(start_time, SYNC_OFFSET_END).get_real_secs() ); 


    // set TX and RX line for each pulse
    for(size_t iPulse = 0; iPulse < pulse_times.size() ; iPulse++) {

       /*   
       // add sync pulse at start of each sequence
       size_t nPulsesPerSequence = 8; // this has to be known from usrp_driver
       if (iPulse % nPulsesPerSequence == 0 && SYNC_FOR_EACH_SEQUENCE ) { // TODO: exclude iPulse == 0
           fprintf( stderr, "DIO SYNC at pulse  %i\n", iPulse ); 
           c.mask     = SYNC_PINS;
           c.value    = SYNC_PINS;
           c.cmd_time = offset_time_spec(pulse_times[iPulse], SYNC_OFFSET_START);
           cmdq.push(c);
           // lower SYNC pin
           c.value = 0;
           c.cmd_time = offset_time_spec(pulse_times[iPulse], SYNC_OFFSET_END);
           cmdq.push(c);
        }
      */


        // set TX high, RX low    
        c.mask     = TR_PINS;
        c.value    = TR_TX;   
        c.cmd_time = pulse_times[iPulse];
        for (iSide=0; iSide<nSides; iSide++) {
           sprintf(bank_name, "TX%c",65 + iSide);
           c.port     = bank_name;
           cmdq.push(c);
        }
    

        // set RX high and TX low
        c.value    = TR_RX;
        c.cmd_time = offset_time_spec(pulse_times[iPulse], pulseLength);
        for (iSide=0; iSide<nSides; iSide++) {
           sprintf(bank_name, "TX%c",65 + iSide);
           c.port     = bank_name;
           cmdq.push(c);
        }

        if (mimic_active) {
           // DEBUG_PRINT("DIO.cpp: using mimic target with %2.4f ms delay\n", mimic_delay*1000);
            // set mimic TX high, mimic RX low   
            c.port     = "TXA";
            c.mask     = MIMIC_PINS;
            c.value    = MIMIC_TX;   
            c.cmd_time = offset_time_spec(pulse_times[iPulse], mimic_delay);
            cmdq.push(c);

            // set mimic RX high and mimic TX low
            c.value    = MIMIC_RX;
            c.cmd_time = offset_time_spec(pulse_times[iPulse], mimic_delay + pulseLength);
            cmdq.push(c);

        }

    }


    float debugt = usrp->get_time_now().get_real_secs();


    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
    std::cout << "GPIO command issue start time is: "<< to_iso_extended_string(now)<< std::endl;



    // DEBUG to check timing
    double time_to_start;
    uhd::time_spec_t rx_usrp_pre_stream_time = usrp->get_time_now();
    time_to_start = start_time.get_real_secs() - rx_usrp_pre_stream_time.get_real_secs() - SYNC_OFFSET_START;
    fprintf(stderr, "#timing: time left for dio_worker %f ms\n", time_to_start*1000);




    DEBUG_PRINT("DIO: pushed gpio commands at usrp_time %2.4f\n", debugt);
    // issue gpio commands in time sorted order 
    // set_command_time must be sent in temporal order, they are sent into a fifo queue on the usrp and will block until executed
    // clear_command_time does not clear or bypass the buffer as of 10/2014..
    while (!cmdq.empty()) {
        c = cmdq.top();
        usrp->set_command_time(c.cmd_time);
        usrp->set_gpio_attr(c.port,c.gpiocmd,c.value,c.mask);

        //debugt = usrp->get_time_now().get_real_secs();
        //DEBUG_PRINT("DIO: sending queue: val:%2.6u mask: %2.6u at usrp_time %2.6f (%2.4f) \n", c.value, c.mask,  c.cmd_time.get_real_secs(), debugt);

        cmdq.pop();
    }


    now = boost::posix_time::microsec_clock::universal_time();
    std::cout << "GPIO command issue end time is: "<< to_iso_extended_string(now)<< std::endl;

    DEBUG_PRINT("All DIO commands sent! clock time\n");

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
    struct RXFESettings rf_settings,
    int nSides
) {
    // load up rxfe_dio with rf_setting struct
    uint16_t rxfe_dio = 0; 
    rxfe_dio |= AMP_1  * (rf_settings.amp1      != 0);
    rxfe_dio |= AMP_2  * (rf_settings.amp2      != 0); 
    rxfe_dio |= ATT_D0 * (rf_settings.att_05_dB != 0);
    rxfe_dio |= ATT_D1 * (rf_settings.att_1_dB  != 0);
    rxfe_dio |= ATT_D2 * (rf_settings.att_2_dB  != 0);
    rxfe_dio |= ATT_D3 * (rf_settings.att_4_dB  != 0);
    rxfe_dio |= ATT_D4 * (rf_settings.att_8_dB  != 0);
    rxfe_dio |= ATT_D5 * (rf_settings.att_16_dB != 0);

    rxfe_dio = (~rxfe_dio) & 0xFF; // invert all, since high means amp  and att off


    DEBUG_PRINT("DIO.cpp: received rf_settings: \n   amp1: %d\n   amp2: %d\n   att 0.5 dB: %d\n   att 1 dB  : %d\n   att 2 dB  : %d\n   att 4 dB  : %d\n   att 8 dB  : %d\n   att 16 dB : %d\n", rf_settings.amp1, rf_settings.amp2, rf_settings.att_05_dB, rf_settings.att_1_dB, rf_settings.att_2_dB, rf_settings.att_4_dB, rf_settings.att_8_dB, rf_settings.att_16_dB );
    DEBUG_PRINT("DIO.cpp: sending rxfe_dio:  %d\n", rxfe_dio);

    usrp->set_gpio_attr("RXA", "OUT", rxfe_dio, RXFE_MASK);
    if (nSides ==2) {
        usrp->set_gpio_attr("RXB", "OUT", rxfe_dio, RXFE_MASK);
    }

}


void kodiak_init_rxfe(uhd::usrp::multi_usrp::sptr usrp, int nSides)
{
    char bank_name[4];

    for (int iSide=0; iSide<nSides; iSide++) {

         sprintf(bank_name, "RX%c",65 + iSide);
         // hardcode RXFE settings...
         usrp->set_gpio_attr(bank_name, "CTRL", MANUAL_CONTROL, RXFE_MASK); // set GPIO to manual control
         usrp->set_gpio_attr(bank_name, "DDR", RXFE_MASK, RXFE_MASK); // set everybody as outputs

         // start up with rxfe disabled, rx mode
         usrp->set_gpio_attr(bank_name, "OUT", 0xFF, RXFE_AMP_MASK);
         usrp->set_gpio_attr(bank_name, "OUT", 0xFF, RXFE_ATT_MASK);

         // set to 1/2 full attenuation, both amps online 
         usrp->set_gpio_attr(bank_name, "OUT", (255 - (AMP_1 + AMP_2 + ATT_D5)), RXFE_MASK);
    }
}

