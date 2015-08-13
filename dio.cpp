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
#include <control_program.h>

#include "rtypes.h"
#include "dio.h"
#include "rosmsg.h"

#define SYNC_PINS 0x02
#define TR_PINS 0x18
#define MIMIC_PINS 0x1800 
#define USEMIMIC 1
#define RXDIO 1
#define TR_TX 0x08
#define TR_RX 0x10

#define MIMIC_TX 0x0800
#define MIMIC_RX 0x1000
#define MIMIC_RANGE 6600 // microseconds
#define FAULT 0x0001

#define MANUAL_CONTROL 0x0

// RXFE pins
#define RXFE_PINS 0xFF 

#define RXFE_AMP_SHIFT 0
#define RXFE_ATT_SHIFT 2

#define RXFE_MASK 0xFF
#define RXFE_CONTROL 0x00 // manual control
#define RXFE_AMP_MASK 0xD0
#define RXFE_ATT_MASK 0x3F
#define ATT_D1 4
#define ATT_D2 3
#define ATT_D3 2
#define ATT_D4 1
#define ATT_D5 0
#define AMP_1 7
#define AMP_2 6

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
    usrp->set_gpio_attr("RXA", "CTRL", RXFE_CONTROL, RXFE_MASK); // set GPIO to manual control
    usrp->set_gpio_attr("RXA", "DDR", RXFE_MASK, RXFE_MASK); // set everybody as outputs

    // start up with rxfe disabled, rx mode
    usrp->set_gpio_attr("RXA", "OUT", 0x00, RXFE_AMP_MASK);
    usrp->set_gpio_attr("RXA", "OUT", 0xFF, RXFE_ATT_MASK);

    // set to 1/2 full attenuation, both amps online 
    usrp->set_gpio_attr("RXA", "OUT", 63, RXFE_ATT_MASK);
    usrp->set_gpio_attr("RXA", "OUT", 3 << 6, RXFE_AMP_MASK);


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
 *
 */
