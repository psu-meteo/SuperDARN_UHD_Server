# TODO: all of these should be moved to a config file at some point..

USRP_MASTER_CLOCK_FREQ = 200e6 # not all values are supported by the ettus UHD driver, changing this will require changing downconversion ratios

# seconds of delay in usrp clock time at the start of an integration period before the first pulse
# to allow for the usrp_drivers receive a command..
INTEGRATION_PERIOD_SYNC_TIME = .2 

# padding in seconds between pulse sequences
# starting from the last transmit pulse within a sequence
# so, this needs to be long enough to avoid any ambiguity between pulse sequences
# for assuming a maximum range of 5000 kilometers, this could be about .033 seconds
# if this is not an integer multiple of the baseband sampling period, the values will be round down
# the rf baseband sampling period is probably 3.333333... kHz
PULSE_SEQUENCE_PADDING_TIME = .033

