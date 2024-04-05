#!/bin/bash

#ssh radar@kodiak-aux '/home/radar/SuperDARN_UHD_Server/srr.py stop &' &
ssh radar@kodiak-aux 'pkill --signal 9 -f srr_watchdog.py &' &
ssh radar@kodiak-aux '/home/radar/repos/SuperDARN_UHD_Server/kill_watchdog' &
ssh radar@kodiak-aux '/home/radar/repos/SuperDARN_UHD_Server/srr.py stop &' &
pkill -KILL -f srr_watchdog.py
/home/radar/repos/usrp/srr.py stop

# from old stop.radar
/usr/bin/killall -s 2 schedule

/usr/bin/killall schedule
/usr/bin/killall rtserver
/usr/bin/killall fitacfwrite
/usr/bin/killall rawacfwrite
/usr/bin/killall iqwrite

/usr/bin/killall shellserver
/usr/bin/killall errlog



# pkill -f usrp_driver
# pkill -f cuda_driver.py
# pkill -f run_cuda_driver
# pkill -f usrp_server
# pkill -f uafscan
