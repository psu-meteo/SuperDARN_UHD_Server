#!/bin/bash

source /home/radar/.profile
source /home/radar/.bashrc

env > /home/radar/hold_env

#sudo ./init_network.sh &

#systemctl --user import-environment

/home/radar/rst/bin/errlog -name ksr.a -lp 41000 &
#/home/radar/rst/bin/errlog -name ksr.b -lp 42000 &

/home/radar/rst/bin/rawacfwrite -r ksr.a -lp 41102 -ep 41000 &
#/home/radar/rst/bin/rawacfwrite -r ksr.b -lp 42102 -ep 42000 &

/home/radar/rst/bin/fitacfwrite -r ksr.a -lp 41103 -ep 41000 &
#/home/radar/rst/bin/fitacfwrite -r ksr.b -lp 42103 -ep 42000 &


/home/radar/rst/bin/rtserver -rp 41104 -ep 41000 -tp 1401 & # ch 4
#/home/radar/rst/bin/rtserver -rp 42104 -ep 42000 -tp 1402 & # ch 3


python3 /home/radar/repos/SuperDARN_UHD_Server/tools/srr_watchdog.py server &

sleep 5
/home/radar/rst/bin/schedule -l /data/ros/scdlog/ksr.a.scdlog -f /data/ros/scd/ksr.a.scd & 
#/home/radar/rst/bin/schedule -l /data/ros/scdlog/ksr.b.scdlog -f /data/ros/scd/ksr.b.scd & 

