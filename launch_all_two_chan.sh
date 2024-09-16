#!/bin/bash

#sudo ./init_network.sh &

errlog -name ksr.a -lp 41000 &
errlog -name ksr.b -lp 43000 &

rawacfwrite -r ksr.a -lp 41102 -ep 41000 &
rawacfwrite -r ksr.b -lp 43102 -ep 43000 &

fitacfwrite -r ksr.a -lp 41103 -ep 41000 &
fitacfwrite -r ksr.b -lp 43103 -ep 43000 &


rtserver -rp 41104 -ep 41000 -tp 1024 & # ch 1
rtserver -rp 43104 -ep 43000 -tp 1402 & # ch 2


python3 /home/radar/repos/SuperDARN_UHD_Server/tools/srr_watchdog.py server &

sleep 15
schedule -l /data/ros/scdlog/ksr.b.scdlog -f /data/ros/scd/ksr.b.scd & 
schedule -l /data/ros/scdlog/ksr.a.scdlog -f /data/ros/scd/ksr.a.scd & 


