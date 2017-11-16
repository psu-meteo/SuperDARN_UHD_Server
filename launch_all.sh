#!/bin/bash

#sudo ./init_network.sh &

errlog -name mcm.a -lp 41000 &
#errlog -name mcm.b -lp 43000 &

rawacfwrite -r mcm.a -lp 41102 -ep 41000 &
#rawacfwrite -r mcm.b -lp 43102 -ep 43000 &

fitacfwrite -r mcm.a -lp 41103 -ep 41000 &
#fitacfwrite -r mcm.b -lp 43103 -ep 43000 &


rtserver -rp 41104 -ep 41000 -tp 1401 & # ch 4
#rtserver -rp 43104 -ep 43000 -tp 1402 & # ch 3


python3 /home/radar/repos/SuperDARN_UHD_Server/tools/srr_watchdog.py server &

sleep 15
#schedule -l /data/ros/scdlog/mcm.a.scdlog -f /data/ros/scd/mcm.a.scd & 
#schedule -l /data/ros/scdlog/mcm.b.scdlog -f /data/ros/scd/mcm.b.scd & 

