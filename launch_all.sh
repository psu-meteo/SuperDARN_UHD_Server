#!/bin/bash

source /home/radar/.profile
source /home/radar/.bashrc

#sudo ./init_network.sh &

errlog -name sps.a -lp 41000 &
#errlog -name mcm.b -lp 43000 &

rawacfwrite -r sps.a -lp 41102 -ep 41000 &
#rawacfwrite -r mcm.b -lp 43102 -ep 43000 &

fitacfwrite -r sps.a -lp 41103 -ep 41000 &
#fitacfwrite -r mcm.b -lp 43103 -ep 43000 &


rtserver -rp 41104 -ep 41000 -tp 10011 & # ch 4
rtserver -rp 43104 -ep 43000 -tp 10012 & # ch 3


python3 /home/radar/SuperDARN_UHD_Server/tools/srr_watchdog.py server &

sleep 15
schedule -l /data/ros/scdlog/sps.a.scdlog -f /data/ros/scd/sps.a.scd & 
#schedule -l /data/ros/scdlog/mcm.b.scdlog -f /data/ros/scd/mcm.b.scd & 

ps -C cuda_driver.py -o pid= | xargs sudo renice -20
ps -C usrp_driver -o pid= | xargs sudo renice -15
ps -C usrp_server.py -o pid= | xargs sudo renice -15

