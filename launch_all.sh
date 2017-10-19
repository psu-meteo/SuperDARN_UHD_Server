#sudo ./init_network.sh &

errlog -name kod.d -lp 41000 &
errlog -name kod.c -lp 43000 &

rawacfwrite -r kod.d -lp 41102 -ep 41000 &
rawacfwrite -r kod.c -lp 43102 -ep 43000 &

fitacfwrite -r kod.d -lp 41103 -ep 41000 &
fitacfwrite -r kod.c -lp 43103 -ep 43000 &


rtserver -rp 41104 -ep 41000 -tp 1401 & # ch 4
rtserver -rp 43104 -ep 43000 -tp 1402 & # ch 3

ssh radar@kodiak-aux 'python3 /home/radar/SuperDARN_UHD_Server/tools/srr_watchdog.py &' &
python3 /home/radar/repos/usrp/tools/srr_watchdog.py server &

sleep 15
schedule -l /data/ros/scdlog -f /data/ros/scd/kod.d.scd & #TODO add channel c

#gdb -ex run --args uafscan --stid mcm -c 1 --debug # control program connects to usrp server
