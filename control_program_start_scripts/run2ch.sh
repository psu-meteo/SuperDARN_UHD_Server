#sudo ./init_network.sh &

#gdb -ex run --args uafscan --stid mcm -c 1 --debug # control program connects to usrp server
uafscan --stid mcm -c 3 --fast --camp 4 &
uafscan --stid mcm -c 4 --fast &

