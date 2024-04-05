#sudo ./init_network.sh &
gdb -ex run --args uafscan --intclk --stid mcm --c 1 --df 13200 --nf 13200 --fast --debug # control program connects to usrp server
