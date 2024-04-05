#sudo ./init_network.sh &
# -ex run
uafscan --stid mcm -c 1 --fast --debug --df 13200 --fixfrq 13200 --nowait # control program connects to usrp server
