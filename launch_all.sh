#sudo ./init_network.sh &
./run_usrp_driver.sh &
./run_cuda_driver.sh &
sleep 13
./run_usrp_server.sh &
sleep 2 
gdb -ex run --args uafscan --stid mcm --debug
