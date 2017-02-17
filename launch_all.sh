#sudo ./init_network.sh &
./cuda_driver/run_cuda_driver.sh & # cuda driver creates shared memory
sleep 1
./usrp_driver/run_usrp_driver.sh & # usrp driver uses that shared memory
sleep 13
./usrp_server/run_usrp_server.sh & # usrp server connects to cuda and usrp drivers
sleep 2 
gdb -ex run --args uafscan --stid mcm -c 1 --debug # control program connects to usrp server
