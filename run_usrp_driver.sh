#!/bin/bash
#pycallgraph graphviz -- 
#gdb --args ./usrp_driver --intclk --antenna 0 --host usrp1

./usrp_driver --intclk --antenna 0 --host usrp1
echo "press enter to continue: "
read
