<!---
*BOLD*
1. List
2. Second List
- unordered list
[Inline Link](www.github.com/loxodes)
```python
print('hello')
```
-->

# SuperDARN USRP Imaging Overview
This repository houses code for a experimental rewrite of SuperDARN ROS code to adapt them use Ettus software defined radios.
The first deployment is planned for Kodiak radar.

This code is intended to integrate with existing code SuperDARN ROS Linux computers.
It replaces all software from arby\_server on down, including the timing, dio, and receiver cards.

## usrp\_server.py
usrp\_server interfaces with existing SuperDARN control programs, it is a replacement for the old QNX arby\_server code. 
usrp\_server talks with usrp\_server\_uhd.py, which is a python layer to interace with the C++ usrp\_driver.
In an imaging configuration with distributed USRPs across multiple  computers, this drive may talk to usrp\_server\_uhd drivers across multiple computers.

## usrp\_server\_uhd.py
usrp\_server\_uhd provides a python interface to the the UHD.

## cuda\_driver.py
cuda\_driver does downconversion of RF samples from usrp\_driver. 

## usrp\_driver.cpp
usrp\_driver uses the UHD API to tune a USRP and grab samples. 
In an imaging configuration with multiple USRPs, one of these would run for each USRP. 
Tested on UHD release 003.009.004, underflow issues on .003.010.001

# Dependencies
python 3 with numpy, pycuda

## Starting:
- run `.tools/init_network.sh`
- run `./launch_all.sh`
- run `uafscan --stid tst --debug`

