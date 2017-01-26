#!/bin/bash
#pycallgraph graphviz -- 
python3 -m cProfile -o cprofile_usrp_server usrp_server.py
