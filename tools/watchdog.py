#!/usr/bin/python3

import os
import datetime 
import sys
import time 
basePath = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, basePath )
import srr


server_status_file = os.path.join(basePath, 'log', "usrp_server_status.txt")
check_period = 2 # sec
file_age_limit = 30 + 10

while True:
   m_time = os.path.getmtime(server_status_file)
   now = time.mktime(time.localtime())
   file_age = now - m_time
#   print("file: {}, now: {}, age: {} s".format(m_time, now, now-m_time))
   if file_age_limit > file_age:
       time.sleep(check_period)
   else:
       print("Age of status file is {} seconds. Restarting all processes (with srr.py) ...")
       srr.restart_all()
       time.sleep(20) # waiting for control loop to start





