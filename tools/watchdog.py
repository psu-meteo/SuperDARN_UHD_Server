#!/usr/bin/python3
import os
import datetime 
import sys
import time 
basePath = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, basePath )
import srr


print_status = True
restart  = False
inf_loop = True



server_status_file = os.path.join(basePath, 'log', "usrp_server_status.txt")
check_period = 1 # sec
file_age_limit = 30 + 10

# define font color and style
HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'

def ok(string):
   return OKGREEN + string + ENDC

def warn(string):
   return WARNING + string + ENDC

def fail(string):
   return FAIL + string + ENDC

class usrpDriverWatcher():
   def __init__(self, fileName):
      self.config = srr.read_config(fileName)
      usrp_name_list = self.config.sections()
      self.usrp_list = []
      known_hosts = []
      for usrpName in usrp_name_list:
         if self.config[usrpName]['usrp_hostname'] in known_hosts:
            usrp = self.usrp_list[known_hosts.index(self.config[usrpName]['usrp_hostname'])]
            usrp.ant.append(self.config[usrpName]['array_idx'])
            usrp.side.append(self.config[usrpName]['side'])
         else:
            self.usrp_list.append(usrpClass(self.config[usrpName]))
            known_hosts.append(self.config[usrpName]['usrp_hostname'])
  
   def check_processes(self):
      usrp_processes = srr.get_usrp_driver_processes()
      active_host_list = [ process['host'] for process in usrp_processes]
      active_pid_list = [ process['pid'] for process in usrp_processes]
      for usrp in self.usrp_list:
         if usrp.host in active_host_list:
            usrp.pid = active_pid_list[active_host_list.index(usrp.host)]
         else:
            usrp.pid = None
   def restart_usrps(self):
       os.chdir(os.path.join(basePath, "usrp_driver") )
       for usrp in self.usrp_list:
         if usrp.pid == None:
            start_arg = usrp.get_start_arguments()
            print("Starting {}".format(" ".join(start_arg) ))
            subprocess.Popen(all_start_arg)

      


   def status_str(self):
      status = []
      status.append("  usrp driver processes")
      for usrp in self.usrp_list:
         if usrp.pid == None:
            tmp_str = fail("NA")
         elif usrp.pid > 0:
            tmp_str = ok("PID: " + str(usrp.pid))
         else:
            tmp_str = warn("restarting... ({})".format(-usrp.pid))
            usrp.pid += check_period
         status.append("   usrp: {} |  antennas:{:>8} |  {}".format(usrp.host, ", ".join(usrp.ant), tmp_str))
      return status

      
   

class usrpClass():
   def __init__(self, configDict):
      self.host = configDict['usrp_hostname']
      self.computer = configDict['driver_hostname']
      self.ant = [configDict['array_idx']]
      self.mainarray = configDict['mainarray']
      self.side = [configDict['side']]
      self.pid = None

   def get_start_arguments(self):
      start_arg = ["./usrp_driver", "--host", self.host]
      for iSide, side in enumerate(self.side):
         if side.lower() == 'a':
            start_arg += [ ["--antennaA"], self.ant[iSide]]
         elif side.lower() == 'b':
            start_arg += [ ["--antennaB"], self.ant[iSide]] 
         else:
            print("unknown usrp side: {}".format(side))
      return start_arg
      
fileName = os.path.join(basePath, 'usrp_config.ini')
usrp_driver_watcher = usrpDriverWatcher(fileName)

while True:
   os.system('clear')
   m_time = os.path.getmtime(server_status_file)
   now = time.mktime(time.localtime())
   file_age = now - m_time
   if print_status:
      print(" Age of status file: {:0.0f} sec\n".format(file_age))
      print("  usrp_server status:")

      with open(server_status_file, "r") as f:
         status = f.readlines()
      for line in status:
         if line[-1] == "\n":
            line = line[:-1]
         print("    " + line)
      print("\n\n")


      print("  usrp_driver:")
      usrp_driver_watcher.check_processes()
      stat_lines = usrp_driver_watcher.status_str()
      for line in stat_lines:
         print(line)


#   print("file: {}, now: {}, age: {} s".format(m_time, now, now-m_time))
   if file_age_limit > file_age:
       time.sleep(check_period)
   else:
       print("Age of status file is {} seconds. Restarting all processes (with srr.py) ...")
       if restart:
          srr.restart_all()
          time.sleep(20) # waiting for control loop to start
       else:
          print("Restarting disabled...")
          time.sleep(check_period)





