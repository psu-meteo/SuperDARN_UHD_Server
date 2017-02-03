#!/usr/bin/python3
# software radio radar
#
#  srr.py [status]                                 : shows all software radio processes
#  srr.py stop [cuda[_driver] | usrp_driver | driver]       : stops all software radio processes or only the specified 
#  srr.py start [cuda[_driver] | usrp_driver | [usrp_]server] : comming soon...

import sys
import os
import subprocess
import re
import socket
import numpy as np
import configparser
import subprocess
import time
import errno
import signal

# TODO: get from config
CUDADriverPort = 55420
USRPDriverPort = 54420
CUDA_EXIT = ord('e')
UHD_EXIT = ord('e')

######################
## Processes:

def get_processes():
   commandList = ["ps", "-aux"]
   s = subprocess.Popen(commandList, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
   out, err = s.communicate()
   processList = out.decode("UTF-8").split("\n")
   return processList

def print_status():
    kownProcessList = ['./usrp_driver', "/usr/bin/python3 ./cuda_driver.py", "/usr/bin/python3 ./usrp_server"]
    processList = get_processes()
    srrProcesses = []
    for line in processList:
        wordList = [word for word in line.split(" " ) if word != ""]
        if len(wordList):
           commandString = " ".join(wordList[10:]) 
           for knowProcess in kownProcessList:
               if commandString.startswith(knowProcess):
                  srrProcesses.append(commandString)
                  break
   
    print("Found {} processes:".format(len(srrProcesses)))
    for line in srrProcesses:
       print("  {}".format(line))

def pid_exists(pid):
    """Check whether pid exists in the current process table.  UNIX only. 
    """
    if pid < 0:
        return False
    if pid == 0:
        raise ValueError('invalid PID 0')
    try:
        os.kill(pid, 0)
    except OSError as err:
        if err.errno == errno.ESRCH:
            # ESRCH == No such process
            return False
        elif err.errno == errno.EPERM:
            # EPERM clearly means there's a process to deny access to
            return True
        else:
            # According to "man 2 kill" possible error values are (EINVAL, EPERM, ESRCH)
            raise
    else:
        return True

def terminate_pid(pid):
  if pid_exists(pid):
     print("   killing pid {}".format(pid))
     os.kill(pid, signal.SIGTERM)

def terminate_all(pidDictList):
    if len(pidDictList):
       for process in pidDictList:
           terminate_pid(process["pid"])


#######################
## STOP:
def get_usrp_driver_processes():
    processList = get_processes()
    usrpProcesses = []
    for line in processList:
        if 'usrp_driver' in line:
            try:
               wordList = [word for word in line.split(" " ) if word != ""]
               antenna = int(wordList[wordList.index("--antenna")+1])
               host = wordList[wordList.index("--host")+1]
               usrpProcesses.append(dict(pid=int(wordList[1]), host=host, antenna=antenna))
            except:
               print("  not a driver process: {}".format(line))

    return usrpProcesses

def stop_usrp_driver_soft():
    usrpProcesses = get_usrp_driver_processes()
    if len(usrpProcesses) == 0:
        print("  No usrp_driver processes")
        return
    print("Found {} usrp_driver processes".format(len(usrpProcesses)))
    
    dtype = np.uint8
    
    for process in usrpProcesses:
        print("  sending UHD_EXIT to {}:{} (pid {})".format(process['host'], process['antenna'] + USRPDriverPort, process['pid']))
        usrpsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        usrpsock.connect(('localhost', process['antenna'] + USRPDriverPort))
    
        usrpsock.sendall(dtype(UHD_EXIT).tobytes())
    
       # dstr = usrpsock.recv(dtype().nbytes )
       # print('  => {}  received ?? as {} ({} / {}  bytes): {}'.format(__file__, dtype, len(dstr), dtype().nbytes , dstr  ))
       # data = np.fromstring(dstr, dtype=dtype)
       # print("   data:{}".format(data))
    
    
    print("  Done.\nAgain checking for usrp_driver processes...")
    usrpProcesses = get_usrp_driver_processes()
    print(" Found {} usrp_driver processes".format(len(usrpProcesses)))

def stop_usrp_driver_hard():
  #  print("Stop usrp_driver hard...")
    terminate_all(get_usrp_driver_processes())


def stop_usrp_driver():
    print(" Stopping usrp_driver...")
    stop_usrp_driver_soft()
    stop_usrp_driver_hard()

def get_cuda_driver_processes():
    processList = get_processes()
    cudaProcesses = []
    for line in processList:
        wordList = [word for word in line.split(" " ) if word != ""]
        if len(wordList):
           commandString = " ".join(wordList[10:])
           if commandString.startswith("/usr/bin/python3 ./cuda_driver.py" ):
              cudaProcesses.append(dict(pid=int(wordList[1]) ))
    return cudaProcesses

def stop_cuda_driver():
    print(" Stopping cuda_driver...")
    cudaProcesses = get_cuda_driver_processes()
    if len(cudaProcesses):
       for process in cudaProcesses:
            print("  sending CUDA_EXIT to localhost:{} (pid {})".format(CUDADriverPort, process['pid']))
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect(('localhost',CUDADriverPort))
            sock.sendall(np.uint8(CUDA_EXIT).tobytes())
        
       time.sleep(1)
       # terminate processes if they still exis
       terminate_all(cudaProcesses)
    else:
       print("  No cuda_driver processes found...")
    
    
    

########################
## START:
def read_config(fileName):
   # READ usrp_config.ini
   config = configparser.ConfigParser()
   config.read(fileName)
   return config

def start_usrps_from_config(usrp_sleep = False):
    print("Starting usrp_driver from config:")
    fileName ='usrp_config.ini'
    usrp_config = read_config(fileName)
    
    usrpNameList = usrp_config.sections()
    usrpPIDlist  = []
    
    print("  Found {} usrps in config {}:".format(len(usrpNameList), fileName))
    for usrpName in usrpNameList:
       print("   {} : antenna {},   ip: {}".format( usrpName , usrp_config[usrpName]['array_idx'],  usrp_config[usrpName]['usrp_hostname'] ))
      
    os.chdir("usrp_driver")    
    for usrpName in usrpNameList:
      # TODO: remove intclk when octoclock is connected
      # TODO: or do this with threading module?
       print("Starting {}: ant {}, ip {}".format(usrpName, usrp_config[usrpName]['array_idx'] , usrp_config[usrpName]['usrp_hostname'] ))
       usrpPIDlist.append( subprocess.Popen(['./usrp_driver', '--intclk', '--antenna', usrp_config[usrpName]['array_idx']  , '--host', usrp_config[usrpName]['usrp_hostname'] ]))
       if usrp_sleep:
          time.sleep(8)
    os.chdir("..")
    
def start_usrp_driver():
    start_usrps_from_config()

def start_cuda_driver():
    print("Starting cuda_driver...")
    os.chdir("cuda_driver")    
    subprocess.Popen(['./cuda_driver.py' ])
    os.chdir("..")


def start_usrp_server():
    print("Starting usrp_server...")
    print("not implemented jet")






#########################
## MAIN PART:

inputArg = sys.argv[1:]
nArguments = len(inputArg)

if nArguments == 0:   # DEFAULT option
   print_status()
else:
   firstArg = inputArg[0].lower()
   
   if firstArg == "status":
       print_status()

   elif firstArg == "start":
      if nArguments == 1 or inputArg[1].lower == "all":
         print("Starting all...")
         start_cuda_driver()
         start_usrp_driver()
         start_usrp_server()
      elif inputArg[1].lower() == "usrp_driver":
         start_usrp_driver()
      elif inputArg[1].lower() in ["cuda_driver", "cuda"]:
         start_cuda_driver()
      elif inputArg[1].lower() == "driver":
         start_usrp_driver()
         start_cuda_driver()
      elif inputArg[1].lower() in ["usrp_server", "server"]:
         start_usrp_server()
      else:
         print("unknown process to start")


   elif firstArg == "stop":
      if nArguments == 1 or inputArg[1].lower == "all":
         print("Stopping all...")
         stop_usrp_driver()
         stop_cuda_driver()
      elif inputArg[1].lower() == "usrp_driver":
         stop_usrp_driver()
      elif inputArg[1].lower() in ["cuda_driver", "cuda"]:
         stop_cuda_driver()
      elif inputArg[1].lower() == "driver":
         stop_usrp_driver()
         stop_cuda_driver()
      else:
         print("unknown process to stop")
   else:
      print("Unknown arguments ")
   
   
   
   
   
   
   
   
   
   
   
   

