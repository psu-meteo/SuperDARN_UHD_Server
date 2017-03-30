#!/usr/bin/python3
# software radio radar
#
#  srr.py [status]                                 : shows all software radio processes
#  srr.py stop [cuda[_driver] | usrp_driver | driver]       : stops all software radio processes or only the specified 
#  srr.py start [cuda[_driver] | usrp_driver | [usrp_]server] : comming soon...

#  TODO:
# restart is processes already running?
# get all differnt call types "python3 cuda_driver.py" ".../python3 ./cuda_driver.py"

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
CUDA_EXIT = ord('e')

USRPDriverPort = 54420
UHD_EXIT = ord('e')

USRP_SERVER_PORT = 45000
USRP_SERVER_QUIT  = '.'


basePath = os.path.dirname(os.path.realpath(__file__))
nSecs_restart_pause = 10

def myPrint(msg):
   print("||>  {}".format(msg))
basePrintLine = "||>==============================================================="

print(basePrintLine)
myPrint("Software Radio Radar")
print(basePrintLine)
myPrint(" ")


def waitFor(nSeconds):
   print("||> Waiting for {} second(s): ".format(nSeconds), end="", flush=True)
   for i in range(nSeconds):
      print('.', end="", flush=True)
      time.sleep(1)
   else:
      print


######################
## Processes:

def get_processes():
   commandList = ["ps", "-aux"]
   s = subprocess.Popen(commandList, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
   out, err = s.communicate()
   processList = out.decode("UTF-8").split("\n")
   return processList

def print_status():
    kownProcessList = ['./usrp_driver', "/usr/bin/python3 ./cuda_driver.py", "python3 cuda_driver.py",  "/usr/bin/python3 ./usrp_server", "uafscan"]
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
   
    myPrint("Found {} processes:".format(len(srrProcesses)))
    for line in srrProcesses:
       myPrint("  {}".format(line))

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
     myPrint("   killing pid {}".format(pid))
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
               myPrint("  not a driver process: {}".format(line))

    return usrpProcesses

def stop_usrp_driver_soft():
    usrpProcesses = get_usrp_driver_processes()
    if len(usrpProcesses) == 0:
        myPrint("  No usrp_driver processes")
        return
    myPrint("Found {} usrp_driver processes".format(len(usrpProcesses)))
    
    dtype = np.uint8
    
    for process in usrpProcesses:
        myPrint("  sending UHD_EXIT to {}:{} (pid {})".format(process['host'], process['antenna'] + USRPDriverPort, process['pid']))
        usrpsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        usrpsock.connect(('localhost', process['antenna'] + USRPDriverPort))
    
        usrpsock.sendall(dtype(UHD_EXIT).tobytes())
    
       # dstr = usrpsock.recv(dtype().nbytes )
       # myPrint('  => {}  received ?? as {} ({} / {}  bytes): {}'.format(__file__, dtype, len(dstr), dtype().nbytes , dstr  ))
       # data = np.fromstring(dstr, dtype=dtype)
       # myPrint("   data:{}".format(data))
    
    
    myPrint("  Done.")
    myPrint("  Again checking for usrp_driver processes...")
    usrpProcesses = get_usrp_driver_processes()
    myPrint("   Found {} usrp_driver processes".format(len(usrpProcesses)))

def stop_usrp_driver_hard():
  #  myPrint("Stop usrp_driver hard...")
    terminate_all(get_usrp_driver_processes())


def stop_usrp_driver():
    myPrint(" Stopping usrp_driver...")
    stop_usrp_driver_soft()
    stop_usrp_driver_hard()

def get_cuda_driver_processes():
    processList = get_processes()
    cudaProcesses = []
    for line in processList:
        wordList = [word for word in line.split(" " ) if word != ""]
        if len(wordList):
           commandString = " ".join(wordList[10:])
           print(commandString)
           if commandString in ["/usr/bin/python3 ./cuda_driver.py", "python3 cuda_driver.py"]:
              cudaProcesses.append(dict(pid=int(wordList[1]) ))
    return cudaProcesses

def get_process_ids(processShortName):
    if processShortName == "cuda":
       processMatchString = ["/usr/bin/python3 ./cuda_driver.py", "python3 cuda_driver.py"]
       nWords = 2 
    elif processShortName == "usrp_server":
       processMatchString = ["/usr/bin/python3 ./usrp_server.py"]
       nWords = 2
    else:
       ValueError("unknown process short name {}".format(processShortName))
     
    processList = get_processes()
    cudaProcesses = []
    for line in processList:
        wordList = [word for word in line.split(" " ) if word != ""]
        if len(wordList):
           commandString = " ".join(wordList[10:11+nWords])
           if commandString in processMatchString:
              cudaProcesses.append(dict(pid=int(wordList[1]) ))
    return cudaProcesses

def stop_cuda_driver():
    myPrint(" Stopping cuda_driver...")
    cudaProcesses = get_process_ids("cuda")
    if len(cudaProcesses):
       for process in cudaProcesses:
            myPrint("  sending CUDA_EXIT to localhost:{} (pid {})".format(CUDADriverPort, process['pid']))
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect(('localhost',CUDADriverPort))
            sock.sendall(np.uint8(CUDA_EXIT).tobytes())
        
       time.sleep(1)
       # terminate processes if they still exis
       terminate_all(cudaProcesses)
    else:
       myPrint("  No cuda_driver processes found...")
    
def stop_usrp_server():
    myPrint(" Stopping usrp_server...")
    serverProcesses = get_process_ids("usrp_server")
    if len(serverProcesses):
       for process in serverProcesses:
            myPrint("  sending SEVER_EXIT to localhost:{} (pid {})".format(USRP_SERVER_PORT, process['pid']))
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect(('localhost',USRP_SERVER_PORT))
            sock.sendall(np.int32(ord(USRP_SERVER_QUIT)).tobytes())
        
       time.sleep(1)
       # terminate processes if they still exis
       terminate_all(serverProcesses)
    else:
       myPrint("  No usrp_server processes found...")
    
    

########################
## START:
def read_config(fileName):
   # READ usrp_config.ini
   config = configparser.ConfigParser()
   config.read(fileName)
   return config

def start_usrps_from_config(usrp_sleep = False):
    myPrint("Starting usrp_driver from config:")
    fileName = os.path.join(basePath, 'usrp_config.ini')
    usrp_config = read_config(fileName)
    
    usrpNameList = usrp_config.sections()
    usrpPIDlist  = []
    
    myPrint("  Found {} usrps in config {}:".format(len(usrpNameList), fileName))
    for usrpName in usrpNameList:
       myPrint("   {} : antenna {},   ip: {}".format( usrpName , usrp_config[usrpName]['array_idx'],  usrp_config[usrpName]['usrp_hostname'] ))
      
    os.chdir(os.path.join(basePath, "usrp_driver") )   
    for usrpName in usrpNameList:
      # TODO: remove intclk when octoclock is connected
      # TODO: or do this with threading module?
       myPrint("Starting {}: ant {}, ip {}".format(usrpName, usrp_config[usrpName]['array_idx'] , usrp_config[usrpName]['usrp_hostname'] ))
       usrpPIDlist.append( subprocess.Popen(['./usrp_driver', '--intclk', '--antenna', usrp_config[usrpName]['array_idx']  , '--host', usrp_config[usrpName]['usrp_hostname'] ]))
       if usrp_sleep:
          time.sleep(8)
    os.chdir(basePath)
    
def start_usrp_driver():
    start_usrps_from_config()

def start_cuda_driver():
    myPrint("Starting cuda_driver...")
    os.chdir(os.path.join(basePath, "cuda_driver") )   
    subprocess.Popen(['./cuda_driver.py' ])
    os.chdir(basePath)


def start_usrp_server():
    myPrint("Starting usrp_server...")
    os.chdir(os.path.join(basePath, "usrp_server") )   
    subprocess.Popen(['./usrp_server.py' ])
    os.chdir(basePath)

def start_uafscan_fixfreq():
    myPrint("Starting uafscan fixfreq...")
#    os.chdir(os.path.join(basePath, "usrp_server") )   
    subprocess.Popen(['uafscan', '--stid', 'mcm', '-c', '1', '--nowait', '--fixfrq', '14000', '--debug' ])
#    os.chdir(basePath)






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
         myPrint("Starting all...")
         start_cuda_driver()
         start_usrp_driver()
         time.sleep(10)
         start_usrp_server()
      elif inputArg[1].lower() in ["usrp_driver", "usrps"]:
         start_usrp_driver()
      elif inputArg[1].lower() in ["cuda_driver", "cuda"]:
         start_cuda_driver()
      elif inputArg[1].lower() == "driver":
         start_cuda_driver()
         start_usrp_driver()
      elif inputArg[1].lower() in ["usrp_server", "server"]:
         start_usrp_server()
      elif inputArg[1].lower() in ["uaf_fix", "uafscan_fix"]:
         start_uafscan_fixfreq()
      else:
         myPrint("unknown process to start")

   elif firstArg == "restart":
      if nArguments == 1 or inputArg[1].lower == "all":
         myPrint("Restarting all...")
         stop_usrp_server()
         waitFor(2)
         stop_usrp_driver()
         stop_cuda_driver()
         myPrint("waiting for {} sec".format(nSecs_restart_pause))
         time.sleep(nSecs_restart_pause)
         start_cuda_driver()
         start_usrp_driver()
#         start_usrp_server()

      elif inputArg[1].lower() in ["usrp_driver", "usrps"]:
         stop_usrp_driver()
         myPrint("waiting for {} sec".format(nSecs_restart_pause))
         time.sleep(nSecs_restart_pause)
         start_usrp_driver()

      elif inputArg[1].lower() in ["cuda_driver", "cuda"]:
         stop_cuda_driver()
         start_cuda_driver()

      elif inputArg[1].lower() == "driver":
         stop_usrp_server() # this does not work....
         waitFor(2)
         stop_usrp_driver()
         stop_cuda_driver()
         waitFor(nSecs_restart_pause)
         start_cuda_driver()
         start_usrp_driver()
      elif inputArg[1].lower() in ["usrp_server", "server"]:
         stop_usrp_server()
         start_usrp_server()

      else:
         myPrint("unknown process to restart")


   elif firstArg == "stop":
      if nArguments == 1 or inputArg[1].lower == "all":
         myPrint("Stopping all...")
         stop_usrp_server()
         time.sleep(1)
         stop_cuda_driver()
         time.sleep(2)
         stop_usrp_driver()
      elif inputArg[1].lower() in ["usrp_driver", "usrps"]:
         stop_usrp_driver()
      elif inputArg[1].lower() in ["cuda_driver", "cuda"]:
         stop_cuda_driver()
      elif inputArg[1].lower() in ["usrp_server", "server"]:
         stop_usrp_server()
      elif inputArg[1].lower() == "driver":
         stop_usrp_driver()
         stop_cuda_driver()
      else:
         myPrint("unknown process to stop")
   elif firstArg == "edit":
       commandList = ['vim ', os.path.realpath(__file__) ] 
       myPrint(commandList)
       subprocess.Popen(commandList)
 
   else:
      myPrint("Unknown arguments ")
   
myPrint(" ")
print(basePrintLine)
print(basePrintLine)
   
   
   
   
   
   
   
   
   
   

