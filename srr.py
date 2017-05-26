#!/usr/bin/python3
# Software Radio Radar
#  Function to quickly start and terminate single radar processes.
#
#  Usage:
#     srr.py [status]              : shows all software radio processes 
#     srr.py init [auto|main|aux]  : create symlink to usrp_config file (default is auto)
#     srr.py start   PROCESS       : start a process or process group (see below for processes) 
#     srr.py stop    PROCESS       : stop a process or process group (see below for processes) 
#     srr.py restart PROCESS       : restart a process or process group with needed wait times
#     srr.py networkTool           : calls networkTool.py 
#     srr.py rawView               : plot raw rx bb samples of all antennas
#     srr.py help                  : show this help
#
#  Available PROCESSES:
#     cuda (or cuda_driver)        : cuda driver
#     usrps (or usrp_driver)       : all usrps defined in usrp_config.ini
#     server (or usrp_server)      : usrp_server
#   
#     errlog (or errorlog)         : errlog server            (no restart)
#     rawacf (or rawacfwrite)      : rawacf write process     (no restart)
#     fitacf (or fitacfwrite)      : fitacf write process     (no restart)
#     rtserver                     : real time display server (no restart)
#   
#     uaf_fix (or uafscan_fix)     : starts uafscan fast with fixfreq 14000 (only start)
#     uaf_fix ch_No                : starts uaf_fix on channel cd_No (1-4)
#     uaf_fix_onesec [ch_No]       : starts uafscan (onsec, fixfreq) default ch_No=1
#
#  Available PROCESS GROUPS:
#     driver                       : cuda_driver and usrp_driver
#     all                          : cuda_driver, usrp_driver and usrp_server
#     allscans                     : all porcesses with scan in their name
#
#
#  TIP:
#    set alias to shorten "srr.py" to "srr" and call it from anywhere:
#       alias srr="/home/radar/SuperDARN_UHD_Server/srr.py "
#    and make it permanent by including it in ~/.bash_aliases
#

#  TODO:
# restart is processes already running?
# get all differnt call types "python3 cuda_driver.py" ".../python3 ./cuda_driver.py"
# add return argument to stop commands and don't wait on restart if nothing has been shut down
# 


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

sys.path.insert(0, 'tools')
import networkTool


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
myPrint(" ")

def waitFor(nSeconds):
   print("||> Waiting for {} second(s): ".format(nSeconds), end="", flush=True)
   for i in range(nSeconds):
      print('.', end="", flush=True)
      time.sleep(1)
   else:
      print
######################
## init
def initialize(inputArg):
   configPath = basePath # might change later...

   usrp_config_source_file = dict(main=configPath + "/usrp_config__kod-main.ini", aux=configPath + "/usrp_config__kod-aux.ini")
   usrp_config_target_file = configPath + "/usrp_config.ini"

   if len(inputArg) == 1 or inputArg[1].lower() == "auto":
      hostName = os.uname()[1].lower()
      myPrint(" Auto detect of hostname: {}".format(hostName))
   else:
      hostName = inputArg[1].lower()

   if hostName in ['main', 'kod-main', 'kodiak-main']:
      computer = "main"
   elif hostName in ['aux', 'kod-aux', 'kodiak-aux']: 
      computer = "aux"
   else:
      myPrint(" Error. Unkown copmuter name. Use MAIN or AUX")
      return

   myPrint(" Initializing for computer: {}".format(computer))

   if os.path.isfile(usrp_config_target_file):
      if os.path.islink(usrp_config_target_file): # is a link
          myPrint("  removing link for {}".format(usrp_config_target_file))
          os.unlink(usrp_config_target_file)
      else: # is a file
          myPrint("  removing file {}".format(usrp_config_target_file))
          os.remove(usrp_config_target_file)
       
   
   os.symlink(usrp_config_source_file[computer], usrp_config_target_file)
   myPrint("Creating symlink {} -> {}".format(usrp_config_target_file, usrp_config_source_file[computer]))

def show_help():
   thisFile = open( os.path.realpath(__file__), 'r')
   helpText = []
   isFirst = True
   for line in thisFile:
      if isFirst:
         isFirst = False
         continue
      if line.startswith("#"):
         helpText.append(line[1:])
         myPrint(line[1:-1])
      else:
         break
#   print(helpText)
      

def set_alias():
   # does only work for the subprocess session...
   aliasPar = 'alias srrt="{}/srr.py " '.format(basePath)
   subprocess.call(['alias', aliasPar], shell=True)
   myPrint("setting alias srr to script")

######################
## Processes:

def get_processes():
   commandList = ["ps", "-aux"]
   s = subprocess.Popen(commandList, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
   out, err = s.communicate()
   processList = out.decode("UTF-8").split("\n")
   return processList

def print_status():
    kownProcessList = ['./usrp_driver', "/usr/bin/python3 ./cuda_driver.py", "python3 cuda_driver.py",  "/usr/bin/python3 ./usrp_server", "uafscan", "fitacfwrite", "rawacfwrite", "errlog", "rtserver"]
    processList = get_processes()
    srrProcesses = []
    for line in processList:
        wordList = [word for word in line.split(" " ) if word != ""]
        if len(wordList):
           commandString = " ".join(wordList[10:]) 
           for knowProcess in kownProcessList:
               if commandString.startswith(knowProcess):
                  srrProcesses.append( " " + commandString + "  (PID " + wordList[1] + ")")
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
    elif processShortName == "rtserver":
       processMatchString = ["rtserver"]
       nWords = 0
    elif processShortName == "scan":
       processMatchString = ["uafscan", "normalscan"]
       nWords = 0
    else:
       nWords = 0
       processMatchString = [processShortName]
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
    
def stop_rtserver():
    myPrint(" Stopping rtserver...")
    serverProcesses = get_process_ids("rtserver")
    if len(serverProcesses):
       terminate_all(serverProcesses)
       return 1
    else:
       myPrint("  No rtserver processes found...")
       return 0
    
def stop_allscans():
    myPrint(" Stopping scans...")
    serverProcesses = get_process_ids("scan")
    if len(serverProcesses):
       terminate_all(serverProcesses)
       return 1
    else:
       myPrint("  No errlog processes found...")
       return 0
def stop_errorlog():
    myPrint(" Stopping errlog...")
    serverProcesses = get_process_ids("errlog")
    if len(serverProcesses):
       terminate_all(serverProcesses)
       return 1
    else:
       myPrint("  No errlog processes found...")
       return 0
    
def stop_fitacf_write():
    myPrint(" Stopping fitacfwrite...")
    serverProcesses = get_process_ids("fitacfwrite")
    if len(serverProcesses):
       terminate_all(serverProcesses)
       return 1
    else:
       myPrint("  No fitacfwrite processes found...")
       return 0
    
def stop_rawacf_write():
    myPrint(" Stopping rawacfwrite...")
    serverProcesses = get_process_ids("rawacfwrite")
    if len(serverProcesses):
       terminate_all(serverProcesses)
       return 1
    else:
       myPrint("  No rawacfwrite processes found...")
       return 0
    
    

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
    if not os.path.isfile(fileName):
       myPrint("  ERROR: usrp_config.ini not found! Run srr init or symlink correct init file by hand.")
       return -1
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
       #with internal clock
#       usrpPIDlist.append( subprocess.Popen(['./usrp_driver', '--intclk', '--antenna', usrp_config[usrpName]['array_idx']  , '--host', usrp_config[usrpName]['usrp_hostname'] ]))
       usrpPIDlist.append( subprocess.Popen(['./usrp_driver',  '--antenna', usrp_config[usrpName]['array_idx']  , '--host', usrp_config[usrpName]['usrp_hostname'] ]))
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

def start_uafscan_fixfreq_onesec(inputArg):
    commandList ='uafscan --stid mcm -c 1 --nowait --fixfrq 14000 --onesec --debug'.split(" ")
    if len(inputArg) == 2:
       myPrint("Starting uafscan fixfreq onesec on default channel 1")
    else:
       myPrint("Starting uafscan fixfreq onesec on default channel {}".format(inputArg[2]))
       commandList[4] = inputArg[2]
    myPrint("  >>>{}".format(" ".join(commandList)))
    subprocess.Popen(commandList)

def start_uafscan_fixfreq(inputArg):
    commandList ='uafscan --stid mcm -c 1 --nowait --fixfrq 14000 --fast --debug'.split(" ")
    if len(inputArg) == 2:
       myPrint("Starting uafscan fixfreq on default channel 1")
    else:
       myPrint("Starting uafscan fixfreq on default channel {}".format(inputArg[2]))
       commandList[4] = inputArg[2]
    myPrint("  >>>{}".format(" ".join(commandList)))
    subprocess.Popen(commandList)


def start_normalscan():
    command = 'normalscan -stid mcm -xcf 1 -fast -df 10400 -nf 10400 -c 4'
    myPrint("Starting normalscan...({})".format(command))
    subprocess.Popen(command.split(" "))
def start_2normalscans():
    command = 'normalscan -stid mcm -xcf 1 -fast -df 10400 -nf 10400 -c 3'
    myPrint("Starting first (of two) normalscan...({})".format(command))
    subprocess.Popen(command.split(" "))
    command = 'normalscan -stid mcm -xcf 1 -fast -df 10400 -nf 10400 -c 4'
    myPrint("Starting second (of two) normalscan...({})".format(command))
    subprocess.Popen(command.split(" "))

def start_rtserver():
    commandList = 'rtserver -rp 41104 -ep 41000 -tp 1401'.split(" ")
    myPrint("Starting {}  ({})".format(commandList[0], " ".join(commandList)))
    subprocess.Popen(commandList)

def start_fitacf_write():
    commandList = 'fitacfwrite -r ade.a -lp 41103 -ep 41000'.split(" ")
    myPrint("Starting {}  ({})".format(commandList[0], " ".join(commandList)))
    subprocess.Popen(commandList)

def start_rawacf_write():
    commandList = 'rawacfwrite -r ade.a -lp 41102 -ep 41000'.split(" ")
    myPrint("Starting {}  ({})".format(commandList[0], " ".join(commandList)))
    subprocess.Popen(commandList)

def start_errorlog():
    commandList = 'errlog -name mcm.a -lp 41000'.split(" ")
    myPrint("Starting {} on port {} ({})".format(commandList[0], commandList[-1], " ".join(commandList)))
    subprocess.Popen(commandList)


def start_network_tool():
    myPrint("Calling printStatus() form tools/networkTool.py...")
    networkTool.printStatus()
    return
   # myPrint("Starting networkTool.py...")
   # os.chdir(os.path.join(basePath, "tools") )   
   # subprocess.Popen(['./networkTool.py' ])

def start_liveRawView_tool():
    myPrint("Starting tools/plotRawSamples_usrpServer.py...")
    os.chdir(os.path.join(basePath, "tools") )   
    subprocess.Popen(['./plotRawSamples_usrpServer.py' ])




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
   elif firstArg == "init":
       initialize(inputArg)
   elif firstArg.lower() in ["liverawview", "rawview"]:
        start_liveRawView_tool()  
   elif firstArg.lower() in ["network", "networktool"]:
        start_network_tool()  
   elif firstArg == "start":
      if nArguments == 1 or inputArg[1].lower == "all":
         myPrint("Starting all...")
         start_cuda_driver()
         start_usrp_driver()
         waitFor(10)
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
      elif inputArg[1].lower() in ["uaf_fix_onesec", "uafscan_fix_onesec"]:
         start_uafscan_fixfreq_onesec(inputArg)
      elif inputArg[1].lower() in ["uaf_fix", "uafscan_fix"]:
         start_uafscan_fixfreq(inputArg)
      elif inputArg[1].lower() in ["normalscan"]:
         start_normalscan()
      elif inputArg[1].lower() in ["2normalscans"]:
         start_2normalscans()
      elif inputArg[1].lower() in ["rtserver"]:
         start_rtserver()
      elif inputArg[1].lower() in ["errorlog", "errlog"]:
         start_errorlog()
      elif inputArg[1].lower() in ["fitacf", "fitacfwrite"]:
         start_fitacf_write()
      elif inputArg[1].lower() in ["rawacf", "rawacfwrite"]:
         start_rawacf_write()
      else:
         myPrint("ERROR: Unknown process to start")
         myPrint("See srr help for process names:")
         myPrint("")
         show_help()

   elif firstArg == "restart":
      if nArguments == 1 or inputArg[1].lower == "all":
         myPrint("Restarting all...")
         stop_usrp_server()
         waitFor(2)
         stop_usrp_driver()
         stop_cuda_driver()
         myPrint("waiting for {} sec".format(nSecs_restart_pause))
         waitFor(nSecs_restart_pause)
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
         myPrint("ERROR: Unknown process to restart")
         myPrint("See srr help for process names:")
         myPrint("")
         show_help()


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
      elif inputArg[1].lower() in ["rtserver"]:
         stop_rtserver()
      elif inputArg[1].lower() == "driver":
         stop_usrp_driver()
         stop_cuda_driver()
      elif inputArg[1].lower() in ["errorlog", "errlog"]:
         stop_errorlog()
      elif inputArg[1].lower() in ["fitacf", "fitacfwrite"]:
         stop_fitacf_write()
      elif inputArg[1].lower() in ["rawacf", "rawacfwrite"]:
         stop_rawacf_write()
      elif inputArg[1].lower() in ["scans", "allscans"]:
         stop_allscans()
      else:
         myPrint("ERROR: Unknown process to stop")
         myPrint("See srr help for process names:")
         myPrint("")
         show_help()
   elif firstArg == "help":
      show_help()
   
   else:
      myPrint("ERROR: UNKNWON COMMAND")
      myPrint("See srr help for usage:")
      myPrint("")
      show_help()
   
myPrint(" ")
print(basePrintLine)
print(basePrintLine)
   
   
   
   
   
   
   
   
   
   

