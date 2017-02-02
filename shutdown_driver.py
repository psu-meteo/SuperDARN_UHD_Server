#!/usr/bin/python3
# 

import subprocess
import re
import socket
import numpy as np

USRPDriverPort = 54420
UHD_EXIT = ord('e')



def get_processes():
   commandList = ["ps", "-aux"]
   s = subprocess.Popen(commandList, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
   out, err = s.communicate()

   processList = out.decode("UTF-8").split("\n")
   return processList


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




#processList = get_processes()


usrpProcesses = get_usrp_driver_processes()
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


print("Done.\nAgain checking for usrp_driver processes...")
usrpProcesses = get_usrp_driver_processes()
print("Found {} usrp_driver processes".format(len(usrpProcesses)))
   




