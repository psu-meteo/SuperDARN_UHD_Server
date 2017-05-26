#!/usr/bin/python3
# 
# function to show network status
# usage:
# 


from os import listdir
import os
import threading
from queue import Queue
import time

# define font color and style
HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'



class interfaceDetails():
    pass
def getIFaceDetails():
    netInfoPath = '/sys/class/net/'
    interFacePaths = listdir(netInfoPath)
    interFacePaths.sort()
    # %%

    ifaceDetailList = [interfaceDetails() for i in interFacePaths]


    for iFace,ifaceName in enumerate(interFacePaths):
        ifaceDetailList[iFace].name = ifaceName
        with open(netInfoPath + ifaceName + '/operstate') as f:
                ifaceDetailList[iFace].state = f.readline()[:-1]
        try:
            with open(netInfoPath + ifaceName + '/carrier') as f:
                ifaceDetailList[iFace].carrier = f.readline()[:-1]
        except:
           ifaceDetailList[iFace].carrier = " "
        with open(netInfoPath + ifaceName + '/flags') as f:
                ifaceDetailList[iFace].flags = f.readline()[:-1]
        with open(netInfoPath + ifaceName + '/mtu') as f:
                ifaceDetailList[iFace].mtu = int(f.readline())
           
    for iIFace in ifaceDetailList:
        f = os.popen('ifconfig ' + iIFace.name +  ' | grep "inet\ addr" | cut -d: -f2 | cut -d" " -f1')
        iIFace.ip=f.read()[:-1]

        f = os.popen('ifconfig ' + iIFace.name +  ' | grep "inet\ addr" | cut -d: -f4 | cut -d" " -f1')
        iIFace.mask=f.read()[:-1]
    
    return ifaceDetailList

def printStatus():

    ifaceDetailList = getIFaceDetails()

    # %% try color output
    fillerSpace = '         '
    for iIFace in ifaceDetailList:
        iIFace.carrier = iIFace.carrier.replace('1', fillerSpace[:5] + OKGREEN + "1" + ENDC)
        iIFace.state   = iIFace.state.replace('up', fillerSpace[:-1] + OKGREEN + "up" + ENDC)
        if iIFace.mtu > 8000:
            tmpFormat = OKGREEN
        else:
            tmpFormat = WARNING
        iIFace.mtu     = "{}{:>10}{}".format(tmpFormat, iIFace.mtu,  ENDC)


           
    # %% print infos
    print('\n   ' + BOLD +UNDERLINE + "{:>3}|{:>8}|{:>10}|{:>9}|{:>6}|{:>9}|{:>15}|{:>15}".format('Index', 'Name',  'State',  'flags', 'Carrier', 'MTU', 'IP', 'Mask') + ENDC)
    for idx, iIFace in enumerate(ifaceDetailList):
        print("{:>6} {:>10} {:>10}    {:<6} {:>6} {:>10} {:>15} {:>15}".format(idx, iIFace.name, iIFace.state, iIFace.flags, iIFace.carrier,  iIFace.mtu, iIFace.ip, iIFace.mask))       


def ipScan(index):
    ifaceDetailList = getIFaceDetails()
    interFace = ifaceDetailList[int(index)]

    if interFace.mask[-4:] != ".0.0":
        print(FAIL + "Net mask has to be *.*.0.0 to be able to reach all IPs" + ENDC)
        return

    subnets =  [10, 20, 30] + list(range(40,60))   # check this ips first
    subnets.extend([i for i in range(1,254) if i not in subnets])
    activeIPs = []
    for ipSubnet in subnets:
        response = os.system("ping -q -c 1 -I " + interFace.name + ' 192.168.'  + str(ipSubnet) + '.2')
        if response == 0:
            print("192.168.{}.2 is responding".format(ipSubnet))
            activeIPs.append(ipSubnet)

    print(OKGREEN + "Finished:\nFound active IPs:")
    for iSubnet in activeIPs:
        print(' 192.168.'+ str(iSubnet) + '.2 is active ')
    print("\n" + ENDC)        



def ipScanFast(index):
    nThreads = 100
    ifaceDetailList = getIFaceDetails()
    interFace = ifaceDetailList[int(index)]
    print_lock = threading.Lock()

   
    if interFace.mask[-4:] != ".0.0":
        print(FAIL + "Net mask has to be *.*.0.0 to be able to reach all IPs" + ENDC)
        return

    def scanOneIP(ipSubnet):
        response = os.system("ping -q -c 1 -I " + interFace.name + ' 192.168.'  + str(ipSubnet) + '.2')
        if response == 0:
         #   print("192.168.{}.2 is responding".format(ipSubnet))
           activeIPs.append(ipSubnet)
           with print_lock:
               print("192.168.{}.2 active".format(ipSubnet))

    def threader():
        while True:
            worker = q.get()
            scanOneIP(worker)
            q.task_done()



    subnets =  [10, 20, 30] + list(range(40,60))   # check this ips first
    subnets.extend([i for i in range(1,254) if i not in subnets])
    activeIPs = []
   
    q = Queue()
    for iThread in range(nThreads):
        t = threading.Thread(target=threader)
        t.daemon = True
        t.start()

    for ipSubnet in subnets:
        q.put(ipSubnet)

    q.join()

    print(OKGREEN + "Finished:\nFound active IPs:")
    for iSubnet in activeIPs:
        print(' 192.168.'+ str(iSubnet) + '.2 is active ')
    print("\n" + ENDC)        





def main():
   userInput = 's'
   printStatus()
   while userInput != "":
       print(HEADER + "\n\n Input:\n  s    : print interface status\n  f    : IP scan fast\n  i    : IP scan (slow)\n  c    : continuous update status\n  Enter: quit" + ENDC)
       userInput = input(' Input: ')
     
       if userInput.lower() == 's':
           printStatus()
       if userInput.lower() == 'f':
           index = input(HEADER + "  Select Interface Index:" + ENDC )
           ipScanFast(index)
       if userInput.lower() == 'i':
           index = input(HEADER + "  Select Interface Index:" + ENDC )
           ipScan(index)
       if userInput.lower() == 'c':
          while True:
              os.system("clear")
              printStatus()
              time.sleep(2)

if __name__ == '__main__':
    main()


