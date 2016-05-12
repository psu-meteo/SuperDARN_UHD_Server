#!/usr/bin/python3
# test the usrp driver..
import unittest
import numpy as np
import mmap
import sys
import posix_ipc
import pdb
import time
import subprocess

from termcolor import cprint

from drivermsg_library import *
from socket_utils import *

# import pycuda stuff
SWING0 = 0
SWING1 = 1

SIDEA = 0
SIDEB = 1

RXDIR = 'rx'
TXDIR = 'tx'

DEBUG = True
C = 3e8

rx_shm_list = [[],[]]
tx_shm_list = []
semaphore_list = [[],[]]
swings = [SWING0, SWING1]
sides = [SIDEA]

# list of all semaphores/shared memory paths for cleaning up
shm_list = []
sem_list = []

swing = SWING0


START_DRIVER = True 
ANTENNA_UNDER_TEST = 1

rx_shm_list = [[],[]]
tx_shm_list = []
rx_semaphore_list = [[],[]] # [side][swing]
swings = [SWING0, SWING1]
sides = [SIDEA]

# list of all semaphores/shared memory paths for cleaning up
shm_list = []
sem_list = []

rxshm_size = 160000000
txshm_size = 51200000

RFRATE = 10000000

def sem_namer(ant, swing):
    name = 'semaphore_ant_{}_swing_{}'.format(int(ant), int(swing))
    return name

def shm_namer(antenna, swing, side, direction = 'rx'):
    name = 'shm_{}_ant_{}_side_{}_swing_{}'.format(direction, int(antenna), int(side), int(swing))
    return name

def create_shm(antenna, swing, side, shm_size, direction = 'rx'):
    name = shm_namer(antenna, swing, side, direction)
    memory = posix_ipc.SharedMemory(name, posix_ipc.O_CREAT, size=int(shm_size))
    mapfile = mmap.mmap(memory.fd, memory.size)
    memory.close_fd()
    shm_list.append(name)
    return mapfile

def create_sem(ant, swing):
    name = sem_namer(ant, swing)
    sem = posix_ipc.Semaphore(name, posix_ipc.O_CREAT)
    sem.release()
    sem_list.append(sem)
    return sem



if sys.hexversion < 0x030300F0:
    print('this code requires python 3.3 or greater')
    sys.exit(0)

def start_usrpserver():
    # open up ports..
    if not START_DRIVER:
        return -1
    usrp_driver = subprocess.Popen(['./usrp_driver', '--intclk', '--antenna', str(ANTENNA_UNDER_TEST), '--host', 'usrp' + str(ANTENNA_UNDER_TEST)])
    time.sleep(8)
    return usrp_driver.pid

def stop_usrpserver(sock, pid):
    # transmit clean exit command
    exitcmd = usrp_exit_command([sock])
    exitcmd.transmit()
    print('stopping USRP server')

    # kill the process just to be sure..
    if not START_DRIVER:
        return
    
    print('killing usrp_driver')
    subprocess.Popen(['pkill', 'usrp_driver'])
    time.sleep(10) # delay to let the usrp_driver settle..
    #pdb.set_trace()
   
if __name__ == '__main__':
    make = subprocess.call(['make'])

    print('creating shared memory..')
    antennas = [1]
    for ant in antennas:
        rx_shm_list[SIDEA].append(create_shm(ant, SWING0, SIDEA, rxshm_size, direction = RXDIR))
        rx_shm_list[SIDEA].append(create_shm(ant, SWING1, SIDEA, rxshm_size, direction = RXDIR))
        tx_shm_list.append(create_shm(ant, SWING0, SIDEA, txshm_size, direction = TXDIR))
        tx_shm_list.append(create_shm(ant, SWING1, SIDEA, txshm_size, direction = TXDIR))

    rx_semaphore_list[SIDEA].append(create_sem(ant, SWING0))
    rx_semaphore_list[SIDEA].append(create_sem(ant, SWING1))

    time.sleep(1)
    
    pid = start_usrpserver()

    print('started usrp server..')
    while True:
        time.sleep(10)
