import numpy as np
import socket

def recv_dtype(sock, dtype, nitems = 1):
    dstr = sock.recv(dtype().nbytes)
    data = np.fromstring(dstr, dtype=dtype, count=nitems)
    if nitems == 1:
        return data[0]
    return data


def transmit_dtype(sock, data):
    dstr = sock.send(data.tostring())
