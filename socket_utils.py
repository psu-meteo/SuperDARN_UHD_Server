import numpy as np
import pickle
import socket

# pack i/q samples into uint32 for sending samples over the network
# between usrp_server and arbyserver
def complex_ui32_pack(isamp, qsamp):
    i_mask = 0xffff0000
    q_mask = 0x0000ffff
    packed_sample = (i_mask & (np.uint16(isamp) << 16)) + (q_mask & np.uint16(qsamp))
    # port of:
    #client_main[isamp] = ((uint32_t) (temp_main[1] << 16) & 0xffff0000) | ((uint32_t) temp_main[0] & 0x0000ffff);
    return np.uint32(packed_sample)

def recv_dtype(sock, dtype, nitems = 1):
    dstr = sock.recv(dtype().nbytes * nitems)
    data = np.fromstring(dstr, dtype=dtype, count=nitems)
    if nitems == 1:
        return data[0]
    return data

# pickle send/recv for sending arbitrary objects over the network between python processes
# could use json for compatibility with other languages..
# mostly to send large messy sequence objects to cuda_driver to generate baseband samples
def pickle_send(sock, data):
    serialized_data = pickle.dumps(data, pickle.HIGHEST_PROTOCOL)
    transmit_dtype(sock, np.uint32(len(serialized_data)))
    sock.sendall(serialized_data)

def pickle_recv(sock):
    pickle_len = recv_dtype(sock, np.uint32)
    pickle_data = sock.recv(pickle_len)
    data = pickle.loads(pickle_data)
    return data

def transmit_dtype(sock, data):
    dstr = sock.sendall(data.tobytes())
