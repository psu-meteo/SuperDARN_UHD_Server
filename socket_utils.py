import numpy as np
import socket

# pack i/q samples into uint32 for sending samples over the network
# between usrp_server and arbyserver
def _complex_ui32_pack(isamp, qsamp):
    i_mask = 0xffff0000
    q_mask = 0x0000ffff
    packed_sample = (i_mask & (np.uint16(isamp) << 16)) + (q_mask & np.uint16(qsamp))
    #client_main[isamp] = ((uint32_t) (temp_main[1] << 16) & 0xffff0000) | ((uint32_t) temp_main[0] & 0x0000ffff);
    return np.uint32(packed_sample)

def recv_dtype(sock, dtype, nitems = 1):
    dstr = sock.recv(dtype().nbytes * nitems)
    data = np.fromstring(dstr, dtype=dtype, count=nitems)
    if nitems == 1:
        return data[0]
    return data


def transmit_dtype(sock, data):
    dstr = sock.sendall(data.tobytes())
