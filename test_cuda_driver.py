#!/usr/bin/python3
# test the cuda driver..
import unittest
import cuda_driver
import numpy as np

import posix_ipc
import pdb

class GPUTestCases(unittest.TestCase):
    def setUp(self):
        self.rxshm = cuda_driver.create_shm(0, 0, 0, 100, 'rx')
        self.txshm = cuda_driver.create_shm(0, 0, 0, 2e6, 'tx')

    def tearDown(self):
        posix_ipc.unlink_shared_memory(cuda_driver.shm_namer(0, 0, 0, 'rx'))
        posix_ipc.unlink_shared_memory(cuda_driver.shm_namer(0, 0, 0, 'tx'))

    def test_init(self):
        gpu = cuda_driver.ProcessingGPU()

    def test_generate_bb(self):
        gpu = cuda_driver.ProcessingGPU()
        trise = 0
        
        seqbuf = np.zeros(100)
        gpu.generate_bbtx(seqbuf, trise)

        self.assertTrue(np.array_equal(seqbuf > 0, gpu.tx_bb_indata > 0))

        seqbuf[0] = cuda_driver.X_BIT
        seqbuf[50] = cuda_driver.X_BIT
        seqbuf[99] = cuda_driver.X_BIT
        gpu.generate_bbtx(seqbuf, trise)
        self.assertTrue(np.array_equal(seqbuf > 0, gpu.tx_bb_indata > 0))

    def test_tx_upsample(self):
        gpu = cuda_driver.ProcessingGPU()

        trise = 0
        # TODO: where is center frequency of USRP sampling set?

        fc = [10e6]
        fsamp = 20e6
        nchannels = 1
        tdelay = [0]
        seqbuf = np.zeros(100)
        seqbuf[50] = cuda_driver.X_BIT

        gpu.generate_bbtx(seqbuf, trise)
        gpu.interpolate_and_multiply(fc, fsamp, nchannels, tdelay)
        gpu.txsamples_host_to_shm(self.txshm)

        
if __name__ == '__main__':
    unittest.main()
