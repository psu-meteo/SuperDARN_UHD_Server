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
    
    def tearDown(self):
        posix_ipc.unlink_shared_memory(cuda_driver.shm_namer(0, 0, 0, 'rx'))

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


if __name__ == '__main__':
    unittest.main()
