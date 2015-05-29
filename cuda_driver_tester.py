# test the cuda driver..
import unittest
import cuda_driver


import posix_ipc


class GPUTestCases(unittest.TestCase):
    def setUp(self):
        self.rxshm = cuda_driver.create_shm(0, 0, 0, 'rx')
    
    def tearDown(self):
        posix_ipc.unlink_shared_memory(shm_namer(0, 0, 0, 'rx'))

    def test_init(self):
        self.gpu = cuda_driver.ProcessingGPU()

if __name__ == '__main__':
    unittest.main()
