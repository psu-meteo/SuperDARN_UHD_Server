#!/usr/bin/python3
# test the cuda driver..
import unittest
import usrp_server 
import numpy as np
import os
import posix_ipc
import pdb

class ServerTestCases(unittest.TestCase):
    def setUp(self):
        # create mock arby server..
        # spawn usrp drivers..
        # spawn cuda server

        pass

    def tearDown(self):
        pass

    def test_syntax(self):
       self.assertTrue(True)
    
if __name__ == '__main__':
    unittest.main()
