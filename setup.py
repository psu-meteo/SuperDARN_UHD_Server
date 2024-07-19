#!/usr/bin/python3

from setuptools import setup, find_packages

setup(
    name="SuperDARN_UHD_Server",
    version="1.0",
    python_requires=">=3.3.0",
    url="https://github.com/UAF-SuperDARN-OPS/SuperDARN_UHD_Server",
    maintainer="UAF SuperDARN",
    maintainer_email="bgklug@alaska.edu",
    packages=find_packages(exclude=["tests", "*.tests", "*.tests.*", "tests.*"]),
    install_requires=[
        'pycuda', 'numpy', 'scipy', 'numexpr', 'posix-ipc', 'termcolor', 'matplotlib',
        'pytest'
    ]
)
