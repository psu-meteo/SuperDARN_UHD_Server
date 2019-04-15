#!/usr/bin/python3

from setuptools import setup, find_packages

setup(
    name="SuperDARN_UHD_Server",
    version="1.0",
    python_requires=">=3.3.0",
    packages=find_packages(exclude=["tests", "*.tests", "*.tests.*", "tests.*"]),
    install_requirements=[
        'pycuda', 'numpy', 'scipy', 'numexpr', 'posix-ipc', 'termcolor', 'matplotllib',
        'pytest'
    ]
)
