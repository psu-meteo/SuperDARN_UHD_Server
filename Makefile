OS:=$(shell uname -s)

NVCC=nvcc
CCpp=g++
CFLAGS=-c -Wall -pthread -std=c++0x -g3
LFLAGS  = -L"../iniparser3.0b" -L"${HOME}/uhd/host/build/lib" -L"/usr/local/cuda/lib64" -L"/usr/local/cuda/lib"
common_libs=-lm -liniparser -lrt -luhd -lfftw3 -lcuda -lcudart
linux_libs=-lboost_thread -lboost_system
INCLUDES=-I"../include/" -I"include/" -I"../tsg/include" -I"../iniparser3.0b/src/" \
	 -I"${HOME}/uhd/host/include" -I"${HOME}/uhd/host/include/uhd/types" -I"${HOME}/uhd/host/include/uhd/usrp" 

CPP_SOURCES=usrp_driver.cpp dio.cpp burst_worker.cpp recv_clr_freq.cpp
CUDA_SOURCE0=rx_cuda.cu
CUDA_SOURCE1=tx_cuda.cu

CPP_OBJECTS=$(CPP_SOURCES:.cpp=.o)
CUDA_OBJECT0=$(CUDA_SOURCE0:.cu=.o)
CUDA_OBJECT1=$(CUDA_SOURCE1:.cu=.o)

EXECUTABLE=usrp_driver

all: $(EXECUTABLE)


$(EXECUTABLE): $(CPP_OBJECTS)
	$(CCpp) -o $@ $(CUDA_OBJECT0) $(CUDA_OBJECT1) $(CPP_OBJECTS) $(LFLAGS) $(common_libs) $(linux_libs) ${INCLUDES}

CUDA_OBJECT0:
	$(NVCC) $(CUFLAGS) $(CUDA_SOURCE0) -o $(CUDA_OBJECT0)

CUDA_OBJECT1:
	$(NVCC) $(CUFLAGS) $(CUDA_SOURCE1) -o $(CUDA_OBJECT1)

.cpp.o:
	$(CCpp) $(CFLAGS) $< -o $@ $(LFLAGS) $(INCLUDES)

clean:
	rm -rf *.o $(EXECUTABLE)

