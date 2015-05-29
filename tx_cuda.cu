// CUDA function for transmit beamforming 
#include <math.h>
#include <stdint.h>
#include <complex.h>
// threadIdx.x - time
// blockDim.x - number of times (bb?)
// threadIdx.y - frequency
// blockDim.y - number of frequencies
// blockIdx.x - upsample factor
// blockIdx.y - antenna number

#define MAXFREQS 4
#define MAXANTS 32

__device__ __constant__ double txfreq_rads[MAXFREQS];
__device__ __constant__ float txphasedelay_rads[MAXANTS];


__global__ void interpolate_and_multiply(
    float* indata,
    int16_t** outdata,
    double* radfreqs,
    float* phase_delays
){
    /*Declare shared memory array for samples.
    Vectors are written into this array, one for each
    frequency/beam channel.  Then the thread block linearly combines
    the frequency/beam channels into a single vector to be
    transmitted on the antenna*/
    __shared__ float irf_samples[2000];
    __shared__ float qrf_samples[2000];

    //Calculate the increment between two adjacent rf samples
    float inc_i;
    float inc_q;
    inc_i = (indata[2*blockIdx.x+2] - indata[2*blockIdx.x]) / blockDim.x;
    inc_q = (indata[2*blockIdx.x+3] - indata[2*blockIdx.x+1]) / blockDim.x;

    /*Calculate the sample's phase value due to NCO mixing and beamforming*/
    float phase = fmod((double)(blockDim.x*blockIdx.x + threadIdx.x)*radfreqs[threadIdx.y], 2*M_PI) +
        blockIdx.y*phase_delays[threadIdx.y];

    /*Calculate the output sample vectors, one for each freq/beam channel*/
    unsigned int localInx = threadIdx.y*blockDim.x+threadIdx.x;
    irf_samples[localInx] =
        (indata[2*blockIdx.x] + threadIdx.x*inc_i) * cos(phase) -
        (indata[2*blockIdx.x+1] + threadIdx.x*inc_q) * sin(phase);
    qrf_samples[localInx] =
        (indata[2*blockIdx.x] + threadIdx.x*inc_i) * sin(phase) +
        (indata[2*blockIdx.x+1] + threadIdx.x*inc_q) * cos(phase);

    /* Now linearly combine all freq/beam channels into a single vector*/
    __syncthreads();
    unsigned int outInx = blockDim.x*blockIdx.x+threadIdx.x; // number of t
    if(threadIdx.y == 0){
        for (unsigned int i=1; i<blockDim.y; i++){
            irf_samples[threadIdx.x] += irf_samples[threadIdx.x + i*blockDim.x];
            qrf_samples[threadIdx.x] += qrf_samples[threadIdx.x + i*blockDim.x];
        }
        outdata[blockIdx.y][2*outInx] = (int16_t) (0.95*32768*irf_samples[threadIdx.x]);
        outdata[blockIdx.y][2*outInx+1] = (int16_t) (0.95*32768*qrf_samples[threadIdx.x]);
    }

}

