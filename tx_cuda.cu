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

// blocks: (sample index, frequency index, 1)
// grids : (baseband sample index, antenna index, pulse index)

#define MAXFREQS 8
#define MAXANTS 32

#define I_OFFSET 0
#define Q_OFFSET 1

__device__ __constant__ double txfreq_rads[MAXFREQS];
__device__ __constant__ float txphasedelay_rads[MAXANTS * MAXFREQS];

// index into rf sample array
__device__ size_t outdata_idx(void) {
    size_t upsamp_idx = threadIdx.x;
    size_t upsamp_offset = blockIdx.x * blockDim.x;
    size_t pulse_offset = blockDim.x * gridDim.x * blockIdx.z;
    size_t antenna_offset = blockIdx.y * blockDim.x * gridDim.z * gridDim.x;
    return antenna_offset + pulse_offset + upsamp_offset + upsamp_idx;
}

// index into input baseband samples
__device__ size_t indata_idx(int32_t offset) {
    return 2*blockIdx.x + offset;
}

// so, structure indata and outdata 
__global__ void interpolate_and_multiply(
    float* indata,
    int16_t* outdata
){
    /* Declare shared memory array for samples.
    Vectors are written into this array, one for each
    frequency/beam channel.  Then the thread block linearly combines
    the frequency/beam channels into a single vector to be
    transmitted on the antenna */
    // irf/qrf_samples indexed as baseband 
    __shared__ float irf_samples[1024];
    __shared__ float qrf_samples[1024];

    //Calculate the increment between two adjacent rf samples
    float inc_i;
    float inc_q;
    inc_i = (indata[indata_idx(I_OFFSET)+2] - indata[indata_idx(I_OFFSET)]) / blockDim.x;
    inc_q = (indata[indata_idx(Q_OFFSET)+2] - indata[indata_idx(Q_OFFSET)]) / blockDim.x;

    /*Calculate the sample's phase value due to NCO mixing and beamforming*/
    float phase = fmod((double)(blockDim.x*blockIdx.x + threadIdx.x)*txfreq_rads[threadIdx.y], 2*M_PI) +
        blockIdx.y*txphasedelay_rads[threadIdx.y];

    /*Calculate the output sample vectors, one for each freq/beam channel*/
    unsigned int localInx = threadIdx.y*blockDim.x+threadIdx.x;
    irf_samples[localInx] =
        (indata[indata_idx(I_OFFSET)] + threadIdx.x*inc_i) * cos(phase) -
        (indata[indata_idx(Q_OFFSET)] + threadIdx.x*inc_q) * sin(phase);
    qrf_samples[localInx] =
        (indata[indata_idx(I_OFFSET)] + threadIdx.x*inc_i) * sin(phase) +
        (indata[indata_idx(Q_OFFSET)] + threadIdx.x*inc_q) * cos(phase);

    /* Now linearly combine all freq/beam channels into a single vector */
    __syncthreads();
    size_t outidx = outdata_idx();
    if(threadIdx.y == 0){
        for (unsigned int i=1; i<blockDim.y; i++){
            irf_samples[threadIdx.x] += irf_samples[threadIdx.x + i*blockDim.x];
            qrf_samples[threadIdx.x] += qrf_samples[threadIdx.x + i*blockDim.x];
        }
        outdata[outidx] = (int16_t) (0.95*32768*irf_samples[threadIdx.x]);
        outdata[outidx+1] = (int16_t) (0.95*32768*qrf_samples[threadIdx.x]);
    }
}

