#include <math.h>
#include <stdint.h>
#include "assert.h"

__global__ void multiply_and_add(float ***samples, float ***odata, float **filter)
{
    __shared__ float itemp[1024];//Array size is max number of threads in a block
    __shared__ float qtemp[1024];

    uint32_t tid = threadIdx.y*blockDim.x+threadIdx.x;
    int32_t tfilt;
    uint32_t tsamp;

    //Each block writes exactly a baseband sample for each frequency
    /*blockIdx.x corresponds to time-domain tiling; NBBSAMPS == gridDim.x
    blockIdx.y corresponds to antenna channel tiling; NANTS == gridDim.y*/
    int32_t output_idx = 2*blockIdx.x;

    // calculate index of sample from global memory samples array
    float stride = 1./8; // The number of filter taps is (1./stride) times the decimation rate
    tsamp = stride*4*(blockIdx.x*blockDim.x) + 4*threadIdx.x;

    float i0 = samples[threadIdx.y][blockIdx.y][tsamp];
    float q0 = samples[threadIdx.y][blockIdx.y][tsamp+1];
    float i1 = samples[threadIdx.y][blockIdx.y][tsamp+2];
    float q1 = samples[threadIdx.y][blockIdx.y][tsamp+3];


    // get filter values from global memory
    tfilt = 4*threadIdx.x;
    float p0re = filter[threadIdx.y][tfilt];
    float p0im = filter[threadIdx.y][tfilt+1];
    float p1re = filter[threadIdx.y][tfilt+2];
    float p1im = filter[threadIdx.y][tfilt+3];

    // mix samples with nco, perform first reduction
    itemp[tid] = p0re * i0 - p0im * q0 + p1re * i1 - p1im * q1;
    qtemp[tid] = p0re * q0 + p0im * i0 + p1re * q1 + p1im * i1;

     __syncthreads();

     // parallel reduce samples (could unroll loop for speedup?)
     // Do this as long as the reduction is a power of 2
     int32_t s, rem;
     s = blockDim.x;// / blockDim.y;
     rem = s % 2;
     while(s > 0 && rem == 0){
        s /= 2;
        if (threadIdx.x < s) {
            itemp[tid] += itemp[tid + s];
            qtemp[tid] += qtemp[tid + s];
        }
        rem = s % 2;

        __syncthreads();
     }
     //// Now do a serial reduction for the remaining
     if(threadIdx.x == 0){
        for(int32_t i=1; i<s; i++){
           itemp[tid] += itemp[tid + i];
           qtemp[tid] += qtemp[tid + i];
        }
     }
     __syncthreads();


     if (threadIdx.x == 0) {
        odata[threadIdx.y][blockIdx.y][output_idx] = (float) itemp[tid];
        odata[threadIdx.y][blockIdx.y][output_idx+1] = (float) qtemp[tid];
     }  
}       


__global__ void multiply_mix_add(int16_t **samples, float ***odata, float **filter)
{
    __shared__ float itemp[1024];
    __shared__ float qtemp[1024];

    uint32_t tid = threadIdx.y*blockDim.x+threadIdx.x;
    int32_t tfilt;
    uint32_t tsamp;

    //Each block writes exactly a baseband sample for each frequency
    /*blockIdx.x corresponds to time-domain tiling; NBBSAMPS == gridDim.x
    blockIdx.y corresponds to antenna channel tiling; NANTS == gridDim.y
    threadIdx.y corresponds to frequency channel tiling; NFREQS == blockDim.y*/
    int32_t output_idx = 2*blockIdx.x;


    // calculate index of sample from global memory samples array
    float stride = 1./2; // The number of filter taps is (1./stride) times the decimation rate
    tsamp = stride*4*(blockIdx.x*blockDim.x) + 4*threadIdx.x;

    // get filter values from global memory
    tfilt = 4*threadIdx.x;
    // mix samples with nco, perform first reduction
    assert(tsamp <= (4 * blockDim.x * gridDim.x));

    itemp[tid] =
        filter[threadIdx.y][tfilt] * samples[blockIdx.y][tsamp] -
        filter[threadIdx.y][tfilt+1] * samples[blockIdx.y][tsamp+1] +
        filter[threadIdx.y][tfilt+2] * samples[blockIdx.y][tsamp+2] -
        filter[threadIdx.y][tfilt+3] * samples[blockIdx.y][tsamp+3];
    qtemp[tid] =
        filter[threadIdx.y][tfilt] * samples[blockIdx.y][tsamp+1] +
        filter[threadIdx.y][tfilt+1] * samples[blockIdx.y][tsamp] +
        filter[threadIdx.y][tfilt+2] * samples[blockIdx.y][tsamp+3] +
        filter[threadIdx.y][tfilt+3] * samples[blockIdx.y][tsamp+2];

     __syncthreads();

     /* Example: dmrate==100,
     100 -> 50 -> 25 -> 5 -> 1*/
     // parallel reduce samples (could unroll loop for speedup?)
     // Do this as long as the reduction is a power of 2
     int32_t s, rem;
     s = blockDim.x;
     rem = blockDim.x % 2;
     while(s > 0 && rem == 0){
        s /= 2;
        if (threadIdx.x < s) {
            itemp[tid] += itemp[tid + s];
            qtemp[tid] += qtemp[tid + s];
        }
        rem = s % 2;

        __syncthreads();
     }
     // Do this as long as the reduction is a power of 5
     rem = s % 5;
     while(s > 0 && rem == 0){
        s /= 5;
        if (threadIdx.x < s) {
            itemp[tid] = itemp[tid] + itemp[tid + s] + itemp[tid+2*s] + itemp[tid+3*s] + itemp[tid+4*s];
            qtemp[tid] = qtemp[tid] + qtemp[tid + s] + qtemp[tid+2*s] + qtemp[tid+3*s] + qtemp[tid+4*s];
        }
        rem = s % 5;

        __syncthreads();
     }

     // Now do a serial reduction for the remaining
     if(threadIdx.x == 0){
        for(int32_t i=1; i<s; i++){
           itemp[tid] += itemp[tid + i];
           qtemp[tid] += qtemp[tid + i];
        }
     }
     __syncthreads();

     if (threadIdx.x == 0) {
        /*Now do phase adjustment on the output samples*/
        // phase increment of the NCO can be calculated from two adjacent filter taps
        double phase_inc =
            atan(filter[threadIdx.y][tfilt+3] / filter[threadIdx.y][tfilt+2]) -
            atan(filter[threadIdx.y][tfilt+1] / filter[threadIdx.y][tfilt]);

        /*Phase remainder exists because the NCO oscillator 
        may not complete an exact 360% rotation in a filter window*/
        double phi_rem = blockIdx.x*fmod((1*blockDim.x) * phase_inc, 2*M_PI);

        double ltemp = (double) itemp[tid];
        itemp[tid] = itemp[tid] * cos(phi_rem) - qtemp[tid] * sin(phi_rem);
        qtemp[tid] = ltemp * sin(phi_rem) + qtemp[tid] * cos(phi_rem);

        //deciding the output
        odata[threadIdx.y][blockIdx.y][output_idx] = (float) itemp[tid];
        odata[threadIdx.y][blockIdx.y][output_idx+1] = (float) qtemp[tid];
     }
}


