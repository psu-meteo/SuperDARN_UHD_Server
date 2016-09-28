#include <math.h>
#include <stdint.h>
#include "assert.h"
#include <stdio.h>

/*
INDEXING for multiply_and_add()

threadIdx.x = iFilterSample*2
threadIdx.y = iChannel
blockDim.x  = nFilterSamples /2
blockDim.y  = nChannles
blockIdx.x  = iSampleIF
blockIdx.y  = iAntenna
gridDim.x   = nSamplesIF
gridDim.y   = nAntennas


*/

/*
// was samples[threadIdx.y][blockIdx.y][tsamp];
__device__ size_t rf_sample_idx(uint32_t tsamp)
{   //      iChannel    * nChannles  * nFilterSamles/2  + iAntenna    * nFilterSamples/2 + 1/8  * 4 * iSampleIF*nFilterSamples/2  + 4* iFilterSample*2
    return (threadIdx.y * blockDim.y * blockDim.x)      + (blockIdx.y * blockDim.x) + tsamp;
   // nSamplesRF = ?
   // return tsamp + threadIdx.y *nSamplesRF + blockIdx.y * blockDim.y * nSamplesRF 

}

// was filter[threadIdx.y][tfilt]
__device__ size_t rf_filter_idx(uint32_t tfilt)
{
    return tfilt + threadIdx.y*blockDim.x*4; // return 4 * threadIdx.x * threadIdx.y;
}


// odata[threadIdx.y][blockIdx.y][output_idx] = (float) itemp[iThread_lin];
__device__ size_t rf_output_idx(int32_t output_idx)
{  		
    // old: return (threadIdx.y * blockDim.y * 2 * blockDim.x) + (blockIdx.y * 2 * blockDim.x) + output_idx; 
    return output_idx + 2 * gridDim.x * ( threadIdx.y + blockIdx.y * blockDim.y); // or even replace output_idx with blockIdx.x*2
}

*/
__global__ void multiply_and_add(float *samples, float *odata, float *filter)
{
    __shared__ float itemp[1024];//Array size is max number of threads in a block
    __shared__ float qtemp[1024];

    uint32_t iThread_lin = threadIdx.y*blockDim.x+threadIdx.x;  // linear thread index in block
 //   int32_t tfilt;
 //   uint32_t tsamp;

//    int32_t output_idx = 2*blockIdx.x;

    // calculate index of sample from global memory samples array
   // float stride = 1./8; // The number of filter taps is (1./stride) times the decimation rate
   // tsamp = stride*4*(blockIdx.x*blockDim.x) + 4*threadIdx.x;

    // TODO: get from cuda_driver.py!
    uint32_t decimationRate_if2bb = 75;  
//    float samplingFreq_rf = 8e6;

    uint32_t idxSample_filter = 4 * ( threadIdx.x  + gridDim.x * threadIdx.y);     // 2 samples per thread (unrolled) times 2 components per sample (I /Q) = 4
    uint32_t nSamples_if = decimationRate_if2bb * (gridDim.x - 1) + gridDim.x *2;  // nSamples_in = decimationRate * ( nSamples_out -1) + nSamples_filter
   
    uint32_t iSample_if   = blockIdx.x * decimationRate_if2bb + threadIdx.x *2;                                            // number of (complex) sample in rf signal
    uint32_t idxSample_if = iSample_if * 2  +  threadIdx.y * nSamples_if * 2 + blockIdx.y * blockDim.y * nSamples_if * 2 ; // index in memory (account for  I/Q, iChannel, iAntenna)


    float i0 =  samples[idxSample_if  ];
    float q0 =  samples[idxSample_if+1];
    float i1 =  samples[idxSample_if+2];
    float q1 =  samples[idxSample_if+3];

    // get filter values from global memory
    float p0re = filter[idxSample_filter  ];
    float p0im = filter[idxSample_filter+1];
    float p1re = filter[idxSample_filter+2];
    float p1im = filter[idxSample_filter+3];

   
    // multiply filter
    itemp[iThread_lin] = p0re * i0 - p0im * q0 + p1re * i1 - p1im * q1;
    qtemp[iThread_lin] = p0re * q0 + p0im * i0 + p1re * q1 + p1im * i1;

     __syncthreads();

    if (threadIdx.x == 0 && blockIdx.x == 1 ) {
       printf("rx_cuda: %d \n", idxSample_if);
       printf("rx_cuda: i/q: %f %f %f %f \n", i0, q0, i1, q1);
       printf("rx_cuda: filter: %f %f %f %f \n",p0re, p0im, p1re, p1im);
       printf("rx_cuda: iq temp 0 : %f %f  \n", itemp[iThread_lin], qtemp[iThread_lin]);
     //  for(int iTmp = 0; iTmp < blockDim.x * blockDim.y; iTmp++){
     //      printf("   tmp %d : %f  + i %f \n", iTmp, itemp[iTmp], qtemp[iTmp]);
     //  }

    }

     // parallel reduce samples (could unroll loop for speedup?)
     // Do this as long as the reduction is a power of 2
     int32_t s, rem;
     s = blockDim.x;// / blockDim.y;
     rem = s % 2;
     while(s > 0 && rem == 0){
        s /= 2;
        if (threadIdx.x < s) {
            itemp[iThread_lin] += itemp[iThread_lin + s];
            qtemp[iThread_lin] += qtemp[iThread_lin + s];
        }
        rem = s % 2;

        __syncthreads();
     }
     //// Now do a serial reduction for the remaining
     if(threadIdx.x == 0){
        for(int32_t i=1; i<s; i++){
           itemp[iThread_lin] += itemp[iThread_lin + i];
           qtemp[iThread_lin] += qtemp[iThread_lin + i];
        }
     }
     __syncthreads();


     if (threadIdx.x == 0) {

        uint32_t  output_idx = 2 * (blockIdx.x +  gridDim.x * ( threadIdx.y + blockIdx.y * blockDim.y)); 
        odata[output_idx  ] = (float)  itemp[iThread_lin];
        odata[output_idx+1] = (float)  qtemp[iThread_lin];
     }  
}       
/*
// was samples[blockIdx.y][tsamp];
__device__ size_t if_sample_idx(uint32_t tsamp)
{
    return (blockIdx.y * blockDim.x) + tsamp;
}

// was filter[threadIdx.y][tfilt]
__device__ size_t if_filter_idx(uint32_t tfilt)
{
    return 4 * threadIdx.x * threadIdx.y; 
}

// odata[threadIdx.y][blockIdx.y][output_idx] = (float) itemp[iThread_lin];
__device__ size_t if_output_idx(int32_t output_idx)
{
    return (threadIdx.y * blockDim.y * 2 * blockDim.x) + (blockIdx.y * 2 * blockDim.x) + output_idx; 
}

*/
__global__ void multiply_mix_add(int16_t *samples, float *odata, float *filter)
{
    __shared__ float itemp[1024];
    __shared__ float qtemp[1024];

    uint32_t iThread_lin = threadIdx.y*blockDim.x+threadIdx.x;
 //   int32_t tfilt;
    uint32_t tsamp;

    //Each block writes exactly a baseband sample for each frequency
    /*blockIdx.x corresponds to time-domain tiling; NBBSAMPS == gridDim.x
    blockIdx.y corresponds to antenna channel tiling; NANTS == gridDim.y
    threadIdx.y corresponds to frequency channel tiling; NFREQS == blockDim.y*/
    int32_t output_idx = 2*blockIdx.x;


    // calculate index of sample from global memory samples array
    float stride = 1./2; // The number of filter taps is (1./stride) times the decimation rate
    tsamp = stride*4*(blockIdx.x*blockDim.x) + 4*threadIdx.x;

    // mix samples with nco, perform first reduction
    assert(tsamp <= (4 * blockDim.x * gridDim.x));

    // TODO: get from cuda_driver.py!
    uint32_t decimationRate_rf2if = 32;  
 //   float samplingFreq_rf = 8e6;

    uint32_t idxSample_filter = 4 * ( threadIdx.x  + gridDim.x * threadIdx.y);     // 2 samples per thread (unrolled) times 2 components per sample (I /Q) = 4
    uint32_t nSamples_rf = decimationRate_rf2if * (gridDim.x - 1) + gridDim.x *2;  // nSamples_in = decimationRate * ( nSamples_out -1) + nSamples_filter
   
    uint32_t iSample_rf   = blockIdx.x * decimationRate_rf2if + threadIdx.x *2;   // number of (complex) sample in rf signal
    uint32_t idxSample_rf = iSample_rf * 2  +  threadIdx.y * nSamples_rf * 2 + blockIdx.y * blockDim.y * nSamples_rf * 2 ; // index in memory (account for  I/Q, iChannel, iAntenna)

    itemp[iThread_lin] =
        filter[idxSample_filter  ] * samples[idxSample_rf  ] -
        filter[idxSample_filter+1] * samples[idxSample_rf+1] +
        filter[idxSample_filter+2] * samples[idxSample_rf+2] -
        filter[idxSample_filter+3] * samples[idxSample_rf+3];
    qtemp[iThread_lin] =
        filter[idxSample_filter  ] * samples[idxSample_rf+1] +
        filter[idxSample_filter+1] * samples[idxSample_rf  ] +
        filter[idxSample_filter+2] * samples[idxSample_rf+3] +
        filter[idxSample_filter+3] * samples[idxSample_rf+2];

    if (threadIdx.x == 0 && blockIdx.x == 1 ) {
       float i0 = (float) samples[idxSample_rf  ];
       float q0 = (float) samples[idxSample_rf+1];
       float i1 = (float) samples[idxSample_rf+2];
       float q1 = (float) samples[idxSample_rf+3];

       // get filter values from global memory
       float p0re = filter[idxSample_filter  ];
       float p0im = filter[idxSample_filter+1];
       float p1re = filter[idxSample_filter+2];
       float p1im = filter[idxSample_filter+3];
     
       printf("rx_cuda:mix %d \n", idxSample_rf);
       printf("rx_cuda:mix i/q: %f %f %f %f \n", i0, q0, i1, q1);
       printf("rx_cuda:mix filter: %f %f %f %f \n",p0re, p0im, p1re, p1im);
    }
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
            itemp[iThread_lin] += itemp[iThread_lin + s];
            qtemp[iThread_lin] += qtemp[iThread_lin + s];
        }
        rem = s % 2;

        __syncthreads();
     }
     // Do this as long as the reduction is a power of 5
     rem = s % 5;
     while(s > 0 && rem == 0){
        s /= 5;
        if (threadIdx.x < s) {
            itemp[iThread_lin] = itemp[iThread_lin] + itemp[iThread_lin + s] + itemp[iThread_lin+2*s] + itemp[iThread_lin+3*s] + itemp[iThread_lin+4*s];
            qtemp[iThread_lin] = qtemp[iThread_lin] + qtemp[iThread_lin + s] + qtemp[iThread_lin+2*s] + qtemp[iThread_lin+3*s] + qtemp[iThread_lin+4*s];
        }
        rem = s % 5;

        __syncthreads();
     }

     // Now do a serial reduction for the remaining
     if(threadIdx.x == 0){
        for(int32_t i=1; i<s; i++){
           itemp[iThread_lin] += itemp[iThread_lin + i];
           qtemp[iThread_lin] += qtemp[iThread_lin + i];
        }
     }
     __syncthreads();

     if (threadIdx.x == 0) {
        /*Now do phase adjustment on the output samples*/
        // phase increment of the NCO can be calculated from two adjacent filter taps
        double phase_inc =
            atan(filter[idxSample_filter+3] / filter[idxSample_filter+2]) -
            atan(filter[idxSample_filter+1] / filter[idxSample_filter]);

        /*Phase remainder exists because the NCO oscillator 
        may not complete an exact 360% rotation in a filter window*/
        double phi_rem = blockIdx.x*fmod((1*blockDim.x) * phase_inc, 2*M_PI);
        double phiOffset = fmod(phase_inc * iSample_rf, 2*M_PI);

        if (blockIdx.x < 3) {
             printf("   => BlockIdx.x %d: phaseInc: %f  phi_rem: %f  phiOffset: %f  \n", blockIdx.x, phase_inc, phi_rem, phiOffset);
        }


     //   phi_rem = phiOffset; 
        double ltemp = (double) itemp[iThread_lin];
        itemp[iThread_lin] = itemp[iThread_lin] * cos(phi_rem) - qtemp[iThread_lin] * sin(phi_rem);
        qtemp[iThread_lin] = ltemp * sin(phi_rem) + qtemp[iThread_lin] * cos(phi_rem);

        output_idx = 2 * (blockIdx.x +  gridDim.x * ( threadIdx.y + blockIdx.y * blockDim.y)); 
        //deciding the output
        odata[output_idx  ] = (float)  itemp[iThread_lin];
        odata[output_idx+1] = (float)  qtemp[iThread_lin];
     }
}


