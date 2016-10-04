#include <math.h>
#include <stdint.h>
#include "assert.h"
#include <stdio.h>

/*
INDEXING for multiply_mix_add() (similar for multiply_and_add(), just IF => BB )

threadIdx.x = iFilterSample*2
threadIdx.y = iChannel
blockDim.x  = nFilterSamples /2
blockDim.y  = nChannels
blockIdx.x  = iSampleIF
blockIdx.y  = iAntenna
gridDim.x   = nSamplesIF
gridDim.y   = nAntennas

Input / Output format:
 - [rx_samples_FB] = nAntennas x nChannels x 2*nSamples_FB(I/Q-Interleaved)
   for all frequency bands FB: RF, IF, and BB
 - Filter has be be time reversed (doesn't matter for symmetric filters)

TODO:
 - if filter_RFIF is complex: check if phase correction of local oscillator is correct
 - if filter is real we could save calc time by removing all calculations with imag(filter) and phase correction, maybe write additional function multiply_add_real
 - what is block arround line 122 checking for?
 - get decimation rates from cuda_driver

*/

__global__ void multiply_and_add(float *samples, float *odata, float *filter)
{
    __shared__ float itemp[1024];//Array size is max number of threads in a block
    __shared__ float qtemp[1024];

    // clear variable names
    uint32_t iFilterSampleTimes2  = threadIdx.x;
    uint32_t iChannel             = threadIdx.y;
    uint32_t nFilterSamplesDivBy2 = blockDim.x;
    uint32_t nChannels            = blockDim.y;
    uint32_t iSampleBB            = blockIdx.x;
    uint32_t iAntenna             = blockIdx.y;
    uint32_t nSamplesBB           = gridDim.x;
//    uint32_t nAntennas            = gridDim.y; 

    uint32_t iThread_lin = iFilterSampleTimes2 + iChannel*nFilterSamplesDivBy2;   // linear thread index in block

    // TODO: get from cuda_driver.py!
    uint32_t decimationRate_if2bb = 75;  

    uint32_t idxSample_filter = 4 * (iFilterSampleTimes2  + iChannel * nFilterSamplesDivBy2); // 2 samples/thread (unrolled) * 2 components/ sample (I /Q) = 4
    uint32_t nSamples_if = decimationRate_if2bb * (nSamplesBB -1) + nFilterSamplesDivBy2 * 2; // nSamples_in=decimationRate*(nSamples_out-1)+nSamples_filter
    uint32_t iSample_if = iSampleBB * decimationRate_if2bb + iFilterSampleTimes2 * 2;         // number of (complex) sample in rf signal
    uint32_t idxSample_if = iSample_if * 2 + iChannel * nSamples_if *2 + iAntenna * nChannels * nSamples_if *2; // index in memory (account for  I/Q, iChannel, iAntenna)

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

     // write back results
     if (threadIdx.x == 0) {

        uint32_t output_idx = iSampleBB *2 + iChannel * nSamplesBB *2 + iAntenna * nChannels * nSamplesBB *2; 
        odata[output_idx  ] = (float)  itemp[iThread_lin];
        odata[output_idx+1] = (float)  qtemp[iThread_lin];
     }  
}       




__global__ void multiply_mix_add(int16_t *samples, float *odata, float *filter)
{
    __shared__ float itemp[1024];
    __shared__ float qtemp[1024];
    

    // clear variable names
    uint32_t iFilterSampleTimes2  = threadIdx.x;
    uint32_t iChannel             = threadIdx.y;
    uint32_t nFilterSamplesDivBy2 = blockDim.x;
    uint32_t nChannels            = blockDim.y;
    uint32_t iSampleIF            = blockIdx.x;
    uint32_t iAntenna             = blockIdx.y;
    uint32_t nSamplesIF           = gridDim.x;
   // uint32_t nAntennas            = gridDim.y; 

    uint32_t iThread_lin = threadIdx.y*blockDim.x+threadIdx.x;
    
    // TODO: mgu delete next block if I know what assert is checking for
    uint32_t tsamp;
    // calculate index of sample from global memory samples array
    float stride = 1./2; // The number of filter taps is (1./stride) times the decimation rate
    tsamp = stride*4*(blockIdx.x*blockDim.x) + 4*threadIdx.x;
    // mix samples with nco, perform first reduction
    assert(tsamp <= (4 * blockDim.x * gridDim.x));

    // TODO: get from cuda_driver.py!
    uint32_t decimationRate_rf2if = 32;  

    uint32_t idxSample_filter = 4 * (iFilterSampleTimes2  + iChannel * nFilterSamplesDivBy2);   // 2 samples/thread (unrolled) * 2 components/ sample (I /Q) = 4
    uint32_t nSamples_rf = decimationRate_rf2if * (nSamplesIF -1) + nFilterSamplesDivBy2 * 2; // nSamples_in=decimationRate*(nSamples_out-1)+nSamples_filter
    uint32_t iSample_rf = iSampleIF * decimationRate_rf2if + iFilterSampleTimes2 * 2;         // number of (complex) sample in rf signal
    uint32_t idxSample_rf = iSample_rf * 2 + iChannel * nSamples_rf *2 + iAntenna * nChannels * nSamples_rf *2; // index in memory (account for  I/Q, iChannel, iAntenna)

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

        // mgu: only makes sense for complex filters, for real phi_rem is always = 0
        /*Phase remainder exists because the NCO oscillator 
        may not complete an exact 360% rotation in a filter window*/
        double phi_rem = blockIdx.x*fmod((1*blockDim.x) * phase_inc, 2*M_PI);
        
        // mgu: these two lines should be correct. TODO: test if we use complex filters 
        double phiOffset = fmod(phase_inc * iSample_rf, 2*M_PI);
        phi_rem = phiOffset;
       
        //if (blockIdx.x < 3) {
        //     printf("   => BlockIdx.x %d: phaseInc: %f  phi_rem: %f  phiOffset: %f  \n", blockIdx.x, phase_inc, phi_rem, phiOffset);
        //}
        double ltemp = (double) itemp[iThread_lin];
        itemp[iThread_lin] = itemp[iThread_lin] * cos(phi_rem) - qtemp[iThread_lin] * sin(phi_rem);
        qtemp[iThread_lin] = ltemp * sin(phi_rem) + qtemp[iThread_lin] * cos(phi_rem);

        // write outout
        uint32_t output_idx = iSampleIF *2 + iChannel * nSamplesIF *2 + iAntenna * nChannels * nSamplesIF *2; 
        odata[output_idx  ] = (float)  itemp[iThread_lin];
        odata[output_idx+1] = (float)  qtemp[iThread_lin];
     }
}

