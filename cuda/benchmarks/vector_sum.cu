// Copyright (c) 2020, 2021, NECSTLab, Politecnico di Milano. All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NECSTLab nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//  * Neither the name of Politecnico di Milano nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "vector_sum.cuh"

//////////////////////////////
//////////////////////////////

__global__ void gpu_vector_sum_1(float *x, float *res_tmp, int N) {
    extern __shared__ float shared_data[];
    // each thread loads one element from global to shared mem (warning: no boundary checks!)
    unsigned int tid = threadIdx.x;
    unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
    shared_data[tid] = x[i];
    __syncthreads();
    // do reduction in shared mem
    for(unsigned int s = 1; s < blockDim.x; s *= 2) {
        if (tid % (2 * s) == 0) {
            shared_data[tid] += shared_data[tid + s];
        }
        __syncthreads();
    }
    // write result for this block to global mem
    if (tid == 0) res_tmp[blockIdx.x] = shared_data[0];
}


//////////////////////////////
//////////////////////////////

// Allocate data on the CPU and GPU;
void VectorSum::alloc() {
    // Compute the number of blocks for implementations where the value is a function of the input size;
    B = (N + block_size_1d - 1) / block_size_1d;

    // Allocate CPU data;
    x = (float*) malloc(sizeof(float) * N);
    res_tmp = (float*) malloc(sizeof(float) * B);
    // Allocate GPU data;
    err = cudaMalloc(&x_d, sizeof(float) * N);
    // The GPU output buffer has size equal to the number of blocks, 
    // as we aggregate partial sums on the CPU;
    err = cudaMalloc(&res_tmp_d, sizeof(float) * B);
}

// Initialize data;
void VectorSum::init() {
    // Just put some values into the array (sum_{i=1}^{N}{1/i**2} is pi^2 / 6);
    for (int i = 0; i < N; i++) {
        x[i] = float(1) / ((i + 1) * (i + 1));
    }
}

// Reset the state of the computation after every iteration.
// Reset the result, and transfer data to the GPU if necessary;
void VectorSum::reset() {
    // Reset the result;
    res = 0.0;
    // Transfer data to the GPU;
    cudaMemcpy(x_d, x, sizeof(float) * N, cudaMemcpyHostToDevice);
}

void VectorSum::vector_sum_1() {
    // Call the GPU computation (and set the size of shared memory!);
    gpu_vector_sum_1<<<B, block_size_1d, sizeof(float) * B>>>(x_d, res_tmp_d, N);
    // Copy the partial result from the GPU to the CPU;
    cudaMemcpy(res_tmp, res_tmp_d, sizeof(float) * B, cudaMemcpyDeviceToHost);
    // Sum the partial results using the CPU;
    for (int i = 0; i < B; i++) {
        res += res_tmp[i];
    }
}

void VectorSum::execute(int iter) {
    switch (implementation)
    {
    case 0:
        vector_sum_1();
        break;
    default:
        break;
    }
}

#define PI 3.14159265358979323846
void VectorSum::cpu_validation(int iter) {
    float cpu_result = PI * PI / 6;
    if (std::abs(res - cpu_result / 6) > 1e-4) std::cout << "result error! GPU=" << res << ", CPU=" << cpu_result << std::endl; 
}

std::string VectorSum::print_result(bool short_form) {
    return std::to_string(res);
}

void VectorSum::clean() {
    free(x);
    free(res_tmp);
    free(x_d);
    free(res_tmp_d);
}
