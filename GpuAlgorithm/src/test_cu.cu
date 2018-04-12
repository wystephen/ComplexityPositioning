
//#include "../include/clion_cuda.h"
#include <iostream>

#include <stdio.h>
#include <cuda_runtime.h>


__global__
void add_f(float *array) {
	int k = blockIdx.x * blockDim.x + threadIdx.x;
	array[k] = 10.0;
}


int main() {

	int blocks = 1024;
	int thread_pre_blocks = 30;
	float *array_host = new float[blocks * thread_pre_blocks];
	float *array_device;
	std::cout << "before" << std::endl;
	cudaMalloc((void**)&array_device, blocks * thread_pre_blocks);

	add_f<<<blocks,thread_pre_blocks>>>(array_device);
	
	cudaMemcpy(array_device, array_host, blocks * thread_pre_blocks, cudaMemcpyDeviceToHost);


	for (int i(0); i < blocks * thread_pre_blocks; ++i) {
		std::cout << array_host[i] << std::endl;
	}
	cudaFree(array_device);

}
