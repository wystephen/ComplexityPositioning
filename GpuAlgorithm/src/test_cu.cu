#include <iostream>
#include <stdio.h>
#include <cuda_runtime.h>

#include "../include/clion_cuda.h"

__device__ float add(float *array){
	const int k = blockIdx.x * blockDim.x + threadIdx.x;

	array[k] = k;
}


int main(){

	int blocks = 1024;
	int thread_pre_blocks = 30;
	float *array_host = new float[blocks*thread_pre_blocks];
	float *array_device;
	cudaMalloc((void**)&array_device,blocks*thread_pre_blocks);
	add<<<blocks,thread_pre_blocks>>>(array_device);
	cudaMemcpy(array_host,array_device,cudaMemcpyDeviceToHost);
	for(int i(0);i<blocks*thread_pre_blocks;++i){
		std::cout << array_host[i] << std::endl;
	}
	cudaFree(array_device);

}
