#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <time.h>

#include <cuda.h>
#include <cuda_runtime.h>
#include <curand.h>
#include <curand_kernel.h>
#include <cuda_texture_types.h>

 #if defined(__CUDACC__) && defined(__CUDA_ARCH__)
 #define _CPU_AND_GPU_CODE_ __device__	// for CUDA device code
 #else
 #define _CPU_AND_GPU_CODE_ 
 #endif

#ifndef M_SQRT1_2
#define M_SQRT1_2 0.707106781186547524401
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.5707963267948966192E0
#endif

// ITMLib for Math.
#include <ITMMath.h>

// #include <device_launch_parameters.h>
// #include <device_functions.h>

// define
#define CV_CUDA_BLOCK_SIZE_X 16		//16
#define CV_CUDA_BLOCK_SIZE_Y 16		//16
#define CV_CUDA_BLOCK_SIZE_Z 1		//16

#define CV_CUDA_BLOCK_SIZE 16		//16
#define CV_CUDA_MAX_BLOCK_SIZE  256 // 128		//256

#define CV_CUDA_TSDF_INVALID 5

#define GK_LEVEL_OF_IMAGE_PYRAMID 3

#define GK_PI 3.1415926535897932384626434

#define ROUNDF(x) (x<0.0f) ? (int)(x-0.5f) : (int)(x+0.5f)

///////////////////////////////////////////////////////////////////////////////
// Common structures
///////////////////////////////////////////////////////////////////////////////
// struct Vector2i{	int x, y;	};
// struct Vector2i{	int x, y;	};

///////////////////////////////////////////////////////////////////////////////
// Common constants
///////////////////////////////////////////////////////////////////////////////
const int StrideAlignment = 32;

///////////////////////////////////////////////////////////////////////////////
// Common functions
///////////////////////////////////////////////////////////////////////////////
// Align up n to the nearest multiple of m
inline int iAlignUp(int n,int m = StrideAlignment)
{
	int mod = n % m;

	if(mod)
		return n + m - mod;
	else
		return n;
}

// round up n/m
inline int iDivUp(int n,int m)
{
	return (n + m - 1) / m;
}

// swap two values
template<typename T>
inline void Swap(T &a,T &b)
{
	T t = a;
	a = b;
	b = t;
}

// Convert data array size if array width and stride are different.
template<typename T>
inline void convertHostArraySizeWidthToStride(
	T *in_array, 
	int in_width, int in_height, int in_stride,
	T *out_array)
{
	if(!out_array)	out_array = (T*)malloc( in_stride * in_height * sizeof(T));

	for(int i=0; i<in_height; i++){
		for(int j=0; j<in_width; j++){

			out_array[j + i * in_stride] = in_array[j + i * in_width];

		}
	}
	
}

// Convert data array size if array width and stride are different.
template<typename T>
inline void convertDeviceArraySizeStrideToWidth(
	T *in_array,
	int in_width,int in_height,int in_stride,
	T *out_array)
{
	if(!out_array)	cudaMalloc(&out_array, in_width * in_height * sizeof(T));

	for(int i=0; i<in_height; i++){
		for(int j=0; j<in_width; j++){

			out_array[j + i * in_width] = in_array[j + i * in_stride];

		}
	}

}

