/*
 * Copyright 1993-2015 NVIDIA Corporation.  All rights reserved.
 *
 * Please refer to the NVIDIA end user license agreement (EULA) associated
 * with this source code for terms and conditions that govern your use of
 * this software. Any use, reproduction, disclosure, or distribution of
 * this software and related documentation outside the terms of the EULA
 * is strictly prohibited.
 *
 */

/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_bilateral_kernel.cpp
/////////////////////////////////////////////////////////////////////////////////////////////
//#include "_yooji_2017_cuda_object_scanner.cuh"
//#define __CUDASCAN__
#include "../../_yooji_2017_cuda_object_scanner.cuh"

// for CUDA samples.
#include <helper_math.h>
#include <helper_functions.h>
#include <helper_cuda.h>       // CUDA device initialization helper functions

__constant__ float cGaussian[64];   //gaussian array in device side
//texture<ushort,2,cudaReadModeElementType> ushortTex;
texture<float,2,cudaReadModeElementType> floatTex;
 
// uint *dImg  = NULL;   //original image
// uint *dTemp   = NULL;   //temp array for iterations

//ushort *hMap = NULL;		// original image in host array.
//ushort *dMap  = NULL;   //original image in device array.
//ushort *dTemp  = NULL;   //temporal image in device array.

float *hMapF = NULL;		// original image in host array.
float *dMapF  = NULL;   //original image in device array.
float *dTempF  = NULL;   //temporal image in device array.
  
int dwidth = 1, dheight = 1;
size_t dpitch = 1, dpitchF = 1;

int iter_num = 1;
float gaussian_delta = 4;
float euclidean_delta = 0.1f;//0.1f;
int filter_radius = 2;

/*
    Perform a simple bilateral filter.

    Bilateral filter is a nonlinear filter that is a mixture of range
    filter and domain filter, the previous one preserves crisp edges and
    the latter one filters noise. The intensity value at each pixel in
    an image is replaced by a weighted average of intensity values from
    nearby pixels.

    The weight factor is calculated by the product of domain filter
    component(using the gaussian distribution as a spatial distance) as
    well as range filter component(Euclidean distance between center pixel
    and the current neighbor pixel). Because this process is nonlinear,
    the sample just uses a simple pixel by pixel step.

    Texture fetches automatically clamp to edge of image. 1D gaussian array
    is mapped to a 1D texture instead of using shared memory, which may
    cause severe bank conflict.

    Threads are y-pass(column-pass), because the output is coalesced.

    Parameters
    od - pointer to output data in global memory
    d_f - pointer to the 1D gaussian array
    e_d - euclidean delta
    w  - image width
    h  - image height
    r  - filter radius
*/

// ////////////////////////////////////////////////////////////////////////////////
// Local functions .///////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////

// ===============================================================================
// Device functions.
// ===============================================================================

//Euclidean Distance (x, y, d) = exp(-(|x - y| / d)^2 / 2)
 __device__ float euclideanDist(float a,float b,float d)
 {

	 float mod = (b - a) * (b - a);

	 return __expf(-mod / (2.f * d * d));
 }

 //Euclidean Similarity (x, y, d) = exp(-(|x - y| / d)^2 / 2)
 __device__ float euclideanSim(float a,float b,float d)
 {

	 float mod = (b - a) * (b - a);

	 return __expf(mod / (2.f * d * d));
 }
 
__global__ void
d_bilateral_filter(float *od,int w,int h,
				   float e_d,int r)
{
	int x = blockIdx.x*blockDim.x + threadIdx.x;
	int y = blockIdx.y*blockDim.y + threadIdx.y;

	if(x >= w || y >= h)
	{
		return;
	}

	float sum = 0.0f;
	float factor;
	float t =0.f;

	float center = tex2D(floatTex,x,y);

 	for(int i = -r; i <= r; i++)
 	{
 		for(int j = -r; j <= r; j++)
 		{
 			//ushort curPix = tex2D(ushortTex,x + j,y + i);
 			float curPix = tex2D(floatTex,x + j,y + i);
			// remove boundary pixels.
			if(curPix <= 0.0f){ od[y * w + x] = 0.0f; return ; }
 			//Euclidean Distance (x, y, d) = exp(-(|x - y| / d)^2 / 2)
 			factor = cGaussian[i + r] * cGaussian[j + r] *     //domain factor
 					 euclideanDist(curPix,center,e_d);             //range factor
 
 			t += factor * curPix;
 			sum += factor;
 		}
 	}

	// convert depth scale from mm to m. (x0.001) 
	//od[y * w + x] = 0.001f*t/sum;
	od[y * w + x] = t/sum;
}

__global__ void
d_inverse_bilateral_filter(float *od,int w,int h,
				   float e_d,int r)
{
	int x = blockIdx.x*blockDim.x + threadIdx.x;
	int y = blockIdx.y*blockDim.y + threadIdx.y;

	if(x >= w || y >= h)
	{
		return;
	}

	float sum = 0.0f;
	float factor;
	float t =0.f;

	float center = tex2D(floatTex,x,y);

	for(int i = -r; i <= r; i++)
	{
		for(int j = -r; j <= r; j++)
		{
			//ushort curPix = tex2D(ushortTex,x + j,y + i);
			float curPix = tex2D(floatTex,x + j,y + i);
			// remove boundary pixels.
			if(curPix <= 0.0f){ od[y * w + x] = 0.0f; return ; }
			//Euclidean Distance (x, y, d) = exp(-(|x - y| / d)^2 / 2)
			factor = cGaussian[i + r] * cGaussian[j + r] *     //domain factor
					 euclideanSim(curPix,center,e_d);             //range factor

			t += factor * curPix;
			sum += factor;
		}
	}

	// convert depth scale from mm to m. (x0.001) 
	//od[y * w + x] = 0.001f*t/sum;
	od[y * w + x] = t/sum;
}

 /*
    Perform 2D bilateral filter on image using CUDA

    Parameters:
    d_dest - pointer to destination image in device memory
    width  - image width
    height - image height
    e_d    - euclidean delta
    radius - filter radius
    iterations - number of iterations
*/

// ===============================================================================
// Host functions.
// ===============================================================================
 __host__ void initTexture(int width,int height)
 {
// 	 if(dMap) checkCudaErrors(cudaFree(dMap));
// 	 if(dTemp) checkCudaErrors(cudaFree(dTemp));
// 	 if(hMap) delete[] hMap; 

	 if(dMapF) checkCudaErrors(cudaFree(dMapF));
	 if(dTempF) checkCudaErrors(cudaFree(dTempF));
	 if(hMapF) delete[] hMapF;

	 // texture setting.
// 	 ushortTex.addressMode[0] = cudaAddressModeMirror;
// 	 ushortTex.addressMode[1] = cudaAddressModeMirror;
// 	 ushortTex.filterMode = cudaFilterModeLinear;
// 	 ushortTex.normalized = false; // don't access with normalized texture coords

	 floatTex.addressMode[0] = cudaAddressModeMirror;
	 floatTex.addressMode[1] = cudaAddressModeMirror;
	 floatTex.filterMode = cudaFilterModeLinear;
	 floatTex.normalized = false; // don't access with normalized texture coords

	 int stride = iAlignUp(width, CV_CUDA_BLOCK_SIZE_X);

	 dwidth = width; dheight = height;
	 //pitchF = stride*sizeof(float);

	 // copy image data to array
// 	 hMap = new ushort[width*height];
// 	 checkCudaErrors(cudaMallocPitch((void **)&dMap,&dpitch,sizeof(ushort)*width,height));
// 	 checkCudaErrors(cudaMallocPitch((void **)&dTemp,&dpitch,sizeof(ushort)*width,height));

	 hMapF = new float[width*height];
	 checkCudaErrors(cudaMallocPitch((void **)&dMapF,&dpitchF,sizeof(float)*width,height));
	 checkCudaErrors(cudaMallocPitch((void **)&dTempF,&dpitchF,sizeof(float)*width,height));
 }

//  __host__ bool importTexture(int width,int height,const float *hMap)
//  {
// 	 if(width != dwidth || height != dheight) return false;
// 
// 	 checkCudaErrors(cudaMemcpy2D(dMap,pitchF,hMap,sizeof(float)*width,
// 								  sizeof(float)*width,height,cudaMemcpyHostToDevice));
// 	 return true;
//  }

 __host__ void freeTextures()
 {

// 	 if(dMap) checkCudaErrors(cudaFree(dMap));		dMap = NULL;
// 	 if(dTemp) checkCudaErrors(cudaFree(dTemp));	dTemp = NULL;
// 	 if(hMap) delete[] hMap;						hMap = NULL;

	 if(dMapF){ 
		 checkCudaErrors(cudaFree(dMapF));	
		 cudaUnbindTexture(floatTex);
	 }
	 dMapF = NULL;
	 if(dTempF) checkCudaErrors(cudaFree(dTempF));  dTempF = NULL;
	 if(hMapF) delete[] hMapF;						hMapF = NULL;

	 dwidth = dheight = dpitch = 1;
 }

 /*
    Because a 2D gaussian mask is symmetry in row and column,
    here only generate a 1D mask, and use the product by row
    and column index later.

    1D gaussian distribution :
        g(x, d) -- C * exp(-x^2/d^2), C is a constant amplifier

    parameters:
    og - output gaussian array in global memory
    delta - the 2nd parameter 'd' in the above function
    radius - half of the filter size
             (total filter size = 2 * radius + 1)
*/
__host__ void updateGaussian(float delta, int radius)
{
    float  fGaussian[64];

    for (int i = 0; i < 2*radius + 1; ++i)
    {
        float x = i-radius;
        fGaussian[i] = expf(-(x*x) / (2*delta*delta));
    }

    checkCudaErrors(cudaMemcpyToSymbol(cGaussian, fGaussian, sizeof(float)*(2*radius+1)));
}

// for depth map.
__host__ bool bilateralFilterDepth(float *dDest,	
	const float *hMapDepth,
	int width,int height,
	float e_d,int radius,int iterations)
{
	// Bind the array to the texture
	if(width != dwidth || height != dheight) return false;

	int ww = width;
	int hh = height;

	//for(int i=0; i<ww*hh; i++) hMapF[i] = hMapDepth[i];
	// + copy depth map from host to device memory.
	cudaMemcpy2D(dMapF,dpitchF,hMapDepth,sizeof(float)*ww,sizeof(float)*ww,hh,cudaMemcpyHostToDevice);
	// + bind texture memory.
	cudaChannelFormatDesc desc = cudaCreateChannelDesc<float>();
	checkCudaErrors(cudaBindTexture2D(0,floatTex,dMapF,desc,ww,hh,sizeof(float)*ww));
 
 	// Perform bilateral filtering.
 	 dim3 blockSize(CV_CUDA_BLOCK_SIZE_X,CV_CUDA_BLOCK_SIZE_Y);
 	 dim3 gridSize(iAlignUp(ww,blockSize.x),iAlignUp(hh,blockSize.y));
 
 	 d_bilateral_filter<<< gridSize,blockSize>>>(
 		 dDest,ww,hh,e_d,radius);

	//checkCudaErrors(cudaUnbindTexture(floatTex));

	return true;
}

// for depth map.
__host__ bool inverseBilateralFilter(float *dDest,
	const float *hMapDepth,
	int width,int height,
	float e_d,int radius,int iterations)
{
	// Bind the array to the texture
	if(width != dwidth || height != dheight) return false;

	int ww = width;
	int hh = height;

	//for(int i=0; i<ww*hh; i++) hMapF[i] = hMapDepth[i];
	// + copy depth map from host to device memory.
	cudaMemcpy2D(dMapF,dpitchF,hMapDepth,sizeof(float)*ww,sizeof(float)*ww,hh,cudaMemcpyHostToDevice);
	// + bind texture memory.
	cudaChannelFormatDesc desc = cudaCreateChannelDesc<float>();
	checkCudaErrors(cudaBindTexture2D(0,floatTex,dMapF,desc,ww,hh,sizeof(float)*ww));

	// Perform bilateral filtering.
	dim3 blockSize(CV_CUDA_BLOCK_SIZE_X,CV_CUDA_BLOCK_SIZE_Y);
	dim3 gridSize(iAlignUp(ww,blockSize.x),iAlignUp(hh,blockSize.y));

	d_inverse_bilateral_filter<<< gridSize,blockSize>>>(
		dDest,ww,hh,e_d,radius);

	//checkCudaErrors(cudaUnbindTexture(floatTex));

	return true;
}




/////////////////////////////////////////////////////////////////////////////////////////////
// LGKvImageProcessor
/////////////////////////////////////////////////////////////////////////////////////////////
// *******************************************************
LGKvImageProcessor::LGKvImageProcessor()
// *******************************************************
{
	z_flag_init = false;
}

// *******************************************************
LGKvImageProcessor::~LGKvImageProcessor()
// *******************************************************
{
	freeTextures();
}

// *******************************************************
bool LGKvImageProcessor::i_Initialize(int ww, int hh)
// *******************************************************
{
	if(ww<=0 || hh<=0) return false;

	if(z_flag_init) freeTextures();
	initTexture(ww, hh);
	updateGaussian(gaussian_delta, filter_radius);

	z_flag_init = true;

	return true;
}

// *******************************************************
void LGKvImageProcessor::r_Release()
// *******************************************************
{
	freeTextures();
	z_flag_init = false;
}

// *******************************************************
bool LGKvImageProcessor::bfd_Bilateral_Filter_Depth(
	float *map_depth_filtered_dev,
	const float *map_depth_host,
	int ww,int hh)
// *******************************************************
{
	if(ww<=0 || hh<=0) return false;

	if(!z_flag_init) i_Initialize(ww, hh);

//  	cudaMemcpy2D(map_depth_filtered_dev, sizeof(float)*ww, map_depth_host, 
//  		sizeof(float)*ww, sizeof(float)*ww, hh, cudaMemcpyHostToDevice);

	bilateralFilterDepth(
		map_depth_filtered_dev,
		map_depth_host,
		ww,hh,
		euclidean_delta,
		filter_radius,
		iter_num);


	return true;
}

// *******************************************************
bool LGKvImageProcessor::bfd_Bilateral_Filter_Depth_Host(
	float *map_depth_filtered_host,
	const float *map_depth_host,
	int ww,int hh)
// *******************************************************
{
	if(ww<=0 || hh<=0) return false;

	if(!z_flag_init) i_Initialize(ww,hh);

	bilateralFilterDepth(
		dTempF,
		map_depth_host,
		ww,hh,
		euclidean_delta,
		filter_radius,
		iter_num);

	cudaMemcpy2D(map_depth_filtered_host,sizeof(float)*ww,dTempF,
		sizeof(float)*ww,sizeof(float)*ww,hh,cudaMemcpyDeviceToHost);

	return true;
}

// *******************************************************
bool LGKvImageProcessor::ibfd_Inverse_Bilateral_Filter_Depth_Host(
	float *map_depth_filtered_host,
	const float *map_depth_host,
	int ww,int hh)
// *******************************************************
{
	if(ww<=0 || hh<=0) return false;

	if(!z_flag_init) i_Initialize(ww,hh);

	inverseBilateralFilter(
		dTempF,
		map_depth_host,
		ww,hh,
		euclidean_delta,
		filter_radius,
		iter_num);

	cudaMemcpy2D(map_depth_filtered_host,sizeof(float)*ww,dTempF,
		sizeof(float)*ww,sizeof(float)*ww,hh,cudaMemcpyDeviceToHost);

	return true;
}