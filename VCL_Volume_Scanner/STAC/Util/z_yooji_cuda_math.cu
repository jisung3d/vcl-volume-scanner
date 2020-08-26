#include "../_yooji_2017_cuda_object_scanner.cuh"

// /////////////////////////////////////////////////////////////////////////////////////////////
// Set 1D array.
// /////////////////////////////////////////////////////////////////////////////////////////////
__global__ void set1D(float* in_vec, int in_dim, float in_value)
 {
 	int tidx = blockIdx.x*blockDim.x+threadIdx.x;
 	if(tidx < in_dim)	in_vec[tidx] = in_value;
 }
 __global__ void set1D(int* in_vec, int in_dim, int in_value)
 {
	 int tidx = blockIdx.x*blockDim.x+threadIdx.x;
	 if(tidx < in_dim)	in_vec[tidx] = in_value;
 }
 __global__ void set1D(short* in_vec, int in_dim, short in_value)
 {
	 int tidx = blockIdx.x*blockDim.x+threadIdx.x;
	 if(tidx < in_dim)	in_vec[tidx] = in_value;
 }
 __global__ void set1D(uchar* in_vec, int in_dim, uchar in_value)
 {
	 int tidx = blockIdx.x*blockDim.x+threadIdx.x;
	 if(tidx < in_dim)	in_vec[tidx] = in_value;
 }
 __global__ void set1D(bool* in_vec, int in_dim, bool in_value)
 {
	 int tidx = blockIdx.x*blockDim.x+threadIdx.x;
	 if(tidx < in_dim)	in_vec[tidx] = in_value;
 }
 
 // /////////////////////////////////////////////////////////////////////////////////////////////
 __host__ void setDeviceMem1D(float* in_vec, int in_dim, float in_value)
 {
  	int block_sz, grid_sz;
  	block_sz = 128;		grid_sz = in_dim/block_sz;	
 	set1D<<<grid_sz, block_sz>>>(in_vec, in_dim, in_value);
 }
 __host__ void setDeviceMem1D(int* in_vec, int in_dim, int in_value)
 {
	 int block_sz, grid_sz;
	 block_sz = 128;		grid_sz = in_dim/block_sz;
	 set1D<<<grid_sz, block_sz>>>(in_vec, in_dim, in_value);
 }
 __host__ void setDeviceMem1D(short* in_vec, int in_dim, short in_value)
 {
	 int block_sz, grid_sz;
	 block_sz = 128;		grid_sz = in_dim/block_sz;
	 set1D<<<grid_sz, block_sz>>>(in_vec, in_dim, in_value);
 }
 __host__ void setDeviceMem1D(uchar* in_vec, int in_dim, uchar in_value)
 {
	 int block_sz, grid_sz;
	 block_sz = 128;		grid_sz = in_dim/block_sz;
	 set1D<<<grid_sz, block_sz>>>(in_vec, in_dim, in_value);
 }
 __host__ void setDeviceMem1D(bool* in_vec, int in_dim, bool in_value)
 {
	 int block_sz, grid_sz;
	 block_sz = 128;		grid_sz = in_dim/block_sz;
	 set1D<<<grid_sz, block_sz>>>(in_vec, in_dim, in_value);
 }
 //////////////////////////////////////////////////////////////////////////

 // /////////////////////////////////////////////////////////////////////////////////////////////
 // Set 2D array.
 // /////////////////////////////////////////////////////////////////////////////////////////////
 __global__ void set2D(float* in_vec, int2 in_dim,float in_value)
 {
	 int tx = blockIdx.x*blockDim.x+threadIdx.x;
	 int ty = blockIdx.y*blockDim.y+threadIdx.y;

	 if(tx < in_dim.x && ty < in_dim.y)	in_vec[ty*in_dim.x + tx] = in_value;
 }
 __global__ void set2D(int* in_vec,int2 in_dim,int in_value)
 {
	 int tx = blockIdx.x*blockDim.x+threadIdx.x;
	 int ty = blockIdx.y*blockDim.y+threadIdx.y;

	 if(tx < in_dim.x && ty < in_dim.y)	in_vec[ty*in_dim.x + tx] = in_value;
 }
 __global__ void set2D(short* in_vec,int2 in_dim,short in_value)
 {
	 int tx = blockIdx.x*blockDim.x+threadIdx.x;
	 int ty = blockIdx.y*blockDim.y+threadIdx.y;

	 if(tx < in_dim.x && ty < in_dim.y)	in_vec[ty*in_dim.x + tx] = in_value;
 }
 __global__ void set2D(uchar* in_vec,int2 in_dim, uchar in_value)
 {
	 int tx = blockIdx.x*blockDim.x+threadIdx.x;
	 int ty = blockIdx.y*blockDim.y+threadIdx.y;

	 if(tx < in_dim.x && ty < in_dim.y)	in_vec[ty*in_dim.x + tx] = in_value;
 }
 __global__ void set2D(bool* in_vec,int2 in_dim,bool in_value)
 {
	 int tx = blockIdx.x*blockDim.x+threadIdx.x;
	 int ty = blockIdx.y*blockDim.y+threadIdx.y;

	 if(tx < in_dim.x && ty < in_dim.y)	in_vec[ty*in_dim.x + tx] = in_value;
 }

 // /////////////////////////////////////////////////////////////////////////////////////////////
 __host__ void setDeviceMem2D(float* in_vec,int2 in_dim,float in_value)
 {
	 dim3 threads(CV_CUDA_BLOCK_SIZE_X,CV_CUDA_BLOCK_SIZE_Y);
	 dim3 blocks(iDivUp(in_dim.x,threads.x),iDivUp(in_dim.y,threads.y));

	 set2D<<<blocks,threads>>>(in_vec,in_dim,in_value);
 }
 __host__ void setDeviceMem2D(int* in_vec,int2 in_dim,int in_value)
 {
	 dim3 threads(CV_CUDA_BLOCK_SIZE_X,CV_CUDA_BLOCK_SIZE_Y);
	 dim3 blocks(iDivUp(in_dim.x,threads.x),iDivUp(in_dim.y,threads.y));

	 set2D<<<blocks,threads>>>(in_vec,in_dim,in_value);
 }
 __host__ void setDeviceMem2D(short* in_vec,int2 in_dim,short in_value)
 {
	 dim3 threads(CV_CUDA_BLOCK_SIZE_X,CV_CUDA_BLOCK_SIZE_Y);
	 dim3 blocks(iDivUp(in_dim.x,threads.x),iDivUp(in_dim.y,threads.y));

	 set2D<<<blocks,threads>>>(in_vec,in_dim,in_value);
 }
 __host__ void setDeviceMem2D(uchar* in_vec,int2 in_dim,uchar in_value)
 {
	 dim3 threads(CV_CUDA_BLOCK_SIZE_X,CV_CUDA_BLOCK_SIZE_Y);
	 dim3 blocks(iDivUp(in_dim.x,threads.x),iDivUp(in_dim.y,threads.y));

	 set2D<<<blocks,threads>>>(in_vec,in_dim,in_value);
 }
 __host__ void setDeviceMem2D(bool* in_vec,int2 in_dim,bool in_value)
 {
	 dim3 threads(CV_CUDA_BLOCK_SIZE_X,CV_CUDA_BLOCK_SIZE_Y);
	 dim3 blocks(iDivUp(in_dim.x,threads.x),iDivUp(in_dim.y,threads.y));

	 set2D<<<blocks,threads>>>(in_vec,in_dim,in_value);
 }

 // /////////////////////////////////////////////////////////////////////////////////////////////
 // Set 3D array.
 // /////////////////////////////////////////////////////////////////////////////////////////////
 __global__ void set3D(float* in_vec,int3 in_dim,float in_value)
 {
	 int tx = blockIdx.x*blockDim.x+threadIdx.x;
	 int ty = blockIdx.y*blockDim.y+threadIdx.y;
	 int tz = blockIdx.z*blockDim.z+threadIdx.z;

	 if(tx < in_dim.x && ty < in_dim.y && tz < in_dim.z)	
		 in_vec[tz*in_dim.x*in_dim.y + ty*in_dim.x + tx] = in_value;
 }
 __global__ void set3D(int* in_vec,int3 in_dim,int in_value)
 {
	 int tx = blockIdx.x*blockDim.x+threadIdx.x;
	 int ty = blockIdx.y*blockDim.y+threadIdx.y;
	 int tz = blockIdx.z*blockDim.z+threadIdx.z;

	 if(tx < in_dim.x && ty < in_dim.y && tz < in_dim.z)
		 in_vec[tz*in_dim.x*in_dim.y + ty*in_dim.x + tx] = in_value;
 }
 __global__ void set3D(short* in_vec,int3 in_dim,short in_value)
 {
	 int tx = blockIdx.x*blockDim.x+threadIdx.x;
	 int ty = blockIdx.y*blockDim.y+threadIdx.y;
	 int tz = blockIdx.z*blockDim.z+threadIdx.z;

	 if(tx < in_dim.x && ty < in_dim.y && tz < in_dim.z)
		in_vec[tz*in_dim.x*in_dim.y + ty*in_dim.x + tx] = in_value;
 }
 __global__ void set3D(uchar* in_vec,int3 in_dim,uchar in_value)
 {
	 int tx = blockIdx.x*blockDim.x+threadIdx.x;
	 int ty = blockIdx.y*blockDim.y+threadIdx.y;
	 int tz = blockIdx.z*blockDim.z+threadIdx.z;

	 if(tx < in_dim.x && ty < in_dim.y && tz < in_dim.z)
		in_vec[tz*in_dim.x*in_dim.y + ty*in_dim.x + tx] = in_value;
 }
 __global__ void set3D(bool* in_vec,int3 in_dim,bool in_value)
 {
	 int tx = blockIdx.x*blockDim.x+threadIdx.x;
	 int ty = blockIdx.y*blockDim.y+threadIdx.y;
	 int tz = blockIdx.z*blockDim.z+threadIdx.z;

	 if(tx < in_dim.x && ty < in_dim.y && tz < in_dim.z)
		 in_vec[tz*in_dim.x*in_dim.y + ty*in_dim.x + tx] = in_value;
 }

 // /////////////////////////////////////////////////////////////////////////////////////////////
 __host__ void setDeviceMem3D(float* in_vec,int3 in_dim,float in_value)
 {
	 dim3 threads(CV_CUDA_BLOCK_SIZE_X,CV_CUDA_BLOCK_SIZE_Y,CV_CUDA_BLOCK_SIZE_Z);
	 dim3 blocks(iDivUp(in_dim.x,threads.x),iDivUp(in_dim.y,threads.y),iDivUp(in_dim.z,threads.z));

	 set3D<<<blocks,threads>>>(in_vec,in_dim,in_value);
 }
 __host__ void setDeviceMem3D(int* in_vec,int3 in_dim,int in_value)
 {
	 dim3 threads(CV_CUDA_BLOCK_SIZE_X,CV_CUDA_BLOCK_SIZE_Y,CV_CUDA_BLOCK_SIZE_Z);
	 dim3 blocks(iDivUp(in_dim.x,threads.x),iDivUp(in_dim.y,threads.y),iDivUp(in_dim.z,threads.z));

	 set3D<<<blocks,threads>>>(in_vec,in_dim,in_value);
 }
 __host__ void setDeviceMem3D(short* in_vec,int3 in_dim,short in_value)
 {
	 dim3 threads(CV_CUDA_BLOCK_SIZE_X,CV_CUDA_BLOCK_SIZE_Y,CV_CUDA_BLOCK_SIZE_Z);
	 dim3 blocks(iDivUp(in_dim.x,threads.x),iDivUp(in_dim.y,threads.y),iDivUp(in_dim.z,threads.z));

	 set3D<<<blocks,threads>>>(in_vec,in_dim,in_value);
 }
 __host__ void setDeviceMem3D(uchar* in_vec,int3 in_dim,uchar in_value)
 {
	 dim3 threads(CV_CUDA_BLOCK_SIZE_X,CV_CUDA_BLOCK_SIZE_Y,CV_CUDA_BLOCK_SIZE_Z);
	 dim3 blocks(iDivUp(in_dim.x,threads.x),iDivUp(in_dim.y,threads.y),iDivUp(in_dim.z,threads.z));

	 set3D<<<blocks,threads>>>(in_vec,in_dim,in_value);
 }
 __host__ void setDeviceMem3D(bool* in_vec,int3 in_dim,bool in_value)
 {
	 dim3 threads(CV_CUDA_BLOCK_SIZE_X,CV_CUDA_BLOCK_SIZE_Y,CV_CUDA_BLOCK_SIZE_Z);
	 dim3 blocks(iDivUp(in_dim.x,threads.x),iDivUp(in_dim.y,threads.y),iDivUp(in_dim.z,threads.z));

	 set3D<<<blocks,threads>>>(in_vec,in_dim,in_value);
 }