#pragma once 

#include "Util/_yooji_math_inlines.h"
#include "Util/_yooji_cuda_defines.h"
#include "Util/_yooji_cuda_math.h"

#include "Object/GPU/_yooji_cuda_rgbd_frame.h"
#include "Object/GPU/_yooji_cuda_tracking_state.h"
#include "Object/GPU/_yooji_cuda_object_cube.h"

#include "Engine/GPU/_yooji_cuda_img_proc.h"
#include "Engine/GPU/_yooji_cuda_pose_tracker.h"
#include "Engine/GPU/_yooji_cuda_tsdf_renderer.h"
#include "Engine/GPU/_yooji_cuda_volume_integrator.h"

///////////////////////////////////////////////////////////////////////////////
/// Constant memory variables.
///////////////////////////////////////////////////////////////////////////////
// If you need to share the same variable across multiple files 
// then you need to use extern.
// 각 cu 파일에 따로 선언을 해주지 않는 이상 제대로 동작하지 않음.
// extern으로 헤더에 선언만 해주는 건 소용이 없다!!!!!
// #ifndef __CUDASCAN__
// extern __constant__ float K_dev[4];
// extern __constant__ float T_gc_dev_c[16];
// extern __constant__ float T_cg_dev_c[16];
// extern __constant__ int dim_map_dev[2];
// 
// extern __constant__ float origin_dev[3];
// extern __constant__ int dim_cube_dev[3];
// extern __constant__ int dim_sc_dev[1];
// 
// extern __constant__ float mu_dev[1];
// extern __constant__ float r_cube_dev[1];
// extern __constant__ float sz_vox_inv_dev[1];
// extern __constant__ float max_w_dev[1];
// #endif

/////////////////////////////////////////////////////////////////////////////////
///// Basic device inline functions.
/////////////////////////////////////////////////////////////////////////////////
////********************************************************************************************
//_CPU_AND_GPU_CODE_ inline void d_p_Project(Vector3f in_p3d, const float *K, Vector2f &out_p2d)
////********************************************************************************************
// {
// 	out_p2d.x = K[0]*in_p3d.x/in_p3d.z+ K[2];
// 	out_p2d.y = K[1]*in_p3d.y/in_p3d.z+ K[3];
// }
////********************************************************************************************
//_CPU_AND_GPU_CODE_ inline void d_bp_Back_Project(Vector2f in_p2d, const float *K, float in_d, Vector3f &out_p3d)
////********************************************************************************************
//{
//	out_p3d.x = in_d*(in_p2d.x-K[2])/K[0];
//	out_p3d.y = in_d*(in_p2d.y-K[3])/K[1];
//	out_p3d.z = in_d;
//}
//
////********************************************************************************************
//_CPU_AND_GPU_CODE_ inline void d_t_Transform(Vector3f in_p3d, const float *T, Vector3f &out_p3d)
////********************************************************************************************
//{
//	out_p3d.x = T[0]*in_p3d.x	+T[1]*in_p3d.y	+T[2]*in_p3d.z	+T[3];
//	out_p3d.y = T[4]*in_p3d.x	+T[5]*in_p3d.y	+T[6]*in_p3d.z	+T[7];
//	out_p3d.z = T[8]*in_p3d.x	+T[9]*in_p3d.y	+T[10]*in_p3d.z	+T[11];
//}
//
////********************************************************************************************
//_CPU_AND_GPU_CODE_ inline void d_gcc_Get_Camera_Center(const float *T, Vector3f &out_center)
////********************************************************************************************
//{
//	// pose = | R t |
//	//		  | 0 1 |
//	// camera center in global coordinates,
//	// C=-R^-1*t from T=-RC.
//	const float *p_mat= T;
//	float tx, ty, tz;
//
//	tx=p_mat[4*0+3];	ty=p_mat[4*1+3];	tz=p_mat[4*2+3];
//	out_center.x=-p_mat[4*0+0]*tx -p_mat[4*1+0]*ty -p_mat[4*2+0]*tz;
//	out_center.y=-p_mat[4*0+1]*tx -p_mat[4*1+1]*ty -p_mat[4*2+1]*tz;
//	out_center.z=-p_mat[4*0+2]*tx -p_mat[4*1+2]*ty -p_mat[4*2+2]*tz;	
//
//}
//
////********************************************************************************************
//_CPU_AND_GPU_CODE_ inline bool d_gpv_Get_Position_in_Voxel(Vector3f in_p3d, 
//	const float *origin, const int *dim_cube, const float sz_vox_inv,
//	Vector3f &out_vox)
////********************************************************************************************
//{
//	bool valid = true;
//
//	// assume that position of origin point in the world is (-0.5, -0.5, -0.5) in voxel coordinates..
//	out_vox.x = (in_p3d.x - origin[0])*sz_vox_inv - 0.5f;
//	out_vox.y = (in_p3d.y - origin[1])*sz_vox_inv - 0.5f;
//	out_vox.z = (in_p3d.z - origin[2])*sz_vox_inv - 0.5f;
//
//	if(out_vox.x < -0.5f || out_vox.x >= (float)dim_cube[0] - 0.5f ||
//		out_vox.y < -0.5f || out_vox.y >= (float)dim_cube[1] - 0.5f ||
//		out_vox.z < -0.5f || out_vox.z >= (float)dim_cube[2] - 0.5f)
//		valid = false;
//
//	return valid;
//}
////********************************************************************************************
//_CPU_AND_GPU_CODE_ inline bool d_gpw_Get_Position_in_World(Vector3f in_vox, 
//	const float *origin, const int *dim_cube, const float sz_vox_inv,
//	Vector3f &out_p3d)
////********************************************************************************************
//{
//	bool valid = true;
//
//	if(in_vox.x < -0.5f || in_vox.x >= (float)dim_cube[0] - 0.5f ||
//		in_vox.y < -0.5f || in_vox.y >= (float)dim_cube[1] - 0.5f ||
//		in_vox.z < -0.5f || in_vox.z >= (float)dim_cube[2] - 0.5f)
//		valid = false;
//
//	out_p3d = (in_vox + 0.5f)/sz_vox_inv + origin;
//
//	return valid;
//}
//
////********************************************************************************************
//_CPU_AND_GPU_CODE_ inline bool d_gid_Get_Interpolated_Depth(Vector2f in_p2d, int in_ww, int in_hh,
//	const float *in_map_depth, float &out_depth)
////********************************************************************************************
//{
//	// d1   d2
//	//    x
//	// d3   d4
//	float d1, d2, d3, d4;
//	float xf, yf;
//	int x, y;
//	float resi_x, resi_y;
//
//	xf = in_p2d.x; yf = in_p2d.y;
//	if(xf<0.0f || xf>float(in_ww-1) ||
//	   yf<0.0f || yf>float(in_hh-1))	return false;
//
//	d2=d3=d4=0.0f;
//
//	x=(int)xf;	y=(int)yf;	resi_x=xf-(float)x;	resi_y=yf-(float)y;
//	d1=in_map_depth[y*in_ww+x];			if(d1<=0.0f)	return false;
//	if(resi_x>0.0f){					d2=in_map_depth[y*in_ww+x+1];		if(d2<=0.0f)	return false;	}
//	if(resi_y>0.0f){					d3=in_map_depth[(y+1)*in_ww+x];		if(d3<=0.0f)	return false;	}
//	if(resi_x>0.0f && resi_y>0.0f){		d4=in_map_depth[(y+1)*in_ww+x+1];	if(d4<=0.0f)	return false;	}
//
//	out_depth=(1.0f-resi_y)*( (1.0f-resi_x)*d1+resi_x*d2 ) + resi_y*( (1.0f-resi_x)*d3+resi_x*d4 );	
//
//	return true;
//}
//
//
////********************************************************************************************
//_CPU_AND_GPU_CODE_ inline bool d_gin_Get_Interpolated_Normal(Vector2f in_p2d, int in_ww, int in_hh,
//	const float *in_normal_map, Vector3f &out_norm)
////********************************************************************************************
//{
//	// n1   n2
//	//    x
//	// n3   n4	
//	Vector3f n1, n2, n3, n4;
//	const float *p_norm = in_normal_map;
//	int x, y, tidx;	float xf, yf;
//	float res_x, res_y, ires_x, ires_y;
//
//	xf = in_p2d.x; yf = in_p2d.y;
//	if(xf<0.0f || xf>float(in_ww-1) ||
//	   yf<0.0f || yf>float(in_hh-1))	return false;
//
//	x = (int)xf;	y = (int)yf;	res_x = xf-(float)x;	res_y = yf-(float)y;
//
//	//zz_n2.s_Set(0.0f, 0.0f, 0.0f);	zz_n3.s_Set(0.0f, 0.0f, 0.0f);	zz_n4.s_Set(0.0f, 0.0f, 0.0f);
//
//	// default value of invalid normal is (-100.0f, -100.0f, -100.0f). 
//	// + n1.
//	tidx = 3*(y*in_ww + x);	if((n1.x=p_norm[tidx++])==-100.0f)	return false; 
//	n1.y=p_norm[tidx++]; n1.z=p_norm[tidx];
//	// + n2.
//	if(res_x>0.0f){	
//		tidx = 3*(y*in_ww + x+1); 
//		n2.x=p_norm[tidx++]; n2.y=p_norm[tidx++]; n2.z=p_norm[tidx];
//	}
//	// + n3.
//	if(res_y>0.0f){			
//		tidx = 3*((y+1)*in_ww + x);
//		n3.x = p_norm[tidx++]; n3.y = p_norm[tidx++]; n3.z = p_norm[tidx];
//	}
//	// + n4.
//	if(res_x>0.0f && res_y>0.0f){	
//		tidx = 3*((y+1)*in_ww + x+1);
//		n4.x = p_norm[tidx++]; n4.y = p_norm[tidx++]; n4.z = p_norm[tidx];
//	}
//
//	ires_x = 1.0f - res_x;	ires_y = 1.0f - res_y;
//	out_norm = (ires_y*(ires_x*n1 + res_x*n2) + res_y*(ires_x*n3 + res_x*n4)).normalised();
//
//	return true;
//}
//
////********************************************************************************************
//// Every symmetric, positive definite matrix A can be decomposed into 
//// a product of a unique lower triangular matrix L and its transpose
//// Cholesky decomposition.
//// Ljj = sqrt( Ajj - sum(Ljk^2)(k=0:j-1) )
//// Lij = ( Aij - sum(Lik*Ljk)(k=0:j-1) )/Ljj
//_CPU_AND_GPU_CODE_ inline bool d_lld_LL_Decomposition(const float *A, int dim, float *L)
////********************************************************************************************
//{
//	float tsum;
//	
//	for(int i=0; i<dim; i++){
//		for(int j=0; j<dim; j++){
//
//			// for elements above the diagonal.
//			if(i<j) continue;
//			// for diagonal elements.
//			// Ljj = sqrt( Ajj - sum(Ljk^2)(k=0:j-1) )
//			else if(i==j){			
//				tsum = 0.0f;
//				for(int k=0; k<j; k++) tsum += SQUARE(L[j*dim + k]);
//				L[j*dim + j] = sqrtf(A[j*dim + j] - tsum);
//			}
//			// for elements below the diagonal.
//			// Lij = ( Aij - sum(Lik*Ljk)(k=0:j-1) )/Ljj
//			else{
//				tsum = 0.0f;
//				for(int k=0; k<j; k++) tsum += L[i*dim + k]*L[j*dim + k];
//				L[i*dim + j] = (A[i*dim + j] - tsum)/L[j*dim + j];
//			}
//		}
//	}
//
//	return true;
//}
//
////********************************************************************************************
//// Ax = b -> LL^Tx = b -> Ly = b -> compute y -> L^Tx = y -> compute x.
//_CPU_AND_GPU_CODE_ inline bool d_sls_Solve_Linear_System_using_LLD(const float *L, const float *b, 
//	float *y, int dim, float *sol_x)
////********************************************************************************************
//{
//	float tsum;
//
//	for(int i=0; i<dim; i++) if(L[i*dim + i] == 0.0f) return false;
//
//	// compute y in ascending order.
//	// y(i) = [b(i) - (L(i,1)*y(1) + ... + L(i,i-1)*y(i-1))]/L(i,i)
//	for(int i=0; i<dim; i++){
//		tsum = 0.0f;
//		for(int k=0; k<i; k++) tsum += L[i*dim + k]*y[k];
//		y[i] = (b[i] - tsum)/L[i*dim + i];
//	}
//	// compute x in descending order.
//	// x(i) = [y(i) - (L(i+1,i)*x(i+1) + ... L(N,i)*x(N))]/L(i,i)	
//	for(int i = dim-1; i>=0; i--){
//		tsum = 0.0f;
//		for(int k = i+1; k<dim; k++) tsum += L[k*dim + i]*sol_x[k];
//		sol_x[i] = (y[i] - tsum)/L[i*dim + i];
//	}
//// 	// x(N-i) = [y(N-i) - (L(N-i+1,N-i)*x(N-i+1) + ... L(N,N-i)*x(N))]/L(N-i,N-i)
//// 	for(int i=dim-1; i>=0; i--){
//// 		tsum = 0.0f;
//// 		for(int k=dim-1; k>i; k--) tsum += L[k*dim + i]*sol_x[k];
//// 		sol_x[i] = (y[i] - tsum)/L[i*dim + i];
//// 	}
//
//	return true;
//}