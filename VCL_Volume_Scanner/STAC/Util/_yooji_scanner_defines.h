//////////////////////////////////////////////////////////////////////
// _yooji_scanner_defines.h
//////////////////////////////////////////////////////////////////////

#pragma once

// defines
#pragma once

#define KV_COMPUTE_ON_CPU 0
#define KV_MODE_CAPTURE_ONLY 0

#define KV_MODE_DEPTH_ONLY 0
#define KV_MODE_DRIFT_DETECT 0

#define KV_FLAG_GROUND_TRUTH 0
#define KV_FLAG_GROUND_REMOVAL 0

#define KV_FLAG_MOTION_SMOOTH_TEMPORAL 0
#define KV_FLAG_SIFT_INITIAL 0
#define KV_FLAG_COLOR_GLOVE 0

#define KV_FLAG_FRAME_TO_MODEL_DEPTH 1

// pyramid level.
#define KV_LEVEL_OF_IMAGE_PYRAMID 3	// 4-level 이상으로 하면 젤 작은 영상에서 rendering이 제대로 안됨.

// for GA-DVO
#define KV_FLAG_RGB_COST 1
#define KV_FLAG_GRAD_COST 1
#define KV_FLAG_DEPTH_COST 1

#define KV_FLAG_RGBD_OLD 0

#define MAX_TMPL_NUM 10

// defines
#define KV_TYPE_DEPTH_FLOAT 0	
#define KV_TYPE_DEPTH_USHORT 1
// for real sequence
#define KV_CAPTURE_FROM_FILES 0
#define KV_CAPTURE_FROM_KINECTV1 1
#define KV_CAPTURE_FROM_KINECTV2 2
#define KV_CAPTURE_FROM_FILES_CXD 3

#define KV_CONVERT_IMAGE_DEPTH_FL_TO_UC 0
#define KV_CONVERT_IMAGE_DEPTH_FL_TO_DISP_UC 1

#define KV_FLAG_SUBCUBE_CARVED -1
#define KV_FLAG_SUBCUBE_INACTIVATED 0
#define KV_FLAG_SUBCUBE_ACTIVATED 1

#define KV_TYPE_TRACK_GROUND 0
#define KV_TYPE_TRACK_DEPTH 1		// Projective ICP.
#define KV_TYPE_TRACK_COLOR 2		// Photoconsistency maximization.
#define KV_TYPE_TRACK_EDGE_D 3		// Edge distance.

#define KV_TYPE_INPUT_OBJECT 0
#define KV_TYPE_INPUT_SCENE 1

#define CHANNEL_RGB_COLOR_FOR_GMM 3

// Response map.
#define KV_TYPE_RESPONSE_HARRIS 0
#define KV_TYPE_RESPONSE_SHITOMASI 1

//// for Real scene (in-hand object)
//#define KV_CUBE_DIM 256
//#define KV_CUBE_SIZE 0.2f
//#define KV_MU_TSDF 0.005f // m
//#define KV_MAX_W_TSDF 100
//#define KV_TH_DIST_SQ_ICP (0.01f*0.01f) // m^2

// image sizes.
// + primesense.
#define KV_X_DEPTH_PS 640
#define KV_Y_DEPTH_PS 480
#define KV_X_RGB_PS 640
#define KV_Y_RGB_PS 480

// cube sizes.
#define KV_CUBE_DIM 512
// for Bunny data (virtual object)
//#define KV_CUBE_SIZE 1.5f
//// for Real scene (in-hand object)
//#define KV_CUBE_SIZE 0.2f
// for Real scene (Face)
//#define KV_CUBE_SIZE 0.3f

// for optical flow.
#define KV_FLOW_BLOCK_SIZE 11

// for Real scene
#define KV_CUBE_SIZE 6.0f	// 0.2f 4.0f 
#define KV_TMPL_SZ_HALF 3

// parameters for ICP.
#define KV_MU_TSDF (7.0f*KV_CUBE_SIZE/(float)KV_CUBE_DIM) // m
#define KV_TH_DIST_SQ_ICP (4*KV_MU_TSDF*KV_MU_TSDF) // m^2
#define KV_MAX_W_TSDF 100

// parameters for mesh-to-docube.
#define MAX_DEPTH_DIFF_FOR_MESH_TRIANGLE_M_UNIT 0.01f
#define MAX_DEPTH_DIFF_FOR_MESH_TRIANGLE_MM_UNIT 10.0f

// size.
#define KV_MAX_NUM_INPUT_IMAGES 3000

// for MonoFusion scene.
//#define KV_CUBE_DIM 


//#define KV_CUBE_SIZE 0.6f
//#define KV_MU_TSDF (7.0f*KV_CUBE_SIZE/(float)KV_CUBE_DIM) // m
//#define KV_MAX_W_TSDF 100
//#define KV_TH_DIST_SQ_ICP (4*KV_MU_TSDF*KV_MU_TSDF) // m^2
//
#define KV_MIN_DEPTH 0.2f // m
#define KV_MAX_DEPTH 4.0f // m

// for camera tracking.
#ifndef M_SQRT1_2
#define M_SQRT1_2 0.707106781186547524401
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.5707963267948966192E0
#endif
//
//// macros
//#define SQUARE(x) (x)*(x)
//#define ROUNDF(x) (x<0.0f) ? (int)(x-0.5f) : (int)(x+0.5f)
//#define DIST_CIRCULAR(x) ( (x<=180.0f) ? x : 360.0f-x );
//
//// basic inline math functions for pointers.
//#include <stdio.h>
//// TO DO:
//// + for use these functions, you should check the dimension of input and output matrix pointers.
//template <typename T>
//inline void d_pm_Printf_Matrix(T* in_mat, int in_dim_r, int in_dim_c, const char *in_string = "A"){
//	
//	printf(in_string);	printf(" = \n");
//
//	for (int j = 0; j < in_dim_r; j++){	
//		for (int i = 0; i < in_dim_c; i++)	printf(" %5.7f ", in_mat[j * in_dim_c + i]);
//		printf("\n");
//	}
//	printf("\n");
//}
//
//inline void d_cv_Copy_Vector(float *in_vec, int in_dim, float *out_vec){
//	for(int i=0; i<in_dim; i++)	out_vec[i] = in_vec[i];
//}
//
//inline void d_n2v_Norm_2_Vector(float *in_vec, int in_dim, float &out_norm){
//	out_norm = 0.0f;
//	for(int i = 0; i<in_dim; i++)	out_norm += SQUARE(in_vec[i]);
//	out_norm = sqrt(out_norm);
//}
//
//inline void d_dev_Distance_Euclidean_Vector(float *in_vec1, float *in_vec2, int in_dim, float &out_dist){
//	out_dist = 0.0f;
//	for(int i = 0; i<in_dim; i++)	out_dist += SQUARE(in_vec1[i]-in_vec2[i]);
//	out_dist = sqrt(out_dist);
//}
//
//inline void d_ipv_Inner_Product_Vector(float *in_vec1, float *in_vec2, int in_dim, float &out_ip){
//
//	out_ip = 0.0f;
//	for(int i = 0; i<in_dim; i++) out_ip += in_vec1[i]*in_vec2[i];
//
//}
//
//inline void d_cpv3_Cross_Product_Vector3D(float *in_vec1, float *in_vec2, float *out_vec){
//
//	out_vec[0] = in_vec1[1]*in_vec2[2] - in_vec1[2]*in_vec2[1];
//	out_vec[1] = in_vec1[2]*in_vec2[0] - in_vec1[0]*in_vec2[2];
//	out_vec[2] = in_vec1[0]*in_vec2[1] - in_vec1[1]*in_vec2[0];
//
//}
//
//inline void d_tms_Transpose_Matrix_Square(float *in_mat, int in_dim, float *out_mat_t){
//	for(int j = 0; j<in_dim; j++){
//		for(int i = 0; i<in_dim; i++){
//
//			out_mat_t[j*in_dim+i] = in_mat[i*in_dim+j];
//		}
//	}
//}
//
//inline void d_avw_Add_Vector_Weighted(float *in_vec1, float *in_vec2, float in_w1, float in_w2, int in_dim, float *out_vec){
//	for(int i=0; i<in_dim; i++)	out_vec[i] = in_w1*in_vec1[i] + in_w2*in_vec2[i];
//}
//
//inline void d_mvvt_Multiply_Vector_Vector_Transpose(float *in_vec1, float *in_vec2, int in_dim, float *out_mat_v1v2t){
//	for(int j = 0; j<in_dim; j++){
//		for(int i = 0; i<in_dim; i++){
//			out_mat_v1v2t[j*in_dim + i] = in_vec1[j]*in_vec2[i];
//		}
//	}
//}
//
//inline void d_mmsv_Multiply_Matrix_Square_Vector(float *in_mat, float *in_vec, int in_dim, float *out_vec){
//	for(int j = 0; j<in_dim; j++){
//		out_vec[j] = 0.0f;
//		for(int i = 0; i<in_dim; i++){
//			out_vec[j] += in_mat[j*in_dim+i]*in_vec[i];
//		}
//	}
//}
//
//inline void d_mms_Multiply_Matrix_Square(float *in_mat1, float *in_mat2, int in_dim, float *out_mat12){
//	for(int j = 0; j<in_dim; j++){
//		for(int i = 0; i<in_dim; i++){
//			out_mat12[j*in_dim+i] = 0.0f;
//			for(int k = 0; k<in_dim; k++) out_mat12[j*in_dim+i] += in_mat1[j*in_dim+k]*in_mat2[k*in_dim+i];
//		}
//	}
//}
//
//inline void d_mmm_Multiply_Matrix_Matrix(
//	float *in_mat1, float *in_mat2, 
//	int in_dim_h1, int in_dim_w1_h2, int in_dim_w2,
//	float *out_mat12){
//
//	for(int j = 0; j<in_dim_h1; j++){
//		for(int i = 0; i<in_dim_w2; i++){
//
//
//			out_mat12[j*in_dim_w2 + i] = 0.0f;
//
//			for(int k = 0; k<in_dim_w1_h2; k++) 
//				out_mat12[j*in_dim_w2 + i] += in_mat1[j*in_dim_w1_h2 + k]*in_mat2[k*in_dim_w2 + i];
//		
//		
//		}
//	}
//}
//
//inline void d_ts_Transpose_Square(float *in_mat, int in_dim, float *out_mat_t){
//	for(int j = 0; j<in_dim; j++){
//		for(int i = 0; i<in_dim; i++){
//			out_mat_t[i*in_dim+j] = in_mat[j*in_dim+i];
//		}
//	}
//}
//
//inline void d_cvmcp_Convert_Vector3_to_Matrix_for_Cross_Product(float *in_vec, float *out_mat){
//
//	out_mat[0] = 0.0f;			out_mat[1] = -in_vec[2];	out_mat[2] = in_vec[1];
//	out_mat[3] = in_vec[2];		out_mat[4] = 0.0f;			out_mat[5] = -in_vec[0];
//	out_mat[6] = -in_vec[1];	out_mat[7] = in_vec[0];		out_mat[8] = 0.0f;
//}
//
//
//inline void d_cmstdv_Compute_Mean_and_STandard_DeViation(
//	float *in_vec,
//	int in_len_vec,
//	float &out_mean,
//	float &out_stdv)
//{
//	int zz, N;
//	float sumX, sumXsquared;
//	float mean, var, x;
//
//	N = in_len_vec;
//
//	sumX = sumXsquared = 0.0f;
//	
//	// Computes sum of elements and squared elements .
//	for (zz = 0; zz < N; zz++){
//		x = in_vec[zz];
//		sumX += x;
//		sumXsquared += x * x;
//	}
//	// Computes mean and standard deviation.
//	mean = sumX / (float)N;
//	var = sumXsquared / (float)N - mean * mean;
//
//	out_mean = mean;
//	out_stdv = sqrt(var);
//}