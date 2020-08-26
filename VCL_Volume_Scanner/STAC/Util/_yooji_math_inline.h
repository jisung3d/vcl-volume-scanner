#pragma once

#include <stdio.h>

// macros
#define SQUARE(x) (x)*(x)
#define ROUNDF(x) (x<0.0f) ? (int)(x-0.5f) : (int)(x+0.5f)
#define DIST_CIRCULAR(x) ( (x<=180.0f) ? x : 360.0f-x );

// basic inline math functions for pointers.
// TO DO:
// + for use these functions, you should check the dimension of input and output matrix pointers.
template <typename T>
inline void d_pm_Printf_Matrix(T *in_mat, int in_dim_r, int in_dim_c, const char *in_string = "A"){

	printf(in_string);	printf(" = \n");

	for(int j = 0; j < in_dim_r; j++){
		for(int i = 0; i < in_dim_c; i++)	printf(" %5.7f ", in_mat[j * in_dim_c + i]);
		printf("\n");
	}
	printf("\n");
}

inline void d_cv_Copy_Vector(float *in_vec, int in_dim, float *out_vec){
	for(int i = 0; i<in_dim; i++)	out_vec[i] = in_vec[i];
}

inline void d_n2v_Norm_2_Vector(float *in_vec, int in_dim, float &out_norm){
	out_norm = 0.0f;
	for(int i = 0; i<in_dim; i++)	out_norm += SQUARE(in_vec[i]);
	out_norm = sqrt(out_norm);
}

// detA = a11*(a22a33-a32a23) + a12*(a23a31 - a21a33) + a13*(a21a32 - a22a31)
inline void d_dm3_Determinant_Matrix_3(float *in_mat, float &out_det){
	
	out_det = in_mat[0]*(in_mat[4]*in_mat[8] - in_mat[5]*in_mat[7])
		+ in_mat[1]*(in_mat[5]*in_mat[6] - in_mat[3]*in_mat[8])
		+ in_mat[2]*(in_mat[3]*in_mat[7] - in_mat[4]*in_mat[6]);
}

inline void d_dev_Distance_Euclidean_Vector(float *in_vec1, float *in_vec2, int in_dim, float &out_dist){
	out_dist = 0.0f;
	for(int i = 0; i<in_dim; i++)	out_dist += SQUARE(in_vec1[i]-in_vec2[i]);
	out_dist = sqrt(out_dist);
}

inline void d_ipv_Inner_Product_Vector(float *in_vec1, float *in_vec2, int in_dim, float &out_ip){

	out_ip = 0.0f;
	for(int i = 0; i<in_dim; i++) out_ip += in_vec1[i]*in_vec2[i];

}

template <typename T>
inline void d_opv_Outer_Product_Vector(T *in_vec1, T *in_vec2, int in_dim, T *out_mat){

	for(int j = 0; j<in_dim; j++)
		for(int i = 0; i<in_dim; i++) out_mat[j*in_dim + i] = in_vec1[j]*in_vec2[i];	

}

template <typename T>
inline void d_cpv3_Cross_Product_Vector3D(T *in_vec1, T *in_vec2, T *out_vec){

	out_vec[0] = in_vec1[1]*in_vec2[2] - in_vec1[2]*in_vec2[1];
	out_vec[1] = in_vec1[2]*in_vec2[0] - in_vec1[0]*in_vec2[2];
	out_vec[2] = in_vec1[0]*in_vec2[1] - in_vec1[1]*in_vec2[0];

}

template <typename T>
inline void d_avw_Add_Vector_Weighted(T *in_vec1, T *in_vec2, float in_w1, float in_w2, int in_dim, T *out_vec){
	for(int i = 0; i<in_dim; i++)	out_vec[i] = in_w1*in_vec1[i] + in_w2*in_vec2[i];
}

// outer product랑 겹침....  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
inline void d_mvvt_Multiply_Vector_Vector_Transpose(float *in_vec1, float *in_vec2, int in_dim, float *out_mat_v1v2t){
	for(int j = 0; j<in_dim; j++){
		for(int i = 0; i<in_dim; i++){
			out_mat_v1v2t[j*in_dim + i] = in_vec1[j]*in_vec2[i];
		}
	}
}

inline void d_mmsv_Multiply_Matrix_Square_Vector(float *in_mat, float *in_vec, int in_dim, float *out_vec){
	for(int j = 0; j<in_dim; j++){
		out_vec[j] = 0.0f;
		for(int i = 0; i<in_dim; i++){
			out_vec[j] += in_mat[j*in_dim+i]*in_vec[i];
		}
	}
}

inline void d_mms_Multiply_Matrix_Square(float *in_mat1, float *in_mat2, int in_dim, float *out_mat12){
	for(int j = 0; j<in_dim; j++){
		for(int i = 0; i<in_dim; i++){
			out_mat12[j*in_dim+i] = 0.0f;
			for(int k = 0; k<in_dim; k++) out_mat12[j*in_dim+i] += in_mat1[j*in_dim+k]*in_mat2[k*in_dim+i];
		}
	}
}

inline void d_ts_Transpose_Square(float *in_mat, int in_dim, float *out_mat_t){
	for(int j = 0; j<in_dim; j++){
		for(int i = 0; i<in_dim; i++){
			out_mat_t[i*in_dim+j] = in_mat[j*in_dim+i];
		}
	}
}


inline void d_cvmcp_Convert_Vector3_to_Matrix_for_Cross_Product(float *in_vec, float *out_mat){

	out_mat[0] = 0.0f;			out_mat[1] = -in_vec[2];	out_mat[2] = in_vec[1];
	out_mat[3] = in_vec[2];		out_mat[4] = 0.0f;			out_mat[5] = -in_vec[0];
	out_mat[6] = -in_vec[1];	out_mat[7] = in_vec[0];		out_mat[8] = 0.0f;
}


inline void d_cmstdv_Compute_Mean_and_STandard_DeViation(
	float *in_vec,
	int in_len_vec,
	float &out_mean,
	float &out_stdv)
{
	int zz, N;
	float sumX, sumXsquared;
	float mean, var, x;

	N = in_len_vec;

	sumX = sumXsquared = 0.0f;

	// Computes sum of elements and squared elements .
	for(zz = 0; zz < N; zz++){
		x = in_vec[zz];
		sumX += x;
		sumXsquared += x * x;
	}
	// Computes mean and standard deviation.
	mean = sumX / (float)N;
	var = sumXsquared / (float)N - mean * mean;

	out_mean = mean;
	out_stdv = sqrt(var);
}

// 이놈이 이상했다아!!!!!!!!!
// 이놈이 이상했다아!!!!!!!!!
// 이놈이 이상했다아!!!!!!!!!
//********************************************************************************************
// https://en.wikipedia.org/wiki/Adjugate_matrix
// inv(A) = adj(A)/det(A)
inline float d_im3_Inverse_Matrix_3(float *in_mat, float *out_mat)
//********************************************************************************************
{
	double det, det_inv;
	
	double a11, a12, a13;
	double a21, a22, a23;
	double a31, a32, a33;

	a11 = in_mat[0]; a12 = in_mat[1]; a13 = in_mat[2];
	a21 = in_mat[3]; a22 = in_mat[4]; a23 = in_mat[5];
	a31 = in_mat[6]; a32 = in_mat[7]; a33 = in_mat[8];

	// det(A)
	det = a11*(a22*a33-a32*a23) + a12*(a23*a31 - a21*a33) + a13*(a21*a32 - a22*a31);

	if(det<10e-7)	return 0.0f;

	// inv(A) = adj(A)/det(A)
	out_mat[0] = +(a22*a33 - a23*a32)/det;
	out_mat[1] = -(a21*a33 - a23*a31)/det;
	out_mat[2] = +(a21*a32 - a22*a31)/det;

	out_mat[3] = -(a12*a33 - a13*a32)/det;
	out_mat[4] = +(a11*a33 - a13*a31)/det;
	out_mat[5] = -(a11*a32 - a12*a31)/det;

	out_mat[6] = +(a12*a23 - a13*a22)/det;
	out_mat[7] = -(a11*a23 - a13*a21)/det;
	out_mat[8] = +(a11*a22 - a12*a21)/det;


	// compute determinant.
// 	d_dm3_Determinant_Matrix_3(in_mat,det);
// 	if(det<10e-7)	return 0.0f;

	// compute inverse matrix.
// 	out_mat[0] = (in_mat[4]*in_mat[8] - in_mat[5]*in_mat[7])/det;
// 	out_mat[1] = -(in_mat[1]*in_mat[8] - in_mat[2]*in_mat[7])/det;
// 	out_mat[2] = (in_mat[1]*in_mat[5] - in_mat[2]*in_mat[4])/det;
// 
// 	out_mat[3] = -(in_mat[3]*in_mat[8] - in_mat[5]*in_mat[6])/det;
// 	out_mat[4] = (in_mat[0]*in_mat[8] - in_mat[2]*in_mat[6])/det;
// 	out_mat[5] = -(in_mat[0]*in_mat[5] - in_mat[2]*in_mat[3])/det;
// 
// 	out_mat[6] = (in_mat[3]*in_mat[7] - in_mat[4]*in_mat[6])/det;
// 	out_mat[7] = -(in_mat[0]*in_mat[7] - in_mat[1]*in_mat[6])/det;
// 	out_mat[8] = (in_mat[0]*in_mat[4] - in_mat[1]*in_mat[3])/det;
// 
// 	det_inv=1.0/det;
// 	for(int i=0; i<9; i++)  out_mat[i] *= det_inv;
	
	return det;
}

//********************************************************************************************
inline float d_im4_Inverse_Matrix_4(float *in_mat, float *out_mat)
//********************************************************************************************
{
	float m11, m12, m13, m14;
	float m21, m22, m23, m24;
	float m31, m32, m33, m34;
	float m41, m42, m43, m44;
	float t[12], det, det_inv;

	// input matrix.
	m11=in_mat[0];	m12=in_mat[1];	m13=in_mat[2];	m14=in_mat[3];
	m21=in_mat[4];	m22=in_mat[5];	m23=in_mat[6];	m24=in_mat[7];
	m31=in_mat[8];	m32=in_mat[9];	m33=in_mat[10];	m34=in_mat[11];
	m41=in_mat[12];	m42=in_mat[13];	m43=in_mat[14];	m44=in_mat[15];

	// pre-computation.
	t[0]=m33*m44;	
	t[1]=m34*m42;	
	t[2]=m32*m43;	
	t[3]=m34*m43;
	t[4]=m32*m44;	
	t[5]=m33*m42;

	t[6]=m31*m44;	
	t[7]=m33*m41;	
	t[8]=m34*m41;	
	t[9]=m31*m43;
	t[10]=m31*m42;	
	t[11]=m32*m41;

	// compute determinant.
	out_mat[0]=m22*t[0]	+m23*t[1] +m24*t[2]	-m22*t[3] -m23*t[4] -m24*t[5];
	out_mat[1]=m12*t[3]	+m13*t[4] +m14*t[5] -m12*t[0] -m13*t[1]	-m14*t[2];
	out_mat[2]=m12*m23*m44	+m13*m24*m42 +m14*m22*m43 -m12*m24*m43	-m13*m22*m44 -m14*m23*m42;
	out_mat[3]=m12*m24*m33	+m13*m22*m34 +m14*m23*m32 -m12*m23*m34	-m13*m24*m32 -m14*m22*m33;

	det=m11*out_mat[0] +m21*out_mat[1] +m31*out_mat[2] + m41*out_mat[3];

	if(det==0.0f)	return det;

	out_mat[4]=m21*t[3]	+m23*t[6] +m24*t[7] -m21*t[0] -m23*t[8] -m24*t[9];
	out_mat[5]=m11*t[0]	+m13*t[8] +m14*t[9] -m11*t[3] -m13*t[6] -m14*t[7];
	out_mat[6]=m11*m24*m43	+m13*m21*m44 +m14*m23*m41 -m11*m23*m44	-m13*m24*m41 -m14*m21*m43;
	out_mat[7]=m11*m23*m34	+m13*m24*m31 +m14*m21*m33 -m11*m24*m33	-m13*m21*m34 -m14*m23*m31;

	out_mat[8]=m21*t[4]	+m22*t[8] +m24*t[10] -m21*t[1] -m22*t[6] -m24*t[11];
	out_mat[9]=m11*t[1]	+m12*t[6] +m14*t[11] -m11*t[4] -m12*t[8] -m14*t[10];
	out_mat[10]=m11*m22*m44 +m12*m24*m41 +m14*m21*m42 -m11*m24*m42	-m12*m21*m44 -m14*m22*m41;
	out_mat[11]=m11*m24*m32 +m12*m21*m34 +m14*m22*m31 -m11*m22*m34	-m12*m24*m31 -m14*m21*m32;

	out_mat[12]=m21*t[5] +m22*t[9] +m23*t[11] -m21*t[2] -m22*t[7] -m23*t[10];
	out_mat[13]=m11*t[2] +m12*t[11] +m13*t[10] -m11*t[5] -m12*t[9] -m13*t[11];
	out_mat[14]=m11*m23*m42 +m12*m21*m43 +m13*m22*m41 -m11*m22*m43	-m12*m23*m41 -m13*m21*m42;
	out_mat[15]=m11*m22*m33 +m12*m23*m31 +m13*m21*m32 -m11*m23*m32	-m12*m21*m33 -m13*m22*m31;

	det_inv=1.0f/det;
	for(int i=0; i<16; i++)  out_mat[i]*=det_inv;	
	
	return det;
}