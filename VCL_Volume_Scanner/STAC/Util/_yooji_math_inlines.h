//////////////////////////////////////////////////////////////////////
// _yooji_math_inlies.h
//////////////////////////////////////////////////////////////////////
#pragma once

#include <stdio.h>

// macros
#define SQUARE(x) (x)*(x)
#define ROUNDF(x) (x<0.0f) ? (int)(x-0.5f) : (int)(x+0.5f)
#define DIST_CIRCULAR(x) ( (x<=180.0f) ? x : 360.0f-x )

#define CLIP(a, b, x) ( ((x<a) ? a : x)>b ? b : x) 

// basic inline math functions for pointers.
// TO DO:
// + for use these functions, you should check the dimension of input and output matrix pointers.
// =========================================================================
// HOST ONLY ===============================================================
// =========================================================================
template <typename T>
inline void d_pm_Printf_Matrix(T* in_mat, int in_dim_r, int in_dim_c, const char *in_string = "A"){
	
	printf(in_string);	printf(" = \n");

	for (int j = 0; j < in_dim_r; j++){	
		//for (int i = 0; i < in_dim_c; i++)	printf(" %5.4f ", in_mat[j * in_dim_c + i]);
		for (int i = 0; i < in_dim_c; i++)	printf(" %5.7e ", in_mat[j * in_dim_c + i]);
		printf("\n");
	}
	printf("\n");
}

// inline void d_dmf_Display_Matrix_Float(CKvMatrixFloat &in_matrix_float, float in_mul, float in_offset)
// {
// 	CKvScreen sc; sc.s_d_Display(in_mul, in_offset, &in_matrix_float);
// 	if(!Kv_Printf("d_dmf_Display_Matrix_Float")) exit(0);
// }

// =========================================================================
// HOST AND DEVICE =========================================================
// =========================================================================
// Basic vector/matrix 
template<typename T>
/*_CPU_AND_GPU_CODE_*/ inline void d_cv_Copy_Vector(T *in_vec, int in_dim, T *out_vec){
	for(int i=0; i<in_dim; i++)	out_vec[i] = in_vec[i];
}

template<typename T>
/*_CPU_AND_GPU_CODE_*/ inline void d_n2v_Norm_2_Vector(T *in_vec, int in_dim, T &out_norm){
	out_norm = 0.0f;
	for(int i = 0; i<in_dim; i++)	out_norm += SQUARE(in_vec[i]);
	out_norm = sqrt(out_norm);
}

template<typename T>
/*_CPU_AND_GPU_CODE_*/ inline void d_niv_Norm_Infinity_Vector(T *in_vec,int in_dim,T &out_norm){
	float norm_inf;	out_norm = -100000.0f;
	for(int i = 0; i<in_dim; i++){
		if(out_norm < (norm_inf = abs(in_vec[i]))) out_norm = norm_inf;
	}
}

/*_CPU_AND_GPU_CODE_*/ inline float d_nv_Normalize_Vector(float *in_vec,int in_dim,float *out_vec){
	float out_norm = 0.0f;
	// compute norm.
	for(int i = 0; i<in_dim; i++)	out_norm += SQUARE(in_vec[i]);
	out_norm = sqrt(out_norm);
	// normalize vector.
	for(int i=0; i<in_dim; i++) out_vec[i] = in_vec[i]/out_norm;

	return out_norm;
}

/*_CPU_AND_GPU_CODE_*/ inline void d_dev_Distance_Euclidean_Vector(float *in_vec1, float *in_vec2, int in_dim, float &out_dist){
	out_dist = 0.0f;
	for(int i = 0; i<in_dim; i++)	out_dist += SQUARE(in_vec1[i]-in_vec2[i]);
	out_dist = sqrt(out_dist);
}

/*_CPU_AND_GPU_CODE_*/ inline void d_ipv_Inner_Product_Vector(float *in_vec1, float *in_vec2, int in_dim, float &out_ip){

	out_ip = 0.0f;
	for(int i = 0; i<in_dim; i++) out_ip += in_vec1[i]*in_vec2[i];

}

/*_CPU_AND_GPU_CODE_*/ inline void d_cpv3_Cross_Product_Vector3D(float *in_vec1, float *in_vec2, float *out_vec){

	out_vec[0] = in_vec1[1]*in_vec2[2] - in_vec1[2]*in_vec2[1];
	out_vec[1] = in_vec1[2]*in_vec2[0] - in_vec1[0]*in_vec2[2];
	out_vec[2] = in_vec1[0]*in_vec2[1] - in_vec1[1]*in_vec2[0];

}

/*_CPU_AND_GPU_CODE_*/ inline void d_tms_Transpose_Matrix_Square(float *in_mat, int in_dim, float *out_mat_t){
	for(int j = 0; j<in_dim; j++){
		for(int i = 0; i<in_dim; i++){

			out_mat_t[j*in_dim+i] = in_mat[i*in_dim+j];
		}
	}
}

template <typename T>
/*_CPU_AND_GPU_CODE_*/ inline void d_tm_Transpose_Matrix(T *in_mat,int in_rows,int in_cols,T *out_mat_t){
	for(int j = 0; j<in_rows; j++){
		for(int i = 0; i<in_cols; i++){
			out_mat_t[i*in_rows+j] = in_mat[j*in_cols+i];
		}
	}
}

template <typename T>
/*_CPU_AND_GPU_CODE_*/ inline void d_cme_Compute_Matrix_Exponential(T *in_vec_6, T *out_mat_4x4){

	float ts,ts2;
	float tv3[3],tv3_2[3];
	float tm3[9],tm3_2[9],tm4[16],tm4_2[16];
	float exp_xi[16],exp_xi_inv[16],w[3],wx[9],wx_sq[9],exp_wx[9],v[3];
	float R[9],t[3];
	float norm;

	//////////////////////////////////////////////////////////////////////////
	/// BASIC EQUATION FOR ESTIMATING RIGID BODY MOTION.
	// REFERENCE: Ma et al. An Invitation to 3D Vision.
	// p. 29. Equation 2.30.
	//////////////////////////////////////////////////////////////////////////
	// in_x is 6-d twist vector.	[ w1, w2, w3, v1, v2, v3 ]	
	//		  |	0		-w3		 w2		v1	|
	// Xi =   |	w3		 0		-w1		v2	|
	//        |	w2		 w1		 0		v3	|
	//		  | 0		 0		 0		0	|

	// T(t1) = exp( (t1-t0)*Xi )*T(t0).

	//		  |   0		-w3		w2  |
	// [w]x = |  w3		 0		-w1 |
	//		  | -w2		w1		0   |

	// exp( [w]x*s ) 
	// = I + sin(s)*[w]x + (1 - cos(s))*[w]x^2.

	// exp( Xi ) 
	// = | exp([w]x*s)  (I-exp([w]x*s))*[w]x*v + w*w^T*v*s |
	//   |    0                     1                |

	// we set t = 1.

	// normalized w.
	for(int i = 0; i<3; i++)	w[i] = in_vec_6[i];
	d_n2v_Norm_2_Vector(w,3,norm);
	for(int i=0; i<3; i++)	w[i] /= norm;
	// v normalized by norm of w.
	for(int i=0; i<3; i++)	v[i] = in_vec_6[3 + i]/norm;
	// [w]x.
	d_cvmcp_Convert_Vector3_to_Matrix_for_Cross_Product(w,wx);
	// [w]x^2
	d_cv_Copy_Vector(wx,9,tm3);
	d_mms_Multiply_Matrix_Square(wx,tm3,3,wx_sq);

	// R = exp([w]x*s)
	// + tm3 = sin(s)*[w]x + (1 - cos(s))*[w]x^2
	d_avw_Add_Vector_Weighted(wx,wx_sq,sin(norm),(1.0f - cos(norm)),9,tm3);
	// + R = I + tm3
	d_cv_Copy_Vector(tm3,9,R);	R[0] += 1.0f;	R[4] += 1.0f; R[8] += 1.0f;

	//d_pm_Printf_Matrix(p_x, 1, 6, "p_x");
	//d_pm_Printf_Matrix(w, 1, 3, "w");
	//d_pm_Printf_Matrix(wx, 3, 3, "wx");
	//d_pm_Printf_Matrix(wx_sq, 3, 3, "wx_sq");
	//d_pm_Printf_Matrix(tm3, 3, 3, "R");
	//if (!Kv_Printf("Cost"))	exit(0);

	// t = (I-exp([w]x*s))*[w]x*v + w*w^T*v*s
	// + tv3 = (I-exp([w]x*s))*[w]x*v
	// + I-exp([w]x*s) = -tm3
	for(int i=0; i<9; i++)	tm3[i] *= -1.0f;
	d_mms_Multiply_Matrix_Square(tm3,wx,3,tm3_2);
	d_mmsv_Multiply_Matrix_Square_Vector(tm3_2,v,3,tv3);
	// + tv3_2 = w*w^T*v*s
	d_ipv_Inner_Product_Vector(w,v,3,ts);	ts *= norm;
	for(int i=0; i<3; i++)	tv3_2[i] = w[i]*ts;
	// + t = tv3 + tv3_2
	for(int i=0; i<3; i++)	t[i] = tv3[i] + tv3_2[i];

	// T(t) = exp(Xi).
	for(int j=0; j<3; j++)	for(int i=0; i<3; i++)	exp_xi[j*4 + i] = R[j*3 + i];
	for(int j=0; j<3; j++)	exp_xi[j*4 + 3] = t[j];
	exp_xi[12] = exp_xi[13] = exp_xi[14] = 0.0f;	exp_xi[15] = 1.0f;

	// T(t1) = exp( (t1-t0)*Xi )*T(t0).
	for(int i=0; i<16; i++)	out_mat_4x4[i] = exp_xi[i];
}

/*_CPU_AND_GPU_CODE_*/ inline void d_avw_Add_Vector_Weighted(float *in_vec1, float *in_vec2, float in_w1, float in_w2, int in_dim, float *out_vec){
	for(int i=0; i<in_dim; i++)	out_vec[i] = in_w1*in_vec1[i] + in_w2*in_vec2[i];
}

template <typename T>
/*_CPU_AND_GPU_CODE_*/ inline void d_mvvt_Multiply_Vector_Vector_Transpose(T *in_vec1, T *in_vec2, int in_dim, T *out_mat_v1v2t){
	for(int j = 0; j<in_dim; j++){
		for(int i = 0; i<in_dim; i++){
			out_mat_v1v2t[j*in_dim + i] = in_vec1[j]*in_vec2[i];
		}
	}
}

/*_CPU_AND_GPU_CODE_*/ inline void d_mmsv_Multiply_Matrix_Square_Vector(float *in_mat, float *in_vec, int in_dim, float *out_vec){
	for(int j = 0; j<in_dim; j++){
		out_vec[j] = 0.0f;
		for(int i = 0; i<in_dim; i++){
			out_vec[j] += in_mat[j*in_dim+i]*in_vec[i];
		}
	}
}

template<typename T>
/*_CPU_AND_GPU_CODE_*/ inline void d_mms_Multiply_Matrix_Square(T *in_mat1, T *in_mat2, int in_dim, T *out_mat12){
	for(int j = 0; j<in_dim; j++){
		for(int i = 0; i<in_dim; i++){
			out_mat12[j*in_dim+i] = 0.0f;
			for(int k = 0; k<in_dim; k++) out_mat12[j*in_dim+i] += in_mat1[j*in_dim+k]*in_mat2[k*in_dim+i];
		}
	}
}

template<typename T>
/*_CPU_AND_GPU_CODE_*/ inline void d_mmm_Multiply_Matrix_Matrix(
	T *in_mat1, T *in_mat2, 
	int in_dim_h1, int in_dim_w1_h2, int in_dim_w2,
	T *out_mat12){

	for(int j = 0; j<in_dim_h1; j++){
		for(int i = 0; i<in_dim_w2; i++){


			out_mat12[j*in_dim_w2 + i] = 0.0f;

			for(int k = 0; k<in_dim_w1_h2; k++) 
				out_mat12[j*in_dim_w2 + i] += in_mat1[j*in_dim_w1_h2 + k]*in_mat2[k*in_dim_w2 + i];
		
		
		}
	}
}

template<typename T>
/*_CPU_AND_GPU_CODE_*/ inline void d_cgm_Compute_Gram_Matrix(T *in_mat,int in_dim_h,int in_dim_w,T *out_mat_gram){
	for(int j = 0; j<in_dim_w; j++){
		for(int i = 0; i<in_dim_w; i++){

			out_mat_gram[j*in_dim_w+i] = 0.0f;
			for(int k=0; k<in_dim_h; k++)
				out_mat_gram[j*in_dim_w+i] += in_mat[k*in_dim_w + j]*in_mat[k*in_dim_w + i];
		}
	}
}

template<typename T>
/*_CPU_AND_GPU_CODE_*/ inline void d_t_Transpose(T *in_mat,int in_dim_h,int in_dim_w,T *out_mat_t){
	for(int j = 0; j<in_dim_h; j++){
		for(int i = 0; i<in_dim_w; i++){
			out_mat_t[i*in_dim_h+j] = in_mat[j*in_dim_w+i];
		}
	}
}

template<typename T>
/*_CPU_AND_GPU_CODE_*/ inline void d_ts_Transpose_Square(T *in_mat, int in_dim, T *out_mat_t){
	for(int j = 0; j<in_dim; j++){
		for(int i = 0; i<in_dim; i++){
			out_mat_t[i*in_dim+j] = in_mat[j*in_dim+i];
		}
	}
}

template<typename T>
/*_CPU_AND_GPU_CODE_*/ inline void d_cvmcp_Convert_Vector3_to_Matrix_for_Cross_Product(T *in_vec, T *out_mat){

	out_mat[0] = T(0);			out_mat[1] = -in_vec[2];	out_mat[2] = in_vec[1];
	out_mat[3] = in_vec[2];		out_mat[4] = T(0);			out_mat[5] = -in_vec[0];
	out_mat[6] = -in_vec[1];	out_mat[7] = in_vec[0];		out_mat[8] = T(0);
}


/*_CPU_AND_GPU_CODE_*/ inline void d_cmstdv_Compute_Mean_and_STandard_DeViation(
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
	for (zz = 0; zz < N; zz++){
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

//********************************************************************************************
// A = [a b]
//	   [c d]
// det([(lambda - a) -b]) = 0
//     [-c (lambda - d)]
// lambda^2 - (a + d)*lambda + (a*d - b*c) = 0
inline bool d_cev2_Compute_Eigenvalue_Matrix_2(float *in_mat, float &out_lambda1, float &out_lambda2)
//********************************************************************************************
{
	float a, b, c; // for polynomial equation.

	a = 1.0f;
	b = - (in_mat[0] + in_mat[3]);
	c = in_mat[0]*in_mat[3] - in_mat[1]*in_mat[2];

	// lambda = (-b +(+-)root(b^2 - 4*a*c))/(2*a)
	float disc;
	// skip negative discriminant.	
	if((disc = SQUARE(b) - 4*a*c) < 0.0f) return false;

	out_lambda1 = (-b + sqrt(disc))/(2*a);
	out_lambda2 = (-b - sqrt(disc))/(2*a);

	//printf("disc: %f lambda: %f %f\n", disc, out_lambda1, out_lambda2);

	return true;
}

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


// Special matrices.
//********************************************************************************************
inline bool d_im_Inverse_Matrix_4x4(float *in_mat, float *out_mat)
//********************************************************************************************
{
	float m11, m12, m13, m14;
	float m21, m22, m23, m24;
	float m31, m32, m33, m34;
	float m41, m42, m43, m44;
	float t[12], det;

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

	if(det==0.0f)	return false;

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

	det=1.0f/det;
	for(int i=0; i<16; i++)  out_mat[i]*=det;
	


	return true;
}

//
////********************************************************************************************
//// Every symmetric, positive definite matrix A can be decomposed into 
//// a product of a unique lower triangular matrix L and its transpose
//// Cholesky decomposition.
//// Ljj = sqrt( Ajj - sum(Ljk^2)(k=0:j-1) )
//// Lij = ( Aij - sum(Lik*Ljk)(k=0:j-1) )/Ljj
//inline bool d_lld_LL_Decomposition(const float *A, int dim, float *L)
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
//inline bool d_sls_Solve_Linear_System_using_LLD(const float *L, const float *b, 
//	float *y, int dim, float *sol_x)
////********************************************************************************************
//{
//	float tsum;
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