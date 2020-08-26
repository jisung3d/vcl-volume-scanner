/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_cuda_volume_integrator.cpp
/////////////////////////////////////////////////////////////////////////////////////////////
//#include "_yooji_2017_cuda_object_scanner.cuh"
//#define __CUDASCAN__
#include "../../_yooji_2017_cuda_object_scanner.cuh"

__constant__ float K_dev[GK_LEVEL_OF_IMAGE_PYRAMID*4];
__constant__ float T_gc_dev_const[16];
__constant__ float T_cg_dev_const[16];
__constant__ int dim_map_dev[GK_LEVEL_OF_IMAGE_PYRAMID*2];

__constant__ float th_icp_dev[1];

// ////////////////////////////////////////////////////////////////////////////////
// Local functions .///////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////
// ===============================================================================
// Host functions.
// ===============================================================================
__host__ bool z_uit_Update_Incremental_Tracking(const float *sol_x, float *T)
//********************************************************************************************
{

 	static float tm[16];
 
 	const float *p_x = sol_x;
 	float *p_mat = T;
 	
 	/// CAUTION
 	// in_x is 6-d vector.	[ r1, r2, r3, t1, t2, t3 ]	
 	// + this transformation Tg,k transforms the k-th camera coordinates to global model coordinates.
 	//        |	1	r3	-r2	t1	|
 	// Tg,k = |	-r3	1	r1	t2	|
 	//        |	r2	-r1	1	t3	|
 	for(int i=0; i<16; i++)	tm[i] = p_mat[i];
 	
 	for(int i = 0; i<4; i++)	p_mat[i] = +1.0f	 *tm[i]	+p_x[2]	*tm[i+4]	-p_x[1]	*tm[i+8]	+p_x[3]*tm[i+12];
 	for(int i = 0; i<4; i++)	p_mat[i+4] = -p_x[2] *tm[i]	+1.0f	*tm[i+4]	+p_x[0]	*tm[i+8]	+p_x[4]*tm[i+12];
 	for(int i = 0; i<4; i++)	p_mat[i+8] = +p_x[1] *tm[i]	-p_x[0]	*tm[i+4]	+1.0f	*tm[i+8]	+p_x[5]*tm[i+12];

	//p_mat[12] = p_mat[13] =  p_mat[14] = 0.0f; p_mat[15] = 1.0f;
	
	return true;
}

//********************************************************************************************
// For KinectFusion.
// REFERENCE: Ma et al. An Invitation to 3D Vision.
__host__ bool z_uitme_Update_Incremental_Tracking_with_Matrix_Exponential(
	float *in_x,
	float *io_hmat_t0_to_t1)
//********************************************************************************************
{
	/// CAUTION
	// in_x is 6-d vector.	[ r1, r2, r3, t1, t2, t3 ]	
	// + this transformation Tg,k transforms the k-th camera coordinates to global model coordinates.
	//        |	1	-r3	r2	t1	|
	// Tg,k = |	r3	1	-r1	t2	|
	//        |	-r2	r1	1	t3	|

 	float *p_mat, *p_x;
 	float ts, ts2;
 	float tv3[3], tv3_2[3];
 	float tm3[9], tm3_2[9], tm4[16], tm4_2[16];
 	float exp_xi[16], exp_xi_inv[16], w[3], wx[9], wx_sq[9], exp_wx[9], v[3];
 	float R[9], t[3];
 	float norm;
 
 	p_x = in_x;
 	p_mat = io_hmat_t0_to_t1;		// 4x4 matrix.
 
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
 	for(int i = 0; i<3; i++)	w[i] = p_x[i];
 	d_n2v_Norm_2_Vector(w, 3, norm);
 	for(int i=0; i<3; i++)	w[i] /= norm;
 	// v normalized by norm of w.
 	for(int i=0; i<3; i++)	v[i] = p_x[3 + i]/norm;
 	// [w]x.
 	d_cvmcp_Convert_Vector3_to_Matrix_for_Cross_Product(w, wx);
 	// [w]x^2
 	d_cv_Copy_Vector(wx, 9, tm3);
 	d_mms_Multiply_Matrix_Square(wx, tm3, 3, wx_sq);
 	
 	// R = exp([w]x*s)
 	// + tm3 = sin(s)*[w]x + (1 - cos(s))*[w]x^2
 	d_avw_Add_Vector_Weighted(wx, wx_sq, sin(norm), (1.0f - cos(norm)), 9, tm3);
 	// + R = I + tm3
 	d_cv_Copy_Vector(tm3, 9, R);	R[0] += 1.0f;	R[4] += 1.0f; R[8] += 1.0f;
 
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
 	d_mms_Multiply_Matrix_Square(tm3, wx, 3, tm3_2);
 	d_mmsv_Multiply_Matrix_Square_Vector(tm3_2, v, 3, tv3);
 	// + tv3_2 = w*w^T*v*s
 	d_ipv_Inner_Product_Vector(w, v, 3, ts);	ts *= norm;
 	for(int i=0; i<3; i++)	tv3_2[i] = w[i]*ts;
 	// + t = tv3 + tv3_2
 	for(int i=0; i<3; i++)	t[i] = tv3[i] + tv3_2[i];
 
 	// T(t) = exp(Xi).
 	for(int j=0; j<3; j++)	for(int i=0; i<3; i++)	exp_xi[j*4 + i] = R[j*3 + i];
 	for(int j=0; j<3; j++)	exp_xi[j*4 + 3] = t[j];
 	exp_xi[12] = exp_xi[13] = exp_xi[14] = 0.0f;	exp_xi[15] = 1.0f;
 
 	// T(t1) = exp( (t1-t0)*Xi )*T(t0).
 	for(int i=0; i<16; i++)	tm4[i]=p_mat[i];
 
 	d_mms_Multiply_Matrix_Square(exp_xi, tm4, 4, p_mat);
	
	
	
	return true;
}

// ===============================================================================
// Device functions.
// ===============================================================================
__device__ bool d_cssbp_Compute_Single_Summand_with_Backward_Params(
	int x,int y,
	const float *in_map_depth_t1,
	const float *in_map_depth_t0,
	const float *in_map_vertex_t0,
	const float *in_map_normal_t0,
	const float *in_T_cg_est,
	int in_lev_of_pyram,
	int ww,int hh,
	float in_th_icp,
	float *ATA,
	float *ATb)
	//float *out_b,
	//bool *flag_inlier)
{
	// for rigid transformation.
	Vector2f tpix,p2d_pred;	Vector3f tp3d,p3d_proj,p3d_pred,norm_pred_g;

	// for linear system.
	float A[6] ={0.0f}; float b = 0.0f;  // A = [A1 A2]
	float *p_K_dev = &K_dev[in_lev_of_pyram*4];

	float td0,td1;
	int tidx,i,j;
	bool flag;

	tpix.x = x; tpix.y = y;
	tidx = y*ww + x;

	// check depth validity of map t1.
	//if((td1 = in_map_depth_t1[tidx]) < 1.0e-6f) return false;
	td1 = in_map_depth_t1[tidx];
	if(td1 < 1.0e-6f) return false;

	// compute 3d point back-projected from current estimated pose in the global coordinates.
	// + [p3d_proj]
	d_bp_Back_Project(tpix,p_K_dev,td1,tp3d);
	d_t_Transform(tp3d,in_T_cg_est,p3d_proj);
	// compute 3d point predicted from the global model and previous camera pose in the global coordinates.
	// + [p3d_pred]
	d_t_Transform(p3d_proj,T_gc_dev_const,tp3d);
	d_p_Project(tp3d,p_K_dev,p2d_pred);
	// + check depth validity.
	//	if(!d_gid_Get_Interpolated_Depth(p2d_pred,ww,hh,in_map_depth_t0,td0)) return false;
	// 	d_bp_Back_Project(p2d_pred,K_dev,td0,tp3d);

	//if(!d_giv_Get_Interpolated_Vertex(p2d_pred,ww,hh,in_map_vertex_t0,tp3d)) return false;
	flag = d_giv_Get_Interpolated_Vertex(p2d_pred,ww,hh,in_map_vertex_t0,tp3d);
	if(!flag) return false;
	d_t_Transform(tp3d,T_cg_dev_const,p3d_pred);

	// /////////////////////////////////////////////////////////////////////////////
	// check distance between [p3d_proj] and [p3d_pred].
	td0 = SQUARE(p3d_pred.x - p3d_proj.x) + SQUARE(p3d_pred.y - p3d_proj.y) + SQUARE(p3d_pred.z - p3d_proj.z);

	//////////////////////////////////////////////////////////////////////////
	if(td0 > in_th_icp)	return false;
	//////////////////////////////////////////////////////////////////////////

	// check correlation between normals of [p3d_proj] and [p3d_pred].
	// + compute normal of [p3d_proj].



	// + compute normal of [p3d_pred].
	//if(!d_gin_Get_Interpolated_Normal(p2d_pred,ww,hh,in_map_normal_t0,norm_pred_g)) return false;
	flag = d_gin_Get_Interpolated_Normal(p2d_pred,ww,hh,in_map_normal_t0,norm_pred_g);
	if(!flag) return false;

	//////////////////////////////////////////////////////////////////////////
	//norm_pred_g.x = norm_pred_g.y = 0.0f; norm_pred_g.z = -1.0f;
	//////////////////////////////////////////////////////////////////////////

	// generate 6x6 linear system by computing the derivative of the objective function (22).
	// refer to equation (24) of "KinectFusion: Real-Time Dense Surface Mapping and Tracking", ISMAR 2011.
	// + compute A.	

	A[0]= __fadd_rn(__fmul_rn(p3d_proj.z,norm_pred_g.y),__fmul_rn(-p3d_proj.y,norm_pred_g.z));
	A[1]= __fadd_rn(__fmul_rn(-p3d_proj.z,norm_pred_g.x),__fmul_rn(p3d_proj.x,norm_pred_g.z));
	A[2]= __fadd_rn(__fmul_rn(p3d_proj.y,norm_pred_g.x),__fmul_rn(-p3d_proj.x,norm_pred_g.y));
	A[3]= norm_pred_g.x;	A[4]= norm_pred_g.y;	A[5]= norm_pred_g.z;

	// + compute b.
	// b = Ng,t-1^T*(Vg,t-1 - Vg,t)
	//float tsum,b1,b2,b3;

	b = dot(norm_pred_g,(p3d_pred - p3d_proj));

	// 	tp3d = p3d_pred - p3d_proj; b = 0.0f;
	// 	b = __fmaf_rn(norm_pred_g.x,tp3d.x,b);
	// 	b = __fmaf_rn(norm_pred_g.y,tp3d.y,b);
	// 	b = __fmaf_rn(norm_pred_g.z,tp3d.z,b);

	// update linear system.
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// This part is important.
	// Shared memory 써서 block 단위로 결과 모은다음 마지막에
	// reduction 써서 block 단위 sum 합쳐야 함.
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// copy ATA and ATb values of current thread to shared memory.
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// 예제의 경우 total thread number (=grid_dim * block dim) 이 vector 전체
	// size 보다 작기 때문에 이 부분이 필요하다.
	// 예제처럼 이렇게 while 문으로 control 하는 것이 필요한가?
	// 하나의 thread 값을 하나의 cache 에 넣으면 되는거 아님 그냥?	
	//while(tx<ww && ty<hh){

	// for ATA
	for(i = 0; i<6; i++) for(j = 0; j<6; j++){
		{
			ATA[i*6 + j] = __fmul_rn(A[i],A[j]);
		}
	}
	// for ATb
	for(i = 0; i<6; i++) ATb[i] = __fmul_rn(A[i],b);
	//////////////////////////////////////
	// for b
	//out_b[0] = td0; // squared point-to-point distance.
	// for inlier.
	//if(td0 > /*0.09**/in_th_icp) return false;
	//////////////////////////////////////

	return true;
}

__device__ bool d_cssfp_Compute_Single_Summand_with_Forward_Params(
	int x,int y,
	const float *in_map_depth_t1,
	const float *in_map_depth_t0,
	const float *in_map_vertex_t0,
	const float *in_map_normal_t0,
	const float *in_T_21_est,
	int in_lev_of_pyram,
	int ww,int hh,
	float in_th_icp,
	float *ATA,
	float *ATb)
	//float *out_b,
	//bool *flag_inlier)
{
	// for rigid transformation.
	Vector2f tpix,p2d_t0;	Vector3f tp3d,p3d_t1,p3d_t0,norm_t0;

	// for linear system.
	float A[6] ={0.0f}; float b = 0.0f;  // A = [A1 A2]
	float *p_K_dev = &K_dev[in_lev_of_pyram*4];

	float td0,td1;
	int tidx,i,j;
	bool flag;

	tpix.x = x; tpix.y = y;
	tidx = y*ww + x;

	// check depth validity of map t1.
	//if((td1 = in_map_depth_t1[tidx]) < 1.0e-6f) return false;
	td1 = in_map_depth_t1[tidx];
	if(td1 < 1.0e-6f) return false;

	// compute 3d point back-projected from current estimated pose in the global coordinates.
	// + [p3d_proj]
	d_bp_Back_Project(tpix,p_K_dev,td1,tp3d);	// 2D cam t1->3D cam t1
	d_t_Transform(tp3d,in_T_21_est,p3d_t1);	// 3D cam t1->3D cam t0	
	// compute 3d point predicted from the global model and previous camera pose in the global coordinates.
	// + [p3d_pred]
	d_p_Project(p3d_t1,p_K_dev,p2d_t0);		// 3D cam t0->2D cam t0
	flag = d_giv_Get_Interpolated_Vertex(p2d_t0,ww,hh,in_map_vertex_t0,p3d_t0);		// 2D cam t0->3D glob t0
	if(!flag) return false;

	// /////////////////////////////////////////////////////////////////////////////
	// check distance between [p3d_proj] and [p3d_pred].
	td0 = SQUARE(p3d_t0.x - p3d_t1.x) + SQUARE(p3d_t0.y - p3d_t1.y) + SQUARE(p3d_t0.z - p3d_t1.z);

	//////////////////////////////////////////////////////////////////////////
	if(td0 > in_th_icp)	return false;
	//////////////////////////////////////////////////////////////////////////

	// check correlation between normals of [p3d_proj] and [p3d_pred].
	// + compute normal of [p3d_proj].



	// + compute normal of [p3d_pred].
	//if(!d_gin_Get_Interpolated_Normal(p2d_pred,ww,hh,in_map_normal_t0,norm_pred_g)) return false;
	flag = d_gin_Get_Interpolated_Normal(p2d_t0,ww,hh,in_map_normal_t0,norm_t0);
	if(!flag) return false;

	//////////////////////////////////////////////////////////////////////////
	//norm_pred_g.x = norm_pred_g.y = 0.0f; norm_pred_g.z = -1.0f;
	//////////////////////////////////////////////////////////////////////////

	// generate 6x6 linear system by computing the derivative of the objective function (22).
	// refer to equation (24) of "KinectFusion: Real-Time Dense Surface Mapping and Tracking", ISMAR 2011.
	// + compute A.	

	A[0]= __fadd_rn(__fmul_rn(+p3d_t1.z,norm_t0.y),__fmul_rn(-p3d_t1.y,norm_t0.z));
	A[1]= __fadd_rn(__fmul_rn(-p3d_t1.z,norm_t0.x),__fmul_rn(+p3d_t1.x,norm_t0.z));
	A[2]= __fadd_rn(__fmul_rn(+p3d_t1.y,norm_t0.x),__fmul_rn(-p3d_t1.x,norm_t0.y));
	A[3]= -norm_t0.x;	A[4]= -norm_t0.y;	A[5]= -norm_t0.z;

	// + compute b.
	// b = Ng,t-1^T*(Vg,t-1 - Vg,t)
	//float tsum,b1,b2,b3;

	b = dot(norm_t0,(p3d_t0 - p3d_t1));

	// 	tp3d = p3d_pred - p3d_proj; b = 0.0f;
	// 	b = __fmaf_rn(norm_pred_g.x,tp3d.x,b);
	// 	b = __fmaf_rn(norm_pred_g.y,tp3d.y,b);
	// 	b = __fmaf_rn(norm_pred_g.z,tp3d.z,b);

	// update linear system.
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// This part is important.
	// Shared memory 써서 block 단위로 결과 모은다음 마지막에
	// reduction 써서 block 단위 sum 합쳐야 함.
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// copy ATA and ATb values of current thread to shared memory.
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// 예제의 경우 total thread number (=grid_dim * block dim) 이 vector 전체
	// size 보다 작기 때문에 이 부분이 필요하다.
	// 예제처럼 이렇게 while 문으로 control 하는 것이 필요한가?
	// 하나의 thread 값을 하나의 cache 에 넣으면 되는거 아님 그냥?	
	//while(tx<ww && ty<hh){

	// for ATA
	for(i = 0; i<6; i++) for(j = 0; j<6; j++){
		{
			ATA[i*6 + j] = __fmul_rn(A[i],A[j]);
		}
	}
	// for ATb
	for(i = 0; i<6; i++) ATb[i] = __fmul_rn(A[i],b);
	//////////////////////////////////////
	// for b
	//out_b[0] = td0; // squared point-to-point distance.
	// for inlier.
	//if(td0 > /*0.09**/in_th_icp) return false;
	//////////////////////////////////////

	return true;
}

__global__ void g_gls_Generate_Linear_System(
	float *out_ATA_partial,
	float *out_ATb_partial,
//	float *out_b_partial,
// 	int *out_num_val_partial,
// 	int *out_num_inlier_partial,
	int in_lev_of_pyram,
	const float *in_map_depth_t1,
	const float *in_map_depth_t0,
	const float *in_map_vertex_t0,
	const float *in_map_normal_t0,
	const float *in_T_cg_est)
{
	// for computing overall summation.
// 	__shared__ float cache_ATA[6*6*CV_CUDA_MAX_BLOCK_SIZE], cache_ATb[6*CV_CUDA_MAX_BLOCK_SIZE];
// 	__shared__ int cache_cnt[CV_CUDA_MAX_BLOCK_SIZE];
	
	const int tx = threadIdx.x + blockIdx.x*blockDim.x;
	const int ty = threadIdx.y + blockIdx.y*blockDim.y;

	const int cidx = threadIdx.x + threadIdx.y*blockDim.x;
	const int bidx = blockIdx.x + blockIdx.y*gridDim.x;

	__shared__ float cache[CV_CUDA_MAX_BLOCK_SIZE];

	// for etc. parameters.	
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// initialize cache values.
	cache[cidx] = 0.0f;
	__syncthreads();

	// for constant variables.
	//////////////////////////////////////////////////////////////////////////
	const int ww = dim_map_dev[in_lev_of_pyram*2 + 0];
	const int hh = dim_map_dev[in_lev_of_pyram*2 + 1];
	//////////////////////////////////////////////////////////////////////////
	const float th_icp = th_icp_dev[0];

	const int dim_ATA = 6*6,dim_ATb = 6;
	const int dim_block = blockDim.x*blockDim.y;

	// for linear system.
	float tATA[6*6],tATb[6],tb[1]={0.0f};
	bool flag_valid = false,tinlier[1]={false};

	// initialize linear system.
	for(int pidx = 0; pidx<dim_ATA; pidx++) tATA[pidx] = 0.0f;
	for(int pidx = 0; pidx<dim_ATb; pidx++) tATb[pidx] = 0.0f;

	if(tx >= 0 && tx < ww && ty >= 0 && ty < hh){
		flag_valid = d_cssfp_Compute_Single_Summand_with_Forward_Params(
			tx,ty,
			in_map_depth_t1,
			in_map_depth_t0,
			in_map_vertex_t0,
			in_map_normal_t0,
			in_T_cg_est,
			in_lev_of_pyram,
			ww,hh,
			th_icp,
			tATA,
			tATb);
			//tb,
			//tinlier);

		if(!flag_valid){
			for(int pidx = 0; pidx<dim_ATA; pidx++) tATA[pidx] = 0.0f;
			for(int pidx = 0; pidx<dim_ATb; pidx++) tATb[pidx] = 0.0f;
			//tb[0] = 0.0f;
			//tinlier[0] = false;
		}
	}

	// set cache values.
	// for ATA
	for(int pidx = 0; pidx<dim_ATA; pidx++){
		cache[cidx] = tATA[pidx];
		__syncthreads();

		int i = dim_block/2;	// half of total thread number per block.
		while(i!=0){
			if(cidx < i){
				// i is idx in cache memory.
				//cache[cidx] = __fadd_rn(cache[cidx],cache[cidx + i]);
				cache[cidx] += cache[cidx + i];
			}
			//////////////////////////////////////////////////////////////////////////
			if(i>32) __syncthreads();
			//////////////////////////////////////////////////////////////////////////
			i /= 2;
		}

		// Update local block sum from shared memory to global memory.
		// Size of global memory should be block number.
		if(cidx == 0)	out_ATA_partial[bidx*dim_ATA + pidx] = cache[0];
		__syncthreads();
	}	

	// for ATb
	for(int pidx = 0; pidx<6; pidx++){

		cache[cidx] = tATb[pidx];
		__syncthreads();

		int i = dim_block/2;	// half of total thread number per block.
		while(i!=0){
			if(cidx < i){
				// i is idx in cache memory.
				//cache[cidx] = __fadd_rn(cache[cidx],cache[cidx + i]);
				cache[cidx] += cache[cidx + i];
			}
			//////////////////////////////////////////////////////////////////////////
			if(i>32) __syncthreads();
			//////////////////////////////////////////////////////////////////////////
			i /= 2;
		}

		// Update local block sum from shared memory to global memory.
		// Size of global memory should be block number.
		if(cidx == 0)	out_ATb_partial[bidx*6 + pidx] = cache[0];
		__syncthreads();
	}
	//__syncthreads();

	// for b
// 	{
// 		cache[cidx] = tb[0];
// 		__syncthreads();
// 
// 		int i = dim_block/2;	// half of total thread number per block.
// 		while(i!=0){
// 			if(cidx < i){
// 				// i is idx in cache memory.
// 				//cache[cidx] = __fadd_rn(cache[cidx],cache[cidx + i]);
// 				cache[cidx] += cache[cidx + i];
// 			}
// 			//////////////////////////////////////////////////////////////////////////
// 			if(i>32) __syncthreads();
// 			//////////////////////////////////////////////////////////////////////////
// 			i /= 2;
// 		}
// 
// 		// Update local block sum from shared memory to global memory.
// 		// Size of global memory should be block number.
// 		//if(cidx == 0)	out_b_partial[bidx] = cache[0];
// 	}
// 	__syncthreads();

		
}

__global__ void g_glsbw_Generate_Linear_System_using_Backward_Warping(
	float *out_ATA_partial,
	float *out_ATb_partial,
//	float *out_b_partial,
// 	int *out_num_val_partial,
// 	int *out_num_inlier_partial,
	int in_lev_of_pyram,
	const float *in_map_depth_t1,
	const float *in_map_depth_t0,
	const float *in_map_vertex_t0,
	const float *in_map_normal_t0,
	const float *in_T_21_est)
{
	// for computing overall summation.
	// 	__shared__ float cache_ATA[6*6*CV_CUDA_MAX_BLOCK_SIZE], cache_ATb[6*CV_CUDA_MAX_BLOCK_SIZE];
	// 	__shared__ int cache_cnt[CV_CUDA_MAX_BLOCK_SIZE];

	const int tx = threadIdx.x + blockIdx.x*blockDim.x;
	const int ty = threadIdx.y + blockIdx.y*blockDim.y;

	const int cidx = threadIdx.x + threadIdx.y*blockDim.x;
	const int bidx = blockIdx.x + blockIdx.y*gridDim.x;

	__shared__ float cache[CV_CUDA_MAX_BLOCK_SIZE];

	// for etc. parameters.	
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// initialize cache values.
	cache[cidx] = 0.0f;
	__syncthreads();

	// for constant variables.
	//////////////////////////////////////////////////////////////////////////
	const int ww = dim_map_dev[in_lev_of_pyram*2 + 0];
	const int hh = dim_map_dev[in_lev_of_pyram*2 + 1];
	//////////////////////////////////////////////////////////////////////////
	const float th_icp = th_icp_dev[0];

	const int dim_ATA = 6*6,dim_ATb = 6;
	const int dim_block = blockDim.x*blockDim.y;

	// for linear system.
	float tATA[6*6],tATb[6],tb[1]={0.0f};
	bool flag_valid = false,tinlier[1]={false};

	// initialize linear system.
	for(int pidx = 0; pidx<dim_ATA; pidx++) tATA[pidx] = 0.0f;
	for(int pidx = 0; pidx<dim_ATb; pidx++) tATb[pidx] = 0.0f;

	if(tx < ww && ty < hh){
		flag_valid = d_cssfp_Compute_Single_Summand_with_Forward_Params(
			tx,ty,
			in_map_depth_t1,
			in_map_depth_t0,
			in_map_vertex_t0,
			in_map_normal_t0,
			in_T_21_est,
			in_lev_of_pyram,
			ww,hh,
			th_icp,
			tATA,
			tATb);
		//tb,
		//tinlier);

		if(!flag_valid){
			for(int pidx = 0; pidx<dim_ATA; pidx++) tATA[pidx] = 0.0f;
			for(int pidx = 0; pidx<dim_ATb; pidx++) tATb[pidx] = 0.0f;
			//tb[0] = 0.0f;
			//tinlier[0] = false;
		}
	}

	// set cache values.
	// for ATA
	for(int pidx = 0; pidx<dim_ATA; pidx++){
		cache[cidx] = tATA[pidx];
		__syncthreads();

		int i = dim_block/2;	// half of total thread number per block.
		while(i!=0){
			if(cidx < i){
				// i is idx in cache memory.
				//cache[cidx] = __fadd_rn(cache[cidx],cache[cidx + i]);
				cache[cidx] += cache[cidx + i];
			}
			//////////////////////////////////////////////////////////////////////////
			if(i>32) __syncthreads();
			//////////////////////////////////////////////////////////////////////////
			i /= 2;
		}

		// Update local block sum from shared memory to global memory.
		// Size of global memory should be block number.
		if(cidx == 0)	out_ATA_partial[bidx*dim_ATA + pidx] = cache[0];
		__syncthreads();
	}

	// for ATb
	for(int pidx = 0; pidx<6; pidx++){

		cache[cidx] = tATb[pidx];
		__syncthreads();

		int i = dim_block/2;	// half of total thread number per block.
		while(i!=0){
			if(cidx < i){
				// i is idx in cache memory.
				//cache[cidx] = __fadd_rn(cache[cidx],cache[cidx + i]);
				cache[cidx] += cache[cidx + i];
			}
			//////////////////////////////////////////////////////////////////////////
			if(i>32) __syncthreads();
			//////////////////////////////////////////////////////////////////////////
			i /= 2;
		}

		// Update local block sum from shared memory to global memory.
		// Size of global memory should be block number.
		if(cidx == 0)	out_ATb_partial[bidx*6 + pidx] = cache[0];
		__syncthreads();
	}
	//__syncthreads();

	// for b
	// 	{
	// 		cache[cidx] = tb[0];
	// 		__syncthreads();
	// 
	// 		int i = dim_block/2;	// half of total thread number per block.
	// 		while(i!=0){
	// 			if(cidx < i){
	// 				// i is idx in cache memory.
	// 				//cache[cidx] = __fadd_rn(cache[cidx],cache[cidx + i]);
	// 				cache[cidx] += cache[cidx + i];
	// 			}
	// 			//////////////////////////////////////////////////////////////////////////
	// 			if(i>32) __syncthreads();
	// 			//////////////////////////////////////////////////////////////////////////
	// 			i /= 2;
	// 		}
	// 
	// 		// Update local block sum from shared memory to global memory.
	// 		// Size of global memory should be block number.
	// 		//if(cidx == 0)	out_b_partial[bidx] = cache[0];
	// 	}
	// 	__syncthreads();


}

/////////////////////////////////////////////////////////////////////////////////////////////
// LGKvVolumeIntegrator
/////////////////////////////////////////////////////////////////////////////////////////////

// *******************************************************
__host__ LGKvPoseTracker::LGKvPoseTracker()
// *******************************************************
{
	for(int i=0; i<GK_LEVEL_OF_IMAGE_PYRAMID; i++){
		z_ATA_partial_dev[i].create(6, 6, 1); 
		z_ATb_partial_dev[i].create(6, 1, 1);
		z_b_partial_dev[i].create(6, 1, 1);
	}
	z_T_cg_est.create(4, 4, 1);
	z_T_cg_prev.create(4, 4, 1);

	z_mat_4x4.create(4, 4, 1);
	//z_T_cg_est1.create(4, 4, 1);

	z_ATA_partial_host = z_ATb_partial_host = z_b_partial_host = NULL;
	z_num_partial_host = z_num_inlier_partial_host = NULL;
}

// *******************************************************
__host__ LGKvPoseTracker::~LGKvPoseTracker()
// *******************************************************
{
	if(z_ATA_partial_host) delete[] z_ATA_partial_host;
	if(z_ATb_partial_host) delete[] z_ATb_partial_host;
	if(z_b_partial_host) delete[] z_b_partial_host;
	if(z_num_partial_host) delete[] z_num_partial_host;
	if(z_num_inlier_partial_host) delete[] z_num_inlier_partial_host;
}


// *******************************************************
__host__ void LGKvPoseTracker::ip_Initialize_Parameters(
	int ww, int hh,
	float fx, float fy,
	float px, float py,
	float th_icp)
// *******************************************************
{
// 	float intrins_host[4];
// 	float th_icp_host[1];
// 	int dim_map_host[2];
// 
// 	dim_map_host[0] = ww;	dim_map_host[1] = hh;
// 	intrins_host[0] = fx;	intrins_host[1] = fy;	intrins_host[2] = px;	intrins_host[3] = py;
// 	th_icp_host[0] = th_icp;		
// 
// 	cudaMemcpyToSymbol(dim_map_dev, dim_map_host, 2 * sizeof(int));
// 	cudaMemcpyToSymbol(K_dev, intrins_host, 4 * sizeof(float));	
// 	cudaMemcpyToSymbol(th_icp_dev, th_icp_host, sizeof(float)); 

	//////////////////////////////////////////////////////////////////////////
	float intrins_host[GK_LEVEL_OF_IMAGE_PYRAMID*4];
	int dim_map_host[GK_LEVEL_OF_IMAGE_PYRAMID*2];
	float th_icp_host[1];

	//printf("dim_cube_host: %d %d %d\n", dim_cube_host[0], dim_cube_host[1], dim_cube_host[2]);
	int tww,thh;	tww = ww; thh = hh;
	float *p_intrins = &intrins_host[0];
	int *p_dim_maps = &dim_map_host[0];

	for(int k=0; k<GK_LEVEL_OF_IMAGE_PYRAMID; k++){

		p_intrins[0] = fx;	p_intrins[1] = fy;	p_intrins[2] = px;	p_intrins[3] = py;
		p_dim_maps[0] = tww;	p_dim_maps[1] = thh;

		// downsizing.
		fx = 0.5f*fx; fy = 0.5f*fy; px = 0.5f*(px - 0.5f); py = 0.5f*(py - 0.5f);
		tww /= 2; thh /= 2;

		p_intrins += 4;
		p_dim_maps += 2;

	}

	th_icp_host[0] = th_icp;

	cudaMemcpyToSymbol(K_dev,intrins_host,GK_LEVEL_OF_IMAGE_PYRAMID*4 * sizeof(float));
	cudaMemcpyToSymbol(dim_map_dev,dim_map_host,GK_LEVEL_OF_IMAGE_PYRAMID*2 * sizeof(int));
	cudaMemcpyToSymbol(th_icp_dev,th_icp_host,sizeof(float));

	
}

// *******************************************************
// Frame-to-model depth tracking.
// Backward warping with forward parameterization.
__host__ bool LGKvPoseTracker::tp_Track_Pose(
	GKvTrackingState *io_track_state,
	GKvRgbdFrame *in_rgbd_frame,
	GKvMatrixFloat *in_pose_init_t0_t1)
// *******************************************************
{
	//static int iter_num[4] ={5,3,3,3};
	static int iter_num[4] ={10,7,7,7};
	static int th_num[4] ={500,300,100,100};
	static float T_12_est_host[16], T_21_est_host[16], T_gc_est_host[16];
	static float T_12_init_host[16], T_tmp[16];

	// current pose to estimate.
	float *p_T_21_est_dev = z_T_cg_est.vp();
	float *T_cg_est_dev;

	//float *ATA_partial_dev,*ATb_partial_dev,*b_partial_dev;
	//int *num_partial_dev,*num_inlier_partial_dev;

	Vector3f cent_prev,cent_est;
	Vector2i sz_lv0 = io_track_state->sz_map;
	float *p_T_gc_dev = io_track_state->vp_T_gc();
	float *p_T_cg_dev = io_track_state->vp_T_cg();

	bool flag_valid;

	//////////////////////////////////////////////////////////////////////////
	int lev_of_pyram = GK_LEVEL_OF_IMAGE_PYRAMID;
	//////////////////////////////////////////////////////////////////////////

	//cudaMemcpy(T_12_est_host,p_T_cg_dev,16*sizeof(float),cudaMemcpyDeviceToHost);
	//cudaMemcpy(T_21_est_host,p_T_gc_dev,16*sizeof(float),cudaMemcpyDeviceToHost);

	// set identity matrix.
	for(int i=0;i<16;i++)	T_12_est_host[i] = T_21_est_host[i] = (i%4 == i/4) ? 1.0f : 0.0f;		

	//d_pm_Printf_Matrix(T_12_est_host, 4, 4, "T_12");
	
	//////////////////////////////////////////////////////////////////////////
	// get initial pose.
	if(in_pose_init_t0_t1){
		cudaMemcpy(T_12_init_host,in_pose_init_t0_t1->vp(),16*sizeof(float),cudaMemcpyDeviceToHost);
		d_cv_Copy_Vector(T_12_init_host, 16, T_12_est_host);
		d_im_Inverse_Matrix_4x4(T_12_init_host, T_21_est_host);
		//cudaMemcpy(z_mat_4x4.vp(),T_tmp,16*sizeof(float),cudaMemcpyHostToDevice);

		//io_track_state->set_transform(&z_mat_4x4);

		lev_of_pyram = 1;
	}
	 cudaMemcpy(p_T_21_est_dev,T_21_est_host,16*sizeof(float),cudaMemcpyHostToDevice);
	//////////////////////////////////////////////////////////////////////////

	// Camera pose at t0.
//	cudaMemcpyToSymbol(T_gc_dev_const,p_T_gc_dev,sizeof(float) * 16,0,cudaMemcpyDeviceToDevice);
//	cudaMemcpyToSymbol(T_cg_dev_const,p_T_cg_dev,sizeof(float) * 16,0,cudaMemcpyDeviceToDevice);
	// Camera pose going to be estimated in current step.
	//cudaMalloc((void**)&T_cg_est_dev, 16*sizeof(float));
// 	cudaMemcpy(p_T_21_est_dev,p_T_cg_dev,16*sizeof(float),cudaMemcpyDeviceToDevice);
// 	cudaMemcpy(T_12_est_host,p_T_21_est_dev,16*sizeof(float),cudaMemcpyDeviceToHost);
	// Previous camera center.
// 	cudaMemcpy(z_T_cg_prev.vp(),p_T_cg_dev,16*sizeof(float),cudaMemcpyDeviceToDevice);
// 	d_gcc_Get_Camera_Center(T_12_est_host,cent_prev);

	// Cuda kernel.
	dim3 threads_lv0(CV_CUDA_BLOCK_SIZE_X,CV_CUDA_BLOCK_SIZE_Y);
	dim3 blocks_lv0(iDivUp(sz_lv0.x,threads_lv0.x),iDivUp(sz_lv0.y,threads_lv0.y));
	int dim_block_lv0 = blocks_lv0.x*blocks_lv0.y;
	
//	printf("dim_block_lv0: %d\n", dim_block_lv0);

// 	if(z_ATA_partial_dev.nch() != dim_block_lv0)	z_ATA_partial_dev.create(6,6,dim_block_lv0);
// 	if(z_ATb_partial_dev.nch() != dim_block_lv0)	z_ATb_partial_dev.create(6,1,dim_block_lv0);
// 	if(z_b_partial_dev.nch() != dim_block_lv0)	z_b_partial_dev.create(1,1,dim_block_lv0);

	//cudaMalloc((void**)&ATA_partial_dev,6*6*dim_block*sizeof(float));
	//cudaMalloc((void**)&ATb_partial_dev,6*dim_block*sizeof(float));
	//cudaMalloc((void**)&b_partial_dev,dim_block*sizeof(float));
// 	cudaMalloc((void**)&num_partial_dev,dim_block_lv0*sizeof(int));
// 	cudaMalloc((void**)&num_inlier_partial_dev,dim_block_lv0*sizeof(int));

	if(!z_ATA_partial_host) z_ATA_partial_host = new float[6*6*dim_block_lv0];
	if(!z_ATb_partial_host) z_ATb_partial_host = new float[6*dim_block_lv0];
// 	if(!z_b_partial_host) z_b_partial_host = new float[dim_block_lv0];
// 	if(!z_num_partial_host) z_num_partial_host = new int[dim_block_lv0];
// 	if(!z_num_inlier_partial_host) z_num_inlier_partial_host = new int[dim_block_lv0];

	flag_valid = true;

	for(int n=lev_of_pyram-1; n>=0; n--){

		const float *map_d_t1 = in_rgbd_frame->vp_map_depth(n);
		const float *map_d_t0 = io_track_state->vp_map_depth(n);
		const float *map_ver_t0 = io_track_state->vp_map_vertex(n);
		const float *map_norm_t0 = io_track_state->vp_map_normal(n);

		Vector2i sz = sz_lv0/pow(2,n);
		dim3 threads(CV_CUDA_BLOCK_SIZE_X,CV_CUDA_BLOCK_SIZE_Y);
		dim3 blocks(iDivUp(sz.x,threads.x),iDivUp(sz.y,threads.y));
		int dim_block = blocks.x*blocks.y;

		if(z_ATA_partial_dev[n].nch() != dim_block)	z_ATA_partial_dev[n].create(6,6,dim_block);
		if(z_ATb_partial_dev[n].nch() != dim_block)	z_ATb_partial_dev[n].create(6,1,dim_block);

		for(int i=0; i<iter_num[n]; i++){

			if(!flag_valid) break;

// 			g_gls_Generate_Linear_System<<<blocks,threads>>>(
// 				z_ATA_partial_dev[n].vp(),z_ATb_partial_dev[n].vp(),
// 				//z_b_partial_dev[n].vp(),
// 				//ATA_partial_dev,ATb_partial_dev,b_partial_dev,
// 				//num_partial_dev,num_inlier_partial_dev,
// 				n,
// 				map_d_t1,
// 				map_d_t0,map_ver_t0,map_norm_t0,
// 				p_T_cg_est_dev);

			g_glsbw_Generate_Linear_System_using_Backward_Warping<<<blocks,threads>>>(
			 	z_ATA_partial_dev[n].vp(),z_ATb_partial_dev[n].vp(),
			 	//z_b_partial_dev[n].vp(),
			 	//ATA_partial_dev,ATb_partial_dev,b_partial_dev,
			 	//num_partial_dev,num_inlier_partial_dev,
			 	n,
			 	map_d_t1,
			 	map_d_t0,map_ver_t0,map_norm_t0,
			 	p_T_21_est_dev);


			cudaMemcpy(z_ATA_partial_host,z_ATA_partial_dev[n].vp(),6*6*dim_block*sizeof(float),cudaMemcpyDeviceToHost); //z_ATA_partial_dev.vp(), 
			cudaMemcpy(z_ATb_partial_host,z_ATb_partial_dev[n].vp(),6*dim_block*sizeof(float),cudaMemcpyDeviceToHost); //z_ATb_partial_dev.vp(),
			//cudaMemcpy(z_b_partial_host,z_b_partial_dev[n].vp(),dim_block*sizeof(float),cudaMemcpyDeviceToHost); //z_ATb_partial_dev.vp(),
			
			//cudaMemcpy(z_ATA_partial_host,ATA_partial_dev,6*6*dim_block*sizeof(float),cudaMemcpyDeviceToHost); //z_ATA_partial_dev.vp(), 
			//cudaMemcpy(z_ATb_partial_host,ATb_partial_dev,6*dim_block*sizeof(float),cudaMemcpyDeviceToHost); //z_ATb_partial_dev.vp(),
			//cudaMemcpy(z_b_partial_host,b_partial_dev,dim_block*sizeof(float),cudaMemcpyDeviceToHost); //z_ATb_partial_dev.vp(),
			
			//cudaMemcpy(z_num_partial_host,num_partial_dev,dim_block*sizeof(int),cudaMemcpyDeviceToHost); //z_ATb_partial_dev.vp(),
			//cudaMemcpy(z_num_inlier_partial_host,num_inlier_partial_dev,dim_block*sizeof(int),cudaMemcpyDeviceToHost); //z_ATb_partial_dev.vp(),

			// Add partial sums.
			for(int k=0; k<6*6; k++) z_ATA[k] = 0.0f;
			for(int k=0; k<6; k++) z_ATb[k] = 0.0f;
// 			z_b = 0.0f;
// 			z_num_val = 0;
// 			z_num_inlier = 0;

			for(int k = 0; k<dim_block; k++){
				for(int j = 0; j<6*6; j++) z_ATA[j] += z_ATA_partial_host[k*6*6 + j];
				for(int j = 0; j<6; j++) z_ATb[j] += z_ATb_partial_host[k*6 + j];
// 				z_b += z_b_partial_host[k];     // squared point-to-point distance
// 				z_num_val += z_num_partial_host[k];
// 				z_num_inlier += z_num_inlier_partial_host[k];
			}

			d_pm_Printf_Matrix(z_ATA, 6, 6);

// 			printf("(#%d) z_ATb: %f %f %f | %f %f %f\n",
// 			z_num_val,z_ATb[0],z_ATb[1],z_ATb[2]
// 			,z_ATb[3],z_ATb[4],z_ATb[5]);

			// Check number of valid pixels.		
// 			if(z_num_val < th_num[n]){
// 				flag_valid = false;
// 				break;
// 			}

			// solve linear system.
			float norm2_x;
			d_lld_LL_Decomposition(z_ATA,6,z_L);

// 			d_pm_Printf_Matrix(z_ATA,6,6,"ATA");
// 			d_pm_Printf_Matrix(z_L,6,6,"L");

			//if(!d_sls_Solve_Linear_System_using_LLD(z_L,z_ATb,z_y,6,z_sol_x)) { flag_valid = false; break; }
			flag_valid = d_sls_Solve_Linear_System_using_LLD(z_L,z_ATb,z_y,6,z_sol_x);
			if(!flag_valid) break;

			// Check 2-norm of solution vector x.
			// 		d_n2v_Norm_2_Vector(z_sol_x,6,norm2_x);
			// 		if(norm2_x < 6.0e-3) break;
			float norm_inf;	d_niv_Norm_Infinity_Vector(z_sol_x,6,norm_inf);
			if(norm_inf < 5.0e-7f) break;

			// Update incremental transformation. (T_cg_est_dev)
			//z_uit_Update_Incremental_Tracking(z_sol_x,T_cg_est_host);
			z_uitme_Update_Incremental_Tracking_with_Matrix_Exponential(z_sol_x,T_12_est_host);
			d_im_Inverse_Matrix_4x4(T_12_est_host, T_21_est_host);
			cudaMemcpy(p_T_21_est_dev,T_21_est_host,16*sizeof(float),cudaMemcpyHostToDevice);

			// check motion validity.
			//	double translation = norm(Rt(Rect(3,0,1,3)));
			//	double rotation = norm(rvec) * 180. / CV_PI;

			// 		for(int j = 0; j<6; j++) printf("%f ", z_sol_x[j]);
			// 		printf("\n");
			//for(int k=0; k<6*6; k++)
		}
	}

	

	if(flag_valid){


		cudaMemcpy(T_tmp,p_T_gc_dev,16*sizeof(float),cudaMemcpyDeviceToHost);
		d_mmm_Multiply_Matrix_Matrix(T_12_est_host, T_tmp, 4, 4, 4, T_gc_est_host);
		cudaMemcpy(z_mat_4x4.vp(),T_gc_est_host,16*sizeof(float),cudaMemcpyHostToDevice);
		io_track_state->set_transform(&z_mat_4x4,false);

		///////////////////////////////////////////////////////////////////////////////////
		// 지금 rmse 에서 Euclidean distance 만 보는데 normal vector 차이도 보도록 해보자....
		// 지금 rmse 에서 Euclidean distance 만 보는데 normal vector 차이도 보도록 해보자....
		// 지금 rmse 에서 Euclidean distance 만 보는데 normal vector 차이도 보도록 해보자....
		// detect motion drift.
		// Estimated current camera center.
		float dist,rmse_surf,inlier_perc;
		//cent_est = io_track_state->center;
		//dist = length(cent_est - cent_prev);

		rmse_surf = sqrt(float(z_b)/(float)z_num_val);
		inlier_perc = 100.0f*float(z_num_inlier)/float(z_num_val);
		//printf(" >>> rmse_surf: %f mm (%% %5.2f)\n",rmse_surf,inlier_perc);

		//if(dist > 0.175f){  // 0.5m*20(deg)*PI/180.0f.
		// 		if(rmse_surf > 0.05f || inlier_perc < 95.0f){  // 0.5m*20(deg)*PI/180.0f.
		// 			io_track_state->set_transform(&z_T_cg_prev,true);
		// 			printf("=================================== Drift! ==========================================");
		// 			flag_valid = false;
		// 		}
		///////////////////////////////////////////////////////////////////////////////////

		// Transform from camera to global coordinates.
		//  		cudaMemcpy(p_T_cg_dev, p_T_cg_est_dev, 16*sizeof(float), cudaMemcpyDeviceToDevice);
		//  		// Transform from global to camera coordinates.
		//  		d_im_Inverse_Matrix_4x4(T_cg_est_host, T_gc_est_host);
		//  		cudaMemcpy(p_T_gc_dev, T_gc_est_host, 16*sizeof(float), cudaMemcpyHostToDevice);
		//  		// Update camera center.
		//  		d_gcc_Get_Camera_Center(T_gc_est_host, io_track_state->center);
	}


	//cudaFree(ATA_partial_dev);
	//cudaFree(ATb_partial_dev);
	//cudaFree(b_partial_dev);

// 	cudaFree(num_partial_dev);
// 	cudaFree(num_inlier_partial_dev);

	//cudaFree(T_cg_est_dev);

	return flag_valid;
}

////********************************************************************************************
//__host__ bool cmv_Check_Motion_Validity(
//	CKvMatrixFloat *io_hmat_4x4,
//	float in_max_translation,	// m
//	float in_max_rotation)		// rad.
////********************************************************************************************
//{
//	Mat Rt, rvec;
//	aa_ilib.cfko_Convert_Format_from_KAISION_to_Opencv(*io_hmat_4x4, Rt);
//
//	Rodrigues(Rt(Rect(0, 0, 3, 3)), rvec);
//
//	double translation = norm(Rt(Rect(3,0,1,3)));
//	double rotation = norm(rvec) * 180. / CV_PI;
//
//	//printf("trans: %f rot: %f\n", translation, rotation);
//
//	return translation <= in_max_translation && rotation <= in_max_rotation;
//}