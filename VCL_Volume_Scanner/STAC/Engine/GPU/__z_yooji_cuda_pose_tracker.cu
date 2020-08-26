/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_cuda_volume_integrator.cpp
/////////////////////////////////////////////////////////////////////////////////////////////
//#include "_yooji_2017_cuda_object_scanner.cuh"
//#define __CUDASCAN__
#include "../../_yooji_2017_cuda_object_scanner.cuh"

__constant__ float K_dev[4];
__constant__ float T_gc_dev_c[16];
__constant__ float T_cg_dev_c[16];
__constant__ int dim_map_dev[2];

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

// ===============================================================================
// Device functions.
// ===============================================================================
__device__ bool d_css_Compute_Single_Summand(
	int x,int y,
	const float *in_map_depth_t1,
	const float *in_map_depth_t0,
	const float *in_map_vertex_t0,
	const float *in_map_normal_t0,
	const float *in_T_cg_est,
	int ww,int hh,
	float in_th_icp,
	float *ATA,
	float *ATb,
	float *out_b)
{
	// for rigid transformation.
	Vector2f tpix,p2d_pred;	Vector3f tp3d,p3d_proj,p3d_pred,norm_pred_g;

	// for linear system.
	Vector3f A1,A2;
	float A[6] ={0.0f}; float b = 0.0f;  // A = [A1 A2]

	float td0,td1;
	int tidx,i,j;

	tpix.x = x; tpix.y = y;
	tidx = y*ww + x;

	// check depth validity of map t1.
	if((td1 = in_map_depth_t1[tidx]) < 1e-8f) return false;

	// compute 3d point back-projected from current estimated pose in the global coordinates.
	// + [p3d_proj]
	d_bp_Back_Project(tpix,K_dev,td1,tp3d);
	d_t_Transform(tp3d,in_T_cg_est,p3d_proj);
	// compute 3d point predicted from the global model and previous camera pose in the global coordinates.
	// + [p3d_pred]
	d_t_Transform(p3d_proj,T_gc_dev_c,tp3d);
	d_p_Project(tp3d,K_dev,p2d_pred);
	// + check depth validity.
	//	if(!d_gid_Get_Interpolated_Depth(p2d_pred,ww,hh,in_map_depth_t0,td0)) return false;
	// 	d_bp_Back_Project(p2d_pred,K_dev,td0,tp3d);
	if(!d_giv_Get_Interpolated_Vertex(p2d_pred,ww,hh,in_map_vertex_t0,tp3d)) return false;
	d_t_Transform(tp3d,T_cg_dev_c,p3d_pred);

	// /////////////////////////////////////////////////////////////////////////////
	// check distance between [p3d_proj] and [p3d_pred].
	td0 = SQUARE(p3d_pred.x - p3d_proj.x) + SQUARE(p3d_pred.y - p3d_proj.y) + SQUARE(p3d_pred.z - p3d_proj.z);
	//if(sqrtf(td0) > in_th_icp)	return false;
	if(td0 > in_th_icp)	return false;

	// check correlation between normals of [p3d_proj] and [p3d_pred].
	// + compute normal of [p3d_proj].

	// + compute normal of [p3d_pred].
	if(!d_gin_Get_Interpolated_Normal(p2d_pred,ww,hh,in_map_normal_t0,norm_pred_g)) return false;

	// generate 6x6 linear system by computing the derivative of the objective function (22).
	// refer to equation (24) of "KinectFusion: Real-Time Dense Surface Mapping and Tracking", ISMAR 2011.
	// + compute A.	

	A[0]= __fadd_rn(__fmul_rn(p3d_proj.z,norm_pred_g.y),__fmul_rn(-p3d_proj.y,norm_pred_g.z));
	A[1]= __fadd_rn(__fmul_rn(-p3d_proj.z,norm_pred_g.x),__fmul_rn(p3d_proj.x,norm_pred_g.z));
	A[2]= __fadd_rn(__fmul_rn(p3d_proj.y,norm_pred_g.x),__fmul_rn(-p3d_proj.x,norm_pred_g.y));
	A[3]= norm_pred_g.x;	A[4]= norm_pred_g.y;	A[5]= norm_pred_g.z;

	// + compute b.
	// b = Ng,t-1^T*(Vg,t-1 - Vg,t)
	float tsum,b1,b2,b3;

	//b = dot(norm_pred, (p3d_pred - p3d_proj));

	tp3d = p3d_pred - p3d_proj; b = 0.0f;
	b = __fmaf_rn(norm_pred_g.x,tp3d.x,b);
	b = __fmaf_rn(norm_pred_g.y,tp3d.y,b);
	b = __fmaf_rn(norm_pred_g.z,tp3d.z,b);

	// update linear system.
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
	out_b[0] = td0; // squared point-to-point distance.
	//////////////////////////////////////

	return true;
}

// __device__ bool d_css_Compute_Single_Summand(
// 	int x,int y,
// 	const float *in_map_depth_t1,
// 	const float *in_map_depth_t0,
// 	const float *in_map_vertex_t0,
// 	const float *in_map_normal_t0,
// 	const float *in_T_cg_est,
// 	int ww,int hh,
// 	float in_th_icp,
// 	float *ATA,
// 	float *ATb,
// 	float *out_b,
// 	bool *flag_inlier)
// {
// 	// for rigid transformation.
// 	Vector2f tpix,p2d_pred;	Vector3f tp3d,p3d_proj,p3d_pred,norm_pred_g;
// 
// 	// for linear system.
// 	Vector3f A1,A2;
// 	float A[6] ={0.0f}; float b = 0.0f;  // A = [A1 A2]
// 
// 	float td0,td1;
// 	int tidx,i,j;
// 
// 	tpix.x = x; tpix.y = y;
// 	tidx = y*ww + x;
// 
// 	// check depth validity of map t1.
// 	if((td1 = in_map_depth_t1[tidx]) < 1e-8f) return false;
// 
// 	// compute 3d point back-projected from current estimated pose in the global coordinates.
// 	// + [p3d_proj]
// 	d_bp_Back_Project(tpix,K_dev,td1,tp3d);
// 	d_t_Transform(tp3d,in_T_cg_est,p3d_proj);
// 	// compute 3d point predicted from the global model and previous camera pose in the global coordinates.
// 	// + [p3d_pred]
// 	d_t_Transform(p3d_proj,T_gc_dev_c,tp3d);
// 	d_p_Project(tp3d,K_dev,p2d_pred);
// 	// + check depth validity.
// //	if(!d_gid_Get_Interpolated_Depth(p2d_pred,ww,hh,in_map_depth_t0,td0)) return false;
// // 	d_bp_Back_Project(p2d_pred,K_dev,td0,tp3d);
// 	if(!d_giv_Get_Interpolated_Vertex(p2d_pred,ww,hh,in_map_vertex_t0,tp3d)) return false;
// 	d_t_Transform(tp3d,T_cg_dev_c,p3d_pred);
// 
// 	// /////////////////////////////////////////////////////////////////////////////
// 	// check distance between [p3d_proj] and [p3d_pred].
// 	td0 = SQUARE(p3d_pred.x - p3d_proj.x) + SQUARE(p3d_pred.y - p3d_proj.y) + SQUARE(p3d_pred.z - p3d_proj.z);
// 	//if(sqrtf(td0) > in_th_icp)	return false;
// 	if(td0 > in_th_icp)	return false;
// 
// 	// check correlation between normals of [p3d_proj] and [p3d_pred].
// 	// + compute normal of [p3d_proj].
// 
// 
// 
// 	// + compute normal of [p3d_pred].
// 	if(!d_gin_Get_Interpolated_Normal(p2d_pred,ww,hh,in_map_normal_t0,norm_pred_g)) return false;
// 
// 	// generate 6x6 linear system by computing the derivative of the objective function (22).
// 	// refer to equation (24) of "KinectFusion: Real-Time Dense Surface Mapping and Tracking", ISMAR 2011.
// 	// + compute A.	
// 
// 	A[0]= __fadd_rn(__fmul_rn(p3d_proj.z,norm_pred_g.y),__fmul_rn(-p3d_proj.y,norm_pred_g.z));
// 	A[1]= __fadd_rn(__fmul_rn(-p3d_proj.z,norm_pred_g.x),__fmul_rn(p3d_proj.x,norm_pred_g.z));
// 	A[2]= __fadd_rn(__fmul_rn(p3d_proj.y,norm_pred_g.x),__fmul_rn(-p3d_proj.x,norm_pred_g.y));
// 	A[3]= norm_pred_g.x;	A[4]= norm_pred_g.y;	A[5]= norm_pred_g.z;
// 
// 	// + compute b.
// 	// b = Ng,t-1^T*(Vg,t-1 - Vg,t)
// 	float tsum,b1,b2,b3;
// 
// 	//b = dot(norm_pred, (p3d_pred - p3d_proj));
// 
// 	tp3d = p3d_pred - p3d_proj; b = 0.0f;
// 	b = __fmaf_rn(norm_pred_g.x,tp3d.x,b);
// 	b = __fmaf_rn(norm_pred_g.y,tp3d.y,b);
// 	b = __fmaf_rn(norm_pred_g.z,tp3d.z,b);
// 
// 	// update linear system.
// 	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// 	// This part is important.
// 	// Shared memory 써서 block 단위로 결과 모은다음 마지막에
// 	// reduction 써서 block 단위 sum 합쳐야 함.
// 	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// 	// copy ATA and ATb values of current thread to shared memory.
// 	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// 	// 예제의 경우 total thread number (=grid_dim * block dim) 이 vector 전체
// 	// size 보다 작기 때문에 이 부분이 필요하다.
// 	// 예제처럼 이렇게 while 문으로 control 하는 것이 필요한가?
// 	// 하나의 thread 값을 하나의 cache 에 넣으면 되는거 아님 그냥?	
// 	//while(tx<ww && ty<hh){
// 
// 	// for ATA
// 	for(i = 0; i<6; i++) for(j = 0; j<6; j++){
// 		{
// 			ATA[i*6 + j] = __fmul_rn(A[i],A[j]);
// 		}
// 	}
// 	// for ATb
// 	for(i = 0; i<6; i++) ATb[i] = __fmul_rn(A[i],b);
// 	//////////////////////////////////////
// 	// for b
// 	out_b[0] = td0; // squared point-to-point distance.
// 	// for inlier.
// 	if(td0 < /*0.09**/in_th_icp) flag_inlier[0] = true;
// 	else flag_inlier[0] = false;
// 	//////////////////////////////////////
// 
// 	return true;
// }
__device__ bool d_css_Compute_Single_Summand(
	int x, int y, 
	const float *in_map_depth_t1,
	const float *in_map_depth_t0,
	const float *in_map_normal_t0,
	const float *in_T_cg_est,
	int ww, int hh,
	float in_th_icp,
	float *ATA,
	float *ATb,
	float *out_b,
	bool *flag_inlier)
{
	// for rigid transformation.
	Vector2f tpix, p2d_pred;	Vector3f tp3d, p3d_curr_g, p3d_prev_g, norm_pred_g;

	// for linear system.
	Vector3f A1,A2; 
	float A[6] ={0.0f}; float b = 0.0f;  // A = [A1 A2]

	float td0, td1;
	int tidx, i, j;

	tpix.x = x; tpix.y = y;
	tidx = y*ww + x;

	// check depth validity of map t1.
	if((td1 = in_map_depth_t1[tidx]) < 1e-8f) return false;

	// compute 3d point back-projected from current estimated pose in the global coordinates.
	// + [p3d_proj]
	d_bp_Back_Project(tpix, K_dev, td1, tp3d);
	d_t_Transform(tp3d, in_T_cg_est, p3d_curr_g);

	// compute 3d point predicted from the global model and previous camera pose in the global coordinates.
	// + [p3d_pred]
	d_t_Transform(p3d_curr_g,T_gc_dev_c,tp3d);
	d_p_Project(tp3d,K_dev,p2d_pred);

	// compute 3d point predicted from the global model and previous camera pose in the global coordinates.
	// + [p3d_pred]
	// with interpolation.
	if(!d_gid_Get_Interpolated_Depth(p2d_pred, ww, hh, in_map_depth_t0, td0)) return false;	
	d_bp_Back_Project(p2d_pred, K_dev, td0, tp3d);
	// without interpolation.
// 	i = __float2int_rn(p2d_pred.x);	j = __float2int_rn(p2d_pred.y);
// 	td0 = in_map_depth_t0[j*ww + i];
// 	p2d_pred.x = i; p2d_pred.y = j;
// 	d_bp_Back_Project(p2d_pred,K_dev,td0,tp3d);

	d_t_Transform(tp3d, T_cg_dev_c, p3d_prev_g);

	// /////////////////////////////////////////////////////////////////////////////
	// check distance between [p3d_proj] and [p3d_pred].
	td0 = SQUARE(p3d_prev_g.x - p3d_curr_g.x) + SQUARE(p3d_prev_g.y - p3d_curr_g.y) + SQUARE(p3d_prev_g.z - p3d_curr_g.z);
	//if(sqrtf(td0) > in_th_icp)	return false;
	if(td0 > in_th_icp)	return false;


	// /////////////////////////////////////////////////////////////////////////////

	// /////////////////////////////////////////////////////////////////////////////
	// /////////////////////////////////////////////////////////////////////////////


	// check correlation between normals of [p3d_proj] and [p3d_pred].
	// + compute normal of [p3d_proj].
	// 추가 !!!!!
	// 추가 !!!!!
	// 추가 !!!!!
	// 추가 !!!!!

	// + compute normal of [p3d_pred].
	if(!d_gin_Get_Interpolated_Normal(p2d_pred, ww, hh, in_map_normal_t0, norm_pred_g)) return false;

	// generate 6x6 linear system by computing the derivative of the objective function (22).
	// refer to equation (24) of "KinectFusion: Real-Time Dense Surface Mapping and Tracking", ISMAR 2011.
	// + compute A.	
	// A = [(Vg,t)x|I]^T*Ng,t-1
	// A1 = -(Vg,t)x*Ng | A2 = Ng,t-1.	
// 	A1 = -cross(p3d_proj, norm_pred); 	A2 = norm_pred;
// 	A[0] = A1.x; A[1] = A1.y; A[2] = A1.z;
// 	A[3] = A2.x; A[4] = A2.y; A[5] = A2.z;

	A[0]= __fadd_rn(__fmul_rn(p3d_curr_g.z,norm_pred_g.y),__fmul_rn(-p3d_curr_g.y,norm_pred_g.z));
	A[1]= __fadd_rn(__fmul_rn(-p3d_curr_g.z,norm_pred_g.x),__fmul_rn(p3d_curr_g.x,norm_pred_g.z));
	A[2]= __fadd_rn(__fmul_rn(p3d_curr_g.y,norm_pred_g.x),__fmul_rn(-p3d_curr_g.x,norm_pred_g.y));
	A[3]= norm_pred_g.x;	A[4]= norm_pred_g.y;	A[5]= norm_pred_g.z;

// 	A[0]= 0.0f; A[0] = __fmaf_rn(p3d_proj.z,norm_pred_g.y,A[0]); A[0] = __fmaf_rn(-p3d_proj.y,norm_pred_g.z,A[0]);
// 	A[1]= 0.0f; A[1] = __fmaf_rn(-p3d_proj.z,norm_pred_g.x,A[1]); A[1] = __fmaf_rn(p3d_proj.x,norm_pred_g.z,A[1]);
// 	A[2]= 0.0f; A[2] = __fmaf_rn(p3d_proj.y,norm_pred_g.x,A[2]); A[2] = __fmaf_rn(-p3d_proj.x,norm_pred_g.y,A[2]);
// 	A[3] = norm_pred_g.x;	A[4] = norm_pred_g.y;	A[5] = norm_pred_g.z;

// 	 	A[0] = +p3d_proj.z*norm_pred.y -p3d_proj.y*norm_pred.z;
// 	 	A[1] = -p3d_proj.z*norm_pred.x +p3d_proj.x*norm_pred.z;
// 	 	A[2] = +p3d_proj.y*norm_pred.x -p3d_proj.x*norm_pred.y;
// 	 	A[3] = norm_pred.x;	A[4] = norm_pred.y;	A[5] = norm_pred.z;

	// + compute b.
	// b = Ng,t-1^T*(Vg,t-1 - Vg,t)
	float tsum,b1,b2,b3;

	//b = dot(norm_pred, (p3d_pred - p3d_proj));

 	tp3d = p3d_prev_g - p3d_curr_g; b = 0.0f; 
	b = __fmaf_rn(norm_pred_g.x,tp3d.x,b); 
	b = __fmaf_rn(norm_pred_g.y,tp3d.y,b); 
	b = __fmaf_rn(norm_pred_g.z,tp3d.z,b);

// 	b1 = __fmul_rn(norm_pred_g.x,__fadd_rn(p3d_pred.x,-p3d_proj.x));
// 	b2 = __fmul_rn(norm_pred_g.y,__fadd_rn(p3d_pred.y,-p3d_proj.y));
// 	b3 = __fmul_rn(norm_pred_g.z,__fadd_rn(p3d_pred.z,-p3d_proj.z));
// 	tsum = __fadd_rn(b1,b2); b = __fadd_rn(tsum,b3);

// 		b = norm_pred.x*(p3d_pred.x-p3d_proj.x)
// 		+norm_pred.y*(p3d_pred.y-p3d_proj.y)
// 		+norm_pred.z*(p3d_pred.z-p3d_proj.z);

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
	out_b[0] = td0; // squared point-to-point distance.
	// for inlier.
	if(td0 < /*0.09**/in_th_icp) flag_inlier[0] = true;
	else flag_inlier[0] = false;
	//////////////////////////////////////

	return true;
}

__global__ void g_gls_Generate_Linear_System(
	float *out_ATA_partial,
	float *out_ATb_partial,
	//float *out_b_partial,
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
	const int ww = dim_map_dev[0];
	const int hh = dim_map_dev[1];
	const float th_icp = th_icp_dev[0];

	const int dim_ATA = 6*6,dim_ATb = 6;
	const int dim_block = blockDim.x*blockDim.y;

	// for linear system.
	float tATA[6*6],tATb[6],tb[1];
	bool flag_valid = false,tinlier[1]={false};

	// initialize linear system.
	for(int pidx = 0; pidx<dim_ATA; pidx++) tATA[pidx] = 0.0f;
	for(int pidx = 0; pidx<dim_ATb; pidx++) tATb[pidx] = 0.0f;

	if(tx >= 0 && tx < ww && ty >= 0 && ty < hh){
		flag_valid = d_css_Compute_Single_Summand(
			tx,ty,
			in_map_depth_t1,
			in_map_depth_t0,
			in_map_vertex_t0,
			in_map_normal_t0,
			in_T_cg_est,
			ww,hh,
			th_icp,
			tATA,
			tATb,
			tb);

		if(!flag_valid){
			for(int pidx = 0; pidx<dim_ATA; pidx++) tATA[pidx] = 0.0f;
			for(int pidx = 0; pidx<dim_ATb; pidx++) tATb[pidx] = 0.0f;
			tb[0] = 0.0f;
		}
	}

	// set cache values.
	// for ATA
	for(int pidx = 0; pidx<dim_ATA; pidx++){
		cache[cidx] = tATA[pidx];
		__syncthreads();

		int i = blockDim.x*blockDim.y/2;	// half of total thread number per block.
		while(i!=0){
			if(cidx < i){
				// i is idx in cache memory.
				cache[cidx] = __fadd_rn(cache[cidx],cache[cidx + i]);
				//cache[cidx] += cache[cidx + i];
			}
			__syncthreads();
			i /= 2;
		}

		// Update local block sum from shared memory to global memory.
		// Size of global memory should be block number.
		if(cidx == 0)	out_ATA_partial[bidx*dim_ATA + pidx] = cache[0];
	}
	__syncthreads();
	// for ATb
	for(int pidx = 0; pidx<6; pidx++){

		cache[cidx] = tATb[pidx];
		__syncthreads();

		int i = blockDim.x*blockDim.y/2;	// half of total thread number per block.
		while(i!=0){
			if(cidx < i){
				// i is idx in cache memory.
				cache[cidx] = __fadd_rn(cache[cidx],cache[cidx + i]);
				//cache[cidx] += cache[cidx + i];
			}
			__syncthreads();
			i /= 2;
		}

		// Update local block sum from shared memory to global memory.
		// Size of global memory should be block number.
		if(cidx == 0)	out_ATb_partial[bidx*6 + pidx] = cache[0];
	}
	__syncthreads();

}

//__global__ void g_gls_Generate_Linear_System(
//	float *out_ATA_partial,
//	float *out_ATb_partial,
//	float *out_b_partial,
//	int *out_num_val_partial,
//	int *out_num_inlier_partial,
//	const float *in_map_depth_t1,
//	const float *in_map_depth_t0,
//	const float *in_map_vertex_t0,
//	const float *in_map_normal_t0,
//	const float *in_T_cg_est)
//{
//	// for computing overall summation.
//// 	__shared__ float cache_ATA[6*6*CV_CUDA_MAX_BLOCK_SIZE], cache_ATb[6*CV_CUDA_MAX_BLOCK_SIZE];
//// 	__shared__ int cache_cnt[CV_CUDA_MAX_BLOCK_SIZE];
//	
//	const int tx = threadIdx.x + blockIdx.x*blockDim.x;
//	const int ty = threadIdx.y + blockIdx.y*blockDim.y;
//
//	const int cidx = threadIdx.x + threadIdx.y*blockDim.x;
//	const int bidx = blockIdx.x + blockIdx.y*gridDim.x;
//
//	__shared__ float cache[CV_CUDA_MAX_BLOCK_SIZE];
//
//	// for etc. parameters.	
//	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//	// initialize cache values.
//	cache[cidx] = 0.0f;
//	__syncthreads();
//
//	// for constant variables.
//	const int ww = dim_map_dev[0];
//	const int hh = dim_map_dev[1];
//	const float th_icp = th_icp_dev[0];
//
//	const int dim_ATA = 6*6,dim_ATb = 6;
//	const int dim_block = blockDim.x*blockDim.y;
//
//	// for linear system.
//	float tATA[6*6],tATb[6],tb[1];
//	bool flag_valid = false,tinlier[1]={false};
//
//	// initialize linear system.
//	for(int pidx = 0; pidx<dim_ATA; pidx++) tATA[pidx] = 0.0f;
//	for(int pidx = 0; pidx<dim_ATb; pidx++) tATb[pidx] = 0.0f;
//
//	if(tx >= 0 && tx < ww && ty >= 0 && ty < hh){
//		flag_valid = d_css_Compute_Single_Summand(
//			tx,ty,
//			in_map_depth_t1,
//			in_map_depth_t0,
//			in_map_vertex_t0,
//			in_map_normal_t0,
//			in_T_cg_est,
//			ww,hh,
//			th_icp,
//			tATA,
//			tATb,
//			tb,
//			tinlier);
//
//		if(!flag_valid){
//			for(int pidx = 0; pidx<dim_ATA; pidx++) tATA[pidx] = 0.0f;
//			for(int pidx = 0; pidx<dim_ATb; pidx++) tATb[pidx] = 0.0f;
//			tb[0] = 0.0f;
//			tinlier[0] = false;
//		}
//	}
//
//	// set cache values.
//	// for ATA
//	for(int pidx = 0; pidx<dim_ATA; pidx++){
//		cache[cidx] = tATA[pidx];
//		__syncthreads();
//
//		int i = blockDim.x*blockDim.y/2;	// half of total thread number per block.
//		while(i!=0){
//			if(cidx < i){
//				// i is idx in cache memory.
//				cache[cidx] = __fadd_rn(cache[cidx],cache[cidx + i]);
//				//cache[cidx] += cache[cidx + i];
//			}
//			__syncthreads();
//			i /= 2;
//		}
//
//		// Update local block sum from shared memory to global memory.
//		// Size of global memory should be block number.
//		if(cidx == 0)	out_ATA_partial[bidx*dim_ATA + pidx] = cache[0];
//	}
//	__syncthreads();
//	// for ATb
//	for(int pidx = 0; pidx<6; pidx++){
//
//		cache[cidx] = tATb[pidx];
//		__syncthreads();
//
//		int i = blockDim.x*blockDim.y/2;	// half of total thread number per block.
//		while(i!=0){
//			if(cidx < i){
//				// i is idx in cache memory.
//				cache[cidx] = __fadd_rn(cache[cidx],cache[cidx + i]);
//				//cache[cidx] += cache[cidx + i];
//			}
//			__syncthreads();
//			i /= 2;
//		}
//
//		// Update local block sum from shared memory to global memory.
//		// Size of global memory should be block number.
//		if(cidx == 0)	out_ATb_partial[bidx*6 + pidx] = cache[0];
//	}
//	__syncthreads();
//	// for b
//	{
//		cache[cidx] = tb[0];
//		__syncthreads();
//
//		int i = blockDim.x*blockDim.y/2;	// half of total thread number per block.
//		while(i!=0){
//			if(cidx < i){
//				// i is idx in cache memory.
//				cache[cidx] = __fadd_rn(cache[cidx],cache[cidx + i]);
//				//cache[cidx] += cache[cidx + i];
//			}
//			__syncthreads();
//			i /= 2;
//		}
//
//		// Update local block sum from shared memory to global memory.
//		// Size of global memory should be block number.
//		if(cidx == 0)	out_b_partial[bidx] = cache[0];
//	}
//	__syncthreads();
//	// for counting number of valid threads.
//	{
//		cache[cidx] = flag_valid;
//		__syncthreads();
//
//		int i = blockDim.x*blockDim.y/2;	// half of total thread number per block.
//		while(i!=0){
//			if(cidx < i){
//				// i is idx in cache memory.
//				cache[cidx] = __fadd_rn(cache[cidx],cache[cidx + i]);
//				//cache[cidx] += cache[cidx + i];
//			}
//			__syncthreads();
//			i /= 2;
//		}
//
//		// Update local block sum from shared memory to global memory.
//		// Size of global memory should be block number.
//		if(cidx == 0)	out_num_val_partial[bidx] = int(cache[0]);
//	}
//	__syncthreads();
//	// for counting number of inlier threads.
//	{
//		cache[cidx] = tinlier[0];
//		__syncthreads();
//
//		int i = blockDim.x*blockDim.y/2;	// half of total thread number per block.
//		while(i!=0){
//			if(cidx < i){
//				// i is idx in cache memory.
//				cache[cidx] = __fadd_rn(cache[cidx],cache[cidx + i]);
//				//cache[cidx] += cache[cidx + i];
//			}
//			__syncthreads();
//			i /= 2;
//		}
//
//		// Update local block sum from shared memory to global memory.
//		// Size of global memory should be block number.
//		if(cidx == 0)	out_num_inlier_partial[bidx] = int(cache[0]);
//	}
//
////
//// 	tidx = ty*ww + tx;
//// 
//// 	// check depth validity of map t1.
//// 	if((td1 = in_map_depth_t1[tidx]) == 0.0f) return ;
//// 	
////  	tpix.x = tx;	tpix.y = ty;
////  	// compute 3d point back-projected from current estimated pose in the global coordinates.
////  	// + [p3d_proj]
////  	d_bp_Back_Project(tpix, K_dev, td1, tp3d);
////  	d_t_Transform(tp3d, in_T_cg_est, p3d_proj);
////  	// compute 3d point predicted from the global model and previous camera pose in the global coordinates.
////  	// + [p3d_pred]
////  	d_t_Transform(p3d_proj, T_gc_dev_c, tp3d);
////  	d_p_Project(tp3d, K_dev, p2d_pred);
//// 	// + check depth validity.
////  	if(!d_gid_Get_Interpolated_Depth(p2d_pred, ww, hh, in_map_depth_t0, td0)) return ;
////  	d_bp_Back_Project(p2d_pred, K_dev, td0, tp3d);
////  	d_t_Transform(tp3d, T_cg_dev_c, p3d_pred);
//// 
//// 	// check distance between [p3d_proj] and [p3d_pred].
//// 	if(length(p3d_pred - p3d_proj) > th_icp)	return ;
//// 
//// 	// check correlation between normals of [p3d_proj] and [p3d_pred].
//// 	// + compute normal of [p3d_proj].
//// 	// + compute normal of [p3d_pred].
//// 	if(!d_gin_Get_Interpolated_Normal(p2d_pred, ww, hh, in_map_normal_t0, norm_pred)) return ;
////
//// 	// generate 6x6 linear system by computing the derivative of the objective function (22).
//// 	// refer to equation (24) of "KinectFusion: Real-Time Dense Surface Mapping and Tracking", ISMAR 2011.
//// 	// + compute A.	
//// 	// A = [(Vg,t)x|I]^T*Ng,t-1
//// 	// A1 = -(Vg,t)x*Ng | A2 = Ng,t-1.	
////   	A1 = -cross(p3d_proj, norm_pred); 	A2 = norm_pred;
////   	A[0] = A1.x; A[1] = A1.y; A[2] = A1.z;
////   	A[3] = A2.x; A[4] = A2.y; A[5] = A2.z;
////
////// 	A[0] = +p3d_proj.z*norm_pred.y -p3d_proj.y*norm_pred.z;
////// 	A[1] = -p3d_proj.z*norm_pred.x +p3d_proj.x*norm_pred.z;
////// 	A[2] = +p3d_proj.y*norm_pred.x -p3d_proj.x*norm_pred.y;
////// 	A[3] = norm_pred.x;	A[4] = norm_pred.y;	A[5] = norm_pred.z;
////  
////  	// + compute b.
////  	// b = Ng,t-1^T*(Vg,t-1 - Vg,t)
////  	b = dot(norm_pred, (p3d_pred - p3d_proj));
////// 	b = norm_pred.x*(p3d_pred.x-p3d_proj.x)
////// 	+norm_pred.y*(p3d_pred.y-p3d_proj.y)
////// 	+norm_pred.z*(p3d_pred.z-p3d_proj.z);
////
////	// update linear system.
////	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
////	// This part is important.
////	// Shared memory 써서 block 단위로 결과 모은다음 마지막에
////	// reduction 써서 block 단위 sum 합쳐야 함.
////	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
////	for(i=0; i<6*6; i++) tATA[i] = 0.0f;
////	for(i=0; i<6; i++) tATb[i] = 0.0f;
////
////	// copy ATA and ATb values of current thread to shared memory.
////	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
////	// 예제의 경우 total thread number (=grid_dim * block dim) 이 vector 전체
////	// size 보다 작기 때문에 이 부분이 필요하다.
////	// 예제처럼 이렇게 while 문으로 control 하는 것이 필요한가?
////	// 하나의 thread 값을 하나의 cache 에 넣으면 되는거 아님 그냥?	
////	//while(tx<ww && ty<hh){
////	
////	// for ATA
//// 	for(i=0; i<6; i++) for(j=0; j<6; j++){{
////		tATA[i*6 + j] = A[i]*A[j];
//// 	}}
////	// for ATb
////	for(i=0; i<6; i++) tATb[i] = A[i]*b;
////
////		//tx += blockDim.x;
////		//ty += blockDim.y;
////	//}
//// 
//// 	// ===========================================================
//// 	// We should add multiple matrices in parallel.
//// 	// This is an extension of summation of all vector elements.
//// 	// -----------------------------------------------------------
//// 	// for reduction, threadsPerBlock must be a power of 2.
//// 	// in our case, block is 2-dimensional.
//// 	// hence, thread number per block is blockDim.x*blockDim.y.
//// 	// ===========================================================
//// 	i = blockDim.x*blockDim.y/2;	// half of total thread number per block.
//// 	while(i!=0){
//// 		if(cidx < i){			
//// 			// j is element idx in matrix.
//// 			// i is idx in cache memory.
//// 			for(j=0; j<6*6; j++) cache_ATA[cidx*6*6 + j] += cache_ATA[(cidx + i)*6*6 + j];
//// 			for(j=0; j<6; j++) cache_ATb[cidx*6 + j] += cache_ATb[(cidx + i)*6 + j];
//// 			cache_cnt[cidx] += cache_cnt[cidx + i];
//// 		}
//// 		__syncthreads();
//// 		i/=2;
//// 	}
//// 
//// 	// Update local block sum from shared memory to global memory.
//// 	// Size of global memory should be block number.
//// 	if(cidx == 0){
//// 		tidx = blockIdx.x + blockIdx.y*gridDim.x;
//// 		for(j = 0; j<6*6; j++) out_ATA_partial[tidx*6*6 + j] = cache_ATA[j];
//// 		for(j = 0; j<6; j++) out_ATb_partial[tidx*6 + j] = cache_ATb[j];
//// 		out_num_val_partial[tidx] = cache_cnt[0];
//// 	}
//
//		
//}

/////////////////////////////////////////////////////////////////////////////////////////////
// LGKvVolumeIntegrator
/////////////////////////////////////////////////////////////////////////////////////////////

// *******************************************************
__host__ LGKvPoseTracker::LGKvPoseTracker()
// *******************************************************
{
	z_ATA_partial_dev.create(6, 6, 1); z_ATb_partial_dev.create(6, 1, 1);
	z_b_partial_dev.create(1, 1, 1);

	z_T_cg_est.create(4, 4, 1);
	z_T_cg_prev.create(4, 4, 1);
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
	float intrins_host[4];
	float th_icp_host[1];
	int dim_map_host[2];

	dim_map_host[0] = ww;	dim_map_host[1] = hh;
	intrins_host[0] = fx;	intrins_host[1] = fy;	intrins_host[2] = px;	intrins_host[3] = py;
	th_icp_host[0] = th_icp;		

	cudaMemcpyToSymbol(dim_map_dev, dim_map_host, 2 * sizeof(int));
	cudaMemcpyToSymbol(K_dev, intrins_host, 4 * sizeof(float));	
	cudaMemcpyToSymbol(th_icp_dev, th_icp_host, sizeof(float)); 
	
}

// *******************************************************
__host__ bool LGKvPoseTracker::tp_Track_Pose(
	GKvTrackingState *io_track_state,
	GKvMatrixFloat *in_map_depth_t1)
	//float *in_map_depth_t1)
// *******************************************************
{
	static int iter_num[4] ={10,7,7,7};
	static int th_num[4] ={500,300,100,100};
	static float T_cg_est_host[16],T_gc_est_host[16];

	const float *map_d_t1 = in_map_depth_t1->vp(); // in_map_depth_t1;//
	const float *map_d_t0 = io_track_state->vp_map_depth();
	const float *map_ver_t0 = io_track_state->vp_map_vertex();
	const float *map_norm_t0 = io_track_state->vp_map_normal();

	// current pose to estimate.
	float *p_T_cg_est_dev = z_T_cg_est.vp();
	float *T_cg_est_dev;

	//float *ATA_partial_dev,*ATb_partial_dev,*b_partial_dev;
	int *num_partial_dev,*num_inlier_partial_dev;

	Vector3f cent_prev,cent_est;
	Vector2i sz = io_track_state->sz_map;
	float *p_T_gc_dev = io_track_state->vp_T_gc();
	float *p_T_cg_dev = io_track_state->vp_T_cg();

	bool flag_valid;

	// Camera pose at t0.
	cudaMemcpyToSymbol(T_gc_dev_c,p_T_gc_dev,sizeof(float) * 16,0,cudaMemcpyDeviceToDevice);
	cudaMemcpyToSymbol(T_cg_dev_c,p_T_cg_dev,sizeof(float) * 16,0,cudaMemcpyDeviceToDevice);
	// Camera pose going to be estimated in current step.
	//cudaMalloc((void**)&T_cg_est_dev, 16*sizeof(float));
	cudaMemcpy(p_T_cg_est_dev,p_T_cg_dev,16*sizeof(float),cudaMemcpyDeviceToDevice);
	cudaMemcpy(T_cg_est_host,p_T_cg_est_dev,16*sizeof(float),cudaMemcpyDeviceToHost);
	// Previous camera center.
	cudaMemcpy(z_T_cg_prev.vp(),p_T_cg_dev,16*sizeof(float),cudaMemcpyDeviceToDevice);
	d_gcc_Get_Camera_Center(T_cg_est_host,cent_prev);

	// Cuda kernel.
	dim3 threads(CV_CUDA_BLOCK_SIZE_X,CV_CUDA_BLOCK_SIZE_Y);
	dim3 blocks(iDivUp(sz.x,threads.x),iDivUp(sz.y,threads.y));
	int dim_block = blocks.x*blocks.y;
	//printf("dim_block: %d\n", dim_block);

	if(z_ATA_partial_dev.nch() != dim_block)	z_ATA_partial_dev.create(6,6,dim_block);
	if(z_ATb_partial_dev.nch() != dim_block)	z_ATb_partial_dev.create(6,1,dim_block);
	if(z_b_partial_dev.nch() != dim_block)		z_b_partial_dev.create(1,1,dim_block);

// 	cudaMalloc((void**)&ATA_partial_dev,6*6*dim_block*sizeof(float));
// 	cudaMalloc((void**)&ATb_partial_dev,6*dim_block*sizeof(float));
// 	cudaMalloc((void**)&b_partial_dev,dim_block*sizeof(float));

//	cudaMalloc((void**)&num_partial_dev,dim_block*sizeof(int));
//	cudaMalloc((void**)&num_inlier_partial_dev,dim_block*sizeof(int));

	if(!z_ATA_partial_host) z_ATA_partial_host = new float[6*6*dim_block];
	if(!z_ATb_partial_host) z_ATb_partial_host = new float[6*dim_block];
	if(!z_b_partial_host) z_b_partial_host = new float[dim_block];

	//if(!z_num_partial_host) z_num_partial_host = new int[dim_block];
	//if(!z_num_inlier_partial_host) z_num_inlier_partial_host = new int[dim_block];

	flag_valid = true;

	for(int i=0; i<iter_num[0]; i++){

		g_gls_Generate_Linear_System<<<blocks,threads>>>(
			z_ATA_partial_dev.vp(), z_ATb_partial_dev.vp(), 
			//z_ATb_partial_dev.vp(),
			//ATA_partial_dev,ATb_partial_dev,b_partial_dev,
			//num_partial_dev,num_inlier_partial_dev,
			map_d_t1,
			map_d_t0,map_ver_t0,map_norm_t0,
			p_T_cg_est_dev);

		cudaMemcpy(z_ATA_partial_host,z_ATA_partial_dev.vp(),6*6*dim_block*sizeof(float),cudaMemcpyDeviceToHost); //z_ATA_partial_dev.vp(), 
		cudaMemcpy(z_ATb_partial_host,z_ATb_partial_dev.vp(),6*dim_block*sizeof(float),cudaMemcpyDeviceToHost); //z_ATb_partial_dev.vp(),
		//cudaMemcpy(z_b_partial_host,z_b_partial_dev.vp(),dim_block*sizeof(float),cudaMemcpyDeviceToHost); //z_ATb_partial_dev.vp(),

		//cudaMemcpy(z_ATA_partial_host,ATA_partial_dev,6*6*dim_block*sizeof(float),cudaMemcpyDeviceToHost); //z_ATA_partial_dev.vp(), 
		//cudaMemcpy(z_ATb_partial_host,ATb_partial_dev,6*dim_block*sizeof(float),cudaMemcpyDeviceToHost); //z_ATb_partial_dev.vp(),		
		//cudaMemcpy(z_b_partial_host,b_partial_dev,dim_block*sizeof(float),cudaMemcpyDeviceToHost); //z_ATb_partial_dev.vp(),

		//cudaMemcpy(z_num_partial_host,num_partial_dev,dim_block*sizeof(int),cudaMemcpyDeviceToHost); //z_ATb_partial_dev.vp(),
		//cudaMemcpy(z_num_inlier_partial_host,num_inlier_partial_dev,dim_block*sizeof(int),cudaMemcpyDeviceToHost); //z_ATb_partial_dev.vp(),

		// Add partial sums.
		for(int k=0; k<6*6; k++) z_ATA[k] = 0.0f;
		for(int k=0; k<6; k++) z_ATb[k] = 0.0f;
		//z_b = 0.0f;
		//z_num_val = 0;
		//z_num_inlier = 0;

		for(int k = 0; k<dim_block; k++){
			for(int j = 0; j<6*6; j++) z_ATA[j] += z_ATA_partial_host[k*6*6 + j];
			for(int j = 0; j<6; j++) z_ATb[j] += z_ATb_partial_host[k*6 + j];
			//z_b += z_b_partial_host[k];     // squared point-to-point distance
			//z_num_val += z_num_partial_host[k];
			//z_num_inlier += z_num_inlier_partial_host[k];
		}

		d_pm_Printf_Matrix(z_ATA, 6, 6, "ATA");
//  		printf("(#%d) z_ATb: %f %f %f | %f %f %f\n",
//  		z_num_val,z_ATb[0],z_ATb[1],z_ATb[2]
//  		,z_ATb[3],z_ATb[4],z_ATb[5]);

		//printf("z_num_val: %d\n", z_num_val);

		// Check number of valid pixels.
		//if(z_num_val < th_num[0]){ flag_valid = false; break; }

		// solve linear system.
		float norm2_x;
		d_lld_LL_Decomposition(z_ATA,6,z_L);
		//if(!d_sls_Solve_Linear_System_using_LLD(z_L,z_ATb,z_y,6,z_sol_x)) { flag_valid = false; break; }
		//////////////////////////////////////////////////////////////////////////
		// 이 부분이 이상함.
		// Diagonal element 가 0 일 때 처리를 잘 해주어야 할 듯??
		// 왜 0 이 생기는 것일까?
		flag_valid = d_sls_Solve_Linear_System_using_LLD(z_L,z_ATb,z_y,6,z_sol_x);
		//////////////////////////////////////////////////////////////////////////
		if(!flag_valid) break;

		// Check 2-norm of solution vector x.
		d_n2v_Norm_2_Vector(z_sol_x,6,norm2_x);
// 		printf("x: %f %f %f | %f %f %f\n"
// 				,z_sol_x[0],z_sol_x[1],z_sol_x[2]
// 				,z_sol_x[3],z_sol_x[4],z_sol_x[5]);
// 		printf("norm2_x: %f (%d valids)\n",norm2_x,z_num_val);
		if(norm2_x < 6.0e-3) break;

		// Update incremental transformation. (T_cg_est_dev)
		z_uit_Update_Incremental_Tracking(z_sol_x,T_cg_est_host);
		cudaMemcpy(p_T_cg_est_dev,T_cg_est_host,16*sizeof(float),cudaMemcpyHostToDevice);


		// 		for(int j = 0; j<6; j++) printf("%f ", z_sol_x[j]);
		// 		printf("\n");
		//for(int k=0; k<6*6; k++)
	}

	if(flag_valid){


		// 이 함수 쓰면 이상ㅎ
		io_track_state->set_transform(&z_T_cg_est,true);

		///////////////////////////////////////////////////////////////////////////////////
		// 지금 rmse 에서 Euclidean distance 만 보는데 normal vector 차이도 보도록 해보자....
		// 지금 rmse 에서 Euclidean distance 만 보는데 normal vector 차이도 보도록 해보자....
		// 지금 rmse 에서 Euclidean distance 만 보는데 normal vector 차이도 보도록 해보자....
		// detect motion drift.
		// Estimated current camera center.
		float dist,rmse_surf,inlier_perc;
		//cent_est = io_track_state->center;
		//dist = length(cent_est - cent_prev);

		//rmse_surf = sqrt(float(z_b)/(float)z_num_val);
		//inlier_perc = 100.0f*float(z_num_inlier)/float(z_num_val);

		rmse_surf = 0.00f;
		inlier_perc = 100.0f;
		//printf(" >>> rmse_surf: %f mm (%% %5.2f)\n",rmse_surf,inlier_perc);

		//if(dist > 0.175f){  // 0.5m*20(deg)*PI/180.0f.
		if(rmse_surf > 0.05f || inlier_perc < 95.0f){  // 0.5m*20(deg)*PI/180.0f.
			io_track_state->set_transform(&z_T_cg_prev,true);
			printf("=================================== Drift! ==========================================");
			return false;
		}
		///////////////////////////////////////////////////////////////////////////////////

		// Transform from camera to global coordinates.
		//  		cudaMemcpy(p_T_cg_dev, p_T_cg_est_dev, 16*sizeof(float), cudaMemcpyDeviceToDevice);
		//  		// Transform from global to camera coordinates.
		//  		d_im_Inverse_Matrix_4x4(T_cg_est_host, T_gc_est_host);
		//  		cudaMemcpy(p_T_gc_dev, T_gc_est_host, 16*sizeof(float), cudaMemcpyHostToDevice);
		//  		// Update camera center.
		//  		d_gcc_Get_Camera_Center(T_gc_est_host, io_track_state->center);
	}


// 	cudaFree(ATA_partial_dev);
// 	cudaFree(ATb_partial_dev);

//	cudaFree(num_partial_dev);
//	cudaFree(num_inlier_partial_dev);

	//cudaFree(T_cg_est_dev);

	return flag_valid;
}


//// *******************************************************
//__host__ bool LGKvPoseTracker::tp_Track_Pose(
//	float *out_T_cg_t1,
//	Vector2i in_sz_map,
//	const float *in_T_gc_t0, const float *in_T_cg_t0,
//	const float *in_map_depth_t0,
//	const float *in_map_normal_t0,
//	const float *in_map_depth_t1)
//// *******************************************************
//{
//	/// LLD test //////////
//	static int iter_num[4] ={10,7,7,7};
//	static int th_num[4] ={2000,500,100,100};
//	static float T_cg_est_host[16],T_gc_est_host[16];
//
//	const float *map_d_t1 = in_map_depth_t1;//in_map_depth_t1->vp();
//	const float *map_d_t0 = in_map_depth_t0;
//	const float *map_norm_t0 = in_map_normal_t0;
//
//	// current pose to estimate.
//	float *p_T_cg_est_dev = out_T_cg_t1;
//
//	float *ATA_partial_dev,*ATb_partial_dev,*b_partial_dev;
//	int *num_partial_dev,*num_inlier_partial_dev;
//
//	Vector3f cent_prev,cent_est;
//	Vector2i sz = in_sz_map;
//	const float *p_T_gc_dev = in_T_gc_t0;
//	const float *p_T_cg_dev = in_T_cg_t0;
//
//	bool flag_valid;
//
//	// Camera pose at t0.
//	cudaMemcpyToSymbol(T_gc_dev_c,p_T_gc_dev,sizeof(float) * 16,0,cudaMemcpyDeviceToDevice);
//	cudaMemcpyToSymbol(T_cg_dev_c,p_T_cg_dev,sizeof(float) * 16,0,cudaMemcpyDeviceToDevice);
//	// Camera pose going to be estimated in current step.
//	//cudaMalloc((void**)&T_cg_est_dev, 16*sizeof(float));
//	cudaMemcpy(p_T_cg_est_dev,p_T_cg_dev,16*sizeof(float),cudaMemcpyDeviceToDevice);
//	cudaMemcpy(T_cg_est_host,p_T_cg_est_dev,16*sizeof(float),cudaMemcpyDeviceToHost);
//	// Previous camera center.
//	cudaMemcpy(z_T_cg_prev.vp(),p_T_cg_dev,16*sizeof(float),cudaMemcpyDeviceToDevice);
//	d_gcc_Get_Camera_Center(T_cg_est_host,cent_prev);
//
//	// Cuda kernel.
//	dim3 threads(CV_CUDA_BLOCK_SIZE_X,CV_CUDA_BLOCK_SIZE_Y);
//	dim3 blocks(iDivUp(sz.x,threads.x),iDivUp(sz.y,threads.y));
//	int dim_block = blocks.x*blocks.y;
//	//printf("dim_block: %d\n", dim_block);
//
//	if(z_ATA_partial_dev.nch() != dim_block)	z_ATA_partial_dev.create(6,6,dim_block);
//	if(z_ATb_partial_dev.nch() != dim_block)	z_ATA_partial_dev.create(1,1,dim_block);
//
//	cudaMalloc((void**)&ATA_partial_dev,6*6*dim_block*sizeof(float));
//	cudaMalloc((void**)&ATb_partial_dev,6*dim_block*sizeof(float));
//	cudaMalloc((void**)&b_partial_dev,dim_block*sizeof(float));
//	cudaMalloc((void**)&num_partial_dev,dim_block*sizeof(int));
//	cudaMalloc((void**)&num_inlier_partial_dev,dim_block*sizeof(int));
//
//	if(!z_ATA_partial_host) z_ATA_partial_host = new float[6*6*dim_block];
//	if(!z_ATb_partial_host) z_ATb_partial_host = new float[6*dim_block];
//	if(!z_b_partial_host) z_b_partial_host = new float[dim_block];
//	
//	if(!z_num_partial_host) z_num_partial_host = new int[dim_block];
//	if(!z_num_inlier_partial_host) z_num_inlier_partial_host = new int[dim_block];
//
//	flag_valid = true;
//
//	for(int i=0; i<iter_num[0]; i++){
//
//// 		g_gls_Generate_Linear_System<<<blocks,threads>>>(
//// 			//z_ATA_partial_dev.vp(), z_ATb_partial_dev.vp(), 
//// 			ATA_partial_dev,ATb_partial_dev,b_partial_dev,
//// 			num_partial_dev,num_inlier_partial_dev,
//// 			map_d_t1,
//// 			map_d_t0,map_norm_t0,
//// 			p_T_cg_est_dev);
//
//		cudaMemcpy(z_ATA_partial_host,ATA_partial_dev,6*6*dim_block*sizeof(float),cudaMemcpyDeviceToHost); //z_ATA_partial_dev.vp(), 
//		cudaMemcpy(z_ATb_partial_host,ATb_partial_dev,6*dim_block*sizeof(float),cudaMemcpyDeviceToHost); //z_ATb_partial_dev.vp(),
//		cudaMemcpy(z_b_partial_host,b_partial_dev,dim_block*sizeof(float),cudaMemcpyDeviceToHost); //z_ATb_partial_dev.vp(),
//		cudaMemcpy(z_num_partial_host,num_partial_dev,dim_block*sizeof(int),cudaMemcpyDeviceToHost); //z_ATb_partial_dev.vp(),
//		cudaMemcpy(z_num_inlier_partial_host,num_inlier_partial_dev,dim_block*sizeof(int),cudaMemcpyDeviceToHost); //z_ATb_partial_dev.vp(),
//
//		// Add partial sums.
//		for(int k=0; k<6*6; k++) z_ATA[k] = 0.0f;
//		for(int k=0; k<6; k++) z_ATb[k] = 0.0f;
//		z_b = 0.0f;
//		z_num_val = 0;
//		z_num_inlier = 0;
//
//		for(int k = 0; k<dim_block; k++){
//			for(int j = 0; j<6*6; j++) z_ATA[j] += z_ATA_partial_host[k*6*6 + j];
//			for(int j = 0; j<6; j++) z_ATb[j] += z_ATb_partial_host[k*6 + j];
//			z_b += z_b_partial_host[k];     // squared point-to-point distance
//			z_num_val += z_num_partial_host[k];
//			z_num_inlier += z_num_inlier_partial_host[k];
//		}
//
//// 		 		printf("(#%d) z_ATb: %f %f %f | %f %f %f\n",
//// 		 		z_num_val,z_ATb[0],z_ATb[1],z_ATb[2]
//// 		 		,z_ATb[3],z_ATb[4],z_ATb[5]);
//
//		// Check number of valid pixels.
//		if(z_num_val < th_num[0]){ flag_valid = false; break; }
//
//		// solve linear system.
//		float norm2_x;
//		d_lld_LL_Decomposition(z_ATA,6,z_L);
//		if(!d_sls_Solve_Linear_System_using_LLD(z_L,z_ATb,z_y,6,z_sol_x)) { flag_valid = false; break; }
//
//		// Check 2-norm of solution vector x.
//		d_n2v_Norm_2_Vector(z_sol_x,6,norm2_x);
//		 	//	printf("x: %f %f %f | %f %f %f\n"
//		 	//			, z_sol_x[0], z_sol_x[1], z_sol_x[2]
//		 	//			, z_sol_x[3], z_sol_x[4], z_sol_x[5]);
//				//printf("norm2_x: %f (%d valids)\n", norm2_x, z_num_val);
//		if(norm2_x < 6.0e-3) break;
//
//		// Update incremental transformation. (T_cg_est_dev)
//		z_uit_Update_Incremental_Tracking(z_sol_x,T_cg_est_host);
//		cudaMemcpy(p_T_cg_est_dev,T_cg_est_host,16*sizeof(float),cudaMemcpyHostToDevice);
//
//
//		// 		for(int j = 0; j<6; j++) printf("%f ", z_sol_x[j]);
//		// 		printf("\n");
//		//for(int k=0; k<6*6; k++)
//	}
//
//	if(flag_valid){
//
//
//		// 이 함수 쓰면 이상ㅎ
//		//io_track_state->set_transform(&z_T_cg_est,true);
//
//		///////////////////////////////////////////////////////////////////////////////////
//		// 지금 rmse 에서 Euclidean distance 만 보는데 normal vector 차이도 보도록 해보자....
//		// 지금 rmse 에서 Euclidean distance 만 보는데 normal vector 차이도 보도록 해보자....
//		// 지금 rmse 에서 Euclidean distance 만 보는데 normal vector 차이도 보도록 해보자....
//		// detect motion drift.
//		// Estimated current camera center.
//// 		float dist,rmse_surf,inlier_perc;
//// 		//cent_est = io_track_state->center;
//// 		//dist = length(cent_est - cent_prev);
//// 
//// 		rmse_surf = sqrt(float(z_b)/(float)z_num_val);
//// 		inlier_perc = 100.0f*float(z_num_inlier)/float(z_num_val);
//// 		printf(" >>> rmse_surf: %f mm (%% %5.2f)\n",rmse_surf,inlier_perc);
//
//		//if(dist > 0.175f){  // 0.5m*20(deg)*PI/180.0f.
//		// 		if(rmse_surf > 0.05f || inlier_perc < 90.0f){  // 0.5m*20(deg)*PI/180.0f.
//		//  			io_track_state->set_transform(&z_T_cg_prev, true);
//		//  			return false;
//		//  		}
//		///////////////////////////////////////////////////////////////////////////////////
//
//		// Transform from camera to global coordinates.
//		//  		cudaMemcpy(p_T_cg_dev, p_T_cg_est_dev, 16*sizeof(float), cudaMemcpyDeviceToDevice);
//		//  		// Transform from global to camera coordinates.
//		//  		d_im_Inverse_Matrix_4x4(T_cg_est_host, T_gc_est_host);
//		//  		cudaMemcpy(p_T_gc_dev, T_gc_est_host, 16*sizeof(float), cudaMemcpyHostToDevice);
//		//  		// Update camera center.
//		//  		d_gcc_Get_Camera_Center(T_gc_est_host, io_track_state->center);
//	}
//
//
//	cudaFree(ATA_partial_dev);
//	cudaFree(ATb_partial_dev);
//	cudaFree(num_partial_dev);
//
//	//cudaFree(T_cg_est_dev);
//
//	return flag_valid;
//}
//
