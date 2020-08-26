/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_cuda_volume_integrator.cpp
/////////////////////////////////////////////////////////////////////////////////////////////
//#include "_yooji_2017_cuda_object_scanner.cuh"
//#define __CUDASCAN__
#include "../../_yooji_2017_cuda_object_scanner.cuh"

__constant__ float K_dev[4];
__constant__ float K_rgb_dev[4];
__constant__ float T_drgb_dev[16];

__constant__ float T_gc_dev_const[16];
__constant__ float T_cg_dev_const[16];
__constant__ int dim_map_dev[2];
__constant__ int dim_map_rgb_dev[2];

__constant__ float origin_dev[3];
__constant__ int dim_cube_dev[3];
__constant__ int dim_sc_dev[1];

__constant__ float mu_dev[1];
__constant__ float r_cube_dev[1];
__constant__ float sz_vox_dev[1];
__constant__ float sz_vox_inv_dev[1];
__constant__ float max_w_dev[1];


//********************************************************************************************
__device__ bool d_gpv_Get_Position_in_Voxel2(Vector3f in_p3d, 
	const float *origin, const int *dim_cube, const float sz_vox_inv,
	Vector3f &out_vox)
//********************************************************************************************
{
	bool valid = true;

	// assume that position of origin point in the world is (-0.5, -0.5, -0.5) in voxel coordinates..
// 	out_vox.x = (in_p3d.x - origin[0])*sz_vox_inv - 0.5f;
// 	out_vox.y = (in_p3d.y - origin[1])*sz_vox_inv - 0.5f;
// 	out_vox.z = (in_p3d.z - origin[2])*sz_vox_inv - 0.5f;

// 	out_vox.x = __fadd_rn(__fmul_rn(__fadd_rn(in_p3d.x,-origin[0]),sz_vox_inv),-0.5f);
// 	out_vox.y = __fadd_rn(__fmul_rn(__fadd_rn(in_p3d.y,-origin[1]),sz_vox_inv),-0.5f);
// 	out_vox.z = __fadd_rn(__fmul_rn(__fadd_rn(in_p3d.z,-origin[2]),sz_vox_inv),-0.5f);

	out_vox.x = __fmaf_rn(__fadd_rn(in_p3d.x, -origin[0]),sz_vox_inv, -0.5f);
	out_vox.y = __fmaf_rn(__fadd_rn(in_p3d.y, -origin[1]),sz_vox_inv, -0.5f);
	out_vox.z = __fmaf_rn(__fadd_rn(in_p3d.z, -origin[2]),sz_vox_inv, -0.5f);

	if(out_vox.x < -0.5f || out_vox.x >= (float)dim_cube[0] - 0.5f ||
		out_vox.y < -0.5f || out_vox.y >= (float)dim_cube[1] - 0.5f ||
		out_vox.z < -0.5f || out_vox.z >= (float)dim_cube[2] - 0.5f)
		valid = false;

	return valid;
}
//********************************************************************************************
__device__ bool d_gpw_Get_Position_in_World2(Vector3f in_vox, 
	const float *origin, const int *dim_cube, const float sz_vox,
	Vector3f &out_p3d)
//********************************************************************************************
{
	bool valid = true;

	if(in_vox.x < -0.5f || in_vox.x >= (float)dim_cube[0] - 0.5f ||
		in_vox.y < -0.5f || in_vox.y >= (float)dim_cube[1] - 0.5f ||
		in_vox.z < -0.5f || in_vox.z >= (float)dim_cube[2] - 0.5f)
		valid = false;

// 	out_p3d.x = (in_vox.x + 0.5f)*sz_vox + origin[0];
// 	out_p3d.y = (in_vox.y + 0.5f)*sz_vox + origin[1];
// 	out_p3d.z = (in_vox.z + 0.5f)*sz_vox + origin[2];

// 	out_p3d.x = __fadd_rn(__fmul_rn(__fadd_rn(in_vox.x,+0.5f),sz_vox),origin[0]);
// 	out_p3d.y = __fadd_rn(__fmul_rn(__fadd_rn(in_vox.y,+0.5f),sz_vox),origin[1]);
// 	out_p3d.z = __fadd_rn(__fmul_rn(__fadd_rn(in_vox.z,+0.5f),sz_vox),origin[2]);

	out_p3d.x = __fmaf_rn(__fadd_rn(in_vox.x, +0.5f),sz_vox, origin[0]);
	out_p3d.y = __fmaf_rn(__fadd_rn(in_vox.y, +0.5f),sz_vox, origin[1]);
	out_p3d.z = __fmaf_rn(__fadd_rn(in_vox.z, +0.5f),sz_vox, origin[2]);

	return valid;
}

__global__ void g_fvsc_Find_Visible_Sub_Cubes(
	bool *sub_cube,
	const float *map_depth)
{
	Vector2f tpix; Vector3f p3d, p3d_block_e, tp3d, tvox, dvec;

	const int sz_sc = dim_sc_dev[0];
	const float mu = mu_dev[0];
	const float sz_vox_inv = sz_vox_inv_dev[0];

	const int ww = dim_map_dev[0];
	const int hh = dim_map_dev[1];

	const int ww_sc = (dim_cube_dev[0] + sz_sc - 1)/sz_sc;
	const int hh_sc = (dim_cube_dev[1] + sz_sc - 1)/sz_sc;
	const int dd_sc = (dim_cube_dev[2] + sz_sc - 1)/sz_sc;

	const int tx = threadIdx.x + blockIdx.x*blockDim.x;
	const int ty = threadIdx.y + blockIdx.y*blockDim.y;

	bool valid = false;
	int tidx, n;
	int x_sc, y_sc, z_sc;

	float td;	

	if(tx < 0 || tx >= ww || ty < 0 || ty >= hh)	return;

	tidx = ty*ww + tx;

	// back-project depth.
	td = map_depth[tidx];
	if(td > 0.0f){
		tpix.x = tx;	tpix.y = ty;
		// compute direction vector of the pixel ray.
		d_bp_Back_Project(tpix, K_dev, td, tp3d);		d_t_Transform(tp3d, T_cg_dev_const, p3d);
		//d_bp_Back_Project(tpix, K_dev, 2.0f, tp3d);	    d_t_Transform(tp3d, T_cg_dev_c, p3d_block_e);
		d_bp_Back_Project(tpix, K_dev, 100.0f, tp3d);	    d_t_Transform(tp3d, T_cg_dev_const, p3d_block_e);

		dvec = (p3d_block_e - p3d).normalised();

		// activates sub-cubes on the pixel ray between +-mu of depth point.
		for(n = -4; n<=1; n++){
			
			tp3d = p3d + (n*mu)*dvec;

			//if(d_gpv_Get_Position_in_Voxel2(tp3d, origin_dev, dim_cube_dev, sz_vox_inv, tvox)){
			if(d_gpv_Get_Position_in_Voxel(tp3d, origin_dev, dim_cube_dev, sz_vox_inv, tvox)){

				x_sc = ROUNDF(tvox.x)/sz_sc;
				y_sc = ROUNDF(tvox.y)/sz_sc;
				z_sc = ROUNDF(tvox.z)/sz_sc;

				if(x_sc<0 || x_sc>=ww_sc ||
				   y_sc<0 || y_sc>=hh_sc ||
				   z_sc<0 || z_sc>=dd_sc) break;

				tidx = z_sc*ww_sc*hh_sc +y_sc*ww_sc +x_sc;

				sub_cube[tidx] = true;

			}

		}

	}


}

__global__ void g_ctv_Compute_TSDF_of_Voxels(
	float *cube_tsdf,
	uchar *cube_weight,
	const bool *sub_cube,	
	const float *map_depth)
{
	Vector2f tpix; Vector3f p3d, p3d_block_e, tp3d, tvox, dvec;

	const int sz_sc = dim_sc_dev[0];
	const float mu = mu_dev[0];
	const float max_w = max_w_dev[0];

	const int ww = dim_map_dev[0];
	const int hh = dim_map_dev[1];
	
	const int ww_c = dim_cube_dev[0];
	const int hh_c = dim_cube_dev[1];
	const int dd_c = dim_cube_dev[2];

	const int ww_sc = (ww_c + sz_sc - 1)/sz_sc;
	const int hh_sc = (hh_c + sz_sc - 1)/sz_sc;
	const int dd_sc = (dd_c + sz_sc - 1)/sz_sc;

	const int tx = threadIdx.x + blockIdx.x*blockDim.x;
	const int ty = threadIdx.y + blockIdx.y*blockDim.y;
	const int tz = threadIdx.z + blockIdx.z*blockDim.z;

	bool valid = false;
	int tidx, n;
	int x_sc, y_sc, z_sc, x_c, y_c, z_c;
	int x, y;

	float td, td_map, td_vox;
	float tsd, oldW, newW;
	
	if(tx < 0 || tx >= ww_sc || 
	   ty < 0 || ty >= hh_sc ||
	   tz < 0 || tz >= dd_sc)	return;

	//////////////////////////////////////////////////////////////////////////
	// 이 과정을 쓰면 GPU 코드 사용시 결과가 이상하게 나오는 경우 발생...
	// 이 과정을 쓰면 GPU 코드 사용시 결과가 이상하게 나오는 경우 발생...
 	// check sub-cube validity.
 	if(!sub_cube[tz*ww_sc*hh_sc +ty*ww_sc +tx])	return ;
	//////////////////////////////////////////////////////////////////////////

 	// inside of each sub-cube.
 	for(z_sc = 0; z_sc<sz_sc; z_sc++){
		z_c = tz*sz_sc + z_sc;
		for(y_sc = 0; y_sc<sz_sc; y_sc++){
 			y_c = ty*sz_sc + y_sc;
 
 			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 			// set range of X.
 			// 						tX_min = X_sc*sz_sc;
 			// 						tX_max = X_sc*sz_sc +sz_sc-1;
 			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 			for(x_sc = 0; x_sc<sz_sc; x_sc++){

				x_c = tx*sz_sc + x_sc;

 				tvox.x = x_c; tvox.y = y_c; tvox.z = z_c;
				//d_gpw_Get_Position_in_World2(tvox, origin_dev, dim_cube_dev, sz_vox_dev[0], tp3d);
				d_gpw_Get_Position_in_World(tvox, origin_dev, dim_cube_dev, sz_vox_inv_dev[0], tp3d);
 
 				// transform voxel from global coord. to depth camera coord.
				d_t_Transform(tp3d, T_gc_dev_const, p3d);
 				// project transformed voxel to depth camera.
				d_p_Project(p3d, K_dev, tpix);
 
 				// uses nearest neighbor lookup to prevent smearing of measurements at depth discontinuities.
 				// + for depth
 				x = ROUNDF(tpix.x);	y = ROUNDF(tpix.y);

 				if(x<0 || x>ww-1 || y<0 || y>hh-1)		continue;
 
 				// ================================================================
 				td_map = map_depth[y*ww + x];		// get depth.
// 				//if(!z_gid_Get_Interpolated_Depth(pp.x, pp.y, ww, hh, p_depth, td_map))	continue;
// 				// ================================================================							
 				td_vox = p3d.z;
 
 				if(td_map>0.0f){

					// calculate TSD value.
 					td = td_map - td_vox;	// difference(m) between depth of voxel and depth of pixel where the voxel was projected.
 
 					/// /////////////////////////////////////
 					if(td<-mu)	continue;		// no surface information is obtained from this range.
 					tsd = fminf(1.0f, td/mu);		// TSD value.
 					/// /////////////////////////////////////				
 
 					// calculate new weight.
 					// + get global voxel index.
 					tidx = z_c*ww_c*hh_c + y_c*ww_c + x_c;
 
 					oldW = (float)cube_weight[tidx];
 					newW = oldW + 1.0f;
 
 					// update voxel information.
 					// + for depth.
 					cube_tsdf[tidx] = (oldW*cube_tsdf[tidx] + tsd)/newW;
 					cube_weight[tidx] = (uchar)fminf(newW, (float)max_w);
	
				}
			}
		}
	}
}


__global__ void g_ctvc_Compute_TSDF_of_Voxels_with_Color(
	float *cube_tsdf,
	uchar *cube_weight,
	uchar *cube_color,
	const bool *sub_cube,
	const float *map_depth,
	const uchar *img_color,
	bool flag_on_rgb = false)
{
	Vector2f tp2d; Vector2i tpix; Vector3f p3d_c,p3d_g,p3d_block_e,tp3d,tvox,dvec; 
	uchar trgb[3], prgb[3];
	float trgbf[3];

	const int sz_sc = dim_sc_dev[0];
	const float mu = mu_dev[0];
	const float max_w = max_w_dev[0];

	const int ww = dim_map_dev[0];
	const int hh = dim_map_dev[1];

	const int ww_rgb = dim_map_rgb_dev[0];
	const int hh_rgb = dim_map_rgb_dev[1];

	const int ww_c = dim_cube_dev[0];
	const int hh_c = dim_cube_dev[1];
	const int dd_c = dim_cube_dev[2];

	const int ww_sc = (ww_c + sz_sc - 1)/sz_sc;
	const int hh_sc = (hh_c + sz_sc - 1)/sz_sc;
	const int dd_sc = (dd_c + sz_sc - 1)/sz_sc;

	const int step_2d = ww_rgb*hh_rgb;
	const int step_3d = ww_c*hh_c*dd_c;

	// sub-cube index.
	const int tx = threadIdx.x + blockIdx.x*blockDim.x;
	const int ty = threadIdx.y + blockIdx.y*blockDim.y;
	const int tz = threadIdx.z + blockIdx.z*blockDim.z;

	bool valid = false;
	int tidx,n,x_rgb,y_rgb;
	int x_sc,y_sc,z_sc,x_c,y_c,z_c;
	int x,y;

	float sdf,td_map;
	float tsdf,oldW,newW,inewW,lambda_inv,norm;

	if(tx < 0 || tx >= ww_sc ||
	   ty < 0 || ty >= hh_sc ||
	   tz < 0 || tz >= dd_sc)	return;

	//////////////////////////////////////////////////////////////////////////
	// check sub-cube validity.
	// 이거 좀 이상한 것 같다.
	// 현재는 sub_cube validity를 지속적으로 update 하면서 수행 하는데,
	// 사실 volume update 에서는 현재 frame 에서의 sub_cube validity 만이 필요하다.
	// 이부분 수정해야한다!
	if(!sub_cube[tz*ww_sc*hh_sc +ty*ww_sc +tx])	return ;
	//////////////////////////////////////////////////////////////////////////

	// inside of each sub-cube.
	for(z_sc = 0; z_sc<sz_sc; z_sc++){
		z_c = tz*sz_sc + z_sc;
		for(y_sc = 0; y_sc<sz_sc; y_sc++){
			y_c = ty*sz_sc + y_sc;

			for(x_sc = 0; x_sc<sz_sc; x_sc++){

				x_c = tx*sz_sc + x_sc;

				tvox.x = x_c; tvox.y = y_c; tvox.z = z_c;
				//d_gpw_Get_Position_in_World2(tvox, origin_dev, dim_cube_dev, sz_vox_dev[0], tp3d);
				d_gpw_Get_Position_in_World(tvox,origin_dev,dim_cube_dev,sz_vox_inv_dev[0],p3d_g);

				// transform voxel from global coord. to depth camera coord.
				d_t_Transform(p3d_g,T_gc_dev_const,p3d_c);
				// project transformed voxel to depth camera.
				d_p_Project(p3d_c,K_dev,tp2d);
				
				// uses nearest neighbor lookup to prevent smearing of measurements at depth discontinuities.
				// + for depth
				//x = ROUNDF(tpix.x);	y = ROUNDF(tpix.y);
				x = __float2int_rn(tp2d.x);	y = __float2int_rn(tp2d.y);

				if(x<0 || x>ww-1 || y<0 || y>hh-1)		continue;

				// ================================================================
				td_map = map_depth[y*ww + x];		// get depth.
				// 				//if(!z_gid_Get_Interpolated_Depth(pp.x, pp.y, ww, hh, p_depth, td_map))	continue;
				// 				// ================================================================							
				//td_vox = p3d_c.z;

				if(td_map>0.0f){

					// compute lambda.
					tpix.x = x; tpix.y = y;
					d_bp_Back_Project(tpix,K_dev,1,tp3d);
					lambda_inv = 1.0f/sqrtf(tp3d.x*tp3d.x + tp3d.y*tp3d.y + 1.0f);

					// calculate SDF value.
					// if voxel is behind depth map, td value is (-), else (+).
					//////////////////////////////////////////////////////////////////////////
					tp3d.x = p3d_g.x - T_cg_dev_const[3];
					tp3d.y = p3d_g.y - T_cg_dev_const[7];
					tp3d.z = p3d_g.z - T_cg_dev_const[11];
					norm = sqrtf(tp3d.x*tp3d.x + tp3d.y*tp3d.y + tp3d.z*tp3d.z);

					sdf = td_map - lambda_inv*norm;	// difference(m) between depth of voxel and depth of pixel where the voxel was projected.
					//////////////////////////////////////////////////////////////////////////


					/// /////////////////////////////////////
					// truncate SDF value.
					if(sdf<-mu)	continue;		// no surface information is obtained from this range.
					//if(td<-mu || td>mu)	continue;		// no surface information is obtained from this range.
					tsdf = fminf(1.0f,sdf/mu);		// TSDF value.
					//tsdf = (sdf > 0.0f) ? fminf(1.0f, sdf/mu) : -fminf(1.0f, sdf/mu);		// TSDF value.
					/// /////////////////////////////////////				

					// calculate new weight.
					// + get global voxel index.
					tidx = z_c*ww_c*hh_c + y_c*ww_c + x_c;

					oldW = (float)cube_weight[tidx];

					// weight for SDF integration.
					newW = oldW + 1.0f;
					inewW = 1.0f/newW;

					/////////////////////////////////////////////////////////////////////////
					if(oldW >= float(max_w)) continue;
					//if(abs(tsdf) > 0.5f) continue; // encode color value close to surface only.
					//////////////////////////////////////////////////////////////////////////

					// update voxel information.
					// + for depth.
					cube_tsdf[tidx] = (oldW*cube_tsdf[tidx] + tsdf)*inewW;///newW;					
					cube_weight[tidx] = (uchar)newW;
										
					// + for color.
 					if(!flag_on_rgb){
 						// transform voxel from depth camera coord. to RGB camera coord.
 						d_t_Transform(p3d_c,T_drgb_dev,tp3d);
 						// project transformed voxel on RGB camera.
 						d_p_Project(tp3d,K_rgb_dev,tp2d);
 
 						x_rgb = __float2int_rn(tp2d.x);	y_rgb = __float2int_rn(tp2d.y);
 						if(x_rgb<0 || x_rgb>ww_rgb-1 || y_rgb<0 || y_rgb>hh_rgb-1)		continue;
 					} 
					else{
 						//// project voxel on RGB camera.
 						//d_p_Project(p3d_c,K_rgb_dev,tpix);
 						//tx_rgb = __float2int_rn(tpix.x);	ty_rgb = __float2int_rn(tpix.y);
 						//if(tx_rgb<0 || tx_rgb>ww_rgb-1 || ty_rgb<0 || ty_rgb>hh_rgb-1)		continue;
						//x_rgb = x; y_rgb = y;
 					}

					// RGB.
					//////////////////////////////////////////////////////////////////////////
					// no problem.
					for(int k=0; k<3; k++) d_gii_Get_Interpolated_Intensity(tp2d,ww,hh,&img_color[k*ww*hh],trgbf[k]);

					//for(int k=0; k<3; k++)	trgb[k] = img_color[y_rgb*ww_rgb+x_rgb + k*step_2d];	// get color.
					//////////////////////////////////////////////////////////////////////////
					//////////////////////////////////////////////////////////////////////////
					// cube_color 의 R channel cube_color[0:step_3d] 만 계속적으로 초기화된다!!! 
					//////////////////////////////////////////////////////////////////////////
					for(int k=0; k<3; k++)	prgb[k] = cube_color[tidx + k*step_3d];

// 					// weight for SDF integration.
// 					newW = oldW + exp(-abs(tsdf));
// 					inewW = 1.0f/newW;

					for(int k=0; k<3; k++){
						cube_color[tidx + k*step_3d] = (uchar)(fmax(0.0f, fmin(255.0f,(oldW*float(prgb[k]) + (trgbf[k]))*inewW)));
					}

// 					for(int k=0; k<3; k++){
// 						cube_color[tidx + k*step_3d] = trgb[k];
// 					}

					//////////////////////////////////////////////////////////////////////////
					// 임시 방편으로 G channel 값을 R channel에!!
					// for R channel.
 					//cube_color[tidx] = (uchar)(sqrtf((oldW*SQUARE(float(prgb[1])) + SQUARE(float(trgb[1])))*inewW));
					//////////////////////////////////////////////////////////////////////////
// 					cube_color[tidx + step_3d] = trgb[1];
// 					cube_color[tidx + 2*step_3d] = trgb[2];
					//for(int k=0; k<3; k++)	cube_color[tidx + 0*ww_c*hh_c*dd_c] = (uchar)255;
					//cube_color[tidx] = 255;

				}
			}
		}
	}
}

__global__ void g_ctvc_Compute_TSDF_of_Voxels_with_Color(
	float *cube_tsdf,	
	uchar *cube_color,
	uchar *cube_weight,
	uchar *cube_weight_color,
	const bool *sub_cube,
	const float *map_depth,
	const uchar *img_color,
	bool flag_on_rgb = false)
{
	Vector2f tp2d; Vector2i tpix; Vector3f p3d_c,p3d_g,p3d_block_e,tp3d,tvox,dvec;
	uchar trgb[3],prgb[3];
	float trgbf[3];

	const int sz_sc = dim_sc_dev[0];
	const float mu = mu_dev[0];
	const float max_w = max_w_dev[0];

	const int ww = dim_map_dev[0];
	const int hh = dim_map_dev[1];

	const int ww_rgb = dim_map_rgb_dev[0];
	const int hh_rgb = dim_map_rgb_dev[1];

	const int ww_c = dim_cube_dev[0];
	const int hh_c = dim_cube_dev[1];
	const int dd_c = dim_cube_dev[2];

	const int ww_sc = (ww_c + sz_sc - 1)/sz_sc;
	const int hh_sc = (hh_c + sz_sc - 1)/sz_sc;
	const int dd_sc = (dd_c + sz_sc - 1)/sz_sc;

	const int step_2d = ww_rgb*hh_rgb;
	const int step_3d = ww_c*hh_c*dd_c;

	// sub-cube index.
	const int tx = threadIdx.x + blockIdx.x*blockDim.x;
	const int ty = threadIdx.y + blockIdx.y*blockDim.y;
	const int tz = threadIdx.z + blockIdx.z*blockDim.z;

	bool valid = false;
	int tidx,n,x_rgb,y_rgb;
	int x_sc,y_sc,z_sc,x_c,y_c,z_c;
	int x,y;

	float sdf,td_map;
	float tsdf,oldW,newW,inewW,lambda_inv,norm;

	if(tx < 0 || tx >= ww_sc ||
	   ty < 0 || ty >= hh_sc ||
	   tz < 0 || tz >= dd_sc)	return;

	//////////////////////////////////////////////////////////////////////////
	// check sub-cube validity.
	// 이거 좀 이상한 것 같다.
	// 현재는 sub_cube validity를 지속적으로 update 하면서 수행 하는데,
	// 사실 volume update 에서는 현재 frame 에서의 sub_cube validity 만이 필요하다.
	// 이부분 수정해야한다!
	if(!sub_cube[tz*ww_sc*hh_sc +ty*ww_sc +tx])	return ;
	//////////////////////////////////////////////////////////////////////////

	// inside of each sub-cube.
	for(z_sc = 0; z_sc<sz_sc; z_sc++){
		z_c = tz*sz_sc + z_sc;
		for(y_sc = 0; y_sc<sz_sc; y_sc++){
			y_c = ty*sz_sc + y_sc;

			for(x_sc = 0; x_sc<sz_sc; x_sc++){

				x_c = tx*sz_sc + x_sc;

				tvox.x = x_c; tvox.y = y_c; tvox.z = z_c;
				//d_gpw_Get_Position_in_World2(tvox, origin_dev, dim_cube_dev, sz_vox_dev[0], tp3d);
				d_gpw_Get_Position_in_World(tvox,origin_dev,dim_cube_dev,sz_vox_inv_dev[0],p3d_g);

				// transform voxel from global coord. to depth camera coord.
				d_t_Transform(p3d_g,T_gc_dev_const,p3d_c);
				// project transformed voxel to depth camera.
				d_p_Project(p3d_c,K_dev,tp2d);

				// uses nearest neighbor lookup to prevent smearing of measurements at depth discontinuities.
				// + for depth
				//x = ROUNDF(tpix.x);	y = ROUNDF(tpix.y);
				x = __float2int_rn(tp2d.x);	y = __float2int_rn(tp2d.y);

				if(x<0 || x>ww-1 || y<0 || y>hh-1)		continue;

				// ================================================================
				td_map = map_depth[y*ww + x];		// get depth.
				// 				//if(!z_gid_Get_Interpolated_Depth(pp.x, pp.y, ww, hh, p_depth, td_map))	continue;
				// 				// ================================================================							
				//td_vox = p3d_c.z;

				if(td_map>0.0f){

					// compute lambda.
					tpix.x = x; tpix.y = y;
					d_bp_Back_Project(tpix,K_dev,1,tp3d);
					lambda_inv = 1.0f/sqrtf(tp3d.x*tp3d.x + tp3d.y*tp3d.y + 1.0f);

					// calculate SDF value.
					// if voxel is behind depth map, td value is (-), else (+).
					//////////////////////////////////////////////////////////////////////////
					tp3d.x = p3d_g.x - T_cg_dev_const[3];
					tp3d.y = p3d_g.y - T_cg_dev_const[7];
					tp3d.z = p3d_g.z - T_cg_dev_const[11];
					norm = sqrtf(tp3d.x*tp3d.x + tp3d.y*tp3d.y + tp3d.z*tp3d.z);

					sdf = td_map - lambda_inv*norm;	// difference(m) between depth of voxel and depth of pixel where the voxel was projected.
					//////////////////////////////////////////////////////////////////////////


					/// /////////////////////////////////////
					// truncate SDF value.
					if(sdf<-mu)	continue;		// no surface information is obtained from this range.
					//if(td<-mu || td>mu)	continue;		// no surface information is obtained from this range.
					tsdf = fminf(1.0f,sdf/mu);		// TSDF value.
					//tsdf = (sdf > 0.0f) ? fminf(1.0f, sdf/mu) : -fminf(1.0f, sdf/mu);		// TSDF value.
					/// /////////////////////////////////////				

					// calculate new weight.
					// + get global voxel index.
					tidx = z_c*ww_c*hh_c + y_c*ww_c + x_c;

					oldW = (float)cube_weight[tidx];

					// weight for SDF integration.
					newW = oldW + 1.0f;
					inewW = 1.0f/newW;

					/////////////////////////////////////////////////////////////////////////
					if(oldW >= float(max_w)) continue;
					//if(abs(tsdf) > 0.5f) continue; // encode color value close to surface only.
					//////////////////////////////////////////////////////////////////////////

					// update voxel information.
					// + for depth.
					cube_tsdf[tidx] = (oldW*cube_tsdf[tidx] + tsdf)*inewW;///newW;					
					cube_weight[tidx] = (uchar)newW;

					// + for color.
					if(!flag_on_rgb){
						// transform voxel from depth camera coord. to RGB camera coord.
						d_t_Transform(p3d_c,T_drgb_dev,tp3d);
						// project transformed voxel on RGB camera.
						d_p_Project(tp3d,K_rgb_dev,tp2d);

						x_rgb = __float2int_rn(tp2d.x);	y_rgb = __float2int_rn(tp2d.y);
						if(x_rgb<0 || x_rgb>ww_rgb-1 || y_rgb<0 || y_rgb>hh_rgb-1)		continue;
					} else{
						//// project voxel on RGB camera.
						//d_p_Project(p3d_c,K_rgb_dev,tpix);
						//tx_rgb = __float2int_rn(tpix.x);	ty_rgb = __float2int_rn(tpix.y);
						//if(tx_rgb<0 || tx_rgb>ww_rgb-1 || ty_rgb<0 || ty_rgb>hh_rgb-1)		continue;
						//x_rgb = x; y_rgb = y;
					}

					// RGB.
					//////////////////////////////////////////////////////////////////////////
					// no problem.
					for(int k=0; k<3; k++) d_gii_Get_Interpolated_Intensity(tp2d,ww,hh,&img_color[k*ww*hh],trgbf[k]);

					//for(int k=0; k<3; k++)	trgb[k] = img_color[y_rgb*ww_rgb+x_rgb + k*step_2d];	// get color.
					//////////////////////////////////////////////////////////////////////////
					//////////////////////////////////////////////////////////////////////////
					// cube_color 의 R channel cube_color[0:step_3d] 만 계속적으로 초기화된다!!! 
					//////////////////////////////////////////////////////////////////////////
					for(int k=0; k<3; k++)	prgb[k] = cube_color[tidx + k*step_3d];

					// weight for SDF integration.
					newW = oldW + exp(-abs(tsdf));
					inewW = 1.0f/newW;

					for(int k=0; k<3; k++){
						//cube_color[tidx + k*step_3d] = (uchar)(sqrtf((oldW*SQUARE(float(prgb[k])) + SQUARE(float(trgbf[k])))*inewW));
						cube_color[tidx + k*step_3d] = (uchar)(sqrtf((oldW*SQUARE(float(prgb[k])) + SQUARE(float(trgbf[k])))*inewW));
					}

					// 					for(int k=0; k<3; k++){
					// 						cube_color[tidx + k*step_3d] = trgb[k];
					// 					}

					//////////////////////////////////////////////////////////////////////////
					// 임시 방편으로 G channel 값을 R channel에!!
					// for R channel.
					//cube_color[tidx] = (uchar)(sqrtf((oldW*SQUARE(float(prgb[1])) + SQUARE(float(trgb[1])))*inewW));
					//////////////////////////////////////////////////////////////////////////
					// 					cube_color[tidx + step_3d] = trgb[1];
					// 					cube_color[tidx + 2*step_3d] = trgb[2];
					//for(int k=0; k<3; k++)	cube_color[tidx + 0*ww_c*hh_c*dd_c] = (uchar)255;
					//cube_color[tidx] = 255;

				}
			}
		}
	}
}

__global__ void g_uvv_Update_Valid_Volume(
	float *cube_tsdf,
	uchar *cube_weight,
	uchar *cube_color,
	const bool *vol_valid)
{
	const int sz_sc = dim_sc_dev[0];

	const int ww_c = dim_cube_dev[0];
	const int hh_c = dim_cube_dev[1];
	const int dd_c = dim_cube_dev[2];

	const int ww_sc = (ww_c + sz_sc - 1)/sz_sc;
	const int hh_sc = (hh_c + sz_sc - 1)/sz_sc;
	const int dd_sc = (dd_c + sz_sc - 1)/sz_sc;

	const int tx = threadIdx.x + blockIdx.x*blockDim.x;
	const int ty = threadIdx.y + blockIdx.y*blockDim.y;
	const int tz = threadIdx.z + blockIdx.z*blockDim.z;

	int tidx;
	int x_sc,y_sc,z_sc,x_c,y_c,z_c;
	int x,y;

	if(tx < 0 || tx >= ww_sc ||
	   ty < 0 || ty >= hh_sc ||
	   tz < 0 || tz >= dd_sc)	return;

	// inside of each sub-cube.
	for(z_sc = 0; z_sc<sz_sc; z_sc++){
		z_c = tz*sz_sc + z_sc;
		for(y_sc = 0; y_sc<sz_sc; y_sc++){
			y_c = ty*sz_sc + y_sc;

			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			// set range of X.
			// 						tX_min = X_sc*sz_sc;
			// 						tX_max = X_sc*sz_sc +sz_sc-1;
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			for(x_sc = 0; x_sc<sz_sc; x_sc++){

				x_c = tx*sz_sc + x_sc;

				// calculate new weight.
				// + get global voxel index.
				tidx = z_c*ww_c*hh_c + y_c*ww_c + x_c;
				
				// re-initialize object model.
				if(!vol_valid[tidx]){
					cube_tsdf[tidx] = 1.0f;
					cube_weight[tidx] = 0.0f;
					for(int k=0; k<3; k++) cube_color[tidx + k*ww_c*hh_c*dd_c] = uchar(0);
				}
			}
		}
	}
}

__global__ void g_uvv_Update_Valid_Volume(
	float *cube_tsdf,
	uchar *cube_weight,
	uchar *cube_color,
	bool *cube_valid,
	const bool *vol_valid)
{
	const int sz_sc = dim_sc_dev[0];

	const int ww_c = dim_cube_dev[0];
	const int hh_c = dim_cube_dev[1];
	const int dd_c = dim_cube_dev[2];

	const int ww_sc = (ww_c + sz_sc - 1)/sz_sc;
	const int hh_sc = (hh_c + sz_sc - 1)/sz_sc;
	const int dd_sc = (dd_c + sz_sc - 1)/sz_sc;

	const int tx = threadIdx.x + blockIdx.x*blockDim.x;
	const int ty = threadIdx.y + blockIdx.y*blockDim.y;
	const int tz = threadIdx.z + blockIdx.z*blockDim.z;

	int tidx;
	int x_sc,y_sc,z_sc,x_c,y_c,z_c;
	int x,y;

	if(tx < 0 || tx >= ww_sc ||
	   ty < 0 || ty >= hh_sc ||
	   tz < 0 || tz >= dd_sc)	return;

	if(!cube_valid[tz*ww_sc*hh_sc +ty*ww_sc +tx])	return ;


	// inside of each sub-cube.
	int cnt = 0;
	// count valid voxels in current sub-cube.
	for(z_sc = 0; z_sc<sz_sc; z_sc++){
		z_c = tz*sz_sc + z_sc;
		for(y_sc = 0; y_sc<sz_sc; y_sc++){
			y_c = ty*sz_sc + y_sc;

			for(x_sc = 0; x_sc<sz_sc; x_sc++){

				x_c = tx*sz_sc + x_sc;

				// get global voxel index.
				tidx = z_c*ww_c*hh_c + y_c*ww_c + x_c;

				// count valid voxels in current sub-cube.
				if(vol_valid[tidx]) cnt++;

				
			}
		}
	}
	if(cnt > 0) return ;

	// re-initialize object model.
	cube_valid[tz*ww_sc*hh_sc +ty*ww_sc +tx] = false;

	for(z_sc = 0; z_sc<sz_sc; z_sc++){
		z_c = tz*sz_sc + z_sc;
		for(y_sc = 0; y_sc<sz_sc; y_sc++){
			y_c = ty*sz_sc + y_sc;

			for(x_sc = 0; x_sc<sz_sc; x_sc++){

				x_c = tx*sz_sc + x_sc;

				// calculate new weight.
				// + get global voxel index.
				tidx = z_c*ww_c*hh_c + y_c*ww_c + x_c;

				// re-initialize object model.
				if(!vol_valid[tidx]){
					cube_tsdf[tidx] = 1.0f;
					cube_weight[tidx] = 0.0f;
					for(int k=0; k<3; k++) cube_color[tidx + k*ww_c*hh_c*dd_c] = uchar(0);
				}
			}
		}
	}
}


/////////////////////////////////////////////////////////////////////////////////////////////
// LGKvVolumeIntegrator 
/////////////////////////////////////////////////////////////////////////////////////////////

// *******************************************************
__host__ LGKvVolumeIntegrator::LGKvVolumeIntegrator()
// *******************************************************
{

}

// *******************************************************
__host__ LGKvVolumeIntegrator::~LGKvVolumeIntegrator()
// *******************************************************
{

}


// *******************************************************
__host__ void LGKvVolumeIntegrator::ip_Initialize_Parameters(
	GKvObjectCubeFloat *in_cube,
	int ww, int hh,
	float fx, float fy,
	float px, float py,
	float mu,
	float max_w)
// *******************************************************
{
	float intrins_host[4], origin_host[3], light_host[3];
	float sz_vox_host[1], sz_vox_inv_host[1], mu_host[1], r_host[1], max_w_host[1];
	int dim_cube_host[3], dim_sc_host[1], dim_map_host[2];

	Vector3f cube_org = in_cube->origin();
	Vector3f cube_cen = in_cube->center();

	intrins_host[0] = fx;	intrins_host[1] = fy;	intrins_host[2] = px;	intrins_host[3] = py;
	origin_host[0] = cube_org.x;	origin_host[1] = cube_org.y;	origin_host[2] = cube_org.z;

	// 	printf("cube_org: %f %f %f\n", cube_org.x, cube_org.y, cube_org.z);
	// 	printf("cube_cen: %f %f %f\n", cube_cen.x, cube_cen.y, cube_cen.z);
	// 	printf("%f %f\n", mu, max_w);

	mu_host[0] = mu;
	sz_vox_host[0] = in_cube->sz_vox();
	sz_vox_inv_host[0] = 1.0f/in_cube->sz_vox();
	r_host[0] = length(cube_org - cube_cen);
	max_w_host[0] = max_w;

	in_cube->ts(dim_cube_host[0], dim_cube_host[1], dim_cube_host[2]);
	dim_map_host[0] = ww;	dim_map_host[1] = hh;
	dim_sc_host[0] = in_cube->dim_sc();

	cudaMemcpyToSymbol(K_dev, intrins_host, 4 * sizeof(float));
	//  	
	cudaMemcpyToSymbol(origin_dev, origin_host, 3 * sizeof(float));
	//  	
	cudaMemcpyToSymbol(mu_dev, mu_host, sizeof(float));
	cudaMemcpyToSymbol(max_w_dev, max_w_host, sizeof(float));
	cudaMemcpyToSymbol(sz_vox_dev, sz_vox_host, sizeof(float));
	cudaMemcpyToSymbol(sz_vox_inv_dev, sz_vox_inv_host, sizeof(float));
	cudaMemcpyToSymbol(r_cube_dev, r_host, sizeof(float));

	// 
	cudaMemcpyToSymbol(dim_map_dev, dim_map_host, 2 * sizeof(int));
	cudaMemcpyToSymbol(dim_cube_dev, dim_cube_host, 3 * sizeof(int));
	cudaMemcpyToSymbol(dim_sc_dev, dim_sc_host, sizeof(int));
}


// *******************************************************
__host__ void LGKvVolumeIntegrator::ip_Initialize_Parameters(
	GKvObjectCubeFloat *in_cube,
	int ww,int hh,
	float fx,float fy,
	float px,float py,
	int ww_rgb,int hh_rgb,
	float fx_rgb,float fy_rgb,
	float px_rgb,float py_rgb,
	const float *in_T_drgb,
	float mu,
	float max_w)
// *******************************************************
{
	float intrins_host[4],intrins_rgb_host[4],origin_host[3],light_host[3];
	float sz_vox_host[1],sz_vox_inv_host[1],mu_host[1],r_host[1],max_w_host[1];
	int dim_cube_host[3],dim_sc_host[1],dim_map_host[2],dim_map_rgb_host[2];

	Vector3f cube_org = in_cube->origin();
	Vector3f cube_cen = in_cube->center();

	intrins_host[0] = fx;	intrins_host[1] = fy;	
	intrins_host[2] = px;	intrins_host[3] = py;
	intrins_rgb_host[0] = fx_rgb;	intrins_rgb_host[1] = fy_rgb;	
	intrins_rgb_host[2] = px_rgb;	intrins_rgb_host[3] = py_rgb;

	origin_host[0] = cube_org.x;	origin_host[1] = cube_org.y;	origin_host[2] = cube_org.z;

	// 	printf("cube_org: %f %f %f\n", cube_org.x, cube_org.y, cube_org.z);
	// 	printf("cube_cen: %f %f %f\n", cube_cen.x, cube_cen.y, cube_cen.z);
	// 	printf("%f %f\n", mu, max_w);

	mu_host[0] = mu;
	sz_vox_host[0] = in_cube->sz_vox();
	sz_vox_inv_host[0] = 1.0f/in_cube->sz_vox();
	r_host[0] = length(cube_org - cube_cen);
	max_w_host[0] = max_w;

	in_cube->ts(dim_cube_host[0],dim_cube_host[1],dim_cube_host[2]);
	dim_map_host[0] = ww;	dim_map_host[1] = hh;
	dim_map_rgb_host[0] = ww_rgb;	dim_map_rgb_host[1] = hh_rgb;
	dim_sc_host[0] = in_cube->dim_sc();

	cudaMemcpyToSymbol(K_dev,intrins_host,4 * sizeof(float));
	cudaMemcpyToSymbol(K_rgb_dev,intrins_rgb_host,4 * sizeof(float));
	cudaMemcpyToSymbol(T_drgb_dev,in_T_drgb,sizeof(float) * 16);

	//  	
	cudaMemcpyToSymbol(origin_dev,origin_host,3 * sizeof(float));
	//  	
	cudaMemcpyToSymbol(mu_dev,mu_host,sizeof(float));
	cudaMemcpyToSymbol(max_w_dev,max_w_host,sizeof(float));
	cudaMemcpyToSymbol(sz_vox_dev,sz_vox_host,sizeof(float));
	cudaMemcpyToSymbol(sz_vox_inv_dev,sz_vox_inv_host,sizeof(float));
	cudaMemcpyToSymbol(r_cube_dev,r_host,sizeof(float));

	// 
	cudaMemcpyToSymbol(dim_map_dev,dim_map_host,2 * sizeof(int));
	cudaMemcpyToSymbol(dim_map_rgb_dev,dim_map_rgb_host,2 * sizeof(int));
	cudaMemcpyToSymbol(dim_cube_dev,dim_cube_host,3 * sizeof(int));
	cudaMemcpyToSymbol(dim_sc_dev,dim_sc_host,sizeof(int));
}

// *******************************************************
__host__ void LGKvVolumeIntegrator::cdtoc_Convert_Depth_to_TSDF_on_Cube(
	GKvObjectCubeFloat *io_cube,
	const float *in_map_depth_dev,
	int in_ww, int in_hh,
	const float *in_T_gc_dev,
	const float *in_T_cg_dev)
// *******************************************************
{
	int block_sz, grid_sz;
	int ww, hh, dd;

	io_cube->ts(ww, hh, dd);

	// Camera pose.
	cudaMemcpyToSymbol(T_gc_dev_const, in_T_gc_dev, sizeof(float) * 16, 0, cudaMemcpyDeviceToDevice);
	cudaMemcpyToSymbol(T_cg_dev_const, in_T_cg_dev, sizeof(float) * 16, 0, cudaMemcpyDeviceToDevice);

	// For model cube update.
	dim3 threads(CV_CUDA_BLOCK_SIZE_X, CV_CUDA_BLOCK_SIZE_Y);
	dim3 blocks(iDivUp(in_ww, threads.x), iDivUp(in_hh, threads.y));
	
	g_fvsc_Find_Visible_Sub_Cubes<<<blocks, threads>>>(
		io_cube->vp_valid(),
		in_map_depth_dev);

	// For model cube update.
	dim3 threads2(CV_CUDA_BLOCK_SIZE_X, CV_CUDA_BLOCK_SIZE_Y, CV_CUDA_BLOCK_SIZE_Z);
	dim3 blocks2(iDivUp(ww, threads2.x), iDivUp(hh, threads2.y), iDivUp(dd, threads2.z));


	g_ctv_Compute_TSDF_of_Voxels<<<blocks2, threads2>>>(
		io_cube->vp_tsdf(),
		io_cube->vp_w(),
		io_cube->vp_valid(),		
		in_map_depth_dev);

}


// *******************************************************
__host__ void LGKvVolumeIntegrator::cdtocc_Convert_Depth_to_TSDF_on_Cube_with_Color(
	GKvObjectCubeFloat *io_cube,
	const float *in_map_depth_dev,
	const uchar *in_img_color_dev,
	int in_ww,int in_hh,
	const float *in_T_gc_dev,
	const float *in_T_cg_dev,
	bool in_flag_on_rgb)
// *******************************************************
{
	int block_sz,grid_sz;
	int ww,hh,dd;

	io_cube->ts(ww,hh,dd);

	// Camera pose.
	cudaMemcpyToSymbol(T_gc_dev_const,in_T_gc_dev,sizeof(float) * 16,0,cudaMemcpyDeviceToDevice);
	cudaMemcpyToSymbol(T_cg_dev_const,in_T_cg_dev,sizeof(float) * 16,0,cudaMemcpyDeviceToDevice);

	// For model cube update.
	dim3 threads(CV_CUDA_BLOCK_SIZE_X,CV_CUDA_BLOCK_SIZE_Y);
	dim3 blocks(iDivUp(in_ww,threads.x),iDivUp(in_hh,threads.y));

 	//////////////////////////////////////////////////////////////////////////
 	// initialize sub-cube!!!
 	// set voxel validity to true for current depth map only.
	//////////////////////////////////////////////////////////////////////////
	// 여기다!!! 요놈은 sub cube dimension 인데 full cube dimension 으로 넣으니
	// 다음에 오는 color cube 영역을 침범하여 초기화 한다!!!
	//////////////////////////////////////////////////////////////////////////
 	int3 dim_cube; 
	dim_cube.x = iDivUp(ww, io_cube->dim_sc()); 
	dim_cube.y = iDivUp(hh, io_cube->dim_sc()); 
	dim_cube.z = iDivUp(dd, io_cube->dim_sc());
	setDeviceMem3D(io_cube->vp_valid(),dim_cube,false);
 	//////////////////////////////////////////////////////////////////////////
 
 	g_fvsc_Find_Visible_Sub_Cubes<<<blocks,threads>>>(
 		io_cube->vp_valid(),
 		in_map_depth_dev);

	// For model cube update.
	dim3 threads2(CV_CUDA_BLOCK_SIZE_X,CV_CUDA_BLOCK_SIZE_Y,CV_CUDA_BLOCK_SIZE_Z);
	dim3 blocks2(iDivUp(ww,threads2.x),iDivUp(hh,threads2.y),iDivUp(dd,threads2.z));

	g_ctvc_Compute_TSDF_of_Voxels_with_Color<<<blocks2,threads2>>>(
		io_cube->vp_tsdf(),
		io_cube->vp_w(),
		io_cube->vp_rgb(),
		io_cube->vp_valid(),
		in_map_depth_dev,
		in_img_color_dev,
		in_flag_on_rgb);


}

// *******************************************************

__host__ void LGKvVolumeIntegrator::uvv_Update_Valid_Volume(
	GKvObjectCubeFloat *io_cube,
	GKvVolumeBool *in_vol_valid)
// *******************************************************
{
	int ww,hh,dd;

	io_cube->ts(ww,hh,dd);

	// For model cube update.
	dim3 threads(CV_CUDA_BLOCK_SIZE_X,CV_CUDA_BLOCK_SIZE_Y,CV_CUDA_BLOCK_SIZE_Z);
	dim3 blocks(iDivUp(ww,threads.x),iDivUp(hh,threads.y),iDivUp(dd,threads.z));

// 	g_uvv_Update_Valid_Volume<<<blocks,threads>>>(
// 		io_cube->vp_tsdf(),
// 		io_cube->vp_w(),
// 		io_cube->vp_rgb(),
// 		in_vol_valid->vp());

	g_uvv_Update_Valid_Volume<<<blocks,threads>>>(
		io_cube->vp_tsdf(),
		io_cube->vp_w(),
		io_cube->vp_rgb(),
		io_cube->vp_valid(),
		in_vol_valid->vp());
}