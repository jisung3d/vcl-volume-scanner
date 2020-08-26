/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_dense_stereo.cpp
/////////////////////////////////////////////////////////////////////////////////////////////
// #include "_yooji_2017_cuda_object_scanner.cuh"
// #define __CUDASCAN__
#include "../../_yooji_2017_cuda_object_scanner.cuh"

__constant__ float K_dev[20];		// Maximum pyramid level is 5.
__constant__ int dim_map_dev[10];	// Maximum pyramid level is 5.

__constant__ float T_gc_dev_const[16];
__constant__ float T_cg_dev_const[16];

__constant__ float origin_dev[3];
__constant__ int dim_cube_dev[3];
__constant__ int dim_sc_dev[1];

__constant__ float mu_dev[1];
__constant__ float r_cube_dev[1];
__constant__ float sz_vox_inv_dev[1];
__constant__ float max_w_dev[1];

///////////////////////////////////////////////////////////////////////////////
/// Device functions.
///////////////////////////////////////////////////////////////////////////////
//
////********************************************************************************************
//__device__ inline bool d_gpv_Get_Position_in_Voxel3(Vector3f in_p3d, 
//	const float *origin, const int *dim_cube, const float sz_vox_inv,
//	Vector3f &out_vox)
////********************************************************************************************
//{
//	bool valid = true;
//
//	// assume that position of origin point in the world is (-0.5, -0.5, -0.5) in voxel coordinates..
//	out_vox.x = __fmaf_rn((in_p3d.x - origin[0]),sz_vox_inv, -0.5f);
//	out_vox.y = __fmaf_rn((in_p3d.y - origin[1]),sz_vox_inv, -0.5f);
//	out_vox.z = __fmaf_rn((in_p3d.z - origin[2]),sz_vox_inv, -0.5f);
//
//	if(out_vox.x < -0.5f || out_vox.x >= (float)dim_cube[0] - 0.5f ||
//		out_vox.y < -0.5f || out_vox.y >= (float)dim_cube[1] - 0.5f ||
//		out_vox.z < -0.5f || out_vox.z >= (float)dim_cube[2] - 0.5f)
//		valid = false;
//
//	return valid;
//}
////********************************************************************************************
//__device__ inline bool d_gpw_Get_Position_in_World3(Vector3f in_vox, 
//	const float *origin, const int *dim_cube, const float sz_vox,
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
//	out_p3d.x = __fmaf_rn((in_vox.x + 0.5f),sz_vox, origin[0]);
//	out_p3d.y = __fmaf_rn((in_vox.y + 0.5f),sz_vox, origin[1]);
//	out_p3d.z = __fmaf_rn((in_vox.z + 0.5f),sz_vox, origin[2]);
//
//	return valid;
//}

//********************************************************************************************
__device__ bool d_gtvu_Get_TSDF_Value_Uninterpolated(
	Vector3f in_vox, 
	const float *vol_tsdf,
	const uchar *vol_w,
	float &out_tsdf)
//********************************************************************************************
{
	int ww, hh, dd;
	int x, y, z, tidx;

	out_tsdf = 1.0f;	// set default value.

	// calculate local indices.		
	x = ROUNDF(in_vox.x);	y = ROUNDF(in_vox.y);	z = ROUNDF(in_vox.z);
	//x = int(in_vox.x);	y = int(in_vox.y);	z = int(in_vox.z);

	if(x<0 || x>=dim_cube_dev[0] || 
	   y<0 || y>=dim_cube_dev[1] || 
	   z<0 || z>=dim_cube_dev[2]) 
	   return false; 
	
 	tidx = z*dim_cube_dev[0]*dim_cube_dev[1] + y*dim_cube_dev[0] + x;
	 
 	if(vol_w[tidx] <= uchar(0)) return false;
 
 	out_tsdf = vol_tsdf[tidx];

	return true;
	
}

//********************************************************************************************
__device__ bool d_gtvi_Get_TSDF_Value_Interpolated(	
	Vector3f in_vox,
	const float *vol_tsdf,
	const uchar *vol_w,
	float &out_tsdf)
//********************************************************************************************
{
	Vector3i offset;
	Vector3f residu;
	Vector3f tpos;

	float inter_f, inter_b;
	float front[4], back[4];
	float dx1[4], dy1[4];
	int i;

	//     5-----6
	//  1=====2
	//     7-----8
	//  3=====4
	/// Get TSDF values of 8 neighbor voxels.
	out_tsdf = 1.0f;
	// get offset voxel index and float residual vector of input 3d point.
	offset = in_vox.toInt(residu);

	// set delta x, y for slice access.
	dx1[0] = 0.0f; dx1[1] = 1.0f; dx1[2] = 0.0f; dx1[3] = 1.0f;
	dy1[0] = 0.0f; dy1[1] = 0.0f; dy1[2] = 1.0f; dy1[3] = 1.0f;

	// get TSDF values of neighbors on the front plane.
	tpos.z = offset.z;	// set Z value.
	for(i = 0; i<4; i++){
		tpos.x = offset.x + dx1[i]; tpos.y = offset.y + dy1[i];
		if(!d_gtvu_Get_TSDF_Value_Uninterpolated(tpos, vol_tsdf, vol_w, front[i])) return false;
	}

	//// get TSDF values of neighbors on the back plane.
	tpos.z = offset.z + 1.0f;	// set Z value.
	for(i = 0; i<4; i++){
		tpos.x = offset.x + dx1[i]; tpos.y = offset.y + dy1[i];
		if(!d_gtvu_Get_TSDF_Value_Uninterpolated(tpos, vol_tsdf, vol_w, back[i])) return false;
	}

	
	// interpolates TSDF value on XY plane.
	inter_f = (1.0f - residu.y)*((1.0f - residu.x)*front[0] + residu.x*front[1])
		+ residu.y*((1.0f - residu.x)*front[2] + residu.x*front[3]);
	inter_b = (1.0f - residu.y)*((1.0f - residu.x)*back[0] + residu.x*back[1])
		+ residu.y*((1.0f - residu.x)*back[2] + residu.x*back[3]);
 
 	out_tsdf = (1.0f - residu.z)*inter_f + residu.z*inter_b;

	return true;
}


//********************************************************************************************
__device__ bool d_grvu_Get_RGB_Value_Uninterpolated(
	Vector3f in_vox,
	const uchar *vol_color,
	const uchar *vol_w,
	Vector3u &out_color)
//********************************************************************************************
{
	int ww,hh,dd;
	int x,y,z,tidx;

	out_color = uchar(0);	// set default value.

	ww = dim_cube_dev[0]; hh = dim_cube_dev[1]; dd = dim_cube_dev[2];

	// calculate local indices.		
	x = ROUNDF(in_vox.x);	y = ROUNDF(in_vox.y);	z = ROUNDF(in_vox.z);
	//x = int(in_vox.x);	y = int(in_vox.y);	z = int(in_vox.z);

	if(x<0 || x>=ww ||
	   y<0 || y>=hh ||
	   z<0 || z>=dd)
	   return false;

	tidx = z*ww*hh + y*ww + x;

	if(vol_w[tidx] <= uchar(0)) return false;

	//////////////////////////////////////////////////////////////////////////
	out_color.x = vol_color[tidx+ww*hh*dd];
	//////////////////////////////////////////////////////////////////////////
	out_color.y = vol_color[tidx+ww*hh*dd];
	out_color.z = vol_color[tidx+2*ww*hh*dd];

	return true;
}

//********************************************************************************************
__device__ bool d_grvu_Get_RGB_Value_Uninterpolated(
	Vector3f in_vox,
	const uchar *vol_color,
	const uchar *vol_w,
	Vector3f &out_color)
//********************************************************************************************
{
	int ww,hh,dd;
	int x,y,z,tidx;

	out_color = 0.0f;	// set default value.

	ww = dim_cube_dev[0]; hh = dim_cube_dev[1]; dd = dim_cube_dev[2];

	// calculate local indices.		
	x = ROUNDF(in_vox.x);	y = ROUNDF(in_vox.y);	z = ROUNDF(in_vox.z);
	//x = int(in_vox.x);	y = int(in_vox.y);	z = int(in_vox.z);

	if(x<0 || x>=ww ||
	   y<0 || y>=hh ||
	   z<0 || z>=dd)
	   return false;

	tidx = z*ww*hh + y*ww + x;

	//////////////////////////////////////////////////////////////////////////
	// 이부분?
	if(vol_w[tidx] <= uchar(0)) return false;
	//////////////////////////////////////////////////////////////////////////

	out_color.x = float(vol_color[tidx]);		// __int2float_rn
	out_color.y = float(vol_color[tidx+ww*hh*dd]);
	out_color.z = float(vol_color[tidx+2*ww*hh*dd]);

	return true;
}

//********************************************************************************************
__device__ bool d_grvi_Get_RGB_Value_Interpolated(
	Vector3f in_vox,
	const uchar *vol_color,
	const uchar *vol_w,
	Vector3u &out_color)
//********************************************************************************************
{
	Vector3i offset;
	Vector3f residu;
	Vector3f tpos;

	Vector3f inter_f,inter_b;
	Vector3f front[4],back[4];
	float dx1[4],dy1[4];
	int i;

	//     5-----6
	//  1=====2
	//     7-----8
	//  3=====4
	/// Get RGB values of 8 neighbor voxels.
	out_color = uchar(0);
	// get offset voxel index and float residual vector of input 3d point.
	offset = in_vox.toInt(residu);

	// set delta x, y for slice access.
	dx1[0] = 0.0f; dx1[1] = 1.0f; dx1[2] = 0.0f; dx1[3] = 1.0f;
	dy1[0] = 0.0f; dy1[1] = 0.0f; dy1[2] = 1.0f; dy1[3] = 1.0f;

	// get RGB values of neighbors on the front plane.
	tpos.z = offset.z;	// set Z value.
	for(i = 0; i<4; i++){
		tpos.x = offset.x + dx1[i]; tpos.y = offset.y + dy1[i];
		if(!d_grvu_Get_RGB_Value_Uninterpolated(tpos,vol_color,vol_w,front[i])) return false;
	}

	//// get RGB values of neighbors on the back plane.
	tpos.z = offset.z + 1.0f;	// set Z value.
	for(i = 0; i<4; i++){
		tpos.x = offset.x + dx1[i]; tpos.y = offset.y + dy1[i];
		if(!d_grvu_Get_RGB_Value_Uninterpolated(tpos,vol_color,vol_w,back[i])) return false;
	}

	// interpolates RGB value on XY plane.
	inter_f = (1.0f - residu.y)*((1.0f - residu.x)*front[0] + residu.x*front[1])
		+ residu.y*((1.0f - residu.x)*front[2] + residu.x*front[3]);
	inter_b = (1.0f - residu.y)*((1.0f - residu.x)*back[0] + residu.x*back[1])
		+ residu.y*((1.0f - residu.x)*back[2] + residu.x*back[3]);
	
	out_color = ((1.0f - residu.z)*inter_f + residu.z*inter_b).toUChar();
	
	return true;
}


//********************************************************************************************
__device__ bool d_csnt_Compute_Surface_Normal_from_TSDF(
	Vector3f in_p3d, 
	const float *vol_tsdf,
	const uchar *vol_w,
	Vector3f &out_surf_norm)
//********************************************************************************************
{
	// we need total 32 neighbors for computing a single surface normal of input 3D point. (refer following pictures.)
	// the offset voxel index is 4 in front XY plane.
	// + the foremost XY slice (Z=-1)
	//    -- X      
	// Y |
	//       1     2      
	//          X
	//       3     4     
	//
	//            
	//float XY_foremost[4];
	// + front XY slice (Z=0)
	//       1     2
	//
	// 3     4     5     6
	//          X
	// 7     8     9     10
	//
	//       11    12
	//float XY_front[12];
	// + back XY slice (Z=1)
	//       1     2
	//
	// 3     4     5     6
	//          X
	// 7     8     9     10
	//
	//       11    12
	//float XY_back[12];
	// + the backmost XY slice (Z=2)
	//       
	//
	//       1     2      
	//          X
	//       3     4     
	//
	//            
	//float XY_backmost[4];

	/// Get TSDF values of 32 neighbor voxels.
 	Vector3i offset;
 	Vector3f residu;
 	Vector3f tpos, vox, norm;
 	
 	float sz_voxel, gx, gy, gz;
 	float foremost[4], front[12], back[12], backmost[4];
 	float dx1[4], dy1[4], dx2[12], dy2[12];

	// coverts input 3D point to position of voxel coordinates.
 	if(!d_gpv_Get_Position_in_Voxel(in_p3d, origin_dev, dim_cube_dev, sz_vox_inv_dev[0], vox)) return false;
 
 	// get offset voxel index and float residual vector of input 3d point.
 	offset = vox.toInt(residu);
 
 	// set delta x, y for slice access.
 	dx1[0] = 0.0f; dx1[1] = 1.0f; dx1[2] = 0.0f; dx1[3] = 1.0f;
 	dy1[0] = 0.0f; dy1[1] = 0.0f; dy1[2] = 1.0f; dy1[3] = 1.0f;
 
 	dx2[0] = 0.0f;   dx2[1] = 1.0f;
 	dx2[2] = -1.0f;  dx2[3] = 0.0f; dx2[4] = 1.0f; dx2[5] = 2.0f;
 	dx2[6] = -1.0f;  dx2[7] = 0.0f; dx2[8] = 1.0f; dx2[9] = 2.0f;
 	dx2[10] = 0.0f;  dx2[11] = 1.0f;
 
 	dy2[0] = -1.0f;  dy2[1] = -1.0f;
 	dy2[2] = 0.0f;  dy2[3] = 0.0f;  dy2[4] = 0.0f; dy2[5] = 0.0f;
 	dy2[6] = 1.0f;  dy2[7] = 1.0f;  dy2[8] = 1.0f; dy2[9] = 1.0f;
 	dy2[10] = 2.0f; dy2[11] = 2.0f;

	// get TSDF values of neighbors on the foremost XY plane. (Z = -1)
	tpos.z = offset.z - 1.0f;
	for(int i=0; i<4; i++){
		tpos.x = offset.x + dx1[i]; tpos.y = offset.y + dy1[i];
		if(!d_gtvu_Get_TSDF_Value_Uninterpolated(tpos, vol_tsdf, vol_w, foremost[i])) return false;
	}
	// get TSDF values of neighbors on the front XY plane. (Z = 0)
	tpos.z = offset.z;
	for(int i = 0; i<12; i++){
		tpos.x = offset.x + dx2[i]; tpos.y = offset.y + dy2[i];
		if(!d_gtvu_Get_TSDF_Value_Uninterpolated(tpos, vol_tsdf, vol_w, front[i])) return false;
	}
	// get TSDF values of neighbors on the back XY plane. (Z = 1)
	tpos.z = offset.z + 1.0f;
	for(int i = 0; i<12; i++){
		tpos.x = offset.x + dx2[i]; tpos.y = offset.y + dy2[i];
		if(!d_gtvu_Get_TSDF_Value_Uninterpolated(tpos, vol_tsdf, vol_w, back[i])) return false;
	}
	// get TSDF values of neighbors on the foremost XY plane. (Z = 2)
	tpos.z = offset.z + 2.0f;
	for(int i = 0; i<4; i++){
		tpos.x = offset.x + dx1[i]; tpos.y = offset.y + dy1[i];
		if(!d_gtvu_Get_TSDF_Value_Uninterpolated(tpos, vol_tsdf, vol_w, backmost[i])) return false;
	}

	
	/// Calculate surface normal.
	float inter2a[2], inter4a[4], inter2b[2], inter4b[4];
	// compute x-direction SDF gradient at point.
	// + get z-direction interpolated SDF values of (3 4 5 6) and (7 8 9 10).
	inter4a[0]=(1.0f-residu.z)*front[2] +residu.z*back[2];	
	inter4a[1]=(1.0f-residu.z)*front[3] +residu.z*back[3];
	inter4a[2]=(1.0f-residu.z)*front[4] +residu.z*back[4];	
	inter4a[3]=(1.0f-residu.z)*front[5] +residu.z*back[5];

	inter4b[0]=(1.0f-residu.z)*front[6] +residu.z*back[6];	
	inter4b[1]=(1.0f-residu.z)*front[7] +residu.z*back[7];
	inter4b[2]=(1.0f-residu.z)*front[8] +residu.z*back[8];	
	inter4b[3]=(1.0f-residu.z)*front[9] +residu.z*back[9];	
	
	// + dF(x, y, z)/dx = F(x+1, y, z)-F(x-1, y, z).
	inter2a[0]=(1.0f-residu.x)*inter4a[0] +residu.x*inter4a[1];	// x-direction interpolation of 3 and 4.
	inter2a[1]=(1.0f-residu.x)*inter4b[0] +residu.x*inter4b[1];	// x-direction interpolation of 7 and 8.
	inter2b[0]=(1.0f-residu.x)*inter4a[2] +residu.x*inter4a[3];	// x-direction interpolation of 5 and 6.
	inter2b[1]=(1.0f-residu.x)*inter4b[2] +residu.x*inter4b[3];	// x-direction interpolation of 9 and 10.

	gx=(1.0f-residu.y)*inter2b[0]+residu.y*inter2b[1];		// y-direction interpolation of (5 6) and (9 10).
	gx-=(1.0f-residu.y)*inter2a[0]+residu.y*inter2a[1];		// y-direction interpolation of (3 4) and (5 6).


	// compute y-direction SDF gradient at point.
	// + get z-direction interpolated SDF values of (1 4 8 11) and (2 5 9 12).
	inter4a[0]=(1.0f-residu.z)*front[0]	 +residu.z*back[0];	
	inter4a[1]=(1.0f-residu.z)*front[3]	 +residu.z*back[3];
	inter4a[2]=(1.0f-residu.z)*front[7]	 +residu.z*back[7];	
	inter4a[3]=(1.0f-residu.z)*front[10] +residu.z*back[10];

	inter4b[0]=(1.0f-residu.z)*front[1]	 +residu.z*back[1];	
	inter4b[1]=(1.0f-residu.z)*front[4]	 +residu.z*back[4];
	inter4b[2]=(1.0f-residu.z)*front[8]	 +residu.z*back[8];	
	inter4b[3]=(1.0f-residu.z)*front[11] +residu.z*back[11];

	// + dF(x, y, z)/dx = F(x, y+1, z)-F(x, y-1, z).
	inter2a[0]=(1.0f-residu.y)*inter4a[0] +residu.y*inter4a[1];	// y-direction interpolation of 1 and 4.
	inter2a[1]=(1.0f-residu.y)*inter4b[0] +residu.y*inter4b[1];	// y-direction interpolation of 2 and 5.
	inter2b[0]=(1.0f-residu.y)*inter4a[2] +residu.y*inter4a[3];	// y-direction interpolation of 8 and 11.
	inter2b[1]=(1.0f-residu.y)*inter4b[2] +residu.y*inter4b[3];	// y-direction interpolation of 9 and 12.


	gy=(1.0f-residu.x)*inter2b[0]	+residu.x*inter2b[1];			// x-direction interpolation of (8 11) and (9 12).
	gy-=(1.0f-residu.x)*inter2a[0]	+residu.x*inter2a[1];			// x-direction interpolation of (1 4) and (2 5).

	// compute z-direction SDF gradient at point.
	// + get y-direction interpolated SDF values of (X=0|Z=-1 0 1 2) and (X=1|Z=-1 0 1 2).
	inter4a[0]=(1.0f-residu.y)*foremost[0]	+residu.y*foremost[2];		
	inter4a[1]=(1.0f-residu.y)*front[3]		+residu.y*front[7];
	inter4a[2]=(1.0f-residu.y)*back[3]		+residu.y*back[7];				
	inter4a[3]=(1.0f-residu.y)*backmost[0]	+residu.y*backmost[2];

	inter4b[0]=(1.0f-residu.y)*foremost[1]	+residu.y*foremost[3];		
	inter4b[1]=(1.0f-residu.y)*front[4]		+residu.y*front[8];
	inter4b[2]=(1.0f-residu.y)*back[4]		+residu.y*back[8];				
	inter4b[3]=(1.0f-residu.y)*backmost[1]	+residu.y*backmost[3];
	
	// + dF(x, y, z)/dx = F(x, y, z+1)-F(x, y, z-1).
	inter2a[0]=(1.0f-residu.z)*inter4a[0] +residu.z*inter4a[1];	// z-direction interpolation of (X=0|Z=-1) and (X=0|Z=0).
	inter2a[1]=(1.0f-residu.z)*inter4b[0] +residu.z*inter4b[1];	// z-direction interpolation of (X=1|Z=-1) and (X=1|Z=0).
	inter2b[0]=(1.0f-residu.z)*inter4a[2] +residu.z*inter4a[3];	// z-direction interpolation of (X=0|Z=1) and (X=0|Z=2).
	inter2b[1]=(1.0f-residu.z)*inter4b[2] +residu.z*inter4b[3];	// z-direction interpolation of (X=1|Z=1) and (X=1|Z=2).

	gz=(1.0f-residu.x)*inter2b[0]  +residu.x*inter2b[1];			// x-direction interpolation of (X=0|Z= 1 2) and (X=1|Z= 1 2).
	gz-=(1.0f-residu.x)*inter2a[0] +residu.x*inter2a[1];			// x-direction interpolation of (X=0|Z=-1 0) and (X=1|Z=-1 0).

	if(gx==0.0f && gy==0.0f && gz==0.0f) return false;

	// save calculation result of surface normal.
	norm.x = gx; norm.y = gy; norm.z = gz;
	out_surf_norm = norm.normalised();

	return true;
	
}

__device__ bool d_crtc_Cast_Ray_on_TSDF_Cube_NEW(
	// variable parameters.
	Vector2f p2d,
	const float *T_gc, const float *T_cg,
	Vector3f cam_cen, Vector3f light, Vector3f norm_cc, float dist_cg_cen, float theta_max,
	// fixed parameters.
	const float *cube_tsdf,
	const uchar *cube_w,
	int ww, int hh,
	int lev_of_pyram,
	float r_cube, float mu, float sz_vox_inv, int dim_sc,
	// for output cross point.
	Vector3f &p3d)
{

	// RENDERING NOW!!!!!
	Vector3f p3d_c,p3d_s,p3d_e,vox_s,vox_e,rd_g,rd_vox;
	float cos_val,theta,mag,dist_min,dist_max;

	bool flag_valid = false;
	bool flag_result = false;
	int cnt = 0;
	float step_sz,step_sz_coarse,step_scale,sz_sub_cube,total_step,total_step_max,tsdf;

	enum { SEARCH_COARSE,SEARCH_FINE,BEHIND_SURFACE,SEARCH_FINE_BACK } state;

	dist_min = dist_cg_cen - r_cube;
	dist_max = dist_cg_cen + r_cube;

	// compute the end point of the pixel ray for the input pixel.
	d_bp_Back_Project(p2d,&K_dev[lev_of_pyram*4],dist_max,p3d_c);
	d_t_Transform(p3d_c,T_cg,p3d_e);

	// ==================================================================
	// Ray validity test
	// ==================================================================
	// compute the direction of the pixel ray for the input pixel using camera center and far plane of view frustum.
	rd_g = (p3d_e - cam_cen).normalised();

	// compute angle between the pixel ray and direction vector between the two centers.
	// check whether the ray crossed the object cube.
	//cos_val = rd.x*norm_cc.x + rd.y*norm_cc.y + rd.z*norm_cc.z;
	theta = acosf(dot(rd_g,norm_cc));

	if(theta > theta_max){ return false; }

	// ==================================================================
	// Ray casting in voxel coordinates.
	// ==================================================================
	// calculates start point and ray direction for ray-casting in voxel coordinates.	
	p3d_s = cam_cen + dist_min*rd_g;
	// converts points and parameter coordinates from the real global coordinates to the cube voxel coordinates. (mm->voxel)
	// CAUTION: the offset position of global voxel cube is (-0.5f, -0.5f, -0.5f) in the cube voxel coordinates.
	// compute scale factor for converting TSDF value to 'mm' unit.	
	step_scale = 0.9f*mu*sz_vox_inv;
	// ======================================================
	// 현재 coarse search 는 step_scale 로 진행해야 함.
	// 현재 ray 가 invalid 한 곳을 지나는 중인 경우 (tsdf 값이 없음) 
	// 현재 coarse search 보다 한 단계 위의 coarse search (step size = dim_sc) 로 진행해도 될 듯.
	step_sz = step_sz_coarse = step_scale; //fmaxf(step_scale+0.1f, float(dim_sc));
	// ======================================================
	// set maximum search length of the pixel ray to 2*in_radius_cube.
	total_step_max = 2.0f*r_cube*sz_vox_inv;		//	 total_step_max=pe_g.d_Distance(ps_g)*in_voxel_sz_inv;

	// starts casting ray.
	total_step = 0.0f;
	// Validity check 안해도 되나??
	d_gpv_Get_Position_in_Voxel(p3d_s,origin_dev,dim_cube_dev,sz_vox_inv_dev[0],vox_s);
	// Validity check 안해도 되나??
	flag_valid = d_gtvu_Get_TSDF_Value_Uninterpolated(vox_s,cube_tsdf,cube_w,tsdf);
	if(!flag_valid)		state = SEARCH_COARSE;
	else if(tsdf>=0.0f)	state = SEARCH_FINE;
	else				return false; //state = SEARCH_FINE_BACK;
	

	// ===================================================
	// Casts ray.
	// ===================================================
	while(state!=BEHIND_SURFACE){

		// ===================================================
		// Sets step size.
		// ===================================================
		if(state == SEARCH_COARSE)  step_sz = step_sz_coarse;
		else step_sz = (tsdf>0.0f) ? fmaxf(step_scale*tsdf,1.0f) : fminf(step_scale*tsdf,-1.0f);
//		else if(state == SEARCH_FINE) step_sz = fmaxf(step_scale*tsdf,0.1f);	// in front of surface.  
//		else if(state == SEARCH_FINE_BACK)  step_sz = fminf(step_scale*tsdf,-0.1f);  // behind surface.

		// ===================================================
		// Proceeds ray casting.
		// ===================================================
		vox_s += step_sz*rd_g;	//vox_s.x += step_sz*rd.x; vox_s.y += step_sz*rd.y; vox_s.z += step_sz*rd.z;
		total_step += step_sz;
		// if total step length exceeds maximum step size, break while loop.
		if(total_step > total_step_max)	break;

		// compute uninterpolated TSDF value.
		flag_valid = d_gtvu_Get_TSDF_Value_Uninterpolated(vox_s,cube_tsdf,cube_w,tsdf);

		// check which current points is in mu band or not. (near the surface)
		if(tsdf>-1.0f && tsdf<1.0f){
			// ===========================================================
			// Mu-band processing.
			// ===========================================================
			// compute trilinear interpolated TSDF value.
			flag_valid = d_gtvi_Get_TSDF_Value_Interpolated(vox_s,cube_tsdf,cube_w,tsdf);

			// ===================================================
			// Checks casting mode.
			// ===================================================
			// check current TSDF value is (-) at fine search step.
			if(tsdf<0.0f){
				// terminate ray casting if current TSDF value is (-) at coarse searching mode.
				if(state == SEARCH_COARSE) break;
				else state = BEHIND_SURFACE;
			}
			else if(tsdf == 0.0f){
				// on surface.
				d_gpw_Get_Position_in_World(vox_s,origin_dev,dim_cube_dev,sz_vox_inv_dev[0],p3d);
				flag_result=true;
				break;
			} 
			else if(state == SEARCH_COARSE)	state = SEARCH_FINE;
		} 
		else{
			if(state == SEARCH_FINE) state == SEARCH_COARSE;
		}

		//// ===================================================
		//// Checks casting mode.
		//// ===================================================
		//if(state == SEARCH_COARSE){
		//	if(tsdf<=0.0f) state = SEARCH_FINE_BACK; // go back.
		//	else           state = SEARCH_FINE;		 // start fine search.
		//} else if(state == SEARCH_FINE){
		//	// ================================================================
		//	// Infinite Loop Cause 1
		//	// : In case of noisy TSDF data processing.
		//	// Solution: Remove noisy data processing part.
		//	// if(tsdf >= 1.0f || tsdf <= -1.0f) state = SEARCH_COARSE;  // convert to coarse search. (ray met noise voxel data)
		//	// ================================================================
		//	if(tsdf<=0.0f)  state = BEHIND_SURFACE; // break while loop, and find intersection.
		//} else if(state == SEARCH_FINE_BACK){
		//	if(tsdf>0.0f) state = SEARCH_FINE;     // start fine search.
		//}

		// for debugging.
		//if(cnt++ > 100)	break;
		// for debugging.
	}

	// ===================================================
	// Find intersection between ray and surface.
	// ===================================================
	if(state==BEHIND_SURFACE){

		// compute trilinear interpolated TSDF value for the last step.
		step_sz = step_scale*tsdf;	vox_s += step_sz*rd_g;
		d_gtvi_Get_TSDF_Value_Interpolated(vox_s,cube_tsdf,cube_w,tsdf);
		// compute the final step of current ray for extracting the surface point.
		step_sz = step_scale*tsdf;	vox_s += step_sz*rd_g;
		// converts points coordinates from the cube voxel coordinates to the real global coordinates . (voxel->mm)
		d_gpw_Get_Position_in_World(vox_s,origin_dev,dim_cube_dev,sz_vox_inv_dev[0],p3d);

		flag_result = true;
	}

	return flag_result;
	
}

__device__ bool d_crtc_Cast_Ray_on_TSDF_Cube(
	// variable parameters.
	Vector2f p2d,
	const float *T_gc,const float *T_cg,
	Vector3f cam_cen,Vector3f light,Vector3f norm_cc,float dist_cg_cen,float theta_max,
	// fixed parameters.
	const float *cube_tsdf,
	const uchar *cube_w,
	int ww,int hh,
	float r_cube,float mu,float sz_vox_inv,int dim_sc,
	// for output cross point.
	Vector3f &p3d)
{

	// RENDERING NOW!!!!!
	Vector3f p3d_c, p3d_s, p3d_e, vox_s, vox_e, rd_g, rd_vox;
	float cos_val, theta, mag, dist_min, dist_max;

	bool flag_valid = false;
	bool flag_result = false;
	int cnt = 0;
	float step_sz, step_sz_coarse, step_scale, sz_sub_cube, total_step, total_step_max, tsdf;

	// cast ray.
	enum { SEARCH_COARSE,SEARCH_FINE,BEHIND_SURFACE,WRONG_STEP } state;

	dist_min = dist_cg_cen - r_cube;
	dist_max = dist_cg_cen + r_cube;

	// compute the end point of the pixel ray for the input pixel.
	d_bp_Back_Project(p2d, K_dev, dist_max, p3d_c);
	d_t_Transform(p3d_c, T_cg, p3d_e);

	// ==================================================================
	// Ray validity test
	// ==================================================================
	// compute the direction of the pixel ray for the input pixel using camera center and far plane of view frustum.
	rd_g = (p3d_e - cam_cen).normalised();

	// compute angle between the pixel ray and direction vector between the two centers.
	// check whether the ray crossed the object cube.
	//cos_val = rd.x*norm_cc.x + rd.y*norm_cc.y + rd.z*norm_cc.z;
	theta = acosf(dot(rd_g, norm_cc));

	if(theta > theta_max){ return false; }

	//// ==================================================================
	//// Ray casting in voxel coordinates.
	//// ==================================================================
	// initialization.
	// calculates start point and ray direction for ray-casting.	
	p3d_s=cam_cen+dist_min*rd_g;
	// converts points and parameter coordinates from the real global coordinates to the cube voxel coordinates. (mm->voxel)
	// CAUTION: the offset position of global voxel cube is (-0.5f, -0.5f, -0.5f) in the cube voxel coordinates.
	d_gpv_Get_Position_in_Voxel(p3d_s, origin_dev, dim_cube_dev, sz_vox_inv_dev[0], vox_s);
	// compute scale factor for converting TSDF value to 'mm' unit.	
	step_scale=mu*sz_vox_inv;
	// set initial and coarse-search step size for ray-casting as the side length of the sub cube.	
	// + coarse-search step size should be larger than 'step_scale'.
	sz_sub_cube=dim_sc;
	step_sz=step_sz_coarse=fmaxf(step_scale+0.1f,sz_sub_cube);
	// set maximum search length of the pixel ray to 2*in_radius_cube.
	total_step_max=2.0f*r_cube*sz_vox_inv;		//	 total_step_max=pe_g.d_Distance(ps_g)*in_voxel_sz_inv;

	// starts casting ray.
	total_step=0.0f;
	flag_valid=d_gtvu_Get_TSDF_Value_Uninterpolated(vox_s,cube_tsdf,cube_w,tsdf);
	if(!flag_valid)		state=SEARCH_COARSE;
	else if(tsdf<=0.0f)	state=WRONG_STEP;
	else				state=SEARCH_FINE;

	while(state!=BEHIND_SURFACE){
		if(!flag_valid){
			switch(state){

			case SEARCH_COARSE:
				step_sz=step_sz_coarse;
				break;

			default:
			case WRONG_STEP:
			case SEARCH_FINE:
				state=SEARCH_COARSE;
				step_sz=step_sz_coarse;
				break;
			}
		} else{
			switch(state){

			case SEARCH_COARSE:
				// at the first SEARCH_COARSE state, 
				// return previous step (the last SEARCH_BLOCK_COARSE state) and do fine search
				// stepLength: SDF_BLOCK_SIZE-> sdfValue * stepScale (at the first fine search, assume that sdfValue is 1.0f).
				//state=BEHIND_SURFACE;
				state=SEARCH_FINE;
				step_sz=step_scale*tsdf;
				break;
			case WRONG_STEP:
				step_sz=fminf(step_scale*tsdf,-1.0f);
				break;

			default:
			case SEARCH_FINE:
				step_sz=fmaxf(step_scale*tsdf,1.0f);
			}
		}

		// proceeds ray casting.
		vox_s+=step_sz*rd_g;	total_step+=step_sz;
		if(total_step>total_step_max)	break;

		flag_valid=d_gtvu_Get_TSDF_Value_Uninterpolated(vox_s,cube_tsdf,cube_w,tsdf);

		if(tsdf>-1.0f && tsdf<1.0f){
			// add interpolation version of TSDF value calculator.
			d_gtvi_Get_TSDF_Value_Interpolated(vox_s,cube_tsdf,cube_w,tsdf);
		}

		if(tsdf<0.0f){			// behind surface.
			if(state==SEARCH_COARSE)	state=WRONG_STEP;
			else						state=BEHIND_SURFACE;	//else if(state==SEARCH_FINE)		state=BEHIND_SURFACE;	
		} 
		else if(tsdf>0.0f){		// in front of surface.
			if(state==WRONG_STEP)	state=SEARCH_FINE;
		} 
		else{					// on surface.
			// converts points coordinates from the cube voxel coordinates to the real global coordinates . (voxel->mm)
			d_gpw_Get_Position_in_World(vox_s, origin_dev, dim_cube_dev, sz_vox_inv_dev[0], p3d);

			return true;
		}

	}

	if(state==BEHIND_SURFACE){

		step_sz=step_scale*tsdf;	vox_s+=step_sz*rd_g;
		//in_cube->gtvu_Get_TSDF_Value_Uninterpolated(ps_vox, tsdf);
		d_gtvi_Get_TSDF_Value_Interpolated(vox_s,cube_tsdf,cube_w,tsdf);

		step_sz=step_scale*tsdf;	vox_s+=step_sz*rd_g;

		// converts points coordinates from the cube voxel coordinates to the real global coordinates . (voxel->mm)
		d_gpw_Get_Position_in_World(vox_s,origin_dev,dim_cube_dev,sz_vox_inv_dev[0],p3d);

		flag_result=true;
	}

	return flag_result;

	//dist_min = dist_cg_cen - r_cube;
	//dist_max = dist_cg_cen + r_cube;

	//// compute the end point of the pixel ray for the input pixel.
	//d_bp_Back_Project(p2d, K_dev, dist_max, p3d_c);
	//d_t_Transform(p3d_c, T_cg, p3d_e);

	//// ==================================================================
	//// Ray validity test
	//// ==================================================================
	//// compute the direction of the pixel ray for the input pixel using camera center and far plane of view frustum.
	//rd_g = (p3d_e - cam_cen).normalised();

	//// compute angle between the pixel ray and direction vector between the two centers.
	//// check whether the ray crossed the object cube.
	////cos_val = rd.x*norm_cc.x + rd.y*norm_cc.y + rd.z*norm_cc.z;
	//theta = acosf(dot(rd_g, norm_cc));

	//if(theta > theta_max){ return false; }

	//// ==================================================================
	//// Ray casting in voxel coordinates.
	//// ==================================================================
	//// calculates start point and ray direction for ray-casting in voxel coordinates.	
	//p3d_s = cam_cen + dist_min*rd_g;
	//// converts points and parameter coordinates from the real global coordinates to the cube voxel coordinates. (mm->voxel)
	//// CAUTION: the offset position of global voxel cube is (-0.5f, -0.5f, -0.5f) in the cube voxel coordinates.
	//// compute scale factor for converting TSDF value to 'mm' unit.	
	//step_scale = mu*sz_vox_inv;
	//// ======================================================
	//// 현재 coarse search 는 step_scale 로 진행해야 함.
	//// 현재 ray 가 invalid 한 곳을 지나는 중인 경우 (tsdf 값이 없음) 
	//// 현재 coarse search 보다 한 단계 위의 coarse search (step size = dim_sc) 로 진행해도 될 듯.
	//step_sz = step_sz_coarse = step_scale; //fmaxf(step_scale+0.1f, float(dim_sc));
	//// ======================================================
	//// set maximum search length of the pixel ray to 2*in_radius_cube.
	//total_step_max = 2.0f*r_cube*sz_vox_inv;		//	 total_step_max=pe_g.d_Distance(ps_g)*in_voxel_sz_inv;

	//// starts casting ray.
	//total_step = 0.0f;
	//// Validity check 안해도 되나??
	//d_gpv_Get_Position_in_Voxel(p3d_s, origin_dev, dim_cube_dev, sz_vox_inv_dev[0], vox_s);
	//// Validity check 안해도 되나??
	//flag_valid = d_gtvu_Get_TSDF_Value_Uninterpolated(vox_s, cube_tsdf, cube_w, tsdf);
	//if(!flag_valid)		state = SEARCH_COARSE;
	//else if(tsdf<=0.0f)	state = SEARCH_FINE_BACK;
	//else				state = SEARCH_FINE;

	//// ===================================================
	//// Casts ray.
	//// ===================================================
	//while(state!=BEHIND_SURFACE){

	//	// ===================================================
	//	// Sets step size.
	//	// ===================================================
	//	if(state == SEARCH_COARSE)    step_sz = step_sz_coarse;
	//	else if(state == SEARCH_FINE) step_sz = fmaxf(step_scale*tsdf, 1.0f);	// in front of surface.  
	//	else if(state == SEARCH_FINE_BACK)  step_sz = fminf(step_scale*tsdf, -1.0f);  // behind surface.

	//	// ===================================================
	//	// Proceeds ray casting.
	//	// ===================================================
	//	vox_s += step_sz*rd_g;	//vox_s.x += step_sz*rd.x; vox_s.y += step_sz*rd.y; vox_s.z += step_sz*rd.z;
	//	total_step += step_sz;
	//	// if total step length exceeds maximum step size, break while loop.
	//	if(total_step > total_step_max)	break;

	//	flag_valid = d_gtvu_Get_TSDF_Value_Uninterpolated(vox_s, cube_tsdf, cube_w, tsdf);

	//	// approached near the surface.
	//	if(tsdf>-1.0f && tsdf<1.0f){
	//		// add interpolation version of TSDF value calculator.
	//		flag_valid = d_gtvi_Get_TSDF_Value_Interpolated(vox_s, cube_tsdf, cube_w, tsdf);
	//	}

	//	// ===================================================
	//	// Checks casting mode.
	//	// ===================================================
	//	if(state == SEARCH_COARSE){
	//		if(tsdf<=0.0f) state = SEARCH_FINE_BACK; // go back.
	//		else if(tsdf<1.0f)    state = SEARCH_FINE;		 // start fine search.
	//	}
	//	else if(state == SEARCH_FINE){
	//		// ================================================================
	//		// Infinite Loop Cause 1
	//		// : In case of noisy TSDF data processing.
	//		// Solution: Remove noisy data processing part.
	//		// if(tsdf >= 1.0f || tsdf <= -1.0f) state = SEARCH_COARSE;  // convert to coarse search. (ray met noise voxel data)
	//		// ================================================================
	//		if(tsdf<=0.0f)  state = BEHIND_SURFACE; // break while loop, and find intersection.
	//	}
	//	else if(state == SEARCH_FINE_BACK){
	//		if(tsdf>0.0f) state = SEARCH_FINE;     // start fine search.
	//	}

	//	// for debugging.
	//	//if(cnt++ > 100)	break;
	//	// for debugging.
	//}

	//// ===================================================
	//// Find intersection between ray and surface.
	//// ===================================================
	//if(state==BEHIND_SURFACE){

	//	// compute trilinear interpolated TSDF value for the last step.
	//	step_sz = step_scale*tsdf;	vox_s += step_sz*rd_g;
	//	d_gtvi_Get_TSDF_Value_Interpolated(vox_s, cube_tsdf, cube_w, tsdf);
	//	// compute the final step of current ray for extracting the surface point.
	//	step_sz = step_scale*tsdf;	vox_s += step_sz*rd_g;
	//	// converts points coordinates from the cube voxel coordinates to the real global coordinates . (voxel->mm)
	//	d_gpw_Get_Position_in_World(vox_s, origin_dev, dim_cube_dev, sz_vox_inv_dev[0], p3d);

	//	flag_result = true;
	//}

	//return flag_result;

}

__device__ bool d_crtcs_Cast_Ray_on_TSDF_Cube_for_Scene(
	// variable parameters.
	Vector2f p2d,
	const float *T_gc,const float *T_cg,
	Vector3f cam_cen,Vector3f light,
	// fixed parameters.
	const float *cube_tsdf,
	const uchar *cube_w,
	int ww,int hh,
	float r_cube,float mu,float sz_vox_inv,int dim_sc,
	// for output cross point.
	Vector3f &p3d)
{
	// RENDERING NOW!!!!!
	Vector3f p3d_c,p3d_s,p3d_e,vox_s,vox_e,rd_g,rd_vox;
	float mag,dist_min,dist_max;

	bool flag_valid = false;
	bool flag_result = false;
	int cnt = 0;
	float step_sz,step_sz_coarse,step_scale,sz_sub_cube,total_step,total_step_max,tsdf;

	enum { SEARCH_COARSE,SEARCH_FINE,BEHIND_SURFACE,SEARCH_FINE_BACK } state;

	dist_min = 0.0f;
	dist_max = 2*r_cube;

	// compute the end point of the pixel ray for the input pixel.
	d_bp_Back_Project(p2d,K_dev,dist_max,p3d_c);
	d_t_Transform(p3d_c,T_cg,p3d_e);

	// ==================================================================
	// Ray validity test
	// ==================================================================
	// compute the direction of the pixel ray for the input pixel using camera center and far plane of view frustum.
	rd_g = (p3d_e - cam_cen).normalised();

	// ==================================================================
	// Ray casting in voxel coordinates.
	// ==================================================================
	// calculates start point and ray direction for ray-casting in voxel coordinates.	
	p3d_s = cam_cen + dist_min*rd_g;
	// converts points and parameter coordinates from the real global coordinates to the cube voxel coordinates. (mm->voxel)
	// CAUTION: the offset position of global voxel cube is (-0.5f, -0.5f, -0.5f) in the cube voxel coordinates.
	// compute scale factor for converting TSDF value to 'mm' unit.	
	step_scale = 0.9f*mu*sz_vox_inv;
	// ======================================================
	// 현재 coarse search 는 step_scale 로 진행해야 함.
	// 현재 ray 가 invalid 한 곳을 지나는 중인 경우 (tsdf 값이 없음) 
	// 현재 coarse search 보다 한 단계 위의 coarse search (step size = dim_sc) 로 진행해도 될 듯.
	step_sz = step_sz_coarse = step_scale; //fmaxf(step_scale+0.1f, float(dim_sc));
	// ======================================================
	// set maximum search length of the pixel ray to 2*in_radius_cube.
	total_step_max = 2.0f*r_cube*sz_vox_inv;		//	 total_step_max=pe_g.d_Distance(ps_g)*in_voxel_sz_inv;

	// starts casting ray.
	total_step = 0.0f;
	// Validity check 안해도 되나??
	d_gpv_Get_Position_in_Voxel(p3d_s,origin_dev,dim_cube_dev,sz_vox_inv_dev[0],vox_s);
	// Validity check 안해도 되나??
	flag_valid = d_gtvu_Get_TSDF_Value_Uninterpolated(vox_s,cube_tsdf,cube_w,tsdf);
	if(!flag_valid)		state = SEARCH_COARSE;
	else if(tsdf>=0.0f)	state = SEARCH_FINE;
	else				return false; //state = SEARCH_FINE_BACK;


	// ===================================================
	// Casts ray.
	// ===================================================
	while(state!=BEHIND_SURFACE){

		// ===================================================
		// Sets step size.
		// ===================================================
		if(state == SEARCH_COARSE)  step_sz = step_sz_coarse;
		else step_sz = (tsdf>0.0f) ? fmaxf(step_scale*tsdf,1.0f) : fminf(step_scale*tsdf,-1.0f);
		//		else if(state == SEARCH_FINE) step_sz = fmaxf(step_scale*tsdf,0.1f);	// in front of surface.  
		//		else if(state == SEARCH_FINE_BACK)  step_sz = fminf(step_scale*tsdf,-0.1f);  // behind surface.

		// ===================================================
		// Proceeds ray casting.
		// ===================================================
		vox_s += step_sz*rd_g;	//vox_s.x += step_sz*rd.x; vox_s.y += step_sz*rd.y; vox_s.z += step_sz*rd.z;
		total_step += step_sz;
		// if total step length exceeds maximum step size, break while loop.
		if(total_step > total_step_max)	break;

		// compute uninterpolated TSDF value.
		flag_valid = d_gtvu_Get_TSDF_Value_Uninterpolated(vox_s,cube_tsdf,cube_w,tsdf);

		// check which current points is in mu band or not. (near the surface)
		if(tsdf>-1.0f && tsdf<1.0f){
			// ===========================================================
			// Mu-band processing.
			// ===========================================================
			// compute trilinear interpolated TSDF value.
			flag_valid = d_gtvi_Get_TSDF_Value_Interpolated(vox_s,cube_tsdf,cube_w,tsdf);

			// ===================================================
			// Checks casting mode.
			// ===================================================
			// check current TSDF value is (-) at fine search step.
			if(tsdf<0.0f){
				// terminate ray casting if current TSDF value is (-) at coarse searching mode.
				if(state == SEARCH_COARSE) break;
				else state = BEHIND_SURFACE;
			} else if(tsdf == 0.0f){
				// on surface.
				d_gpw_Get_Position_in_World(vox_s,origin_dev,dim_cube_dev,sz_vox_inv_dev[0],p3d);
				flag_result=true;
				break;
			} else if(state == SEARCH_COARSE)	state = SEARCH_FINE;
		} else{
			if(state == SEARCH_FINE) state == SEARCH_COARSE;
		}

		//// ===================================================
		//// Checks casting mode.
		//// ===================================================
		//if(state == SEARCH_COARSE){
		//	if(tsdf<=0.0f) state = SEARCH_FINE_BACK; // go back.
		//	else           state = SEARCH_FINE;		 // start fine search.
		//} else if(state == SEARCH_FINE){
		//	// ================================================================
		//	// Infinite Loop Cause 1
		//	// : In case of noisy TSDF data processing.
		//	// Solution: Remove noisy data processing part.
		//	// if(tsdf >= 1.0f || tsdf <= -1.0f) state = SEARCH_COARSE;  // convert to coarse search. (ray met noise voxel data)
		//	// ================================================================
		//	if(tsdf<=0.0f)  state = BEHIND_SURFACE; // break while loop, and find intersection.
		//} else if(state == SEARCH_FINE_BACK){
		//	if(tsdf>0.0f) state = SEARCH_FINE;     // start fine search.
		//}

		// for debugging.
		//if(cnt++ > 100)	break;
		// for debugging.
	}

	// ===================================================
	// Find intersection between ray and surface.
	// ===================================================
	if(state==BEHIND_SURFACE){

		// compute trilinear interpolated TSDF value for the last step.
		step_sz = step_scale*tsdf;	vox_s += step_sz*rd_g;
		d_gtvi_Get_TSDF_Value_Interpolated(vox_s,cube_tsdf,cube_w,tsdf);
		// compute the final step of current ray for extracting the surface point.
		step_sz = step_scale*tsdf;	vox_s += step_sz*rd_g;
		// converts points coordinates from the cube voxel coordinates to the real global coordinates . (voxel->mm)
		d_gpw_Get_Position_in_World(vox_s,origin_dev,dim_cube_dev,sz_vox_inv_dev[0],p3d);

		flag_result = true;
	}

	return flag_result;

}



__global__ void g_rmi_Render_Maps_for_ICP(
	//const float *T_gc_dev, const float *T_cg_dev,
	Vector3f cam_cen, Vector3f light, Vector3f norm_cc, float dist_cg_cen, float theta_max,
	// fixed parameters.
	const float *cube_tsdf,
	const uchar *cube_color,
	const uchar *cube_w,	

	int lev_of_pyramid,
	// output.
	float *io_map_depth, float *io_map_vertex,
	float *io_map_normals, uchar *io_img_normals,
	uchar *io_img_color)
{
	Vector2f p2d; Vector3f p3d_g, p3d_c, p3d_v, sn_g, sn_c; Vector3u rgb;	

	bool valid = false;
	int tidx;
	float surf_angle;

	int tx = threadIdx.x + blockIdx.x*blockDim.x;
	int ty = threadIdx.y + blockIdx.y*blockDim.y;

	int ww = dim_map_dev[lev_of_pyramid*2 + 0];
	int hh = dim_map_dev[lev_of_pyramid*2 + 1];

	if(tx < 0 || tx >= ww || ty < 0 || ty >= hh)	return;

	//	if(tx % 3 || ty % 3)	return ;

	tidx = ty*ww + tx;

	// do ray casting.
	p2d.x = tx;	p2d.y = ty;
	valid = d_crtc_Cast_Ray_on_TSDF_Cube_NEW(
	//valid = d_crtc_Cast_Ray_on_TSDF_Cube(
		p2d,
		T_gc_dev_const, T_cg_dev_const,
		//T_gc_dev, T_cg_dev,
		cam_cen, light, norm_cc, dist_cg_cen, theta_max,

		cube_tsdf,
		cube_w,
// 		K_dev,
// 		origin_dev,
// 		dim_cube_dev,
//		sz_vox_inv_dev[0],
		ww, hh,
		lev_of_pyramid,

		r_cube_dev[0], mu_dev[0], sz_vox_inv_dev[0], dim_sc_dev[0],

		p3d_g);

	// update maps.
	if(valid){
		if(d_csnt_Compute_Surface_Normal_from_TSDF(
			p3d_g, 
			cube_tsdf, cube_w, 
// 			origin_dev,
// 			dim_cube_dev,
//			sz_vox_inv_dev[0],
			sn_g)){

			// set depth.
			d_t_Transform(p3d_g, T_gc_dev_const, p3d_c);
			io_map_depth[tidx] = p3d_c.z;

			//////////////////////////////////////////////////////////////////////////
			// + in global coordinates.
			//////////////////////////////////////////////////////////////////////////
// 			// set vertex.
// 			io_map_vertex[3*tidx] = p3d_g.x;
// 			io_map_vertex[3*tidx + 1] = p3d_g.y;
// 			io_map_vertex[3*tidx + 2] = p3d_g.z;
// 			// set normal.
//  		io_map_normals[3*tidx] = sn.x; 
//  		io_map_normals[3*tidx + 1] = sn.y; 
//  		io_map_normals[3*tidx + 2] = sn.z;

			//////////////////////////////////////////////////////////////////////////
			// + in local camera coordinates.
			//////////////////////////////////////////////////////////////////////////
			// set vertex.
			io_map_vertex[3*tidx] = p3d_c.x;
			io_map_vertex[3*tidx + 1] = p3d_c.y;
			io_map_vertex[3*tidx + 2] = p3d_c.z;
			// set normal.
			d_r_Rotate(sn_g,T_gc_dev_const,sn_c);
			io_map_normals[3*tidx] = sn_c.x;
			io_map_normals[3*tidx + 1] = sn_c.y;
			io_map_normals[3*tidx + 2] = sn_c.z;

			// set normal image.
			surf_angle = sn_c.x*light.x + sn_c.y*light.y + sn_c.z*light.z;
			io_img_normals[tidx] = (uchar)(fmaxf(0.0f, fminf(255.0f, (0.8f * surf_angle + 0.2f) * 255.0f)));
			// set RGB color image.
			if(io_img_color){
				d_gpv_Get_Position_in_Voxel(p3d_g, origin_dev, dim_cube_dev, sz_vox_inv_dev[0], p3d_v);
				
				d_grvi_Get_RGB_Value_Interpolated(p3d_v, cube_color, cube_w, rgb);

				io_img_color[tidx] = rgb.x;
				io_img_color[tidx+ww*hh] = rgb.y;
				io_img_color[tidx+2*ww*hh] = rgb.z;

				//////////////////////////////////////////////////////////////////////////
				// 임시 방편으로 G channel 값을 R channel에!!
// 				d_grvu_Get_RGB_Value_Uninterpolated(p3d_v,cube_color,cube_w,rgb);
// 				io_img_color[tidx] = rgb.y;
// 				io_img_color[tidx+ww*hh] = rgb.y;
// 				io_img_color[tidx+2*ww*hh] = rgb.z;
				//////////////////////////////////////////////////////////////////////////


			}
		}
		else valid = false;
	}
	
	if(!valid){
		io_map_depth[tidx] = 0.0f;
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// CPU version 에서 default value 를 -100.0f 로 한듯...	
// 		const char type = '0';
// 		io_map_vertex[3*tidx] = io_map_vertex[3*tidx + 1] = io_map_vertex[3*tidx + 2] = nanf(&type);
// 		io_map_normals[3*tidx] = io_map_normals[3*tidx + 1] = io_map_normals[3*tidx + 2] = nanf(&type);
		io_map_vertex[3*tidx] = io_map_vertex[3*tidx + 1] = io_map_vertex[3*tidx + 2] = -100.0f;
		io_map_normals[3*tidx] = io_map_normals[3*tidx + 1] = io_map_normals[3*tidx + 2] = -100.0f;
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		io_img_normals[tidx] = uchar(0);
		if(io_img_color){
			io_img_color[tidx] = uchar(0);
			io_img_color[tidx+ww*hh] = uchar(0);
			io_img_color[tidx+2*ww*hh] = uchar(0);

		}
	}


}

__global__ void g_rmi_Render_Maps_for_ICP(
	//const float *T_gc_dev, const float *T_cg_dev,
	Vector3f cam_cen,Vector3f light,Vector3f norm_cc,float dist_cg_cen,float theta_max,
	// fixed parameters.
	const float *cube_tsdf,
	const uchar *cube_color,
	const uchar *cube_w,
	int lev_of_pyramid,
	// output.
	float *io_map_depth,float *io_map_normals,uchar *io_img_normals,
	uchar *io_img_color)
{
	Vector2f p2d; Vector3f p3d_g,p3d_c,p3d_v,sn; Vector3u rgb;

	bool valid = false;
	int tidx;
	float surf_angle;

	int tx = threadIdx.x + blockIdx.x*blockDim.x;
	int ty = threadIdx.y + blockIdx.y*blockDim.y;

	int ww = dim_map_dev[lev_of_pyramid*2 + 0];
	int hh = dim_map_dev[lev_of_pyramid*2 + 1];

	if(tx < 0 || tx >= ww || ty < 0 || ty >= hh)	return;

	//	if(tx % 3 || ty % 3)	return ;

	tidx = ty*ww + tx;

	// do ray casting.
	p2d.x = tx;	p2d.y = ty;
	valid = d_crtc_Cast_Ray_on_TSDF_Cube_NEW(
	//valid = d_crtc_Cast_Ray_on_TSDF_Cube(
		p2d,
		T_gc_dev_const,T_cg_dev_const,
		//T_gc_dev, T_cg_dev,
		cam_cen,light,norm_cc,dist_cg_cen,theta_max,

		cube_tsdf,
		cube_w,
// 		K_dev,
// 		origin_dev,
// 		dim_cube_dev,
//		sz_vox_inv_dev[0],
		ww,hh,
		lev_of_pyramid,

		r_cube_dev[0],mu_dev[0],sz_vox_inv_dev[0],dim_sc_dev[0],
		p3d_g);

	// update maps.
	if(valid){
		if(d_csnt_Compute_Surface_Normal_from_TSDF(
			p3d_g,
			cube_tsdf,cube_w,
// 			origin_dev,
// 			dim_cube_dev,
//			sz_vox_inv_dev[0],
			sn)){

			surf_angle = sn.x*light.x + sn.y*light.y + sn.z*light.z;

			// set depth.
			d_t_Transform(p3d_g,T_gc_dev_const,p3d_c);
			io_map_depth[tidx] = p3d_c.z;
			// set normal.
			io_map_normals[3*tidx] = sn.x;
			io_map_normals[3*tidx + 1] = sn.y;
			io_map_normals[3*tidx + 2] = sn.z;
			// set normal image.
			io_img_normals[tidx] = (uchar)(fmaxf(0.0f,fminf(255.0f,(0.8f * surf_angle + 0.2f) * 255.0f)));
			// set RGB color image.
			if(io_img_color){
				d_gpv_Get_Position_in_Voxel(p3d_g,origin_dev,dim_cube_dev,sz_vox_inv_dev[0],p3d_v);
					
				d_grvi_Get_RGB_Value_Interpolated(p3d_v,cube_color,cube_w,rgb);

				io_img_color[tidx] = rgb.x;
				io_img_color[tidx+ww*hh] = rgb.y;
				io_img_color[tidx+2*ww*hh] = rgb.z;

				//////////////////////////////////////////////////////////////////////////
				// 임시 방편으로 G channel 값을 R channel에!!
// 					d_grvu_Get_RGB_Value_Uninterpolated(p3d_v,cube_color,cube_w,rgb);
// 					io_img_color[tidx] = rgb.y;
// 					io_img_color[tidx+ww*hh] = rgb.y;
// 					io_img_color[tidx+2*ww*hh] = rgb.z;
				//////////////////////////////////////////////////////////////////////////

				//////////////////////////////////////////////////////////////////////////
				// 임시 방편으로 G channel 값을 R channel에!!
				//d_grvu_Get_RGB_Value_Uninterpolated(p3d_v,cube_color,cube_w,rgb);
				//io_img_color[tidx] = rgb.y;
				//////////////////////////////////////////////////////////////////////////

			}
		} else valid = false;
	}

	if(!valid){
		io_map_depth[tidx] = 0.0f;
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// CPU version 에서 default value 를 -100.0f 로 한듯...	
		//const char type = '0';
		//io_map_normals[3*tidx] = io_map_normals[3*tidx + 1] = io_map_normals[3*tidx + 2] = nanf(&type);
		io_map_normals[3*tidx] = io_map_normals[3*tidx + 1] = io_map_normals[3*tidx + 2] = -100.0f;
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		io_img_normals[tidx] = uchar(0);
		if(io_img_color){
			// black background
			io_img_color[tidx] = uchar(0);
			io_img_color[tidx+ww*hh] = uchar(0);
			io_img_color[tidx+2*ww*hh] = uchar(0);

			// white background
			io_img_color[tidx] = uchar(255);
			io_img_color[tidx+ww*hh] = uchar(255);
			io_img_color[tidx+2*ww*hh] = uchar(255);


		}
	}


}

__global__ void g_rmis_Render_Maps_for_Scene(
//const float *T_gc_dev, const float *T_cg_dev,
	Vector3f cam_cen,Vector3f light,
	// fixed parameters.
	const float *cube_tsdf,
	const uchar *cube_color,
	const uchar *cube_w,
	// output.
	float *io_map_depth,float *io_map_vertex,float *io_map_normals,uchar *io_img_normals,
	uchar *io_img_color)
{
	Vector2f p2d; Vector3f p3d_g,p3d_c,p3d_v,sn; Vector3u rgb;

	bool valid = false;
	int tidx;
	float surf_angle;

	int tx = threadIdx.x + blockIdx.x*blockDim.x;
	int ty = threadIdx.y + blockIdx.y*blockDim.y;

	int ww = dim_map_dev[0];
	int hh = dim_map_dev[1];

	if(tx < 0 || tx >= ww || ty < 0 || ty >= hh)	return;

	//	if(tx % 3 || ty % 3)	return ;

	tidx = ty*ww + tx;

	// do ray casting.
	p2d.x = tx;	p2d.y = ty;
	valid = d_crtcs_Cast_Ray_on_TSDF_Cube_for_Scene(
		p2d,
		T_gc_dev_const,T_cg_dev_const,
		//T_gc_dev, T_cg_dev,
		cam_cen,light,

		cube_tsdf,
		cube_w,
// 		K_dev,
// 		origin_dev,
// 		dim_cube_dev,
//		sz_vox_inv_dev[0],
		ww,hh,
		r_cube_dev[0],mu_dev[0],sz_vox_inv_dev[0],dim_sc_dev[0],

		p3d_g);

	// update maps.
	if(valid){
		if(d_csnt_Compute_Surface_Normal_from_TSDF(
			p3d_g,
			cube_tsdf,cube_w,
// 			origin_dev,
// 			dim_cube_dev,
//			sz_vox_inv_dev[0],
			sn)){

			surf_angle = sn.x*light.x + sn.y*light.y + sn.z*light.z;

			// set depth.
			d_t_Transform(p3d_g,T_gc_dev_const,p3d_c);
			io_map_depth[tidx] = p3d_c.z;
			// set vertex.
			io_map_vertex[3*tidx] = p3d_c.x;
			io_map_vertex[3*tidx + 1] = p3d_c.y;
			io_map_vertex[3*tidx + 2] = p3d_c.z;
			// set normal.
			io_map_normals[3*tidx] = sn.x;
			io_map_normals[3*tidx + 1] = sn.y;
			io_map_normals[3*tidx + 2] = sn.z;
			// set normal image.
			io_img_normals[tidx] = (uchar)(fmaxf(0.0f,fminf(255.0f,(0.8f * surf_angle + 0.2f) * 255.0f)));
			// set RGB color image.
			if(io_img_color){
				d_gpv_Get_Position_in_Voxel(p3d_g,origin_dev,dim_cube_dev,sz_vox_inv_dev[0],p3d_v);
				//d_grvi_Get_RGB_Value_Interpolated(p3d_v,cube_color,cube_w,rgb);
				d_grvu_Get_RGB_Value_Uninterpolated(p3d_v,cube_color,cube_w,rgb);

				// 임시 방편으로 G channel 값을 R channel에!!
				io_img_color[tidx] = rgb.y;
				//io_img_color[tidx] = rgb.x;
				io_img_color[tidx+ww*hh] = rgb.y;
				io_img_color[tidx+2*ww*hh] = rgb.z;

			}
		} else valid = false;
	}

	if(!valid){
		io_map_depth[tidx] = 0.0f;
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// CPU version 에서 default value 를 -100.0f 로 한듯...	
// 		const char type = '0';
// 		io_map_normals[3*tidx] = io_map_normals[3*tidx + 1] = io_map_normals[3*tidx + 2] = nanf(&type);
		io_map_vertex[3*tidx] = io_map_vertex[3*tidx + 1] = io_map_vertex[3*tidx + 2] = -100.0f;
		io_map_normals[3*tidx] = io_map_normals[3*tidx + 1] = io_map_normals[3*tidx + 2] = -100.0f;
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		io_img_normals[tidx] = uchar(0);
		if(io_img_color){
			io_img_color[tidx] = uchar(0);
			io_img_color[tidx+ww*hh] = uchar(0);
			io_img_color[tidx+2*ww*hh] = uchar(0);

		}
	}


}

__global__ void g_rmis_Render_Maps_for_Scene(
//const float *T_gc_dev, const float *T_cg_dev,
	Vector3f cam_cen,Vector3f light,
	// fixed parameters.
	const float *cube_tsdf,
	const uchar *cube_color,
	const uchar *cube_w,
	// output.
	float *io_map_depth,float *io_map_normals,uchar *io_img_normals,
	uchar *io_img_color)
{
		Vector2f p2d; Vector3f p3d_g,p3d_c,p3d_v,sn; Vector3u rgb;

		bool valid = false;
		int tidx;
		float surf_angle;

		int tx = threadIdx.x + blockIdx.x*blockDim.x;
		int ty = threadIdx.y + blockIdx.y*blockDim.y;

		int ww = dim_map_dev[0];
		int hh = dim_map_dev[1];

		if(tx < 0 || tx >= ww || ty < 0 || ty >= hh)	return;

		//	if(tx % 3 || ty % 3)	return ;

		tidx = ty*ww + tx;

		// do ray casting.
		p2d.x = tx;	p2d.y = ty;
		valid = d_crtcs_Cast_Ray_on_TSDF_Cube_for_Scene(
			p2d,
			T_gc_dev_const,T_cg_dev_const,
			//T_gc_dev, T_cg_dev,
			cam_cen,light,

			cube_tsdf,
			cube_w,
	// 		K_dev,
	// 		origin_dev,
	// 		dim_cube_dev,
	//		sz_vox_inv_dev[0],
			ww,hh,
			r_cube_dev[0],mu_dev[0],sz_vox_inv_dev[0],dim_sc_dev[0],

			p3d_g);

		// update maps.
		if(valid){
			if(d_csnt_Compute_Surface_Normal_from_TSDF(
				p3d_g,
				cube_tsdf,cube_w,
	// 			origin_dev,
	// 			dim_cube_dev,
	//			sz_vox_inv_dev[0],
				sn)){

				surf_angle = sn.x*light.x + sn.y*light.y + sn.z*light.z;

				// set depth.
				d_t_Transform(p3d_g,T_gc_dev_const,p3d_c);
				io_map_depth[tidx] = p3d_c.z;
				// set normal.
				io_map_normals[3*tidx] = sn.x;
				io_map_normals[3*tidx + 1] = sn.y;
				io_map_normals[3*tidx + 2] = sn.z;
				// set normal image.
				io_img_normals[tidx] = (uchar)(fmaxf(0.0f,fminf(255.0f,(0.8f * surf_angle + 0.2f) * 255.0f)));
				// set RGB color image.
				if(io_img_color){
					d_gpv_Get_Position_in_Voxel(p3d_g,origin_dev,dim_cube_dev,sz_vox_inv_dev[0],p3d_v);
					d_grvi_Get_RGB_Value_Interpolated(p3d_v,cube_color,cube_w,rgb);

					io_img_color[tidx] = rgb.x;
					io_img_color[tidx+ww*hh] = rgb.y;
					io_img_color[tidx+2*ww*hh] = rgb.z;

				}
			} else valid = false;
		}

		if(!valid){
			io_map_depth[tidx] = 0.0f;
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			// CPU version 에서 default value 를 -100.0f 로 한듯...	
			// 		const char type = '0';
			// 		io_map_normals[3*tidx] = io_map_normals[3*tidx + 1] = io_map_normals[3*tidx + 2] = nanf(&type);
			io_map_normals[3*tidx] = io_map_normals[3*tidx + 1] = io_map_normals[3*tidx + 2] = -100.0f;
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			io_img_normals[tidx] = uchar(0);
			if(io_img_color){
				io_img_color[tidx] = uchar(0);
				io_img_color[tidx+ww*hh] = uchar(0);
				io_img_color[tidx+2*ww*hh] = uchar(0);

			}
		}


	}


// ==================================================================================================
// HOST / DEVICE BARRIER | HOST / DEVICE BARRIER | HOST / DEVICE BARRIER | HOST / DEVICE BARRIER | 
// ==================================================================================================


/////////////////////////////////////////////////////////////////////////////////////////////
// LGKvRendererTSDF 
/////////////////////////////////////////////////////////////////////////////////////////////

// *******************************************************
__host__ LGKvRendererTSDF::LGKvRendererTSDF()
// *******************************************************
{

}

// *******************************************************
__host__ LGKvRendererTSDF::~LGKvRendererTSDF()
// *******************************************************
{

}

// *******************************************************
// CAUTION: 이 클래스를 동시에 여러개 쓰려면 내부에서 쓰는 constant memory 들을 독립시켜야 할듯 하다.
// 현재 이것들이 공유가 되어서 서로 다른 rendering class 를 동시에 사용하면 오류가 발생하는듯.
__host__ void LGKvRendererTSDF::ip_Initialize_Parameters(
	GKvObjectCubeFloat *in_cube,
	int ww, int hh,
	float fx, float fy,
	float px, float py,
	float mu,
	float max_w)
// *******************************************************
{
	float intrins_host[3*4], origin_host[3], light_host[3];
	float sz_vox_inv_host[1], mu_host[1], r_host[1], max_w_host[1];
	int dim_cube_host[3], dim_sc_host[1], dim_map_host[3*2];

	Vector3f cube_org = in_cube->origin();
	Vector3f cube_cen = in_cube->center();
	
	origin_host[0] = cube_org.x;	origin_host[1] = cube_org.y;	origin_host[2] = cube_org.z;

//	printf("cube_org: %f %f %f\n", cube_org.x, cube_org.y, cube_org.z);
// 	printf("cube_cen: %f %f %f\n", cube_cen.x, cube_cen.y, cube_cen.z);
// 	printf("%f %f\n", mu, max_w);

	mu_host[0] = mu;
	sz_vox_inv_host[0] = 1.0f/in_cube->sz_vox();
	r_host[0] = length(cube_org - cube_cen);
	max_w_host[0] = max_w;
	
	in_cube->ts(dim_cube_host[0], dim_cube_host[1], dim_cube_host[2]);
	dim_sc_host[0] = in_cube->dim_sc();

	//printf("dim_cube_host: %d %d %d\n", dim_cube_host[0], dim_cube_host[1], dim_cube_host[2]);
	int tww, thh;	tww = ww; thh = hh; 
	float *p_intrins = &intrins_host[0];
	int *p_dim_maps = &dim_map_host[0];
	for(int k=0; k<3; k++){
		
		p_intrins[0] = fx;	p_intrins[1] = fy;	p_intrins[2] = px;	p_intrins[3] = py;
		p_dim_maps[0] = tww;	p_dim_maps[1] = thh;

		// downsizing.
		fx = 0.5f*fx; fy = 0.5f*fy; px = 0.5f*(px - 0.5f); py = 0.5f*(py - 0.5f);
		tww /= 2; thh /= 2;

		p_intrins += 4;
		p_dim_maps += 2;
		
	}
// 	for(int i=0; i<12; i++) printf("%f\n", intrins_host[i]);
// 	for(int i=0; i<6; i++) printf("%d\n", dim_map_host[i]);
	cudaMemcpyToSymbol(K_dev,intrins_host,3*4 * sizeof(float));
	cudaMemcpyToSymbol(dim_map_dev,dim_map_host,3*2 * sizeof(int));
	//  	
	cudaMemcpyToSymbol(origin_dev, origin_host, 3 * sizeof(float));
	//  	
	cudaMemcpyToSymbol(mu_dev, mu_host, sizeof(float));
	cudaMemcpyToSymbol(max_w_dev, max_w_host, sizeof(float));
	cudaMemcpyToSymbol(sz_vox_inv_dev, sz_vox_inv_host, sizeof(float));
	cudaMemcpyToSymbol(r_cube_dev, r_host, sizeof(float));

	// 
	cudaMemcpyToSymbol(dim_cube_dev, dim_cube_host, 3 * sizeof(int));
	cudaMemcpyToSymbol(dim_sc_dev, dim_sc_host, sizeof(int));
}

// *******************************************************
__host__ void LGKvRendererTSDF::rmi_Render_Maps_for_ICP(
	GKvObjectCubeFloat *in_cube,
	int in_ww, int in_hh,
	const float *in_T_gc_dev, const float *in_T_cg_dev,	

	Vector3f in_cam_cen, Vector3f in_light,

	int in_lev_of_pyram,

	float *io_map_depth_dev,
	float *io_map_vertex_dev,
	float *io_map_normals_dev,
	uchar *io_img_normals_dev,
	
	uchar *io_img_color_dev)
// *******************************************************
{
	float *map_depth_dev = io_map_depth_dev;
	float *map_vertex_dev = io_map_vertex_dev;
	float *map_normal_dev = io_map_normals_dev;
	uchar *img_normal_dev = io_img_normals_dev;
	
	int block_sz, grid_sz;

	// Rendering parameters.
	Vector3f cube_org = in_cube->origin();
	Vector3f cube_cen = in_cube->center();
	Vector3f cam_cen = in_cam_cen;
	Vector3f vec_cc = cube_cen - cam_cen;
	Vector3f norm_cc = vec_cc.normalised();

	float r_cube = length(cube_org - cube_cen); //sqrtf(rvec.x*rvec.x + rvec.y*rvec.y + rvec.z*rvec.z);
	float dist_cc = length(vec_cc);
	float theta_max = asin(r_cube/dist_cc);
	
	// For depth map rendering.
	dim3 threads(CV_CUDA_BLOCK_SIZE_X, CV_CUDA_BLOCK_SIZE_Y);
	dim3 blocks(iDivUp(in_ww, threads.x), iDivUp(in_hh, threads.y));

	//printf("block: %d %d / thread: %d %d\n", blocks.x, blocks.y, threads.x, threads.y);

	// Camera pose.
	cudaMemcpyToSymbol(T_gc_dev_const, in_T_gc_dev, sizeof(float) * 16, 0, cudaMemcpyDeviceToDevice);
	cudaMemcpyToSymbol(T_cg_dev_const, in_T_cg_dev, sizeof(float) * 16, 0, cudaMemcpyDeviceToDevice);

  	g_rmi_Render_Maps_for_ICP<<<blocks, threads>>>(
		//in_T_gc_dev, in_T_cg_dev,		
		in_cam_cen, in_light, norm_cc, dist_cc, theta_max, 		
		//test_tsdf_dev, 
		in_cube->vp_tsdf(),
		in_cube->vp_rgb(),
		in_cube->vp_w(),

		in_lev_of_pyram,

 		map_depth_dev, map_vertex_dev, map_normal_dev, img_normal_dev,
		io_img_color_dev);

}

// *******************************************************
__host__ void LGKvRendererTSDF::rmi_Render_Maps_for_ICP(
	GKvObjectCubeFloat *in_cube,
	int in_ww,int in_hh,
	const float *in_T_gc_dev,const float *in_T_cg_dev,

	Vector3f in_cam_cen,Vector3f in_light,

	int in_lev_of_pyram,

	float *io_map_depth_dev,
	float *io_map_normals_dev,
	uchar *io_img_normals_dev,

	uchar *io_img_color_dev)
// *******************************************************
{
	float *map_depth_dev = io_map_depth_dev;
	float *map_normal_dev = io_map_normals_dev;
	uchar *img_normal_dev = io_img_normals_dev;

	int block_sz,grid_sz;

	// Rendering parameters.
	Vector3f cube_org = in_cube->origin();
	Vector3f cube_cen = in_cube->center();
	Vector3f cam_cen = in_cam_cen;
	Vector3f vec_cc = cube_cen - cam_cen;
	Vector3f norm_cc = vec_cc.normalised();

	float r_cube = length(cube_org - cube_cen); //sqrtf(rvec.x*rvec.x + rvec.y*rvec.y + rvec.z*rvec.z);
	float dist_cc = length(vec_cc);
	float theta_max = asin(r_cube/dist_cc);

	// For depth map rendering.
	dim3 threads(CV_CUDA_BLOCK_SIZE_X,CV_CUDA_BLOCK_SIZE_Y);
	dim3 blocks(iDivUp(in_ww,threads.x),iDivUp(in_hh,threads.y));

	// Camera pose.
	cudaMemcpyToSymbol(T_gc_dev_const,in_T_gc_dev,sizeof(float) * 16,0,cudaMemcpyDeviceToDevice);
	cudaMemcpyToSymbol(T_cg_dev_const,in_T_cg_dev,sizeof(float) * 16,0,cudaMemcpyDeviceToDevice);

	g_rmi_Render_Maps_for_ICP<<<blocks,threads>>>(
		//in_T_gc_dev, in_T_cg_dev,		
		in_cam_cen,in_light,norm_cc,dist_cc,theta_max,
		//test_tsdf_dev, 

		in_cube->vp_tsdf(),
		in_cube->vp_rgb(),
		in_cube->vp_w(),

		in_lev_of_pyram,

		map_depth_dev,map_normal_dev,img_normal_dev,
		io_img_color_dev);

}

// *******************************************************
__host__ void LGKvRendererTSDF::rmis_Render_Maps_for_Scene(
	GKvObjectCubeFloat *in_cube,
	int in_ww, int in_hh,
	const float *in_T_gc_dev, const float *in_T_cg_dev,	

	Vector3f in_cam_cen, Vector3f in_light,

	float *io_map_depth_dev,
	float *io_map_vertex_dev,
	float *io_map_normals_dev,
	uchar *io_img_normals_dev,
	
	uchar *io_img_color_dev)
// *******************************************************
{
	float *map_depth_dev = io_map_depth_dev;
	float *map_vertex_dev = io_map_vertex_dev;
	float *map_normal_dev = io_map_normals_dev;
	uchar *img_normal_dev = io_img_normals_dev;
	
	int block_sz, grid_sz;

	// Rendering parameters.
	Vector3f cube_org = in_cube->origin();
	Vector3f cube_cen = in_cube->center();
	Vector3f cam_cen = in_cam_cen;

	float r_cube = length(cube_org - cube_cen); //sqrtf(rvec.x*rvec.x + rvec.y*rvec.y + rvec.z*rvec.z);

// 	printf("r_cube: %f\n", r_cube);
// 	printf("cube_cen: %f %f %f\n", cube_cen.x ,cube_cen.y, cube_cen.z);
// 	printf("in_light: %f %f %f\n", in_light.x, in_light.y, in_light.z);
// 	printf("in_cam_cen: %f %f %f\n", in_cam_cen.x, in_cam_cen.y, in_cam_cen.z);
	
	// For depth map rendering.
	dim3 threads(CV_CUDA_BLOCK_SIZE_X, CV_CUDA_BLOCK_SIZE_Y);
	dim3 blocks(iDivUp(in_ww, threads.x), iDivUp(in_hh, threads.y));

	// Camera pose.
	cudaMemcpyToSymbol(T_gc_dev_const, in_T_gc_dev, sizeof(float) * 16, 0, cudaMemcpyDeviceToDevice);
	cudaMemcpyToSymbol(T_cg_dev_const, in_T_cg_dev, sizeof(float) * 16, 0, cudaMemcpyDeviceToDevice);

  	g_rmis_Render_Maps_for_Scene<<<blocks, threads>>>(
		//in_T_gc_dev, in_T_cg_dev,		
		in_cam_cen, in_light,	
		//test_tsdf_dev, 
		in_cube->vp_tsdf(),
		in_cube->vp_rgb(),
		in_cube->vp_w(),

 		map_depth_dev, map_vertex_dev, map_normal_dev, img_normal_dev,
		io_img_color_dev);

}

// *******************************************************
__host__ void LGKvRendererTSDF::rmis_Render_Maps_for_Scene(
	GKvObjectCubeFloat *in_cube,
	int in_ww,int in_hh,
	const float *in_T_gc_dev,const float *in_T_cg_dev,

	Vector3f in_cam_cen,Vector3f in_light,

	float *io_map_depth_dev,
	float *io_map_normals_dev,
	uchar *io_img_normals_dev,

	uchar *io_img_color_dev)
// *******************************************************
{
	float *map_depth_dev = io_map_depth_dev;
	float *map_normal_dev = io_map_normals_dev;
	uchar *img_normal_dev = io_img_normals_dev;

	int block_sz,grid_sz;

	// Rendering parameters.
	Vector3f cube_org = in_cube->origin();
	Vector3f cube_cen = in_cube->center();
	Vector3f cam_cen = in_cam_cen;

	float r_cube = length(cube_org - cube_cen); //sqrtf(rvec.x*rvec.x + rvec.y*rvec.y + rvec.z*rvec.z);

	// 	printf("r_cube: %f\n", r_cube);
	// 	printf("cube_cen: %f %f %f\n", cube_cen.x ,cube_cen.y, cube_cen.z);
	// 	printf("in_light: %f %f %f\n", in_light.x, in_light.y, in_light.z);
	// 	printf("in_cam_cen: %f %f %f\n", in_cam_cen.x, in_cam_cen.y, in_cam_cen.z);

	// For depth map rendering.
	dim3 threads(CV_CUDA_BLOCK_SIZE_X,CV_CUDA_BLOCK_SIZE_Y);
	dim3 blocks(iDivUp(in_ww,threads.x),iDivUp(in_hh,threads.y));

	// Camera pose.
	cudaMemcpyToSymbol(T_gc_dev_const,in_T_gc_dev,sizeof(float) * 16,0,cudaMemcpyDeviceToDevice);
	cudaMemcpyToSymbol(T_cg_dev_const,in_T_cg_dev,sizeof(float) * 16,0,cudaMemcpyDeviceToDevice);

	g_rmis_Render_Maps_for_Scene<<<blocks,threads>>>(
		//in_T_gc_dev, in_T_cg_dev,		
		in_cam_cen,in_light,
		//test_tsdf_dev, 
		in_cube->vp_tsdf(),
		in_cube->vp_rgb(),
		in_cube->vp_w(),

		map_depth_dev,map_normal_dev,img_normal_dev,
		io_img_color_dev);

}
