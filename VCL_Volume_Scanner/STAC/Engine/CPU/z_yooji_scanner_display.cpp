/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_scanner_display.cpp 
/////////////////////////////////////////////////////////////////////////////////////////////

#include "../../../stdafx.h"
#include "../../_yooji_2016_object_scanning.h"

/////////////////////////////////////////////////////////////////////////////////////////////
// LCKvYooji_Scanner_Display 
/////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
LCKvYooji_Scanner_Display::LCKvYooji_Scanner_Display()
//********************************************************************************************
{
	// set default value of projected camera center.
	zz_classname = "LCKvYooji_Scanner_Display";
	zz_mat_4x4.ci_Create_Identity_matrix(4);

	zz_cen_prev.s_Set(-1, -1);
}

//********************************************************************************************
LCKvYooji_Scanner_Display::~LCKvYooji_Scanner_Display()
//********************************************************************************************
{
}

//********************************************************************************************
void LCKvYooji_Scanner_Display::rmi_Render_Maps_for_ICP(
	CKvYooji_Cube_TSDF_Float *in_cube,
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Scanner_Parameter *in_params,
	CKvYooji_Tracking_State *io_state)
//********************************************************************************************
{	
	CKvPoint3Df sp_g, sp_c, sp_vox;		// surface points in global, camera and voxel coordinates.
	CKvPoint3Df sn, sn_off, light;		// surface normal.
	CKvPoint3Df c_cam, c_cube;	// centers of camera and cube.
	CKvPoint3Df cd;				// the direction from camera center to cube center.
	CKvPixel t_pix;

	float *p_depth, *p_weight;
	double *p_mat;
	CKvPoint3Df *p_normal;
	UCHAR *p_img_normal;

	int ww, hh;
	bool flag_found;

	// for parameters.
	CKvPoint3Df origin; int ww_c, hh_c, dd_c, sz_sc; float sz_vox;
	float th_dist_ICP, mu, max_w;

	int i, j;
	int num_level;
	float vox_sz_inv, r_cube;
	float dist_cc, theta_max;
	float surf_angle, w_depth;	// for surface rendering.
	
	// load scanner parameters.
	in_params->gpic_Get_Parameters_for_Initializing_Cube(ww_c, hh_c, dd_c, sz_sc, sz_vox, origin);
	in_params->gpicp_Get_Parameters_for_ICP_algorithm(num_level, th_dist_ICP, mu, max_w);
	// load camera parameters.
	zz_intrin_d.copy(io_state->p_intrinsics_depth());
	zz_extrin.copy(io_state->p_extrinsics_glob_to_cam());
	// get pointers.
	p_depth=io_state->p_map_depth_rendered()->mps(ww, hh)[0];	
	p_weight=io_state->p_map_weight_rendered()->vp();
	p_normal=io_state->p_map_normals_rendered()->vp();
	p_img_normal=io_state->p_image_normals_rendered()->vp();

	// set light source vector as inverse of principal axis of current camera.	
	p_mat = io_state->p_P_matrix_depth()->mpp()->vp();	
	light.x=-p_mat[8];	light.y=-p_mat[9];	light.z=-p_mat[10];		light.n_Normalize();
	// compute inverse of voxel size.
	vox_sz_inv=1.0f/sz_vox;
	// calculate cube center.
	in_cube->gcciw_Get_Cube_Center_In_World(c_cube);	
	//c_cube.x=origin.x + sz_vox*(float)ww_c*0.5f;	c_cube.y=origin.y + sz_vox*(float)hh_c*0.5f;	c_cube.z=origin.z + sz_vox*(float)dd_c*0.5f;
	// get camera center.
	zz_extrin.get_cam_center(c_cam);
	// compute direction vector between cube center and camera center.
	cd.x=c_cube.x-c_cam.x;	cd.y=c_cube.y-c_cam.y;	cd.z=c_cube.z-c_cam.z;		
	dist_cc=cd.n_Normalize();
	// 
	r_cube=origin.d_Distance(c_cube);		// radius of sphere wrapping the object cube.
	theta_max=asin(r_cube/dist_cc);			// maximum 'theta' so that the pixel ray goes through the object cube. 
		
	///
	int cnt=0, tidx;
	sn_off.x=sn_off.y=sn_off.z=-100.0f;	// no normal.
	for(j=0; j<hh; j++)
	{ 
		for(i=0; i<ww; i++)
		{
			// initializes values.
			tidx=j*ww+i;
			p_depth[tidx]=p_weight[tidx]=0.0f;	// set depth.
			p_normal[tidx].s_Set(sn_off.x, sn_off.y, sn_off.z);	// set normal.
			p_img_normal[tidx]=(UCHAR)0;		// set normal image.

			// cast ray.
			t_pix.x=i;	t_pix.y=j;
			flag_found=crtc_Cast_Ray_on_TSDF_Cube(t_pix,
				in_cube,
				&zz_intrin_d,
				&zz_extrin,

				cd,
				c_cam,
				r_cube,
				dist_cc,
				theta_max,

				mu,
				vox_sz_inv,
				sp_g);

			//printf("sp_g: %f\n", sp_c.z);
		
			if(flag_found){
				if(in_cube->csnt_Compute_Surface_Normal_from_TSDF(sp_g, sn)){			

					surf_angle=sn.x*light.x+sn.y*light.y+sn.z*light.z;	//surf_angle*=-1.0f;
					if(surf_angle>0.0f){					

						// set depth.
						zz_extrin.transform(sp_g, sp_c);	// get depth in camera coordinates.								
						p_depth[tidx]=sp_c.z;
						// set normal.
						p_normal[tidx].s_Set(sn.x, sn.y, sn.z);
						// set depth weight.
						in_cube->gpv_Get_Position_in_Voxel(sp_g, sp_vox);
						in_cube->gwdi_Get_Weight_Depth_Interpolated(sp_vox, w_depth);//in_cube->gwdi_Get_Wegith_Depth_Interpolated(sp_g, w_depth);
						p_weight[tidx]=w_depth;	
						// set normal image.
						p_img_normal[tidx]=(UCHAR)(clip(0.0f, 255.0f, (0.8f * surf_angle + 0.2f) * 255.0f));	
					}				
				}
			}

		}
	}

}

//********************************************************************************************
void LCKvYooji_Scanner_Display::rmi_Render_Maps_for_ICP(
	CKvYooji_Cube_TSDF_Float *in_cube,	
	CKvYooji_Scanner_Parameter *in_params,
	CKvYooji_Tracking_State *io_state)
//********************************************************************************************
{	
	CKvPoint3Df sp_g, sp_c, sp_vox;		// surface points in global, camera and voxel coordinates.
	CKvPoint3Df sn, sn_off, light;		// surface normal.
	CKvPoint3Df c_cam, c_cube;	// centers of camera and cube.
	CKvPoint3Df cd;				// the direction from camera center to cube center.
	CKvPixel t_pix;

	float *p_depth, *p_weight;	double *p_mat;
	CKvPoint3Df *p_normal;
	UCHAR *p_img_normal;

	int ww, hh;
	bool flag_found;
//printf(" aaaa\n");
	// for parameters.
	CKvPoint3Df origin; int ww_c, hh_c, dd_c, sz_sc; float sz_vox;
	float th_dist_ICP, mu, max_w;

	int i, j;
	int num_level;
	float vox_sz_inv, r_cube;
	float dist_cc, theta_max;
	float surf_angle, w_depth;	// for surface rendering.
	
	// load scanner parameters.
	in_params->gpic_Get_Parameters_for_Initializing_Cube(ww_c, hh_c, dd_c, sz_sc, sz_vox, origin);
	in_params->gpicp_Get_Parameters_for_ICP_algorithm(num_level, th_dist_ICP, mu, max_w);

//	if(!Kv_Printf("%d %d %d %d %f / %f %f %f", ww_c, hh_c, dd_c, sz_sc, sz_vox, th_dist_ICP, mu, max_w))	exit(0);

	// load camera parameters.
// 	zz_intrin_d.spm_Set_from_P_Matrix(in_view->ppd_Pointer_of_Pmatrix_Depth());
// 	io_state->ph3d_Pointer_of_3D_Homography_Global_to_Camera()->e_Export(&zz_mat_4x4);
// 	zz_extrin.st_Set_from_Transformation(&zz_mat_4x4);
	zz_intrin_d.copy(io_state->p_intrinsics_depth());
	zz_extrin.copy(io_state->p_extrinsics_glob_to_cam());
	// get pointers.
	p_depth=io_state->p_map_depth_rendered()->mps(ww, hh)[0];	
	p_weight=io_state->p_map_weight_rendered()->vp();
	p_normal=io_state->p_map_normals_rendered()->vp();
	p_img_normal=io_state->p_image_normals_rendered()->vp();
//printf(" bbbb\n");
	// set light source vector as inverse of principal axis of current camera.	
	p_mat = io_state->p_P_matrix_depth()->mpp()->vp();
	light.x=-p_mat[8];	light.y=-p_mat[9];	light.z=-p_mat[10];		light.n_Normalize();
	// compute inverse of voxel size.
	vox_sz_inv=1.0f/sz_vox;
	// calculate cube center.
	c_cube.x=origin.x + sz_vox*(float)ww_c*0.5f;	c_cube.y=origin.y + sz_vox*(float)hh_c*0.5f;	c_cube.z=origin.z + sz_vox*(float)dd_c*0.5f;
	// get camera center.
	zz_extrin.get_cam_center(c_cam);
	// compute direction vector between cube center and camera center.
	cd.x=c_cube.x-c_cam.x;	cd.y=c_cube.y-c_cam.y;	cd.z=c_cube.z-c_cam.z;		
	dist_cc=cd.n_Normalize();
	// 
	r_cube=origin.d_Distance(c_cube);		// radius of sphere wrapping the object cube.
	theta_max=asin(r_cube/dist_cc);			// maximum 'theta' so that the pixel ray goes through the object cube. 
//printf(" cccc\n");	
	///
	int cnt=0, tidx;
	sn_off.x=sn_off.y=sn_off.z=-100.0f;	// no normal.
	for(j=0; j<hh; j++)
	{ 
		for(i=0; i<ww; i++)
		{
			// initializes values.
			tidx=j*ww+i;
			p_depth[tidx]=p_weight[tidx]=0.0f;	// set depth.
			p_normal[tidx].s_Set(sn_off.x, sn_off.y, sn_off.z);	// set normal.
			p_img_normal[tidx]=(UCHAR)0;		// set normal image.

			// cast ray.
			t_pix.x=i;	t_pix.y=j;
			flag_found=crtc_Cast_Ray_on_TSDF_Cube(t_pix,
				in_cube,
				&zz_intrin_d,
				&zz_extrin,

				cd,
				c_cam,
				r_cube,
				dist_cc,
				theta_max,

				mu,
				vox_sz_inv,
				sp_g);
		
			if(flag_found){
				if(in_cube->csnt_Compute_Surface_Normal_from_TSDF(sp_g, sn)){

					surf_angle=sn.x*light.x+sn.y*light.y+sn.z*light.z;	//surf_angle*=-1.0f;
					if(surf_angle>0.0f){
						// set depth.
						zz_extrin.transform(sp_g, sp_c);	// get depth in camera coordinates.								
						p_depth[tidx]=sp_c.z;
						// set normal.
						p_normal[tidx].s_Set(sn.x, sn.y, sn.z);
						// set depth weight.
						in_cube->gpv_Get_Position_in_Voxel(sp_g, sp_vox);
						in_cube->gwdi_Get_Weight_Depth_Interpolated(sp_vox, w_depth);//in_cube->gwdi_Get_Wegith_Depth_Interpolated(sp_g, w_depth);
						p_weight[tidx]=w_depth;	
						// set normal image.
						p_img_normal[tidx]=(UCHAR)(clip(0.0f, 255.0f, (0.8f * surf_angle + 0.2f) * 255.0f));	
					}				
				}
			}

		}
	}
	
	//printf("time: %f %f %f\n", time[0], time[1], time[2]);	system("pause");
}

//********************************************************************************************
void LCKvYooji_Scanner_Display::rinrc_Render_Image_Normal_on_RGB_Camera(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Tracking_State *in_state,
	CKvMatrixUcharRgb *out_rendered_normal)
//********************************************************************************************
{
	CKvPointf t_pixf_d, t_pixf_rgb;

	int ww_rgb, hh_rgb, ww_d, hh_d, x, y, tidx;
	float *p_depth, td;
	UCHAR *p_color, *p_out_color, *p_normal;

	p_color=in_view->p_image_rgb()->mps(ww_rgb, hh_rgb)[0];

	//if(out_rendered_normal->mw()!=ww_rgb || out_rendered_normal->mh()!=hh_rgb)	out_rendered_normal->c_Create(hh_rgb, ww_rgb);
	p_out_color=out_rendered_normal->cp_Copy(in_view->p_image_rgb())[0];
	p_normal=in_state->p_image_normals_rendered()->mps(ww_d, hh_d)[0];
	p_depth=in_state->p_map_depth_rendered()->vp();

	for(int k=0; k<ww_d*hh_d; k++){
		if(p_normal[k]>(UCHAR)0){

			t_pixf_d.x=(float)(k%ww_d); t_pixf_d.y=(float)(k/ww_d);
			td=p_depth[k];

			if(!in_view->convert_pixel_coord(t_pixf_d, td, t_pixf_rgb))	continue;
						
			x=(int)t_pixf_rgb.x;	y=(int)t_pixf_rgb.y;

			tidx=y*ww_rgb+x;
			p_out_color[tidx]=p_normal[k];
			p_out_color[tidx+ww_rgb*hh_rgb]=p_normal[k];
			p_out_color[tidx+2*ww_rgb*hh_rgb]=p_normal[k];
		}
	}

}

//********************************************************************************************
void LCKvYooji_Scanner_Display::rmd_Render_Map_Depth(
	CKvYooji_Cube_TSDF_Float *in_cube,		
	CKvYooji_Scanner_Parameter *in_params,
	CKvYooji_Intrinsics *in_intrinsics,
	CKvYooji_Extrinsics *in_extrinsics,
	CKvMatrixFloat *out_map_depth)
//********************************************************************************************
{	
	CKvPoint3Df sp_g, sp_c, sp_vox;		// surface points in global, camera and voxel coordinates.
	CKvPoint3Df sn, sn_off, light;		// surface normal.
	CKvPoint3Df c_cam, c_cube;	// centers of camera and cube.
	CKvPoint3Df cd;				// the direction from camera center to cube center.
	CKvPixel t_pix;

	float *p_mat, *p_depth;

	int ww, hh;
	bool flag_found;

	// for parameters.
	CKvPoint3Df origin; int ww_c, hh_c, dd_c, sz_sc; float sz_vox;
	float th_dist_ICP, mu, max_w;

	int i, j;
	int lev_pyram;
	float vox_sz_inv, r_cube;
	float dist_cc, theta_max;
	float surf_angle;	// for surface rendering.
	
	// load scanner parameters.
	in_params->gpic_Get_Parameters_for_Initializing_Cube(ww_c, hh_c, dd_c, sz_sc, sz_vox, origin);
	in_params->gpicp_Get_Parameters_for_ICP_algorithm(lev_pyram, th_dist_ICP, mu, max_w);
	// get pointers.
	p_depth=out_map_depth->mps(ww, hh)[0];

	// set light source vector as inverse of principal axis of current camera.		
	p_mat = in_extrinsics->mp_transform()->vp();	
	light.x=-p_mat[8];	light.y=-p_mat[9];	light.z=-p_mat[10];		light.n_Normalize();
	// compute inverse of voxel size.
	vox_sz_inv=1.0f/sz_vox;
	// calculate cube center.
	c_cube.x=origin.x + sz_vox*(float)ww_c*0.5f;	c_cube.y=origin.y + sz_vox*(float)hh_c*0.5f;	c_cube.z=origin.z + sz_vox*(float)dd_c*0.5f;
	// get camera center.
	in_extrinsics->get_cam_center(c_cam);
	// compute direction vector between cube center and camera center.
	cd.x=c_cube.x-c_cam.x;	cd.y=c_cube.y-c_cam.y;	cd.z=c_cube.z-c_cam.z;		
	dist_cc=cd.n_Normalize();
	// 
	r_cube=origin.d_Distance(c_cube);		// radius of sphere wrapping the object cube.
	theta_max=asin(r_cube/dist_cc);			// maximum 'theta' so that the pixel ray goes through the object cube. 
	
	///
	int cnt=0, tidx;
	sn_off.x=sn_off.y=sn_off.z=-100.0f;	// no normal.
	for(j=0; j<hh; j++)
	{ 
		for(i=0; i<ww; i++)
		{
			// initializes values.
			tidx=j*ww+i;
			p_depth[tidx]=0.0f;		// set normal image.

			// cast ray.
			t_pix.x=i;	t_pix.y=j;
			flag_found=crtc_Cast_Ray_on_TSDF_Cube(t_pix,
				in_cube,
				in_intrinsics,
				in_extrinsics,

				cd,
				c_cam,
				r_cube,
				dist_cc,
				theta_max,

				mu,
				vox_sz_inv,
				sp_g);
		
			if(flag_found){
				if(in_cube->csnt_Compute_Surface_Normal_from_TSDF(sp_g, sn)){

					surf_angle=sn.x*light.x+sn.y*light.y+sn.z*light.z;	//surf_angle*=-1.0f;
					if(surf_angle>0.0f){
						// set depth.
						in_extrinsics->transform(sp_g, sp_c);	// get depth in camera coordinates.								
						p_depth[tidx]=sp_c.z;
					}				
				}
			}

		}
	}	
	
}


//********************************************************************************************
void LCKvYooji_Scanner_Display::rin_Render_Image_Normal(
	CKvYooji_Cube_TSDF_Float *in_cube,		
	CKvYooji_Scanner_Parameter *in_params,
	CKvYooji_Intrinsics *in_intrinsics,
	CKvYooji_Extrinsics *in_extrinsics,
	CKvMatrixUchar *out_img_normal)
//********************************************************************************************
{	
	CKvPoint3Df sp_g, sp_c, sp_vox;		// surface points in global, camera and voxel coordinates.
	CKvPoint3Df sn, sn_off, light;		// surface normal.
	CKvPoint3Df c_cam, c_cube;	// centers of camera and cube.
	CKvPoint3Df cd;				// the direction from camera center to cube center.
	CKvPixel t_pix;

	float *p_mat;
	UCHAR *p_img_normal;

	int ww, hh;
	bool flag_found;

	// for parameters.
	CKvPoint3Df origin; int ww_c, hh_c, dd_c, sz_sc; float sz_vox;
	float th_dist_ICP, mu, max_w;

	int i, j;
	int lev_pyram;
	float vox_sz_inv, r_cube;
	float dist_cc, theta_max;
	float surf_angle;	// for surface rendering.
	
	// load scanner parameters.
	in_params->gpic_Get_Parameters_for_Initializing_Cube(ww_c, hh_c, dd_c, sz_sc, sz_vox, origin);
	in_params->gpicp_Get_Parameters_for_ICP_algorithm(lev_pyram, th_dist_ICP, mu, max_w);
	// get pointers.
	p_img_normal=out_img_normal->mps(ww, hh)[0];

	// set light source vector as inverse of principal axis of current camera.	
	p_mat = zz_mat_4x4.vp();
	light.x=-p_mat[8];	light.y=-p_mat[9];	light.z=-p_mat[10];		light.n_Normalize();
	// compute inverse of voxel size.
	vox_sz_inv=1.0f/sz_vox;
	// calculate cube center.
	c_cube.x=origin.x + sz_vox*(float)ww_c*0.5f;	c_cube.y=origin.y + sz_vox*(float)hh_c*0.5f;	c_cube.z=origin.z + sz_vox*(float)dd_c*0.5f;
	// get camera center.
	in_extrinsics->get_cam_center(c_cam);
	// compute direction vector between cube center and camera center.
	cd.x=c_cube.x-c_cam.x;	cd.y=c_cube.y-c_cam.y;	cd.z=c_cube.z-c_cam.z;		
	dist_cc=cd.n_Normalize();
	// 
	r_cube=origin.d_Distance(c_cube);		// radius of sphere wrapping the object cube.
	theta_max=asin(r_cube/dist_cc);			// maximum 'theta' so that the pixel ray goes through the object cube. 
	
	///
	int cnt=0, tidx;
	sn_off.x=sn_off.y=sn_off.z=-100.0f;	// no normal.
	for(j=0; j<hh; j++)
	{ 
		for(i=0; i<ww; i++)
		{
			// initializes values.
			tidx=j*ww+i;
			p_img_normal[tidx]=(UCHAR)0;		// set normal image.

			// cast ray.
			t_pix.x=i;	t_pix.y=j;
			flag_found=crtc_Cast_Ray_on_TSDF_Cube(t_pix,
				in_cube,
				in_intrinsics,
				in_extrinsics,

				cd,
				c_cam,
				r_cube,
				dist_cc,
				theta_max,

				mu,
				vox_sz_inv,
				sp_g);
		
			if(flag_found){
				if(in_cube->csnt_Compute_Surface_Normal_from_TSDF(sp_g, sn)){

					surf_angle=sn.x*light.x+sn.y*light.y+sn.z*light.z;	//surf_angle*=-1.0f;
					if(surf_angle>0.0f){
						// set normal image.
						p_img_normal[tidx]=(UCHAR)(clip(0.0f, 255.0f, (0.8f * surf_angle + 0.2f) * 255.0f));	
					}				
				}
			}

		}
	}
	
}

//********************************************************************************************
bool LCKvYooji_Scanner_Display::rct_Render_Camera_Trace(
	CKvYooji_Extrinsics *in_view_ref,
	CKvYooji_Extrinsics *in_view_trace,
	CKvYooji_Intrinsics *in_intrinsics_ref,
	CKvYooji_Intrinsics *in_intrinsics_trace,
	CKvMatrixUcharRgb *io_img_trace,
	float in_ratio_between_camera_size_and_focal_length)
//********************************************************************************************
{
	CKvPoint3Df cen_tr_g, cen_tr_ref, cor_tr_g, cor_tr_ref;
	CKvPointf cen_tr_proj, lt_tr_proj, rt_tr_proj, lb_tr_proj, rb_tr_proj;
	CKvPixel t_pix;
	CKvRgb color_cam, color_trace;
	UCHAR *p_img_tr;
	float f, fx, fy, px, py, d_min, d_max;
	int ww, hh, tx, ty, dx, dy;
	
	color_cam.s_Set((UCHAR)(rand()%128+128), (UCHAR)(rand()%128+128), (UCHAR)(rand()%128+128));		// random color.
	color_trace.s_Set((UCHAR)0, (UCHAR)255, (UCHAR)0);

	// get camera center position in the reference view coordinates.
	in_view_trace->get_cam_center(cen_tr_g);
	in_view_ref->transform(cen_tr_g, cen_tr_ref);
	if(!in_intrinsics_ref->project(cen_tr_ref, cen_tr_proj))	return false;

	p_img_tr=io_img_trace->mps(ww, hh)[0];
	tx=(int)cen_tr_proj.x;	ty=(int)cen_tr_proj.y;

	if(tx<0 || tx>ww-1 || ty<0 || ty>hh-1)	return false;	// check center visibility.

	// get 3D corner position of image plane in the reference view coordinates.
	// + for left-top corner.
	in_intrinsics_trace->get_params(fx, fy, px, py, d_min, d_max);
	f=(fx+fy)*0.5f*0.001f;		// mm->m
	f = in_ratio_between_camera_size_and_focal_length * f;
	t_pix.x=0;	t_pix.y=0;	in_intrinsics_trace->back_project(t_pix, f, cor_tr_ref);
	in_view_trace->transform_inv(cor_tr_ref, cor_tr_g);
	in_view_ref->transform(cor_tr_g, cor_tr_ref);
	if(!in_intrinsics_ref->project(cor_tr_ref, lt_tr_proj))	return false;
	tx=(int)lt_tr_proj.x;	ty=(int)lt_tr_proj.y;
	if(tx<0 || tx>ww-1 || ty<0 || ty>hh-1)	return false;	// check corner visibility.

	// + for right-top corner.	
	t_pix.x=ww-1;	t_pix.y=0;	in_intrinsics_trace->back_project(t_pix, f, cor_tr_ref);
	in_view_trace->transform_inv(cor_tr_ref, cor_tr_g);
	in_view_ref->transform(cor_tr_g, cor_tr_ref);
	if(!in_intrinsics_ref->project(cor_tr_ref, rt_tr_proj))	return false;
	tx=(int)rt_tr_proj.x;	ty=(int)rt_tr_proj.y;
	if(tx<0 || tx>ww-1 || ty<0 || ty>hh-1)	return false;	// check corner visibility.

	// + for left-bottom corner.	
	t_pix.x=0;	t_pix.y=hh-1;	in_intrinsics_trace->back_project(t_pix, f, cor_tr_ref);
	in_view_trace->transform_inv(cor_tr_ref, cor_tr_g);
	in_view_ref->transform(cor_tr_g, cor_tr_ref);
	if(!in_intrinsics_ref->project(cor_tr_ref, lb_tr_proj))	return false;
	tx=(int)lb_tr_proj.x;	ty=(int)lb_tr_proj.y;
	if(tx<0 || tx>ww-1 || ty<0 || ty>hh-1)	return false;	// check corner visibility.

	// + for right-bottom corner.	
	t_pix.x=ww-1;	t_pix.y=hh-1;	in_intrinsics_trace->back_project(t_pix, f, cor_tr_ref);
	in_view_trace->transform_inv(cor_tr_ref, cor_tr_g);
	in_view_ref->transform(cor_tr_g, cor_tr_ref);
	if(!in_intrinsics_ref->project(cor_tr_ref, rb_tr_proj))	return false;
	tx=(int)rb_tr_proj.x;	ty=(int)rb_tr_proj.y;
	if(tx<0 || tx>ww-1 || ty<0 || ty>hh-1)	return false;	// check corner visibility.
	
	// draw camera trace.	
	// + center-to-corners.
	tx=(int)cen_tr_proj.x;		ty=(int)cen_tr_proj.y;	
	dx=(int)lt_tr_proj.x-tx;	dy=(int)lt_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
	dx=(int)rt_tr_proj.x-tx;	dy=(int)rt_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
	dx=(int)lb_tr_proj.x-tx;	dy=(int)lb_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
	dx=(int)rb_tr_proj.x-tx;	dy=(int)rb_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
	// + center-to-center.
	if(zz_cen_prev.x>=0 && zz_cen_prev.y>=0){
		dx=zz_cen_prev.x-tx;	dy=zz_cen_prev.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_trace);		
	}
	zz_cen_prev.x=tx;	zz_cen_prev.y=ty;
	// + corner-to-corners.
	tx=(int)lt_tr_proj.x;		ty=(int)lt_tr_proj.y;	
	dx=(int)rt_tr_proj.x-tx;	dy=(int)rt_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
	dx=(int)lb_tr_proj.x-tx;	dy=(int)lb_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
	tx=(int)rb_tr_proj.x;		ty=(int)rb_tr_proj.y;	
	dx=(int)rt_tr_proj.x-tx;	dy=(int)rt_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
	dx=(int)lb_tr_proj.x-tx;	dy=(int)lb_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);

	

	//io_img_trace->scr_Set_Cross((int)cen_tr_proj.x, (int)cen_tr_proj.y, 3, 3, color);

	return true;
	

}


////********************************************************************************************
//bool LCKvYooji_Scanner_Display::rct_Render_Camera_Trace(
//	CKvYooji_Extrinsics *in_view_ref,
//	CKvYooji_Extrinsics *in_view_trace,
//	CKvYooji_Intrinsics *in_intrinsics_ref,
//	CKvYooji_Intrinsics *in_intrinsics_trace,
//	CKvMatrixUcharRgb *io_img_trace,
//	CKvRgb *in_cam_color,
//	float in_ratio_between_camera_size_and_focal_length)
////********************************************************************************************
//{
//	CKvPoint3Df cen_tr_g, cen_tr_ref, cor_tr_g, cor_tr_ref;
//	CKvPointf cen_tr_proj, lt_tr_proj, rt_tr_proj, lb_tr_proj, rb_tr_proj;
//	CKvPixel t_pix;
//	CKvRgb color_cam, color_trace;
//	UCHAR *p_img_tr;
//	float f, fx, fy, px, py, d_min, d_max;
//	int ww, hh, tx, ty, dx, dy;
//	
//	color_cam = *in_cam_color;
//	//color_cam.s_Set((UCHAR)(rand()%128+128), (UCHAR)(rand()%128+128), (UCHAR)(rand()%128+128));		// random color.
//	color_trace.s_Set((UCHAR)0, (UCHAR)255, (UCHAR)0);
//
//	// get camera center position in the reference view coordinates.
//	in_view_trace->gcc_Get_Camera_Center(cen_tr_g);
//	in_view_ref->t_Transform(cen_tr_g, cen_tr_ref);
//	if(!in_intrinsics_ref->p_Project(cen_tr_ref, cen_tr_proj))	return false;
//	
//	p_img_tr=io_img_trace->mps(ww, hh)[0];
//	tx=(int)cen_tr_proj.x;	ty=(int)cen_tr_proj.y;
//
//	if(tx<0 || tx>ww-1 || ty<0 || ty>hh-1)	return false;	// check center visibility.
//
//	// get 3D corner position of image plane in the reference view coordinates.
//	// + for left-top corner.
//	in_intrinsics_trace->gp_Get_Parameters(fx, fy, px, py, d_min, d_max);
//	f=(fx+fy)*0.5f*0.001f;		// mm->m
//	f = in_ratio_between_camera_size_and_focal_length * f;
//	t_pix.x=0;	t_pix.y=0;	in_intrinsics_trace->bp_Back_Project(t_pix, f, cor_tr_ref);
//	in_view_trace->ti_Transform_Inverse(cor_tr_ref, cor_tr_g);
//	in_view_ref->t_Transform(cor_tr_g, cor_tr_ref);
//	if(!in_intrinsics_ref->p_Project(cor_tr_ref, lt_tr_proj))	return false;
//	tx=(int)lt_tr_proj.x;	ty=(int)lt_tr_proj.y;
//	if(tx<0 || tx>ww-1 || ty<0 || ty>hh-1)	return false;	// check corner visibility.
//
//	// + for right-top corner.	
//	t_pix.x=ww-1;	t_pix.y=0;	in_intrinsics_trace->bp_Back_Project(t_pix, f, cor_tr_ref);
//	in_view_trace->ti_Transform_Inverse(cor_tr_ref, cor_tr_g);
//	in_view_ref->t_Transform(cor_tr_g, cor_tr_ref);
//	if(!in_intrinsics_ref->p_Project(cor_tr_ref, rt_tr_proj))	return false;
//	tx=(int)rt_tr_proj.x;	ty=(int)rt_tr_proj.y;
//	if(tx<0 || tx>ww-1 || ty<0 || ty>hh-1)	return false;	// check corner visibility.
//
//	// + for left-bottom corner.	
//	t_pix.x=0;	t_pix.y=hh-1;	in_intrinsics_trace->bp_Back_Project(t_pix, f, cor_tr_ref);
//	in_view_trace->ti_Transform_Inverse(cor_tr_ref, cor_tr_g);
//	in_view_ref->t_Transform(cor_tr_g, cor_tr_ref);
//	if(!in_intrinsics_ref->p_Project(cor_tr_ref, lb_tr_proj))	return false;
//	tx=(int)lb_tr_proj.x;	ty=(int)lb_tr_proj.y;
//	if(tx<0 || tx>ww-1 || ty<0 || ty>hh-1)	return false;	// check corner visibility.
//
//	// + for right-bottom corner.	
//	t_pix.x=ww-1;	t_pix.y=hh-1;	in_intrinsics_trace->bp_Back_Project(t_pix, f, cor_tr_ref);
//	in_view_trace->ti_Transform_Inverse(cor_tr_ref, cor_tr_g);
//	in_view_ref->t_Transform(cor_tr_g, cor_tr_ref);
//	if(!in_intrinsics_ref->p_Project(cor_tr_ref, rb_tr_proj))	return false;
//	tx=(int)rb_tr_proj.x;	ty=(int)rb_tr_proj.y;
//	if(tx<0 || tx>ww-1 || ty<0 || ty>hh-1)	return false;	// check corner visibility.
//	
//	// draw camera trace.	
//	// + center-to-corners.
//	tx=(int)cen_tr_proj.x;		ty=(int)cen_tr_proj.y;	
//	dx=(int)lt_tr_proj.x-tx;	dy=(int)lt_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
//	dx=(int)rt_tr_proj.x-tx;	dy=(int)rt_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
//	dx=(int)lb_tr_proj.x-tx;	dy=(int)lb_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
//	dx=(int)rb_tr_proj.x-tx;	dy=(int)rb_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
//	// + center-to-center.
//	if(zz_cen_prev.x>=0 && zz_cen_prev.y>=0){
//		dx=zz_cen_prev.x-tx;	dy=zz_cen_prev.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_trace);		
//	}
//	zz_cen_prev.x=tx;	zz_cen_prev.y=ty;
//	// + corner-to-corners.
//	tx=(int)lt_tr_proj.x;		ty=(int)lt_tr_proj.y;	
//	dx=(int)rt_tr_proj.x-tx;	dy=(int)rt_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
//	dx=(int)lb_tr_proj.x-tx;	dy=(int)lb_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
//	tx=(int)rb_tr_proj.x;		ty=(int)rb_tr_proj.y;	
//	dx=(int)rt_tr_proj.x-tx;	dy=(int)rt_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
//	dx=(int)lb_tr_proj.x-tx;	dy=(int)lb_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
//
//	
//
//	//io_img_trace->scr_Set_Cross((int)cen_tr_proj.x, (int)cen_tr_proj.y, 3, 3, color);
//
//	return true;
//	
//
//}

////********************************************************************************************
//bool LCKvYooji_Scanner_Display::rsct_Render_Set_of_Camera_Traces(
//	CKvYooji_Extrinsics *in_view_ref,
//	vector<CKvYooji_Extrinsics*> *in_set_of_views_trace,
//	CKvYooji_Intrinsics *in_intrinsics_ref,
//	CKvYooji_Intrinsics *in_intrinsics_trace,
//	CKvMatrixUcharRgb *io_img_trace,
//	float in_ratio_between_camera_size_and_focal_length)
////********************************************************************************************
//{
//	// for debugging.
//	CKvPoint3Df cen_tr_g, cen_tr_ref; CKvPointf cen_tr_proj;
//	bool flag;
//	(*in_set_of_views_trace).back()->gcc_Get_Camera_Center(cen_tr_g);
//	in_view_ref->t_Transform(cen_tr_g, cen_tr_ref);
//	flag = in_intrinsics_ref->p_Project(cen_tr_ref, cen_tr_proj);
//
//	printf("%d %f %f %f -> %f %f\n", flag, cen_tr_ref.x, cen_tr_ref.y, cen_tr_ref.z,
//		cen_tr_proj.x, cen_tr_proj.y);
//	// for debugging.
//
//
//	zz_cen_prev.s_Set(-1, -1);
//	
//	// for previous cameras.
//	for(int i=0; i<in_set_of_views_trace->size() - 1; i++){
//		if(!rct_Render_Camera_Trace(in_view_ref, (*in_set_of_views_trace)[i], in_intrinsics_ref, in_intrinsics_trace, io_img_trace,
//			&Kv_Rgb(255, 0, 0),
//			in_ratio_between_camera_size_and_focal_length))
//			return false;
//	}
//
//	// for current cameras.
//	if(!rct_Render_Camera_Trace(in_view_ref, (*in_set_of_views_trace).back(), in_intrinsics_ref, in_intrinsics_trace, io_img_trace,
//		&Kv_Rgb(0, 255, 0),
//		in_ratio_between_camera_size_and_focal_length))
//		return false;
//
//	return true;
//}

//********************************************************************************************
bool LCKvYooji_Scanner_Display::rct_Render_Camera_Trace(
	CKvYooji_Extrinsics *in_view_ref,
	CKvYooji_Extrinsics *in_view_trace,
	CKvYooji_Intrinsics *in_intrinsics_ref,
	CKvYooji_Intrinsics *in_intrinsics_trace,
	CKvMatrixUcharRgb *io_img_trace,
	CKvRgb *in_cam_color,
	float in_ratio_between_camera_size_and_focal_length,
	bool in_flag_center_only)
//********************************************************************************************
{
	CKvPoint3Df cen_tr_g, cen_tr_ref, cor_tr_g, cor_tr_ref;
	CKvPointf cen_tr_proj, lt_tr_proj, rt_tr_proj, lb_tr_proj, rb_tr_proj;
	CKvPixel t_pix;
	CKvRgb color_cam, color_trace;
	UCHAR *p_img_tr;
	float f, fx, fy, px, py, d_min, d_max;
	int ww, hh, tx, ty, dx, dy;
	
	color_cam = *in_cam_color;
	//color_cam.s_Set((UCHAR)(rand()%128+128), (UCHAR)(rand()%128+128), (UCHAR)(rand()%128+128));		// random color.
	color_trace.s_Set((UCHAR)0, (UCHAR)255, (UCHAR)0);

	// get camera center position in the reference view coordinates.
	in_view_trace->get_cam_center(cen_tr_g);
	in_view_ref->transform(cen_tr_g, cen_tr_ref);
	if(!in_intrinsics_ref->project(cen_tr_ref, cen_tr_proj))	return false;

	p_img_tr=io_img_trace->mps(ww, hh)[0];
	tx=(int)cen_tr_proj.x;	ty=(int)cen_tr_proj.y;

	if(tx<0 || tx>ww-1 || ty<0 || ty>hh-1)	return false;	// check center visibility.

	if(!in_flag_center_only){
		// get 3D corner position of image plane in the reference view coordinates.
		// + for left-top corner.
		in_intrinsics_trace->get_params(fx, fy, px, py, d_min, d_max);
		f=(fx+fy)*0.5f*0.001f;		// mm->m
		f = in_ratio_between_camera_size_and_focal_length * f;
		t_pix.x=0;	t_pix.y=0;	in_intrinsics_trace->back_project(t_pix, f, cor_tr_ref);
		in_view_trace->transform_inv(cor_tr_ref, cor_tr_g);
		in_view_ref->transform(cor_tr_g, cor_tr_ref);
		if(!in_intrinsics_ref->project(cor_tr_ref, lt_tr_proj))	return false;
		tx=(int)lt_tr_proj.x;	ty=(int)lt_tr_proj.y;
		if(tx<0 || tx>ww-1 || ty<0 || ty>hh-1)	return false;	// check corner visibility.

		// + for right-top corner.	
		t_pix.x=ww-1;	t_pix.y=0;	in_intrinsics_trace->back_project(t_pix, f, cor_tr_ref);
		in_view_trace->transform_inv(cor_tr_ref, cor_tr_g);
		in_view_ref->transform(cor_tr_g, cor_tr_ref);
		if(!in_intrinsics_ref->project(cor_tr_ref, rt_tr_proj))	return false;
		tx=(int)rt_tr_proj.x;	ty=(int)rt_tr_proj.y;
		if(tx<0 || tx>ww-1 || ty<0 || ty>hh-1)	return false;	// check corner visibility.

		// + for left-bottom corner.	
		t_pix.x=0;	t_pix.y=hh-1;	in_intrinsics_trace->back_project(t_pix, f, cor_tr_ref);
		in_view_trace->transform_inv(cor_tr_ref, cor_tr_g);
		in_view_ref->transform(cor_tr_g, cor_tr_ref);
		if(!in_intrinsics_ref->project(cor_tr_ref, lb_tr_proj))	return false;
		tx=(int)lb_tr_proj.x;	ty=(int)lb_tr_proj.y;
		if(tx<0 || tx>ww-1 || ty<0 || ty>hh-1)	return false;	// check corner visibility.

		// + for right-bottom corner.	
		t_pix.x=ww-1;	t_pix.y=hh-1;	in_intrinsics_trace->back_project(t_pix, f, cor_tr_ref);
		in_view_trace->transform_inv(cor_tr_ref, cor_tr_g);
		in_view_ref->transform(cor_tr_g, cor_tr_ref);
		if(!in_intrinsics_ref->project(cor_tr_ref, rb_tr_proj))	return false;
		tx=(int)rb_tr_proj.x;	ty=(int)rb_tr_proj.y;
		if(tx<0 || tx>ww-1 || ty<0 || ty>hh-1)	return false;	// check corner visibility.
	
		// draw camera trace.	
		// + center-to-corners.
		tx=(int)cen_tr_proj.x;		ty=(int)cen_tr_proj.y;	
		dx=(int)lt_tr_proj.x-tx;	dy=(int)lt_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
		dx=(int)rt_tr_proj.x-tx;	dy=(int)rt_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
		dx=(int)lb_tr_proj.x-tx;	dy=(int)lb_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
		dx=(int)rb_tr_proj.x-tx;	dy=(int)rb_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);

		// + corner-to-corners.
		tx=(int)lt_tr_proj.x;		ty=(int)lt_tr_proj.y;	
		dx=(int)rt_tr_proj.x-tx;	dy=(int)rt_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
		dx=(int)lb_tr_proj.x-tx;	dy=(int)lb_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
		tx=(int)rb_tr_proj.x;		ty=(int)rb_tr_proj.y;	
		dx=(int)rt_tr_proj.x-tx;	dy=(int)rt_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
		dx=(int)lb_tr_proj.x-tx;	dy=(int)lb_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);

	}

	// draw camera trace.
	// + center-to-center.
	if(zz_cen_prev.x>=0 && zz_cen_prev.y>=0){
		tx=(int)cen_tr_proj.x;	ty=(int)cen_tr_proj.y;
		dx = zz_cen_prev.x-tx;	dy = zz_cen_prev.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_trace);
	}
	zz_cen_prev.x = tx;	zz_cen_prev.y = ty;

	//io_img_trace->scr_Set_Cross((int)cen_tr_proj.x, (int)cen_tr_proj.y, 3, 3, color);

	return true;
	

}

	//********************************************************************************************
bool LCKvYooji_Scanner_Display::rct_Render_Camera_Trace(
	CKvYooji_Extrinsics *in_view_ref,
	CKvYooji_Extrinsics *in_view_trace,
	CKvYooji_Intrinsics *in_intrinsics_ref,
	CKvYooji_Intrinsics *in_intrinsics_trace,
	CKvMatrixUcharRgb *io_img_trace,
	CKvRgb *in_cam_color,
	float in_ratio_between_camera_size_and_focal_length,
	bool in_flag_center_only,
	CKvRgb *in_center_color)
//********************************************************************************************
{
	CKvPoint3Df cen_tr_g, cen_tr_ref, cor_tr_g, cor_tr_ref;
	CKvPointf cen_tr_proj, lt_tr_proj, rt_tr_proj, lb_tr_proj, rb_tr_proj;
	CKvPixel t_pix;
	CKvRgb color_cam, color_trace;
	UCHAR *p_img_tr;
	float f, fx, fy, px, py, d_min, d_max;
	int ww, hh, tx, ty, dx, dy;
	
	color_cam = *in_cam_color;
	//color_cam.s_Set((UCHAR)(rand()%128+128), (UCHAR)(rand()%128+128), (UCHAR)(rand()%128+128));		// random color.
	if(in_center_color)	color_trace.s_Set(in_center_color->r, in_center_color->g, in_center_color->b);
	else				color_trace.s_Set((UCHAR)0, (UCHAR)255, (UCHAR)0);

	if(in_cam_color)	color_cam.s_Set(in_cam_color->r,in_cam_color->g,in_cam_color->b);
	else				color_cam.s_Set((UCHAR)0,(UCHAR)255,(UCHAR)0);

	// get camera center position in the reference view coordinates.
	in_view_trace->get_cam_center(cen_tr_g);
	in_view_ref->transform(cen_tr_g, cen_tr_ref);
	if(!in_intrinsics_ref->project(cen_tr_ref, cen_tr_proj))	return false;

	p_img_tr=io_img_trace->mps(ww, hh)[0];
	tx=(int)cen_tr_proj.x;	ty=(int)cen_tr_proj.y;

	if(tx<0 || tx>ww-1 || ty<0 || ty>hh-1)	return false;	// check center visibility.

	if(!in_flag_center_only){
		// get 3D corner position of image plane in the reference view coordinates.
		// + for left-top corner.
		in_intrinsics_trace->get_params(fx, fy, px, py, d_min, d_max);
		f=(fx+fy)*0.5f*0.001f;		// mm->m
		f = in_ratio_between_camera_size_and_focal_length * f;
		t_pix.x=0;	t_pix.y=0;	in_intrinsics_trace->back_project(t_pix, f, cor_tr_ref);
		in_view_trace->transform_inv(cor_tr_ref, cor_tr_g);
		in_view_ref->transform(cor_tr_g, cor_tr_ref);
		if(!in_intrinsics_ref->project(cor_tr_ref, lt_tr_proj))	return false;
		tx=(int)lt_tr_proj.x;	ty=(int)lt_tr_proj.y;
		if(tx<0 || tx>ww-1 || ty<0 || ty>hh-1)	return false;	// check corner visibility.

		// + for right-top corner.	
		t_pix.x=ww-1;	t_pix.y=0;	in_intrinsics_trace->back_project(t_pix, f, cor_tr_ref);
		in_view_trace->transform_inv(cor_tr_ref, cor_tr_g);
		in_view_ref->transform(cor_tr_g, cor_tr_ref);
		if(!in_intrinsics_ref->project(cor_tr_ref, rt_tr_proj))	return false;
		tx=(int)rt_tr_proj.x;	ty=(int)rt_tr_proj.y;
		if(tx<0 || tx>ww-1 || ty<0 || ty>hh-1)	return false;	// check corner visibility.

		// + for left-bottom corner.	
		t_pix.x=0;	t_pix.y=hh-1;	in_intrinsics_trace->back_project(t_pix, f, cor_tr_ref);
		in_view_trace->transform_inv(cor_tr_ref, cor_tr_g);
		in_view_ref->transform(cor_tr_g, cor_tr_ref);
		if(!in_intrinsics_ref->project(cor_tr_ref, lb_tr_proj))	return false;
		tx=(int)lb_tr_proj.x;	ty=(int)lb_tr_proj.y;
		if(tx<0 || tx>ww-1 || ty<0 || ty>hh-1)	return false;	// check corner visibility.

		// + for right-bottom corner.	
		t_pix.x=ww-1;	t_pix.y=hh-1;	in_intrinsics_trace->back_project(t_pix, f, cor_tr_ref);
		in_view_trace->transform_inv(cor_tr_ref, cor_tr_g);
		in_view_ref->transform(cor_tr_g, cor_tr_ref);
		if(!in_intrinsics_ref->project(cor_tr_ref, rb_tr_proj))	return false;
		tx=(int)rb_tr_proj.x;	ty=(int)rb_tr_proj.y;
		if(tx<0 || tx>ww-1 || ty<0 || ty>hh-1)	return false;	// check corner visibility.
	
		// draw camera trace.	
		// + center-to-corners.
		tx=(int)cen_tr_proj.x;		ty=(int)cen_tr_proj.y;	
		dx=(int)lt_tr_proj.x-tx;	dy=(int)lt_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
		dx=(int)rt_tr_proj.x-tx;	dy=(int)rt_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
		dx=(int)lb_tr_proj.x-tx;	dy=(int)lb_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
		dx=(int)rb_tr_proj.x-tx;	dy=(int)rb_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);

		// + corner-to-corners.
		tx=(int)lt_tr_proj.x;		ty=(int)lt_tr_proj.y;	
		dx=(int)rt_tr_proj.x-tx;	dy=(int)rt_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
		dx=(int)lb_tr_proj.x-tx;	dy=(int)lb_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
		tx=(int)rb_tr_proj.x;		ty=(int)rb_tr_proj.y;	
		dx=(int)rt_tr_proj.x-tx;	dy=(int)rt_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);
		dx=(int)lb_tr_proj.x-tx;	dy=(int)lb_tr_proj.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_cam);

	}

	// draw camera trace.
	// + center-to-center.
	if(zz_cen_prev.x>=0 && zz_cen_prev.y>=0){
		tx=(int)cen_tr_proj.x;	ty=(int)cen_tr_proj.y;
		dx = zz_cen_prev.x-tx;	dy = zz_cen_prev.y-ty;	io_img_trace->sl_Set_Line(tx, ty, dx, dy, color_trace);
	}
	zz_cen_prev.x = tx;	zz_cen_prev.y = ty;

	//io_img_trace->scr_Set_Cross((int)cen_tr_proj.x, (int)cen_tr_proj.y, 3, 3, color);

	return true;
	

}

//********************************************************************************************
bool LCKvYooji_Scanner_Display::rsct_Render_Set_of_Camera_Traces(
	CKvYooji_Extrinsics *in_view_ref,
	vector<CKvYooji_Extrinsics*> *in_set_of_views_trace,
	CKvYooji_Intrinsics *in_intrinsics_ref,
	CKvYooji_Intrinsics *in_intrinsics_trace,
	CKvMatrixUcharRgb *io_img_trace,
	float in_ratio_between_camera_size_and_focal_length,
	int in_sampling_rate,
	CKvRgb *in_center_color)
//********************************************************************************************
{
	bool flag_center_only;

	// for previous cameras.
	// render camera first.
	zz_cen_prev.s_Set(-1, -1);
	for(int i=0; i<in_set_of_views_trace->size() - 1; i++){

		if(in_sampling_rate > 0 && i%in_sampling_rate == 0)		flag_center_only = false;
		else													flag_center_only = true;

		rct_Render_Camera_Trace(in_view_ref, (*in_set_of_views_trace)[i], in_intrinsics_ref, in_intrinsics_trace, io_img_trace,
				&Kv_Rgb(0, 255, 0),
				in_ratio_between_camera_size_and_focal_length,
				//////////////////////////////////////////////////////////////////////////
				//true,
				flag_center_only,
				//////////////////////////////////////////////////////////////////////////
				in_center_color);
	}

	// render camera centers.
	zz_cen_prev.s_Set(-1,-1);
	for(int i=0; i<in_set_of_views_trace->size() - 1; i++){

		rct_Render_Camera_Trace(in_view_ref,(*in_set_of_views_trace)[i],in_intrinsics_ref,in_intrinsics_trace,io_img_trace,
				&Kv_Rgb(0,255,0),
				in_ratio_between_camera_size_and_focal_length,
				true,				
				in_center_color);
	}

	// for current cameras.
	if(!rct_Render_Camera_Trace(in_view_ref, (*in_set_of_views_trace).back(), in_intrinsics_ref, in_intrinsics_trace, io_img_trace,
		&Kv_Rgb(0, 0, 255),
		in_ratio_between_camera_size_and_focal_length,
		//////////////////////////////////////////////////////////////////////////
		false, //true,
		//////////////////////////////////////////////////////////////////////////
		in_center_color))
		return false;

	return true;
}

//********************************************************************************************
bool LCKvYooji_Scanner_Display::rsct_Render_Set_of_Camera_Traces_New(
	CKvYooji_Extrinsics *in_view_ref,
	vector<CKvYooji_Extrinsics*> *in_set_of_views_trace,
	CKvYooji_Intrinsics *in_intrinsics_ref,
	CKvYooji_Intrinsics *in_intrinsics_trace,
	CKvMatrixUcharRgb *io_img_trace,
	int in_idx_of_current_frame,
	float in_ratio_between_camera_size_and_focal_length,
	int in_sampling_rate,
	CKvRgb *in_center_color,
	CKvRgb *in_cam_color)
//********************************************************************************************
{
	bool flag_center_only;
	int curr_idx;

	if(in_set_of_views_trace->size() <= 0) return false;
	if(in_idx_of_current_frame < 0) curr_idx = in_set_of_views_trace->size() - 1;
	else curr_idx = in_idx_of_current_frame;

	// for previous cameras.
	// render camera first.
	zz_cen_prev.s_Set(-1, -1);
	for(int i=0; i<=curr_idx - 1; i++){

		if(in_sampling_rate > 0 && i%in_sampling_rate == 0)		flag_center_only = false;
		else													flag_center_only = true;

		rct_Render_Camera_Trace(in_view_ref, (*in_set_of_views_trace)[i], in_intrinsics_ref, in_intrinsics_trace, io_img_trace,
				&Kv_Rgb(0, 255, 0),
				in_ratio_between_camera_size_and_focal_length,
				//////////////////////////////////////////////////////////////////////////
				//true,
				flag_center_only,
				//////////////////////////////////////////////////////////////////////////
				in_center_color);
	}

	// render camera centers.
	zz_cen_prev.s_Set(-1,-1);
	for(int i=0; i<=curr_idx - 1; i++){

		rct_Render_Camera_Trace(in_view_ref,(*in_set_of_views_trace)[i],in_intrinsics_ref,in_intrinsics_trace,io_img_trace,
				&Kv_Rgb(0,255,0),
				in_ratio_between_camera_size_and_focal_length,
				true,				
				in_center_color);
	}

	// for current cameras.
	if(!rct_Render_Camera_Trace(in_view_ref, (*in_set_of_views_trace)[curr_idx], in_intrinsics_ref, in_intrinsics_trace, io_img_trace,
		in_cam_color,
		in_ratio_between_camera_size_and_focal_length,
		//////////////////////////////////////////////////////////////////////////
		false, //true,
		//////////////////////////////////////////////////////////////////////////
		in_center_color))
		return false;

	return true;
}

//********************************************************************************************
bool LCKvYooji_Scanner_Display::rsct_Render_Set_of_Camera_Traces(
	CKvYooji_Extrinsics *in_view_ref,
	vector<CKvYooji_Extrinsics*> *in_set_of_views_trace,
	CKvYooji_Intrinsics *in_intrinsics_ref,
	CKvYooji_Intrinsics *in_intrinsics_trace,
	CKvMatrixUcharRgb *io_img_trace,
	float in_ratio_between_camera_size_and_focal_length,
	int in_sampling_rate)
//********************************************************************************************
{
	bool flag_center_only;
	zz_cen_prev.s_Set(-1, -1);

	// for previous cameras.
	for(int i=0; i<in_set_of_views_trace->size() - 1; i++){

// 		if(!rct_Render_Camera_Trace(in_view_ref, (*in_set_of_views_trace)[i], in_intrinsics_ref, in_intrinsics_trace, io_img_trace,
// 			&Kv_Rgb(255, 0, 0),
// 			in_ratio_between_camera_size_and_focal_length))
// 			return false;

		if(i%in_sampling_rate == 0)		flag_center_only = false;
		else							flag_center_only = true;

		rct_Render_Camera_Trace(in_view_ref, (*in_set_of_views_trace)[i], in_intrinsics_ref, in_intrinsics_trace, io_img_trace,
				&Kv_Rgb(255, 0, 0),
				in_ratio_between_camera_size_and_focal_length,
				flag_center_only);
	}
	// for current cameras.
	if(!rct_Render_Camera_Trace(in_view_ref, (*in_set_of_views_trace).back(), in_intrinsics_ref, in_intrinsics_trace, io_img_trace,
		&Kv_Rgb(0, 255, 0),
		in_ratio_between_camera_size_and_focal_length))
		return false;

	return true;
}

//********************************************************************************************
// This function was written with reference to the following article.
// Very High Frame Rate Volumetric Integration of Depth Images on Mobile Devices. TVCG. 2015.
bool LCKvYooji_Scanner_Display::crtc_Cast_Ray_on_TSDF_Cube_NEW(
	CKvPixel &in_xy,
	CKvYooji_Cube_TSDF_Float *in_cube,
	CKvYooji_Intrinsics *in_intrinsic,
	CKvYooji_Extrinsics *in_hmat_glob_to_cam,

	CKvPoint3Df &in_norm_cc,
	CKvPoint3Df &in_cam_cen,
	float in_radius_cube,
	float in_dist_cam_center_to_cube_center,
	float in_theta_max,

	float in_mu,
	float in_voxel_sz_inv,
	CKvPoint3Df &out_p3d)
//********************************************************************************************
{
	bool flag_result;
	float theta, mag, dist_min, dist_max;

	flag_result=false;
	dist_min = in_dist_cam_center_to_cube_center-in_radius_cube;
	dist_max = in_dist_cam_center_to_cube_center+in_radius_cube;

	// compute the end point of the pixel ray for the input pixel.
	in_intrinsic->back_project(in_xy, dist_max, zz_pe_c);
	in_hmat_glob_to_cam->transform_inv(zz_pe_c, zz_pe_g);
	// compute the direction of the pixel ray for the input pixel using camera center and far plane of view frustum.
	zz_rd_g.x=zz_pe_g.x-in_cam_cen.x;	zz_rd_g.y=zz_pe_g.y-in_cam_cen.y;	zz_rd_g.z=zz_pe_g.z-in_cam_cen.z;		
	mag=zz_rd_g.n_Normalize();		// norm.
	
	// compute angle between the pixel ray and direction vector between the two centers.
	// check whether the ray crossed the object cube.
 	theta=acos(zz_rd_g.ps_Product_Scalar(in_norm_cc));
 	if(theta>in_theta_max)	return false;

	// cast ray.
	enum { SEARCH_COARSE, SEARCH_FINE, BEHIND_SURFACE, WRONG_STEP } state;

	bool flag_valid;
	float step_sz, step_sz_coarse, step_scale, sz_sub_cube, total_step, total_step_max, tsdf;
	
	// initialization.
	// calculates start point and ray direction for ray-casting.	
	zz_ps_g.x=in_cam_cen.x+dist_min*zz_rd_g.x;	
	zz_ps_g.y=in_cam_cen.y+dist_min*zz_rd_g.y;	
	zz_ps_g.z=in_cam_cen.z+dist_min*zz_rd_g.z;	
	// converts points and parameter coordinates from the real global coordinates to the cube voxel coordinates. (mm->voxel)
	// CAUTION: the offset position of global voxel cube is (-0.5f, -0.5f, -0.5f) in the cube voxel coordinates.
 	in_cube->gpv_Get_Position_in_Voxel(zz_ps_g, zz_ps_vox);
	//////////////////////////////////////////////////////////////////////////
	// compute scale factor for converting TSDF value to 'mm' unit.	
	step_scale = 0.9f*in_mu*in_voxel_sz_inv;						
	//////////////////////////////////////////////////////////////////////////
	// set initial and coarse-search step size for ray-casting as the side length of the sub cube.	
	// + coarse-search step size should be larger than 'step_scale'.
	sz_sub_cube=(float)in_cube->elsc_Edge_Length_of_Sub_Cube();
	step_sz=step_sz_coarse=max(step_scale+0.1f, sz_sub_cube);
	// without sub-cube check.
	//step_sz = step_sz_coarse = step_scale;
	//////////////////////////////////////////////////////////////////////////
	// set maximum search length of the pixel ray to 2*in_radius_cube.
	total_step_max=2.0f*in_radius_cube*in_voxel_sz_inv;		//	 total_step_max=pe_g.d_Distance(ps_g)*in_voxel_sz_inv;

	// starts casting ray.
	total_step=0.0f;
	flag_valid=in_cube->gtvu_Get_TSDF_Value_Uninterpolated(zz_ps_vox, tsdf);
	if(!flag_valid)		  state=SEARCH_COARSE;
	else if(tsdf >= 0.0f) state=SEARCH_FINE;
	else return false; //state=WRONG_STEP;
	

	while(state!=BEHIND_SURFACE){

		// decides step size according to searching mode.
		if(state == SEARCH_COARSE) step_sz = step_sz_coarse;
		//////////////////////////////////////////////////////////////////////////
		// min/max thresholding 없으면 무한 루프에 빠지는 경우가 발생한다. Why?
		else if(state == SEARCH_FINE) 
			step_sz = (tsdf > 0.0f) ? max(step_scale*tsdf, 1.0f) : min(step_scale*tsdf, -1.0f);
		//////////////////////////////////////////////////////////////////////////

		// proceeds ray casting.
		zz_ps_vox.x+=step_sz*zz_rd_g.x;		zz_ps_vox.y+=step_sz*zz_rd_g.y;		zz_ps_vox.z+=step_sz*zz_rd_g.z;	
		total_step += step_sz;
		
		// terminate ray casting if total step exceeds maximum step size.
		if(total_step>total_step_max)	break;

		// compute uninterpolated TSDF value.
		flag_valid=in_cube->gtvu_Get_TSDF_Value_Uninterpolated(zz_ps_vox, tsdf);	

		// check which current points is in mu band or not.
		if(tsdf>-1.0f && tsdf<1.0f){
			// ===========================================================
			// Mu-band processing.
			// ===========================================================
			// compute trilinear interpolated TSDF value.
			in_cube->gtvi_Get_TSDF_Value_Interpolated(zz_ps_vox, tsdf);
			// check current TSDF value is (-) at fine search step.
			if(tsdf<0.0f){
				if(state == SEARCH_FINE) state = BEHIND_SURFACE;
				// terminate ray casting if current TSDF value is (-) at coarse searching mode.
				else break;
			}
			else if(tsdf == 0.0f){
				// on surface.
				in_cube->gpw_Get_Position_in_World(zz_ps_vox,out_p3d);
				flag_result=true;
				break;
			}
			else if(state == SEARCH_COARSE)	state = SEARCH_FINE;			
		}
		else{
			if(state == SEARCH_FINE) state == SEARCH_COARSE;
		}
	}

	if(state==BEHIND_SURFACE){

		// compute trilinear interpolated TSDF value for the last step.
		step_sz = step_scale*tsdf;
		zz_ps_vox.x += step_sz*zz_rd_g.x;		zz_ps_vox.y+=step_sz*zz_rd_g.y;		zz_ps_vox.z+=step_sz*zz_rd_g.z;	
		in_cube->gtvi_Get_TSDF_Value_Interpolated(zz_ps_vox, tsdf);
		// compute the final step of current ray for extracting the surface point.
		step_sz=step_scale*tsdf;	
		zz_ps_vox.x+=step_sz*zz_rd_g.x;		zz_ps_vox.y+=step_sz*zz_rd_g.y;		zz_ps_vox.z+=step_sz*zz_rd_g.z;	
		// converts points coordinates from the cube voxel coordinates to the real global coordinates . (voxel->mm)
		in_cube->gpw_Get_Position_in_World(zz_ps_vox, out_p3d);

		flag_result=true;
	}

	return flag_result; 
}


//********************************************************************************************
bool LCKvYooji_Scanner_Display::crtc_Cast_Ray_on_TSDF_Cube(
	CKvPixel &in_xy,
	CKvYooji_Cube_TSDF_Float *in_cube,
	CKvYooji_Intrinsics *in_intrinsic,
	CKvYooji_Extrinsics *in_hmat_glob_to_cam,

	CKvPoint3Df &in_norm_cc,
	CKvPoint3Df &in_cam_cen,
	float in_radius_cube,
	float in_dist_cam_center_to_cube_center,
	float in_theta_max,

	float in_mu,
	float in_voxel_sz_inv,
	CKvPoint3Df &out_p3d)
//********************************************************************************************
{
	bool flag_result;
	float theta, mag, dist_min, dist_max;

	flag_result=false;
	dist_min = in_dist_cam_center_to_cube_center-in_radius_cube;
	dist_max = in_dist_cam_center_to_cube_center+in_radius_cube;

	// compute the end point of the pixel ray for the input pixel.
	in_intrinsic->back_project(in_xy, dist_max, zz_pe_c);
	in_hmat_glob_to_cam->transform_inv(zz_pe_c, zz_pe_g);
	// compute the direction of the pixel ray for the input pixel using camera center and far plane of view frustum.
	zz_rd_g.x=zz_pe_g.x-in_cam_cen.x;	zz_rd_g.y=zz_pe_g.y-in_cam_cen.y;	zz_rd_g.z=zz_pe_g.z-in_cam_cen.z;		
	mag=zz_rd_g.n_Normalize();		// norm.
	
	// compute angle between the pixel ray and direction vector between the two centers.
	// check whether the ray crossed the object cube.
 	theta=acos(zz_rd_g.ps_Product_Scalar(in_norm_cc));
 	if(theta>in_theta_max)	return false;

	// cast ray.
	enum { SEARCH_COARSE, SEARCH_FINE, BEHIND_SURFACE, WRONG_STEP } state;

	bool flag_valid;
	float step_sz, step_sz_coarse, step_scale, sz_sub_cube, total_step, total_step_max, tsdf;
	
	// initialization.
	// calculates start point and ray direction for ray-casting.	
	zz_ps_g.x=in_cam_cen.x+dist_min*zz_rd_g.x;	
	zz_ps_g.y=in_cam_cen.y+dist_min*zz_rd_g.y;	
	zz_ps_g.z=in_cam_cen.z+dist_min*zz_rd_g.z;	
	// converts points and parameter coordinates from the real global coordinates to the cube voxel coordinates. (mm->voxel)
	// CAUTION: the offset position of global voxel cube is (-0.5f, -0.5f, -0.5f) in the cube voxel coordinates.
 	in_cube->gpv_Get_Position_in_Voxel(zz_ps_g, zz_ps_vox);
	// compute scale factor for converting TSDF value to 'mm' unit.	
	step_scale=in_mu*in_voxel_sz_inv;						
	// set initial and coarse-search step size for ray-casting as the side length of the sub cube.	
	// + coarse-search step size should be larger than 'step_scale'.
	sz_sub_cube=(float)in_cube->elsc_Edge_Length_of_Sub_Cube();
	//step_sz=step_sz_coarse=max(step_scale+0.1f, sz_sub_cube);
	// + without sub-cube skipping.
	step_sz=step_sz_coarse=step_scale+0.1f;
	// set maximum search length of the pixel ray to 2*in_radius_cube.
	total_step_max=2.0f*in_radius_cube*in_voxel_sz_inv;		//	 total_step_max=pe_g.d_Distance(ps_g)*in_voxel_sz_inv;

	// starts casting ray.
	total_step=0.0f;
	flag_valid=in_cube->gtvu_Get_TSDF_Value_Uninterpolated(zz_ps_vox, tsdf);
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
		}
		else{
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
				step_sz=min(step_scale*tsdf, -1.0f);			
				break;

			default:
			case SEARCH_FINE:
				step_sz=max(step_scale*tsdf, 1.0f);	
			}
		}

		// proceeds ray casting.
		zz_ps_vox.x+=step_sz*zz_rd_g.x;		zz_ps_vox.y+=step_sz*zz_rd_g.y;		zz_ps_vox.z+=step_sz*zz_rd_g.z;	
		total_step+=step_sz;
		if(total_step>total_step_max)	break;

		flag_valid=in_cube->gtvu_Get_TSDF_Value_Uninterpolated(zz_ps_vox, tsdf);	
		
		if(tsdf>-1.0f && tsdf<1.0f){
			// add interpolation version of TSDF value calculator.
			in_cube->gtvi_Get_TSDF_Value_Interpolated(zz_ps_vox, tsdf);
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
			in_cube->gpw_Get_Position_in_World(zz_ps_vox, out_p3d);

			return true;
		}

	}

	if(state==BEHIND_SURFACE){

		step_sz=min(step_scale*tsdf, -1.0f);
		zz_ps_vox.x+=step_sz*zz_rd_g.x;		zz_ps_vox.y+=step_sz*zz_rd_g.y;		zz_ps_vox.z+=step_sz*zz_rd_g.z;	
		//in_cube->gtvu_Get_TSDF_Value_Uninterpolated(ps_vox, tsdf);
		in_cube->gtvi_Get_TSDF_Value_Interpolated(zz_ps_vox, tsdf);

		step_sz=step_scale*tsdf;	
		zz_ps_vox.x+=step_sz*zz_rd_g.x;		zz_ps_vox.y+=step_sz*zz_rd_g.y;		zz_ps_vox.z+=step_sz*zz_rd_g.z;	

		// converts points coordinates from the cube voxel coordinates to the real global coordinates . (voxel->mm)
		in_cube->gpw_Get_Position_in_World(zz_ps_vox, out_p3d);

		flag_result=true;
	}

	return flag_result; 
}


//********************************************************************************************
void LCKvYooji_Scanner_Display::rdmrc_Render_Depth_Map_on_RGB_Camera(
	CKvYooji_MatrixRgbD *in_view)
//********************************************************************************************
{
	//CKvMatrixUcharRgb out_color;

	CKvPointf t_pixf_d, t_pixf_rgb;

	int ww_rgb, hh_rgb, ww_d, hh_d, x, y, tidx;
	float *p_depth, *p_depth_rgb, td;
	UCHAR *p_color, *p_out_color, *p_normal;

	p_color=in_view->p_image_rgb()->mps(ww_rgb, hh_rgb)[0];
	//p_out_color = out_color.cp_Copy(in_view->prgbi_Pointer_of_RGB_Image())[0];

	//if(out_rendered_normal->mw()!=ww_rgb || out_rendered_normal->mh()!=hh_rgb)	out_rendered_normal->c_Create(hh_rgb, ww_rgb);
	p_depth = in_view->p_map_depth_raw()->mps(ww_d, hh_d)[0];
	if(ww_d != in_view->p_map_depth_filtered_on_RGB()->mw()
		|| hh_d != in_view->p_map_depth_filtered_on_RGB()->mh()){
		in_view->p_map_depth_filtered_on_RGB()->c_Create(hh_d, ww_d, 0.0f);
	}
	p_depth_rgb = in_view->p_map_depth_filtered_on_RGB()->vp();

	for(int k=0; k<ww_d*hh_d; k++){
		if(p_depth[k] > 0.0f){

			t_pixf_d.x=(float)(k%ww_d); t_pixf_d.y=(float)(k/ww_d);
			td=p_depth[k];

			if(!in_view->convert_pixel_coord(t_pixf_d, td, t_pixf_rgb))	continue;
						
			x=(int)t_pixf_rgb.x;	y=(int)t_pixf_rgb.y;

			tidx=y*ww_rgb+x;
			p_depth_rgb[tidx] = p_depth[k];		// set depth w/o interpolation.

// 			p_out_color[tidx] = (UCHAR)(0.5f*p_color[tidx] + 0.0f);
// 			p_out_color[tidx+ww_rgb*hh_rgb] = (UCHAR)(0.5f*p_color[tidx+ww_rgb*hh_rgb] + 128.0f);
// 			p_out_color[tidx+2*ww_rgb*hh_rgb] = (UCHAR)(0.5f*p_color[tidx+2*ww_rgb*hh_rgb] + 0.0f);
		}
	}

	//CKvScreen sc;
	//sc.s_d_Display(&out_color);
	//if(!Kv_Printf("Depth on RGB"))	exit(0);
}

//********************************************************************************************
void LCKvYooji_Scanner_Display::rn_Render_Normal_on_RGB_Image(
	CKvMatrixUcharRgb *in_img_rgb,
	CKvMatrixUchar *in_img_normal,
	CKvMatrixUcharRgb *out_img_rgb)
//********************************************************************************************
{
	int ww, hh, len;
	UCHAR *p_color, *p_out_color, *p_normal;

	p_color=in_img_rgb->mps(ww, hh)[0]; len = ww*hh;
	p_normal = in_img_normal->vp();

// 	printf("size: %d %d\n", ww, hh);
// 	printf("size: %d %d\n",in_img_normal->mw(),in_img_normal->mh());


	out_img_rgb->cp_Copy(in_img_rgb);
// 	if(out_img_rgb->mw() != ww || out_img_rgb->mh() != hh)
// 		out_img_rgb->c_Create(hh, ww, Kv_Rgb(0,0,0));
	p_out_color = out_img_rgb->vp();

	for(int tidx=0; tidx<len; tidx++){
		if(p_normal[tidx] > uchar(0)){

			for(int n=0; n<3; n++)	p_out_color[tidx + n*len] = p_normal[tidx];
		}
	}

	//CKvScreen sc;
	//sc.s_d_Display(&out_color);
	//if(!Kv_Printf("Depth on RGB"))	exit(0);
}

//********************************************************************************************
void LCKvYooji_Scanner_Display::pt_Put_Text_using_OpenCV(
	CKvMatrixUcharRgb &io_img_rgb)
//********************************************************************************************
{
	CKvYooji_InterLib_Convert zz_ilc;
	Mat timg_cv;

	/// Text
	string myText = "Re-localization...";

	/// Text Location
	cv::Point myPoint;
	myPoint.x = 10;
	myPoint.y = 40;

	/// Font Face
	int myFontFace = 2;

	/// Font Scale
	double myFontScale = 1.2;

	zz_ilc.cfko_Convert_Format_from_KAISION_to_Opencv(io_img_rgb,timg_cv);
	cv::putText(timg_cv,myText,myPoint,myFontFace,myFontScale,cv::Scalar::all(255));
	zz_ilc.cfok_Convert_Format_from_Opencv_to_KAISION(timg_cv,io_img_rgb);
}

//********************************************************************************************
void LCKvYooji_Scanner_Display::dpc_Display_Point_Clouds(
	CKvMatrixFloat *in_mapDepth,
	CKvMatrixUcharRgb *in_imgColor,
	CKvYooji_Intrinsics *in_intrinsics)
//********************************************************************************************
{
	CKvGraph3D zz_gd;
	CKvHpoint3D temp_Hp;

	CKvDepot_of_Point3D depot_3d_points;
	CKvDepot_of_RgbaF depot_colors;
	CKvMesh_of_Triangle mesh_triangle;

	// Makes meshes.
	z_mmpt_Make_Mesh_Points_with_Texture(
		in_mapDepth,
		in_imgColor,
		in_intrinsics,
		&depot_3d_points,
		&depot_colors,
		&mesh_triangle);

	// Displays meshes.
	zz_gd.g_sp_Set_display_Parameters(false, Kv_RgbF(0.0f, 0.0f, 0.0f), false, temp_Hp, 0.0, 0.0, false, false, 0);	
	zz_gd.g_id_Initialize_Depots(&depot_3d_points, &depot_colors, NULL, NULL, NULL, NULL, NULL);
	zz_gd.g_pt_Plot_Triangle(NULL, NULL, &mesh_triangle, true);

	if(!Kv_Printf("Graph 3D!"))	exit(0);

}

//********************************************************************************************
void LCKvYooji_Scanner_Display::gcps_Get_Camera_Projection_Sphere(
	CKvPoint3D &in_center,
	float in_radius,
	CKvDepot_of_Point3D *out_depot_of_points,
	CKvDepot_of_RgbaF *out_depot_of_colors,
	CKvMesh_of_Triangle *out_mesh_tri)
//********************************************************************************************
{
	LCKvUtility_for_Mesh_Creation aa_mc;

	CKvBsphere projectSphere;
	CKvRgbaF color_f, color_b;

	CKvSet_of_VectorInt set_vi;
	CKvVectorInt set_pi;
	int tmp, num_points, offset[3];
	bool tmp_b;

	color_f.s_Set(0.0f, 0.0f, 0.0f, 1.0f);		color_b.s_Set(0.0f, 0.0f, 0.0f, 1.0f);
	projectSphere.s_Set(in_center, in_radius);
	aa_mc.mil_Make_Index_List(projectSphere, 64, 64, out_depot_of_points, &set_pi, tmp_b);
	num_points=out_depot_of_points->ne_Number_of_Elements();
	
	offset[0]=offset[1]=offset[2]=0;
	out_mesh_tri->in_Initialize();
	out_mesh_tri->u_me_Make_Element(&set_pi, &set_pi, &set_pi, &set_vi);	
	out_mesh_tri->ap_Append(true, &set_vi, offset, tmp);

	out_depot_of_colors->in_Initialize();
	for(int i=0; i<num_points; i++){
		out_depot_of_colors->ap_Append(false, Kv_RgbaF(1.0f, 0.0f, 0.0f, 1.0f), tmp);
	}		

}

//********************************************************************************************
void LCKvYooji_Scanner_Display::dcps_Display_Camera_Projection_Sphere(
	CKvGraph3D *in_graph_3d, 
	CKvYooji_Tracking_State *in_state_track,
	CKvDepot_of_Point3D *in_point_depot_sphere, 
	CKvDepot_of_RgbaF *io_color_depot_sphere,
	CKvMesh_of_Triangle *in_mesh_triangle_sphere)
//********************************************************************************************
{
	// check each surface voxel of camera projection sphere covered by current camera.
	CKvPmatrix3D pmat;
	CKvYooji_Intrinsics *intrins = in_state_track->p_intrinsics_RGB();
	CKvYooji_Extrinsics *pose = in_state_track->p_extrinsics_glob_to_cam_RGB();

	CKvPoint3D tp_3d;
	CKvPoint3D tp_out;
	CKvPointf tp2d;
	CKvRgbaF tc;
		
	CKvPoint3Df p3d_s, p3d_c;

	Vector2f v2f;

	float *p_depth;
	float *p_P;

	int x, y, ww, hh;
	float fx_d, fy_d, px_d, py_d, scale;

	p_depth = in_state_track->p_pyramid_map_depth_rendered()->imgs[0].mps(ww, hh)[0];

	for(int i=0; i<in_point_depot_sphere->ne_Number_of_Elements(); i++){

		io_color_depot_sphere->ge_Get_Element(i, tc);
		if(tc.r==1.0f && tc.g==0.0f && tc.b==0.0f){
			

			in_point_depot_sphere->ge_Get_Element(i, tp_3d);

			p3d_s.x = tp_3d.x; p3d_s.y = tp_3d.y; p3d_s.z = tp_3d.z;

			pose->transform(p3d_s, p3d_c);
			intrins->project(p3d_c, tp2d);

			//printf("p3d: %f %f %f\n", p3d_s.x, p3d_s.y, p3d_s.z);
			
			
			float td;

			v2f.x = tp2d.x; v2f.y = tp2d.y;
			

			if(!d_gid_Get_Interpolated_Depth(v2f,ww,hh,p_depth,td)) continue;

			// check sphere point is in front of depth surface. visibility check!
			if(td < p3d_c.z){	
				io_color_depot_sphere->se_Set_Element(i, Kv_RgbaF(0.0f, 1.0f, 0.0f, 1.0f));	
				in_state_track->update_num_coverage();
			}

		}

		//io_color_depot->se_Set_Element(i, Kv_RgbaF(0.0f, 1.0f, 0.0f, 1.0f));
	}


// 	in_graph_3d->g_id_Initialize_Depots(
// 		in_point_depot, //CKvDepot_of_Point3D *in_set_of_points_or_NULL,
// 		io_color_depot, //CKvDepot_of_RgbaF *in_set_of_colors_or_NULL,
// 		NULL, //&zz_depot_image, //CKvDepot_of_String *in_set_of_images_or_NULL,
// 		NULL, //&zz_depot_image_point, //CKvDepot_of_Point *in_set_of_image_points_or_NULL,
// 		NULL, //&depot_Pmat, //CKvDepot_of_Pmatrix3D *in_set_of_P_matrices_or_NULL,
// 		NULL, //CKvDepot_of_Font *in_set_of_fonts_or_NULL,
// 		NULL); //CKvDepot_of_String *in_set_of_text_strings_or_NULL);
	if(in_graph_3d){
		
		in_state_track->compute_P_matrix(intrins,pose,&pmat);
		in_graph_3d->g_p_Plot(
			&pmat, 
			//NULL,
			in_point_depot_sphere, NULL, 
			io_color_depot_sphere, 
			in_mesh_triangle_sphere, NULL, NULL, NULL, NULL, NULL, 
			true);

		//if(!Kv_Printf("Stop"))		exit(0);
	}

}


//********************************************************************************************
void LCKvYooji_Scanner_Display::gco_Get_Cube_Origin_in_world(
	CKvYooji_MatrixRgbD &in_rgbd_mat_init,
	float in_cube_size_in_meter,
	float in_z_offset_of_cube_in_meter,	
	CKvPoint3Df &out_cube_origin)
//********************************************************************************************
{	
	CKvScreen sc;
	CKvString fn;
		
	CKvMatrixFloat *p_img_depth;
	CKvMatrixUcharRgb img_depth_u8;
	CKvMatrixUchar img_depth_u8g;

	CKvPixel pix, pix_prev;
	CKvPointf p2d;
	CKvPoint3Df p3d,p3d2;

	CKvYooji_Intrinsics intrin;
	CKvYooji_Extrinsics extrin1, extrin2;

	int flag_mouse_prev, flag_mouse_curr, flag_xy_trans;
	int ww, hh, x_cen, y_cen;

	float *p_img_depth_float;

	CKvPmatrix3D *p_pmat;
	CKvMatrixFloat tmat, hmat;	

	CKvPoint3D oc, pa;
	CKvPoint3Df ocf, origin;
	float ref_d, def_d, cube_near_d, delta;
	
	float min_d, max_d;
	def_d = 0.0f;
	min_d = 100000.0f; max_d = -100000.0f;

	// get pointers.
	p_img_depth = in_rgbd_mat_init.p_map_depth_raw();
	p_pmat = in_rgbd_mat_init.p_P_matrix_depth();

	// set cameras for drawing cube.
	p_img_depth_float=p_img_depth->mps(ww, hh)[0];
	intrin.set_from_P_matrix(p_pmat);
	extrin1.set_from_params(0.0f,0.0f,0.0f, 0.0f,0.0f,0.0f);
	//extrin2.sp_Set_from_Parameters(0.0f,0.5f*PI,0.0f, -2.0f,0.0f,2.0f);	

	p_pmat->oc_Optical_Center().g_Get(oc);
	pa=p_pmat->dvo_Direction_Vector_of_Optical_axis();
	origin.x=(float)-oc.x; origin.y=(float)-oc.y; origin.z=(float)-oc.z;
	ref_d = (float)(origin.x*pa.x +origin.y*pa.y +origin.z*pa.z);

	// set volume of interest.		
	x_cen=ww/2;	y_cen=hh/2;
	p2d.x=(float)x_cen;	p2d.y=(float)y_cen;
	cube_near_d = in_z_offset_of_cube_in_meter;
	if(cube_near_d<=0.0f) cube_near_d = ref_d-in_cube_size_in_meter*0.5f;
	intrin.back_project(p2d, cube_near_d, p3d);
	p3d.x-=in_cube_size_in_meter*0.5f;	p3d.y-=in_cube_size_in_meter*0.5f;	

	// compensate depth values by reference offset.
	for(int i=0; i<ww*hh; i++){
		// for depth image captured from KAISION.
		if(p_img_depth_float[i]<=0.0f){
			p_img_depth_float[i] = def_d;
			continue;
		}

		if(min_d>p_img_depth_float[i]) min_d = p_img_depth_float[i];
		if(max_d<p_img_depth_float[i]) max_d = p_img_depth_float[i];
	}

	// front view.
	dvoi_Draw_Volume_Of_Interest(
		p_img_depth,
		&intrin,
		&extrin1,
		in_cube_size_in_meter,
		p3d,
		&img_depth_u8);

	sc.s_smbm_Set_Mouse_Button_Mode(false);
	sc.s_d_Display(&img_depth_u8);	

	flag_mouse_curr=flag_mouse_prev=flag_xy_trans=-100000;	
	while(1){
		pix = sc.s_gmps_Get_Mouse_Position_and_Status(flag_mouse_curr);		
	
		// display new object cube.
		// x-y translation.
		if(flag_mouse_curr==1){

			p2d.x=(float)pix.x;	p2d.y=(float)pix.y;
			intrin.back_project(p2d, cube_near_d, p3d);
			p3d.x-=in_cube_size_in_meter*0.5f;	p3d.y-=in_cube_size_in_meter*0.5f;

			dvoi_Draw_Volume_Of_Interest(
				p_img_depth,
				&intrin,
				&extrin1,
				in_cube_size_in_meter,
				p3d,
				&img_depth_u8);
			sc.s_d_Display(&img_depth_u8);			
		}
		// z translation.
		if(flag_xy_trans==1 && flag_mouse_curr==-1){

			delta = (float)(pix.y-pix_prev.y);
			cube_near_d -= 0.01f*delta;
			intrin.back_project(p2d, cube_near_d, p3d);
			p3d.x-=in_cube_size_in_meter*0.5f;	p3d.y-=in_cube_size_in_meter*0.5f;

			dvoi_Draw_Volume_Of_Interest(
				p_img_depth,
				&intrin,
				&extrin1,
				in_cube_size_in_meter,
				p3d,
				&img_depth_u8);
			sc.s_d_Display(&img_depth_u8);			
		}

		// set cube origin.
		if(flag_mouse_curr==0 && flag_mouse_prev==1){
			flag_xy_trans = 1;
		}
		if(flag_xy_trans==1 && flag_mouse_curr==0 && flag_mouse_prev==-1){
			//if(!Kv_Printf("!!!!!"))	exit(0);
			out_cube_origin=p3d;
			//if(!Kv_Printf("!!!!!"))	exit(0);
			break;
		}

		flag_mouse_prev=flag_mouse_curr;
		pix_prev = pix;		

		Kv_Pause(10);
	}	

}


//********************************************************************************************
void LCKvYooji_Scanner_Display::soc_Set_Object_Cube(
	CKvYooji_MatrixRgbD &in_rgbd_mat_init,
	float in_z_offset_of_cube_in_meter,
	float &io_cube_size_in_meter,
	CKvPoint3Df &out_cube_origin)
//********************************************************************************************
{	
	CKvScreen sc;
	CKvString fn;
		
	CKvMatrixFloat *p_img_depth;
	CKvMatrixUcharRgb img_depth_u8;
	CKvMatrixUchar img_depth_u8g;

	CKvPixel pix, pix_prev;
	CKvPointf p2d;
	CKvPoint3Df p3d,p3d2;

	CKvYooji_Intrinsics intrin;
	CKvYooji_Extrinsics extrin1, extrin2;

	int flag_mouse_prev, flag_mouse_curr, flag_xy_trans;
	int ww, hh, x_cen, y_cen;

	float *p_img_depth_float;

	CKvPmatrix3D *p_pmat;
	CKvMatrixFloat tmat, hmat;	

	CKvPoint3D oc, pa;
	CKvPoint3Df ocf, origin;
	float ref_d, def_d, cube_near_d, delta;
	
	float min_d, max_d;
	def_d = 0.0f;
	min_d = 100000.0f; max_d = -100000.0f;

	// get pointers.
	p_img_depth = in_rgbd_mat_init.p_map_depth_raw();
	p_pmat = in_rgbd_mat_init.p_P_matrix_depth();

	// set cameras for drawing cube.
	p_img_depth_float=p_img_depth->mps(ww, hh)[0];
	intrin.set_from_P_matrix(p_pmat);
	extrin1.set_from_params(0.0f,0.0f,0.0f, 0.0f,0.0f,0.0f);
	//extrin2.sp_Set_from_Parameters(0.0f,0.5f*PI,0.0f, -2.0f,0.0f,2.0f);	

	p_pmat->oc_Optical_Center().g_Get(oc);
	pa=p_pmat->dvo_Direction_Vector_of_Optical_axis();
	origin.x=(float)-oc.x; origin.y=(float)-oc.y; origin.z=(float)-oc.z;
	ref_d = (float)(origin.x*pa.x +origin.y*pa.y +origin.z*pa.z);

	// set volume of interest.		
	x_cen=ww/2;	y_cen=hh/2;
	p2d.x=(float)x_cen;	p2d.y=(float)y_cen;
	cube_near_d = in_z_offset_of_cube_in_meter;
	if(cube_near_d<=0.0f) cube_near_d = ref_d-io_cube_size_in_meter*0.5f;
	intrin.back_project(p2d, cube_near_d, p3d);
	p3d.x-=io_cube_size_in_meter*0.5f;	p3d.y-=io_cube_size_in_meter*0.5f;	

	// compensate depth values by reference offset.
	for(int i=0; i<ww*hh; i++){
		// for depth image captured from KAISION.
		if(p_img_depth_float[i]<=0.0f){
			p_img_depth_float[i] = def_d;
			continue;
		}

		if(min_d>p_img_depth_float[i]) min_d = p_img_depth_float[i];
		if(max_d<p_img_depth_float[i]) max_d = p_img_depth_float[i];
	}

	// front view.
	dvoi_Draw_Volume_Of_Interest(
		p_img_depth,
		&intrin,
		&extrin1,
		io_cube_size_in_meter,
		p3d,
		&img_depth_u8);

	sc.s_smbm_Set_Mouse_Button_Mode(false);
	sc.s_d_Display(&img_depth_u8);	

	flag_mouse_curr=flag_mouse_prev=flag_xy_trans=-100000;	
	while(1){
		pix = sc.s_gmps_Get_Mouse_Position_and_Status(flag_mouse_curr);		
			
		//printf("flag_mouse_curr: %d\n", flag_mouse_curr);	
	
		// display new object cube.
		// x-y translation.
		if(flag_mouse_curr==1){

			p2d.x=(float)pix.x;	p2d.y=(float)pix.y;
			intrin.back_project(p2d, cube_near_d, p3d);
			p3d.x-=io_cube_size_in_meter*0.5f;	p3d.y-=io_cube_size_in_meter*0.5f;

			dvoi_Draw_Volume_Of_Interest(
				p_img_depth,
				&intrin,
				&extrin1,
				io_cube_size_in_meter,
				p3d,
				&img_depth_u8);
			sc.s_d_Display(&img_depth_u8);			
		}
		// z translation.
		if(flag_xy_trans==1 && flag_mouse_curr==-1){

			delta = (float)(pix.y-pix_prev.y);
			cube_near_d -= 0.01f*delta;
			intrin.back_project(p2d, cube_near_d, p3d);
			p3d.x-=io_cube_size_in_meter*0.5f;	p3d.y-=io_cube_size_in_meter*0.5f;

			dvoi_Draw_Volume_Of_Interest(
				p_img_depth,
				&intrin,
				&extrin1,
				io_cube_size_in_meter,
				p3d,
				&img_depth_u8);
			sc.s_d_Display(&img_depth_u8);
		}
		// cube size.


		// set cube origin.
		if(flag_mouse_curr==0 && flag_mouse_prev==1){
			flag_xy_trans = 1;
		}

		if(sc.s_gc_Get_Character() == 'q'){
		//if(flag_xy_trans==1 && flag_mouse_curr==0 && flag_mouse_prev==-1){
			//if(!Kv_Printf("!!!!!"))	exit(0);
			out_cube_origin=p3d;
			//if(!Kv_Printf("!!!!!"))	exit(0);
			break;
		}

		flag_mouse_prev=flag_mouse_curr;
		pix_prev = pix;		

		Kv_Pause(10);
	}	

}

//********************************************************************************************
void LCKvYooji_Scanner_Display::cidfu_Convert_Image_Depth_Float_to_Uchar(CKvMatrixFloat *in_img_d,
	int in_mode_convert,
	CKvMatrixUchar *out_img_d)
//********************************************************************************************
{
	int out_ww, out_hh;
	float max_d, min_d, scale, offset;
	unsigned char *p_out_img_d;
	float *p_in_img_d;

	offset = 30.0f;

	p_out_img_d=out_img_d->mps(out_hh, out_ww)[0];
	p_in_img_d=in_img_d->vp();

	if(out_ww != in_img_d->mw() || out_hh != in_img_d->mh()){

		out_ww=in_img_d->mw();	out_hh=in_img_d->mh();
		p_out_img_d=out_img_d->c_Create(out_hh, out_ww, (unsigned char)0)[0];
	}

	max_d=-1000000.0f;
	min_d=1000000.0f;

	// find min/max depth value.
  	for(int i=0; i<out_ww*out_hh; i++){
  		if(p_in_img_d[i]==0.0f)	continue;
  		if(p_in_img_d[i]<min_d)		min_d=p_in_img_d[i];
  		if(p_in_img_d[i]>max_d)		max_d=p_in_img_d[i];
  	}
  	scale=1.0f/(max_d-min_d);
	
	// convert raw depth value to 8bit depth or disparity.
	for(int i=0; i<out_ww*out_hh; i++){
		if(p_in_img_d[i]==0.0f)	continue;
		if(in_mode_convert==KV_CONVERT_IMAGE_DEPTH_FL_TO_DISP_UC)
			p_out_img_d[i]=(unsigned char)(255.0f-(255.0f-offset)*(p_in_img_d[i]-min_d)*scale);
		else
			p_out_img_d[i]=(unsigned char)((255.0f-offset)*(p_in_img_d[i]-min_d)*scale +offset);
	}
}


//********************************************************************************************
void LCKvYooji_Scanner_Display::cidfu_Convert_Image_Depth_Float_to_Uchar(CKvMatrixFloat *in_img_d,
	int in_mode_convert,
	float in_min_depth,
	float in_max_depth,
	CKvMatrixUchar *out_img_d)
//********************************************************************************************
{
	int out_ww, out_hh;
	float max_d, min_d, scale, offset;
	unsigned char *p_out_img_d;
	float *p_in_img_d;

	offset = 30.0f;

	p_out_img_d=out_img_d->mps(out_hh, out_ww)[0];
	p_in_img_d=in_img_d->vp();

	if(out_ww != in_img_d->mw() || out_hh != in_img_d->mh()){

		out_ww=in_img_d->mw();	out_hh=in_img_d->mh();
		p_out_img_d=out_img_d->c_Create(out_hh, out_ww, (unsigned char)0)[0];
	}

	// set min/max value and scale.
	max_d=in_max_depth;
	min_d=in_min_depth;

  	scale=1.0f/(max_d-min_d);
	
	// convert raw depth value to 8bit depth or disparity.
	for(int i=0; i<out_ww*out_hh; i++){
		
		if(p_in_img_d[i]==0.0f)	continue;

		if(p_in_img_d[i]<min_d || p_in_img_d[i]>max_d){
			p_out_img_d[i] = (unsigned char)0;
			continue;
		}

		if(in_mode_convert==KV_CONVERT_IMAGE_DEPTH_FL_TO_DISP_UC)
			p_out_img_d[i]=(unsigned char)(255.0f-(255.0f-offset)*(p_in_img_d[i]-min_d)*scale);
		else
			p_out_img_d[i]=(unsigned char)((255.0f-offset)*(p_in_img_d[i]-min_d)*scale +offset);
	}
}

//********************************************************************************************
void LCKvYooji_Scanner_Display::cidfu_Convert_Image_Depth_Float_to_Uchar(CKvMatrixFloat *in_img_d,
	int in_mode_convert,
	float in_default_val,
	CKvMatrixUchar *out_img_d)
//********************************************************************************************
{
	int out_ww, out_hh;
	float max_d, min_d, scale, offset;
	unsigned char *p_out_img_d;
	float *p_in_img_d;

	p_out_img_d=out_img_d->mps(out_hh, out_ww)[0];
	p_in_img_d=in_img_d->vp();

	if(out_ww != in_img_d->mw() || out_hh != in_img_d->mh()){

		out_ww=in_img_d->mw();	out_hh=in_img_d->mh();
		p_out_img_d=out_img_d->c_Create(out_hh, out_ww, (unsigned char)0)[0];
	}

	max_d=-1000000.0f;
	min_d=1000000.0f;
	
	// find min/max depth value.
  	for(int i=0; i<out_ww*out_hh; i++){
  		if(p_in_img_d[i]==in_default_val)	continue;
  		if(p_in_img_d[i]<min_d)		min_d=p_in_img_d[i];
  		if(p_in_img_d[i]>max_d)		max_d=p_in_img_d[i];
  	}
	if(max_d == min_d) max_d = min_d + 2.0f;
	if(max_d == in_default_val) max_d -= 0.1f;
  	
	scale=1.0f/(max_d-min_d);
	offset = 30.0f;

	// convert raw depth value to 8bit depth or disparity.
	for(int i=0; i<out_ww*out_hh; i++){
		if(p_in_img_d[i]==in_default_val)	continue;
		if(in_mode_convert==KV_CONVERT_IMAGE_DEPTH_FL_TO_DISP_UC)
			p_out_img_d[i]=(unsigned char)(255.0f-(255.0f-offset)*(p_in_img_d[i]-min_d)*scale);
		else
			p_out_img_d[i]=(unsigned char)((255.0f-offset)*(p_in_img_d[i]-min_d)*scale +offset);
	}
}

//********************************************************************************************
void LCKvYooji_Scanner_Display::dvoi_Draw_Volume_Of_Interest(CKvMatrixFloat *in_imgDepth,
	CKvYooji_Intrinsics *in_intrinsics_d,
	CKvYooji_Extrinsics *in_extrinsics,
	float in_cube_size,
	CKvPoint3Df &in_cube_offset,
	CKvMatrixUcharRgb *out_img_VOI)
//********************************************************************************************
{
	int ww, hh, i, j, k, tidx;
	float *pImgDepth = in_imgDepth->mps(ww, hh)[0];
	UCHAR *pImgVOI;

	if (out_img_VOI->mw() != ww || out_img_VOI->mh() != hh){
		out_img_VOI->c_Create(hh, ww);
	}
	pImgVOI = out_img_VOI->vp();

	// initialize VOI image.
	for (i = 0; i < 3 * ww*hh; i++)	{ pImgVOI[i] = (UCHAR)0; }

	// draw VOI on depth map.
	CKvPointf p2d;	CKvPoint3Df p3d, p3d2;
	CKvPoint3Df cornersVOI[8];
	CKvPointf projectedVOI[8];
	float sizeCube, td;
	float x_min, y_min, z_min, x_max, y_max, z_max;

	sizeCube=in_cube_size;
 	// front layer.
 	cornersVOI[0].x = in_cube_offset.x;		cornersVOI[0].y = in_cube_offset.y;		cornersVOI[0].z = in_cube_offset.z;
 	cornersVOI[1].x = cornersVOI[0].x + sizeCube;	cornersVOI[1].y = cornersVOI[0].y;				cornersVOI[1].z = cornersVOI[0].z;
 	cornersVOI[2].x = cornersVOI[0].x + sizeCube;	cornersVOI[2].y = cornersVOI[0].y + sizeCube;	cornersVOI[2].z = cornersVOI[0].z;
 	cornersVOI[3].x = cornersVOI[0].x;				cornersVOI[3].y = cornersVOI[0].y + sizeCube;	cornersVOI[3].z = cornersVOI[0].z;
 	// back layer.
 	cornersVOI[4].x = in_cube_offset.x;		cornersVOI[4].y = in_cube_offset.y;		cornersVOI[4].z = in_cube_offset.z + sizeCube;
 	cornersVOI[5].x = cornersVOI[4].x + sizeCube;	cornersVOI[5].y = cornersVOI[4].y;				cornersVOI[5].z = cornersVOI[4].z;
 	cornersVOI[6].x = cornersVOI[4].x + sizeCube;	cornersVOI[6].y = cornersVOI[4].y + sizeCube;	cornersVOI[6].z = cornersVOI[4].z;
 	cornersVOI[7].x = cornersVOI[4].x;				cornersVOI[7].y = cornersVOI[4].y + sizeCube;	cornersVOI[7].z = cornersVOI[4].z;
 	// project 3D VOI corners to depth image.
 	for (k = 0; k < 8; k++){
 		in_extrinsics->transform(cornersVOI[k], p3d);
 		in_intrinsics_d->project(p3d, projectedVOI[k]);
 	}
 
 	// draw VOI boundaries.
	// 화면 벗어나는 부분 check 해줘야 함!!
	// 화면 벗어나는 부분 check 해줘야 함!!
	// 화면 벗어나는 부분 check 해줘야 함!!
	// 화면 벗어나는 부분 check 해줘야 함!!
 	for (i = 0; i<2; i++){
 		// front and back layers.
 		k = i * 4;
 		out_img_VOI->sl_Set_Line((int)projectedVOI[k].x, (int)projectedVOI[k].y,
 			(int)projectedVOI[k + 1].x - (int)projectedVOI[k].x, (int)projectedVOI[k + 1].y - (int)projectedVOI[k].y, Kv_Rgb(0, 255, 0));
 		out_img_VOI->sl_Set_Line((int)projectedVOI[k + 1].x, (int)projectedVOI[k + 1].y,
 			(int)projectedVOI[k + 2].x - (int)projectedVOI[k + 1].x, (int)projectedVOI[k + 2].y - (int)projectedVOI[k + 1].y, Kv_Rgb(0, 255, 0));
 		out_img_VOI->sl_Set_Line((int)projectedVOI[k + 2].x, (int)projectedVOI[k + 2].y,
 			(int)projectedVOI[k + 3].x - (int)projectedVOI[k + 2].x, (int)projectedVOI[k + 3].y - (int)projectedVOI[k + 2].y, Kv_Rgb(0, 255, 0));
 		out_img_VOI->sl_Set_Line((int)projectedVOI[k + 3].x, (int)projectedVOI[k + 3].y,
 			(int)projectedVOI[k + 0].x - (int)projectedVOI[k + 3].x, (int)projectedVOI[k + 0].y - (int)projectedVOI[k + 3].y, Kv_Rgb(0, 255, 0));
 	}
 	// edges between front and back layers.
 	for (int k = 0; k<4; k++){
 		out_img_VOI->sl_Set_Line((int)projectedVOI[k].x, (int)projectedVOI[k].y,
 			(int)projectedVOI[k + 4].x - (int)projectedVOI[k].x, (int)projectedVOI[k + 4].y - (int)projectedVOI[k].y, Kv_Rgb(0, 255, 0));
 	}

	// set 3D VOI cube.
	x_min=in_cube_offset.x;		x_max=x_min+in_cube_size;
	y_min=in_cube_offset.y;		y_max=y_min+in_cube_size;	
	z_min=in_cube_offset.z;		z_max=z_min+in_cube_size;	

	// convert raw depth value to 8bit.
	float max_d, min_d;
	float offset=30.0f;
	max_d = -10000000;	min_d = 10000000;
	for (j = 0; j<hh; j++){
		for (i = 0; i<ww; i++){

			tidx=j*ww+i;
			td=pImgDepth[tidx];
			if (td > 0.0f){
				// calculate 3D point in the current (camera) frame coordinates.
				p2d.x=(float)i;	p2d.y=(float)j;
				in_intrinsics_d->back_project(p2d, td, p3d2);
				in_extrinsics->transform_inv(p3d2, p3d);

				if(p3d.x>=x_min && p3d.x<x_max && 
					p3d.y>=y_min && p3d.y<y_max && 
					p3d.z>=z_min && p3d.z<z_max)
				{
					pImgVOI[tidx]=(UCHAR)255;

					// find min/max depth values.
					if (td<min_d)	min_d = td;
					if (td>max_d)	max_d = td;
				}
				else{
					pImgVOI[tidx]=(UCHAR)254;
				}
			}
		}
	}

	// display disparity.
	for (i=0; i<ww*hh; i++){
		if(pImgVOI[i]==(UCHAR)255){
			for (k=0; k<3; k++)	pImgVOI[i + k*ww*hh] 
			= (unsigned char)(255.0f - (255.0f-offset)*(pImgDepth[i] - min_d) / (max_d - min_d));
		}	
	}

}

//********************************************************************************************
void LCKvYooji_Scanner_Display::dvoi_Draw_Volume_Of_Interest(
	CKvMatrixFloat *in_map_depth,
	CKvMatrixUchar *in_img_normal,
	CKvYooji_Intrinsics *in_intrinsics_d,
	CKvYooji_Extrinsics *in_extrinsics,
	float in_cube_size,
	CKvPoint3Df &in_cube_offset,
	CKvMatrixUcharRgb *out_img_VOI)
//********************************************************************************************
{
	int ww, hh, i, j, k;
	int tidx, tx1, tx2, ty1, ty2;
	float *pImgDepth = in_map_depth->mps(ww, hh)[0];
	UCHAR *pImgVOI, *pImgNormal;

	if (out_img_VOI->mw() != ww || out_img_VOI->mh() != hh){
		out_img_VOI->c_Create(hh, ww);
	}
	pImgVOI = out_img_VOI->vp();
	pImgNormal = in_img_normal->vp();

	// initialize VOI image.
	for (i = 0; i < 3 * ww*hh; i++)	{ pImgVOI[i] = (UCHAR)0; }

	// draw VOI on depth map.
	CKvPointf p2d;	CKvPoint3Df p3d, p3d2;
	CKvPoint3Df cornersVOI[8];	
	CKvPixel pixelsVOI[8];
	float sizeCube, td;
	float x_min, y_min, z_min, x_max, y_max, z_max;

	sizeCube = in_cube_size;
	// front layer.
	cornersVOI[0].x = in_cube_offset.x;		cornersVOI[0].y = in_cube_offset.y;		cornersVOI[0].z = in_cube_offset.z;
	cornersVOI[1].x = cornersVOI[0].x + sizeCube;	cornersVOI[1].y = cornersVOI[0].y;				cornersVOI[1].z = cornersVOI[0].z;
	cornersVOI[2].x = cornersVOI[0].x + sizeCube;	cornersVOI[2].y = cornersVOI[0].y + sizeCube;	cornersVOI[2].z = cornersVOI[0].z;
	cornersVOI[3].x = cornersVOI[0].x;				cornersVOI[3].y = cornersVOI[0].y + sizeCube;	cornersVOI[3].z = cornersVOI[0].z;
	// back layer.
	cornersVOI[4].x = in_cube_offset.x;		cornersVOI[4].y = in_cube_offset.y;		cornersVOI[4].z = in_cube_offset.z + sizeCube;
	cornersVOI[5].x = cornersVOI[4].x + sizeCube;	cornersVOI[5].y = cornersVOI[4].y;				cornersVOI[5].z = cornersVOI[4].z;
	cornersVOI[6].x = cornersVOI[4].x + sizeCube;	cornersVOI[6].y = cornersVOI[4].y + sizeCube;	cornersVOI[6].z = cornersVOI[4].z;
	cornersVOI[7].x = cornersVOI[4].x;				cornersVOI[7].y = cornersVOI[4].y + sizeCube;	cornersVOI[7].z = cornersVOI[4].z;
	// project 3D VOI corners to depth image.
	for (k = 0; k < 8; k++){
		in_extrinsics->transform(cornersVOI[k], p3d);
		in_intrinsics_d->project(p3d, p2d);
		
		pixelsVOI[k].x = (int)p2d.x;
		pixelsVOI[k].y = (int)p2d.y;

		//// clip line segment by image boundary.
// 		tx1 = (int)pixelsVOI[k].x;	tx2 = (int)pixelsVOI[k].x;
// 		ty1 = (int)pixelsVOI[k].x;	ty2 = (int)pixelsVOI[k].x;
// 
// 		if((int)pixelsVOI[k].x < 0 || (int)pixelsVOI[k].x > ww ||
// 			(int)pixelsVOI[k].y < 0 || (int)pixelsVOI[k].y > hh){
// 		}
	}

	// draw VOI boundaries.
	for (i = 0; i<2; i++){
		// front and back layers.
		k = i * 4;
		out_img_VOI->sl_Set_Line(pixelsVOI[k].x, pixelsVOI[k].y,
			pixelsVOI[k + 1].x - pixelsVOI[k].x, pixelsVOI[k + 1].y - pixelsVOI[k].y, Kv_Rgb(0, 255, 0));
		out_img_VOI->sl_Set_Line(pixelsVOI[k + 1].x, pixelsVOI[k + 1].y,
			pixelsVOI[k + 2].x - pixelsVOI[k + 1].x, pixelsVOI[k + 2].y - pixelsVOI[k + 1].y, Kv_Rgb(0, 255, 0));
		out_img_VOI->sl_Set_Line(pixelsVOI[k + 2].x, pixelsVOI[k + 2].y,
			pixelsVOI[k + 3].x - pixelsVOI[k + 2].x, pixelsVOI[k + 3].y - pixelsVOI[k + 2].y, Kv_Rgb(0, 255, 0));
		out_img_VOI->sl_Set_Line(pixelsVOI[k + 3].x, pixelsVOI[k + 3].y,
			pixelsVOI[k + 0].x - pixelsVOI[k + 3].x, pixelsVOI[k + 0].y - pixelsVOI[k + 3].y, Kv_Rgb(0, 255, 0));
	}
	// edges between front and back layers.
	for (int k = 0; k<4; k++){
		out_img_VOI->sl_Set_Line(pixelsVOI[k].x, pixelsVOI[k].y,
			pixelsVOI[k + 4].x - pixelsVOI[k].x, pixelsVOI[k + 4].y - pixelsVOI[k].y, Kv_Rgb(0, 255, 0));
	}

	// set 3D VOI cube.
	x_min = in_cube_offset.x;		x_max = x_min + in_cube_size;
	y_min = in_cube_offset.y;		y_max = y_min + in_cube_size;
	z_min = in_cube_offset.z;		z_max = z_min + in_cube_size;

	// convert raw depth value to 8bit.
	float max_d, min_d;
	float offset = 30.0f;
	max_d = -10000000;	min_d = 10000000;
	for (j = 0; j<hh; j++){
		for (i = 0; i<ww; i++){

			tidx = j*ww + i;
			td = pImgDepth[tidx];
			if (td > 0.0f){
				// calculate 3D point in the current (camera) frame coordinates.
				p2d.x = (float)i;	p2d.y = (float)j;
				in_intrinsics_d->back_project(p2d, td, p3d2);
				in_extrinsics->transform_inv(p3d2, p3d);

				if (p3d.x >= x_min && p3d.x<x_max &&
					p3d.y >= y_min && p3d.y<y_max &&
					p3d.z >= z_min && p3d.z<z_max)
				{
					pImgVOI[tidx] = (UCHAR)255;

					// find min/max depth values.
					if (td<min_d)	min_d = td;
					if (td>max_d)	max_d = td;
				}
				else{
					pImgVOI[tidx] = (UCHAR)254;
				}
			}
		}
	}

	// display disparity.
	for (i = 0; i<ww*hh; i++){
		if (pImgVOI[i] == (UCHAR)255){
			for (k = 0; k<3; k++)	pImgVOI[i + k*ww*hh] = pImgNormal[i];
		}
	}

	// display cube offset.
	out_img_VOI->scr_Set_Cross(pixelsVOI[0].x, pixelsVOI[0].y, 5, 5, Kv_Rgb(255, 0, 0));
	out_img_VOI->sbox_Set_Box(pixelsVOI[0].x - 8, pixelsVOI[0].y - 8, 16 , 16, Kv_Rgb(255, 0, 255));
	//cornersVOI[0]

}

//********************************************************************************************
void LCKvYooji_Scanner_Display::dmr_Display_Matching_Result(CKvScreen *in_screen,
	CKvMatrixUcharRgb *in_img_src,
	CKvMatrixUcharRgb *in_img_dst,
	vector<Point2f> &in_p2d_matched_src,
	vector<Point2f> &in_p2d_matched_dst)
//********************************************************************************************
{
	CKvMatrixUcharRgb img_disp;
	CKvRgb color_line, color_cross;
	uchar *p_img_disp;
	int ww, hh, tx, ty, dx, dy;
	ww=in_img_src->mw()+in_img_dst->mw();
	hh=max(in_img_src->mh(), in_img_dst->mh());
	p_img_disp=img_disp.c_Create(hh, ww, Kv_Rgb((uchar)0, (uchar)0, (uchar)0))[0];
	img_disp.sb_Set_Block(0, 0, in_img_src);
	img_disp.sb_Set_Block(in_img_src->mw(), 0, in_img_dst);

	color_line.s_Set(0, 0, 255);	color_cross.s_Set(0, 255, 0);
	for(int i=0; i<in_p2d_matched_src.size(); i++){
		Point2f &p2d_src = in_p2d_matched_src[i];
		Point2f &p2d_dst = in_p2d_matched_dst[i];

		tx=(int)p2d_src.x;	
		ty=(int)p2d_src.y;
		dx=(int)p2d_dst.x +in_img_src->mw() -tx;	
		dy=(int)p2d_dst.y -ty;

		img_disp.sl_Set_Line(tx, ty, dx, dy, color_line);
		img_disp.scr_Set_Cross(tx, ty, 2, 2, color_cross);
		img_disp.scr_Set_Cross(tx+dx, ty+dy, 2, 2, color_cross);
	}

	in_screen->s_d_Display(&img_disp);
}

//********************************************************************************************
void LCKvYooji_Scanner_Display::dmr_Display_Matching_Result(CKvScreen *in_screen,
	CKvMatrixUcharRgb *in_img_src,
	CKvMatrixUcharRgb *in_img_dst,
	int in_num_matched,
	float *in_matched_src,
	float *in_matched_dst)
//********************************************************************************************
{
	CKvMatrixUcharRgb img_disp;
	CKvRgb color_line, color_cross;
	uchar *p_img_disp;
	int ww, hh, tx, ty, dx, dy;
	ww=in_img_src->mw()+in_img_dst->mw();
	hh=max(in_img_src->mh(), in_img_dst->mh());
	p_img_disp=img_disp.c_Create(hh, ww, Kv_Rgb((uchar)0, (uchar)0, (uchar)0))[0];
	img_disp.sb_Set_Block(0, 0, in_img_src);
	img_disp.sb_Set_Block(in_img_src->mw(), 0, in_img_dst);

	color_line.s_Set(0, 0, 255);	color_cross.s_Set(0, 255, 0);
	for(int i=0; i<in_num_matched; i++){
		tx=(int)in_matched_src[2*i];	
		ty=(int)in_matched_src[2*i+1];
		dx=(int)in_matched_dst[2*i] +in_img_src->mw() -tx;	
		dy=(int)in_matched_dst[2*i+1] -ty;

		img_disp.sl_Set_Line(tx, ty, dx, dy, color_line);
		img_disp.scr_Set_Cross(tx, ty, 2, 2, color_cross);
		img_disp.scr_Set_Cross(tx+dx, ty+dy, 2, 2, color_cross);
	}

	in_screen->s_d_Display(&img_disp);
}


//********************************************************************************************
void LCKvYooji_Scanner_Display::dtf_Display_Tracked_Flow(CKvScreen *in_screen,
	CKvMatrixUcharRgb *in_img_t1,
	CKvSet2d_of_Pointf *in_set_of_flows,
	int in_block_sz)
//********************************************************************************************
{
// 	CKvMatrixUcharRgb img_disp;
// 	CKvRgb color_line, color_cross, color_cross2;
// 	uchar *p_img_disp;
// 	int ww, hh, tx, ty, dx, dy;
// 	ww=in_img_t1->mw();
// 	hh=in_img_t1->mh();
// 	p_img_disp=img_disp.c_Create(hh, ww, Kv_Rgb((uchar)0, (uchar)0, (uchar)0))[0];
// 	img_disp.cp_Copy(in_img_t1);
// 
// 	color_line.s_Set(0, 255, 0);	color_cross.s_Set(255, 255, 255); color_cross2.s_Set(255, 0, 0);
// 
// 	int num_blocks,block_dim_x,block_dim_y;
// 	int num_valid,num_level,num_th;
// 	bool flag_valid=true;
// 	//////////////////////////////////////////////////////////////////////////
// 	float scale = 1.5f;
// 	//////////////////////////////////////////////////////////////////////////
// 	float th_dist_ICP,mu,max_w;
// 
// 	// initialize depth flows.
// 	//CKvPointf *p_flow = out_depth_flow->mps(block_dim_x,block_dim_y)[0];
// 	CKvPointf *p_flow = in_set_of_flows->mps(block_dim_x,block_dim_y)[0];
// 	num_blocks = block_dim_x*block_dim_y;
// 	
// 	//printf("num_blocks: %d\n", num_blocks);
// 
// 	for(int n=0; n<num_blocks; n++){
// 
// 		//tx = (n % block_dim_x);
// 		//ty = (n / block_dim_x);
// 
// 		tx = (n % block_dim_x)*in_block_sz + in_block_sz/2;
// 		ty = (n / block_dim_x)*in_block_sz + in_block_sz/2;
// 
// 		dx=(int)(scale*p_flow[n].x);
// 		dy=(int)(scale*p_flow[n].y);
// 
// 		
// 		if(p_flow[n].x == 0.0f && p_flow[n].y == 0.0f) continue;
// 
// 		if(abs(p_flow[n].x) > 12.0f || abs(p_flow[n].y) > 12.0f) continue;
// 		
// 		//img_disp.scr_Set_Cross(tx,ty,1,1,color_cross);
// 
// 		img_disp.sl_Set_Line(tx,ty,dx,dy,color_line);
// 				
// 		img_disp.scr_Set_Cross(tx+dx, ty+dy, 1, 1, color_cross2);
// 	}
// 
// 	in_screen->s_d_Display(&img_disp);

	CKvMatrixUcharRgb img_disp;
	CKvRgb color_line,color_cross;
	uchar *p_img_disp;
	int ww,hh,tx,ty,dx,dy;
	ww=in_img_t1->mw();
	hh=in_img_t1->mh();
	p_img_disp=img_disp.c_Create(hh,ww,Kv_Rgb((uchar)0,(uchar)0,(uchar)0))[0];
	img_disp.cp_Copy(in_img_t1);

	color_line.s_Set(0,255,0);	color_cross.s_Set(0,0,255);

	int num_blocks,block_dim_x,block_dim_y;
	int num_valid,num_level,num_th;
	bool flag_valid=true;
	//////////////////////////////////////////////////////////////////////////
	float scale = 1.0f;
	//////////////////////////////////////////////////////////////////////////
	float th_dist_ICP,mu,max_w;

	// initialize depth flows.
	//CKvPointf *p_flow = out_depth_flow->mps(block_dim_x,block_dim_y)[0];
	CKvPointf *p_flow = in_set_of_flows->mps(block_dim_x,block_dim_y)[0];
	num_blocks = block_dim_x*block_dim_y;

	//printf("num_blocks: %d\n", num_blocks);

	for(int n=0; n<num_blocks; n++){

		if(abs(p_flow[n].x) <= 0.0f && abs(p_flow[n].y) <= 0.0f) continue;
		if(abs(p_flow[n].x) > 50.0f || abs(p_flow[n].y) > 50.0f) continue;

		tx = (n % block_dim_x)*in_block_sz + in_block_sz/2;
		ty = (n / block_dim_x)*in_block_sz + in_block_sz/2;

		//if(tx%7 != 0 || ty%7 != 0) continue;

		dx=(int)(scale*p_flow[n].x);
		dy=(int)(scale*p_flow[n].y);

		img_disp.scr_Set_Cross(tx,ty,1,1,color_cross);

		//if(p_flow[n].x == 0.0f && p_flow[n].y == 0.0f) continue;
		img_disp.sl_Set_Line(tx,ty,dx,dy,color_line);

		//img_disp.scr_Set_Cross(tx+dx, ty+dy, 2, 2, color_cross);
	}

	in_screen->s_d_Display(&img_disp);
}

//********************************************************************************************
void LCKvYooji_Scanner_Display::dtfsc_Display_Tracked_Flow_Pseudo_Color(CKvScreen *in_screen,
	CKvMatrixUcharRgb *in_img_t1,
	CKvSet2d_of_Pointf *in_set_of_flows,
	int in_block_sz)
//********************************************************************************************
{

	CKvMatrixUcharRgb img_disp;
	CKvPointf normal;	float mag;
	CKvRgb color_line,color_cross;
	uchar *p_img_disp;
	int ww,hh,tx,ty,dx,dy;
	ww=in_img_t1->mw();
	hh=in_img_t1->mh();
	p_img_disp=img_disp.c_Create(hh,ww,Kv_Rgb((uchar)0,(uchar)0,(uchar)0))[0];
	img_disp.cp_Copy(in_img_t1);

	color_line.s_Set(0,255,0);	color_cross.s_Set(0,0,255);

	int num_blocks,block_dim_x,block_dim_y;
	int num_valid,num_level,num_th;
	bool flag_valid=true;
	//////////////////////////////////////////////////////////////////////////
	float scale = 0.5f*in_block_sz;
	float max_inten = 20.0f;
	float mag_ratio_for_pseudo = 255.0f/max_inten;
	//////////////////////////////////////////////////////////////////////////
	float th_dist_ICP,mu,max_w;

	// initialize depth flows.
	//CKvPointf *p_flow = out_depth_flow->mps(block_dim_x,block_dim_y)[0];
	CKvPointf *p_flow = in_set_of_flows->mps(block_dim_x,block_dim_y)[0];
	num_blocks = block_dim_x*block_dim_y;

	//printf("num_blocks: %d\n", num_blocks);

	for(int n=0; n<num_blocks; n++){

		mag = p_flow[n].m_Magnitude();
		normal = p_flow[n]/mag;

		if(mag < 1.0f) continue;

		tx = (n % block_dim_x)*in_block_sz + in_block_sz/2;
		ty = (n / block_dim_x)*in_block_sz + in_block_sz/2;

		//if(tx%7 != 0 || ty%7 != 0) continue;

		dx=(int)(scale*normal.x);
		dy=(int)(scale*normal.y);


		//if(p_flow[n].x == 0.0f && p_flow[n].y == 0.0f) continue;
		// set line in direction of gradient vector with pseudo coloring.
		float mag_pseudo = min(max_inten, mag)*mag_ratio_for_pseudo;
		cipc_Convert_Intensity_to_Pseudo_Color(mag_pseudo, color_line);
//  		img_disp.sl_Set_Line(tx,ty,dx,dy,color_line);
//  
//  		// set cross at the end of gradient vector.
//  		img_disp.scr_Set_Cross(tx + dx,ty + dy,1,1,color_line);

		img_disp.sa_Set_Arrow(tx,ty,dx,dy,color_line);

		//img_disp.scr_Set_Cross(tx+dx, ty+dy, 2, 2, color_cross);
	}

	in_screen->s_d_Display(&img_disp);
}

//********************************************************************************************
void LCKvYooji_Scanner_Display::dtf_Display_Tracked_Flow_HSV(
	CKvScreen *in_screen,
	CKvSet2d_of_Pointf *in_set_of_flows,
	int in_block_sz)
//********************************************************************************************
{
	LCKvUtility_for_Color_Conversion aa_cc;
	CKvIHS ihs_color;
	CKvRgbF rgbf_color;

	CKvMatrixUcharRgb img_disp;
	uchar *p_img_disp;
	int ww, hh, tx, ty, dx, dy;
	ww=in_set_of_flows->mw();
	hh=in_set_of_flows->mh();
	p_img_disp=img_disp.c_Create(hh, ww, Kv_Rgb((uchar)0, (uchar)0, (uchar)0))[0];

	int num_blocks,block_dim_x,block_dim_y;
	int num_valid,num_level,num_th;
	bool flag_valid=true;

	float th_dist_ICP,mu,max_w;

	// initialize depth flows.
	//CKvPointf *p_flow = out_depth_flow->mps(block_dim_x,block_dim_y)[0];
	CKvPointf *p_flow = in_set_of_flows->mps(block_dim_x,block_dim_y)[0];
	num_blocks = block_dim_x*block_dim_y;
	
	//printf("num_blocks: %d\n", num_blocks);
	float mag_max = 4.0f;

	for(int n=0; n<num_blocks; n++){

		tx = n % block_dim_x;
		ty = n / block_dim_x;

		// Convert motion vector to HSV color value.
		float rho, theta; 
		p_flow[n].gp_Get_Polar_coordinates(rho, theta);

		//printf("polar: %f %f\n", rho, theta);
		rho = min(rho, mag_max)/mag_max;

		ihs_color.s_Set(1.0f, theta, rho);	// case 1.
		//ihs_color.s_Set(rho, theta, 1.0f);	// case 2.
		// Convert HSI color to RGB color.
		aa_cc.ir_IHS_to_RgbF(ihs_color, rgbf_color);


		// set flow color on image.
		p_img_disp[ty*block_dim_x + tx] = (uchar)(clip(0.0f, 255.0f, 255.0f*rgbf_color.r));
		p_img_disp[ty*block_dim_x + tx + num_blocks] = (uchar)(clip(0.0f, 255.0f, 255.0f*rgbf_color.g));
		p_img_disp[ty*block_dim_x + tx + 2*num_blocks] = (uchar)(clip(0.0f, 255.0f, 255.0f*rgbf_color.b));
		//printf("rgbf: %f %f %f\n", rgbf_color.r, rgbf_color.g, rgbf_color.b);

	}

	in_screen->s_d_Display(&img_disp);
}

//********************************************************************************************
void LCKvYooji_Scanner_Display::dtf_Draw_Tracked_Flow(
	CKvMatrixUcharRgb *out_img_flow,
	CKvMatrixUcharRgb *in_img_t1,
	CKvSet2d_of_Pointf *in_set_of_flows,
	int in_block_sz)
//********************************************************************************************
{
	CKvRgb color_line, color_cross;
	uchar *p_img_disp;
	int ww, hh, tx, ty, dx, dy;
	ww=in_img_t1->mw();
	hh=in_img_t1->mh();
	p_img_disp=out_img_flow->c_Create(hh, ww, Kv_Rgb((uchar)0, (uchar)0, (uchar)0))[0];
	out_img_flow->cp_Copy(in_img_t1);

	color_line.s_Set(0, 255, 0);	color_cross.s_Set(0, 0, 255);

	int num_blocks,block_dim_x,block_dim_y;
	int num_valid,num_level,num_th;
	bool flag_valid=true;
	//////////////////////////////////////////////////////////////////////////
	float scale = 1.5f;
	//////////////////////////////////////////////////////////////////////////
	float th_dist_ICP,mu,max_w;

	// initialize depth flows.
	//CKvPointf *p_flow = out_depth_flow->mps(block_dim_x,block_dim_y)[0];
	CKvPointf *p_flow = in_set_of_flows->mps(block_dim_x,block_dim_y)[0];
	num_blocks = block_dim_x*block_dim_y;
	
	//printf("num_blocks: %d\n", num_blocks);

	for(int n=0; n<num_blocks; n++){

		tx = (n % block_dim_x)*in_block_sz + in_block_sz/2;
		ty = (n / block_dim_x)*in_block_sz + in_block_sz/2;

		dx=(int)(scale*p_flow[n].x);
		dy=(int)(scale*p_flow[n].y);

		out_img_flow->scr_Set_Cross(tx, ty, 1, 1, color_cross);

		//if(p_flow[n].x == 0.0f && p_flow[n].y == 0.0f) continue;
		if(abs(p_flow[n].x) > 12.0f || abs(p_flow[n].y) > 12.0f) continue;
		//if(abs(p_flow[n].x) < 0.02f && abs(p_flow[n].y) < 0.02f) continue;

		out_img_flow->sl_Set_Line(tx,ty,dx,dy,color_line);

		//img_disp.scr_Set_Cross(tx+dx, ty+dy, 2, 2, color_cross);
	}
}

//********************************************************************************************
void LCKvYooji_Scanner_Display::dtf_Draw_Tracked_Flow_HSV(
	CKvMatrixUcharRgb *out_img_flow,
	CKvSet2d_of_Pointf *in_set_of_flows,
	int in_block_sz)
//********************************************************************************************
{
	LCKvUtility_for_Color_Conversion aa_cc;
	CKvIHS ihs_color; CKvIHS_sdkim ihs_sdkim;
	CKvRgbF rgbf_color;

	CKvMatrixUcharRgb img_disp;
	uchar *p_img_disp;
	int ww, hh, tx, ty, dx, dy;
	ww=in_set_of_flows->mw();
	hh=in_set_of_flows->mh();
	p_img_disp = out_img_flow->c_Create(hh, ww, Kv_Rgb((uchar)0, (uchar)0, (uchar)0))[0];

	int num_blocks,block_dim_x,block_dim_y;
	int num_valid,num_level,num_th;
	bool flag_valid=true;

	float th_dist_ICP,mu,max_w;
	float r, g, b;

	// initialize depth flows.
	//CKvPointf *p_flow = out_depth_flow->mps(block_dim_x,block_dim_y)[0];
	CKvPointf *p_flow = in_set_of_flows->mps(block_dim_x,block_dim_y)[0];
	num_blocks = block_dim_x*block_dim_y;
	
	//printf("num_blocks: %d\n", num_blocks);
	float mag_max = 8.0f;

	for(int n=0; n<num_blocks; n++){

		tx = n % block_dim_x;
		ty = n / block_dim_x;

		// Convert motion vector to HSV color value.
		float rho, theta; 
		p_flow[n].gp_Get_Polar_coordinates(rho,theta);

		// Normalize magnitude and angle of motion vector.
		rho = min(rho,mag_max)/mag_max;
		theta /= 2*PI;

		ihs_sdkim.s_Set(1.0f,theta,rho);	//ihs_color.s_Set(1.0f,theta,rho);			
		//// Convert HSI color to RGB color.
		aa_cc.ir_IHS_sdkim_to_RgbF(ihs_sdkim,rgbf_color);	//aa_cc.ir_IHS_to_RgbF(ihs_color,rgbf_color);

		// scaling.
		r = 255*rgbf_color.r;
		g = 255*rgbf_color.g;
		b = 255*rgbf_color.b;
		//////////////////////////////////////////////////////////////////////////

		// set flow color on image.
		p_img_disp[ty*block_dim_x + tx] = (uchar)(clip(0.0f,255.0f,r));
		p_img_disp[ty*block_dim_x + tx + num_blocks] = (uchar)(clip(0.0f,255.0f,g));
		p_img_disp[ty*block_dim_x + tx + 2*num_blocks] = (uchar)(clip(0.0f,255.0f,b));
		//printf("rgbf: %f %f %f\n", rgbf_color.r, rgbf_color.g, rgbf_color.b);

	}

}

//*********************************************************************************************************
void LCKvYooji_Scanner_Display::dtf_Draw_Flow_Color_Coding_HSV(
	CKvMatrixUcharRgb *out_img_flow,
	int in_block_sz)
//*********************************************************************************************************
{
	LCKvUtility_for_Color_Conversion aa_cc;
	CKvIHS ihs_color;
	CKvIHS_sdkim ihs_sdkim;
	CKvRgbF rgbf_color;	

	uchar *p_img_disp;

	CKvPointf tp2d, center;
	float rho, theta, mag_max;
	float H, S, I, X, C, m, r, g, b;

	p_img_disp = out_img_flow->c_Create(in_block_sz, in_block_sz)[0];

	mag_max = center.x = center.y = (in_block_sz-1)/2.0f;	

	for(int j=0; j<in_block_sz; j++){
		for(int i=0; i<in_block_sz; i++){

			tp2d.x = i - center.x;
			tp2d.y = j - center.y;

			// Convert motion vector to HSV color value.
			tp2d.gp_Get_Polar_coordinates(rho, theta);

			// Normalize magnitude and angle of motion vector.
			rho = min(rho, mag_max)/mag_max;
			theta /= 2*PI;

			ihs_sdkim.s_Set(1.0f,theta,rho);	//ihs_color.s_Set(1.0f,theta,rho);			
			//// Convert HSI color to RGB color.
			aa_cc.ir_IHS_sdkim_to_RgbF(ihs_sdkim,rgbf_color);	//aa_cc.ir_IHS_to_RgbF(ihs_color,rgbf_color);

			// scaling.
			r = 255*rgbf_color.r;
			g = 255*rgbf_color.g;
			b = 255*rgbf_color.b;
			//////////////////////////////////////////////////////////////////////////

			// set flow color on image.
			p_img_disp[j*in_block_sz + i] = (uchar)(clip(0.0f,255.0f,r));
			p_img_disp[j*in_block_sz + i + SQUARE(in_block_sz)] = (uchar)(clip(0.0f,255.0f,g));
			p_img_disp[j*in_block_sz + i + 2*SQUARE(in_block_sz)] = (uchar)(clip(0.0f,255.0f,b));
		}
	}
}

//********************************************************************************************
void LCKvYooji_Scanner_Display::dnm_Display_Normal_Map(
	CKvScreen *in_screen,
	CKvSet2d_of_Point3Df *in_map_normal)
//********************************************************************************************
{
	CKvMatrixUchar img_disp;
	CKvPoint3Df light;
	CKvPoint3Df *p_normal; uchar *p_img_disp;

	int ww, hh, tx, ty, dx, dy;

	p_normal = in_map_normal->mps(ww, hh)[0];
	p_img_disp = img_disp.c_Create(hh, ww, uchar(0))[0];

	// default light vector is (0,0,-1).
	light.s_Set(1, -2, -3); light.n_Normalize();

	for(int len=0; len<ww*hh; len++){
		
		if(p_normal[len].x == -100.0f) continue;

		float surf_angle = p_normal[len].ps_Product_Scalar(light);	
		if(surf_angle>0.0f){
			// set normal image.
			p_img_disp[len]=(UCHAR)(clip(0.0f,255.0f,(0.8f * surf_angle + 0.2f) * 255.0f));
		}

	}

	in_screen->s_d_Display(&img_disp);
}

//*********************************************************************************************************
// CAUTION: INPUT INTENSITY RANGE IS [0, 255].
void LCKvYooji_Scanner_Display::cipc_Convert_Intensity_to_Pseudo_Color(
	float in_mag_inten,
	CKvRgb &out_pseudo_color)
//*********************************************************************************************************
{
	static uchar pseudo_r, pseudo_g, pseudo_b;

	if(in_mag_inten>=0 && in_mag_inten<=63){ pseudo_r=0;		pseudo_g=(int)(255.0f/63.0f*(float)in_mag_inten);		pseudo_b=255; } 
	else if(in_mag_inten>=63 && in_mag_inten<=127)	{ pseudo_r=0;		pseudo_g=255;		pseudo_b=255-(int)(255.0f/(127.0f-63.0f)*((float)in_mag_inten-63.0f)); } 
	else if(in_mag_inten>=127 && in_mag_inten<=191)	{ pseudo_r=(int)(255.0f/(191.0f-127.0f)*((float)in_mag_inten-127.0f));		pseudo_g=255;		pseudo_b=0; } 
	else{ pseudo_r=255;		pseudo_g=255-(int)(255.0f/(255.0f-191.0f)*((float)in_mag_inten-191.0f));		pseudo_b=0; }

	out_pseudo_color.r = pseudo_r;
	out_pseudo_color.g = pseudo_g;
	out_pseudo_color.b = pseudo_b;
}


//*********************************************************************************************************
void LCKvYooji_Scanner_Display::z_mmpt_Make_Mesh_Points_with_Texture(
	CKvMatrixFloat *in_depth_map, 
	CKvMatrixUcharRgb *in_color_image,
	CKvYooji_Intrinsics *in_intrinsics,	
	CKvDepot_of_Point3D *out_depot_3d_points,
	CKvDepot_of_RgbaF *out_depot_colors,
	CKvMesh_of_Triangle *out_mesh_element)
//*********************************************************************************************************
{
	const float max_depth_diff = 0.01f;

	CKvSet_of_VectorInt set_vi; 
	CKvVectorInt v_index, v_index_color, v_index_size;
	CKvMatrixInt map_index_new;
	CKvSet2d_of_Point3Df set_p3d;
	CKvPoint3Df *p_set_p3d;

	CKvPixel pix;

	float *p_depth;
	unsigned char *p_r, *p_g, *p_b;
	int *p_map_index_new;

	int *v, *v_color;
	float pt[3],rgb[4], rgb_back[4];
	unsigned char rgb_uchar[3];

	int i, j, k, m;
	int k0, k1, k2, k3, kk0, kk1, kk2, kk3;
	int num_reduced, itmp, offset[3];
	int numPoint, numTriangle, numTriangle_valid=0;

	CKvString *pstr;
	int ww, hh, widthStep;
	int x, y;
	float depth_meter, scale;
	CKvVectorFloat vec_xx,vec_yy; float *p_x,*p_y;

	rgb[0] = rgb[1] = rgb[2] = 0.0f;		rgb[3] = 1.0f;
	rgb_back[0] = rgb_back[1] = rgb_back[2] = 0.5f;		rgb_back[3] = 1.0f;

	in_color_image->ms(ww, hh);
	
	p_depth = in_depth_map->vp();
	p_r = in_color_image->vp(p_g, p_b);
	p_map_index_new = map_index_new.c_Create(hh, ww, 0)[0];

	p_x = vec_xx.c_Create(ww*hh);
	p_y = vec_yy.c_Create(ww*hh);

	// offset setting.
	offset[0]=offset[1]=offset[2]=0;
	numPoint = ww*hh;
	numTriangle = 2*(ww-1)*(hh-1);

	// make depot_of_point3D.	
	k0 = num_reduced = 0;
	p_set_p3d = set_p3d.c_Create(hh, ww)[0];
	for(j = 0; j<hh; j++){
		for(i = 0; i<ww; i++, k0++){

			CKvPoint3Df *p3d = &p_set_p3d[k0];
			
			// check valid depth.
			if(j==hh-1 || i==ww-1 || p_depth[k0] <= 0.0f) 
			{ 
				p3d->x = p3d->y = p3d->z = 0.0f;
				p_depth[k0] = 0.0f;

				// make new index map.
				p_map_index_new[k0] = -1;

				num_reduced++;
				//p_x[i] = p_y[i] = 0.0f; 
			}
			else
			{
				pix.x = i;	
				pix.y = j;

				in_intrinsics->back_project(pix, p_depth[k0], *p3d);

				// make new index map.
				p_map_index_new[k0] = k0 - num_reduced;
			}		
		}
	}
	// make point and color depot.	
	out_depot_3d_points->in_Initialize(numPoint - num_reduced);
	out_depot_colors->in_Initialize(numPoint - num_reduced);

	// append 3D points and colors and check number of valid triangles.
	k0 = numTriangle_valid = 0;
	for(j=0; j<hh; j++){
		for(i=0; i<ww; i++, k0++){
			
			if(p_map_index_new[k0] < 0){
			//if(j==hh-1 || i==ww-1 || p_depth[k0] <= 0.0f){	
			//	num_reduced++;
				continue;
			}

			pt[0] = p_set_p3d[k0].x;		pt[1] = p_set_p3d[k0].y;		pt[2] = p_set_p3d[k0].z;

			rgb[0] = (float)p_r[k0]/255.0f;		
			rgb[1] = (float)p_g[k0]/255.0f;		
			rgb[2] = (float)p_b[k0]/255.0f;		

			// count valid triangle.
			k1 = k0 + 1; k2 = k0 + ww; k3 = k2 + 1;

			if(p_map_index_new[k1] >= 0 && p_map_index_new[k2] >= 0){

				if(abs(p_depth[k0] - p_depth[k1]) < max_depth_diff
					&& abs(p_depth[k0] - p_depth[k2]) < max_depth_diff
					&& abs(p_depth[k1] - p_depth[k2]) < max_depth_diff){
					numTriangle_valid++;
				}

				if(p_map_index_new[k3] >= 0
					&& abs(p_depth[k3] - p_depth[k1]) < max_depth_diff
					&& abs(p_depth[k3] - p_depth[k2]) < max_depth_diff
					&& abs(p_depth[k1] - p_depth[k2]) < max_depth_diff){
					numTriangle_valid++;
				}

			}			

			out_depot_3d_points->ap_Append(false, pt, itmp);
			out_depot_colors->ap_Append(false, rgb, itmp);
		}
	}

	// make point indices.
	v=v_index.c_Create(3*numTriangle_valid);

	// make mesh triangles.
	k0 = m = num_reduced = 0; 
	for(j=0; j<hh; j++){
		for(i=0; i<ww; i++, k0++){

			if(p_map_index_new[k0] < 0){
			//if(j==hh-1 || i==ww-1 || p_set_p3d[k0].z <= 0.0f){
				//num_reduced++;
				continue;
			}

			k1 = k0 + 1; k2 = k0 + ww; k3 = k2 + 1;

			// for reduced point index.
			kk0 = p_map_index_new[k0];	
			kk1 = p_map_index_new[k1];	
			kk2 = p_map_index_new[k2];	
			kk3 = p_map_index_new[k3];				
			
			if(kk1 >= 0 && kk2 >= 0){
				// first triangle.
				if(abs(p_depth[k0] - p_depth[k1]) < max_depth_diff
					&& abs(p_depth[k0] - p_depth[k2]) < max_depth_diff
					&& abs(p_depth[k1] - p_depth[k2]) < max_depth_diff){
					v[m++]=kk0;		v[m++]=kk1;		v[m++]=kk2;		
				}	

				// second triangle.
				if(kk3 >= 0
					&& abs(p_depth[k3] - p_depth[k1]) < max_depth_diff
					&& abs(p_depth[k3] - p_depth[k2]) < max_depth_diff
					&& abs(p_depth[k1] - p_depth[k2]) < max_depth_diff){
					v[m++]=kk2;		v[m++]=kk1;		v[m++]=kk3;
				}	

			}
		}
	}

	// make point indices.
	// triangle mesh.
	out_mesh_element->in_Initialize(1);
	out_mesh_element->u_me_Make_Element(&v_index, &v_index, &v_index, &set_vi);
	out_mesh_element->ap_Append(true, &set_vi, offset, itmp);
}