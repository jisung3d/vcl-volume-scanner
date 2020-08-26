/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_volume_integrator.cpp 
/////////////////////////////////////////////////////////////////////////////////////////////

#include "../../../stdafx.h"
#include "../../_yooji_2016_object_scanning.h"


/////////////////////////////////////////////////////////////////////////////////////////////
// LCKvYooji_Volume_Integrator 
/////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
LCKvYooji_Volume_Integrator::LCKvYooji_Volume_Integrator()
//********************************************************************************************
{
	zz_classname = "LCKvYooji_Volume_Integrator";
	zz_mat_4x4.ci_Create_Identity_matrix(4);
}

//********************************************************************************************
LCKvYooji_Volume_Integrator::~LCKvYooji_Volume_Integrator()
//********************************************************************************************
{
}

// ===================================== global functions ==================================== //
// ===================================== global functions ==================================== //
// ===================================== global functions ==================================== //

//********************************************************************************************
void LCKvYooji_Volume_Integrator::cdtoc_Convert_Depth_to_TSDF_on_Cube(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Scanner_Parameter *in_setting,
	CKvYooji_Tracking_State *in_state,	
	CKvYooji_Cube_TSDF_Float *io_cube,
	bool in_flag_color)
//********************************************************************************************
{
	float th_dist_ICP, mu, max_w;
	int sz_sub_cube, lev_pyram;

// 	in_state->ph3d_Pointer_of_3D_Homography_Global_to_Camera()->e_Export(&zz_mat_4x4);
// 	zz_hmat_glob_to_cam.st_Set_from_Transformation(&zz_mat_4x4);	
// 	in_setting->gpicp_Get_Parameters_for_ICP_algorithm(th_dist_ICP, mu, max_w);
// 	sz_sub_cube = io_cube->elsc_Edge_Length_of_Sub_Cube();

	zz_transform_glob_to_cam.copy(in_state->p_extrinsics_glob_to_cam());

	in_setting->gpicp_Get_Parameters_for_ICP_algorithm(lev_pyram, th_dist_ICP, mu, max_w);
	sz_sub_cube = io_cube->elsc_Edge_Length_of_Sub_Cube();

	fvsc_Find_Visible_Sub_Cubes_using_Casting_Pixel_Rays(in_view, 
		&zz_transform_glob_to_cam,
		sz_sub_cube,
		mu,
		io_cube);

	ctvovsc_Compute_TSDF_of_Voxels_on_Visible_Sub_Cubes(in_view, 
		&zz_transform_glob_to_cam, 
		sz_sub_cube,
		mu, max_w,
		io_cube,
		in_flag_color);
}


//********************************************************************************************
void LCKvYooji_Volume_Integrator::cdtoc_Convert_Depth_to_TSDF_on_Cube_RGB_NEW(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Scanner_Parameter *in_setting,
	CKvYooji_Tracking_State *in_state,	
	CKvYooji_Cube_TSDF_Float *io_cube,
	bool in_flag_color)
//********************************************************************************************
{
	float th_dist_ICP, mu, max_w;
	int sz_sub_cube, lev_pyram;

// 	in_state->ph3d_Pointer_of_3D_Homography_Global_to_Camera()->e_Export(&zz_mat_4x4);
// 	zz_hmat_glob_to_cam.st_Set_from_Transformation(&zz_mat_4x4);	
// 	in_setting->gpicp_Get_Parameters_for_ICP_algorithm(th_dist_ICP, mu, max_w);
// 	sz_sub_cube = io_cube->elsc_Edge_Length_of_Sub_Cube();

	zz_transform_glob_to_cam.copy(in_state->p_extrinsics_glob_to_cam_RGB());

	in_setting->gpicp_Get_Parameters_for_ICP_algorithm(lev_pyram, th_dist_ICP, mu, max_w);
	sz_sub_cube = io_cube->elsc_Edge_Length_of_Sub_Cube();

	fvsc_Find_Visible_Sub_Cubes_using_Casting_Pixel_Rays_RGB_NEW(in_view, 
		&zz_transform_glob_to_cam,
		sz_sub_cube,
		mu,
		io_cube);

	ctvovsc_Compute_TSDF_of_Voxels_on_Visible_Sub_Cubes_RGB_NEW(in_view, 
		&zz_transform_glob_to_cam, 
		sz_sub_cube,
		mu, max_w,
		io_cube,
		in_flag_color);
}


//********************************************************************************************
int LCKvYooji_Volume_Integrator::csc_Carve_Sub_Cubes_using_Casting_Pixel_Rays(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Scanner_Parameter *in_setting,
	CKvYooji_Tracking_State *in_state,
	CKvYooji_Cube_TSDF_Float *io_cube)
//********************************************************************************************
{
	CKvPoint3Df p3d, p3d_block_e, tp3d, tvox;
	CKvPoint3Df dvec;
	CKvPixel tpix;

	float *p_depth, *p_weight;
	char *p_flag_sc;

	float td;

	int ww, hh, cnt, tidx;
	int ww_sc, hh_sc, dd_sc;
	int x, y, n, x_sc, y_sc, z_sc;

	float th_dist_ICP, mu, max_w;
	int sz_sub_cube, lev_pyram;

	// get parameters.
	in_setting->gpicp_Get_Parameters_for_ICP_algorithm(lev_pyram, th_dist_ICP, mu, max_w);
	sz_sub_cube = io_cube->elsc_Edge_Length_of_Sub_Cube();	

	// get pointers.
	p_depth = in_view->p_map_depth_raw()->mps(ww, hh)[0];
	p_weight = in_state->p_map_weight_rendered()->vp();
	p_flag_sc = io_cube->pvfsc_Pointer_of_Volume_Flag_of_Sub_Cube()->tps(ww_sc, hh_sc, dd_sc)[0][0];	
		
	//zz_intrin.spm_Set_from_P_Matrix(in_view->ppd_Pointer_of_Pmatrix_Depth());
	//in_state->ph3d_Pointer_of_3D_Homography_Global_to_Camera()->e_Export(&zz_mat_4x4);
	//zz_transform_glob_to_cam.st_Set_from_Transformation(&zz_mat_4x4);
	zz_intrin.copy(in_state->p_intrinsics_depth());
	zz_transform_glob_to_cam.copy(in_state->p_extrinsics_glob_to_cam());

	// cast rays for setting sub-cube validities.
	cnt = 0;
	for(y=0; y<hh; y++){	for(x=0; x<ww; x++){

		// back-project depth.
		td = p_depth[y*ww + x];
		if (td > 0.0f && p_weight[y*ww + x]>10.0f){
			tpix.x = x;	tpix.y = y;
			// compute direction vector of the pixel ray.
			zz_intrin.back_project(tpix, td, tp3d);		zz_transform_glob_to_cam.transform_inv(tp3d, p3d);
			zz_intrin.back_project(tpix, 2.0f, tp3d);	zz_transform_glob_to_cam.transform_inv(tp3d, p3d_block_e);

			dvec.x=p3d_block_e.x-p3d.x;	dvec.y=p3d_block_e.y-p3d.y;	dvec.z=p3d_block_e.z-p3d.z;
			dvec.n_Normalize();

			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			// carves sub-cubes in front of depth point.
			for(n=-5; ; n--){
				tp3d.x=p3d.x +(float)n*mu*dvec.x;
				tp3d.y=p3d.y +(float)n*mu*dvec.y;
				tp3d.z=p3d.z +(float)n*mu*dvec.z;

				if(io_cube->gpv_Get_Position_in_Voxel(tp3d, tvox)){

					x_sc = ROUNDF(tvox.x)/sz_sub_cube;
					y_sc = ROUNDF(tvox.y)/sz_sub_cube;
					z_sc = ROUNDF(tvox.z)/sz_sub_cube;

					if(x_sc<0 || x_sc>ww_sc-1 || y_sc<0 || y_sc>hh_sc-1 || z_sc<0 || z_sc>dd_sc-1) break;

					tidx = z_sc*ww_sc*hh_sc +y_sc*ww_sc +x_sc;

					if(p_flag_sc[tidx]!=KV_FLAG_SUBCUBE_ACTIVATED)	p_flag_sc[tidx] = (char)KV_FLAG_SUBCUBE_CARVED;
					cnt++;

				}
				else break;
			}

			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		
		}
				
	}}

	return cnt;
}

//********************************************************************************************
bool LCKvYooji_Volume_Integrator::rodvhs_Remove_Outlier_Depth_using_Visual_Hull_Shield(
	CKvMatrixFloat *in_map_depth_of_visual_hull,
	CKvMatrixFloat *io_map_depth_of_object)
//********************************************************************************************
{
	int ww, hh;
	float *p_d_obj, *p_d_vh;

	io_map_depth_of_object->ms(ww, hh);

	if(in_map_depth_of_visual_hull->mw() != ww
		|| in_map_depth_of_visual_hull->mh() != hh)	return false;

	p_d_vh = in_map_depth_of_visual_hull->vp();
	p_d_obj = io_map_depth_of_object->vp();

	for(int i=0; i<ww*hh; i++){
		if(p_d_obj[i] == 0.0f)	continue;
		else if(p_d_vh[i] == 0.0f) p_d_obj[i] = 0.0f; 
		else if(p_d_vh[i] - 0.002f/*thershold*/ > p_d_obj[i])	p_d_obj[i] = 0.0f;
	}

	return true;
}

//********************************************************************************************
void LCKvYooji_Volume_Integrator::ctvovsc_Compute_TSDF_of_Voxels_on_Visible_Sub_Cubes(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Extrinsics *in_hmat_glob_to_cam,
	int in_edge_length_of_sub_cube,
	float in_mu, float in_maxW,
	CKvYooji_Cube_TSDF_Float *io_cube,
	bool in_flag_color)
//********************************************************************************************
{
	CKvSet2d_of_VectorShort *p_set_of_segs;
	CKvPoint3Df origin, p3d_c, p3d_g, tvox, tp3d, tp3d_rgb;
	CKvPointf pp, pp_rgb;
	CKvRgbF rgb;
	
	float *p_depth_raw, *p_tsdf;
	SHORT *p_seg;
	UCHAR *p_tsdf_rgb, *p_img_rgb;
	UCHAR *p_w_depth;
	char *p_flag_sc;
	float trgb[3], t_cg[3];

	int ww_d, hh_d, ww_rgb, hh_rgb, tidx, tx, ty, tx_rgb, ty_rgb;
	int WW_c, HH_c, DD_c;
	int WW_sc, HH_sc, DD_sc;
	int sz_sc;

	float sz_vox, tsdf, oldW, newW, lambda_inv;
	float td_map, td_vox, sdf, td_min;
	bool flag_color=false;
	int cnt=0;

	sz_sc = in_edge_length_of_sub_cube;
	zz_intrin.set_from_P_matrix(in_view->p_P_matrix_depth());

	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// Edge field TEST!!!!
	float *p_mag_edge = in_view->p_map_edge()->grad_mag.vp();
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	
	// get pointers.
	p_depth_raw = in_view->p_map_depth_raw()->mps(ww_d, hh_d)[0];
	p_img_rgb = in_view->p_image_rgb()->mps(ww_rgb, hh_rgb)[0];

	p_tsdf = io_cube->pvd_Pointer_of_Volume_Depth()->tps(WW_c, HH_c, DD_c)[0][0];
	p_tsdf_rgb = io_cube->pvr_Pointer_of_Volume_Rgb()->vp();
	p_w_depth = io_cube->pvwd_Pointer_of_Volume_Weight_Depth()->vp();
	p_set_of_segs = io_cube->pvhs_Pointer_of_Visual_Hull_Segments();
	p_flag_sc = io_cube->pvfsc_Pointer_of_Volume_Flag_of_Sub_Cube()->tps(WW_sc, HH_sc, DD_sc)[0][0];
	io_cube->goiw_Get_Origin_In_World(origin);
	sz_vox = io_cube->svm_Size_of_Voxel_in_Meter();

	// for TSDF computation.
	float norm;
	t_cg[0] = in_hmat_glob_to_cam->mp_transform_inv()->vp()[3];
	t_cg[1] = in_hmat_glob_to_cam->mp_transform_inv()->vp()[7];
	t_cg[2] = in_hmat_glob_to_cam->mp_transform_inv()->vp()[11];
	
	int Y, Z;
	int X_sc, Y_sc, Z_sc;
	int tX, tY, tZ, tX_min;
	int tX_max, tX_td_min;

//#pragma omp parallel for
	for(Z_sc=0; Z_sc<DD_sc; Z_sc++)
	{	
		for(Y_sc=0; Y_sc<HH_sc; Y_sc++)
		{	
			for(X_sc=0; X_sc<WW_sc; X_sc++)
			{
				// check sub-cube validity.
				if(p_flag_sc[Z_sc*WW_sc*HH_sc +Y_sc*WW_sc +X_sc]!=(char)KV_FLAG_SUBCUBE_ACTIVATED)	continue;

				// inside of each sub-cube.
				for(Y=0; Y<sz_sc; Y++)
				{	
					for(Z=0; Z<sz_sc; Z++)
					{
						tZ=Z_sc*sz_sc+Z;	tY=Y_sc*sz_sc+Y;		

						// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
						// set range of X.
// 						tX_min = X_sc*sz_sc;
// 						tX_max = X_sc*sz_sc +sz_sc-1;
						// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
						// set range of X according to visual hull segments.
						p_seg=p_set_of_segs->gpe(tZ, tY)->vp();
						
 						if(p_seg[0]>=(X_sc+1)*sz_sc)		continue;
 						else if(p_seg[0]>=X_sc*sz_sc)	tX_min=p_seg[0];
 						else tX_min=X_sc*sz_sc;
 
 						if(p_seg[1]<X_sc*sz_sc)		continue;
 						else if(p_seg[1]<(X_sc+1)*sz_sc)	tX_max=p_seg[1];
 						else tX_max=X_sc*sz_sc +sz_sc-1;						
						// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

						// find the voxel closest to depth surface between tZ_min and tZ_max.
						td_min=100000.0f;	tX_td_min=-1;
						for(tX=tX_min; tX<=tX_max; tX++)
						{	
							tvox.s_Set((float)(tX), (float)(tY), (float)(tZ));
							io_cube->gpw_Get_Position_in_World(tvox, p3d_g);
							
							// transform voxel from global coord. to depth camera coord.
							in_hmat_glob_to_cam->transform(p3d_g, p3d_c);			
							// project transformed voxel to depth camera.
							zz_intrin.project(p3d_c, pp);
	
							// uses nearest neighbor lookup to prevent smearing of measurements at depth discontinuities.
							// + for depth
							tx=ROUNDF(pp.x);	ty=ROUNDF(pp.y);								

							if(tx<0 || tx>ww_d-1 || ty<0 || ty>hh_d-1)		continue;

							// ================================================================
							td_map=p_depth_raw[ty*ww_d+tx];		// get depth.
							//if(!z_gid_Get_Interpolated_Depth(pp.x, pp.y, ww_d, hh_d, p_depth, td_map))	continue;
							// ================================================================							
							//td_vox=p3d_c.z;
			
							if(td_map>0.0f){

								// compute lambda.
								pp.x = float(tx); pp.y = float(ty);
								zz_intrin.back_project(pp,1,tp3d);
								lambda_inv = 1.0f/sqrtf(tp3d.x*tp3d.x + tp3d.y*tp3d.y + 1.0f);

								// calculate SDF value.
								tp3d.x = p3d_g.x - t_cg[0];
								tp3d.y = p3d_g.y - t_cg[1];
								tp3d.z = p3d_g.z - t_cg[2];
								norm = sqrtf(tp3d.x*tp3d.x + tp3d.y*tp3d.y + tp3d.z*tp3d.z);

								sdf=td_map - lambda_inv*norm;	// difference(m) between depth of voxel and depth of pixel where the voxel was projected.
			
								/// /////////////////////////////////////
								// truncate SDF value.
								if(sdf<-in_mu)	continue;		// no surface information is obtained from this range.
								tsdf=min(1.0f, sdf/in_mu);		// TSD value.
								/// /////////////////////////////////////				

								// calculate new weight.
								// + get global voxel index.
								tidx=tZ*WW_c*HH_c + tY*WW_c + tX;

								oldW = (float)p_w_depth[tidx];
								newW=oldW + 1.0f;

								// update voxel information.
								// + for depth.
								p_tsdf[tidx]=(oldW*p_tsdf[tidx] + tsdf)/newW;
								p_w_depth[tidx]=(UCHAR)min(newW, (float)in_maxW);

								//printf("p_tsdf[%d]: %f\n", tidx, p_tsdf[tidx]);

								// + for rgb.
								if(in_flag_color){
									in_view->convert_pixel_coord(pp, td_map, pp_rgb);
									tx_rgb = ROUNDF(pp_rgb.x);	ty_rgb = ROUNDF(pp_rgb.y);
									if(tx_rgb<0 || tx_rgb>ww_rgb-1 || ty_rgb<0 || ty_rgb>hh_rgb-1)		continue;

									// RGB.
  									for(int k=0; k<3; k++)	trgb[k] = p_img_rgb[ty_rgb*ww_rgb+tx_rgb + k*ww_rgb*hh_rgb];	// get color.
  									for(int k=0; k<3; k++)	
  										p_tsdf_rgb[tidx + k*WW_c*HH_c*DD_c] = 
  										(UCHAR)((oldW*(float)p_tsdf_rgb[tidx + k*WW_c*HH_c*DD_c] + (float)trgb[k])/newW);

									// Gradient magnitude.
// 									for(int k = 0; k<3; k++)	trgb[k] = min(255.0f, max(0.0f, p_mag_edge[ty_rgb*ww_rgb+tx_rgb]));	// get color.
// 									for(int k = 0; k<3; k++)
// 										p_tsdf_rgb[tidx + k*WW_c*HH_c*DD_c] =
// 										(UCHAR)((oldW*(float)p_tsdf_rgb[tidx + k*WW_c*HH_c*DD_c] + (float)trgb[k])/newW);
								}
								//// find the voxel closest to depth surface between tZ_min and tZ_max.
								//if(newW>10.0f && abs(td)<td_min){	td_min=abs(td);	tX_td_min=tX;	}
							}
						}

						// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 						//// set visual hull segments.
 						//if(tZ_td_min>=1 && tZ_td_min<DD_c-1 && td_min<0.0001f){	// m unit.
 						//	// compute z-direction gradient at the voxel closest to the depth surface.
 						//	grad = p_tsdf[(tZ_td_min+1)*WW_c*HH_c + tY*WW_c + tX] - p_tsdf[(tZ_td_min-1)*WW_c*HH_c + tY*WW_c + tX];
 
 						//	if(grad<0.0f && p_seg[0]==(USHORT)0){
 						//		p_seg[0] = (USHORT)max(tZ_td_min-10, 0);
 						//	}
 						//	else if(grad>0.0f && p_seg[1]==(USHORT)DD_c-1){
 						//		p_seg[1] = (USHORT)min(tZ_td_min+10, DD_c-1);
 						//	}
 						//}



						// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
						// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
						// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
						// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


					}
				}

			}
		}
	}
	
}

// ===================================== local functions ==================================== //
// ===================================== local functions ==================================== //
// ===================================== local functions ==================================== //

//********************************************************************************************
int LCKvYooji_Volume_Integrator::fvsc_Find_Visible_Sub_Cubes_using_Casting_Pixel_Rays(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Extrinsics *in_hmat_glob_to_cam,
	int in_edge_length_of_sub_cube,
	float in_mu,
	CKvYooji_Cube_TSDF_Float *io_cube)
//********************************************************************************************
{
	CKvPoint3Df p3d, p3d_block_e, tp3d, tvox;
	CKvPoint3Df dvec;
	CKvPixel tpix;

	float *p_depth_raw;
	char *p_flag_sc;

	float td;

	int ww, hh, cnt, tidx;
	int ww_sc, hh_sc, dd_sc;
	int x, y, n, x_sc, y_sc, z_sc;
	

	// get pointers.
	//////////////////////////////////////////////////////////////////////////
	// raw depth data.
	p_depth_raw = in_view->p_map_depth_raw()->mps(ww, hh)[0];
	//////////////////////////////////////////////////////////////////////////
	p_flag_sc = io_cube->pvfsc_Pointer_of_Volume_Flag_of_Sub_Cube()->tps(ww_sc, hh_sc, dd_sc)[0][0];	
	
	zz_intrin.set_from_P_matrix(in_view->p_P_matrix_depth());

	// cast rays for setting sub-cube validities.
	cnt = 0;
 	for(y=0; y<hh; y++){	for(x=0; x<ww; x++){
 
 		// back-project depth.
 		td = p_depth_raw[y*ww + x];
 		if (td > 0.0f){
 			tpix.x = x;	tpix.y = y;
 			// compute direction vector of the pixel ray.
 			zz_intrin.back_project(tpix, td, tp3d);		in_hmat_glob_to_cam->transform_inv(tp3d, p3d);
 			zz_intrin.back_project(tpix, 100.0f, tp3d);	in_hmat_glob_to_cam->transform_inv(tp3d, p3d_block_e);
 
 			dvec.x=p3d_block_e.x-p3d.x;	dvec.y=p3d_block_e.y-p3d.y;	dvec.z=p3d_block_e.z-p3d.z;
 			dvec.n_Normalize();
 
 			// activates sub-cubes on the pixel ray between +-mu of depth point.
 			for(n=-1; n<=1; n++){
 				tp3d.x=p3d.x +(float)n*in_mu*dvec.x;
 				tp3d.y=p3d.y +(float)n*in_mu*dvec.y;
 				tp3d.z=p3d.z +(float)n*in_mu*dvec.z;
 
 				if(io_cube->gpv_Get_Position_in_Voxel(tp3d, tvox)){
 
 					x_sc = ROUNDF(tvox.x)/in_edge_length_of_sub_cube;
 					y_sc = ROUNDF(tvox.y)/in_edge_length_of_sub_cube;
 					z_sc = ROUNDF(tvox.z)/in_edge_length_of_sub_cube;
 
 					if(x_sc<0 || x_sc>ww_sc-1 || y_sc<0 || y_sc>hh_sc-1 || z_sc<0 || z_sc>dd_sc-1)	continue;

 					tidx = z_sc*ww_sc*hh_sc +y_sc*ww_sc +x_sc;
 
 					if(p_flag_sc[tidx]!=(char)KV_FLAG_SUBCUBE_CARVED)	p_flag_sc[tidx] = (char)KV_FLAG_SUBCUBE_ACTIVATED;
 					cnt++;
 
 				}
 			}
 	
 		}
 				
 	}}
	// process all sub-blocks.
	//io_cube->pvfsc_Pointer_of_Volume_Flag_of_Sub_Cube()->st_Set_Volume((char)KV_FLAG_SUBCUBE_ACTIVATED);

	return cnt;
}

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//********************************************************************************************
void LCKvYooji_Volume_Integrator::ctvovsc_Compute_TSDF_of_Voxels_on_Visible_Sub_Cubes_RGB_NEW(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Extrinsics *in_hmat_glob_to_cam,
	int in_edge_length_of_sub_cube,
	float in_mu, float in_maxW,
	CKvYooji_Cube_TSDF_Float *io_cube,
	bool in_flag_color)
//********************************************************************************************
{
	CKvSet2d_of_VectorShort *p_set_of_segs;
	CKvPoint3Df origin, p3d, tvox, tp3d, tp3d_rgb;
	CKvPointf pp;
	CKvRgbF rgb;
	
	float *p_depth_raw, *p_tsdf;
	SHORT *p_seg;
	UCHAR *p_tsdf_rgb, *p_img_rgb;
	UCHAR *p_w_depth;
	char *p_flag_sc;
	float trgb[3];

	int ww, hh, tidx, tx, ty;
	int WW_c, HH_c, DD_c;
	int WW_sc, HH_sc, DD_sc;
	int sz_sc;

	float sz_vox, tsd, oldW, newW;
	float td_map, td_vox, td, td_min;
	bool flag_color=false;
	int cnt=0;

	sz_sc = in_edge_length_of_sub_cube;
	zz_intrin.set_from_P_matrix(in_view->p_P_matrix_RGB());

	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// Edge field TEST!!!!
	float *p_mag_edge = in_view->p_map_edge()->grad_mag.vp();
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	
	// get pointers.
	//////////////////////////////////////////////////////////////////////////
	// raw depth map.
	p_depth_raw = in_view->p_map_depth_filtered_on_RGB()->mps(ww, hh)[0];
	//////////////////////////////////////////////////////////////////////////
	p_img_rgb = in_view->p_image_rgb()->vp();

	p_tsdf = io_cube->pvd_Pointer_of_Volume_Depth()->tps(WW_c, HH_c, DD_c)[0][0];
	p_tsdf_rgb = io_cube->pvr_Pointer_of_Volume_Rgb()->vp();
	p_w_depth = io_cube->pvwd_Pointer_of_Volume_Weight_Depth()->vp();
	p_set_of_segs = io_cube->pvhs_Pointer_of_Visual_Hull_Segments();
	p_flag_sc = io_cube->pvfsc_Pointer_of_Volume_Flag_of_Sub_Cube()->tps(WW_sc, HH_sc, DD_sc)[0][0];
	io_cube->goiw_Get_Origin_In_World(origin);
	sz_vox = io_cube->svm_Size_of_Voxel_in_Meter();
	
	int Y, Z;
	int X_sc, Y_sc, Z_sc;
	int tX, tY, tZ, tX_min;
	int tX_max, tX_td_min;

	for(Z_sc=0; Z_sc<DD_sc; Z_sc++)
	{	
		for(Y_sc=0; Y_sc<HH_sc; Y_sc++)
		{	
			for(X_sc=0; X_sc<WW_sc; X_sc++)
			{
				// check sub-cube validity.
				if(p_flag_sc[Z_sc*WW_sc*HH_sc +Y_sc*WW_sc +X_sc]!=(char)KV_FLAG_SUBCUBE_ACTIVATED)	continue;

				// inside of each sub-cube.
				for(Y=0; Y<sz_sc; Y++)
				{	
					for(Z=0; Z<sz_sc; Z++)
					{
						tZ=Z_sc*sz_sc+Z;	tY=Y_sc*sz_sc+Y;		

						// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
						// set range of X.
// 						tX_min = X_sc*sz_sc;
// 						tX_max = X_sc*sz_sc +sz_sc-1;
						// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
						// set range of X according to visual hull segments.
						p_seg=p_set_of_segs->gpe(tZ, tY)->vp();
						
 						if(p_seg[0]>=(X_sc+1)*sz_sc)		continue;
 						else if(p_seg[0]>=X_sc*sz_sc)	tX_min=p_seg[0];
 						else tX_min=X_sc*sz_sc;
 
 						if(p_seg[1]<X_sc*sz_sc)		continue;
 						else if(p_seg[1]<(X_sc+1)*sz_sc)	tX_max=p_seg[1];
 						else tX_max=X_sc*sz_sc +sz_sc-1;						
						// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

						// find the voxel closest to depth surface between tZ_min and tZ_max.
						td_min=100000.0f;	tX_td_min=-1;
						for(tX=tX_min; tX<=tX_max; tX++)
						{	
							tvox.s_Set((float)(tX), (float)(tY), (float)(tZ));
							io_cube->gpw_Get_Position_in_World(tvox, tp3d);
							
							// transform voxel from global coord. to depth camera coord.
							in_hmat_glob_to_cam->transform(tp3d, p3d);			
							// project transformed voxel to depth camera.
							zz_intrin.project(p3d, pp);
	
							// uses nearest neighbor lookup to prevent smearing of measurements at depth discontinuities.
							// + for depth
							tx=ROUNDF(pp.x);	ty=ROUNDF(pp.y);								

							if(tx<0 || tx>ww-1 || ty<0 || ty>hh-1)		continue;

							// ================================================================
							td_map=p_depth_raw[ty*ww+tx];		// get depth.
							//if(!z_gid_Get_Interpolated_Depth(pp.x, pp.y, ww, hh, p_depth, td_map))	continue;
							// ================================================================							
							td_vox=p3d.z;
			
							if(td_map>0.0f){

								// calculate TSD value.
								td=td_map-td_vox;	// difference(m) between depth of voxel and depth of pixel where the voxel was projected.
			
								/// /////////////////////////////////////
								if(td<-in_mu)	continue;		// no surface information is obtained from this range.
								tsd=min(1.0f, td/in_mu);		// TSD value.
								/// /////////////////////////////////////				

								// calculate new weight.
								// + get global voxel index.
								tidx=tZ*WW_c*HH_c + tY*WW_c + tX;

								oldW = (float)p_w_depth[tidx];
								newW=oldW + 1.0f;

								// update voxel information.
								// + for depth.
								p_tsdf[tidx]=(oldW*p_tsdf[tidx] + tsd)/newW;
								p_w_depth[tidx]=(UCHAR)min(newW, (float)in_maxW);

								//printf("p_tsdf[%d]: %f\n", tidx, p_tsdf[tidx]);

								// + for rgb.
								if(in_flag_color){
									
									// RGB.
  									for(int k=0; k<3; k++)	trgb[k] = p_img_rgb[ty*ww+tx + k*ww*hh];	// get color.
  									for(int k=0; k<3; k++)	
  										p_tsdf_rgb[tidx + k*WW_c*HH_c*DD_c] = 
  										(UCHAR)((oldW*(float)p_tsdf_rgb[tidx + k*WW_c*HH_c*DD_c] + (float)trgb[k])/newW);

									// Gradient magnitude.
// 									for(int k = 0; k<3; k++)	trgb[k] = min(255.0f, max(0.0f, p_mag_edge[ty_rgb*ww_rgb+tx_rgb]));	// get color.
// 									for(int k = 0; k<3; k++)
// 										p_tsdf_rgb[tidx + k*WW_c*HH_c*DD_c] =
// 										(UCHAR)((oldW*(float)p_tsdf_rgb[tidx + k*WW_c*HH_c*DD_c] + (float)trgb[k])/newW);
								}
								//// find the voxel closest to depth surface between tZ_min and tZ_max.
								//if(newW>10.0f && abs(td)<td_min){	td_min=abs(td);	tX_td_min=tX;	}
							}
						}

						// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 						//// set visual hull segments.
 						//if(tZ_td_min>=1 && tZ_td_min<DD_c-1 && td_min<0.0001f){	// m unit.
 						//	// compute z-direction gradient at the voxel closest to the depth surface.
 						//	grad = p_tsdf[(tZ_td_min+1)*WW_c*HH_c + tY*WW_c + tX] - p_tsdf[(tZ_td_min-1)*WW_c*HH_c + tY*WW_c + tX];
 
 						//	if(grad<0.0f && p_seg[0]==(USHORT)0){
 						//		p_seg[0] = (USHORT)max(tZ_td_min-10, 0);
 						//	}
 						//	else if(grad>0.0f && p_seg[1]==(USHORT)DD_c-1){
 						//		p_seg[1] = (USHORT)min(tZ_td_min+10, DD_c-1);
 						//	}
 						//}



						// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
						// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
						// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
						// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


					}
				}

			}
		}
	}
	
}

// ===================================== local functions ==================================== //
// ===================================== local functions ==================================== //
// ===================================== local functions ==================================== //

//********************************************************************************************
int LCKvYooji_Volume_Integrator::fvsc_Find_Visible_Sub_Cubes_using_Casting_Pixel_Rays_RGB_NEW(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Extrinsics *in_hmat_glob_to_cam,
	int in_edge_length_of_sub_cube,
	float in_mu,
	CKvYooji_Cube_TSDF_Float *io_cube)
//********************************************************************************************
{
	CKvPoint3Df p3d, p3d_block_e, tp3d, tvox;
	CKvPoint3Df dvec;
	CKvPixel tpix;

	float *p_depth;
	char *p_flag_sc;

	float td;

	int ww, hh, cnt, tidx;
	int ww_sc, hh_sc, dd_sc;
	int x, y, n, x_sc, y_sc, z_sc;
	

	// get pointers.
	p_depth = in_view->p_map_depth_filtered_on_RGB()->mps(ww, hh)[0];
	p_flag_sc = io_cube->pvfsc_Pointer_of_Volume_Flag_of_Sub_Cube()->tps(ww_sc, hh_sc, dd_sc)[0][0];	
	
	zz_intrin.set_from_P_matrix(in_view->p_P_matrix_RGB());

	// cast rays for setting sub-cube validities.
	cnt = 0;
 	for(y=0; y<hh; y++){	for(x=0; x<ww; x++){
 
 		// back-project depth.
 		td = p_depth[y*ww + x];
 		if (td > 0.0f){
 			tpix.x = x;	tpix.y = y;
 			// compute direction vector of the pixel ray.
 			zz_intrin.back_project(tpix, td, tp3d);		in_hmat_glob_to_cam->transform_inv(tp3d, p3d);
 			zz_intrin.back_project(tpix, 100.0f, tp3d);	in_hmat_glob_to_cam->transform_inv(tp3d, p3d_block_e);
 
 			dvec.x=p3d_block_e.x-p3d.x;	dvec.y=p3d_block_e.y-p3d.y;	dvec.z=p3d_block_e.z-p3d.z;
 			dvec.n_Normalize();
 
 			// activates sub-cubes on the pixel ray between +-mu of depth point.
 			for(n=-1; n<=1; n++){
 				tp3d.x=p3d.x +(float)n*in_mu*dvec.x;
 				tp3d.y=p3d.y +(float)n*in_mu*dvec.y;
 				tp3d.z=p3d.z +(float)n*in_mu*dvec.z;
 
 				if(io_cube->gpv_Get_Position_in_Voxel(tp3d, tvox)){
 
 					x_sc = ROUNDF(tvox.x)/in_edge_length_of_sub_cube;
 					y_sc = ROUNDF(tvox.y)/in_edge_length_of_sub_cube;
 					z_sc = ROUNDF(tvox.z)/in_edge_length_of_sub_cube;
 
 					if(x_sc<0 || x_sc>ww_sc-1 || y_sc<0 || y_sc>hh_sc-1 || z_sc<0 || z_sc>dd_sc-1)	continue;

 					tidx = z_sc*ww_sc*hh_sc +y_sc*ww_sc +x_sc;
 
 					if(p_flag_sc[tidx]!=(char)KV_FLAG_SUBCUBE_CARVED)	p_flag_sc[tidx] = (char)KV_FLAG_SUBCUBE_ACTIVATED;
 					cnt++;
 
 				}
 			}
 	
 		}
 				
 	}}
	// process all sub-blocks.
	//io_cube->pvfsc_Pointer_of_Volume_Flag_of_Sub_Cube()->st_Set_Volume((char)KV_FLAG_SUBCUBE_ACTIVATED);

	return cnt;
}

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// ************************************ private functions ************************************ //
// ************************************ private functions ************************************ //
// ************************************ private functions ************************************ //
//********************************************************************************************
bool LCKvYooji_Volume_Integrator::z_gid_Get_Interpolated_Depth(
	float in_x, float in_y,
	int in_ww, int in_hh,
	float *in_depth_map,
	float &out_depth)
//********************************************************************************************
{
	// d1   d2
	//    x
	// d3   d4
	float d1, d2, d3, d4;
	int x, y;
	float resi_x, resi_y;

	if(in_x<=0.0f || in_x>float(in_ww-1) ||in_y<=0.0f || in_y>float(in_hh-1))	return false;

	x=(int)in_x;	y=(int)in_y;	resi_x=in_x-(float)x;	resi_y=in_y-(float)y;
	d1=in_depth_map[y*in_ww+x];			if(d1==0.0f)	return false;
	if(resi_x>0.0f){					d2=in_depth_map[y*in_ww+x+1];		if(d2==0.0f)	return false;	}
	if(resi_y>0.0f){					d3=in_depth_map[(y+1)*in_ww+x];		if(d3==0.0f)	return false;	}
	if(resi_x>0.0f && resi_y>0.0f){		d4=in_depth_map[(y+1)*in_ww+x+1];	if(d4==0.0f)	return false;	}

	out_depth=(1.0f-resi_y)*( (1.0f-resi_x)*d1+resi_x*d2 ) + resi_y*( (1.0f-resi_x)*d3+resi_x*d4 );	

	return true;
}

//********************************************************************************************
bool LCKvYooji_Volume_Integrator::z_gic_Get_Interpolated_Color(
	float in_x, float in_y,
	int in_ww, int in_hh,
	UCHAR *in_rgb_image,
	CKvRgbF &out_color)
//********************************************************************************************
{
	// d1   d2
	//    x
	// d3   d4
	CKvRgbF c1, c2, c3, c4;
	int x, y;
	float resi_x, resi_y;

	if(in_x<=0.0f || in_x>float(in_ww-1) ||in_y<=0.0f || in_y>float(in_hh-1))	return false;

	x=(int)in_x;	y=(int)in_y;	resi_x=in_x-(float)x;	resi_y=in_y-(float)y;
	c1.r=(float)in_rgb_image[y*in_ww+x];	
	c1.g=(float)in_rgb_image[y*in_ww+x +in_ww*in_hh];	
	c1.b=(float)in_rgb_image[y*in_ww+x +2*in_ww*in_hh];
	if(resi_x>0.0f){	
		c2.r=(float)in_rgb_image[y*in_ww+x+1];	
		c2.g=(float)in_rgb_image[y*in_ww+x+1 +in_ww*in_hh];	
		c2.b=(float)in_rgb_image[y*in_ww+x+1 +2*in_ww*in_hh];				
	}
	if(resi_y>0.0f){	
		c3.r=(float)in_rgb_image[(y+1)*in_ww+x];	
		c3.g=(float)in_rgb_image[(y+1)*in_ww+x +in_ww*in_hh];	
		c3.b=(float)in_rgb_image[(y+1)*in_ww+x +2*in_ww*in_hh];				
	}
	if(resi_x>0.0f && resi_y>0.0f){		
		c4.r=(float)in_rgb_image[(y+1)*in_ww+x+1];	
		c4.g=(float)in_rgb_image[(y+1)*in_ww+x+1 +in_ww*in_hh];	
		c4.b=(float)in_rgb_image[(y+1)*in_ww+x+1 +2*in_ww*in_hh];		
	}

	out_color.r=(1.0f-resi_y)*( (1.0f-resi_x)*c1.r+resi_x*c2.r ) + resi_y*( (1.0f-resi_x)*c3.r+resi_x*c4.r );
	out_color.g=(1.0f-resi_y)*( (1.0f-resi_x)*c1.g+resi_x*c2.g ) + resi_y*( (1.0f-resi_x)*c3.g+resi_x*c4.g );
	out_color.b=(1.0f-resi_y)*( (1.0f-resi_x)*c1.b+resi_x*c2.b ) + resi_y*( (1.0f-resi_x)*c3.b+resi_x*c4.b );

	return true;
}
