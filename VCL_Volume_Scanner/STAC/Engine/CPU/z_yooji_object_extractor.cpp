/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_object_extractor.cpp
/////////////////////////////////////////////////////////////////////////////////////////////

#include "../../../stdafx.h"
#include "../../_yooji_2016_object_scanning.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#endif

//********************************************************************************************
LCKvYooji_Object_Extractor::LCKvYooji_Object_Extractor(void)
//********************************************************************************************
{
	zz_classname="CKvYooji_Object_Extractor";
	zz_mat_4x4.ci_Create_Identity_matrix(4);
	
	// GMM background removal.
// 	zz_gmm_back = NULL;
// 	zz_mask_back = NULL;
	// Silhouette extraction using Graph Cut.
//	zz_color_model = NULL;
	zz_vec3.c_Create(3,0.0f);
	zz_beta = zz_L = zz_lambda = 0.0f;

	zz_flag_back = zz_flag_color = false;
	//c_Create(Kv_Point3Df(0.0f,0.0f,0.0f),8,8,8,8);
}

//********************************************************************************************
LCKvYooji_Object_Extractor::~LCKvYooji_Object_Extractor(void)
//********************************************************************************************
{
// 	if(zz_gmm_back)		delete zz_gmm_back;
// 	if(zz_mask_back)	delete zz_mask_back;
// 	if(zz_color_model)	delete zz_color_model;
}

//********************************************************************************************
void LCKvYooji_Object_Extractor::i_Initialize(
	CKvYooji_ColorModel *in_gmm_back_or_NULL,	
	CKvVectorInt *in_mask_back_or_NULL,
	int in_ww_rgb, int in_hh_rgb)
//********************************************************************************************
{
	if(in_gmm_back_or_NULL){
		//if(!zz_gmm_back)	zz_gmm_back = new CKvYooji_ColorModel;
		zz_flag_color = true;
		zz_gmm_back.cp_Copy(in_gmm_back_or_NULL);
	}

	if(in_mask_back_or_NULL){
		//if(!zz_mask_back)	zz_mask_back = new CKvMatrixBool;
		zz_flag_back = true;
		zz_mask_back.c_Create(in_hh_rgb, in_ww_rgb, false);
		z_sbm_Set_Background_Mask(in_mask_back_or_NULL, &zz_mask_back);
	}

	zz_mask_obj.c_Create(in_hh_rgb, in_ww_rgb, false);


}

//********************************************************************************************
void LCKvYooji_Object_Extractor::tbcmp_Train_Background_Color_Model_by_Pixel(
	vector<CKvMatrixUcharRgb> &in_img_rgb,
	CKvYooji_ColorModel *out_color_model_back_pixel,
	CKvYooji_ColorModel *out_color_model_back_global)
//********************************************************************************************
{
	vector<CKvVectorFloat> tmp_feat;
	int ww, hh, num_imgs;
	bool flag_full_cov;
	unsigned char *p_cimg;

	in_img_rgb[0].ms(ww, hh);
	num_imgs = in_img_rgb.size();

	// initialize feature vectors.
	tmp_feat.clear(); tmp_feat.resize(num_imgs);
	for(int i=0; i<num_imgs; i++) tmp_feat[i].c_Create(3, 0.0f);

	// select type of covariance matrix.
	//////////////////////////////////////////////////////////////////////////
	flag_full_cov = false;
	//////////////////////////////////////////////////////////////////////////
	out_color_model_back_pixel->c_Create(ww*hh,1,KV_COLOR_MODEL_RGB,flag_full_cov);
	zz_color_seg.i_Initialize(flag_full_cov);

	printf("================ START PIXELWISE BACKGROUND COLOR MODELING ================\n");
	// for pixel-wise model.
 	for(int i=0; i<ww*hh; i++){
 
 		float *p_rgb;
 
 		// extract color features from each image.
 		for(int n=0; n<num_imgs; n++){
 
 			p_cimg = in_img_rgb[n].vp();
 			p_rgb = tmp_feat[n].vp();
 
 			for(int k=0; k<3; k++)	p_rgb[k] = p_cimg[i + k*ww*hh];
 		}
 
 		// make color model of current pixel using multiple images.
 		zz_color_seg.cgmm_Clustering_using_Gaussian_Mixture_Model(
 			tmp_feat,
 			i,
 			out_color_model_back_pixel);
 		if(i % 1000 == 0) printf("-- #%d pixel was processed..\n",i);
 	}

	printf("================ START GLOBAL BACKGROUND COLOR MODELING ================\n");
	// for global model using last frame only.
	vector<CKvVectorFloat> feat_bg;

	float *p_v;
	unsigned char *p_img;
	int step_sz;

	p_img = in_img_rgb[num_imgs - 1].mps(ww,hh)[0];

	// Get features from exact background and foreground regions.
	p_v = zz_vec3.vp();

	step_sz = ww*hh;

	//if(!Kv_Printf("%d %d / %d %d", ww, hh, in_mask_bg_train.mw(), in_mask_bg_train.mh()))	exit(0);
	feat_bg.clear();
	for(int i=0; i<ww*hh; i++){

		for(int k = 0; k<3; k++)	p_v[k] = (float)p_img[i + k*step_sz];
		feat_bg.push_back(zz_vec3);
			
		//printf("%f %f %f\n", p_v[0], p_v[1], p_v[2]);
	}

	//printf("feat_bg: %d\n", feat_bg.size());

	// ==================================================================
	// Train color models for global background region.
	// ==================================================================
	out_color_model_back_global->c_Create(1,15,KV_COLOR_MODEL_RGB,true);
	zz_color_seg.i_Initialize(true);

	// trains global background model.
	zz_color_seg.cgmm_Clustering_using_Gaussian_Mixture_Model(
		feat_bg,
		0,
		out_color_model_back_global);	

	printf("================ COMPLETE ================\n");
}

//********************************************************************************************
void LCKvYooji_Object_Extractor::ibm_Import_Background_Models(
		CKvYooji_ColorModel &in_color_model_back_pixel,
		CKvYooji_ColorModel &in_color_model_back_global)
//********************************************************************************************
{
	zz_color_model_back_pixel.cp_Copy(&in_color_model_back_pixel);
	zz_color_model_back.cp_Copy(&in_color_model_back_global);
}

//********************************************************************************************
int LCKvYooji_Object_Extractor::gvd_Get_Valid_Depth(
	CKvMatrixFloat *io_depth_map,
	float in_depth_min, float in_depth_max)
//********************************************************************************************
{
	float *p_depth;
	int ww, hh, tidx, num_valid;
	int i, j;

	p_depth=io_depth_map->mps(ww, hh)[0];
	
	num_valid=0;
	for(j=0; j<hh; j++){	for(i=0; i<ww; i++){
		tidx=j*ww+i;		
		if(p_depth[tidx]<in_depth_min || p_depth[tidx]>in_depth_max){	p_depth[tidx]=0.0f;	continue;	}
		num_valid++;
	}}

	return num_valid;
}


//********************************************************************************************
int LCKvYooji_Object_Extractor::god_Get_Object_Depth(
	CKvMatrixFloat *io_depth_map,
	float in_depth_min, float in_depth_max)
//********************************************************************************************
{
	float *p_depth;
	int ww, hh, tidx, num_valid;
	int i, j;

	p_depth=io_depth_map->mps(ww, hh)[0];
	
	num_valid=0;
	for(j=0; j<hh; j++){	for(i=0; i<ww; i++){
		tidx=j*ww+i;		
		if(p_depth[tidx]<in_depth_min || p_depth[tidx]>in_depth_max){	p_depth[tidx]=0.0f;	continue;	}
		num_valid++;
	}}

	// pre-processing added at 2016.03.29.
	CKvMatrixBool obj_mask;
	bool *p_obj_mask;

	p_obj_mask = obj_mask.c_Create(hh, ww, false)[0];

	zz_color_seg.sibf_Smooth_Image_using_Bilateral_Filter(io_depth_map, 2.0f, io_depth_map);

	for(j = 0; j < hh; j++){
		for(i = 0; i < ww; i++){
			tidx = j*ww + i;
			if(p_depth[tidx]>0.0f)	p_obj_mask[tidx] = true;
		}
	}
	eo_Erode_Object(&obj_mask, 1, 1, false);
	for(j = 0; j < hh; j++){
		for(i = 0; i < ww; i++){
			tidx = j*ww + i;
			if(!p_obj_mask[tidx])	p_depth[tidx] = 0.0f;
		}
	}

	return num_valid;
}

//********************************************************************************************
int LCKvYooji_Object_Extractor::god_Get_Object_Depth(
	CKvMatrixFloat *io_depth_map,
	CKvPmatrix3D &in_pmat_depth,
	CKvPoint3Df &in_origin_cube,
	float in_size_cube)
//********************************************************************************************
{
	CKvYooji_Intrinsics intrinsics;
	CKvPoint3Df tp3d;

	float *p_depth;
	float td;
	int ww, hh, tidx, num_valid;
	int i, j;

	p_depth=io_depth_map->mps(ww, hh)[0];
	
	intrinsics.set_from_P_matrix(&in_pmat_depth);

	num_valid=0;
	for(j=0; j<hh; j++){	for(i=0; i<ww; i++){
		tidx=j*ww+i;	td = p_depth[tidx];
				
		intrinsics.back_project(Kv_Pixel(i, j), td, tp3d);

		if(tp3d.x < in_origin_cube.x || tp3d.x > in_origin_cube.x + in_size_cube
			|| tp3d.y < in_origin_cube.y || tp3d.y > in_origin_cube.y + in_size_cube
			|| tp3d.z < in_origin_cube.z || tp3d.z > in_origin_cube.z + in_size_cube){
			p_depth[tidx]=0.0f;
			continue;	
		}
		num_valid++;
	}}

	return num_valid;
}

////********************************************************************************************
//int LCKvYooji_Object_Extractor::god_Get_Object_Depth(
//	CKvYooji_MatrixRgbD *io_rgbd_img,
//	CKvYooji_Tracking_State *in_state,
//	CKvYooji_Cube_TSDF_Float *in_cube)
////********************************************************************************************
//{
//	CKvYooji_Intrinsics int_d, int_rgb;
//	CKvYooji_Extrinsics ext_d;
//
//	CKvMatrixBool obj_mask;
//
//	CKvPoint3Df origin_cube;
//	float size_cube;
//	CKvPoint3Df tp3d, tp3d_world;
//
//	float *p_depth;
//	bool *p_obj_mask;
//
//	float td;
//	int ww, hh, dim_cube, tidx, num_valid;
//	int max_d, min_d, cnt;
//	int i, j;
//
//	// Initialize parameters.
//	p_depth = (*io_rgbd_img).pdm_Pointer_of_Depth_Map()->mps(ww, hh)[0];
//	(*in_cube).goiw_Get_Origin_In_World(origin_cube);
//	(*in_cube).ts(dim_cube, dim_cube, dim_cube);
//	size_cube = (float)dim_cube*(*in_cube).svm_Size_of_Voxel_in_Meter();
//
//	int_d.spm_Set_from_P_Matrix((*io_rgbd_img).ppd_Pointer_of_Pmatrix_Depth());
//	//ext_d.spm_Set_from_P_Matrix((*io_rgbd_img).ppd_Pointer_of_Pmatrix_Depth());
//	//int_rgb.spm_Set_from_P_Matrix((*io_rgbd_img).pprgb_Pointer_of_Pmatrix_Rgb());
//
//	// Remove invalid depths out of object cube.
//	num_valid = 0;
//	for(j = 0; j<hh; j++){
//		for(i = 0; i<ww; i++){
//			tidx = j*ww+i;	td = p_depth[tidx];
//
//			int_d.bp_Back_Project(Kv_Pixel(i, j), td, tp3d);
//			ext_d.ti_Transform_Inverse(tp3d, tp3d_world);
//
//			if(tp3d_world.x < origin_cube.x || tp3d_world.x > origin_cube.x + size_cube
//				|| tp3d_world.y < origin_cube.y || tp3d_world.y > origin_cube.y + size_cube
//				|| tp3d_world.z < origin_cube.z || tp3d_world.z > origin_cube.z + size_cube){
//				p_depth[tidx] = 0.0f;
//				continue;
//			}
//			num_valid++;
//		}
//	}
//
//	//if(!Kv_Printf("num_valid %d", num_valid))	exit(0);
//
//	// pre-processing added at 2016.03.29.
//
//// 
//// 	p_obj_mask = obj_mask.c_Create(hh, ww, false)[0];
//// 
//// 	zz_color_seg.sibf_Smooth_Image_using_Bilateral_Filter(
//// 		(*io_rgbd_img).pdi_Pointer_of_Depth_Image(), 
//// 		2.0f, 
//// 		(*io_rgbd_img).pdi_Pointer_of_Depth_Image());
//// 
//// 	for(j = 0; j < hh; j++){
//// 		for(i = 0; i < ww; i++){
//// 			tidx = j*ww + i;
//// 			if(p_depth[tidx]>0.0f)	p_obj_mask[tidx] = true;
//// 		}
//// 	}
//// 
//// 	eo_Erode_Object(&obj_mask, 1, 1, false);
//// 
//// 	for(j = 0; j < hh; j++){
//// 		for(i = 0; i < ww; i++){
//// 			tidx = j*ww + i;
//// 			if(!p_obj_mask[tidx])	p_depth[tidx] = 0.0f;
//// 		}
//// 	}
//
//	return num_valid;
//}

//********************************************************************************************
int LCKvYooji_Object_Extractor::god_Get_Object_Depth(
	CKvYooji_MatrixRgbD *io_rgbd_img,
	CKvYooji_Tracking_State *in_state,
	CKvYooji_Cube_TSDF_Float *in_cube,
	bool in_mode_erosion,
	CKvYooji_ColorModel *in_color_model_of_hand,
	CKvMatrixBool *out_hand_mask)
//********************************************************************************************
{
	CKvYooji_Intrinsics int_d, int_rgb;
	CKvYooji_Extrinsics ext_d;

	CKvMatrixBool obj_mask, hand_mask;

	CKvPoint3Df origin_cube;
	float size_cube;
	CKvPoint3Df tp3d, tp3d_world;
	CKvPixel tpix;

	float *p_depth_raw, *p_depth_bottom;
	bool *p_mask_hand, *p_mask_obj;

	float td;
	int ww, hh, dim_cube, tidx, num_valid;
	int max_d, min_d, cnt;
	int i, j;

	// Initialize parameters.
	p_depth_raw = (*io_rgbd_img).p_map_depth_raw()->mps(ww, hh)[0];
	p_depth_bottom = (*io_rgbd_img).p_map_depth_filtered()->vp();

	(*in_cube).goiw_Get_Origin_In_World(origin_cube);
	(*in_cube).ts(dim_cube, dim_cube, dim_cube);
	size_cube = (float)dim_cube*(*in_cube).svm_Size_of_Voxel_in_Meter();

	int_d.copy((*in_state).p_intrinsics_depth());
	ext_d.copy((*in_state).p_extrinsics_glob_to_cam());
	//int_rgb.spm_Set_from_P_Matrix((*io_rgbd_img).pprgb_Pointer_of_Pmatrix_Rgb());

	//d_pm_Printf_Matrix((*in_state).p_extrinsics_glob_to_cam()->mp_transform()->vp(), 4, 4, "T_state");

	// Remove invalid depths out of object cube.
	num_valid=0;
	for(j=0; j<hh; j++){	for(i=0; i<ww; i++){
		tidx=j*ww+i;	td = p_depth_raw[tidx];
				
		tpix.x = i; tpix.y = j;
		int_d.back_project(tpix, td, tp3d);
		ext_d.transform_inv(tp3d, tp3d_world);

		if(tp3d_world.x < origin_cube.x || tp3d_world.x > origin_cube.x + size_cube
			|| tp3d_world.y < origin_cube.y || tp3d_world.y > origin_cube.y + size_cube
			|| tp3d_world.z < origin_cube.z || tp3d_world.z > origin_cube.z + size_cube){
			
			//////////////////////////////////////////////////////////////////////////
			p_depth_raw[tidx] = p_depth_bottom[tidx] = 0.0f;
			//////////////////////////////////////////////////////////////////////////

			continue;	
		}
		num_valid++;
	}}

	cnt = num_valid;

	if(in_color_model_of_hand){

		// Extract hand region.
		z_ghm_Get_Hand_Mask(
			io_rgbd_img,
			in_color_model_of_hand,
			&hand_mask);

		do_Dilate_Object(&hand_mask, 2, 2, false);

		// Get object mask.
		cnt = 0;
		p_mask_hand = hand_mask.vp();
		p_mask_obj = obj_mask.c_Create(hh, ww, false)[0];	
		
		for(int j = 0; j<hh; j++){
			for(int i = 0; i<ww; i++){

				tidx = j*ww+i;
				td = p_depth_raw[tidx];
				if(td>0.0f)	{
					if(!p_mask_hand[tidx]){ 
						p_mask_obj[tidx] = true;			
						cnt++; 
					} 
				}

			}
		}

		// find the largest object and erode object boundary region.
		eo_Erode_Object(&obj_mask, 1, 1, true);
		
		// Remove hand region.
		//////////////////////////////////////////////////////////////////////////
		for(int i=0; i<ww*hh; i++)	if(!p_mask_obj[i])	p_depth_raw[i] = p_depth_bottom[i] = 0.0f;
		//////////////////////////////////////////////////////////////////////////

		io_rgbd_img->p_image_silhouette()->cp_Copy(&obj_mask);

		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// 여기로 안들어 온다??
// 
// 		CKvScreen sc;
// 		sc.s_d_Display(&obj_mask);
// 		if(!Kv_Printf("ps_Pointer_of_Silhouette"))	exit(0);
// 		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// 
// 		printf("Color glove!!!\n");

		if(out_hand_mask) out_hand_mask->cp_Copy(&hand_mask);

	}
	else if(in_mode_erosion){
		// depth erosion.
		p_mask_obj = obj_mask.c_Create(hh, ww, false)[0];	
		for(int j = 0; j<hh; j++){
			for(int i = 0; i<ww; i++){

				tidx = j*ww+i;
				td = p_depth_raw[tidx];
				if(td>0.0f)	{
					p_mask_obj[tidx] = true;
					cnt++;
				}
			}
		}

		eo_Erode_Object(&obj_mask,1,1,false);
		//////////////////////////////////////////////////////////////////////////
		for(int i=0; i<ww*hh; i++)	if(!p_mask_obj[i])	p_depth_raw[i] = p_depth_bottom[i] = 0.0f;
		//////////////////////////////////////////////////////////////////////////
	}
	
	//////////////////////////////////////////////////////////////////////////

	//cout << obj_roi.br() << obj_roi.tl() << endl;

	//printf("roi: %d %d %d %d\n", imin, jmin, imax, jmax);
	//cout << obj_roi << endl;

	// for display ==================================================
// 	CKvScreen sc[3];
// 	sc[0].s_d_Display(&hand_mask);
// 	sc[1].s_d_Display(&obj_mask);
// 	
// 	if(!Kv_Printf("[#%d valid depths] Hand mask!", num_valid))	exit(0);
	// for display ==================================================

	return cnt;
}

//********************************************************************************************
int LCKvYooji_Object_Extractor::rbd_Remove_Board_Depth(
		CKvMatrixFloat *in_map_depth,
		CKvYooji_Intrinsics *in_K,
		Mat in_pi_board,
		CKvMatrixBool *out_mask_obj)
//********************************************************************************************
{
	CKvPoint3Df tp3d;
	CKvPixel tpix;

	float a,b,c,d;	// pi
	int ww, hh;
	float *p_depth = in_map_depth->mps(ww, hh)[0];
	bool *p_mask = out_mask_obj->c_Create(hh,ww,false)[0];

	a = in_pi_board.at<float>(0);
	b = in_pi_board.at<float>(1);
	c = in_pi_board.at<float>(2);
	d = in_pi_board.at<float>(3);

	float norm = sqrt(SQUARE(a) + SQUARE(b) + SQUARE(c));
	a /= norm; b /= norm; c /= norm; d /= norm;

	//printf("pi : %f %f %f %f\n", a, b, c, d);

	for(int i=0; i<ww*hh; i++){
		
		float td = p_depth[i];
		if(td <= 0.0f) continue;

		tpix.x = i%ww; tpix.y = i/ww;

		in_K->back_project(tpix,td,tp3d);

		float dist = a*tp3d.x + b*tp3d.y + c*tp3d.z + d;

		//printf("dist: %f\n", dist);

		if(dist > 0.125f) p_mask[i] = true;
		else            p_depth[i] = 0.0f;
	}


	return 1;
}


//********************************************************************************************
int LCKvYooji_Object_Extractor::godr_Get_Object_Depth_on_RGB_Camera(
	CKvYooji_MatrixRgbD *io_rgbd_img,
	CKvYooji_ColorModel *in_color_model_of_hand)
//********************************************************************************************
{
	CKvMatrixBool obj_mask, hand_mask;

	float *p_depth;
	bool *p_mask_hand, *p_mask_obj;

	float td;
	int ww, hh, dim_cube, tidx, num_valid;
	int max_d, min_d, cnt;
	int i, j;

	// Initialize parameters.
	p_depth = (*io_rgbd_img).p_map_depth_filtered_on_RGB()->mps(ww, hh)[0];

	// Remove invalid depths out of object cube.
	num_valid=0;
	for(j=0; j<hh; j++){	for(i=0; i<ww; i++){
		tidx=j*ww+i;	
		if(p_depth[tidx] <= 0.0f) continue;			
		
		num_valid++;
	}}
	cnt = num_valid;

	if(in_color_model_of_hand){

		// Extract hand region.
		z_ghmr_Get_Hand_Mask_on_RGB_Camera(
			io_rgbd_img,
			in_color_model_of_hand,
			&hand_mask);

		do_Dilate_Object(&hand_mask, 2, 2, false);

		// Get object mask.
		cnt = 0;
		p_mask_hand = hand_mask.vp();
		p_mask_obj = obj_mask.c_Create(hh, ww, false)[0];	
		
		for(int j = 0; j<hh; j++){
			for(int i = 0; i<ww; i++){

				tidx = j*ww+i;
				td = p_depth[tidx];
				if(td>0.0f)	{
					if(!p_mask_hand[tidx]){ 
						p_mask_obj[tidx] = true;			
						cnt++; 
					} 
				}

			}
		}

		// find the largest object and erode object boundary region.
		//eo_Erode_Object(&obj_mask, 1, 1, true);
		
		// Remove hand region.
		for(int i=0; i<ww*hh; i++)	if(!p_mask_obj[i])	p_depth[i] = 0.0f;

		io_rgbd_img->p_image_silhouette()->cp_Copy(&obj_mask);

		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// 여기로 안들어 온다??
// 
// 		CKvScreen sc;
// 		sc.s_d_Display(&obj_mask);
// 		if(!Kv_Printf("ps_Pointer_of_Silhouette"))	exit(0);
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		//printf("Color glove!!!\n");

	}

	// for display ==================================================
// 	CKvScreen sc[3];
// 	sc[0].s_d_Display(&hand_mask);
// 	sc[1].s_d_Display(&obj_mask);
// 	
// 	if(!Kv_Printf("[#%d valid depths] Hand mask!", num_valid))	exit(0);
	// for display ==================================================

	return cnt;
}

//********************************************************************************************
int LCKvYooji_Object_Extractor::gods_Get_Object_Depth_and_Silhouette(
	CKvYooji_MatrixRgbD *io_rgbd_img,
	CKvYooji_Tracking_State *in_state,
	CKvYooji_Cube_TSDF_Float *in_cube)
//********************************************************************************************
{
	CKvYooji_Intrinsics int_d, int_rgb;
	CKvYooji_Extrinsics ext_d;

	CKvMatrixBool obj_mask, hand_mask;

	CKvPoint3Df origin_cube;
	float size_cube;
	CKvPoint3Df tp3d, tp3d_world;
	CKvPixel tpix;

	float *p_depth;
	bool *p_mask_hand, *p_mask_obj;

	float td;
	int ww, hh, dim_cube, tidx, num_valid;
	int max_d, min_d, cnt;
	int i, j;

	// Initialize parameters.
	p_depth = (*io_rgbd_img).p_map_depth_raw()->mps(ww, hh)[0];
	(*in_cube).goiw_Get_Origin_In_World(origin_cube);
	(*in_cube).ts(dim_cube, dim_cube, dim_cube);
	size_cube = (float)dim_cube*(*in_cube).svm_Size_of_Voxel_in_Meter();

	int_d.set_from_P_matrix((*io_rgbd_img).p_P_matrix_depth());
	ext_d.copy((*in_state).p_extrinsics_glob_to_cam());

	//ext_d.spm_Set_from_P_Matrix((*io_rgbd_img).ppd_Pointer_of_Pmatrix_Depth());
	//int_rgb.spm_Set_from_P_Matrix((*io_rgbd_img).pprgb_Pointer_of_Pmatrix_Rgb());

	// Remove invalid depths out of object cube.
	num_valid = 0;
	for(j = 0; j<hh; j++){
		for(i = 0; i<ww; i++){
			tidx = j*ww+i;	td = p_depth[tidx];

			tpix.x = i; tpix.y = j;
			int_d.back_project(tpix, td, tp3d);
			ext_d.transform_inv(tp3d, tp3d_world);

			if(tp3d_world.x < origin_cube.x || tp3d_world.x > origin_cube.x + size_cube
				|| tp3d_world.y < origin_cube.y || tp3d_world.y > origin_cube.y + size_cube
				|| tp3d_world.z < origin_cube.z || tp3d_world.z > origin_cube.z + size_cube){
				p_depth[tidx] = 0.0f;
				continue;
			}
			num_valid++;
		}
	}

	eos_Extract_Object_Silhouette(
		io_rgbd_img,
		in_state);


// 	CKvScreen sc;
// 	sc.s_d_Display(io_rgbd_img->ps_Pointer_of_Silhouette());
// 	if(!Kv_Printf("Sil!!!"))	exit(0);


	//sc_oe[0].s_d_Display(io_rgbd_img->psi_Pointer_of_Silhouette_Image());

	return num_valid;
}

//********************************************************************************************
int LCKvYooji_Object_Extractor::dppod_Do_Pre_Processing_for_Object_Depth(
		CKvMatrixFloat *io_depth_map)
//********************************************************************************************
{
	float *p_depth;
	int ww, hh, tidx, num_valid;
	int i, j;

	p_depth=io_depth_map->mps(ww, hh)[0];
	
	// pre-processing added at 2016.03.29.
	CKvMatrixBool obj_mask;
	bool *p_obj_mask;

	p_obj_mask = obj_mask.c_Create(hh, ww, false)[0];

	//zz_color_seg.sibf_Smooth_Image_using_Bilateral_Filter(io_depth_map, 2.0f, io_depth_map);

	for(j = 0; j < hh; j++){
		for(i = 0; i < ww; i++){
			tidx = j*ww + i;
			if(p_depth[tidx]>0.0f)	p_obj_mask[tidx] = true;
		}// 	z_tdrcc_Transform_Depth_to_Rgb_Camera_Coordinates(
	}
	eo_Erode_Object(&obj_mask, 1, 1, false);

	num_valid = 0;
	for(j = 0; j < hh; j++){
		for(i = 0; i < ww; i++){
			tidx = j*ww + i;
			if(!p_obj_mask[tidx])	p_depth[tidx] = 0.0f;
			else					num_valid++;
		}
	}

	return num_valid;
}


//********************************************************************************************
bool LCKvYooji_Object_Extractor::eos_Extract_Object_Silhouette(
	CKvYooji_MatrixRgbD *in_rgbd_images,
	CKvYooji_Tracking_State *in_tracking_state)
//********************************************************************************************
{
	//int num_labels;

	CKvMatrixBool *p_mask = in_rgbd_images->p_image_silhouette();
	
//  	z_tdrcc_Transform_Depth_to_Rgb_Camera_Coordinates(
//  		in_rgbd_images,
//  		&zz_img_d_in_rgb);

// 	z_cidfu_Convert_Image_Depth_Float_to_Uchar(
// 		in_rgbd_images->pdi_Pointer_of_Depth_Image(),
// 		KV_CONVERT_IMAGE_DEPTH_FL_TO_DISP_UC,
// 		&zz_img_d8);
// 
// 	z_cidfu_Convert_Image_Depth_Float_to_Uchar(
// 		&zz_img_d_in_rgb,
// 		KV_CONVERT_IMAGE_DEPTH_FL_TO_DISP_UC,
// 		&zz_img_d8_in_rgb);
// 
// 	sc_oe[0].s_d_Display(&zz_img_d8);
// 	sc_oe[1].s_d_Display(&zz_img_d8_in_rgb);
// 
// 	if(!Kv_Printf("%d %d <-> %d %d", 
// 		in_rgbd_images->pdi_Pointer_of_Depth_Image()->mw(), 
// 		in_rgbd_images->pdi_Pointer_of_Depth_Image()->mh(), 
// 		zz_img_d_in_rgb.mw(), 
// 		zz_img_d_in_rgb.mh()))	exit(0);

	if(zz_flag_back){
		
		if(p_mask->mw() != zz_mask_back.mw() || p_mask->mh() != zz_mask_back.mh())
			p_mask->c_Create(zz_mask_back.mh(), zz_mask_back.mw(), false);

// 		zz_color_seg.brgmm_Background_Removal_using_GMM(
// 			&zz_gmm_back,
// 			&zz_mask_back,
// 			in_rgbd_images->prgbi_Pointer_of_RGB_Image(),
// 			NULL,
// 			//in_rgbd_images->pdmrc_Pointer_of_Depth_Map_on_RGB_Camera(),//&zz_img_d_in_rgb,
// 			p_mask);

		eo_Erode_Object(p_mask, 1, 1, true);
		do_Dilate_Object(p_mask, 3, 3, true);
	}
	else 
		return false;

	// Remove background region in RGB image.
// 	for(int i = 0; i<zz_img_d_in_rgb.vs(); i++){
// 		if(!p_mask->vp()[i]){
// 			in_rgbd_images->prgbi_Pointer_of_RGB_Image()->vp()[i] =
// 				in_rgbd_images->prgbi_Pointer_of_RGB_Image()->vp()[i + zz_img_d_in_rgb.vs()] =
// 				in_rgbd_images->prgbi_Pointer_of_RGB_Image()->vp()[i + 2*zz_img_d_in_rgb.vs()] = (UCHAR)0;
// 		}
// 	}


//	sc_oe[0].s_d_Display(in_rgbd_images->prgbi_Pointer_of_RGB_Image());
//	if(!Kv_Printf("RGB"))	exit(0);


//  	z_sslic_Segmentation_using_SLIC(
//  		io_rgbd_images->mprgb_Matrix_Pointer_of_RGB_Image(),
//  		&zz_slic_labels,
// 		&zz_slic_connect,
// 		num_labels,
// 		&zz_slic_img_superpixels);

// 	z_cidfu_Convert_Image_Depth_Float_to_Uchar(
// 		io_rgbd_images->mpd_Matrix_Pointer_of_Depth_Image(),
// 		KV_CONVERT_IMAGE_DEPTH_FL_TO_DISP_UC,
// 		&zz_img_d8);
// 	z_cidfu_Convert_Image_Depth_Float_to_Uchar(
// 		&zz_img_d_in_rgb,
// 		KV_CONVERT_IMAGE_DEPTH_FL_TO_DISP_UC,
// 		&zz_img_d8_in_rgb);
// 
//  	sc_oe[0].s_d_Display(in_rgbd_images->prgbi_Pointer_of_RGB_Image());



	return true;
}


//********************************************************************************************
bool LCKvYooji_Object_Extractor::eosbc_Extract_Object_Silhouette_using_Background_Cut(
	CKvYooji_MatrixRgbD *in_rgbd_images,
	CKvDepot_of_Point3D *in_points_cube,
	CKvVectorInt *in_indices_cube,
	CKvPmatrix3D *in_pmat_current,
	CKvMatrixBool *out_mask_object)
//********************************************************************************************
{
	vector<CKvMatrixBool> set_of_masks;
	CKvMatrixUcharRgb *p_in_cimg = in_rgbd_images->p_image_rgb();
	CKvMatrixFloat *p_in_depth = in_rgbd_images->p_map_depth_filtered_on_RGB();

	int ww, hh;	p_in_cimg->ms(ww, hh);

	set_of_masks.resize(2);
	set_of_masks[0].c_Create(hh, ww, false);
	set_of_masks[1].c_Create(hh, ww, false);

	// Get exact background using object cube projection.
	z_gszbi_Generate_Silhouette_using_Z_buffering_for_an_Image(
		in_points_cube,
		in_indices_cube,
		in_pmat_current,
		&set_of_masks[1]);
	for(int i=0; i<ww*hh; i++) set_of_masks[0].vp()[i] = (set_of_masks[1].vp()[i]) ? false : true;

	// Get exact foreground using object depth region.
	for(int i=0; i<ww*hh; i++){
		if(!set_of_masks[1].vp()[i]) continue;
		set_of_masks[1].vp()[i] = (p_in_depth->vp()[i] > 0.0f) ? true : false;
	}
	eo_Erode_Object(&set_of_masks[1], 1, 1, true);

printf("dbc_Do_Background_Cut\n");
	dbc_Do_Background_Cut(
		*in_rgbd_images->p_image_rgb(),
		set_of_masks[0],
		set_of_masks[1],
		*out_mask_object);

	//////////////////////////////////////////////////////////////////////////
	do_Dilate_Object(out_mask_object, 2, 2, true);
	//////////////////////////////////////////////////////////////////////////

	set_of_masks.clear();
	
	return true;
}

//********************************************************************************************
bool LCKvYooji_Object_Extractor::eosrd_Extract_Obejct_Silhouette_from_Rendered_Depth(
	CKvYooji_MatrixRgbD *in_rgbd_images,
	CKvYooji_Tracking_State *in_tracking_state,
	CKvMatrixBool *out_mask_object)
//********************************************************************************************
{
	//int num_labels;

	z_tdrcc_Transform_Depth_to_Rgb_Camera_Coordinates(
		in_rgbd_images,
// 		in_rgbd_images->prgbi_Pointer_of_RGB_Image(),
// 		in_tracking_state->prd_Pointer_of_Rendered_Depth(),
// 		in_rgbd_images->ppd_Pointer_of_Pmatrix_Depth(),
// 		in_rgbd_images->pprgb_Pointer_of_Pmatrix_Rgb(),
		&zz_img_d_in_rgb);

	if(zz_flag_color){
// 		zz_color_seg.brgmm_Background_Removal_using_GMM(
// 			&zz_color_model_inter,
// 			&zz_mask_back,
// 			in_rgbd_images->prgbi_Pointer_of_RGB_Image(),
// 			&zz_img_d_in_rgb,
// 			out_mask_object);

		eo_Erode_Object(out_mask_object, 1, 1, true);
		do_Dilate_Object(out_mask_object, 2, 2, true);
	}


//  	z_sslic_Segmentation_using_SLIC(
//  		io_rgbd_images->mprgb_Matrix_Pointer_of_RGB_Image(),
//  		&zz_slic_labels,
// 		&zz_slic_connect,
// 		num_labels,
// 		&zz_slic_img_superpixels);

// 	z_cidfu_Convert_Image_Depth_Float_to_Uchar(
// 		io_rgbd_images->mpd_Matrix_Pointer_of_Depth_Image(),
// 		KV_CONVERT_IMAGE_DEPTH_FL_TO_DISP_UC,
// 		&zz_img_d8);
// 	z_cidfu_Convert_Image_Depth_Float_to_Uchar(
// 		&zz_img_d_in_rgb,
// 		KV_CONVERT_IMAGE_DEPTH_FL_TO_DISP_UC,
// 		&zz_img_d8_in_rgb);
// 
//  	sc_oe[0].s_d_Display(in_rgbd_images->prgbi_Pointer_of_RGB_Image());
//  	sc_oe[1].s_d_Display(&out_mask_object);

//	if(!Kv_Printf("!!!!"))	exit(0);



	return true;
}


//********************************************************************************************
// CAUTION: Currently, depth and color image should have same image size.
void LCKvYooji_Object_Extractor::z_ghm_Get_Hand_Mask(
	CKvYooji_MatrixRgbD *in_rgbd_img,
	CKvYooji_ColorModel *in_color_model_of_hand,
	CKvMatrixBool *out_mask_hand)
//********************************************************************************************
{
	int in_class_num, in_gauss_num, in_color_type;

	in_class_num = (*in_color_model_of_hand).gnc_Get_Number_of_Classes();
	in_gauss_num = (*in_color_model_of_hand).gnm_Get_Number_of_Models();
	in_color_type = (*in_color_model_of_hand).gct_Get_Color_Type();

	if(in_class_num<=0){
		printf("[LCKvYooji_Object_Extractor::rhr_Remove_Hand_Region] Invalid GMM model was loaded.\n");
		system("pause");
		exit(0);
	}

	CKvVectorFloat tmp_rgb, tmp_pcs, tmp_hue, tmp_vec;
	CKvVectorFloat *p_mean_vec, *p_cov_mat_inv_diag;
	CKvPointf tp_d, tp_rgb;

	int ww, hh, tidx, x_rgb, y_rgb;

	bool *p_mask_hand;
	float *p_map_d;
	unsigned char *p_r, *p_g, *p_b;

	unsigned char r_val, g_val, b_val;
	double h_val, s_val, i_val;
	float *p_tmp_rgb, *p_tmp_pcs, *p_tmp_hue;
	float *p_w, *p_det;

	float *p_tmp_vec, prob;

	p_map_d = (*in_rgbd_img).p_map_depth_raw()->mps(ww, hh)[0];
	if(ww!=out_mask_hand->mw() || hh!=out_mask_hand->mh())		p_mask_hand = out_mask_hand->c_Create(hh, ww, false)[0];
	else														p_mask_hand = out_mask_hand->vp();
	p_r = (*in_rgbd_img).p_image_rgb()->vp(p_g, p_b);
	p_tmp_vec = tmp_vec.c_Create(3, 0.0f);

	if((*in_rgbd_img).p_image_rgb()->mw()!=ww || 
		(*in_rgbd_img).p_image_rgb()->mh()!=hh){
		printf("[LCKvYooji_Object_Extractor::rhr_Remove_Hand_Region] Dimensions of input images are different.\n");
		system("pause");
		exit(0);
	}

	if(in_color_type==KV_COLOR_MODEL_HUE){
		p_tmp_hue = tmp_hue.c_Create(1, 0.0f);
		p_tmp_rgb = p_tmp_pcs = NULL;
	}
	else{
		p_tmp_rgb = tmp_rgb.c_Create(3, 0.0f);
		p_tmp_pcs = tmp_pcs.c_Create(3, 0.0f);
		p_tmp_hue = NULL;
	}

	/// Currently, select single model of which the weight is the largest value for each class. 
	// get pointers of color model.
	p_mean_vec = (*in_color_model_of_hand).gpmvs_Get_Pointer_of_Mean_Vector_Set(0);
	p_cov_mat_inv_diag = (*in_color_model_of_hand).gpcmids_Get_Pointer_of_Covariance_Matrix_Inverse_Diagonal_Set(0);
	p_det = (*in_color_model_of_hand).gpdcm_Get_Pointer_of_Determinant_of_Covariance_Matrix(0);
	/// ///////////////////////////////////////////////////
	// find hand region.
	for(int j = 0; j<hh; j++){
		for(int i = 0; i<ww; i++){
			tidx = j*ww+i;
			if(p_map_d[tidx]>0){

				/// find pixel on rgb camera coordinates.
				// This is bottle-neck!!!!!
				// This is bottle-neck!!!!!
				// This is bottle-neck!!!!!
				tp_d.x = (float)i;	tp_d.y = (float)j;
				(*in_rgbd_img).convert_pixel_coord(
					tp_d,
					p_map_d[tidx],
					tp_rgb);
			
				x_rgb =(int)tp_rgb.x;	y_rgb =(int)tp_rgb.y;
				// This is bottle-neck!!!!!
				// This is bottle-neck!!!!!
				// This is bottle-neck!!!!!
				if(x_rgb<0 || y_rgb<0 || x_rgb>=ww || y_rgb>=hh)	continue;
				r_val = p_r[y_rgb*ww+x_rgb];		g_val = p_g[y_rgb*ww+x_rgb];			b_val = p_b[y_rgb*ww+x_rgb];

				// convert color coordinates.
				if(in_color_type==KV_COLOR_MODEL_RGB){
					p_tmp_rgb[0] = (float)r_val;	p_tmp_rgb[1] = (float)g_val;	p_tmp_rgb[2] = (float)b_val;
					prob = (*in_color_model_of_hand).gmd_Get_Mahalanobis_Distance(&tmp_rgb, 0, 0);
					//prob=z_gmd_Get_Mahalanobis_Distance(&tmp_rgb, &p_mean_vec[0], &p_cov_mat_inv_diag[0], p_det[0]);
				}
				else if(in_color_type==KV_COLOR_MODEL_PCS){
					p_tmp_rgb[0] = (float)r_val;	p_tmp_rgb[1] = (float)g_val;	p_tmp_rgb[2] = (float)b_val;
					zz_color_seg.cc_Convert_Color_Rgb_to_PCS(tmp_rgb, tmp_pcs);
					prob = (*in_color_model_of_hand).gmd_Get_Mahalanobis_Distance(&tmp_pcs, 0, 0);
					//prob=z_gmd_Get_Mahalanobis_Distance(&tmp_pcs, &p_mean_vec[0], &p_cov_mat_inv_diag[0], p_det[0]);
				}
				else{
					zz_color_seg.ccvrh_Convert_Color_Value_RGB_to_HSI(r_val, g_val, b_val, h_val, s_val, i_val);
					if(s_val>0.2 && i_val <200.0f)	p_tmp_hue[0] = (float)h_val;
					else{ p_tmp_hue[0] = -1.0f;		continue; }
					prob = (*in_color_model_of_hand).gmdc_Get_Mahalanobis_Distance_Circular(&tmp_hue, 0, 0);
					//prob=z_gmdc_Get_Mahalanobis_Distance_Circular(&tmp_hue, &p_mean_vec[0], &p_cov_mat_inv_diag[0], p_det[0]);					
				}

				if(prob<3.0f)	p_mask_hand[tidx] = true;
				else			p_mask_hand[tidx] = false;
			}
		}
	}
}


//********************************************************************************************
// CAUTION: Currently, depth and color image should have same image size.
void LCKvYooji_Object_Extractor::z_ghmr_Get_Hand_Mask_on_RGB_Camera(
	CKvYooji_MatrixRgbD *in_rgbd_img,
	CKvYooji_ColorModel *in_color_model_of_hand,
	CKvMatrixBool *out_mask_hand)
//********************************************************************************************
{
	int in_class_num, in_gauss_num, in_color_type;

	in_class_num = (*in_color_model_of_hand).gnc_Get_Number_of_Classes();
	in_gauss_num = (*in_color_model_of_hand).gnm_Get_Number_of_Models();
	in_color_type = (*in_color_model_of_hand).gct_Get_Color_Type();

	if(in_class_num<=0){
		printf("[LCKvYooji_Object_Extractor::rhr_Remove_Hand_Region] Invalid GMM model was loaded.\n");
		system("pause");
		exit(0);
	}

	CKvVectorFloat tmp_rgb, tmp_pcs, tmp_hue, tmp_vec;
	CKvVectorFloat *p_mean_vec, *p_cov_mat_inv_diag;

	int ww, hh, tidx, x_rgb, y_rgb;

	bool *p_mask_hand;
	float *p_map_d;
	unsigned char *p_r, *p_g, *p_b;

	unsigned char r_val, g_val, b_val;
	double h_val, s_val, i_val;
	float *p_tmp_rgb, *p_tmp_pcs, *p_tmp_hue;
	float *p_w, *p_det;

	float *p_tmp_vec, prob;

	p_map_d = (*in_rgbd_img).p_map_depth_filtered_on_RGB()->mps(ww, hh)[0];
	if(ww!=out_mask_hand->mw() || hh!=out_mask_hand->mh())		p_mask_hand = out_mask_hand->c_Create(hh, ww, false)[0];
	else														p_mask_hand = out_mask_hand->vp();
	p_r = (*in_rgbd_img).p_image_rgb()->vp(p_g, p_b);
	p_tmp_vec = tmp_vec.c_Create(3, 0.0f);

	if((*in_rgbd_img).p_image_rgb()->mw()!=ww || 
		(*in_rgbd_img).p_image_rgb()->mh()!=hh){
		printf("[LCKvYooji_Object_Extractor::rhr_Remove_Hand_Region] Dimensions of input images are different.\n");
		system("pause");
		exit(0);
	}

	if(in_color_type==KV_COLOR_MODEL_HUE){
		p_tmp_hue = tmp_hue.c_Create(1, 0.0f);
		p_tmp_rgb = p_tmp_pcs = NULL;
	}
	else{
		p_tmp_rgb = tmp_rgb.c_Create(3, 0.0f);
		p_tmp_pcs = tmp_pcs.c_Create(3, 0.0f);
		p_tmp_hue = NULL;
	}

	/// Currently, select single model of which the weight is the largest value for each class. 
	// get pointers of color model.
	p_mean_vec = (*in_color_model_of_hand).gpmvs_Get_Pointer_of_Mean_Vector_Set(0);
	p_cov_mat_inv_diag = (*in_color_model_of_hand).gpcmids_Get_Pointer_of_Covariance_Matrix_Inverse_Diagonal_Set(0);
	p_det = (*in_color_model_of_hand).gpdcm_Get_Pointer_of_Determinant_of_Covariance_Matrix(0);
	/// ///////////////////////////////////////////////////
	// find hand region.
	for(int j = 0; j<hh; j++){
		for(int i = 0; i<ww; i++){
			tidx = j*ww+i;
			if(p_map_d[tidx]>0.0f){

				x_rgb =i;	y_rgb =j;

				r_val = p_r[tidx];		g_val = p_g[tidx];			b_val = p_b[tidx];

				// convert color coordinates.
				if(in_color_type==KV_COLOR_MODEL_RGB){
					p_tmp_rgb[0] = (float)r_val;	p_tmp_rgb[1] = (float)g_val;	p_tmp_rgb[2] = (float)b_val;
					prob = (*in_color_model_of_hand).gmd_Get_Mahalanobis_Distance(&tmp_rgb, 0, 0);
					//prob=z_gmd_Get_Mahalanobis_Distance(&tmp_rgb, &p_mean_vec[0], &p_cov_mat_inv_diag[0], p_det[0]);
				}
				else if(in_color_type==KV_COLOR_MODEL_PCS){
					p_tmp_rgb[0] = (float)r_val;	p_tmp_rgb[1] = (float)g_val;	p_tmp_rgb[2] = (float)b_val;
					zz_color_seg.cc_Convert_Color_Rgb_to_PCS(tmp_rgb, tmp_pcs);
					prob = (*in_color_model_of_hand).gmd_Get_Mahalanobis_Distance(&tmp_pcs, 0, 0);
					//prob=z_gmd_Get_Mahalanobis_Distance(&tmp_pcs, &p_mean_vec[0], &p_cov_mat_inv_diag[0], p_det[0]);
				}
				else{
					zz_color_seg.ccvrh_Convert_Color_Value_RGB_to_HSI(r_val, g_val, b_val, h_val, s_val, i_val);
					if(s_val>0.2 && i_val <200.0f)	p_tmp_hue[0] = (float)h_val;
					else{ p_tmp_hue[0] = -1.0f;		continue; }
					prob = (*in_color_model_of_hand).gmdc_Get_Mahalanobis_Distance_Circular(&tmp_hue, 0, 0);
					//prob=z_gmdc_Get_Mahalanobis_Distance_Circular(&tmp_hue, &p_mean_vec[0], &p_cov_mat_inv_diag[0], p_det[0]);					
				}

				if(prob<3.0f)	p_mask_hand[tidx] = true;
				else			p_mask_hand[tidx] = false;
			}
		}
	}
}

//********************************************************************************************
void LCKvYooji_Object_Extractor::z_tdrcc_Transform_Depth_to_Rgb_Camera_Coordinates(	
	CKvYooji_MatrixRgbD *in_rgbd_img,
// 	CKvMatrixUcharRgb *in_img_rgb,
// 	CKvMatrixFloat *in_img_d,	
// 	CKvPmatrix3D *in_pmat_d,
// 	CKvPmatrix3D *in_pmat_rgb,
	CKvMatrixFloat *out_img_d_transformed)
//********************************************************************************************
{	
	CKvPointf p2d_d, p2d_rgb;
	CKvPoint3Df p3d_d;

	float *p_img_d, *p_img_d_trans;

	float td;
	int ww_d, hh_d, ww_rgb, hh_rgb;
	int i, j, tx, ty;


	p_img_d = (*in_rgbd_img).p_map_depth_raw()->mps(ww_d, hh_d)[0];
	(*in_rgbd_img).p_image_rgb()->ms(ww_rgb, hh_rgb);	

	if(out_img_d_transformed->mw()!=ww_rgb || out_img_d_transformed->mh()!=hh_rgb)	
		out_img_d_transformed->c_Create(hh_rgb, ww_rgb, 0.0f);
	p_img_d_trans = out_img_d_transformed->vp();	

	// Initialization.
	for(i=0; i<ww_rgb*hh_rgb; i++)	p_img_d_trans[i] = 0.0f;

	for(j=0; j<hh_d; j++){			
		for(i=0; i<ww_d; i++){

			td = p_img_d[j*ww_d + i];
			if(td>0.0f){

				p2d_d.x = (float)i;	p2d_d.y = (float)j;

				(*in_rgbd_img).convert_pixel_coord(
					p2d_d,
					td,
					p2d_rgb);
				
//				if(!Kv_Printf("%f %f <-> %f %f", p2d_d.x, p2d_d.y, p2d_rgb.x, p2d_rgb.y))	exit(0);

// 				zz_intrin_d.bp_Back_Project(p2d_d, td, p3d_d);
// 				in_pmat_rgb->tp_Transform_Point(Kv_Point3D(p3d_d.x, p3d_d.y, p3d_d.z), p2d_rgb);

				tx = ROUNDF(p2d_rgb.x);		ty = ROUNDF(p2d_rgb.y);
				if(tx<0 || tx>ww_rgb-1 || ty<0 || ty>hh_rgb-1)	continue;

				p_img_d_trans[ty*ww_rgb + tx] = td;

			}
		}
	}
}

//*******************************************************************************************
void LCKvYooji_Object_Extractor::z_gszbi_Generate_Silhouette_using_Z_buffering_for_an_Image(
	CKvDepot_of_Point3D *in_depot_point, 
	CKvVectorInt *in_idx_mesh,
	CKvPmatrix3D *in_p_mat,
	CKvMatrixBool *out_silhouette)
//*******************************************************************************************
{
	int *p_idx_mesh, total_vertices, num_triangle,k;
	CKvSet_of_Point3D pt;
	CKvPoint3D *p_pt;
	float t_pt[9],p_mat[12],t_depth[3],t_position[6];

	//Initialization
	p_idx_mesh     = in_idx_mesh->vps(total_vertices);
	num_triangle   = total_vertices/3;
	in_depot_point->e_Export(&pt);
	in_p_mat      ->e_Export(true,p_mat);
	p_pt		   = pt.vp();
	
	//Perform z-buffering 
	for(k=0;k<num_triangle;k++){
		t_pt[0] = (float)p_pt[p_idx_mesh[3*k]  ].x;
		t_pt[1] = (float)p_pt[p_idx_mesh[3*k]  ].y;
		t_pt[2] = (float)p_pt[p_idx_mesh[3*k]  ].z;
		t_pt[3] = (float)p_pt[p_idx_mesh[3*k+1]].x;
		t_pt[4] = (float)p_pt[p_idx_mesh[3*k+1]].y;
		t_pt[5] = (float)p_pt[p_idx_mesh[3*k+1]].z;
		t_pt[6] = (float)p_pt[p_idx_mesh[3*k+2]].x;
		t_pt[7] = (float)p_pt[p_idx_mesh[3*k+2]].y;
		t_pt[8] = (float)p_pt[p_idx_mesh[3*k+2]].z;

		gd2dpt_Get_Depth_and_2D_Position_of_a_Triangle(t_pt,p_mat,t_position,t_depth);
		ub_Update_Buffer(t_position,t_depth,k,out_silhouette);
	}
}

//*******************************************************************************************
void LCKvYooji_Object_Extractor::ub_Update_Buffer(
	float *in_position,
	float *in_depth,
	int in_idx_mesh,
	CKvMatrixBool *io_silhouette)
//*******************************************************************************************
{
	//Depth interpolation 영역 찾기 
	//투영된 영역의 depth값 비교
	//비교하여 더 가까우면 값 치환 
	//frame buffer에 triangle index 넣음 
	//in_position: 0:x 1:y 2:x 3:y 4:x 5:y
	//LTRB: 0:LT.x 1:LT.y 2:RB.x 3:RB.y
	bool **p_sil;
	int LTRB[4],j,i;
	int x,y,k;
	/// ///////////////////////////////////////
	int ww, hh;
	/// ///////////////////////////////////////
	p_sil   = io_silhouette->mps(ww, hh);
	
	//Apply depth directly
	for(k=0;k<3;k++){
		// bounding position.
		x = min(max((int)in_position[2*k], 0), ww-1); 
		y = min(max((int)in_position[2*k+1], 0), hh-1);		
		
		p_sil[y][x]    = true;		
	}	
	//Interpolate  depth 
	gbi_Get_Block_for_Interpolation(in_position,LTRB);
	// bounding position.
	LTRB[0]=min(max(LTRB[0], 0), ww-1);		LTRB[1]=min(max(LTRB[1], 0), hh-1);		
	LTRB[2]=min(max(LTRB[2], 0), ww-1);		LTRB[3]=min(max(LTRB[3], 0), hh-1);

	for(j=LTRB[1];j<LTRB[3];j++){
		for(i=LTRB[0];i<LTRB[2];i++){
			if(!p_sil[j][i]){		p_sil[j][i] = true;	}
		}
	}
}

//*******************************************************************************************
void LCKvYooji_Object_Extractor::gbi_Get_Block_for_Interpolation(
	float *in_position,
	int *out_LTRB)
//*******************************************************************************************
{
	//in_position: 0:x 1:y 2:x 3:y 4:x 5:y
	//LTRB: 0:LT.x 1:LT.y 2:RB.x 3:RB.y
	int k;
	out_LTRB[0] = 100000.0f;
	out_LTRB[1] = 100000.0f;
	out_LTRB[2] = -100000.0f;
	out_LTRB[3] = -100000.0f;

	for(k=0;k<3;k++){
		if(in_position[2*k  ]<out_LTRB[0]) out_LTRB[0] = nint(in_position[2*k  ]  );
		if(in_position[2*k  ]>out_LTRB[2]) out_LTRB[2] = nint(in_position[2*k  ]+1);
		if(in_position[2*k+1]<out_LTRB[1]) out_LTRB[1] = nint(in_position[2*k+1]  );
		if(in_position[2*k+1]>out_LTRB[3]) out_LTRB[3] = nint(in_position[2*k+1]+1);
	}
}


//*******************************************************************************************
void LCKvYooji_Object_Extractor::gd2dpt_Get_Depth_and_2D_Position_of_a_Triangle(
	float *in_pt, 
	float *in_p_mat,
	float *out_position,
	float *out_depth)
//*******************************************************************************************
{
	int k;
	for(k=0;k<3;k++){gd2dpp_Get_Depth_and_2D_Position_of_a_Point(&in_pt[3*k],in_p_mat,&out_position[2*k],out_depth[k]);}
}
//*******************************************************************************************
void LCKvYooji_Object_Extractor::gd2dpp_Get_Depth_and_2D_Position_of_a_Point(
	float *in_pt, 
	float *in_p_mat,
	float *out_position,
	float &out_depth)
//*******************************************************************************************
{
	double x,y,w;
	double detM,m_3;

	//x= PX
	x = in_p_mat[0]*in_pt[0]+in_p_mat[1]*in_pt[1]+in_p_mat[2] *in_pt[2]+in_p_mat[3] *1;
	y = in_p_mat[4]*in_pt[0]+in_p_mat[5]*in_pt[1]+in_p_mat[6] *in_pt[2]+in_p_mat[7] *1;
	w = in_p_mat[8]*in_pt[0]+in_p_mat[9]*in_pt[1]+in_p_mat[10]*in_pt[2]+in_p_mat[11]*1;
	
	out_position[0] = (float)(x/w);
	out_position[1] = (float)(y/w);

	//Get depth
	detM =	in_p_mat[0]*(in_p_mat[5]*in_p_mat[10] - in_p_mat[6]*in_p_mat[9])-
		    in_p_mat[1]*(in_p_mat[4]*in_p_mat[10] - in_p_mat[6]*in_p_mat[8])+
			in_p_mat[2]*(in_p_mat[4]*in_p_mat[9 ] - in_p_mat[5]*in_p_mat[8]);
	m_3  =  sqrt(in_p_mat[8]*in_p_mat[8] + in_p_mat[9]*in_p_mat[9] + in_p_mat[10]*in_p_mat[10]);

	if(detM>0)	out_depth =    w/m_3;
	else		out_depth = -1*w/m_3;
}

////********************************************************************************************
//bool LCKvYooji_Object_Extractor::z_sbm_Set_Background_Mask(
//	CKvVectorInt *in_background_region,
//	CKvMatrixBool *out_background_mask)
////********************************************************************************************
//{	
//	float a[4], b[4], c[4];		// for equations of boundary lines.
//	int *p_back_region;
//	bool *p_back_mask;
//
//	float fx;
//	int x1, x2, y1, y2;
//	int i, j, ww, hh;
//
//	if(in_background_region->vs()!=8)	return false;
//	p_back_region = in_background_region->vp();
//	p_back_mask = out_background_mask->mps(ww, hh)[0];
//
//	// calculate boundary lines.
//	for(i=0; i<4; i++){
//
//		x1=p_back_region[2*i];		y1=p_back_region[2*i+1];
//		if(i==3){	x2=p_back_region[0];	y2=p_back_region[1];	}
//		else{		x2=p_back_region[2*i+2];	y2=p_back_region[2*i+3];	}
//
//		if(x2-x1==0){
//			a[i]=1.0f;	b[i]=0.0f; c[i]=-x1;
//		}
//		else{
//			a[i]=(float)(y2-y1)/(float)(x2-x1);	b[i]=-1.0f; c[i]=-a[i]*(float)x1-b[i]*(float)y1;
//		}		
//
//		//if(!Kv_Printf("%d %d", p_back_region[2*i], p_back_region[2*i+1]))	exit(0);
//		//if(!Kv_Printf("%f %f %f", a[i], b[i], c[i]))	exit(0);
//	}
//
//	// set boundary mask.
//	for(j=0; j<hh; j++){
//		for(i=0; i<ww; i++){
//			
//			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//
//			fx = a[0]*(float)i + b[0]*(float)j + c[0];	if(a[0]*fx<0.0f){	p_back_mask[j*ww+i] = false;	continue;	}
//			fx = a[1]*(float)i + b[1]*(float)j + c[1];	if(a[0]*fx<0.0f){	p_back_mask[j*ww+i] = false;	continue;	}
// 			fx = a[2]*(float)i + b[2]*(float)j + c[2];	if(a[0]*fx>0.0f){	p_back_mask[j*ww+i] = false;	continue;	}
// 			fx = a[3]*(float)i + b[3]*(float)j + c[3];	if(a[0]*fx<0.0f){	p_back_mask[j*ww+i] = false;	continue;	}
//			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//			p_back_mask[j*ww+i] = true;
//
//		}
//	}	
//
//}

//********************************************************************************************
bool LCKvYooji_Object_Extractor::z_sbm_Set_Background_Mask(
	CKvVectorInt *in_background_region,
	CKvMatrixBool *io_background_mask)
//********************************************************************************************
{	
	vector<CKvMatrixFloat> set_of_half_lines;
	CKvPoint3Df tmp_line;		// for equations of boundary lines.

	float *p_tv;
	int *p_back_region;
	bool *p_back_mask;

	float fx;
	int x1, x2, y1, y2, tx, ty;
	int i, j, k, ww, hh, cnt;

	if(in_background_region->vs()!=8)	return false;
	p_back_region = in_background_region->vp();
	p_back_mask = io_background_mask->mps(ww, hh)[0];


	// Generates set of half lines.
	set_of_half_lines.resize(4);	for(i=0; i<4; i++)	set_of_half_lines[i].c_Create(3, 3, 0.0f);
	for(i = 0; i<4; i++){

		cnt = 0;

		x1 = p_back_region[2*i];			y1 = p_back_region[2*i+1];				
		for(j = 0; j<4; j++){
			if(j!=i){

				x2 = p_back_region[2*j];	y2 = p_back_region[2*j+1];

				// Computes line coefficients.
				// + y = ax + c. -> ax - y + c = 0.
				if(x2-x1==0){
					tmp_line.x = 1.0f;	tmp_line.y = 0.0f; tmp_line.z = -x1;
				}
				else{
					tmp_line.x = (float)(y2-y1)/(float)(x2-x1);
					tmp_line.y = -1.0f;
					tmp_line.z = -tmp_line.x*(float)x1-tmp_line.y*(float)y1;
				}

				// Corrects sign of line coefficients.
				// + Considering direction vector,
				// + if ax - y + c >= 0, (x, y) is on the right of the line.
				// + Finds signs of dx and dy are different.
				//  				if(((coeff_line.x >= 0) && (x2-x1 <= 0))			// case 1.
				//  					|| ((coeff_line.x <= 0) && (x2-x1 <=0)))		// case 2.
				// 				coeff_line = -coeff_line;

				if(x2-x1 >= 0){
					tmp_line.x = -tmp_line.x;
					tmp_line.y = -tmp_line.y;
					tmp_line.z = -tmp_line.z;
				}

				p_tv = set_of_half_lines[i].mp()[cnt];
				p_tv[0] = tmp_line.x;	p_tv[1] = tmp_line.y;	p_tv[2] = tmp_line.z;

				cnt++;
				//set_of_half_lines[i].push_back(tmp_line);
				// 				cout << in_rect_corners[i] << endl;
				// 				cout << in_rect_corners[j] << endl;
				// 				cout << coeff_line << endl;
			}
		}

	}

	/// //////////////////////////////////////////////////
	/// //////////////////////////////////////////////////
	/// //////////////////////////////////////////////////
	/// //////////////////////////////////////////////////
	/// //////////////////////////////////////////////////
	/// //////////////////////////////////////////////////
	/// //////////////////////////////////////////////////
	/// //////////////////////////////////////////////////
	/// //////////////////////////////////////////////////


	// under construction... 2016.08.12.
	for(k = 0; k<ww*hh; k++){

		p_back_mask[k] = true;		// initialize.

		// Checks which current pixel is on the right of boundary lines or not.
		for(i = 0; i<set_of_half_lines.size(); i++){
			for(j = 0; j<3; j++){

				tx = k%ww;	ty = k/ww;

				p_tv = set_of_half_lines[i].mp()[j];
				tmp_line.x = p_tv[0];	tmp_line.y = p_tv[1];	tmp_line.z = p_tv[2];
				fx = tmp_line.x*(float)tx + tmp_line.y*(float)ty + tmp_line.z;
				// Removes pixel on the left of all 3 lines.
				if(fx > 0.0f){		// if pixel is on the right of line.					
					break;
				}
				if(j == 2){
					p_back_mask[k] = false;
				}
			}

		}
	}

	/// //////////////////////////////////////////////////
	/// //////////////////////////////////////////////////
	/// //////////////////////////////////////////////////
	/// //////////////////////////////////////////////////
	/// //////////////////////////////////////////////////
	/// //////////////////////////////////////////////////
	/// //////////////////////////////////////////////////
	/// //////////////////////////////////////////////////
	/// //////////////////////////////////////////////////

	return true;

}

//********************************************************************************************
void LCKvYooji_Object_Extractor::z_cidfu_Convert_Image_Depth_Float_to_Uchar(
	CKvMatrixFloat *in_img_d,
	int in_mode_convert,
	CKvMatrixUchar *out_img_d)
//********************************************************************************************
{
	int out_ww, out_hh;
	float max_d, min_d, scale;
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

	//// find min/max depth value.
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
			p_out_img_d[i]=(unsigned char)(255.0f-255.0f*(p_in_img_d[i]-min_d)*scale);
		else
			p_out_img_d[i]=(unsigned char)(255.0f*(p_in_img_d[i]-min_d)*scale);
	}
}

//********************************************************************************************
void LCKvYooji_Object_Extractor::do_Dilate_Object(CKvMatrixBool*io_mask_obj, 
	int in_dx, int in_dy, 
	bool in_largest_blob_select_mode)
//********************************************************************************************
{
	CKvRunSet zz_rs;		/// CkvRunSet 대신 CKvSdkRunset의 dil_Dilation 함수 사용 시 죽을 때가 많음.

	zz_rs.i_Import(io_mask_obj);		
	zz_rs.d_Dilation(&zz_rs, in_dx, in_dy);		
	zz_rs.e_Export(true, false, io_mask_obj);

	if(in_largest_blob_select_mode){
		CKvSdkCodeRun zz_code_run;
		CKvSdkRunset zz_srs;		
		zz_code_run.im_Import(io_mask_obj);		
		zz_code_run.eo_Extract_Objects(0, 1, &zz_srs);		//zz_code_run.eo_Extract_an_Object(0, &zz_srs);
		zz_srs.ex_Export(true, false, io_mask_obj);
	}
}


//********************************************************************************************
void LCKvYooji_Object_Extractor::eo_Erode_Object(CKvMatrixBool *io_mask_obj, 
	int in_dx, int in_dy, 
	bool in_largest_blob_select_mode)
//********************************************************************************************
{
	CKvRunSet zz_rs;

	zz_rs.i_Import(io_mask_obj);		
	zz_rs.e_Erosion(&zz_rs, in_dx, in_dy);			
	zz_rs.e_Export(true, false, io_mask_obj);

	if(in_largest_blob_select_mode){
		CKvSdkCodeRun zz_code_run;
		CKvSdkRunset zz_srs;
		zz_code_run.im_Import(io_mask_obj);		
		zz_code_run.eo_Extract_Objects(0, 1, &zz_srs);		//zz_code_run.eo_Extract_an_Object(0, &zz_srs);
		zz_srs.ex_Export(true, false, io_mask_obj);
	}
}

//********************************************************************************************
// Graph cut with MaxFlow 3.01.
void LCKvYooji_Object_Extractor::dgc_Do_Grab_Cut(
		CKvMatrixUcharRgb &in_img,
		CKvMatrixBool &in_bg, 
 		CKvMatrixBool &in_fg,
		CKvMatrixBool &out_mask_obj)
//********************************************************************************************
{
	CKvScreen sc;
	CKvMatrixUcharRgb timg;

 	typedef Graph<float, float, float> GraphType;
 	GraphType *g;// = new GraphType(/*estimated # of nodes*/ 2, /*estimated # of edges*/ 1);

	unsigned char *p_img, *p_timg;
	bool *p_bg, *p_fg, *p_mask_obj;
	
	float tsmooth, max_smooth, K, beta;

	int ww, hh, tidx, step, cnt_edge;
	int max_iter, num_iter;
	int num_node, num_edge;
	
	p_img = in_img.mps(ww, hh)[0];			step = ww*hh;
	zz_img.cp_Copy(&in_img);
	p_mask_obj = out_mask_obj.c_Create(hh, ww, false)[0];

	p_bg = in_bg.vp();
	p_fg = in_fg.vp();
	
	// ==================================================================
	// Estimate probable fore/background region using initial GMM.
	// 0 : Probable background region.
	// 1 : Probable foreground region.
	// ==================================================================
	ipfm_Initialize_Probable_Foreground_Mask(
		in_img,
		in_bg,
		in_fg,
		zz_color_model_inter,
		out_mask_obj);

	sc.s_d_Display(&out_mask_obj);
	if(!Kv_Printf("Initial probable mask!"))	exit(0);

	// =======================================================
	// Set lambda and L.
	// =======================================================	
	zz_lambda = 50.0f;
	zz_L = 8*zz_lambda + 1;

	// =======================================================
	// Compute beta.
	// =======================================================	
	beta = 0.0f;	cnt_edge = 0;
	// 8-neighborhood.
	for(int j = 0; j<hh; j++){
		for(int i = 0; i<ww; i++){

			tidx = j*ww + i;

			if(i<ww - 1){
				for(int k=0; k<3; k++)	beta += SQUARE(p_img[tidx + k*step] - p_img[tidx + 1 + k*step]);		// right.
				cnt_edge++;
			}
			if(i<ww - 1 && j>0){
				for(int k=0; k<3; k++)	beta += SQUARE(p_img[tidx + k*step] - p_img[tidx + 1 - ww + k*step]);	// upright.
				cnt_edge++;
			}
			if(j>0){
				for(int k=0; k<3; k++)	beta += SQUARE(p_img[tidx + k*step] - p_img[tidx - ww + k*step]);		// up.				
				cnt_edge++;
			}
			if(i>0 && j>0){
				for(int k=0; k<3; k++)	beta += SQUARE(p_img[tidx + k*step] - p_img[tidx - 1 - ww + k*step]);	// upleft.				
				cnt_edge++;
			}

		}
	}
	zz_beta = float(cnt_edge)/(2.0f*beta);


	// =======================================================
	// Do iterative optimization.
	// =======================================================
	// Initialize graph.
	num_node = ww*hh;	num_edge = cnt_edge;
	max_iter = 10;
	for(int num_iter=0; num_iter<max_iter; num_iter++){

		// Initialize Graph.
		g = new GraphType(num_node, num_edge);
		g->add_node(num_node);

		// =======================================================
		// Update GMM parameters.
		// =======================================================
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// 이 부분도 full covariance matrix 적용하여 코딩해야 함.
		// Update Gaussian Mixture Model using updated labels.
		ugmmp_Update_GMM_Parameters(
			in_img,
			out_mask_obj,
			&zz_color_model_inter);
// 
// 		sc.s_d_Display(&out_mask_obj);
// 		if(!Kv_Printf("Initialized probable mask"))	exit(0);
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// =======================================================
		// Compute data term.
		// SOURCE -> background | SINK -> object
		// Case 1: object		-> T(source): 0 | S(sink): K
		// Case 2: background	-> T(source): K | S(sink): 0
		// Case 3: undetermined 
		// -> T(source): dataCost(background) | S(sink): dataCost(object)
		// =======================================================
		for(int j = 0; j<hh; j++){
			for(int i = 0; i<ww; i++){

				tidx = j*ww + i;

				if(p_fg[tidx]){				// the pixel is in foreground.
					g->add_tweights(tidx, 0.0f, zz_L);	// Graph cut paper....?
					//g->add_tweights(tidx, K, 0.0f);		// -> OpenCV version.
				}
				else if(p_bg[tidx]){		// the pixel is in background.
					g->add_tweights(tidx, zz_L, 0.0f);	// Graph cut paper....?
					//g->add_tweights(tidx, 0.0f, K);		// -> OpenCV version.
				}
				else{						// the pixel is undetermined.				
					//g->add_tweights(tidx, dataCost(tidx, 0), dataCost(tidx, 1));
					g->add_tweights(tidx, dataCost(tidx, 1), dataCost(tidx, 0));					
				}

			}
		}

		// =======================================================
		// Compute smoothness term.
		// =======================================================
		// 8-neighborhood.
 		for(int j = 0; j<hh; j++){
 			for(int i = 0; i<ww; i++){
 
 				tidx = j*ww + i;
 
 				if(i<ww - 1){
 					tsmooth = smoothCost(tidx, tidx + 1, 0, 1);		// right.
 					g->add_edge(tidx, tidx + 1, tsmooth, tsmooth);
 				}
 				if(i<ww - 1 && j>0){	
 					tsmooth = smoothCost(tidx, tidx + 1 - ww, 0, 1);	// upright.
 					g->add_edge(tidx, tidx + 1 - ww, tsmooth, tsmooth);
 				}
 				if(j>0){
 					tsmooth = smoothCost(tidx, tidx - ww, 0, 1);		// up.
 					g->add_edge(tidx, tidx - ww, tsmooth, tsmooth);
 				}
 				if(i>0 && j>0){
 					tsmooth = smoothCost(tidx, tidx - 1 - ww, 0, 1);	// upleft.
 					g->add_edge(tidx, tidx - 1 - ww, tsmooth, tsmooth);
 				}
 
 			}
 		}

		// Optimize labels.
		int flow = g->maxflow();
		
		// update segmentation results.	
		p_timg = timg.cp_Copy(&in_img)[0];
		for(tidx = 0; tidx<ww*hh; tidx++){			
			if(!(p_mask_obj[tidx] = g->what_segment(tidx)))	
				for(int n=0; n<3; n++)	p_timg[tidx + ww*hh*n] = (unsigned char)0;
		}

		sc.s_d_Display(&timg);
		if(!Kv_Printf("[#%d iter]Flow = %d\n", num_iter, flow))	exit(0);


		delete g;
	} 	
}

//********************************************************************************************
// Graph cut with MaxFlow 3.01.
void LCKvYooji_Object_Extractor::dbc_Do_Background_Cut(
		CKvMatrixUcharRgb &in_img,
		CKvMatrixBool &in_bg, 
 		CKvMatrixBool &in_fg,
		CKvMatrixBool &out_mask_obj)
//********************************************************************************************
{

	CKvMatrixUcharRgb timg;

 	typedef Graph<float, float, float> GraphType;
 	GraphType *g;// = new GraphType(/*estimated # of nodes*/ 2, /*estimated # of edges*/ 1);

	unsigned char *p_img, *p_timg;
	bool *p_bg, *p_fg, *p_mask_obj;
	
	float tsmooth, max_smooth, K, beta;

	int ww, hh, tidx, step, cnt_edge;
	int max_iter, num_iter;
	int num_node, num_edge;
	
	p_img = in_img.mps(ww, hh)[0];			step = ww*hh;
	zz_img.cp_Copy(&in_img);
	p_mask_obj = out_mask_obj.c_Create(hh, ww, false)[0];

	p_bg = in_bg.vp();
	p_fg = in_fg.vp();
	
	// ==================================================================
	// Estimate probable fore/background region using initial GMM.
	// 0 : Probable background region.
	// 1 : Probable foreground region.
// 	// ==================================================================
	ipfm_Initialize_Probable_Foreground_Mask(
		in_img,
		in_bg,
		in_fg,
		zz_color_model_back_pixel,
		zz_color_model_obj,
		out_mask_obj);

// 	CKvScreen sc;
// 	sc.s_d_Display(&in_fg);
// 	if(!Kv_Printf("Initial probable mask!"))	exit(0);

	// extract feature vectors from foreground mask.
	vector<CKvVectorFloat> feat_fg;	float *p_v; p_v  = zz_vec3.vp();
	for(int i=0; i<ww*hh; i++){
		if(p_fg[i]){
			for(int k=0; k<3; k++)	p_v[k] = (float)p_img[i + k*ww*hh];
			feat_fg.push_back(zz_vec3);
		}
	}
	// trains global foreground model.
	zz_color_model_obj.c_Create(1,5,KV_COLOR_MODEL_RGB,true);
	zz_color_seg.i_Initialize(true);

	zz_color_seg.cgmm_Clustering_using_Gaussian_Mixture_Model(
		feat_fg,
		0,
		&zz_color_model_obj);

	// =======================================================
	// Set lambda and L.
	// =======================================================	
	zz_lambda = 50.0f;
	zz_L = 8*zz_lambda + 1;

	// =======================================================
	// Compute beta.
	// =======================================================	
	beta = 0.0f;	cnt_edge = 0;
	// 8-neighborhood.
	for(int j = 0; j<hh; j++){
		for(int i = 0; i<ww; i++){

			tidx = j*ww + i;

			if(i<ww - 1){
				for(int k=0; k<3; k++)	beta += SQUARE(p_img[tidx + k*step] - p_img[tidx + 1 + k*step]);		// right.
				cnt_edge++;
			}
			if(i<ww - 1 && j>0){
				for(int k=0; k<3; k++)	beta += SQUARE(p_img[tidx + k*step] - p_img[tidx + 1 - ww + k*step]);	// upright.
				cnt_edge++;
			}
			if(j>0){
				for(int k=0; k<3; k++)	beta += SQUARE(p_img[tidx + k*step] - p_img[tidx - ww + k*step]);		// up.				
				cnt_edge++;
			}
			if(i>0 && j>0){
				for(int k=0; k<3; k++)	beta += SQUARE(p_img[tidx + k*step] - p_img[tidx - 1 - ww + k*step]);	// upleft.				
				cnt_edge++;
			}

		}
	}
	zz_beta = float(cnt_edge)/(2.0f*beta);


	// =======================================================
	// Do iterative optimization.
	// =======================================================
	// Initialize graph.
	num_node = ww*hh;	num_edge = cnt_edge;
	max_iter = 1;
	for(int num_iter=0; num_iter<max_iter; num_iter++){

		// Initialize Graph.
		g = new GraphType(num_node, num_edge);
		g->add_node(num_node);


		// =======================================================
		// Compute data term.
		// SOURCE -> background | SINK -> object
		// Case 1: object		-> T(source): 0 | S(sink): K
		// Case 2: background	-> T(source): K | S(sink): 0
		// Case 3: undetermined 
		// -> T(source): dataCost(background) | S(sink): dataCost(object)
		// =======================================================
		for(int j = 0; j<hh; j++){
			for(int i = 0; i<ww; i++){

				tidx = j*ww + i;

				if(p_fg[tidx]){				// the pixel is in foreground.
				//if(p_mask_obj[tidx]){				// the pixel is in foreground.
					g->add_tweights(tidx, 0.0f, zz_L);	// Graph cut paper....?
					//g->add_tweights(tidx, K, 0.0f);		// -> OpenCV version.
				}
				else if(p_bg[tidx]){		// the pixel is in background.
				//else if(p_mask_obj[tidx]){		// the pixel is in background.
					g->add_tweights(tidx, zz_L, 0.0f);	// Graph cut paper....?
					//g->add_tweights(tidx, 0.0f, K);		// -> OpenCV version.
				}
				else{						// the pixel is undetermined.				
					//g->add_tweights(tidx, dataCost(tidx, 1), dataCost(tidx, 0));					
					g->add_tweights(tidx, 
						dataCostWithTrainedBackground(tidx, 1), 
						dataCostWithTrainedBackground(tidx, 0));					
				}

			}
		}

		// =======================================================
		// Compute smoothness term.
		// =======================================================
		// 8-neighborhood.
 		for(int j = 0; j<hh; j++){
 			for(int i = 0; i<ww; i++){
 
 				tidx = j*ww + i;
 
 				if(i<ww - 1){
 					tsmooth = smoothCost(tidx, tidx + 1, 0, 1);		// right.
 					g->add_edge(tidx, tidx + 1, tsmooth, tsmooth);
 				}
 				if(i<ww - 1 && j>0){	
 					tsmooth = smoothCost(tidx, tidx + 1 - ww, 0, 1);	// upright.
 					g->add_edge(tidx, tidx + 1 - ww, tsmooth, tsmooth);
 				}
 				if(j>0){
 					tsmooth = smoothCost(tidx, tidx - ww, 0, 1);		// up.
 					g->add_edge(tidx, tidx - ww, tsmooth, tsmooth);
 				}
 				if(i>0 && j>0){
 					tsmooth = smoothCost(tidx, tidx - 1 - ww, 0, 1);	// upleft.
 					g->add_edge(tidx, tidx - 1 - ww, tsmooth, tsmooth);
 				}
 
 			}
 		}

		// Optimize labels.
		int flow = g->maxflow();
		
		// update segmentation results.	
		p_timg = timg.cp_Copy(&in_img)[0];
		for(tidx = 0; tidx<ww*hh; tidx++){			
			if(!(p_mask_obj[tidx] = g->what_segment(tidx)))	
				for(int n=0; n<3; n++)	p_timg[tidx + ww*hh*n] = (unsigned char)0;
		}

// 		sc.s_d_Display(&timg);
// 		if(!Kv_Printf("[#%d iter]Flow = %d\n", num_iter, flow))	exit(0);


		delete g;
	} 	
}

//********************************************************************************************
void LCKvYooji_Object_Extractor::ugmmp_Update_GMM_Parameters(
	CKvMatrixUcharRgb &in_img,
	CKvMatrixBool &in_mask_fg,
	CKvYooji_ColorModel *io_gmm)
//********************************************************************************************
{
	vector<vector<CKvVectorFloat>> *set_of_features;	
	vector<int> num_feats;

	float prob_w, prob_max;
	int num_gmm, num_class;
	int ww, hh, step_sz, vec_sz, idx_max, idx_class;

	float *p_fv, *p_mv, *p_cmv, *p_cmiv, *p_wv;
	unsigned char *p_img;
	bool *p_mask_fg;

	p_mask_fg = in_mask_fg.mps(ww, hh)[0];
	p_img = in_img.vp();

	step_sz = ww*hh;	
	num_gmm = (*io_gmm).gnm_Get_Number_of_Models();
	num_class = (*io_gmm).gnc_Get_Number_of_Classes();
	
	num_feats.resize(num_class);
	set_of_features = new vector<vector<CKvVectorFloat>>[num_class];
	for(int k=0; k<num_class; k++)	(set_of_features[k]).resize(num_gmm);

	p_fv = zz_vec3.vp();

	// Assigns GMM components to pixels.
	for(int i=0; i<step_sz; i++){

		// Get RGB features of each pixel.
		for(int k=0; k<3; k++)	p_fv[k] = (float)p_img[i + step_sz*k];
		// Find GMM component which gives maximum likelihood.
		if(!p_mask_fg[i])	idx_class = 0;	// if pixel is background. (label 0)		
		else				idx_class = 1;	// if pixel is foreground. (label 1)

		prob_max = -100000.0f;
		for(int k = 0; k<num_gmm; k++){
			prob_w = (*io_gmm).gpd_Get_Probability_Density(&zz_vec3, idx_class, k);
			if(prob_w>prob_max){
				prob_max = prob_w;
				idx_max = k;
			}
		}
		// Update set of features.
		(set_of_features[idx_class])[idx_max].push_back(zz_vec3);	
		
	}

	// Compute mean values of each GMM component.
	for(int k=0; k<num_class; k++){
		for(int i=0; i<num_gmm; i++){

			// Get mean vector pointer of current GMM component.
			p_mv = (*io_gmm).gpmvs_Get_Pointer_of_Mean_Vector_Set(k)[i].vp();
			for(int kk=0; kk<3; kk++)	p_mv[kk] = 0.0f;
			// Compute mean vector of RGB colors.
			vec_sz = (set_of_features[k])[i].size();
			for(int n=0; n<vec_sz; n++){
				// Get a feature.
				p_fv = (set_of_features[k])[i][n].vp();
				for(int kk=0; kk<3; kk++)	p_mv[kk] += p_fv[kk];
			}
			for(int kk=0; kk<3; kk++)	p_mv[kk] /= (float)vec_sz;
			// Compute covariance matrix diagonal.
			p_cmv = (*io_gmm).gpcmds_Get_Pointer_of_Covariance_Matrix_Diagonal_Set(k)[i].vp();
			for(int n = 0; n<vec_sz; n++){
				// Get a feature.
				p_fv = (set_of_features[k])[i][n].vp();
				for(int kk = 0; kk<3; kk++)	p_cmv[kk] += SQUARE(p_mv[kk] - p_fv[kk]);
			}
			for(int kk=0; kk<3; kk++)	p_cmv[kk] /= (float)vec_sz;
			// Compute covariance matrix diagonal inverse.
			p_cmiv = (*io_gmm).gpcmids_Get_Pointer_of_Covariance_Matrix_Inverse_Diagonal_Set(k)[i].vp();
			for(int kk=0; kk<3; kk++)	p_cmiv[kk] = 1.0f/p_cmv[kk];
		}
	}

	// Compute weights.	
	for(int k=0; k<num_class; k++){
		num_feats[k] = 0;
		p_wv = (*io_gmm).gpmw_Get_Pointer_of_Model_Weights(k);
		for(int i=0; i<num_gmm; i++){			
			p_wv[i] = (float)(set_of_features[k])[i].size()/(float)num_feats[k];			
		}		
	}
	
	// Release.
	for(int k = 0; k<num_class; k++){
		for(int i = 0; i<num_gmm; i++)	set_of_features[k][i].clear();
		set_of_features[k].clear();
	}
	for(int k = 0; k<num_class; k++)	set_of_features[k].clear();
	num_feats.clear();

	delete[] set_of_features;
}

//********************************************************************************************
void LCKvYooji_Object_Extractor::ipfm_Initialize_Probable_Foreground_Mask(
	CKvMatrixUcharRgb &in_img,
	CKvMatrixBool &in_mask_bg,
	CKvMatrixBool &in_mask_fg,	
	CKvYooji_ColorModel &in_gmm,
	CKvMatrixBool &out_fg_pb)
//********************************************************************************************
{
	float prob_w, prob_max;
	int num_gmm, num_class;
	int ww, hh, step_sz, vec_sz, idx_max, idx_class;

	float *p_fv;
	unsigned char *p_img;
	bool *p_mask_fg, *p_mask_bg, *p_mask_pb;

	p_mask_fg = in_mask_fg.mps(ww, hh)[0];
	p_mask_bg = in_mask_bg.vp();
	p_mask_pb = out_fg_pb.vp();
	p_img = in_img.vp();

	step_sz = ww*hh;
	num_gmm = in_gmm.gnm_Get_Number_of_Models();
	num_class = in_gmm.gnc_Get_Number_of_Classes();


	p_fv = zz_vec3.vp();

	// ======================================================
	// Assigns GMM components to pixels.
	// ======================================================
	for(int i = 0; i<step_sz; i++){

		if(p_mask_fg[i])	 {	p_mask_pb[i] = true;	continue;	}
		else if(p_mask_bg[i]){	p_mask_pb[i] = false;	continue;	}

		// Get RGB features of each pixel.
		for(int k = 0; k<3; k++)	p_fv[k] = (float)p_img[i + step_sz*k];
		// Classify input pixels using input GMM.
		idx_max = zz_color_seg.cfgmm_Classify_A_Feature_using_Gaussian_Mixture_Model(
			zz_vec3,
			in_gmm,
			num_class);
		
		// Set results.
		p_mask_pb[i] = (bool)idx_max;
	}
}
// 
// void LCKvYooji_Object_Extractor::z_cdt_Compute_Data_Term(
// 		CKvMatrixUcharRgb &in_img,
// 		CKvMatrixBool &in_bg,
// 		CKvMatrixBool &in_fg,
// 		MRF::CostVal *out_data_cost)
// {
// 	unsigned char *p_img;
// 	bool *p_bg, *p_fg;
// 	int ww, hh, tidx;
// 
// 	p_img = in_img.mps(ww, hh)[0];		zz_img.cp_Copy(&in_img);
// 
// 	if(out_data_cost)	delete []out_data_cost;
// 	out_data_cost = new MRF::CostVal[ww*hh*2];
// 
// 	printf("out_data_cost: %d (%dx%d)\n", out_data_cost, ww, hh);
// 
// 	p_bg = in_bg.vp();
// 	p_fg = in_fg.vp();
// 
// 	for(int j = 0; j<hh; j++){
// 		for(int i = 0; i<ww; i++){
// 
// 			tidx = j*ww + i;
// 
// 			if(p_bg[tidx]){				// the pixel is in background.
// 				out_data_cost[(tidx)*2 + 0] = 0.0;				// Background T-link.
// 				out_data_cost[(tidx)*2 + 1] = zz_K;				// Foreground T-link.
// 			}
// 			else if(p_fg[tidx]){		// the pixel is in foreground.
// 				out_data_cost[(tidx)*2 + 0] = zz_K;			// Background T-link.
// 				out_data_cost[(tidx)*2 + 1] = 0.0;				// Foreground T-link.
// 			}
// 			else{																																							// the pixel is undetermined.
// 				out_data_cost[(tidx)*2 + 0] = dataCost(tidx, 0);
// 				out_data_cost[(tidx)*2 + 1] = dataCost(tidx, 1);
// 			}
// 
// 		}
// 	}
// }
// 
// MRF::CostVal* LCKvYooji_Object_Extractor::z_cdt_Compute_Data_Term(
// 		CKvMatrixUcharRgb &in_img,
// 		CKvMatrixBool &in_bg,
// 		CKvMatrixBool &in_fg)
// {
// 	unsigned char *p_img;
// 	bool *p_bg, *p_fg;
// 	int ww, hh, tidx;
// 
// 	p_img = in_img.mps(ww, hh)[0];		zz_img.cp_Copy(&in_img);
// 
// 	MRF::CostVal *out_data_cost = new MRF::CostVal[ww*hh*2];
// 
// 	printf("out_data_cost: %d (%dx%d)\n", out_data_cost, ww, hh);
// 
// 	p_bg = in_bg.vp();
// 	p_fg = in_fg.vp();
// 
// 	for(int j = 0; j<hh; j++){
// 		for(int i = 0; i<ww; i++){
// 
// 			tidx = j*ww + i;
// 
// 			if(p_bg[tidx]){				// the pixel is in background.
// 				out_data_cost[(tidx)*2 + 0] = 0.0;				// Background T-link.
// 				out_data_cost[(tidx)*2 + 1] = zz_K;				// Foreground T-link.
// 			}
// 			else if(p_fg[tidx]){		// the pixel is in foreground.
// 				out_data_cost[(tidx)*2 + 0] = zz_K;			// Background T-link.
// 				out_data_cost[(tidx)*2 + 1] = 0.0;				// Foreground T-link.
// 			}
// 			else{																																							// the pixel is undetermined.
// 				out_data_cost[(tidx)*2 + 0] = dataCost(tidx, 0);
// 				out_data_cost[(tidx)*2 + 1] = dataCost(tidx, 1);
// 			}
// 
// 		}
// 	}
// 
// 	return out_data_cost;
// }


//********************************************************************************************
void LCKvYooji_Object_Extractor::ipfm_Initialize_Probable_Foreground_Mask(
	CKvMatrixUcharRgb &in_img,
	CKvMatrixBool &io_mask_bg,
	CKvMatrixBool &io_mask_fg,
	CKvYooji_ColorModel &in_gmm_back_pixel,
	CKvYooji_ColorModel &in_gmm_obj_global,
	CKvMatrixBool &out_mask_fg_pb)
//********************************************************************************************
{
	float mahal_back_pixel, mahal_obj;
	int num_gmm, num_class;
	int ww, hh, step_sz, vec_sz;

	float *p_fv;
	unsigned char *p_img;
	bool *p_mask_fg, *p_mask_bg, *p_mask_pb;	

	p_mask_fg = io_mask_fg.mps(ww, hh)[0];
	p_mask_bg = io_mask_bg.vp();
	p_mask_pb = out_mask_fg_pb.vp();
	p_img = in_img.vp();

	step_sz = ww*hh;


	p_fv = zz_vec3.vp();

	// ======================================================
	// Do initial background subtraction using pre-trained models based on Mahalanobis distance.
	// ======================================================
	for(int i = 0; i<step_sz; i++){

		float min_dist = 10e15f;
		int n_gauss = in_gmm_obj_global.gnm_Get_Number_of_Models();


		if(p_mask_fg[i])	 {	p_mask_pb[i] = true;	continue;	}
		else if(p_mask_bg[i]){	p_mask_pb[i] = false;	continue;	}

		// Get RGB features of each pixel.
		for(int k = 0; k<3; k++)	p_fv[k] = (float)p_img[i + step_sz*k];
		// Compute probability of input pixels using input GMM.
		mahal_back_pixel = in_gmm_back_pixel.gmd_Get_Mahalanobis_Distance(&zz_vec3, i, 0);
		
		// if current pixel is exact background.
		// 0.5398 at z = 0.1.
		// 0.6179 at z = 0.3.
		// 0.5199 at z = 0.05.
		if(mahal_back_pixel < 0.05f){	
			p_mask_pb[i] = false;
			p_mask_bg[i] = true; 
			p_mask_fg[i] = false; 
		}
		// if current pixel is exact foreground.
		// 0.9032 at z = 1.3.
		// 0.9554 at z = 1.7.
		// 0.9773 at z = 2.0.
		else if(mahal_back_pixel > 3.0f){ 
			p_mask_pb[i] = true; 
			p_mask_fg[i] = true; 
			p_mask_bg[i] = false;
		}
// 		else{
// 			// in case of GMM, there are multiple Mahalanobis distances.
// 			// find minimum Mahalanobis distance and thresholding.
// 			min_dist = 10e15f;
// 			for(int k=0; k<n_gauss; k++){ 				
// 				mahal_obj = in_gmm_obj_global.gmd_Get_Mahalanobis_Distance(&zz_vec3, 0, k, true);
// 				if(mahal_obj < min_dist) min_dist = mahal_obj;
// 			}
// 
//  			if(min_dist < 0.05f){ p_mask_pb[i] = true; p_mask_fg[i] = true; }
// 
// 			printf("mahal_dist: %f %f\n", mahal_back_pixel,	min_dist);
// 		}		
	}

	eo_Erode_Object(&io_mask_fg, 1, 1, false);
}
// Graph cut.

float LCKvYooji_Object_Extractor::dataCostWithTrainedBackground(int pix,int label)
{
	float *p_v;
	unsigned char *p_img;

	float out_val,cost_back,cost_label;
	float alpha;
	int ww,hh;
	int x,y,step_sz;

	//////////////////////////////////////////////////////////////////////////
	alpha = 0.5f;
	//////////////////////////////////////////////////////////////////////////

	p_img = zz_img.mps(ww,hh)[0];
	step_sz = ww*hh;

	p_v = zz_vec3.vp();

	p_v[0] = (float)p_img[pix];					//	R
	p_v[1] = (float)p_img[pix + step_sz];		//	G
	p_v[2] = (float)p_img[pix + step_sz*2];		//	B	

	out_val = 0.0f;
	// for trained background.
	if(label == 0){
		// pixel-wise background model.
		cost_back = zz_color_model_back_pixel.gpd_Get_Probability_Density(&zz_vec3,pix,0);
		// global background model.
		cost_label = zz_color_model_back.gpdw_Get_Probability_Density_Weighted(&zz_vec3,0);

		//printf("b %f\n",zz_color_model_obj.gpdw_Get_Probability_Density_Weighted(&zz_vec3,0));

		out_val = -log(max(0.0000001f,(1.0f - alpha)*cost_back + alpha*cost_label));
	} else{
		// global foreground model.
		cost_label = -log(max(0.0000001f,zz_color_model_obj.gpdw_Get_Probability_Density_Weighted(&zz_vec3,0)));

		//printf("%f\n", zz_color_model_obj.gpdw_Get_Probability_Density_Weighted(&zz_vec3, 0));

		out_val = cost_label;
	}


	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// Data term 도 full covariance 적용되게 고쳐야 함!!!
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	//out_val = -log(zz_color_model.gpdw_Get_Probability_Density_Weighted(&zz_vec3, label, true));
	//cost_label = -log(max(0.0000001f, zz_color_model_inter.gpdw_Get_Probability_Density_Weighted(&zz_vec3, label, true)));
	//  	for(int k=0; k<zz_color_model.gnm_Get_Number_of_Models(); k++){
	// 
	//  		//out_val += (-log(max(0.01f, p_w[k]*zz_color_model.gpd_Get_Probability_Density(&zz_vec3, label, k, true))));
	// 		out_val += -log(p_w[k]*zz_color_model.gpd_Get_Probability_Density(&zz_vec3, label, k, true));
	// 
	// 	}


	//	printf("[#%d] d : %f (%d, %d)\n", zz_color_model.gnm_Get_Number_of_Models(), out_val, pix, label);

	return out_val;
}


float LCKvYooji_Object_Extractor::dataCost(int pix,int label)
{
	float *p_v,*p_w;
	unsigned char *p_img;

	float out_val;
	int ww,hh;
	int x,y,step_sz;

	p_img = zz_img.mps(ww,hh)[0];
	step_sz = ww*hh;

	p_w = zz_color_model_inter.gpmw_Get_Pointer_of_Model_Weights(label);
	p_v = zz_vec3.vp();

	p_v[0] = (float)p_img[pix];					//	R
	p_v[1] = (float)p_img[pix + step_sz];		//	G
	p_v[2] = (float)p_img[pix + step_sz*2];		//	B	

	out_val = 0.0f;
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// Data term 도 full covariance 적용되게 고쳐야 함!!!
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	//out_val = -log(zz_color_model.gpdw_Get_Probability_Density_Weighted(&zz_vec3, label, true));
	out_val = -log(max(0.0000001f,zz_color_model_inter.gpdw_Get_Probability_Density_Weighted(&zz_vec3,label)));
	//  	for(int k=0; k<zz_color_model.gnm_Get_Number_of_Models(); k++){
	// 
	//  		//out_val += (-log(max(0.01f, p_w[k]*zz_color_model.gpd_Get_Probability_Density(&zz_vec3, label, k, true))));
	// 		out_val += -log(p_w[k]*zz_color_model.gpd_Get_Probability_Density(&zz_vec3, label, k, true));
	// 
	// 	}

	//	printf("[#%d] d : %f (%d, %d)\n", zz_color_model.gnm_Get_Number_of_Models(), out_val, pix, label);

	return out_val;
}

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// 추후에 미리 계산해 놓을 필요가 있음!!!!!!!!!
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float LCKvYooji_Object_Extractor::smoothCost(int pix1,int pix2,int label1,int label2)
{
	static float _sample1[CHANNEL_RGB_COLOR_FOR_GMM],_sample2[CHANNEL_RGB_COLOR_FOR_GMM];

	unsigned char *p_img;
	float out_val,dist,norm_color;
	int ww,hh,step_sz;
	int x1,x2,y1,y2;

	if(label1 == label2){
		out_val = 0.0f;
	} else{
		p_img = zz_img.mps(ww,hh)[0];
		step_sz = ww*hh;

		x1 = pix1%ww; y1 = pix1/ww;

		_sample1[0] = p_img[pix1];						//	R
		_sample1[1] = p_img[pix1 + step_sz];			//	G
		_sample1[2] = p_img[pix1 + step_sz*2];			//	B

		x2 = pix2%ww; y2 = pix2/ww;

		_sample2[0] = p_img[pix2];						//	R
		_sample2[1] = p_img[pix2 + step_sz];			//	G
		_sample2[2] = p_img[pix2 + step_sz*2];			//	B

		dist = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
		norm_color = SQUARE(_sample1[0]-_sample2[0])
			+ SQUARE(_sample1[1]-_sample2[1])
			+ SQUARE(_sample1[2]-_sample2[2]);

		out_val = zz_lambda*exp(-zz_beta*norm_color)/dist;
	}



	return out_val;
}