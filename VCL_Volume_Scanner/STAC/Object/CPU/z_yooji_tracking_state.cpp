/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_tracking_state.cpp 
/////////////////////////////////////////////////////////////////////////////////////////////

#include "../../../stdafx.h"
#include "../../_yooji_2016_object_scanning.h"

/////////////////////////////////////////////////////////////////////////////////////////////
// CKvYooji_Tracking_State 
/////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
CKvYooji_Tracking_State::CKvYooji_Tracking_State()
//********************************************************************************************
{
	zz_classname="CKvYooji_TrackingState";

}

//********************************************************************************************
CKvYooji_Tracking_State::~CKvYooji_Tracking_State()
//********************************************************************************************
{
	zz_cnt_sphere = 0;
}

//***********************************************************************************************************************
CKvYooji_Tracking_State::CKvYooji_Tracking_State(CKvYooji_Tracking_State &a)
//***********************************************************************************************************************
{
	copy(&a);
}


//***********************************************************************************************************************
//***********************************************************************************************************************
//***********************************************************************************************************************
//***********************************************************************************************************************
//***********************************************************************************************************************
void CKvYooji_Tracking_State::copy( CKvYooji_Tracking_State *a)
//***********************************************************************************************************************
{
	zz_pmat_d.cp_Copy(&a->zz_pmat_d);
	zz_pmat_rgb.cp_Copy(&a->zz_pmat_rgb);

	zz_intrinsics.copy(&a->zz_intrinsics);
	zz_intrinsics_rgb.copy(&a->zz_intrinsics_rgb);
	zz_transform_glob_to_cam.copy(&a->zz_transform_glob_to_cam);
	zz_transform_glob_to_cam_rgb.copy(&a->zz_transform_glob_to_cam_rgb);

	zz_rendered_depth.cp_Copy(&a->zz_rendered_depth);
	zz_rendered_weight.cp_Copy(&a->zz_rendered_weight);
	zz_rendered_normals.cp_Copy(&a->zz_rendered_normals);

	zz_image_of_rendered_texture.cp_Copy(&a->zz_image_of_rendered_texture);
	zz_image_of_rendered_depth.cp_Copy(&a->zz_image_of_rendered_depth);
	zz_image_of_rendered_normals.cp_Copy(&a->zz_image_of_rendered_normals);
	                           
	zz_image_of_texture.cp_Copy(&a->zz_image_of_texture);
	zz_image_of_texture_gray.cp_Copy(&a->zz_image_of_texture_gray);
	zz_map_edge.cp_Copy(&a->zz_map_edge);
	zz_image_of_rendered_depth.cp_Copy(&a->zz_image_of_rendered_depth);
	zz_image_of_rendered_normals.cp_Copy(&a->zz_image_of_rendered_normals);

	zz_roi_obj = a->zz_roi_obj;

	//zz_cnt_sphere = a->zz_cnt_sphere;

// 	zz_pyram_intrin_d.resize(in_set_of_elements->zz_pyram_intrin_d.size());
// 	for(int i=0; i<zz_pyram_intrin_d.size(); i++)
// 		zz_pyram_intrin_d[i].cp_Copy(&in_set_of_elements->zz_pyram_intrin_d[i]);
}

//********************************************************************************************
void CKvYooji_Tracking_State::initialize(int in_width, int in_height, int in_level_pyram)
//********************************************************************************************
{
	if(in_width<=0 || in_height<=0)	gerr("1");
	
// 	zz_pyram_img_gray.c_Create(in_height, in_width, (UCHAR)0);
// 	zz_pyram_edge_map.c_Create(in_height, in_width, 3);

	zz_rendered_depth.c_Create(in_height, in_width, 0.0f);	
	zz_rendered_weight.c_Create(in_height, in_width, 0.0f);	
	zz_rendered_normals.c_Create(in_height, in_width);
	
	zz_image_of_rendered_texture.c_Create(in_height, in_width);
	zz_image_of_rendered_depth.c_Create(in_height, in_width, (UCHAR)0);
	zz_image_of_rendered_normals.c_Create(in_height, in_width, (UCHAR)0);

// 	zz_pyram_sil.c_Create(in_width, in_height, in_level_pyram);
// 	zz_pyram_img_gray.c_Create(in_width, in_height, in_level_pyram);
// 	zz_pyram_edge_map.c_Create(in_width, in_height, in_level_pyram);
// 
// 	zz_pyram_depth.c_Create(in_width,in_height,in_level_pyram);
// 	zz_pyram_p3d.c_Create(in_width,in_height,in_level_pyram);
// 	zz_pyram_intrin.c_Create(in_level_pyram);

	// optical flow.
// 	int num_block_x, num_block_y;
// //    	num_block_x = iDivUp(in_width, KV_FLOW_BLOCK_SIZE);
// //    	num_block_y = iDivUp(in_height, KV_FLOW_BLOCK_SIZE);
// 
//  	num_block_x = in_width;
//  	num_block_y = in_height;
// 
// 	zz_set_of_flows.c_Create(num_block_y, num_block_x);
// 
// 	zz_map_flows.c_Create(in_height, in_width, Kv_Rgb(0, 0, 0));

	zz_cnt_sphere = 0;

	return;
error:
	zpme("i_Initialize");
}

////********************************************************************************************
//void CKvYooji_Tracking_State::i_Initialize(
//	int in_width, int in_height,
//	int in_width_rgb, int in_height_rgb)
////********************************************************************************************
//{
//	if(in_width<=0 || in_height<=0 || in_width_rgb<=0 || in_height_rgb<=0)	gerr("1");
//	
//// 	zz_pyram_img_gray.c_Create(in_height, in_width, (UCHAR)0);
//// 	zz_pyram_edge_map.c_Create(in_height, in_width, 3);
//
//
//	zz_rendered_depth.c_Create(in_height_rgb, in_width_rgb, 0.0f);	
//	zz_rendered_weight.c_Create(in_height_rgb, in_width_rgb, 0.0f);	
//	zz_rendered_normals.c_Create(in_height_rgb, in_width_rgb);
//	
//	zz_image_of_rendered_depth.c_Create(in_height_rgb, in_width_rgb, (UCHAR)0);
//	zz_image_of_rendered_normals.c_Create(in_height_rgb, in_width_rgb, (UCHAR)0);
//
//	return;
//error:
//	zpme("i_Initialize");
//}

//********************************************************************************************
void CKvYooji_Tracking_State::import_P_matrix(
	CKvPmatrix3D *in_pmat,
	CKvPmatrix3D *in_pmat_rgb)
//********************************************************************************************
{	
	//zz_hmat_glob_to_cam.c_Create(in_pmat->mphc_Matrix_Pointer_of_H_matrix_for_Canonical());	

	zz_intrinsics.set_from_P_matrix(in_pmat);
	zz_transform_glob_to_cam.set_from_P_matrix(in_pmat);
	zz_pmat_d.cp_Copy(in_pmat);

	if(in_pmat_rgb){
		zz_intrinsics_rgb.set_from_P_matrix(in_pmat_rgb);
		zz_transform_glob_to_cam_rgb.set_from_P_matrix(in_pmat_rgb);
		zz_pmat_rgb.cp_Copy(in_pmat_rgb);
	}

	return;
}

//********************************************************************************************
void CKvYooji_Tracking_State::compute_P_matrix(
		CKvYooji_Intrinsics *in_intrinsics,
		CKvYooji_Extrinsics *in_extrinsics,
		CKvPmatrix3D *out_pmatrix)
//********************************************************************************************
{
	CKvMatrixFloat P, KR, KT;
	CKvMatrixFloat K;
	CKvMatrixFloat R, T;

	float *p_K, *p_R, *p_T, *p_KR, *p_KT;

	P.c_Create(3, 4, 0.0f);

	K.cp_Copy((*in_intrinsics).mp());
	R.cp_Copy((*in_extrinsics).mp_rotation());
	T.cp_Copy((*in_extrinsics).mp_translation());

	p_K = K.vp();	p_R = R.vp();	p_T = T.vp();
	p_KR = KR.c_Create(3, 3, 0.0f)[0];
	p_KT = KT.c_Create(3, 1, 0.0f)[0];

	d_mms_Multiply_Matrix_Square(p_K, p_R, 3, p_KR);
	d_mmsv_Multiply_Matrix_Square_Vector(p_K, p_T, 3, p_KT);
	
	P.sb_Set_Block(0, 0, &KR);
	P.sb_Set_Block(3, 0, &KT);

	(*out_pmatrix).i_Import(&P);

}