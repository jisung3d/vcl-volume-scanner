/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_matrix_rgbd.cpp 
/////////////////////////////////////////////////////////////////////////////////////////////

#include "../../../stdafx.h"
#include "../../_yooji_2016_object_scanning.h"

/////////////////////////////////////////////////////////////////////////////////////////////
// CKvYooji_MatrixRgbD 
/////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
CKvYooji_MatrixRgbD::CKvYooji_MatrixRgbD(void)
//********************************************************************************************
{
	zz_classname="CKvYooji_MatrixRgbD";

	zz_flag_silhouette = false;
	zz_flag_extrins = false;
}

//********************************************************************************************
CKvYooji_MatrixRgbD::~CKvYooji_MatrixRgbD(void)
//********************************************************************************************
{
}

//***********************************************************************************************************************
CKvYooji_MatrixRgbD::CKvYooji_MatrixRgbD(CKvYooji_MatrixRgbD &a)
//***********************************************************************************************************************
{
	copy(&a);
}


//***********************************************************************************************************************
//***********************************************************************************************************************
//***********************************************************************************************************************
//***********************************************************************************************************************
//***********************************************************************************************************************
void CKvYooji_MatrixRgbD::copy( CKvYooji_MatrixRgbD *a)
//***********************************************************************************************************************
{
	zz_image_rgb.cp_Copy(&a->zz_image_rgb);

	zz_map_depth.cp_Copy(&a->zz_map_depth);	
	zz_map_depth_on_rgb.cp_Copy(&a->zz_map_depth_on_rgb);
	zz_map_depth_filt.cp_Copy(&a->zz_map_depth_filt);
	zz_map_depth_filt_on_rgb.cp_Copy(&a->zz_map_depth_filt_on_rgb);
	
	zz_image_sil.cp_Copy(&a->zz_image_sil);
	zz_edge_map.cp_Copy(&a->zz_edge_map);

	zz_roi_obj = a->zz_roi_obj;

	zz_pmat_d.cp_Copy(&a->zz_pmat_d);
	zz_pmat_rgb.cp_Copy(&a->zz_pmat_rgb);

	zz_intrin_d.copy(&a->zz_intrin_d);
	zz_intrin_rgb.copy(&a->zz_intrin_rgb);
	zz_extrin_d.copy(&a->zz_extrin_d);
	zz_extrin_rgb.copy(&a->zz_extrin_rgb);
	zz_extrin_rgb_wrt_d.copy(&a->zz_extrin_rgb_wrt_d);
	zz_extrin_d_wrt_rgb.copy(&a->zz_extrin_d_wrt_rgb);
	
	zz_flag_silhouette = a->zz_flag_silhouette;
	zz_flag_extrins = a->zz_flag_extrins;

}

//***********************************************************************************************************************
void CKvYooji_MatrixRgbD::ms(
	int &out_width,int &out_height)
//***********************************************************************************************************************
{
	out_width = zz_map_depth.mw();	out_height = zz_map_depth.mh();
}

//***********************************************************************************************************************
void CKvYooji_MatrixRgbD::ms_Matrix_Width_Height(
	int &out_width,int &out_height)
//***********************************************************************************************************************
{
	out_width = zz_map_depth.mw();	out_height = zz_map_depth.mh();
}

//***********************************************************************************************************************
void CKvYooji_MatrixRgbD::set_camera_information(
	CKvPmatrix3D &in_pmat_depth,
	CKvPmatrix3D &in_pmat_rgb)
//***********************************************************************************************************************
{
	zz_pmat_d.cp_Copy(&in_pmat_depth);				zz_pmat_rgb.cp_Copy(&in_pmat_rgb);
	zz_intrin_d.set_from_P_matrix(&zz_pmat_d);	zz_intrin_rgb.set_from_P_matrix(&zz_pmat_rgb);
	zz_extrin_d.set_from_P_matrix(&zz_pmat_d);	zz_extrin_rgb.set_from_P_matrix(&zz_pmat_rgb);

	zz_extrin_rgb_wrt_d.get_pose_relative(zz_extrin_d, zz_extrin_rgb, zz_extrin_rgb_wrt_d);
	zz_extrin_d_wrt_rgb.get_pose_relative(zz_extrin_rgb, zz_extrin_d, zz_extrin_d_wrt_rgb);
}

//***********************************************************************************************************************
bool CKvYooji_MatrixRgbD::convert_pixel_coord(
	CKvPointf &in_point_depth,
	float in_depth,
	CKvPointf &out_point_rgb)
//***********************************************************************************************************************
{
	CKvPoint3Df tp3d, p3d_rgb;

	zz_intrin_d.back_project(in_point_depth, in_depth, tp3d);	
	zz_extrin_rgb_wrt_d.transform(tp3d, p3d_rgb);	
	zz_intrin_rgb.project(p3d_rgb, out_point_rgb);

	//d_pm_Printf_Matrix(zz_extrin_rgb_wrt_d.mp_transform()->vp(), 4, 4, "TTT");

	if(out_point_rgb.x < 0.0f || out_point_rgb.x > (float)(zz_image_rgb.mw() - 1)
		|| out_point_rgb.y < 0.0f || out_point_rgb.y > (float)(zz_image_rgb.mh() - 1))
		return false;
	else
		return true;
}

//
////***********************************************************************************************************************
//CKvMatrixUcharRgb* CKvYooji_MatrixRgbD::prgbi_Pointer_of_RGB_Image()
////***********************************************************************************************************************
//{
//	return &zz_image_rgb;
//}
//
////***********************************************************************************************************************
//CKvMatrixFloat* CKvYooji_MatrixRgbD::pdi_Pointer_of_Depth_Image()
////***********************************************************************************************************************
//{
//	return &zz_image_depth;
//}
//
////***********************************************************************************************************************
//CKvMatrixBool* CKvYooji_MatrixRgbD::ps_Pointer_of_Silhouette()
////***********************************************************************************************************************
//{
//	return &zz_image_sil;
//}
//
//
////***********************************************************************************************************************
//CKvPmatrix3D* CKvYooji_MatrixRgbD::pprgb_Pointer_of_Pmatrix_Rgb()
////***********************************************************************************************************************
//{
//	return &zz_pmat_rgb;
//}
//
////***********************************************************************************************************************
//CKvPmatrix3D* CKvYooji_MatrixRgbD::ppd_Pointer_of_Pmatrix_Depth()
////***********************************************************************************************************************
//{
//	return &zz_pmat_d;
//}