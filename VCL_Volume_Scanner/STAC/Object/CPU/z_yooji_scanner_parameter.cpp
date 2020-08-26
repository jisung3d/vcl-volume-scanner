/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_object_scanning.cpp
/////////////////////////////////////////////////////////////////////////////////////////////

#include "../../../stdafx.h"
#include "../../_yooji_2016_object_scanning.h"

/////////////////////////////////////////////////////////////////////////////////////////////
// CKvYooji_Scanner_Parameter 
/////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
CKvYooji_Scanner_Parameter::CKvYooji_Scanner_Parameter(void)
//********************************************************************************************
{
	zz_classname="CKvYooji_Scanner_Parameter";	
}

//********************************************************************************************
CKvYooji_Scanner_Parameter::~CKvYooji_Scanner_Parameter(void)
//********************************************************************************************
{
}

//***********************************************************************************************************************
CKvYooji_Scanner_Parameter::CKvYooji_Scanner_Parameter(CKvYooji_Scanner_Parameter &a)
//***********************************************************************************************************************
{
	cp_Copy(&a);
}

//***********************************************************************************************************************
void CKvYooji_Scanner_Parameter::cp_Copy( CKvYooji_Scanner_Parameter *a)
//***********************************************************************************************************************
{
	zz_width = a->zz_width;
	zz_height = a->zz_height;
	zz_depth = a->zz_depth;
	zz_edge_length_of_sub_cube = a->zz_edge_length_of_sub_cube;
	zz_voxel_size_in_meter = a->zz_voxel_size_in_meter;
	zz_org_point_in_world = a->zz_org_point_in_world;

	zz_lev_of_pyramid = a->zz_lev_of_pyramid;
	zz_threshold_distance_ICP = a->zz_threshold_distance_ICP;
	zz_mu = a->zz_mu;
	zz_max_w = a->zz_max_w;
	zz_d_min = a->zz_d_min;
	zz_d_max = a->zz_d_max;
			
}


//***********************************************************************************************************************
void CKvYooji_Scanner_Parameter::sp_Set_Parameters(
		int in_width_of_cube_TSDF,
		int in_height_of_cube_TSDF,
		int in_depth_of_cube_TSDF,
		int in_edge_length_of_sub_cube_in_voxel,
		float in_voxel_size_of_cube_TSDF_in_meter,
		CKvPoint3Df in_origin_in_world,
		int in_lev_of_pyramid,
		float in_threshold_for_distance_used_in_ICP_algorithm, 
		float in_parameter_MU,
		float in_max_number_of_accumulated_depths,
		float in_d_min,
		float in_d_max)
//***********************************************************************************************************************
{
	float f; int a,len;

	len=in_edge_length_of_sub_cube_in_voxel; if((len<4)) gerr("1"); 

	a=in_width_of_cube_TSDF; if((a<8)||(a%len)) gerr("2"); 
	a=in_height_of_cube_TSDF; if((a<8)||(a%len)) gerr("3");
	a=in_depth_of_cube_TSDF; if((a<8)||(a%len)) gerr("4");

	f=in_voxel_size_of_cube_TSDF_in_meter; if(f<=0.0f) gerr("5");

	f=in_threshold_for_distance_used_in_ICP_algorithm; if(f<=0.0f) gerr("6");
	f=in_parameter_MU; if(f<=0.0f) gerr("7");
	f=in_max_number_of_accumulated_depths; if(f<=0.0f) gerr("8");
	f=in_d_min; f=in_d_max; if((in_d_min>=f)||(f<=0.0f)) gerr("10");
	

	zz_width = in_width_of_cube_TSDF;
	zz_height = in_height_of_cube_TSDF;
	zz_depth = in_depth_of_cube_TSDF;
	zz_edge_length_of_sub_cube = in_edge_length_of_sub_cube_in_voxel;
	zz_voxel_size_in_meter = in_voxel_size_of_cube_TSDF_in_meter;
	zz_org_point_in_world = in_origin_in_world;

	zz_lev_of_pyramid = in_lev_of_pyramid;
	zz_threshold_distance_ICP = in_threshold_for_distance_used_in_ICP_algorithm;
	zz_mu = in_parameter_MU;
	zz_max_w = in_max_number_of_accumulated_depths;
	zz_d_min = in_d_min;
	zz_d_max = in_d_max;

	return;
error:
	zpme("sp_Set_Parameters");
}

//***********************************************************************************************************************
void CKvYooji_Scanner_Parameter::gp_Get_Parameters(
	int &out_width_of_cube_TSDF,
	int &out_height_of_cube_TSDF,
	int &out_depth_of_cube_TSDF,
	int &out_edge_length_of_sub_cube_in_voxel,
	float &out_voxel_size_of_cube_TSDF_in_meter,
	CKvPoint3Df &out_origin_in_world,
	int &out_level_of_pyramid,
	float &out_threshold_for_distance_used_in_ICP_algorithm, 
	float &out_parameter_MU,
	float &out_max_number_of_accumulated_depths,
	float &out_d_min,	// 0.3f,
	float &out_d_max)
//***********************************************************************************************************************
{
	out_width_of_cube_TSDF = zz_width;
	out_height_of_cube_TSDF = zz_height;
	out_depth_of_cube_TSDF = zz_depth;
	out_edge_length_of_sub_cube_in_voxel = zz_edge_length_of_sub_cube;
	out_voxel_size_of_cube_TSDF_in_meter = zz_voxel_size_in_meter;
	out_origin_in_world = zz_org_point_in_world;

	out_level_of_pyramid = zz_lev_of_pyramid;
	out_threshold_for_distance_used_in_ICP_algorithm = zz_threshold_distance_ICP;
	out_parameter_MU = zz_mu;
	out_max_number_of_accumulated_depths = zz_max_w;
	out_d_min = zz_d_min;
	out_d_max = zz_d_max;
}

//***********************************************************************************************************************
void CKvYooji_Scanner_Parameter::gpic_Get_Parameters_for_Initializing_Cube(
	int &out_width_of_cube_TSDF,
	int &out_height_of_cube_TSDF,
	int &out_depth_of_cube_TSDF,
	int &out_edge_length_of_sub_cube_in_voxel,
	float &out_voxel_size_of_cube_TSDF_in_meter,
	CKvPoint3Df &out_origin_in_world)
//***********************************************************************************************************************
{
	out_width_of_cube_TSDF = zz_width;
	out_height_of_cube_TSDF = zz_height;
	out_depth_of_cube_TSDF = zz_depth;
	out_edge_length_of_sub_cube_in_voxel = zz_edge_length_of_sub_cube;
	out_voxel_size_of_cube_TSDF_in_meter = zz_voxel_size_in_meter;
	out_origin_in_world = zz_org_point_in_world;
}

//***********************************************************************************************************************
void CKvYooji_Scanner_Parameter::gptd_Get_Parameters_for_Thresholding_Depth(
	float &out_d_min,	// 0.3f,
	float &out_d_max)
//***********************************************************************************************************************
{
	out_d_min = zz_d_min;
	out_d_max = zz_d_max;
}


//***********************************************************************************************************************
void CKvYooji_Scanner_Parameter::gpicp_Get_Parameters_for_ICP_algorithm(
	int &out_lev_of_pyramid,
	float &out_threshold_for_distance_used_in_ICP_algorithm, 
	float &out_parameter_MU,
	float &out_max_number_of_accumulated_depths)
//***********************************************************************************************************************
{
	out_lev_of_pyramid = zz_lev_of_pyramid;
	out_threshold_for_distance_used_in_ICP_algorithm = zz_threshold_distance_ICP;
	out_parameter_MU = zz_mu;
	out_max_number_of_accumulated_depths = zz_max_w;
}