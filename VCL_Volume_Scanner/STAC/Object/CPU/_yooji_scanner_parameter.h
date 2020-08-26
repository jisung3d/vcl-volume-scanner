//********************************************************************************************
//class AFX_EXT_CLASS CKvYooji_Scanner_Parameter : public CKvClass
class CKvYooji_Scanner_Parameter : public CKvClass
//********************************************************************************************
{
public:
	CKvYooji_Scanner_Parameter(void);
	virtual ~CKvYooji_Scanner_Parameter(void);
	CKvYooji_Scanner_Parameter(CKvYooji_Scanner_Parameter &a);

public:
	void cp_Copy( CKvYooji_Scanner_Parameter *in_set_of_elements);

	void sp_Set_Parameters(
		int in_width_of_cube_TSDF,
		int in_height_of_cube_TSDF,
		int in_depth_of_cube_TSDF,
		int in_edge_length_of_sub_cube_in_voxel,
		float in_voxel_size_of_cube_TSDF_in_meter,
		CKvPoint3Df in_origin_in_world,
		int in_level_of_pyramid = 3,
		float in_threshold_for_distance_used_in_ICP_algorithm = KV_TH_DIST_SQ_ICP, //0.1f*0.1f, 
		float in_parameter_MU = KV_MU_TSDF,	// 0.005f,
		float in_max_number_of_accumulated_depths = KV_MAX_W_TSDF,		// 100.0f,
		float in_d_min = KV_MIN_DEPTH,	// 0.3f,
		float in_d_max = KV_MAX_DEPTH);		// 0.7f);	

	void gp_Get_Parameters(
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
		float &out_d_max);		// 0.7f);	

	void gpic_Get_Parameters_for_Initializing_Cube(
		int &out_width_of_cube_TSDF,
		int &out_height_of_cube_TSDF,
		int &out_depth_of_cube_TSDF,
		int &out_edge_length_of_sub_cube_in_voxel,
		float &out_voxel_size_of_cube_TSDF_in_meter,
		CKvPoint3Df &out_origin_in_world);		// 0.7f);	

	void gptd_Get_Parameters_for_Thresholding_Depth(
		float &out_d_min,	// 0.3f,
		float &out_d_max);		// 0.7f);	

	void gpicp_Get_Parameters_for_ICP_algorithm(
		int &out_lev_of_pyramid,
		float &out_threshold_for_distance_used_in_ICP_algorithm, 
		float &out_parameter_MU,
		float &out_max_number_of_accumulated_depths);



protected:
	int zz_width,zz_height,zz_depth;
	int zz_edge_length_of_sub_cube;
	int zz_lev_of_pyramid;
	float zz_voxel_size_in_meter;
	CKvPoint3Df zz_org_point_in_world;

	float zz_threshold_distance_ICP;		// distance threshold used in ICP camera tracker.
	float zz_mu,zz_max_w,zz_d_min,zz_d_max;	// parameters for TSDF representation.
};