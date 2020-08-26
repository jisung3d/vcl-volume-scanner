//********************************************************************************************
//class AFX_EXT_CLASS CKvYooji_Cube_TSDF_Float : public CKvClass
class CKvYooji_Cube_TSDF_Float : public CKvClass
//********************************************************************************************
{
public:
	CKvYooji_Cube_TSDF_Float(void);
	virtual ~CKvYooji_Cube_TSDF_Float(void);
	CKvYooji_Cube_TSDF_Float(CKvYooji_Cube_TSDF_Float &a);
public:
	void c_Create(
			CKvPoint3Df in_origin_in_world,
			int in_depth,
			int in_height,
			int in_width,
			int in_edge_length_of_sub_cube_in_voxel,
			float in_size_of_voxel);

	void cp_Copy(CKvYooji_Cube_TSDF_Float *in_set_of_elements);

	void ts(
			int &out_number_of_columns,
			int &out_number_of_rows,
			int &out_number_of_matrices);
	void ts_Volume_Width_Height_Depth(
			int &out_number_of_columns,
			int &out_number_of_rows,
			int &out_number_of_matrices);

	void goiw_Get_Origin_In_World(CKvPoint3Df &out_origin_in_world) 
	{  out_origin_in_world = zz_org_point_in_world; }

	void gcciw_Get_Cube_Center_In_World(CKvPoint3Df &out_center_in_world);
	
	bool gpv_Get_Position_in_Voxel(
		CKvPoint3Df &in_point_in_world,
		CKvPoint3Df &out_point_in_voxel);

	bool gpw_Get_Position_in_World(
		CKvPoint3Df &in_point_in_voxel,
		CKvPoint3Df &out_point_in_world);
	
	bool grvu_Get_RGB_Value_Uninterpolated(
		CKvPoint3Df &in_point_in_voxel,
		CKvRgb &out_rgb);

	bool grvi_Get_RGB_Value_Interpolated(
		CKvPoint3Df &in_point_in_voxel,
		CKvRgb &out_rgb);

	bool gtvu_Get_TSDF_Value_Uninterpolated(
		CKvPoint3Df &in_point_in_voxel,
		float &out_tsdf);

	bool gtvi_Get_TSDF_Value_Interpolated(	
		CKvPoint3Df &in_point_in_voxel, 
		float &out_tsdf);

	bool gwdu_Get_Weight_Depth_Uninterpolated(
		CKvPoint3Df &in_point_in_voxel,
		float &out_w_depth);

	bool gwdi_Get_Weight_Depth_Interpolated(
		CKvPoint3Df &in_point_in_voxel,
		float &out_w_depth);

	bool csnt_Compute_Surface_Normal_from_TSDF(CKvPoint3Df &in_p3d,	
		CKvPoint3Df &out_surf_norm);

	// pointers.	
	CKvVolumeUcharRgb* pvr_Pointer_of_Volume_Rgb(){ return &zz_vol_rgb; }
	CKvVolumeFloat* pvd_Pointer_of_Volume_Depth(){ return &zz_vol_depth; }
	CKvSet2d_of_VectorShort* pvhs_Pointer_of_Visual_Hull_Segments(){ return &zz_set_of_visual_hull_segments; }
	CKvMatrixFloat* prpdv_Pointer_of_Reference_Points_and_Direction_Vectors(){ return &zz_set_of_ref_pt_and_dir_vec; }
	CKvHmatrix3D* ph3dnc_Homography_3D_for_Normalizing_Cube(){ return &zz_hmat_normal; }
	CKvVolumeUchar* pvwd_Pointer_of_Volume_Weight_Depth(){ return &zz_vol_w_depth; }
	CKvVolumeUchar* pvwr_Pointer_of_Volume_Weight_RGB(){ return &zz_vol_w_rgb; }
	CKvVolumeChar* pvfsc_Pointer_of_Volume_Flag_of_Sub_Cube(){ return  &zz_vol_flag_of_sub_cube; }

	float svm_Size_of_Voxel_in_Meter(){ return zz_sz_voxel; }
	int elsc_Edge_Length_of_Sub_Cube(){ return zz_edge_length_of_sub_cube; }

public:
	CKvPoint3Df zz_org_point_in_world;
	float zz_sz_voxel;
	int zz_edge_length_of_sub_cube;

protected:
	
	CKvVolumeUcharRgb zz_vol_rgb;
	CKvVolumeFloat zz_vol_depth;

	CKvSet2d_of_VectorShort zz_set_of_visual_hull_segments;
	CKvMatrixFloat zz_set_of_ref_pt_and_dir_vec;
	CKvHmatrix3D zz_hmat_normal;

	CKvVolumeUchar zz_vol_w_depth,zz_vol_w_rgb;
	//CKvVolumeBool zz_vol_flag_of_sub_cube;
	CKvVolumeChar zz_vol_flag_of_sub_cube;
	


private:
	/// these variables contains values temporarily during computation only.
	CKvPoint3Df zz_tpos;

	float zz_offset[3];
	float zz_residu[3];
	float zz_front[4], zz_back[4];
	float zz_inter_f, zz_inter_b;

	CKvRgb zz_front_rgb[4], zz_back_rgb[4];
	CKvRgb zz_inter_f_rgb, zz_inter_b_rgb;

	// variables for 'csnt_Compute_Surface_Normal_from_TSDF'
	CKvPoint3Df zz_origin;
	float zz_XY_foremost[4];
	float zz_XY_front[12];
	float zz_XY_back[12];
	float zz_XY_backmost[4];
	float zz_pos_vox[3];
};