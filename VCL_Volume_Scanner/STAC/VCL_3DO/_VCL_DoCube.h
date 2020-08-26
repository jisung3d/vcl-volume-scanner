/////////////////////////////////////////
//DoCube Ver.1.0, 2013.12.11/////////////
/////////////////////////////////////////
class VCL_DoCube: public CKvDoCubeShort
{
public:
	VCL_DoCube();
	virtual ~VCL_DoCube();
	//Parameter Setting
	void sc_Set_Cube(int in_width,int in_height,int in_depth);
	void sr_Set_Resolution(int in_width,int in_height,int in_depth);	
	void srpdv_Set_Reference_Points_and_Direction_Vectors(int in_width,int in_height);
	
	//Get Parameter Info.
	void gr_Get_Resolution(int &out_width,int &out_height,int &out_depth);
	CKvMatrixFloat * grpdv_Get_Reference_Points_and_Direction_Vectors();
	void gvr_Get_Valid_Rays(int &out_valid_rays);

	///////////////////////////////////////
	//Preparation for Contents Generation//
	///////////////////////////////////////
	//For Visual hull computation(CPU)
	void sr_Set_Rays(float in_scale,float in_ray_angle_x,float in_ray_angle_y,float in_ray_angle_z,CKvSet_of_Pmatrix3D *in_pmat,CKvSet_of_SdkCode *in_sil,CKvSet_of_Pmatrix3D *out_n_pmat);
	void sr_Set_Rays(CKvSet_of_Pmatrix3D *in_pmat,CKvSet_of_SdkCode *in_sil);
	//For Visual hull computation(CPU, Affine mode)
	void sr_Set_Rays_Affine(float in_scale,float in_ray_angle_x,float in_ray_angle_y,float in_ray_angle_z,CKvSet_of_Pmatrix3D *in_pmat,CKvSet_of_SdkCode *in_sil,CKvSet_of_Pmatrix3D *out_n_pmat);
	//For Mesh to DoS
	void sr_Set_Rays(float in_angle_x,float in_angle_y,float in_angle_z); 


	void sr_Set_Rays_using_Temporal_Coherence(float in_scale,float in_ray_angle_x,float in_ray_angle_y,float in_ray_angle_z,CKvSet_of_Pmatrix3D *in_pmat,CKvSet_of_SdkCode *in_sil,CKvSet_of_Pmatrix3D *out_n_pmat);


	///////////////////////////////////////
	//////////Visual hull Computation//////
	///////////////////////////////////////
	//For Visual hull computation(CPU)
	void drc_Do_Ray_Carving_using_rectified_silhouette_Lossy(CKvSet_of_SdkCode *in_sil,float *out_time);
	void drc_Do_Ray_Carving_using_Precomputed_Ray_Relation(CKvSet_of_SdkCode *in_sil,float *out_time);
	void drc_Do_Ray_Carving(CKvSet_of_SdkCode *in_sil,CKvSet_of_Pmatrix3D *in_pmat); 
	
	void lfs_Load_From_Silhouette(CKvSet_of_Pmatrix3D *in_pmat,CKvSet_of_SdkCode *in_sil);
	void lfs_Load_From_Silhouette(float in_scale,float in_ray_angle_x,float in_ray_angle_y,float in_ray_angle_z,CKvSet_of_Pmatrix3D *in_pmat,CKvSet_of_SdkCode *in_sil,CKvSet_of_Pmatrix3D *out_n_pmat);
	
	void drc_Do_Ray_Carving_using_Temporal_Coherence(CKvSet_of_SdkCode *in_sil,CKvSet_of_Pmatrix3D *in_pmat,int in_interval_I_frame,int in_current_frame);




	//Converting 3D Mesh to DoCube
	void lfm_Load_From_Mesh(CKvSet_of_Point3D *in_ori_p3d,CKvMatrixInt *in_ori_mesh);

	void cmdc_Convert_Mesh_to_DoCube(
	CKvSet_of_Point3D *in_set_of_p3d_world,
	CKvHmatrix3D *in_homo_rotation_or_NULL,
	CKvHmatrix3D *in_homo_norm,
	CKvMatrixInt *in_mesh_of_depth_surface,
	float in_theta_z_axis_and_opt_axis_depth);

	bool cmdc_Convert_Mesh_to_DoCube(
		CKvSet_of_Point3D *in_set_of_p3d_world,
		CKvMatrixInt *in_mesh_of_depth_surface,
		int in_num_valid_p3ds,
		int in_num_valid_mesh_tris,
		CKvPmatrix3D *in_pmat,
		CKvHmatrix3D *in_homo_norm,
		int in_ray_direction,
		float in_shield_margin = 0.0f);

	//Import from Ray carving results
	void i_Import(
		CKvSet2d_of_VectorFloat *in_analog_depth_map,
		CKvHmatrix3D *in_homo_or_NULL);
	void i_Import(
		CKvSet2d_of_VectorFloat *in_analog_depth_map,
		CKvHmatrix3D *in_homo_or_NULL,
		CKvMatrixFloat *);
	void i_Import(CKvSet2d_of_VectorShort *in_quantized_depth_map);

	void gidd_Get_Information_of_Depth_Data(
		CKvSet2d_of_VectorShort *in_depth_data,
		float in_maximum_magnitude_of_depth,
		float &out_quantization_step_size,
		short &out_max_depth_value,
		short &out_min_depth_value,
		int &out_number_of_layers,
		int &out_number_of_points,
		float &out_dynamic_range_of_input,
		CKvMatrixInt *out_start_point_index_map);
		
	void ian_Import_from_Analog_depth_map_NEW(
		CKvSet2d_of_VectorFloat *in_depth_map_in_increasing_order,
		bool in_mode_for_testing_input_depth_map,
		float in_maximum_magnitude_of_depth,
		short in_number_of_quantization_levels,
		CKvMatrixFloat *in_set_of_reference_and_direction_vectors,
		CKvHmatrix3D *in_homography_for_transforming_object);
	
	bool u_qdm_Quantize_Depth_Map_2d_NEW(
		float in_maximum_of_depth_magnitude,
		short in_number_of_levels,
		CKvSet2d_of_VectorFloat *in_depth_map_in_increasing_order,
		bool in_mode_for_testing_input_depth_map,
		CKvVectorShort *out_depth_index_array,
		int &out_width_of_depth_map,
		int &out_height_of_depth_map,
		float &out_quantization_step_size,
		short &out_max_depth_value,
		short &out_min_depth_value,
		int &out_number_of_layers,
		int &out_number_of_points,
		CKvMatrixInt *out_start_point_index_map);
	
	bool zu_qdm_Quantize_Depth_Map_NEW(
		float in_maximum_of_depth_magnitude,
		short in_number_of_levels,
		int in_number_of_rays,
		CKvVectorFloat *in_set_of_depth_jets,
		bool in_mode_for_testing_input_depth_map,
		CKvVectorShort *out_depth_index_array,
		float &out_quantization_step_size,
		short &out_max_depth_value,
		short &out_min_depth_value,
		int &out_number_of_layers,
		int &out_number_of_points,
		int *out_set_of_start_point_indices);
	
	bool zu_qdj_Quantize_Depth_Jet_NEW(
		float in_maximum_of_depth_magnitude,
		short in_number_of_levels,
		CKvVectorFloat *in_depth_jet,
		bool in_mode_for_testing_input_depth_map,
		int &out_number_of_points,
		CKvVectorShort *out_depth_index_jet,
		float &out_quantization_step_size,
		short &io_max_depth_value,
		short &io_min_depth_value,
		int &io_number_of_layers,
		int &io_total_number_of_points);

	VCL_Mesh_to_DoS     zz_mesh_to_DoS;

	LCKvUtility_for_DoSurfaceFloat zz_util_DoSf;
};