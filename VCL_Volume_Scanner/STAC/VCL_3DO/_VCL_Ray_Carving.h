/////////////////////////////////////////
//Ray Carving Ver.2.0, 2014.04.15////////
/////////////////////////////////////////
class VCL_Ray_Carving : public CKvClass
{
private:
	//Output
	CKvSet2d_of_VectorFloat zz_analog_map_2D;
	//For pre-computation of 2D-to-3D relation
	CKvMatrixFloat		    zz_float_val_Nx11_matrix;
	CKvMatrixBool		    zz_bool_val_Nx3_matrix;
	//For Affine recfitied silhouette carving
	CKvVectorBool zz_Is_A_Vanishing_Point_In_Image;
	CKvSet_of_Matrix zz_affine_homography_for_each_camera;
	CKvSet_of_Matrix zz_inverse_affine_homography_for_each_camera;
	CKvSet_of_Matrix zz_look_up_table_of_projected_rays_for_each_camera;
	CKvSet_of_MatrixInt zz_index_of_projected_rays_for_each_camera;
	CKvSet_of_VectorInt zz_number_of_index_of_projected_rays_for_each_camera;
	CKvVectorInt zz_how_many_points_on_each_ray;

public:
	VCL_Ray_Carving();
	~VCL_Ray_Carving();
	//////////////////////////////////////////////////
	//////////Preparation for Ray Carving/////////////
	//////////////////////////////////////////////////
	void sp2d3dr_Set_Parameters_and_2Dim3Dim_Relations(
		float in_scale,
		CKvSet_of_Pmatrix3D *in_p_mat,
		CKvSet_of_SdkCode *in_silhouette,
		float in_angle_x,
		float in_angle_y,
		float in_angle_z,
		CKvMatrixFloat *in_rpdv,
		CKvHmatrix3D *out_homo,
		CKvSet_of_Pmatrix3D *out_p_matrices_or_NULL);
	void sp2d3dr_Set_Parameters_and_2Dim3Dim_Relations(	
		float in_scale,
		CKvSet_of_Pmatrix3D *in_p_mat,
		CKvSet_of_MatrixBool *in_silhouette,
		float in_angle_x,
		float in_angle_y,
		float in_angle_z,
		CKvMatrixFloat *in_rpdv,
		CKvHmatrix3D *out_homo,
		CKvSet_of_Pmatrix3D *out_p_matrices_or_NULL);
	void sp2d3dr_Set_Parameters_and_2Dim3Dim_Relations(	
		CKvSet_of_Pmatrix3D *in_p_mat,
		CKvSet_of_SdkCode *in_silhouette,
		CKvMatrixFloat *in_rpdv);
	void ngh_Normalize_into_unit_sphere_and_Get_Homography(
		double in_scale_factor_X_axis,
		double in_scale_factor_Y_axis,
		double in_scale_factor_Z_axis,
		CKvSet_of_Pmatrix3D *in_set_of_P_matrices,
		CKvSet_of_SdkCode *in_set_of_silhouette_images,
		double in_angle_in_radian_of_rotation_around_X_axis,
		double in_angle_in_radian_of_rotation_around_Y_axis,
		double in_angle_in_radian_of_rotation_around_Z_axis,
		CKvHmatrix3D *out_homography_for_transforming_object,
		CKvSet_of_Pmatrix3D *out_set_of_P_matrix_for_normalized_object);
	void s2d3dr_Set_2dim_3dim_Relation(
		CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
		int in_width_of_image,
		int in_height_of_image,
		CKvSet_of_Pmatrix3D *in_P_matrices);
	void z_pr_Project_a_Ray(
		int in_width_of_image,
		int in_height_of_image,
		float *in_reference_3d_point_of_a_ray,
		float *in_unit_direction_3d_vector_of_a_ray,
		float *in_P_matrix_3x4_in_row_first_mode,
		float *out_image_2d_point_of_a_visible_point_in_3D_world,
		float *out_unit_direction_2d_vector_on_image_plane,
		float *out_vanishing_2d_point,
		float &out_coordinate_of_vanishing_2d_point,
		float *out_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode,
		float *out_H_matrix_2x2_from_3_dim_to_2_dim_in_row_first_mode,
		bool &out_existence_of_a_visible_3d_point_on_the_ray,
		float &io_lower_limit_of_visible_zone,
		float &io_upper_limit_of_visible_zone,
		bool &out_existence_of_vanishing_2d_point,
		bool &out_flag_indicating_direction_2d_vector_is_zero);
	void z_fvz_Find_a_Visible_Zone(
		int in_width_of_image,
		int in_height_of_image,
		float *in_reference_3d_point_of_a_ray,
		float *in_unit_direction_3d_vector_of_a_ray,
		float *in_P_matrix_3x4_in_row_first_mode,
		float &io_lower_limit_of_visible_zone,
		float &io_upper_limit_of_visible_zone,
		bool &out_existence_of_visible_3d_point);
	void z_prv_Project_a_Ray_defined_by_a_Visible_reference_point(
		float *in_a_3d_point_visible_by_this_P_matrix,
		float *in_unit_direction_3d_vector_of_a_ray,
		float *in_P_matrix_3x4_in_row_first_mode,
		float *out_corresponding_2d_point_on_image_plane,
		float *out_unit_direction_2d_vector_on_image_plane,
		float *out_vanishing_2d_point,
		float &out_coordinate_of_vanishing_2d_point,
		float *out_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode,
		float *out_H_matrix_2x2_from_3_dim_to_2_dim_in_row_first_mode,
		bool &out_existence_of_vanishing_2d_point,
		bool &out_flag_indicating_direction_2d_vector_is_zero);	

	//////////////////////////////////////////////////
	//////////Ray Carving using Rectification/////////
	//////////////////////////////////////////////////
	void cvpi_Check_Vanishing_Point_In_Image(
		CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
		CKvSet_of_SdkCode *in_sil,
		CKvSet_of_Pmatrix3D *in_P_matrices);
	void cahc_Calculate_Affine_Homography_for_each_Camera
		(CKvSet_of_Pmatrix3D *in_p_mat,
		CKvMatrixFloat *in_rpdv);
	void ahp_Apply_Homography_to_Pmatrix(
		CKvSet_of_Pmatrix3D *in_p_mat,
		CKvSet_of_Pmatrix3D *out_p_mat);
	void snpm_Set_New_P_Set_of_Matrix(
		CKvSet_of_Pmatrix3D *in_P_matrices,
		CKvSet_of_Pmatrix3D *in_rec__P_matrices,
		CKvSet_of_Pmatrix3D *out_P_matrices);
	void slc_Set_Lookup_table_for_each_Camera(
		CKvSet_of_Pmatrix3D *in_p_mat,
		int width_of_rays,
		int height_of_rays,
		CKvMatrixFloat *in_rpdv);	
	void gss_Get_size_of_silhouettes_minmax(
		CKvSet_of_SdkCode *in_set_of_silhouette_images,
		CKvMatrixFloat *size_mat);
	void coopm_Change_Order_of_P_Matrix(
		CKvSet_of_Pmatrix3D *io_P_matrices,
		CKvMatrixFloat *size_mat,int start_img);	
	void s2d3dr_Set_2dim_3dim_Relation_rectified(
		CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
		CKvMatrixFloat *size_mat,
		CKvSet_of_Pmatrix3D *in_P_matrices);
	void z_pr_Project_a_Ray_Rectified(
		float *size_vec,
		float *in_reference_3d_point_of_a_ray,
		float *in_unit_direction_3d_vector_of_a_ray,
		float *in_P_matrix_3x4_in_row_first_mode,
		float *out_image_2d_point_of_a_visible_point_in_3D_world,
		float *out_unit_direction_2d_vector_on_image_plane,
		float *out_vanishing_2d_point,
		float &out_coordinate_of_vanishing_2d_point,
		float *out_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode,
		float *out_H_matrix_2x2_from_3_dim_to_2_dim_in_row_first_mode,
		bool &out_existence_of_a_visible_3d_point_on_the_ray,
		float &io_lower_limit_of_visible_zone,
		float &io_upper_limit_of_visible_zone,
		bool &out_existence_of_vanishing_2d_point,
		bool &out_flag_indicating_direction_2d_vector_is_zero,
		bool &in_Is_A_Vanishing_Point_In_Image);
	void z_fvz_Find_a_Visible_Zone_Rectified(
		float *size_vec,
		float *in_reference_3d_point_of_a_ray,
		float *in_unit_direction_3d_vector_of_a_ray,
		float *in_P_matrix_3x4_in_row_first_mode,
		float &io_lower_limit_of_visible_zone,
		float &io_upper_limit_of_visible_zone,
		bool &out_existence_of_visible_3d_point,
		bool &in_Is_A_Vanishing_Point_In_Image);
	void z_prv_Project_a_Ray_defined_by_a_Visible_reference_point_Rectified(
		float *in_a_3d_point_visible_by_this_P_matrix,
		float *in_unit_direction_3d_vector_of_a_ray,
		float *in_P_matrix_3x4_in_row_first_mode,
		float *out_corresponding_2d_point_on_image_plane,
		float *out_unit_direction_2d_vector_on_image_plane,
		float *out_vanishing_2d_point,
		float &out_coordinate_of_vanishing_2d_point,
		float *out_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode,
		float *out_H_matrix_2x2_from_3_dim_to_2_dim_in_row_first_mode,
		bool &out_existence_of_vanishing_2d_point,
		bool &out_flag_indicating_direction_2d_vector_is_zero,
		float *size_vec,
		bool &in_Is_A_Vanishing_Point_In_Image);
	bool drc_Do_Ray_Carving_using_recitified_silhouette_Lossy(
		CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
		int ray_width,
		int ray_height,
		CKvSet_of_SdkCode *in_set_of_silhouette_images,
		float *io_elapsed_time);
	bool z_drc_Do_Ray_Carving_using_recitified_silhouette_Lossy(
		CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
		int ray_width,
		int ray_height,
		CKvSdkCode *in_set_of_silhouette_images,
		int in_number_of_images,
		CKvVectorFloat *out_set_of_depth_jets,
		float *io_elapsed_time);
	void Apply_Homography_to_Binary_Image(
		CKvSdkCode *in_set_of_silhouette_images,
		double &io_minx,
		double &io_maxx,
		double &io_miny,
		double &io_maxy,
		CKvMatrix *homography,
		CKvMatrix *inv_homography,
		CKvSdkCode &rectified_sil);
	bool z_idj_Initialize_Depth_Jets_using_rectified_silhouette_Lossy(
		CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
		int ray_width,
		int ray_height,
		double in_minx,
		double in_miny,
		double **start_point_of_ray,
		CKvSdkCode *in_silhouette_image,
		CKvVectorFloat *out_set_of_depth_jets);
	bool z_udj_Update_Depth_Jets_using_rectified_silhouette_Lossy(
		CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
		int ray_width,
		int ray_height,
		double in_minx,
		double in_miny,
		double **start_point_of_ray,
		CKvSdkCode *in_silhouette_image,
		int in_idx_silhouettes,
		CKvVectorFloat *out_set_of_depth_jets);
	bool z_ss_Shave_Segments_using_rectified_silhouette_Lossy(
		CKvVectorInt *out_list_for_x1,
		CKvVectorInt *out_list_for_x2,
		CKvVectorInt *out_list_for_index_of_last_run_in_a_row,
		CKvVectorInt *out_list_for_number_of_runs_in_a_row,
		float *in_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode,//새로운 것으로 교체 
		double in_minx,
		double in_miny,
		double *start_point_of_ray,
		bool in_existence_of_vanishing_2d_point,
		float in_coordinate_of_vanishing_2d_point,
		CKvVectorFloat *io_depth_jet,//i-1번쨰 실루엣 carving 결과를 받아 i번쨰 실루엣 까지의 carving 결과를 리턴
		bool &out_validity);
	bool z_fsb_Find_Segments_carved_by_Boundaries_using_rectified_silhouette_Lossy(
		CKvVectorInt *out_list_for_x1,
		CKvVectorInt *out_list_for_x2,
		CKvVectorInt *out_list_for_index_of_last_run_in_a_row,
		CKvVectorInt *out_list_for_number_of_runs_in_a_row,
		double in_minx,
		double in_miny,
		double *start_point_of_ray,
		CKvVectorFloat *out_set_of_segment_coordinates,
		bool &out_validity);

	bool bp_Back_Projection(
		CKvVectorFloat *in_vec,
		bool in_existence_of_vanishing_2d_point,
		float *in_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode,//새로운 것으로 교체 
		CKvVectorFloat *io_depth_jet,//i-1번쨰 실루엣 carving 결과를 받아 i번쨰 실루엣 까지의 carving 결과를 리턴
		float in_coordinate_of_vanishing_2d_point,
		bool &out_validity);


	//////////////////////////////////////////////////////////////
	//////////Ray Carving using pre-computed ray relation/////////
	//////////////////////////////////////////////////////////////
	bool drc_Do_Ray_Carving_using_Precomputed_Ray_Relation(
		CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
		CKvSet_of_SdkCode *in_set_of_silhouette_images);
	bool z_drc_Do_Ray_Carving_using_Precomputed_Ray_Relation(
		CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
		CKvSdkCode *in_set_of_silhouette_images,
		int in_number_of_images,
		CKvVectorFloat *out_set_of_depth_jets);	
	bool z_idj_Initialize_Depth_Jets_using_Precomputed_Ray_Relation(
		CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
		CKvSdkCode *in_silhouette_image,
		CKvVectorFloat *out_set_of_depth_jets);
	bool z_udj_Update_Depth_Jets_using_Precomputed_Ray_Relation(
		CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
		CKvSdkCode *in_silhouette_image,
		int in_idx_silhouettes,
		CKvVectorFloat *io_set_of_depth_jets);
	bool z_ss_Shave_Segments_using_Precomputed_Ray_Relation(
		int in_number_of_contours,
		CKvSet_of_MatrixInt *in_set_of_Nx2_matrix_x_y_OR_Nx3_matrix_x_y_c,
		float *in_origin_2d_point_of_a_projected_ray,
		float *in_unit_direction_2d_vector_of_a_projected_ray,
		float *in_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode,
		bool in_existence_of_vanishing_2d_point,
		float in_coordinate_of_vanishing_2d_point,
		bool in_flag_indicating_direction_2d_vector_is_zero,
		bool in_equal_distance_mode,
		CKvVectorFloat *io_depth_jet,
		bool &out_validity);

	//////////////////////////////////////////////////////////////
	//////////Ray Carving, Original version///////////////////////
	//////////////////////////////////////////////////////////////
	// customized by Yooji... start
	bool drc_Do_Ray_Carving(
		CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
		CKvSet_of_Pmatrix3D *in_set_of_P_matrices,
		CKvSet_of_SdkCode *in_set_of_silhouette_images,
		CKvSet_of_VectorFloat *io_docube_segments);
	// customized by Yooji... end
	bool drc_Do_Ray_Carving(
		CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
		int in_width_of_depth_map,
		int in_height_of_depth_map,
		CKvSet_of_Pmatrix3D *in_set_of_P_matrices,
		CKvSet_of_SdkCode *in_set_of_silhouette_images);
	bool z_drc_Do_Ray_Carving(
		CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
		CKvPmatrix3D *in_set_of_P_matrices,
		CKvSdkCode *in_set_of_silhouette_images,
		int in_number_of_images,
		CKvVectorFloat *out_set_of_depth_jets);
	bool z_idj_Initialize_Depth_Jets(
		CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
		CKvPmatrix3D *in_P_matrix,
		CKvSdkCode *in_silhouette_image,
		CKvVectorFloat *out_set_of_depth_jets);
	bool z_udj_Update_Depth_Jets(
		CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
		CKvPmatrix3D *in_P_matrix,
		CKvSdkCode *in_silhouette_image,
		CKvVectorFloat *io_set_of_depth_jets);
	bool z_ss_Shave_Segments(
		int in_number_of_contours,
		CKvSet_of_MatrixInt *in_set_of_Nx2_matrix_x_y_OR_Nx3_matrix_x_y_c,
		float *in_origin_2d_point_of_a_projected_ray,
		float *in_unit_direction_2d_vector_of_a_projected_ray,
		float *in_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode,
		bool in_existence_of_vanishing_2d_point,
		float in_coordinate_of_vanishing_2d_point,
		bool in_flag_indicating_direction_2d_vector_is_zero,
		bool in_equal_distance_mode,
		CKvVectorFloat *io_depth_jet,
		bool &out_validity);
	bool z_fsb_Find_Segments_carved_by_Boundaries(
		int in_number_of_contours,
		CKvSet_of_MatrixInt *in_set_of_Nx2_matrix_x_y_OR_Nx3_matrix_x_y_c,
		float *in_origin_2d_point_of_a_projected_ray,
		float *in_unit_direction_2d_vector_of_a_projected_ray,
		bool in_equal_distance_mode,
		CKvVectorFloat *out_set_of_segment_coordinates,
		bool &out_validity);

	//////////////////////////////////////////////////////////////
	//////////Voxel carving///////////////////////////////////////
	//////////////////////////////////////////////////////////////
	bool Voxel_carving(
		CKvSet_of_Pmatrix3D *in_pmat,
		CKvSet_of_MatrixBool *in_silhouette,
		int in_width,
		int in_height,
		int in_depth,
		int &num_voxe,
		float &out_time);

	//////////////////////////////////////////////////////////////
	////////////////////////Utility///////////////////////////////
	//////////////////////////////////////////////////////////////
	void idm_Initialize_Depth_Map(int in_width_of_depth_map,int in_height_of_depth_map);
	bool inj_Is_NULL_Jet(CKvVectorFloat *in_jet);
	CKvSet2d_of_VectorFloat* edm_Export_Depth_Map();

protected:
	LCKvUtility_for_DoSurfaceFloat zz_udos;
};

