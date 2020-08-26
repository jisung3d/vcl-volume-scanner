//********************************************************************************************
//class AFX_EXT_CLASS LCKvYooji_SfS : public CKvClass
// + This class contains the functions for volumetric integration.
class LCKvYooji_Shape_from_Silhouette : public CKvClass
//********************************************************************************************
{
public:

	LCKvYooji_Shape_from_Silhouette();
	~LCKvYooji_Shape_from_Silhouette();

	// global functions.
	void i_Initialize(CKvYooji_Cube_TSDF_Float *in_cube);

	void usif_Update_Silhouette_Induced_Field(
		vector<CKvYooji_MatrixRgbD> *in_set_of_mat_rgbd,
		//CKvYooji_MatrixRgbD *in_mat_rgbd,
		vector<CKvPmatrix3D> *in_set_of_p_matrices,
		CKvYooji_Cube_TSDF_Float *io_cube);
	
// 	void sfsi_Shape_From_Silhouette_Incremental(
// 		CKvPmatrix3D *in_pmat_rgb,
// 		CKvMatrixBool *in_mask_object,
// 		CKvYooji_Tracking_State *in_state,
// 		CKvSet2d_of_VectorFloat *io_docube_segments,
// 		bool in_flag_is_first_frame = false);

	void sfsi_Shape_From_Silhouette_Incremental(
		CKvPmatrix3D *in_pmat_rgb,
		CKvMatrixBool *in_mask_object,
		CKvYooji_Tracking_State *in_state,
		CKvSet2d_of_VectorFloat *io_docube_segments = NULL,
		bool in_flag_is_first_frame = false);

	bool rsdi_Refine_Shape_using_Depth_Incremental(
		CKvPmatrix3D *in_pmat_rgb,
		CKvMatrixFloat *in_map_depth,
		CKvSet2d_of_VectorFloat *io_docube_segments = NULL,
		bool in_flag_is_first_frame = false);

	// SFS.
	bool drc_Do_Ray_Carving(
		CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
		CKvSet_of_Pmatrix3D *in_set_of_P_matrices,
		CKvSet_of_SdkCode *in_set_of_silhouette_images,
		CKvSet_of_VectorFloat *io_ray_segments);

	// ==========================================================
	// 기존 drc_Do_Ray_Carving 함수는 이상함.
	// Incremental 하게 바꾸고 나서 잘 돌아감...
	bool drci_Do_Ray_Carving_Incremental(
		CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
		CKvPmatrix3D *in_P_matrix,
		CKvSdkCode *in_silhouette_image,
		CKvSet2d_of_VectorFloat *io_docube_segments,
		bool in_flag_is_first_frame = false);
	// ==========================================================

	void uvv_Update_Valid_Volume(CKvVolumeBool *io_valid_volume);

 	void nccsfs_Normalize_Cube_Coordinates_into_unit_sphere_for_Shape_From_Silhouette(
 		CKvYooji_Cube_TSDF_Float *io_cube);

	void ncsfs_Normalize_Camera_Matrix_for_Shape_From_Silhouette(
		CKvHmatrix3D *in_hmat_normal,
		CKvSet_of_Pmatrix3D *in_set_of_pmat,
		CKvSet_of_Pmatrix3D *out_set_of_pmat);

	bool inj_Is_NULL_Jet(CKvVectorShort *in_jet);
	bool inj_Is_NULL_Jet(CKvVectorFloat *in_jet);

	// Display.
	// ======================================================
	void cdcm_Convert_Docube_to_Mesh(
		CKvSet2d_of_VectorFloat *in_docube_segments,
		CKvMatrixFloat *in_ref_points_and_direc_vecs,
		CKvPmatrix3D *in_p_mat,
		CKvGraph3D *io_graph_3d = NULL);
	void cdcm_Convert_Docube_to_Mesh(		
		CKvPmatrix3D *in_p_mat,
		VCL_DoCube *in_docube = NULL,
		//CKvSet2d_of_VectorFloat *in_docube_segments = NULL,
		CKvGraph3D *io_graph_3d = NULL);

	void em_Export_Mesh(
		CKvDepot_of_Point3D *out_depot_p3d,
		CKvVectorInt *out_idx_pnts_vhs);
	// ======================================================
	void rdvh_Render_Depth_from_Visual_Hull(
		CKvSet2d_of_VectorFloat *in_docube_segments,
		CKvMatrixFloat *in_ref_points_and_direc_vecs,
		CKvPmatrix3D *in_p_mat,
		CKvHmatrix3D *in_homo_norm,
		CKvMatrixFloat *io_map_depth,
		CKvGraph3D *io_graph_3d = NULL);

	void rdvh_Render_Depth_from_Visual_Hull(
		CKvPmatrix3D *in_p_mat,
		CKvMatrixFloat *io_map_depth);

private:
	void z_cor_Compute_Object_Rims(
		vector<CKvYooji_MatrixRgbD> *in_set_of_mat_rgbd,		
		vector<CKvPmatrix3D> *in_set_of_p_matrices,
		CKvYooji_Cube_TSDF_Float *io_cube);

	bool z_drc_Do_Ray_Carving(
		CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
		CKvPmatrix3D *in_set_of_P_matrices,
		CKvSdkCode *in_set_of_silhouette_images,
		int in_number_of_images,
		CKvVectorFloat *out_set_of_depth_jets);

	bool z_drci_Do_Ray_Carving_Incremental(
		CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
		CKvPmatrix3D *in_P_matrix,
		CKvSdkCode *in_silhouette_image,
		CKvVectorFloat *out_set_of_depth_jets,
		bool in_flag_is_first_frame = false);

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


	bool z_fsb_Find_Segments_carved_by_Boundaries(
		int in_number_of_contours,
		CKvSet_of_MatrixInt *in_set_of_Nx2_matrix_x_y_OR_Nx3_matrix_x_y_c,
		float *in_origin_2d_point_of_a_projected_ray,
		float *in_unit_direction_2d_vector_of_a_projected_ray,
		bool in_equal_distance_mode,
		CKvVectorFloat *out_set_of_segment_coordinates,
		bool &out_validity);


	////
private:
	void z_gmtdm_Get_Mesh_of_Triangles_from_a_Depth_Map(
		CKvMatrixFloat *in_depth_map,
		CKvPmatrix3D *in_pmat_depth,
		CKvSet_of_Point3D &out_set_of_p3d_in_world,
		CKvMatrixInt &out_mesh_indices);

	void gmtdm_Get_Mesh_of_Triangles_from_a_Depth_Map_New(
		CKvMatrixFloat *in_depth_map,
		CKvPmatrix3D *in_pmat_depth,
		CKvSet_of_Point3D &out_set_of_p3d_in_world,
		CKvMatrixInt &out_mesh_indices,
		int &out_number_of_valid_p3ds,
		int &out_number_of_valid_mesh_tris);
	
	void z_sord_Select_Optimal_Ray_Direction(
		CKvPmatrix3D *in_pmat,
		int &out_docube_axis);

	void z_crddc_Convert_Ray_Direction_of_DoCube(
		VCL_DoCube &in_docube,
		int in_ray_direction,
		VCL_DoCube &out_docube);

	void z_ursitdc_Update_Ray_Segments_based_on_Intersection_of_Two_DoCubes(
		VCL_DoCube &in_docube_ref,
		VCL_DoCube &io_docube_target);


	// 	void cdmcm_Create_Depth_Map_for_Current_Model(CKvTrackingState *io_track_state,
	// 		CKvCubeFloatTSDF *io_cube);

	//CKvMatrixFloat zz_transf_glob_to_curr;	
private:
	LCKvYooji_Scanner_Display zz_disp;

	VCL_DoCube zz_docube, zz_docube_sfs, zz_docube_init, zz_docube_for_mesh;
	CKvSet2d_of_VectorFloat zz_set_ray_float;

	//////////////////////////////////////////////////////////////////////////
	CKvMatrixFloat zz_rpdv; 
	CKvHmatrix3D zz_hmat_norm;

	CKvVectorInt zz_v_index;
	CKvMatrixInt zz_map_index_new;
	CKvSet_of_Point3Df zz_set_p3d;

	//For dual ray carving.
	CKvRunSet zz_set_run;
	CKvMatrixBool zz_slice,zz_slice_t;
	CKvSet2d_of_VectorShort zz_set_of_rays;
	//////////////////////////////////////////////////////////////////////////

	LCKvUtility_for_DoSurfaceFloat zz_util_dosf;
	VCL_Ray_Carving zz_ray_carving;

	LCKvAlgebra_for_MatrixFloat zz_alg_mf;
	CKvYooji_Intrinsics zz_intrinsics;
	CKvYooji_Extrinsics zz_transform_glob_to_cam;
	CKvMatrixFloat zz_mat_4x4;

	CKvDepot_of_Point3D zz_depot_p3d, zz_depot_p3d2;
	CKvDepot_of_RgbaF zz_depot_rgba, zz_depot_rgba2;
	CKvMesh_of_Triangle zz_mesh_tri, zz_mesh_tri2;
	CKvVectorInt zz_idx_pnts_total;

	// 	CKvVector3f zz_p3d;
	// 	CKvVector3i zz_idx_sc;	
	CKvPixel zz_tpix;

	// for 'z_utvs_Update_TSDF_Values_of_Segment'
	//	CKvVector3f zz_tp3d1, zz_tp3d2;
	CKvPointf zz_pix_start, zz_pix_end, zz_tpixf;
};