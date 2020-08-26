/////////////////////////////////////////////
//Utility VCL3DO Ver.1.0, 2013.12.11////////
////////////////////////////////////////////
class Util_VCL_3DO: public CKvClass
{
public:
	Util_VCL_3DO();
	virtual ~Util_VCL_3DO();

	////////////////////////////
	///////Get Projected Image//
	////////////////////////////
	//By Z-buffering
	void gdizb_Get_Depth_Images_using_Z_Buffering(
		CKvDepot_of_Point3D *in_depot_point,
		CKvDepot_of_RgbaF *in_depot_color,
		CKvMesh_of_Triangle *in_mesh_triagnle,
		CKvSet_of_Pmatrix3D *in_pmat,
		CKvSet_of_MatrixUcharRgb *in_imgs,
		CKvSet_of_MatrixFloat *out_depth_map,
		CKvSet_of_MatrixInt *out_buffer);

	// CAUTION ============================================================ //
	// YOU SHOULD INITIALZE DEPTH MAP AS 255.0F VALUES. //
	void gdmzbi_Generate_Depth_Map_using_Z_buffering_for_an_Image(
		CKvDepot_of_Point3D *in_depot_point,
		CKvVectorInt *in_idx_mesh,
		CKvPmatrix3D *in_p_mat,
		CKvMatrixFloat *out_d_map,
		CKvMatrixInt *out_f_buffer);

	void gdmzbibe_Generate_Depth_Map_using_Z_buffering_for_an_Image_Boundary_Erosion(
		CKvDepot_of_Point3D *in_depot_point,
		CKvVectorInt *in_idx_mesh,
		CKvPmatrix3D *in_p_mat,
		CKvMatrixFloat *out_d_map,
		CKvMatrixInt *out_f_buffer);

	/// ///////////////////////////////////////////////////////////////
	void gszbi_Generat_Silhouettes_using_Z_buffering(
		CKvDepot_of_Point3D *in_dp,
		CKvDepot_of_RgbaF *in_dc,
		CKvMesh_of_Triangle *in_mesh,
		CKvSet_of_Pmatrix3D *in_pmat,
		CKvSet_of_MatrixBool *io_mask,
		CKvSet_of_SdkCode *io_mask_sdkcode);
	void gszbi_Generat_Silhouettes_using_Z_buffering(
		CKvDepot_of_Point3D *in_depot_point,
		CKvDepot_of_RgbaF *in_depot_color,
		CKvMesh_of_Triangle *in_mesh_triangle,
		CKvSet_of_Pmatrix3D *in_set_of_pmat,
		int *in_ww_hh,
		int in_dilation,
		CKvSet_of_SdkCode *out_mask_sdkcode);
	void gszbi_Generat_Silhouette_using_Z_buffering_for_an_Image(
		CKvDepot_of_Point3D *in_dp,
		CKvVectorInt *in_idx_mesh,
		CKvPmatrix3D *in_p_mat,
		CKvMatrixBool *out_silhouette);
	/// ///////////////////////////////////////////////////////////////
	
	void gim_Get_Indices_of_Mesh(CKvMesh_of_Triangle *in_mesh_triangle,int in_0_pt_1_front_color_2_back_color,CKvVectorInt *out_idx_mesh);
	
	
	
	
	/// to be fixed...
	void gdmzbi_Generate_Depth_Map_using_Z_buffering_for_an_Image_S_Series_Data(CKvDepot_of_Point3D *in_depot_point,CKvVectorInt *in_idx_mesh,CKvPmatrix3D *in_p_mat,CKvMatrixFloat *out_d_map,CKvMatrixInt *out_f_buffer);		
	
	void ub_Update_Buffer(float *in_position,float *in_depth,int in_idx_mesh,CKvMatrixFloat *io_d_map,CKvMatrixInt *io_f_buffer);
	/// ///////////////////////////////////////////////////////////////
	void ub_Update_Buffer(float *in_position,float *in_depth,int in_idx_mesh,CKvMatrixBool *io_silhouette);
	/// ///////////////////////////////////////////////////////////////

	void gd2dpp_Get_Depth_and_2D_Position_of_a_Point(float *in_pt,float *in_p_mat,float *out_position,float &out_depth);
	void gd2dpt_Get_Depth_and_2D_Position_of_a_Triangle(float *in_pt,float *in_p_mat,float *out_position,float *out_depth);

	void gd2dpt_Get_Depth_and_2D_Position_of_a_Triangle_S_Series_Data(float *in_pt,float *in_p_mat,float *out_position,float *out_depth);
	void gd2dpp_Get_Depth_and_2D_Position_of_a_Point_S_Series_Data(float *in_pt,float *in_p_mat,float *out_position,float &out_depth);

	void id_Interpolate_Depth(float *in_position,float *in_depth,int in_x,int in_y,float &out_depth);
	void cw_Compute_Weight(float *in_position,int in_x,int in_y,float *out_weight,bool &out_is_inside);
	void mat_Measure_Area_of_a_Triangle(float *in_coordinates_of_3_points,float &area_or_a_triangle);
	void gbi_Get_Block_for_Interpolation(float *in_position,int *out_LTRB);
		


	////////////////////////////
	///Get visibility zone//////
	////////////////////////////
	void cip_Create_Image_Planes(
		int in_num_images,
		CKvSet_of_MatrixUcharRgb *in_background_img_set,
		CKvSet_of_MatrixBool *out_mask,
		CKvSet_of_SdkCode *out_mask_sdkcode);

	void gvz_Get_Visibility_Zone(
		CKvDepot_of_Point3D *in_depot_point,
		CKvDepot_of_RgbaF *in_depot_color,
		CKvMesh_of_Triangle *in_mesh_triangle,
		CKvSet_of_Pmatrix3D *in_set_of_pmat,
		CKvSet_of_MatrixBool *out_mask,
		CKvSet_of_SdkCode *out_mask_sdkcode);

	void gvzi_Get_Visibility_Zone_For_an_Image(	
		CKvDepot_of_Point3D *in_depot_point,
		CKvDepot_of_RgbaF *in_depot_color,
		CKvMesh_of_Triangle *in_mesh_triangle,
		CKvSet_of_Pmatrix3D *in_set_of_pmat,
		CKvSet_of_MatrixBool *io_mask,
		CKvSet_of_SdkCode *io_mask_sdkcode);	


	////////////////////////////
	///Create cameras///////////
	////////////////////////////
	void cc_Create_Cameras(int *in_angle_of_camera,int in_num_camera,int *in_ww_hh,float in_reciprocal_of_f,float in_scale,	CKvSet_of_Pmatrix3D *out_p_mat_set);
	void cci_Create_Camera_for_an_Image(int *in_angle_of_camera,int *in_ww_hh,float in_reciprocal_of_f,float in_scale,CKvPmatrix3D *out_p_mat);
	void gac_Get_Angle_of_Camera(int *out_angle_of_camera);
		
	//Get point-wise normal vector	
	void fnv_find_normal_vector(Util_Mesh_Generation *in_initial_volume_sp,CKvSet_of_Point3D *in_set_of_points,CKvSet_of_Point3D *out_set_of_normal_vectors);

	void cp_cross_product(CKvPoint3D in1,CKvPoint3D in2,CKvPoint3D &out_normalized,double &magnitude);
	
	// Add h.d.
	void _Get_Depth_Image_and_visibility(
		CKvDoCubeShort *in,
		CKvPmatrix3D *in_pmat,
		int in_width,
		int in_height,
		int in_id_pmat,	
		float in_th_for_visibility,
		CKvVolumeBool *in_voxel,
		CKvMatrixBool *io_visibility, // width=num_point, height=visible:true, invisible:false
		CKvMatrixFloat *out_depth_map);
	void _Get_Depth_Image_and_visibility(
		CKvDoCubeShort *in,
		CKvSet_of_Pmatrix3D *in_pmat,
		int in_width,
		int in_height,
		float in_th_for_visibility,
		CKvVolumeBool *in_voxel,
		int num_voxel,
		CKvMatrixBool *out_visibility, // width=num_point, height=visible:true, invisible:false
		CKvSet_of_MatrixFloat *out_depth_map);		

	//General utility
	void _Cross_Product(CKvVector *in_data1,CKvVector *in_data2,CKvVector *out_data);		// (size=3)	
	void _Color_Conversion_Rgb_to_Yuv(CKvMatrixUcharRgb *in_img,       CKvSet_of_Matrix *out_img);
	void _Color_Conversion_Rgb_to_Yuv(CKvSet_of_MatrixUcharRgb *in_img,CKvSet2d_of_Matrix *out_img); // height=3: Y, U, V; width=num_img
	void cmi_Convert_Mask_Image(CKvSet_of_SdkCode *in_mask,CKvSet_of_MatrixBool *out_mask);

	//Plot float image as pseudo color.
	void dmfpc_Display_Matrix_Float_using_Pseudo_Color(CKvScreen *in_screen,CKvMatrixFloat &in_matrix,int in_normalized_mode);


	//Refinement and dilation
	void rvh_Refine_Visual_Hull(CKvSet2d_of_VectorShort *io_vh);
	void od_Object_dilation(CKvSet2d_of_VectorShort *in_object,	CKvSet2d_of_VectorShort *out_object,int in_depth);
	void od_Object_dilation(VCL_DoCube *io_object,int in_depth);

	
	LCKvUtility_for_Import  zz_util_import;
	CKvRunSet zz_runset;
};