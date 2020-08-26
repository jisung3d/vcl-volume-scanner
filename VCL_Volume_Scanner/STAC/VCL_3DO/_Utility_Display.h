/////////////////////////////////////////////
//Utility Display Ver.1.0, 2013.12.18////////
////////////////////////////////////////////
class Util_Display: public CKvClass
{
public:
	Util_Display();
	virtual ~Util_Display();

	////////////////////////////
	/////////Display////////////
	////////////////////////////
	//Plot 3D object from DoCube
	void po_Plot_Object(CKvGraph3D *in_scr,CKvDoCubeShort *in_object,CKvPmatrix3D *in_pmat_or_NULL											  );
	void po_Plot_Object(CKvGraph3D *in_scr,CKvDoCubeShort *in_object,CKvPmatrix3D *in_pmat_or_NULL,CKvRgbaF in_color,		 int in_point_size);
	void po_Plot_Object(CKvGraph3D *in_scr,CKvDoCubeShort *in_object,CKvPmatrix3D *in_pmat_or_NULL,CKvSet_of_RgbaF *in_color,int in_point_size);
	
	//Plot 3D object from set_of_point3D
	void po_Plot_Object(CKvGraph3D *in_scr,CKvSet_of_Point3D *in_point,CKvPmatrix3D *in_pmat_or_NULL                                            );
	void po_Plot_Object(CKvGraph3D *in_scr,CKvSet_of_Point3D *in_point,CKvPmatrix3D *in_pmat_or_NULL,CKvRgbaF in_color,        int in_point_size);
	void po_Plot_Object(CKvGraph3D *in_scr,CKvSet_of_Point3D *in_point,CKvPmatrix3D *in_pmat_or_NULL,CKvSet_of_RgbaF *in_color,int in_point_size);

	//Plot 3D mesh object from DoCube
	void pmo_Plot_Mesh_Object(CKvGraph3D *in_scr,CKvDepot_of_Point3D *in_depot_point,CKvDepot_of_RgbaF *in_depot_color,CKvMesh_of_Triangle *in_mesh_triangle,CKvPmatrix3D *in_pmat_or_NULL);
	
	//Plot 3D mesh object from set_of_point3D
	void pmo_Plot_Mesh_Object(CKvGraph3D *in_scr,CKvSet_of_Point3D *in_p3d, CKvMatrixInt *in_3xn_mesh);
	
	//Plot 3D mesh object from precomputed mesh elements
	void pmo_Plot_Mesh_Object(CKvGraph3D *in_scr,CKvDepot_of_Point3D *in_dp,CKvDepot_of_RgbaF *in_dc_or_NULL,CKvPmatrix3D *in_pmat_or_NULL,CKvMesh_of_Triangle *in_mesh_triangle);
	
	//Plot 3D mesh object using images
	void pmo_Plot_Mesh_Object(CKvGraph3D *in_scr,CKvDepot_of_Point3D *in_dp,CKvDepot_of_Point3D *in_sn_or_NULL,CKvDepot_of_Point *in_dp_img,CKvSet_of_String *in_texture_set,CKvPmatrix3D *in_pmat_or_NULL,CKvMesh_of_TriImage *in_mesh_tri_image);

	void pmosvd_Plot_Mesh_Object_Sequence_View_Dependent(
		CKvGraph3D *in_scr,
		int in_ww, int in_hh,
		int in_max_frame_num,
		float in_frame_rate,
		CKvSet_of_Pmatrix3D *in_set_of_pmats,
		CKvDepot_of_Point3D *in_set_of_depot_points,
		CKvDepot_of_RgbaF *in_depot_colors,
		CKvMesh_of_Triangle *in_set_of_mesh_triangle,
		CKvSet_of_VectorBool *in_set_of_per_mesh_vis,
		CKvSet_of_VectorUcharRgb *in_set_of_vertex_colors_for_all_cam);

	void pmosvd_Plot_Mesh_Object_Sequence_View_Dependent_NEW(
		CKvGraph3D *in_scr,
		int in_ww, int in_hh,
		int in_max_frame_num,
		float in_frame_rate,
		CKvSet_of_Pmatrix3D *in_set_of_pmats,
		CKvDepot_of_Point3D *in_set_of_depot_points,
		CKvDepot_of_RgbaF *in_depot_colors,
		CKvMesh_of_Triangle *in_set_of_mesh_triangle,
		CKvVectorUcharRgb *in_color_depot,
		CKvSet_of_VectorInt *in_vertex_color_indices_for_all_cam);

	void pmosvd_Plot_Mesh_Object_Sequence_View_Dependent_NEW_TOTAL(
		CKvGraph3D *in_scr,
		int in_ww, int in_hh,
		int in_max_frame_num,
		int in_num_of_GOP,
		float in_frame_rate,
		CKvSet_of_Pmatrix3D *in_set_of_pmats,
		CKvDepot_of_Point3D *in_set_of_depot_points,
		CKvDepot_of_RgbaF *in_depot_colors,
		CKvMesh_of_Triangle *in_set_of_mesh_triangle,
		CKvVectorUcharRgb *in_set_of_color_palette,
		CKvSet_of_VectorInt *in_vertex_color_indices_for_all_cam);

	void gi_Get_Images(
		CKvGraph3D *in_scr,
		int in_ww, 
		int in_hh,
		int in_current_frame,
		int in_num_of_GOP,
		float in_frame_rate,
		CKvSet_of_Pmatrix3D *in_set_of_pmats,
		CKvDepot_of_Point3D *in_set_of_depot_points,
		CKvDepot_of_RgbaF *in_depot_colors,
		CKvMesh_of_Triangle *in_set_of_mesh_triangle,
		CKvVectorUcharRgb *in_set_of_color_palette,
		CKvSet_of_VectorInt *in_vertex_color_indices_for_all_cam,
		CKvSet_of_MatrixUcharRgb *out_generated_imgs,
		CKvSet_of_MatrixUchar    *out_generated_depth);

	void ccwac_Compute_Color_Weights_for_All_Cameras(
		CKvSet_of_Pmatrix3D *in_P_matrix_set,	
		CKvPmatrix3D *in_virtual_viewpoint,
		CKvVectorFloat *out_color_weights_for_all_cameras);

	void vdtmm_View_Dependent_Texture_Mapping_on_Mesh(
		CKvSet_of_Pmatrix3D *in_P_matrix_set,				
		CKvSet_of_VectorBool *in_per_mesh_visibility,
		CKvSet_of_VectorUcharRgb *in_vertex_colors_for_all_cameras,		
		CKvVectorFloat *in_color_weights_for_all_cameras,				
		CKvPmatrix3D *in_virtual_viewpoint,		//int in_view_index, //
		CKvMesh_of_Triangle *io_mesh_triangle,
		CKvDepot_of_RgbaF *io_depot_color);

	void vdtmm_View_Dependent_Texture_Mapping_on_Mesh_NEW(
		//CKvDepot_of_Point3D *in_depot_point, 
		//CKvSet_of_MatrixUcharRgb *in_texture_set,
		CKvSet_of_Pmatrix3D *in_P_matrix_set,				
		CKvVectorUcharRgb *in_color_depot,
		CKvSet_of_VectorInt *in_vertex_color_indices_for_all_cameras,		
		CKvVectorFloat *in_color_weights_for_all_cameras,				
		CKvPmatrix3D *in_virtual_viewpoint,		//int in_view_index, //
		CKvMesh_of_Triangle *io_mesh_triangle,
		CKvDepot_of_RgbaF *io_depot_color);

	void cvc_Compute_Vertex_Color(
		bool *in_vertex_visibility,
		unsigned char *in_vertex_colors,
		float *in_color_weights,
		int in_camera_number,
		CKvRgbaF &out_vertex_color);

	void cvc_Compute_Vertex_Color_NEW(
		int *in_vertex_color_indices,
		CKvVectorUcharRgb *in_vertex_color_depot,
		float *in_color_weights,
		int in_camera_number,
		CKvRgbaF &out_vertex_color);

	///////////////////////////////////////////////////////////////////
	/////////Get depth maps and silhouettes by using OpenGL////////////
	//////////////////////////////////////////////////////////////////
	void gdi_Get_Depth_Images(CKvGraph3D *in_scr,CKvDepot_of_Point3D *in_dp,CKvDepot_of_RgbaF *in_dc,CKvMesh_of_Triangle *in_mesh,CKvSet_of_Pmatrix3D *in_pmat,int in_width,int in_height,CKvSet_of_MatrixFloat *out_depth);
	void gdi_Get_Depth_Images(CKvGraph3D *in_scr,CKvDepot_of_Point3D *in_dp,CKvDepot_of_RgbaF *in_dc,CKvMesh_of_Triangle *in_mesh,CKvSet_of_Pmatrix3D *in_pmat,	CKvSet_of_MatrixUcharRgb *in_imgs,CKvSet_of_MatrixFloat *out_depth);	
	void gdii_Get_Depth_Image_for_an_Image(CKvGraph3D *in_scr,CKvDepot_of_Point3D *in_dp,CKvDepot_of_RgbaF *in_dc,CKvMesh_of_Triangle *in_mesh,CKvPmatrix3D *in_pmat,int in_width,int in_height,CKvMatrixFloat *out_depth);
	
	void gsp_Get_Silhouettes(CKvGraph3D *in_scr,CKvDepot_of_Point3D *in_dp,CKvDepot_of_RgbaF *in_dc,CKvMesh_of_Triangle *in_mesh,CKvSet_of_Pmatrix3D *in_pmat,int *in_ww_hh,int in_dilation,CKvSet_of_SdkCode *out_sdk_set);
	void gspi_Get_Silhouette_for_an_Image(CKvGraph3D *in_scr,CKvDepot_of_Point3D *in_dp,CKvDepot_of_RgbaF *in_dc,CKvMesh_of_Triangle *in_mesh,CKvPmatrix3D *in_pmat,int *in_ww_hh,int in_dilation,CKvSdkCode *out_sdk_set);
		
	void ss_Set_Size(CKvGraph3D *in_scr,int in_ww,int in_hh);
			
	LCKvUtility_for_Windows zz_util_window;
	LCKvUtility_for_Import  zz_util_import;
	CKvRunSet zz_runset;
};