//********************************************************************************************
//class AFX_EXT_CLASS LCKvYooji_Scanner_Display : public CKvClass
class LCKvYooji_Scanner_Display : public CKvClass
//********************************************************************************************
{
public:
	/// constructor.
	LCKvYooji_Scanner_Display();
	virtual ~LCKvYooji_Scanner_Display();

	// Render images from TSDF cube.
	void rmi_Render_Maps_for_ICP(
		CKvYooji_Cube_TSDF_Float *in_cube,
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Scanner_Parameter *in_params,
		CKvYooji_Tracking_State *io_state);

	void rmi_Render_Maps_for_ICP(
		CKvYooji_Cube_TSDF_Float *in_cube,
		CKvYooji_Scanner_Parameter *in_params,
		CKvYooji_Tracking_State *io_state);

	void rinrc_Render_Image_Normal_on_RGB_Camera(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Tracking_State *in_state,
		CKvMatrixUcharRgb *out_rendered_normal);


	//////////////////////////////////////////////////////////////////////////
	void rn_Render_Normal_on_RGB_Image(
		CKvMatrixUcharRgb *in_img_rgb,
		CKvMatrixUchar *in_img_normal,
		CKvMatrixUcharRgb *out_img_rgb);
	//////////////////////////////////////////////////////////////////////////

	void rmd_Render_Map_Depth(
		CKvYooji_Cube_TSDF_Float *in_cube,		
		CKvYooji_Scanner_Parameter *in_params,
		CKvYooji_Intrinsics *in_intrinsics,
		CKvYooji_Extrinsics *in_extrinsics,
		CKvMatrixFloat *out_map_depth);

	void rin_Render_Image_Normal(
		CKvYooji_Cube_TSDF_Float *in_cube,		
		CKvYooji_Scanner_Parameter *in_params,
		CKvYooji_Intrinsics *in_intrinsics,
		CKvYooji_Extrinsics *in_extrinsics,
		CKvMatrixUchar *out_img_normal);

	bool rct_Render_Camera_Trace(
		CKvYooji_Extrinsics *in_view_ref,
		CKvYooji_Extrinsics *in_view_trace,
		CKvYooji_Intrinsics *in_intrinsics_ref,
		CKvYooji_Intrinsics *in_intrinsics_trace,
		CKvMatrixUcharRgb *io_img_trace,
		float in_ratio_between_camera_size_and_focal_length = 0.1f);

// 	bool rct_Render_Camera_Trace(
// 		CKvYooji_Extrinsics *in_view_ref,
// 		CKvYooji_Extrinsics *in_view_trace,
// 		CKvYooji_Intrinsics *in_intrinsics_ref,
// 		CKvYooji_Intrinsics *in_intrinsics_trace,
// 		CKvMatrixUcharRgb *io_img_trace,
// 		CKvRgb *in_cam_color = NULL,
// 		float in_ratio_between_camera_size_and_focal_length = 0.1f);

// 	bool rsct_Render_Set_of_Camera_Traces(
// 		CKvYooji_Extrinsics *in_view_ref,
// 		vector<CKvYooji_Extrinsics*> *in_set_of_views_trace,
// 		CKvYooji_Intrinsics *in_intrinsics_ref,
// 		CKvYooji_Intrinsics *in_intrinsics_trace,
// 		CKvMatrixUcharRgb *io_img_trace,
// 		float in_ratio_between_camera_size_and_focal_length = 0.1f);

	bool rct_Render_Camera_Trace(
		CKvYooji_Extrinsics *in_view_ref,
		CKvYooji_Extrinsics *in_view_trace,
		CKvYooji_Intrinsics *in_intrinsics_ref,
		CKvYooji_Intrinsics *in_intrinsics_trace,
		CKvMatrixUcharRgb *io_img_trace,
		CKvRgb *in_cam_color = NULL,
		float in_ratio_between_camera_size_and_focal_length = 0.1f,
		bool in_flag_center_only = false);

	bool rct_Render_Camera_Trace(
		CKvYooji_Extrinsics *in_view_ref,
		CKvYooji_Extrinsics *in_view_trace,
		CKvYooji_Intrinsics *in_intrinsics_ref,
		CKvYooji_Intrinsics *in_intrinsics_trace,
		CKvMatrixUcharRgb *io_img_trace,
		CKvRgb *in_cam_color,
		float in_ratio_between_camera_size_and_focal_length,
		bool in_flag_center_only,
		CKvRgb *in_center_color);

	bool rsct_Render_Set_of_Camera_Traces(
		CKvYooji_Extrinsics *in_view_ref,
		vector<CKvYooji_Extrinsics*> *in_set_of_views_trace,
		CKvYooji_Intrinsics *in_intrinsics_ref,
		CKvYooji_Intrinsics *in_intrinsics_trace,
		CKvMatrixUcharRgb *io_img_trace,
		float in_ratio_between_camera_size_and_focal_length = 0.1f,
		int in_sampling_rate = 1,
		CKvRgb *in_center_color = NULL);

	bool rsct_Render_Set_of_Camera_Traces_New(
		CKvYooji_Extrinsics *in_view_ref,
		vector<CKvYooji_Extrinsics*> *in_set_of_views_trace,
		CKvYooji_Intrinsics *in_intrinsics_ref,
		CKvYooji_Intrinsics *in_intrinsics_trace,
		CKvMatrixUcharRgb *io_img_trace,
		int in_idx_of_current_frame = -1,
		float in_ratio_between_camera_size_and_focal_length = 0.1f,
		int in_sampling_rate = 1,
		CKvRgb *in_center_color = NULL,
		CKvRgb *in_cam_color = NULL);

	bool rsct_Render_Set_of_Camera_Traces(
		CKvYooji_Extrinsics *in_view_ref,
		vector<CKvYooji_Extrinsics*> *in_set_of_views_trace,
		CKvYooji_Intrinsics *in_intrinsics_ref,
		CKvYooji_Intrinsics *in_intrinsics_trace,
		CKvMatrixUcharRgb *io_img_trace,
		float in_ratio_between_camera_size_and_focal_length = 0.1f,
		int in_sampling_rate = 1);

	// ray casting with step size in the voxel coordinates (/voxel).
	bool crtc_Cast_Ray_on_TSDF_Cube(
		CKvPixel &in_xy,
		CKvYooji_Cube_TSDF_Float *in_cube,
		CKvYooji_Intrinsics *in_intrinsic,
		CKvYooji_Extrinsics *in_pose,

		CKvPoint3Df &in_norm_cc,
		CKvPoint3Df &in_cam_cen,
		float in_radius_cube,
		float in_dist_cam_center_to_cube_center,
		float in_theta_max,

		float in_mu,
		float in_voxel_sz_inv,
		CKvPoint3Df &out_p3d);

	bool crtc_Cast_Ray_on_TSDF_Cube_NEW(
		CKvPixel &in_xy,
		CKvYooji_Cube_TSDF_Float *in_cube,
		CKvYooji_Intrinsics *in_intrinsic,
		CKvYooji_Extrinsics *in_pose,

		CKvPoint3Df &in_norm_cc,
		CKvPoint3Df &in_cam_cen,
		float in_radius_cube,
		float in_dist_cam_center_to_cube_center,
		float in_theta_max,

		float in_mu,
		float in_voxel_sz_inv,
		CKvPoint3Df &out_p3d);

	//
	void rdmrc_Render_Depth_Map_on_RGB_Camera(
		CKvYooji_MatrixRgbD *in_view);

	void pt_Put_Text_using_OpenCV(CKvMatrixUcharRgb &io_img_rgb);

	void dpc_Display_Point_Clouds(
		CKvMatrixFloat *in_mapDepth,
		CKvMatrixUcharRgb *in_imgColor,
		CKvYooji_Intrinsics *in_intrinsics);

	//////////////////////////////////////////////////////////////////////////
	void gcps_Get_Camera_Projection_Sphere(
		CKvPoint3D &in_center,
		float in_radius,
		CKvDepot_of_Point3D *out_depot_of_points,
		CKvDepot_of_RgbaF *out_depot_of_colors,
		CKvMesh_of_Triangle *out_mesh_tri);
	
	void dcps_Display_Camera_Projection_Sphere(
		CKvGraph3D *in_graph_3d,
		CKvYooji_Tracking_State *in_mat_rgbd,
		CKvDepot_of_Point3D *in_point_depot_sphere,
		CKvDepot_of_RgbaF *io_color_depot_sphere,
		CKvMesh_of_Triangle *in_mesh_triangle_sphere);
	//////////////////////////////////////////////////////////////////////////

	// Set initial cube.
	void gco_Get_Cube_Origin_in_world(
		CKvYooji_MatrixRgbD &in_rgbd_mat_init,
		float in_cube_size_in_meter,
		float in_z_offset_of_cube_in_meter,
		CKvPoint3Df &out_cube_origin);

	void soc_Set_Object_Cube(
		CKvYooji_MatrixRgbD &in_rgbd_mat_init,		
		float in_z_offset_of_cube_in_meter,
		float &io_cube_size_in_meter,
		CKvPoint3Df &out_cube_origin);

	// display
	void cidfu_Convert_Image_Depth_Float_to_Uchar(CKvMatrixFloat *in_img_d,
		int in_mode_convert,
		CKvMatrixUchar *out_img_d);
	void cidfu_Convert_Image_Depth_Float_to_Uchar(CKvMatrixFloat *in_img_d,
		int in_mode_convert,
		float in_min_depth,
		float in_max_depth,
		CKvMatrixUchar *out_img_d);
	void cidfu_Convert_Image_Depth_Float_to_Uchar(CKvMatrixFloat *in_img_d,
		int in_mode_convert,
		float in_default_val,
		CKvMatrixUchar *out_img_d);

	// local functions...
	void dvoi_Draw_Volume_Of_Interest(CKvMatrixFloat *in_imgDepth,
		CKvYooji_Intrinsics *in_intrinsics_d,
		CKvYooji_Extrinsics *in_extrinsics,
		float in_cube_size,
		CKvPoint3Df &in_cube_offset,
		CKvMatrixUcharRgb *out_img_VOI);

	void dvoi_Draw_Volume_Of_Interest(
		CKvMatrixFloat *in_img_depth,
		CKvMatrixUchar *in_img_normal,
		CKvYooji_Intrinsics *in_intrinsics_d,
		CKvYooji_Extrinsics *in_extrinsics,
		float in_cube_size,
		CKvPoint3Df &in_cube_offset,
		CKvMatrixUcharRgb *out_img_VOI);

	void dmr_Display_Matching_Result(CKvScreen *in_screen,
		CKvMatrixUcharRgb *in_img_src,
		CKvMatrixUcharRgb *in_img_dst,
		vector<Point2f> &in_p2d_matched_src,
		vector<Point2f> &in_p2d_matched_dst);

	void dmr_Display_Matching_Result(CKvScreen *in_screen,
		CKvMatrixUcharRgb *in_img_src,
		CKvMatrixUcharRgb *in_img_dst,
		int in_num_matched,
		float *in_matched_src,
		float *in_matched_dst);

	//////////////////////////////////////////////////////////////////////////
// 	void dgf_Display_Gradient_Field(
// 		CKvMatrixUcharRgb *out_img_grad_field,
// 		CKvMatrixUcharRgb *in_img_color,
// 		CKvSet2d_of_Pointf *in_set_of_grads,
// 		int in_level_of_magnification = 3,
// 		float in_ratio_of_gradient_magnitude = 0.1f);
	//////////////////////////////////////////////////////////////////////////

	void dtf_Display_Tracked_Flow(CKvScreen *in_screen,
		CKvMatrixUcharRgb *in_img_t1,
		CKvSet2d_of_Pointf *in_set_of_flows,
		int in_block_sz);

	void dtfsc_Display_Tracked_Flow_Pseudo_Color(CKvScreen *in_screen,
		CKvMatrixUcharRgb *in_img_t1,
		CKvSet2d_of_Pointf *in_set_of_flows,
		int in_block_sz);

	void dtf_Display_Tracked_Flow_HSV(CKvScreen *in_screen,
		CKvSet2d_of_Pointf *in_set_of_flows,
		int in_block_sz);

	void dtf_Draw_Tracked_Flow(CKvMatrixUcharRgb *out_img_flow,
		CKvMatrixUcharRgb *in_img_t1,
		CKvSet2d_of_Pointf *in_set_of_flows,
		int in_block_sz);

	void dtf_Draw_Tracked_Flow_HSV(
		CKvMatrixUcharRgb *out_img_flow,
		CKvSet2d_of_Pointf *in_set_of_flows,
		int in_block_sz);

	void dtf_Draw_Flow_Color_Coding_HSV(
		CKvMatrixUcharRgb *out_img_flow,
		int in_block_sz);

	//////////////////////////////////////////////////////////////////////////
	void cipc_Convert_Intensity_to_Pseudo_Color(
		float in_mag_inten,
		CKvRgb &out_pseudo_color);

	//////////////////////////////////////////////////////////////////////////

	void dnm_Display_Normal_Map(
		CKvScreen *in_screen,
		CKvSet2d_of_Point3Df *in_map_point);
	//////////////////////////////////////////////////////////////////////////

private:
	void z_mmpt_Make_Mesh_Points_with_Texture(
		CKvMatrixFloat *in_depth_map,
		CKvMatrixUcharRgb *in_color_image,
		CKvYooji_Intrinsics *in_intrinsics,
		CKvDepot_of_Point3D *out_depot_3d_points,
		CKvDepot_of_RgbaF *out_depot_colors,
		CKvMesh_of_Triangle *out_mesh_element);

private:
	/// these variables contains values temporarily during computation only.
	CKvYooji_Intrinsics zz_intrin_d, zz_intrin_rgb;
	CKvYooji_Extrinsics zz_extrin;
	CKvMatrixFloat zz_mat_4x4;

	// variables for 'crtc_Cast_Ray_on_TSDF_Cube'
	CKvPoint3Df zz_origin;
	CKvPoint3Df zz_ps_c, zz_pe_c;		// start and end points of the pixel ray in the camera coordinates.
	CKvPoint3Df zz_ps_g, zz_pe_g;		// start and end points of the pixel ray in the global coordinates.
	CKvPoint3Df zz_rd_g;				// the direction of the pixel ray in the global coordinates.
	CKvPoint3Df zz_ps_vox, zz_pe_vox;

	// variables for 'rct_Render_Camera_Trace'
	CKvPixel zz_cen_prev;

	// variables for Gaussian pyramid.
	CKvMatrixFloat zz_map_tmp[3];
	CKvMatrixUchar zz_img_tmp[3];	
	CKvMatrixInt zz_img_padding[3];

};