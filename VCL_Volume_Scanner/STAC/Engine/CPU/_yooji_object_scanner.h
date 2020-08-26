//********************************************************************************************
class CKvYooji_3D_Object_Scanner  : public CKvClass
//********************************************************************************************
{

public:
	CKvYooji_3D_Object_Scanner(void);
	virtual ~CKvYooji_3D_Object_Scanner(void);

 	bool hvo_Has_Valid_Output() { return zz_valid_output; }
 
 	int npf_Number_of_Processed_Frames() { return zz_number_of_processed_frames; }
 	int npf() { return zz_number_of_processed_frames; }
 	CKvYooji_Tracking_State* out_pss_OUTput_Pointer_of_Scanning_State(){ return &zz_state; }
	//////////////////////////////////////////////////////////////////////////
	CKvMatrixBool *out_hm_Hand_Mask(){ return &zz_hand_mask; }

	void grt_Get_Reference_Camera_for_Trace(CKvYooji_Intrinsics &out_intrins_ref,
		CKvYooji_Extrinsics &out_extrins_ref){
		out_intrins_ref.copy(&zz_int_ref); out_extrins_ref.copy(&zz_ext_ref);
	}
	//////////////////////////////////////////////////////////////////////////
 
	// for save.
	void sis_Save_Input_Sequence(CKvString &in_dn_depth, CKvString &in_dn_texture);
	void sss_Save_Silhouette_Sequence(CKvString &in_dn_sil);
	void itsdfcd_Import_TSDF_Cube_from_Device();
 	void stsdfc_Save_TSDF_Cube(CKvString &in_foder_path);
	void sdc_Save_Do_Cube(CKvString &in_foder_path);

	//////////////////////////////////////////////////////////////////////////
	// new for background cut.
	void is_Initialize_Scanner(
		CKvYooji_Scanner_Parameter &in_system_parameter,
		CKvYooji_MatrixRgbD &in_frame_of_rgbd_camera,
		bool in_flag_gpu = false,
		bool in_flag_sfs = false,
		bool in_flag_color = false,
		CKvYooji_ColorModel *in_color_model_of_hand = NULL,
		CKvYooji_ColorModel *in_gmm_back_pixel = NULL,
		CKvYooji_ColorModel *in_gmm_back_global = NULL);
	
	//////////////////////////////////////////////////////////////////////////
	void pis_Pre_Initialize_Scanner();
 	void rs_Release_Scanner();
	//////////////////////////////////////////////////////////////////////////
	
	// =============================================================
 	bool so_Scan_Object(
 		CKvYooji_MatrixRgbD *in_frame_of_rgbd_camera,
		bool in_flag_sfs = false,
		bool in_flag_on_rgb = false,
		int in_type_of_data = KV_TYPE_INPUT_OBJECT,
 		int in_mode_tracking_camera = KV_TYPE_TRACK_DEPTH,
		int in_level_of_pyramid = 3,
		bool in_mode_board_removal = false,
		bool in_flag_gpu = false,
		bool in_flag_time = false);
	//////////////////////////////////////////////////////////////////////////
	bool rcd_Recover_Camera_Drift(
		CKvYooji_MatrixRgbD *in_frame_of_rgbd_camera,
		CKvMatrixUcharRgb *out_results = NULL);

	float hmihwsbtn_Histogram_Matching_using_Intergral_Histogram_of_Weighted_Sub_Block_Tree(
		int in_ww_tar, int in_hh_tar,
		int in_ww_roi, int in_hh_roi,
		const float in_variance_ROI,				// if you do not want to use this, please enter any negative number.
		CKvHistogram *in_hist_ROI,
		CKvMatrixFloat *in_sum_image_target,
		CKvMatrixFloat *in_sum_of_square_image_target,
		CKvSet_of_MatrixInt **in_set_of_integral_hist_target,
		CKvSet_of_MatrixFloat *in_set_of_sub_block_position_ratios,
		CKvSet_of_VectorFloat *in_set_of_weight_vectors,
		int in_mask_sz_of_hist_filter,
		int in_minimum_block_size,
		int in_vector_distance_type,
		int in_normalize_mode,
		int in_dominant_gradient_reduction_mode,
		int in_max_window_width,
		int in_min_window_width,
		int in_window_size_delta,
		float in_window_move_ratio,
		int in_show_similarity_map,
		CKvPoint *out_LT,
		CKvPoint *out_RB,
		int &out_tree_level,
		bool in_mode_dge = false);
	//////////////////////////////////////////////////////////////////////////



	bool rbd_Remove_Board_Depth(
		CKvYooji_MatrixRgbD *in_frame_of_rgbd_camera);
	// =============================================================

 
 	bool out_tif_Tracked_Information_of_a_Frame(int in_idx_frame,
 		CKvYooji_Tracking_State *out_current_state);
 	void out_ccti_Current_Camera_Trace_Image(
 		CKvMatrixUcharRgb &out_img_trace,
 		bool in_flag_for_scanning_result = false,
		int in_sampling_rate = 5);

	// pointers of trace.
	vector<CKvYooji_Extrinsics*>* p_trace_estimated(){ return &zz_trace; }
	vector<CKvYooji_Extrinsics*>* p_trace_gt(){ return &zz_trace_gt; }


private:
	// main functions.
	bool z_lid_Load_Input_Data(bool in_flag_gpu = false);

	void z_uid_Update_Input_Data(CKvYooji_MatrixRgbD *in_frame_of_rgbd_camera, 
		bool in_mode_board_remove = false,
		bool in_flag_gpu = false);
	void z_uvv_Update_Valid_Volume();
	bool z_ro_Remove_Occlusion();
	void z_cip_Construct_Image_Pyramids(
		int in_lev_pyram,
		int in_mode_track,
		bool in_flag_gpu);
	bool z_ecp_Estimate_Camera_Pose(
		int in_in_level_of_pyramid,
		int in_mode_tracking_camera,
		bool in_flag_gpu = false);

	void z_std_Setting_for_Trace_Display();
	void z_uom_Update_Object_Model(
		bool in_flag_color,
		bool in_flag_gpu = false);
	void z_uts_Update_Tracking_State(
		int in_mode_tracking_camera,
		int in_type_of_data = KV_TYPE_INPUT_OBJECT,
		int in_level_of_pyram = KV_LEVEL_OF_IMAGE_PYRAMID,
		bool in_flag_gpu = false);
	void z_esp_Evaluate_System_Performance();

	
	//
	//////////////////////////////////////////////////////////////////////////
	void z_gkt_Get_Key_Template();
	//////////////////////////////////////////////////////////////////////////

	void z_gmc_Get_Mesh_of_Cube();

	int z_god_Get_Object_Depth(
		CKvYooji_MatrixRgbD *in_frame_of_rgbd_camera,
		int in_capture_mode);

	void z_tdrcc_Transform_Depth_to_Rgb_Camera_Coordinates(
		CKvYooji_MatrixRgbD *in_rgbd_img,
		CKvMatrixFloat *in_img_d,
		CKvMatrixFloat *out_img_d_transformed);

	void z_srctd_Set_Reference_Camera_for_Trace_Display(
		CKvYooji_Extrinsics &in_pose_init,
		CKvYooji_Intrinsics &out_intrins_ref,
		CKvYooji_Extrinsics &out_extrins_ref);

	void z_uct_Update_Camera_Trace(
		CKvYooji_Tracking_State &in_tracking_state,
		vector<CKvYooji_Extrinsics*> *io_set_of_traces);


public:
private:

	Util_VCL_3DO zz_3do;

	LCKvIO_FileVcl zz_iof;
	LCKvUtility_for_YCbCr zz_ycc;
	LCKvUtility_for_Import zz_im;
	CKvYooji_ContourDetection zz_cd;

	// //////////////////////////////////////////////
	// for device functions. 
	// //////////////////////////////////////////////
	LGKvImageProcessor g_ip;
	LGKvPoseTracker g_track;
	LGKvVolumeIntegrator g_integ_d;
	LGKvVolumeIntegrator g_integ_obj;
	LGKvRendererTSDF g_render_obj, g_render_trace;
	GKvObjectCubeFloat g_cube_obj;
	GKvVolumeBool g_vol_valid;
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	GKvRgbdFrame g_rgbd_frame;
	GKvTrackingState g_state_obj, /*g_state_obj_raw, */g_state_trace;
// 	float *g_map_depth_t0;
// 	float *g_map_normal_t0;	// 3xN
// 	uchar *g_img_normal_t0;
	float *g_T_gc_t0,*g_T_cg_t0;
	float *g_T_gc_t1,*g_T_cg_t1;
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	GKvMatrixFloat g_map_depth_t1, g_map_depth_t1_raw, g_map_depth_t1_filt;
	GKvMatrixUchar g_img_color_t1;
 	float *g_map_depth_t1_raw_;
 	float *g_map_depth_t1_filt_;
 	uchar *g_img_color_t1_;

	float *zz_map_depth;
	float *zz_pointer_p3d;
	
	// for input depth map reprojection.
	LGKvVolumeIntegrator g_integ_rgb_d;
	LGKvVolumeIntegrator g_integ_rgb;
	LGKvRendererTSDF g_render_rgb_d;
	GKvObjectCubeFloat g_cube_rgb_d;
	GKvTrackingState g_state_rgb_d;
	GKvMatrixFloat g_T_mat;
	float *g_eye4; // 4x4 identity matrix.
	bool g_mode_rgb;

	// tracking state.

	// //////////////////////////////////////////////

	CKvYooji_FrameCapture zz_capture;
	LCKvYooji_Camera_Tracker zz_tracker;
	LCKvImageMatcher zz_imatch;
	LCKvYooji_Image_Processor zz_ip;
 	LCKvYooji_Volume_Integrator zz_integ;
 	LCKvYooji_Shape_from_Silhouette zz_sfs;
 	LCKvYooji_Scanner_Display zz_disp;
 	LCKvYooji_Evaluation zz_eval;		

	CKvYooji_Scanner_Parameter zz_params;
	CKvYooji_MatrixRgbD zz_rgbd_frame;	
	CKvYooji_Cube_TSDF_Float zz_cube;	
	CKvYooji_Tracking_State zz_state;

	CKvYooji_MatrixRgbD zz_rgbd_frame_for_sfs;
	CKvYooji_Tracking_State zz_state_for_sfs;
 
	//////////////////////////////////////////////////////////////////////////
	CKvYooji_Extrinsics zz_extrins_d_rgb, zz_extrins_rgb_d;
	CKvYooji_Extrinsics zz_extrins_t0_t1;
	//////////////////////////////////////////////////////////////////////////

	CKvYooji_Intrinsics zz_intrins_depth, zz_intrins_rgb;
	CKvYooji_Extrinsics zz_extrins_init_depth, zz_extrins_init_rgb;
	CKvYooji_Extrinsics zz_extrins_canonical;
	int zz_number_of_processed_frames;
	bool zz_valid_output;
	bool zz_flag_integ, zz_flag_track;
	
	//// for display.
	//CKvScreen zz_sc[4];
	// for camera trace.	
  	CKvYooji_Tracking_State zz_state_trace;
  	vector<CKvYooji_Extrinsics*> zz_trace;
	vector<CKvYooji_Extrinsics*> zz_trace_gt;
	//CKvYooji_Extrinsics zz_trace[1000];
	CKvYooji_Extrinsics zz_ext_ref;
	CKvYooji_Intrinsics zz_int_ref;
	Vector3f zz_cam_cen_trace, zz_light_trace;
	CKvVectorFloat zz_v_prin_axis, zz_v_center;
// 
 	CKvMatrixFloat zz_mat_4x4;
 	CKvYooji_Intrinsics zz_intrin;
 	CKvYooji_Extrinsics zz_extrin;

	// for SFS.	
	CKvGraph3D zz_g3d;
	LCKvUtility_for_Windows aa_uw;
 	LCKvAlgebra_for_MatrixFloat zz_alg_mf;
	
	//CKvSet2d_of_VectorFloat zz_docube_segments;//CKvDoCubeFloat zz_docube;
	// ========================================
	// Visual-hull shield rendering 을 위한 mesh data.
	// Main tracker 에서 visual-hull shield 를 사용하지 않을 때,
	// update 된 visual-hull shield 의 mesh 정보를 이 곳으로 update.
	// Visual data 는 LCKvYooji_Shape_from_Silhouette 내부에 존재.
	CKvDepot_of_Point3D zz_depot_p3d_vhs, zz_depot_p3d_cube;
	CKvVectorInt zz_idx_pnts_vhs, zz_idx_pnts_cube;
	CKvVolumeBool zz_vol_valid;
	CKvPoint3Df zz_origin_cube;
	float zz_sz_cube;
	// ========================================

	CKvMatrixBool zz_mask_obj;
	CKvMatrixFloat zz_depth_vhs;
	CKvMatrixInt zz_buffer_vhs;

	bool zz_call_scan_to_shield, zz_call_shield_to_scan;
	bool zz_call_vhs_mesh_update, zz_call_vhs_valid_update;
	bool zz_init_shield, zz_flag_occlusion_removal;

	// for Gaussian pyramid.
	CKvMatrixUchar zz_img;

	// 이 부분이 프로젝트를 불안정하게 함....===============================================================
	LCKvYooji_Object_Extractor zz_obj_ext;
	// ===============================================================
	CKvYooji_ColorModel *zz_gmm_hand;
	CKvMatrixBool zz_hand_mask;

	//////////////////////////////////////////////////////////////////////////
	// for Aruco module.
	CKvYooji_InterLib_Convert zz_ilc;
// 	float vals[] ={FX,0.,CX,0.,FY,CY,0.,0.,1.};
// 	float coeffs[] ={0.,0.,0.,0.,0.,};

	Mat zz_cameraMatrix;
	Mat zz_distCoeffs;;

	cv::Ptr<cv::aruco::Dictionary> zz_dictionary;
	cv::Ptr<cv::aruco::GridBoard> zz_board;
	cv::Ptr<aruco::DetectorParameters> zz_detectorParams;
	//////////////////////////////////////////////////////////////////////////

	/// for UI based on camera projection sphere. 	
	CKvMesh_of_Triangle zz_mesh_sphere;
	CKvDepot_of_Point3D zz_p3d_sphere;
	CKvDepot_of_RgbaF zz_rgbf_sphere;

	//////////////////////////////////////////////////////////////////////////
	// for dominant gradient template.
	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	// Multiple template matching
	CKvContourDetectionJM zz_jm;
	vector<CKvMatrixUchar> zz_set_of_templates;
	CKvYooji_Tracking_State zz_set_of_key_states[MAX_TMPL_NUM];
	//vector<GKvTrackingState> set_of_key_states_gpu;

	float zz_cover_prev;

	bool zz_flag_relocal;
	int zz_cnt_sphere_last;
	//////////////////////////////////////////////////////////////////////////

	/// ///////////////////////////////////////////////////////////////////////////
	CKvSet_of_MatrixInt *zz_integral_hist_ori;
	CKvSet_of_VectorFloat weight_vector_tree[MAX_TMPL_NUM];
	CKvSet_of_MatrixFloat sub_block_tree[MAX_TMPL_NUM];
	CKvSet_of_VectorInt valid_block_indice_tree[MAX_TMPL_NUM];

	CKvHistogram zz_hist_ROI[MAX_TMPL_NUM],zz_hist_ori;
	CKvPoint zz_LT[MAX_TMPL_NUM],zz_RB[MAX_TMPL_NUM];

	int zz_min_block_sz,zz_out_tr_level;
	int zz_ori_ww,zz_ori_hh,zz_roi_ww[MAX_TMPL_NUM],zz_roi_hh[MAX_TMPL_NUM],zz_num_imgs_target,zz_num_imgs_tmpl;
	float zz_var_ROI[MAX_TMPL_NUM],zz_scale,zz_dist_max;

	CKvScreen m_sc[2];

	//////////////////////////////////////////////////////////////////////////

	CKvScreen zz_sc_ios;
	CKvStopWatch sw;
	float elapsed_t[7],avg_t;	int cnt;

	// save.
	CKvSet_of_MatrixFloat zz_set_of_input_depth;
	CKvSet_of_MatrixUcharRgb zz_set_of_input_rgb;
	CKvSet_of_MatrixBool zz_set_of_input_sil;
};