//********************************************************************************************
//class AFX_EXT_CLASS LCKvYooji_CameraTracker : public CKvClass
// + This class contains the functions for camera(object) pose tracking.
class LCKvYooji_Camera_Tracker : public CKvClass
//********************************************************************************************
{
public:

	LCKvYooji_Camera_Tracker();
	~LCKvYooji_Camera_Tracker();

	// ==============================================================
	// Camera pose estimation.
	// ==============================================================
	//////////////////////////////////////////////////////////////////////////
	bool tcsift_Track_Camera_with_SIFT(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Scanner_Parameter *in_params,
		CKvYooji_Tracking_State *in_state,
		LCKvImageMatcher *in_matcher,
		CKvMatrixFloat *out_mat_pose);

	bool tccpnp_Track_Camera_Coarse_using_PnP(		
		vector<Point2f> &in_p2d_matched_t0,
		vector<Point2f> &in_p2d_matched_t1,
		CKvSet2d_of_Point3Df &in_map_p3d_t0,
		CKvYooji_Intrinsics &in_intrinsics,
		CKvMatrixFloat *out_pose_init);

	bool tcdfm_Track_Camera_with_Depth_Frame_to_Model(CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Scanner_Parameter *in_params,
		CKvYooji_Tracking_State *in_state,
		bool in_mode_forward_params = false,
		bool in_mode_on_rgb = false,
		CKvMatrixFloat *in_init_pose = NULL);

	// + Basic incremental ICP. (w/ coarse-to-fine approach)
	bool tccf_Track_Camera_Coarse_to_Fine_with_KinFu(CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Scanner_Parameter *in_params,
		CKvYooji_Tracking_State *in_state,
		bool in_mode_on_rgb = false);

	int gls_Generate_Linear_System_ICP_for_KinFu(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Tracking_State *in_state,
		CKvMatrixFloat *in_hmat_cam_to_glob_estimated,
		CKvMatrixFloat *out_ATA,
		CKvVectorFloat *out_ATb,
		int in_level_of_pyramid,
		float in_th_dist_ICP,
		bool in_param_mode_forward = false,
		bool in_mode_on_rgb = false);

	int gls_Generate_Linear_System_ICP_for_DVO(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Tracking_State *in_state,
		CKvMatrixFloat *in_hmat_cam_to_glob_estimated,
		CKvMatrixFloat *out_ATA,
		CKvVectorFloat *out_ATb,
		int in_level_of_pyramid,
		float in_th_dist_ICP,
		bool in_param_mode_forward = false,
		bool in_mode_on_rgb = false);

	// + Photoconsistency maximization.
	bool tccadvo_Track_Camera_using_Confidence_Adaptive_DVO(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Scanner_Parameter *in_params,
		CKvYooji_Tracking_State *in_state,
		CKvMatrixFloat *in_pose_init = NULL);

	bool tcpm_Track_Camera_with_Photoconsist_Max_Yooji_Weighted(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Scanner_Parameter *in_params,
		CKvYooji_Tracking_State *in_state);

	int glsew_Generate_Least_Square_Equation_with_Photoconsist_Max_Yooji_Weighted(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Tracking_State *in_state,
		CKvSet2d_of_Point3Df *in_map_p3d,
		CKvMatrixFloat *in_map_diff,
		CKvMatrixFloat *in_map_weight,
		///////////////////////////////////////////////////////////////////////////
		CKvMatrixBool *in_map_valid,
		//////////////////////////////////////////////////////////////////////////
		int in_level_of_pyramid,
		CKvMatrixFloat *out_ATA,
		CKvVectorFloat *out_ATb);

	int glhlse_Generate_Left_Hands_of_Least_Square_Equation(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Tracking_State *in_state,
		CKvSet2d_of_VectorFloat *in_map_jacobi,
		///////////////////////////////////////////////////////////////////////////
		CKvMatrixBool *in_map_valid,
		//////////////////////////////////////////////////////////////////////////
		int in_level_of_pyramid,
		CKvMatrixFloat *out_ATA);

	int grhlse_Generate_Right_Hands_of_Least_Square_Equation(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Tracking_State *in_state,
		CKvSet2d_of_VectorFloat *in_map_jacobi,
		CKvMatrixFloat *in_map_diff,
		///////////////////////////////////////////////////////////////////////////
		CKvMatrixBool *in_map_valid,
		//////////////////////////////////////////////////////////////////////////
		int in_level_of_pyramid,
		CKvVectorFloat *out_ATb);

	//////////////////////////////////////////////////////////////////////////
	int glhlse_Generate_Left_Hands_of_Least_Square_Equation(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Tracking_State *in_state,
		CKvSet2d_of_MatrixFloat *in_map_jacobi,
		///////////////////////////////////////////////////////////////////////////
		CKvMatrixBool *in_map_valid,
		//////////////////////////////////////////////////////////////////////////
		int in_level_of_pyramid,
		CKvMatrixFloat *out_ATA);

	int grhlse_Generate_Right_Hands_of_Least_Square_Equation(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Tracking_State *in_state,
		CKvSet2d_of_MatrixFloat *in_map_jacobi,
		CKvSet2d_of_VectorFloat *in_map_diff,
		///////////////////////////////////////////////////////////////////////////
		CKvMatrixBool *in_map_valid,
		//////////////////////////////////////////////////////////////////////////
		int in_level_of_pyramid,
		CKvVectorFloat *out_ATb);
	//////////////////////////////////////////////////////////////////////////

	int glsew_Generate_Least_Square_Equation_with_Photoconsist_Max_Yooji_Weighted(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Tracking_State *in_state,
		CKvSet2d_of_VectorFloat *in_map_jacobi,
		CKvMatrixFloat *in_map_diff,
		CKvMatrixFloat *in_map_weight,
		///////////////////////////////////////////////////////////////////////////
		CKvMatrixBool *in_map_valid,
		//////////////////////////////////////////////////////////////////////////
		int in_level_of_pyramid,
		CKvMatrixFloat *out_ATA,
		CKvVectorFloat *out_ATb);


	//////////////////////////////////////////////////////////////////////////
 	float cidm_Compute_Intensity_Difference_Map(
 		CKvYooji_MatrixRgbD *in_view,
 		CKvYooji_Tracking_State *in_state,
 		CKvMatrixBool *in_map_valid,
 		CKvSet2d_of_Pointf *in_map_reproj,
 		int in_level_of_pyramid,
 		CKvMatrixFloat *out_map_diff);


	float cgmdm_Compute_Gradient_Magnitude_Difference_Map(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Tracking_State *in_state,
		CKvMatrixBool *in_map_valid,
		CKvSet2d_of_Pointf *in_map_reproj,
		int in_level_of_pyramid,
		CKvMatrixFloat *out_map_diff);

	int cgdm_Compute_Gradient_Difference_Map_New(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Tracking_State *in_state,
		CKvMatrixBool *in_map_valid,
		CKvSet2d_of_Pointf *in_map_reproj,
		CKvMatrixFloat *in_hmat_t0_t1,
		int in_level_of_pyramid,
		CKvSet2d_of_VectorFloat *out_map_diff);
	//////////////////////////////////////////////////////////////////////////

	int cwmtd_Compute_Weight_Map_based_on_T_Distribution(
		CKvMatrixFloat *in_map_diff,
		CKvMatrixBool *in_map_valid,
		CKvMatrixFloat *out_map_weight,
		int in_num_mode = 5);

	int ivm_Initialize_Validity_Map(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Tracking_State *in_state,
		int in_level_of_pyramid,
		CKvMatrixBool *out_map_valid);

	int ivm_Initialize_Validity_Maps(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Tracking_State *in_state,
		int in_level_of_pyramid,
		CKvMatrixBool *out_map_valid_pm,
		CKvMatrixBool *out_map_valid_grad,
		CKvMatrixBool *out_map_valid_icp);

	int cvm_Compute_Validity_Map_for_PhotoMax(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Tracking_State *in_state,
		int in_level_of_pyramid,
		CKvMatrixBool *out_map_valid);

	int cjpt_Compute_Jacobian_Projection_and_Transformation(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Tracking_State *in_state,
		CKvMatrixBool *in_map_valid,
		int in_level_of_pyramid,
		CKvSet2d_of_MatrixFloat *out_map_jacobian);

	int cjpti_Compute_Jacobian_Projection_and_Transformation_Iterative(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Tracking_State *in_state,
		CKvMatrixBool *in_map_valid,
		CKvSet2d_of_Point3Df *in_map_p3d_warped,
		int in_level_of_pyramid,
		CKvSet2d_of_MatrixFloat *out_map_jacobian);

	int cjpm_Compute_Jacobian_PhotoMax(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Tracking_State *in_state,
		CKvSet2d_of_Pointf *in_map_reproj,
		CKvSet2d_of_MatrixFloat *in_map_jacobian_proj_and_trans,
		int in_level_of_pyramid,
		CKvMatrixBool *io_map_valid,
		CKvSet2d_of_VectorFloat *out_map_jacobian);

	int cjgm_Compute_Jacobian_Gradinet_Magnitude(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Tracking_State *in_state,
		CKvSet2d_of_Pointf *in_map_reproj,
		CKvSet2d_of_MatrixFloat *in_map_jacobian_proj_and_trans,
		int in_level_of_pyramid,
		CKvMatrixBool *io_map_valid,
		CKvSet2d_of_VectorFloat *out_map_jacobian);

	int cjgm_Compute_Jacobian_Gradient_based_Method_New(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Tracking_State *in_state,
		CKvSet2d_of_Pointf *in_map_reproj,
		CKvSet2d_of_MatrixFloat *in_map_jacobian_proj_and_trans,
		CKvSet2d_of_MatrixFloat *in_map_jacobian_plane_induced_homo,
		int in_level_of_pyramid,
		CKvMatrixBool *io_map_valid,
		CKvSet2d_of_MatrixFloat *out_map_jacobian);

	//////////////////////////////////////////////////////////////////////////
	int crp2dm_Compute_Reprojected_Point2D_Map(
		CKvYooji_MatrixRgbD *in_view,
		CKvSet2d_of_Point3Df *in_map_p3d,
		CKvMatrixBool *in_map_valid_pm,
		CKvMatrixBool *in_map_valid_icp,
		int in_level_of_pyramid,
		CKvSet2d_of_Pointf *out_map_reproj);

	int cp3dm_Compute_Point3D_Map(
		CKvYooji_Tracking_State *in_state,
		CKvMatrixFloat *in_hmat_glob_to_cam_rgb_est,
		CKvMatrixBool *in_map_valid_pm,
		CKvMatrixBool *in_map_valid_icp,
		int in_level_of_pyramid,
		CKvSet2d_of_Point3Df *out_map_p3d);

	int crp2dm_Compute_Reprojected_Point2D_Map(
		CKvYooji_MatrixRgbD *in_view,
		CKvSet2d_of_Point3Df *in_map_p3d,		
		CKvMatrixBool *in_map_valid,
		int in_level_of_pyramid,
		CKvSet2d_of_Pointf *out_map_reproj);

	int cnm_Compute_Normal_Map(
		CKvYooji_MatrixRgbD *in_view,
		CKvMatrixBool *in_map_valid,
		CKvSet2d_of_Pointf *in_map_reproj,
		int in_level_of_pyramid,
		CKvSet2d_of_Point3Df *out_map_norm);

	int cp3dm_Compute_Point3D_Map(
		CKvYooji_Tracking_State *in_state,
		CKvMatrixFloat *in_hmat_glob_to_cam_rgb_est,
		CKvMatrixBool *in_map_valid,
		int in_level_of_pyramid,
		CKvSet2d_of_Point3Df *out_map_p3d);
	
	int cp3dm_Compute_Point3D_Map(
		CKvYooji_Tracking_State *in_state,
		CKvMatrixFloat *in_hmat_glob_to_cam_rgb_est,
		int in_level_of_pyramid,
		CKvSet2d_of_Point3Df *out_map_p3d);

	int cp3dm_Compute_Point3D_Map(
		CKvYooji_Tracking_State *in_state,
		CKvMatrix *in_hmat_glob_to_cam_rgb_est,
		int in_level_of_pyramid,
		CKvSet2d_of_Point3Df *out_map_p3d);

	//////////////////////////////////////////////////////////////////////////
	// level independent functions.
	int cp3dm_Compute_Point3D_Map(
		CKvSet2d_of_Point3Df *in_map_p3d_src,
		CKvMatrixFloat *in_hmat_src_to_dst,
		CKvMatrixBool *in_map_valid,
		CKvSet2d_of_Point3Df *out_map_p3d_dst);
	//////////////////////////////////////////////////////////////////////////
		
	bool sls_Solve_Linear_System(
		CKvMatrixFloat *in_ATA,
		CKvVectorFloat *in_ATb,
		CKvVectorFloat *out_x);
	
	// For KinectFusion.
	bool uitcg_Update_Incremental_Tracking_Cam_to_Glob_with_Backward_Params(
		CKvVectorFloat *in_x,
		CKvMatrixFloat *io_hmat_4x4);

	bool uitcg_Update_Incremental_Tracking_t1_to_t0_with_Backward_Parameters(
		CKvVectorFloat *in_x,
		CKvMatrixFloat *io_hmat_t1_to_t0_est);

	bool umf_Update_Motion_Forward(
		CKvVectorFloat *in_x,
		CKvMatrixFloat *io_hmat_t1_to_t0_est,
		bool in_mode_param_forward = true);

	bool umb_Update_Motion_Backward(
		CKvVectorFloat *in_x,
		CKvMatrixFloat *io_hmat_t1_to_t0_est,
		bool in_mode_param_forward = true);


	bool cmv_Check_Motion_Validity(
		CKvMatrixFloat *io_hmat_4x4,
		float in_max_translation,	// m
		float in_max_rotation);		// rad.

	bool cmv_Check_Motion_Validity(
		CKvMatrix *io_hmat_4x4,
		float in_max_translation,	// m
		float in_max_rotation);		// rad.

	bool cmv_Check_Cube_Position(
		CKvYooji_Cube_TSDF_Float *in_cube,
		CKvYooji_Tracking_State *in_state);
	

private:
	bool z_ss_Single_Summand_ICP_Backward(
		CKvPixel &in_xy,
		float in_depth_view,
		float *in_depth_map_model,
		CKvSet2d_of_Point3Df *in_p3d_model,
		CKvSet2d_of_Point3Df *in_normal_map_model,
		int in_ww,int in_hh,
		CKvYooji_Intrinsics *in_K,
		CKvMatrixFloat *in_hmat_t1_to_t0_estimated,
		CKvYooji_Extrinsics *in_pose_prev,
		float in_th_dist_ICP,
		float out_A[],
		float &out_b,
		bool in_mode_param_forward = true);

	bool z_ss_Single_Summand_Photo_Max_Forward(
		CKvPointf &in_p2d_t0,
		const float in_inten_t0,
		const float in_depth_t0,
		const bool *in_sil_t1,
		const unsigned char *in_img_gray_t1,
		const float *in_grad_x_t1,
		const float *in_grad_y_t1,
		const int in_ww,const int in_hh,
		CKvYooji_Intrinsics *in_K,
		CKvMatrixFloat *in_pose_t0_to_t1_estimated,
		float out_A[],
		float &out_b);

	template<typename T>
	bool z_ccpm_Compute_Coefficients_with_Photo_Max_Yooji(
		T **in_J_pit,
		const float in_Ix,const float in_Iy,
		const float in_fx,const float in_fy,
		T out_A[]); // 1x6

	template<typename T>
	bool z_cjpt_Compute_Jacobians_of_Projection_and_Transformation(
		CKvPoint3Df &in_p3d_t0,
		T **out_J);
	
	template<typename T>
	void z_uls_Update_Linear_System(float in_A[],
		T in_b,
		T *io_ATA,
		T *io_ATb);

	template<typename T>
	void z_ulsw_Update_Linear_System_Weighted(T in_A[],
		T in_b,
		T in_weight,
		T *io_ATA,
		T *io_ATb);

	bool z_gii_Get_Interpolated_Intensity(float in_x, float in_y,
		int in_ww, int in_hh,
		unsigned char *in_img_gray,
		float &out_intensity);

	bool z_gig_Get_Interpolated_Gradient(float in_x, float in_y,
		int in_ww, int in_hh,
		short *in_map_grad,
		float &out_grad);

	bool z_gim_Get_Interpolated_Magnitude(float in_x,float in_y,
		int in_ww,int in_hh,
		float *in_img_mag,
		float &out_intensity);

	bool z_gig_Get_Interpolated_Gradient(float in_x,float in_y,
		int in_ww,int in_hh,
		float *in_map_grad,
		float &out_grad);

	bool z_gid_Get_Interpolated_Depth(float in_x, float in_y,
		int in_ww, int in_hh,
		float *in_depth_map,
		float &out_depth);

	bool z_gip3d_Get_Interpolated_Point3D(float in_x,float in_y,
		int in_ww,int in_hh,
		CKvSet2d_of_Point3Df *in_p3d_map,
		CKvPoint3Df &out_p3d);

	bool z_gin_Get_Interpolated_Normal(float in_x, float in_y,
		int in_ww, int in_hh,
		CKvSet2d_of_Point3Df *in_normal_map,
		CKvPoint3Df &out_norm);

	bool z_tp3d_Transform_Point3D(CKvMatrixFloat *in_mat,
		CKvPoint3Df &in_p3d,
		CKvPoint3Df &out_p3d);	

	bool z_tp3d_Transform_Point3D(CKvMatrix *in_mat,
		CKvPoint3Df &in_p3d,
		CKvPoint3Df &out_p3d);

	bool z_tp3di_Transform_Point3D_Inverse(CKvMatrixFloat *in_mat,
		CKvPoint3Df &in_p3d,
		CKvPoint3Df &out_p3d);

public:
private:
	LCKvUtility_for_Linear_Equation_Float aa_lef;
	LCKvUtility_for_YCbCr aa_ycc;
	LCKvUtility_for_Import aa_im;
	CKvYooji_InterLib_Convert aa_ilib;

	CKvScreen sc[2];

	LCKvYooji_Scanner_Display aa_disp;
	LCKvImageMatcher aa_imatch;
	LCKvYooji_Image_Processor aa_ip;
	CKvYooji_ContourDetection zz_cd;

	/// these variables contains values temporarily during computation only.
	// variables for 'tc_Track_Camera'
	CKvYooji_Intrinsics zz_intrinsics;
	CKvYooji_Intrinsics zz_intrinsics_rgb;
	CKvYooji_Extrinsics zz_transform_rgb_to_d;
	CKvYooji_Extrinsics zz_transform_glob_to_cam;
	CKvYooji_Extrinsics zz_transform_glob_to_cam_rgb;

	CKvMatrixFloat zz_mat_4x4;
	CKvMatrix zz_mat_4x4_d;
	CKvMatrixFloat zz_hmat_t0_to_t1_estimated;
	CKvMatrixFloat zz_hmat_t1_to_t0_estimated;
	CKvMatrix zz_hmat_t0_to_t1_estimated_d;
	CKvMatrix zz_hmat_glob_to_cam_d;

	CKvMatrixFloat zz_hmat_glob_to_cam_estimated;
	CKvMatrixFloat zz_hmat_cam_to_glob_estimated;
	CKvMatrixFloat zz_ATA_6x6[4], zz_ATA_3x3, zz_ATA_2x2;
	CKvVectorFloat zz_ATb_6[4], zz_sol_x_6;
	CKvVectorFloat zz_ATb_3, zz_sol_x_3;
	CKvVectorFloat zz_ATb_2, zz_sol_x_2;

	CKvMatrix zz_ATA_6x6_d,zz_ATA_3x3_d,zz_ATA_2x2_d;
	CKvVector zz_ATb_6_d,zz_sol_x_6_d;
	CKvVector zz_ATb_3_d,zz_sol_x_3_d;
	CKvVector zz_ATb_2_d,zz_sol_x_2_d;

	//Vector3f *zz_p3d_matched_prev, *zz_p3d_matched_curr;
	//Vector2f *zz_p2d_matched_prev,*zz_p2d_matched_curr;

	//CKvPixel *zz_pix_selected;
	//CKvPixel *zz_pix_inliers,*zz_pix_outliers;
	//CKvPixel *zz_max_pix_inliers,*zz_max_pix_outliers;
	CKvMatrixBool zz_img_check;
	CKvMatrixUchar zz_img_gray_prev,zz_img_gray_curr;
	CKvMatrixFloat zz_pose_est_inv,zz_pose_est_inv_ransac;

	// variables for 'z_cssls_Compute_Single_Summand_of_Linear_System'
	CKvPointf zz_p2d_t1, zz_p2d_t0;
	CKvPoint3Df zz_p3d_t1, zz_p3d_t0, zz_p3d_g, zz_tp3d;
	CKvPoint3Df zz_norm_proj, zz_norm_pred;
	CKvPixel zz_pix;	

	// variables for 'z_gin_Get_Interpolated_Normal'
	CKvPoint3Df zz_n1, zz_n2, zz_n3, zz_n4;
	
	CKvPixel zz_offset;
	
	// for display.
	CKvMatrixUcharRgb zz_img_rgb_t0, zz_img_rgb_t1;
	CKvMatrixUcharRgb zz_timg;

	// 이거 나중에 한 클래스로 묶자.
	CKvMatrixFloat zz_map_weight[KV_LEVEL_OF_IMAGE_PYRAMID];
	CKvMatrixFloat zz_map_diff_pm[KV_LEVEL_OF_IMAGE_PYRAMID];	// intensity difference.	
	CKvMatrixFloat zz_map_diff_gradm[KV_LEVEL_OF_IMAGE_PYRAMID];	// intensity difference.	
	CKvMatrixFloat zz_map_ppe[KV_LEVEL_OF_IMAGE_PYRAMID];	// point-to-plane error.
	CKvSet2d_of_VectorFloat zz_map_diff_grad[KV_LEVEL_OF_IMAGE_PYRAMID];	// gradient difference.
	
	CKvSet2d_of_Pointf zz_map_reproj[KV_LEVEL_OF_IMAGE_PYRAMID];
	CKvSet2d_of_Point3Df zz_map_p3d[KV_LEVEL_OF_IMAGE_PYRAMID];
	CKvSet2d_of_Point3Df zz_map_norm[KV_LEVEL_OF_IMAGE_PYRAMID];
	
	CKvSet2d_of_VectorFloat zz_map_jacobian_photomax[KV_LEVEL_OF_IMAGE_PYRAMID];	
	CKvSet2d_of_VectorFloat zz_map_jacobian_icp[KV_LEVEL_OF_IMAGE_PYRAMID];
	CKvSet2d_of_MatrixFloat zz_map_jacobian_grad[KV_LEVEL_OF_IMAGE_PYRAMID];

	CKvSet2d_of_MatrixFloat zz_map_jacobian_Pi_T[KV_LEVEL_OF_IMAGE_PYRAMID];	// jacobian of projection and transformation.
	CKvSet2d_of_MatrixFloat zz_map_jacobian_pi_homo[KV_LEVEL_OF_IMAGE_PYRAMID];	// jacobian of projection and transformation.
	
	CKvMatrixBool zz_map_valid_icp[KV_LEVEL_OF_IMAGE_PYRAMID];
	CKvMatrixBool zz_map_valid_photomax[KV_LEVEL_OF_IMAGE_PYRAMID];
	CKvMatrixBool zz_map_valid_gradient[KV_LEVEL_OF_IMAGE_PYRAMID];
	

	//////////////////////////////////////////////////////////////////////////
	CKvMatrixBool zz_map_valid_current_grad[KV_LEVEL_OF_IMAGE_PYRAMID];
	CKvMatrixBool zz_map_valid_current_photo[KV_LEVEL_OF_IMAGE_PYRAMID];
	CKvMatrixBool zz_map_valid_current_icp[KV_LEVEL_OF_IMAGE_PYRAMID];
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	// for motion prior
	CKvMatrixFloat zz_prior_4x4;
	CKvVectorFloat zz_prior_x_6;
	CKvMatrixFloat zz_eye_6x6;
	//////////////////////////////////////////////////////////////////////////

};