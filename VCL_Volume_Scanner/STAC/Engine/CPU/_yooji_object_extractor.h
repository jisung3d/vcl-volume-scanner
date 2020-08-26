//********************************************************************************************
//class AFX_EXT_CLASS CKvYooji_Object_Extractor : public CKvClass
class LCKvYooji_Object_Extractor : public CKvClass
//********************************************************************************************
{
public:
	LCKvYooji_Object_Extractor(void);
	virtual ~LCKvYooji_Object_Extractor(void);

	void i_Initialize(
		CKvYooji_ColorModel *in_gmm_back_or_NULL,
		CKvVectorInt *in_mask_back_or_NULL,
		int in_ww_rgb, int in_hh_rgb);

	//////////////////////////////////////////////////////////////////////////
	void tbcmp_Train_Background_Color_Model_by_Pixel(
		vector<CKvMatrixUcharRgb> &in_img_rgb,
		CKvYooji_ColorModel *out_color_model_back_pixel,
		CKvYooji_ColorModel *out_color_model_back_global);

	void ibm_Import_Background_Models(
		CKvYooji_ColorModel &in_color_model_back_pixel,
		CKvYooji_ColorModel &in_color_model_back_global);
	//////////////////////////////////////////////////////////////////////////

	int gvd_Get_Valid_Depth(
		CKvMatrixFloat *io_depth_map,
		float in_depth_min,float in_depth_max);

	int god_Get_Object_Depth(
		CKvMatrixFloat *io_depth_map,
		float in_depth_min, float in_depth_max);

	int god_Get_Object_Depth(
		CKvMatrixFloat *io_depth_map,
		CKvPmatrix3D &in_pmat_depth,
		CKvPoint3Df &in_origin_cube,
		float in_size_cube);

// 	int god_Get_Object_Depth(
// 		CKvYooji_MatrixRgbD *io_rgbd_img,
// 		CKvYooji_Tracking_State *in_state,
// 		CKvYooji_Cube_TSDF_Float *in_cube);

	int god_Get_Object_Depth(
		CKvYooji_MatrixRgbD *io_rgbd_img,
		CKvYooji_Tracking_State *in_state,
		CKvYooji_Cube_TSDF_Float *in_cube,
		bool in_mode_erosion = false,
		CKvYooji_ColorModel *in_color_model_of_hand = NULL,
		CKvMatrixBool *out_hand_mask = NULL);

	//////////////////////////////////////////////////////////////////////////
	int rbd_Remove_Board_Depth(
		CKvMatrixFloat *in_map_depth,
		CKvYooji_Intrinsics *in_K,
		Mat in_pi_board,
		CKvMatrixBool *out_mask_obj);
	//////////////////////////////////////////////////////////////////////////

	int godr_Get_Object_Depth_on_RGB_Camera(
		CKvYooji_MatrixRgbD *io_rgbd_img,
		CKvYooji_ColorModel *in_color_model_of_hand);

	int gods_Get_Object_Depth_and_Silhouette(
		CKvYooji_MatrixRgbD *io_rgbd_img,
		CKvYooji_Tracking_State *in_state,
		CKvYooji_Cube_TSDF_Float *in_cube);

	// Silhouette extraction using background subtraction.
	bool eos_Extract_Object_Silhouette(
		CKvYooji_MatrixRgbD *in_rgbd_images,
		CKvYooji_Tracking_State *in_tracking_state);

	// Silhouette extraction using Graph Cut and Superpixel.
	int dppod_Do_Pre_Processing_for_Object_Depth(
		CKvMatrixFloat *io_depth_map);

	bool eosbc_Extract_Object_Silhouette_using_Background_Cut(
		CKvYooji_MatrixRgbD *in_rgbd_images,
		CKvDepot_of_Point3D *in_points_cube,
		CKvVectorInt *in_indices_cube,
		CKvPmatrix3D *in_pmat_current,
		CKvMatrixBool *out_mask_object);

	bool eosrd_Extract_Obejct_Silhouette_from_Rendered_Depth(
		CKvYooji_MatrixRgbD *in_rgbd_images,
		CKvYooji_Tracking_State *in_tracking_state,
		CKvMatrixBool *out_mask_object);

	void do_Dilate_Object(
		CKvMatrixBool *io_mask_obj, 
		int in_dx, int in_dy, 
		bool in_largest_blob_select_mode);
	void eo_Erode_Object(CKvMatrixBool *io_mask_obj, 
		int in_dx, int in_dy, 
		bool in_largest_blob_select_mode);

//protected:
	void z_ghm_Get_Hand_Mask(
		CKvYooji_MatrixRgbD *in_rgbd_img,
		CKvYooji_ColorModel *in_color_model_of_hand,
		CKvMatrixBool *out_mask_hand);

	void z_ghmr_Get_Hand_Mask_on_RGB_Camera(
		CKvYooji_MatrixRgbD *in_rgbd_img,
		CKvYooji_ColorModel *in_color_model_of_hand,
		CKvMatrixBool *out_mask_hand);

	void z_tdrcc_Transform_Depth_to_Rgb_Camera_Coordinates(
		CKvYooji_MatrixRgbD *in_rgbd_img,
// 		CKvMatrixUcharRgb *in_img_rgb,
// 		CKvMatrixFloat *in_img_d,
// 		CKvPmatrix3D *in_pmat_d,
// 		CKvPmatrix3D *in_pmat_rgb,		
		CKvMatrixFloat *out_img_d_transformed);

	//////////////////////////////////////////////////////////////////////////
	// New for background cut.
	void z_gszbi_Generate_Silhouette_using_Z_buffering_for_an_Image(
		CKvDepot_of_Point3D *in_depot_point,
		CKvVectorInt *in_idx_mesh,
		CKvPmatrix3D *in_p_mat,
		CKvMatrixBool *out_silhouette);

	void ub_Update_Buffer(float *in_position,float *in_depth,int in_idx_mesh,CKvMatrixBool *io_silhouette);

	void gd2dpp_Get_Depth_and_2D_Position_of_a_Point(float *in_pt,float *in_p_mat,float *out_position,float &out_depth);
	void gd2dpt_Get_Depth_and_2D_Position_of_a_Triangle(float *in_pt,float *in_p_mat,float *out_position,float *out_depth);

	void gbi_Get_Block_for_Interpolation(float *in_position,int *out_LTRB);
	//////////////////////////////////////////////////////////////////////////


	bool z_sbm_Set_Background_Mask(
		CKvVectorInt *in_background_region,
		CKvMatrixBool *io_background_mask);

	void z_cidfu_Convert_Image_Depth_Float_to_Uchar(
		CKvMatrixFloat *in_img_d,
		int in_mode_convert,
		CKvMatrixUchar *out_img_d);

	// for GraphCut with Maxflow 3.01.
	void dgc_Do_Grab_Cut(
		CKvMatrixUcharRgb &in_img,		
		CKvMatrixBool &in_bg,
		CKvMatrixBool &in_fg,
		CKvMatrixBool &out_mask_obj);

	void dbc_Do_Background_Cut(
		CKvMatrixUcharRgb &in_img,
		CKvMatrixBool &in_bg,
		CKvMatrixBool &in_fg,
		CKvMatrixBool &out_mask_obj);

	void ugmmp_Update_GMM_Parameters(
		CKvMatrixUcharRgb &in_img,
		CKvMatrixBool &in_mask_fg,
		CKvYooji_ColorModel *io_gmm);

	// for GraphCut.
	void ipfm_Initialize_Probable_Foreground_Mask(
		CKvMatrixUcharRgb &in_img,
		CKvMatrixBool &in_mask_bg,
		CKvMatrixBool &in_mask_fg,
		CKvYooji_ColorModel &in_gmm,
		CKvMatrixBool &out_fg_pb);

	void ipfm_Initialize_Probable_Foreground_Mask(
		CKvMatrixUcharRgb &in_img,
		CKvMatrixBool &io_mask_bg,
		CKvMatrixBool &io_mask_fg,
		CKvYooji_ColorModel &in_gmm_back_pixel,
		CKvYooji_ColorModel &in_gmm_obj_global,
		CKvMatrixBool &out_mask_fg_pb);

	float dataCost(int pix,int label);
	float dataCostWithTrainedBackground(int pix,int label);

	float smoothCost(int pix1,int pix2,int label1,int label2);


public:
protected:
	//CKvYooji_Scanner_Parameter zz_params;

private:
	LCKvUtility_for_Windows zz_uw;
	LCKvUtility_for_YCbCr aa_ycc;

	//CKvScreen sc_oe[3];
	CKvMatrixFloat zz_img_d_in_rgb;
	CKvMatrixUchar zz_img_d8, zz_img_d8_in_rgb;

	CKvYooji_Intrinsics zz_intrin_d, zz_intrin_rgb;
	CKvYooji_Extrinsics zz_extrin_rgb;
	CKvMatrixFloat zz_mat_4x4;

	// gmm background removal.
	LCKvYooji_Color_Segmentation zz_color_seg;
	CKvYooji_ColorModel zz_gmm_back;
	CKvMatrixBool zz_mask_obj,zz_mask_back;

	// for Graph Cut.
	CKvYooji_ColorModel zz_color_model_inter;
	CKvYooji_ColorModel zz_color_model_back,zz_color_model_obj;
	CKvYooji_ColorModel zz_color_model_back_pixel;
	CKvMatrixUcharRgb zz_img;
	CKvVectorFloat zz_vec3;
	double zz_beta,zz_lambda,zz_L;

	// flags.
	bool zz_flag_back,zz_flag_color;
};