//********************************************************************************
//class AFX_EXT_CLASS CKvYooji_ColorModel : public CKvClass
class CKvYooji_ColorModel : public CKvClass
//********************************************************************************
{
public:
	CKvYooji_ColorModel(void);
	virtual ~CKvYooji_ColorModel(void);
	CKvYooji_ColorModel(CKvYooji_ColorModel &a);
public:
	void c_Create(int in_class_num,int in_gaussian_num,int in_color_type,
			bool in_flag_full_cov = true);
	void cp_Copy(CKvYooji_ColorModel *in_color_model);

	bool gffc_Get_Flag_of_Full_Covariance(){ return zz_flag_full_cov; }
	int gnc_Get_Number_of_Classes(){ return zz_class_num; }
	int gnm_Get_Number_of_Models(){	return zz_gaussian_num; }
	int gdf_Get_Dimension_of_Feature(){ return zz_feat_dim; }
	int gct_Get_Color_Type(){ return zz_color_type; }

	void cnm_Change_Number_of_Models(int in_gaussian_num);

	CKvVectorFloat *gpmvs_Get_Pointer_of_Mean_Vector_Set(int in_class_idx);
	CKvMatrixFloat* gpcms_Get_Pointer_of_Covariance_Matrix_Set(int in_class_idx);
	CKvMatrixFloat* gpcmis_Get_Pointer_of_Covariance_Matrix_Inverse_Set(int in_class_idx);

	CKvVectorFloat *gpcmds_Get_Pointer_of_Covariance_Matrix_Diagonal_Set(int in_class_idx);
	CKvVectorFloat *gpcmids_Get_Pointer_of_Covariance_Matrix_Inverse_Diagonal_Set(int in_class_idx);

	float *gpmw_Get_Pointer_of_Model_Weights(int in_class_idx);
	float *gpdcm_Get_Pointer_of_Determinant_of_Covariance_Matrix(int in_class_idx);

	float gpdw_Get_Probability_Density_Weighted(
		CKvVectorFloat *in_sample_feature,
		int in_class_idx);

	float gpd_Get_Probability_Density(CKvVectorFloat *in_sample_feature,
		int in_class_idx,
		int in_model_idx);
	float gmd_Get_Mahalanobis_Distance(CKvVectorFloat *in_sample_feature,
		int in_class_idx,
		int in_model_idx);

	float gpdc_Get_Probability_Density_Circular(CKvVectorFloat *in_sample_feature,
		int in_class_idx,
		int in_model_idx);
	float gmdc_Get_Mahalanobis_Distance_Circular(CKvVectorFloat *in_sample_feature,
		int in_class_idx,
		int in_model_idx);

// 	void scm_Save_Color_Model(CKvString in_save_file);
// 	bool lcm_Load_Color_Model(CKvString in_load_file);

	//////////////////////////////////////////////////////////////////////////
	void scm_Save_Color_Model(CKvString in_save_file);
	void scmp_Save_Color_Model_Pixelwise(CKvString in_save_file);

	bool lcm_Load_Color_Model(CKvString in_load_file);
	bool lcmp_Load_Color_Model_Pixelwise(CKvString in_load_file);
	//////////////////////////////////////////////////////////////////////////

private:

	int zz_class_num,zz_gaussian_num,zz_color_type,zz_feat_dim;
	bool zz_flag_full_cov;
	// for RGB, PCS features.
	//vector<CKvVectorFloat> zz_test_vec;
	vector<CKvSet_of_VectorFloat> zz_set_of_mean_vec;
	vector<CKvSet_of_VectorFloat> zz_set_of_cov_mat_diag,zz_set_of_cov_mat_inv_diag;
	vector<CKvSet_of_MatrixFloat> zz_set_of_cov_mat_full,zz_set_of_cov_mat_inv_full;
	vector<CKvVectorFloat> zz_set_of_weights,zz_set_of_det_of_cov_mat;

	CKvVectorFloat zz_tvec1,zz_tvec2,zz_tvec3;
};

//********************************************************************************
class LCKvYooji_Color_Segmentation : public CKvClass
//********************************************************************************
{
public:
	LCKvYooji_Color_Segmentation();
	~LCKvYooji_Color_Segmentation();

	void i_Initialize(bool in_flag_full_cov_mat = false);

	/// ******************************* Global functions ******************************* ///

	// ******************************* Hand segmentation  ******************************* ///
// 	void brgmm_Background_Removal_using_GMM(
// 		CKvYooji_ColorModel *in_color_model, 
// 		CKvMatrixBool *in_mask_background_or_NULL,
// 		CKvMatrixUcharRgb *in_image_color,
// 		CKvMatrixFloat *in_image_depth,
// 		CKvMatrixBool *out_mask_object);

	void scd_Skin_Color_Detection(
		CKvYooji_ColorModel *in_color_model,
		CKvMatrixUcharRgb *in_image_color,
		CKvMatrixBool *out_mask_object);

	void rhr_Remove_Hand_Region(
		CKvYooji_ColorModel &in_color_model, 
		CKvMatrixUcharRgb &in_image_color, 
		CKvMatrixUshort &io_image_depth);

	// ******************************* Image filtering  ******************************* ///
	// Bilateral filtering.
	void sibf_Smooth_Image_using_Bilateral_Filter(CKvMatrixUchar *in_img,
		CKvMatrixUchar *out_img);
	void sibf_Smooth_Image_using_Bilateral_Filter(CKvMatrixFloat *in_img,
		float in_sigma_range,
		CKvMatrixFloat *out_img);

	void sisf_Smooth_Image_using_Spatial_Filter(CKvMatrixUchar *in_img,
		CKvMatrixUchar *out_img);
		

	// ******************************* Color conversion  ******************************* ///
	void ccirl_Convert_Color_Image_RGB_to_LAB(CKvMatrixUcharRgb &in_img_RGB,
		CKvMatrixFloat &out_L_image,
		CKvMatrixFloat &out_A_image,
		CKvMatrixFloat &out_B_image);
	void chiri_Compute_Hue_Image_from_RGB_image(CKvMatrixUcharRgb &in_rgb_img, 
		CKvMatrixFloat &out_HSI_img);
	void chiri_Compute_Hue_Image_from_RGB_image(CKvMatrixUcharRgb &in_rgb_img,
	float in_th_saturation,
	float in_th_intensity,
	CKvMatrixFloat &out_hue_img);


	// ******************************* GMM clustering  ******************************* ///
	// Training
	void tcmgmm_Training_Color_Model_using_Gaussian_Mixture_Model(CKvString &in_load_folder,		
		CKvString &in_save_file_name,
		int in_class_num,
		int in_gaussian_num,
		int in_color_type);

	void cgmm_Clustering_using_Gaussian_Mixture_Model(vector<CKvVectorFloat> &in_sample_features,
		int in_class_idx,
		CKvYooji_ColorModel *io_color_model);
	
	// Classification.
	// + return class index which has maximum likelihood.
	int cfgmm_Classify_A_Feature_using_Gaussian_Mixture_Model(
		CKvVectorFloat &in_feature_vector,
		CKvYooji_ColorModel &in_gmm_model,
		int in_class_num);

	// ****************************** Morphological filtering ******************************* //
	void do_Dilate_Object(CKvMatrixBool &io_mask_obj, 
		int in_dx, int in_dy, 
		bool in_largest_blob_select_mode);
	void eo_Erode_Object(CKvMatrixBool &io_mask_obj, 
		int in_dx, int in_dy, 
		bool in_largest_blob_select_mode);

	/// ******************************* Local functions ******************************* ///
	// ******************************* Image filtering ********************************* //
	void gsfg_Get_Spatial_Filter_Gaussian(CKvMatrixFloat &out_filter);

	// ******************************* Color feature extraction ******************************* ///
	void gsfgc_Get_Sample_Features_RGB_for_Grab_Cut(
		CKvMatrixUcharRgb &in_color_image,
		vector<vector<CKvVectorFloat>> *out_set_of_feature_clusters,
		vector<CKvMatrixBool> *out_set_of_masks);

	void gsfb_Get_Sample_Features_RGB_using_Brushing(CKvMatrixUcharRgb &in_color_image,
		int in_cluster_num,
		vector<vector<CKvVectorFloat>> *out_set_of_feature_clusters,
		vector<CKvMatrixBool> *out_set_of_masks);
	void gsfb_Get_Sample_Features_RGB_using_Brushing(CKvMatrixUcharRgb &in_color_image,
		int in_cluster_num,
		vector<CKvVectorFloat> *out_set_of_feature_clusters);

	void gsf_Get_Sample_Features_PCS(CKvMatrixUcharRgb &in_color_image,
		int in_cluster_num,
		vector<CKvVectorFloat> *out_set_of_feature_clusters);
	void gsfis_Get_Sample_Features_from_Image_Sequence(CKvString &in_directory_name_color_sequence,
		int in_class_num,
		int in_color_type,
		vector<CKvVectorFloat> *out_set_of_feature_clusters);
	// Save and load.
	void sgmmp_Save_Gaussian_Mixture_Model_Parameters(CKvString &in_save_filename,
		int in_cluster_num, 
		int in_gaussian_num,
		int in_pcs_color_flag,
		vector<CKvVectorFloat> *out_set_of_mean_vector,
		vector<CKvMatrixFloat> *out_set_of_covariance_matrix,
		CKvVectorFloat *out_set_of_weights);
	void lgmmp_Load_Gaussian_Mixture_Model_Parameters(CKvString &in_load_filename,
		int &out_class_num,
		int &out_gaussian_num,
		int &out_pcs_color_flag,
		vector<CKvSet_of_VectorFloat> &out_set_of_mean_vector,
		vector<CKvSet_of_MatrixFloat> &out_set_of_covariance_matrix,
		vector<CKvVectorFloat> &out_set_of_weights);
	void lgmmp_Load_Gaussian_Mixture_Model_Parameters(CKvString &in_load_filename);

	// ******************************* PDF computation ******************************* ///
	
	// ******************************* Color conversion  ******************************* //
	void ccvrx_Convert_Color_Value_RGB_to_XYZ(unsigned char in_sR, unsigned char in_sG, unsigned char in_sB,
		double &out_X, double &out_Y, double &out_Z);
	void ccvrl_Convert_Color_Value_RGB_to_LAB(unsigned char in_sR, unsigned char in_sG, unsigned char in_sB,
		double &out_l_star, double &out_a_star, double &out_b_star);
	void ccvrh_Convert_Color_Value_RGB_to_HSI(unsigned char in_sR, unsigned char in_sG, unsigned char in_sB,
		double &out_H, double &out_S, double &out_I);

	void cc_Convert_Color_Rgb_to_PCS(CKvVectorFloat &in_rgb_vector,
		CKvVectorFloat &out_pcs_vector);

	void in_Intenisty_Normalization(CKvMatrixUchar &in_image_gray, 
		int in_min_intensity,
		int in_max_intensity,
		CKvMatrixUchar &out_image_gray);

	void biac_Bilinear_Interpolation_using_Averaging_Color(float in_real_x,
		float in_real_y,
		CKvMatrixUcharRgb *in_image,
		CKvRgb *out_interpolated_color);

	private:
		void z_ickm_Initial_Clustering_using_K_Means_Algorithm(vector<CKvVectorFloat> &in_sample_features,
			int in_class_idx,
			CKvVectorInt *out_cluster_indices_or_NULL,			
			CKvYooji_ColorModel *io_color_model);
		
		void z_em_Expectation_and_Maximization(vector<CKvVectorFloat> &in_sample_features,
			int in_class_idx,
			CKvYooji_ColorModel *io_color_model);
		
	//CKvScreen m_sc;
	LCKvUtility_for_MatrixFloat_Factorization aa_mff;
	LCKvAlgebra_for_Matrix aa_am;
	LCKvAlgebra_for_MatrixFloat aa_amf;
	LCKvUtility_for_Linear_Equation_Float aa_lef;

	/// ///////////////////////////////
	int zz_class_num, zz_gaussian_num, zz_pcs_color_flag;
	// for RGB, PCS features.
	vector<CKvSet_of_VectorFloat> zz_set_of_mean_vector;
	vector<CKvSet_of_MatrixFloat> zz_set_of_covariance_matrix, zz_set_of_covariance_matrix_inverse;
	vector<CKvVectorFloat> zz_set_of_weights, zz_set_of_determinant_of_covariance_matrix;
	// for HUE feature.
	vector<CKvVectorFloat> zz_set_of_mean_value;
	vector<CKvVectorFloat> zz_set_of_stdv_value;

};

