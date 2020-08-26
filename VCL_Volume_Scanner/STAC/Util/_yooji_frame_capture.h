//********************************************************************************************
//class AFX_EXT_CLASS CKvYooji_FrameCapture : public CKvClass
class CKvYooji_FrameCapture : public CKvClass
//********************************************************************************************
{
public:
	CKvYooji_FrameCapture(void);
	virtual ~CKvYooji_FrameCapture(void);
	void cp_Copy(CKvYooji_FrameCapture *a);

public:
	void i_Initialize(int in_caputure_type);	

	bool lci_Load_Camera_Information(
		CKvString &in_filename_of_camera_information,
		CKvYooji_MatrixRgbD *out_view);

	bool lcis_Load_Camera_Information_System(
		CKvYooji_MatrixRgbD *out_view);

	bool lpm_Load_P_Matrix(
		int in_file_idx, 
		CKvPmatrix3D *out_pmat);

	bool lpmt_Load_P_Matrix_from_Txt(
		CKvString &in_filename_of_camera_information,
		CKvPmatrix3D *out_pmat);

	//////////////////////////////////////////////////////////////////////////
	bool ltt_Load_Trace_from_Txt_for_SDF2SDF(
		CKvString &in_filename_of_camera_information,
		vector<CKvYooji_Extrinsics*> *out_trace);

	bool stt_Save_Trace_to_Txt_for_SDF2SDF(
		CKvString &in_filename_of_camera_information,
		vector<CKvYooji_Extrinsics*> *in_trace);

	bool ltt_Load_Trace_from_Txt_for_Steinsbrucker(
		CKvString &in_filename_of_camera_information,
		vector<CKvYooji_Extrinsics*> *out_trace);

	bool stt_Save_Trace_to_Txt_for_Steinsbrucker(
		CKvString &in_filename_of_camera_information,
		vector<CKvYooji_Extrinsics*> *in_trace);
	//////////////////////////////////////////////////////////////////////////
	
	bool ls_Load_Silhouette(
		int in_file_idx,
		CKvMatrixBool *out_silhouette);

	bool lri_Load_Rgb_Image(
		int in_file_idx,
		CKvMatrixUcharRgb *out_img_rgb);

	bool ldm_Load_Depth_Map(
		int in_file_idx,		
		CKvMatrixFloat *out_map_depth);

	bool lrdi_Load_Rgb_and_Depth_Images(
		int in_file_idx, 
		CKvMatrixUcharRgb *out_img_rgb, 
		CKvMatrixFloat *out_img_d);

	bool lrdc_Load_Rgb_and_Depth_from_CxD_File(
		int in_file_idx,
		CKvMatrixUcharRgb *out_img_rgb,
		CKvMatrixFloat *out_img_d);

	bool lprdi_Load_P_Matrices_and_Rgb_and_Depth_Images(
		int in_file_idx, 
		CKvYooji_MatrixRgbD *out_mat_rgbd);

	bool loc_Load_Object_Cube(
		CKvPoint3Df &out_cube_origin,
		float &out_cube_size);

	bool loc_Load_Object_Cube(
		CKvString &in_file_name,
		CKvPoint3Df &out_cube_origin,
		float &out_cube_size,
		int &out_cube_dim);

	bool soc_Save_Object_Cube(
		CKvPoint3Df &in_cube_origin,
		float in_cube_size);

	bool lgcm_Load_Glove_Color_Model(CKvYooji_ColorModel *out_color_model);

	//////////////////////////////////////////////////////////////////////////
	// New for background cut at 2017.06.12.
	void sbcmp_Save_Background_Color_Model_Pixelwise(
		CKvYooji_ColorModel *in_color_model);
	bool lbcmp_Load_Background_Color_Model_Pixelwise(
		CKvYooji_ColorModel *out_color_model);

	void sbcmg_Save_Background_Color_Model_Global(
		CKvYooji_ColorModel *in_color_model);
	bool lbcmg_Load_Background_Color_Model_Global(
		CKvYooji_ColorModel *out_color_model);
	//////////////////////////////////////////////////////////////////////////

	// I/O for TSDF cube.
	void stsdfc_Save_TSDF_Cube(CKvString &in_foder_path);
	void sdc_Save_Do_Cube(CKvString &in_foder_path);

	void ltsdfc_Load_TSDF_Cube(CKvString &in_foder_path,
		CKvYooji_Cube_TSDF_Float &out_tsdf_cube);

	// pointers.
	int gct_Get_Capture_Type(){ return zz_type_capture; }
	CKvString dn_Directory_Name_of_Silhouette(){ return zz_dn_sil; }
	CKvString dn_Directory_Name_of_Texture(){ return zz_dn_texture; }
	CKvString dn_Directory_Name_of_Depth(){ return zz_dn_depth; }
	CKvString dn_Directory_Name_of_Pmat(){ return zz_dn_pmat; }
	
protected:

	// for loading object data sequence.
	LCKvUtility_for_Import zz_uim;
	CKvData_Color_x_Depth zz_cxd;

	CKvString zz_fn_gmm_glove, zz_fn_gmm_back, zz_fn_gmm_back_pixels;
	CKvString zz_fn_object_cube, zz_fn_cam_params;

	CKvString zz_dn_sil,zz_dn_texture,zz_dn_depth,zz_dn_pmat;

	CKvSet_of_String zz_set_of_fn_pmat;		// set of file names for P matrices.
	CKvSet_of_String zz_set_of_fn_sil;		// set of file names for silhouettes.
	CKvSet_of_String zz_set_of_fn_color;	// set of file names for color images.
	CKvSet_of_String zz_set_of_fn_depth;	// set of file names for depth images.
	CKvSet_of_String zz_set_of_fn_cxd;		// set of file names for cxd files.

	int zz_num_file;			// the number of input frames that will be loaded.	
	int zz_type_capture;
	int zz_type_depth;			// 0: float type, 1: unsigned short type.
	
	bool zz_flag_pmat_gt;			// true: if there are ground truth P matrices.
	bool zz_flag_sil;			// true: if there are silhouettes.
	bool zz_flag_rgb_img;			// true: if there are RGB images.
	bool zz_flag_depth_map_float;	// true: if there are float depth maps.
	bool zz_flag_depth_map_short;	// true: if there are ushort depth maps.
	bool zz_flag_cxd;				// true: if there are cxd files.

	LCKvIO_FileJpg zz_iof;
};