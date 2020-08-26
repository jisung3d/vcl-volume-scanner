//********************************************************************************************
//class AFX_EXT_CLASS LCKvYooji_Image_Processor : public CKvClass
class LCKvYooji_Image_Processor : public CKvClass
//********************************************************************************************
{
public:
	LCKvYooji_Image_Processor(void);
	virtual ~LCKvYooji_Image_Processor(void);

	// Edge operators.
	void dec_Detect_Edges_using_Canny(
		CKvMatrixUchar *in_img,
		CKvYooji_Edge_Map *out_edge_map,
		float in_th_for_edge_mask = 30.0f,
		CKvMatrixBool *in_roi_mask = NULL);

	// compute point map from depth map.
	void cpm_Compute_Point_Map(
		CKvMatrixFloat *in_map_depth,
		CKvYooji_Intrinsics *in_intrinsics,
		CKvSet2d_of_Point3Df *out_map_p3d);

	// estimate normal map from point map.
	void enm_Estimate_Normal_Map(
		CKvSet2d_of_Point3Df *in_map_p3d,
		CKvSet2d_of_Point3Df *out_map_normal,
		int in_sz_mask_half = 1);

	// remove specular region.
	void rsd_Remove_Specular_Depth(
		CKvSet2d_of_Point3Df *in_map_p3d,
		CKvSet2d_of_Point3Df *in_map_normal,
		CKvMatrixBool *out_map_specular,
		float in_th_angle_with_Z = 30.0f);
	
	// make image pyramids.
	void cpd_Construct_Pyramids_Down(
		CKvMatrixBool *in_img,
		CKvYooji_Pyramid_Mat_Bool *out_pyrams_img,
		int in_level_of_pyrams = 3);

	void cpd_Construct_Pyramids_Down(
		CKvMatrixUchar *in_img,
		CKvYooji_Pyramid_Mat_Uchar *out_pyrams_img,
		int in_level_of_pyrams = 3);

	void cpde_Construct_Pyramids_Down_Edge(
		CKvYooji_Pyramid_Mat_Uchar *in_pyram_imgs,
		CKvYooji_Pyramid_Map_Edge *out_pyram_edges,
		float in_th_for_edge_mask = 50.0f);

	void cpdd_Construct_Pyramids_Down_Depth(
		CKvMatrixFloat *in_map_depth,
		CKvYooji_Pyramid_Mat_Float *out_pyrams_map,
		int in_level_of_pyrams = 3);

	void cpdi_Construct_Pyramids_Down_Intrinsics(
		CKvYooji_Intrinsics *in_intrinsics,
		CKvYooji_Pyramid_Intrinsics *out_pyrams_intrinsics,
		int in_level_of_pyrams = 3);

	void dih_Downsample_Image_Half(
		CKvMatrixBool *in_img_gray,
		CKvMatrixBool *out_img_downsampled);

	void dih_Downsample_Image_Half(
		CKvMatrixUchar *in_img_gray,
		CKvMatrixUchar *out_img_downsampled);

	void ddh_Downsample_Depthmap_Half(
		CKvMatrixFloat *in_map_depth,
		CKvMatrixFloat *out_map_downsampled);

	void dih_Downsample_Intrinsics_Half(
		CKvYooji_Intrinsics *in_intrinsics,
		CKvYooji_Intrinsics *out_intrinsics_downsampled);

	void sig5_Smooth_Image_Gaussian_5x5(
		CKvMatrixUchar *in_img_gray,
		CKvMatrixUchar *out_img_smoothed,
		bool in_flag_boundary_padding = false,
		CKvMatrixInt *in_img_for_padding = NULL);

	void sim5_Smooth_Image_Mean_5x5(
		CKvMatrixUchar *in_img_gray,
		CKvMatrixUchar *out_img_downsampled,
		bool in_flag_boundary_padding = false,
		CKvMatrixInt *in_img_for_padding = NULL);

	void sdg5_Smooth_Depthmap_Gaussian_5x5(
		CKvMatrixFloat *in_map_depth,
		CKvMatrixFloat *out_map_downsampled,
		bool in_flag_boundary_padding = false,
		CKvMatrixFloat *in_img_for_padding = NULL);

protected:

	CKvEdgeDetector_Canny zz_canny;

	// variables for Gaussian pyramid.
	CKvMatrixFloat zz_map_tmp[3];
	CKvMatrixUchar zz_img_tmp[3];
	CKvMatrixInt zz_img_padding[3];
	CKvMatrixFloat zz_map_padding[3];
};