//********************************************************************************************
//class AFX_EXT_CLASS CKvYooji_Tracking_State : public CKvClass
// + This class contains the information of camera(object) pose tracking.
class CKvYooji_Tracking_State : public CKvClass
//********************************************************************************************
{
public:

	CKvYooji_Tracking_State(void);
	virtual ~CKvYooji_Tracking_State(void);
	CKvYooji_Tracking_State(CKvYooji_Tracking_State &a);

	void initialize(int in_width, int in_height, int in_level_pyram = 1);
	void copy( CKvYooji_Tracking_State *in_set_of_elements);

	void import_P_matrix(CKvPmatrix3D *in_pmat,
		CKvPmatrix3D *in_pmat_rgb = NULL);
	void compute_P_matrix(
		CKvYooji_Intrinsics *in_intrinsics,
		CKvYooji_Extrinsics *in_extrinsics,
		CKvPmatrix3D *out_pmatrix);
	
	CKvPmatrix3D *p_P_matrix_depth(){		return &zz_pmat_d; }
	CKvPmatrix3D *p_P_matrix_RGB(){		return &zz_pmat_rgb; }

	CKvYooji_Intrinsics* p_intrinsics_depth(){ return &zz_intrinsics; };
	CKvYooji_Intrinsics* p_intrinsics_RGB(){ return &zz_intrinsics_rgb; };
	CKvYooji_Extrinsics* p_extrinsics_glob_to_cam(){ return &zz_transform_glob_to_cam; };
	CKvYooji_Extrinsics* p_extrinsics_glob_to_cam_RGB(){ return &zz_transform_glob_to_cam_rgb; };

	CKvMatrixFloat* p_map_depth_rendered(){	return &zz_rendered_depth; };
	CKvMatrixFloat* p_map_weight_rendered(){	return &zz_rendered_weight; };
	CKvSet2d_of_Point3Df* p_map_normals_rendered(){	return &zz_rendered_normals; };

	CKvMatrixUcharRgb* p_image_RGB(){	return &zz_image_of_texture; };
	CKvMatrixUchar* p_image_gray(){	return &zz_image_of_texture_gray; };
	CKvYooji_Edge_Map* p_map_edge(){	return &zz_map_edge; };

	CKvMatrixUcharRgb* p_image_texture_rendered(){ return &zz_image_of_rendered_texture; }
	CKvMatrixUchar* p_image_depth_rendered(){	return &zz_image_of_rendered_depth; }
	CKvMatrixUchar* p_image_normals_rendered(){	return &zz_image_of_rendered_normals; }

	CKvMatrixFloat* p_roi_depth(){ return &zz_roi_depth; };
	CKvMatrixUcharRgb* p_roi_image_rgb(){ return &zz_roi_img_rgb; };
	CKvMatrixUchar* p_roi_image_gray(){ return &zz_roi_img_gray; };
	CKvYooji_Edge_Map* p_roi_edge(){ return &zz_roi_edge; };
	Rect *p_roi_object(){ return &zz_roi_obj; }

	//for pyramids.
	// Previous data.
	CKvYooji_Pyramid_Mat_Bool *p_pyramid_silhouette(){ return &zz_pyram_sil; }
	CKvYooji_Pyramid_Mat_Uchar *p_pyramid_image_gray(){ return &zz_pyram_img_gray; }
	CKvYooji_Pyramid_Map_Edge *p_pyramid_map_edge(){ return &zz_pyram_edge_map; }
	CKvYooji_Pyramid_Map_Edge *p_pyramid_map_grad_edge(){ return &zz_pyram_grad_edge_map; }
	CKvYooji_Pyramid_Map_Edge *p_pyramid_map_depth_edge(){ return &zz_pyram_depth_edge_map; }
	CKvYooji_Pyramid_Mat_Float *p_pyramid_map_confidence(){ return &zz_pyram_confidence; }

	CKvYooji_Pyramid_Mat_Float *p_pyramid_map_depth(){ return &zz_pyram_depth; }
	CKvYooji_Image_Pyramid_Point3Df *p_pyramid_map_p3d(){ return &zz_pyram_p3d; }
	CKvYooji_Image_Pyramid_Point3Df *p_pyramid_map_normals(){ return &zz_pyram_normals; }

	CKvYooji_Pyramid_Mat_Float *p_pyramid_map_depth_rendered(){ return &zz_pyram_depth_rendered; }
	CKvYooji_Image_Pyramid_Point3Df *p_pyramid_map_p3d_rendered(){ return &zz_pyram_p3d_rendered; }
	CKvYooji_Image_Pyramid_Point3Df *p_pyramid_map_normals_rendered(){ return &zz_pyram_normals_rendered; }

	CKvYooji_Pyramid_Intrinsics *p_pyramid_intrinsics(){ return &zz_pyram_intrin; }

	CKvSet2d_of_Pointf *p_set_of_flows(){ return &zz_set_of_flows; }
	CKvMatrixUcharRgb *p_image_flows(){ return &zz_map_flows; }

	int num_sphere_coverage(){ return zz_cnt_sphere; }
	void update_num_coverage(){ zz_cnt_sphere++; }

	int zz_cnt_sphere;

protected:

	CKvPmatrix3D zz_pmat_d, zz_pmat_rgb;

	CKvYooji_Intrinsics zz_intrinsics;
	CKvYooji_Intrinsics zz_intrinsics_rgb;
	CKvYooji_Extrinsics zz_transform_glob_to_cam;
	CKvYooji_Extrinsics zz_transform_glob_to_cam_rgb;

	CKvMatrixFloat zz_rendered_depth, zz_rendered_weight;
	CKvSet2d_of_Point3Df zz_rendered_normals;

	// for pyramids.
	CKvYooji_Pyramid_Mat_Bool zz_pyram_sil;
	CKvYooji_Pyramid_Mat_Uchar zz_pyram_img_gray;
	CKvYooji_Pyramid_Map_Edge zz_pyram_edge_map;
	CKvYooji_Pyramid_Map_Edge zz_pyram_grad_edge_map;
	CKvYooji_Pyramid_Map_Edge zz_pyram_depth_edge_map;
	CKvYooji_Pyramid_Mat_Float zz_pyram_confidence;

	CKvYooji_Pyramid_Mat_Float zz_pyram_depth, zz_pyram_depth_rendered;
	CKvYooji_Image_Pyramid_Point3Df zz_pyram_p3d, zz_pyram_p3d_rendered;
	CKvYooji_Image_Pyramid_Point3Df zz_pyram_normals, zz_pyram_normals_rendered;

	CKvYooji_Pyramid_Intrinsics zz_pyram_intrin;


// 	vector<CKvMatrixFloat> zz_pyram_rendered_depth;
// 	vector<CKvSet2d_of_Point3Df> zz_pyram_rendered_normals;
// 	vector<CKvMatrixUchar> zz_pyram_img_rendered_normals;

//	vector<CKvYooji_Intrinsics> zz_pyram_intrin_d;

	CKvMatrixUcharRgb zz_image_of_texture;
	CKvMatrixUchar zz_image_of_texture_gray;
	CKvYooji_Edge_Map zz_map_edge;

	CKvMatrixUcharRgb zz_image_of_rendered_texture;
	CKvMatrixUchar zz_image_of_rendered_depth;
	CKvMatrixUchar zz_image_of_rendered_normals;

	CKvMatrixFloat zz_roi_depth;
	CKvMatrixUcharRgb zz_roi_img_rgb;
	CKvMatrixUchar zz_roi_img_gray;
	CKvYooji_Edge_Map zz_roi_edge;
	Rect zz_roi_obj;

	// for depth flow or optical flow.
	CKvSet2d_of_Pointf zz_set_of_flows;
	CKvMatrixUcharRgb zz_map_flows;


};