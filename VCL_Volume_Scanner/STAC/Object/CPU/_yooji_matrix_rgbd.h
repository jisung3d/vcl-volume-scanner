//********************************************************************************************
//class AFX_EXT_CLASS CKvYooji_MatrixRgbD : public CKvClass
class CKvYooji_MatrixRgbD : public CKvClass
//********************************************************************************************
{
public:
	CKvYooji_MatrixRgbD(void);
	virtual ~CKvYooji_MatrixRgbD(void);
	CKvYooji_MatrixRgbD(CKvYooji_MatrixRgbD &a);

public:
	void copy(CKvYooji_MatrixRgbD *a);
	
	void ms(int &out_width,int &out_height);
	void ms_Matrix_Width_Height(int &out_width,int &out_height);

	void set_camera_information(
		CKvPmatrix3D &in_pmat_depth,
		CKvPmatrix3D &in_pmat_rgb);

	bool convert_pixel_coord(
		CKvPointf &in_point_depth,
		float in_depth,
		CKvPointf &out_point_rgb);

	CKvMatrixUcharRgb *p_image_rgb(){	return &zz_image_rgb;	}
	CKvMatrixFloat *p_map_depth_raw(){	return &zz_map_depth;	}
	CKvMatrixFloat *p_map_depth_raw_on_RGB(){	return &zz_map_depth_on_rgb;	}
	CKvMatrixFloat *p_map_depth_filtered(){	return &zz_map_depth_filt;	}
	CKvMatrixFloat *p_map_depth_filtered_on_RGB(){	return &zz_map_depth_filt_on_rgb;	}
	CKvMatrixBool *p_image_silhouette(){	return &zz_image_sil;	}
	CKvYooji_Edge_Map *p_map_edge(){	return &zz_edge_map;	}

	CKvMatrixFloat* p_roi_depth(){ return &zz_roi_depth; };
	CKvMatrixUcharRgb* p_roi_image_rgb(){ return &zz_roi_img_rgb; };
	CKvMatrixUchar* p_roi_image_gray(){ return &zz_roi_img_gray; };
	CKvYooji_Edge_Map* p_roi_edge(){ return &zz_roi_edge; };
	Rect *p_roi_object(){	return &zz_roi_obj; }

	CKvPmatrix3D *p_P_matrix_RGB(){	return &zz_pmat_rgb;	}
	CKvPmatrix3D *p_P_matrix_depth(){	return &zz_pmat_d;		}
	CKvYooji_Intrinsics *p_intrinsics_RGB(){	return &zz_intrin_rgb;	}
	CKvYooji_Intrinsics *p_intrinsics_depth(){	return &zz_intrin_d;	}
	CKvYooji_Extrinsics *p_extrinsics_RGB(){ return &zz_extrin_rgb; }
	CKvYooji_Extrinsics *p_extrinsics_depth(){ return &zz_extrin_d; }
	CKvYooji_Extrinsics *p_extrinsics_depth_to_RGB(){ return &zz_extrin_rgb_wrt_d; }
	CKvYooji_Extrinsics *p_extrinsics_RGB_to_depth(){ return &zz_extrin_d_wrt_rgb; }

	// for pyramids.
	CKvYooji_Pyramid_Mat_Bool *p_pyramid_silhouette(){ return &zz_pyram_sil; }
	CKvYooji_Pyramid_Mat_Uchar *p_pyramid_image_gray(){ return &zz_pyram_img_gray; }
	CKvYooji_Pyramid_Map_Edge *p_pyramid_map_edge(){ return &zz_pyram_edge_map; }
	CKvYooji_Pyramid_Map_Edge *p_pyramid_map_grad_edge(){ return &zz_pyram_grad_edge_map; }
	CKvYooji_Pyramid_Map_Edge *p_pyramid_map_depth_edge(){ return &zz_pyram_depth_edge_map; }

	CKvYooji_Pyramid_Mat_Float *p_pyramid_map_depth(){ return &zz_pyram_map_depth; }
	CKvYooji_Image_Pyramid_Point3Df *p_pyramid_map_p3d(){ return &zz_pyram_p3d; }
	CKvYooji_Image_Pyramid_Point3Df *p_pyramid_map_normals(){ return &zz_pyram_normals; }
	CKvYooji_Pyramid_Intrinsics *p_pyramid_intrinsics(){ return &zz_pyram_intrin_d; }

	void set_validity_silhouette(bool in_valid){ zz_flag_silhouette = in_valid; }
	void set_validity_extrinsics(bool in_valid){ zz_flag_extrins = in_valid; }

	bool is_valid_silhouette(){ return zz_flag_silhouette; }
	bool is_valid_extrinsics(){ return zz_flag_extrins; }

protected:

	// images.
	CKvMatrixUcharRgb zz_image_rgb;
	CKvMatrixFloat zz_map_depth, zz_map_depth_on_rgb, zz_map_depth_filt, zz_map_depth_filt_on_rgb;
	CKvMatrixBool zz_image_sil;
	CKvYooji_Edge_Map zz_edge_map;

	CKvMatrixFloat zz_roi_depth;
	CKvMatrixUcharRgb zz_roi_img_rgb;
	CKvMatrixUchar zz_roi_img_gray;
	CKvYooji_Edge_Map zz_roi_edge;
	Rect zz_roi_obj;

	// pyramids.
	CKvYooji_Pyramid_Mat_Bool zz_pyram_sil;
	CKvYooji_Pyramid_Mat_Uchar zz_pyram_img_gray;
	CKvYooji_Pyramid_Map_Edge zz_pyram_edge_map;
	CKvYooji_Pyramid_Map_Edge zz_pyram_grad_edge_map;
	CKvYooji_Pyramid_Map_Edge zz_pyram_depth_edge_map;
	CKvYooji_Pyramid_Intrinsics zz_pyram_intrin_d,zz_pyram_intrin_rgb;

	CKvYooji_Pyramid_Mat_Float zz_pyram_map_depth,zz_pyram_map_depth_rgb;
	CKvYooji_Image_Pyramid_Point3Df zz_pyram_p3d;
	CKvYooji_Image_Pyramid_Point3Df zz_pyram_normals;

	// camera parameters.
	CKvPmatrix3D zz_pmat_rgb, zz_pmat_d;
	CKvYooji_Intrinsics zz_intrin_d, zz_intrin_rgb;
	CKvYooji_Extrinsics zz_extrin_d, zz_extrin_rgb;
	CKvYooji_Extrinsics zz_extrin_rgb_wrt_d, zz_extrin_d_wrt_rgb;

	// flags
	bool zz_flag_silhouette;
	bool zz_flag_extrins;
};
