//********************************************************************************************
//class AFX_EXT_CLASS LCKvYooji_Volume_Integrator : public CKvClass
// + This class contains the functions for volumetric integration.
class LCKvYooji_Volume_Integrator : public CKvClass
//********************************************************************************************
{
public:

	LCKvYooji_Volume_Integrator();
	~LCKvYooji_Volume_Integrator();

	// global functions.
	void cdtoc_Convert_Depth_to_TSDF_on_Cube(
		CKvYooji_MatrixRgbD *in_view,		
		CKvYooji_Scanner_Parameter *in_setting,
		CKvYooji_Tracking_State *in_state,
		CKvYooji_Cube_TSDF_Float *io_cube,
		bool in_flag_color = false);

	void cdtoc_Convert_Depth_to_TSDF_on_Cube_RGB_NEW(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Scanner_Parameter *in_setting,
		CKvYooji_Tracking_State *in_state,
		CKvYooji_Cube_TSDF_Float *io_cube,
		bool in_flag_color = false);

	int csc_Carve_Sub_Cubes_using_Casting_Pixel_Rays(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Scanner_Parameter *in_setting,
		CKvYooji_Tracking_State *in_state,
		CKvYooji_Cube_TSDF_Float *io_cube);

	bool rodvhs_Remove_Outlier_Depth_using_Visual_Hull_Shield(
		CKvMatrixFloat *in_map_depth_of_visual_hull,
		CKvMatrixFloat *io_map_depth_of_object);

	// local functions.
	int fvsc_Find_Visible_Sub_Cubes_using_Casting_Pixel_Rays(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Extrinsics *in_hmat_cam_to_glob,
		int in_edge_length_of_sub_cube,
		float in_mu,
		CKvYooji_Cube_TSDF_Float *io_cube);

	void ctvovsc_Compute_TSDF_of_Voxels_on_Visible_Sub_Cubes(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Extrinsics *in_hmat_cam_to_glob,
		int in_edge_length_of_sub_cube,
		float in_mu, float in_maxW,
		CKvYooji_Cube_TSDF_Float *io_cube,
		bool in_flag_color = false);

	int fvsc_Find_Visible_Sub_Cubes_using_Casting_Pixel_Rays_RGB_NEW(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Extrinsics *in_hmat_cam_to_glob,
		int in_edge_length_of_sub_cube,
		float in_mu,
		CKvYooji_Cube_TSDF_Float *io_cube);

	void ctvovsc_Compute_TSDF_of_Voxels_on_Visible_Sub_Cubes_RGB_NEW(
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Extrinsics *in_hmat_cam_to_glob,
		int in_edge_length_of_sub_cube,
		float in_mu, float in_maxW,
		CKvYooji_Cube_TSDF_Float *io_cube,
		bool in_flag_color = false);

	//// for SfS.
	//void nccsfs_Normalize_Cube_Coordinates_into_unit_sphere_for_Shape_From_Silhouette(
	//	CKvYooji_Cube_TSDF_Float *io_cube);

private:

	bool z_gid_Get_Interpolated_Depth(
		float in_x, float in_y,
		int in_ww, int in_hh,
		float *in_depth_map,
		float &out_depth);

	bool z_gic_Get_Interpolated_Color(
		float in_x, float in_y,
		int in_ww, int in_hh,
		UCHAR *in_rgb_image,
		CKvRgbF &out_color);
				
private:

	LCKvAlgebra_for_MatrixFloat zz_alg_mf;
	CKvYooji_Intrinsics zz_intrin;
	CKvYooji_Extrinsics zz_transform_glob_to_cam;
	CKvMatrixFloat zz_mat_4x4;

	CKvPixel zz_tpix;

	// for 'z_utvs_Update_TSDF_Values_of_Segment'
//	CKvVector3f zz_tp3d1, zz_tp3d2;
	CKvPointf zz_pix_start, zz_pix_end, zz_tpixf;
};