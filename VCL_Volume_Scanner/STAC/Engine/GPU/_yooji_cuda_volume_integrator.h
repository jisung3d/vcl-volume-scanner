//********************************************************************************************
class LGKvVolumeIntegrator
//********************************************************************************************
{
public:
	LGKvVolumeIntegrator();
	~LGKvVolumeIntegrator();

	__host__ void ip_Initialize_Parameters(
		GKvObjectCubeFloat *in_cube,
		int ww, int hh,
		float fx, float fy,
		float px, float py,
		float mu,
		float max_w);

	__host__ void ip_Initialize_Parameters(
		GKvObjectCubeFloat *in_cube,		
		int ww,int hh,
		float fx,float fy,
		float px,float py,
		int ww_rgb,int hh_rgb,
		float fx_rgb,float fy_rgb,
		float px_rgb,float py_rgb,
		const float *in_T_drgb_dev,
		float mu,
		float max_w);

	__host__ void cdtoc_Convert_Depth_to_TSDF_on_Cube(
		GKvObjectCubeFloat *io_cube,
		const float *in_map_depth_dev,
		int in_ww, int in_hh,
		const float *in_T_gc_dev, 
		const float *in_T_cg_dev);

	__host__ void cdtocc_Convert_Depth_to_TSDF_on_Cube_with_Color(
		GKvObjectCubeFloat *io_cube,
		const float *in_map_depth_dev,
		const uchar *in_img_color_dev,
		int in_ww,int in_hh,
		const float *in_T_gc_dev,
		const float *in_T_cg_dev,
		bool in_flag_on_rgb = false);

	__host__ void uvv_Update_Valid_Volume(
		GKvObjectCubeFloat *io_cube,
		GKvVolumeBool *in_vol_valid);

	//////////////////////////////////////////////////////////////////////////
//	__host__ void setDeviceMemTEST(GKvObjectCubeFloat *io_cube,int in_dim,int in_value);

};