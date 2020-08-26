//********************************************************************************************
class LGKvRendererTSDF
//********************************************************************************************
{
public:
	LGKvRendererTSDF();
	~LGKvRendererTSDF();

	__host__ void ip_Initialize_Parameters(
		GKvObjectCubeFloat *in_cube,
		int ww, int hh,
		float fx, float fy, float px, float py,
		float mu,
		float max_w = 100.0f);

	__host__ void rmi_Render_Maps_for_ICP(
		GKvObjectCubeFloat *in_cube,
		int in_ww,int in_hh,
		const float *in_T_gc_dev,const float *in_T_cg_dev,
		Vector3f in_cam_cen,Vector3f in_light,

		int in_level_of_pyramid,

		float *io_map_depth_dev,
		float *io_map_vertex_dev,
		float *io_map_normals_dev,
		uchar *io_img_normals_dev,

		// TEST
		uchar *io_img_color_dev = NULL);
	
	__host__ void rmi_Render_Maps_for_ICP(
		GKvObjectCubeFloat *in_cube,
		int in_ww, int in_hh,
		const float *in_T_gc_dev, const float *in_T_cg_dev,	
		Vector3f in_cam_cen, Vector3f in_light,

		int in_lev_of_pyram,

		float *io_map_depth_dev,
		float *io_map_normals_dev,		
		uchar *io_img_normals_dev,
		
		// TEST
		uchar *io_img_color_dev = NULL);

	__host__ void rmis_Render_Maps_for_Scene(
		GKvObjectCubeFloat *in_cube,
		int in_ww,int in_hh,
		const float *in_T_gc_dev,const float *in_T_cg_dev,
		Vector3f in_cam_cen,Vector3f in_light,

		float *io_map_depth_dev,
		float *io_map_normals_dev,
		uchar *io_img_normals_dev,

		// TEST
		uchar *io_img_color_dev = NULL);

	__host__ void rmis_Render_Maps_for_Scene(
		GKvObjectCubeFloat *in_cube,
		int in_ww,int in_hh,
		const float *in_T_gc_dev,const float *in_T_cg_dev,
		Vector3f in_cam_cen,Vector3f in_light,

		float *io_map_depth_dev,
		float *io_map_vertex_dev,
		float *io_map_normals_dev,
		uchar *io_img_normals_dev,

		// TEST
		uchar *io_img_color_dev = NULL);


// 		float *in_cube_origin,
// 		float in_voxel_sz,
// 		int in_sub_cube_dim,
// 
// 		float in_fx, float in_fy, 
// 		float in_px, float in_py,	
// 
// 		float *in_T_cg_prev,
// 			
// 
// 		CKvPoint3Df &in_norm_cc,
// 		CKvPoint3Df &in_cam_cen,
// 		float in_radius_cube,
// 		float in_dist_cam_center_to_cube_center,
// 		float in_theta_max,
// 
// 		float in_mu,
// 		float in_voxel_sz_inv,
// 		CKvPoint3Df &out_p3d);


};