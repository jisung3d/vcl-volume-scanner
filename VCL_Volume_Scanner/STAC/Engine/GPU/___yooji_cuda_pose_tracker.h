//********************************************************************************************
class LGKvPoseTracker
//********************************************************************************************
{
public:
	LGKvPoseTracker();
	~LGKvPoseTracker();

	__host__ void ip_Initialize_Parameters(
		int ww, int hh,
		float fx, float fy,
		float px, float py,
		float th_icp);

	__host__ bool tp_Track_Pose(
		GKvTrackingState *io_track_state,
		GKvMatrixFloat *in_map_depth_t1);
		//float *in_map_depth_t1);

// 	__host__ bool tp_Track_Pose(
// 		float *out_T_cg_t1,
// 		Vector2i in_sz_map,
// 		const float *in_T_gc_t0, const float *in_T_cg_t0,
// 		const float *in_map_depth_t0,
// 		const float *in_map_normal_t0,	
// 		const float *in_map_depth_t1);

private:

	// HOST
	int z_num_val, z_num_inlier;	
	int *z_num_partial_host, *z_num_inlier_partial_host;
	float z_ATA[6*6], z_ATb[6], z_b, z_L[6*6], z_y[6], z_sol_x[6];	
	float *z_ATA_partial_host, *z_ATb_partial_host, *z_b_partial_host; 	
	// DEVICE
	GKvMatrixFloat z_ATA_partial_dev, z_ATb_partial_dev, z_b_partial_dev;
	GKvMatrixFloat z_T_cg_est, z_T_cg_prev;
	//GKvMatrixFloat1 z_T_cg_est1;
};