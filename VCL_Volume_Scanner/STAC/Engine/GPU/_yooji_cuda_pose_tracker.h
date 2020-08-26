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

	//////////////////////////////////////////////////////////////////////////
	__host__ bool tp_Track_Pose(
		GKvTrackingState *io_track_state,
		GKvRgbdFrame *in_rgbd_frame,
		GKvMatrixFloat *in_pose_init_t0_t1 = NULL);
	//////////////////////////////////////////////////////////////////////////


	__host__ bool cmv_Check_Motion_Validity(
		GKvMatrixFloat *io_hmat_4x4,
		float in_max_translation,	// m
		float in_max_rotation);		// rad.
	

	

private:

	// HOST
	int z_num_val, z_num_inlier;	int *z_num_partial_host, *z_num_inlier_partial_host;
	float z_ATA[6*6], z_ATb[6], z_b, z_L[6*6], z_y[6], z_sol_x[6];	
	float *z_ATA_partial_host, *z_ATb_partial_host, *z_b_partial_host; 	
	// DEVICE
	GKvMatrixFloat z_ATA_partial_dev[GK_LEVEL_OF_IMAGE_PYRAMID], z_ATb_partial_dev[GK_LEVEL_OF_IMAGE_PYRAMID], z_b_partial_dev[GK_LEVEL_OF_IMAGE_PYRAMID];
	GKvMatrixFloat z_T_cg_est, z_T_cg_prev;
	GKvMatrixFloat z_mat_4x4;
	//GKvMatrixFloat1 z_T_cg_est1;
};