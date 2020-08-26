//********************************************************************************************
//class AFX_EXT_CLASS LCKvYooji_Evaluation : public CKvClass
class LCKvYooji_Evaluation : public CKvClass
//********************************************************************************************
{
public:
	/// constructor.
	LCKvYooji_Evaluation();
	virtual ~LCKvYooji_Evaluation();	

	bool mee_Measure_Estimation_Error_using_Principal_Axis(
		CKvYooji_Extrinsics *in_pose_init_or_NULL,
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Tracking_State *in_state,
		float &out_dist_rotation,
		float &out_dist_translation);

	bool mee_Measure_Estimation_Error_using_Geodesic_Distance(
		CKvYooji_Extrinsics *in_pose_init_or_NULL,
		CKvYooji_MatrixRgbD *in_view,
		CKvYooji_Tracking_State *in_state,
		float &out_dist_rotation,
		float &out_dist_translation);

	/// 
// 	float mhem_Make_Histogram_of_Error_Measurement(
// 		CKvXvectorFloat *in_vec_error,
// 		int in_num_bin,
// 		float in_step_error,
// 		CKvHistogram *out_hist_error);
	/// 	

private:

	CKvMatrixFloat zz_mat_4x4;

};