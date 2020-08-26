//********************************************************************************************
//class AFX_EXT_CLASS CKvYooji_Intrinsics : public CKvClass
class CKvYooji_Intrinsics : CKvClass
//********************************************************************************************
{
public:
	CKvYooji_Intrinsics();
	virtual ~CKvYooji_Intrinsics();
	CKvYooji_Intrinsics(CKvYooji_Intrinsics &a);

	void copy(CKvYooji_Intrinsics *a);

	bool set_from_params(float in_fx, float in_fy, 
		float in_px, float in_py, 
		float in_dmin = 0.0f, float in_dmax = 100.0f);
	bool set_from_P_matrix(CKvPmatrix3D *in_pmat);
	void get_params(float &in_fx, float &in_fy, 
		float &in_px, float &in_py, 
		float &in_dmin, float &in_dmax);

	CKvMatrixFloat* mp(){ return &zz_mat; };

	bool project(CKvPoint3Df &in_p3d, CKvPointf &out_p);
	void back_project(CKvPixel &in_xy, float in_Z, CKvPoint3Df &out_p3d);
	void back_project(CKvPointf &in_xy, float in_Z, CKvPoint3Df &out_p3d);

protected:
	CKvMatrixFloat zz_mat;	// 3x3 intrinsic matrix of camera.
	float zz_fx, zz_fy, zz_px, zz_py;		// focal length and principal point of camera.
	float zz_d_min, zz_d_max;			// depths(mm) of near and far planes of the view frustum.
};

//********************************************************************************************
//class AFX_EXT_CLASS CKvYooji_Extrinsics : public CKvClass
class CKvYooji_Extrinsics : CKvClass
//********************************************************************************************
{
public:
	CKvYooji_Extrinsics();
	virtual ~CKvYooji_Extrinsics();
	CKvYooji_Extrinsics(CKvYooji_Extrinsics &a);

	//CKvYooji_Extrinsics operator=(const CKvYooji_Extrinsics &a);

	void copy(CKvYooji_Extrinsics *a);

	bool set_from_params(float in_rx, float in_ry, float in_rz, float in_tx, float in_ty, float in_tz);
	bool set_from_params(CKvVectorFloat *in_prin_vec_rotated, 
		CKvVectorFloat *in_prin_vec_ref_or_NULL,	
		CKvVectorFloat *in_cam_center);
	bool set_from_transform(CKvMatrixFloat *in_mat_4x4);
	bool set_from_P_matrix(CKvPmatrix3D *in_pmat);
	bool set_from_transform_inv(CKvMatrixFloat *in_mat_4x4_inv);

	void convert_quaternion_to_rotation_mat(
		CKvVectorFloat &in_quat_unit,
		CKvMatrixFloat &out_mat_rot);

	void convert_rotation_mat_to_quaternion(
		CKvMatrixFloat &in_mat_rot,
		CKvVectorFloat &out_quat_unit);

	void get_cam_center(CKvPoint3Df &out_center);
	void get_pose_relative(
		CKvYooji_Extrinsics &in_pose_ref,
		CKvYooji_Extrinsics &in_pose_curr,
		CKvYooji_Extrinsics &out_pose_relative);
	void get_pose_successive(
		CKvYooji_Extrinsics &in_pose_ref,
		CKvYooji_Extrinsics &in_pose_relative,
		CKvYooji_Extrinsics &out_pose_successive);

	void print_transform(const char *in_string = "T");

	CKvMatrixFloat* mp_rotation(){ return &zz_R_mat; };
	CKvMatrixFloat* mp_translation(){ return &zz_t_mat; };
	CKvMatrixFloat* mp_transform(){ return &zz_mat; };
	CKvMatrixFloat* mp_transform_inv(){ return &zz_mat_inv; };

	void transform(CKvPoint3Df &in_p3d, CKvPoint3Df &out_p);
	void transform_inv(CKvPoint3Df &in_p3d, CKvPoint3Df &out_p);

protected:
	bool z_im4x4_Inverse_Matrix_4x4(float *in_mat, float *out_mat);
	bool z_gtp_Get_Transformation_from_Params(const float *in_params, float *out_mat);
	bool z_gpt_Get_Params_from_Transformation(const float *in_mat, float *out_params);

public:
protected:
	// | R t | R: 3x3 3D rotation matrix. 
	// | 0 1 | t: 3x1 3D translation matrix.
	CKvMatrixFloat zz_mat, zz_mat_inv;		// 4x4 transformation matrix between camera and global coordinates.
	CKvMatrixFloat zz_R_mat, zz_t_mat;		// 3x3, 3x1 matrices.
	/// these following factors are not used yet...
	float zz_rx, zz_ry, zz_rz, zz_tx, zz_ty, zz_tz;	// rotation and translation factors of each axis (x,y,z). 
};