//********************************************************************************************
//class AFX_EXT_CLASS CKvYooji_Intrinsics : public CKvClass
class CvYooji_Camera_Information
//********************************************************************************************
{
public:
	CvYooji_Camera_Information();
	virtual ~CvYooji_Camera_Information();
	CvYooji_Camera_Information(const CvYooji_Camera_Information &a);

	void cp_Copy(const CvYooji_Camera_Information *a);

	
	bool sp_Set_from_P_matrix(Mat &in_pmat);
	bool skrt_Set_from_KRT_matrices(Mat &in_kmat, Mat &in_rmat, Mat &in_tvec);
// 	bool p_Project(CKvPoint3Df &in_p3d, CKvPointf &out_p);
// 	void bp_Back_Project(CKvPixel &in_xy, float in_Z, CKvPoint3Df &out_p3d);
// 	void bp_Back_Project(CKvPointf &in_xy, float in_Z, CKvPoint3Df &out_p3d);

	bool p_Project(Point3d &in_p3d, Point2d &out_p);
	void bp_Back_Project(Point2d &in_xy, float in_Z, Point3d &out_p3d);
	void t_Transform(Point3d &in_p3d, Point3d &out_p);
	void ti_Transform_Inverse(Point3d &in_p3d, Point3d &out_p);

	bool p_Project(Point3f &in_p3d, Point2f &out_p);
	void bp_Back_Project(Point2f &in_xy, float in_Z, Point3f &out_p3d);
	void t_Transform(Point3f &in_p3d, Point3f &out_p);
	void ti_Transform_Inverse(Point3f &in_p3d, Point3f &out_p);
	void r_Rotate(Point3f &in_p3d, Point3f &out_p);
	void ri_Rotate_Inverse(Point3f &in_p3d, Point3f &out_p);

public:
	Mat intrinsics;
	Mat distCoeffs;
	Mat rvec, rmat;
	Mat tvec;
	Mat pmat;
	
	float fx, fy, px, py;		// focal length and principal point of camera.
	//float zz_d_min, zz_d_max;			// depths(mm) of near and far planes of the view frustum.

};
