/////////////////////////////////////////////////////////////////////////////////////////////
// _yooji_camera_information.cpp
/////////////////////////////////////////////////////////////////////////////////////////////

#include "../../../stdafx.h"
#include "../../_yooji_2016_object_scanning.h"

/////////////////////////////////////////////////////////////////////////////////////////////
// CKvYooji_Dense_Stereo 
/////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
CvYooji_Camera_Information::CvYooji_Camera_Information()
//********************************************************************************************
{
	fx = fy = 1.0f;		px = py = 0.0f;		
	intrinsics = Mat::eye(3, 3, CV_64F);		
	distCoeffs = Mat::zeros(5, 1, CV_64F);

	rmat = Mat::eye(3, 3, CV_64F);
	rvec = Mat::zeros(3, 1, CV_64F);
	tvec = Mat::zeros(3, 1, CV_64F);

	pmat = Mat::zeros(3, 4, CV_64F);	
	rmat.copyTo(pmat(Rect(0, 0, 3, 3)));

}
//********************************************************************************************
CvYooji_Camera_Information::~CvYooji_Camera_Information()
//********************************************************************************************
{
}

//***********************************************************************************************************************
CvYooji_Camera_Information::CvYooji_Camera_Information(const CvYooji_Camera_Information &a)
//***********************************************************************************************************************
{
	cp_Copy(&a);
}

//********************************************************************************************
void CvYooji_Camera_Information::cp_Copy(const CvYooji_Camera_Information *a)
//********************************************************************************************
{
	a->intrinsics.copyTo(intrinsics);
	a->distCoeffs.copyTo(distCoeffs);
	a->rvec.copyTo(rvec);
	a->rmat.copyTo(rmat);
	a->tvec.copyTo(tvec);
	a->pmat.copyTo(pmat);

	fx = a->fx; fy = a->fy;
	px = a->px; py = a->py;
}

//********************************************************************************************
// CAUTION: 4D TRANSLATION VECTOR OF decomposeProjectionMatrix IS CAMERA CENTER.
// SO, YOU MUST MULTIPLY (-) MINUS 1 TO THIS 4D OUTPUT VECTOR 
// TO GET 3D TRANSLATION VECTOR, t, OF [R | t], RIGID TRANSFORMATION FOR THIS CAMERA.
bool CvYooji_Camera_Information::sp_Set_from_P_matrix(Mat &in_pmat)
//********************************************************************************************
{
	
	if (in_pmat.cols != 4 || in_pmat.rows != 3)	return false;

	Mat tvec4D;

	in_pmat.copyTo(pmat);
	decomposeProjectionMatrix(
		pmat, 
		intrinsics, rmat, tvec4D,
		noArray(), noArray(), noArray(),
		rvec);	
		
	// CAUTION: 4D TRANSLATION VECTOR OF decomposeProjectionMatrix IS CAMERA CENTER.
	// SO, YOU MUST MULTIPLY (-) MINUS 1 TO THIS 4D OUTPUT VECTOR 
	// TO GET 3D TRANSLATION VECTOR, t, OF [R | t], RIGID TRANSFORMATION FOR THIS CAMERA.
	for (int i = 0; i < 3; i++)	
		tvec.at<double>(i) = - tvec4D.at<double>(i) / tvec4D.at<double>(3);

	fx = intrinsics.at<double>(0, 0);
	fy = intrinsics.at<double>(1, 1);
	px = intrinsics.at<double>(0, 2);
	py = intrinsics.at<double>(1, 2);
		
	return true;
}

//********************************************************************************************
bool CvYooji_Camera_Information::skrt_Set_from_KRT_matrices(Mat &in_kmat, Mat &in_rmat, Mat &in_tvec)
//********************************************************************************************
{

	if (in_kmat.cols != 3 || in_kmat.rows != 3
		|| in_rmat.cols != 3 || in_rmat.rows != 3
		|| in_tvec.cols != 1 || in_tvec.rows != 3)
		return false;

	in_kmat.copyTo(intrinsics);
	in_rmat.copyTo(rmat);	Rodrigues(rmat, rvec);
	in_tvec.copyTo(tvec);

	rmat.copyTo(pmat(Rect(0, 0, 3, 3)));
	tvec.copyTo(pmat(Rect(3, 0, 1, 3)));
	pmat = intrinsics * pmat;

	fx = intrinsics.at<double>(0, 0);
	fy = intrinsics.at<double>(1, 1);
	px = intrinsics.at<double>(0, 2);
	py = intrinsics.at<double>(1, 2);

	return true;
}

//********************************************************************************************
bool CvYooji_Camera_Information::p_Project(Point3d &in_p3d, Point2d &out_p)
//********************************************************************************************
{
	//if (in_p3d.z>zz_d_max || in_p3d.z<zz_d_min)	return false;
// 	Point3d tp3d;
// 	t_Transform(in_p3d, tp3d);
// 	out_p.x = fx*tp3d.x / tp3d.z + px;	out_p.y = fy*tp3d.y / tp3d.z + py;
	
	out_p.x = fx*in_p3d.x / in_p3d.z + px;	out_p.y = fy*in_p3d.y / in_p3d.z + py;

	return true;
}
//********************************************************************************************
void CvYooji_Camera_Information::bp_Back_Project(Point2d &in_xy, float in_Z, Point3d &out_p3d)
//********************************************************************************************
{
// 	Point3d tp3d;
// 	tp3d.x = in_Z*(in_xy.x - px) / fx;	tp3d.y = in_Z*(in_xy.y - py) / fy;	tp3d.z = in_Z;
// 	ti_Transform_Inverse(tp3d, out_p3d);

	out_p3d.x = in_Z*(in_xy.x - px) / fx;	out_p3d.y = in_Z*(in_xy.y - py) / fy;	out_p3d.z = in_Z;
}

//********************************************************************************************
void CvYooji_Camera_Information::t_Transform(Point3d &in_p3d, Point3d &out_p)
//********************************************************************************************
{
	double *p_R = (double*)rmat.data;
	double *p_T = (double*)tvec.data;

	out_p.x = p_R[0] * in_p3d.x + p_R[1] * in_p3d.y + p_R[2] * in_p3d.z + p_T[0];
	out_p.y = p_R[3] * in_p3d.x + p_R[4] * in_p3d.y + p_R[5] * in_p3d.z + p_T[1];
	out_p.z = p_R[6] * in_p3d.x + p_R[7] * in_p3d.y + p_R[8] * in_p3d.z + p_T[2];
}
//********************************************************************************************
void CvYooji_Camera_Information::ti_Transform_Inverse(Point3d &in_p3d, Point3d &out_p)
//********************************************************************************************
{
	double *p_R = (double*)rmat.data;
	double *p_T = (double*)tvec.data;

	double x, y, z;

	// out_p = R' * (in_p - T).
	x = in_p3d.x - p_T[0];
	y = in_p3d.y - p_T[1];
	z = in_p3d.z - p_T[2];

// 	cout << in_p3d << endl;
// 	cout << p_T[0] << endl;
// 	cout << p_T[1] << endl;
// 	cout << p_T[2] << endl;
// 	cvWaitKey();

	out_p.x = p_R[0] * x + p_R[3] * y + p_R[6] * z;
	out_p.y = p_R[1] * x + p_R[4] * y + p_R[7] * z;
	out_p.z = p_R[2] * x + p_R[5] * y + p_R[8] * z;
}


//********************************************************************************************
bool CvYooji_Camera_Information::p_Project(Point3f &in_p3d, Point2f &out_p)
//********************************************************************************************
{
	//if (in_p3d.z>zz_d_max || in_p3d.z<zz_d_min)	return false;
// 	Point3f tp3d;
// 	t_Transform(in_p3d, tp3d);
// 	out_p.x = fx*tp3d.x / tp3d.z + px;	out_p.y = fy*tp3d.y / tp3d.z + py;

	out_p.x = fx*in_p3d.x / in_p3d.z + px;	out_p.y = fy*in_p3d.y / in_p3d.z + py;

	return true;
}
//********************************************************************************************
void CvYooji_Camera_Information::bp_Back_Project(Point2f &in_xy, float in_Z, Point3f &out_p3d)
//********************************************************************************************
{
// 	Point3f tp3d;
// 	tp3d.x = in_Z*(in_xy.x - px) / fx;	tp3d.y = in_Z*(in_xy.y - py) / fy;	tp3d.z = in_Z;
// 	ti_Transform_Inverse(tp3d, out_p3d);

	out_p3d.x = in_Z*(in_xy.x - px) / fx;	out_p3d.y = in_Z*(in_xy.y - py) / fy;	out_p3d.z = in_Z;
}

//********************************************************************************************
void CvYooji_Camera_Information::t_Transform(Point3f &in_p3d, Point3f &out_p)
//********************************************************************************************
{
	double *p_R = (double*)rmat.data;
	double *p_T = (double*)tvec.data;

	out_p.x = p_R[0] * in_p3d.x + p_R[1] * in_p3d.y + p_R[2] * in_p3d.z + p_T[0];
	out_p.y = p_R[3] * in_p3d.x + p_R[4] * in_p3d.y + p_R[5] * in_p3d.z + p_T[1];
	out_p.z = p_R[6] * in_p3d.x + p_R[7] * in_p3d.y + p_R[8] * in_p3d.z + p_T[2];
}
//********************************************************************************************
void CvYooji_Camera_Information::ti_Transform_Inverse(Point3f &in_p3d, Point3f &out_p)
//********************************************************************************************
{
	double *p_R = (double*)rmat.data;
	double *p_T = (double*)tvec.data;

	float x, y, z;

	// out_p = R' * (in_p - T).
	x = in_p3d.x - p_T[0];
	y = in_p3d.y - p_T[1];
	z = in_p3d.z - p_T[2];

// 	cout << in_p3d << endl;
// 	cout << p_T[0] << endl;
// 	cout << p_T[1] << endl;
// 	cout << p_T[2] << endl;
// 	cvWaitKey();

	out_p.x = p_R[0] * x + p_R[3] * y + p_R[6] * z;
	out_p.y = p_R[1] * x + p_R[4] * y + p_R[7] * z;
	out_p.z = p_R[2] * x + p_R[5] * y + p_R[8] * z;
}


//********************************************************************************************
void CvYooji_Camera_Information::r_Rotate(Point3f &in_p3d, Point3f &out_p)
//********************************************************************************************
{
	double *p_R = (double*)rmat.data;

	out_p.x = p_R[0] * in_p3d.x + p_R[1] * in_p3d.y + p_R[2] * in_p3d.z;
	out_p.y = p_R[3] * in_p3d.x + p_R[4] * in_p3d.y + p_R[5] * in_p3d.z;
	out_p.z = p_R[6] * in_p3d.x + p_R[7] * in_p3d.y + p_R[8] * in_p3d.z;
}
//********************************************************************************************
void CvYooji_Camera_Information::ri_Rotate_Inverse(Point3f &in_p3d, Point3f &out_p)
//********************************************************************************************
{
	double *p_R = (double*)rmat.data;

	float x, y, z;

	x = in_p3d.x;
	y = in_p3d.y;
	z = in_p3d.z;
	
	out_p.x = p_R[0] * x + p_R[3] * y + p_R[6] * z;
	out_p.y = p_R[1] * x + p_R[4] * y + p_R[7] * z;
	out_p.z = p_R[2] * x + p_R[5] * y + p_R[8] * z;
}



