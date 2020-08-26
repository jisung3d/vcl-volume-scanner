/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_camera_parameters.cpp
/////////////////////////////////////////////////////////////////////////////////////////////

#include "../../../stdafx.h"
#include "../../_yooji_2016_object_scanning.h"

/////////////////////////////////////////////////////////////////////////////////////////////
// CKvYooji_Intrinsics 
/////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
CKvYooji_Intrinsics::CKvYooji_Intrinsics()
//********************************************************************************************
{
	zz_fx=zz_fy=1.0f;		zz_px=zz_py=0.0f;		
	zz_d_min=0.0f; zz_d_max=100.0f;	
	zz_mat.ci_Create_Identity_matrix(3);	
}
//********************************************************************************************
CKvYooji_Intrinsics::~CKvYooji_Intrinsics()
//********************************************************************************************
{	
}

//***********************************************************************************************************************
CKvYooji_Intrinsics::CKvYooji_Intrinsics(CKvYooji_Intrinsics &a)
//***********************************************************************************************************************
{
	copy(&a);
}

//********************************************************************************************
void CKvYooji_Intrinsics::copy(CKvYooji_Intrinsics *a)
//********************************************************************************************
{	
	zz_fx=a->zz_fx; zz_fy=a->zz_fy;	
	zz_px=a->zz_px; zz_py=a->zz_py;	
	zz_d_min=a->zz_d_min; zz_d_max=a->zz_d_max;	
	zz_mat.cp_Copy(&a->zz_mat);
}
//********************************************************************************************
bool CKvYooji_Intrinsics::set_from_params(float in_fx, float in_fy, 
	float in_px, float in_py, 
	float in_dmin, float in_dmax)
//********************************************************************************************
{	
	if(in_fx<0.0f || in_fy<0.0f || in_dmin>in_dmax)	return false;
	float *p_mat;

	zz_fx=in_fx; zz_fy=in_fy;	zz_px=in_px; zz_py=in_py;	zz_d_min=in_dmin; zz_d_max=in_dmax;

	p_mat=zz_mat.vp();
	p_mat[0]=zz_fx;	p_mat[1]=0.0f;	p_mat[2]=zz_px;
	p_mat[3]=0.0f;	p_mat[4]=zz_fy;	p_mat[5]=zz_py;
	p_mat[6]=0.0f;	p_mat[7]=0.0f;	p_mat[8]=1.0f;

	return true;
}

//********************************************************************************************
bool CKvYooji_Intrinsics::set_from_P_matrix(CKvPmatrix3D *in_pmat)
//********************************************************************************************
{
	double *p_in_K;
	float *p_zz_mat;
	int ww, hh, i;
	
	p_in_K = in_pmat->mpk()->mps(ww, hh)[0];	if(ww!=3 || hh!=3)	return false;
	p_zz_mat = zz_mat.vp();

	for(i=0; i<9; i++){	p_zz_mat[i] = (float)p_in_K[i]; }
	zz_fx=p_zz_mat[0]; zz_fy=p_zz_mat[4];	
	zz_px=p_zz_mat[2]; zz_py=p_zz_mat[5];	
	zz_d_min=0.0f; zz_d_max=100.0f;	// set default values (m).
	
	return true;
}
//********************************************************************************************
void CKvYooji_Intrinsics::get_params(float &in_fx, float &in_fy, 
	float &in_px, float &in_py, 
	float &in_dmin, float &in_dmax)
//********************************************************************************************
{
	in_fx=zz_fx; in_fy=zz_fy;	
	in_px=zz_px; in_py=zz_py;
	in_dmin=zz_d_min; in_dmax=zz_d_max;	
}
//********************************************************************************************
bool CKvYooji_Intrinsics::project(CKvPoint3Df &in_p3d, CKvPointf &out_p)
//********************************************************************************************
{ 
	//if(in_p3d.z>zz_d_max || in_p3d.z<zz_d_min)	return false;
	out_p.x=zz_fx*in_p3d.x/in_p3d.z+zz_px;	out_p.y=zz_fy*in_p3d.y/in_p3d.z+zz_py;	

	return true;
}
//********************************************************************************************
void CKvYooji_Intrinsics::back_project(CKvPixel &in_xy, float in_Z, CKvPoint3Df &out_p3d)
//********************************************************************************************
{
	out_p3d.x=in_Z*((float)in_xy.x-zz_px)/zz_fx;	 out_p3d.y=in_Z*((float)in_xy.y-zz_py)/zz_fy;	out_p3d.z=in_Z;
}
//********************************************************************************************
void CKvYooji_Intrinsics::back_project(CKvPointf &in_xy, float in_Z, CKvPoint3Df &out_p3d)
//********************************************************************************************
{
	out_p3d.x=in_Z*(in_xy.x-zz_px)/zz_fx;	out_p3d.y=in_Z*(in_xy.y-zz_py)/zz_fy;	out_p3d.z=in_Z;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// CKvYooji_Extrinsics 
/////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
CKvYooji_Extrinsics::CKvYooji_Extrinsics()
//********************************************************************************************
{	
	zz_rx=zz_ry=zz_rz=0.0f;	zz_tx=zz_ty=zz_tz=0.0f;	
	zz_mat.ci_Create_Identity_matrix(4);	
	zz_mat_inv.ci_Create_Identity_matrix(4);	
	zz_R_mat.ci_Create_Identity_matrix(3);
	zz_t_mat.c_Create(3,1,0.0f);
}
//********************************************************************************************
CKvYooji_Extrinsics::~CKvYooji_Extrinsics()
//********************************************************************************************
{	
}
//***********************************************************************************************************************
CKvYooji_Extrinsics::CKvYooji_Extrinsics(CKvYooji_Extrinsics &a)
//***********************************************************************************************************************
{
	copy(&a);
// 	zz_rx = a.zz_rx; zz_ry = a.zz_ry; zz_rz = a.zz_rz;	zz_tx = a.zz_tx; zz_ty = a.zz_ty; zz_tz = a.zz_tz;
// 	zz_mat.cp_Copy(&(a.zz_mat));	zz_mat_inv.cp_Copy(&a.zz_mat_inv);
// 	zz_R_mat.cp_Copy(&a.zz_R_mat);	zz_t_mat.cp_Copy(&a.zz_t_mat);
}

//********************************************************************************************
void CKvYooji_Extrinsics::copy(CKvYooji_Extrinsics *a)
//********************************************************************************************
{
	zz_rx=a->zz_rx; zz_ry=a->zz_ry; zz_rz=a->zz_rz;	zz_tx=a->zz_tx; zz_ty=a->zz_ty; zz_tz=a->zz_tz;	
	zz_mat.cp_Copy(&a->zz_mat);	zz_mat_inv.cp_Copy(&a->zz_mat_inv);
	zz_R_mat.cp_Copy(&a->zz_R_mat);	zz_t_mat.cp_Copy(&a->zz_t_mat);
}

////********************************************************************************************
//// CAUTION: This could make error when this function uses as object pose controller.
//bool CKvYooji_Extrinsics::sp_Set_from_Parameters(float in_rx, float in_ry, float in_rz, float in_tx, float in_ty, float in_tz)
////********************************************************************************************
//{	
//	float params[6];
//	float *p_mat;
//
//	// set transformation matrix.
//	p_mat = zz_mat.vp();
//
//	params[0] = zz_rx = in_rx; params[1] = zz_ry =  in_ry; params[2] = zz_rz =  in_rz;
//	params[3] = zz_tx = in_tx; params[4] = zz_ty =  in_rx; params[5] = zz_tz =  in_tz;
//
//	// set transformation from params.
//	d_gtp_Get_Transformation_from_Params(params, p_mat);
//
//	// set inverse transformation.
//	d_im_Inverse_Matrix_4x4(p_mat,zz_mat_inv.vp());
//
//	// set rotation and translation matrices.
//	float *p_R = zz_R_mat.vp();
//	float *p_t = zz_t_mat.vp();
//	// + R.
//	for(int j=0; j<3; j++){ for(int i=0; i<3; i++){ p_R[j*3+i] = p_mat[j*4+i]; } }
//	// + t.
//	for(int j=0; j<3; j++)	p_t[j] = p_mat[j*4+3];
//
//	return true;
//}

//********************************************************************************************
bool CKvYooji_Extrinsics::set_from_params(float in_rx, float in_ry, float in_rz, float in_tx, float in_ty, float in_tz)
//********************************************************************************************
{	
	float *p_R, *p_t;
	float *p_mat;

	// set transformation matrix.
	p_mat = zz_mat.vp();
	p_R = zz_R_mat.vp();
	p_t = zz_t_mat.vp();

	// set rotation matrix.
	p_R[0] = +cos(in_rz)*cos(in_ry);
	p_R[1] = +sin(in_rz)*cos(in_rx) +cos(in_rz)*sin(in_ry)*sin(in_rx);
	p_R[2] = +sin(in_rz)*sin(in_rx) -cos(in_rz)*sin(in_ry)*cos(in_rx);

	p_R[3] = -sin(in_rz)*cos(in_ry);
	p_R[4] = +cos(in_rz)*cos(in_rx) -sin(in_rz)*sin(in_ry)*sin(in_rx);
	p_R[5] = +cos(in_rz)*sin(in_rx) +sin(in_rz)*sin(in_ry)*cos(in_rx);

	p_R[6] = +sin(in_ry);
	p_R[7] = -cos(in_ry)*sin(in_rx);
	p_R[8] = +cos(in_ry)*cos(in_rx);

	for(int j=0; j<3; j++)	for(int i=0; i<3; i++)	p_mat[j*4+i]=p_R[j*3+i];	
	// set translation vector.
	p_t[0] = in_tx;			p_t[1] = in_ty;			p_t[2] = in_tz;
	p_mat[0*4+3] = in_tx;	p_mat[1*4+3] = in_ty;	p_mat[2*4+3] = in_tz;

	// set inverse transformation.
	z_im4x4_Inverse_Matrix_4x4(p_mat, zz_mat_inv.vp());

	return true;
}

//********************************************************************************************
bool CKvYooji_Extrinsics::set_from_params(CKvVectorFloat *in_prin_vec_rotated, 
	CKvVectorFloat *in_prin_vec_ref_or_NULL,		
	CKvVectorFloat *in_cam_center)
//********************************************************************************************
{	
	CKvVectorFloat t_vec;
	float *p_vec1, *p_vec2;
	float *p_R, *p_t;
	float *p_mat;

	float axis_Euler[3], angle_Euler;
	float mag_axis_Euler, mag_vec1, mag_vec2, cs, ss;
	int sz;

	// set transformation matrix.
	p_mat = zz_mat.vp();
	p_R = zz_R_mat.vp();
	p_t = zz_t_mat.vp();

	// compute Euler axis and angle.
	// set rotated axis.
	p_vec2 = in_prin_vec_rotated->vps(sz); if(sz!=3) return false;
	// set reference axis.
	if(in_prin_vec_ref_or_NULL){
		p_vec1 = in_prin_vec_ref_or_NULL->vps(sz); if(sz!=3) return false;
	}
	else{
		p_vec1 = t_vec.c_Create(3, 0.0f);	p_vec1[2] = 1.0f; // set reference principal axis to z-axis.
	}
	// compute Euler axis.
	d_cpv3_Cross_Product_Vector3D(p_vec1, p_vec2, axis_Euler);
	d_n2v_Norm_2_Vector(axis_Euler, 3, mag_axis_Euler);
	if(mag_axis_Euler==0.0f){
		p_R[0] = 1.0f; p_R[1] = 0.0f; p_R[2] = 0.0f;
		p_R[3] = 0.0f; p_R[4] = 1.0f; p_R[5] = 0.0f;
		p_R[6] = 0.0f; p_R[7] = 0.0f; p_R[8] = 1.0f;
	}
	else{
		for(int i=0; i<3; i++) axis_Euler[i]/=mag_axis_Euler;
		// compute Euler angle.
		d_n2v_Norm_2_Vector(p_vec1, 3, mag_vec1);	d_n2v_Norm_2_Vector(p_vec2, 3, mag_vec2);
		angle_Euler = asin(mag_axis_Euler/(mag_vec1*mag_vec2));
		angle_Euler = -angle_Euler; // for coordinates transformation.

		// set rotation matrix from Euler axis and angle.		
		cs = cos(angle_Euler); ss = sin(angle_Euler);
		p_R[0] = cs + SQUARE(axis_Euler[0])*(1.0f-cs);
		p_R[1] = axis_Euler[0]*axis_Euler[1]*(1.0f-cs) - axis_Euler[2]*ss;
		p_R[2] = axis_Euler[0]*axis_Euler[2]*(1.0f-cs) + axis_Euler[1]*ss;

		p_R[3] = axis_Euler[1]*axis_Euler[0]*(1.0f-cs) + axis_Euler[2]*ss;
		p_R[4] = cs + SQUARE(axis_Euler[1])*(1.0f-cs);
		p_R[5] = axis_Euler[1]*axis_Euler[2]*(1.0f-cs) - axis_Euler[0]*ss;

		p_R[6] = axis_Euler[2]*axis_Euler[0]*(1.0f-cs) - axis_Euler[1]*ss;
		p_R[7] = axis_Euler[2]*axis_Euler[1]*(1.0f-cs) + axis_Euler[0]*ss;
		p_R[8] = cs + SQUARE(axis_Euler[2])*(1.0f-cs);
	}

	for(int j=0; j<3; j++)	for(int i=0; i<3; i++)	p_mat[j*4+i]=p_R[j*3+i];	

	// set translation vector.
	p_vec1 = in_cam_center->vps(sz); if(sz!=3) return false;	
	for(int j=0; j<3; j++){
		p_t[j] = 0.0f;
		for(int i=0; i<3; i++){
			p_t[j] += (float)(-p_R[j*3+i]*p_vec1[i]);
		}
		p_mat[j*4+3] = p_t[j];
	}

	// set inverse transformation.
	z_im4x4_Inverse_Matrix_4x4(p_mat, zz_mat_inv.vp());

	return true;
}
//********************************************************************************************
bool CKvYooji_Extrinsics::set_from_transform(CKvMatrixFloat *in_mat)
//********************************************************************************************
{
	int mw,mh;
	float R[9],t[3],params[6];
	float *p_mat,*p_mat_inv,*p_mat_in;

	p_mat_in=in_mat->mps(mw,mh)[0];
	if(mw!=4 || mh!=4)	return false;

	// get pointers.
	p_mat=zz_mat.vp(); p_mat_inv = zz_mat_inv.vp();

// 	// extract rotation parameters(Euler angles) from R matrix.
// 	d_gpt_Get_Params_from_Transformation(p_mat_in,params);
// 
// 	zz_rx=params[0];	zz_ry=params[1];	zz_rz=params[2];
// 	zz_tx=params[3];	zz_ty=params[4];	zz_tz=params[5];
// 
// 	//if(!Kv_Printf("%f %f %f %f %f %f\n", zz_rx, zz_ry, zz_rz, zz_tx, zz_ty, zz_tz)) exit(0);
// 
// 	// set transformation matrix.
// 	d_gtp_Get_Transformation_from_Params(params,p_mat);

	// set transformation matrix.
	zz_mat.cp_Copy(in_mat);

	// set inverse transformation.
	z_im4x4_Inverse_Matrix_4x4(p_mat,zz_mat_inv.vp());

	// set rotation and translation matrices.
	float *p_R = zz_R_mat.vp();
	float *p_t = zz_t_mat.vp();
	// + R.
	for(int j=0; j<3; j++){ for(int i=0; i<3; i++){ p_R[j*3+i] = p_mat[j*4+i]; } }
	// + t.
	for(int j=0; j<3; j++)	p_t[j] = p_mat[j*4+3];

	return true;
}

//********************************************************************************************
bool CKvYooji_Extrinsics::set_from_transform_inv(CKvMatrixFloat *in_mat_inv)
//********************************************************************************************
{
	int mw, mh;
	float R[9], t[3], params[6];
	float *p_mat, *p_mat_inv, *p_mat_in;

	p_mat_in=in_mat_inv->mps(mw, mh)[0];
	if(mw!=4 || mh!=4)	return false;

	// get pointers.
	p_mat=zz_mat.vp(); p_mat_inv = zz_mat_inv.vp();

	// get forward transformation.
	z_im4x4_Inverse_Matrix_4x4(p_mat_in, p_mat);

 	// set inverse transformation.
	zz_mat_inv.cp_Copy(in_mat_inv);

// 	// extract rotation parameters(Euler angles) from R matrix.
// 	d_gpt_Get_Params_from_Transformation(p_mat, params);
// 
// 	zz_rx=params[0];	zz_ry=params[1];	zz_rz=params[2];
// 	zz_tx=params[3];	zz_ty=params[4];	zz_tz=params[5];
// 
// 	//if(!Kv_Printf("%f %f %f %f %f %f\n", zz_rx, zz_ry, zz_rz, zz_tx, zz_ty, zz_tz)) exit(0);
// 	
// 	// set transformation matrix.
// 	d_gtp_Get_Transformation_from_Params(params, p_mat);
// 
// 	// set inverse transformation.
// 	z_im4x4_Inverse_Matrix_4x4(p_mat, zz_mat_inv.vp());

	// set rotation and translation matrices.
	float *p_R = zz_R_mat.vp();
	float *p_t = zz_t_mat.vp();
	// + R.
	for(int j=0; j<3; j++){ for(int i=0; i<3; i++){ p_R[j*3+i] = p_mat[j*4+i]; } }
	// + t.
	for(int j=0; j<3; j++)	p_t[j] = p_mat[j*4+3];

	return true;
}

//********************************************************************************************
bool CKvYooji_Extrinsics::set_from_P_matrix(CKvPmatrix3D *in_pmat)
//********************************************************************************************
{
	float R[9], t[3], params[6];
	double *p_in_R, p_in_oc[4];
	float *p_mat;	

	p_mat = zz_mat.vp();
	p_in_R = in_pmat->mpr()->vp();
	in_pmat->oc_Optical_Center().g_Get(p_in_oc);	

	// get optical center in inhomogenious coordinates.
	for(int i=0; i<3; i++) p_in_oc[i] = p_in_oc[i]/p_in_oc[3];
	// get rotation matrix.
	for(int j=0; j<3; j++){	for(int i=0; i<3; i++){
		p_mat[j*4+i] = (float)p_in_R[j*3+i];	
	}}

	// get translation vector.
	for(int j=0; j<3; j++){
		t[j] = 0.0f;
		for(int i=0; i<3; i++) t[j] += (float)(-p_in_R[j*3+i]*p_in_oc[i]);
		p_mat[j*4+3] = t[j]; 
	}

	// extract rotation parameters(Euler angles) from R matrix.
	d_gpt_Get_Params_from_Transformation(p_mat, params);
		
	// set transformation matrix.
	d_gtp_Get_Transformation_from_Params(params,p_mat);

	// set parameters.
	zz_rx=params[0];	zz_ry=params[1];	zz_rz=params[2];
	zz_tx=params[3];	zz_ty=params[4];	zz_tz=params[5];
	
	// set inverse transformation.
	d_im_Inverse_Matrix_4x4(p_mat, zz_mat_inv.vp());

	// set rotation and translation matrices.
	float *p_R = zz_R_mat.vp();
	float *p_t = zz_t_mat.vp();
	// + R.
	for(int j=0; j<3; j++){ for(int i=0; i<3; i++){ p_R[j*3+i] = p_mat[j*4+i]; } }
	// + t.
	for(int j=0; j<3; j++)	p_t[j] = p_mat[j*4+3];

	return true;
}

//
////********************************************************************************************
//bool CKvYooji_Extrinsics::sit_Set_from_Inverse_Transformation(CKvMatrixFloat *in_mat_inv)
////********************************************************************************************
//{
//	int mw, mh;
//	float R[9], t[3];
//	float *p_R, *p_t;
//	float *p_mat, *p_mat_inv, *p_mat_in;
//
//	p_mat_in=in_mat_inv->mps(mw, mh)[0];
//	if(mw!=4 || mh!=4)	return false;
//
//	// get pointers.
//	p_mat=zz_mat.vp(); p_mat_inv = zz_mat_inv.vp();
//
//	// get forward transformation.
//	p_mat=zz_mat.vp();		z_im4x4_Inverse_Matrix_4x4(p_mat_in, p_mat);
//	// get rotation matrix.
//	for(int j=0; j<3; j++)	for(int i=0; i<3; i++)	R[j*3+i]=p_mat[j*4+i];	
//	// get translation vector.
//	for(int j=0; j<3; j++)	t[j]=p_mat[j*4+3];	
//
//	// extract rotation parameters(Euler angles) from R matrix.
//	//		| r11 r12 r13 |
//	// R =	| r21 r22 r23 |
//	//		| r31 r32 r33 |
//	// rx = atan2(r32, r33)
//	// ry = atan2(-r31, sqrt(r32^2 + r33^2))
//	// rz = atan2(r21, r11)
//	/// /////////
//	zz_rx=atan2f(R[7], R[8]);	zz_ry=atan2f(-R[6], sqrt(SQUARE(R[7])+SQUARE(R[8])));	zz_rz=atan2f(R[3], R[0]);
//	zz_tx=t[0];	zz_ty=t[1];	zz_tz=t[2];
//	/// /////////
//	
//	// set transformation matrix.
//	p_R = zz_R_mat.vp();
//	p_t = zz_t_mat.vp();
//	// set rotation matrix.
//	p_R[0]= +cos(zz_rz)*cos(zz_ry);
//	p_R[1]= +cos(zz_rz)*sin(zz_ry)*sin(zz_rx) - sin(zz_rz)*cos(zz_rx);
//	p_R[2]= +cos(zz_rz)*sin(zz_ry)*cos(zz_rx) + sin(zz_rz)*sin(zz_rx);
//
//	p_R[3]= +sin(zz_rz)*cos(zz_ry);
//	p_R[4]= +sin(zz_rz)*sin(zz_ry)*sin(zz_rx) + cos(zz_rz)*cos(zz_rx);
//	p_R[5]= +sin(zz_rz)*sin(zz_ry)*cos(zz_rx) - cos(zz_rz)*sin(zz_rx);
//
//	p_R[6]= -sin(zz_ry);
//	p_R[7]= +cos(zz_ry)*sin(zz_rx);
//	p_R[8]= +cos(zz_ry)*cos(zz_rx);
//
//	for(int j=0; j<3; j++)	for(int i=0; i<3; i++)	p_mat[j*4+i]=p_R[j*3+i];	
//	// set translation vector.
//	for(int j=0; j<3; j++){
//		p_t[j] = t[j];
//		p_mat[j*4+3]=t[j];
//	}
//
//	// set inverse transformation.
//	z_im4x4_Inverse_Matrix_4x4(p_mat, zz_mat_inv.vp());
//
//	return true;
//}

//********************************************************************************************
// https://en.wikipedia.org/wiki/Euler%E2%80%93Rodrigues_formula
// https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
// quaternion: qx*i + qy*j + qz*k + qw (qx, qy, qz, qw)
void CKvYooji_Extrinsics::convert_quaternion_to_rotation_mat(
	CKvVectorFloat &in_quat_unit,
	CKvMatrixFloat &out_mat_rot)
//********************************************************************************************
{
	float *p_q = in_quat_unit.vp();
	float *p_rot = out_mat_rot.vp();

	double qq[4][4], q[4];
	
	double nq = in_quat_unit.n2_Norm_Two();
	if(nq < 4.0e-7){ out_mat_rot.ci_Create_Identity_matrix(3); return; }	
	if(out_mat_rot.mw() != 3 || out_mat_rot.mh() != 3) out_mat_rot.ci_Create_Identity_matrix(3);
	
	// normalized q.
	in_quat_unit *= sqrt(2.0f)/nq;
	//for(int i=0; i<4; i++) p_q[i] *= sqrt(2.0f)/nq;
	//in_quat_unit /= nq;

	//printf("q: %f %f %f %f\n", p_quater[0], p_quater[1], p_quater[2], p_quater[3]);

	// https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
	// q*q^T.
	for(int i=0; i<4; i++) q[i] = p_q[i];
	d_mvvt_Multiply_Vector_Vector_Transpose(q, q, 4, qq[0]);

	// R(w) = (cos(theta))I + sin(theta)[n]x + (1-cos(theta))n*n^T
	p_rot[0] = 1.0 - qq[1][1] - qq[2][2]; p_rot[1] =     + qq[0][1] - qq[2][3]; p_rot[2] =     + qq[0][2] + qq[1][3];
	p_rot[3] =	   + qq[0][1] + qq[2][3]; p_rot[4] = 1.0 - qq[0][0] - qq[2][2]; p_rot[5] =     + qq[1][2] - qq[0][3];
	p_rot[6] =     + qq[0][2] - qq[1][3]; p_rot[7] =     + qq[1][2] + qq[0][3]; p_rot[8] = 1.0 - qq[0][0] - qq[1][1];
// 
//  	p_rot[0] = 1.0 - 2*(p_q[1]*p_q[1] + p_q[2]*p_q[2]);
//  	p_rot[1] =     + 2*(p_q[0]*p_q[1] - p_q[2]*p_q[3]);
//  	p_rot[2] =     + 2*(p_q[0]*p_q[2] + p_q[1]*p_q[3]);
//  
//  	p_rot[3] =	   + 2*(p_q[0]*p_q[1] + p_q[2]*p_q[3]);
//  	p_rot[4] = 1.0 - 2*(p_q[0]*p_q[0] + p_q[2]*p_q[2]); 
//  	p_rot[5] =     + 2*(p_q[1]*p_q[2] - p_q[0]*p_q[3]); 
//  
//  	p_rot[6] =     + 2*(p_q[0]*p_q[2] - p_q[1]*p_q[3]);
//  	p_rot[7] =     + 2*(p_q[1]*p_q[2] + p_q[0]*p_q[3]);
//  	p_rot[8] = 1.0 - 2*(p_q[0]*p_q[0] + p_q[1]*p_q[1]); 
// 
// 	// https://en.wikipedia.org/wiki/Euler%E2%80%93Rodrigues_formula
// 	// normalized q.
// 	in_quat_unit /= nq;
// 	
// 	float a,b,c,d;
// 
// 	//a = p_q[3]; b = p_q[0]; c = p_q[1]; d = p_q[2];
// 	a = p_q[0]; b = p_q[1]; c = p_q[2]; d = p_q[3];
// 
// 	p_rot[0] = a*a + b*b - c*c - d*d; p_rot[1] = 2.0f*(b*c - a*d); p_rot[2] = 2.0f*(b*d + a*c);
// 	p_rot[3] = 2.0f*(b*c + a*d); p_rot[4] = a*a - b*b + c*c - d*d; p_rot[5] = 2.0f*(c*d - a*b);
// 	p_rot[6] = 2.0f*(b*d - a*c); p_rot[7] = 2.0f*(c*d + a*b); p_rot[8] = a*a - b*b - c*c + d*d;

}

//********************************************************************************************
// https://en.wikipedia.org/wiki/Rotation_matrix#Quaternion
// 아직 불안정하다...
void CKvYooji_Extrinsics::convert_rotation_mat_to_quaternion(
		CKvMatrixFloat &in_mat_rot,
		CKvVectorFloat &out_quat_unit)
//********************************************************************************************
{
	float *p_R = in_mat_rot.vp();
	float *p_q = out_quat_unit.vp();

	double tr_R = p_R[0] + p_R[4] + p_R[8];

	if(1){
	//if(tr_R > 0.0f){

		double r = sqrt(1 + tr_R);
		double s = 0.5f/r;
		double qw = 0.5f*r;
		double qx = (p_R[2*3 + 1] - p_R[1*3 + 2])*s;		// (Qzy - Qyz)*s
		double qy = (p_R[0*3 + 2] - p_R[2*3 + 0])*s;		// (Qxz - Qzx)*s
		double qz = (p_R[1*3 + 0] - p_R[0*3 + 1])*s;		// (Qyx - Qxy)*s

		p_q[0] = qx; p_q[1] = qy; p_q[2] = qz; p_q[3] = qw;

		out_quat_unit /= out_quat_unit.n2_Norm_Two();

	}
	else{
		
		float max_r = max(abs(p_R[0]), max(abs(p_R[1]), abs(p_R[2])));

		if(abs(p_R[0]) == max_r){

		}
		else if(p_R[1] == max_r){

		}
		else{

		}

	}	
}

//********************************************************************************************
void CKvYooji_Extrinsics::get_cam_center(CKvPoint3Df &out_center)
//********************************************************************************************
{
	// pose = | R t |
	//		  | 0 1 |
	// camera center in global coordinates,
	// C=-R^-1*t from T=-RC.
	float *p_mat=zz_mat.vp();
	float tx, ty, tz;

	tx=p_mat[4*0+3];	ty=p_mat[4*1+3];	tz=p_mat[4*2+3];
	out_center.x=-p_mat[4*0+0]*tx -p_mat[4*1+0]*ty -p_mat[4*2+0]*tz;
	out_center.y=-p_mat[4*0+1]*tx -p_mat[4*1+1]*ty -p_mat[4*2+1]*tz;
	out_center.z=-p_mat[4*0+2]*tx -p_mat[4*1+2]*ty -p_mat[4*2+2]*tz;	

}

//********************************************************************************************
void CKvYooji_Extrinsics::get_pose_relative(
		CKvYooji_Extrinsics &in_pose_ref,
		CKvYooji_Extrinsics &in_pose_curr,
		CKvYooji_Extrinsics &out_pose_relative)
//********************************************************************************************
{
	CKvMatrixFloat mat_relative;
	float *p_mat_ref_inv, *p_mat_curr, *p_mat_relative;

	p_mat_ref_inv = in_pose_ref.mp_transform_inv()->vp();
	p_mat_curr = in_pose_curr.mp_transform()->vp();
	
	p_mat_relative = mat_relative.ci_Create_Identity_matrix(4)[0];

	// T_relative * T_ref = T_curr -> T_relative = T_curr * T_ref_inv
	d_mms_Multiply_Matrix_Square(p_mat_curr, p_mat_ref_inv, 4, p_mat_relative);
	out_pose_relative.set_from_transform(&mat_relative);

}

//********************************************************************************************
void CKvYooji_Extrinsics::get_pose_successive(
		CKvYooji_Extrinsics &in_pose_ref,
		CKvYooji_Extrinsics &in_pose_relative,
		CKvYooji_Extrinsics &out_pose_successive)
//********************************************************************************************
{
	CKvMatrixFloat mat_successive;
	float *p_mat_ref, *p_mat_relative, *p_mat_successive;

	p_mat_ref = in_pose_ref.mp_transform()->vp();
	p_mat_relative = in_pose_relative.mp_transform()->vp();
	
	p_mat_successive = mat_successive.ci_Create_Identity_matrix(4)[0];

	// T_successive = T_relative * T_ref
	d_mms_Multiply_Matrix_Square(p_mat_relative, p_mat_ref, 4, p_mat_successive);
	out_pose_successive.set_from_transform(&mat_successive);

}

//********************************************************************************************
void CKvYooji_Extrinsics::print_transform(const char *in_string)
//********************************************************************************************
{
	float *p_t = zz_mat.vp();

	printf(in_string);	printf(" = \n");
	
	for (int j = 0; j < 4; j++){
		for (int i = 0; i < 4; i++){
			printf(" %5.7f ", p_t[j * 4 + i]);
		}
		printf("\n");
	}

	printf("\n");	
}

//********************************************************************************************
void CKvYooji_Extrinsics::transform(CKvPoint3Df &in_p3d, CKvPoint3Df &out_p)
//********************************************************************************************
{ 
	float *p_mat=zz_mat.vp();
	out_p.x=p_mat[0]*in_p3d.x	+p_mat[1]*in_p3d.y	+p_mat[2]*in_p3d.z	+p_mat[3];
	out_p.y=p_mat[4]*in_p3d.x	+p_mat[5]*in_p3d.y	+p_mat[6]*in_p3d.z	+p_mat[7];
	out_p.z=p_mat[8]*in_p3d.x	+p_mat[9]*in_p3d.y	+p_mat[10]*in_p3d.z	+p_mat[11];
}
//********************************************************************************************
void CKvYooji_Extrinsics::transform_inv(CKvPoint3Df &in_p3d, CKvPoint3Df &out_p)
//********************************************************************************************
{ 
	float *p_mat=zz_mat_inv.vp();
	out_p.x=p_mat[0]*in_p3d.x	+p_mat[1]*in_p3d.y+p_mat[2]*in_p3d.z	+p_mat[3];
	out_p.y=p_mat[4]*in_p3d.x	+p_mat[5]*in_p3d.y+p_mat[6]*in_p3d.z	+p_mat[7];
	out_p.z=p_mat[8]*in_p3d.x	+p_mat[9]*in_p3d.y+p_mat[10]*in_p3d.z	+p_mat[11];
}

//********************************************************************************************
bool CKvYooji_Extrinsics::z_im4x4_Inverse_Matrix_4x4(float *in_mat, float *out_mat)
//********************************************************************************************
{
	float m11, m12, m13, m14;
	float m21, m22, m23, m24;
	float m31, m32, m33, m34;
	float m41, m42, m43, m44;
	float t[12], det;

	// input matrix.
	m11=in_mat[0];	m12=in_mat[1];	m13=in_mat[2];	m14=in_mat[3];
	m21=in_mat[4];	m22=in_mat[5];	m23=in_mat[6];	m24=in_mat[7];
	m31=in_mat[8];	m32=in_mat[9];	m33=in_mat[10];	m34=in_mat[11];
	m41=in_mat[12];	m42=in_mat[13];	m43=in_mat[14];	m44=in_mat[15];

	// pre-computation.
	t[0]=m33*m44;	
	t[1]=m34*m42;	
	t[2]=m32*m43;	
	t[3]=m34*m43;
	t[4]=m32*m44;	
	t[5]=m33*m42;

	t[6]=m31*m44;	
	t[7]=m33*m41;	
	t[8]=m34*m41;	
	t[9]=m31*m43;
	t[10]=m31*m42;	
	t[11]=m32*m41;

	// compute determinant.
	out_mat[0]=m22*t[0]	+m23*t[1] +m24*t[2]	-m22*t[3] -m23*t[4] -m24*t[5];
	out_mat[1]=m12*t[3]	+m13*t[4] +m14*t[5] -m12*t[0] -m13*t[1]	-m14*t[2];
	out_mat[2]=m12*m23*m44	+m13*m24*m42 +m14*m22*m43 -m12*m24*m43	-m13*m22*m44 -m14*m23*m42;
	out_mat[3]=m12*m24*m33	+m13*m22*m34 +m14*m23*m32 -m12*m23*m34	-m13*m24*m32 -m14*m22*m33;

	det=m11*out_mat[0] +m21*out_mat[1] +m31*out_mat[2] + m41*out_mat[3];

	if(det==0.0f)	return false;

	out_mat[4]=m21*t[3]	+m23*t[6] +m24*t[7] -m21*t[0] -m23*t[8] -m24*t[9];
	out_mat[5]=m11*t[0]	+m13*t[8] +m14*t[9] -m11*t[3] -m13*t[6] -m14*t[7];
	out_mat[6]=m11*m24*m43	+m13*m21*m44 +m14*m23*m41 -m11*m23*m44	-m13*m24*m41 -m14*m21*m43;
	out_mat[7]=m11*m23*m34	+m13*m24*m31 +m14*m21*m33 -m11*m24*m33	-m13*m21*m34 -m14*m23*m31;

	out_mat[8]=m21*t[4]	+m22*t[8] +m24*t[10] -m21*t[1] -m22*t[6] -m24*t[11];
	out_mat[9]=m11*t[1]	+m12*t[6] +m14*t[11] -m11*t[4] -m12*t[8] -m14*t[10];
	out_mat[10]=m11*m22*m44 +m12*m24*m41 +m14*m21*m42 -m11*m24*m42	-m12*m21*m44 -m14*m22*m41;
	out_mat[11]=m11*m24*m32 +m12*m21*m34 +m14*m22*m31 -m11*m22*m34	-m12*m24*m31 -m14*m21*m32;

	out_mat[12]=m21*t[5] +m22*t[9] +m23*t[11] -m21*t[2] -m22*t[7] -m23*t[10];
	out_mat[13]=m11*t[2] +m12*t[11] +m13*t[10] -m11*t[5] -m12*t[9] -m13*t[11];
	out_mat[14]=m11*m23*m42 +m12*m21*m43 +m13*m22*m41 -m11*m22*m43	-m12*m23*m41 -m13*m21*m42;
	out_mat[15]=m11*m22*m33 +m12*m23*m31 +m13*m21*m32 -m11*m23*m32	-m12*m21*m33 -m13*m22*m31;

	det=1.0f/det;
	for(int i=0; i<16; i++)  out_mat[i]*=det;
	


	return true;
}

//********************************************************************************************
// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM
// Reference: http://crrl.poly.edu/522/rotation_matrix_representations.pdf
// Axis-angle representation:
bool CKvYooji_Extrinsics::z_gtp_Get_Transformation_from_Params(const float *in_params, float *out_mat)
//********************************************************************************************
{
	//float *p_mat = zz_mat.vp();
	float *p_R = zz_R_mat.vp();
	float *p_t = zz_t_mat.vp();

	float one_6th = 1.0f/6.0f;
	float one_20th = 1.0f/20.0f;

	Vector3f w; w.x = in_params[0]; w.y = in_params[1]; w.z = in_params[2];
	Vector3f t; t.x = in_params[3]; t.y = in_params[4]; t.z = in_params[5];

	//float theta_sq = VectorDotProduct_3(&w, &w);
	float theta_sq = dot(w,w);
	float theta = sqrtf(theta_sq);

	float A,B;

	Vector3f crossV = cross(w,t);// , buffV3; VectorCrossProduct_3(&crossV, &w, &t, &buffV3);
	if(theta_sq < 1e-8f)
	{
		A = 1.0f - one_6th * theta_sq; B = 0.5f;
		p_t[0] = t.x + 0.5f * crossV.x; p_t[1] = t.y + 0.5f * crossV.y; p_t[2] = t.z + 0.5f * crossV.z;
	} else
	{
		float C;
		if(theta_sq < 1e-6f)
		{
			C = one_6th * (1.0f - one_20th * theta_sq);
			A = 1.0f - theta_sq * C;
			B = 0.5f - 0.25f * one_6th * theta_sq;
		} else
		{
			float inv_theta = 1.0f / theta;
			A = sinf(theta) * inv_theta;
			B = (1.0f - cosf(theta)) * (inv_theta * inv_theta);
			C = (1.0f - A) * (inv_theta * inv_theta);
		}

		Vector3f cross2 = cross(w,crossV);
		//VectorCrossProduct_3(&cross2, &w, &crossV, &buffV3);

		p_t[0] = t.x + B * crossV.x + C * cross2.x; p_t[1] = t.y + B * crossV.y + C * cross2.y; p_t[2] = t.z + B * crossV.z + C * cross2.z;
	}

	float wx2 = w.x * w.x,wy2 = w.y * w.y,wz2 = w.z * w.z;
	p_R[0*3 + 0] = 1.0f - B*(wy2 + wz2);
	p_R[1*3 + 1] = 1.0f - B*(wx2 + wz2);
	p_R[2*3 + 2] = 1.0f - B*(wx2 + wy2);

	float a,b;
	a = A * w.z,b = B * (w.x * w.y);
	p_R[0*3 + 1] = b - a;
	p_R[1*3 + 0] = b + a;

	a = A * w.y,b = B * (w.x * w.z);
	p_R[0*3 + 2] = b + a;
	p_R[2*3 + 0] = b - a;

	a = A * w.x,b = B * (w.y * w.z);
	p_R[1*3 + 2] = b - a;
	p_R[2*3 + 1] = b + a;

	// set output trasformation matrix.
	for(int i=0; i<3; i++) for(int j=0; j<3; j++) out_mat[i*4+j] = p_R[i*3+j];
	for(int i=0; i<3; i++) out_mat[i*4+3] = p_t[i];

	return true;
}

//********************************************************************************************
// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM
// Reference: http://crrl.poly.edu/522/rotation_matrix_representations.pdf
// Axis-angle representation:
bool CKvYooji_Extrinsics::z_gpt_Get_Params_from_Transformation(const float *in_mat, float *out_params)
//********************************************************************************************
{
	float R[9], t[3];

	// get rotation matrix.
	for(int j=0; j<3; j++)	for(int i=0; i<3; i++)	R[j*3+i]=in_mat[j*4+i];
	// get translation vector.
	for(int j=0; j<3; j++)	t[j]=in_mat[j*4+3];

	Vector3f resultRot;

	float cos_angle = (R[0*3 + 0]  + R[1*3 + 1] + R[2*3 + 2] - 1.0f) * 0.5f;
	resultRot.x = (R[2*3 + 1] - R[1*3 + 2]) * 0.5f;
	resultRot.y = (R[0*3 + 2] - R[2*3 + 0]) * 0.5f;
	resultRot.z = (R[1*3 + 0] - R[0*3 + 1]) * 0.5f;

// 	if(!Kv_Printf("%f %f %f %f %f %f\n",resultRot.x,resultRot.y,resultRot.z,
// 	t[0],t[1],t[2])) exit(0);

	float sin_angle_abs = sqrtf(dot(resultRot,resultRot));

	// Compute rotation axis.
	if(cos_angle > M_SQRT1_2)
	{
		if(sin_angle_abs)
		{
			float p = asinf(sin_angle_abs) / sin_angle_abs;
			resultRot.x *= p; resultRot.y *= p; resultRot.z *= p;
		}
	} else
	{
		if(cos_angle > -M_SQRT1_2)
		{
			float p = acosf(cos_angle) / sin_angle_abs;
			resultRot.x *= p; resultRot.y *= p; resultRot.z *= p;
		} else
		{
			float angle = (float)M_PI - asinf(sin_angle_abs);
			float d0 = R[0*3 + 0] - cos_angle;
			float d1 = R[1*3 + 1] - cos_angle;
			float d2 = R[2*3 + 2] - cos_angle;

			Vector3f r2;

			if(fabsf(d0) > fabsf(d1) && fabsf(d0) > fabsf(d2))
			{
				r2.x = d0; r2.y = (R[1*3 + 0] + R[0*3 + 1]) * 0.5f; r2.z = (R[0*3 + 2] + R[2*3 + 0]) * 0.5f;
			} else
			{
				if(fabsf(d1) > fabsf(d2))
				{
					r2.x = (R[1*3 + 0] + R[0*3 + 1]) * 0.5f; r2.y = d1; r2.z = (R[2*3 + 1] + R[1*3 + 2]) * 0.5f;
				} else { r2.x = (R[0*3 + 2] + R[2*3 + 0]) * 0.5f; r2.y = (R[2*3 + 1] + R[1*3 + 2]) * 0.5f; r2.z = d2; }
			}

			//if (VectorDotProduct_3(&r2, &resultRot) < 0.0f)
			if(dot(r2,resultRot) < 0.0f) { r2.x *= -1.0f; r2.y *= -1.0f; r2.z *= -1.0f; }

			//VectorNormalize_3(&r2, &r2);
			r2 = normalize(r2);

			resultRot.x = angle * r2.x; resultRot.y = angle * r2.y; resultRot.z = angle * r2.z;
		}
	}

	float shtot = 0.5f;
	//float theta = sqrtf(VectorDotProduct_3(&resultRot, &resultRot));
	float theta = sqrtf(dot(resultRot,resultRot));

	if(theta > 0.00001f) shtot = sinf(theta * 0.5f) / theta;

	CKvYooji_Extrinsics halfrotor;	
	CKvMatrixFloat mat;
	float halfrotorparams[6], *p_mat;
	float rott[3], *p_R;
	halfrotorparams[0] = resultRot.x * -0.5f; halfrotorparams[1] = resultRot.y * -0.5f; halfrotorparams[2] = resultRot.z * -0.5f;
	halfrotorparams[3] = 0.0f; halfrotorparams[4] = 0.0f; halfrotorparams[5] = 0.0f;
	
	p_mat = mat.c_Create(4, 4)[0];
	z_gtp_Get_Transformation_from_Params(halfrotorparams, p_mat);
	halfrotor.set_from_transform(&mat); 

	p_R = halfrotor.mp_rotation()->vp();
	d_mmsv_Multiply_Matrix_Square_Vector(p_R, t, 3, rott);
	//Vector3f rottrans, buffV3;
	//MatrixVectorMultiply_3(&rottrans, &halfrotor.R, &this->T, &buffV3);
	
	Vector3f rottrans, T;// = halfrotor.R * T;
	rottrans.x = rott[0]; rottrans.y = rott[1]; rottrans.z = rott[2];
	T.x = t[0]; T.y = t[1]; T.z = t[2];

// 	if(!Kv_Printf("%f %f %f %f %f %f\n", rottrans.x, rottrans.y, rottrans.z, 
// 		T.x, T.y, T.z)) exit(0);

	if(theta > 0.001f)
	{
		//float denom = VectorDotProduct_3(&resultRot, &resultRot);
		//float param = VectorDotProduct_3(&this->T, &resultRot) * (1 - 2 * shtot) / denom;
		float denom = dot(resultRot,resultRot);
		float param = dot(T,resultRot) * (1 - 2 * shtot) / denom;

		rottrans.x -= resultRot.x * param; rottrans.y -= resultRot.y * param; rottrans.z -= resultRot.z * param;
	} else
	{
		//float param = VectorDotProduct_3(&this->T, &resultRot) / 24;
		float param = dot(T,resultRot) / 24;
		rottrans.x -= resultRot.x * param; rottrans.y -= resultRot.y * param; rottrans.z -= resultRot.z * param;
	}

	rottrans.x /= 2 * shtot; rottrans.y /= 2 * shtot; rottrans.z /= 2 * shtot;

	out_params[0] = resultRot.x; out_params[1] = resultRot.y; out_params[2] = resultRot.z;
	out_params[3] = rottrans.x; out_params[4] = rottrans.y; out_params[5] = rottrans.z;


	return true;
} 