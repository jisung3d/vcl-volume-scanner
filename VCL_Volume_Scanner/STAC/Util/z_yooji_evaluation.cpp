/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_evaluation.cpp 
/////////////////////////////////////////////////////////////////////////////////////////////

#include "../../stdafx.h"
#include "../_yooji_2016_object_scanning.h"

/////////////////////////////////////////////////////////////////////////////////////////////
// LCKvYooji_Evaluation 
/////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
LCKvYooji_Evaluation::LCKvYooji_Evaluation()
//********************************************************************************************
{
	// set default value of projected camera center.
	zz_classname = "LCKvYooji_Evaluation";
	zz_mat_4x4.ci_Create_Identity_matrix(4);
}

//********************************************************************************************
LCKvYooji_Evaluation::~LCKvYooji_Evaluation()
//********************************************************************************************
{
}

//********************************************************************************************
// CAUTION:
// + Transformation of principal axis is quite different with transformation of 3D point.
// + Transformation of 3D point is changing 3D point position to the new transformed coordinates.
// + However, transformation of principal axis is ...
bool LCKvYooji_Evaluation::mee_Measure_Estimation_Error_using_Principal_Axis(
	CKvYooji_Extrinsics *in_pose_init_or_NULL,
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Tracking_State *in_state,
	float &out_dist_rotation,
	float &out_dist_translation)
//********************************************************************************************
{
	CKvYooji_Extrinsics ext1, ext2;
	CKvPmatrix3D *p_pmat;
	CKvMatrixFloat *mp_t_gt, *mp_t_est;	
	CKvPoint3D tp3d;
	float axis_init[4], axis_gt[4], axis_est[4], axis_tmp[4];

	float *p_mat1, a_mat2[9];

	out_dist_rotation = out_dist_translation = 0.0f;

	// get ground truth camera information.
	p_pmat = in_view->p_P_matrix_depth();
	tp3d = p_pmat->dvo_Direction_Vector_of_Optical_axis();

	ext1.set_from_P_matrix(p_pmat);
	mp_t_gt = ext1.mp_translation();

	axis_gt[0] = (float)tp3d.x;	axis_gt[1] = (float)tp3d.y;	axis_gt[2] = (float)tp3d.z;  axis_gt[3] = 1.0f;
	axis_init[0] = 0.0f;	axis_init[1] = 0.0f;	axis_init[2] = 1.0f;  axis_init[3] = 1.0f;
	// ///////////////////////////////////////////////////////////////////	
	
	// compensate ground truth camera information based on initial (1th frame) camera pose.
	if(in_pose_init_or_NULL){

		float *p_T_gt, *p_T_init_inv, *p_T_curr;

		p_T_gt = zz_mat_4x4.vp();
		p_T_curr = ext1.mp_transform()->vp();
		p_T_init_inv = in_pose_init_or_NULL->mp_transform_inv()->vp();		

		// compute ground truth principal axis.
		// get transformation that transforms initial principal axis to z-axis (p_mat1).
		p_mat1 = in_pose_init_or_NULL->mp_rotation()->vp();	// principal axis -> z-axis. (p_mat1)	
		for(int i=0; i<4; i++)	axis_tmp[i] = axis_gt[i];
		d_mmsv_Multiply_Matrix_Square_Vector(p_mat1, axis_tmp, 3, axis_gt); 
		
		// compute ground truth translation.
		d_mms_Multiply_Matrix_Square(p_T_curr, p_T_init_inv, 4, p_T_gt);
		p_mat1 = mp_t_gt->vp();	for(int i=0; i<3; i++){	p_mat1[i] = p_T_gt[i*4+3];	}
	}

	// get estimated camera information.
	//in_state->ph3d_Pointer_of_3D_Homography_Global_to_Camera()->e_Export(&zz_mat_4x4);	
	//ext2.st_Set_from_Transformation(&zz_mat_4x4);
	ext2.copy(in_state->p_extrinsics_glob_to_cam());
	// get estimated principal axis.
	// get transformation that transforms z-axis to estimated principal axis (a_mat).
	p_mat1 = ext2.mp_rotation()->vp(); // principal axis -> z-axis. (p_mat1)
	d_ts_Transpose_Square(p_mat1, 3, a_mat2); // z-axis -> principal axis. (a_mat2)
	d_mmsv_Multiply_Matrix_Square_Vector(a_mat2, axis_init, 3, axis_est);
	
	// get estimated translation.
	mp_t_est = ext2.mp_translation();
	// measure distance of rotation matrices.
	d_ipv_Inner_Product_Vector(axis_est, axis_gt, 3, out_dist_rotation);
	out_dist_rotation = (float)(acos(out_dist_rotation)*180.0f/PI);
	// measure distance of translations.
	d_dev_Distance_Euclidean_Vector(mp_t_gt->vp(), mp_t_est->vp(), 3, out_dist_translation);

	return true;
}

//********************************************************************************************
bool LCKvYooji_Evaluation::mee_Measure_Estimation_Error_using_Geodesic_Distance(
	CKvYooji_Extrinsics *in_pose_init_or_NULL,
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Tracking_State *in_state,
	float &out_dist_rotation,
	float &out_dist_translation)
//********************************************************************************************
{
	CKvYooji_Extrinsics ext1, ext2;
	CKvPmatrix3D *p_pmat, tpmat;
	CKvMatrixFloat *mp_R_gt, *mp_t_gt, *mp_R_est, *mp_t_est;	

	float *p_mat1, p_mat2[9], p_R_dist[9];
	float trace;

	out_dist_rotation = out_dist_translation = 0.0f;

	// get ground truth camera information.
	p_pmat = in_view->p_P_matrix_depth();
	ext1.set_from_P_matrix(p_pmat);
	mp_R_gt = ext1.mp_rotation();
	mp_t_gt = ext1.mp_translation();
	
	if(in_pose_init_or_NULL){
		CKvYooji_Intrinsics int_init;
		CKvYooji_Extrinsics ext_rel, ext_init;
		int_init.set_from_P_matrix((*in_view).p_P_matrix_depth());
		ext_init.set_from_P_matrix((*in_view).p_P_matrix_depth());
		ext_rel.get_pose_relative(*in_pose_init_or_NULL, ext_init, ext_rel);
		in_state->compute_P_matrix(&int_init, &ext_rel, &tpmat);
		ext1.set_from_P_matrix(&tpmat);
// 		float *p_T_init_inv, *p_T_gt, *p_T_curr;
// 
// 		p_T_gt = zz_mat_4x4.vp();
// 		p_T_curr = ext1.mp_Matrix_Pointer()->vp();
// 		p_T_init_inv = in_pose_init_or_NULL->mpi_Matrix_Pointer_Inverse()->vp();
// 		d_mms_Multiply_Matrix_Square(p_T_curr, p_T_init_inv, 4, p_T_gt);
// 
// 		p_mat1 = mp_R_gt->vp();	for(int j=0; j<3; j++){	for(int i=0; i<3; i++){	p_mat1[j*3+i] = p_T_gt[j*4+i];	}}
// 		p_mat1 = mp_t_gt->vp();	for(int i=0; i<3; i++){	p_mat1[i] = p_T_gt[i*4+3];	}
	}

	// get estimated camera information.
	//in_state->ph3d_Pointer_of_3D_Homography_Global_to_Camera()->e_Export(&zz_mat_4x4);
	//ext2.st_Set_from_Transformation(&zz_mat_4x4);

	ext2.copy(in_state->p_extrinsics_glob_to_cam());
	mp_R_est = ext2.mp_rotation();
	mp_t_est = ext2.mp_translation();
	
	// measure distance of rotation matrices.
	p_mat1 = mp_R_gt->vp();		
	d_ts_Transpose_Square(mp_R_est->vp(), 3, p_mat2);	
	d_mms_Multiply_Matrix_Square(p_mat1, p_mat2, 3, p_R_dist);	

	trace = p_R_dist[0] + p_R_dist[4] + p_R_dist[8];
	out_dist_rotation = (float)(acos((trace-1.0f)*0.5f)*180.0f/PI);

	d_dev_Distance_Euclidean_Vector(mp_t_gt->vp(), mp_t_est->vp(), 3, out_dist_translation);

	return true;
}
