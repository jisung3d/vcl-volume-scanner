/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_camera_tracker.cpp
/////////////////////////////////////////////////////////////////////////////////////////////

#include "../../../stdafx.h"
#include "../../_yooji_2016_object_scanning.h"

/////////////////////////////////////////////////////////////////////////////////////////////
// LCKvYooji_Camera_Tracker 
/////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
LCKvYooji_Camera_Tracker::LCKvYooji_Camera_Tracker()
//********************************************************************************************
{
	zz_classname="LCKvYooji_CameraTracker";

	//////////////////////////////////////////////////////////////////////////
	//aa_imatch.i_Initialize();
	//////////////////////////////////////////////////////////////////////////

	zz_mat_4x4.ci_Create_Identity_matrix(4);
	for(int i=0; i<4; i++) zz_ATA_6x6[i].c_Create(6, 6, 0.0f);
	zz_ATA_3x3.c_Create(3, 3, 0.0f); zz_ATA_2x2.c_Create(2, 2, 0.0f);
	for(int i=0; i<4; i++) zz_ATb_6[i].c_Create(6, 0.0f);	
	zz_ATb_3.c_Create(3, 0.0f);		zz_ATb_2.c_Create(2, 0.0f);
	zz_sol_x_6.c_Create(6, 0.0f);	zz_sol_x_3.c_Create(3, 0.0f);	 zz_sol_x_2.c_Create(2, 0.0f);

	zz_mat_4x4_d.ci_Create_Identity_matrix(4);
	zz_ATA_6x6_d.c_Create(6,6,0.0);	zz_ATA_3x3_d.c_Create(3,3,0.0); zz_ATA_2x2_d.c_Create(2,2,0.0);
	zz_ATb_6_d.c_Create(6,0.0);		zz_ATb_3_d.c_Create(3,0.0);		zz_ATb_2_d.c_Create(2,0.0);
	zz_sol_x_6_d.c_Create(6,0.0);	zz_sol_x_3_d.c_Create(3,0.0);	zz_sol_x_2_d.c_Create(2,0.0);

	zz_hmat_t0_to_t1_estimated.cp_Copy(&zz_mat_4x4);
	zz_hmat_t1_to_t0_estimated.cp_Copy(&zz_mat_4x4);

	zz_hmat_cam_to_glob_estimated.ci_Create_Identity_matrix(4);
	zz_hmat_glob_to_cam_d.ci_Create_Identity_matrix(4);

	/////////////////////////////////////////////////////////////////////////////
	zz_prior_4x4.ci_Create_Identity_matrix(4);
	zz_prior_x_6.c_Create(6,0.0f);
	zz_eye_6x6.ci_Create_Identity_matrix(6);
		//////////////////////////////////////////////////////////////////////////

	// for SIFT.
// 	zz_p3d_matched_prev.clear();	zz_p3d_matched_curr.clear();
// 	zz_p2d_matched_prev.clear();	zz_p2d_matched_curr.clear();
	//zz_p3d_matched_prev=new Vector3f[2*1000];	zz_p3d_matched_curr=new Vector3f[2*1000];
	//zz_p2d_matched_prev=new Vector2f[2*1000];	zz_p2d_matched_curr=new Vector2f[2*1000];
}

//********************************************************************************************
LCKvYooji_Camera_Tracker::~LCKvYooji_Camera_Tracker()
//********************************************************************************************
{
// 	zz_p3d_matched_prev.clear();	zz_p3d_matched_curr.clear();
// 	zz_p2d_matched_prev.clear();	zz_p2d_matched_curr.clear();

	//delete[] zz_p3d_matched_prev;
	//delete[] zz_p3d_matched_curr;
	//delete[] zz_p2d_matched_prev;
	//delete[] zz_p2d_matched_curr;
}

//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************
//********************************************************************************************


//********************************************************************************************
bool LCKvYooji_Camera_Tracker::tcsift_Track_Camera_with_SIFT(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Scanner_Parameter *in_params,
	CKvYooji_Tracking_State *io_state,
	LCKvImageMatcher *in_matcher,
	CKvMatrixFloat *out_mat_pose)
//********************************************************************************************
{
	bool flag_valid = false;
	//////////////////////////////////////////////////////////////////////////
	bool flag_roi = false; //true;
	//////////////////////////////////////////////////////////////////////////

	// initialize inverse of estimated pose.
	//////////////////////////////////////////////////////////////////////////
	// initialize pose using spatial keypoints.
	//LCKvImageMatcher imatcher;	imatcher.i_Initialize();
	CKvMatrixUchar roi_t0, roi_t1;
	vector<Point2f> set_of_p2d_src, set_of_p2d_dst;
	Rect *p_roi_t0, *p_roi_t1;

	//set_of_p2d_src.reserve(1000);	set_of_p2d_dst.reserve(1000);

 //	CKvScreen sc[2];

// 	sc[0].s_d_Display(io_state->p_roi_image_rgb());
// 	sc[1].s_d_Display(in_view->p_roi_image_rgb());
// 	if(!Kv_Printf("ROI!")) exit(0);

	//int num_feat;
	//float p2d_matched_src[2*2000],p2d_matched_dst[2*2000];

// 	sc.s_d_Display(&io_state->p_pyramid_image_gray()->imgs[0]); 	if(!Kv_Printf("Gray 1!")) exit(0);
// 	sc.s_d_Display(&in_view->p_pyramid_image_gray()->imgs[0]);		if(!Kv_Printf("Gray 2!")) exit(0);
// 
// 	sc.s_d_Display(100.0f, 0.0f, io_state->p_roi_depth());		if(!Kv_Printf("Depth1!")) exit(0);
// 
// 	sc.s_d_Display(100.0f, 0.0f, &io_state->p_pyramid_map_depth_rendered()->imgs[0]);	
// 	if(!Kv_Printf("Depth2")) exit(0);


// 	cout << io_state->p_roi_object()->tl() << endl;
// 	cout << io_state->p_roi_object()->br() << endl;
// 	
// 	cout << in_view->p_roi_object()->tl() << endl;
// 	cout << in_view->p_roi_object()->br() << endl;



	
	//if(!Kv_Printf("ROI!")) exit(0);


	in_matcher->emf_Extract_and_Match_Features_using_SiftGPU(
		flag_roi ? *io_state->p_roi_image_gray() : io_state->p_pyramid_image_gray()->imgs[0],
		flag_roi ? *in_view->p_roi_image_gray(): in_view->p_pyramid_image_gray()->imgs[0],		
		set_of_p2d_src,
		set_of_p2d_dst,
		flag_roi ? io_state->p_roi_depth() : &io_state->p_pyramid_map_depth_rendered()->imgs[0]); //,in_view->p_roi_depth());

	if(set_of_p2d_src.size() < 6)	return false;

	// update matched points from roi to original image coordinates.
	if(flag_roi){
		p_roi_t0 = io_state->p_roi_object();
		p_roi_t1 = in_view->p_roi_object();

		for(int i=0; i<set_of_p2d_dst.size(); i++){

			Point2f &tp2d_src = set_of_p2d_src[i];
			Point2f &tp2d_dst = set_of_p2d_dst[i];

			tp2d_src.x += p_roi_t0->x;	tp2d_src.y += p_roi_t0->y;
			tp2d_dst.x += p_roi_t1->x;	tp2d_dst.y += p_roi_t1->y;
		}
	}

 	if(tccpnp_Track_Camera_Coarse_using_PnP(
 		set_of_p2d_src,
 		set_of_p2d_dst,
 		io_state->p_pyramid_map_p3d_rendered()->imgs[0],
 		//io_state->p_pyramid_map_p3d()->imgs[0],
 		in_view->p_pyramid_intrinsics()->intrins[0],
 		&zz_mat_4x4))
 	{
 		out_mat_pose->cp_Copy(&zz_mat_4x4);
 		flag_valid = true;
 	};

	
// 	aa_disp.dmr_Display_Matching_Result(
// 	 	&sc,
// 	 	io_state->p_image_RGB(),
// 	 	in_view->p_image_rgb(),
// 	 	set_of_p2d_src,
// 	 	set_of_p2d_dst);
// 	 
// 	//if(!Kv_Printf("Matched results!")) exit(0);


	return flag_valid;
}

//********************************************************************************************
bool LCKvYooji_Camera_Tracker::tccpnp_Track_Camera_Coarse_using_PnP(
	vector<Point2f> &in_p2d_matched_t0,
	vector<Point2f> &in_p2d_matched_t1,
	CKvSet2d_of_Point3Df &in_map_p3d_t0,
	CKvYooji_Intrinsics &in_intrinsics,
	CKvMatrixFloat *out_pose_init)
//********************************************************************************************
{
	vector<Point3f> p3d_matched_t0;
	vector<int> indices_inlier;
	CKvPoint3Df tp3d_k;

	Mat rvec,tvec;
	Mat intrins, distCoeffs;
	Mat matR,matKT,matKR;
	
	float *p_pose;
	float fx, fy, px, py, dmin, dmax;
	bool flag_suc;

	/// ****************************************************************
	intrins = Mat::eye(3,3,CV_64F);
	distCoeffs = Mat::zeros(5,1,CV_64F);

	// get vector of point 3D at t0.
	int num_points = in_p2d_matched_t1.size();
	int ww, hh;	in_map_p3d_t0.ms(ww, hh);

	in_intrinsics.get_params(fx, fy, px, py, dmin, dmax);
	intrins.ptr<double>()[0] = fx;	intrins.ptr<double>()[2] = px;
	intrins.ptr<double>()[4] = fy;  intrins.ptr<double>()[5] = py;

	p3d_matched_t0.clear();	p3d_matched_t0.resize(num_points);

	for(int i=0; i<num_points; i++){
		Point2f &tp2d = in_p2d_matched_t0[i];
		Point3f &tp3d = p3d_matched_t0[i];

		z_gip3d_Get_Interpolated_Point3D(tp2d.x, tp2d.y, ww, hh, &in_map_p3d_t0, tp3d_k);
		
		tp3d.x = tp3d_k.x; tp3d.y = tp3d_k.y; tp3d.z = tp3d_k.z;
	}
	
	//////////////////////////////////////////////////////////////////////////
	// 쓸데없는 페어가 많을 때에는 란삭이 더 좋음.
// 	if(!solvePnP(
// 		p3d_matched_t0,
// 		in_p2d_matched_t1,
// 		intrins,
// 		distCoeffs,
// 		rvec,tvec)) return false;
	if(!solvePnPRansac(
		p3d_matched_t0,
		in_p2d_matched_t1,
		intrins,
		distCoeffs,
		rvec,tvec,
		false, 50,
		2.0f, 0.95,
		indices_inlier)) return false;

// 	if(!Kv_Printf("Inliers: %f %% (%d/%d)!", (float)indices_inlier.size()/(float)p3d_matched_t0.size()*100.0f, 
// 		indices_inlier.size(), p3d_matched_t0.size())) exit(0);


	//////////////////////////////////////////////////////////////////////////

	Rodrigues(rvec,matR);

	// check maximum translation.
	float mag_sq = SQUARE(tvec.ptr<double>()[0])+SQUARE(tvec.ptr<double>()[1])+SQUARE(tvec.ptr<double>()[2]);
	if(mag_sq > SQUARE(MAX_TRANSLATION)) return false;

	if(out_pose_init->mw() != 4 || out_pose_init->mh() != 4) out_pose_init->ci_Create_Identity_matrix(4);
	p_pose = out_pose_init->vp();

	// R matrix.
	for(int j=0; j<3; j++) for(int i=0; i<3; i++) p_pose[j*4 + i] = matR.ptr<double>()[j*3 + i];
	// t vector.
	for(int j=0; j<3; j++) p_pose[j*4 + 3] = tvec.ptr<double>()[j];

	return true;
}

//********************************************************************************************
bool LCKvYooji_Camera_Tracker::tcdfm_Track_Camera_with_Depth_Frame_to_Model(CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Scanner_Parameter *in_params,
	CKvYooji_Tracking_State *io_state,
	bool in_mode_forward_params,
	bool in_mode_on_rgb,
	CKvMatrixFloat *in_init_pose)
//********************************************************************************************
{
	static int iter_num[4] = {10, 7, 7, 7}; 
	static float th_num[4] = {SQUARE(50), SQUARE(40), SQUARE(30), SQUARE(20)};

	int num_valid, num_level, num_th;
	bool flag_valid=true;
	float th_dist_ICP, mu, max_w;
		
	// initialize inverse of estimated pose.
	if(!in_mode_on_rgb) zz_transform_glob_to_cam.copy(io_state->p_extrinsics_glob_to_cam()); 
	else         		zz_transform_glob_to_cam.copy(io_state->p_extrinsics_glob_to_cam_RGB()); 

	//zz_transform_glob_to_cam.print_transform("zz_transform_glob_to_cam");	
	
	zz_hmat_t1_to_t0_estimated.ci_Create_Identity_matrix(4);
	in_params->gpicp_Get_Parameters_for_ICP_algorithm(num_level,th_dist_ICP,mu,max_w);

	if(in_init_pose){

		num_level = 1;

		// frame-to-frame
		zz_hmat_t0_to_t1_estimated.cp_Copy(in_init_pose);
		// frame-to-model
		d_im_Inverse_Matrix_4x4(in_init_pose->vp(),zz_hmat_t1_to_t0_estimated.vp());
	} else{
		//////////////////////////////////////////////////////////////////////////
		zz_hmat_t0_to_t1_estimated.ci_Create_Identity_matrix(4);
		zz_hmat_t1_to_t0_estimated.ci_Create_Identity_matrix(4);
		//////////////////////////////////////////////////////////////////////////
	}

	//d_pm_Printf_Matrix(zz_hmat_t0_to_t1_estimated.vp(),4,4,"T_init");

	//////////////////////////////////////////////////////////////////////////
	num_th = 20;
	//////////////////////////////////////////////////////////////////////////

	for(int k=num_level-1; k>=0; k--){
		
		//printf("num_level: %d\n",k);

		// iteration.
		for(int i=0; i<iter_num[k]; i++)
		{
			//printf("iter_num: %d\n",i);
			//start=clock();
			num_valid = gls_Generate_Linear_System_ICP_for_KinFu(
				in_view,
				io_state,
				&zz_hmat_t1_to_t0_estimated,
				&zz_ATA_6x6[0],
				&zz_ATb_6[0],
				k,
				th_dist_ICP,
				in_mode_forward_params,
				in_mode_on_rgb);

// 			printf("num_valid: %d\n", num_valid);
// 			d_pm_Printf_Matrix(zz_ATA_6x6[0].vp(), 6, 6, "ATA");
			//if(!Kv_Printf("!!!")) exit(0);




			if(num_valid > th_num[k])
			{		
				// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
				// TEST own linear system solver using LLD.
				float L[6*6], y[6];
				d_lld_LL_Decomposition(zz_ATA_6x6[0].vp(), 6, L);
				flag_valid = d_sls_Solve_Linear_System_using_LLD(L, zz_ATb_6[0].vp(), y, 6, zz_sol_x_6.vp());
				// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

				if(flag_valid)
				{
 					//printf("[%d] norm2: %f (%d valids)\n", k, zz_sol_x_6.n2_Norm_Two(), num_valid);

					// stop iteration if estimated pose was converged.
					//if(zz_sol_x_6.n2_Norm_Two()<6.0e-4)	break;
					float norm_inf;	d_niv_Norm_Infinity_Vector(zz_sol_x_6.vp(),6,norm_inf);
					if(norm_inf < 5.0e-7f) break;

					umb_Update_Motion_Backward(&zz_sol_x_6, &zz_hmat_t1_to_t0_estimated, in_mode_forward_params);
					//////////////////////////////////////////////////////////////////////////
					if(!cmv_Check_Motion_Validity(&zz_hmat_t1_to_t0_estimated,MAX_TRANSLATION,MAX_ROTATION)){
						if(k==0){
							//cp_T_t0_to_t1_est->cp_Copy(&zz_prior_4x4);	
							return false;
						} 
						else{

							//////////////////////////////////////////////////////////////////////////
							// re-initialize motion estimate.
							zz_hmat_t1_to_t0_estimated.ci_Create_Identity_matrix(4);
							zz_hmat_t1_to_t0_estimated.ci_Create_Identity_matrix(4);
							//////////////////////////////////////////////////////////////////////////
							break;
						}
					}
					//////////////////////////////////////////////////////////////////////////
					//else	uitcg_Update_Incremental_Tracking_t1_to_t0_with_Backward_Parameters(&zz_sol_x_6, &zz_hmat_t1_to_t0_estimated);				
				}
				else if(k>0){
					break;	// go to next level!
				}
				else return false;

			}
			else if(k>0){
				break;	// go to next level!
			} else return false;
		}

		// update number of iteration.
		//num_th *= 4;

	}

	//d_pm_Printf_Matrix(zz_hmat_cam_to_glob_estimated.vp(), 4, 4, "Hmat");
		
	if(flag_valid)
	{
		CKvYooji_Intrinsics *p_intrin_d = io_state->p_intrinsics_depth();
		CKvYooji_Intrinsics *p_intrin_rgb = io_state->p_intrinsics_depth();
		CKvYooji_Extrinsics *p_T_gc_d = io_state->p_extrinsics_glob_to_cam();
		CKvYooji_Extrinsics *p_T_gc_rgb = io_state->p_extrinsics_glob_to_cam_RGB();

		CKvYooji_Extrinsics *p_T_drgb = in_view->p_extrinsics_depth_to_RGB();
		CKvYooji_Extrinsics *p_T_rgbd = in_view->p_extrinsics_RGB_to_depth();

// 		p_T_drgb->print_transform("p_T_drgb");
// 		p_T_rgbd->print_transform("p_T_rgbd");


		//////////////////////////////////////////////////////////////////////////
		// 3D rigid transformation.
		//////////////////////////////////////////////////////////////////////////
		d_mms_Multiply_Matrix_Square(
			p_T_gc_rgb->mp_transform_inv()->vp(),
			zz_hmat_t1_to_t0_estimated.vp(),
			4,
			zz_hmat_cam_to_glob_estimated.vp());
		//////////////////////////////////////////////////////////////////////////
		//zz_transform_glob_to_cam.set_from_transform_inv(&zz_hmat_cam_to_glob_estimated);
		if(!in_mode_on_rgb){	
			// pose of depth camera.
			p_T_gc_d->set_from_transform_inv(&zz_hmat_cam_to_glob_estimated);			
			// pose of RGB camera.
			p_T_drgb->get_pose_successive(*p_T_gc_d, *p_T_drgb, *p_T_gc_rgb);
		}
		else{
			// pose of RGB camera.
			p_T_gc_rgb->set_from_transform_inv(&zz_hmat_cam_to_glob_estimated);			
			// pose of depth camera.
			p_T_rgbd->get_pose_successive(*p_T_gc_rgb, *p_T_rgbd, *p_T_gc_d);
		}

		io_state->compute_P_matrix(p_intrin_d, p_T_gc_d, io_state->p_P_matrix_depth());
		io_state->compute_P_matrix(p_intrin_rgb, p_T_gc_rgb, io_state->p_P_matrix_RGB());	
	}

	return flag_valid;
	
}

//********************************************************************************************
bool LCKvYooji_Camera_Tracker::tccf_Track_Camera_Coarse_to_Fine_with_KinFu(CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Scanner_Parameter *in_params,
	CKvYooji_Tracking_State *io_state,
	bool in_mode_on_rgb)
//********************************************************************************************
{
	static int iter_num[4] = {10, 7, 7, 7}; 

	int num_valid, num_level, num_th;
	bool flag_valid=true;
	float th_dist_ICP, mu, max_w;
		
	// initialize inverse of estimated pose.
	if(!in_mode_on_rgb) zz_transform_glob_to_cam.copy(io_state->p_extrinsics_glob_to_cam()); 
	else         		zz_transform_glob_to_cam.copy(io_state->p_extrinsics_glob_to_cam_RGB()); 

	//zz_transform_glob_to_cam.print_transform("zz_transform_glob_to_cam");	

	zz_hmat_cam_to_glob_estimated.cp_Copy(zz_transform_glob_to_cam.mp_transform_inv());
	in_params->gpicp_Get_Parameters_for_ICP_algorithm(num_level,th_dist_ICP,mu,max_w);

	//////////////////////////////////////////////////////////////////////////
	num_th = 20;
	//////////////////////////////////////////////////////////////////////////

	for(int k=num_level-1; k>=0; k--){

		// iteration.
		for(int i=0; i<iter_num[k]; i++)
		{
			//start=clock();
			num_valid = gls_Generate_Linear_System_ICP_for_KinFu(
				in_view,
				io_state,
				&zz_hmat_cam_to_glob_estimated,
				&zz_ATA_6x6[0],
				&zz_ATb_6[0],
				k,
				th_dist_ICP,
				false,
				in_mode_on_rgb);

			//printf("num_valid: %d\n", num_valid);
			//d_pm_Printf_Matrix(zz_ATA_6x6[0].vp(), 6, 6, "ATA");
			//if(!Kv_Printf("!!!")) exit(0);

			if(num_valid > num_th)
			{		
				// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
				// TEST own linear system solver using LLD.
				float L[6*6], y[6];
				d_lld_LL_Decomposition(zz_ATA_6x6[0].vp(), 6, L);
				flag_valid = d_sls_Solve_Linear_System_using_LLD(L, zz_ATb_6[0].vp(), y, 6, zz_sol_x_6.vp());
				// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

				// the number of overlapped surface points.	
// 				flag_valid=sls_Solve_Linear_System(&zz_ATA_6x6,
// 					&zz_ATb_6,
// 					&zz_sol_x_6);	

				if(flag_valid)
				{
// 					printf("x: %f %f %f | %f %f %f\n"
// 						, zz_sol_x_6.vp()[0], zz_sol_x_6.vp()[1], zz_sol_x_6.vp()[2]
// 						, zz_sol_x_6.vp()[3], zz_sol_x_6.vp()[4], zz_sol_x_6.vp()[5]);
 				//	printf("[%d] norm2: %f (%d valids)\n", k, zz_sol_x_6.n2_Norm_Two(), num_valid);

					// stop iteration if estimated pose was converged.
					//if(zz_sol_x_6.n2_Norm_Two()<6.0e-4)	break;
					float norm_inf;	d_niv_Norm_Infinity_Vector(zz_sol_x_6.vp(),6,norm_inf);
					if(norm_inf < 5.0e-7f) break;

					uitcg_Update_Incremental_Tracking_Cam_to_Glob_with_Backward_Params(&zz_sol_x_6, &zz_hmat_cam_to_glob_estimated);				
				}
				else if(k>0){
					break;	// go to next level!
				}
				else return false;

			}
			else if(k>0){
				break;	// go to next level!
			} else return false;
		}

		// update number of iteration.
		//num_th *= 4;

	}

	//d_pm_Printf_Matrix(zz_hmat_cam_to_glob_estimated.vp(), 4, 4, "Hmat");
		
	if(flag_valid)
	{
		CKvYooji_Intrinsics *p_intrin_d = io_state->p_intrinsics_depth();
		CKvYooji_Intrinsics *p_intrin_rgb = io_state->p_intrinsics_depth();
		CKvYooji_Extrinsics *p_T_gc_d = io_state->p_extrinsics_glob_to_cam();
		CKvYooji_Extrinsics *p_T_gc_rgb = io_state->p_extrinsics_glob_to_cam_RGB();

		CKvYooji_Extrinsics *p_T_drgb = in_view->p_extrinsics_depth_to_RGB();
		CKvYooji_Extrinsics *p_T_rgbd = in_view->p_extrinsics_RGB_to_depth();

// 		p_T_drgb->print_transform("p_T_drgb");
// 		p_T_rgbd->print_transform("p_T_rgbd");

		//////////////////////////////////////////////////////////////////////////
		// 3D rigid transformation.
		//zz_transform_glob_to_cam.set_from_transform_inv(&zz_hmat_cam_to_glob_estimated);
		if(!in_mode_on_rgb){	
			// pose of depth camera.
			p_T_gc_d->set_from_transform_inv(&zz_hmat_cam_to_glob_estimated);			
			// pose of RGB camera.
			p_T_drgb->get_pose_successive(*p_T_gc_d, *p_T_drgb, *p_T_gc_rgb);
		}
		else{
			// pose of RGB camera.
			p_T_gc_rgb->set_from_transform_inv(&zz_hmat_cam_to_glob_estimated);			
			// pose of depth camera.
			p_T_rgbd->get_pose_successive(*p_T_gc_rgb, *p_T_rgbd, *p_T_gc_d);
		}

		io_state->compute_P_matrix(p_intrin_d, p_T_gc_d, io_state->p_P_matrix_depth());
		io_state->compute_P_matrix(p_intrin_rgb, p_T_gc_rgb, io_state->p_P_matrix_RGB());	
	}

	return flag_valid;
	
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::gls_Generate_Linear_System_ICP_for_KinFu(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Tracking_State *in_state,
	CKvMatrixFloat *in_hmat_t1_to_t0_estimated,
	CKvMatrixFloat *out_ATA,
	CKvVectorFloat *out_ATb,
	int in_level_of_pyramid,
	float in_th_dist_ICP,
	bool in_param_mode_forward,
	bool in_mode_on_rgb)
//********************************************************************************************
{
	CKvPixel t_pix;
	CKvYooji_Intrinsics *p_intrins;
	CKvSet2d_of_Point3Df *p_surf_normal_model, *p_p3d_model;
	float *p_depth_map_view_float;
	float *p_depth_map_model;
	float A[6], b;
	float *p_ATA, *p_ATb;

	bool flag_valid;
	int ww, hh, cnt, x, y;
	
	// get pointers.
	p_ATA=out_ATA->vp();		p_ATb=out_ATb->vp();
	for(int j=0; j<6; j++){
		A[j]=p_ATb[j]=0.0f;
		for(int i=0; i<6; i++){
			p_ATA[j*6+i]=0.0f;
		}
	}

	//d_dmf_Display_Matrix_Float(in_view->p_pyramid_map_depth()->imgs[in_level_of_pyramid], 100.f, 0.0f);
	//d_dmf_Display_Matrix_Float(in_state->p_pyramid_map_depth()->imgs[in_level_of_pyramid], 100.f, 0.0f);

	p_depth_map_view_float = in_view->p_pyramid_map_depth()->imgs[in_level_of_pyramid].mps(ww, hh)[0];

	if(KV_FLAG_FRAME_TO_MODEL_DEPTH){
		p_depth_map_model = in_state->p_pyramid_map_depth_rendered()->imgs[in_level_of_pyramid].vp();
		p_p3d_model = &in_state->p_pyramid_map_p3d_rendered()->imgs[in_level_of_pyramid];
		p_surf_normal_model = &in_state->p_pyramid_map_normals_rendered()->imgs[in_level_of_pyramid];
	}	
	else{
		p_depth_map_model = in_state->p_pyramid_map_depth()->imgs[in_level_of_pyramid].vp();
		p_p3d_model = &in_state->p_pyramid_map_p3d()->imgs[in_level_of_pyramid];
		p_surf_normal_model = &in_state->p_pyramid_map_normals()->imgs[in_level_of_pyramid];
	}

	p_intrins = &in_view->p_pyramid_intrinsics()->intrins[in_level_of_pyramid];	
	//////////////////////////////////////////////////////////////////////////
	// 여기가 문제였다!!!!!
	// p_extrinsics_glob_to_cam_RGB 가 p_extrinsics_glob_to_cam 로 되어있었다.
	if(!in_mode_on_rgb) zz_transform_glob_to_cam.copy(in_state->p_extrinsics_glob_to_cam());
	else			    zz_transform_glob_to_cam.copy(in_state->p_extrinsics_glob_to_cam_RGB());	
	//////////////////////////////////////////////////////////////////////////

	d_pm_Printf_Matrix(in_state->p_extrinsics_glob_to_cam_RGB()->mp_transform()->vp(), 4, 4, "T_gc");
	
// 	float fx, fy, px, py, dmin, dmax;
// 	zz_intrinsics.gp_Get_Parameters(fx, fy, px, py, dmin, dmax);
	//if(!Kv_Printf("%f %f %f %f", fx, fy, px, py))	exit(0);
	
	cnt=0;
	for(y=0; y<hh; y++){
		for(x=0; x<ww; x++){
			t_pix.x=x;	t_pix.y=y;

			float td = p_depth_map_view_float[y*ww+x];

			if(td <= 1.0e-7f) continue;

			// computes a single summand.
			flag_valid=z_ss_Single_Summand_ICP_Backward(t_pix,
			td,
			/// make structure for render map...?
			p_depth_map_model,
			//////////////////////////////////////////////////////////////////////////
			p_p3d_model,
			//////////////////////////////////////////////////////////////////////////
			p_surf_normal_model,//in_state->prsn_Pointer_of_Rendered_Surface_Normals(),
			ww,hh,
			/// //////////////////////////////
			p_intrins,
			in_hmat_t1_to_t0_estimated,
			&zz_transform_glob_to_cam,
			in_th_dist_ICP,
			A,
			b,
			in_param_mode_forward);
		
 			
			// update ATA and ATb.
			if(flag_valid){
				z_uls_Update_Linear_System(A, b, p_ATA, p_ATb);
				cnt++;
			}
		}
	}
	
	return cnt;
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::gls_Generate_Linear_System_ICP_for_DVO(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Tracking_State *in_state,
	CKvMatrixFloat *in_hmat_t1_to_t0_estimated,
	CKvMatrixFloat *out_ATA,
	CKvVectorFloat *out_ATb,
	int in_level_of_pyramid,
	float in_th_dist_ICP,
	bool in_param_mode_forward,
	bool in_mode_on_rgb)
//********************************************************************************************
{

	CKvPixel t_pix;
	CKvYooji_Intrinsics *p_intrins;
	CKvSet2d_of_Point3Df *p_surf_normal_model, *p_p3d_model;
	float *p_depth_map_view_float;
	float *p_depth_map_model;
	float A[6], b;
	float *p_ATA, *p_ATb;

	float total_diff = 0.0f;

	bool flag_valid;
	int ww, hh, cnt, x, y;
	
	// get pointers.
	p_ATA=out_ATA->vp();		p_ATb=out_ATb->vp();
	for(int j=0; j<6; j++){
		A[j]=p_ATb[j]=0.0f;
		for(int i=0; i<6; i++){
			p_ATA[j*6+i]=0.0f;
		}
	}

	//d_dmf_Display_Matrix_Float(in_view->p_pyramid_map_depth()->imgs[in_level_of_pyramid], 100.f, 0.0f);
	//d_dmf_Display_Matrix_Float(in_state->p_pyramid_map_depth()->imgs[in_level_of_pyramid], 100.f, 0.0f);

	p_depth_map_view_float = in_view->p_pyramid_map_depth()->imgs[in_level_of_pyramid].mps(ww, hh)[0];

	p_depth_map_model = in_state->p_pyramid_map_depth_rendered()->imgs[in_level_of_pyramid].vp();
	p_p3d_model = &in_state->p_pyramid_map_p3d_rendered()->imgs[in_level_of_pyramid];
	p_surf_normal_model = &in_state->p_pyramid_map_normals_rendered()->imgs[in_level_of_pyramid];

// 	p_depth_map_model = in_state->p_pyramid_map_depth()->imgs[in_level_of_pyramid].vp();
// 	p_p3d_model = &in_state->p_pyramid_map_p3d()->imgs[in_level_of_pyramid];
// 	p_surf_normal_model = &in_state->p_pyramid_map_normals()->imgs[in_level_of_pyramid];

	p_intrins = &in_view->p_pyramid_intrinsics()->intrins[in_level_of_pyramid];	
	//////////////////////////////////////////////////////////////////////////
	// 여기가 문제였다!!!!!
	// p_extrinsics_glob_to_cam_RGB 가 p_extrinsics_glob_to_cam 로 되어있었다.
	if(!in_mode_on_rgb) zz_transform_glob_to_cam.copy(in_state->p_extrinsics_glob_to_cam());
	else			    zz_transform_glob_to_cam.copy(in_state->p_extrinsics_glob_to_cam_RGB());	
	//////////////////////////////////////////////////////////////////////////
	
// 	float fx, fy, px, py, dmin, dmax;
// 	zz_intrinsics.gp_Get_Parameters(fx, fy, px, py, dmin, dmax);
	//if(!Kv_Printf("%f %f %f %f", fx, fy, px, py))	exit(0);
	
	cnt=0;
	for(y=0; y<hh; y++){
		for(x=0; x<ww; x++){
			t_pix.x=x;	t_pix.y=y;

			float td = p_depth_map_view_float[y*ww+x];

			if(td <= 1.0e-7f) continue;

			// computes a single summand.
			flag_valid=z_ss_Single_Summand_ICP_Backward(
				t_pix,
				td,
				/// make structure for render map...?
				p_depth_map_model,
				//////////////////////////////////////////////////////////////////////////
				p_p3d_model,
				//////////////////////////////////////////////////////////////////////////
				p_surf_normal_model,//in_state->prsn_Pointer_of_Rendered_Surface_Normals(),
				ww,hh,
				/// //////////////////////////////
				p_intrins,
				in_hmat_t1_to_t0_estimated,
				&zz_transform_glob_to_cam,
				in_th_dist_ICP,
				A,
				b,
				in_param_mode_forward);			
 			
			// update ATA and ATb.
			if(flag_valid){
				z_uls_Update_Linear_System(A, b, p_ATA, p_ATb);
				cnt++;

				total_diff += b;
			}
		}
	}
	
	return cnt;
}

//********************************************************************************************
bool LCKvYooji_Camera_Tracker::tccadvo_Track_Camera_using_Confidence_Adaptive_DVO(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Scanner_Parameter *in_params,
	CKvYooji_Tracking_State *io_state,
	CKvMatrixFloat *in_pose_init)
//********************************************************************************************
{
	CKvMatrixFloat *cp_T_t0_to_t1_est = &zz_hmat_t0_to_t1_estimated;
	CKvMatrixFloat *cp_T_t1_to_t0_est = &zz_hmat_t1_to_t0_estimated;
	 
	 
	CKvYooji_Extrinsics T_t0_to_t1;
	 
	CKvYooji_Extrinsics *cp_T_gc_rgb = io_state->p_extrinsics_glob_to_cam_RGB();
	CKvYooji_Extrinsics *cp_T_gc_d = io_state->p_extrinsics_glob_to_cam();
	CKvYooji_Extrinsics *cp_T_rgb_d = in_view->p_extrinsics_RGB_to_depth();

	static int iter_num[4] ={10,7,7,7};
	static float th_num[4] ={SQUARE(50),SQUARE(30),SQUARE(10),SQUARE(10)};

	static float cov_diag[6] = {1.,1.,1.,1.,1.,1.,}; 
	static float prior_iter[6];		for(int i=0; i<6; i++) zz_sol_x_6.vp()[i] = 0.0f;

	int num_valid,num_level,num_th;
	bool flag_valid = true;
	float th_dist_ICP,mu,max_w;

	//////////////////////////////////////////////////////////////////////////
	// Joint optimization weight.
	float lambda_sq = SQUARE(KV_CUBE_SIZE/255.0f);
	//////////////////////////////////////////////////////////////////////////


	// initialize inverse of estimated pose.
	zz_transform_glob_to_cam.copy(io_state->p_extrinsics_glob_to_cam_RGB());

	//zz_transform_glob_to_cam.print_transform("zz_transform_glob_to_cam");	

	zz_hmat_cam_to_glob_estimated.cp_Copy(zz_transform_glob_to_cam.mp_transform_inv());
	in_params->gpicp_Get_Parameters_for_ICP_algorithm(num_level,th_dist_ICP,mu,max_w);

	//////////////////////////////////////////////////////////////////////////
	// initialize pose using spatial keypoints.
	//LCKvImageMatcher imatcher;	imatcher.i_Initialize();
	//if(0){
	if(in_pose_init){
	  
	  	num_level = 1;
	  
	  	// frame-to-frame
	  	cp_T_t0_to_t1_est->cp_Copy(in_pose_init);
	  	// frame-to-model
		d_im_Inverse_Matrix_4x4(in_pose_init->vp(), cp_T_t1_to_t0_est->vp());
	}
	else{
	 	//////////////////////////////////////////////////////////////////////////
		cp_T_t0_to_t1_est->ci_Create_Identity_matrix(4);
		cp_T_t1_to_t0_est->ci_Create_Identity_matrix(4);
	 	//////////////////////////////////////////////////////////////////////////
	}

	//////////////////////////////////////////////////////////////////////////
	num_th = 20;
	//////////////////////////////////////////////////////////////////////////
	//for(int k=-1; k>=0; k--){
	for(int k=num_level-1; k>=0; k--){

		CKvSet2d_of_Pointf *cp_map_reproj = &zz_map_reproj[k];
		CKvSet2d_of_Point3Df *cp_map_p3d = &zz_map_p3d[k];
		CKvSet2d_of_Point3Df *cp_map_norm = &zz_map_norm[k];

		CKvMatrixFloat *cp_map_weight = &zz_map_weight[k];
		CKvMatrixFloat *cp_map_diff_pm = &zz_map_diff_pm[k];
		CKvMatrixFloat *cp_map_diff_gradm = &zz_map_diff_gradm[k];
		CKvSet2d_of_VectorFloat *cp_map_diff_grad = &zz_map_diff_grad[k];		
		 
		CKvMatrixBool *cp_map_valid_grad = &zz_map_valid_gradient[k];
		CKvMatrixBool *cp_map_valid_pm = &zz_map_valid_photomax[k];
		CKvMatrixBool *cp_map_valid_icp = &zz_map_valid_icp[k];

		CKvSet2d_of_VectorFloat *cp_map_jacobi_photomax = &zz_map_jacobian_photomax[k];		
		CKvSet2d_of_MatrixFloat *cp_map_jacobi_grad = &zz_map_jacobian_grad[k];

		CKvSet2d_of_MatrixFloat *cp_mat_jacobi_pit = &zz_map_jacobian_Pi_T[k];

		//////////////////////////////////////////////////////////////////////////
		// Ver.1. Uniform weight 버전에서는 validity 계산은 일단 여기서 끝내는 것으로 한다!
		// Ver.2. 초반 validity는 t0 frame의 depth 및 normal 정보로만 check 한다.
		// initialize validity map.
		//num_valid = ivm_Initialize_Validity_Map(in_view, io_state, k, cp_map_valid_icp);
		//cp_map_valid_pm->cp_Copy(cp_map_valid_icp);
		//cp_map_valid_grad->cp_Copy(cp_map_valid_icp);

		num_valid = ivm_Initialize_Validity_Maps(in_view, io_state, k, cp_map_valid_pm, cp_map_valid_grad, cp_map_valid_icp);

		//printf("valid pixels: %d\n", num_valid);
		//////////////////////////////////////////////////////////////////////////

		// compute jacobians.
 		//cjpt_Compute_Jacobian_Projection_and_Transformation(in_view,io_state,cp_map_valid_pm,k,cp_mat_jacobi_pit);
 		//cjpih_Compute_Jacobian_Plane_Induced_Homography(in_view,io_state,cp_map_valid_grad,k,cp_mat_jacobi_pih);

		// iteration.
		for(int i=0; i<iter_num[k]; i++)
		{
			CKvMatrixBool *cp_map_valid_curr_grad = &zz_map_valid_current_grad[k];
			CKvMatrixBool *cp_map_valid_curr_pm = &zz_map_valid_current_photo[k];
			CKvMatrixBool *cp_map_valid_curr_icp = &zz_map_valid_current_icp[k];

			cp_map_valid_curr_grad->cp_Copy(cp_map_valid_grad);
			cp_map_valid_curr_pm->cp_Copy(cp_map_valid_pm);
			cp_map_valid_curr_icp->cp_Copy(cp_map_valid_icp);

			float diff_scale[2];
			int num_valid[3];

			if(KV_FLAG_RGB_COST || KV_FLAG_GRAD_COST){
				//////////////////////////////////////////////////////////////////////////
				// Compute 3D point map.
				//////////////////////////////////////////////////////////////////////////
				cp3dm_Compute_Point3D_Map(io_state,cp_T_t0_to_t1_est,cp_map_valid_curr_icp,k,cp_map_p3d);
			 
				//////////////////////////////////////////////////////////////////////////
				// Compute reprojected point map.
				//////////////////////////////////////////////////////////////////////////
				crp2dm_Compute_Reprojected_Point2D_Map(in_view,cp_map_p3d,cp_map_valid_curr_icp,k,cp_map_reproj);
			 
				//////////////////////////////////////////////////////////////////////////
				// Compute Jacobian map.
				//////////////////////////////////////////////////////////////////////////
 				cjpti_Compute_Jacobian_Projection_and_Transformation_Iterative(in_view,io_state,cp_map_valid_curr_pm,cp_map_p3d,k,cp_mat_jacobi_pit);
				//////////////////////////////////////////////////////////////////////////
			}

			//////////////////////////////////////////////////////////////////////////
			// 1. Photoconsistency Maximization.
			//////////////////////////////////////////////////////////////////////////
			if(KV_FLAG_RGB_COST){
				// Compute jacobians.
				num_valid[0] = cjpm_Compute_Jacobian_PhotoMax(in_view,io_state,cp_map_reproj,cp_mat_jacobi_pit,k,
					cp_map_valid_curr_pm,cp_map_jacobi_photomax);	
				// Compute residual map.
				diff_scale[0] = cidm_Compute_Intensity_Difference_Map(in_view,io_state,cp_map_valid_curr_pm,cp_map_reproj,k,cp_map_diff_pm);

				
				// Generate least square system.
				grhlse_Generate_Right_Hands_of_Least_Square_Equation(
					in_view,io_state,cp_map_jacobi_photomax,cp_map_diff_pm,cp_map_valid_curr_pm,k,&zz_ATb_6[0]);
				// compute JTJ matrices.
				glhlse_Generate_Left_Hands_of_Least_Square_Equation(
					in_view,io_state,cp_map_jacobi_photomax,cp_map_valid_curr_pm,k,&zz_ATA_6x6[0]);
			}
			
			//////////////////////////////////////////////////////////////////////////
			// 2. Gradient Maximization.
			//////////////////////////////////////////////////////////////////////////
			if(KV_FLAG_GRAD_COST){
				// gradient mag.
				// Compute jacobians.
				num_valid[1] = cjgm_Compute_Jacobian_Gradinet_Magnitude(in_view,io_state,cp_map_reproj,cp_mat_jacobi_pit,k,
					cp_map_valid_curr_grad,cp_map_jacobi_photomax);
				// Compute residual map.
				diff_scale[1] = cgmdm_Compute_Gradient_Magnitude_Difference_Map(in_view,io_state,
					cp_map_valid_curr_grad,cp_map_reproj,k,cp_map_diff_gradm);

				// full gradient vector.
// 				num_valid[1] = cjgm_Compute_Jacobian_Gradient_based_Method_New(in_view,io_state,cp_map_reproj,cp_mat_jacobi_pit,NULL,k,
// 					cp_map_valid_curr_grad,cp_map_jacobi_grad);
// 				// Compute residual map.
// 				cgdm_Compute_Gradient_Difference_Map_New(in_view,io_state,cp_map_valid_curr_grad,cp_map_reproj,cp_T_t0_to_t1_est,k,cp_map_diff_grad);

				// defense.
// 				grhlse_Generate_Right_Hands_of_Least_Square_Equation(
// 					in_view,io_state,cp_map_jacobi_grad,cp_map_diff_grad,cp_map_valid_curr_grad,k,&zz_ATb_6[1]);
// 				// compute JTJ matrices.
// 				glhlse_Generate_Left_Hands_of_Least_Square_Equation(
// 					in_view,io_state,cp_map_jacobi_grad,cp_map_valid_curr_grad,k,&zz_ATA_6x6[1]);

				grhlse_Generate_Right_Hands_of_Least_Square_Equation(
					in_view,io_state,cp_map_jacobi_photomax,
					cp_map_diff_gradm,cp_map_valid_curr_grad,
					k,&zz_ATb_6[1]);
				// compute JTJ matrices.
				glhlse_Generate_Left_Hands_of_Least_Square_Equation(
					in_view,io_state,cp_map_jacobi_photomax,
					cp_map_valid_curr_grad,
					k,&zz_ATA_6x6[1]);
			}
			//printf("num_valid[#%d lv.]: %d %d\n", k, num_valid[0], num_valid[1]);
						
			//////////////////////////////////////////////////////////////////////////
			// 3. Point-to-Plane distance minimization.
			//////////////////////////////////////////////////////////////////////////
			if(KV_FLAG_DEPTH_COST){
				num_valid[2] = gls_Generate_Linear_System_ICP_for_DVO(
					in_view,
					io_state,
					cp_T_t1_to_t0_est,
					&zz_ATA_6x6[2],
					&zz_ATb_6[2],
					k,
					th_dist_ICP,
					true,
					true);

				// check depth validity.
				//if(num_valid[2] < th_num[k]){
				//	if(k>0){
// 						/////////////////////////////////////////////////////////////////////////
// 						// re-initialize motion estimate.
// 						cp_T_t0_to_t1_est->ci_Create_Identity_matrix(4);
// 						cp_T_t1_to_t0_est->ci_Create_Identity_matrix(4);
// 						//////////////////////////////////////////////////////////////////////////
// 						break;	// go to next level!
// 					} 
// 					else return false;
// 				}
			}
			//
			
		
			// Weighted linear system for joint optimization.
			float s1, s2, s2_sq;
			//s1 = 0.0f; s2 = 1.0f;
			//s1 = 1.0f; s2 = 0.0f;
			// using total residual sum.
			//s1 = 1.0f/abs(diff_scale[0]);	s2 = 1.0f/abs(diff_scale[1]);
			// using range difference.
			// depth range = (0, cube size), intensity range = (0, 255)
			//s1 = 1.0f/255.0f;	s2 = 100.0f/KV_CUBE_SIZE;

			//printf("Residual scale: %f %f\n", diff_scale[0], diff_scale[1]);
			// Photomax + ICP.
// 			d_avw_Add_Vector_Weighted(zz_ATA_6x6[0].vp(),zz_ATA_6x6[2].vp(),s1,s2,6*6,zz_ATA_6x6[3].vp());			
// 			d_avw_Add_Vector_Weighted(zz_ATb_6[0].vp(),zz_ATb_6[2].vp(),s1,s2,6,zz_ATb_6[3].vp());

			//////////////////////////////////////////////////////////////////////////
			// Construct linear equation.
			//////////////////////////////////////////////////////////////////////////
			// Photomax + Gradmax.
			if(KV_FLAG_RGB_COST || KV_FLAG_GRAD_COST){
				s1 = KV_FLAG_RGB_COST ? lambda_sq : 0.0f;
				s2 = KV_FLAG_GRAD_COST ? lambda_sq : 0.0f;

				d_avw_Add_Vector_Weighted(zz_ATA_6x6[0].vp(),zz_ATA_6x6[1].vp(),s1,s2,6*6,zz_ATA_6x6[3].vp());
				d_avw_Add_Vector_Weighted(zz_ATb_6[0].vp(),zz_ATb_6[1].vp(),s1,s2,6,zz_ATb_6[3].vp());
			}

			// ICP + Photo.
			if(KV_FLAG_DEPTH_COST){

				//s1 = sqrt(0.1f); s2 = 1.0f;		// Kintinuous.
				//s1 = 1.0f;	s2 = 255.0f*100.0f/KV_CUBE_SIZE; s3 = sqrt(s2);
				//s1 = 0.0f;	s2 = 1.0f;
				s1 = (KV_FLAG_DEPTH_COST) ? 1.0f : 0.0f;
				s2 = (KV_FLAG_RGB_COST || KV_FLAG_GRAD_COST) ? 1.0f : 0.0f;
	
				d_avw_Add_Vector_Weighted(zz_ATA_6x6[2].vp(),zz_ATA_6x6[3].vp(),s1,s2,6*6,zz_ATA_6x6[0].vp());
				d_avw_Add_Vector_Weighted(zz_ATb_6[2].vp(),zz_ATb_6[3].vp(),s1,s2,6,zz_ATb_6[0].vp());
			}


// 			if(num_valid > num_th)
// 			{
				// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
				// TEST own linear system solver using LLD.
				float L[6*6],y[6];
				float *p_ATA = KV_FLAG_DEPTH_COST ? zz_ATA_6x6[0].vp() : zz_ATA_6x6[3].vp();
				float *p_ATb = KV_FLAG_DEPTH_COST ? zz_ATb_6[0].vp() : zz_ATb_6[3].vp();

// 				d_pm_Printf_Matrix(p_ATA,6,6,"ATA");
// 				d_pm_Printf_Matrix(p_ATb,1,6,"ATb");
				
				//////////////////////////////////////////////////////////////////////////
				// Temporal smoothing using motion prior.
				if(KV_FLAG_MOTION_SMOOTH_TEMPORAL){
					//////////////////////////////////////////////////////////////////////////
					// How to set alpha?
					float alpha = 0.01f;
					d_avw_Add_Vector_Weighted(zz_prior_x_6.vp(), zz_sol_x_6.vp(), 1.0f, -1.0f, 6, prior_iter);
					//////////////////////////////////////////////////////////////////////////
					d_avw_Add_Vector_Weighted(p_ATA,zz_eye_6x6.vp(),1.0f,alpha,6*6,zz_ATA_6x6[1].vp());
					d_avw_Add_Vector_Weighted(p_ATb,prior_iter,1.0f,alpha,6,zz_ATb_6[1].vp());
					// re-pointing.
					p_ATA = zz_ATA_6x6[1].vp();
					p_ATb = zz_ATb_6[1].vp();

				}
				/////////////////////////////////////////////////////////////////////////


				d_lld_LL_Decomposition(p_ATA,6,L);
				d_sls_Solve_Linear_System_using_LLD(L,p_ATb,y,6,zz_sol_x_6.vp());
				// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

				// the number of overlapped surface points.	
				// 				flag_valid=sls_Solve_Linear_System(&zz_ATA_6x6,
				// 					&zz_ATb_6,
				// 					&zz_sol_x_6);	

// 				if(flag_valid)
// 				{
					// 					printf("x: %f %f %f | %f %f %f\n"
					// 						, zz_sol_x_6.vp()[0], zz_sol_x_6.vp()[1], zz_sol_x_6.vp()[2]
					// 						, zz_sol_x_6.vp()[3], zz_sol_x_6.vp()[4], zz_sol_x_6.vp()[5]);
					//printf("[%d] norm2: %f (%d valids)\n",k,zz_sol_x_6.n2_Norm_Two(),num_valid);

				//////////////////////////////////////////////////////////////////////////
				// check residual map.
// 				CKvMatrixFloat map_diff_mag;
// 				map_diff_mag.cp_Copy(cp_map_diff_pm);
// 				for(int kk = 0; kk<map_diff_mag.vs(); kk++) 
// 					map_diff_mag.vp()[kk] = abs(map_diff_mag.vp()[kk]);
// 				sc[0].s_d_Display(10.0f, 0.0f, &map_diff_mag);
// 				//sc[1].s_d_Display(20.0f, 128.0f, cp_map_diff_gradm);
// 				//////////////////////////////////////////////////////////////////////////
// 				// k 값의 갑자기 점핑!
// 				//////////////////////////////////////////////////////////////////////////
// 				if(!Kv_Printf("[#%d lv] residual! %f %f", 
// 					k,
// 					diff_scale[0], diff_scale[1]))	exit(0);
// 				//////////////////////////////////////////////////////////////////////////

				// stop iteration if estimated pose was converged.
				//if(zz_sol_x_6.n2_Norm_Two()<6.0e-4)	break;
				float norm_inf;	d_niv_Norm_Infinity_Vector(zz_sol_x_6.vp(),6,norm_inf);
				if(norm_inf < 5.0e-7f) break;

				//////////////////////////////////////////////////////////////////////////
				// 여기가 조금 이상한듯.
				// 매트릭스 인버스를 구해서 해줘야하나 backward warping update를...ㅔ
				// for frame-to-frame color tracking using forward warping.
				//uitme_Update_Incremental_Tracking_with_Matrix_Exponential(&zz_sol_x_6,cp_T_t0_to_t1_est);
				umf_Update_Motion_Forward(&zz_sol_x_6,cp_T_t0_to_t1_est);					
				d_im_Inverse_Matrix_4x4(cp_T_t0_to_t1_est->vp(),cp_T_t1_to_t0_est->vp());


				//uit_Update_Incremental_Tracking_Yooji(&zz_sol_x_6, cp_T_t0_to_t1_est);				
					
				//uitcg_Update_Incremental_Tracking_t1_to_t0_with_Forward_Parameters(&zz_sol_x_6, cp_T_t1_to_t0_est);
				//d_im_Inverse_Matrix_4x4(cp_T_t1_to_t0_est->vp(), cp_T_t0_to_t1_est->vp());
					
				//uitcg_Update_Incremental_Tracking_t1_to_t0_with_Forward_Parameters(&zz_sol_x_6, cp_T_t1_to_t0_est);			
				//////////////////////////////////////////////////////////////////////////
	
  				if(!cmv_Check_Motion_Validity(cp_T_t0_to_t1_est,MAX_TRANSLATION,MAX_ROTATION)){
  					if(k==0){
						//cp_T_t0_to_t1_est->cp_Copy(&zz_prior_4x4);	
						return false;	
					}
  					else{

						//////////////////////////////////////////////////////////////////////////
						// re-initialize motion estimate.
						cp_T_t0_to_t1_est->ci_Create_Identity_matrix(4);
						cp_T_t1_to_t0_est->ci_Create_Identity_matrix(4);
						//////////////////////////////////////////////////////////////////////////
						break;
					}
  				}
				//} else	return false;

			//} else flag_valid = false;
			
		}

		// update number of iteration.
		//num_th *= 4;

	}

	//if(!Kv_Printf("tracking complete!")) exit(0);

	//d_pm_Printf_Matrix(zz_hmat_cam_to_glob_estimated.vp(), 4, 4, "Hmat");

// 	if(flag_valid)
// 	{
// 		CKvYooji_Intrinsics *p_intrin_d = io_state->p_intrinsics_depth();
// 		CKvYooji_Intrinsics *p_intrin_rgb = io_state->p_intrinsics_depth();
// 		CKvYooji_Extrinsics *p_T_gc_d = io_state->p_extrinsics_glob_to_cam();
// 		CKvYooji_Extrinsics *p_T_gc_rgb = io_state->p_extrinsics_glob_to_cam_RGB();
// 
// 		CKvYooji_Extrinsics *p_T_drgb = in_view->p_extrinsics_depth_to_RGB();
// 		CKvYooji_Extrinsics *p_T_rgbd = in_view->p_extrinsics_RGB_to_depth();
// 
// 		// 		p_T_drgb->print_transform("p_T_drgb");
// 		// 		p_T_rgbd->print_transform("p_T_rgbd");
// 
// 		//////////////////////////////////////////////////////////////////////////
// 		// 3D rigid transformation.
// 		//zz_transform_glob_to_cam.set_from_transform_inv(&zz_hmat_cam_to_glob_estimated);
// 
// 		// pose of RGB camera.
// 		p_T_gc_rgb->set_from_transform_inv(cp_T_cg_est);
// 		// pose of depth camera.
// 		p_T_rgbd->get_pose_successive(*p_T_gc_rgb,*p_T_rgbd,*p_T_gc_d);
// 		
// 
// 		io_state->compute_P_matrix(p_intrin_d,p_T_gc_d,io_state->p_P_matrix_depth());
// 		io_state->compute_P_matrix(p_intrin_rgb,p_T_gc_rgb,io_state->p_P_matrix_RGB());

		// update motion prior.
	 	Eigen::Matrix4f mat, mat_log;
		float *p_T = cp_T_t0_to_t1_est->vp();
	 
	 	mat << p_T[0],p_T[1],p_T[2],p_T[3],
	 			p_T[4],p_T[5],p_T[6],p_T[7],
	 			p_T[8],p_T[9],p_T[10],p_T[11],
	 			p_T[12],p_T[13],p_T[14],p_T[15];

		mat_log = mat.log();

		//cout << mat_log << endl;
	 
		float *p_prior = zz_prior_x_6.vp();

		p_prior[0] = -mat_log.coeff(1,2);
		p_prior[1] = mat_log.coeff(0,2);
		p_prior[2] = -mat_log.coeff(0,1);
		p_prior[3] = mat_log.coeff(0,3);
		p_prior[4] = mat_log.coeff(1,3);
		p_prior[5] = mat_log.coeff(2,3);

		d_pm_Printf_Matrix(zz_prior_x_6.vp(), 1, 6, "Prior");

		zz_prior_4x4.cp_Copy(cp_T_t0_to_t1_est);


		// Compute new 3D rigid transformation.
		// + for RGB camera.	
		d_mmm_Multiply_Matrix_Matrix(cp_T_t0_to_t1_est->vp(),cp_T_gc_rgb->mp_transform()->vp(),
			4,4,4,zz_mat_4x4.vp());

		cp_T_gc_rgb->set_from_transform(&zz_mat_4x4);
		//cp_T_gc_rgb->print_transform();

		// + for depth camera.
		zz_transform_glob_to_cam_rgb.get_pose_successive(
			*cp_T_gc_rgb,//zz_transform_glob_to_cam_rgb,
			*cp_T_rgb_d,
			*cp_T_gc_d);
		//io_state->pt3d_Pointer_of_3D_Transform_Global_to_Camera()->cp_Copy(&zz_transform_glob_to_cam);
		// + P matrix depth.
		io_state->compute_P_matrix(
			&io_state->p_pyramid_intrinsics()->intrins[0],
			cp_T_gc_d,//io_state->pt3d_Pointer_of_3D_Transform_Global_to_Camera(),
			io_state->p_P_matrix_depth());
		// + P matrix rgb.
		io_state->compute_P_matrix(
			&io_state->p_pyramid_intrinsics()->intrins[0],
			cp_T_gc_rgb,//io_state->pt3dr_Pointer_of_3D_Transform_Global_to_Camera_RGB(),
			io_state->p_P_matrix_RGB());
//	}

	return flag_valid;

}

//********************************************************************************************
bool LCKvYooji_Camera_Tracker::tcpm_Track_Camera_with_Photoconsist_Max_Yooji_Weighted(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Scanner_Parameter *in_params,
	CKvYooji_Tracking_State *io_state)
//********************************************************************************************
{
	CKvScreen sc[2];	

	CKvMatrixFloat *cp_T_t0_to_t1_est = &zz_hmat_t0_to_t1_estimated;

	CKvYooji_Extrinsics T_t0_to_t1;

	CKvYooji_Extrinsics *cp_T_gc_rgb = io_state->p_extrinsics_glob_to_cam_RGB();
	CKvYooji_Extrinsics *cp_T_gc_d = io_state->p_extrinsics_glob_to_cam();
	CKvYooji_Extrinsics *cp_T_rgb_d = in_view->p_extrinsics_RGB_to_depth();

	vector<CKvMatrixFloat> *cp_pyr_img_d;
	vector<CKvYooji_Intrinsics> *cp_pyr_intrin;

	int num_iter[4] ={100,70,70,70};
	int sz_window[4] ={9,7,5,3};
	int num_valid,num_level,num_th;
	bool flag_valid = true;

	// initialize inverse of estimated pose.
	cp_T_t0_to_t1_est->ci_Create_Identity_matrix(4);
	// initialize inverse of estimated pose.
	// 	zz_transform_glob_to_cam.cp_Copy(io_state->pt3d_Pointer_of_3D_Transform_Global_to_Camera());
	// 	zz_hmat_cam_to_glob_estimated.cp_Copy(zz_transform_glob_to_cam.mpi_Matrix_Pointer_Inverse());

	//p_pyram_img_depth = in_view->ppdm_Pointer_of_Pyramid_Depth_Map();
	//p_pyram_intrins = in_view->ppid_Pointer_of_Pyramid_Intrinsics_Depth();

	num_level = in_view->p_pyramid_image_gray()->levOfScale;

	num_th = 300;

	for(int k=num_level-1; k>=0; k--){

		CKvSet2d_of_Pointf *cp_map_reproj = &zz_map_reproj[k];
		CKvSet2d_of_Point3Df *cp_map_p3d = &zz_map_p3d[k];
		CKvMatrixFloat *cp_map_diff = &zz_map_diff_pm[k];
		CKvMatrixFloat *cp_map_weight = &zz_map_weight[k];
		CKvSet2d_of_VectorFloat *cp_map_jacobi = &zz_map_jacobian_photomax[k];
		CKvMatrixBool *cp_map_valid = &zz_map_valid_photomax[k];

		cvm_Compute_Validity_Map_for_PhotoMax(in_view, io_state, k, cp_map_valid);
	//	cjpm_Compute_Jacobian_PhotoMax(in_view, io_state, cp_map_valid, cp_map_reproj, k, cp_map_jacobi);

		// iteration.
		for(int i = 0; i<num_iter[k]; i++)
		{
			//start=clock();
			//num_valid = glsitpm_Generate_Linear_System_for_Incremental_Tracking_with_Photoconsist_Max_Steinbrucker(
			//////////////////////////////////////////////////////////////////////////
			// Compute 3D point map.
			//////////////////////////////////////////////////////////////////////////
			cp3dm_Compute_Point3D_Map(io_state,cp_T_t0_to_t1_est,cp_map_valid,k,cp_map_p3d);
			//printf("cp3dm_Compute_Point3D_Map\n");

			//////////////////////////////////////////////////////////////////////////
			// Compute reprojected point map.
			//////////////////////////////////////////////////////////////////////////
			crp2dm_Compute_Reprojected_Point2D_Map(in_view,cp_map_p3d,cp_map_valid,k,cp_map_reproj);

			//////////////////////////////////////////////////////////////////////////
			// Compute intensity difference map.
			//////////////////////////////////////////////////////////////////////////
			cidm_Compute_Intensity_Difference_Map(in_view,io_state,cp_map_valid,cp_map_reproj,k,cp_map_diff);
			//cidm_Compute_Intensity_Difference_Map(in_view,io_state,cp_map_p3d,k,cp_map_valid,cp_map_diff);

			//////////////////////////////////////////////////////////////////////////
			// Compute residual weight map.
			//////////////////////////////////////////////////////////////////////////
			cwmtd_Compute_Weight_Map_based_on_T_Distribution(cp_map_diff,cp_map_valid,cp_map_weight);

			if(k<=1 && i % 5 == 0){
				sc[0].s_d_Display(100.0f, 0.0f, cp_map_weight);
				sc[1].s_d_Display(10.0f, 0.0f, cp_map_diff);
				if(!Kv_Printf("Level: %d iter: %d", k, i)) exit(0);
			}
			//////////////////////////////////////////////////////////////////////////
			// Generate least square system.
			//////////////////////////////////////////////////////////////////////////
			// 1. Photoconsistency Maximization.
// 			num_valid = glsew_Generate_Least_Square_Equation_with_Photoconsist_Max_Yooji_Weighted(
// 				in_view,io_state,cp_map_p3d,cp_map_diff,cp_map_weight,cp_map_valid,k,&zz_ATA_6x6,&zz_ATb_6);
			num_valid = glsew_Generate_Least_Square_Equation_with_Photoconsist_Max_Yooji_Weighted(
				in_view,io_state,cp_map_jacobi,cp_map_diff,cp_map_weight,cp_map_valid,k,&zz_ATA_6x6[0],&zz_ATb_6[0]);


			// 			printf("[lev: %d] num_valid: %d\n", k, num_valid);
			//d_pm_Printf_Matrix(zz_ATb_6.vp(), 1, 6, "ATb");

			//if(!Kv_Printf("[lev: %d] num_valid: %d\n", k, num_valid)) exit(0);

			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			// TEST own linear system solver using LLD.
			float L[6*6],y[6];
			d_lld_LL_Decomposition(zz_ATA_6x6[0].vp(),6,L);
			if(!d_sls_Solve_Linear_System_using_LLD(L,zz_ATb_6[0].vp(),y,6,zz_sol_x_6.vp())) flag_valid = false;
			
			//d_pm_Printf_Matrix(zz_sol_x_6.vp(), 1, 6, "sol");
			
			float norm_inf;	d_niv_Norm_Infinity_Vector(zz_sol_x_6.vp(), 6, norm_inf);
			if(norm_inf < 5.0e-7f) break;
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


			umf_Update_Motion_Forward(
				&zz_sol_x_6,
				cp_T_t0_to_t1_est);

			//d_pm_Printf_Matrix(cp_T_t0_to_t1_est->vp(),4,4,"T");

			if(!cmv_Check_Motion_Validity(cp_T_t0_to_t1_est,MAX_TRANSLATION,MAX_ROTATION)) return false;

			//printf("cmv_Check_Motion_Validity\n");

			//flag_valid=false;	

		}

		//num_iter *= 2;
		//num_th *= 4;
	}


	//T_t0_to_t1.set_from_transform_inv(cp_T_t0_to_t1_est);		

	// Compute new 3D rigid transformation.
	// + for RGB camera.	
	d_mmm_Multiply_Matrix_Matrix(cp_T_t0_to_t1_est->vp(),cp_T_gc_rgb->mp_transform()->vp(),
		4,4,4,zz_mat_4x4.vp());

	cp_T_gc_rgb->set_from_transform(&zz_mat_4x4);
	//cp_T_gc_rgb->print_transform();

	// + for depth camera.
	zz_transform_glob_to_cam_rgb.get_pose_successive(
		*cp_T_gc_rgb,//zz_transform_glob_to_cam_rgb,
		*cp_T_rgb_d,
		*cp_T_gc_d);
	//io_state->pt3d_Pointer_of_3D_Transform_Global_to_Camera()->cp_Copy(&zz_transform_glob_to_cam);
	// + P matrix depth.
	io_state->compute_P_matrix(
		&io_state->p_pyramid_intrinsics()->intrins[0],
		cp_T_gc_d,//io_state->pt3d_Pointer_of_3D_Transform_Global_to_Camera(),
		io_state->p_P_matrix_depth());
	// + P matrix rgb.
	io_state->compute_P_matrix(
		&io_state->p_pyramid_intrinsics()->intrins[0],
		cp_T_gc_rgb,//io_state->pt3dr_Pointer_of_3D_Transform_Global_to_Camera_RGB(),
		io_state->p_P_matrix_RGB());
	// Texture image.
	//io_state->pti_Pointer_of_Texture_Image()->cp_Copy(in_view->prgbi_Pointer_of_RGB_Image());
	//p_hmat->i_Import(zz_transform_glob_to_cam.mp_Matrix_Pointer());


	return true;
	
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::glsew_Generate_Least_Square_Equation_with_Photoconsist_Max_Yooji_Weighted(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Tracking_State *in_state,
	CKvSet2d_of_Point3Df *in_map_p3d,
	CKvMatrixFloat *in_map_diff,
	CKvMatrixFloat *in_map_weight,
	CKvMatrixBool *in_map_valid,
	int in_level_of_pyramid,
	CKvMatrixFloat *out_ATA,
	CKvVectorFloat *out_ATb)
//********************************************************************************************
{
	CKvYooji_Intrinsics *p_intrin_rgb;
	CKvYooji_Extrinsics *p_pose_gc_rgb_t0;
	CKvPointf t_pix, t_pix_rgb;
	CKvPoint3Df *p_p3d;

	static const float th_grad_sq[4] = {SQUARE(1), SQUARE(3), SQUARE(5), SQUARE(12)};

	bool *p_sil_t1;
	unsigned char *p_img_gray_t0, *p_img_gray_t1;
	float *p_grad_x_t0, *p_grad_y_t0, grad[2];
	float *p_depth_t0;
	float *p_confidence_t0, *p_diff, *p_weight;
	bool *p_valid;

	float A[6], b;
	float *p_ATA, *p_ATb;

	float inten_t0, td_rgb_t0;
	bool flag_valid = false;
	int ww, hh, cnt, x, y;
	float fx, fy, px, py, dmin, dmax;
	
	// get pointers.
	p_ATA=out_ATA->vp();		p_ATb=out_ATb->vp();
	for(int j=0; j<6; j++){
		A[j]=p_ATb[j]=0.0f;
		for(int i=0; i<6; i++){
			p_ATA[j*6+i]=0.0f;
		}
	}
		
	p_img_gray_t0 = in_state->p_pyramid_image_gray()->imgs[in_level_of_pyramid].mps(ww, hh)[0];
	p_img_gray_t1 = in_view->p_pyramid_image_gray()->imgs[in_level_of_pyramid].vp();
	
	p_sil_t1 = in_state->p_pyramid_silhouette()->imgs[in_level_of_pyramid].vp();

// 	p_grad_x_t1 = in_view->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_x.vp();
//	p_grad_y_t1 = in_view->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_y.vp();
	
	p_grad_x_t0 = in_state->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_x.vp();
	p_grad_y_t0 = in_state->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_y.vp();
	p_confidence_t0 = in_state->p_pyramid_map_confidence()->imgs[in_level_of_pyramid].vp();

	p_p3d = in_map_p3d->vp();
	p_diff = in_map_diff->vp();
	p_weight = in_map_weight->vp();
	p_valid = in_map_valid->vp();

	p_intrin_rgb = &in_view->p_pyramid_intrinsics()->intrins[in_level_of_pyramid];	
 	p_intrin_rgb->get_params(fx, fy, px, py, dmin, dmax);

	cnt=0;
	for(y=0; y<hh; y++){
		for(x=0; x<ww; x++){

			int tidx = y*ww + x;
			
			if(!p_valid[tidx]) continue;
			
			t_pix.x=x;	t_pix.y=y;

			grad[0] = p_grad_x_t0[tidx];
			grad[1] = p_grad_y_t0[tidx];
			
			//////////////////////////////////////////////////////////////////////////
			if(SQUARE(grad[0]) + SQUARE(grad[1]) < th_grad_sq[in_level_of_pyramid]) continue;
			//////////////////////////////////////////////////////////////////////////

			// computes left hand.
// 			z_ccpm_Compute_Coefficients_with_Photo_Max_Yooji(
// 				&p_p3d[tidx], grad[0], grad[1], fx, fy, A);
			
			//////////////////////////////////////////////////////////////////////////
			// computes right hand.
			b = -p_diff[tidx];
			//////////////////////////////////////////////////////////////////////////	

  			//d_pm_Printf_Matrix(A, 1, 6, "A");
  			//if(b>1000) printf("b: %f\n", b);

			// update ATA and ATb.
			
			//float weight = p_confidence_t0[y*ww + x];
			float weight = p_weight[tidx];

			z_ulsw_Update_Linear_System_Weighted(A, b, weight, p_ATA, p_ATb);
			cnt++;

		}
	}

	//printf("[%d] cnt: %d\n", in_level_of_pyramid, cnt);
	
	return cnt;
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::glhlse_Generate_Left_Hands_of_Least_Square_Equation(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Tracking_State *in_state,
	CKvSet2d_of_VectorFloat *in_map_jacobi,
	CKvMatrixBool *in_map_valid,
	int in_level_of_pyramid,
	CKvMatrixFloat *out_ATA)
//********************************************************************************************
{
	CKvVectorFloat *cp_jacobi;

	bool *p_valid;

	float *p_ATA;
	float tATA[6*6];
	int ww, hh, cnt;
	
	// get pointers.
	p_ATA=out_ATA->vp();	for(int i=0; i<6*6; i++)	p_ATA[i]=0.0f;
	
	cp_jacobi = in_map_jacobi->mps(ww, hh)[0];
	p_valid = in_map_valid->vp();
	
	cnt=0;

	for(int tidx=0; tidx<ww*hh; tidx++){
			
		if(!p_valid[tidx]) continue;

		float *J = cp_jacobi[tidx].vp();
			
		// update ATA.
		d_cgm_Compute_Gram_Matrix(J, 1, 6, tATA);
		for(int i=0; i<6*6; i++){ p_ATA[i] += tATA[i]; } 

		//for(int j=0; j<6; j++){ for(int i=0; i<6; i++){ p_ATA[j*6+i]+=J[j]*J[i]; } }
		cnt++;

	}
	
	return cnt;
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::grhlse_Generate_Right_Hands_of_Least_Square_Equation(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Tracking_State *in_state,
	CKvSet2d_of_VectorFloat *in_map_jacobi,
	CKvMatrixFloat *in_map_diff,
	CKvMatrixBool *in_map_valid,
	int in_level_of_pyramid,
	CKvVectorFloat *out_ATb)
//********************************************************************************************
{
	CKvVectorFloat *cp_jacobi;
	float *p_diff;
	bool *p_valid;

	float *p_ATb;

	int ww, hh, cnt;
	
	// get pointers.
	p_ATb=out_ATb->vp();
	for(int j=0; j<6; j++)	p_ATb[j]=0.0f;

	cp_jacobi = in_map_jacobi->mps(ww, hh)[0];
	p_diff = in_map_diff->vp();
	p_valid = in_map_valid->vp();

	cnt=0;
	for(int tidx=0; tidx<ww*hh; tidx++){
			
		if(!p_valid[tidx]) continue;

		float *J = cp_jacobi[tidx].vp();
			
		//////////////////////////////////////////////////////////////////////////
		// computes right hand.
		float b = -p_diff[tidx];
		//////////////////////////////////////////////////////////////////////////

		// update ATb.
		for(int i=0; i<6; i++)	p_ATb[i]+=J[i]*b;
		cnt++;

	}


	//printf("[%d] cnt: %d\n", in_level_of_pyramid, cnt);
	
	return cnt;
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::glhlse_Generate_Left_Hands_of_Least_Square_Equation(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Tracking_State *in_state,
	CKvSet2d_of_MatrixFloat *in_map_jacobi,
	CKvMatrixBool *in_map_valid,
	int in_level_of_pyramid,
	CKvMatrixFloat *out_JTJ)
//********************************************************************************************
{
	CKvMatrixFloat *cp_jacobi;

	bool *p_valid;

	float *p_JTJ;
	float J_t[2*6], tJTJ[6*6];
	int ww, hh, cnt;
	
	// get pointers.
	p_JTJ=out_JTJ->vp();	for(int i=0; i<6*6; i++)	p_JTJ[i]=0.0f;
	
	cp_jacobi = in_map_jacobi->mps(ww, hh)[0];
	p_valid = in_map_valid->vp();
	
	cnt=0;

	for(int tidx=0; tidx<ww*hh; tidx++){
			
		if(!p_valid[tidx]) continue;

		int tw, th;
		float *J = cp_jacobi[tidx].vp();
					
		// update JTJ.
		//d_t_Transpose(J, 2, 6, J_t);
		//d_mmm_Multiply_Matrix_Matrix(J_t, J, 6, 2, 6, tJTJ);
		d_cgm_Compute_Gram_Matrix(J, 2, 6, tJTJ);
		
		for(int i=0; i<6*6; i++) p_JTJ[i] += tJTJ[i]; 
		cnt++;

	}

	//d_pm_Printf_Matrix(p_JTJ, 6, 6, "JTJ");
	
	return cnt;
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::grhlse_Generate_Right_Hands_of_Least_Square_Equation(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Tracking_State *in_state,
	CKvSet2d_of_MatrixFloat *in_map_jacobi,
	CKvSet2d_of_VectorFloat *in_map_diff,
	CKvMatrixBool *in_map_valid,
	int in_level_of_pyramid,
	CKvVectorFloat *out_ATb)
//********************************************************************************************
{
	CKvMatrixFloat *cp_jacobi;
	CKvVectorFloat *cp_diff;
	bool *p_valid;

	float *p_JTb;
	float J_t[6*2], b[2], tJTb[6];
	int ww, hh, cnt;
	
	// get pointers.
	p_JTb=out_ATb->vp();
	for(int j=0; j<6; j++)	p_JTb[j]=0.0f;

	cp_jacobi = in_map_jacobi->mps(ww, hh)[0];
	cp_diff = in_map_diff->vp();
	p_valid = in_map_valid->vp();

	cnt=0;
	for(int tidx=0; tidx<ww*hh; tidx++){
			
		if(!p_valid[tidx]) continue;

		float *p_diff = cp_diff[tidx].vp();
		float *J = cp_jacobi[tidx].vp();
			
		//////////////////////////////////////////////////////////////////////////
		// computes right hand.
		d_t_Transpose(J, 2, 6, J_t);
		for(int i=0; i<2; i++) b[i] = -p_diff[i];
		d_mmm_Multiply_Matrix_Matrix(J_t, b, 6, 2, 1, tJTb);
		
		for(int i=0; i<6; i++) p_JTb[i] += tJTb[i];
		//////////////////////////////////////////////////////////////////////////

		//d_pm_Printf_Matrix(p_JTb, 1, 6, "Jtb");

		cnt++;

	}


	//printf("[%d] cnt: %d\n", in_level_of_pyramid, cnt);
	
	return cnt;
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::glsew_Generate_Least_Square_Equation_with_Photoconsist_Max_Yooji_Weighted(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Tracking_State *in_state,
	CKvSet2d_of_VectorFloat *in_map_jacobi,
	CKvMatrixFloat *in_map_diff,
	CKvMatrixFloat *in_map_weight,
	CKvMatrixBool *in_map_valid,
	int in_level_of_pyramid,
	CKvMatrixFloat *out_ATA,
	CKvVectorFloat *out_ATb)
//********************************************************************************************
{
	CKvYooji_Intrinsics *p_intrin_rgb;
	CKvYooji_Extrinsics *p_pose_gc_rgb_t0;
	CKvPointf t_pix, t_pix_rgb;
	CKvPoint3Df *p_p3d;

	CKvVectorFloat *cp_jacobi;

	bool *p_sil_t1;
	unsigned char *p_img_gray_t0, *p_img_gray_t1;
	float *p_grad_x_t0, *p_grad_y_t0, grad[2];
	float *p_depth_t0;
	float *p_confidence_t0, *p_diff, *p_weight;
	bool *p_valid;

	float A[6], b;
	float *p_ATA, *p_ATb;

	float inten_t0, td_rgb_t0;
	bool flag_valid = false;
	int ww, hh, cnt, x, y;
	float fx, fy, px, py, dmin, dmax;
	
	// get pointers.
	p_ATA=out_ATA->vp();		p_ATb=out_ATb->vp();
	for(int j=0; j<6; j++){
		A[j]=p_ATb[j]=0.0f;
		for(int i=0; i<6; i++){
			p_ATA[j*6+i]=0.0f;
		}
	}
		
	p_img_gray_t0 = in_state->p_pyramid_image_gray()->imgs[in_level_of_pyramid].mps(ww, hh)[0];
	p_img_gray_t1 = in_view->p_pyramid_image_gray()->imgs[in_level_of_pyramid].vp();
	
	cp_jacobi = in_map_jacobi->vp();
	p_diff = in_map_diff->vp();
	p_weight = in_map_weight->vp();
	p_valid = in_map_valid->vp();

	cnt=0;
	for(y=0; y<hh; y++){
		for(x=0; x<ww; x++){

			int tidx = y*ww + x;
			
			if(!p_valid[tidx]) continue;

			float *J = cp_jacobi[tidx].vp();
			
			//////////////////////////////////////////////////////////////////////////
			// computes right hand.
			b = -p_diff[tidx];
			//////////////////////////////////////////////////////////////////////////	

			// update ATA and ATb.			
			//float weight = p_confidence_t0[y*ww + x];
			float weight = p_weight[tidx];

			z_ulsw_Update_Linear_System_Weighted(J, b, weight, p_ATA, p_ATb);
			cnt++;

		}
	}

	//printf("[%d] cnt: %d\n", in_level_of_pyramid, cnt);
	
	return cnt;
}


//********************************************************************************************
float LCKvYooji_Camera_Tracker::cidm_Compute_Intensity_Difference_Map(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Tracking_State *in_state,
	CKvMatrixBool *in_map_valid,
	CKvSet2d_of_Pointf *in_map_reproj,
	int in_level_of_pyramid,
	CKvMatrixFloat *out_map_diff)
//********************************************************************************************
{
	const CKvPointf *p_p2d;
	const bool *p_valid;

	unsigned char *p_img_gray_t0, *p_img_gray_t1;
	float *p_diff;
		
	float inten_t0, inten_t1, total_diff;
	int ww, hh, cnt, x, y;
	
	// get pointers.
	p_img_gray_t0 = in_state->p_pyramid_image_gray()->imgs[in_level_of_pyramid].mps(ww, hh)[0];
	p_img_gray_t1 = in_view->p_pyramid_image_gray()->imgs[in_level_of_pyramid].vp();
	
	if(out_map_diff->mw() != ww || out_map_diff->mh() != hh) out_map_diff->c_Create(hh, ww, 0.0f);
	
	// In
	p_valid = in_map_valid->vp();
	p_p2d = in_map_reproj->vp();
	// Out
	p_diff = out_map_diff->vp();

	float tx, ty;

	cnt=0;	total_diff = 0.0f;
	for(int tidx=0; tidx<ww*hh; tidx++){

		p_diff[tidx] = 0.0f;
			
		if(!p_valid[tidx]) continue;

		//const CKvPointf &p_tp2d = p_p2d[tidx];

		inten_t0 = p_img_gray_t0[tidx];

		tx = p_p2d[tidx].x; ty = p_p2d[tidx].y;

		// Compute residual b for normal equation.
		z_gii_Get_Interpolated_Intensity(tx,ty,ww,hh,p_img_gray_t1,inten_t1);
		
		// scaling.
		float It = inten_t1 - inten_t0;
		p_diff[tidx] = It;///255.0f;

		total_diff += p_diff[tidx];

		//printf("p_map_diff[%d] : %f\n", tidx, p_diff[tidx]);
	}
	
	
	return total_diff;
}

//********************************************************************************************
float LCKvYooji_Camera_Tracker::cgmdm_Compute_Gradient_Magnitude_Difference_Map(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Tracking_State *in_state,
	CKvMatrixBool *in_map_valid,
	CKvSet2d_of_Pointf *in_map_reproj,
	int in_level_of_pyramid,
	CKvMatrixFloat *out_map_diff)
//********************************************************************************************
{
	const CKvPointf *p_p2d;
	const bool *p_valid;

	float *p_img_gradm_t0, *p_img_gradm_t1;
	float *p_diff;
		
	float inten_t0, inten_t1, total_diff;
	int ww, hh, cnt, x, y;
	
	// get pointers.
	p_img_gradm_t0 = in_state->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_mag.mps(ww, hh)[0];
	p_img_gradm_t1 = in_view->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_mag.vp();
	
	if(out_map_diff->mw() != ww || out_map_diff->mh() != hh) out_map_diff->c_Create(hh, ww, 0.0f);
	
	// In
	p_valid = in_map_valid->vp();
	p_p2d = in_map_reproj->vp();
	// Out
	p_diff = out_map_diff->vp();

	float tx, ty;

	cnt=0;	total_diff = 0.0f;
	for(int tidx=0; tidx<ww*hh; tidx++){

		p_diff[tidx] = 0.0f;
			
		if(!p_valid[tidx]) continue;

		//const CKvPointf &p_tp2d = p_p2d[tidx];

		inten_t0 = p_img_gradm_t0[tidx];

		tx = p_p2d[tidx].x; ty = p_p2d[tidx].y;

		// Compute residual b for normal equation.
		z_gim_Get_Interpolated_Magnitude(tx,ty,ww,hh,p_img_gradm_t1,inten_t1);
		
		// scaling.
		float It = inten_t1 - inten_t0;
		p_diff[tidx] = It;///255.0f;

		total_diff += p_diff[tidx];

		//printf("p_map_diff[%d] : %f\n", tidx, p_diff[tidx]);
	}
	
	
	return total_diff;
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::cgdm_Compute_Gradient_Difference_Map_New(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Tracking_State *in_state,
	CKvMatrixBool *in_map_valid,
	CKvSet2d_of_Pointf *in_map_reproj,
	CKvMatrixFloat *in_hmat_t0_t1,
	int in_level_of_pyramid,
	CKvSet2d_of_VectorFloat *out_map_diff)
//********************************************************************************************
{
	const CKvPointf *p_p2d;
	const CKvPoint3Df *p_p3d_t0, *p_norm_t0;
	const bool *p_valid;

	CKvYooji_Intrinsics *p_intrinsics;

	unsigned char *p_img_gray_t0, *p_img_gray_t1;
	float *p_grad_x_t0, *p_grad_y_t0;
	float *p_grad_x_t1, *p_grad_y_t1;
	CKvVectorFloat *cp_diff;

	float *p_hmat;
	static float R[9], t[3], n[3], tn[9], T_pi[9], tvec[3];

	CKvPoint3Df tp3d;
	CKvPointf tp2d;
	
	float grad_t0[2], grad_t1[2], nV;
	float inten_t0, inten_t1;
	int ww, hh, cnt, x, y;
	
	// get pointers.
	p_img_gray_t0 = in_state->p_pyramid_image_gray()->imgs[in_level_of_pyramid].mps(ww, hh)[0];
	p_img_gray_t1 = in_view->p_pyramid_image_gray()->imgs[in_level_of_pyramid].vp();

	p_grad_x_t0 = in_state->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_x.vp();
	p_grad_y_t0 = in_state->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_y.vp();

	p_grad_x_t1 = in_view->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_x.vp();
	p_grad_y_t1 = in_view->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_y.vp();

	p_p3d_t0 = in_state->p_pyramid_map_p3d()->imgs[in_level_of_pyramid].vp();
	p_norm_t0 = in_state->p_pyramid_map_normals()->imgs[in_level_of_pyramid].vp();
	
	if(out_map_diff->mw() != ww || out_map_diff->mh() != hh){
		out_map_diff->c_Create(hh, ww);
		for(int i=0; i<ww*hh; i++) out_map_diff->vp()[i].c_Create(2, 0.0f);
	}
	
	// In
	p_valid = in_map_valid->vp();
	p_p2d = in_map_reproj->vp();
	p_hmat = in_hmat_t0_t1->vp();
	// Out
	cp_diff = out_map_diff->vp();

	p_intrinsics = &in_view->p_pyramid_intrinsics()->intrins[in_level_of_pyramid];

	// Get R, t.
	for(int j=0; j<3; j++){
		t[j] = p_hmat[j*4 + 3];
		for(int i=0; i<3; i++)	R[j*3 + i] = p_hmat[j*4 + i];
	}

	cnt=0;
	for(int tidx=0; tidx<ww*hh; tidx++){

		float *p_diff = cp_diff[tidx].vp();
			
		if(!p_valid[tidx]) continue;

		const CKvPointf &p_tp2d = p_p2d[tidx];
		const CKvPoint3Df &p_tp3d = p_p3d_t0[tidx];
		const CKvPoint3Df &p_tnorm = p_norm_t0[tidx];

		for(int i=0; i<2; i++) p_diff[i] = 0.0f;

		//////////////////////////////////////////////////////////////////////////
		// use transformed gradient vector.
		//////////////////////////////////////////////////////////////////////////
// 		tp2d.x = p_grad_x_t0[tidx];	tp2d.y = p_grad_y_t0[tidx];
// 
// 		p_intrinsics->back_project(tp2d, 1.0f, tp3d);
// 
// 		// normalize plane normal.
//  		nV = p_tnorm.x*p_tp3d.x + p_tnorm.y*p_tp3d.y + p_tnorm.z*p_tp3d.z;
//  		n[0] = -p_tnorm.x/nV; n[1] = -p_tnorm.y/nV; n[2] = -p_tnorm.z/nV;
//  
//  		d_mvvt_Multiply_Vector_Vector_Transpose(t,n,3,tn);
//  		d_avw_Add_Vector_Weighted(R,tn,1.0f, -1.0f, 9, T_pi);
//  
//  		tvec[0] = tp3d.x; tvec[1] = tp3d.y; tvec[2] = tp3d.z;
//  		d_mmsv_Multiply_Matrix_Square_Vector(T_pi, tvec, 3, n);
//  
// 		//d_pm_Printf_Matrix(R, 3, 3, "R"); d_pm_Printf_Matrix(t, 1, 3, "t");
// 
//  		tp3d.x = n[0]; tp3d.y = n[1]; tp3d.z = n[2];
//  
//  		p_intrinsics->project(tp3d, tp2d);
//  
//  		grad_t0[0] = tp2d.x; grad_t0[1] = tp2d.y;

		//////////////////////////////////////////////////////////////////////////
		// use original gradient.
		//////////////////////////////////////////////////////////////////////////
		grad_t0[0] = p_grad_x_t0[tidx]; grad_t0[1] = p_grad_y_t0[tidx];

		// Compute residual b for normal equation.
		z_gig_Get_Interpolated_Gradient(p_tp2d.x,p_tp2d.y,ww,hh,p_grad_x_t1,grad_t1[0]);
		z_gig_Get_Interpolated_Gradient(p_tp2d.x,p_tp2d.y,ww,hh,p_grad_y_t1,grad_t1[1]);

// 		float mag_t1 = sqrt(SQUARE(grad_t1[0]) + SQUARE(grad_t1[1]));
// 		float mag_t0 = sqrt(SQUARE(grad_t0[0]) + SQUARE(grad_t0[1]));
// 		float scale_t1 = min(20.0f, mag_t1)/mag_t1;
// 		float scale_t0 = min(20.0f, mag_t0)/mag_t0;
// 
// 		// clip magnitude of gradient vectors.
// 		for(int i=0; i<2; i++){
// 			grad_t0[i] = scale_t0*grad_t0[i];
// 			grad_t1[i] = scale_t1*grad_t1[i];
// 		}

		// compute vector difference.
		for(int i=0; i<2; i++)	p_diff[i] = grad_t1[i] - grad_t0[i];

		//d_pm_Printf_Matrix(p_diff, 1, 2, "Grad Diff");
		
		//p_diff[tidx] /= 255.0f;

		//printf("grad1: %f %f -> %f %f / grad2: %f %f\n", p_grad_x_t0[tidx], p_grad_y_t0[tidx], grad_t0[0], grad_t0[1], grad_t1[0], grad_t1[1]);

		//printf("p_map_diff[%d] : %f\n", tidx, p_diff[tidx]);
	}
	
	
	return cnt;
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::cwmtd_Compute_Weight_Map_based_on_T_Distribution(
	CKvMatrixFloat *in_map_diff,
	CKvMatrixBool *in_map_valid,
	CKvMatrixFloat *out_map_weight,
	int in_num_mode)
//********************************************************************************************
{
	int ww, hh, cnt;

	float *p_diff = in_map_diff->mps(ww, hh)[0];
	bool *p_valid = in_map_valid->vp();
	
	if(out_map_weight->mw() != ww || out_map_weight->mh() != hh)	out_map_weight->c_Create(hh, ww, 0.0f);

	float *p_weight = out_map_weight->vp();

	//////////////////////////////////////////////////////////////////////////
	// Compute initial sigma of difference image. (sigma of Gaussian distribution)
	//////////////////////////////////////////////////////////////////////////
// 	float var_prev = 0.0f;
// 	cnt = 0;
// 	for(int i=0; i<ww*hh; i++){
// 		if(p_valid[i]){
// 			var_prev += SQUARE(p_diff[i]);
// 			cnt++;
// 		}
// 	}
// 	var_prev = var_prev/cnt;
// 
// 	if(cnt == 0 || var_prev < 1.0e-7f){
// 		for(int i=0; i<ww*hh; i++)	p_weight[i] = 1.0f;
// 		return cnt;
// 	}

	float var_prev = SQUARE(5.0f);
	cnt = 0;
	for(int i=0; i<ww*hh; i++){	if(p_valid[i])	cnt++;	}

	//////////////////////////////////////////////////////////////////////////
	// Compute sigma of t-distribution iteratively.
	//////////////////////////////////////////////////////////////////////////
	float v = in_num_mode;
	float inv_var = 1.0f/var_prev;
	float inv_var_prev;

	do{
		inv_var_prev = inv_var;
		inv_var = 0.0f;
		
		for(int i=0; i<ww*hh; i++){
			if(p_valid[i]){
				float res = p_diff[i];
				inv_var +=  SQUARE(res)*(v + 1)/(v + inv_var_prev*SQUARE(res));
			}			
		}
		// compute inverse of variance.
		inv_var = 1.0f/(inv_var/cnt);

		//printf("inv_var: %f (%f)\n", inv_var, inv_var_prev);

	} while(abs(inv_var - inv_var_prev) > 1.0e-3f);

	//////////////////////////////////////////////////////////////////////////
	// Compute residual weight based on t-distribution.
	//////////////////////////////////////////////////////////////////////////
	for(int i=0; i<ww*hh; i++){
		if(p_valid[i]){
			p_weight[i] = (v + 1)/(v + inv_var*SQUARE(p_diff[i]));

			//printf("weight:%f\n", p_weight[i]);
		}
	}

	return cnt;
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::ivm_Initialize_Validity_Map(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Tracking_State *in_state,
	int in_level_of_pyramid,
	CKvMatrixBool *out_map_valid)
//********************************************************************************************
{
	CKvYooji_Intrinsics *p_intrin_rgb;
	CKvPoint3Df *p_p3d;
	CKvPoint3Df *p_norm;

	unsigned char *p_img_gray_t0,*p_img_gray_t1;
	float *p_grad_x_t0,*p_grad_y_t0,grad[2];
	float *p_depth_t0;
	float *p_confidence_t0,*p_diff,*p_weight;
	bool *p_valid;

	float A[6],b;
	float *p_ATA,*p_ATb;

	float inten_t0,td_rgb_t0;
	bool flag_valid = false;
	int ww,hh,cnt,x,y;
	float fx,fy,px,py,dmin,dmax;

	float angle_th = cos(PI*60.0f/180.0f);

	// get pointers.
	for(int j=0; j<6; j++)	A[j]=0.0f;

	p_p3d = in_state->p_pyramid_map_p3d()->imgs[in_level_of_pyramid].mps(ww, hh)[0];
	p_norm = in_state->p_pyramid_map_normals()->imgs[in_level_of_pyramid].vp();

	if(out_map_valid->mw() != ww || out_map_valid->mh() != hh) out_map_valid->c_Create(hh, ww, false);

	p_valid = out_map_valid->vp();

	cnt=0;
	for(int tidx=0; tidx<ww*hh; tidx++){

		p_valid[tidx] = false;

		// check depth.
		if(p_p3d[tidx].z < MIN_DEPTH || p_p3d[tidx].z > MAX_DEPTH) continue;

		// check normal.
		if(p_norm[tidx].z <= -100.0f || -p_norm[tidx].z < angle_th) continue;

		p_valid[tidx] = true;

		cnt++;

	}
	

	//printf("[%d] cnt: %d\n", in_level_of_pyramid, cnt);

	return cnt;
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::ivm_Initialize_Validity_Maps(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Tracking_State *in_state,
	int in_level_of_pyramid,
	CKvMatrixBool *out_map_valid_pm,
	CKvMatrixBool *out_map_valid_grad,
	CKvMatrixBool *out_map_valid_icp)
//********************************************************************************************
{
	CKvYooji_Intrinsics *p_intrin_rgb;
	CKvPoint3Df *p_p3d;
	CKvPoint3Df *p_norm;

	unsigned char *p_img_gray_t0,*p_img_gray_t1;
	float *p_grad_x_t0,*p_grad_y_t0, *p_grad_D_x_t0,*p_grad_D_y_t0, grad[2];
	float *p_grad_xx_t0, *p_grad_yy_t0, *p_laplace;
	float *p_grad_mag_x_t0,*p_grad_mag_y_t0;

	float *p_depth_t0;
	float *p_confidence_t0,*p_diff,*p_weight;
	bool *p_valid_pm, *p_valid_grad, *p_valid_icp;

	//////////////////////////////////////////////////////////////////////////
	// for RGB-D test setting.
	static float th_grad_sq[4] ={SQUARE(1),SQUARE(2),SQUARE(3),SQUARE(4)};
	static const float th_grad_2_sq[4] ={SQUARE(20),SQUARE(30),SQUARE(30),SQUARE(30)};
	//static const float th_grad_2_sq[4] ={SQUARE(0.5),SQUARE(0.5),SQUARE(0.5),SQUARE(0.5)};
	
	//	static const float th_grad_2_sq[4] ={SQUARE(10),SQUARE(15),SQUARE(20),SQUARE(25)};
	//	static const float th_grad_2_sq[4] ={SQUARE(5),SQUARE(10),SQUARE(15),SQUARE(20)};

	static const float th_D_sq[4] ={SQUARE(0.02f),SQUARE(0.03f),SQUARE(0.04f),SQUARE(0.05f)};
	//static const float th_D_sq[4] = {SQUARE(0.02f),SQUARE(0.02f),SQUARE(0.02f),SQUARE(0.02f)};
	//////////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////////
	// Proposed test setting.
// 	static const float th_grad_sq[4] ={SQUARE(1),SQUARE(3),SQUARE(5),SQUARE(12)};
// //	static const float th_grad_sq[4] ={SQUARE(1),SQUARE(2),SQUARE(3),SQUARE(4)};
// 	static const float th_grad_2_sq[4] ={SQUARE(20),SQUARE(30),SQUARE(30),SQUARE(30)};
// //	static const float th_grad_2_sq[4] ={SQUARE(10),SQUARE(15),SQUARE(20),SQUARE(25)};
// //	static const float th_grad_2_sq[4] ={SQUARE(5),SQUARE(10),SQUARE(15),SQUARE(20)};
// 
// 	static const float th_D_sq[4] = {SQUARE(0.02f),SQUARE(0.03f),SQUARE(0.04f),SQUARE(0.05f)}; 
// 	//static const float th_D_sq[4] = {SQUARE(0.02f),SQUARE(0.02f),SQUARE(0.02f),SQUARE(0.02f)}; 
	//////////////////////////////////////////////////////////////////////////

	if(!KV_FLAG_RGBD_OLD){
		th_grad_sq[0] = SQUARE(3);
		th_grad_sq[1] = SQUARE(5);
		th_grad_sq[2] = SQUARE(7);
		th_grad_sq[3] = SQUARE(12);
	}

	float A[6],b;
	float *p_ATA,*p_ATb;

	float inten_t0,td_rgb_t0;
	bool flag_valid = false;
	int ww,hh,cnt,x,y;
	float fx,fy,px,py,dmin,dmax;

	float angle_th = cos(PI*60.0f/180.0f);

	// get pointers.
	for(int j=0; j<6; j++)	A[j]=0.0f;

	p_p3d = in_state->p_pyramid_map_p3d()->imgs[in_level_of_pyramid].mps(ww, hh)[0];
	p_norm = in_state->p_pyramid_map_normals()->imgs[in_level_of_pyramid].vp();

	p_grad_x_t0 = in_state->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_x.vp();
	p_grad_y_t0 = in_state->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_y.vp();

	p_grad_xx_t0 = in_state->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_xx.vp();
	p_grad_yy_t0 = in_state->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_yy.vp();
	p_laplace = in_state->p_pyramid_map_edge()->maps[in_level_of_pyramid].laplacian.vp();

	p_grad_mag_x_t0 = in_state->p_pyramid_map_grad_edge()->maps[in_level_of_pyramid].grad_x.vp();
	p_grad_mag_y_t0 = in_state->p_pyramid_map_grad_edge()->maps[in_level_of_pyramid].grad_y.vp();

	p_grad_D_x_t0 = in_state->p_pyramid_map_depth_edge()->maps[in_level_of_pyramid].grad_x.vp();
	p_grad_D_y_t0 = in_state->p_pyramid_map_depth_edge()->maps[in_level_of_pyramid].grad_y.vp();
		
	if(out_map_valid_pm->mw() != ww || out_map_valid_pm->mh() != hh) out_map_valid_pm->c_Create(hh, ww, false);
	if(out_map_valid_grad->mw() != ww || out_map_valid_grad->mh() != hh) out_map_valid_grad->c_Create(hh, ww, false);
	if(out_map_valid_icp->mw() != ww || out_map_valid_icp->mh() != hh) out_map_valid_icp->c_Create(hh, ww, false);

	p_valid_pm = out_map_valid_pm->vp();
	p_valid_grad = out_map_valid_grad->vp();
	p_valid_icp = out_map_valid_icp->vp();

	bool flag_pm, flag_grad, flag_icp;
	float mag_grad, GLR_sq;
	cnt=0;	
	for(int tidx=0; tidx<ww*hh; tidx++){

		flag_pm = flag_grad = flag_icp = true;
		//p_valid_pm[tidx] = p_valid_grad[tidx] = p_valid_icp[tidx] = false;

		// check depth.
		if(p_p3d[tidx].z < MIN_DEPTH || p_p3d[tidx].z > MAX_DEPTH) flag_pm = flag_grad = flag_icp = false;
		
		// check normal.
		if(p_norm[tidx].z <= -100.0f || -p_norm[tidx].z < angle_th) flag_pm = flag_grad = flag_icp = false;

		
		// check image gradient.
		mag_grad = SQUARE(p_grad_x_t0[tidx]) + SQUARE(p_grad_y_t0[tidx]);
		//////////////////////////////////////////////////////////////////////////
		GLR_sq = SQUARE(p_laplace[tidx]);
		//GLR_sq = SQUARE(p_laplace[tidx])/mag_grad; // gradient-Laplacian ratio.
		//////////////////////////////////////////////////////////////////////////

		if(KV_FLAG_RGB_COST){
			if(mag_grad < th_grad_sq[in_level_of_pyramid])	 flag_pm = false;
			// threshold gradient magnitude
			//if(!KV_FLAG_RGBD_OLD)	if(mag_grad > th_grad_2_sq[in_level_of_pyramid]) flag_pm = false;
			// threshold laplacian magnitude
			else{
			//////////////////////////////////////////////////////////////////////////
				if(!KV_FLAG_RGBD_OLD){
					if(GLR_sq > th_grad_2_sq[in_level_of_pyramid]) flag_pm = false;
					//printf("GLR: %f\n", GLR_sq);
				}
			}
			//////////////////////////////////////////////////////////////////////////
		}
		
		if(KV_FLAG_GRAD_COST){
			//if(mag_grad > SQUARE(100))	flag_grad = false;
			// threshold gradient magnitude
			//if(KV_FLAG_RGB_COST) if(mag_grad <= th_grad_2_sq[in_level_of_pyramid]) flag_grad = false;
			// threshold laplacian magnitude
			if(KV_FLAG_RGB_COST) if(GLR_sq <= th_grad_2_sq[in_level_of_pyramid]) flag_grad = false;


			// check gradient of gradient magnitude.
			mag_grad = SQUARE(p_grad_mag_x_t0[tidx]) + SQUARE(p_grad_mag_x_t0[tidx]);
			if(mag_grad < th_grad_sq[in_level_of_pyramid])	flag_grad = false;

			// check image hessian.
// 			mag_grad = SQUARE(p_grad_xx_t0[tidx]) + SQUARE(p_grad_yy_t0[tidx]);
// 			if(mag_grad < th_grad_sq[in_level_of_pyramid])	flag_grad = false;
		}
		
		//////////////////////////////////////////////////////////////////////////
		if(KV_FLAG_GRAD_COST){
			// check depth gradient.
			mag_grad = SQUARE(p_grad_D_x_t0[tidx]) + SQUARE(p_grad_D_y_t0[tidx]);
			if(mag_grad > th_D_sq[in_level_of_pyramid]) flag_pm = flag_grad = false;
		}
		//////////////////////////////////////////////////////////////////////////

		p_valid_pm[tidx] = flag_pm;
		p_valid_grad[tidx] = flag_grad;
		p_valid_icp[tidx] = flag_icp;

		cnt++;

	}
	

	//printf("[%d] cnt: %d\n", in_level_of_pyramid, cnt);

	return cnt;
}


//********************************************************************************************
int LCKvYooji_Camera_Tracker::cvm_Compute_Validity_Map_for_PhotoMax(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Tracking_State *in_state,
	int in_level_of_pyramid,
	CKvMatrixBool *out_map_valid)
//********************************************************************************************
{
	CKvYooji_Intrinsics *p_intrin_rgb;
	CKvPoint3Df *p_p3d;
	CKvPoint3Df *p_norm;

	static const float th_grad_sq[4] ={SQUARE(1),SQUARE(3),SQUARE(5),SQUARE(12)};

	bool *p_sil_t1;
	unsigned char *p_img_gray_t0,*p_img_gray_t1;
	float *p_grad_x_t0,*p_grad_y_t0,grad[2];
	float *p_depth_t0;
	float *p_confidence_t0,*p_diff,*p_weight;
	bool *p_valid;

	float A[6],b;
	float *p_ATA,*p_ATb;

	float inten_t0,td_rgb_t0;
	bool flag_valid = false;
	int ww,hh,cnt,x,y;
	float fx,fy,px,py,dmin,dmax;

	float angle_th = cos(PI*60.0f/180.0f);

	// get pointers.
	for(int j=0; j<6; j++)	A[j]=0.0f;

	p_grad_x_t0 = in_state->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_x.mps(ww, hh)[0];
	p_grad_y_t0 = in_state->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_y.vp();

	p_p3d = in_state->p_pyramid_map_p3d()->imgs[in_level_of_pyramid].vp();
	p_norm = in_state->p_pyramid_map_normals()->imgs[in_level_of_pyramid].vp();

	if(out_map_valid->mw() != ww || out_map_valid->mh() != hh) out_map_valid->c_Create(hh, ww, false);

	p_valid = out_map_valid->vp();

	cnt=0;
	for(int tidx=0; tidx<ww*hh; tidx++){

		p_valid[tidx] = false;

		// check gradient.
		//////////////////////////////////////////////////////////////////////////
		if(SQUARE(p_grad_x_t0[tidx]) + SQUARE(p_grad_y_t0[tidx]) < th_grad_sq[in_level_of_pyramid]) continue;
		//////////////////////////////////////////////////////////////////////////

		// check depth.
		if(p_p3d[tidx].z < MIN_DEPTH || p_p3d[tidx].z > MAX_DEPTH) continue;

		// check normal.
		if(p_norm[tidx].z <= -100.0f || -p_norm[tidx].z < angle_th) continue;

		p_valid[tidx] = true;

		cnt++;

	}
	

	//printf("[%d] cnt: %d\n", in_level_of_pyramid, cnt);

	return cnt;
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::cjpt_Compute_Jacobian_Projection_and_Transformation(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Tracking_State *in_state,
	CKvMatrixBool *in_map_valid,
	int in_level_of_pyramid,
	CKvSet2d_of_MatrixFloat *out_map_jacobian)
//********************************************************************************************
{
	CKvPoint3Df *p_p3d;
	CKvMatrixFloat *cp_jacobi;

	static float th_grad_sq[4] ={SQUARE(1),SQUARE(3),SQUARE(7),SQUARE(12)};
	
	unsigned char *p_img_gray_t0,*p_img_gray_t1;
	float *p_grad_x_t0,*p_grad_y_t0,grad[2];
	float *p_depth_t0;
	float *p_confidence_t0,*p_diff,*p_weight;
	bool *p_valid;

	float A[6],b;
	float *p_ATA,*p_ATb;

	float inten_t0,td_rgb_t0;
	bool flag_valid = false;
	int ww,hh,cnt,x,y;
	float fx,fy,px,py,dmin,dmax;

	// get pointers.
	p_valid = in_map_valid->vp();
	
	p_grad_x_t0 = in_state->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_x.vp();
	p_grad_y_t0 = in_state->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_y.vp();
	
	p_p3d = in_state->p_pyramid_map_p3d()->imgs[in_level_of_pyramid].mps(ww,hh)[0];
	//p_p3d = zz_map_p3d[in_level_of_pyramid].mps(ww,hh)[0];

	//p_p3d = in_state->p_pyramid_map_p3d_rendered()->imgs[in_level_of_pyramid].mps(ww,hh)[0];

	if(out_map_jacobian->mw() != ww || out_map_jacobian->mh() != hh){
		out_map_jacobian->c_Create(hh,ww);
		for(int i=0; i<ww*hh; i++) out_map_jacobian->vp()[i].c_Create(2,6,0.0f);
	}

	cp_jacobi = out_map_jacobian->vp();

	cnt=0;
	for(int tidx=0; tidx<ww*hh; tidx++){

		float **J = cp_jacobi[tidx].mp();
		for(int i=0; i<2; i++) for(int j=0; j<6; j++)	J[i][j] = 0.0f;

		if(!p_valid[tidx]) continue;

		//////////////////////////////////////////////////////////////////////////
		float mag_sq = SQUARE(p_grad_x_t0[tidx]) + SQUARE(p_grad_y_t0[tidx]);
		if(mag_sq < th_grad_sq[in_level_of_pyramid]){
			p_valid[tidx] = false;
			continue;
		}
		////////////////////////////////////////////////////////////////

		// computes left hand.
		z_cjpt_Compute_Jacobians_of_Projection_and_Transformation(p_p3d[tidx],J);

		//d_pm_Printf_Matrix(J[0], 2, 6, "J");

		cnt++;
	}


	//printf("[%d] cnt: %d\n", in_level_of_pyramid, cnt);

	return cnt;
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::cjpti_Compute_Jacobian_Projection_and_Transformation_Iterative(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Tracking_State *in_state,
	CKvMatrixBool *in_map_valid,
	CKvSet2d_of_Point3Df *in_map_p3d_warped,
	int in_level_of_pyramid,
	CKvSet2d_of_MatrixFloat *out_map_jacobian)
//********************************************************************************************
{
	CKvPoint3Df *p_p3d;
	CKvMatrixFloat *cp_jacobi;

	unsigned char *p_img_gray_t0,*p_img_gray_t1;
	float *p_grad_x_t0,*p_grad_y_t0,grad[2];
	float *p_depth_t0;
	float *p_confidence_t0,*p_diff,*p_weight;
	bool *p_valid;

	float A[6],b;
	float *p_ATA,*p_ATb;

	float inten_t0,td_rgb_t0;
	bool flag_valid = false;
	int ww,hh,cnt,x,y;
	float fx,fy,px,py,dmin,dmax;

	// get pointers.
	p_valid = in_map_valid->vp();
		
	p_p3d = in_map_p3d_warped->mps(ww,hh)[0];
	//p_p3d = zz_map_p3d[in_level_of_pyramid].mps(ww,hh)[0];

	//p_p3d = in_state->p_pyramid_map_p3d_rendered()->imgs[in_level_of_pyramid].mps(ww,hh)[0];

	if(out_map_jacobian->mw() != ww || out_map_jacobian->mh() != hh){
		out_map_jacobian->c_Create(hh,ww);
		for(int i=0; i<ww*hh; i++) out_map_jacobian->vp()[i].c_Create(2,6,0.0f);
	}

	cp_jacobi = out_map_jacobian->vp();

	cnt=0;
	for(int tidx=0; tidx<ww*hh; tidx++){

		float **J = cp_jacobi[tidx].mp();
		for(int i=0; i<2; i++) for(int j=0; j<6; j++)	J[i][j] = 0.0f;

		if(!p_valid[tidx]) continue;

		// computes left hand.
		z_cjpt_Compute_Jacobians_of_Projection_and_Transformation(p_p3d[tidx],J);

		//d_pm_Printf_Matrix(J[0], 2, 6, "J");

		cnt++;
	}


	//printf("[%d] cnt: %d\n", in_level_of_pyramid, cnt);

	return cnt;
}


//********************************************************************************************
int LCKvYooji_Camera_Tracker::cjpm_Compute_Jacobian_PhotoMax(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Tracking_State *in_state,
	CKvSet2d_of_Pointf *in_map_reproj,
	CKvSet2d_of_MatrixFloat *in_map_jacobian_proj_and_trans,
	int in_level_of_pyramid,
	CKvMatrixBool *io_map_valid,
	CKvSet2d_of_VectorFloat *out_map_jacobian)
//********************************************************************************************
{
	CKvYooji_Intrinsics *p_intrin_rgb;
	CKvPoint3Df *p_p3d;
	CKvPointf *p_p2d;
	CKvVectorFloat *cp_jacobi;
	CKvMatrixFloat *cp_jacobi_pit;

	unsigned char *p_img_gray_t0,*p_img_gray_t1;
	float *p_grad_x_t0,*p_grad_y_t0;
	float *p_grad_x_t1,*p_grad_y_t1;
	float grad_t0[2], grad_t1[2];
	float *p_depth_t0;
	float *p_confidence_t0,*p_diff,*p_weight;
	bool *p_valid;

	float A[6],b;
	float *p_ATA,*p_ATb;

	float inten_t0,td_rgb_t0;
	bool flag_valid = false;
	int ww,hh,cnt,x,y;
	float fx,fy,px,py,dmin,dmax;

	static const float th_grad_sq[4] ={SQUARE(1),SQUARE(3),SQUARE(5),SQUARE(12)};
	static const float th_grad_2_sq[4] ={SQUARE(20),SQUARE(30),SQUARE(40),SQUARE(50)};


	// get pointers.
	for(int j=0; j<6; j++)	A[j]=0.0f;

	p_grad_x_t0 = in_state->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_x.mps(ww,hh)[0];
	p_grad_y_t0 = in_state->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_y.vp();

	p_grad_x_t1 = in_view->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_x.mps(ww, hh)[0];
	p_grad_y_t1 = in_view->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_y.vp();

	p_p2d = in_map_reproj->vp();
	
	p_p3d = in_state->p_pyramid_map_p3d()->imgs[in_level_of_pyramid].vp();	
	//p_p3d = in_state->p_pyramid_map_p3d_rendered()->imgs[in_level_of_pyramid].vp();
	
	cp_jacobi_pit = in_map_jacobian_proj_and_trans->vp();

	if(out_map_jacobian->mw() != ww || out_map_jacobian->mh() != hh){
		out_map_jacobian->c_Create(hh, ww);
		for(int i=0; i<ww*hh; i++) out_map_jacobian->vp()[i].c_Create(6, 0.0f);
	}

	p_valid = io_map_valid->vp();
	cp_jacobi = out_map_jacobian->vp();

	p_intrin_rgb = &in_view->p_pyramid_intrinsics()->intrins[in_level_of_pyramid];
	p_intrin_rgb->get_params(fx,fy,px,py,dmin,dmax);

	cnt=0;

	float *J, **J_pit;
	float tx0, ty0, tx1, ty1;

	for(int tidx=0; tidx<ww*hh; tidx++){

		if(!p_valid[tidx]) continue;

 		//CKvPointf &tp2d = p_p2d[tidx];
 		J = cp_jacobi[tidx].vp();	for(int i=0; i<6; i++) J[i] = 0.0f;
 		J_pit = cp_jacobi_pit[tidx].mp();
 		for(int i=0; i<6; i++)	J[i] = 0.0f;
 
		float mag_sq;

		//tx0 = tidx%ww; ty0 = tidx/ww;
		//tx1 = p_p2d[tidx].x; ty1 = p_p2d[tidx].y;
 		
 		// computes left hand.
		// + grad t0.
		grad_t0[0] = p_grad_x_t0[tidx];
		grad_t0[1] = p_grad_y_t0[tidx];
		//z_gig_Get_Interpolated_Gradient(tx0,ty0,ww,hh,p_grad_x_t0,grad_t0[0]);
		//z_gig_Get_Interpolated_Gradient(tx0,ty0,ww,hh,p_grad_y_t0,grad_t0[1]);

// 		mag_sq = SQUARE(grad_t0[0]) + SQUARE(grad_t0[1]);
// 		if(mag_sq < th_grad_sq[in_level_of_pyramid]	|| mag_sq > th_grad_2_sq[in_level_of_pyramid]){
// 			p_valid[tidx] = false;
// 			continue;
// 		}

// 		// + grad t1.
//  		z_gig_Get_Interpolated_Gradient(tx1, ty1, ww, hh, p_grad_x_t1, grad_t1[0]);
//  		z_gig_Get_Interpolated_Gradient(tx1, ty1, ww, hh, p_grad_y_t1, grad_t1[1]);
// 
// 		mag_sq = SQUARE(grad_t1[0]) + SQUARE(grad_t1[1]);
// 		if(mag_sq < th_grad_sq[in_level_of_pyramid]/*	|| mag_sq > th_grad_2_sq[in_level_of_pyramid]*/){
// 			p_valid[tidx] = false;
// 			continue;
// 		}
 
		// Efficient Second-order Minimization!!!!
// 		grad_t1[0] = 0.5f*(grad_t0[0] + grad_t1[0]);
// 		grad_t1[1] = 0.5f*(grad_t0[1] + grad_t1[1]);

 		// scaling.
 		//grad[0] /= 255.0f;	grad[1] /= 255.0f;
 
  		z_ccpm_Compute_Coefficients_with_Photo_Max_Yooji(
  			J_pit, grad_t0[0], grad_t0[1], fx, fy, J);

		cnt++;
	}
	

	//printf("[%d] cnt: %d\n", in_level_of_pyramid, cnt);

	return cnt;
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::cjgm_Compute_Jacobian_Gradinet_Magnitude(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Tracking_State *in_state,
	CKvSet2d_of_Pointf *in_map_reproj,
	CKvSet2d_of_MatrixFloat *in_map_jacobian_proj_and_trans,
	int in_level_of_pyramid,
	CKvMatrixBool *io_map_valid,
	CKvSet2d_of_VectorFloat *out_map_jacobian)
//********************************************************************************************
{
	CKvYooji_Intrinsics *p_intrin_rgb;
	CKvPoint3Df *p_p3d;
	CKvPointf *p_p2d;
	CKvVectorFloat *cp_jacobi;
	CKvMatrixFloat *cp_jacobi_pit;

	unsigned char *p_img_gray_t0,*p_img_gray_t1;
	float *p_grad_x_t0,*p_grad_y_t0;
	float *p_grad_x_t1,*p_grad_y_t1;
	float grad_t0[2], grad_t1[2];
	float *p_depth_t0;
	float *p_confidence_t0,*p_diff,*p_weight;
	bool *p_valid;

	float A[6],b;
	float *p_ATA,*p_ATb;

	float inten_t0,td_rgb_t0;
	bool flag_valid = false;
	int ww,hh,cnt,x,y;
	float fx,fy,px,py,dmin,dmax;

	static const float th_grad_sq[4] ={SQUARE(1),SQUARE(3),SQUARE(5),SQUARE(12)};
	static const float th_grad_2_sq[4] ={SQUARE(20),SQUARE(30),SQUARE(40),SQUARE(50)};


	// get pointers.
	for(int j=0; j<6; j++)	A[j]=0.0f;

	p_grad_x_t0 = in_state->p_pyramid_map_grad_edge()->maps[in_level_of_pyramid].grad_x.mps(ww,hh)[0];
	p_grad_y_t0 = in_state->p_pyramid_map_grad_edge()->maps[in_level_of_pyramid].grad_y.vp();

	p_grad_x_t1 = in_view->p_pyramid_map_grad_edge()->maps[in_level_of_pyramid].grad_x.mps(ww, hh)[0];
	p_grad_y_t1 = in_view->p_pyramid_map_grad_edge()->maps[in_level_of_pyramid].grad_y.vp();

	p_p2d = in_map_reproj->vp();
	
	p_p3d = in_state->p_pyramid_map_p3d()->imgs[in_level_of_pyramid].vp();	
	//p_p3d = in_state->p_pyramid_map_p3d_rendered()->imgs[in_level_of_pyramid].vp();
	
	cp_jacobi_pit = in_map_jacobian_proj_and_trans->vp();

	if(out_map_jacobian->mw() != ww || out_map_jacobian->mh() != hh){
		out_map_jacobian->c_Create(hh, ww);
		for(int i=0; i<ww*hh; i++) out_map_jacobian->vp()[i].c_Create(6, 0.0f);
	}

	p_valid = io_map_valid->vp();
	cp_jacobi = out_map_jacobian->vp();

	p_intrin_rgb = &in_view->p_pyramid_intrinsics()->intrins[in_level_of_pyramid];
	p_intrin_rgb->get_params(fx,fy,px,py,dmin,dmax);

	cnt=0;

	float *J, **J_pit;
	float tx0, ty0, tx1, ty1;

	for(int tidx=0; tidx<ww*hh; tidx++){

		if(!p_valid[tidx]) continue;

 		//CKvPointf &tp2d = p_p2d[tidx];
 		J = cp_jacobi[tidx].vp();	for(int i=0; i<6; i++) J[i] = 0.0f;
 		J_pit = cp_jacobi_pit[tidx].mp();
 		for(int i=0; i<6; i++)	J[i] = 0.0f;
 
		float mag_sq;

		//tx0 = tidx%ww; ty0 = tidx/ww;
		//tx1 = p_p2d[tidx].x; ty1 = p_p2d[tidx].y;
 		
 		// computes left hand.
		// + grad t0.
		grad_t0[0] = p_grad_x_t0[tidx];
		grad_t0[1] = p_grad_y_t0[tidx];
		//z_gig_Get_Interpolated_Gradient(tx0,ty0,ww,hh,p_grad_x_t0,grad_t0[0]);
		//z_gig_Get_Interpolated_Gradient(tx0,ty0,ww,hh,p_grad_y_t0,grad_t0[1]);

// 		mag_sq = SQUARE(grad_t0[0]) + SQUARE(grad_t0[1]);
// 		if(mag_sq < th_grad_sq[in_level_of_pyramid]	|| mag_sq > th_grad_2_sq[in_level_of_pyramid]){
// 			p_valid[tidx] = false;
// 			continue;
// 		}

// 		// + grad t1.
//  		z_gig_Get_Interpolated_Gradient(tx1, ty1, ww, hh, p_grad_x_t1, grad_t1[0]);
//  		z_gig_Get_Interpolated_Gradient(tx1, ty1, ww, hh, p_grad_y_t1, grad_t1[1]);
// 
// 		mag_sq = SQUARE(grad_t1[0]) + SQUARE(grad_t1[1]);
// 		if(mag_sq < th_grad_sq[in_level_of_pyramid]/*	|| mag_sq > th_grad_2_sq[in_level_of_pyramid]*/){
// 			p_valid[tidx] = false;
// 			continue;
// 		}
 
		// Efficient Second-order Minimization!!!!
// 		grad_t1[0] = 0.5f*(grad_t0[0] + grad_t1[0]);
// 		grad_t1[1] = 0.5f*(grad_t0[1] + grad_t1[1]);

 		// scaling.
 		//grad[0] /= 255.0f;	grad[1] /= 255.0f;
 
  		z_ccpm_Compute_Coefficients_with_Photo_Max_Yooji(
  			J_pit, grad_t0[0], grad_t0[1], fx, fy, J);

		cnt++;
	}
	

	//printf("[%d] cnt: %d\n", in_level_of_pyramid, cnt);

	return cnt;
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::cjgm_Compute_Jacobian_Gradient_based_Method_New(
	CKvYooji_MatrixRgbD *in_view,
	CKvYooji_Tracking_State *in_state,
	CKvSet2d_of_Pointf *in_map_reproj,
	CKvSet2d_of_MatrixFloat *in_map_jacobian_proj_and_trans,
	CKvSet2d_of_MatrixFloat *in_map_jacobian_plane_induced_homo_or_NULL,
	int in_level_of_pyramid,
	CKvMatrixBool *io_map_valid,
	CKvSet2d_of_MatrixFloat *out_map_jacobian)
//********************************************************************************************
{
	CKvYooji_Intrinsics *p_intrin_rgb;
	CKvPoint3Df *p_p3d, *p_norm;
	CKvPointf *p_p2d;
	CKvMatrixFloat *cp_jacobi;
	CKvMatrixFloat *cp_jacobi_pit, *cp_jacobi_pih;

	CKvPoint3Df tp3d;

	unsigned char *p_img_gray_t0,*p_img_gray_t1;
	float *p_grad_x_t0,*p_grad_y_t0;
	float *p_grad_x_t1,*p_grad_y_t1; 
	float *p_grad_xx_t0,*p_grad_yy_t0,*p_grad_xy_t0,*p_grad_yx_t0;
	float *p_grad_xx_t1,*p_grad_yy_t1,*p_grad_xy_t1,*p_grad_yx_t1; 
	float grad[2],f[4],hessian[4], Hf[4];
	float *p_depth_t0;
	float *p_confidence_t0,*p_diff,*p_weight;
	bool *p_valid;

	float A[12],b;
	float *p_ATA,*p_ATb;

	float inten_t0,td_rgb_t0;
	bool flag_valid = false;
	int ww,hh,cnt,x,y;
	float fx,fy,px,py,dmin,dmax;

	static const float th_grad_sq[4] ={SQUARE(1),SQUARE(3),SQUARE(5),SQUARE(12)};
	static const float th_grad_2_sq[4] ={SQUARE(20),SQUARE(30),SQUARE(40),SQUARE(50)};

	// get pointers.
	for(int j=0; j<12; j++)	A[j]=0.0f;

 	p_grad_x_t0 = in_state->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_x.mps(ww,hh)[0];
 	p_grad_y_t0 = in_state->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_y.vp();

	p_grad_x_t1 = in_view->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_x.mps(ww, hh)[0];
	p_grad_y_t1 = in_view->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_y.vp();

	p_grad_xx_t0 = in_state->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_xx.vp();
	p_grad_xy_t0 = in_state->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_xy.vp();
	p_grad_yx_t0 = in_state->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_yx.vp();
	p_grad_yy_t0 = in_state->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_yy.vp();

	p_grad_xx_t1 = in_view->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_xx.vp();
	p_grad_xy_t1 = in_view->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_xy.vp();
	p_grad_yx_t1 = in_view->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_yx.vp();
	p_grad_yy_t1 = in_view->p_pyramid_map_edge()->maps[in_level_of_pyramid].grad_yy.vp();

	p_p2d = in_map_reproj->vp();
	p_p3d = in_state->p_pyramid_map_p3d()->imgs[in_level_of_pyramid].vp();
	p_norm = in_state->p_pyramid_map_normals()->imgs[in_level_of_pyramid].vp();

	cp_jacobi_pit = in_map_jacobian_proj_and_trans->vp();
	if(in_map_jacobian_plane_induced_homo_or_NULL) 
		cp_jacobi_pih = in_map_jacobian_plane_induced_homo_or_NULL->vp();

	if(out_map_jacobian->mw() != ww || out_map_jacobian->mh() != hh){
		out_map_jacobian->c_Create(hh, ww);
		for(int i=0; i<ww*hh; i++) out_map_jacobian->vp()[i].c_Create(2, 6, 0.0f);
	}

	p_valid = io_map_valid->vp();
	cp_jacobi = out_map_jacobian->vp();

	p_intrin_rgb = &in_view->p_pyramid_intrinsics()->intrins[in_level_of_pyramid];
	p_intrin_rgb->get_params(fx,fy,px,py,dmin,dmax);

	f[0] = fx; f[1] = f[2] = 0.0f; f[3] = fy;

	cnt=0;
	for(int tidx=0; tidx<ww*hh; tidx++){

		if(!p_valid[tidx]) continue;

 		CKvPointf &tp2d = p_p2d[tidx];

 		float *J = cp_jacobi[tidx].vp();	for(int i=0; i<12; i++) J[i] = 0.0f;
 		float **J_pit = cp_jacobi_pit[tidx].mp();
		float **J_pih = cp_jacobi_pih[tidx].mp();

		//////////////////////////////////////////////////////////////////////////
		// use jacobians of image gradient and plane-induced homography.
		//for(int j=0; j<2; j++) for(int i=0; i<6; i++) A[2*j + i] = J_pit[j][i] + J_pih[j][i];
		// use jacobian of image gradient only.
		for(int j=0; j<2; j++) for(int i=0; i<6; i++) A[2*j + i] = J_pit[j][i];
		//////////////////////////////////////////////////////////////////////////
 
		//////////////////////////////////////////////////////////////////////////
		// jacobians for plane-induced homoagraphy.
		//////////////////////////////////////////////////////////////////////////
		if(in_map_jacobian_plane_induced_homo_or_NULL){
			// computes left hand.
			//   		z_gig_Get_Interpolated_Gradient(tp2d.x, tp2d.y, ww, hh, p_grad_x_t1, grad[0]);
			//   		z_gig_Get_Interpolated_Gradient(tp2d.x, tp2d.y, ww, hh, p_grad_y_t1, grad[1]);
			//   
			//   		// scaling.
			//  		float norm = SQUARE(grad[0]) + SQUARE(grad[1]);
			//   		//grad[0] /= 255.0f;	grad[1] /= 255.0f;
			//   
			//   		if(norm < th_grad_2_sq[in_level_of_pyramid]){
			//   			p_valid[tidx] = false;
			//   			continue;
			//   		} 
			//  		norm = sqrt(norm);
			// 		
			// 		// direction vector of gradient.
			// 		grad[0] /= norm;	grad[1] /= norm;

			//////////////////////////////////////////////////////////////////////////
			// jacobians for image gradient warping.
			//////////////////////////////////////////////////////////////////////////
			// 		z_gig_Get_Interpolated_Gradient(tp2d.x, tp2d.y, ww, hh, p_grad_xx_t1, hessian[0]);
			// 		z_gig_Get_Interpolated_Gradient(tp2d.x, tp2d.y, ww, hh, p_grad_xy_t1, hessian[1]);
			// 		z_gig_Get_Interpolated_Gradient(tp2d.x, tp2d.y, ww, hh, p_grad_yx_t1, hessian[2]);
			// 		z_gig_Get_Interpolated_Gradient(tp2d.x, tp2d.y, ww, hh, p_grad_yy_t1, hessian[3]);

			// direction vector of gradient.
			// 		float norm;
			// 		grad[0] = p_grad_x_t0[tidx];	grad[1] = p_grad_y_t0[tidx];
			// 		norm = SQUARE(grad[0]) + SQUARE(grad[1]);

			// check gradient magnitude.
			// 		if(norm < th_grad_2_sq[in_level_of_pyramid]){
			// 			p_valid[tidx] = false;
			// 			continue;
			// 		}
		}

  		

		hessian[0] = p_grad_xx_t0[tidx];
		hessian[1] = p_grad_xy_t0[tidx];
		hessian[2] = p_grad_yx_t0[tidx];
		hessian[3] = p_grad_yy_t0[tidx];

		// check diagonal term of hessian matrix.
// 		norm = SQUARE(hessian[0]) + SQUARE(hessian[3]);
// 		//grad[0] /= 255.0f;	grad[1] /= 255.0f;
// 
// 		if(norm < th_grad_sq[in_level_of_pyramid]){
// 			p_valid[tidx] = false;
// 			continue;
// 		}

		//d_pm_Printf_Matrix(hessian, 2, 2, "Hessian");

		// compute focal_length*hessian term.
		d_mmm_Multiply_Matrix_Matrix(hessian, f, 2, 2, 2, Hf);


		//////////////////////////////////////////////////////////////////////////


		d_mmm_Multiply_Matrix_Matrix(Hf, J_pit[0], 2, 2, 6, J);
		//////////////////////////////////////////////////////////////////////////

		//printf("grad: %f %f %f\n", grad[0], grad[1], norm);

//  		d_pm_Printf_Matrix(fH, 2, 2, "fH");
//  		d_pm_Printf_Matrix(J, 2, 6, "J");

		cnt++;
	}
	

	//printf("[%d] cnt: %d\n", in_level_of_pyramid, cnt);

	return cnt;
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::crp2dm_Compute_Reprojected_Point2D_Map(
	CKvYooji_MatrixRgbD *in_view,
	CKvSet2d_of_Point3Df *in_map_p3d,
	CKvMatrixBool *in_map_valid_pm,
	CKvMatrixBool *in_map_valid_icp,
	int in_level_of_pyramid,
	CKvSet2d_of_Pointf *out_map_reproj)
//********************************************************************************************
{
	CKvYooji_Intrinsics *p_intrin_rgb;
	
	CKvPointf *p_p2d;
	CKvPoint3Df *p_p3d;

	unsigned char *p_img_gray_t0, *p_img_gray_t1;
	float *p_depth_t0;
	float *p_map_diff;
	bool *p_valid_pm, *p_valid_icp;

	float inten_t0, td_rgb_t0;
	bool flag_valid;
	int ww, hh, cnt, x, y;
	
	// get pointers.
	in_view->p_pyramid_image_gray()->imgs[in_level_of_pyramid].ms(ww, hh);
	
	if(out_map_reproj->mw() != ww || out_map_reproj->mh() != hh) out_map_reproj->c_Create(hh,ww);

	p_p3d = in_map_p3d->vp();

	p_p2d = out_map_reproj->vp();
	p_valid_pm = in_map_valid_pm->vp();
	p_valid_icp = in_map_valid_icp->vp();

	p_intrin_rgb = &in_view->p_pyramid_intrinsics()->intrins[in_level_of_pyramid];

	cnt=0;
	for(int tidx=0; tidx<ww*hh; tidx++){

		CKvPoint3Df &p_tp3d = p_p3d[tidx];
		CKvPointf &p_tp2d = p_p2d[tidx];

		if(!p_valid_pm[tidx] && !p_valid_icp[tidx]) continue;
												
		// 3D cam t1 -> 2D cam t1.
		p_intrin_rgb->project(p_tp3d, zz_p2d_t1);

		p_tp2d.x = zz_p2d_t1.x;	p_tp2d.y = zz_p2d_t1.y;

		//printf("p_map_diff[%d] : %f\n", tidx, p_map_diff[tidx]);
	}
	
	return cnt;
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::cp3dm_Compute_Point3D_Map(
	CKvYooji_Tracking_State *in_state,
	CKvMatrixFloat *in_hmat_t0_to_t1_est,
	CKvMatrixBool *in_map_valid_pm,
	CKvMatrixBool *in_map_valid_icp,
	int in_level_of_pyramid,
	CKvSet2d_of_Point3Df *out_map_p3d)
//********************************************************************************************
{
	CKvYooji_Intrinsics *p_intrin_rgb;
	CKvPointf t_pix, t_pix_rgb;
	CKvPoint3Df *p_p3d_t0, *p_p3d_t0_trans;
	CKvPoint3Df zero_p3d;	zero_p3d.s_Set(0.0f, 0.0f, 0.0f);

	float *p_depth_t0;
	bool *p_valid_pm, *p_valid_icp;

	float inten_t0, td_rgb_t0;
	bool flag_valid;
	int ww, hh, ww_rgb, hh_rgb, cnt, x, y;

	// get pointers.	
	p_depth_t0 = in_state->p_pyramid_map_depth()->imgs[in_level_of_pyramid].mps(ww, hh)[0];
	p_intrin_rgb = &in_state->p_pyramid_intrinsics()->intrins[in_level_of_pyramid];

	if(out_map_p3d->mw() != ww || out_map_p3d->mh() !=hh) out_map_p3d->c_Create(hh, ww);

	p_p3d_t0 = in_state->p_pyramid_map_p3d()->imgs[in_level_of_pyramid].vp();
	p_p3d_t0_trans = out_map_p3d->vp();
	p_valid_pm = in_map_valid_pm->vp();
	p_valid_icp = in_map_valid_icp->vp();

	cnt=0;
	for(y=0; y<hh; y++){
		for(x=0; x<ww; x++){

			int tidx = y*ww + x;

			if(!p_valid_pm[tidx] && !p_valid_icp[tidx]) continue;
			
			// compute 3d point back-projected from the global model and previous camera pose in the global coordinates.
			z_tp3d_Transform_Point3D(in_hmat_t0_to_t1_est,p_p3d_t0[tidx],p_p3d_t0_trans[tidx]);	// 3D cam t0 -> t1.

			//printf("p_map_p3d[%d]: %f %f %f\n", tidx, p_map_p3d[tidx].x, p_map_p3d[tidx].y, p_map_p3d[tidx].z);
		}
	}
	
	return cnt;
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::crp2dm_Compute_Reprojected_Point2D_Map(
	CKvYooji_MatrixRgbD *in_view,
	CKvSet2d_of_Point3Df *in_map_p3d,
	CKvMatrixBool *in_map_valid,
	int in_level_of_pyramid,
	CKvSet2d_of_Pointf *out_map_reproj)
//********************************************************************************************
{
	CKvYooji_Intrinsics *p_intrin_rgb;
	
	CKvPointf *p_p2d;
	CKvPoint3Df *p_p3d;

	unsigned char *p_img_gray_t0, *p_img_gray_t1;
	float *p_depth_t0;
	float *p_map_diff;
	bool *p_valid;

	float inten_t0, td_rgb_t0;
	bool flag_valid;
	int ww, hh, cnt, x, y;
	
	// get pointers.
	in_view->p_pyramid_image_gray()->imgs[in_level_of_pyramid].ms(ww, hh);
	
	if(in_map_valid->mw() != ww || in_map_valid->mh() != hh) in_map_valid->c_Create(hh, ww, 0.0f);
	if(out_map_reproj->mw() != ww || out_map_reproj->mh() != hh) out_map_reproj->c_Create(hh,ww);

	p_p3d = in_map_p3d->vp();

	p_p2d = out_map_reproj->vp();
	p_valid = in_map_valid->vp();

	p_intrin_rgb = &in_view->p_pyramid_intrinsics()->intrins[in_level_of_pyramid];

	cnt=0;
	for(int tidx=0; tidx<ww*hh; tidx++){

		CKvPoint3Df &p_tp3d = p_p3d[tidx];
		CKvPointf &p_tp2d = p_p2d[tidx];

		if(!p_valid[tidx]) continue;
												
		// 3D cam t1 -> 2D cam t1.
		p_intrin_rgb->project(p_tp3d, zz_p2d_t1);

		p_tp2d.x = zz_p2d_t1.x;	p_tp2d.y = zz_p2d_t1.y;

		//printf("p_map_diff[%d] : %f\n", tidx, p_map_diff[tidx]);
	}
	
	return cnt;
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::cnm_Compute_Normal_Map(
	CKvYooji_MatrixRgbD *in_view,
	CKvMatrixBool *in_map_valid,
	CKvSet2d_of_Pointf *in_map_reproj,
	int in_level_of_pyramid,
	CKvSet2d_of_Point3Df *out_map_norm)
//********************************************************************************************
{
	CKvSet2d_of_Point3Df *cp_norm_t1 = &in_view->p_pyramid_map_normals()->imgs[in_level_of_pyramid];

	CKvPointf *p_p2d;
	CKvPoint3Df *p_out_norm;

	unsigned char *p_img_gray_t0, *p_img_gray_t1;
	float *p_depth_t0;
	float *p_map_diff;
	bool *p_valid;

	float inten_t0, td_rgb_t0;
	bool flag_valid;
	int ww, hh, cnt, x, y;
	
	// get pointers.
	in_view->p_pyramid_image_gray()->imgs[in_level_of_pyramid].ms(ww, hh);
	
	if(out_map_norm->mw() != ww || out_map_norm->mh() != hh) out_map_norm->c_Create(hh,ww);

	p_p2d = in_map_reproj->vp();
	p_valid = in_map_valid->vp();
	p_out_norm = out_map_norm->vp();

	cnt=0;
	for(int tidx=0; tidx<ww*hh; tidx++){

		CKvPoint3Df &p_tnorm = p_out_norm[tidx];
		CKvPointf &p_tp2d = p_p2d[tidx];

		if(!p_valid[tidx]) continue;

		z_gin_Get_Interpolated_Normal(p_tp2d.x, p_tp2d.y, ww, hh, cp_norm_t1, p_tnorm);

		//printf("p_map_diff[%d] : %f\n", tidx, p_map_diff[tidx]);
	}
	
	return cnt;
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::cp3dm_Compute_Point3D_Map(
	CKvYooji_Tracking_State *in_state,
	CKvMatrixFloat *in_hmat_t0_to_t1_est,
	CKvMatrixBool *in_map_valid,
	int in_level_of_pyramid,
	CKvSet2d_of_Point3Df *out_map_p3d)
//********************************************************************************************
{
	CKvYooji_Intrinsics *p_intrin_rgb;
	CKvPointf t_pix, t_pix_rgb;
	CKvPoint3Df *p_p3d_t0, *p_p3d_t0_trans;
	CKvPoint3Df zero_p3d;	zero_p3d.s_Set(0.0f, 0.0f, 0.0f);

	float *p_depth_t0;
	bool *p_valid;

	float inten_t0, td_rgb_t0;
	bool flag_valid;
	int ww, hh, ww_rgb, hh_rgb, cnt, x, y;

	// get pointers.	
	p_intrin_rgb = &in_state->p_pyramid_intrinsics()->intrins[in_level_of_pyramid];

	p_valid = in_map_valid->mps(ww, hh)[0];

	if(out_map_p3d->mw() != ww || out_map_p3d->mh() !=hh) out_map_p3d->c_Create(hh, ww);

	p_p3d_t0 = in_state->p_pyramid_map_p3d()->imgs[in_level_of_pyramid].vp();
	//p_p3d_t0 = in_state->p_pyramid_map_p3d_rendered()->imgs[in_level_of_pyramid].vp();

	p_p3d_t0_trans = out_map_p3d->vp();

	cnt=0;
	for(y=0; y<hh; y++){
		for(x=0; x<ww; x++){

			int tidx = y*ww + x;

			if(!p_valid[tidx])	continue;
			
			// compute 3d point back-projected from the global model and previous camera pose in the global coordinates.
			z_tp3d_Transform_Point3D(in_hmat_t0_to_t1_est,p_p3d_t0[tidx],p_p3d_t0_trans[tidx]);	// 3D cam t0 -> t1.
	
			//printf("p_map_p3d[%d]: %f %f %f\n", tidx, p_map_p3d[tidx].x, p_map_p3d[tidx].y, p_map_p3d[tidx].z);
		}
	}
	
	return cnt;
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::cp3dm_Compute_Point3D_Map(
	CKvYooji_Tracking_State *in_state,
	CKvMatrixFloat *in_hmat_t0_to_t1_est,
	int in_level_of_pyramid,
	CKvSet2d_of_Point3Df *out_map_p3d)
//********************************************************************************************
{
	CKvYooji_Intrinsics *p_intrin_rgb;
	CKvPointf t_pix, t_pix_rgb;
	CKvPoint3Df *p_map_p3d;
	CKvPoint3Df zero_p3d;	zero_p3d.s_Set(0.0f, 0.0f, 0.0f);

	float *p_depth_t0;

	float inten_t0, td_rgb_t0;
	bool flag_valid;
	int ww, hh, ww_rgb, hh_rgb, cnt, x, y;

	// get pointers.	
	p_depth_t0 = in_state->p_pyramid_map_depth()->imgs[in_level_of_pyramid].mps(ww, hh)[0];
	p_intrin_rgb = &in_state->p_pyramid_intrinsics()->intrins[in_level_of_pyramid];

	if(out_map_p3d->mw() != ww || out_map_p3d->mh() !=hh) out_map_p3d->c_Create(hh, ww);

	p_map_p3d = out_map_p3d->vp();

	cnt=0;
	for(y=0; y<hh; y++){
		for(x=0; x<ww; x++){

			int tidx = y*ww + x;

			p_map_p3d[tidx] = zero_p3d;
			
			if(p_depth_t0[tidx] <= 0.0f) continue;
			
			t_pix.x=x;	t_pix.y=y;

			td_rgb_t0 = p_depth_t0[tidx];

			// compute 3d point back-projected from the global model and previous camera pose in the global coordinates.
			p_intrin_rgb->back_project(t_pix,td_rgb_t0,zz_p3d_t0);				// 2D cam -> 3D cam t0.
			z_tp3d_Transform_Point3D(in_hmat_t0_to_t1_est,zz_p3d_t0,zz_p3d_t1);	// 3D cam t0 -> t1.

			p_map_p3d[tidx] = zz_p3d_t1;

			//printf("p_map_p3d[%d]: %f %f %f\n", tidx, p_map_p3d[tidx].x, p_map_p3d[tidx].y, p_map_p3d[tidx].z);
		}
	}
	
	return cnt;
}

//********************************************************************************************
int LCKvYooji_Camera_Tracker::cp3dm_Compute_Point3D_Map(
	CKvYooji_Tracking_State *in_state,
	CKvMatrix *in_hmat_t0_to_t1_est,
	int in_level_of_pyramid,
	CKvSet2d_of_Point3Df *out_map_p3d)
//********************************************************************************************
{
	CKvYooji_Intrinsics *p_intrin_rgb;
	CKvPointf t_pix, t_pix_rgb;
	CKvPoint3Df *p_map_p3d;
	CKvPoint3Df zero_p3d;

	float *p_depth_t0;

	float inten_t0, td_rgb_t0;
	bool flag_valid;
	int ww, hh, ww_rgb, hh_rgb, cnt, x, y;

	// get pointers.	
	p_depth_t0 = in_state->p_pyramid_map_depth()->imgs[in_level_of_pyramid].mps(ww, hh)[0];
	p_intrin_rgb = &in_state->p_pyramid_intrinsics()->intrins[in_level_of_pyramid];

	if(out_map_p3d->mw() != ww || out_map_p3d->mh() !=hh) out_map_p3d->c_Create(hh, ww);

	p_map_p3d = out_map_p3d->vp();

	cnt=0;
	for(y=0; y<hh; y++){
		for(x=0; x<ww; x++){

			int tidx = y*ww + x;

			p_map_p3d[tidx] = zero_p3d;
			
			if(p_depth_t0[tidx] <= 0.0f) continue;
			
			t_pix.x=x;	t_pix.y=y;

			td_rgb_t0 = p_depth_t0[tidx];

			// compute 3d point back-projected from the global model and previous camera pose in the global coordinates.
			p_intrin_rgb->back_project(t_pix,td_rgb_t0,zz_p3d_t0);				// 2D cam -> 3D cam t0.
			z_tp3d_Transform_Point3D(in_hmat_t0_to_t1_est,zz_p3d_t0,zz_p3d_t1);	// 3D cam t0 -> t1.

			p_map_p3d[tidx] = zz_p3d_t1;

			//printf("p_map_p3d[%d]: %f %f %f\n", tidx, p_map_p3d[tidx].x, p_map_p3d[tidx].y, p_map_p3d[tidx].z);
		}
	}
	
	return cnt;
}

//////////////////////////////////////////////////////////////////////////
//********************************************************************************************
int LCKvYooji_Camera_Tracker::cp3dm_Compute_Point3D_Map(
	CKvSet2d_of_Point3Df *in_map_p3d_src,
	CKvMatrixFloat *in_hmat_src_to_dst,
	CKvMatrixBool *in_map_valid,
	CKvSet2d_of_Point3Df *out_map_p3d_dst)
//********************************************************************************************
{
	CKvPoint3Df *p_p3d_t0, *p_p3d_t0_trans;

	bool *p_valid;

	bool flag_valid;
	int ww, hh, cnt, x, y;
	
	// get pointers.	
	p_valid = in_map_valid->mps(ww, hh)[0];
	p_p3d_t0 = in_map_p3d_src->vp();
	
	if(out_map_p3d_dst->mw() != ww || out_map_p3d_dst->mh() !=hh) out_map_p3d_dst->c_Create(hh, ww);

	p_p3d_t0_trans = out_map_p3d_dst->vp();

	cnt=0;
	for(y=0; y<hh; y++){
		for(x=0; x<ww; x++){

			int tidx = y*ww + x;

			if(!p_valid[tidx])	continue;
			
			// compute 3d point back-projected from the global model and previous camera pose in the global coordinates.
			z_tp3d_Transform_Point3D(in_hmat_src_to_dst,p_p3d_t0[tidx],p_p3d_t0_trans[tidx]);	// 3D cam t0 -> t1.
	
			//printf("p_map_p3d[%d]: %f %f %f\n", tidx, p_map_p3d[tidx].x, p_map_p3d[tidx].y, p_map_p3d[tidx].z);
		}
	}
	
	return cnt;
}

//////////////////////////////////////////////////////////////////////////

//********************************************************************************************
bool LCKvYooji_Camera_Tracker::sls_Solve_Linear_System(CKvMatrixFloat *in_ATA,
	CKvVectorFloat *in_ATb,
	CKvVectorFloat *out_x)
//********************************************************************************************
{

	//return aa_lef.sle_Solve_Linear_Equation(Kv2008_ALGEBRA_MODE_SVD, in_ATA, in_ATb, out_x);
	//return aa_lef.slu_Solve_based_on_LUD(in_ATA, in_ATb, out_x);
	return aa_lef.sle_Solve_Linear_Equation(Kv2008_ALGEBRA_MODE_LLD, in_ATA, in_ATb, out_x);
}

//********************************************************************************************
template<typename T>
void LCKvYooji_Camera_Tracker::z_uls_Update_Linear_System(float in_A[],
	T in_b,
	T *io_ATA,
	T *io_ATb)
//********************************************************************************************
{
	// update ATA.
	for(int j=0; j<6; j++){	for(int i=0; i<6; i++){		io_ATA[j*6+i]+=in_A[j]*in_A[i];		}}
	// update ATb.
	for(int i=0; i<6; i++)	io_ATb[i]+=in_A[i]*in_b;

}

//********************************************************************************************
template<typename T>
void LCKvYooji_Camera_Tracker::z_ulsw_Update_Linear_System_Weighted(T in_A[],
	T in_b,
	T in_weight,
	T *io_ATA,
	T *io_ATb)
//********************************************************************************************
{
	//static T A[6];	for(int i=0; i<6; i++)	A[i] = in_A[i];

	// update ATA.
	for(int j=0; j<6; j++){	for(int i=0; i<6; i++){		io_ATA[j*6+i]+=in_weight*in_A[j]*in_A[i];		}}
	// update ATb.
	for(int i=0; i<6; i++)	io_ATb[i]+=in_weight*in_A[i]*in_b;

}


//********************************************************************************************
// For KinectFusion.
bool LCKvYooji_Camera_Tracker::uitcg_Update_Incremental_Tracking_Cam_to_Glob_with_Backward_Params(CKvVectorFloat *in_x,
	CKvMatrixFloat *io_hmat_4x4)
//********************************************************************************************
{
	float *p_mat, *p_x, tm[16];
	/// CAUTION
	// in_x is 6-d vector.	[ r1, r2, r3, t1, t2, t3 ]	
	// + this transformation Tg,k transforms the k-th camera coordinates to global model coordinates.
	//        |	1	-r3	r2	t1	|
	// Tg,k = |	r3	1	-r1	t2	|
	//        |	-r2	r1	1	t3	|
	

	p_x=in_x->vp();
	p_mat=io_hmat_4x4->vp();		// 4x4 matrix.

	for(int i=0; i<16; i++)	tm[i]=p_mat[i];
	
	for(int i = 0; i<4; i++)	p_mat[i] = +1.0f	 *tm[i]	-p_x[2]	*tm[i+4]	+p_x[1]	*tm[i+8]	+p_x[3]*tm[i+12];
	for(int i = 0; i<4; i++)	p_mat[i+4] = +p_x[2] *tm[i]	+1.0f	*tm[i+4]	-p_x[0]	*tm[i+8]	+p_x[4]*tm[i+12];
	for(int i = 0; i<4; i++)	p_mat[i+8] = -p_x[1] *tm[i]	+p_x[0]	*tm[i+4]	+1.0f	*tm[i+8]	+p_x[5]*tm[i+12];



	return true;
}

//********************************************************************************************
// For KinectFusion.
bool LCKvYooji_Camera_Tracker::uitcg_Update_Incremental_Tracking_t1_to_t0_with_Backward_Parameters(CKvVectorFloat *in_x,
	CKvMatrixFloat *io_hmat_t1_to_t0_est)
//********************************************************************************************
{
	float *p_mat_10,*p_x,tm[16],xm[16];
	/// CAUTION
	// in_x is 6-d vector.	[ r1, r2, r3, t1, t2, t3 ]	
	// + this transformation Tg,k transforms the k-th camera coordinates to global model coordinates.
	//        |	1	-r3	r2	t1	|
	// Tg,k = |	r3	1	-r1	t2	|
	//        |	-r2	r1	1	t3	|


	p_x=in_x->vp();

	d_cme_Compute_Matrix_Exponential(p_x, xm);

// 	xm[0] = +1.0f;		xm[1] =-p_x[2];		xm[2] = +p_x[1];	xm[3] = +p_x[3];
// 	xm[4] = +p_x[2];	xm[5] = +1.0f;		xm[6] = -p_x[0];	xm[7] = +p_x[4];
// 	xm[8] = -p_x[1];	xm[9] = +p_x[0];	xm[10] = +1.0f;		xm[11] = +p_x[5];
// 	xm[12] = 0.0f;		xm[13] = 0.0f;		xm[14] = 0.0f;		xm[15] = 1.0f;

	// update transformation matrix t1 to t0.
	p_mat_10=io_hmat_t1_to_t0_est->vp();		// 4x4 matrix.
	for(int i=0; i<16; i++)	tm[i]=p_mat_10[i];
	d_mms_Multiply_Matrix_Square(xm,tm,4,p_mat_10);

	return true;
}

//********************************************************************************************
// For KinectFusion.
bool LCKvYooji_Camera_Tracker::umf_Update_Motion_Forward(CKvVectorFloat *in_x,
	CKvMatrixFloat *io_hmat_t1_to_t0_est,
	bool in_mode_param_forward)
//********************************************************************************************
{
	float *p_mat_10, *p_mat_cg, *p_x, tm[16], xm[16], xm_inv[16];
	/// CAUTION
	// in_x is 6-d vector.	[ r1, r2, r3, t1, t2, t3 ]	
	// + this transformation Tg,k transforms the k-th camera coordinates to global model coordinates.
	//        |	1	-r3	r2	t1	|
	// Tg,k = |	r3	1	-r1	t2	|
	//        |	-r2	r1	1	t3	|

	p_x=in_x->vp();	

	p_mat_10=io_hmat_t1_to_t0_est->vp();		// 4x4 matrix.
	for(int i=0; i<16; i++)	tm[i]=p_mat_10[i];

	d_cme_Compute_Matrix_Exponential(p_x,xm);

	////////////////////////////////////////////////////////////////////////
// 	Eigen::Matrix4f mat_xsi,mat_exp;
// 
// 	mat_xsi << 0.0f,-p_x[2],+p_x[1],+p_x[3],
// 			+p_x[2],0.0f,-p_x[0],+p_x[4],
// 			-p_x[1],+p_x[0],0.0f,+p_x[5],
// 			0.0f,0.0f,0.0f,0.0f;
// 
// 	mat_exp = mat_xsi.exp();
// 
// 	for(int j=0; j<4; j++) for(int i=0; i<4; i++) xm[j*4 + i] = mat_exp.coeff(j,i);
	////////////////////////////////////////////////////////////////////////

	if(in_mode_param_forward)	d_mms_Multiply_Matrix_Square(xm,tm,4,p_mat_10);
	else{
		d_im_Inverse_Matrix_4x4(xm,xm_inv);
		d_mms_Multiply_Matrix_Square(xm_inv,tm,4,p_mat_10);
	}


	return true;
}

//********************************************************************************************
// For KinectFusion.
bool LCKvYooji_Camera_Tracker::umb_Update_Motion_Backward(CKvVectorFloat *in_x,
	CKvMatrixFloat *io_hmat_t1_to_t0_est,
	bool in_mode_param_forward)
//********************************************************************************************
{
	float *p_mat_10, *p_mat_cg, *p_x, tm[16], xm[16], xm_inv[16];
	/// CAUTION
	// in_x is 6-d vector.	[ r1, r2, r3, t1, t2, t3 ]	
	// + this transformation Tg,k transforms the k-th camera coordinates to global model coordinates.
	//        |	1	-r3	r2	t1	|
	// Tg,k = |	r3	1	-r1	t2	|
	//        |	-r2	r1	1	t3	|
	

	p_x=in_x->vp();	

	p_mat_10=io_hmat_t1_to_t0_est->vp();		// 4x4 matrix.
	for(int i=0; i<16; i++)	tm[i]=p_mat_10[i];

	d_cme_Compute_Matrix_Exponential(p_x,xm);

	//////////////////////////////////////////////////////////////////////////
// 	Eigen::Matrix4f mat_xsi, mat_exp;
// 
// 	mat_xsi << 0.0f,-p_x[2],+p_x[1],+p_x[3],
// 		   +p_x[2],0.0f,-p_x[0],+p_x[4],
// 		   -p_x[1],+p_x[0],0.0f,+p_x[5],
// 		   0.0f,0.0f,0.0f,0.0f;
// 
// 	mat_exp = mat_xsi.exp();
// 
// 	for(int j=0; j<4; j++) for(int i=0; i<4; i++) xm[j*4 + i] = mat_exp.coeff(j,i);
	//////////////////////////////////////////////////////////////////////////

	if(in_mode_param_forward){	
		d_im_Inverse_Matrix_4x4(xm, xm_inv);
		d_mms_Multiply_Matrix_Square(xm_inv, tm, 4, p_mat_10);
	}
	else d_mms_Multiply_Matrix_Square(xm,tm,4,p_mat_10);


	return true;
}



//********************************************************************************************
bool LCKvYooji_Camera_Tracker::cmv_Check_Motion_Validity(
	CKvMatrixFloat *io_hmat_4x4,
	float in_max_translation,	// m
	float in_max_rotation)		// rad.
//********************************************************************************************
{
	Mat Rt, rvec;
	aa_ilib.cfko_Convert_Format_from_KAISION_to_Opencv(*io_hmat_4x4, Rt);

	Rodrigues(Rt(Rect(0, 0, 3, 3)), rvec);

	double translation = norm(Rt(Rect(3,0,1,3)));
	double rotation = norm(rvec) * 180. / CV_PI;

	//printf("trans: %f rot: %f\n", translation, rotation);

	return translation <= in_max_translation && rotation <= in_max_rotation;
}

//********************************************************************************************
bool LCKvYooji_Camera_Tracker::cmv_Check_Motion_Validity(
	CKvMatrix *io_hmat_4x4,
	float in_max_translation,	// m
	float in_max_rotation)		// rad.
//********************************************************************************************
{
	Mat Rt, rvec;
	aa_ilib.cfko_Convert_Format_from_KAISION_to_Opencv(*io_hmat_4x4, Rt);

	Rodrigues(Rt(Rect(0, 0, 3, 3)), rvec);

	double translation = norm(Rt(Rect(3,0,1,3)));
	double rotation = norm(rvec) * 180. / CV_PI;

	return translation <= in_max_translation && rotation <= in_max_rotation;
}

//********************************************************************************************
bool LCKvYooji_Camera_Tracker::cmv_Check_Cube_Position(
	CKvYooji_Cube_TSDF_Float *in_cube,
	CKvYooji_Tracking_State *in_state)
//********************************************************************************************
{
	bool flag = true;
	CKvPoint3Df cen_cube, tp3d;
	CKvPointf tp2d;

	int ww, hh;
	
	in_state->p_image_RGB()->ms(ww,hh);

	in_cube->gcciw_Get_Cube_Center_In_World(cen_cube);
	in_state->p_extrinsics_glob_to_cam_RGB()->transform(cen_cube,tp3d);
	in_state->p_intrinsics_RGB()->project(tp3d,tp2d);

	if(tp2d.x > ww || tp2d.x < 0
		|| tp2d.y >hh || tp2d.y < 0) flag = false;

	return flag;
}


//********************************************************************************************
bool LCKvYooji_Camera_Tracker::z_ss_Single_Summand_ICP_Backward(
	CKvPixel &in_xy,	
	float in_depth_view,
	float *in_depth_map_model,
	CKvSet2d_of_Point3Df *in_p3d_model,
	CKvSet2d_of_Point3Df *in_normal_map_model,
	int in_ww, int in_hh,
	CKvYooji_Intrinsics *in_K,
	CKvMatrixFloat *in_hmat_t1_to_t0_estimated,
	CKvYooji_Extrinsics *in_pose_prev,
	float in_th_dist_ICP,
	float out_A[],
	float &out_b,
	bool in_mode_param_forward)
//********************************************************************************************
{
	if(in_depth_view<=0.0f)	return false;

	float td;

	// compute 3d point back-projected from current estimated pose in the global coordinates.
	// + [p3d_t1]
	in_K->back_project(in_xy,in_depth_view,zz_tp3d);								// 2D cam t1->3D cam t1
	z_tp3d_Transform_Point3D(in_hmat_t1_to_t0_estimated,zz_tp3d,zz_p3d_t1);		// 3D cam t1->3D cam t0	

	// + check depth validity.
	//if(!z_gid_Get_Interpolated_Depth(zz_p2d_pred.x, zz_p2d_pred.y, in_ww, in_hh, in_depth_map_model, td))	return false;
	//in_K->bp_Back_Project(zz_p2d_pred, td, zz_tp3d);												// 2D cam->3D cam
	// + [p3d_t0]
	in_K->project(zz_p3d_t1,zz_p2d_t0);			// 3D cam t0->2D cam t0
	z_gip3d_Get_Interpolated_Point3D(zz_p2d_t0.x,zz_p2d_t0.y,in_ww,in_hh,
		in_p3d_model,zz_p3d_t0);					// 2D cam t0->3D glob t0
	
	// check squared distance between [p3d_proj] and [p3d_pred].
	td=zz_p3d_t1.ds_Distance_Squared(zz_p3d_t0);	if(td>in_th_dist_ICP)	return false;

	// check correlation between normals of [p3d_proj] and [p3d_pred].
	// + compute normal of [p3d_proj].
	// + compute normal of [p3d_pred].
	if(!z_gin_Get_Interpolated_Normal(zz_p2d_t0.x, zz_p2d_t0.y, in_ww, in_hh, in_normal_map_model, zz_norm_pred))	return false;

	//printf("normal: %f %f %f\n", zz_norm_pred.x, zz_norm_pred.y, zz_norm_pred.z);

	// + compute A.	
	if(in_mode_param_forward){
		out_A[0]= +zz_p3d_t1.z*zz_norm_pred.y -zz_p3d_t1.y*zz_norm_pred.z;
		out_A[1]= -zz_p3d_t1.z*zz_norm_pred.x +zz_p3d_t1.x*zz_norm_pred.z;
		out_A[2]= +zz_p3d_t1.y*zz_norm_pred.x -zz_p3d_t1.x*zz_norm_pred.y;
		out_A[3]= -zz_norm_pred.x;	out_A[4]= -zz_norm_pred.y;	out_A[5]= -zz_norm_pred.z;
	}
	else{
		out_A[0]= -zz_p3d_t1.z*zz_norm_pred.y +zz_p3d_t1.y*zz_norm_pred.z;
		out_A[1]= +zz_p3d_t1.z*zz_norm_pred.x -zz_p3d_t1.x*zz_norm_pred.z;
		out_A[2]= -zz_p3d_t1.y*zz_norm_pred.x +zz_p3d_t1.x*zz_norm_pred.y;
		out_A[3]= zz_norm_pred.x;	out_A[4]= zz_norm_pred.y;	out_A[5]= zz_norm_pred.z;
	}

	// + compute b.
	out_b= zz_norm_pred.x*(zz_p3d_t0.x-zz_p3d_t1.x)
		+zz_norm_pred.y*(zz_p3d_t0.y-zz_p3d_t1.y)
		+zz_norm_pred.z*(zz_p3d_t0.z-zz_p3d_t1.z);
	

	return true;
}

//********************************************************************************************
bool LCKvYooji_Camera_Tracker::z_ss_Single_Summand_Photo_Max_Forward(
	CKvPointf &in_p2d_t0,
	const float in_inten_t0,
	const float in_depth_t0,
	const bool *in_sil_t1,
	const unsigned char *in_img_gray_t1,
	const float *in_grad_x_t1,
	const float *in_grad_y_t1,
	const int in_ww,const int in_hh,
	CKvYooji_Intrinsics *in_K,
	CKvMatrixFloat *in_pose_t0_to_t1_estimated,
	float out_A[],
	float &out_b)
//********************************************************************************************
{
	// V0 = zz_p3d_t0.
	// T(V0) = zz_p3d_t1.
	// pi(T(V0)) = zz_p2d_t1.
	if(in_depth_t0==0.0f)	return false;

	float td, inten_t1, g3_inv;
	float fx, fy, px, py, dmin, dmax;
	float grad[2], dpi_dT[2*3], tA[3], V0[3], V1[3], Jgrad_f[2], Jpi_T[2*6];

	// compute 3d point back-projected from the global model and previous camera pose in the global coordinates.
	in_K->get_params(fx, fy, px, py, dmin, dmax);
	// + V1 = T(V0).
	in_K->back_project(in_p2d_t0, in_depth_t0, zz_p3d_t0);						// 2D cam -> 3D cam t0.
	z_tp3d_Transform_Point3D(in_pose_t0_to_t1_estimated, zz_p3d_t0, zz_p3d_t1);		// 3D cam t0 -> t1.

	in_K->project(zz_p3d_t1, zz_p2d_t1);

	V0[0] = zz_p3d_t0.x;	V0[1] = zz_p3d_t0.y;	V0[2] = zz_p3d_t0.z;
	//V1[0] = zz_p3d_t1.x;	V1[1] = zz_p3d_t1.y;	V1[2] = zz_p3d_t1.z;
	V1[0] = zz_p3d_t0.x;	V1[1] = zz_p3d_t0.y;	V1[2] = zz_p3d_t0.z;

	////// 2..............................................................................
  	//// + I(pi(T(V^z-1)),t1). 	
  	//if(!in_sil_t1[int(zz_p2d_t1.y)*in_ww+ int(zz_p2d_t1.x)])	return false;
  	if(!z_gii_Get_Interpolated_Intensity(zz_p2d_t1.x, zz_p2d_t1.y, in_ww, in_hh, (unsigned char*)in_img_gray_t1, inten_t1))	return false;

  	// + del(I)(u,t0).	
	//z_gig_Get_Interpolated_Gradient(zz_p2d_t1.x, zz_p2d_t1.y, in_ww, in_hh, (float*)in_grad_x_t1, grad[0]);
	//z_gig_Get_Interpolated_Gradient(zz_p2d_t1.x, zz_p2d_t1.y, in_ww, in_hh, (float*)in_grad_y_t1, grad[1]);

	z_gig_Get_Interpolated_Gradient(in_p2d_t0.x,in_p2d_t0.y,in_ww,in_hh,(float*)in_grad_x_t1,grad[0]);
	z_gig_Get_Interpolated_Gradient(in_p2d_t0.x,in_p2d_t0.y,in_ww,in_hh,(float*)in_grad_y_t1,grad[1]);


	//if(abs(grad[0]) <= 0 && abs(grad[1]) <= 0) return false;
	//if(SQUARE(grad[0]) + SQUARE(grad[1]) > 2500) return false;

	// Jacobian of gradient and intrinsics. (1x2 matrix)
	Jgrad_f[0] = grad[0]*fx;
	Jgrad_f[1] = grad[1]*fy;

	//printf("grad: %f %f\n", grad[0], grad[1]);

	// Jacobian of projection and transformation. (2x6 matrix)
	g3_inv = 1.0f/V1[2];
	// 1st row.
	//Jpi_T[0] = -V1[0]*V1[1]*SQUARE(g3_inv); 
	Jpi_T[0] = -V1[0]*V1[1]*g3_inv;
	Jpi_T[1] = 1.0f + SQUARE(V1[0]*g3_inv);
	Jpi_T[2] = -V1[1]*g3_inv;
	Jpi_T[3] = g3_inv; Jpi_T[4] = 0.0f; Jpi_T[5] = -V1[0]*SQUARE(g3_inv);
	// 2nd row.
	Jpi_T[6] = -1.0f - SQUARE(V1[1]*g3_inv);
	//Jpi_T[7] = V1[0]*V1[1]*SQUARE(g3_inv); 
	Jpi_T[7] = V1[0]*V1[1]*g3_inv;
	Jpi_T[8] = V1[0]*g3_inv;
	Jpi_T[9] = 0.0f; Jpi_T[10] = g3_inv; Jpi_T[11] = -V1[1]*SQUARE(g3_inv);

	// Compute A matrix for normal equation.
	d_mmm_Multiply_Matrix_Matrix(Jgrad_f, Jpi_T, 1, 2, 6, out_A);

	// Compute residual b for normal equation.
	out_b = -(inten_t1 - in_inten_t0);

	return true;
}

//********************************************************************************************
template<typename T>
bool LCKvYooji_Camera_Tracker::z_cjpt_Compute_Jacobians_of_Projection_and_Transformation(
	CKvPoint3Df &in_p3d_t0,
	T **out_J)	// 2x6 matrix.
//********************************************************************************************
{
	// A = [cY-bZ aZ-cX bX-aY a b c]
	// a = Ix*fx/Z, b = Iy*fy/Z, c = -(a*X+b*Y)/Z
	T x = in_p3d_t0.x, y = in_p3d_t0.y, z = in_p3d_t0.z;	
	T invz = 1.0f/z;
	T a, b, c;

	a = x*invz;	b = y*invz;
	
	out_J[0][0] = -x*y*SQUARE(invz);	//out_J[0][0] = -x*y*invz;
	out_J[0][1] = 1.0f + SQUARE(x*invz);
	out_J[0][2] = -y*invz;
	out_J[0][3] = invz;
	out_J[0][4] = 0.0f;
	out_J[0][5] = -x * SQUARE(invz);

	out_J[1][0] = -1.0f - SQUARE(y*invz);
	out_J[1][1] = x*y*SQUARE(invz);	//out_J[1][1] = x*y*invz;
	out_J[1][2] = x*invz;
	out_J[1][3] = 0.0f;
	out_J[1][4] = invz;
	out_J[1][5] = -y * SQUARE(invz);

	return true;
}

//********************************************************************************************
template<typename T>
bool LCKvYooji_Camera_Tracker::z_ccpm_Compute_Coefficients_with_Photo_Max_Yooji(
	T **in_J_pit,
	const float in_Ix, const float in_Iy,
	const float in_fx, const float in_fy,
	T out_A[])
//********************************************************************************************
{
	T grad[2];
	grad[0] = in_Ix*in_fx; grad[1] = in_Iy*in_fy;

	d_mmm_Multiply_Matrix_Matrix(grad, in_J_pit[0], 1, 2, 6, out_A);

	return true;
}

//********************************************************************************************
bool LCKvYooji_Camera_Tracker::z_gii_Get_Interpolated_Intensity(float in_x, float in_y,
	int in_ww, int in_hh,
	unsigned char *in_img_gray,
	float &out_intensity)
//********************************************************************************************
{
	// d1   d2
	//    x
	// d3   d4
	float d1, d2, d3, d4;
	int x, y;
	float resi_x, resi_y;

	if(in_x<0.0f || in_x>float(in_ww-1) ||in_y<0.0f || in_y>float(in_hh-1))	return false;

	d2=d3=d4=0.0f;

	x=(int)in_x;	y=(int)in_y;	resi_x=in_x-(float)x;	resi_y=in_y-(float)y;
	d1=(float)in_img_gray[y*in_ww+x];
	if(resi_x>0.0f){					d2=(float)in_img_gray[y*in_ww+x+1];		}
	if(resi_y>0.0f){					d3=(float)in_img_gray[(y+1)*in_ww+x];	}
	if(resi_x>0.0f && resi_y>0.0f){		d4=(float)in_img_gray[(y+1)*in_ww+x+1];	}

	out_intensity=(1.0f-resi_y)*( (1.0f-resi_x)*d1+resi_x*d2 ) + resi_y*( (1.0f-resi_x)*d3+resi_x*d4 );	

	return true;
}

//********************************************************************************************
bool LCKvYooji_Camera_Tracker::z_gim_Get_Interpolated_Magnitude(float in_x, float in_y,
	int in_ww, int in_hh,
	float *in_img_mag,
	float &out_intensity)
//********************************************************************************************
{
	// d1   d2
	//    x
	// d3   d4
	float d1, d2, d3, d4;
	int x, y;
	float resi_x, resi_y;

	if(in_x<0.0f || in_x>float(in_ww-1) ||in_y<0.0f || in_y>float(in_hh-1))	return false;

	d2=d3=d4=0.0f;

	x=(int)in_x;	y=(int)in_y;	resi_x=in_x-(float)x;	resi_y=in_y-(float)y;
	d1=in_img_mag[y*in_ww+x];
	if(resi_x>0.0f){					d2=in_img_mag[y*in_ww+x+1];		}
	if(resi_y>0.0f){					d3=in_img_mag[(y+1)*in_ww+x];	}
	if(resi_x>0.0f && resi_y>0.0f){		d4=in_img_mag[(y+1)*in_ww+x+1];	}

	out_intensity=(1.0f-resi_y)*( (1.0f-resi_x)*d1+resi_x*d2 ) + resi_y*( (1.0f-resi_x)*d3+resi_x*d4 );	

	return true;
}

//********************************************************************************************
bool LCKvYooji_Camera_Tracker::z_gig_Get_Interpolated_Gradient(float in_x, float in_y,
	int in_ww, int in_hh,
	float *in_map_grad,
	float &out_grad)
//********************************************************************************************
{
	// d1   d2
	//    x
	// d3   d4
	float d1, d2, d3, d4;
	int x, y;
	float resi_x, resi_y;

	out_grad = 0.0f;

	if(in_x<0.0f || in_x>float(in_ww-1) ||in_y<0.0f || in_y>float(in_hh-1))	return false;

	d2=d3=d4=0.0f;

	x=(int)in_x;	y=(int)in_y;	resi_x=in_x-(float)x;	resi_y=in_y-(float)y;
	d1=(float)in_map_grad[y*in_ww+x];
	if(resi_x>0.0f){					d2=in_map_grad[y*in_ww+x+1];		}
	if(resi_y>0.0f){					d3=in_map_grad[(y+1)*in_ww+x];	}
	if(resi_x>0.0f && resi_y>0.0f){		d4=in_map_grad[(y+1)*in_ww+x+1];	}

	out_grad=(1.0f-resi_y)*( (1.0f-resi_x)*d1+resi_x*d2 ) + resi_y*( (1.0f-resi_x)*d3+resi_x*d4 );	

	return true;
}

//********************************************************************************************
bool LCKvYooji_Camera_Tracker::z_gig_Get_Interpolated_Gradient(float in_x, float in_y,
	int in_ww, int in_hh,
	short *in_map_grad,
	float &out_grad)
//********************************************************************************************
{
	// d1   d2
	//    x
	// d3   d4
	float d1, d2, d3, d4;
	int x, y;
	float resi_x, resi_y;

	if(in_x<0.0f || in_x>float(in_ww-1) ||in_y<0.0f || in_y>float(in_hh-1))	return false;

	d2=d3=d4=0.0f;

	x=(int)in_x;	y=(int)in_y;	resi_x=in_x-(float)x;	resi_y=in_y-(float)y;
	d1=(float)in_map_grad[y*in_ww+x];
	if(resi_x>0.0f){					d2=(float)in_map_grad[y*in_ww+x+1];		}
	if(resi_y>0.0f){					d3=(float)in_map_grad[(y+1)*in_ww+x];	}
	if(resi_x>0.0f && resi_y>0.0f){		d4=(float)in_map_grad[(y+1)*in_ww+x+1];	}

	out_grad=(1.0f-resi_y)*( (1.0f-resi_x)*d1+resi_x*d2 ) + resi_y*( (1.0f-resi_x)*d3+resi_x*d4 );	

	return true;
}

//********************************************************************************************
bool LCKvYooji_Camera_Tracker::z_gid_Get_Interpolated_Depth(float in_x, float in_y,
	int in_ww, int in_hh,
	float *in_depth_map,
	float &out_depth)
//********************************************************************************************
{
	// d1   d2
	//    x
	// d3   d4
	float d1, d2, d3, d4;
	int x, y;
	float resi_x, resi_y;

	if(in_x<0.0f || in_x>float(in_ww-1) ||in_y<0.0f || in_y>float(in_hh-1))	return false;

	out_depth = d2=d3=d4=0.0f;

	x=(int)in_x;	y=(int)in_y;	resi_x=in_x-(float)x;	resi_y=in_y-(float)y;
	d1=in_depth_map[y*in_ww+x];			if(d1<=0.0f)	return false;
	if(resi_x>0.0f){					d2=in_depth_map[y*in_ww+x+1];		if(d2<=0.0f)	return false;	}
	if(resi_y>0.0f){					d3=in_depth_map[(y+1)*in_ww+x];		if(d3<=0.0f)	return false;	}
	if(resi_x>0.0f && resi_y>0.0f){		d4=in_depth_map[(y+1)*in_ww+x+1];	if(d4<=0.0f)	return false;	}

	out_depth=(1.0f-resi_y)*( (1.0f-resi_x)*d1+resi_x*d2 ) + resi_y*( (1.0f-resi_x)*d3+resi_x*d4 );	

	return true;
}

//********************************************************************************************
bool LCKvYooji_Camera_Tracker::z_gip3d_Get_Interpolated_Point3D(float in_x,float in_y,
	int in_ww,int in_hh,
	CKvSet2d_of_Point3Df *in_p3d_map,
	CKvPoint3Df &out_p3d)
//********************************************************************************************
{
	// n1   n2
	//    x
	// n3   n4	
	CKvPoint3Df *p_p3d;
	int x, y;
	float resi_x, resi_y;

	if(in_x<0.0f || in_x>float(in_ww-1) ||in_y<0.0f || in_y>float(in_hh-1))	return false;

	x=(int)in_x;	y=(int)in_y;	resi_x=in_x-(float)x;	resi_y=in_y-(float)y;

	//zz_n2.s_Set(0.0f, 0.0f, 0.0f);	zz_n3.s_Set(0.0f, 0.0f, 0.0f);	zz_n4.s_Set(0.0f, 0.0f, 0.0f);

	// default value of invalid normal is (-100.0f, -100.0f, -100.0f). 
	p_p3d = in_p3d_map->vp();
	zz_n1=p_p3d[y*in_ww+x];	if(zz_n1.z<=0.0f)	return false;									
	if(resi_x>0.0f){				zz_n2=p_p3d[y*in_ww+x+1];		if(zz_n2.z<=0.0f)	return false;	}
	if(resi_y>0.0f){				zz_n3=p_p3d[(y+1)*in_ww+x];		if(zz_n3.z<=0.0f)	return false;	}
	if(resi_x>0.0f && resi_y>0.0f){	zz_n4=p_p3d[(y+1)*in_ww+x+1];	if(zz_n4.z<=0.0f)	return false;	}

	out_p3d.x=(1.0f-resi_y)*( (1.0f-resi_x)*zz_n1.x+resi_x*zz_n2.x ) + resi_y*( (1.0f-resi_x)*zz_n3.x+resi_x*zz_n4.x );	
	out_p3d.y=(1.0f-resi_y)*( (1.0f-resi_x)*zz_n1.y+resi_x*zz_n2.y ) + resi_y*( (1.0f-resi_x)*zz_n3.y+resi_x*zz_n4.y );	
	out_p3d.z=(1.0f-resi_y)*( (1.0f-resi_x)*zz_n1.z+resi_x*zz_n2.z ) + resi_y*( (1.0f-resi_x)*zz_n3.z+resi_x*zz_n4.z );	

	return true;
}

//********************************************************************************************
bool LCKvYooji_Camera_Tracker::z_gin_Get_Interpolated_Normal(float in_x, float in_y,
	int in_ww, int in_hh,
	CKvSet2d_of_Point3Df *in_normal_map,
	CKvPoint3Df &out_norm)
//********************************************************************************************
{
	// n1   n2
	//    x
	// n3   n4	
	CKvPoint3Df *p_normals;
	int x, y;
	float resi_x, resi_y;

	if(in_x<0.0f || in_x>float(in_ww-1) ||in_y<0.0f || in_y>float(in_hh-1))	return false;

	x=(int)in_x;	y=(int)in_y;	resi_x=in_x-(float)x;	resi_y=in_y-(float)y;

	//zz_n2.s_Set(0.0f, 0.0f, 0.0f);	zz_n3.s_Set(0.0f, 0.0f, 0.0f);	zz_n4.s_Set(0.0f, 0.0f, 0.0f);

	// default value of invalid normal is (-100.0f, -100.0f, -100.0f). 
	p_normals = in_normal_map->vp();
	zz_n1=p_normals[y*in_ww+x];	if(zz_n1.x==-100.0f)	return false;									
	if(resi_x>0.0f){				zz_n2=p_normals[y*in_ww+x+1];		if(zz_n2.x==-100.0f)	return false;	}
	if(resi_y>0.0f){				zz_n3=p_normals[(y+1)*in_ww+x];		if(zz_n3.x==-100.0f)	return false;	}
	if(resi_x>0.0f && resi_y>0.0f){	zz_n4=p_normals[(y+1)*in_ww+x+1];	if(zz_n4.x==-100.0f)	return false;	}

	out_norm.x=(1.0f-resi_y)*( (1.0f-resi_x)*zz_n1.x+resi_x*zz_n2.x ) + resi_y*( (1.0f-resi_x)*zz_n3.x+resi_x*zz_n4.x );	
	out_norm.y=(1.0f-resi_y)*( (1.0f-resi_x)*zz_n1.y+resi_x*zz_n2.y ) + resi_y*( (1.0f-resi_x)*zz_n3.y+resi_x*zz_n4.y );	
	out_norm.z=(1.0f-resi_y)*( (1.0f-resi_x)*zz_n1.z+resi_x*zz_n2.z ) + resi_y*( (1.0f-resi_x)*zz_n3.z+resi_x*zz_n4.z );	

	out_norm.n_Normalize();

	return true;
}

//********************************************************************************************
bool LCKvYooji_Camera_Tracker::z_tp3d_Transform_Point3D(CKvMatrixFloat *in_mat,
	CKvPoint3Df &in_p3d,
	CKvPoint3Df &out_p3d)
//********************************************************************************************
{
	int mw, mh;
	float *p_mat;

	p_mat=in_mat->mps(mw, mh)[0];

	if(mw!=4 || mh!=4)	return false;
	
	out_p3d.x	=p_mat[0]*in_p3d.x		+p_mat[1]*in_p3d.y		+p_mat[2]*in_p3d.z		+p_mat[3];
	out_p3d.y	=p_mat[0+4*1]*in_p3d.x	+p_mat[1+4*1]*in_p3d.y	+p_mat[2+4*1]*in_p3d.z	+p_mat[3+4*1];
	out_p3d.z	=p_mat[0+4*2]*in_p3d.x	+p_mat[1+4*2]*in_p3d.y	+p_mat[2+4*2]*in_p3d.z	+p_mat[3+4*2];

	return true;
}

//********************************************************************************************
bool LCKvYooji_Camera_Tracker::z_tp3d_Transform_Point3D(CKvMatrix *in_mat,
	CKvPoint3Df &in_p3d,
	CKvPoint3Df &out_p3d)
//********************************************************************************************
{
	int mw, mh;
	double *p_mat;

	p_mat=in_mat->mps(mw, mh)[0];

	if(mw!=4 || mh!=4)	return false;
	
	out_p3d.x	=p_mat[0]*in_p3d.x		+p_mat[1]*in_p3d.y		+p_mat[2]*in_p3d.z		+p_mat[3];
	out_p3d.y	=p_mat[0+4*1]*in_p3d.x	+p_mat[1+4*1]*in_p3d.y	+p_mat[2+4*1]*in_p3d.z	+p_mat[3+4*1];
	out_p3d.z	=p_mat[0+4*2]*in_p3d.x	+p_mat[1+4*2]*in_p3d.y	+p_mat[2+4*2]*in_p3d.z	+p_mat[3+4*2];

	return true;
}

//********************************************************************************************
bool LCKvYooji_Camera_Tracker::z_tp3di_Transform_Point3D_Inverse(CKvMatrixFloat *in_mat,
	CKvPoint3Df &in_p3d,
	CKvPoint3Df &out_p3d)
//********************************************************************************************
{
	// X' = R'*(X-t)
	int mw, mh;
	float *p_mat;
	float x, y, z;

	p_mat=in_mat->mps(mw, mh)[0];

	if(mw!=4 || mh!=4)	return false;
	
	// X-t.
	x = in_p3d.x - p_mat[3];	y = in_p3d.y - p_mat[7];	z = in_p3d.z - p_mat[11];
	// R'*(X-t).
	out_p3d.x	=p_mat[0]*x	+p_mat[4]*y	+p_mat[8]*z;
	out_p3d.y	=p_mat[1]*x	+p_mat[5]*y	+p_mat[9]*z;
	out_p3d.z	=p_mat[2]*x	+p_mat[6]*y	+p_mat[10]*z;

	return true;
}