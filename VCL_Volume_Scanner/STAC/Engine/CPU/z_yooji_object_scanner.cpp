/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_object_scanning.cpp
/////////////////////////////////////////////////////////////////////////////////////////////

#include "../../../stdafx.h"
#include "../../_yooji_2016_object_scanning.h"

/////////////////////////////////////////////////////////////////////////////////////////////
// CKvYooji_3D_Object_Scanner 
/////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
CKvYooji_3D_Object_Scanner::CKvYooji_3D_Object_Scanner(void)
//********************************************************************************************
{
	zz_classname="CKvYooji_3D_Object_Scanner";

	pis_Pre_Initialize_Scanner();
}

//********************************************************************************************
CKvYooji_3D_Object_Scanner::~CKvYooji_3D_Object_Scanner(void)
//********************************************************************************************
{	
	rs_Release_Scanner();
}


//********************************************************************************************
void CKvYooji_3D_Object_Scanner::pis_Pre_Initialize_Scanner()
//********************************************************************************************
{
	zz_number_of_processed_frames = -1;

	zz_call_scan_to_shield = zz_call_shield_to_scan = zz_init_shield = zz_flag_occlusion_removal = false;

	zz_mat_4x4.ci_Create_Identity_matrix(4);

	//// for trace.
	//zz_trace=new vector<CKvYooji_Extrinsics*>;
	//zz_trace->clear();
	//	zz_state_trace.i_Initialize(640, 480);

	// for segmentation.
	zz_gmm_hand = NULL;

	g_T_cg_t0 = g_T_gc_t0 = NULL;
	g_T_cg_t1 = g_T_gc_t1 = NULL;

	// depth maps.
	g_img_color_t1_ = NULL;
	g_map_depth_t1_raw_ = g_map_depth_t1_filt_ = NULL;
	zz_map_depth = zz_pointer_p3d = NULL;

	// for error measurement.
	for(int i=0; i<5; i++) elapsed_t[i] = 0.0f;
	avg_t = 0.0f;
	cnt = 0;

	// for re-localization.
	zz_cover_prev = 0.0f;
	zz_flag_relocal = false;

	zz_integral_hist_ori=NULL;
	zz_set_of_templates.clear();


	//zz_trace = zz_trace_gt = NULL;
	zz_trace.clear(); zz_trace_gt.clear();

	zz_set_of_input_depth.c_Create(KV_MAX_NUM_INPUT_IMAGES);
	zz_set_of_input_rgb.c_Create(KV_MAX_NUM_INPUT_IMAGES);
	if(KV_FLAG_GROUND_REMOVAL) zz_set_of_input_sil.c_Create(KV_MAX_NUM_INPUT_IMAGES);
}

//********************************************************************************************
void CKvYooji_3D_Object_Scanner::rs_Release_Scanner()
//********************************************************************************************
{
	CKvPoint3Df origin = Kv_Point3Df(1.0f,1.0f,1.0f);

	zz_number_of_processed_frames = -1;
	zz_flag_integ = zz_flag_track = false;

	zz_call_scan_to_shield = zz_call_shield_to_scan = zz_init_shield = zz_flag_occlusion_removal = false;

	zz_mat_4x4.ci_Create_Identity_matrix(4);

	//////////////////////////////////////////////////////////////////////////
	// Host memory.
	//////////////////////////////////////////////////////////////////////////
	zz_cube.c_Create(origin, 4, 4, 4, 4, 1.0f);	// minimum dimension is 4.

	// for depth camera.	
	zz_state.initialize(1, 1);
	zz_rgbd_frame.p_image_silhouette()->c_Create(1,1,false);

	if(g_map_depth_t1_raw_) cudaFree(g_map_depth_t1_raw_);
	if(g_map_depth_t1_filt_) cudaFree(g_map_depth_t1_filt_);
	if(g_img_color_t1_) cudaFree(g_img_color_t1_);

	if(zz_pointer_p3d) delete[] zz_pointer_p3d; 
	if(zz_map_depth) delete[] zz_map_depth; 

	g_img_color_t1_ = NULL;
	g_map_depth_t1_raw_ = g_map_depth_t1_filt_ = NULL;
	zz_map_depth = zz_pointer_p3d = NULL;

	// for trace display.

	for(int i=0; i<zz_trace.size(); i++)	delete zz_trace[i];
	zz_trace.clear();

	for(int i=0; i<zz_trace_gt.size(); i++)	delete zz_trace_gt[i];
	zz_trace_gt.clear();

	// for color model.
	if(zz_gmm_hand) delete zz_gmm_hand; zz_gmm_hand = NULL;

	// for error measurement.
	for(int i=0; i<5; i++) elapsed_t[i] = 0.0f;
	avg_t = 0.0f;
	cnt = 0;
	zz_cover_prev = 0.0f;

	// for re-localization.
	zz_flag_relocal = false;
	if(zz_integral_hist_ori) delete[] zz_integral_hist_ori; zz_integral_hist_ori = NULL;
	zz_set_of_templates.clear();

	// for image saving.
	zz_set_of_input_depth.c_Create(1); //zz_set_of_input_depth.c_Create(KV_MAX_NUM_INPUT_IMAGES);
	zz_set_of_input_rgb.c_Create(1);   //zz_set_of_input_rgb.c_Create(KV_MAX_NUM_INPUT_IMAGES);
	zz_set_of_input_sil.c_Create(1);

	//////////////////////////////////////////////////////////////////////////
	// Device memory.
	//////////////////////////////////////////////////////////////////////////
	g_ip.r_Release();

	g_cube_obj.release();
	g_cube_rgb_d.release();
	g_vol_valid.release();

	g_state_rgb_d.release();

	g_img_color_t1.release();
	g_map_depth_t1_filt.release();
	g_map_depth_t1_raw.release();
	g_map_depth_t1.release();
	
	g_T_mat.release();
	
	if(g_T_gc_t0) cudaFree(g_T_gc_t0); if(g_T_cg_t0) cudaFree(g_T_cg_t0); g_T_cg_t0 = g_T_gc_t0 = NULL;
	if(g_T_gc_t1) cudaFree(g_T_gc_t1); if(g_T_cg_t1) cudaFree(g_T_cg_t1); g_T_cg_t1 = g_T_gc_t1 = NULL;
}

//********************************************************************************************
void CKvYooji_3D_Object_Scanner::is_Initialize_Scanner(
	CKvYooji_Scanner_Parameter &in_system_parameter,
	CKvYooji_MatrixRgbD &in_frame_of_rgbd_camera,
	bool in_flag_gpu,
	bool in_flag_sfs,
	bool in_flag_color,
	CKvYooji_ColorModel *in_color_model_of_hand,
	CKvYooji_ColorModel *in_gmm_back_pixel,
	CKvYooji_ColorModel *in_gmm_back_global)
//********************************************************************************************
{
	CKvPmatrix3D pmat, pmat_rgb;

	// remove pre-scanned object data.
 //	pis_Pre_Initialize_Scanner();

	// for trace.
	
	//zz_trace=new vector<CKvYooji_Extrinsics*>;
//printf("zz_trace\n");	
	
	zz_trace.clear();
	zz_state_trace.initialize(640,480);

	CKvPoint3Df origin_k, center_k;
	int ww_d, hh_d, ww_rgb, hh_rgb, ww_c, hh_c ,dd_c, lev_pyram, sz_sc_iv;
	float sz_vox, th_ICP, mu, max_w, dmin, dmax;

	// Get parameters.
	// ==========================================================
 	zz_number_of_processed_frames = 0;//200;
	/// ///////////////////////////////////////////////////////////
	zz_cover_prev = 0.0f;

	zz_flag_relocal = false;
	zz_integral_hist_ori=new CKvSet_of_MatrixInt[1];



//printf("zz_params\n");	

	/// ///////////////////////////////////////////////////////////
 	zz_flag_integ = zz_flag_track = false;
 	zz_params.cp_Copy(&in_system_parameter);
 	zz_params.gp_Get_Parameters(
 		ww_c, hh_c, dd_c,
 		sz_sc_iv,
 		sz_vox,
 		origin_k,
 		lev_pyram,
 		th_ICP, mu, max_w, dmin, dmax);

	// Initialize system structure.
	// ==========================================================
	// + import RGB-D frame.
	zz_rgbd_frame.copy(&in_frame_of_rgbd_camera);
	// + set matrices.
	zz_rgbd_frame.p_map_depth_raw()->ms(ww_d, hh_d);
	zz_rgbd_frame.p_image_rgb()->ms(ww_rgb, hh_rgb);

	zz_rgbd_frame.p_map_depth_raw_on_RGB()->c_Create(hh_rgb, ww_rgb, 0.0f);
	zz_rgbd_frame.p_map_depth_filtered()->c_Create(hh_d, ww_d, 0.0f);
	zz_rgbd_frame.p_map_depth_filtered_on_RGB()->c_Create(hh_rgb, ww_rgb);
	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	// + set initial pose of depth camera to canonical camera.
	zz_intrins_depth.set_from_P_matrix(zz_rgbd_frame.p_P_matrix_depth());
	zz_extrins_init_depth.set_from_P_matrix(zz_rgbd_frame.p_P_matrix_depth());
	zz_state.compute_P_matrix(&zz_intrins_depth,&zz_extrins_init_depth,&pmat);	// depth	
	// + set initial camera pose of RGB camera.
	zz_intrins_rgb.set_from_P_matrix(zz_rgbd_frame.p_P_matrix_RGB());
	zz_extrins_init_rgb.set_from_P_matrix(zz_rgbd_frame.p_P_matrix_RGB());
	zz_extrins_d_rgb.get_pose_relative(zz_extrins_init_depth,zz_extrins_init_rgb,zz_extrins_d_rgb);
	// + set relative pose between depth and RGB cameras. (depth -> rgb)
	zz_rgbd_frame.p_extrinsics_depth_to_RGB()->copy(&zz_extrins_d_rgb);	// rgb wrt depth
	zz_rgbd_frame.p_extrinsics_RGB_to_depth()->set_from_transform_inv(zz_extrins_d_rgb.mp_transform()); // depth wrt rgb.
	zz_state.compute_P_matrix(&zz_intrins_rgb,&zz_extrins_d_rgb, &pmat_rgb);		//rgb

	// set initial pose of extrinsics.
	zz_state.p_extrinsics_glob_to_cam()->set_from_P_matrix(&pmat);
	zz_state.p_extrinsics_glob_to_cam_RGB()->set_from_P_matrix(&pmat_rgb);
	
printf("pyramids\n");	

	//////////////////////////////////////////////////////////////////////////
	// for pyramids.
	//////////////////////////////////////////////////////////////////////////
	// + set intrinsic pyramids of depth and RGB cameras.
	//////////////////////////////////////////////////////////////////////////

	zz_rgbd_frame.p_pyramid_map_depth()->c_Create(ww_rgb,hh_rgb,lev_pyram);
	zz_rgbd_frame.p_pyramid_map_normals()->c_Create(ww_rgb,hh_rgb,lev_pyram);
	zz_rgbd_frame.p_pyramid_map_p3d()->c_Create(ww_rgb,hh_rgb,lev_pyram);

	zz_rgbd_frame.p_pyramid_image_gray()->c_Create(ww_rgb,hh_rgb,lev_pyram);
	zz_rgbd_frame.p_pyramid_map_edge()->c_Create(ww_rgb,hh_rgb,lev_pyram);
	zz_rgbd_frame.p_pyramid_map_grad_edge()->c_Create(ww_rgb,hh_rgb,lev_pyram);
	zz_rgbd_frame.p_pyramid_map_depth_edge()->c_Create(ww_rgb,hh_rgb,lev_pyram);
// 	//////////////////////////////////////////////////////////////////////////
	// + set depth pyramids.
	zz_ip.cpdd_Construct_Pyramids_Down_Depth(
		zz_rgbd_frame.p_map_depth_raw_on_RGB(),
		zz_rgbd_frame.p_pyramid_map_depth(),
		lev_pyram);
	// + set intrinsic pyramids of RGB cameras.
	zz_ip.cpdi_Construct_Pyramids_Down_Intrinsics(
		&zz_intrins_rgb,
		zz_rgbd_frame.p_pyramid_intrinsics(),
		lev_pyram);

	if(!Kv_Printf("lev_pyram: %d",lev_pyram)) exit(0);
	//////////////////////////////////////////////////////////////////////////

	// + set pyramids of state.
	zz_state.initialize(ww_rgb, hh_rgb, lev_pyram);
	zz_state.p_pyramid_silhouette()->c_Create(ww_rgb,hh_rgb,lev_pyram);
	zz_state.p_pyramid_image_gray()->c_Create(ww_rgb,hh_rgb,lev_pyram);
	zz_state.p_pyramid_map_edge()->c_Create(ww_rgb,hh_rgb,lev_pyram);
	zz_state.p_pyramid_map_grad_edge()->c_Create(ww_rgb,hh_rgb,lev_pyram);

	zz_state.p_pyramid_map_depth_edge()->c_Create(ww_rgb,hh_rgb,lev_pyram);
	zz_state.p_pyramid_map_confidence()->c_Create(ww_rgb,hh_rgb,lev_pyram);

	zz_state.p_pyramid_map_depth()->c_Create(ww_rgb,hh_rgb,lev_pyram);
	zz_state.p_pyramid_map_normals()->c_Create(ww_rgb,hh_rgb,lev_pyram);
	zz_state.p_pyramid_map_p3d()->c_Create(ww_rgb,hh_rgb,lev_pyram);

	zz_state.p_pyramid_map_depth_rendered()->c_Create(ww_rgb,hh_rgb,lev_pyram);
	zz_state.p_pyramid_map_normals_rendered()->c_Create(ww_rgb,hh_rgb,lev_pyram);
	zz_state.p_pyramid_map_p3d_rendered()->c_Create(ww_rgb,hh_rgb,lev_pyram);

	zz_state.p_pyramid_intrinsics()->cp_Copy(
	zz_rgbd_frame.p_pyramid_intrinsics());
		
// 	// + set initial pose of depth camera to canonical camera.
// 	zz_intrins_depth.copy(zz_rgbd_frame.p_intrinsics_depth());
// 	zz_extrins_init_depth.copy(zz_rgbd_frame.p_extrinsics_depth());
// 	// + set initial camera pose of RGB camera.
// 	zz_intrins_rgb.copy(zz_rgbd_frame.p_intrinsics_RGB());
// 	zz_extrins_init_rgb.copy(zz_rgbd_frame.p_extrinsics_RGB());
// 	zz_extrins_d_rgb.get_pose_relative(zz_extrins_init_depth, zz_extrins_init_rgb,
// 		zz_extrins_d_rgb);
// 	zz_extrins_rgb_d.set_from_transform_inv(zz_extrins_d_rgb.mp_transform());

	//////////////////////////////////////////////////////////////////////////
	// Set initial object cube.
	//////////////////////////////////////////////////////////////////////////
	Vector3i dim(ww_c,hh_c,dd_c);
	Vector3f origin(origin_k.x,origin_k.y,origin_k.z);
	Vector3f center = origin + dim.toFloat()*sz_vox/2.0f;

	// set object cube in CPU.
	zz_cube.c_Create(origin_k, dd_c, hh_c, ww_c, sz_sc_iv, sz_vox);
	// get object origin and cube size.
	zz_cube.goiw_Get_Origin_In_World(zz_origin_cube);
	zz_sz_cube = ww_c*zz_cube.svm_Size_of_Voxel_in_Meter();

	//////////////////////////////////////////////////////////////////////////
	// bounding sphere setting.
	CKvPoint3D cen_cube; float len_diag;
	
	cen_cube.x = center.x; cen_cube.y = center.y; cen_cube.z = center.z;
	len_diag = sqrt(3*SQUARE(zz_sz_cube/2));
	
	zz_disp.gcps_Get_Camera_Projection_Sphere(
		cen_cube,
		len_diag,
		&zz_p3d_sphere,
		&zz_rgbf_sphere,
		&zz_mesh_sphere);

	// for re-localization.
	if(zz_integral_hist_ori) delete[] zz_integral_hist_ori;
	zz_integral_hist_ori=new CKvSet_of_MatrixInt[1];


// 	zz_g3d.g_sp_Set_display_Parameters(true,Kv_RgbF(0.0f,0.0f,0.0f),false,Kv_Hpoint3D(0.0,0.0,0.0,1.0),0.0f,0.0f,false,true,0);
// 	aa_uw.ss_Set_Size(&zz_g3d,false,640,480);
	//////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////
	g_mode_rgb = in_flag_color;
	///////////////////////////////////////

	///////////////////////////////////////
	// initialize object cube.
	g_cube_obj.create(dim, origin, center, sz_vox, true);
	//////////////////////////////////////////////////////////////////////////
	// initialize TSDF values to 1.0f.
	int3 dim3d; 	dim3d.x = ww_c; dim3d.y = hh_c; dim3d.z = dd_c;
	setDeviceMem3D(g_cube_obj.vp_tsdf(), dim3d, 1.0f);
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	// GPU memory initialization.
	//////////////////////////////////////////////////////////////////////////
	g_map_depth_t1_raw.create(hh_rgb, ww_rgb, 1);
	g_map_depth_t1_filt.create(hh_rgb, ww_rgb, 1);

	if(g_map_depth_t1_raw_) cudaFree(g_map_depth_t1_raw_);
	if(g_map_depth_t1_filt_) cudaFree(g_map_depth_t1_filt_);
	if(g_img_color_t1_) cudaFree(g_img_color_t1_);

	cudaMalloc((void**)&g_map_depth_t1_raw_,ww_rgb*hh_rgb*sizeof(float));
	cudaMalloc((void**)&g_map_depth_t1_filt_,ww_rgb*hh_rgb*sizeof(float));
	cudaMalloc((void**)&g_img_color_t1_,3*ww_rgb*hh_rgb*sizeof(uchar));

	g_img_color_t1.create(hh_rgb, ww_rgb, 3);


	// initialize extrinsics for ICP.
	if(g_T_gc_t0) cudaFree(g_T_gc_t0);	if(g_T_cg_t0) cudaFree(g_T_cg_t0);
	if(g_T_gc_t1) cudaFree(g_T_gc_t1);	if(g_T_cg_t1) cudaFree(g_T_cg_t1);

	cudaMalloc((void**)&g_T_gc_t0,16*sizeof(float));	cudaMalloc((void**)&g_T_cg_t0,16*sizeof(float));
	cudaMalloc((void**)&g_T_gc_t1,16*sizeof(float));	cudaMalloc((void**)&g_T_cg_t1,16*sizeof(float));

	g_T_mat.create(4,4,1);

	// initialize RGB-D frame for gpu.
	g_rgbd_frame.create(Vector2i(ww_rgb,hh_rgb));

	// initialize maps for ICP.
//	g_state_obj_raw.create(Vector2i(ww_d, hh_d));
	g_state_obj.create(Vector2i(ww_rgb,hh_rgb));

	if(zz_map_depth) delete[] zz_map_depth; zz_map_depth = new float[ww_rgb*hh_rgb];

	///////////////////////////////////////////////////////////////////////////////////////
	// initialize relative extrinsics for RGB and depth camera.
	// + depth camera.
	if(g_eye4) cudaFree(g_eye4); cudaMalloc((void**)&g_eye4,16*sizeof(float));
	cudaMemcpy(g_eye4,g_state_obj.vp_T_gc(),16*sizeof(float),cudaMemcpyDeviceToDevice);
	// + RGB camera.
	cudaMemcpy(g_T_mat.vp(),zz_extrins_d_rgb.mp_transform()->vp(),16*sizeof(float),cudaMemcpyHostToDevice);
	
	g_state_obj.set_transform(&g_T_mat);

	// initialize rendering parameters for GPU.
	float fx_d,fy_d,px_d,py_d;
	float fx_rgb,fy_rgb,px_rgb,py_rgb;

	zz_intrins_depth.get_params(fx_d,fy_d,px_d,py_d,dmin,dmax);
	zz_intrins_rgb.get_params(fx_rgb,fy_rgb,px_rgb,py_rgb,dmin,dmax);


	// for pose tracking and rendering on RGB camera.
	g_track.ip_Initialize_Parameters(
		ww_rgb,hh_rgb,
		fx_rgb,fy_rgb,px_rgb,py_rgb,
		th_ICP);
	
	//////////////////////////////////////////////////////////////////////////
	g_integ_obj.ip_Initialize_Parameters(
		&g_cube_obj,
		//ww_d,hh_d,
		//fx_d,fy_d,px_d,py_d,
		ww_rgb,hh_rgb,
		fx_rgb,fy_rgb,px_rgb,py_rgb,
		ww_rgb,hh_rgb,
		fx_rgb,fy_rgb,px_rgb,py_rgb,
		zz_extrins_d_rgb.mp_transform()->vp(),
		mu,max_w);
	//////////////////////////////////////////////////////////////////////////

	g_render_obj.ip_Initialize_Parameters(
		&g_cube_obj,ww_rgb,hh_rgb,
		fx_rgb,fy_rgb,px_rgb,py_rgb,
		mu,max_w);
	//zz_extrins_d_rgb.print_transform();

	// =================================================================================
	// + initialize tracking state.	
	//zz_state.initialize(ww_rgb, hh_rgb, KV_LEVEL_OF_IMAGE_PYRAMID);
	zz_state.import_P_matrix(&pmat, &pmat_rgb);

	// Initialize reference poses for display trace =============================== //
	// ============================================================================ //
	z_std_Setting_for_Trace_Display();

	//////////////////////////////////////////////////////////////////////////
	// For SiftGPU.
	//////////////////////////////////////////////////////////////////////////
	zz_imatch.i_Initialize();

// 	zz_state.im_Import(
// 		zz_rgbd_frame.p_P_matrix_depth(), 
// 		zz_rgbd_frame.p_P_matrix_RGB());

	// for hand segmentation.
	if(in_color_model_of_hand){
		zz_gmm_hand = new CKvYooji_ColorModel;
		(*zz_gmm_hand).cp_Copy(in_color_model_of_hand);
	}

	// for Aruco module.
	float vals[] ={FX,0.,CX,0.,FY,CY,0.,0.,1.};
	float coeffs[] ={0.,0.,0.,0.,0.,};

	zz_cameraMatrix = Mat(3,3,CV_32FC1,vals);
	zz_distCoeffs = Mat(5,1,CV_32FC1,coeffs);

	zz_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	zz_board = cv::aruco::GridBoard::create(5,7,0.04,0.01,zz_dictionary);
	zz_detectorParams = aruco::DetectorParameters::create();

	zz_detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers

}


//********************************************************************************************
//CKvStopWatch sw;	
//float elapsed_t[5] = {0.0f}, avg_t = 0.0f;	int cnt=0;

bool CKvYooji_3D_Object_Scanner::so_Scan_Object(
	CKvYooji_MatrixRgbD *in_frame_of_rgbd_camera,
	bool in_flag_sfs,
	bool in_flag_on_rgb,
	int in_type_of_data,
	int in_mode_track,
	int in_lev_pyram,
	bool in_mode_board_removal,
	bool in_flag_gpu,
	bool in_flag_time)
//********************************************************************************************
{	
	if(KV_MODE_CAPTURE_ONLY){
		zz_set_of_input_depth.gpe(zz_number_of_processed_frames)->cp_Copy(in_frame_of_rgbd_camera->p_map_depth_raw());
		zz_set_of_input_rgb.gpe(zz_number_of_processed_frames)->cp_Copy(in_frame_of_rgbd_camera->p_image_rgb());
	}
	else{
 		if(in_flag_time){ sw.c_Create(1);	cnt++; }
 
 		CKvPmatrix3D pmat;
 		CKvMatrixBool obj_roi;
 		CKvYooji_Extrinsics extrins_rel,extrins_curr;
 
		//printf("Here????\n");

 		if(in_flag_time) sw.r_Reset(0);
 		//printf("++ Update input data. ++								\r");
 		// Update input data.
 		// ============================================================================
 		z_uid_Update_Input_Data(in_frame_of_rgbd_camera, in_mode_board_removal, in_flag_gpu);
 
 		//////////////////////////////////////////////////////////////////////////
 
 		if(in_flag_time){
 			elapsed_t[0] += sw.get_Get_Elapsed_Time(0);
 			avg_t = elapsed_t[0]/cnt;
 			printf(" [%5.3f ms] z_uid_Update_Input_Data\n",avg_t*1000.0f);
 			//if(!Kv_Printf("STOP!")) exit(0);
 		}
 
 		if(in_flag_time) sw.r_Reset(0);
 
 		//printf("++ Construct input image pyramid. ++					\r");
 		// Construct input image pyramid for coarse-to-fine pose estimation.
 		// ============================================================================
 		z_cip_Construct_Image_Pyramids(
 			in_lev_pyram,
 			in_mode_track,
 			in_flag_gpu);
 
 		zz_flag_integ = true;
 
 		if(in_flag_time){
 			elapsed_t[2] += sw.get_Get_Elapsed_Time(0);
 			avg_t = elapsed_t[2]/cnt;
 			printf(" [%5.3f ms] z_cip_Construct_Image_Pyramids\n",avg_t*1000.0f);
 			//if(!Kv_Printf("STOP!")) exit(0);
 		}
 
 		//printf("++ Estimate camera pose. ++								\r");
 		// Estimate object pose =================================================== //
 		// ======================================================================== //
 		if(zz_flag_track){
 			//printf("++ Object pose estimation ++\n");
 			if(in_flag_time) sw.r_Reset(0);
 
 			zz_flag_integ = z_ecp_Estimate_Camera_Pose(
 				in_lev_pyram,
 				in_mode_track,
 				in_flag_gpu);
					

			// if re-localization is complete.
			if(zz_flag_relocal){
				if(!Kv_Printf("zz_flag_integ: %d\n", zz_flag_integ)) exit(0);
				zz_flag_relocal = false;
			}

			//////////////////////////////////////////////////////////////////////////
			// drift detector.
			//////////////////////////////////////////////////////////////////////////
			if(!zz_flag_integ){
				zz_flag_relocal = true;
				zz_number_of_processed_frames++;
				return false;
			}
			//////////////////////////////////////////////////////////////////////////
 
 			if(in_flag_time){
 				elapsed_t[3] += sw.get_Get_Elapsed_Time(0);
 				avg_t = elapsed_t[3]/cnt;
 				printf(" [%5.3f ms] z_ecp_Estimate_Camera_Pose\n",avg_t*1000.0f);
 				//if(!Kv_Printf("STOP!")) exit(0);
 			}
 
  		}
 
 
 		//// Initialize reference poses for display trace =============================== //
 		//// ============================================================================ //
 		if(!zz_flag_track){
 			// + start camera tracking.
 			zz_flag_track = true;
 			//z_std_Setting_for_Trace_Display();
 		}
 
 		// Update object model ======================================================== //
 		// ============================================================================ //
 		if(zz_flag_integ){
 
 			if(in_flag_time) sw.r_Reset(0);
 
 			//printf("++ Update object model ++							\r");
 			z_uom_Update_Object_Model(true,in_flag_gpu);
 
 			if(in_flag_time){
 				elapsed_t[5] += sw.get_Get_Elapsed_Time(0);
 				avg_t = elapsed_t[5]/cnt;
 				printf(" [%5.3f ms] z_uom_Update_Object_Model\n",avg_t*1000.0f);
 				//if(!Kv_Printf("STOP!")) exit(0);
 			}
 
 			if(in_flag_time) sw.r_Reset(0);
 
 			//printf("++ Update tracking state ++							\r");
 			z_uts_Update_Tracking_State(in_mode_track,in_type_of_data,in_flag_gpu);
 
 			if(in_flag_time){
 				elapsed_t[6] += sw.get_Get_Elapsed_Time(0);
 				avg_t = elapsed_t[6]/cnt;
 				printf(" [%5.3f ms] z_uts_Update_Tracking_State\n",avg_t*1000.0f);
 				//if(!Kv_Printf("STOP! %d", zz_number_of_processed_frames)) exit(0);
 			}
 		} 
 		else{
 			printf("============================ Tracking fail ============================\n");
 			if(!Kv_Printf("Tracking Fail!")) exit(0);
 		}
 
 
 		//printf("++ Evaluation ++\n");
 		// Evaluate system performance ================================================ //
 		// ============================================================================ //
 		if(0){
 			z_esp_Evaluate_System_Performance();
 		}
 		//printf("++ Update tracking state ++							\r");
 
		/// frame 수가 안맞나봐1!!! 시바라릴!~!!
//  		zz_set_of_input_depth.gpe(zz_number_of_processed_frames)->cp_Copy(zz_rgbd_frame.p_map_depth_raw());
//  		zz_set_of_input_rgb.gpe(zz_number_of_processed_frames)->cp_Copy(zz_rgbd_frame.p_image_rgb());
// 		if(in_mode_board_removal) zz_set_of_input_sil.gpe(zz_number_of_processed_frames)->cp_Copy(zz_rgbd_frame.p_image_silhouette());
	}	

	zz_number_of_processed_frames++;

	return true;
}

//********************************************************************************************
bool CKvYooji_3D_Object_Scanner::rbd_Remove_Board_Depth(CKvYooji_MatrixRgbD *in_frame_of_rgbd_camera)
//********************************************************************************************
{
	//////////////////////////////////////////////////////////////////////////
	// ArUco Module
	//////////////////////////////////////////////////////////////////////////
	float vals[] ={FX,0.,CX,0.,FY,CY,0.,0.,1.};
	float coeffs[] ={0.,0.,0.,0.,0.,};

	zz_cameraMatrix = Mat(3,3,CV_32FC1,vals);
	zz_distCoeffs = Mat(5,1,CV_32FC1,coeffs);

	cv::Mat image,imageCopy;
	//inputVideo.retrieve(image);
	zz_ilc.cfko_Convert_Format_from_KAISION_to_Opencv(*zz_rgbd_frame.p_image_rgb(),image);

	image.copyTo(imageCopy);
	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f> > corners,rejected;
	cv::aruco::detectMarkers(image,zz_dictionary,corners,ids,zz_detectorParams,rejected);
	// refine strategy to detect more markers
	aruco::refineDetectedMarkers(image,zz_board,corners,ids,rejected);
	
	// if at least one marker detected
	cv::Mat pose = cv::Mat::eye(CvSize(4,4),CV_32FC1);	
	cv::Mat rmat;
	if(ids.size() > 0) {
		cv::aruco::drawDetectedMarkers(imageCopy,corners,ids);
		cv::Mat rvec,tvec;
		int valid = estimatePoseBoard(corners,ids,zz_board,zz_cameraMatrix,zz_distCoeffs,rvec,tvec);
		// if at least one board marker detected
		if(valid > 0){
			cv::aruco::drawAxis(imageCopy,zz_cameraMatrix,zz_distCoeffs,rvec,tvec,0.1);

			//////////////////////////////////////////////////////////////////////////
			Rodrigues(rvec,rmat);
			rmat.copyTo(pose(CvRect(0,0,3,3)));
			tvec.copyTo(pose(CvRect(3,0,1,3)));
		}
		else return false;
	}
	else return false;
	cv::imshow("out",imageCopy);
	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	// compute world's Z-plane in camera coordinates.				
	// pi_z'*X = 0
	// pi_cam'*(T*X) = 0 -> T'*pi_cam = pi_z
	// pi_cam = inv(T')*pi_z = inv(T')(:,2) = (inv(T)(2,:))'
 	//Mat pi_z = pose.inv()(CvRect(0,2,4,1)).t();
 	Mat pi_z = pose.t().inv()(CvRect(2,0,1,4));
 
 	cout << pi_z << endl;
	
	d_pm_Printf_Matrix(zz_rgbd_frame.p_intrinsics_RGB()->mp()->vp(), 3, 3, "K");
 
 	//////////////////////////////////////////////////////////////////////////
 	zz_obj_ext.rbd_Remove_Board_Depth(
 		zz_rgbd_frame.p_map_depth_raw_on_RGB(),
 		zz_rgbd_frame.p_intrinsics_RGB(),
 		pi_z,
 		zz_rgbd_frame.p_image_silhouette());
 
 	int ww,hh;
 	float *p_depth_filt = zz_rgbd_frame.p_map_depth_filtered_on_RGB()->mps(ww,hh)[0];
 	bool *p_mask = zz_rgbd_frame.p_image_silhouette()->vp();
 	for(int i=0; i<ww*hh; i++) if(!p_mask[i]) p_depth_filt[i] = 0.0f;
 	//////////////////////////////////////////////////////////////////////////
//  
//  	CKvScreen sc;
//  	sc.s_d_Display(zz_rgbd_frame.p_image_silhouette());
//  
//  	if(!Kv_Printf("Object mask!")) exit(0);
	
	

	return true;
}

//********************************************************************************************
bool CKvYooji_3D_Object_Scanner::z_lid_Load_Input_Data(bool in_flag_gpu)
//********************************************************************************************
{
	int type_cature = zz_capture.gct_Get_Capture_Type();
	bool valid = false;

	if(type_cature == KV_CAPTURE_FROM_FILES){

		valid = zz_capture.lrdi_Load_Rgb_and_Depth_Images(
			zz_number_of_processed_frames,
			zz_rgbd_frame.p_image_rgb(),
			zz_rgbd_frame.p_map_depth_raw());
	
		// Get pre-extracted silhouette.
		zz_capture.ls_Load_Silhouette(zz_number_of_processed_frames,zz_rgbd_frame.p_image_silhouette());
		// Get ground truth camera pose.
		zz_capture.lpm_Load_P_Matrix(zz_number_of_processed_frames,zz_rgbd_frame.p_P_matrix_depth());
	
	}
	else if(type_cature == KV_CAPTURE_FROM_KINECTV1){
		
	}

	return valid;
}

//********************************************************************************************
void CKvYooji_3D_Object_Scanner::z_uid_Update_Input_Data(CKvYooji_MatrixRgbD *in_frame_of_rgbd_camera,
	bool in_mode_board_remove,
	bool in_flag_gpu)
//********************************************************************************************
{
	int ww,hh;

	in_frame_of_rgbd_camera->p_map_depth_raw()->ms(ww, hh);

	// import input raw depth data.
	zz_rgbd_frame.p_map_depth_raw()->cp_Copy(in_frame_of_rgbd_camera->p_map_depth_raw());

	//////////////////////////////////////////////////////////////////////////
	// 반복실행하면 여기서 죽네!!?
	// 뭔가 release 나 초기화가 잘못된듯?! Fuck!
	// 어???
	// Rendering 결과가 제대로 안나오네!! 이부분이 이상한거 같기도!!!ㅅㅂㅅㅂㅅㅂㅅㅂㅅㅂ
	// do bilateral filtering and save it to bottom of depth pyramid.
 	g_ip.bfd_Bilateral_Filter_Depth_Host(
 		zz_rgbd_frame.p_map_depth_filtered()->vp(),
 		zz_rgbd_frame.p_map_depth_raw()->vp(),
 		ww,hh);
//	zz_rgbd_frame.p_map_depth_filtered()->cp_Copy(zz_rgbd_frame.p_map_depth_raw());
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	// thresholding depth map.
// 	zz_obj_ext.gvd_Get_Valid_Depth(
// 		zz_rgbd_frame.p_map_depth_raw(),
// 		MIN_DEPTH,MAX_DEPTH);
// 	zz_rgbd_frame.p_map_depth_filtered()->cp_Copy(zz_rgbd_frame.p_map_depth_raw());
// 	zz_obj_ext.gvd_Get_Valid_Depth(
// 		zz_rgbd_frame.p_map_depth_filtered(),
// 		MIN_DEPTH,MAX_DEPTH);
	//////////////////////////////////////////////////////////////////////////

	// + depth map.
	zz_rgbd_frame.p_P_matrix_depth()->cp_Copy(in_frame_of_rgbd_camera->p_P_matrix_depth());

	// + RGB image.
	zz_rgbd_frame.p_image_silhouette()->cp_Copy(in_frame_of_rgbd_camera->p_image_silhouette());
	zz_rgbd_frame.p_image_rgb()->cp_Copy(in_frame_of_rgbd_camera->p_image_rgb());
	zz_rgbd_frame.p_P_matrix_RGB()->cp_Copy(in_frame_of_rgbd_camera->p_P_matrix_RGB());

	// + flag.
	zz_rgbd_frame.set_validity_silhouette(in_frame_of_rgbd_camera->is_valid_silhouette());
	zz_rgbd_frame.set_validity_extrinsics(in_frame_of_rgbd_camera->is_valid_extrinsics());

	//////////////////////////////////////////////////////////////////////////
	// 나중에 update input data 랑 pre-processing 단을 좀 나누자!!
	//////////////////////////////////////////////////////////////////////////
		
	//////////////////////////////////////////////////////////////////////////
	// + Extract depth in object cube only.
	//if(!zz_flag_relocal)

	if(KV_FLAG_FRAME_TO_MODEL_DEPTH)
		zz_obj_ext.god_Get_Object_Depth(
			&zz_rgbd_frame,
			&zz_state,
			&zz_cube,
			true,
			zz_gmm_hand,
			&zz_hand_mask);
	//////////////////////////////////////////////////////////////////////////

//
	//////////////////////////////////////////////////////////////////////////
	// Compute depth map on RGB camera.
	//////////////////////////////////////////////////////////////////////////
	// 항상 RGB 카메라에서 본 깊이 영상을 생성하고 시작하는 걸로 결정!
	// 이부분 GPU 로 빠르게 구현해보자.
 	// raw depth map.
 	z_tdrcc_Transform_Depth_to_Rgb_Camera_Coordinates(
 		&zz_rgbd_frame,
 		zz_rgbd_frame.p_map_depth_raw(),
 		zz_rgbd_frame.p_map_depth_raw_on_RGB());
 
 	// filtered depth map.
 	z_tdrcc_Transform_Depth_to_Rgb_Camera_Coordinates(
 		&zz_rgbd_frame,
 		zz_rgbd_frame.p_map_depth_filtered(),
 		zz_rgbd_frame.p_map_depth_filtered_on_RGB());

	//////////////////////////////////////////////////////////////////////////
	// Silhouette
	//////////////////////////////////////////////////////////////////////////
	if(zz_rgbd_frame.is_valid_silhouette()){
		// for ground truth depth silhouette. (same camera with depth map)
		int ww,hh;
		bool *p_mask_obj = zz_rgbd_frame.p_image_silhouette()->mps(ww,hh)[0];
		float *p_depth_raw = zz_rgbd_frame.p_map_depth_raw_on_RGB()->vp();
		float *p_depth_filt = zz_rgbd_frame.p_map_depth_filtered_on_RGB()->vp();
		//////////////////////////////////////////////////////////////////////////
		for(int i=0; i<ww*hh; i++)  if(!p_mask_obj[i])	p_depth_raw[i] = p_depth_filt[i] = 0.0f;
		//////////////////////////////////////////////////////////////////////////
	}

	//////////////////////////////////////////////////////////////////////////
	// Remove board depth.
	//////////////////////////////////////////////////////////////////////////
	if(in_mode_board_remove){
		rbd_Remove_Board_Depth(&zz_rgbd_frame);
	}

	//if(!zz_flag_relocal){
		//////////////////////////////////////////////////////////////////////////
		// get roi region.
		//////////////////////////////////////////////////////////////////////////
		// Get object ROI
		//////////////////////////////////////////////////////////////////////////
 		int imax,jmax,imin,jmin;
 		float *p_depth_raw = zz_rgbd_frame.p_map_depth_raw_on_RGB()->vp();
 
 		imin = jmin = 100000;
 		imax = jmax = -100000;
 
 		for(int j=0; j<hh; j++){
 			for(int i=0; i<ww; i++){
 
 				int tidx = j*ww + i;
 				if(p_depth_raw[tidx] <= 0.0f) continue;
 
 				if(imax < i) imax = i;  if(imin > i) imin = i;
 				if(jmax < j) jmax = j;	if(jmin > j) jmin = j;
 
 			}
 		}
 
 		Rect obj_roi(imin,jmin,imax - imin + 1,jmax - jmin + 1);
 
 		(*zz_rgbd_frame.p_roi_object()) = obj_roi;
 
 		// update roi images.
		int margin = 5;
		imin = max(0, min(ww, imin - margin));
		jmin = max(0, min(hh, jmin - margin));
		imax = max(0, min(ww, imax + margin));
		jmax = max(0, min(hh, jmax + margin));
	
 		zz_rgbd_frame.p_map_depth_raw_on_RGB()->gb_Get_Block(imin,jmin,imax - imin + 1,jmax - jmin + 1,
 			zz_rgbd_frame.p_roi_depth());
 		zz_rgbd_frame.p_image_rgb()->gb_Get_Block(imin,jmin,imax - imin + 1,jmax - jmin + 1,
 			zz_rgbd_frame.p_roi_image_rgb());

		printf("[%d %d %d %d]\n", imin, imax, jmin, jmax);
 
 		zz_ycc.mrg_Matrix_Rgb_to_Gray(zz_state.p_roi_image_rgb(),false,zz_state.p_roi_image_gray());
 		zz_ycc.mrg_Matrix_Rgb_to_Gray(zz_rgbd_frame.p_roi_image_rgb(),false,zz_rgbd_frame.p_roi_image_gray());
		//////////////////////////////////////////////////////////////////////////

	//}
// 	zz_rgbd_frame.p_map_depth_raw_on_RGB()->cp_Copy(zz_rgbd_frame.p_map_depth_raw());
// 	zz_rgbd_frame.p_map_depth_filtered_on_RGB()->cp_Copy(zz_rgbd_frame.p_map_depth_filtered());

}

//********************************************************************************************
void CKvYooji_3D_Object_Scanner::z_cip_Construct_Image_Pyramids(
	int in_lev_pyram,
	int in_mode_track,
	bool in_flag_gpu)
//********************************************************************************************
{
	int level = in_lev_pyram;
	int ww,hh,len;

	// + depth map.
	zz_ip.cpdd_Construct_Pyramids_Down_Depth(
		zz_rgbd_frame.p_map_depth_filtered_on_RGB(),
		zz_rgbd_frame.p_pyramid_map_depth(),
		in_lev_pyram);

	//////////////////////////////////////////////////////////////////////////
	// + gray image.
	zz_ycc.mrg_Matrix_Rgb_to_Gray(zz_rgbd_frame.p_image_rgb(),false,&zz_img);	
// 	uchar *p_r, *p_g, *p_b, *p_gray;
// 	zz_rgbd_frame.p_image_rgb()->ms(ww, hh);	
// 	p_gray = zz_img.c_Create(hh, ww)[0];
// 	p_r = zz_rgbd_frame.p_image_rgb()->vps(p_g, p_b, len);
// 	for(int i=0; i<ww*hh; i++) p_gray[i] = (uchar)clip(0.0f, 255.0f, 0.299f*(float)p_r[i] + 0.587f*(float)p_g[i] + 0.114f*(float)p_b[i]);
	//////////////////////////////////////////////////////////////////////////

	zz_ip.cpd_Construct_Pyramids_Down(
		&zz_img,
		zz_rgbd_frame.p_pyramid_image_gray(),
		in_lev_pyram);

	//////////////////////////////////////////////////////////////////////////
	if(KV_COMPUTE_ON_CPU && !KV_MODE_DEPTH_ONLY){
		
		// compute point maps.
 		if(level != zz_rgbd_frame.p_pyramid_map_p3d()->levOfScale){
 			zz_rgbd_frame.p_pyramid_map_p3d()->c_Create(
 				zz_rgbd_frame.p_pyramid_image_gray()->imgs[0].mw(),
 				zz_rgbd_frame.p_pyramid_image_gray()->imgs[0].mh(),
 				level);
 		}
 
 		for(int i = 0; i<level; i++)
 		zz_ip.cpm_Compute_Point_Map(
 			&zz_rgbd_frame.p_pyramid_map_depth()->imgs[i],
 			&zz_rgbd_frame.p_pyramid_intrinsics()->intrins[i],
 			&zz_rgbd_frame.p_pyramid_map_p3d()->imgs[i]);
 
 
 		// compute normal maps.
 		if(level != zz_rgbd_frame.p_pyramid_map_normals()->levOfScale){
 			zz_rgbd_frame.p_pyramid_map_normals()->c_Create(
 				zz_rgbd_frame.p_pyramid_image_gray()->imgs[0].mw(),
 				zz_rgbd_frame.p_pyramid_image_gray()->imgs[0].mh(),
 				level);
 		}
 
 		for(int i = 0; i<level; i++){
 
 			zz_ip.enm_Estimate_Normal_Map(
 				&zz_rgbd_frame.p_pyramid_map_p3d()->imgs[i],
 				&zz_rgbd_frame.p_pyramid_map_normals()->imgs[i]);
	 // 		zz_disp.dnm_Display_Normal_Map(&zz_sc_ios,
	 // 			&zz_rgbd_frame.p_pyramid_map_normals()->imgs[i]);
 			//if(!Kv_Printf("Level: %d", i)) exit(0);
 		}

		//////////////////////////////////////////////////////////////////////////
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 		// + Depth edge map.
	  	if(level != zz_rgbd_frame.p_pyramid_map_depth_edge()->levOfScale){
	  		zz_rgbd_frame.p_pyramid_map_depth_edge()->c_Create(
	  			zz_rgbd_frame.p_pyramid_map_depth()->imgs[0].mw(),
	  			zz_rgbd_frame.p_pyramid_map_depth()->imgs[0].mh(),
	  			level);
	  	}
	  
	  	for(int i = 0; i<level; i++){
	  		zz_cd.cig_Compute_Image_Gradients(
	  		//zz_cd.cigs_Compute_Image_Gradients_using_Sobel(
	  		//zz_cd.cigs_Compute_Image_Gradients_using_Scharr(
	  			zz_rgbd_frame.p_pyramid_map_depth()->imgs[i],
	  			zz_rgbd_frame.p_pyramid_map_depth_edge()->maps[i].grad_x,
	  			zz_rgbd_frame.p_pyramid_map_depth_edge()->maps[i].grad_y);
	  // 		zz_sc_ios.s_d_Display(1.0f, 0.0f, &zz_rgbd_frame.p_pyramid_map_depth_edge()->maps[i].grad_y);
	  // 		if(!Kv_Printf("Level: %d",i)) exit(0);
	  	}
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		//////////////////////////////////////////////////////////////////////////

		//if(in_mode_track == KV_TYPE_TRACK_COLOR){
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// + Sobel edge map.
		if(level != zz_rgbd_frame.p_pyramid_map_edge()->levOfScale){
			zz_rgbd_frame.p_pyramid_map_edge()->c_Create(
				zz_rgbd_frame.p_pyramid_image_gray()->imgs[0].mw(),
				zz_rgbd_frame.p_pyramid_image_gray()->imgs[0].mh(),
				level);
		}

		for(int i = 0; i<level; i++)
			zz_cd.cig_Compute_Image_Gradients(
			//zz_cd.cigs_Compute_Image_Gradients_using_Sobel(
			//zz_cd.cigs_Compute_Image_Gradients_using_Scharr(
				zz_rgbd_frame.p_pyramid_image_gray()->imgs[i],
				zz_rgbd_frame.p_pyramid_map_edge()->maps[i].grad_x,
				zz_rgbd_frame.p_pyramid_map_edge()->maps[i].grad_y);

		//////////////////////////////////////////////////////////////////////////
		// edge of gradient magnitude.
		//////////////////////////////////////////////////////////////////////////
		for(int i = 0; i<level; i++){

			zz_cd.cmm_Compute_Magnitude_Map(
				zz_rgbd_frame.p_pyramid_map_edge()->maps[i].grad_x,
				zz_rgbd_frame.p_pyramid_map_edge()->maps[i].grad_y,
				zz_rgbd_frame.p_pyramid_map_edge()->maps[i].grad_mag);

			zz_cd.cigs_Compute_Image_Gradients_using_Scharr(
				zz_rgbd_frame.p_pyramid_map_edge()->maps[i].grad_mag,
				zz_rgbd_frame.p_pyramid_map_grad_edge()->maps[i].grad_x,
				zz_rgbd_frame.p_pyramid_map_grad_edge()->maps[i].grad_y);

		}

		// 2nd order.
		for(int i = 0; i<level; i++){
			zz_cd.cig_Compute_Image_Gradients(
			//zz_cd.cigs_Compute_Image_Gradients_using_Sobel(
			//zz_cd.cigs_Compute_Image_Gradients_using_Scharr(
				zz_rgbd_frame.p_pyramid_map_edge()->maps[i].grad_x,
				zz_rgbd_frame.p_pyramid_map_edge()->maps[i].grad_xx,
				zz_rgbd_frame.p_pyramid_map_edge()->maps[i].grad_xy);
			
			zz_cd.cig_Compute_Image_Gradients(
			//zz_cd.cigs_Compute_Image_Gradients_using_Sobel(
			//zz_cd.cigs_Compute_Image_Gradients_using_Scharr(
				zz_rgbd_frame.p_pyramid_map_edge()->maps[i].grad_y,
				zz_rgbd_frame.p_pyramid_map_edge()->maps[i].grad_yx,
				zz_rgbd_frame.p_pyramid_map_edge()->maps[i].grad_yy);

			zz_cd.clm_Compute_Laplacian_Map(
				zz_rgbd_frame.p_pyramid_image_gray()->imgs[i],
				zz_rgbd_frame.p_pyramid_map_edge()->maps[i].laplacian);
// 			zz_cd.clm_Compute_Laplacian_Map(
// 				zz_rgbd_frame.p_pyramid_map_edge()->maps[i].grad_xx,
// 				zz_rgbd_frame.p_pyramid_map_edge()->maps[i].grad_yy,
// 				zz_rgbd_frame.p_pyramid_map_edge()->maps[i].laplacian);
		}

	}
// 	 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// 	}
	 



	// raw depth map and color image of original size for volumetric integration.
	float *map_depth_raw = zz_rgbd_frame.p_map_depth_raw_on_RGB()->vp();
	uchar *img_color = zz_rgbd_frame.p_image_rgb()->mps(ww,hh)[0];
	
	cudaMemcpy(g_map_depth_t1_raw_,map_depth_raw,ww*hh*sizeof(float),cudaMemcpyHostToDevice);
	cudaMemcpy(g_img_color_t1_,img_color,3*ww*hh*sizeof(uchar),cudaMemcpyHostToDevice);

	// pyramid of filtered depth map for pose estimation.
	for(int i=0; i<KV_LEVEL_OF_IMAGE_PYRAMID; i++){
		// update input depth.
		float *map_depth_filtered = zz_rgbd_frame.p_pyramid_map_depth()->imgs[i].mps(ww,hh)[0];
		cudaMemcpy(g_rgbd_frame.vp_map_depth(i),map_depth_filtered,ww*hh*sizeof(float),cudaMemcpyHostToDevice);
	}
		//}

//	}
}

//********************************************************************************************
bool CKvYooji_3D_Object_Scanner::z_ecp_Estimate_Camera_Pose(
	int in_lev_pyram,
	int in_mode_track,
	bool in_flag_gpu)
//********************************************************************************************
{
	CKvYooji_Extrinsics *p_pose = zz_state.p_extrinsics_glob_to_cam_RGB();
	CKvYooji_Extrinsics *p_pose_raw = zz_state.p_extrinsics_glob_to_cam();
	CKvYooji_Intrinsics *p_intrins = zz_rgbd_frame.p_intrinsics_RGB();
	CKvYooji_Intrinsics *p_intrins_raw = zz_rgbd_frame.p_intrinsics_depth();
	CKvYooji_Extrinsics *p_pose_rgb_d = zz_rgbd_frame.p_extrinsics_RGB_to_depth();

	zz_flag_integ = false;

//	p_pose->print_transform("p_extrinsics_glob_to_cam_RGB");
	//////////////////////////////////////////////////////////////////////////
	// CPU
	bool flag_cpu = false;

	//////////////////////////////////////////////////////////////////////////
	if(KV_COMPUTE_ON_CPU){
		flag_cpu = true;
	}
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	// save previous camera pose.
	zz_extrin.copy(p_pose);
	//////////////////////////////////////////////////////////////////////////

	if(in_mode_track == KV_TYPE_TRACK_GROUND){
		//////////////////////////////////////////////////////////////////////////
		// ground truth pose.
		zz_flag_integ = true;
		if(zz_trace_gt.size() > zz_number_of_processed_frames){
			p_pose->copy(zz_trace_gt[zz_number_of_processed_frames]);

			// CPU -> GPU.
			cudaMemcpy(
				g_T_mat.vp(),
				p_pose->mp_transform()->vp(),
				16 * sizeof(float),
				cudaMemcpyHostToDevice);
			g_state_obj.set_transform(&g_T_mat);

			// Get rigid transformation of depth camera.
			// compute P matrix.
			// + for RGB camera.
			zz_state.compute_P_matrix(p_intrins,p_pose,zz_state.p_P_matrix_RGB());
			// + for depth camera.
			p_pose_rgb_d->get_pose_successive(*p_pose,*p_pose_rgb_d,*p_pose_raw);
			zz_state.compute_P_matrix(p_intrins_raw,p_pose_raw,zz_state.p_P_matrix_depth());
		}
		else return false;

		//d_pm_Printf_Matrix(p_pose->mp_transform()->vp(), 4, 4, "T");
	}
	else{
		//////////////////////////////////////////////////////////////////////////
		bool valid_initial = false;

		if(zz_flag_relocal){
			// check template.
			zz_jm.dmfpc_Display_Matrix_Float_using_Pseudo_Color(
				&m_sc[0],
				*zz_rgbd_frame.p_roi_depth(),
				true);

			m_sc[1].s_d_Display(zz_rgbd_frame.p_roi_image_gray());

			if(!Kv_Printf("SIFT!")) exit(0);
		}

		// Pose initialization using SIFT matcher.
		if(KV_FLAG_SIFT_INITIAL)
			valid_initial = zz_tracker.tcsift_Track_Camera_with_SIFT(
				&zz_rgbd_frame,
				&zz_params,
				&zz_state,
				&zz_imatch,
				&zz_mat_4x4);

		if(!valid_initial){
		
// 			CKvScreen tsc[2];
// 			zz_jm.dmfpc_Display_Matrix_Float_using_Pseudo_Color(&tsc[0],
// 				zz_rgbd_frame.p_pyramid_map_depth()->imgs[0], true);
// 			zz_jm.dmfpc_Display_Matrix_Float_using_Pseudo_Color(&tsc[1],
// 				zz_state.p_pyramid_map_depth_rendered()->imgs[0],true);
// 			if(!Kv_Printf("False initial")) exit(0);
		}
		//////////////////////////////////////////////////////////////////////////


		if(flag_cpu){

			if(in_mode_track == KV_TYPE_TRACK_DEPTH){
				// 			if(!(zz_flag_integ = zz_tracker.tccf_Track_Camera_Coarse_to_Fine_with_KinFu(
				// 			&zz_rgbd_frame,
				// 			&zz_params,
				// 			&zz_state,
				// 			true))) return false;
				if(!(zz_flag_integ = zz_tracker.tcdfm_Track_Camera_with_Depth_Frame_to_Model(
					&zz_rgbd_frame,
					&zz_params,
					&zz_state,
					true,
					true,
					valid_initial ? &zz_mat_4x4 : NULL))){
				
					return false;
				}
			} 
			else if(in_mode_track == KV_TYPE_TRACK_COLOR){
				if(!(zz_flag_integ = zz_tracker.tccadvo_Track_Camera_using_Confidence_Adaptive_DVO(
					&zz_rgbd_frame,
					&zz_params,
					&zz_state,
					valid_initial ? &zz_mat_4x4 : NULL))){

					//////////////////////////////////////////////////////////////////////////
					// Drift 발생하면!!??
					//////////////////////////////////////////////////////////////////////////
					// strategy 1.
					// 이전 camera pose를 유지하여 다음 frame 에 pose 를 찾을 수 있길 기원한다.
					// 만약 overlapping region 이 너무 적어지면 re-localization step 으로 넘어간다.
					return false;

					// strategy 2.
					// Tracker를 종료하고 object re-localization step으로 넘어간다.
					//return false;
				}
			}
			//////////////////////////////////////////////////////////////////////////

			// CPU -> GPU.
			cudaMemcpy(
				g_T_mat.vp(),
				p_pose->mp_transform()->vp(),
				16 * sizeof(float),
				cudaMemcpyHostToDevice);
			g_state_obj.set_transform(&g_T_mat);
		} 
		else{
			// track pose of RGB camera.
			//zz_flag_integ = g_track.tp_Track_Pose(&g_state_obj, g_rgbd_frame.vp_map_depth(0));
			if(valid_initial)	cudaMemcpy(g_T_mat.vp(),zz_mat_4x4.vp(),16*sizeof(float),cudaMemcpyHostToDevice);
			zz_flag_integ = g_track.tp_Track_Pose(&g_state_obj,&g_rgbd_frame,valid_initial ? &g_T_mat : NULL);

			if(!zz_flag_integ) return false;

			// GPU -> CPU.
			CKvMatrixFloat t_ext; t_ext.c_Create(4,4,0.0f);

			cudaMemcpy(t_ext.vp(),g_state_obj.vp_T_gc(),16*sizeof(float),cudaMemcpyDeviceToHost);
			p_pose->set_from_transform(&t_ext);

			//cudaMemcpy(t_ext.vp(),g_state_obj.vp_T_cg(),16*sizeof(float),cudaMemcpyDeviceToHost);
			//p_pose->set_from_transform_inv(&t_ext);

			// Get rigid transformation of depth camera.
			// compute P matrix.
			// + for RGB camera.
			zz_state.compute_P_matrix(p_intrins,p_pose,zz_state.p_P_matrix_RGB());
			// + for depth camera.
			p_pose_rgb_d->get_pose_successive(*p_pose,*p_pose_rgb_d,*p_pose_raw);
			zz_state.compute_P_matrix(p_intrins_raw,p_pose_raw,zz_state.p_P_matrix_depth());
		}	
		
	}

	//////////////////////////////////////////////////////////////////////////
	// Drift detection 3rd.
	//////////////////////////////////////////////////////////////////////////
	if(KV_MODE_DRIFT_DETECT){
		if(!zz_tracker.cmv_Check_Cube_Position(&zz_cube, &zz_state)){
			printf("cube is out-of-sight!!\n");
			return false;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// update pose from t0 to t1.
	zz_extrins_t0_t1.get_pose_relative(zz_extrin, *p_pose, zz_extrins_t0_t1);
	//////////////////////////////////////////////////////////////////////////
		
	//if(zz_flag_integ)	z_uct_Update_Camera_Trace(zz_state,&zz_trace);

	return zz_flag_integ;
}


//********************************************************************************************
void CKvYooji_3D_Object_Scanner::z_uom_Update_Object_Model(
	bool in_flag_color,
	bool in_flag_gpu)
//********************************************************************************************
{
	int ww,hh;

 	zz_rgbd_frame.p_map_depth_raw_on_RGB()->ms(ww,hh);
 
   	g_integ_obj.cdtocc_Convert_Depth_to_TSDF_on_Cube_with_Color(
   		&g_cube_obj,
   		//g_map_depth_t1.vp(),
   		g_map_depth_t1_raw_, //g_map_depth_t1_filtered,
   		g_img_color_t1_,
   		ww,hh,
   		g_state_obj.vp_T_gc(),g_state_obj.vp_T_cg(),
   		true);

}

//********************************************************************************************
void CKvYooji_3D_Object_Scanner::z_uts_Update_Tracking_State(
	int in_mode_track,
	int in_type_of_data,
	int in_level_of_pyram,
	bool in_flag_gpu)
//********************************************************************************************
{
	//CKvScreen sc[4];

	Vector3f cam_cen,light;
	int ww,hh;
	float T_gc_host[16];

	uchar *p_img_normal = zz_state.p_image_normals_rendered()->mps(ww,hh)[0];
	uchar *p_img_texture = zz_state.p_image_texture_rendered()->vp();
	float *p_map_depth = zz_state.p_map_depth_rendered()->vp();

//	float *p_pyram_depth	= zz_state.p_pyramid_map_depth()->imgs[0].vp();
// 	CKvPoint3Df *p_pyram_p3d	= zz_state.p_pyramid_map_p3d()->imgs[0].vp();
// 	CKvPoint3Df *p_pyram_normal	= zz_state.p_pyramid_map_normals()->imgs[0].vp();

	if(!zz_pointer_p3d) zz_pointer_p3d = new float[3*ww*hh];

	//////////////////////////////////////////////////////////////////////////
 	// set light vector.	
  	cudaMemcpy(T_gc_host,g_state_obj.vp_T_gc(),16*sizeof(float),cudaMemcpyDeviceToHost);
  	//light.x = -T_gc_host[8];	light.y = -T_gc_host[9];	light.z = -T_gc_host[10];	light.normalised();
	light.x = 0.0f;	light.y = 0.0f;	light.z = -1.0f;
  	
   	// Render ICP maps.
   	if(in_type_of_data == KV_TYPE_INPUT_SCENE){
   		//g_render_obj.rmi_Render_Maps_for_ICP(
   		g_render_obj.rmis_Render_Maps_for_Scene(
   			&g_cube_obj,
   			ww,hh,
   			g_state_obj.vp_T_gc(),g_state_obj.vp_T_cg(),
   			g_state_obj.center,light,
   
   			g_state_obj.vp_map_depth(0),
   			g_state_obj.vp_map_vertex(0),
   			g_state_obj.vp_map_normal(0),
   			g_state_obj.vp_img_norm(0),
   
   			g_state_obj.vp_img_texture(0));
   	}
   	else{
  		int tww, thh; tww = ww; thh = hh;
  		for(int i=0; i<in_level_of_pyram; i++){
  			//////////////////////////////////////////////////////////////////////////
  			g_render_obj.rmi_Render_Maps_for_ICP(
  				&g_cube_obj,
  				tww,thh,
  				g_state_obj.vp_T_gc(),g_state_obj.vp_T_cg(),
  				g_state_obj.center,light,
  
  				i,
  
  				g_state_obj.vp_map_depth(i),
  				g_state_obj.vp_map_vertex(i),
  				g_state_obj.vp_map_normal(i),
  				g_state_obj.vp_img_norm(i),
  
  				g_state_obj.vp_img_texture(i));
  
 // 			// for debugging.
 //    			CKvMatrixUchar img_norm; img_norm.c_Create(thh,tww)[0];
 //    			cudaMemcpy(img_norm.vp(),g_state_obj.vp_img_norm(i),tww * thh * sizeof(uchar),cudaMemcpyDeviceToHost);
 //    			sc[i].s_d_Display(&img_norm);
  		
  			tww /= 2; thh /= 2;
  		}
  		//////////////////////////////////////////////////////////////////////////
  
  		//if(!Kv_Printf("Render pyramid!")) exit(0);
  
   	} 
  
  	// update rendering results.
  	// + depth map for display.
  	//cudaMemcpy(p_map_depth,g_state_obj.vp_map_depth(0),ww * hh * sizeof(float),cudaMemcpyDeviceToHost);		// for GPU tracking.
  	//////////////////////////////////////////////////////////////////////////
  	// + depth map for tracking.
  //	cudaMemcpy(p_pyram_depth,g_state_obj.vp_map_depth(),ww * hh * sizeof(float),cudaMemcpyDeviceToHost);	// for CPU tracking.
  // 	cudaMemcpy(p_pyram_depth,zz_rgbd_frame.p_pyramid_map_depth()->imgs[0].vp(),
  // 		ww * hh * sizeof(float),cudaMemcpyHostToHost);	// for CPU tracking.
  	//////////////////////////////////////////////////////////////////////////
  	
	////////////////////////////////////////////////////////////////////////////
 	//// for frame-to-model tracking.
	if(KV_COMPUTE_ON_CPU){
		int tww,thh;
		tww = ww; thh = hh;
		for(int k=0; k<in_level_of_pyram; k++){
     		
     		float *p_pyram_depth	= zz_state.p_pyramid_map_depth_rendered()->imgs[k].vp();
     		CKvPoint3Df *p_pyram_p3d	= zz_state.p_pyramid_map_p3d_rendered()->imgs[k].vp();
     		CKvPoint3Df *p_pyram_normal	= zz_state.p_pyramid_map_normals_rendered()->imgs[k].vp();
     
     		// + depth map.
     		cudaMemcpy(p_pyram_depth,g_state_obj.vp_map_depth(k),tww*thh * sizeof(float),cudaMemcpyDeviceToHost);		
     		// + vertex map.
     		cudaMemcpy(zz_pointer_p3d,g_state_obj.vp_map_vertex(k),3*tww*thh*sizeof(float),cudaMemcpyDeviceToHost);
     		// import updated rendered vertex(Vector3f) to Point3Df image.
     		for(int i = 0; i<tww*thh; i++){
     			p_pyram_p3d[i].x = zz_pointer_p3d[3*i];
     			p_pyram_p3d[i].y = zz_pointer_p3d[3*i + 1];
     			p_pyram_p3d[i].z = zz_pointer_p3d[3*i + 2];
     		}
     		// + normal map.
     		cudaMemcpy(zz_pointer_p3d,g_state_obj.vp_map_normal(k),3*tww*thh*sizeof(float),cudaMemcpyDeviceToHost);
     		// import updated rendered normal(Vector3f) to Point3Df image.
     		for(int i = 0; i<tww*thh; i++){
     			p_pyram_normal[i].x = zz_pointer_p3d[3*i];
     			p_pyram_normal[i].y = zz_pointer_p3d[3*i + 1];
     			p_pyram_normal[i].z = zz_pointer_p3d[3*i + 2];
     		}
     
     		tww /= 2; thh /= 2;
		}
	}
	//////////////////////////////////////////////////////////////////////////
 	
   	// + normal image.
   	cudaMemcpy(p_img_normal,/*g_img_normal_t0*/g_state_obj.vp_img_norm(0),ww * hh * sizeof(uchar),cudaMemcpyDeviceToHost);
   	
   	// + texture image.
   	cudaMemcpy(p_img_texture,g_state_obj.vp_img_texture(0),ww * hh * 3 * sizeof(uchar),cudaMemcpyDeviceToHost);
   
   	// update previous camera pose.
   	cudaMemcpy(g_T_gc_t0,g_state_obj.vp_T_gc(),16*sizeof(float),cudaMemcpyDeviceToDevice);
   	cudaMemcpy(g_T_cg_t0,g_state_obj.vp_T_cg(),16*sizeof(float),cudaMemcpyDeviceToDevice);
	//////////////////////////////////////////////////////////////////////////

	// update previous image pyramids. 	
	//////////////////////////////////////////////////////////////////////////
	if(KV_COMPUTE_ON_CPU){
		// for frame-to-frame tracking.
 		zz_state.p_pyramid_map_depth()->cp_Copy(zz_rgbd_frame.p_pyramid_map_depth());
 		zz_state.p_pyramid_map_p3d()->cp_Copy(zz_rgbd_frame.p_pyramid_map_p3d());
 		zz_state.p_pyramid_map_normals()->cp_Copy(zz_rgbd_frame.p_pyramid_map_normals());

		//////////////////////////////////////////////////////////////////////////

		// for sift.
		zz_state.p_pyramid_image_gray()->cp_Copy(zz_rgbd_frame.p_pyramid_image_gray());


		if(!KV_MODE_DEPTH_ONLY){
 			zz_state.p_pyramid_map_edge()->cp_Copy(zz_rgbd_frame.p_pyramid_map_edge());
			zz_state.p_pyramid_map_grad_edge()->cp_Copy(zz_rgbd_frame.p_pyramid_map_grad_edge());
			zz_state.p_pyramid_map_depth_edge()->cp_Copy(zz_rgbd_frame.p_pyramid_map_depth_edge());
		}
		zz_state.p_image_RGB()->cp_Copy(zz_rgbd_frame.p_image_rgb());

	}

	zz_state.p_roi_depth()->cp_Copy(zz_rgbd_frame.p_roi_depth());
	zz_state.p_roi_image_rgb()->cp_Copy(zz_rgbd_frame.p_roi_image_rgb());
	zz_state.p_roi_image_gray()->cp_Copy(zz_rgbd_frame.p_roi_image_gray());
	//zz_state.p_roi_edge()->cp_Copy(zz_rgbd_frame.p_roi_edge());
	(*zz_state.p_roi_object()) = (*zz_rgbd_frame.p_roi_object());	
	
	z_uct_Update_Camera_Trace(zz_state, &zz_trace);	


	//////////////////////////////////////////////////////////////////////////
	// Bounding sphere for measuring coverage and selecting keypframe.
	//////////////////////////////////////////////////////////////////////////
	zz_disp.dcps_Display_Camera_Projection_Sphere(
		//&zz_g3d,
		NULL,
		&zz_state,
		&zz_p3d_sphere,
		&zz_rgbf_sphere,
		&zz_mesh_sphere);

	//////////////////////////////////////////////////////////////////////////
	// check coverage and add key-template.
	// 여기는 back-end thread 에서 해야한다!!!
	// 오래걸리무....

	if(!KV_COMPUTE_ON_CPU){
		int tww, thh;
		float *p_pyram_depth = zz_state.p_pyramid_map_depth_rendered()->imgs[0].mps(tww,thh)[0];

		// + depth map.
		cudaMemcpy(p_pyram_depth,g_state_obj.vp_map_depth(0),tww*thh * sizeof(float),cudaMemcpyDeviceToHost);
	}

	float coverage = 100.0f*((float)zz_state.num_sphere_coverage()/(float)zz_p3d_sphere.ne_Number_of_Elements());
	if(coverage - zz_cover_prev > 5.0f){

 		z_gkt_Get_Key_Template();

// 		CKvScreen sc; 
// 		sc.s_d_Display(&set_of_templates.back());
// 		if(!Kv_Printf("Key-template!")) exit(0);

		zz_cover_prev = coverage;
	}


	//////////////////////////////////////////////////////////////////////////


// 	printf("===============================================================\n");
// 	printf("Coverage: %% %f\n", coverage);
// 	printf("===============================================================\n");
}

//********************************************************************************************
void CKvYooji_3D_Object_Scanner::z_std_Setting_for_Trace_Display()
//********************************************************************************************
{
// 	zz_extrins_init_depth.spm_Set_from_P_Matrix(zz_rgbd_frame.ppd_Pointer_of_Pmatrix_Depth());
// 	z_srctd_Set_Reference_Camera_for_Trace_Display(zz_extrins_canonical,
// 		zz_int_ref,
// 		zz_ext_ref);

	zz_extrins_init_rgb.set_from_P_matrix(zz_rgbd_frame.p_P_matrix_RGB());
	z_srctd_Set_Reference_Camera_for_Trace_Display(zz_extrins_d_rgb,
		zz_int_ref,
		zz_ext_ref);

	zz_state_trace.p_intrinsics_depth()->copy(&zz_int_ref);
	zz_state_trace.p_extrinsics_glob_to_cam()->copy(&zz_ext_ref);

	// for rendering reconstructed object.
// 	float fx, fy, px, py, dmin, dmax;
// 	float th, mu, max_w;
// 	int lev;
// 
// 	zz_params.gpicp_Get_Parameters_for_ICP_algorithm(lev, th, mu, max_w);
// 	zz_int_ref.get_params(fx, fy, px, py, dmin, dmax);
// 
	//////////////////////////////////////////////////////////////////////////
	// 이부분 추가하면 기존 g_render 와 충돌하는듯!!
// 	g_render_trace.ip_Initialize_Parameters(
// 		&g_cube_obj,640,480,
// 		fx, fy, px, py,
// 		0.03);
	//////////////////////////////////////////////////////////////////////////
// 
// 	// CPU -> GPU.
// 	cudaMemcpy(		
// 		g_T_mat.vp(),
// 		zz_ext_ref.mp_transform()->vp(),		
// 		16 * sizeof(float),
// 		cudaMemcpyHostToDevice);
// 	g_state_trace.set_transform(&g_T_mat);
// 
// 	// set light vector.	
// 	float *T_gc_host = zz_ext_ref.mp_transform()->vp();
// 	zz_light_trace.x = -T_gc_host[8];	zz_light_trace.y = -T_gc_host[9];	zz_light_trace.z = -T_gc_host[10];	zz_light_trace.normalised();


	//////////////////////////////////////////////////////////////////////////
	// initialize state for trace.
// 	float fx, fy, px, py, dmin, dmax;
// 	zz_int_ref.gp_Get_Parameters(fx, fy, px, py, dmin, dmax);
// 
// 	g_state_trace.create(Vector2i(640,640));
// 	cudaMemcpy(g_state_trace.vp_T_cg(),zz_ext_ref.mpi_Matrix_Pointer_Inverse()->vp(),16*sizeof(float),cudaMemcpyHostToDevice);
// 	cudaMemcpy(g_state_trace.vp_T_gc(),zz_ext_ref.mp_Matrix_Pointer()->vp(),16*sizeof(float),cudaMemcpyHostToDevice);
// 
// 	g_render_trace.ip_Initialize_Parameters(
// 		&g_cube_obj,640,640,
// 		fx, fy, px, py,
// 		0.03);
// 
// 	// set light vector.	
// 	float *T_gc_host = zz_ext_ref.mp_Matrix_Pointer()->vp();
// 	zz_light_trace.x = -T_gc_host[8];	zz_light_trace.y = -T_gc_host[9];	zz_light_trace.z = -T_gc_host[10];	zz_light_trace.normalised();

	//////////////////////////////////////////////////////////////////////////
}

//********************************************************************************************
void CKvYooji_3D_Object_Scanner::z_esp_Evaluate_System_Performance()
//********************************************************************************************
{
	float dist_rot, dist_trans;
	zz_eval.mee_Measure_Estimation_Error_using_Geodesic_Distance(
		&zz_extrins_init_depth,
		&zz_rgbd_frame,
		&zz_state,
		dist_rot,
		dist_trans);

	printf("Error: %f %f\n", dist_rot, dist_trans);
}

//********************************************************************************************
void CKvYooji_3D_Object_Scanner::z_gkt_Get_Key_Template()
//********************************************************************************************
{
	// Load template image.
	CKvMatrixFloat edge_mag_ori,edge_ori_ori,edge_mag_roi,edge_ori_roi,edge_mag_ori_filtered[3],edge_mag_roi_filtered,sub_blocks;

	// Initialize images.
	CKvMatrixUchar *p_roi_gray = zz_state.p_roi_image_gray();
	CKvMatrixFloat *p_roi_grad_mag;
	CKvMatrixFloat *p_roi_grad_ori;

	//////////////////////////////////////////////////////////////////////////
	// New key-template update.
	int ww_tmpl,hh_tmpl,idx_key;
	zz_state.p_roi_image_gray()->ms(ww_tmpl,hh_tmpl);
	zz_state.p_roi_edge()->grad_mag.c_Create(hh_tmpl,ww_tmpl,0.0f);	p_roi_grad_mag = &zz_state.p_roi_edge()->grad_mag;
	zz_state.p_roi_edge()->grad_ori.c_Create(hh_tmpl,ww_tmpl,0.0f);	p_roi_grad_ori = &zz_state.p_roi_edge()->grad_ori;
	
	idx_key = zz_set_of_templates.size();
	zz_set_of_templates.push_back(*p_roi_gray);
	zz_roi_ww[idx_key] = ww_tmpl;	zz_roi_hh[idx_key] = hh_tmpl;
	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	// New key-state update.
	int ww_ori, hh_ori;
	zz_state.p_image_depth_rendered()->ms(ww_ori, hh_ori);

	zz_set_of_key_states[idx_key].initialize(ww_ori, hh_ori, zz_state.p_pyramid_image_gray()->imgs.size());
	zz_set_of_key_states[idx_key].copy(&zz_state);

	zz_set_of_key_states[idx_key].zz_cnt_sphere = zz_cnt_sphere_last;

	// for frame-to-model tracking.
	zz_set_of_key_states[idx_key].p_pyramid_map_depth_rendered()->cp_Copy(zz_state.p_pyramid_map_depth_rendered());
	zz_set_of_key_states[idx_key].p_pyramid_map_p3d_rendered()->cp_Copy(zz_state.p_pyramid_map_p3d_rendered());
	zz_set_of_key_states[idx_key].p_pyramid_map_normals_rendered()->cp_Copy(zz_state.p_pyramid_map_normals_rendered());

	// for sift.
	zz_set_of_key_states[idx_key].p_pyramid_image_gray()->cp_Copy(zz_state.p_pyramid_image_gray());
	//////////////////////////////////////////////////////////////////////////

	if(!KV_MODE_DEPTH_ONLY){
		zz_set_of_key_states[idx_key].p_pyramid_map_edge()->cp_Copy(zz_state.p_pyramid_map_edge());
		zz_set_of_key_states[idx_key].p_pyramid_map_grad_edge()->cp_Copy(zz_state.p_pyramid_map_grad_edge());
		zz_set_of_key_states[idx_key].p_pyramid_map_depth_edge()->cp_Copy(zz_state.p_pyramid_map_depth_edge());
	}

	zz_set_of_key_states[idx_key].p_roi_depth()->cp_Copy(zz_state.p_roi_depth());
	zz_set_of_key_states[idx_key].p_roi_image_rgb()->cp_Copy(zz_state.p_roi_image_rgb());
	zz_set_of_key_states[idx_key].p_roi_image_gray()->cp_Copy(zz_state.p_roi_image_gray());
	//zz_state.p_roi_edge()->cp_Copy(zz_rgbd_frame.p_roi_edge());
	(*zz_set_of_key_states[idx_key].p_roi_object()) = (*zz_state.p_roi_object());

	zz_set_of_key_states[idx_key].p_image_RGB()->cp_Copy(zz_state.p_image_RGB());
	//////////////////////////////////////////////////////////////////////////

	// check key-states.
// 	zz_jm.dmfpc_Display_Matrix_Float_using_Pseudo_Color(
// 		&m_sc[0],
// 		zz_set_of_key_states[idx_key].p_pyramid_map_depth_rendered()->imgs[0],
// 		true);
// 	m_sc[1].s_d_Display(zz_set_of_key_states[idx_key].p_image_RGB());
// 	if(!Kv_Printf("key states!")) exit(0);
	//////////////////////////////////////////////////////////////////////////

//	printf("idx_key: %d (%d %d)\n", idx_key, ww_tmpl, hh_tmpl);

	//////////////////////////////////////////////////////////////////////////
	// Extract dominant gradient template extraction.
	//////////////////////////////////////////////////////////////////////////
	int num_bins = 16;		
	int min_block_sz=10;
	int mask_size;
	///Make gradient map.
	zz_jm.exso_Edge_Extraction_using_Sobel_Operator(*p_roi_gray,*p_roi_grad_mag,*p_roi_grad_ori);
	printf(".");
	/// Dominant Gradient Enhancement.
	printf(".");
	/// processing template image.
	// Make sub block.
	zz_hist_ROI[idx_key].csbiqt_Compose_Sub_Blocks_of_Image_using_Quad_Tree(
		p_roi_grad_mag,//&edge_mag_roi, //
		p_roi_grad_ori,
		min_block_sz,
		3,
		&sub_block_tree[idx_key],
		0);
	printf(".");
	p_roi_gray->m_Mean_and_Stdev(zz_var_ROI[idx_key]);		zz_var_ROI[idx_key] = SQUARE(zz_var_ROI[idx_key]);
	zz_hist_ROI[idx_key].cnhogwsbt_Compute_Normalized_Histogram_of_Oriented_Gradient_of_Weighted_Sub_Block_Tree(
		p_roi_grad_mag, //&edge_mag_roi, //
		p_roi_grad_ori,
		&sub_block_tree[idx_key],
		num_bins,
		KV_HIST_NORMALIZE_L1_NORM,
		0,
		&weight_vector_tree[idx_key]);
	printf(".\n");




	//
	

	///
}

//********************************************************************************************
bool CKvYooji_3D_Object_Scanner::rcd_Recover_Camera_Drift(CKvYooji_MatrixRgbD *in_frame_of_rgbd_camera,
	CKvMatrixUcharRgb *out_results)
//********************************************************************************************
{
	CKvYooji_MatrixRgbD *p_rgbd = in_frame_of_rgbd_camera;
	CKvMatrixFloat zz_sum_image_ori,zz_sum_of_square_image_ori;
	CKvVectorInt valid_block_indices;

	float block_step_ww,block_step_hh,block_offset_ww,block_offset_hh,block_ratio_ww,block_ratio_hh;
	int num_bins,hist_array_width;

	float max_mask_ratio,min_mask_ratio;
	int max_mask_sz,min_mask_sz,step_sz;

	int mask_size = 0;

	int lev_pyram = 2;

	//////////////////////////////////////////////////////////////////////////
	// counting processed frame.
	zz_number_of_processed_frames++;
	//////////////////////////////////////////////////////////////////////////

	///Original initialization
	num_bins = 16;		zz_min_block_sz=10;
	// =======================================
	max_mask_ratio = 1.5f;
	min_mask_ratio = 0.5f;
	// =======================================
	// 	in_img_roi.ms(roi_ww,roi_hh);
	// 	edge_mag_roi.c_Create(roi_hh,roi_ww,0.0f);
	// 	edge_ori_roi.c_Create(roi_hh,roi_ww,0.0f);
	// 	edge_mag_roi_filtered.c_Create(roi_hh,roi_ww,0.0f);

	zz_num_imgs_tmpl = zz_set_of_templates.size();

	//////////////////////////////////////////////////////////////////////////
	zz_ycc.mrg_Matrix_Rgb_to_Gray(p_rgbd->p_image_rgb(), false, &p_rgbd->p_pyramid_image_gray()->imgs[0]);
	//////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////
	CKvMatrixUchar *cp_img_gray = &p_rgbd->p_pyramid_image_gray()->imgs[0];	// scale...
	CKvMatrixFloat *cp_map_grad_mag;
	CKvMatrixFloat *cp_map_grad_ori;

	int idx_key;
	cp_img_gray->ms(zz_ori_ww,zz_ori_hh);
	p_rgbd->p_map_edge()->grad_mag.c_Create(zz_ori_hh,zz_ori_ww,0.0f);	cp_map_grad_mag = &p_rgbd->p_map_edge()->grad_mag;
	p_rgbd->p_map_edge()->grad_ori.c_Create(zz_ori_hh,zz_ori_ww,0.0f);	cp_map_grad_ori = &p_rgbd->p_map_edge()->grad_ori;

	zz_sum_image_ori.c_Create(zz_ori_hh,zz_ori_ww,0.0f);
	zz_sum_of_square_image_ori.c_Create(zz_ori_hh,zz_ori_ww,0.0f);
	//////////////////////////////////////////////////////////////////////////

	printf("size: %d %d\n", zz_ori_ww, zz_ori_hh);

	printf(".");
	///Processing target image.
	zz_jm.exso_Edge_Extraction_using_Sobel_Operator(*cp_img_gray,*cp_map_grad_mag,*cp_map_grad_ori);
	/// ///////////////////////////////////////////////////////////////////////////////////////////
	// dominant gradient enhancement of target image.
// 	sc.s_d_Display(cp_img_gray);
// 	if(!Kv_Printf("Image target")) exit(0);

	zz_jm.mii_Make_Integral_Image(
		cp_img_gray,
		&zz_sum_image_ori,
		&zz_sum_of_square_image_ori);

	zz_hist_ori.mihog_Make_Integral_Histogram_Of_Gradient(
		cp_map_grad_mag, //&edge_mag_ori, //
		cp_map_grad_ori,
		num_bins,
		&zz_integral_hist_ori[0]);

// 	zz_jm.dmf_Display_Matrix_Float(&sc,*cp_map_grad_mag, true);
// 	if(!Kv_Printf("Edge target")) exit(0);

	//printf("hmihwsbtn_Histogram_Matching_using_Intergral_Histogram_of_Weighted_Sub_Block_Tree?");


	float max_sim = -100000.0f;
	int max_idx = 0;

	for(int k=0; k<zz_num_imgs_tmpl; k++){

		///ROI initialization	
		// =======================================================
		max_mask_sz = min((int)(max_mask_ratio*zz_roi_ww[k]),zz_ori_ww);
		min_mask_sz = max(40,(int)(min_mask_ratio*zz_roi_ww[k]));
		step_sz = max(1,(int)(0.01f*zz_ori_ww));
		// =======================================================
		
	
		/// ///////////////////////////////////////////////////////////////////////////////////////////	

		printf(".");
		///Histogram matching between ROI image and original image	
		float sim = hmihwsbtn_Histogram_Matching_using_Intergral_Histogram_of_Weighted_Sub_Block_Tree(
			zz_ori_ww, zz_ori_hh,
			zz_roi_ww[k], zz_roi_hh[k],

			zz_var_ROI[k], //-1.0f, //
			&zz_hist_ROI[k],
			&zz_sum_image_ori,
			&zz_sum_of_square_image_ori,
			&zz_integral_hist_ori,
			&sub_block_tree[k],
			&weight_vector_tree[k],//NULL,//
			mask_size,
			zz_min_block_sz,
			KV_HIST_SIMILARITY_OVERLAP,
			KV_HIST_NORMALIZE_L1_NORM,
			0,
			max_mask_sz, //min(3*roi_ww, ori_ww),
			min_mask_sz, //max(40, (int)(0.05f*roi_ww)),
			step_sz, //max(1, (int)(0.01f*ori_ww)),
			0.1f,
			0,
			&zz_LT[k],
			&zz_RB[k],
			zz_out_tr_level);
		printf(".");

		//////////////////////////////////////////////////////////////////////////
		if(sim > max_sim){
			max_sim = sim;
			max_idx = k;
		}
		//////////////////////////////////////////////////////////////////////////


		// 		fn_tar.fm_Format("%s//%d.bmp", dn_save.bp(), n);
		// 		aa_hh.iofv.si_Save_Image(fn_tar, false, &in_img_ori);

	}

	//delete[] zz_integral_hist_ori;

	printf("[#%d key-template] sim: %f\n",max_idx,max_sim);

	//////////////////////////////////////////////////////////////////////////
	// display re-localization mode.
	if(out_results){

		out_results->cp_Copy(in_frame_of_rgbd_camera->p_image_rgb());
	
		zz_disp.pt_Put_Text_using_OpenCV(*out_results);
// 		m_sc[0].s_d_Display(out_results);
// 		if(!Kv_Printf("Put text!")) exit(0);
	}
	//////////////////////////////////////////////////////////////////////////
	// check maximum similarity.
	// double check!
	if(max_sim < 0.85f)	return false;
	
	// center of roi check!
	Point center_tmpl;
	Point center_key_roi = 0.5f*(
		zz_set_of_key_states[max_idx].p_roi_object()->tl()+
		zz_set_of_key_states[max_idx].p_roi_object()->br());

	center_tmpl.x = 0.5f*(zz_LT[max_idx].x + zz_RB[max_idx].x);
	center_tmpl.y = 0.5f*(zz_LT[max_idx].y + zz_RB[max_idx].y);

	//////////////////////////////////////////////////////////////////////////
	float dist = sqrt(SQUARE(center_tmpl.x - center_key_roi.x) 
		+ SQUARE(center_tmpl.y - center_key_roi.y));

	printf("center dist: %f (%d %d - %d %d)\n",dist,center_tmpl.x,center_tmpl.y, center_key_roi.x
 		,center_key_roi.y);

	if(dist > 10) return false;

	if(!Kv_Printf("center dist: %f (%d %d - %d %d)\n",dist,center_tmpl.x,center_key_roi.x
		,center_tmpl.y,center_key_roi.y)) exit(0);
	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////

	/// Display matching results.
	int pos_x,pos_y,win_ww,win_hh,delta_ww,delta_hh,true_ww;
	float true_scale;
	win_ww = zz_RB[max_idx].x-zz_LT[max_idx].x+1;	win_hh = zz_RB[max_idx].y-zz_LT[max_idx].y+1;
	/// ////////////////////////////////////////////////////////////////
	delta_ww = (int)(2.0f*(float)mask_size/(float)(zz_roi_ww[max_idx]-2.0f*(float)mask_size)*(float)win_ww);
	delta_hh = (int)(2.0f*(float)mask_size/(float)(zz_roi_hh[max_idx]-2.0f*(float)mask_size)*(float)win_hh);
	win_ww += delta_ww;			win_hh += delta_hh;
	pos_x = zz_LT[max_idx].x-delta_ww/2;	pos_y = zz_LT[max_idx].y-delta_hh/2;
	//////////////////////////////////////////////////////////////////////////

	/// Display matching results.
	CKvMatrixUcharRgb img_disp;	img_disp.cp_Copy(p_rgbd->p_image_rgb());
	img_disp.sbox_Set_Box(pos_x,pos_y,win_ww,win_hh,Kv_Rgb((unsigned char)(255),(unsigned char)(255),(unsigned char)(255)));
	img_disp.sbox_Set_Box(pos_x-1,pos_y-1,win_ww+2,win_hh+2,Kv_Rgb((unsigned char)(255),(unsigned char)(255),(unsigned char)(255)));
	m_sc[0].s_d_Display(&img_disp);
	m_sc[1].s_d_Display(&zz_set_of_templates[max_idx]);				aa_uw.sp_Set_Position(&m_sc[1],&m_sc[0],0);

	Kv_Pause(10);

	//printf("[#%d key-template] sim: %f\n",max_idx,max_sim);

	if(!Kv_Printf("SITM")) exit(0);

	
	//////////////////////////////////////////////////////////////////////////
	// update current pose using key-pose.
	//////////////////////////////////////////////////////////////////////////
	zz_state.copy(&zz_set_of_key_states[max_idx]);

	//////////////////////////////////////////////////////////////////////////
	// for frame-to-model tracking.
	zz_state.p_pyramid_map_depth_rendered()->cp_Copy(zz_set_of_key_states[max_idx].p_pyramid_map_depth_rendered());
	zz_state.p_pyramid_map_p3d_rendered()->cp_Copy(zz_set_of_key_states[max_idx].p_pyramid_map_p3d_rendered());
	zz_state.p_pyramid_map_normals_rendered()->cp_Copy(zz_set_of_key_states[max_idx].p_pyramid_map_normals_rendered());

	// for sift.
	zz_state.p_pyramid_image_gray()->cp_Copy(zz_set_of_key_states[max_idx].p_pyramid_image_gray());
	//////////////////////////////////////////////////////////////////////////

	if(!KV_MODE_DEPTH_ONLY){
		zz_state.p_pyramid_map_edge()->cp_Copy(zz_set_of_key_states[max_idx].p_pyramid_map_edge());
		zz_state.p_pyramid_map_grad_edge()->cp_Copy(zz_set_of_key_states[max_idx].p_pyramid_map_grad_edge());
		zz_state.p_pyramid_map_depth_edge()->cp_Copy(zz_set_of_key_states[max_idx].p_pyramid_map_depth_edge());
	}

	zz_state.p_roi_depth()->cp_Copy(zz_set_of_key_states[max_idx].p_roi_depth());
	zz_state.p_roi_image_rgb()->cp_Copy(zz_set_of_key_states[max_idx].p_roi_image_rgb());
	zz_state.p_roi_image_gray()->cp_Copy(zz_set_of_key_states[max_idx].p_roi_image_gray());
	//zz_state.p_roi_edge()->cp_Copy(zz_rgbd_frame.p_roi_edge());
	(*zz_state.p_roi_object()) = (*zz_set_of_key_states[max_idx].p_roi_object());

	zz_state.p_image_RGB()->cp_Copy(zz_set_of_key_states[max_idx].p_image_RGB());

	// check template.
// 	zz_jm.dmfpc_Display_Matrix_Float_using_Pseudo_Color(
// 		&m_sc[0],
// 		zz_state.p_pyramid_map_depth_rendered()->imgs[0],
// 		true);
// 
// 	m_sc[1].s_d_Display(&zz_state.p_pyramid_image_gray()->imgs[0]);
// 
// 	if(!Kv_Printf("key states!")) exit(0);
	//////////////////////////////////////////////////////////////////////////

	return true;
}

//*********************************************************************************************************
float CKvYooji_3D_Object_Scanner::hmihwsbtn_Histogram_Matching_using_Intergral_Histogram_of_Weighted_Sub_Block_Tree(
	int in_ww_tar, int in_hh_tar,
	int in_ww_roi, int in_hh_roi,
	const float in_variance_ROI,				// if you do not want to use this, please enter any negative number.
	CKvHistogram *in_hist_ROI,
	CKvMatrixFloat *in_sum_image_target,
	CKvMatrixFloat *in_sum_of_square_image_target,
	CKvSet_of_MatrixInt **in_set_of_integral_hist_target,
	CKvSet_of_MatrixFloat *in_set_of_sub_block_position_ratios,
	CKvSet_of_VectorFloat *in_set_of_weight_vectors,
	int in_mask_sz_of_hist_filter,
	int in_minimum_block_size,
	int in_vector_distance_type,
	int in_normalize_mode,
	int in_dominant_gradient_reduction_mode,
	int in_max_window_width,
	int in_min_window_width,
	int in_window_size_delta,
	float in_window_move_ratio,
	int in_show_similarity_map,
	CKvPoint *out_LT,
	CKvPoint *out_RB,
	int &out_tree_level,
	bool in_mode_dge)
//*********************************************************************************************************
{

// 	CKvContourDetectionJM zz_cd;	
// 	LCKvUtility_for_YCbCr aa_ycc;
// 	LCKvUtility_for_Windows aa_uw;

	int change_size_flag=0;
	float mean_sim;

	int num_bins, block_num, hist_vector_dim;
	int x,y,ww, hh,mask_size_x, mask_size_y, pixel_increase_x, pixel_increase_y, cnt=0, pass_cnt=0;
	int max_mask_size_x, max_mask_size_y,min_mask_size_x, min_mask_size_y, mask_size_step;
	/// /////////////////////////////////
	int new_ww, new_hh;
	int min_blk_sz, max_tr_lev, tr_lev, max_hist_vector_dim, total_block_num;
	float *p_hist_roi_select;

	float *p_weight_vector;
	/// ///////////////////////////////////
	int coarse_mask_sz_x, coarse_mask_sz_y, coarse_x, coarse_y;
	int max_x, max_y, max_ww, max_hh, tr_lev_local;
	int *p_in_valid_block_indices;
	float *hist_from_integral_hist, *p_hist_roi, *p_similarity_map;
	float aspect_ratio, distance, temp_var, min_dist_global, min_dist_local, similarity, max_sim_global, max_sim_local, sim_diff;
	CKvMatrixFloat edge_mag_roi, edge_ori_roi;
	CKvMatrixFloat similarity_map, zeros, max_sim_map;
	CKvMatrixInt *p_integral_hist;
	CKvHistogram integral_hist;

	//For debugging	
// 	CKvMatrixUcharRgb zz_temp_img;
// 	CKvScreen zz_scr[2];
	
//	FILE *fp;
// 	fopen_s(&fp, "max_similarities_for_scale.txt", "w");
// 	if(in_vector_distance_type==KV_HIST_SIMILARITY_OVERLAP)		fprintf_s(fp, "scale	similarity\n");
// 	else																												fprintf_s(fp, "scale	distance\n");

	//Initialization
	ww = in_ww_tar; hh = in_hh_tar;
	num_bins = (*in_set_of_integral_hist_target)[0].vs();
	min_blk_sz = in_minimum_block_size;
	max_tr_lev = in_set_of_sub_block_position_ratios->vs();
	max_hist_vector_dim = num_bins*in_set_of_sub_block_position_ratios->gpe(max_tr_lev-1)->mw();
//printf("min_blk_sz: %d\nmax_tr_lev: %d\nmax_hist_vector_dim: %d\n", min_blk_sz, max_tr_lev, max_hist_vector_dim)	;
	total_block_num=0;	
	for(int i=0; i<max_tr_lev; i++)	total_block_num+=in_set_of_sub_block_position_ratios->gpe(i)->mw();
	
	//hist_vector_dim = num_bins*block_num;
	hist_from_integral_hist   = new float[max_hist_vector_dim];		for(int i=0; i<max_hist_vector_dim; i++)		hist_from_integral_hist[i]=0.0f;
	p_hist_roi = in_hist_ROI->hp_Histogram_Pointer();

	//////////////////////////////////////////////////////////////////////////
	// max mask size 가 이상??
	// max mask size 가 이상??
	new_ww=in_ww_roi-2*in_mask_sz_of_hist_filter;		new_hh=in_hh_roi-2*in_mask_sz_of_hist_filter;
	aspect_ratio			= (float)new_hh/(float)new_ww;
	max_mask_size_x = min(in_max_window_width, ww);			max_mask_size_y = (int)(aspect_ratio*max_mask_size_x);
	if(max_mask_size_y>hh){
		max_mask_size_y=hh;
		max_mask_size_x=(int)((float)max_mask_size_y/aspect_ratio);
	}	
	min_mask_size_x = max(in_min_window_width, 4);
	min_mask_size_y = (int)(aspect_ratio*min_mask_size_x);
	mask_size_step = in_window_size_delta;
	//////////////////////////////////////////////////////////////////////////

	//For debugging	
//	CKvMatrixUcharRgb temp_img;
//  	if(in_show_similarity_map){
//  		temp_img.c_Create(hh, ww);						temp_img.cp_Copy(in_img);
//   		p_similarity_map = similarity_map.c_Create(hh, ww, 0.0f)[0];
//   		max_sim_map.c_Create(hh, ww, 0.0f);
//   		zeros.c_Create(hh, ww, 0.0f);
//  	}
	//Kv_Printf("%d, %d", in_min_window_width, in_sub_block_position_ratios->mw());
	///Histogram_Matching_using_Intergral_Histogram
	//--start: coarse matching.
	//tree initialization.
	tr_lev=2;		if(new_ww/2<min_blk_sz || new_hh/2 <min_blk_sz)		tr_lev=1;
	p_weight_vector=in_set_of_weight_vectors->gpe(tr_lev-1)->vps(block_num);
	hist_vector_dim=num_bins*block_num;

	p_hist_roi_select = p_hist_roi;
	for(int i=0; i<tr_lev-1; i++){
		p_hist_roi_select+=num_bins*in_set_of_sub_block_position_ratios->gpe(i)->mw();
	}

	max_sim_global = max_sim_local = -100000.0f;
	min_dist_global = min_dist_local = 100000.0f;	
	mask_size_x = min_mask_size_x;	mask_size_y = min_mask_size_y;
	while(mask_size_x<=max_mask_size_x && mask_size_y<=max_mask_size_y){		//mask_size_x = 105;		mask_size_y = (int)(aspect_ratio*mask_size_x);

		if(in_show_similarity_map){
			pixel_increase_x=pixel_increase_y=1;
			similarity_map.cp_Copy(&zeros);
		}
		else{
			pixel_increase_x = max(1, (int)(mask_size_x*in_window_move_ratio));
			pixel_increase_y = max(1, (int)(aspect_ratio*pixel_increase_x));
		}

	
		/// //////////////////////////////////////////////////////////////////
		int max_local_x, max_local_y;
		mean_sim=0.0f;	cnt=0;
		for(y=0; y<hh-(mask_size_y); y+=pixel_increase_y){
			for(x=0; x<ww-(mask_size_x); x+=pixel_increase_x){				

				// variance matching.
				if(!in_show_similarity_map && (in_variance_ROI > 0.0f)){
					zz_cd.gvii_Get_Variance_from_Integral_Image(in_sum_image_target,
						in_sum_of_square_image_target,
						x, y, 
						mask_size_x, mask_size_y,
						temp_var);	

					if(temp_var < in_variance_ROI*0.5f){		pass_cnt++;	continue;	}
				}

				/// HOG matching.
				// Get histogram of window from integral histogram.
				int hist_mask_level;

				//////////////////////////////////////////////////////////////////////////
				if(in_mode_dge){
					hist_mask_level=(int)(mask_size_x*(float)in_mask_sz_of_hist_filter/(float)in_ww_roi);
					if(hist_mask_level<2)			hist_mask_level=0;
					else if(hist_mask_level<5)		hist_mask_level=1;
					else							hist_mask_level=2;
				}
				else hist_mask_level = 0;				
				//////////////////////////////////////////////////////////////////////////

				integral_hist.gnhwsbih_Get_Normalized_Histogram_of_Weighted_Sub_Blocks_from_Integral_Histogram(
					&((*in_set_of_integral_hist_target)[hist_mask_level]), 
					x, y,
					mask_size_x, mask_size_y, 
					in_set_of_sub_block_position_ratios->gpe(tr_lev-1),
					in_normalize_mode,
					in_dominant_gradient_reduction_mode,
					hist_from_integral_hist);

				// Histogram similarity			
				// using similarity.
				similarity = zz_jm.cwhvs_Caculate_Weighted_Histogram_Vector_Similarity_using_Overlapped_Intensity(
					hist_from_integral_hist,
					p_hist_roi_select,
					p_weight_vector,
					block_num,
					num_bins);			
				
				mean_sim+=similarity;		cnt++;
					
				// local maximum similarity.
				if(max_sim_local<similarity){
					max_sim_local = similarity;			
					max_local_x = x;
					max_local_y = y;
					tr_lev_local = tr_lev;
				}
				if(in_show_similarity_map)		p_similarity_map[(y+mask_size_y/2)*ww+(x+mask_size_x/2)] = similarity;

				

			}
		}

		mean_sim/=(float)cnt;
		sim_diff=max_sim_local-mean_sim;
		/// extracting similarity-weighted position.
		float sw_x, sw_y, total_sim, stdv;	
		float *p_sim_map;		
		p_sim_map=similarity_map.vp();

		// compute stdv.
// 		stdv=0.0f;
// 		for(int j=mask_size_y/2; j<hh-mask_size_y/2; j++){	for(int i=mask_size_x/2; i<ww-mask_size_x/2; i++){	stdv+=SQUARE(p_sim_map[j*ww+i]-mean_sim);	}	}		
// 		stdv=sqrt(stdv/(float)(cnt));
// 		total_sim=sw_x=sw_y=0.0f;		
// 		for(int i=0; i<ww*hh; i++){
// 			if(p_sim_map[i]<=max_sim_local*0.8f)	p_sim_map[i]=0.0f;
// 			else{
// 				sw_x+=(float)(i%ww)*p_sim_map[i];
// 				sw_y+=(float)(i/ww)*p_sim_map[i];
// 				total_sim+=p_sim_map[i];
// 			}
// 		}
// 		sw_x/=total_sim;		sw_y/=total_sim;

		if(in_vector_distance_type == KV_HIST_SIMILARITY_OVERLAP){
			// global maximum similarity.
			if(max_sim_global<max_sim_local){ 
				max_x = max_local_x;		max_y = max_local_y;
				max_ww = mask_size_x;		max_hh = mask_size_y;
				max_sim_global = max_sim_local;
				out_tree_level = tr_lev_local;
			}			
		}
		else	if(in_vector_distance_type == KV_HIST_DISTANCE_L2_NORM){
			// global minimum distance.
			if(min_dist_global>min_dist_local){ 
				max_x = max_local_x;		max_y = max_local_y;
				max_ww = mask_size_x;		max_hh = mask_size_y;
				min_dist_global = min_dist_local;
				out_tree_level = tr_lev_local;
			}
		}

	
		//mask_size_x+=mask_size_step;		mask_size_y = (int)(aspect_ratio*mask_size_x);	
		mask_size_x*=1.05f;		mask_size_y = (int)(aspect_ratio*mask_size_x);	

		//For debugging	
		//if(in_show_similarity_map)		if(max_ww == mask_size_x)	max_sim_map.cp_Copy(&similarity_map);
		if(in_vector_distance_type==KV_HIST_SIMILARITY_OVERLAP){
			//For debugging	
			//fprintf_s(fp, "%5.2f	%f\n", (float)in_img_ROI->mw()/(float)mask_size_x, max_sim_local);	
			max_sim_local = -100000.0f;
		}
		else{
			//For debugging	
			//fprintf_s(fp, "%5.2f	%f\n", (float)in_img_ROI->mw()/(float)mask_size_x, min_dist_local);		
			min_dist_local = 100000.0f;
		}


	}
	//--end: coarse matching.

  	//--start: fine matching.
    max_sim_global = max_sim_local = -100000.0f;
    min_dist_global = min_dist_local = 100000.0f;	
    
    coarse_x = max_x;							coarse_y = max_y;
    coarse_mask_sz_x = max_ww;		coarse_mask_sz_y = max_hh;
    
    mask_size_x=(int)((float)coarse_mask_sz_x*0.8f);
    mask_size_y=(int)((float)coarse_mask_sz_y*0.8f);
/// ///////////////////////////////////////////////////////////////////////////
max_mask_size_x=min(new_ww, (int)((float)coarse_mask_sz_x*1.2f));
max_mask_size_y=min(new_hh, (int)((float)coarse_mask_sz_y*1.2f));
/// ///////////////////////////////////////////////////////////////////////////
    while(mask_size_x<max_mask_size_x && mask_size_y < max_mask_size_y){		//mask_size_x = 105;		mask_size_y = (int)(aspect_ratio*mask_size_x);
    
    	/// Select tree level.
    	/// //////////////////////////////////////////////////////////////////////////////////////
    	for(tr_lev=max_tr_lev; tr_lev>1; tr_lev--){		if(tr_lev==2 || (float)mask_size_x/pow(2.0f, tr_lev) > (float)min_blk_sz || (float)mask_size_y/pow(2.0f, tr_lev) > (float)min_blk_sz)	break; 	}
    	/// //////////////////////////////////////////////////////////////////////////////////////
    	p_weight_vector=in_set_of_weight_vectors->gpe(tr_lev-1)->vps(block_num);
    	hist_vector_dim=num_bins*block_num;
    	p_hist_roi_select = p_hist_roi;
    	for(int i=0; i<tr_lev-1; i++){
    		p_hist_roi_select+=num_bins*in_set_of_sub_block_position_ratios->gpe(i)->mw();
    	}
    
    	int max_local_x, max_local_y;
    	for(y=max(0, coarse_y-(int)(coarse_mask_sz_y*0.1f)); 
    		y<min(hh-mask_size_y, coarse_y+(int)(coarse_mask_sz_y*0.1f)); 
    		y++){	
    		for(x=max(0, coarse_x-(int)(coarse_mask_sz_x*0.1f)); 
    			x<min(ww-mask_size_x, coarse_x+(int)(coarse_mask_sz_x*0.1f)); 
    			x++){				
 
    			/// HOG matching.
    			// Get histogram of window from integral histogram.
 			int hist_mask_level;
 			hist_mask_level=(int)(mask_size_x*(float)in_mask_sz_of_hist_filter/(float)in_ww_roi);
 			if(hist_mask_level<2)				hist_mask_level=0;
 			else if(hist_mask_level<5)		hist_mask_level=1;
 			else												hist_mask_level=2;
 			integral_hist.gnhwsbih_Get_Normalized_Histogram_of_Weighted_Sub_Blocks_from_Integral_Histogram(&((*in_set_of_integral_hist_target)[hist_mask_level]), 
 				x, y,
 				mask_size_x, mask_size_y, 
 				in_set_of_sub_block_position_ratios->gpe(tr_lev-1),
 				in_normalize_mode,
 				in_dominant_gradient_reduction_mode,
 				hist_from_integral_hist);
 
 			// Histogram distance(or similarity)
 			if(in_vector_distance_type == KV_HIST_SIMILARITY_OVERLAP){
 				// using similarity.
 				similarity = zz_jm.cwhvs_Caculate_Weighted_Histogram_Vector_Similarity_using_Overlapped_Intensity(hist_from_integral_hist,
 					p_hist_roi_select,
 					p_weight_vector,
 					block_num,
 					num_bins);			
    
    				// global maximum similarity.
    				if(max_sim_global<similarity){ 
    					max_x = x;		max_y = y;
    					max_ww = mask_size_x;		max_hh = mask_size_y;
    					max_sim_global = similarity;
   					out_tree_level = tr_lev;
    				}				
    			}
    			   
    		}
    	}
    	mask_size_x+=1;		mask_size_y = (int)(aspect_ratio*mask_size_x);	
    }
   	//--end: fine matching.

	out_LT->x=max_x;			out_LT->y=max_y;
	out_RB->x=max_x+max_ww;		out_RB->y=max_y+max_hh;

	//For debugging	
// 	if(in_show_similarity_map){
// 		temp_img.sbox_Set_Box(max_x, max_y, max_ww, max_hh, Kv_Rgb((unsigned char)(rand()%256), (unsigned char)(rand()%256), (unsigned char)(rand()%256)));
// 		scr[0].s_d_Display(&temp_img);
// 		zz_cd.dmfpc_Display_Matrix_Float_using_Pseudo_Color(&scr[1], max_sim_map, true);		aa_uw.sp_Set_Position(&scr[1], &scr[0], 0);
// 		if(!Kv_Printf("Scale x%f (%dx%d mask)\nSim : %f", (float)in_img_ROI->mw()/(float)max_ww, max_ww, max_hh, max_sim_global)) exit(0);
// 	}
//	fclose(fp);

// 	if(!Kv_Printf("%f %f", max_sim_global, max_sim_local))		exit(0);
// 	if(!Kv_Printf("%f %f", min_dist_global, min_dist_local))	exit(0);

	delete[] hist_from_integral_hist;

	return max_sim_global;

}


//********************************************************************************************
void CKvYooji_3D_Object_Scanner::z_uvv_Update_Valid_Volume()
//********************************************************************************************
{
	int ww, hh, dd;
	g_vol_valid.ts(ww, hh, dd);
	cudaMemcpy(g_vol_valid.vp(), zz_vol_valid.vp(), sizeof(bool)*ww*hh*dd, cudaMemcpyHostToDevice);

	g_integ_obj.uvv_Update_Valid_Volume(&g_cube_obj, &g_vol_valid);
}

//********************************************************************************************
bool CKvYooji_3D_Object_Scanner::z_ro_Remove_Occlusion()
//********************************************************************************************
{
	int ww, hh;
	float *p_depth;
	int *p_buffer;
	
	// initialize maps.
	zz_rgbd_frame.p_map_depth_filtered_on_RGB()->ms(ww, hh);

	if(zz_depth_vhs.mw() != ww || zz_depth_vhs.mh() != hh) zz_depth_vhs.c_Create(hh, ww, 255.0f);
	if(zz_buffer_vhs.mw() != ww || zz_buffer_vhs.mh() != hh) zz_buffer_vhs.c_Create(hh, ww, -1);

	p_depth = zz_depth_vhs.vp();	for(int i=0; i<ww*hh; i++) p_depth[i] = 255.0f;
	p_buffer = zz_buffer_vhs.vp();	for(int i=0; i<ww*hh; i++) p_buffer[i] = -1;

	zz_3do.gdmzbi_Generate_Depth_Map_using_Z_buffering_for_an_Image(
		&zz_depot_p3d_vhs,
		&zz_idx_pnts_vhs,
		zz_state.p_P_matrix_RGB(),
		&zz_depth_vhs,
		&zz_buffer_vhs);

 	zz_integ.rodvhs_Remove_Outlier_Depth_using_Visual_Hull_Shield(
		&zz_depth_vhs,
 		zz_rgbd_frame.p_map_depth_filtered_on_RGB());

//   	CKvScreen sc;
   	//zz_sc_ios.s_d_Display(50.0f, 0.0f, &zz_depth_vhs);	//if(!Kv_Printf("VHS")) exit(0);
// 		

	return true;
}


//********************************************************************************************
void CKvYooji_3D_Object_Scanner::z_gmc_Get_Mesh_of_Cube()
//********************************************************************************************
{
	//////////////////////////////////////////////////////////////////////////
	// create mesh for cube projection.
	CKvPoint3D tp3d; CKvPoint3Df origin_k; int tidx;
	zz_cube.goiw_Get_Origin_In_World(origin_k);
	//  3 --- 2
	// 0 --- 1
	//  7 --- 6
	// 4 --- 5
	zz_depot_p3d_cube.in_Initialize();
	// + set upper vertices. (4 vertices)
	tp3d.y = origin_k.y;
	tp3d.x = origin_k.x;			  tp3d.z = origin_k.z;				zz_depot_p3d_cube.ap_Append(true,tp3d,tidx);
	tp3d.x = origin_k.x + zz_sz_cube; tp3d.z = origin_k.z;				zz_depot_p3d_cube.ap_Append(false,tp3d,tidx);
	tp3d.x = origin_k.x + zz_sz_cube; tp3d.z = origin_k.z + zz_sz_cube; zz_depot_p3d_cube.ap_Append(false,tp3d,tidx);
	tp3d.x = origin_k.x;			  tp3d.z = origin_k.z + zz_sz_cube; zz_depot_p3d_cube.ap_Append(false,tp3d,tidx);
	// + set lower vertices. (4 vertices)
	tp3d.y = origin_k.y + zz_sz_cube;
	tp3d.x = origin_k.x;			  tp3d.z = origin_k.z;				zz_depot_p3d_cube.ap_Append(false,tp3d,tidx);
	tp3d.x = origin_k.x + zz_sz_cube; tp3d.z = origin_k.z;				zz_depot_p3d_cube.ap_Append(false,tp3d,tidx);
	tp3d.x = origin_k.x + zz_sz_cube; tp3d.z = origin_k.z + zz_sz_cube; zz_depot_p3d_cube.ap_Append(false,tp3d,tidx);
	tp3d.x = origin_k.x;			  tp3d.z = origin_k.z + zz_sz_cube; zz_depot_p3d_cube.ap_Append(false,tp3d,tidx);
	// set meshes. (12 meshes)
	int *p_mesh = zz_idx_pnts_cube.c_Create(12*3,0);
	tidx = 0;
	p_mesh[tidx++] = 0; p_mesh[tidx++] = 4; p_mesh[tidx++] = 1;
	p_mesh[tidx++] = 1; p_mesh[tidx++] = 4; p_mesh[tidx++] = 5;

	p_mesh[tidx++] = 1; p_mesh[tidx++] = 5; p_mesh[tidx++] = 2;
	p_mesh[tidx++] = 2; p_mesh[tidx++] = 5; p_mesh[tidx++] = 6;

	p_mesh[tidx++] = 2; p_mesh[tidx++] = 6; p_mesh[tidx++] = 3;
	p_mesh[tidx++] = 3; p_mesh[tidx++] = 6; p_mesh[tidx++] = 7;

	p_mesh[tidx++] = 3; p_mesh[tidx++] = 7; p_mesh[tidx++] = 0;
	p_mesh[tidx++] = 0; p_mesh[tidx++] = 7; p_mesh[tidx++] = 4;

	p_mesh[tidx++] = 3; p_mesh[tidx++] = 0; p_mesh[tidx++] = 2;
	p_mesh[tidx++] = 2; p_mesh[tidx++] = 0; p_mesh[tidx++] = 1;

	p_mesh[tidx++] = 4; p_mesh[tidx++] = 7; p_mesh[tidx++] = 5;
	p_mesh[tidx++] = 5; p_mesh[tidx++] = 7; p_mesh[tidx++] = 6;
	//////////////////////////////////////////////////////////////////////////
}

//********************************************************************************************
int CKvYooji_3D_Object_Scanner::z_god_Get_Object_Depth(CKvYooji_MatrixRgbD *io_frame_of_rgbd_camera,
	int in_capture_mode)
//********************************************************************************************
{
	float *p_img_depth_float;
	float d_min, d_max, def_d;
	int ww, hh, tidx, num_valid;

	p_img_depth_float=io_frame_of_rgbd_camera->p_map_depth_raw()->mps(ww, hh)[0];

	num_valid = 0;

	// for virtual sequence.
	if(in_capture_mode == KV_CAPTURE_FROM_FILES_CXD){
	
		def_d = 0.0f;
		for(int i=0; i<ww*hh; i++){
			// for depth image captured from KAISION.
			if(p_img_depth_float[i]<=0.0f){
				p_img_depth_float[i] = def_d;				
				continue;
			}
			num_valid++;			
		}
	}
	// for real sequence.
	else{
		zz_params.gptd_Get_Parameters_for_Thresholding_Depth(d_min, d_max);

		for(int j=0; j<hh; j++){	for(int i=0; i<ww; i++){
			tidx=j*ww+i;		
			if(p_img_depth_float[tidx]<d_min || p_img_depth_float[tidx]>d_max){	
				p_img_depth_float[tidx]=0.0f;	
				continue;	
			}
			num_valid++;
		}}
	}	

	return num_valid;
}


//********************************************************************************************
// 이 부분을 RGB camera 상에 object cube 를 투영시켜서 ROI를 활성화 시키고 거기서 
// Depth camera 상의 depth 값으로 interpolation 해서 채워넣는 식으로 변경해야 할 듯!!!
void CKvYooji_3D_Object_Scanner::z_tdrcc_Transform_Depth_to_Rgb_Camera_Coordinates(	
	CKvYooji_MatrixRgbD *in_rgbd_img,
	CKvMatrixFloat *in_img_d,
	CKvMatrixFloat *out_img_d_transformed)
//********************************************************************************************
{	
	CKvPointf p2d_d, p2d_rgb;
	CKvPoint3Df p3d_d;

	float *p_img_d_raw, *p_img_d_trans_raw;

	float td;
	int ww_d, hh_d, ww_rgb, hh_rgb;
	int i, j, tx, ty;
	
	p_img_d_raw = in_img_d->mps(ww_d, hh_d)[0];
	p_img_d_trans_raw = out_img_d_transformed->mps(ww_rgb, hh_rgb)[0];

	// Initialization.
	for(i=0; i<ww_rgb*hh_rgb; i++)	p_img_d_trans_raw[i] = 0.0f;

	for(j=0; j<hh_d; j++){			
		for(i=0; i<ww_d; i++){

			td = p_img_d_raw[j*ww_d + i];
			if(td>0.0f){

				p2d_d.x = (float)i;	p2d_d.y = (float)j;

				(*in_rgbd_img).convert_pixel_coord(
					p2d_d,
					td,
					p2d_rgb);

				//printf("p2d_d: %f %f p2d_rgb: %f %f\n", p2d_d.x, p2d_d.y, p2d_rgb.x, p2d_rgb.y);

				tx = ROUNDF(p2d_rgb.x);		ty = ROUNDF(p2d_rgb.y);
				if(tx<0 || tx>ww_rgb-1 || ty<0 || ty>hh_rgb-1)	continue;

				// compare previous and current depth.
				if(p_img_d_trans_raw[ty*ww_rgb + tx] > td || p_img_d_trans_raw[ty*ww_rgb + tx] <= 0.0f)
					p_img_d_trans_raw[ty*ww_rgb + tx] = td;

			}

		}
	}
}

//********************************************************************************************
bool CKvYooji_3D_Object_Scanner::out_tif_Tracked_Information_of_a_Frame(int in_idx_frame,
		CKvYooji_Tracking_State *out_state)
//********************************************************************************************
{
	if(in_idx_frame >= zz_number_of_processed_frames)	return false;

	(*out_state).copy(&zz_state);
	// ===============================================================================================
	//(*out_state).pt3d_Pointer_of_3D_Transform_Global_to_Camera()->cp_Copy(&zz_trace[in_idx_frame]);
	// ===============================================================================================
 	(*out_state).compute_P_matrix(
 		(*out_state).p_intrinsics_depth(),
 		(*out_state).p_extrinsics_glob_to_cam(),
 		(*out_state).p_P_matrix_depth());

	zz_disp.rmi_Render_Maps_for_ICP(
		&zz_cube,		
		&zz_params,
		out_state);

	return true;
}

//********************************************************************************************
void CKvYooji_3D_Object_Scanner::out_ccti_Current_Camera_Trace_Image(
	CKvMatrixUcharRgb &out_img_trace,
	bool in_flag_for_scanning_result,
	int in_sampling_rate)
//********************************************************************************************
{
	CKvPoint3Df origin;
	float sz_cube;
	int ww_c, hh_c, dd_c;	

	zz_intrin.copy(zz_state.p_intrinsics_depth());
	zz_extrin.copy(zz_state.p_extrinsics_glob_to_cam());

	zz_cube.ts(ww_c, hh_c, dd_c);
	sz_cube = zz_cube.svm_Size_of_Voxel_in_Meter()*(float)ww_c;
	zz_cube.goiw_Get_Origin_In_World(origin);

	// Display scanned result at current state.
	if(in_flag_for_scanning_result){
// 		zz_disp.rmi_Render_Maps_for_ICP(
// 			&zz_cube,
// 			&zz_params,
// 			&zz_state_trace);
	
		Vector2i sz_map;
		sz_map = g_state_trace.sz_map;
		//////////////////////////////////////////////////////////////////////////
// 		g_render_trace.rmi_Render_Maps_for_ICP(
// 			&g_cube_obj,
// 			sz_map.x, sz_map.y,
// 			g_state_trace.vp_T_gc(),g_state_trace.vp_T_cg(),
// 			g_state_trace.center,zz_light_trace,
// 
// 			g_state_trace.vp_map_depth(),
// 			g_state_trace.vp_map_normal(),
// 			g_state_trace.vp_img_norm());
// 
// 		cudaMemcpy(out_img_trace.vp(), g_state_trace.vp_img_norm(), 640*480*sizeof(uchar), cudaMemcpyDeviceToHost);
		//////////////////////////////////////////////////////////////////////////

// 		zz_disp.dvoi_Draw_Volume_Of_Interest(
// 			zz_state_trace.p_map_depth_rendered(),
// 			zz_state_trace.p_image_normals_rendered(),
// 			&zz_int_ref,
// 			&zz_ext_ref,
// 			sz_cube,
// 			origin,
// 			&out_img_trace);
	}

	
	// Display groundtruth camera trace at current state.
	// ===============================================================================================
	//printf("size: %d\n", zz_number_of_processed_frames);
	
	zz_disp.rsct_Render_Set_of_Camera_Traces_New(&zz_ext_ref,&zz_trace_gt,&zz_int_ref,&zz_intrin,&out_img_trace,
		zz_number_of_processed_frames-1,
		0.1*(sz_cube/0.5),-1,
		&Kv_Rgb(0,0,255),
		&Kv_Rgb(255,0,0));
	// ===============================================================================================

	// Display camera trace at current state.
	// ===============================================================================================
	//printf("size: %d\n", zz_trace->size());
 	zz_disp.rsct_Render_Set_of_Camera_Traces_New(&zz_ext_ref, &zz_trace, &zz_int_ref, &zz_intrin, &out_img_trace,
 		zz_number_of_processed_frames-1,
		0.1*(sz_cube/0.5), in_sampling_rate,
		&Kv_Rgb(255, 0, 0),
		&Kv_Rgb(0,0,255));
	// ===============================================================================================
}

//********************************************************************************************
void CKvYooji_3D_Object_Scanner::z_srctd_Set_Reference_Camera_for_Trace_Display(
	CKvYooji_Extrinsics &in_pose_init,
	CKvYooji_Intrinsics &out_intrins_ref,
	CKvYooji_Extrinsics &out_extrins_ref)
//********************************************************************************************
{
	// set reference view for tracing camera.
	CKvPoint3Df center, org_cube;

	float dx, dy, dz;

	zz_v_prin_axis.c_Create(3, 0.0f); 
	zz_v_center.c_Create(3, 0.0f);		

	// using cube center
	float scale = 1.0f;
	float cube_sz = zz_cube.svm_Size_of_Voxel_in_Meter()*KV_CUBE_DIM;

	zz_cube.gcciw_Get_Cube_Center_In_World(center);

	dx = 0.0f;
	dy = -1.0f*scale*cube_sz;
	dz = -1.0f;

	zz_v_center.se_Set_Element(0,center.x + dx);
	zz_v_center.se_Set_Element(1,center.y + dy);
	zz_v_center.se_Set_Element(2,center.z + dz);
	
	zz_v_prin_axis.se_Set_Element(0,0.0f);
	zz_v_prin_axis.se_Set_Element(1,1.0f);
	zz_v_prin_axis.se_Set_Element(2,0.0f);

// 	// using origin of object coordinates.
// 	float scale = 2.0f*zz_cube.svm_Size_of_Voxel_in_Meter()*KV_CUBE_DIM;
// 
// 	dx = 0.0f;//-0.1*scale;
// 	dy = -0.3*scale;
// 	dz = -0.5*scale;
// 
// 	in_pose_init.get_cam_center(center);
// 
// 	zz_v_center.se_Set_Element(0, -center.x + dx);	
// 	zz_v_center.se_Set_Element(1, -center.y + dy);	
// 	zz_v_center.se_Set_Element(2, -center.z + dz);
// 
// 	zz_v_prin_axis.se_Set_Element(0, 0.0f);
// 	zz_v_prin_axis.se_Set_Element(1, 0.2f);
// 	zz_v_prin_axis.se_Set_Element(2, 1.0f);

	out_intrins_ref.set_from_params(300.0f, 300.0f, 320.0f, 240.0f);
	out_extrins_ref.set_from_params(&zz_v_prin_axis, NULL, &zz_v_center);		
	
}

//********************************************************************************************
void CKvYooji_3D_Object_Scanner::z_uct_Update_Camera_Trace(CKvYooji_Tracking_State &in_tracking_state,
	vector<CKvYooji_Extrinsics*> *io_set_of_traces)
//********************************************************************************************
{
	CKvYooji_Extrinsics *t_ext = new CKvYooji_Extrinsics;   // this should be released before the end of this program.

	//zz_extrin.copy(in_tracking_state.p_extrinsics_glob_to_cam_RGB());		
	zz_extrin.copy(in_tracking_state.p_extrinsics_glob_to_cam());


	t_ext->copy(&zz_extrin);	io_set_of_traces->push_back(t_ext);
}



//********************************************************************************************
void CKvYooji_3D_Object_Scanner::sis_Save_Input_Sequence(CKvString &in_dn_depth, CKvString &in_dn_texture)
//********************************************************************************************
{
	LCKvIO_FileVcl iof;
	CKvString save_fn,dn_texture,dn_depth;

	for(int i=0; i<zz_number_of_processed_frames; i++){

		// save depth image.
		save_fn.fm_Format("%s\\%04d.vcl",in_dn_depth.bp(),i);
		iof.svf_Save_as_Vcl_File(save_fn,zz_set_of_input_depth.gpe(i));

		// save color image.
		save_fn.fm_Format("%s\\%04d.bmp",in_dn_texture.bp(),i);
		iof.si_Save_Image(save_fn,true,zz_set_of_input_rgb.gpe(i));
		
	}
	
}

//********************************************************************************************
void CKvYooji_3D_Object_Scanner::sss_Save_Silhouette_Sequence(CKvString &in_dn_sil)
//********************************************************************************************
{
	LCKvIO_FileVcl iof;
	CKvString save_fn;
	CKvMatrixUchar timg;

	for(int i=0; i<zz_number_of_processed_frames; i++){
		// save color image.
		zz_im.mbuc_Matrix_Bool_to_Uchar(zz_set_of_input_sil.gpe(i),255,0,&timg);
		save_fn.fm_Format("%s\\%04d.bmp",in_dn_sil.bp(),i);
		iof.si_Save_Image(save_fn,false,&timg);
		
	}
	
}

//********************************************************************************************
void CKvYooji_3D_Object_Scanner::itsdfcd_Import_TSDF_Cube_from_Device()
//********************************************************************************************
{
	Vector3i dim;
	Vector3f origin,center;

	float *tsdf=NULL;
	uchar *weight=NULL, *color=NULL;
	
	float sz_vox;
	int ww, hh, dd;
	int len;

	g_cube_obj.ts(ww, hh, dd);

	len = ww*hh*dd;
	tsdf = new float[len];
	weight = new uchar[len];
	color = new uchar[3*len];

	// get cube information from device to host.
	g_cube_obj.to_host(
		tsdf,
		weight,
		color,
		dim,
		origin,
		center,
		sz_vox);

	// copy cube inform. to CKvYooji_TSDF_Cube_Float
	CKvPoint3Df origin_k;	
	origin_k.x = origin.x; origin_k.y = origin.y; origin_k.z = origin.z; 
	zz_cube.c_Create(origin_k, dim.z, dim.y, dim.x, 8, sz_vox);

	memcpy(zz_cube.pvd_Pointer_of_Volume_Depth()->vp(),tsdf,sizeof(float)*len);
	memcpy(zz_cube.pvwd_Pointer_of_Volume_Weight_Depth()->vp(),weight,sizeof(uchar)*len);
	memcpy(zz_cube.pvr_Pointer_of_Volume_Rgb()->vp(),color,sizeof(uchar)*3*len);

	if(tsdf) delete[] tsdf;
	if(weight) delete[] weight;
	if(color) delete[] color;
}

//********************************************************************************************
void CKvYooji_3D_Object_Scanner::stsdfc_Save_TSDF_Cube(CKvString &in_foder_path)
//********************************************************************************************
{
	LCKvIO_FileVcl iofv;
	CKvString fn_depth, fn_color, fn_weight;

	fn_depth.fm_Format("%s\\tsdf_depth.vcl", in_foder_path.bp());
	fn_color.fm_Format("%s\\tsdf_color.vcl", in_foder_path.bp());
	fn_weight.fm_Format("%s\\tsdf_weight.vcl", in_foder_path.bp());

	iofv.svf_Save_as_Vcl_File(fn_depth, zz_cube.pvd_Pointer_of_Volume_Depth());
	iofv.svf_Save_as_Vcl_File(fn_color, zz_cube.pvr_Pointer_of_Volume_Rgb());
	iofv.svf_Save_as_Vcl_File(fn_weight, zz_cube.pvwd_Pointer_of_Volume_Weight_Depth());
}

//********************************************************************************************
void CKvYooji_3D_Object_Scanner::sdc_Save_Do_Cube(CKvString &in_foder_path)
//********************************************************************************************
{
	LCKvIO_FileVcl iofv;
	CKvString fn_depth, fn_color, fn_weight;

	fn_depth.fm_Format("%s\\do_cube.vcl", in_foder_path.bp());

	//iofv.svf_Save_as_Vcl_File(fn_depth, zz_cube.pvhs_Pointer_of_Visual_Hull_Segments());
}
