/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_frame_capture.cpp 
/////////////////////////////////////////////////////////////////////////////////////////////

#include "../../stdafx.h"
#include "../_yooji_2016_object_scanning.h"

/////////////////////////////////////////////////////////////////////////////////////////////
// CKvYooji_FrameCapture 
/////////////////////////////////////////////////////////////////////////////////////////////

//********************************************************************************************
CKvYooji_FrameCapture::CKvYooji_FrameCapture(void)
//********************************************************************************************
{
	zz_classname="CKvYooji_FrameCapture";
	//c_Create(Kv_Point3Df(0.0f,0.0f,0.0f),8,8,8,8);

	zz_flag_pmat_gt = zz_flag_sil = zz_flag_rgb_img = zz_flag_depth_map_float = zz_flag_depth_map_short = zz_flag_cxd = false;
}

//********************************************************************************************
CKvYooji_FrameCapture::~CKvYooji_FrameCapture(void)
//********************************************************************************************
{
}

////***********************************************************************************************************************
//CKvYooji_FrameCapture::CKvYooji_FrameCapture(CKvYooji_FrameCapture &a)
////***********************************************************************************************************************
//{
//	cp_Copy(&a);
//}

//***********************************************************************************************************************
void CKvYooji_FrameCapture::i_Initialize(int in_capture_type)
//***********************************************************************************************************************
{
	LCKvIO io;
	LCKvIO_FileVcl iof;
	CKvString load_dn;
	CKvString *p_tmp = NULL;

	int nd,nf,tmp;

	zz_type_capture = in_capture_type;

	zz_flag_pmat_gt = zz_flag_sil = zz_flag_rgb_img = zz_flag_depth_map_float = zz_flag_depth_map_short = false;

	io.gfn_Get_Folder_Name("Open folder for loading object data sequence.",load_dn);

	// ================================================================================
	// load file name of background color model.
	zz_fn_gmm_glove.fm_Format("%s\\gmm_color_glove.txt",load_dn.bp());
	zz_fn_gmm_back_pixels.fm_Format("%s\\background\\gmm_back_pixels.vcl",load_dn.bp());
	zz_fn_gmm_back.fm_Format("%s\\background\\gmm_color_model.txt",load_dn.bp());
	// ================================================================================
	// ================================================================================

	// load file name of zz_fn_object_cube.
	zz_fn_cam_params.fm_Format("%s\\camera_params.txt",load_dn.bp());
	// load file name of zz_fn_object_cube.
	zz_fn_object_cube.fm_Format("%s\\object_cube.txt",load_dn.bp());


	if(in_capture_type==KV_CAPTURE_FROM_FILES){

		// load file names of depth images.
		zz_dn_depth.fm_Format("%s\\depth16u",load_dn.bp());
		if(!iof.gdi_Get_Directory_Information(zz_dn_depth,false,nd,nf) || nf<=0){
			zz_dn_depth.fm_Format("%s\\depth16f",load_dn.bp());
			if(iof.gdi_Get_Directory_Information(zz_dn_depth,false,nd,nf)){
				if(nf > 0){
					zz_type_depth = KV_TYPE_DEPTH_FLOAT;
					zz_flag_depth_map_float = true;
				}
			}
		} else{
			if(nf > 0){
				zz_type_depth = KV_TYPE_DEPTH_USHORT;
				zz_flag_depth_map_short = true;
			}
		}
		if(zz_flag_depth_map_float || zz_flag_depth_map_short){
			zz_set_of_fn_depth.c_Create(nf);
			iof.gdi_Get_Directory_Information(zz_dn_depth,false,tmp,tmp,false,true,nd,nf,p_tmp,zz_set_of_fn_depth.vp());
		}
		// load file name of P matrices.
		zz_dn_pmat.fm_Format("%s\\pmat",load_dn.bp());
		if(iof.gdi_Get_Directory_Information(zz_dn_pmat,false,nd,nf)){
			if(nf > 0){
				zz_set_of_fn_pmat.c_Create(nf);
				iof.gdi_Get_Directory_Information(zz_dn_pmat,false,tmp,tmp,false,true,nd,nf,p_tmp,zz_set_of_fn_pmat.vp());

				zz_flag_pmat_gt = true;
			}
		}
		// load file names of silhouettes.
		zz_dn_sil.fm_Format("%s\\binary",load_dn.bp());
		if(iof.gdi_Get_Directory_Information(zz_dn_sil,false,nd,nf)){
			if(nf > 0){
				zz_set_of_fn_sil.c_Create(nf);
				iof.gdi_Get_Directory_Information(zz_dn_sil,false,tmp,tmp,false,true,nd,nf,p_tmp,zz_set_of_fn_sil.vp());
				zz_flag_sil = true;
			}
		}
		// load file names of texture images.
		zz_dn_texture.fm_Format("%s\\texture",load_dn.bp());
		if(iof.gdi_Get_Directory_Information(zz_dn_texture,false,nd,nf)){
			if(nf > 0){
				zz_set_of_fn_color.c_Create(nf);
				iof.gdi_Get_Directory_Information(zz_dn_texture,false,tmp,tmp,false,true,nd,nf,p_tmp,zz_set_of_fn_color.vp());
				zz_flag_rgb_img = true;
			}
		}

		zz_num_file = max(zz_set_of_fn_depth.vs(),zz_set_of_fn_color.vs());
		zz_type_capture = in_capture_type;
	} 
	else if(in_capture_type==KV_CAPTURE_FROM_FILES_CXD){


		// load file names of cxd files.
		zz_dn_depth.fm_Format("%s\\cxd",load_dn.bp());
		if(iof.gdi_Get_Directory_Information(zz_dn_depth,false,nd,nf)){
			if(nf>0){
				zz_flag_cxd = true;
				zz_set_of_fn_cxd.c_Create(nf);
				iof.gdi_Get_Directory_Information(zz_dn_depth,false,tmp,tmp,false,true,nd,nf,p_tmp,zz_set_of_fn_cxd.vp());
			}
		}
		

		//////////////////////////////////////////////////////////////////////////
		//// for debugging.
		////// load file names of cxd files.
		//zz_dn_depth.fm_Format("%s\\depth16f",load_dn.bp());
		//if(iof.gdi_Get_Directory_Information(zz_dn_depth,false,nd,nf)){
		//	if(nf>0){
		//		zz_type_depth = KV_TYPE_DEPTH_FLOAT;
		//		zz_flag_depth_map_float = true;

		//		zz_set_of_fn_depth.c_Create(nf);
		//		iof.gdi_Get_Directory_Information(zz_dn_depth,false,tmp,tmp,false,true,nd,nf,p_tmp,zz_set_of_fn_depth.vp());
		//	}
		//}
		//// load file names of texture images.
		//zz_dn_texture.fm_Format("%s\\texture",load_dn.bp());
		//if(iof.gdi_Get_Directory_Information(zz_dn_texture,false,nd,nf)){
		//	if(nf > 0){
		//		zz_set_of_fn_color.c_Create(nf);
		//		iof.gdi_Get_Directory_Information(zz_dn_texture,false,tmp,tmp,false,true,nd,nf,p_tmp,zz_set_of_fn_color.vp());
		//		zz_flag_rgb_img = true;
		//	}
		//}
		////////////////////////////////////////////////////////////////////////////

		// load file name of P matrices.
		zz_dn_pmat.fm_Format("%s\\pmat",load_dn.bp());
		if(iof.gdi_Get_Directory_Information(zz_dn_pmat,false,nd,nf)){
			if(nf > 0){
				zz_set_of_fn_pmat.c_Create(nf);
				iof.gdi_Get_Directory_Information(zz_dn_pmat,false,tmp,tmp,false,true,nd,nf,p_tmp,zz_set_of_fn_pmat.vp());

				zz_flag_pmat_gt = true;
			}
		}
		// load file names of silhouettes.
		zz_dn_sil.fm_Format("%s\\binary",load_dn.bp());
		if(iof.gdi_Get_Directory_Information(zz_dn_sil,false,nd,nf)){
			if(nf > 0){
				zz_set_of_fn_sil.c_Create(nf);
				iof.gdi_Get_Directory_Information(zz_dn_sil,false,tmp,tmp,false,true,nd,nf,p_tmp,zz_set_of_fn_sil.vp());
				zz_flag_sil = true;
			}
		}

		zz_num_file = zz_set_of_fn_cxd.vs();
		zz_type_capture = in_capture_type;
	}
	else{
		// load file names of depth images.
		zz_dn_depth.fm_Format("%s\\depth16f",load_dn.bp());
		// load file name of P matrices.
		zz_dn_pmat.fm_Format("%s\\pmat",load_dn.bp());
		// load file names of silhouettes.
		zz_dn_sil.fm_Format("%s\\binary",load_dn.bp());
		// load file names of texture images.
		zz_dn_texture.fm_Format("%s\\texture",load_dn.bp());

		zz_num_file = 0;
		zz_type_capture = in_capture_type;
	}

	//if(in_capture_type==KV_CAPTURE_FROM_KAISION_FILES){
	//	LCKvIO io;
	//	LCKvIO_FileVcl iof;
	//	CKvString load_dn, load_dn_pmat, load_dn_hmat, load_dn_texture, load_dn_depth;
	//	CKvString *p_tmp = NULL;

	//	int nd, nf, tmp;

	//	io.gfn_Get_Folder_Name("Open folder for loading object data sequence.", load_dn);

	//	// load file name of zz_fn_object_cube.
	//	zz_fn_object_cube.fm_Format("%s\\object_cube.txt", load_dn.bp());
	//	// load file names of depth images.
	//	load_dn_depth.fm_Format("%s\\depth16f", load_dn.bp());	zz_type_depth = KV_TYPE_DEPTH_FLOAT;
	//	if(!iof.gdi_Get_Directory_Information(load_dn_depth, false, nd, nf)){

	//		load_dn_depth.fm_Format("%s\\depth16u", load_dn.bp());	zz_type_depth = KV_TYPE_DEPTH_USHORT;
	//		if(!iof.gdi_Get_Directory_Information(load_dn_depth, false, nd, nf))	gerr("2");
	//		if(nf <= 0)	gerr("3");
	//	}

	//	zz_set_of_fn_depth.c_Create(nf);
	//	iof.gdi_Get_Directory_Information(load_dn_depth, false, tmp, tmp, false, true, nd, nf, p_tmp, zz_set_of_fn_depth.vp());
	//	// load file name of P matrices.
	//	zz_flag_pmat_gt = true;
	//	load_dn_pmat.fm_Format("%s\\pmat", load_dn.bp());
	//	if(iof.gdi_Get_Directory_Information(load_dn_pmat, false, nd, nf)){
	//		if(nf == zz_set_of_fn_depth.vs()){
	//			zz_flag_pmat_gt = true;
	//			zz_set_of_fn_pmat.c_Create(nf);
	//			iof.gdi_Get_Directory_Information(load_dn_pmat, false, tmp, tmp, false, true, nd, nf, p_tmp, zz_set_of_fn_pmat.vp());
	//		}
	//	}
	//	// load file names of texture images.
	//	load_dn_texture.fm_Format("%s\\texture", load_dn.bp());
	//	iof.gdi_Get_Directory_Information(load_dn_texture, false, nd, nf);
	//	if(nf==zz_set_of_fn_depth.vs()){
	//		zz_set_of_fn_color.c_Create(nf);
	//		iof.gdi_Get_Directory_Information(load_dn_texture, false, tmp, tmp, false, true, nd, nf, p_tmp, zz_set_of_fn_color.vp());
	//	}

	//	zz_num_file = zz_set_of_fn_depth.vs();
	//	zz_type_capture = in_capture_type;
	//}
	
	return;
 error:
 	zpme("i_Initialize");
}

//***********************************************************************************************************************
void CKvYooji_FrameCapture::cp_Copy(CKvYooji_FrameCapture *a)
//***********************************************************************************************************************
{
	zz_fn_gmm_back.cp_Copy(a->zz_fn_gmm_back);			
	zz_fn_object_cube.cp_Copy(a->zz_fn_object_cube);	zz_fn_cam_params.cp_Copy(a->zz_fn_cam_params);

	zz_set_of_fn_pmat.cp_Copy(&a->zz_set_of_fn_pmat);	zz_set_of_fn_sil.cp_Copy(&a->zz_set_of_fn_sil);
	zz_set_of_fn_color.cp_Copy(&a->zz_set_of_fn_color);	zz_set_of_fn_depth.cp_Copy(&a->zz_set_of_fn_depth);

	zz_num_file = a->zz_num_file;
	zz_type_capture = a->zz_type_capture;
	zz_type_depth = a->zz_type_depth;

	zz_flag_pmat_gt = a->zz_flag_pmat_gt;
	zz_flag_sil = a->zz_flag_sil;
	zz_flag_rgb_img = a->zz_flag_rgb_img;
	zz_flag_depth_map_float = a->zz_flag_depth_map_float;
	zz_flag_depth_map_short = a->zz_flag_depth_map_short;
}

//***********************************************************************************************************************
bool CKvYooji_FrameCapture::lpm_Load_P_Matrix(
	int in_file_idx,
	CKvPmatrix3D *out_pmat)
//***********************************************************************************************************************
{
	if(in_file_idx<0 || in_file_idx>=zz_num_file)	return false;
	if(!zz_flag_pmat_gt)	return false;
		
	CKvMatrixFloat tmat;
	if(!zz_iof.lvf_Load_from_Vcl_File(zz_set_of_fn_pmat.vp()[in_file_idx], &tmat)){		// from vcl file.		
		if(!lpmt_Load_P_Matrix_from_Txt(zz_set_of_fn_pmat.vp()[in_file_idx], out_pmat)){		// from txt file.		
				return false;
		}		
	}
	else out_pmat->i_Import(&tmat);

	return true;
}


//***********************************************************************************************************************
bool CKvYooji_FrameCapture::lpmt_Load_P_Matrix_from_Txt(
	CKvString &in_filename_of_camera_information,
	CKvPmatrix3D *out_pmat)
//***********************************************************************************************************************
{
	const char *bp = (const char *)in_filename_of_camera_information.bp();
	std::ifstream f(bp);

	if(f.fail())	return false;

	CKvMatrix P;
	double *p_P;
	int cnt, ww, hh;

	// read camera matrices.
	cnt = 0;	p_P = P.c_Create(3, 4, 0.0f)[0];

	while(!f.eof()){

		f >> p_P[cnt%12];	cnt++;

		if(cnt==12)			out_pmat->i_Import(&P);	// the first 12 elements are P matrix of depth camera.
	}
	f.close();

	if(cnt<12)	return false;


	return true;
}

//***********************************************************************************************************************
bool CKvYooji_FrameCapture::ltt_Load_Trace_from_Txt_for_SDF2SDF(
	CKvString &in_filename_of_camera_information,
	vector<CKvYooji_Extrinsics*> *out_trace)
//***********************************************************************************************************************
{
 	const char *bp = (const char *)in_filename_of_camera_information.bp();
 	std::ifstream f(bp);
 
 	if(f.fail())	return false;
 
 	CKvMatrixFloat t_mat;	t_mat.ci_Create_Identity_matrix(4);
 	int cnt, ww, hh;

	if(out_trace->size()>0){
		for(int i=0; i<out_trace->size(); i++){
			delete (*out_trace)[i];
		}
		out_trace->clear();
	}
 	
 	while(!f.eof()){
 
 		f >> cnt;	// get frame number.
 
 		for(int i=0; i<16; i++) f >> t_mat.vp()[i];
 
		// 이 부분 출력하게 하면 initialization 과정에서 이상하게 죽네 ㅂㄷㅂㄷ...
// 		d_pm_Printf_Matrix(t_mat.vp(), 4, 4, "t_mat");
 
		CKvYooji_Extrinsics *t_ext = new CKvYooji_Extrinsics;
		
		//////////////////////////////////////////////////////////////////////////
		// SDF-2-SDF
		//t_ext->set_from_transform_inv(&t_mat);
		//////////////////////////////////////////////////////////////////////////
		// Aruco pose
		t_ext->set_from_transform(&t_mat);
		//////////////////////////////////////////////////////////////////////////

		out_trace->push_back(t_ext);
 
 		cout << "#" << cnt << " frame was loaded..." << endl;
 	}
	f.close();

	cout << "Trace loading was complete." << endl;

	return true;
}

//***********************************************************************************************************************
bool CKvYooji_FrameCapture::stt_Save_Trace_to_Txt_for_SDF2SDF(
	CKvString &in_filename_of_camera_information,
	vector<CKvYooji_Extrinsics*> *in_trace)
//***********************************************************************************************************************
{
	const char *bp = (const char *)in_filename_of_camera_information.bp();
	std::ofstream f(bp);

	if(f.fail())	return false;

	CKvMatrixFloat t_mat;	t_mat.ci_Create_Identity_matrix(4);
	int cnt, ww, hh;
	
	cnt = 0;
	while(!f.eof()){

		f << cnt << endl;	// get frame number.

		t_mat.cp_Copy((*in_trace)[cnt]->mp_transform());

		for(int i=0; i<16; i++){
						
			if(i%4==0) f << endl;
			f << t_mat.vp()[i] << " ";
		}		
		f << endl;

		d_pm_Printf_Matrix(t_mat.vp(), 4, 4, "t_mat");
				
		cout << "#" << cnt << " frame was saved..." << endl;

		cnt++;
	}
	f.close();

	cout << "Trace saving was complete." << endl;

	return true;
}

//***********************************************************************************************************************
bool CKvYooji_FrameCapture::ltt_Load_Trace_from_Txt_for_Steinsbrucker(
	CKvString &in_filename_of_camera_information,
	vector<CKvYooji_Extrinsics*> *out_trace)
//***********************************************************************************************************************
{
	const char *bp = (const char *)in_filename_of_camera_information.bp();
	std::ifstream f(bp);
	
	if(f.fail())	return false;

	CKvYooji_Extrinsics pose_init, pose_prev;
	CKvYooji_Extrinsics pose_curr;

	CKvMatrixFloat mat_T;	float *p_mat = mat_T.ci_Create_Identity_matrix(4)[0];
	CKvMatrixFloat mat_R;	float *p_R = mat_R.ci_Create_Identity_matrix(3)[0];
	CKvVectorFloat vec_cen;	float *p_cen = vec_cen.c_Create(3);
	CKvVectorFloat vec_q;	float *p_q = vec_q.c_Create(4);

	double time_stamp;
	int cnt, ww, hh;

	//////////////////////////////////////////////////////////////////////////
	// for debugging.
// 	CKvString fn;
// 	LCKvIO io; io.gsf_Get_Save_Filename("Open save file name.", fn);
// 	std::ofstream f_sv(fn.bp());	f_sv.precision(14);
// 
// 	if(f_sv.fail())	return false;
	//////////////////////////////////////////////////////////////////////////
		
	cnt = 0;
	while(!f.eof() && cnt < zz_set_of_fn_depth.vs()){

		CKvYooji_Extrinsics *t_ext = new CKvYooji_Extrinsics;

		// timestamp tx ty tz qx qy qz qw (8 elements)
		f >> time_stamp;	// get frame number.

		for(int i=0; i<3; i++) f >> vec_cen.vp()[i];	// get translation vector.
		for(int i=0; i<4; i++) f >> vec_q.vp()[i];		// get unit-quaternion vector.

//		printf("vec: %f %f %f\n", vec_cen.vp()[0], vec_cen.vp()[1], vec_cen.vp()[2]);
 		printf("vec: %f %f %f %f\n", vec_q.vp()[0], vec_q.vp()[1], vec_q.vp()[2], vec_q.vp()[3]);
// 		system("pause");
		
		//////////////////////////////////////////////////////////////////////////
		// convert unit quaternion vector.
		t_ext->convert_quaternion_to_rotation_mat(vec_q, mat_R);
		
		// for debugging.
// 		t_ext->convert_rotation_mat_to_quaternion(mat_R, vec_q);
// 		f_sv << time_stamp << " " << vec_cen.vp()[0] << " " << vec_cen.vp()[1] << " " << vec_cen.vp()[2] << " ";
// 		f_sv << vec_q.vp()[0] << " " << vec_q.vp()[1] << " " << vec_q.vp()[2] << " " << vec_q.vp()[3] << endl;
// 
// 		printf("vec_2: %f %f %f %f\n", vec_q.vp()[0], vec_q.vp()[1], vec_q.vp()[2], vec_q.vp()[3]);
		//////////////////////////////////////////////////////////////////////////

		// set transformation matrix.
		// + R.
		for(int j=0; j<3; j++){ for(int i=0; i<3; i++){ p_mat[j*4+i] = p_R[j*3+i]; } }
		
  		// + t = -RC.
 		for(int j=0; j<3; j++){
 			p_mat[j*4+3] = 0.0f;
 			for(int i=0; i<3; i++) p_mat[j*4+3] += -p_R[j*3+i]*p_cen[i];
 		}
		// + t = t.
		//for(int j=0; j<3; j++)	p_mat[j*4+3] = p_cen[j];

		pose_curr.set_from_transform(&mat_T);		

		// 1 ---------
		// set initial pose.		
// 		if(cnt==0) pose_init.copy(&pose_curr);
// 		// get relative pose for initial frame.
// 		pose_init.get_pose_relative(pose_init, pose_curr, pose_prev);

		t_ext->set_from_transform(&mat_T);

 		out_trace->push_back(t_ext);

		cout << "#" << cnt << " frame was loaded..." << endl;
		cnt++;
	}
	f.close();
//	f_sv.close();

	cout << "Trace loading was complete." << endl;

	return true;
}

//***********************************************************************************************************************
bool CKvYooji_FrameCapture::stt_Save_Trace_to_Txt_for_Steinsbrucker(
	CKvString &in_filename_of_camera_information,
	vector<CKvYooji_Extrinsics*> *in_trace)
//***********************************************************************************************************************
{
	const char *bp = (const char *)in_filename_of_camera_information.bp();
	std::ofstream f(bp);

	if(f.fail())	return false;

	CKvMatrixFloat t_mat;	t_mat.ci_Create_Identity_matrix(4);
	int cnt, ww, hh;
	
	cnt = 0;
	while(!f.eof()){

		f << cnt << endl;	// get frame number.

		t_mat.cp_Copy((*in_trace)[cnt]->mp_transform());

		for(int i=0; i<16; i++){
						
			if(i%4==0) f << endl;
			f << t_mat.vp()[i] << " ";
		}		
		f << endl;

		d_pm_Printf_Matrix(t_mat.vp(), 4, 4, "t_mat");
				
		cout << "#" << cnt << " frame was saved..." << endl;

		cnt++;
	}
	f.close();

	cout << "Trace saving was complete." << endl;

	return true;
}

//***********************************************************************************************************************
bool CKvYooji_FrameCapture::ls_Load_Silhouette(
		int in_file_idx,
		CKvMatrixBool *out_silhouette)
//***********************************************************************************************************************
{
	out_silhouette->c_Create(1, 1, false);

	if(in_file_idx<0 || in_file_idx>=zz_num_file)	return false;
	if(!zz_flag_sil)	return false;

	CKvMatrixUchar sil_img;

	// load silhouette.	
	if(!zz_iof.li_Load_Image(zz_set_of_fn_sil.vp()[in_file_idx], false, &sil_img)) return false;
	zz_uim.mucb_Matrix_Uchar_to_Bool(&sil_img, 1, out_silhouette);

	return true;
}

//***********************************************************************************************************************
bool CKvYooji_FrameCapture::lri_Load_Rgb_Image(
		int in_file_idx,
		CKvMatrixUcharRgb *out_img_rgb)
//***********************************************************************************************************************
{
	if(in_file_idx<0 || in_file_idx>=zz_num_file)	return false;
	if(!zz_flag_rgb_img)	return false;

	// load texture image.	
	if(!zz_iof.li_Load_Image(zz_set_of_fn_color.vp()[in_file_idx], false, out_img_rgb)) return false;

	return true;
}

//***********************************************************************************************************************
bool CKvYooji_FrameCapture::ldm_Load_Depth_Map(
	int in_file_idx,
	CKvMatrixFloat *out_map_depth)
//***********************************************************************************************************************
{
	CKvMatrixUshort tdepth;

	if(in_file_idx<0 || in_file_idx>=zz_num_file)	gerr("1");
	if(!zz_flag_depth_map_short && !zz_flag_depth_map_float)	gerr("2");//return false;

	// load depth map.
	if(zz_type_depth == KV_TYPE_DEPTH_FLOAT){
		if(!zz_iof.lvf_Load_from_Vcl_File(zz_set_of_fn_depth.vp()[in_file_idx], out_map_depth)) gerr("3");//return false;
	}
	else if(zz_type_depth == KV_TYPE_DEPTH_USHORT){
		if(!zz_iof.lvf_Load_from_Vcl_File(zz_set_of_fn_depth.vp()[in_file_idx], &tdepth)) gerr("4");//return false;
		zz_uim.musf_Matrix_Ushort_to_Float(&tdepth, 0.001f, out_map_depth);
	}
	else  gerr("5");//return false;

	return true;

error:
	Kv_Printf("ldm_Load_Depth_Map");
}

//***********************************************************************************************************************
bool CKvYooji_FrameCapture::lrdi_Load_Rgb_and_Depth_Images(
	int in_file_idx,
	CKvMatrixUcharRgb *out_img_rgb, 
	CKvMatrixFloat *out_img_d)
//***********************************************************************************************************************
{
	if(in_file_idx<0 || in_file_idx>=zz_num_file)	return false;
	if(!zz_flag_rgb_img)	return false;
	if(!zz_flag_depth_map_short && !zz_flag_depth_map_float)	return false;

	CKvMatrixUshort tdepth;
	
	// load depth map.
	if(zz_type_depth == KV_TYPE_DEPTH_FLOAT){
		if(!zz_iof.lvf_Load_from_Vcl_File(zz_set_of_fn_depth.vp()[in_file_idx], out_img_d)) return false;
	}
	else if(zz_type_depth == KV_TYPE_DEPTH_USHORT){
		if(!zz_iof.lvf_Load_from_Vcl_File(zz_set_of_fn_depth.vp()[in_file_idx], &tdepth)) return false;
		zz_uim.musf_Matrix_Ushort_to_Float(&tdepth, 0.001f, out_img_d);
	}
	else return false;
	// load texture image.	
	if(!zz_iof.li_Load_Image(zz_set_of_fn_color.vp()[in_file_idx], false, out_img_rgb)) return false;

	return true;
}

//***********************************************************************************************************************
bool CKvYooji_FrameCapture::lrdc_Load_Rgb_and_Depth_from_CxD_File(
	int in_file_idx,
	CKvMatrixUcharRgb *out_img_rgb,
	CKvMatrixFloat *out_img_d)
//***********************************************************************************************************************
{
	if(in_file_idx<0 || in_file_idx>=zz_num_file)	return false;
	if(!zz_flag_cxd)	return false;

	// load cxd file.
	zz_cxd.ld_Load_Data(zz_set_of_fn_cxd.vp()[in_file_idx], false);
	// get depth map and color image.
	out_img_rgb->cp_Copy(zz_cxd.pi());
	zz_uim.musf_Matrix_Ushort_to_Float(zz_cxd.pd(), 0.001f, out_img_d);

	return true;

}

//***********************************************************************************************************************
bool CKvYooji_FrameCapture::lprdi_Load_P_Matrices_and_Rgb_and_Depth_Images(
	int in_file_idx,
	CKvYooji_MatrixRgbD *out_mat_rgbd)
//***********************************************************************************************************************
{
	if(in_file_idx<0 || in_file_idx>=zz_num_file)	return false;

	CKvMatrixUshort tdepth;
	CKvMatrixFloat tmat;
	if(!zz_iof.lvf_Load_from_Vcl_File(zz_set_of_fn_pmat.vp()[in_file_idx], &tmat)) return false;
	out_mat_rgbd->p_P_matrix_depth()->i_Import(&tmat);
	out_mat_rgbd->p_P_matrix_RGB()->i_Import(&tmat);

	// load depth map.
	if(zz_type_depth == KV_TYPE_DEPTH_FLOAT){
		if(!zz_iof.lvf_Load_from_Vcl_File(zz_set_of_fn_depth.vp()[in_file_idx], out_mat_rgbd->p_map_depth_raw())) return false;
	}
	else if(zz_type_depth == KV_TYPE_DEPTH_USHORT){
		if(!zz_iof.lvf_Load_from_Vcl_File(zz_set_of_fn_depth.vp()[in_file_idx], &tdepth)) return false;
		zz_uim.musf_Matrix_Ushort_to_Float(&tdepth, 0.001f, out_mat_rgbd->p_map_depth_raw());
	}
	else return false;
	// load texture image.	
	if(!zz_iof.li_Load_Image(zz_set_of_fn_color.vp()[in_file_idx], false, out_mat_rgbd->p_image_rgb())) return false;

	return true;
}

//***********************************************************************************************************************
bool CKvYooji_FrameCapture::soc_Save_Object_Cube(
	CKvPoint3Df &in_cube_origin,
	float in_cube_size)
//***********************************************************************************************************************
{
	const char *bp = (const char *)zz_fn_object_cube.bp();
	std::ofstream f;
	f.open(bp);

	f << in_cube_origin.x << ' ' << in_cube_origin.y << ' ' << in_cube_origin.z << ' ' << in_cube_size << endl;
	f.close();

	return true;
}

//***********************************************************************************************************************
bool CKvYooji_FrameCapture::loc_Load_Object_Cube(
	CKvString &in_file_name,
	CKvPoint3Df &out_cube_origin,
	float &out_cube_size,
	int &out_cube_dim)
//***********************************************************************************************************************
{
	const char *bp = (const char *)in_file_name.bp();
	std::ifstream f(bp);

	if(f.fail())	return false;

	float tvec[5];
	int cnt;

	// read object cube. 
	// --> 4 values [cube origin x y z / cube size w / cube dimension d]
	cnt=0;	
	while(!f.eof()){
		f >> tvec[cnt];	cnt++;
		
	}
	f.close();

	if(cnt<4) return false;

	out_cube_origin.x = tvec[0];
	out_cube_origin.y = tvec[1];
	out_cube_origin.z = tvec[2];
	out_cube_size = tvec[3];	
	out_cube_dim = (int)tvec[4];

	return true;
}

//***********************************************************************************************************************
bool CKvYooji_FrameCapture::loc_Load_Object_Cube(
	CKvPoint3Df &out_cube_origin,
	float &out_cube_size)
//***********************************************************************************************************************
{
	const char *bp = (const char *)zz_fn_object_cube.bp();
	std::ifstream f(bp);

	if(f.fail())	return false;

	float tvec[4];
	int cnt;

	// read object cube. 
	// --> 4 values [cube origin x y z / cube size w]
	cnt=0;	
	while(!f.eof()){
		f >> tvec[cnt];	cnt++;
		
	}
	f.close();

	if(cnt<4) return false;

	out_cube_origin.x = tvec[0];
	out_cube_origin.y = tvec[1];
	out_cube_origin.z = tvec[2];
	out_cube_size = tvec[3];	

	return true;
}

//***********************************************************************************************************************
 bool CKvYooji_FrameCapture::lgcm_Load_Glove_Color_Model(CKvYooji_ColorModel *out_color_model)
//***********************************************************************************************************************
 {
	 return out_color_model->lcm_Load_Color_Model(zz_fn_gmm_glove);
 }

//***********************************************************************************************************************
bool CKvYooji_FrameCapture::lci_Load_Camera_Information(
	CKvString &in_filename_of_camera_information,
	CKvYooji_MatrixRgbD *out_view)
//***********************************************************************************************************************
{
	const char *bp = (const char *)in_filename_of_camera_information.bp();
	std::ifstream f(bp);

	if(f.fail())	return false;

	CKvPmatrix3D pmat_depth, pmat_rgb;
	CKvMatrix P;
	double *p_P;
	int cnt, ww, hh;

	// read image sizes.
	f >> ww;	f >> hh;	out_view->p_map_depth_raw()->c_Create(hh, ww, 0.0f);	
	f >> ww;	f >> hh;	out_view->p_image_rgb()->c_Create(hh, ww, Kv_Rgb(0, 0, 0));	
	// read camera matrices.
	cnt = 0;	p_P = P.c_Create(3, 4, 0.0f)[0];

	while(!f.eof()){

		f >> p_P[cnt%12];	cnt++;

		if(cnt==12)			pmat_depth.i_Import(&P);	// the first 12 elements are P matrix of depth camera.
		else if(cnt==24)	pmat_rgb.i_Import(&P);		// the second 12 elements are P matrix of color camera.
	}
	f.close();

	if(cnt<24)	return false;

	// Convert extrinsics of depth camera to canonical form.
	CKvYooji_Tracking_State state;
	CKvYooji_Intrinsics int_depth, int_rgb;
	CKvYooji_Extrinsics ext_depth, ext_rgb, ext_rel, ext_cano;

	int_depth.set_from_P_matrix(&pmat_depth);	ext_depth.set_from_P_matrix(&pmat_depth);
	int_rgb.set_from_P_matrix(&pmat_rgb);		ext_rgb.set_from_P_matrix(&pmat_rgb);
	ext_cano.set_from_params(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	ext_rgb.get_pose_relative(ext_depth, ext_rgb, ext_rel);

	// + compute new p matrices.
	state.compute_P_matrix(&int_depth, &ext_cano, &pmat_depth);
	state.compute_P_matrix(&int_rgb, &ext_rel, &pmat_rgb);

	// update intrinsics and extrinsics of depth and rgb camera.
	out_view->set_camera_information(pmat_depth, pmat_rgb);

	d_pm_Printf_Matrix(pmat_depth.mpp()->vp(), 3, 4, "pmat_d");
	d_pm_Printf_Matrix(pmat_rgb.mpp()->vp(), 3, 4, "pmat_rgb");

	return true;
}


//***********************************************************************************************************************
bool CKvYooji_FrameCapture::lcis_Load_Camera_Information_System(
	CKvYooji_MatrixRgbD *out_view)
//***********************************************************************************************************************
{
	const char *bp = (const char *)zz_fn_cam_params.bp();
	std::ifstream f(bp);

	if(f.fail())	return false;

	CKvPmatrix3D pmat_depth, pmat_rgb;
	CKvMatrix P;
	double *p_P;
	int cnt, ww, hh;

	// read image sizes.
	f >> ww;	f >> hh;	out_view->p_map_depth_raw()->c_Create(hh, ww, 0.0f);	
	f >> ww;	f >> hh;	out_view->p_image_rgb()->c_Create(hh, ww, Kv_Rgb(0, 0, 0));	
	// read camera matrices.
	cnt = 0;	p_P=P.c_Create(3, 4, 0.0f)[0];
	while(!f.eof()){

		f >> p_P[cnt%12];	cnt++;

		if(cnt==12)			pmat_depth.i_Import(&P);	// the first 12 elements are P matrix of depth camera.
		else if(cnt==24)	pmat_rgb.i_Import(&P);		// the second 12 elements are P matrix of color camera.
	}
	f.close();

	if(cnt<24)	return false;

	// Convert extrinsics of depth camera to canonical form.
	CKvYooji_Tracking_State state;
	CKvYooji_Intrinsics int_depth,int_rgb;
	CKvYooji_Extrinsics ext_depth,ext_rgb,ext_rel,ext_cano;

	int_depth.set_from_P_matrix(&pmat_depth);	ext_depth.set_from_P_matrix(&pmat_depth);
	int_rgb.set_from_P_matrix(&pmat_rgb);		ext_rgb.set_from_P_matrix(&pmat_rgb);

	ext_cano.set_from_params(0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);
	ext_rgb.get_pose_relative(ext_depth,ext_rgb,ext_rel);

	// + compute new p matrices.
	state.compute_P_matrix(&int_depth,&ext_cano,&pmat_depth);
	state.compute_P_matrix(&int_rgb,&ext_rel,&pmat_rgb);

	d_pm_Printf_Matrix(ext_rel.mp_transform()->vp(),4,4,"T_Rgb");
	d_pm_Printf_Matrix(pmat_rgb.mpp()->vp(),3,4,"pmat_rgb");

	out_view->set_camera_information(pmat_depth, pmat_rgb);

	return true;
}

//////////////////////////////////////////////////////////////////////////
// New for background cut at 2017.06.12.
//***********************************************************************************************************************
void CKvYooji_FrameCapture::sbcmp_Save_Background_Color_Model_Pixelwise(
	CKvYooji_ColorModel *in_color_model)
//***********************************************************************************************************************
{

	in_color_model->scmp_Save_Color_Model_Pixelwise(zz_fn_gmm_back_pixels);

	return ;
}


//***********************************************************************************************************************
void CKvYooji_FrameCapture::sbcmg_Save_Background_Color_Model_Global(
	CKvYooji_ColorModel *in_color_model)
//***********************************************************************************************************************
{

	in_color_model->scm_Save_Color_Model(zz_fn_gmm_back);

	return ;
}


//***********************************************************************************************************************
bool CKvYooji_FrameCapture::lbcmp_Load_Background_Color_Model_Pixelwise(
	CKvYooji_ColorModel *out_color_model)
//***********************************************************************************************************************
{
	return out_color_model->lcmp_Load_Color_Model_Pixelwise(zz_fn_gmm_back_pixels);	
}

//***********************************************************************************************************************
bool CKvYooji_FrameCapture::lbcmg_Load_Background_Color_Model_Global(
	CKvYooji_ColorModel *out_color_model)
//***********************************************************************************************************************
{
	return out_color_model->lcm_Load_Color_Model(zz_fn_gmm_back);	
}

// =======================================================================
// Save & load reconstructed results.
// =======================================================================

//********************************************************************************************
void CKvYooji_FrameCapture::stsdfc_Save_TSDF_Cube(CKvString &in_foder_path)
//********************************************************************************************
{
	LCKvIO_FileVcl iofv;
	CKvString fn_depth, fn_color, fn_weight;

	fn_depth.fm_Format("%s\\tsdf_depth.vcl", in_foder_path.bp());
	fn_color.fm_Format("%s\\tsdf_color.vcl", in_foder_path.bp());
	fn_weight.fm_Format("%s\\tsdf_weight.vcl", in_foder_path.bp());

// 	iofv.svf_Save_as_Vcl_File(fn_depth, zz_cube.pvd_Pointer_of_Volume_Depth());
// 	iofv.svf_Save_as_Vcl_File(fn_color, zz_cube.pvr_Pointer_of_Volume_Rgb());
// 	iofv.svf_Save_as_Vcl_File(fn_weight, zz_cube.pvwd_Pointer_of_Volume_Weight_Depth());
}

//********************************************************************************************
void CKvYooji_FrameCapture::sdc_Save_Do_Cube(CKvString &in_foder_path)
//********************************************************************************************
{
	LCKvIO_FileVcl iofv;
	CKvString fn_depth, fn_color, fn_weight;

	fn_depth.fm_Format("%s\\do_cube.vcl", in_foder_path.bp());

	//iofv.svf_Save_as_Vcl_File(fn_depth, zz_cube.pvhs_Pointer_of_Visual_Hull_Segments());
}

//********************************************************************************************
void CKvYooji_FrameCapture::ltsdfc_Load_TSDF_Cube(CKvString &in_foder_path,
	CKvYooji_Cube_TSDF_Float &out_tsdf_cube)
//********************************************************************************************
{
	LCKvIO_FileVcl iofv;
	CKvString fn_cube_inform, fn_depth, fn_color, fn_weight;
	CKvPoint3Df origin;
	float sz_cube, len_edge;
	int dim_cube;

	fn_cube_inform.fm_Format("%s\\object_cube.txt", in_foder_path.bp());

	fn_depth.fm_Format("%s\\tsdf_depth.vcl", in_foder_path.bp());
	fn_color.fm_Format("%s\\tsdf_color.vcl", in_foder_path.bp());
	fn_weight.fm_Format("%s\\tsdf_weight.vcl", in_foder_path.bp());

	loc_Load_Object_Cube(fn_cube_inform, origin, sz_cube, dim_cube);
	out_tsdf_cube.c_Create(origin, dim_cube, dim_cube, dim_cube, 8, sz_cube/(float)dim_cube);

	iofv.lvf_Load_from_Vcl_File(fn_depth, out_tsdf_cube.pvd_Pointer_of_Volume_Depth());
	iofv.lvf_Load_from_Vcl_File(fn_color, out_tsdf_cube.pvr_Pointer_of_Volume_Rgb());
	iofv.lvf_Load_from_Vcl_File(fn_weight, out_tsdf_cube.pvwd_Pointer_of_Volume_Weight_Depth());
}