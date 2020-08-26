/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_shape_from_silhouette.cpp 
/////////////////////////////////////////////////////////////////////////////////////////////

#include "../../../stdafx.h"
#include "../../_yooji_2016_object_scanning.h"

/////////////////////////////////////////////////////////////////////////////////////////////
// LCKvYooji_Shape_from_Silhouette 
/////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
LCKvYooji_Shape_from_Silhouette::LCKvYooji_Shape_from_Silhouette()
//********************************************************************************************
{
	zz_classname = "LCKvYooji_Shape_from_Silhouette";
	zz_mat_4x4.ci_Create_Identity_matrix(4);
}

//********************************************************************************************
LCKvYooji_Shape_from_Silhouette::~LCKvYooji_Shape_from_Silhouette()
//********************************************************************************************
{
}

//********************************************************************************************
void LCKvYooji_Shape_from_Silhouette::i_Initialize(CKvYooji_Cube_TSDF_Float *in_cube)
//********************************************************************************************
{
	int ww, hh, dd;
	in_cube->ts(ww, hh ,dd);

	// for incremental SfS.
	zz_set_ray_float.c_Create(ww, hh);
	zz_docube_sfs.sc_Set_Cube(ww, hh, dd);
	zz_docube_sfs.gp_rp_Get_Pointer_of_Reference_and_Direction_Vectors()->cp_Copy(in_cube->prpdv_Pointer_of_Reference_Points_and_Direction_Vectors());
	zz_docube_sfs.gp_h_Get_Pointer_of_Homography()->cp_Copy(in_cube->ph3dnc_Homography_3D_for_Normalizing_Cube());

	// for overall visual-hull shield.
 	zz_docube.sc_Set_Cube(ww, hh, dd);
 	zz_docube.gp_rp_Get_Pointer_of_Reference_and_Direction_Vectors()->cp_Copy(in_cube->prpdv_Pointer_of_Reference_Points_and_Direction_Vectors());
 	zz_docube.gp_h_Get_Pointer_of_Homography()->cp_Copy(in_cube->ph3dnc_Homography_3D_for_Normalizing_Cube());

	// for incremental depth carving.
	zz_set_of_rays.c_Create(ww,hh);
	zz_docube_for_mesh.sc_Set_Cube(ww, hh, dd);
	zz_docube_for_mesh.gp_rp_Get_Pointer_of_Reference_and_Direction_Vectors()->cp_Copy(in_cube->prpdv_Pointer_of_Reference_Points_and_Direction_Vectors());
	zz_docube_for_mesh.gp_h_Get_Pointer_of_Homography()->cp_Copy(in_cube->ph3dnc_Homography_3D_for_Normalizing_Cube());

	zz_docube_init.sc_Set_Cube(ww, hh, dd);
	zz_docube_init.gp_rp_Get_Pointer_of_Reference_and_Direction_Vectors()->cp_Copy(in_cube->prpdv_Pointer_of_Reference_Points_and_Direction_Vectors());
	zz_docube_init.gp_h_Get_Pointer_of_Homography()->cp_Copy(in_cube->ph3dnc_Homography_3D_for_Normalizing_Cube());

}

//********************************************************************************************
void LCKvYooji_Shape_from_Silhouette::usif_Update_Silhouette_Induced_Field(
	vector<CKvYooji_MatrixRgbD> *in_set_of_mat_rgbd,	
	vector<CKvPmatrix3D> *in_set_of_p_matrices,
	CKvYooji_Cube_TSDF_Float *io_cube)
//********************************************************************************************
{
	z_cor_Compute_Object_Rims(in_set_of_mat_rgbd, in_set_of_p_matrices, io_cube);
}
//
////********************************************************************************************
//void LCKvYooji_Shape_from_Silhouette::sfsi_Shape_From_Silhouette_Incremental(
//	CKvPmatrix3D *in_pmat_rgb,
//	CKvMatrixBool *in_mask_object,
//	CKvYooji_Tracking_State *in_state,
//	CKvSet2d_of_VectorFloat *io_docube_segments,
//	bool in_flag_is_first_frame)
////********************************************************************************************
//{
//	// for shape-from-silhouette.
//	CKvSet_of_Pmatrix3D set_of_pmat_rgb, set_of_npmat_rgb;
//	CKvSet_of_SdkCode set_of_sdkc;		
//	CKvRunSet rs;
//
//	CKvHmatrix3D *p_h3d_norm;
//	CKvPmatrix3D *pmat_rgb, *npmat_rgb;
//	CKvMatrixFloat *p_rpdv;
//	CKvMatrixFloat in_mat_3x4, in_mat_4x4, out_mat_3x4;
//	
//	int ww_c, hh_c, dd_c;
//
//
//	CKvSet2d_of_VectorShort *visual_hull;
//	CKvVectorFloat *p_seg;
//	CKvPoint3D tp3d, p3d_start, p3d_end;
//	CKvPoint3Df vox_start, vox_end, tp3df;
//
//	float *vp_seg, *vp_rpdv;
//	SHORT *vp_visual_hull;
//	int i, j, sz_vec;	
//	bool flag_valid;
//
//	//io_cube->ts(ww_c, hh_c, dd_c);
//	zz_docube.gr_Get_Resolution(ww_c, hh_c, dd_c);
//
//	// XY-plane SFS.
//	//if(io_docube_segments->vs()!=ww_c*hh_c)	io_docube_segments->c_Create(ww_c*hh_c);
//	// ZY-plane SFS.
//	if(io_docube_segments->mw()!=dd_c || io_docube_segments->mh()!=hh_c)	io_docube_segments->c_Create(hh_c, dd_c);
//
//	p_h3d_norm = zz_docube.gp_h_Get_Pointer_of_Homography();//io_cube->ph3dnc_Homography_3D_for_Normalizing_Cube();
//	p_rpdv = zz_docube.grpdv_Get_Reference_Points_and_Direction_Vectors();//io_cube->prpdv_Pointer_of_Reference_Points_and_Direction_Vectors();
//	//if(!Kv_Printf("%d %d\n", p_rpdv->mw(), p_rpdv->mh())) exit(0);
//
//	set_of_pmat_rgb.c_Create(1);	set_of_npmat_rgb.c_Create(1);	set_of_sdkc.c_Create(1);
//	in_mat_3x4.c_Create(3, 4, 0.0f);	in_mat_4x4.c_Create(4, 4, 0.0f);	out_mat_3x4.c_Create(3, 4, 0.0f);	
//	
//	// set inputs of do-ray-carving function.
//	rs.i_Import(in_mask_object);
//	set_of_sdkc.gpe_Get_Pointer_of_Element(0)->i_Import(&rs, true);
//	pmat_rgb = set_of_pmat_rgb.gpe_Get_Pointer_of_Element(0);
//	npmat_rgb = set_of_npmat_rgb.gpe_Get_Pointer_of_Element(0);
//
//	// =================================================================== ???????
//	in_state->cp_Compute_Pmatrix(
//		&in_state->ppir_Pointer_of_Pyramid_Intrinsics_RGB()->intrins[0],
//		in_state->pt3dr_Pointer_of_3D_Transform_Global_to_Camera_RGB(),
//		pmat_rgb);
//
//	//d_pm_Printf_Matrix(pmat_rgb->mpp()->vp(), 3, 4, "P_rgb");
//	//d_pm_Printf_Matrix(in_state->ppir_Pointer_of_Pyramid_Intrinsics_RGB()->intrins[0].mp_Matrix_Pointer()->vp(), 3, 3, "K_rgb");
//	// ===================================================================
//
//	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//
//	// Ray carving 하려면 현재 설정된 cube를 원점을 중심으로 하는 unit sphere 안에 들어오게 해야함.
//	zz_util_dosf.tsp_Transform_Set_of_P_matrices(
//		&set_of_pmat_rgb,
//		p_h3d_norm,
//		&set_of_npmat_rgb);
//
//	//d_pm_Printf_Matrix(npmat_rgb->mpp()->vp(), 3, 4, "P_n_rgb");
//
//	drci_Do_Ray_Carving_Incremental(
//		p_rpdv,
//		set_of_npmat_rgb.gpe_Get_Pointer_of_Element(0),
//		set_of_sdkc.gpe_Get_Pointer_of_Element(0),
//		io_docube_segments,
//		in_flag_is_first_frame);
//
//}


//********************************************************************************************
void LCKvYooji_Shape_from_Silhouette::sfsi_Shape_From_Silhouette_Incremental(
	CKvPmatrix3D *in_pmat_rgb, // 현재 안쓰이는 듯...
	CKvMatrixBool *in_mask_object,
	CKvYooji_Tracking_State *in_state,	
	CKvSet2d_of_VectorFloat *io_docube_segments,
	bool in_flag_is_first_frame)
//********************************************************************************************
{
	// for shape-from-silhouette.
	CKvSet_of_Pmatrix3D set_of_pmat_rgb, set_of_npmat_rgb;
	CKvSet_of_SdkCode set_of_sdkc;		
	CKvRunSet rs;

	CKvHmatrix3D *p_h3d_norm;
	CKvPmatrix3D *pmat_rgb, *npmat_rgb;
	CKvMatrixFloat *p_rpdv;
	CKvMatrixFloat in_mat_3x4, in_mat_4x4, out_mat_3x4;
	
	int ww_c, hh_c, dd_c, ww, hh, dd;


	CKvSet2d_of_VectorShort *visual_hull;
	CKvVectorFloat *p_seg;
	CKvPoint3D tp3d, p3d_start, p3d_end;
	CKvPoint3Df vox_start, vox_end, tp3df;

	float *vp_seg, *vp_rpdv;
	SHORT *vp_visual_hull;
	int i, j, sz_vec;	
	bool flag_valid;

	zz_docube_sfs.gr_Get_Resolution(ww_c, hh_c, dd_c);
	
	// XY-plane SFS.
	//if(io_docube_segments->vs()!=ww_c*hh_c)	io_docube_segments->c_Create(ww_c*hh_c);
	if(io_docube_segments){
		if(io_docube_segments->mw()!=ww_c || io_docube_segments->mh()!=hh_c)	io_docube_segments->c_Create(hh_c,ww_c);
	}
	else{
		zz_set_ray_float.ms(ww,hh);
		if(ww !=ww_c || hh != hh_c)	 zz_set_ray_float.c_Create(ww_c,hh_c);
	}
	// ZY-plane SFS. --> YZ 가 되어야 하는거 아닌가? 그리고 원래대로 XY plane 기준으로 DoCube 를 만들어도 될듯...
// 	if(io_docube_segments)
// 		if(io_docube_segments->mw()!=dd_c || io_docube_segments->mh()!=hh_c)	io_docube_segments->c_Create(hh_c, dd_c);
// 	else{
// 		zz_set_ray_float.ms(ww, hh);
// 		if(ww !=dd_c || hh != hh_c)	 zz_set_ray_float.c_Create(dd_c, hh_c);
// 	}

	p_h3d_norm = zz_docube_sfs.gp_h_Get_Pointer_of_Homography();//io_cube->ph3dnc_Homography_3D_for_Normalizing_Cube();
	p_rpdv = zz_docube_sfs.grpdv_Get_Reference_Points_and_Direction_Vectors();//io_cube->prpdv_Pointer_of_Reference_Points_and_Direction_Vectors();
	//if(!Kv_Printf("%d %d\n", p_rpdv->mw(), p_rpdv->mh())) exit(0);

	set_of_pmat_rgb.c_Create(1);	set_of_npmat_rgb.c_Create(1);	set_of_sdkc.c_Create(1);
	in_mat_3x4.c_Create(3, 4, 0.0f);	in_mat_4x4.c_Create(4, 4, 0.0f);	out_mat_3x4.c_Create(3, 4, 0.0f);	
	
	// set inputs of do-ray-carving function.
	rs.i_Import(in_mask_object);
	set_of_sdkc.gpe_Get_Pointer_of_Element(0)->i_Import(&rs, true);
	pmat_rgb = set_of_pmat_rgb.gpe_Get_Pointer_of_Element(0);
	npmat_rgb = set_of_npmat_rgb.gpe_Get_Pointer_of_Element(0);

	// =================================================================== ???????
// 	in_state->cp_Compute_Pmatrix(
// 		&in_state->ppir_Pointer_of_Pyramid_Intrinsics_RGB()->intrins[0],
// 		in_state->pt3dr_Pointer_of_3D_Transform_Global_to_Camera_RGB(),
// 		pmat_rgb);
	pmat_rgb->cp_Copy(in_pmat_rgb);

	//CKvMatrixFloat tmat;
	//p_h3d_norm->e_Export(&tmat);
	//d_pm_Printf_Matrix(tmat.vp(), 4, 4, "H_norm");
	d_pm_Printf_Matrix(pmat_rgb->mpp()->vp(), 3, 4, "P_rgb");
	//d_pm_Printf_Matrix(in_state->ppir_Pointer_of_Pyramid_Intrinsics_RGB()->intrins[0].mp_Matrix_Pointer()->vp(), 3, 3, "K_rgb");
	// ===================================================================

	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

	// Ray carving 하려면 cube가 unit sphere를 접하면서 감싸도록 normalize 해 주어야 함.
	zz_util_dosf.tsp_Transform_Set_of_P_matrices(
		&set_of_pmat_rgb,
		p_h3d_norm,
		&set_of_npmat_rgb);

	d_pm_Printf_Matrix(npmat_rgb->mpp()->vp(), 3, 4, "P_n_rgb");

	CKvMatrixFloat mat;
	p_h3d_norm->e_Export(&mat);
	d_pm_Printf_Matrix(mat.vp(), 3, 4, "p_h3d_norm");

	drci_Do_Ray_Carving_Incremental(
		p_rpdv,
		set_of_npmat_rgb.gpe_Get_Pointer_of_Element(0),
		set_of_sdkc.gpe_Get_Pointer_of_Element(0),
		&zz_set_ray_float,
		in_flag_is_first_frame);

	//if(!Kv_Printf("%d %d", zz_set_ray_float.mw(), zz_set_ray_float.mh())) exit(0);

	if(in_flag_is_first_frame){
		zz_docube_sfs.i_Import(&zz_set_ray_float,NULL);
		zz_docube.i_Import(&zz_set_ray_float,NULL);
	}
	else{
		// import results of ray carving to DoCube for SfS.
		zz_docube_sfs.i_Import(&zz_set_ray_float,NULL);

		// update global visual-hull shield with new SfS results.
		z_ursitdc_Update_Ray_Segments_based_on_Intersection_of_Two_DoCubes(
			zz_docube_sfs,
			zz_docube);

	}
	
	//if(!Kv_Printf("%d %d", zz_set_ray_float.mw(), zz_set_ray_float.mh())) exit(0);

}


//********************************************************************************************
bool LCKvYooji_Shape_from_Silhouette::rsdi_Refine_Shape_using_Depth_Incremental(
	CKvPmatrix3D *in_pmat_depth,
	CKvMatrixFloat *in_map_depth,
	CKvSet2d_of_VectorFloat *io_docube_segments,
	bool in_flag_is_first_frame)
//********************************************************************************************
{
	// For refine initial visual hull.
	LCKvUtility_for_Pmatrix3D u_pmat;
	CKvMatrixInt m_mesh_indices;
	CKvHmatrix3D *p_homo_norm,homo_rotation;
	CKvMatrix T_mat,R_mat;	CKvVector angles,tvec;
	CKvYooji_Extrinsics ext_cam,ext_rot;
	CKvPoint3D opt_axis;

	// For 
	LCKvUtility_for_Import import;
	CKvSet_of_MatrixUshort set_of_map_depth_obj;
	CKvMesh_of_Triangle mesh_tri;
	CKvSet_of_Point3D set_of_p3d;
	CKvMatrixFloat mesh_indices;

	// ===============================================================================
	// Extract object depth ==========================================================
	// ===============================================================================
	CKvPmatrix3D *p_pmat;
	CKvMatrixFloat *p_map_depth;
	CKvHmatrix3D *p_h3d_norm;

	double theta,d,ip;
	float rx,ry,rz,tx,ty,tz;

	// Get pointers of depth map and camera matrix.
	p_map_depth = in_map_depth;
	p_pmat = in_pmat_depth;	

	// ===============================================================================
	// For display ===================================================================
	// ===============================================================================

	// ===============================================================================
	// Refine initial visual hull ====================================================
	// ===============================================================================
	int numPoints, numTrinangles;
	// Make mesh triangles from object depth map.
	gmtdm_Get_Mesh_of_Triangles_from_a_Depth_Map_New(
		p_map_depth,
		p_pmat,
		set_of_p3d,
		m_mesh_indices,
		numPoints,
		numTrinangles);

	//////////////////////////////////////////////////////////////////////////
// 	CKvVectorInt v_index;
// 	CKvSet_of_VectorInt set_vi;
// 	CKvRgbaF rgb; rgb.r = 0.0f; rgb.g = 255.0f; rgb.b = 0.0f;
// 	int offset, itmp;
// 	zz_depot_p3d2.im_Import(&set_of_p3d);
// 	zz_depot_rgba2.in_Initialize();	
// 	for(int i=0; i<set_of_p3d.vs(); i++) zz_depot_rgba2.ap_Append(false, rgb, itmp);
// 
// 	v_index.c_Create(3*numTrinangles);
// 	for(int i=0; i<numTrinangles; i++){
// 		v_index.vp()[3*i] = m_mesh_indices.mp()[0][i];
// 		v_index.vp()[3*i + 1] = m_mesh_indices.mp()[1][i];
// 		v_index.vp()[3*i + 2] = m_mesh_indices.mp()[2][i];
// 	}
// 	zz_mesh_tri2.in_Initialize(1);
// 	zz_mesh_tri2.u_me_Make_Element(&v_index,&v_index,&v_index,&set_vi);
// 	zz_mesh_tri2.ap_Append(true, &set_vi, &offset, itmp);
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	// 이 함수 내부에 이상한 점 없는지 탐색...
	// 현재 mesh로 깍은 cube랑 sil로 깎은 cube 랑 잘 안맞는듯...
	// 현재 mesh로 깍은 cube랑 sil로 깎은 cube 랑 잘 안맞는듯...
	// 현재 mesh로 깍은 cube랑 sil로 깎은 cube 랑 잘 안맞는듯...
	// 현재 mesh로 깍은 cube랑 sil로 깎은 cube 랑 잘 안맞는듯...
	// 현재 mesh로 깍은 cube랑 sil로 깎은 cube 랑 잘 안맞는듯...
	// 현재 mesh로 깍은 cube랑 sil로 깎은 cube 랑 잘 안맞는듯...

	// Select optimal ray direction for DoCube of mesh surface.
	int ray_direc;
	z_sord_Select_Optimal_Ray_Direction(
		p_pmat,
		ray_direc);

	// Generate DoCube for object mesh triangles by mesh-to-DoCube conversion.
	// + Set initial ray direction is Z axis (0, 0, 1).
	// X axis.
	if(ray_direc == 0){
		zz_docube_init.sr_Set_Rays(0.0f,90.0f,0.0f);
	}
	// Y axis.
	else if(ray_direc == 1){
		zz_docube_init.sr_Set_Rays(-90.0f,0.0f,0.0f);
	}
	// Z axis.
	else if(ray_direc == 2){
		zz_docube_init.sr_Set_Rays(0.0f,0.0f,0.0f);
	}

	//////////////////////////////////////////////////////////////////////////
	// Margin 추가!
	CKvMatrixFloat mat_homo;
	float scale, margin, *p_homo = NULL;
	zz_docube.gp_h_Get_Pointer_of_Homography()->e_Export(&mat_homo);
	p_homo = mat_homo.vp();
	scale = sqrt(SQUARE(p_homo[0])+SQUARE(p_homo[4])+SQUARE(p_homo[8]));
	// set shield margin as thickness of hand. (2cm)
	margin = 0.02f*scale;
	//////////////////////////////////////////////////////////////////////////

	//if(!Kv_Printf("margin: %f scale: %f", margin, scale)) exit(0);
	printf("margin: %f scale: %f\n", margin, scale);

	if(!zz_docube_init.cmdc_Convert_Mesh_to_DoCube(
		&set_of_p3d,
		&m_mesh_indices,
		numPoints,
		numTrinangles,
		p_pmat,
		zz_docube.gp_h_Get_Pointer_of_Homography(), //NULL, //zz_docube.gp_h_Get_Pointer_of_Homography(),
		ray_direc,
		margin))
		return false;

	printf("Ray direction: %d\n", ray_direc);
 
 	// If ray direction of DoCube is not Z-axis, convert ray direction to Z-axis.
 	if(ray_direc != 2){
 		//////////////////////////////////////////////////////////////////////////
 		//return false;
 		//////////////////////////////////////////////////////////////////////////
 		z_crddc_Convert_Ray_Direction_of_DoCube(
 			zz_docube_init,
 			ray_direc,
 			zz_docube_for_mesh);
 	}
	else{
 		//////////////////////////////////////////////////////////////////////////
 		//continue;
 		zz_docube_for_mesh.cp_Copy(&zz_docube_init);
 	}
 
 	//// Refine initial object visual hull using intersecting two DoCubes.
	// 현재 depth DoCube 랑 silhouette DoCube 랑 coordinates 가 안 맞춰져 있는듯?
	// 현재 depth DoCube 랑 silhouette DoCube 랑 coordinates 가 안 맞춰져 있는듯?
	// 현재 depth DoCube 랑 silhouette DoCube 랑 coordinates 가 안 맞춰져 있는듯?
	z_ursitdc_Update_Ray_Segments_based_on_Intersection_of_Two_DoCubes(
		zz_docube_for_mesh,
		zz_docube);

	return true;
}

//********************************************************************************
bool LCKvYooji_Shape_from_Silhouette::drc_Do_Ray_Carving(
	CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
	CKvSet_of_Pmatrix3D *in_set_of_P_matrices,
	CKvSet_of_SdkCode *in_set_of_silhouette_images,
	CKvSet_of_VectorFloat *io_ray_segments)
//********************************************************************************
{
	// <assumption> 
	// (1) -1.0 <= depth <= 1.0
	// (2) rp dot dv = 0
	// (3) mag(dv) = 1
	// (4) mag(rp) < 1 : rp is inside the unit sphere
	CKvMatrixFloat *rp_dv;
	CKvVectorFloat *DOC; 
	CKvPmatrix3D *Pmat;
	CKvSdkCode *sdk;
	int nb_ray,nb_img,ww1; 

	//Initialization
	in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix->ms(ww1,nb_ray); 
	rp_dv = in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix;
	DOC	  = io_ray_segments->vp();
	Pmat  = in_set_of_P_matrices->vps(nb_img);
	sdk   = in_set_of_silhouette_images->vps(ww1);
	
	//Do Ray Carving
	if(!z_drc_Do_Ray_Carving(rp_dv,Pmat,sdk,nb_img,DOC)) gerr("4");

	return true;
error:
	zsm("drc_Do_Ray_Carving");
	return false;
}

//********************************************************************************
bool LCKvYooji_Shape_from_Silhouette::drci_Do_Ray_Carving_Incremental(
	CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
	CKvPmatrix3D *in_P_matrix,
	CKvSdkCode *in_silhouette_image,
	CKvSet2d_of_VectorFloat *io_docube_segments,
	bool in_flag_is_first_frame)
//********************************************************************************
{
	// <assumption> 
	// (1) -1.0 <= depth <= 1.0
	// (2) rp dot dv = 0
	// (3) mag(dv) = 1
	// (4) mag(rp) < 1 : rp is inside the unit sphere
	CKvMatrixFloat *rp_dv;
	CKvVectorFloat *DOC; 
	CKvPmatrix3D *Pmat;
	CKvSdkCode *sdk;
	int nb_ray,nb_img,ww1; 

	//Initialization
	in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix->ms(ww1,nb_ray); 
	rp_dv = in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix;
	DOC	  = io_docube_segments->vp();
	Pmat = in_P_matrix;
	sdk = in_silhouette_image;

	//Do Ray Carving
	if(!z_drci_Do_Ray_Carving_Incremental(
		rp_dv,
		Pmat,
		sdk,
		DOC,
		in_flag_is_first_frame)) gerr("4");

	return true;
error:
	zsm("drci_Do_Ray_Carving_Incremental");
	return false;
}

//********************************************************************************************
void LCKvYooji_Shape_from_Silhouette::uvv_Update_Valid_Volume(CKvVolumeBool *io_valid_volume)
//********************************************************************************************
{
	int ww, hh, dd;
	CKvSet2d_of_VectorShort *p_set_of_seg = zz_docube.gp_Get_Pointer();
	bool *p_vol = io_valid_volume->tps(ww, hh, dd)[0][0];
	
	// re-initialization.
	for(int i=0; i<ww*hh*dd; i++) p_vol[i] = false;
	for(int y=0; y<hh; y++){
		for(int x=0; x<ww; x++){

			CKvVectorShort *cp_seg = p_set_of_seg->gpe(x, y);

			// check null jet.
			if(inj_Is_NULL_Jet(cp_seg)) continue;
			
			int num_seg;
			short *p_seg = cp_seg->vps(num_seg);

			num_seg /= 2;			

			// update valid volume using ray segments of visual-hull shield.
			for(int k=0; k<num_seg; k++){
				// for single ray segment.
				for(int z=p_seg[2*k]; z<=p_seg[2*k+1]; z++) p_vol[z*ww*hh + y*ww + x] = true;
			}
			

		}
	}
}

//********************************************************************************************
void LCKvYooji_Shape_from_Silhouette::nccsfs_Normalize_Cube_Coordinates_into_unit_sphere_for_Shape_From_Silhouette(
	CKvYooji_Cube_TSDF_Float *io_cube)
//********************************************************************************************
{
	CKvMatrixFloat *p_rpdv;
	CKvHmatrix3D *p_hmat;
	float *vp_rpdv;

	CKvPoint3D trans, rp_in, rp_out;
	CKvPoint3Df center;
	float scale, radius, sz_voxel;
	int ww_c, hh_c, dd_c;
	int i, j;

	io_cube->ts(ww_c, hh_c, dd_c);
	p_rpdv=io_cube->prpdv_Pointer_of_Reference_Points_and_Direction_Vectors();
	p_hmat=io_cube->ph3dnc_Homography_3D_for_Normalizing_Cube();	
	// m unit.
	sz_voxel=io_cube->svm_Size_of_Voxel_in_Meter();
	// mm unit.
	//sz_voxel=1000.0f*io_cube->svm_Size_of_Voxel_in_Meter();

	// Normalize min/max value of object cube axes between -1.0 and 1.0.
	// compute maximum radius of input object cube.
	//radius = sz_voxel*sqrt((float)SQUARE(ww_c+1)+(float)SQUARE(hh_c+1)+(float)SQUARE(dd_c+1))/2.0f;
	//scale = 1.0f/radius;	// scale for normalizing input cube.
	//////////////////////////////////////////////////////////////////////////
	// This scale is important because it is related with offset value of DoCube.
	scale = 2.0f/(sz_voxel*(float)(dd_c+1));
	// This scale is important because it is related with offset value of DoCube.
	//////////////////////////////////////////////////////////////////////////
	
	//if(!Kv_Printf("%f %f", scale, 2.0f/(sz_voxel*(float)dd_c))) exit(0);

	// get center of cube for computing translation.	
	io_cube->gcciw_Get_Cube_Center_In_World(center);

	printf("center_cube: %f %f %f\n", center.x, center.y, center.z);


// 	io_cube->gpw_Get_Position_in_World(Kv_Point3Df(
// 		(float)(ww_c-1)/2.0f, (float)(hh_c-1)/2.0f, (float)(dd_c-1)/2.0f), 
// 		center);
	trans.x=-scale*center.x;	trans.y=-scale*center.y;	trans.z=-scale*center.z;
	
	// compute 3D homography for normalizing cube coordinates.
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

	p_hmat->cst_Create_for_Scaling_Translation(scale, trans);

	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

 	// compute set of rays for SFS in normazlied coordinates.
	// + XY plane.
 	if(p_rpdv->mw()!=6 || p_rpdv->mh()!=ww_c*hh_c) p_rpdv->c_Create(ww_c*hh_c, 6);
 	for(j=0; j<hh_c; j++){
 		for(i=0; i<ww_c; i++){
 
 			vp_rpdv=p_rpdv->mp()[j*ww_c+i];
 			rp_in.x=vp_rpdv[0];	rp_in.y=vp_rpdv[1];	rp_in.z=vp_rpdv[2];
 			p_hmat->tp_Transform_Point(true, rp_in, rp_out);
 			vp_rpdv[0]=rp_out.x;	vp_rpdv[1]=rp_out.y;	vp_rpdv[2]=rp_out.z;	// reference point.
 			vp_rpdv[3]=0.0f;		vp_rpdv[4]=0.0f;		vp_rpdv[5]=1.0f;	// direction vector.

			//printf("%f %f %f\n", rp_out.x, rp_out.y, rp_out.z);
 
 		}
 	}
	// + ZY plane.
// 	if(p_rpdv->mw()!=6 || p_rpdv->mh()!=dd_c*hh_c) p_rpdv->c_Create(dd_c*hh_c, 6);
// 	for(j=0; j<hh_c; j++){
// 		for(i=0; i<dd_c; i++){
// 
// 			vp_rpdv=p_rpdv->mp()[j*dd_c+i];
// 			rp_in.x=vp_rpdv[0];	rp_in.y=vp_rpdv[1];	rp_in.z=vp_rpdv[2];
// 			p_hmat->tp_Transform_Point(true, rp_in, rp_out);
// 
// 			vp_rpdv[0]=rp_out.x;	vp_rpdv[1]=rp_out.y;	vp_rpdv[2]=rp_out.z;	// reference point.
// 			vp_rpdv[3]=1.0f;		vp_rpdv[4]=0.0f;		vp_rpdv[5]=0.0f;	// direction vector.
// 
// 		}
// 	}


}

//********************************************************************************************
void LCKvYooji_Shape_from_Silhouette::ncsfs_Normalize_Camera_Matrix_for_Shape_From_Silhouette(
	CKvHmatrix3D *in_hmat_normal,
	CKvSet_of_Pmatrix3D *in_set_of_pmat,
	CKvSet_of_Pmatrix3D *out_set_of_pmat)
//********************************************************************************************
{
	zz_util_dosf.tsp_Transform_Set_of_P_matrices(
		in_set_of_pmat,
		in_hmat_normal,
		out_set_of_pmat);
}

//*****************************************************************************************
bool LCKvYooji_Shape_from_Silhouette::inj_Is_NULL_Jet(CKvVectorShort *in_jet)
//*****************************************************************************************
{
	short *D; int sz;

	D=in_jet->vps(sz); 

	if(sz!=2) return false;
	if(D[0]!=Kv2008_DO_SURFACE_INVALID_MARKER) return false; 
	if(D[0]!=D[1]) return false;

	return true;
}


//*****************************************************************************************
bool LCKvYooji_Shape_from_Silhouette::inj_Is_NULL_Jet(CKvVectorFloat *in_jet)
//*****************************************************************************************
{
	float *D; int sz;

	D=in_jet->vps(sz); //Kv_Printf("sz")

	if(sz!=2) return false;
	if(D[0]!=Kv2008_DO_SURFACE_INVALID_MARKER) return false; 
	if(D[0]!=D[1]) return false;

	return true;
}

//********************************************************************************************
void LCKvYooji_Shape_from_Silhouette::cdcm_Convert_Docube_to_Mesh(
	CKvSet2d_of_VectorFloat *in_docube_segments,
	CKvMatrixFloat *in_ref_points_and_direc_vecs,
	CKvPmatrix3D *in_p_mat,
	CKvGraph3D *io_graph_3d)
//********************************************************************************************
{
	Util_Display util_display;
 	Util_Mesh_Generation mesh_gen;
	Util_VCL_3DO util_3do;

 	CKvSet_of_Pmatrix3D pmat, pmat_norm;
	CKvHmatrix3D *p_homo_norm;
	
	CKvMatrixUchar img_depth;
	CKvVectorInt *p_idx_pnts, *p_face_color1, *p_face_color2;
	CKvVectorInt idx_pnts_total;
	CKvMatrixInt buffer;

	CKvPoint3D p3d_org, p3d_new;
 
 	pmat.c_Create(1);	pmat_norm.c_Create(1);
 	pmat.gpe_Get_Pointer_of_Element(0)->cp_Copy(in_p_mat);

	p_homo_norm = zz_docube.gp_h_Get_Pointer_of_Homography();
 
 	zz_util_dosf.tsp_Transform_Set_of_P_matrices(
 		&pmat,
 		p_homo_norm,
 		&pmat_norm);

	// set.............
	//docube.sr_Set_Resolution(KV_CUBE_DIM, KV_CUBE_DIM, KV_CUBE_DIM);
	//docube.gp_rp_Get_Pointer_of_Reference_and_Direction_Vectors()->cp_Copy(in_ref_points_and_direc_vecs);
	zz_docube.i_Import(in_docube_segments, NULL);
	// set.............
	
	// Render depth map.
	//LCKvYooji_Scanner_Display display;
	//CKvScreen sc;

	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// 0-th element: triangles
	// 1-th element: quadrangles.
	mesh_gen.mm_Make_Meshes(&zz_docube, &zz_depot_p3d, &zz_depot_rgba, &zz_mesh_tri);
	mesh_gen.gimt_Get_Indices_of_Mesh_with_Tetragon(&zz_mesh_tri, 0, &zz_idx_pnts_total);
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

	// Transform point clouds to original coordinates.
	for(int i=0; i<zz_depot_p3d.ne_Number_of_Elements(); i++){
		zz_depot_p3d.ge_Get_Element(i, p3d_org);

		p_homo_norm->tp_Transform_Point(false, p3d_org, p3d_new);

		zz_depot_p3d.se_Set_Element(i, p3d_new);
	}

	// Render 3D model.
	if(io_graph_3d){	
		util_display.pmo_Plot_Mesh_Object(io_graph_3d, &zz_depot_p3d, &zz_depot_rgba, &zz_mesh_tri, NULL);
	}
	
}


//********************************************************************************************
void LCKvYooji_Shape_from_Silhouette::cdcm_Convert_Docube_to_Mesh(	
	CKvPmatrix3D *in_p_mat,
	VCL_DoCube *in_docube,
	//CKvSet2d_of_VectorFloat *in_docube_segments,
	CKvGraph3D *io_graph_3d)
//********************************************************************************************
{
	Util_Display util_display;
 	Util_Mesh_Generation mesh_gen;
	Util_VCL_3DO util_3do;

 	CKvSet_of_Pmatrix3D pmat, pmat_norm;
	CKvHmatrix3D *p_homo_norm;

	CKvMatrixUchar img_depth;
	CKvVectorInt *p_idx_pnts, *p_face_color1, *p_face_color2;
	CKvVectorInt idx_pnts_total;
	CKvMatrixInt buffer;

	CKvPoint3D p3d_org, p3d_new;
	 
 	pmat.c_Create(1);	pmat_norm.c_Create(1);
 	pmat.gpe_Get_Pointer_of_Element(0)->cp_Copy(in_p_mat);
 
	p_homo_norm = zz_docube.gp_h_Get_Pointer_of_Homography();

 	zz_util_dosf.tsp_Transform_Set_of_P_matrices(
 		&pmat,
 		p_homo_norm,
 		&pmat_norm);

	// set.............
	//docube.sr_Set_Resolution(KV_CUBE_DIM, KV_CUBE_DIM, KV_CUBE_DIM);
	//docube.gp_rp_Get_Pointer_of_Reference_and_Direction_Vectors()->cp_Copy(in_ref_points_and_direc_vecs);
	//if(!in_docube) zz_docube.i_Import(&zz_set_ray_float, NULL);
	// set.............
	
	// Render depth map.
	//LCKvYooji_Scanner_Display display;
	//CKvScreen sc;

	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// 0-th element: triangles
	// 1-th element: quadrangles.
// 	if(in_docube)  mesh_gen.mm_Make_Meshes(in_docube, &zz_depot_p3d, &zz_depot_rgba, &zz_mesh_tri);
// 	else    	   mesh_gen.mm_Make_Meshes(&zz_docube_sfs, &zz_depot_p3d, &zz_depot_rgba, &zz_mesh_tri);

	if(in_docube)  mesh_gen.mm_Make_Meshes(in_docube,&zz_depot_p3d,&zz_depot_rgba,&zz_mesh_tri);
	else    	   mesh_gen.mm_Make_Meshes(&zz_docube,&zz_depot_p3d,&zz_depot_rgba,&zz_mesh_tri);

	//////////////////////////////////////////////////////////////////////////
//  	CKvRgbaF rgb; rgb.r=0.0f; rgb.g = 1.0f; rgb.b=0.0f; rgb.a = 1.0f;
// 	mesh_gen.mm_Make_Meshes(&zz_docube_for_mesh, &zz_depot_p3d2, &zz_depot_rgba2, &zz_mesh_tri2,
// 		&rgb);
	//////////////////////////////////////////////////////////////////////////
	// 이놈땜에 죽었어 이노무 시키야
	mesh_gen.gimt_Get_Indices_of_Mesh_with_Tetragon(&zz_mesh_tri, 0, &zz_idx_pnts_total);
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

	// Transform point clouds to original coordinates.
// 	float xx[2], yy[2], zz[2];
// 	xx[0] = yy[0] = zz[0] = 100000.0f;
// 	xx[1] = yy[1] = zz[1] = -100000.0f;
	for(int i=0; i<zz_depot_p3d.ne_Number_of_Elements(); i++){
		zz_depot_p3d.ge_Get_Element(i, p3d_org);

		p_homo_norm->tp_Transform_Point(false, p3d_org, p3d_new);

		zz_depot_p3d.se_Set_Element(i, p3d_new);
	}

//  	for(int i=0; i<zz_depot_p3d2.ne_Number_of_Elements(); i++){
//  
//  		zz_depot_p3d2.ge_Get_Element(i,p3d_org);
//  
//  		p_homo_norm->tp_Transform_Point(false,p3d_org,p3d_new);
//  
//  		zz_depot_p3d2.se_Set_Element(i,p3d_new);
//  
//  		// 		if(p3d_new.x < xx[0]) xx[0] = p3d_new.x; if(p3d_new.x > xx[1]) xx[1] = p3d_new.x;
//  		// 		if(p3d_new.y < yy[0]) yy[0] = p3d_new.y; if(p3d_new.y > yy[1]) yy[1] = p3d_new.y;
//  		// 		if(p3d_new.z < zz[0]) zz[0] = p3d_new.z; if(p3d_new.z > zz[1]) zz[1] = p3d_new.z;
//  
//  		//printf("%f %f %f -> %f %f %f\n",p3d_org.x,p3d_org.y,p3d_org.z
//  		//	,p3d_new.x,p3d_new.y,p3d_new.z);
//  	}

//  		printf("%f %f %f -> %f %f %f\n", p3d_org.x, p3d_org.y, p3d_org.z
//  			, p3d_new.x, p3d_new.y, p3d_new.z);

//	if(!Kv_Printf("x: %f ~ %f y: %f ~ %f z: %f ~ %f\n", xx[0], xx[1], yy[0], yy[1], zz[0], zz[1])) exit(0);
	//if(!Kv_Printf("%d", zz_depot_p3d.ne_Number_of_Elements())) exit(0);

	// Render 3D model.
	if(io_graph_3d){	
		CKvGraph3D g3d;
		util_display.pmo_Plot_Mesh_Object(io_graph_3d, &zz_depot_p3d, &zz_depot_rgba, &zz_mesh_tri, NULL);
		//io_graph_3d->g_p_Plot(NULL,&zz_depot_p3d2,NULL,&zz_depot_rgba2, &zz_mesh_tri2,NULL,NULL,NULL,NULL,NULL,false);
		//util_display.pmo_Plot_Mesh_Object(&g3d, &zz_depot_p3d2, &zz_depot_rgba2, &zz_mesh_tri2, NULL);
		//if(!Kv_Printf("G")) exit(0);

		Kv_Pause(1);
	}

	//if(!Kv_Printf("cdcm_Convert_Docube_to_Mesh: %d", zz_depot_p3d.ne_Number_of_Elements())) exit(0);
	
}

//********************************************************************************************
void LCKvYooji_Shape_from_Silhouette::em_Export_Mesh(
	CKvDepot_of_Point3D *out_depot_p3d,
	CKvVectorInt *out_idx_pnts_vhs)
//********************************************************************************************
{
	// creation 해주어야 할 듯?
	//if(!Kv_Printf("em_Export_Mesh: %d", zz_depot_p3d.ne_Number_of_Elements())) exit(0);
	//if(zz_depot_p3d.ne_Number_of_Elements() != out_depot_p3d->ne_Number_of_Elements())
	//	out_depot_p3d->in_Initialize(zz_depot_p3d.ne_Number_of_Elements());

	out_depot_p3d->cp_Copy(&zz_depot_p3d);
	out_idx_pnts_vhs->cp_Copy(&zz_idx_pnts_total);
}

//********************************************************************************************
void LCKvYooji_Shape_from_Silhouette::rdvh_Render_Depth_from_Visual_Hull(
	CKvSet2d_of_VectorFloat *in_docube_segments,
	CKvMatrixFloat *in_ref_points_and_direc_vecs,
	CKvPmatrix3D *in_p_mat,
	CKvHmatrix3D *in_homo_norm,
	CKvMatrixFloat *io_map_depth,
	CKvGraph3D *io_graph_3d)
//********************************************************************************************
{
	Util_Display util_display;
 	Util_Mesh_Generation mesh_gen;
	Util_VCL_3DO util_3do;

 	CKvSet_of_Pmatrix3D pmat, pmat_norm;
 	CKvDepot_of_Point3D depot_p3d;
 	CKvDepot_of_RgbaF depot_rgba;
 	CKvMesh_of_Triangle mesh_tri;

	CKvMatrixUchar img_depth;
	CKvVectorInt *p_idx_pnts, *p_face_color1, *p_face_color2;
	CKvVectorInt idx_pnts_total;
	CKvMatrixInt buffer;

	CKvPoint3D p3d_org, p3d_new;
 
 	pmat.c_Create(1);	pmat_norm.c_Create(1);
 	pmat.gpe_Get_Pointer_of_Element(0)->cp_Copy(in_p_mat);
 
 	zz_util_dosf.tsp_Transform_Set_of_P_matrices(
 		&pmat,
 		in_homo_norm,
 		&pmat_norm);

	// set.............
	//docube.sr_Set_Resolution(KV_CUBE_DIM, KV_CUBE_DIM, KV_CUBE_DIM);
	//docube.gp_rp_Get_Pointer_of_Reference_and_Direction_Vectors()->cp_Copy(in_ref_points_and_direc_vecs);
	zz_docube.i_Import(in_docube_segments, NULL);
	// set.............
	
	// Render depth map.
	//LCKvYooji_Scanner_Display display;
	//CKvScreen sc;

	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// 0-th element: triangles
	// 1-th element: quadrangles.
	mesh_gen.mm_Make_Meshes(&zz_docube, &depot_p3d, &depot_rgba, &mesh_tri);
	mesh_gen.gimt_Get_Indices_of_Mesh_with_Tetragon(&mesh_tri, 0, &idx_pnts_total);
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

	// Transform point clouds to original coordinates.
	for(int i=0; i<depot_p3d.ne_Number_of_Elements(); i++){
		depot_p3d.ge_Get_Element(i, p3d_org);

		in_homo_norm->tp_Transform_Point(false, p3d_org, p3d_new);

		depot_p3d.se_Set_Element(i, p3d_new);
	}

	// =========================================================================
	// Buffer and depth map arrays should be created before putting it into function.
	buffer.c_Create(io_map_depth->mh(), io_map_depth->mw(), -1);
	// =========================================================================
	util_3do.gdmzbi_Generate_Depth_Map_using_Z_buffering_for_an_Image(
		&depot_p3d,
		&idx_pnts_total,
		in_p_mat,
		io_map_depth,
		&buffer);

// 	display.cidfu_Convert_Image_Depth_Float_to_Uchar(
// 		io_map_depth, 
// 		KV_CONVERT_IMAGE_DEPTH_FL_TO_DISP_UC,
// 		0.2f,
// 		2.0f,
// 		&img_depth);
	//sc.s_d_Display(&img_depth);

	//if(!Kv_Printf("Depth!"))	exit(0);

	// Render 3D model.
	if(io_graph_3d){
	
		util_display.pmo_Plot_Mesh_Object(io_graph_3d, &depot_p3d, &depot_rgba, &mesh_tri, NULL);
	
	//if(!Kv_Printf("[mm_Make_Meshes] num pnts: %d", depot_p3d.ne_Number_of_Elements()))	exit(0);
	}
	
}


//********************************************************************************************
void LCKvYooji_Shape_from_Silhouette::rdvh_Render_Depth_from_Visual_Hull(
	CKvPmatrix3D *in_p_mat,
	CKvMatrixFloat *io_map_depth)
//********************************************************************************************
{
	Util_VCL_3DO util_3do;
	CKvMatrixInt buffer;
	
	// =========================================================================
	// Buffer and depth map arrays should be created before putting it into function.
	buffer.c_Create(io_map_depth->mh(), io_map_depth->mw(), -1);
	// =========================================================================
	util_3do.gdmzbi_Generate_Depth_Map_using_Z_buffering_for_an_Image(
		&zz_depot_p3d,
		&zz_idx_pnts_total,
		in_p_mat,
		io_map_depth,
		&buffer);
	
}


//********************************************************************************************
void LCKvYooji_Shape_from_Silhouette::z_cor_Compute_Object_Rims(
	vector<CKvYooji_MatrixRgbD> *in_set_of_mat_rgbd,
	vector<CKvPmatrix3D> *in_set_of_p_matrices,
	CKvYooji_Cube_TSDF_Float *io_cube)
//********************************************************************************************
{
	// ========================================================================
	// Initialization
	// ========================================================================
	CKvSet_of_MatrixBool set_of_sils;
	CKvSet_of_Pmatrix3D set_of_pmats;
	CKvSet_of_SdkCode set_of_codes;

	CKvRunSet rs;

	int sz;

	sz = (*in_set_of_mat_rgbd).size();

	set_of_sils.c_Create(sz);
	set_of_pmats.c_Create(sz);
	set_of_codes.c_Create(sz);

	// Reference index is 1. (0, 1, 2).
	for(int i=0; i<sz; i++){
		// Silhouettes.
		set_of_sils.gpe(i)->cp_Copy((*in_set_of_mat_rgbd)[i].p_image_silhouette());
		// P matrices.
		set_of_pmats.gpe_Get_Pointer_of_Element(i)->cp_Copy(&(*in_set_of_p_matrices)[i]);
		// Silhouettes -> SdkCodes.
		rs.i_Import(set_of_sils.gpe(i));	(*set_of_codes.gpe_Get_Pointer_of_Element(i)).i_Import(&rs, true);
	}
	
	// ========================================================================
	// Ray carving.
	// ========================================================================
	LCKvUtility_for_DoSurface util_ds;
	CKvDoBoundaryShort dbs;

	CKvSet_of_VectorFloat jets;
	CKvMatrixFloat rpdv;

	dbs.mrd_Make_set_of_Reference_points_and_Direction_vectors(
		set_of_pmats.gpe_Get_Pointer_of_Element(1),
		set_of_codes.gpe_Get_Pointer_of_Element(1),
		&rpdv,
		&jets);
	
	/// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	/// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	drc_Do_Ray_Carving(
		&rpdv,
		&set_of_pmats,
		&set_of_codes,
		&jets);
	/// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	/// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

	// ========================================================================
	// Display.
	// ========================================================================
 	CKvScreen sc;

	sc.s_d_Display((*in_set_of_mat_rgbd)[1].p_image_silhouette());
	if(!Kv_Printf("object size: %d jet num: %d", rs.nr_Number_of_Runs(), jets.vs()))	exit(0);

	for(int i=0; i<rpdv.mh(); i++){
		if(!Kv_Printf("[%d/%d] %f %f %f", 
			i,
			rpdv.mh(),
			rpdv.mp()[i][0], 
			rpdv.mp()[i][1],
			rpdv.mp()[i][2]))	exit(0);
	}
}

//********************************************************************************
bool LCKvYooji_Shape_from_Silhouette::z_drc_Do_Ray_Carving(
	CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
	CKvPmatrix3D *in_set_of_P_matrices,
	CKvSdkCode *in_set_of_silhouette_images,
	int in_number_of_images,
	CKvVectorFloat *out_set_of_depth_jets)
//********************************************************************************
{
	// <assumption> 
	// (1) -1.0 <= depth <= 1.0
	// (2) rp dot dv = 0
	// (3) mag(dv) = 1
	// (4) mag(rp) < 1 : rp is inside the unit sphere
	CKvSdkCode *sdk;
	CKvPmatrix3D *Pmat; 
	CKvVectorFloat *DOC;
	CKvMatrixFloat *rp_dv; 
	int k,nb_img,nb_ray;;

	//Initialization
	in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix->ms(k,nb_ray); 
	nb_img= in_number_of_images; 
	rp_dv = in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix;
	DOC   = out_set_of_depth_jets;
	sdk   = in_set_of_silhouette_images;
	Pmat  = in_set_of_P_matrices;
	
	//Do Ray Carving
	z_idj_Initialize_Depth_Jets(rp_dv,&Pmat[0],&sdk[0],DOC);	
	for(k=1;k<nb_img;k++){z_udj_Update_Depth_Jets(rp_dv,&Pmat[k],&sdk[k],DOC);}
	return true;
}

//********************************************************************************
bool LCKvYooji_Shape_from_Silhouette::z_drci_Do_Ray_Carving_Incremental(
		CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
		CKvPmatrix3D *in_P_matrix,
		CKvSdkCode *in_silhouette_image,
		CKvVectorFloat *out_set_of_depth_jets,
		bool in_flag_is_first_frame)
//********************************************************************************
{
	// <assumption> 
	// (1) -1.0 <= depth <= 1.0
	// (2) rp dot dv = 0
	// (3) mag(dv) = 1
	// (4) mag(rp) < 1 : rp is inside the unit sphere
	CKvSdkCode *sdk;
	CKvPmatrix3D *Pmat; 
	CKvVectorFloat *DOC;
	CKvMatrixFloat *rp_dv; 
	int k,nb_img,nb_ray;;

	//Initialization
	in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix->ms(k,nb_ray); 
	rp_dv = in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix;
	DOC   = out_set_of_depth_jets;
	Pmat = in_P_matrix;
	sdk = in_silhouette_image;

	//Do Ray Carving
	if(in_flag_is_first_frame)	z_idj_Initialize_Depth_Jets(rp_dv,Pmat,sdk,DOC);	
	else						z_udj_Update_Depth_Jets(rp_dv,Pmat,sdk,DOC);
	return true;
}

//********************************************************************************
bool LCKvYooji_Shape_from_Silhouette::z_idj_Initialize_Depth_Jets(
	CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
	CKvPmatrix3D *in_P_matrix,
	CKvSdkCode *in_silhouette_image,
	CKvVectorFloat *out_set_of_depth_jets)
//********************************************************************************
{
	// <assumption> 
	// (1) -1.0 <= depth <= 1.0
	// (2) rp dot dv = 0
	// (3) mag(dv) = 1
	// (4) mag(rp) < 1 : rp is inside the unit sphere
	static CKvMatrix Hmat,Hmat_inv; 
	static float p[12],h[4],g[4],vanish[2];

	CKvSet_of_MatrixInt *mat_set;
	CKvVectorFloat *DOC; 
	double *q;
	bool valid,zero_direction_vector_flag,exist_vp,eq_mode;
	float **RD,*rp,*dv,*D,fip[2],fidv[2],zvp,vmin,vmax,tmp; 
	int N,ww,hh,i,k,nc;

	//Initialization
	in_silhouette_image->ms(ww,hh); 
	RD      = in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix->mps(i,N);
	DOC     = out_set_of_depth_jets;
	mat_set = in_silhouette_image->gpc_Get_Pointer_of_modified_Chain_code();
	nc      = in_silhouette_image->nc_Get_Number_of_Contours();
	q       = in_P_matrix->mpp()->mp()[0]; 
	for(k=0;k<12;k++) p[k] = (float)q[k];
	eq_mode =!in_silhouette_image->nh_Get_Neighborhood();
	
	//Exception handling
	if(i!=6){ Kv_Printf("i!=6"); exit(0);}
	
	for(i=0;i<N;i++){

		if(inj_Is_NULL_Jet(&DOC[i])) continue;
		if(DOC[i].vs()<2)	DOC[i].c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER);

		//////////////////////////////////////////////////////////////////////////
// 		DOC[i].vp()[0] = -1.0f;
// 		DOC[i].vp()[1] = 1.0f;
		//////////////////////////////////////////////////////////////////////////

  		rp  = RD[i]; 
  		dv  = rp+3;		
  		D   = DOC[i].vp();
  		tmp = 0.0f; 
  		for(k=0;k<3;k++) tmp+=(rp[k]*rp[k]); 
  
  		//printf("%f %f %f\n", rp[0], rp[1], rp[2]);
  		//////////////////////////////////////////////////////////////////////////
  		// 여기서 스킵 당함 ㅅㅂ?
  		// 호모그래피 스케일이 이상한 것 같기도 함...
  		if(tmp>=3.0f) continue;
  		// 여기서 스킵 당함 ㅅㅂ?
   
    		vmin = -1.0f; 
    		vmax = 1.0f;
  
   		z_pr_Project_a_Ray(
   			ww,hh, // int in_width_of_image,int in_height_of_image,
   			rp, // float *in_reference_3d_point_of_a_ray,
   			dv, // float *in_unit_direction_3d_vector_of_a_ray,
   			p, // float *in_P_matrix_3x4_in_row_first_mode,
   			fip, // float *out_image_2d_point_of_a_visible_point_in_3D_world,
   			fidv, // float *out_direction_2d_vector_on_image_plane,
   			vanish,
   			zvp, // float &out_coordinate_of_vanishing_2d_point,
   			g, // float *out_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode,
   			h, // float *out_H_matrix_2x2_from_3_dim_to_2_dim_in_row_first_mode,
   			valid, // bool &out_existence_of_a_visible_3d_point_on_the_ray,
   			vmin, // float &io_lower_limit_of_visible_zone,
   			vmax, // float &io_upper_limit_of_visible_zone,
   			exist_vp, // bool &out_existence_of_vanishing_2d_point,
   			zero_direction_vector_flag); // bool &out_flag_indicating_direction_2d_vector_is_zero)
   
   		if(!valid) continue;
   		
   		//printf("%f %f / %f %f  \n", fip[0], fip[1], fidv[0], fidv[1]);	system("pause");
   
   
   		D[0] = vmin; 
   		D[1] = vmax; 
   
   		if(!z_ss_Shave_Segments(
   			nc,       // int in_number_of_contours,
   			mat_set,  // CKvSet_of_MatrixInt *in_set_of_Nx2_matrix_x_y_OR_Nx3_matrix_x_y_c,
   			fip,      // float *in_origin_2d_point_of_a_projected_ray,
   			fidv,     // float *in_unit_direction_2d_vector_of_a_projected_ray,
   			g,        // float *in_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode,
   			exist_vp, // bool in_existence_of_vanishing_2d_point,
   			zvp,      // float in_coordinate_of_vanishing_2d_point,
   			zero_direction_vector_flag,
   			eq_mode,  // bool in_equal_distance_mode,
   			&DOC[i],  // CKvVectorFloat *io_depth_jet,
   			valid)    // bool &out_validity)
  		){
  			Kv_Printf("z_ss_Shave_Segments");
  			exit(0);
  		}

// 		DOC[i].c_Create(2);
// 		DOC[i].vp()[0] = -1.0f;
// 		DOC[i].vp()[1] = 1.0f;

		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// if there is no cross point, remove this segment from object cube.
// 		if(!valid){
// 
// 		}
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//  		if(valid){
// 			if(DOC[i].vp()[0] < -1.0 || DOC[i].vp()[0] > 1.0 
// 				|| DOC[i].vp()[1] < -1.0 || DOC[i].vp()[1] > 1.0 )
//  				if(!Kv_Printf("%d] %f %f / %f\n", DOC[i].vs(), DOC[i].vp()[0], DOC[i].vp()[1], rp[0])) exit(0);
//  			//system("pause");
//  		}
	}
	return true;
}

//********************************************************************************
bool LCKvYooji_Shape_from_Silhouette::z_udj_Update_Depth_Jets(
	CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
	CKvPmatrix3D *in_P_matrix,
	CKvSdkCode *in_silhouette_image,
	CKvVectorFloat *io_set_of_depth_jets)
//********************************************************************************
{
	// <assumption> 
	// (1) -1.0 <= depth <= 1.0
	// (2) rp dot dv = 0
	// (3) mag(dv) = 1
	// (4) mag(rp) < 1 : rp is inside the unit sphere

	static CKvVector list;
	static float p[12],h[4],g[4],vanish[2];

	CKvSet_of_MatrixInt *sil_set;
	CKvVectorFloat *DOC;

	bool valid,zero_direction_vector_flag,exist_vp,eq_mode;
	double *q;
	float **RD,*rp,*dv,*D,fip[2],fidv[2],zvp,vmin,vmax; 
	int N,i,k,ww,hh,sz,nc;

	//Initialization
	in_silhouette_image->ms(ww,hh); 
	RD       = in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix->mps(i,N);
	sil_set = in_silhouette_image->gpc_Get_Pointer_of_modified_Chain_code();
	nc       = in_silhouette_image->nc_Get_Number_of_Contours();
	DOC      = io_set_of_depth_jets;
	q        = in_P_matrix->mpp()->mp()[0]; 
	for(k=0;k<12;k++) p[k]=(float)q[k];
	eq_mode  =!in_silhouette_image->nh_Get_Neighborhood();

	//Exception handling
	if(i!=6){Kv_Printf("i!=6"); exit(0);}


	for(i=0;i<N;i++){
		rp = RD[i]; 
		dv = rp+3;
		D  = DOC[i].vps(sz); 

		if(sz%2){ Kv_Printf("sz%2"); exit(0);}

		if(inj_Is_NULL_Jet(&DOC[i])) continue;
		
		vmin =-1.0f; 
		vmax = 1.0f;

		z_pr_Project_a_Ray(
			ww,hh, // int in_width_of_image,int in_height_of_image,
			rp, // float *in_reference_3d_point_of_a_ray,
			dv, // float *in_unit_direction_3d_vector_of_a_ray,
			p, // float *in_P_matrix_3x4_in_row_first_mode,
			fip, // float *out_image_2d_point_of_a_visible_point_in_3D_world,
			fidv, // float *out_direction_2d_vector_on_image_plane,
			vanish,
			zvp, // float &out_coordinate_of_vanishing_2d_point,
			g, // float *out_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode,
			h, // float *out_H_matrix_2x2_from_3_dim_to_2_dim_in_row_first_mode,
			valid, // bool &out_existence_of_a_visible_3d_point_on_the_ray,
			vmin, // float &out_lower_limit_of_visible_zone,
			vmax, // float &out_upper_limit_of_visible_zone,
			exist_vp, // bool &out_existence_of_vanishing_2d_point,
			zero_direction_vector_flag); // bool &out_flag_indicating_direction_2d_vector_is_zero)

		if(!valid) { DOC[i].c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER); continue; }

		if(!z_ss_Shave_Segments(
			nc, // int in_number_of_contours,
			sil_set, // CKvSet_of_MatrixInt *in_set_of_Nx2_matrix_x_y_OR_Nx3_matrix_x_y_c,
			fip, // float *in_origin_2d_point_of_a_projected_ray,
			fidv, // float *in_unit_direction_2d_vector_of_a_projected_ray,
			g, // float *in_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode,
			exist_vp, // bool in_existence_of_vanishing_2d_point,
			zvp, // float in_coordinate_of_vanishing_2d_point,
			zero_direction_vector_flag,
			eq_mode, //	bool in_equal_distance_mode,
			&DOC[i], // CKvVectorFloat *io_set_of_3d_segment_coordinates,
			valid) // bool &out_validity)
			){
				Kv_Printf("z_ss_Shave_Segments");
				exit(0);
		}
	}
	
	return true;
}

//*****************************************************************************************
void LCKvYooji_Shape_from_Silhouette::z_pr_Project_a_Ray(
	int in_width_of_image,
	int in_height_of_image,
	float *in_reference_3d_point_of_a_ray,
	float *in_unit_direction_3d_vector_of_a_ray,
	float *in_P_matrix_3x4_in_row_first_mode,
	float *out_image_2d_point_of_a_visible_point_in_3D_world,
	float *out_unit_direction_2d_vector_on_image_plane,
	float *out_vanishing_2d_point,
	float &out_coordinate_of_vanishing_2d_point,
	float *out_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode,
	float *out_H_matrix_2x2_from_3_dim_to_2_dim_in_row_first_mode,
	bool &out_existence_of_a_visible_3d_point_on_the_ray,
	float &io_lower_limit_of_visible_zone,
	float &io_upper_limit_of_visible_zone,
	bool &out_existence_of_vanishing_2d_point,
	bool &out_flag_indicating_direction_2d_vector_is_zero)
//*****************************************************************************************
{
	//
	// <assumption> 
	// (1) -1.0 <= depth <= 1.0
	// (2) rp dot dv = 0
	// (3) mag(dv) = 1
	// (4) mag(rp) < 1 : rp is inside the unit sphere
	//
	int k; float wp[3],*h,*g,zvis,*rp,*dv;

	rp=in_reference_3d_point_of_a_ray;
	dv=in_unit_direction_3d_vector_of_a_ray;

	g=out_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode;
	h=out_H_matrix_2x2_from_3_dim_to_2_dim_in_row_first_mode;

	z_fvz_Find_a_Visible_Zone(
		in_width_of_image,in_height_of_image,
		rp,dv,in_P_matrix_3x4_in_row_first_mode,
		io_lower_limit_of_visible_zone,
		io_upper_limit_of_visible_zone,
		out_existence_of_a_visible_3d_point_on_the_ray);

	if(!out_existence_of_a_visible_3d_point_on_the_ray) return;

//  	printf("visible zone: %f %f\n", io_lower_limit_of_visible_zone, io_upper_limit_of_visible_zone);
//  	system("pause");

	if((io_lower_limit_of_visible_zone*io_upper_limit_of_visible_zone)<=0.0f) zvis=0.0f;
	else zvis=(io_lower_limit_of_visible_zone+io_upper_limit_of_visible_zone)*0.5f; 
	
	for(k=0;k<3;k++) wp[k]=rp[k]+dv[k]*zvis;

	z_prv_Project_a_Ray_defined_by_a_Visible_reference_point(
		wp, // float *in_a_3d_point_visible_by_this_P_matrix,
		dv, // float *in_unit_direction_3d_vector_of_a_ray,
		in_P_matrix_3x4_in_row_first_mode,
		out_image_2d_point_of_a_visible_point_in_3D_world,
		out_unit_direction_2d_vector_on_image_plane,
		out_vanishing_2d_point,
		out_coordinate_of_vanishing_2d_point,
		g,//float *out_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode,
		h,//float *out_H_matrix_2x2_from_3_dim_to_2_dim_in_row_first_mode,
		out_existence_of_vanishing_2d_point,
		out_flag_indicating_direction_2d_vector_is_zero);

	h[1] = -h[0]*zvis; h[3] -= (h[2]*zvis);
	g[0] += (g[2]*zvis); g[1] = g[3]*zvis; 

	return;
}

//*****************************************************************************************
bool LCKvYooji_Shape_from_Silhouette::z_ss_Shave_Segments(
	int in_number_of_contours,
	CKvSet_of_MatrixInt *in_set_of_Nx2_matrix_x_y_OR_Nx3_matrix_x_y_c,
	float *in_origin_2d_point_of_a_projected_ray,
	float *in_unit_direction_2d_vector_of_a_projected_ray,
	float *in_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode,
	bool in_existence_of_vanishing_2d_point,
	float in_coordinate_of_vanishing_2d_point,
	bool in_flag_indicating_direction_2d_vector_is_zero,
	bool in_equal_distance_mode,
	CKvVectorFloat *io_depth_jet,
	bool &out_validity)
//*****************************************************************************************
{
	// assumption : 
	// (1) io_depth_jet is valid and not NULL jet. 
	// (2) valid segments exist between -1 and 1
	static CKvVectorFloat vec,tvec,*DOC; 
	float zmin,zmax,*a,*b,*c,v1,v2,zvp,*g,xx,tmp; 
	int i,j,k,np,np_old,np_new;
	int start,last;

	//Initialization
	DOC = io_depth_jet;
	g   = in_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode;
	zvp = in_coordinate_of_vanishing_2d_point;

	if(!z_fsb_Find_Segments_carved_by_Boundaries(
		in_number_of_contours,
		in_set_of_Nx2_matrix_x_y_OR_Nx3_matrix_x_y_c,
		in_origin_2d_point_of_a_projected_ray,
		in_unit_direction_2d_vector_of_a_projected_ray,
		in_equal_distance_mode,
		&vec,out_validity)){Kv_Printf("[z_ss_Shave_Segments] 1"); exit(0);} 

	if(!out_validity){ DOC->c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER); return true;}

	a   = vec.vps(np);	// this vector value is 1-dimensional distance from origin of ray segment in 1-D ray segment coordinates.

// 	printf("%f %f / %f %f\n", in_origin_2d_point_of_a_projected_ray[0], in_origin_2d_point_of_a_projected_ray[1],
// 		in_unit_direction_2d_vector_of_a_projected_ray[0], in_unit_direction_2d_vector_of_a_projected_ray[1]);
// 	printf("%f %f / %f %f\n", in_origin_2d_point_of_a_projected_ray[0]+a[0]*in_unit_direction_2d_vector_of_a_projected_ray[0]
// 	, in_origin_2d_point_of_a_projected_ray[1]+a[0]*in_unit_direction_2d_vector_of_a_projected_ray[1]
// 	, in_origin_2d_point_of_a_projected_ray[0]+a[1]*in_unit_direction_2d_vector_of_a_projected_ray[0]
// 	, in_origin_2d_point_of_a_projected_ray[1]+a[1]*in_unit_direction_2d_vector_of_a_projected_ray[1]);
// 		system("pause");

	if(!in_existence_of_vanishing_2d_point){ //affine mode
		for(k=0;k<np;k++){
			xx   = a[k]; 
			tmp  =  g[2]*xx+g[3];
			a[k] = (g[0]*xx+g[1])/tmp;
			if(a[k]>1.0f) {a[k]=1.0f; break; }
		}
		last = k;
		if(last%2) last++; 
		for(;k>=0;k--){	if(a[k]<-1.0f) {a[k]=-1.0f; break; }}
		start=k; 
		if((start%2)) start++;
	}
	else{ //projective mode
		//In case of Null jet
		if(a[0]==Kv2008_DO_SURFACE_INVALID_MARKER){
			DOC->c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER); 
			out_validity = false;
			return true;
		}
		//3차원 ray가 점으로 투영되는 경우, 해당 점이 실루엣 안에 있을경우, skip, 그렇지 않을 경우 null jet으로 만든다 
		if(zvp==0.0f){
			for(k=0;k<np;k++)  { if(a[k]>0.0f) break;}
			if((k>=np)||(k==0)){
				DOC->c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER); 
				out_validity=false;
				return true;
			}
			else{
				if((k%2==0)||(a[k-1]>=0.0f)){
					DOC->c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER); 
					out_validity=false;
				}
				return true;
			}
		} 
		if(zvp>0.0f){
			//Back projection of a projected line
			for(k=0;k<np;k++){
				if(a[k]<zvp){
					xx  = a[k];
					tmp = g[2]*xx+g[3];
					a[k]= (g[0]*xx+g[1])/tmp;
					if(a[k]>1.0f) { a[k]=1.0f; break; }
				}
				else { a[k]=1.0f; break; }
			}

			last = k; 
			if(last%2) last++; 			
			if(k==np) k--;
			for(;k>=0;k--){	if(a[k]<-1.0f) { a[k]=-1.0f; break;}}
			start=k; 
			if((start%2)) start++;
		}
		else{
			//Back projection of a projected line
			for(k=np-1;k>=0;k--){
				if(a[k]>zvp){
					xx  = a[k]; 
					tmp = g[2]*xx+g[3];
					a[k]= (g[0]*xx+g[1])/tmp; 
					if(a[k]<=-1.0f) { a[k]=-1.0f; break; }
				}
				else { a[k]=-1.0f; break; }
			}
			start = k; 
			if(start%2) start++; 
			if(k<0) k++;			
			for(;k<np;k++){if(a[k]>1.0f) { a[k]=1.0f; break; }}
			last=k; 
			if((last%2)) last++;			
		}
	}
	if(start>=last){
		DOC->c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER); 
		out_validity=false;
		return true;
	}
	
	tvec.cp_Copy(DOC); 
	b = tvec.vps(np_old); 
	a = vec.vps(np);
	
	for(np_new=i=0;i<np_old;i+=2){
		zmin=b[i]; 
		zmax=b[i+1];
		for(j=start;j<last;j+=2){ 
			if(a[j+1]<=zmin) continue;
			if(a[j]  >=zmax) break;
			v1 = max(a[j],zmin);
			v2 = min(a[j+1],zmax);
			if(v1<=v2) np_new++; 
		}
	}
	if(np_new<=0){ 
		out_validity= false; 
		b           = DOC->c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER); 
		return true;
	}

	(np_new<<=1);

	c = DOC->c_Create(np_new);

	for(k=i=0;i<np_old;i+=2){
		zmin = b[i]; 
		zmax = b[i+1];
		for(j=start;j<last;j+=2){ 
			if(a[j+1]<= zmin) continue;
			if(a[j]  >= zmax) break;
			v1 = max(a[j],zmin);
			v2 = min(a[j+1],zmax);
			if(v1<=v2){
				c[k++]=v1;
				c[k++]=v2;
			}
		}
	}

	out_validity = true;
	return true;
}

//*****************************************************************************************
void LCKvYooji_Shape_from_Silhouette::z_fvz_Find_a_Visible_Zone(
	int in_width_of_image,
	int in_height_of_image,
	float *in_reference_3d_point_of_a_ray,
	float *in_unit_direction_3d_vector_of_a_ray,
	float *in_P_matrix_3x4_in_row_first_mode,
	float &io_lower_limit_of_visible_zone,
	float &io_upper_limit_of_visible_zone,
	bool &out_existence_of_visible_3d_point)
//*****************************************************************************************
{
	//
	// <assumption> 
	// (1) -1.0 <= depth <= 1.0
	// (2) rp dot dv = 0
	// (3) mag(dv) = 1
	// (4) mag(rp) < 1 : rp is inside the unit sphere
	//
	float *p,*Xo,*dv,xo[3],vt[3],vp,vmin,vmax,tmp,eps;
	int i,k,sz[2]; 

	eps=(float)(1.0e-10); // tres important

	sz[0]=in_width_of_image-1; sz[1]=in_height_of_image-1;
	if((sz[0]<=0)||(sz[1]<=0)){
		Kv_Printf("[z_fvz_Find_a_Visible_Zone] image size is bad");
		exit(0);
	} 

	p=in_P_matrix_3x4_in_row_first_mode;
	Xo=in_reference_3d_point_of_a_ray;  
	dv=in_unit_direction_3d_vector_of_a_ray; 

	for(i=k=0;k<12;k+=4,i++)
	{
		xo[i] = (p[k]*Xo[0]) + (p[k+1]*Xo[1]) + (p[k+2]*Xo[2]) + (p[k+3]) ;
		vt[i] = (p[k]*dv[0]) + (p[k+1]*dv[1]) + (p[k+2]*dv[2]) ;
	}

	out_existence_of_visible_3d_point=true;

	vmin=io_lower_limit_of_visible_zone;
	vmax=io_upper_limit_of_visible_zone;

//	vmin=-1.0f; vmax=1.0f; // tres important 이것이 너무 크면 이상한 현상이 생김. 절대 조심할 것 

	if((p[8]!=0.0f)||(p[9]!=0.0f)||(p[10]!=0.0f)) // projective case
	{
		if(vt[2]==0.0f) 
		{
			if(xo[2]<=0.0f) out_existence_of_visible_3d_point=false;
			else out_existence_of_visible_3d_point=true;
			return;
		}
		else 
		{
			if(vt[2]>0.0f) 
			{
				vmin=max(vmin,(eps-xo[2]/vt[2])); 
				if(vmin>=vmax) { out_existence_of_visible_3d_point=false; return; }
			}
			else if(vt[2]<0.0f) 
			{
				vmax=min(vmax,(-xo[2]/vt[2]-eps)); 
				if(vmin>=vmax) { out_existence_of_visible_3d_point=false; return; }
			}

			for(k=0;k<2;k++)
			{
				if(vt[k]>0) 
				{
					vmin=max(vmin,-xo[k]/vt[k]);
					if(vmin>=vmax) { out_existence_of_visible_3d_point=false; return; }
				}
				else if(vt[k]<0.0f) 
				{
					vmax=min(vmax,-xo[k]/vt[k]);
					if(vmin>=vmax) { out_existence_of_visible_3d_point=false; return; }
				}
				else if(xo[k]<0.0f) 
				{
					out_existence_of_visible_3d_point=false;
					return;
				}

				vp=(vt[k]-sz[k]*vt[2]); tmp=(sz[k]*xo[2]-xo[k]); 
				if(vp>0.0f) 
				{
					vmax=min(vmax,tmp/vp); 
					if(vmin>=vmax) { out_existence_of_visible_3d_point=false; return; }
				}
				else if(vp<0.0f) 
				{
					vmin=max(vmin,tmp/vp);
					if(vmin>=vmax) { out_existence_of_visible_3d_point=false; return; }
				}
				else if(tmp<0.0f) 
				{
					out_existence_of_visible_3d_point=false;
					return;
				}
			}
		}
	}
	else // affine case
	{
		if(p[11]<=0.0){
			Kv_Printf("[z_fvz_Find_a_Visible_Zone] Bad P matrix");
			exit(0);
		} 
		for(k=0;k<2;k++)
		{
			if(vt[k]>0.0f) 
			{
				vmin=max(vmin,-xo[k]/vt[k]);
				vmax=min(vmax,(sz[k]*xo[2]-xo[k])/vt[k]);
				if(vmin>=vmax) { out_existence_of_visible_3d_point=false; return; }
			}
			else if(vt[k]<0.0f ) 
			{
				vmin=max(vmin,(sz[k]*xo[2]-xo[k])/vt[k]);
				vmax=min(vmax,-xo[k]/vt[k]);
				if(vmin>=vmax) { out_existence_of_visible_3d_point=false; return; }
			}
			else if((xo[k]<0.0f)||((sz[k]*xo[2]-xo[k])<0.0f))
			{
				out_existence_of_visible_3d_point=false;
				return;
			}
		}
	}
	io_lower_limit_of_visible_zone=vmin;
	io_upper_limit_of_visible_zone=vmax;

	return;

}

//*****************************************************************************************
void LCKvYooji_Shape_from_Silhouette::z_prv_Project_a_Ray_defined_by_a_Visible_reference_point(
	float *in_a_3d_point_visible_by_this_P_matrix,
	float *in_unit_direction_3d_vector_of_a_ray,
	float *in_P_matrix_3x4_in_row_first_mode,
	float *out_corresponding_2d_point_on_image_plane,
	float *out_unit_direction_2d_vector_on_image_plane,
	float *out_vanishing_2d_point,
	float &out_coordinate_of_vanishing_2d_point,
	float *out_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode,
	float *out_H_matrix_2x2_from_3_dim_to_2_dim_in_row_first_mode,
	bool &out_existence_of_vanishing_2d_point,
	bool &out_flag_indicating_direction_2d_vector_is_zero)
//*****************************************************************************************
{
	int i,k; float *h,*g,*p,*Xo,*dv,xo[3],vt[3],*vd,vd_mag;

	p=in_P_matrix_3x4_in_row_first_mode;
	Xo=in_a_3d_point_visible_by_this_P_matrix;
	dv=in_unit_direction_3d_vector_of_a_ray;
	g=out_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode;
	h=out_H_matrix_2x2_from_3_dim_to_2_dim_in_row_first_mode;
	vd=out_unit_direction_2d_vector_on_image_plane;

	for(i=k=0;k<12;i++,k+=4) 
	{
		xo[i] = (p[k]*Xo[0]) + (p[k+1]*Xo[1]) + (p[k+2]*Xo[2]) + (p[k+3]);
		vt[i] = (p[k]*dv[0]) + (p[k+1]*dv[1]) + (p[k+2]*dv[2]);
	}

	if(xo[2]<=0.0f){
		Kv_Printf("the visible point is not visible");
		exit(0);
	}

	out_corresponding_2d_point_on_image_plane[0]=xo[0]/xo[2];
	out_corresponding_2d_point_on_image_plane[1]=xo[1]/xo[2];

	vd[0]=vt[0]*xo[2]-vt[2]*xo[0]; 
	vd[1]=vt[1]*xo[2]-vt[2]*xo[1];

	vd_mag=(float)sqrt(vd[0]*vd[0]+vd[1]*vd[1]);

	h[0]=g[3]=vd_mag; h[1]=g[1]=0.0; 
	h[2]=xo[2]*vt[2]; h[3]=g[0]=xo[2]*xo[2];

	g[2]=-h[2];

	out_flag_indicating_direction_2d_vector_is_zero=false;
	out_existence_of_vanishing_2d_point=true;


	if(vd_mag!=0.0f)		 /// Checking ----- It's OK.		// if(vd_mag<0.0001f) //				
	{
		vd[0]/=vd_mag; vd[1]/=vd_mag;			/// Checking ----- It's OK.

		if(vt[2]!=0.0f)												/// Checking ----- It's OK.
		{		
			out_coordinate_of_vanishing_2d_point=vd_mag/h[2];
			out_vanishing_2d_point[0]=vt[0]/vt[2]; 
			out_vanishing_2d_point[1]=vt[1]/vt[2];
		}
		else
		{
			out_existence_of_vanishing_2d_point=false;
			out_coordinate_of_vanishing_2d_point=
				out_vanishing_2d_point[0]=
				out_vanishing_2d_point[1]=-100000.0f;
		}
	}
	else 
	{
		out_flag_indicating_direction_2d_vector_is_zero=true;
		vd[0]=1.0f; vd[1]=0.0f;

		out_coordinate_of_vanishing_2d_point=0.0f;
		out_vanishing_2d_point[0]=out_corresponding_2d_point_on_image_plane[0];
		out_vanishing_2d_point[1]=out_corresponding_2d_point_on_image_plane[1];
	}

	return;
}

//*****************************************************************************************
bool LCKvYooji_Shape_from_Silhouette::z_fsb_Find_Segments_carved_by_Boundaries(
	int in_number_of_contours,
	CKvSet_of_MatrixInt *in_set_of_Nx2_matrix_x_y_OR_Nx3_matrix_x_y_c,
	float *in_origin_2d_point_of_a_projected_ray,
	float *in_unit_direction_2d_vector_of_a_projected_ray,
	bool in_equal_distance_mode,
	CKvVectorFloat *out_set_of_segment_coordinates,
	bool &out_validity)
//*****************************************************************************************
{
	static CKvVectorInt LUT;
	static CKvVectorFloat Avec,TAvec; 
	static CKvVectorBool Cvec,TCvec; 
	static CKvRanker rank;

	int x1,y1,x2,y2,ax,ay,np,sz,kk,k,i,j,**m,*L,jump,f707; 
	float *B,vx,vy,xo,yo,x1o,y1o,r[2],muo,gamma,denom,lambda;
	float *A; bool *T,*TT,valid;
	double eps;
	CKvMatrixInt *mat;

	//When chain code does not exist
	if(in_number_of_contours<=0){out_set_of_segment_coordinates->c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER);return true;}

	//Initialization
	f707         = (in_equal_distance_mode==true) ? 707 : 1000;
	out_validity = false; 
	eps          = 1.0e-5; // f707=0.707f; 
	mat          = in_set_of_Nx2_matrix_x_y_OR_Nx3_matrix_x_y_c->vps(kk);

	vx           = in_unit_direction_2d_vector_of_a_projected_ray[0];
	vy           = in_unit_direction_2d_vector_of_a_projected_ray[1];
	xo           = in_origin_2d_point_of_a_projected_ray[0];
	yo           = in_origin_2d_point_of_a_projected_ray[1];
	muo          = xo*vy-vx*yo;
	
	sz           = in_number_of_contours*16; 
	A            = Avec.c_Create(sz); 
	T            = Cvec.c_Create(sz);
		
	//Find_Segments_carved_by_Boundaries
	kk= 0;
	for(k=0;k<in_number_of_contours;k++){
		m = mat[k].mps(i,np); 
		if(np<=2) continue;

		//j=np-1로 시작하여 r[0]가 eps보다 크게되는 위치까지 감소
		for(j=np-1;j>0;j--){
			x1 = m[j][0]; y1 = m[j][1];
			if(fabs(r[0]=(x1*vy-y1*vx)-muo)>eps) break;
		}
		if(j==0) continue; 
		else np=j+1; // last pt to process

		for(i=1;i<np;){
			x2 = m[i][0]; y2 = m[i][1];
			if(fabs(r[1]=(x2*vy-y2*vx)-muo)<=eps) r[1]=0.0f;  
			//gamma가 음수이면 projected ray와의 교차가 일어나고 그렇지 않은 경우 양수임
			//gamma가 0인 경우에 대한 handling 필요함 
			gamma = r[0]*r[1]; 
			valid = false;

			if(gamma>0.0f){//양수일 경우 jumping을 수행함 
				jump = (abs((int)r[1])*707/f707)-1;
				if(jump >=1){//수직거리가 1이상일 경우
					i+=(jump); 
					if(i<np){
						x1=m[i][0]; y1=m[i][1]; 
						i++;
						r[0]=(x1*vy-y1*vx)-muo;   
					}
				}
				else { i++; x1=x2; y1=y2; r[0]=r[1]; }//수직 거리가 1 이하일 경우 한 step 전진
			}
			else{//음수거나 0 일 경우 
				x1o = x1-xo; 
				y1o = y1-yo;

				if(gamma<0.0f){//음수인 경우 원점으로부터 거리 계산. 거리는 lamda에 저장됨 
					ax     = x2-x1; ay     = y2-y1; //두개의 chain code로부터 구성되는 segment의 방향 벡터
					denom  =   ax*vy-ay*vx;
					lambda = -(ay*x1o-ax*y1o);
					lambda/=   denom;
					valid  = true;
				}
				else{ //0인 경우, r[0]는 0일 수 없으므로, r[1] == 0.0임
					//j=i+1로 시작하여 r[1]가 eps보다 크게되는 위치까지 증가
					for(j=i+1;j<np;j++){
						x2 = m[j][0]; 
						y2 = m[j][1];
						if(fabs(r[1]=(x2*vy-y2*vx)-muo)>eps) break;   
					}
					//한 바퀴 돌때가지 없으면 에러 
					//if(j==np){ Kv_Printf("[z_fsb_Find_Segments_carved_by_Boundaries] 3"); exit(0);} //For debugging 
					i = j;
					
					if((r[0]*r[1])<0.0f){ //gamma가 음수인 경우 원점으로부터 거리 다시 계산. 거리는 lamda에 저장됨
						ax     = x2-x1; ay     = y2-y1;
						denom  = ax*vy-ay*vx;
						lambda =-(ay*x1o-ax*y1o);
						lambda/=denom;
						valid=true;
					}
				}
				//그 다음 좌표로 한 step전진. 예전 값 복사 
				i++; x1=x2; y1=y2; r[0]=r[1];
			}

			if(valid){
				//A에 lamda값 저장 
				A[kk] = lambda; 
				//chain code로 구성된 segment와 projected ray의 cross product의 부호를 T에 저장함 
				if(denom>0) T[kk] = true; 
				else        T[kk] = false;
				kk++; 
				//애초에 잡아놓은 메모리보다 더 큰 메모리가 필요한 경우 
				if(kk>=sz){
					sz*=2;
					TAvec.cp_Copy(&Avec); 
					TCvec.cp_Copy(&Cvec);
					A=Avec.c_Create(sz); Avec.s_Set(0,&TAvec);
					T=Cvec.c_Create(sz); Cvec.s_Set(0,&TCvec);
				}
			}
		}
	}
	if(kk<=0) return true;
	//if(kk%2){ Kv_Printf("[z_fsb_Find_Segments_carved_by_Boundaries] 4"); exit(0);} //For debugging  

	//nrqs_Non_Recursive_Quick_Sort(&Avec, sz, 0, kk-1);//nrqs_Non_Recursive_Quick_Sort(Avec, sz_Avec, 0, kk-1);	//s_Sort(Avec,sz_Avec,false,0,kk,LUT);		//	
	rank.s_Sort(&Avec,false,0,kk,&LUT); 
	//T에 저장된 chain code로 구성된 segment와 projected ray의 cross product의 부호를 통한 Validity test

	L  = LUT.vp();
	TT = TCvec.c_Create(kk); 
	for(k=0;k<kk;k++) TT[k]=T[L[k]];
	T  = TT; 
	np = kk-1;

	if(T[np-1]==T[np]) valid=true;
	else if(fabs(A[np-1]-A[np])>eps) valid=true;
	else valid=false;

	//교차점 간의 거리가 일정 크기 이하일 경우 merge
	for(i=k=0;k<np;){
		if(fabs(A[k]-A[k+1])>eps){A[i] = A[k]; T[i] = T[k]; i++; k++;}
		else{
			//if(T[k]==T[k+1]){ Kv_Printf("[z_fsb_Find_Segments_carved_by_Boundaries] 5"); exit(0);} //For debugging    
			k  += 2; 
			kk -= 2;
		}
	}
	if(valid){ A[i]=A[k]; T[i]=T[k]; i++; k++; }//?

	if(kk<=0) return true;

	//최종 결과는 B에 저장됨. 교차점의 갯수는 kk개임 
	A = Avec.vp();
	B = out_set_of_segment_coordinates->c_Create(kk);
	for(k=0;k<kk;k++) B[k]=A[k];
	
// 	printf("%f %f\n", B[0], B[1]);
// 	system("pause");

	out_validity = true;

	return true;
}

/////



//**************************************************
void LCKvYooji_Shape_from_Silhouette::z_gmtdm_Get_Mesh_of_Triangles_from_a_Depth_Map(
		CKvMatrixFloat *in_depth_map,
		CKvPmatrix3D *in_pmat_depth,
		CKvSet_of_Point3D &out_set_of_p3d_in_world,
		CKvMatrixInt &out_mesh_indices)
//**************************************************
{	
	CKvYooji_Intrinsics intrins_depth;

	CKvVectorInt v_index;
	CKvMatrixInt map_index_new;
	CKvSet_of_Point3Df set_p3d;
	CKvPoint3Df *p_set_p3d;

	CKvPointf p2d_rgb, p2d_depth;
	CKvPixel pix;

	float *p_depth;
	int *p_map_index_new;

	int *v, *p_indices;
	float pt[3];

	int m, k0, k1, k2, k3, kk0, kk1, kk2, kk3, k0_rgb;
	int num_reduced;
	int numPoint, numTriangle, numTriangle_valid = 0;

	CKvString *pstr;
	int ww_d, hh_d, widthStep;
	int x, y;
	float depth_meter, scale;

	// Gets input rgb and depth images.
	p_depth = (*in_depth_map).mps(ww_d, hh_d)[0];

	// Gets camera information.
	intrins_depth.set_from_P_matrix(in_pmat_depth);

	// Sets index map.
	p_map_index_new = map_index_new.c_Create(hh_d, ww_d, 0)[0];

	numPoint = ww_d*hh_d;
	numTriangle = 2*(ww_d-1)*(hh_d-1);

	// make depot_of_point3D.	
	k0 = num_reduced = 0;
	p_set_p3d = set_p3d.c_Create(hh_d*ww_d);
	for(int j = 0; j<hh_d; j++){
		for(int i = 0; i<ww_d; i++, k0++){

			CKvPoint3Df *p3d = &p_set_p3d[k0];
			float td = p_depth[k0];

			// check valid depth.
			if(j==hh_d-1 || i==ww_d-1 || td <= 0.0f)
			{
				p3d->x = p3d->y = p3d->z = 0.0f;
				p_depth[k0] = 0.0f;

				// make new index map.
				p_map_index_new[k0] = -1;

				num_reduced++;
				//p_x[i] = p_y[i] = 0.0f; 
			}
			else
			{
				pix.x = i;
				pix.y = j;

				intrins_depth.back_project(pix, td, *p3d);

				// make new index map.
				p_map_index_new[k0] = k0 - num_reduced;
			}
		}
	}
	// make point and color depot.	
	out_set_of_p3d_in_world.c_Create(numPoint - num_reduced);

	// append 3D points and colors and check number of valid triangles.
	CKvPoint3D *p_p3d;
	float td1, td2, td3, td0;
	float th_depth = 1000.0f;//MAX_DEPTH_DIFF_FOR_MESH_TRIANGLE_MM_UNIT;

	m = k0 = numTriangle_valid = 0;

	for(int j = 0; j<hh_d; j++){
		for(int i = 0; i<ww_d; i++, k0++){

			if(p_map_index_new[k0] < 0){
				//if(j==hh-1 || i==ww-1 || p_depth[k0] <= 0.0f){	
				//	num_reduced++;
				continue;
			}

			// Append current 3D point to set of point 3D.
			p_p3d = out_set_of_p3d_in_world.gpe_Get_Pointer_of_Element(m++);
			
			p_p3d->x = p_set_p3d[k0].x;		p_p3d->y = p_set_p3d[k0].y;		p_p3d->z = p_set_p3d[k0].z;
			
			// count valid triangle.
			k1 = k0 + 1; k2 = k0 + ww_d; k3 = k2 + 1;

			// for depth points.
			td0 = p_depth[k0];
			td1 = p_depth[k1];
			td2 = p_depth[k2];
			td3 = p_depth[k3];

			if(p_map_index_new[k1] >= 0 && p_map_index_new[k2] >= 0){

				if(abs(td0 - td1) < th_depth
					&& abs(td0 - td2) < th_depth
					&& abs(td1 - td2) < th_depth){
					numTriangle_valid++;
				}

				if(p_map_index_new[k3] >= 0
					&& abs(td3 - td1) < th_depth
					&& abs(td3 - td2) < th_depth
					&& abs(td1 - td2) < th_depth){
					numTriangle_valid++;
				}

			}

		}
	}

	// make point indices.
	v = v_index.c_Create(3*numTriangle_valid);

	// make mesh triangles.
	k0 = m = 0;
	for(int j = 0; j<hh_d; j++){
		for(int i = 0; i<ww_d; i++, k0++){

			if(p_map_index_new[k0] < 0){
				//if(j==hh-1 || i==ww-1 || p_set_p3d[k0].z <= 0.0f){
				//num_reduced++;
				continue;
			}
			
			k1 = k0 + 1; k2 = k0 + ww_d; k3 = k2 + 1;

			// for depth points.
			td0 = p_depth[k0];
			td1 = p_depth[k1];
			td2 = p_depth[k2];
			td3 = p_depth[k3];

			// for reduced point index.
			kk0 = p_map_index_new[k0];
			kk1 = p_map_index_new[k1];
			kk2 = p_map_index_new[k2];
			kk3 = p_map_index_new[k3];

			if(kk1 >= 0 && kk2 >= 0){
				// first triangle.
				if(abs(td0 - td1) < th_depth
					&& abs(td0 - td2) < th_depth
					&& abs(td1 - td2) < th_depth){
					v[m++] = kk0;		v[m++] = kk1;		v[m++] = kk2;
				}

				// second triangle.
				if(kk3 >= 0
					&& abs(td3 - td1) < th_depth
					&& abs(td3 - td2) < th_depth
					&& abs(td1 - td2) < th_depth){
					v[m++] = kk2;		v[m++] = kk1;		v[m++] = kk3;
				}

			}
		}
	}

	//Transforms 3D points to the world coordinates.
	CKvYooji_Extrinsics ext_depth;
	CKvPoint3D *p_set_of_p3d;
	CKvPoint3Df p3d_cam, p3d_world; 
	 	 
	p_set_of_p3d = out_set_of_p3d_in_world.vp();
	ext_depth.set_from_P_matrix(in_pmat_depth);	 
	
	for(int k=0; k<out_set_of_p3d_in_world.vs(); k++){
	 
	 	p3d_cam.x = (float)p_set_of_p3d[k].x;
	 	p3d_cam.y = (float)p_set_of_p3d[k].y;
	 	p3d_cam.z = (float)p_set_of_p3d[k].z;
	 
	 	ext_depth.transform_inv(p3d_cam, p3d_world);
	 
	 	p_set_of_p3d[k].x = p3d_world.x;
	 	p_set_of_p3d[k].y = p3d_world.y;
	 	p_set_of_p3d[k].z = p3d_world.z;	 
	}

	// Save computed mesh indices.
	p_indices = out_mesh_indices.c_Create(3, numTriangle_valid)[0];
	for(int j=0; j<3; j++){
		for(int i=0; i<numTriangle_valid; i++){
			p_indices[numTriangle_valid*j + i] = v[3*i + j];
// 			if(!Kv_Printf("%d] %f %f %f", p_indices[numTriangle_valid*j + i]
// 				, out_set_of_p3d.gpe_Get_Pointer_of_Element(p_indices[numTriangle_valid*j + i])->x
// 				, out_set_of_p3d.gpe_Get_Pointer_of_Element(p_indices[numTriangle_valid*j + i])->y
// 				, out_set_of_p3d.gpe_Get_Pointer_of_Element(p_indices[numTriangle_valid*j + i])->z))	
// 				exit(0);
		}
	}

}


//**************************************************
// 1. Compute 3D positions of each depth pixel.
// 2. Get connectivity of 3D points from neighbor inform. on 2D image coordinates.
// 3. Validity check using normal of mesh triangle.
void LCKvYooji_Shape_from_Silhouette::gmtdm_Get_Mesh_of_Triangles_from_a_Depth_Map_New(
	CKvMatrixFloat *in_depth_map,
	CKvPmatrix3D *in_pmat_depth,
	CKvSet_of_Point3D &out_set_of_p3d_in_world,
	CKvMatrixInt &out_mesh_indices,
	int &out_number_of_valid_p3ds,
	int &out_number_of_valid_mesh_tris)
//**************************************************
{	
	CKvYooji_Intrinsics intrins_depth;

// 	CKvVectorInt zz_v_index;
// 	CKvMatrixInt zz_map_index_new;
//	CKvSet_of_Point3Df zz_set_p3d;
	CKvPoint3D *p_set_p3d;

	CKvPointf p2d_rgb, p2d_depth;
	CKvPixel pix;

	float *p_depth;
	int *p_map_index_new;

	int *p_indices;
	float pt[3];

	int m, k0, k1, k2, k3, kk0, kk1, kk2, kk3, k0_rgb;
	int num_reduced;
	int numPoint, numTriangle, numTriangle_valid = 0, numPoint_valid = 0;

	CKvString *pstr;
	int ww_d, hh_d, widthStep;
	int x, y;
	float depth_meter, scale;

	// Gets input rgb and depth images.
	p_depth = (*in_depth_map).mps(ww_d, hh_d)[0];

	// Gets camera information.
	intrins_depth.set_from_P_matrix(in_pmat_depth);

	// Sets index map.
	if(zz_map_index_new.mw() != ww_d || zz_map_index_new.mh() != hh_d)	
		p_map_index_new = zz_map_index_new.c_Create(hh_d, ww_d, 0)[0];
	else{
		p_map_index_new = zz_map_index_new.vp();
		for(int i=0; i<ww_d*hh_d; i++) p_map_index_new[i] = 0;
	}

	// Set depot of point 3D and mesh triangles.
	numPoint = ww_d*hh_d;
	numTriangle = 2*(ww_d-1)*(hh_d-1);

	if(zz_set_p3d.vs() != numPoint)	zz_set_p3d.c_Create(numPoint);		
	if(out_set_of_p3d_in_world.vs() != numPoint) out_set_of_p3d_in_world.c_Create(numPoint);
	if(out_mesh_indices.vs() != numTriangle){
		out_mesh_indices.c_Create(3, numTriangle);
		zz_v_index.c_Create(3*numTriangle);
	}
	
	// 이 부분에서 num_reduced 때문에 index가 꼬이는 것 같음...
	// 이 부분에서 num_reduced 때문에 index가 꼬이는 것 같음...
	// 이 부분에서 num_reduced 때문에 index가 꼬이는 것 같음...
	// ===========================================================
	// 1. Compute 3D positions of each depth pixel.
	// ===========================================================
	// make depot_of_point3D.	
	CKvPoint3Df tp3d;
	k0 = num_reduced = 0;
	p_set_p3d = out_set_of_p3d_in_world.vp();
	//p_set_p3d = zz_set_p3d.vp();
	//p_set_p3d = zz_set_p3d.c_Create(hh_d*ww_d);
	for(int j = 0; j<hh_d; j++){
		for(int i = 0; i<ww_d; i++, k0++){

			//CKvPoint3D *p3d = &p_set_p3d[k0];
			float td = (float)p_depth[k0];

			// check valid depth.
			if(j==hh_d-1 || i==ww_d-1 || td <= 0.0f)
			{
				//p3d->x = p3d->y = p3d->z = 0.0f;
				p_depth[k0] = (unsigned short)0;

				// make new index map.
				p_map_index_new[k0] = -1;

				num_reduced++;
				//p_x[i] = p_y[i] = 0.0f; 
			}
			else
			{
				pix.x = i;
				pix.y = j;

				intrins_depth.back_project(pix, td, tp3d);

				// update output point set.
				p_set_p3d[k0].x = tp3d.x;
				p_set_p3d[k0].y = tp3d.y;
				p_set_p3d[k0].z = tp3d.z;

				// make new index map.
				p_map_index_new[k0] = k0;

				//printf("p_map_index_new[k0]: %d\n", p_map_index_new[k0]);
			}
		}
	}
	// make point and color depot.	
	numPoint_valid = numPoint - num_reduced;
	//out_set_of_p3d_in_world.c_Create(numPoint - num_reduced);

	// 이 부분에서 num_reduced 때문에 index가 꼬이는 것 같음...
	// 이 부분에서 num_reduced 때문에 index가 꼬이는 것 같음...
	// 이 부분에서 num_reduced 때문에 index가 꼬이는 것 같음...

	// ================================================================================
	// 2. Get connectivity of 3D points from neighbor inform. on 2D image coordinates.
	// ================================================================================
	// append 3D points and colors and check number of valid triangles.
	CKvPoint3D *p_p3d[4];
	int **mv;
	float v1[3], v2[3], v3[3], nv[3];
	int idx[4];
	float td1, td2, td3, td0;
	float th_depth = MAX_DEPTH_DIFF_FOR_MESH_TRIANGLE_M_UNIT;
	//float th_depth = MAX_DEPTH_DIFF_FOR_MESH_TRIANGLE_MM_UNIT;
	float th_normal = 60.0f;

	// Get pointer of point indices.
	mv = out_mesh_indices.mp();

	m = k0 = numTriangle_valid = 0;

	for(int j = 0; j<hh_d; j++){
		for(int i = 0; i<ww_d; i++, k0++){
		
			// count valid triangle.
			k1 = k0 + 1; k2 = k0 + ww_d; k3 = k2 + 1;

			// Get point indices.
			idx[0] = p_map_index_new[k0];
			idx[1] = p_map_index_new[k1];
			idx[2] = p_map_index_new[k2];
			idx[3] = p_map_index_new[k3];				

			/////////////////////////////////////////////////////////////////
			
			// Append valid meshes to output mesh set.

			// 이 부분 mesh normal 을 체크해서 validity 판별하는 것으로 바꾸자.
			// 이 부분 mesh normal 을 체크해서 validity 판별하는 것으로 바꾸자.
			// 이 부분 mesh normal 을 체크해서 validity 판별하는 것으로 바꾸자.

			if(idx[1] >= 0 && idx[2] >= 0){

				// Get 3D points.
				p_p3d[1] = out_set_of_p3d_in_world.gpe_Get_Pointer_of_Element(k1);
				p_p3d[2] = out_set_of_p3d_in_world.gpe_Get_Pointer_of_Element(k2);

				// for 1st triangle.
				if(idx[0] >= 0){
					// Get 3D points.
					p_p3d[0] = out_set_of_p3d_in_world.gpe_Get_Pointer_of_Element(k0);
					// Check validity of mesh triangle. (0-1-2: v1(0-2) x v2(0-1))
					// v1
					v1[0] = p_p3d[2]->x - p_p3d[0]->x;
					v1[1] = p_p3d[2]->y - p_p3d[0]->y;
					v1[2] = p_p3d[2]->z - p_p3d[0]->z;
					// v2
					v2[0] = p_p3d[1]->x - p_p3d[0]->x;
					v2[1] = p_p3d[1]->y - p_p3d[0]->y;
					v2[2] = p_p3d[1]->z - p_p3d[0]->z;
					// normal vector.
					d_cpv3_Cross_Product_Vector3D(v1, v2, v3);
					d_nv_Normalize_Vector(v3, 3, nv);
					// Compare normal vector with -Z-axis.
					if(acos(-nv[2])*180.0f/PI < th_normal){
						mv[0][m] = idx[0];		mv[1][m] = idx[1];		mv[2][m] = idx[2];
						m++;
					} 
					//printf("nv %f %f %f\n", nv[0], nv[1], nv[2]);
				}
				// for 2nd triangle.
				if(idx[3] >= 0){
					// Get 3D points.
					p_p3d[3] = out_set_of_p3d_in_world.gpe_Get_Pointer_of_Element(k3);
					// Check validity of mesh triangle. (2-1-3: v1(3-1) x v2(3-2))
					// v1
					v1[0] = p_p3d[1]->x - p_p3d[3]->x;
					v1[1] = p_p3d[1]->y - p_p3d[3]->y;
					v1[2] = p_p3d[1]->z - p_p3d[3]->z;
					// v2
					v2[0] = p_p3d[2]->x - p_p3d[3]->x;
					v2[1] = p_p3d[2]->y - p_p3d[3]->y;
					v2[2] = p_p3d[2]->z - p_p3d[3]->z;
					// normal vector.
					d_cpv3_Cross_Product_Vector3D(v1,v2,v3);
					d_nv_Normalize_Vector(v3, 3, nv);
					// Compare normal vector with -Z-axis.
					if(acos(-nv[2])*180.0f/PI < th_normal){
						mv[0][m] = idx[2];		mv[1][m] = idx[1];		mv[2][m] = idx[3];
						m++;
					}
					//printf("nv %f %f %f\n", nv[0], nv[1], nv[2]);
					
				}

			}

			/////////////////////////////////////////////////////////////////

		}
	}
	numTriangle_valid = m;


	//Transforms 3D points to the world coordinates.
	CKvYooji_Extrinsics ext_depth;
	CKvPoint3D *p_set_of_p3d;
	CKvPoint3Df p3d_cam, p3d_world; 
	 	 
	p_set_of_p3d = out_set_of_p3d_in_world.vp();
	ext_depth.set_from_P_matrix(in_pmat_depth);	 
	
	for(int k=0; k<numPoint; k++){
	 
		if(p_map_index_new[k] < 0) continue;

	 	p3d_cam.x = (float)p_set_of_p3d[k].x;
	 	p3d_cam.y = (float)p_set_of_p3d[k].y;
	 	p3d_cam.z = (float)p_set_of_p3d[k].z;
	 
	 	ext_depth.transform_inv(p3d_cam, p3d_world);
	 
	 	p_set_of_p3d[k].x = p3d_world.x;
	 	p_set_of_p3d[k].y = p3d_world.y;
	 	p_set_of_p3d[k].z = p3d_world.z;	 
	}

	out_number_of_valid_p3ds = numPoint_valid;
	out_number_of_valid_mesh_tris = numTriangle_valid;

	printf("numPoint_valid: %d numTriangle_valid: %d\n", numPoint_valid, numTriangle_valid);

}

//**************************************************
void LCKvYooji_Shape_from_Silhouette::z_sord_Select_Optimal_Ray_Direction(
		CKvPmatrix3D *in_pmat,
		int &out_docube_axis)
//**************************************************
{
	CKvPoint3D prin_axis;
	prin_axis = in_pmat->dvo_Direction_Vector_of_Optical_axis();

	float mval = -1000000.0f;
	if(abs(prin_axis.x) > mval){ mval = abs(prin_axis.x); out_docube_axis = 0; }
	if(abs(prin_axis.y) > mval){ mval = abs(prin_axis.y); out_docube_axis = 1; }
	if(abs(prin_axis.z) > mval){ mval = abs(prin_axis.z); out_docube_axis = 2; }
}

// Refinement using depth image.

//**************************************************
void LCKvYooji_Shape_from_Silhouette::z_crddc_Convert_Ray_Direction_of_DoCube(
		VCL_DoCube &in_docube,
		int in_ray_direction,
		VCL_DoCube &out_docube)
//**************************************************
{

	CKvRunSet run_slice_zx; CKvMatrixBool img_slice_zx;
	CKvRunSet run_slice_zy; CKvMatrixBool img_slice_zy;
	CKvMatrixInt RRR; CKvVectorInt YYY;

	int mode;

	// access i-th slice of initial DoCube.
	for(int i=0; i<256; i++){

		// case 1: ray_direc = 0 (Z=real X, X=real Y, [Y=real Z])
		// -> get YZ plane. (in real world, ZX plane)
		if(in_ray_direction == 0){
			// get ZY plane. (in real world, XZ plane)
			in_docube.gszy_Get_a_Slice_parallel_to_ZY_plane(i,mode,&run_slice_zy);
			run_slice_zy.e_Export(true,false,&zz_slice_t);
			// transpose ZY plane to YZ plane.
			zz_slice.cp_Copy(&zz_slice_t);
			d_tm_Transpose_Matrix(zz_slice_t.vp(),256,256,zz_slice.vp());
		}
		// case 2: ray_direc = 1 (Z=real Y, [X=real Z], Y=real X)
		// -> get XZ plane. (in real world, ZY plane)
		else if(in_ray_direction == 1){
			// get ZX plane. (in real world, YZ plane)
			in_docube.gszx_Get_a_Slice_parallel_to_ZX_plane(i,mode,&run_slice_zx);
			run_slice_zx.e_Export(true,false,&zz_slice_t);
			// transpose ZX plane to XZ plane.
			zz_slice.cp_Copy(&zz_slice_t);
			d_tm_Transpose_Matrix(zz_slice_t.vp(),256,256,zz_slice.vp());
		}
		// case 3: ray_direc = 2 ([Z=real Z axis], X=X, Y=Y)
		// -> no change.
		else if(in_ray_direction == 2){
			//continue;
			// 				zz_DoCube_init.gszy_Get_a_Slice_parallel_to_ZY_plane(i,mode,&run_slice_zy);
			// 				run_slice_zy.e_Export(true,false,&zz_slice_t);
			// 				zz_slice.cp_Copy(&zz_slice);
			// 				i_tm_Transpose_Matrix(zz_slice_t.vp(),256,256,zz_slice.vp());
		}

		// Get runset of slice image.
		zz_set_run.i_Import(&zz_slice);
		// + RRR: pointer of start and end X position of runs.
		// + YYY: pointer of last index of run in each column.
		zz_set_run.e_Export(mode,mode,&RRR,&YYY);

		// 				sc[0].s_d_Display(&zz_slice);
		// 				if(!Kv_Printf("Slice!"))	exit(0);

		int num_run_totalx2,num_run_total,num_run;
		int y_prev,y_curr;
		int *p_RRR = RRR.mps(num_run_total,mode)[0]; // 2xN
		int *p_YYY = YYY.vp(); // Hx1
		short *p_seg = NULL;

		// set new DoCube segments of i-th slice.
		y_prev = -1;
		for(int y=0; y<256; y++){

			y_curr = p_YYY[y];
			num_run = y_curr - y_prev;

			//printf("[%d] num_run: %d\n", y, num_run);

			// Runset image is ZX plane.
			if(in_ray_direction == 0){	// y: X axis, i: Y axis, p_seg: Z axis

				p_seg = zz_set_of_rays.gpe(y,i)->c_Create(2*num_run);
			}
			// Runset image is ZY plane.
			else if(in_ray_direction == 1){ // i: X axis, y: y axis, p_seg: Z axis
				p_seg = zz_set_of_rays.gpe(i,y)->c_Create(2*num_run);
			}

			for(int k=0; k<num_run; k++){
				p_seg[2*k] = p_RRR[(k + y_prev+1)];
				p_seg[2*k + 1] = p_RRR[(k + y_prev+1) + num_run_total];
			}

			//if(!Kv_Printf("[%d] %d", y, p_YYY[y]))	exit(0);

			y_prev = y_curr;
		}
	}

	// copy.
	out_docube.i_Import(&zz_set_of_rays);
}

//******************************************************************************
void LCKvYooji_Shape_from_Silhouette::z_ursitdc_Update_Ray_Segments_based_on_Intersection_of_Two_DoCubes(
	VCL_DoCube &in_docube_ref,
	VCL_DoCube &io_docube_target)
//******************************************************************************
{
	CKvSet2d_of_VectorShort *p_docube_ref, *p_docube_target;
	CKvVectorBool ray_ref_voxelized, ray_target_voxelized;
	
	bool *p_rrv, *p_rtv;
	int dim_x, dim_y, dim_z;

	p_docube_ref = in_docube_ref.gp_Get_Pointer();
	p_docube_target = io_docube_target.gp_Get_Pointer();
		
	in_docube_ref.gr_Get_Resolution(dim_x, dim_y, dim_z);

	p_rrv = ray_ref_voxelized.c_Create(dim_z, false);
	p_rtv = ray_target_voxelized.c_Create(dim_z, false);

	for(int j=0; j<dim_y; j++){
		for(int i=0; i<dim_x; i++){

			CKvVectorShort *vp_ray_ref, *vp_ray_target;
			short *p_rr, *p_rt;
			int num_sr, num_st, idx_start, idx_end;
			
			vp_ray_ref = p_docube_ref->gpe(i, j);
			vp_ray_target = p_docube_target->gpe(i, j);

			// Check which current ray is null jet or not.
			if(inj_Is_NULL_Jet(vp_ray_target))	continue;
			// If reference ray is null, remove current ray of target cube.
 			if(inj_Is_NULL_Jet(vp_ray_ref)){
				vp_ray_target->c_Create(2, Kv2008_DO_SURFACE_INVALID_MARKER);		
				continue;
			} 			

			p_rr = vp_ray_ref->vps(num_sr);
			p_rt = vp_ray_target->vps(num_st);
			
			// Voxelize current rays.
			// + for initialization.
			for(int kk=0; kk<dim_z; kk++)	p_rrv[kk] = p_rtv[kk] = false;
			// + for reference ray.		
			//for(int kk=0; kk<dim_z; kk++)	p_rrv[kk] = true;
			for(int k = 0; k<num_sr; k += 2){
				idx_start = p_rr[k];	idx_end = p_rr[k+1];
				for(int kk = idx_start; kk<=idx_end; kk++)	p_rrv[kk] = true;
			}
			// + for target ray.
			//for(int kk=0; kk<dim_z; kk++)	p_rtv[kk] = true;
 			for(int k = 0; k<num_st; k += 2){
 				idx_start = p_rt[k];	idx_end = p_rt[k+1];
 				//if(!Kv_Printf("%d %d", idx_start, idx_end))	exit(0);
 				for(int kk = idx_start; kk<=idx_end; kk++)	p_rtv[kk] = true;
 			}

			// Find intersection between two rays.
			int num_surf_pnts, cnt;
			bool flag_prev;

			num_surf_pnts = 0;	flag_prev = false;
			for(int kk=0; kk<dim_z; kk++){
				
				// And operation between two rays.
				//p_rtv[kk] = p_rrv[kk];
				p_rtv[kk] = p_rrv[kk] & p_rtv[kk];		
				// Count number of surface points.
				if(p_rtv[kk] != flag_prev)	num_surf_pnts++;
				flag_prev = p_rtv[kk];
			}
			// + Add end points
			if(flag_prev)	num_surf_pnts++;

			//printf("num_st: %d %d (%d, %d)\n", num_st, num_surf_pnts, i, j);

			if(num_surf_pnts <=0){
				vp_ray_target->c_Create(2, NOPOINTVALDoCube);
				continue;
			}

			// Update target ray. 			
 			p_rt = vp_ray_target->c_Create(num_surf_pnts);

 			cnt = 0;	flag_prev = false;
 			for(int kk=0; kk<dim_z; kk++){
 
 				// Count number of surface points.
 				if(p_rtv[kk] != flag_prev){

					if(flag_prev)	p_rt[cnt++] = (short)(kk-1);
					else			p_rt[cnt++] = (short)kk;
 				}
 
 				flag_prev = p_rtv[kk];
 			}
			// + Add end points
			if(flag_prev)	p_rt[cnt++] = (short)(dim_z-1);
			

		}
	}


//	in_docube_ref.i_Import(p_docube_target);

}