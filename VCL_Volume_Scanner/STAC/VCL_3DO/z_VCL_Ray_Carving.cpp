/////////////////////////////////////////////////////////////////////////////////////////////
// z_VCL_Ray_Carving.cpp
/////////////////////////////////////////////////////////////////////////////////////////////

#include "../../stdafx.h"
// #include "VCL_Inhand_Object_Scanning.h"
// #include "VCL_Inhand_Object_ScanningDlg.h"
//#include "_VCL_Ray_Carving.h"
#include "../_VCL_2014_3DO.h"

VCL_Ray_Carving::VCL_Ray_Carving()
{
}
VCL_Ray_Carving::~VCL_Ray_Carving()
{
}
//********************************************************************************
//********************************************************************************
//********************************************************************************
//**********************Preparation for Ray Carving ******************************
//********************************************************************************
//********************************************************************************
void VCL_Ray_Carving::sp2d3dr_Set_Parameters_and_2Dim3Dim_Relations(	
	float in_scale,
	CKvSet_of_Pmatrix3D *in_p_mat,
	CKvSet_of_SdkCode *in_silhouette,
	float in_angle_x,
	float in_angle_y,
	float in_angle_z,
	CKvMatrixFloat *in_rpdv,
	CKvHmatrix3D *out_homo,
	CKvSet_of_Pmatrix3D *out_p_matrices_or_NULL)
//******************************************************************************
{
	CKvSet_of_Pmatrix3D normalized_p_mat;
	CKvSdkCode *sdk;
	int ww, hh;
	float pi180;		  
	//Initialization
	sdk   = in_silhouette->vp();
	pi180 = (float)PI/180.0f;
	sdk[0].ms(ww,hh);

	ngh_Normalize_into_unit_sphere_and_Get_Homography(
		in_scale,
		in_scale,
		in_scale,
		in_p_mat,
		in_silhouette,
		in_angle_x*pi180,
		in_angle_y*pi180,
		in_angle_z*pi180,
		out_homo,
		&normalized_p_mat);
	s2d3dr_Set_2dim_3dim_Relation(in_rpdv,ww,hh,&normalized_p_mat);
	if(out_p_matrices_or_NULL!=NULL) out_p_matrices_or_NULL->cp_Copy(&normalized_p_mat);
}
//******************************************************************************
void VCL_Ray_Carving::sp2d3dr_Set_Parameters_and_2Dim3Dim_Relations(	
	float in_scale,
	CKvSet_of_Pmatrix3D *in_p_mat,
	CKvSet_of_MatrixBool *in_silhouette,
	float in_angle_x,
	float in_angle_y,
	float in_angle_z,
	CKvMatrixFloat *in_rpdv,
	CKvHmatrix3D *out_homo,
	CKvSet_of_Pmatrix3D *out_p_matrices_or_NULL)
//******************************************************************************
{
	CKvSet_of_Pmatrix3D normalized_p_mat;
	CKvSet_of_SdkCode silhouette_sdkcode;
	CKvSdkCode *p_silhouette_sdkcode;
	CKvRunSet run;
	CKvMatrixBool *p_sil;
	int k,ww, hh,sz;
	float pi180;	

	//Initialization
	pi180				= (float)PI/180.0f;
	p_sil				=in_silhouette->vps(sz);
	p_silhouette_sdkcode=silhouette_sdkcode.c_Create(sz);
	for(k=0;k<sz;k++){run.i_Import(&p_sil[k]);p_silhouette_sdkcode[k].i_Import(&run, true);}	
	p_silhouette_sdkcode[0].ms(ww,hh);	

	ngh_Normalize_into_unit_sphere_and_Get_Homography(
		in_scale,
		in_scale,
		in_scale,
		in_p_mat,
		&silhouette_sdkcode,
		in_angle_x*pi180,
		in_angle_y*pi180,
		in_angle_z*pi180,
		out_homo,
		&normalized_p_mat);

	s2d3dr_Set_2dim_3dim_Relation(in_rpdv,ww,hh,&normalized_p_mat);
	if(out_p_matrices_or_NULL!=NULL) out_p_matrices_or_NULL->cp_Copy(&normalized_p_mat);
}
//******************************************************************************
void VCL_Ray_Carving::sp2d3dr_Set_Parameters_and_2Dim3Dim_Relations(	
	CKvSet_of_Pmatrix3D *in_p_mat,
	CKvSet_of_SdkCode *in_silhouette,
	CKvMatrixFloat *in_rpdv)
//******************************************************************************
{
	CKvSdkCode *sdk;
	int ww, hh;
	float pi180;		  
	//Initialization
	sdk   = in_silhouette->vp();
	pi180 = (float)PI/180.0f;
	sdk[0].ms(ww,hh);

	s2d3dr_Set_2dim_3dim_Relation(in_rpdv,ww,hh,in_p_mat);
}
//******************************************************************************
void VCL_Ray_Carving::ngh_Normalize_into_unit_sphere_and_Get_Homography(
	double in_scale_factor_X_axis,
	double in_scale_factor_Y_axis,
	double in_scale_factor_Z_axis,
	CKvSet_of_Pmatrix3D *in_set_of_P_matrices,
	CKvSet_of_SdkCode *in_set_of_silhouette_images,
	double in_angle_in_radian_of_rotation_around_X_axis,
	double in_angle_in_radian_of_rotation_around_Y_axis,
	double in_angle_in_radian_of_rotation_around_Z_axis,
	CKvHmatrix3D *out_homography_for_transforming_object,
	CKvSet_of_Pmatrix3D *out_set_of_P_matrix_for_normalized_object)
//******************************************************************************
{
	CKvHmatrix3D temp_homography;
	CKvMatrix THmat, Hmat;
	static CKvSet_of_Rect rec_set;
	CKvPoint3D cc;
	double R, SH[4][4], **TH, **HH;
	int i, j, k;
	//Estimate object information and normalize into unit sphere
	zz_udos.grs_Get_Rectangles_of_Silhouettes(in_set_of_silhouette_images,&rec_set);
	zz_udos.eoin_Estimate_Object_Information_and_Normalize_into_unit_sphere(
		in_set_of_P_matrices,
		&rec_set,
		in_angle_in_radian_of_rotation_around_X_axis,
		in_angle_in_radian_of_rotation_around_Y_axis,
		in_angle_in_radian_of_rotation_around_Z_axis,
		cc,
		R,
		&temp_homography);
	//-in_scale_factor*center.x
	SH[0][0]=in_scale_factor_X_axis;	SH[0][1]=0.0;						SH[0][2]=0.0;						SH[0][3]=0.0;
	SH[1][0]=0.0;						SH[1][1]=in_scale_factor_Y_axis;	SH[1][2]=0.0;						SH[1][3]=0.0;
	SH[2][0]=0.0;						SH[2][1]=0.0;						SH[2][2]=in_scale_factor_Z_axis;	SH[2][3]=0.0;
	SH[3][0]=0.0;						SH[3][1]=0.0;						SH[3][2]=0.0;						SH[3][3]=1.0;

	temp_homography.e_Export(&THmat);
	TH = THmat.mp();
	HH = Hmat.c_Create(4,4);

	for(j=0; j<4; j++){
		for(i=0; i<4; i++){
			HH[j][i] = 0; 
			for(k=0; k<4; k++){HH[j][i]+=(SH[j][k]*TH[k][i]);}
		}
	}
	out_homography_for_transforming_object->c_Create(&Hmat);
	zz_udos.tsp_Transform_Set_of_P_matrices(
		in_set_of_P_matrices,
		out_homography_for_transforming_object,
		out_set_of_P_matrix_for_normalized_object);
}
//********************************************************************************
void VCL_Ray_Carving::s2d3dr_Set_2dim_3dim_Relation(
	CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
	int in_width_of_image,
	int in_height_of_image,
	CKvSet_of_Pmatrix3D *in_P_matrices)
//********************************************************************************
{
	// fip:2 fidv:2 zvp:1 g:4 vmin:1 vmax:1
	// valid:1 exist_vp:1 zero_direction_vector_flag:1
	CKvPmatrix3D *Pmat;
	static float p[12],h[4],vanish[2];
	double *q;
	float **RD,*rp,*dv,tmp,**p_float_val, *fip, *fidv,*zvp,*g,*vmin, *vmax;
	float t_vmin,t_vmax;
	int nb_rays,nb_img,i,j,k;
	bool **p_bool_val,*valid,*exist_vp, *zero_direction_vector_flag;
	
	//Initialization		
	RD			= in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix->mps(i,nb_rays);
	Pmat        = in_P_matrices->vps(nb_img);
	p_float_val = zz_float_val_Nx11_matrix.c_Create(nb_rays*nb_img,11);
	p_bool_val  = zz_bool_val_Nx3_matrix  .c_Create(nb_rays*nb_img,3);
	
	//Set relation for each ray 
	for(j=0;j<nb_img;j++){
		q = Pmat[j].mpp()->mp()[0]; 
		for(k=0;k<12;k++)	p[k]=(float)q[k];			

		for(i=0;i<nb_rays;i++){
			rp						   = RD[i]; 
			dv						   = rp+3;
			fip					       = p_float_val[j*nb_rays+i];
			fidv					   = fip +2;
			zvp						   = fidv+2;
			g						   = zvp +1;
			vmin					   = g   +4;
			vmax				       = vmin+1;
			valid					   = p_bool_val[j*nb_rays+i];
			exist_vp				   = valid+1;
			zero_direction_vector_flag = exist_vp +1;

			t_vmin = -1.0f;
			t_vmax =  1.0f;

			//Ignore rays outside the unit sphere
			tmp=0.0f; 
			for(k=0;k<3;k++) tmp+=(rp[k]*rp[k]); 
			if(tmp>=1.0f){(*valid)=false; continue;}
	
			z_pr_Project_a_Ray(
				in_width_of_image,				// int in_width_of_image,
				in_height_of_image,				//int in_height_of_image,
				rp,								// float *in_reference_3d_point_of_a_ray,
				dv,								// float *in_unit_direction_3d_vector_of_a_ray,
				p,								// float *in_P_matrix_3x4_in_row_first_mode,
				fip,							// float *out_image_2d_point_of_a_visible_point_in_3D_world,
				fidv,							// float *out_direction_2d_vector_on_image_plane,
				vanish,
				(*zvp),							// float &out_coordinate_of_vanishing_2d_point,
				g,								// float *out_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode,
				h,								// float *out_H_matrix_2x2_from_3_dim_to_2_dim_in_row_first_mode,
				(*valid),						// bool &out_existence_of_a_visible_3d_point_on_the_ray,
				t_vmin,							// float &io_lower_limit_of_visible_zone,
				t_vmax,							// float &io_upper_limit_of_visible_zone,
				(*exist_vp),					// bool &out_existence_of_vanishing_2d_point,
				(*zero_direction_vector_flag)); // bool &out_flag_indicating_direction_2d_vector_is_zero)

			vmin[0] = t_vmin;		
			vmax[0] = t_vmax;				
		}
	}
}
//********************************************************************************
void VCL_Ray_Carving::idm_Initialize_Depth_Map(
	int in_width_of_depth_map,
	int in_height_of_depth_map)
//********************************************************************************
{
	zz_analog_map_2D.c_Create(in_height_of_depth_map,in_width_of_depth_map);
}
//********************************************************************************
CKvSet2d_of_VectorFloat* VCL_Ray_Carving::edm_Export_Depth_Map()
//********************************************************************************
{
	return &zz_analog_map_2D;
}
//********************************************************************************
//********************************************************************************
//********************************************************************************
//****************Ray Carving using Precomputed Ray Relation**********************
//********************************************************************************
//********************************************************************************
//********************************************************************************
//********************************************************************************
bool VCL_Ray_Carving::drc_Do_Ray_Carving_using_Precomputed_Ray_Relation(
	CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
	CKvSet_of_SdkCode *in_set_of_silhouette_images)
//********************************************************************************
{
	// <assumption> 
	// (1) -1.0 <= depth <= 1.0
	// (2) rp dot dv = 0
	// (3) mag(dv) = 1
	// (4) mag(rp) < 1 : rp is inside the unit sphere
	CKvVectorFloat *DOC; 
	CKvSdkCode *sdk;
	int nb_img; 

	//Initialization
	DOC	  = zz_analog_map_2D.mp()[0];
	sdk   = in_set_of_silhouette_images->vps(nb_img);
	
	//Do Ray Carving
	if(!z_drc_Do_Ray_Carving_using_Precomputed_Ray_Relation(in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,sdk,nb_img,DOC)) gerr("4");
	return true;
error:
	zsm("drc_Do_Ray_Carving");
	return false;
}
//********************************************************************************
bool VCL_Ray_Carving::z_drc_Do_Ray_Carving_using_Precomputed_Ray_Relation(
	CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
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
	CKvVectorFloat *DOC;
	CKvMatrixFloat *rp_dv; 
	int k,nb_img,nb_ray;;

	//Initialization
	in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix->ms(k,nb_ray); 
	nb_img= in_number_of_images; 
	rp_dv = in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix;
	DOC   = out_set_of_depth_jets;
	sdk   = in_set_of_silhouette_images;

	//Do Ray Carving
	z_idj_Initialize_Depth_Jets_using_Precomputed_Ray_Relation(rp_dv,&sdk[0],DOC);
	for(k=1;k<nb_img;k++){z_udj_Update_Depth_Jets_using_Precomputed_Ray_Relation(rp_dv,&sdk[k],k,DOC);}
	return true;
}
//********************************************************************************
bool VCL_Ray_Carving::z_idj_Initialize_Depth_Jets_using_Precomputed_Ray_Relation(
	CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
	CKvSdkCode *in_silhouette_image,
	CKvVectorFloat *out_set_of_depth_jets)
//********************************************************************************
{
	// <assumption> 
	// (1) -1.0 <= depth <= 1.0
	// (2) rp dot dv = 0
	// (3) mag(dv) = 1
	// (4) mag(rp) < 1 : rp is inside the unit sphere
	// (5) io_depth_jet is valid and not NULL jet. 
	// (6) valid segments exist between -1 and 1
	CKvSet_of_MatrixInt *mat_set;
	CKvVectorFloat *DOC; 
	bool valid,*valid_ray,*zero_direction_vector_flag,*exist_vp,eq_mode;
	float **RD,*rp,*dv,*D,*fip,*fidv,*zvp,*g,*vmin,*vmax,tmp; 
	float **p_float_val, *pp_float_val;
	bool  **p_bool_val,  *pp_bool_val;
	int N,ww,hh,i,k,nc;
	static CKvVectorFloat vec; 
	
	//Initialization
	in_silhouette_image->ms(ww,hh); 
	RD          = in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix->mps(i,N);
	DOC         = out_set_of_depth_jets;
	mat_set     = in_silhouette_image->gpc_Get_Pointer_of_modified_Chain_code();
	nc          = in_silhouette_image->nc_Get_Number_of_Contours();
	eq_mode     =!in_silhouette_image->nh_Get_Neighborhood();
	p_float_val = zz_float_val_Nx11_matrix.mp();
	p_bool_val  = zz_bool_val_Nx3_matrix.mp();

	for(i=0;i<N;i++){
		rp                         = RD[i]; 
		dv                         = rp+3;
		D                          = DOC[i].c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER);
		//Get results from Set_Rays
		pp_float_val               = p_float_val[i];
		pp_bool_val                = p_bool_val[i];
		fip					       = pp_float_val;
		fidv					   = fip +2;
		zvp						   = fidv+2;
		g						   = zvp +1;
		vmin					   = g   +4;
		vmax				       = vmin+1;
		valid_ray  			       = pp_bool_val;
		exist_vp				   = valid_ray+1;
		zero_direction_vector_flag = exist_vp +1;

		//Ignore rays outside the unit sphere
		tmp = 0.0f; 
		for(k=0;k<3;k++) tmp+=(rp[k]*rp[k]); 
		if(tmp>=1.0f){ 
			continue;
		}

		//Ignore invalid rays
		if(!(*valid_ray)) { 
			continue;
		}

		//Shave segments
		D[0] = (*vmin);	
		D[1] = (*vmax);
		
		if(!z_fsb_Find_Segments_carved_by_Boundaries(
			nc,
			mat_set,
			fip,
			fidv,
			eq_mode,
			&vec,
			valid)){Kv_Printf("[z_ss_Shave_Segments] 1"); exit(0);} 

		if(!valid){ 
			DOC[i].c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER); 
			continue;
		}

		bp_Back_Projection(
			&vec,
			(*exist_vp),
			g,
			&DOC[i],
			(*zvp),
			valid);
	}
	return true;
}
//********************************************************************************
bool VCL_Ray_Carving::z_udj_Update_Depth_Jets_using_Precomputed_Ray_Relation(
	CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
	CKvSdkCode *in_silhouette_image,
	int in_idx_silhouettes,
	CKvVectorFloat *io_set_of_depth_jets)
//********************************************************************************
{
	// <assumption> 
	// (1) -1.0 <= depth <= 1.0
	// (2) rp dot dv = 0
	// (3) mag(dv) = 1
	// (4) mag(rp) < 1 : rp is inside the unit sphere
	CKvSet_of_MatrixInt *pmat_set;
	CKvVectorFloat *DOC;

	bool valid,*valid_ray,*zero_direction_vector_flag,*exist_vp,eq_mode;
	float **RD,*rp,*dv,*D,*fip,*fidv,*zvp,*g,*vmin,*vmax; 
	float **p_float_val,*pp_float_val;
	bool  **p_bool_val, *pp_bool_val;
	int N,i,ww,hh,sz,nc;
	static CKvVectorFloat vec; 
	
	//Initialization
	in_silhouette_image->ms(ww,hh); 
	RD          = in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix->mps(i,N);
	pmat_set    = in_silhouette_image->gpc_Get_Pointer_of_modified_Chain_code();
	nc          = in_silhouette_image->nc_Get_Number_of_Contours();
	DOC         = io_set_of_depth_jets;
	eq_mode     =!in_silhouette_image->nh_Get_Neighborhood();
	p_float_val = zz_float_val_Nx11_matrix.mp();
	p_bool_val  = zz_bool_val_Nx3_matrix.mp();

	for(i=0;i<N;i++){
		rp = RD[i]; 
		dv = rp+3;
		D  = DOC[i].vps(sz); 
		//if ray is already null jet, skip
		if(inj_Is_NULL_Jet(&DOC[i])){
			continue;
		}

		//Get results from Set_Rays
		pp_float_val               = p_float_val[in_idx_silhouettes*N+i];
		pp_bool_val                = p_bool_val [in_idx_silhouettes*N+i];
		fip					       = pp_float_val;
		fidv					   = fip +2;
		zvp						   = fidv+2;
		g						   = zvp +1;
		vmin					   = g   +4;
		vmax				       = vmin+1;
		valid_ray  			       = pp_bool_val;
		exist_vp				   = valid_ray+1;
		zero_direction_vector_flag = exist_vp +1;

		//Ignore invalid rays
		if(!(*valid_ray)) { 
			DOC[i].c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER); 
			continue; 
		};

		if(!z_fsb_Find_Segments_carved_by_Boundaries(
			nc,
			pmat_set,
			fip,
			fidv,
			eq_mode,
			&vec,
			valid)){Kv_Printf("[z_ss_Shave_Segments] 1"); exit(0);} 

		if(!valid){ 
			DOC[i].c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER); 
			continue;
		}

		bp_Back_Projection(
			&vec,
			(*exist_vp),
			g,
			&DOC[i],
			(*zvp),
			valid);			
	}

	return true;
}
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////Original Version////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//********************************************************************************
bool VCL_Ray_Carving::drc_Do_Ray_Carving(
	CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
	CKvSet_of_Pmatrix3D *in_set_of_P_matrices,
	CKvSet_of_SdkCode *in_set_of_silhouette_images,
	CKvSet_of_VectorFloat *io_docube_segments)
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
bool VCL_Ray_Carving::drc_Do_Ray_Carving(
	CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
	int in_width_of_depth_map,
	int in_height_of_depth_map,
	CKvSet_of_Pmatrix3D *in_set_of_P_matrices,
	CKvSet_of_SdkCode *in_set_of_silhouette_images)
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
	DOC	  = zz_analog_map_2D.mp()[0];
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
bool VCL_Ray_Carving::z_drc_Do_Ray_Carving(
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
bool VCL_Ray_Carving::z_idj_Initialize_Depth_Jets(
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
		rp  = RD[i]; 
		dv  = rp+3;
		D   = DOC[i].c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER);
		tmp = 0.0f; 
		for(k=0;k<3;k++) tmp+=(rp[k]*rp[k]); 
		if(tmp>=1.0f) continue;

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

// 		if(valid){
// 			printf("%d] %f %f / %f\n", DOC[i].vs(), rp[0]+DOC[i].vp()[0], rp[0]+DOC[i].vp()[1],
// 				rp[0]);
// 			system("pause");
// 		}
	}
	return true;
}

//********************************************************************************
bool VCL_Ray_Carving::z_udj_Update_Depth_Jets(
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
void VCL_Ray_Carving::z_pr_Project_a_Ray(
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
void VCL_Ray_Carving::z_prv_Project_a_Ray_defined_by_a_Visible_reference_point(
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
bool VCL_Ray_Carving::z_ss_Shave_Segments_using_Precomputed_Ray_Relation(
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
	static CKvVectorFloat vec; 

	if(!z_fsb_Find_Segments_carved_by_Boundaries(
		in_number_of_contours,
		in_set_of_Nx2_matrix_x_y_OR_Nx3_matrix_x_y_c,
		in_origin_2d_point_of_a_projected_ray,
		in_unit_direction_2d_vector_of_a_projected_ray,
		in_equal_distance_mode,
		&vec,out_validity)){Kv_Printf("[z_ss_Shave_Segments] 1"); exit(0);} 

	if(!out_validity){ io_depth_jet->c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER); return true;}

	bp_Back_Projection(
		&vec,
		in_existence_of_vanishing_2d_point,
		in_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode,
		io_depth_jet,
		in_coordinate_of_vanishing_2d_point,
		out_validity);

	return true;
}

//*****************************************************************************************
bool VCL_Ray_Carving::z_ss_Shave_Segments(
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
bool VCL_Ray_Carving::z_fsb_Find_Segments_carved_by_Boundaries(
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
//*****************************************************************************************
void VCL_Ray_Carving::z_fvz_Find_a_Visible_Zone(
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
bool VCL_Ray_Carving::
	inj_Is_NULL_Jet(CKvVectorFloat *in_jet)
//*****************************************************************************************
{
	float *D; int sz;

	D=in_jet->vps(sz); 

	if(sz!=2) return false;
	if(D[0]!=Kv2008_DO_SURFACE_INVALID_MARKER) return false; 
	if(D[0]!=D[1]) return false;

	return true;
}


//********************************************************************************
void VCL_Ray_Carving::cvpi_Check_Vanishing_Point_In_Image(
	CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
	CKvSet_of_SdkCode *in_sil,
	CKvSet_of_Pmatrix3D *in_P_matrices)
//********************************************************************************
{
	CKvSdkCode *sc;
	CKvPmatrix3D *pmat; CKvMatrixFloat fpmat;
	float **RD,*rp,*dv,**p_fpmat; float vanishing_point[3];
	int i, nb_rays, num_cam; int ww, hh; int count = 0;
	bool dum = false; bool *p_vec;

	//Initialization		
	RD			= in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix->mps(i,nb_rays);
	rp						   = RD[0]; 
	dv						   = rp+3;
	pmat = in_P_matrices->vps(num_cam);
	fpmat.c_Create(3,4);
	p_vec = zz_Is_A_Vanishing_Point_In_Image.c_Create(num_cam,dum);
	sc    = in_sil->vp();
	for(i = 0 ; i < num_cam ; i++){
		sc[i].ms(ww,hh);
		pmat[i].e_Export(&fpmat);
		p_fpmat = fpmat.mp();

		vanishing_point[0] = p_fpmat[0][0]*dv[0] + p_fpmat[0][1]*dv[1] + p_fpmat[0][2]*dv[2];
		vanishing_point[1] = p_fpmat[1][0]*dv[0] + p_fpmat[1][1]*dv[1] + p_fpmat[1][2]*dv[2];
		vanishing_point[2] = p_fpmat[2][0]*dv[0] + p_fpmat[2][1]*dv[1] + p_fpmat[2][2]*dv[2];
		vanishing_point[0] /= vanishing_point[2]; vanishing_point[1] /= vanishing_point[2];

		if(  ( vanishing_point[0] > 0 ) && ( vanishing_point[0] < ww ) && (vanishing_point[1] > 0) && (vanishing_point[1] < hh) ){
			p_vec[i] = true; count++;
		 }
	 }
}
//******************************************************************************
void VCL_Ray_Carving::cahc_Calculate_Affine_Homography_for_each_Camera(	
	CKvSet_of_Pmatrix3D *in_p_mat,
	CKvMatrixFloat *in_rpdv)
//******************************************************************************
{
	LCKvAlgebra_for_Matrix am;
	LCKvUtility_for_Linear_Equation le;
	CKvPmatrix3D *pmat;
	CKvPoint3D oa;
	CKvMatrix *affine_homography, *rotation_matrix, *intrinsic_matrix, *inv_affine_homography;
	CKvMatrix r_prime, inv_intrinsic_matrix, inv_rotation_matrix;
	
	int i,num, dir;
	int num_camera;
	double temp;
	float **p_in_rpdv, *direction_of_ray; double **p_ah, **p_rp;
	double optical_axis[3];
	double r2[3]; double r3[3];

	//Initialization
	pmat                  = in_p_mat->vps(num_camera);
	p_in_rpdv             = in_rpdv->mps(dir,num);
	affine_homography     = zz_affine_homography_for_each_camera.c_Create(num_camera);
	inv_affine_homography = zz_inverse_affine_homography_for_each_camera.c_Create(num_camera);
	direction_of_ray      = p_in_rpdv[0];
	direction_of_ray      = direction_of_ray + 3;
	p_rp                  = r_prime.c_Create(3,3,0.0);

	for(i = 0 ; i < num_camera ; i++){
		oa              = pmat[i].dvo_Direction_Vector_of_Optical_axis();
		optical_axis[0] = oa.x;		
		optical_axis[1] = oa.y;		
		optical_axis[2] = oa.z;

		//cross product for calculating r2
		r2[0] = optical_axis[1]*direction_of_ray[2] - optical_axis[2]*direction_of_ray[1];
		r2[1] = optical_axis[2]*direction_of_ray[0] - optical_axis[0]*direction_of_ray[2];
		r2[2] = optical_axis[0]*direction_of_ray[1] - optical_axis[1]*direction_of_ray[0];
		temp  = sqrt(r2[0]*r2[0] + r2[1]*r2[1] + r2[2]*r2[2]);
		r2[0] /= temp;		r2[1] /= temp;		r2[2] /= temp;

		//cross product for calculating r3
		r3[0] = direction_of_ray[1]*r2[2] - direction_of_ray[2]*r2[1];
		r3[1] = direction_of_ray[2]*r2[0] - direction_of_ray[0]*r2[2];
		r3[2] = direction_of_ray[0]*r2[1] - direction_of_ray[1]*r2[0];
		temp = sqrt(r3[0]*r3[0] + r3[1]*r3[1] + r3[2]*r3[2]);
		r3[0] /= temp;		r3[1] /= temp;		r3[2] /= temp;
		
		rotation_matrix  = pmat[i].mpr_Matrix_Pointer_of_R_matrix();		
		intrinsic_matrix = pmat[i].mpk_Matrix_Pointer_of_K_matrix();
		p_ah             = affine_homography[i].c_Create(3,3,0.0); 
		inv_affine_homography[i].c_Create(3,3,0.0);
		
		//Direction vector of rays is z-axis//
		p_rp[0][0] = direction_of_ray[0];	p_rp[0][1] = direction_of_ray[1];		p_rp[0][2] = direction_of_ray[2];
		p_rp[1][0] = r2[0];		            p_rp[1][1] = r2[1];		                p_rp[1][2] = r2[2];
		p_rp[2][0] = r3[0];		            p_rp[2][1] = r3[1];		                p_rp[2][2] = r3[2];

		le.ilu_Inverse_matrix_based_on_LUD(intrinsic_matrix,&inv_intrinsic_matrix);
		inv_rotation_matrix.t_Transpose(rotation_matrix);
		am.mmm_Multiply_Matrix_Matrix(intrinsic_matrix,&r_prime,&inv_rotation_matrix,&inv_intrinsic_matrix,&affine_homography[i]);
		le.ilu_Inverse_matrix_based_on_LUD(&affine_homography[i], &inv_affine_homography[i]);
	}
}
//******************************************************************************
void VCL_Ray_Carving::ahp_Apply_Homography_to_Pmatrix(
	CKvSet_of_Pmatrix3D *in_p_mat,
	CKvSet_of_Pmatrix3D *out_p_mat)
//******************************************************************************
{
	LCKvAlgebra_for_Matrix am;
	CKvPmatrix3D *pmat, *t_pmat; 
	CKvMatrix pmat_export, transformed_p_mat;
	CKvMatrix *pah;
	double **p_tpmat,tmp;
	int num_camera,i,j,k;

	pmat   = in_p_mat->vps(num_camera);
	pah    = zz_affine_homography_for_each_camera.vp();
	t_pmat = out_p_mat->c_Create(num_camera);
	pmat_export.c_Create(3,4); 
	p_tpmat = transformed_p_mat.c_Create(3,4);

	for(i = 0 ; i < num_camera ; i++){
		pmat[ i ].e_Export(&pmat_export);
		am.mmm_Multiply_Matrix_Matrix(&pah[ i ], &pmat_export, &transformed_p_mat);
		
		tmp = sqrt(p_tpmat[2][0]*p_tpmat[2][0] + p_tpmat[2][1]*p_tpmat[2][1] + p_tpmat[2][2]*p_tpmat[2][2]);

		for(k = 0 ; k < 4 ; k++ )
			for(j = 0 ; j < 3 ; j++ )
				p_tpmat[j][k] /= tmp;
		t_pmat[ i ].i_Import(&transformed_p_mat);
	}
}
//********************************************************************************
void VCL_Ray_Carving::snpm_Set_New_P_Set_of_Matrix(
	CKvSet_of_Pmatrix3D *in_P_matrices,
	CKvSet_of_Pmatrix3D *in_rec_P_matrices,
	CKvSet_of_Pmatrix3D *out_P_matrices)
//********************************************************************************
{
	CKvPmatrix3D *in_pmat, *in_rec_pmat, *out_pmat;
	CKvMatrix pmat;
	bool *p_vp;
	int i,num;

	//Initialization
	in_pmat     = in_P_matrices->vps(num);
	in_rec_pmat = in_rec_P_matrices->vp();
	out_pmat    = out_P_matrices->c_Create(num);
	p_vp        = zz_Is_A_Vanishing_Point_In_Image.vp();
	pmat.c_Create(3,4); 

	for(i = 0 ; i < num ; i++){
		if(p_vp[i] == true){
			in_pmat[i].e_Export(&pmat);
			out_pmat[i].i_Import(&pmat);
		}
		else{
			in_rec_pmat[i].e_Export(&pmat);
			out_pmat[i].i_Import(&pmat);
		}
	}
}
//******************************************************************************
void VCL_Ray_Carving::slc_Set_Lookup_table_for_each_Camera(
	CKvSet_of_Pmatrix3D *in_p_mat,
	int width_of_rays,
	int height_of_rays,
	CKvMatrixFloat *in_rpdv)
//******************************************************************************
{
	CKvPmatrix3D *pmat; 
	CKvMatrix pmat_export;
	CKvMatrix *p_lut;
	float projected_point[3];
	float **p_in_rpdv;
	double **p_p_lut, **p_pmat_export;
	int i,j,num, dir, num_camera;

	//Initialization
	pmat      = in_p_mat->vps(num_camera);
	p_in_rpdv = in_rpdv->mps(dir,num);
	zz_look_up_table_of_projected_rays_for_each_camera.c_Create(num_camera);
	p_lut     = zz_look_up_table_of_projected_rays_for_each_camera.vp_Vector_Pointer();
	pmat_export.c_Create(3,4); 

	for(i = 0 ; i < num_camera ; i++){
		p_lut[i].c_Create(num, 2, 0.0);
		pmat[i].e_Export(&pmat_export);
		p_pmat_export = pmat_export.mp();
		p_p_lut       = p_lut[i].mp();

		for(j = 0 ; j < num ; j++){
			projected_point[0] = p_pmat_export[0][0]*p_in_rpdv[j][0] + p_pmat_export[0][1]*p_in_rpdv[j][1] + p_pmat_export[0][2]*p_in_rpdv[j][2] + p_pmat_export[0][3];
			projected_point[1] = p_pmat_export[1][0]*p_in_rpdv[j][0] + p_pmat_export[1][1]*p_in_rpdv[j][1] + p_pmat_export[1][2]*p_in_rpdv[j][2] + p_pmat_export[1][3];
			projected_point[2] = p_pmat_export[2][0]*p_in_rpdv[j][0] + p_pmat_export[2][1]*p_in_rpdv[j][1] + p_pmat_export[2][2]*p_in_rpdv[j][2] + p_pmat_export[2][3];
			projected_point[0]/= projected_point[2]; projected_point[1] /= projected_point[2]; projected_point[2] /= projected_point[2];
			p_p_lut[j][0]      = (double)projected_point[0];  p_p_lut[j][1] = (double)projected_point[1];
		}
	}
}
//********************************************************************************
void VCL_Ray_Carving::gss_Get_size_of_silhouettes_minmax(
	CKvSet_of_SdkCode *in_set_of_silhouette_images,
	CKvMatrixFloat *size_mat)
//******************************************************************************
{
	CKvSdkCode *in_sil; 
	CKvMatrix *homography; 
	CKvMatrixUchar mcu;
	CKvRunSet src_runset;
	double **p_h;
	bool *ivpii; bool dummy; float **p_size_mat;
	unsigned char **p_src_m;
	int img_width, img_height, num_img,i,j,k; 
	float xx, yy, kk;
	float minx, maxx, miny, maxy;

	in_sil     = in_set_of_silhouette_images->vps(num_img);
	ivpii      = zz_Is_A_Vanishing_Point_In_Image.vp();
	homography = zz_affine_homography_for_each_camera.vp();
	p_size_mat = size_mat->c_Create(num_img, 4);
		 
	for(k = 0 ; k < num_img ; k++){
		maxx = -99999.0f;
		minx = 99999.0f; 
		maxy = -99999.0f; 
		miny = 99999.0f;
		in_sil [ k ].ms(img_width, img_height);

		if(ivpii[k] == true) {		
			p_size_mat[k][0] = img_width;
			p_size_mat[k][1] = img_height;
			p_size_mat[k][2] = 0.0f;
			p_size_mat[k][3] = 0.0f;
		}
		else {		
			in_sil[k].e_Export(&src_runset, dummy);
			src_runset.e_Export(255,0,&mcu);
			p_src_m = mcu.mp();
			p_h     = homography[k].mp();

			for(i = 0 ; i < img_width ; i++)
				for(j = 0 ; j < img_height ; j++){
			 		xx  = (float)(p_h[0][0]*(float)i + p_h[0][1]*(float)j + p_h[0][2]);
			 		yy  = (float)(p_h[1][0]*(float)i + p_h[1][1]*(float)j + p_h[1][2]);
			 		kk  = (float)(p_h[2][0]*(float)i + p_h[2][1]*(float)j + p_h[2][2]);
			 		xx /= kk; 
					yy /= kk;
			 
			 		if (maxx < xx)	maxx = xx;
			 		if (minx > xx) minx = xx;
			 		if (maxy < yy) maxy = yy;
			 		if (miny > yy) miny = yy;
			}
			p_size_mat[k][0] = maxx - minx;
			p_size_mat[k][1] = maxy - miny;
			p_size_mat[k][2] = minx;
			p_size_mat[k][3] = miny;
		}
	}
}
//********************************************************************************
void VCL_Ray_Carving::coopm_Change_Order_of_P_Matrix(
	CKvSet_of_Pmatrix3D *io_P_matrices,
	CKvMatrixFloat *size_mat,
	int start_img)
//********************************************************************************
{
	if(start_img == 0) return;
	if(size_mat = NULL) return;
	CKvPmatrix3D *in_pmat;
	CKvSet_of_Pmatrix3D set_tmp_pmat;	CKvSet_of_Matrix set_tmp_mat;
	CKvMatrix buf_mat; CKvMatrix *p_tmp_mat;
	CKvMatrixFloat tmp_size; float buf_size[4];
	int num, size_num, dum;

	float **p_size_mat = size_mat->mps(dum,size_num);
	float **p_tmp_size = tmp_size.c_Create(start_img,4);
	in_pmat            = io_P_matrices->vps(num);
	p_tmp_mat          = set_tmp_mat.c_Create(start_img);
	
	if(size_num != num) { printf("trouble\n");return;}

	set_tmp_pmat.c_Create(start_img);
	buf_mat.c_Create(3,4);

	for(int j = 0 ; j < start_img ; j++){
		in_pmat[j].e_Export(&p_tmp_mat[j]);		
		p_tmp_size[j][0] = p_size_mat[j][0]; p_tmp_size[j][1] = p_size_mat[j][1];
		p_tmp_size[j][2] = p_size_mat[j][2]; p_tmp_size[j][3] = p_size_mat[j][3];		
	}

	for(int j = start_img ; j < num ; j++){
		in_pmat[ j ].e_Export(&buf_mat);
		in_pmat[ j - start_img ].i_Import(&buf_mat);
		buf_size[ 0 ] = p_size_mat[ j ][ 0 ]; buf_size[ 1 ] = p_size_mat[ j ][ 1 ];
		buf_size[ 2 ] = p_size_mat[ j ][ 2 ]; buf_size[ 3 ] = p_size_mat[ j ][ 3 ];
		p_size_mat[ j - start_img ][ 0 ] = buf_size[ 0 ]; p_size_mat[ j - start_img ][ 1 ] = buf_size[ 1 ];
		p_size_mat[ j - start_img ][ 2 ] = buf_size[ 2 ]; p_size_mat[ j - start_img ][ 3 ] = buf_size[ 3 ];
	}

	for(int j = num-start_img ; j < num ; j++){
		in_pmat[ j ].i_Import(&p_tmp_mat[ j - (num - start_img) ]);
		p_size_mat[ j ][ 0 ] = p_tmp_size[ j - (num - start_img) ][ 0 ]; p_size_mat[ j ][ 1 ] = p_tmp_size[ j - (num - start_img) ][ 1 ];
		p_size_mat[ j ][ 2 ] = p_tmp_size[ j - (num - start_img) ][ 2 ]; p_size_mat[ j ][ 3 ] = p_tmp_size[ j - (num - start_img) ][ 3 ];
	}
}
//*****************************************************************************************
void VCL_Ray_Carving::s2d3dr_Set_2dim_3dim_Relation_rectified(
	CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
	CKvMatrixFloat *size_mat,
	CKvSet_of_Pmatrix3D *in_P_matrices)
//********************************************************************************
{
	// fip:2 fidv:2 zvp:1 g:4 vmin:1 vmax:1
	// valid:1 exist_vp:1 zero_direction_vector_flag:1
	CKvPmatrix3D *Pmat;
	static float p[12],h[4],vanish[2];
	double *q;
	float **RD,*rp,*dv,tmp,**p_float_val, *fip, *fidv,*zvp,*g,*vmin, *vmax;
	float t_vmin,t_vmax;
	int nb_rays,nb_img,i,j,k; float **p_size_mat;
	bool **p_bool_val,*valid,*exist_vp, *zero_direction_vector_flag;
	bool *is_a_vanish_point;
	//Initialization		
	RD			= in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix->mps(i,nb_rays);
	Pmat        = in_P_matrices->vps(nb_img);
	p_float_val = zz_float_val_Nx11_matrix.c_Create(nb_rays*nb_img,11);
	p_bool_val  = zz_bool_val_Nx3_matrix  .c_Create(nb_rays*nb_img,3);
	is_a_vanish_point = zz_Is_A_Vanishing_Point_In_Image.vp();
	p_size_mat = size_mat->mp();

	//Set relation for each ray 
	for( j = 0 ; j < nb_img ; j++ ){
		q = Pmat[j].mpp()->mp()[0]; 
		for(k=0;k<12;k++)	p[k]=(float)q[k];			

		for(i=0;i<nb_rays;i++){
			rp						   = RD[i]; 
			dv						   = rp+3;
			fip					       = p_float_val[j*nb_rays+i];
			fidv					   = fip +2;
			zvp						   = fidv+2;
			g						   = zvp +1;
			vmin					   = g   +4;
			vmax				       = vmin+1;
			valid					   = p_bool_val[j*nb_rays+i];
			exist_vp				   = valid+1;
			zero_direction_vector_flag = exist_vp +1;

			t_vmin = -1.0f;
			t_vmax =  1.0f;

			//Ignore rays outside the unit sphere
			tmp=0.0f; 
			for(k=0;k<3;k++) tmp+=(rp[k]*rp[k]); 
			if(tmp>=1.0f){(*valid)=false; continue;}
	
			z_pr_Project_a_Ray_Rectified(
				p_size_mat[ j ],			// int in_width_of_image,
				rp,								// float *in_reference_3d_point_of_a_ray,
				dv,								// float *in_unit_direction_3d_vector_of_a_ray,
				p,								// float *in_P_matrix_3x4_in_row_first_mode,
				fip,							// float *out_image_2d_point_of_a_visible_point_in_3D_world,
				fidv,							// float *out_direction_2d_vector_on_image_plane,
				vanish,
				(*zvp),							// float &out_coordinate_of_vanishing_2d_point,
				g,								// float *out_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode,
				h,								// float *out_H_matrix_2x2_from_3_dim_to_2_dim_in_row_first_mode,
				(*valid),						// bool &out_existence_of_a_visible_3d_point_on_the_ray,
				t_vmin,							// float &io_lower_limit_of_visible_zone,
				t_vmax,							// float &io_upper_limit_of_visible_zone,
				(*exist_vp),					// bool &out_existence_of_vanishing_2d_point,
				(*zero_direction_vector_flag), // bool &out_flag_indicating_direction_2d_vector_is_zero)
				is_a_vanish_point[j]);

			vmin[0] = t_vmin;		
			vmax[0] = t_vmax;				
		}
	}
}
//********************************************************************************
void VCL_Ray_Carving::z_pr_Project_a_Ray_Rectified(
	float *size_vec,
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
	bool &out_flag_indicating_direction_2d_vector_is_zero,
	bool &in_Is_A_Vanishing_Point_In_Image)
//*****************************************************************************************
{
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

	z_fvz_Find_a_Visible_Zone_Rectified(
		size_vec,
		rp, dv, in_P_matrix_3x4_in_row_first_mode,
		io_lower_limit_of_visible_zone,
		io_upper_limit_of_visible_zone,
		out_existence_of_a_visible_3d_point_on_the_ray,
		in_Is_A_Vanishing_Point_In_Image);

	if(!out_existence_of_a_visible_3d_point_on_the_ray) return;

	if((io_lower_limit_of_visible_zone*io_upper_limit_of_visible_zone)<=0.0f) zvis=0.0f;
	else zvis=(io_lower_limit_of_visible_zone+io_upper_limit_of_visible_zone)*0.5f; 
	
	for(k=0;k<3;k++) wp[k]=rp[k]+dv[k]*zvis;

	z_prv_Project_a_Ray_defined_by_a_Visible_reference_point_Rectified(
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
		out_flag_indicating_direction_2d_vector_is_zero,
		size_vec,
		in_Is_A_Vanishing_Point_In_Image);

	h[1] = -h[0]*zvis; h[3] -= (h[2]*zvis);
	g[0] += (g[2]*zvis); g[1] = g[3]*zvis; 
	return;
}
//*****************************************************************************************
void VCL_Ray_Carving::z_fvz_Find_a_Visible_Zone_Rectified(
	float *size_vec,
	float *in_reference_3d_point_of_a_ray,
	float *in_unit_direction_3d_vector_of_a_ray,
	float *in_P_matrix_3x4_in_row_first_mode,
	float &io_lower_limit_of_visible_zone,
	float &io_upper_limit_of_visible_zone,
	bool &out_existence_of_visible_3d_point,
	bool &in_Is_A_Vanishing_Point_In_Image)
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

	sz[0]=(int)(size_vec[0]-1.0f); // width
	sz[1]=(int)(size_vec[1]-1.0f);// height

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
			if(in_Is_A_Vanishing_Point_In_Image == false){

				xo[0] -= xo[2]*size_vec[2]; xo[1] -= xo[2]*size_vec[3];

			}
			 

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
void VCL_Ray_Carving::z_prv_Project_a_Ray_defined_by_a_Visible_reference_point_Rectified(
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
	bool &out_flag_indicating_direction_2d_vector_is_zero,
	float *size_vec,
	bool &in_Is_A_Vanishing_Point_In_Image)
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
	if(in_Is_A_Vanishing_Point_In_Image == false){

		xo[0] -= xo[2]*size_vec[2]; xo[0] -= xo[2]*size_vec[3];
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


//******************************************************************************
bool VCL_Ray_Carving::drc_Do_Ray_Carving_using_recitified_silhouette_Lossy(
	CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
	int ray_width,
	int ray_height,
	CKvSet_of_SdkCode *in_set_of_silhouette_images,
	float *io_elapsed_time)
//******************************************************************************
{
	// <assumption> 
	// (1) -1.0 <= depth <= 1.0
	// (2) rp dot dv = 0
	// (3) mag(dv) = 1
	// (4) mag(rp) < 1 : rp is inside the unit sphere
	CKvVectorFloat *DOC; 
	CKvSdkCode *sdk;
	int nb_img; 
	
	//Initialization
	DOC	  = zz_analog_map_2D.mp()[0];
	sdk   = in_set_of_silhouette_images->vps(nb_img);

	//Do Ray Carving
	if(!z_drc_Do_Ray_Carving_using_recitified_silhouette_Lossy(
		in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
		ray_width,
		ray_height,
		sdk,
		nb_img,
		DOC,
		io_elapsed_time))gerr("4");
	return true;
error:
	zsm("drc_Do_Ray_Carving");
	return false;
}
//******************************************************************************
bool VCL_Ray_Carving::z_drc_Do_Ray_Carving_using_recitified_silhouette_Lossy(
	CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
	int ray_width,
	int ray_height,
	CKvSdkCode *in_set_of_silhouette_images,
	int in_number_of_images,
	CKvVectorFloat *out_set_of_depth_jets,
	float *io_elapsed_time)
//******************************************************************************
{
	// <assumption> 
	// (1) -1.0 <= depth <= 1.0
	// (2) rp dot dv = 0
	// (3) mag(dv) = 1
	// (4) mag(rp) < 1 : rp is inside the unit sphere
	CKvSdkCode *sdk;
	CKvSdkCode rec_sdk;
	CKvVectorFloat *DOC;
	CKvMatrix *mat_lut, *mat_homo, *inv_mat_homo;
	int k, nb_img; 
	bool *p_vp; 
	double **p_mat_lut;
	double io_minx, io_maxx, io_miny, io_maxy;

	//Initialization
	nb_img		   = in_number_of_images; 
	DOC			   = out_set_of_depth_jets;
	sdk			   = in_set_of_silhouette_images;
	p_vp		   = zz_Is_A_Vanishing_Point_In_Image.vp();
	mat_homo       = zz_affine_homography_for_each_camera.vp();
	inv_mat_homo   = zz_inverse_affine_homography_for_each_camera.vp();
	mat_lut        = zz_look_up_table_of_projected_rays_for_each_camera.vp();	

	//Do Ray Carving
	int kkk = 0;
	if(p_vp[kkk] == false){
		p_mat_lut = mat_lut[kkk].mp();
		Apply_Homography_to_Binary_Image(
			&sdk[kkk],//CKvSdkCode *in_set_of_silhouette_images,
			io_minx,//double io_minx,
			io_maxx,//double io_maxx,
			io_miny,//double io_miny,
			io_maxy,//double io_maxy,
			&mat_homo[kkk],//CKvMatrix *homography,
			&inv_mat_homo[kkk],//CKvMatrix *inv_homography,
			rec_sdk);//CKvSdkCode &rectified_sil)

		z_idj_Initialize_Depth_Jets_using_rectified_silhouette_Lossy(
			in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
			ray_width,//int ray_width,
			ray_height,//int ray_height,
			io_minx,//double in_minx,
			io_miny,//double in_miny,
			p_mat_lut,//double **start_point_of_ray,
			&rec_sdk,//CKvSdkCode *in_silhouette_image,
			DOC);
	}
	else{
		z_idj_Initialize_Depth_Jets_using_Precomputed_Ray_Relation(
			in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,//CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
			&sdk[kkk],//CKvSdkCode *in_silhouette_image,
			DOC);//CKvVectorFloat *out_set_of_depth_jets);
	}
 	for(k = 1 ; k < nb_img; k++){
 		if(p_vp[k] == false){
 			p_mat_lut = mat_lut[k].mp();
 			Apply_Homography_to_Binary_Image(
 				&sdk[k],//CKvSdkCode *in_set_of_silhouette_images,
 				io_minx,//double io_minx,
 				io_maxx,//double io_maxx,
 				io_miny,//double io_miny,
 				io_maxy,//double io_maxy,
 				&mat_homo[k],//CKvMatrix *homography,
 				&inv_mat_homo[k],
 				rec_sdk);//CKvSdkCode &rectified_sil)

			z_udj_Update_Depth_Jets_using_rectified_silhouette_Lossy(
	 			in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
	 			ray_width,//int ray_width,
	 			ray_height,//int ray_height,
	 			io_minx,//double in_minx,
	 			io_miny,//double in_miny,
	 			p_mat_lut,//double **start_point_of_ray,
	 			&rec_sdk,//CKvSdkCode *in_silhouette_image,
	 			k,
	 			DOC);
	 	}
	 	else{	 
	 		z_udj_Update_Depth_Jets_using_Precomputed_Ray_Relation(
	 			in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,//CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
	 			&sdk[k],//CKvSdkCode *in_silhouette_image,
	 			k,//int in_idx_silhouettes,
	 			DOC);//CKvVectorFloat *io_set_of_depth_jets);
 		}
 	}
	return true;
}
//******************************************************************************
void VCL_Ray_Carving::Apply_Homography_to_Binary_Image(
	CKvSdkCode *in_set_of_silhouette_images,
	double &io_minx,
	double &io_maxx,
	double &io_miny,
	double &io_maxy,
	CKvMatrix *homography,
	CKvMatrix *inv_homography,
	CKvSdkCode &rectified_sil)
//******************************************************************************
{
	CKvRunSet src_rs; CKvMatrixUchar src_m;
	CKvRunSet dst_rs; CKvMatrixUchar dst_m;
	unsigned char init_value;
	unsigned char  **p_src_m, **p_dst_m;
	double **p_h, **p_ivh;
	bool dummy;

	float xx, yy, kk;
	int x, y, img_width, img_height; 
	int minx, maxx, miny, maxy;
	
	//Initialization
	init_value = 0;	
	in_set_of_silhouette_images->e_Export(&src_rs,dummy);
	src_rs.e_Export(255,0,&src_m);
	p_src_m = src_m.mp(); src_m.ms(img_width, img_height);

	p_h   = homography->mp();
	p_ivh = inv_homography->mp();

	io_maxx = -99999.0f; 
	io_minx = 99999.0f;	
	io_maxy = -99999.0f; 
	io_miny = 99999.0f;

	for(int i = 0 ; i < img_width ; i++){
		for(int j = 0 ; j < img_height ; j++){
			
			if(p_src_m[j][i]!= 0){
				xx = (float)(p_h[0][0]*(float)i + p_h[0][1]*(float)j + p_h[0][2]);
				yy = (float)(p_h[1][0]*(float)i + p_h[1][1]*(float)j + p_h[1][2]);
				kk = (float)(p_h[2][0]*(float)i + p_h[2][1]*(float)j + p_h[2][2]);
				xx /= kk; yy /= kk;

				if (io_maxx < xx) io_maxx = xx;
				if (io_minx > xx) io_minx = xx;
				if (io_maxy < yy) io_maxy = yy;
				if (io_miny > yy) io_miny = yy;
			}
		}
	}

 	minx = (int)(io_minx + 0.5f); maxx = (int)(io_maxx + 0.5f);
 	miny = (int)(io_miny + 0.5f); maxy = (int)(io_maxy + 0.5f);

	p_dst_m = dst_m.c_Create(maxy-miny, maxx-minx, init_value);
	
	for(int i = minx ; i < maxx ; i++){
		for(int j = miny ; j < maxy ; j++){

			xx = (float)(p_ivh[0][0]*(float)i + p_ivh[0][1]*(float)j + p_ivh[0][2]); 
			yy = (float)(p_ivh[1][0]*(float)i + p_ivh[1][1]*(float)j + p_ivh[1][2]);
			kk = (float)(p_ivh[2][0]*(float)i + p_ivh[2][1]*(float)j + p_ivh[2][2]);
			xx/= kk ; 
			yy/= kk;
			x  = (int)(xx+0.5f); 
			y  = (int)(yy+0.5f);

			if(x < 0 || x > img_width - 1 || y < 0 || y > img_height - 1) continue;

			p_dst_m[j - miny][i - minx]= p_src_m[y][x];

		}
	}
	dst_rs.i_Import(&dst_m, 128 , dummy);
	rectified_sil.i_Import(&dst_rs, dummy);
}
//******************************************************************************
bool VCL_Ray_Carving::z_idj_Initialize_Depth_Jets_using_rectified_silhouette_Lossy(
	CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
	int ray_width,
	int ray_height,
	double in_minx,
	double in_miny,
	double **start_point_of_ray,
	CKvSdkCode *in_silhouette_image,
	CKvVectorFloat *out_set_of_depth_jets)
//********************************************************************************
{
	// <assumption> 
	// (1) -1.0 <= depth <= 1.0
	// (2) rp dot dv = 0
	// (3) mag(dv) = 1
	// (4) mag(rp) < 1 : rp is inside the unit sphere
	// (5) io_depth_jet is valid and not NULL jet. 
	// (6) valid segments exist between -1 and 1
	CKvRunSet rs;
	CKvVectorFloat *DOC; 
	CKvVectorInt out_list_for_type_of_run, out_list_for_label_of_run;
	CKvVectorInt out_list_for_x1, out_list_for_x2, out_list_for_index_of_last_run_in_a_row, out_list_for_number_of_runs_in_a_row;
	bool *valid_ray,*zero_direction_vector_flag,*exist_vp;
	float **RD,*rp,*dv,*D,*fip,*fidv,*zvp,*g,*vmin,*vmax,tmp; 
	float **p_float_val, *pp_float_val;
	bool  **p_bool_val,  *pp_bool_val;
	bool dummy, out_validity;
	int N,ww,hh,i,k;
	static CKvVectorFloat vec; 

	//Initialization
	in_silhouette_image->ms(ww,hh); 
	RD          = in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix->mps(i, N);
	DOC         = out_set_of_depth_jets;
	p_float_val = zz_float_val_Nx11_matrix.mp();
	p_bool_val  = zz_bool_val_Nx3_matrix.mp();

	//실루엣 rectification수행
	N = ray_width*ray_height; // ray의 갯수
 	in_silhouette_image->e_Export(&rs, dummy);
	rs.gri_Get_Run_Informations(
		&out_list_for_type_of_run, //CKvVectorInt *out_list_for_type_of_run,
		&out_list_for_label_of_run,//CKvVectorInt *out_list_for_label_of_run,
		&out_list_for_x1,//CKvVectorInt *out_list_for_x1,
		&out_list_for_x2,//CKvVectorInt *out_list_for_x2,
		&out_list_for_index_of_last_run_in_a_row,//CKvVectorInt *out_list_for_index_of_last_run_in_a_row,
		&out_list_for_number_of_runs_in_a_row);//CKvVectorInt *out_list_for_number_of_runs_in_a_row);

	for(i = 0 ; i < N ; i++){
 		//Get results from Set_Rays
		rp                         = RD[i]; 
		dv                         = rp+3;
		D                          = DOC[i].c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER);

		//Get results from Set_Rays
		pp_float_val               = p_float_val[i];
		pp_bool_val                = p_bool_val[i];
		fip					       = pp_float_val;
		fidv					   = fip +2;
		zvp						   = fidv+2;
		g						   = zvp +1;
		vmin					   = g   +4;
		vmax				       = vmin+1;
		valid_ray  			       = pp_bool_val;
		exist_vp				   = valid_ray+1;
		zero_direction_vector_flag = exist_vp +1;
		
		//Ignore rays outside the unit sphere
		tmp = 0.0f; 
		for(k=0;k<3;k++) tmp+=(rp[k]*rp[k]); 
		if(tmp>=1.0f){
			continue;
		}
		
		//Ignore invalid rays
		if(!(*valid_ray)){
			continue;
		}

		//Shave segments
		D[0] = (*vmin);	
  		D[1] = (*vmax);

		if(!z_fsb_Find_Segments_carved_by_Boundaries_using_rectified_silhouette_Lossy(
			&out_list_for_x1,
			&out_list_for_x2,
			&out_list_for_index_of_last_run_in_a_row,
			&out_list_for_number_of_runs_in_a_row,
			in_minx,
			in_miny,
			start_point_of_ray[i],
			&vec,
			out_validity) ){Kv_Printf("[z_ss_Shave_Segments] 1"); exit(0);}
		if(!out_validity){ 
			DOC[i].c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER);
			continue;
		}

		bp_Back_Projection(
			&vec,
			(*exist_vp),
			g,
			&DOC[i],
			(*zvp),
			out_validity);
	}
	return true;
}
//********************************************************************************
bool VCL_Ray_Carving::z_udj_Update_Depth_Jets_using_rectified_silhouette_Lossy(
	CKvMatrixFloat *in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix,
	int ray_width,
	int ray_height,
	double in_minx,
	double in_miny,
	double **start_point_of_ray,
	CKvSdkCode *in_silhouette_image,
	int in_idx_silhouettes,
	CKvVectorFloat *out_set_of_depth_jets)
//********************************************************************************
{
	// <assumption> 
	// (1) -1.0 <= depth <= 1.0
	// (2) rp dot dv = 0
	// (3) mag(dv) = 1
	// (4) mag(rp) < 1 : rp is inside the unit sphere
	CKvRunSet rs;
	CKvVectorFloat *DOC;
	CKvVectorInt out_list_for_type_of_run, out_list_for_label_of_run;
	CKvVectorInt out_list_for_x1, out_list_for_x2, out_list_for_index_of_last_run_in_a_row, out_list_for_number_of_runs_in_a_row;
	bool *valid_ray,*zero_direction_vector_flag,*exist_vp;
	float **RD,*rp,*dv,*fip,*fidv,*zvp,*g,*vmin,*vmax; 
	float **p_float_val, *pp_float_val;
	bool  **p_bool_val,  *pp_bool_val;
	bool dummy, out_validity;
	int N,ww,hh,i;
	static CKvVectorFloat vec; 

	//Initialization
	in_silhouette_image->ms(ww,hh); 
	RD          = in_set_of_3d_ref_pt_and_dir_vectors_Nx6_matrix->mps(i,N);
	DOC         = out_set_of_depth_jets;
	p_float_val = zz_float_val_Nx11_matrix.mp();
	p_bool_val  = zz_bool_val_Nx3_matrix.mp();

	//실루엣 rectification수행
	N = ray_width*ray_height; // ray의 갯수
	in_silhouette_image->e_Export(&rs, dummy);
	rs.gri_Get_Run_Informations(
		&out_list_for_type_of_run, //CKvVectorInt *out_list_for_type_of_run,
		&out_list_for_label_of_run,//CKvVectorInt *out_list_for_label_of_run,
		&out_list_for_x1,//CKvVectorInt *out_list_for_x1,
		&out_list_for_x2,//CKvVectorInt *out_list_for_x2,
		&out_list_for_index_of_last_run_in_a_row,//CKvVectorInt *out_list_for_index_of_last_run_in_a_row,
		&out_list_for_number_of_runs_in_a_row);//CKvVectorInt *out_list_for_number_of_runs_in_a_row);

	for(i = 0 ; i < N ; i++){
		//Get results from Set_Rays
		rp                         = RD[i]; 
		dv                         = rp+3;

		if(inj_Is_NULL_Jet(&DOC[i])){
			continue;
		}
		//Get results from Set_Rays
		pp_float_val               = p_float_val[in_idx_silhouettes*N+i];
		pp_bool_val                = p_bool_val [in_idx_silhouettes*N+i];
		fip					       = pp_float_val;
		fidv					   = fip +2;
		zvp						   = fidv+2;
		g						   = zvp +1;
		vmin					   = g   +4;
		vmax				       = vmin+1;
		valid_ray  			       = pp_bool_val;
		exist_vp				   = valid_ray+1;
		zero_direction_vector_flag = exist_vp +1;

		if(!(*valid_ray)) { 
			DOC[i].c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER); 
			continue;
		}

		if(!z_fsb_Find_Segments_carved_by_Boundaries_using_rectified_silhouette_Lossy(
			&out_list_for_x1,
			&out_list_for_x2,
			&out_list_for_index_of_last_run_in_a_row,
			&out_list_for_number_of_runs_in_a_row,
			in_minx,
			in_miny,
			start_point_of_ray[i],
			&vec,
			out_validity) ){Kv_Printf("[z_ss_Shave_Segments] 1"); exit(0);}

		if(!out_validity){ 
			DOC[i].c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER);
			continue;
		}

		bp_Back_Projection(
			&vec,
			(*exist_vp),
			g,
			&DOC[i],
			(*zvp),
			out_validity);
	}

	return true;
}
//*****************************************************************************************
bool VCL_Ray_Carving::z_ss_Shave_Segments_using_rectified_silhouette_Lossy(
	CKvVectorInt *out_list_for_x1,
	CKvVectorInt *out_list_for_x2,
	CKvVectorInt *out_list_for_index_of_last_run_in_a_row,
	CKvVectorInt *out_list_for_number_of_runs_in_a_row,
	float *in_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode,//새로운 것으로 교체 
	double in_minx,
	double in_miny,
	double *start_point_of_ray,
	bool in_existence_of_vanishing_2d_point,
	float in_coordinate_of_vanishing_2d_point,
	CKvVectorFloat *io_depth_jet,//i-1번쨰 실루엣 carving 결과를 받아 i번쨰 실루엣 까지의 carving 결과를 리턴
	bool &out_validity)
//*****************************************************************************************
{
	// assumption : 
	// (1) io_depth_jet is valid and not NULL jet. 
	// (2) valid segments exist between -1 and 1
	static CKvVectorFloat vec; 

	if(!z_fsb_Find_Segments_carved_by_Boundaries_using_rectified_silhouette_Lossy(
		out_list_for_x1,
		out_list_for_x2,
		out_list_for_index_of_last_run_in_a_row,
		out_list_for_number_of_runs_in_a_row,
		in_minx,
		in_miny,
		start_point_of_ray,
		&vec,
		out_validity) ){Kv_Printf("[z_ss_Shave_Segments] 1"); exit(0);}

	if(!out_validity){ io_depth_jet->c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER); return true;}

	bp_Back_Projection(
		&vec,
		in_existence_of_vanishing_2d_point,
		in_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode,
		io_depth_jet,
		in_coordinate_of_vanishing_2d_point,
		out_validity);

	return true;
}

//*****************************************************************************************
bool VCL_Ray_Carving::z_fsb_Find_Segments_carved_by_Boundaries_using_rectified_silhouette_Lossy(
	CKvVectorInt *out_list_for_x1,
	CKvVectorInt *out_list_for_x2,
	CKvVectorInt *out_list_for_index_of_last_run_in_a_row,
	CKvVectorInt *out_list_for_number_of_runs_in_a_row,
	double in_minx,
	double in_miny,
	double *start_point_of_ray,
	CKvVectorFloat *out_set_of_segment_coordinates,
	bool &out_validity)
//*****************************************************************************************
{
	float *p_set_of_segment;
	int minx, size;
	int coordinate_y_of_ray, num;
	int index_of_first_run, index_of_last_run, count;
	minx = (int)( in_minx + 0.5f ); 
	int *p_index_of_last_run = out_list_for_index_of_last_run_in_a_row->vps(size);
	int *p_list_for_number_of_run = out_list_for_number_of_runs_in_a_row->vp();
	int *x1 = out_list_for_x1->vp();
	int *x2 = out_list_for_x2->vp();

	coordinate_y_of_ray = (int)(start_point_of_ray[1] - in_miny + 0.5f);
	
	if( (coordinate_y_of_ray > -1) && (coordinate_y_of_ray < size) ){
		count             = 0;
		index_of_last_run = p_index_of_last_run[coordinate_y_of_ray];
		if(index_of_last_run == -1){ out_validity = false; return true; }
	
		num                = p_list_for_number_of_run[coordinate_y_of_ray];
		p_set_of_segment   = out_set_of_segment_coordinates->c_Create(num*2);//
		index_of_first_run = index_of_last_run - num + 1;

		for(int i = index_of_first_run ; i <= index_of_last_run ; i++){
 			p_set_of_segment[count]     = (float)( ( x1[i] - (start_point_of_ray[0] - minx)  - 0.5f) );
 			p_set_of_segment[count + 1] = (float)( ( x2[i] - (start_point_of_ray[0] - minx) + 0.5f) );			
			count += 2;
		}
		out_validity = true;
	}
	else{  out_validity = false;	return true;}
	return true;
}

//*****************************************************************************************
bool VCL_Ray_Carving::bp_Back_Projection(
	CKvVectorFloat *in_vec,
	bool in_existence_of_vanishing_2d_point,
	float *in_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode,//새로운 것으로 교체 
	CKvVectorFloat *io_depth_jet,//i-1번쨰 실루엣 carving 결과를 받아 i번쨰 실루엣 까지의 carving 결과를 리턴
	float in_coordinate_of_vanishing_2d_point,
	bool &out_validity)
//*****************************************************************************************
{
	// assumption : 
	// (1) io_depth_jet is valid and not NULL jet. 
	// (2) valid segments exist between -1 and 1
	static CKvVectorFloat tvec; 
	float zmin,zmax,*a, *b, *c, v1, v2, zvp, *g, xx, tmp;
	int i, j, k, np, np_old, np_new;
	int start, last;

	//Initialization
	a   = in_vec->vps(np);
	g   = in_H_matrix_2x2_from_2_dim_to_3_dim_in_row_first_mode;
	zvp = in_coordinate_of_vanishing_2d_point;

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
		//In case of null jet
		if(a[0]==Kv2008_DO_SURFACE_INVALID_MARKER){
			io_depth_jet->c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER); 
			out_validity = false;
			return true;
		}
		//3차원 ray가 점으로 투영되는 경우, 해당 점이 실루엣 안에 있을경우, skip, 그렇지 않을 경우 null jet으로 만든다 
		if(zvp==0.0f){
			for(k=0;k<np;k++)  { if(a[k]>0.0f) break;}
			if((k>=np)||(k==0)){
				io_depth_jet->c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER); 
				out_validity=false;
				return true;
			}
			else{
				if((k%2==0)||(a[k-1]>=0.0f)){
					io_depth_jet->c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER); 
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
		io_depth_jet->c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER); 
		out_validity=false;
		return true;
	}
	tvec.cp_Copy(io_depth_jet); 
	b = tvec.vps(np_old); 
	a = in_vec->vps(np);

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
		b           = io_depth_jet->c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER); 
		return true;
	}
	(np_new<<=1);

	c = io_depth_jet->c_Create(np_new);

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

//***********************************************************************************//
bool VCL_Ray_Carving::Voxel_carving(
	CKvSet_of_Pmatrix3D *in_pmat,
	CKvSet_of_MatrixBool *in_silhouette,
	int in_width,
	int in_height,
	int in_depth,
	int &num_voxel,
	float &out_time)
	//***********************************************************************************//
{
	int i, j, k, l, num_img, w, h;
	CKvDepth_on_Surface dos;
	CKvSet_of_Rect rect_set;
	double radius;
	CKvPoint3D center, r, step;
	CKvPoint3D bound_tl, bound_br;
	double dx, dy, dz;
	CKvVectorInt wset, hset;
	int *p_w, *p_h;

	CKvMatrixBool *p_b_img;
	CKvPmatrix3D *p_pmat;
	CKvPoint3D p3d;
	CKvPoint p2d;
	bool flag;

	CKvStopWatch watch;
	watch.c_Create(1);
	watch.r_Reset(0);

	dos.u_grs_Get_Rectangles_of_Silhouettes(in_silhouette, &rect_set);
	dos.u_elo_Estimate_Location_of_Object(in_pmat, &rect_set, center, radius);

	r.x=r.y=r.z=radius;
	bound_tl=center+r;
	bound_br=center-r;

	dx=bound_tl.x-bound_br.x;
	dy=bound_tl.y-bound_br.y;
	dz=bound_tl.z-bound_br.z;
	step.x=dx/in_width;
	step.y=dy/in_height;
	step.z=dz/in_depth;

	p_b_img=in_silhouette->vps_Vector_Pointer_and_Size(num_img);
	p_pmat=in_pmat->vp_Vector_Pointer();
	p_w=wset.c_Create(num_img);
	p_h=hset.c_Create(num_img);
	for(i=0;i<num_img;i++)
	{
		p_b_img[i].ms_Matrix_Width_Height(p_w[i],p_h[i]);
	}

	num_voxel=0;
	for(k=0;k<in_depth;k++){
		for(j=0;j<in_height;j++){
			for(i=0;i<in_width;i++){
				p3d=bound_tl-Kv_Point3D(0.5*step.x, 0.5*step.y, 0.5*step.z)
					-Kv_Point3D(i*step.x, j*step.y, k*step.z);
				flag=true;
				for(l=0;l<num_img;l++)
				{
					p_pmat[l].tp_Transform_Point(p3d, p2d);
					if((p_b_img[l].ge_Get_Element((float)p2d.x, (float)p2d.y)==false)||
						(p2d.x<0)||(p2d.x>p_w[l])||
						(p2d.y<0)||(p2d.y>p_h[l]))
					{
						flag=false;
						l=num_img;
					}
				}
				if(flag==true)
				{
					num_voxel++;
				}

			}
		}
	}

	out_time=watch.get_Get_Elapsed_Time(0);

	return true;
}