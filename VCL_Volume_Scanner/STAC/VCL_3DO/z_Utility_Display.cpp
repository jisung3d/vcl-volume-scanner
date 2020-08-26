/////////////////////////////////////////////////////////////////////////////////////////////
// z_Utility_Display.cpp
/////////////////////////////////////////////////////////////////////////////////////////////

#include "../../stdafx.h"
// #include "VCL_Inhand_Object_Scanning.h"
// #include "VCL_Inhand_Object_ScanningDlg.h"
//#include "_Utility_Display.h"
#include "../_VCL_2014_3DO.h"

Util_Display::Util_Display()
{
}
Util_Display::~Util_Display()
{
}

////////////////////////////
/////////Display////////////
////////////////////////////
//Plot 3D object from DoCube
//******************************************************************************
void Util_Display::po_Plot_Object(
	CKvGraph3D *in_scr,
	CKvDoCubeShort *in_object,
	CKvPmatrix3D *in_pmat_or_NULL)
//******************************************************************************
{
	CKvVectorInt list_point;
	CKvDepot_of_Point3D depot_point;
	CKvDepot_of_RgbaF depot_color;
	int i,*I,nb,color_index[6],offset[3];
	CKvSet_of_RgbaF set_color;
	CKvSet_of_Point3D set_point;
	CKvMesh_of_Point pt_mesh;
	CKvSet_of_VectorInt element;

	//Preparing point depot
	in_object->gp_Get_set_of_Points(true, &set_point, NULL);
	depot_point.ap_Append(true,&set_point,i,i); 
	nb = depot_point.ne_Number_of_Elements();

	//Preparing color depot
	for(i=0; i<6; i++) color_index[i] = 0;
	set_color.c_Create(1);
	set_color.se_Set_Element   (0, &Kv_RgbaF(1.0, 0.0, 0.0, 0.0));
	depot_color.ap_Append(true,&set_color,i,i);

	//Assign info for each point
	I=list_point.c_Create(nb);
	for(i=0;i<nb;i++) { I[i]=i; }
	pt_mesh.u_me_Make_Element(&list_point,0,2,&element);
	offset[0]=offset[1]=offset[2]=0;
	pt_mesh.ap_Append(true,&element,offset,i); 

	//Preparing 3D point object display
	in_scr->g_sp_Set_display_Parameters(true, Kv_RgbF(255.0f,255.0f,255.0f));
	in_scr->g_id_Initialize_Depots(&depot_point,&depot_color,NULL,NULL,NULL,NULL,NULL);
	
	//Plot 3D point object
	in_scr->g_p_Plot(in_pmat_or_NULL,&depot_point,NULL, &depot_color,NULL,NULL,NULL,NULL,NULL,&pt_mesh, true);
	return;
}
//******************************************************************************
void Util_Display::po_Plot_Object(
	CKvGraph3D *in_scr,
	CKvDoCubeShort *in_object,
	CKvPmatrix3D *in_pmat_or_NULL,
	CKvRgbaF in_color,
	int in_point_size)
//******************************************************************************
{
	CKvVectorInt idx_point,idx_color,idx_size;
	CKvDepot_of_Point3D depot_point;
	CKvSet_of_Point3D set_point;
	CKvDepot_of_RgbaF depot_color;
	int num_elements,dummy1,dummy2,k,*p_idx_point,*p_idx_color,*p_idx_size,offset[3];
	CKvSet_of_RgbaF set_color;
	CKvMesh_of_Point pt_mesh;
	CKvSet_of_VectorInt element;
	CKvRgbaF *p_set_color;
	//Initialization
	num_elements = in_object->np_Number_of_Points();
	p_set_color  = set_color.c_Create(num_elements);
	p_idx_point  = idx_point.c_Create(num_elements);
	p_idx_color  = idx_color.c_Create(num_elements);
	p_idx_size   = idx_size .c_Create(num_elements);
	
	//Preparing point depot
	in_object->gp_Get_set_of_Points(true, &set_point, NULL);
	depot_point.ap_Append(true,&set_point,dummy1,dummy2); 
	
	//Preparing color depot
	for(k=0;k<num_elements;k++) p_set_color[k] = in_color;
	depot_color.ap_Append(true,&set_color,dummy1,dummy2);
	
	//Assign info for each point
	for(k=0;k<num_elements;k++) { p_idx_point[k]=k;p_idx_color[k]=k;p_idx_size[k]=in_point_size; }
	pt_mesh.u_me_Make_Element(&idx_point,&idx_color,&idx_size,&element); 
	offset[0]=offset[1]=offset[2]=0;
	pt_mesh.ap_Append(true,&element,offset,dummy1); 
	
	//Preparing 3D point object display
	in_scr->g_sp_Set_display_Parameters(true, Kv_RgbF(255.0f,255.0f,255.0f));
	in_scr->g_id_Initialize_Depots(&depot_point,&depot_color,NULL,NULL,NULL,NULL,NULL);
	
	//Plot 3D point object
	in_scr->g_p_Plot(in_pmat_or_NULL,&depot_point,NULL, &depot_color, NULL, NULL, NULL, NULL,NULL,&pt_mesh, true);
	return;
}
//******************************************************************************
void Util_Display::po_Plot_Object(
	CKvGraph3D *in_scr,
	CKvDoCubeShort *in_object,
	CKvPmatrix3D *in_pmat_or_NULL,
	CKvSet_of_RgbaF *in_color,
	int in_point_size)
//******************************************************************************
{
	CKvVectorInt idx_point,idx_color,idx_size;
	CKvDepot_of_Point3D depot_point;
	CKvSet_of_Point3D set_point;
	CKvDepot_of_RgbaF depot_color;
	int num_elements,dummy1,dummy2,k,*p_idx_point,*p_idx_color,*p_idx_size,offset[3];
	CKvSet_of_RgbaF set_color;
	CKvMesh_of_Point pt_mesh;
	CKvSet_of_VectorInt element;
	CKvRgbaF *p_set_color;
	//Initialization
	num_elements = in_object->np_Number_of_Points();
	p_set_color  = set_color.c_Create(num_elements);
	p_idx_point  = idx_point.c_Create(num_elements);
	p_idx_color  = idx_color.c_Create(num_elements);
	p_idx_size   = idx_size .c_Create(num_elements);
	
	//Preparing point depot
	in_object->gp_Get_set_of_Points(true, &set_point, NULL);
	depot_point.ap_Append(true,&set_point,dummy1,dummy2); 
	
	//Preparing color depot
	depot_color.ap_Append(true,in_color,dummy1,dummy2);
	
	//Assign info for each point
	for(k=0;k<num_elements;k++) { p_idx_point[k]=k;p_idx_color[k]=k;p_idx_size[k]=in_point_size; }
	pt_mesh.u_me_Make_Element(&idx_point,&idx_color,&idx_size,&element); 
	offset[0]=offset[1]=offset[2]=0;
	pt_mesh.ap_Append(true,&element,offset,dummy1); 
	
	//Preparing 3D point object display
	in_scr->g_sp_Set_display_Parameters(true, Kv_RgbF(255.0f,255.0f,255.0f));
	in_scr->g_id_Initialize_Depots(&depot_point,&depot_color,NULL,NULL,NULL,NULL,NULL);
	
	//Plot 3D point object
	in_scr->g_p_Plot(in_pmat_or_NULL,&depot_point,NULL, &depot_color, NULL, NULL, NULL, NULL,NULL,&pt_mesh, true);
	return;
}
//Plot 3D object from set_of_points3D
//******************************************************************************
void Util_Display::po_Plot_Object(
	CKvGraph3D *in_scr,
	CKvSet_of_Point3D *in_point,
	CKvPmatrix3D *in_pmat_or_NULL)
//******************************************************************************
{
	CKvVectorInt list_point;
	CKvDepot_of_Point3D depot_point;
	CKvDepot_of_RgbaF depot_color;
	CKvMesh_of_Triangle mesh_triangle;
	int i,*I,nb,color_index[6],offset[3];
	CKvSet_of_RgbaF set_color;
	CKvMesh_of_Point pt_mesh;
	CKvSet_of_VectorInt element;

	//Preparing point depot
	depot_point.ap_Append(true,in_point,i,i); 
	nb = depot_point.ne_Number_of_Elements();

	//Preparing color depot
	for(i=0; i<6; i++) color_index[i] = 0;
	set_color.c_Create(1);
	set_color.se_Set_Element   (0, &Kv_RgbaF(1.0, 0.0, 0.0, 0.0));
	depot_color.ap_Append(true,&set_color,i,i);	

	//Assign info for each point
	I=list_point.c_Create(nb);
	for(i=0;i<nb;i++) { I[i]=i; }
	pt_mesh.u_me_Make_Element(&list_point,0,4,&element);
	offset[0]=offset[1]=offset[2]=0;
	pt_mesh.ap_Append(true,&element,offset,i); 
	
	//Preparing 3D point object display
	in_scr->g_sp_Set_display_Parameters(true, Kv_RgbF(255.0f, 255.0f, 255.0f));
	in_scr->g_id_Initialize_Depots(&depot_point,&depot_color,NULL,NULL,NULL,NULL,NULL);
	
	//Plot 3D point object
	in_scr->g_p_Plot(in_pmat_or_NULL,&depot_point,NULL, &depot_color,NULL,NULL,NULL,NULL,NULL,&pt_mesh, true);
	return;
}
//******************************************************************************
void Util_Display::po_Plot_Object(
	CKvGraph3D *in_scr,
	CKvSet_of_Point3D *in_point,
	CKvPmatrix3D *in_pmat_or_NULL,
	CKvRgbaF in_color,
	int in_point_size)
//******************************************************************************
{
	CKvVectorInt idx_point,idx_color,idx_size;
	CKvDepot_of_Point3D depot_point;
	CKvDepot_of_RgbaF depot_color;
	int num_elements,dummy1,dummy2,k,*p_idx_point,*p_idx_color,*p_idx_size,offset[3];
	CKvSet_of_RgbaF set_color;
	CKvMesh_of_Point pt_mesh;
	CKvSet_of_VectorInt element;
	CKvRgbaF *p_set_color;
	
	//Initialization
	num_elements = in_point->vs();
	p_set_color  = set_color.c_Create(num_elements);
	p_idx_point  = idx_point.c_Create(num_elements);
	p_idx_color  = idx_color.c_Create(num_elements);
	p_idx_size   = idx_size .c_Create(num_elements);
	
	//Preparing point depot
	depot_point.ap_Append(true,in_point,dummy1,dummy2); 
	
	//Preparing color depot
	for(k=0;k<num_elements;k++) p_set_color[k] = in_color;
	depot_color.ap_Append(true,&set_color,dummy1,dummy2);
	
	//Assign info for each point
	for(k=0;k<num_elements;k++) { p_idx_point[k]=k;p_idx_color[k]=k;p_idx_size[k]=in_point_size; }
	pt_mesh.u_me_Make_Element(&idx_point,&idx_color,&idx_size,&element); 
	offset[0]=offset[1]=offset[2]=0;
	pt_mesh.ap_Append(true,&element,offset,dummy1); 
	
	//Preparing 3D point object display
	in_scr->g_sp_Set_display_Parameters(true, Kv_RgbF(255.0f,255.0f,255.0f));
	in_scr->g_id_Initialize_Depots(&depot_point,&depot_color,NULL,NULL,NULL,NULL,NULL);
	
	//Plot 3D point object
	in_scr->g_p_Plot(in_pmat_or_NULL,&depot_point,NULL, &depot_color, NULL, NULL, NULL, NULL,NULL,&pt_mesh, true);
	return;
}
//******************************************************************************
void Util_Display::po_Plot_Object(
	CKvGraph3D *in_scr,
	CKvSet_of_Point3D *in_point,
	CKvPmatrix3D *in_pmat_or_NULL,
	CKvSet_of_RgbaF *in_color,
	int in_point_size)
//******************************************************************************
{
	CKvVectorInt idx_point,idx_color,idx_size;
	CKvDepot_of_Point3D depot_point;
	CKvDepot_of_RgbaF depot_color;
	int num_elements,dummy1,dummy2,k,*p_idx_point,*p_idx_color,*p_idx_size,offset[3];
	CKvSet_of_RgbaF set_color;
	CKvMesh_of_Point pt_mesh;
	CKvSet_of_VectorInt element;
	CKvRgbaF *p_set_color;
	
	//Initialization
	num_elements = in_point->vs();
	p_set_color  = set_color.c_Create(num_elements);
	p_idx_point  = idx_point.c_Create(num_elements);
	p_idx_color  = idx_color.c_Create(num_elements);
	p_idx_size   = idx_size .c_Create(num_elements);
	
	//Preparing point depot
	depot_point.ap_Append(true,in_point,dummy1,dummy2); 
	
	//Preparing color depot
	depot_color.ap_Append(true,in_color,dummy1,dummy2);
	
	//Assign info for each point
	for(k=0;k<num_elements;k++) { p_idx_point[k]=k;p_idx_color[k]=k;p_idx_size[k]=in_point_size; }
	pt_mesh.u_me_Make_Element(&idx_point,&idx_color,&idx_size,&element); 
	offset[0]=offset[1]=offset[2]=0;
	pt_mesh.ap_Append(true,&element,offset,dummy1); 
	
	//Preparing 3D point object display
	in_scr->g_sp_Set_display_Parameters(true, Kv_RgbF(255.0f,255.0f,255.0f));
	in_scr->g_id_Initialize_Depots(&depot_point,&depot_color,NULL,NULL,NULL,NULL,NULL);
	//Plot 3D point object
	in_scr->g_p_Plot(in_pmat_or_NULL,&depot_point,NULL, &depot_color, NULL, NULL, NULL, NULL,NULL,&pt_mesh, true);
	return;
}

//Plot 3D mesh object from DoCube
//******************************************************************************
void Util_Display::pmo_Plot_Mesh_Object(
	CKvGraph3D *in_scr,
	CKvDepot_of_Point3D *in_depot_point,
	CKvDepot_of_RgbaF *in_depot_color,
	CKvMesh_of_Triangle *in_mesh_triangle,
	CKvPmatrix3D *in_pmat_or_NULL)
//******************************************************************************
{
	//Preparing 3D point object display
	in_scr->g_sp_Set_display_Parameters(true, Kv_RgbF(0.0f, 0.0f, 255.0f));
	in_scr->g_id_Initialize_Depots(
		in_depot_point, //CKvDepot_of_Point3D *in_set_of_points_or_NULL,
		in_depot_color, //CKvDepot_of_RgbaF *in_set_of_colors_or_NULL,
		NULL, //&zz_depot_image, //CKvDepot_of_String *in_set_of_images_or_NULL,
		NULL, //&zz_depot_image_point, //CKvDepot_of_Point *in_set_of_image_points_or_NULL,
		NULL, //&depot_Pmat, //CKvDepot_of_Pmatrix3D *in_set_of_P_matrices_or_NULL,
		NULL, //CKvDepot_of_Font *in_set_of_fonts_or_NULL,
		NULL); //CKvDepot_of_String *in_set_of_text_strings_or_NULL);
	
	//Plot 3D mesh object
	in_scr->g_p_Plot(in_pmat_or_NULL,in_depot_point,NULL,in_depot_color,in_mesh_triangle,NULL,NULL,NULL,NULL,NULL,true);
	return;
}
//******************************************************************************
void Util_Display::pmo_Plot_Mesh_Object(
	CKvGraph3D *in_scr,
	CKvSet_of_Point3D *in_p3d, 
	CKvMatrixInt *in_3xn_mesh)
//******************************************************************************
{
	CKvDepot_of_Point3D depot_point;
	CKvDepot_of_RgbaF depot_color;
	CKvMesh_of_Triangle mesh_triangle;
	CKvSet_of_RgbaF set_color;
	CKvVectorInt element, color;
	CKvSet_of_VectorInt set_element;
	int i,ww,offset[3];
	int **p_ori_mesh,*p_element,*p_color;

	depot_point.ap_Append(true,in_p3d,i,i); 

	//Preparing color depot
	set_color  .c_Create(1);
	set_color  .se_Set_Element   (0, &Kv_RgbaF(1.0, 0.0, 0.0, 0.0));
	depot_color.ap_Append(true,&set_color,i,i);

	//Preparing mesh index
	p_ori_mesh = in_3xn_mesh->mps(ww,i);
	p_element  = element.c_Create(ww*3);
	p_color    = color.c_Create(ww*3);
	for(i=0;i<ww;i++){
		p_element[3*i  ]= p_ori_mesh[0][i]; 
		p_element[3*i+1]= p_ori_mesh[1][i];
		p_element[3*i+2]= p_ori_mesh[2][i];
		p_color  [3*i  ]= p_color  [3*i+1]= p_color  [3*i+2]= 0;
	}

	//Make mesh triangle
	offset[0]=offset[1]=offset[2]=0;
	mesh_triangle.in_Initialize();
	mesh_triangle.u_me_Make_Element(&element, &color, &color, &set_element);
	mesh_triangle.ap_Append(true, &set_element, offset, i);
	
	//Display mesh triangle
	in_scr->g_sp_Set_display_Parameters(true, Kv_RgbF(255.0f, 255.0f, 255.0f));
	in_scr->g_id_Initialize_Depots(
		&depot_point, //CKvDepot_of_Point3D *in_set_of_points_or_NULL,
		&depot_color, //CKvDepot_of_RgbaF *in_set_of_colors_or_NULL,
		NULL, //&zz_depot_image, //CKvDepot_of_String *in_set_of_images_or_NULL,
		NULL, //&zz_depot_image_point, //CKvDepot_of_Point *in_set_of_image_points_or_NULL,
		NULL, //&depot_Pmat, //CKvDepot_of_Pmatrix3D *in_set_of_P_matrices_or_NULL,
		NULL, //CKvDepot_of_Font *in_set_of_fonts_or_NULL,
		NULL); //CKvDepot_of_String *in_set_of_text_strings_or_NULL);
	
	//Plot 3D mesh object
	in_scr->g_p_Plot(NULL,&depot_point,NULL,&depot_color,&mesh_triangle,NULL,NULL,NULL,NULL,NULL,true);
	return;
}
//Plot 3D mesh object from precomputed mesh elements
//******************************************************************************
void Util_Display::pmo_Plot_Mesh_Object(
	CKvGraph3D *in_scr,
	CKvDepot_of_Point3D *in_dp,
	CKvDepot_of_RgbaF *in_dc_or_NULL,
	CKvPmatrix3D *in_pmat_or_NULL,
	CKvMesh_of_Triangle *in_mesh_triangle)
//******************************************************************************
{
	//Preparing 3D point object display
	in_scr->g_sp_Set_display_Parameters(true, Kv_RgbF(255.0f, 255.0f, 255.0f));
	in_scr->g_id_Initialize_Depots(in_dp,in_dc_or_NULL,NULL,NULL,NULL,NULL,NULL);
	//Plot 3D point object
	in_scr->g_p_Plot(in_pmat_or_NULL,in_dp,NULL,in_dc_or_NULL,in_mesh_triangle, NULL, NULL, NULL,NULL,NULL,true);
	return;
}
//Plot 3D mesh object using images
//******************************************************************************
void Util_Display::pmo_Plot_Mesh_Object(
	CKvGraph3D *in_scr,
	CKvDepot_of_Point3D *in_dp,
	CKvDepot_of_Point3D *in_sn_or_NULL,
	CKvDepot_of_Point *in_dp_img,
	CKvSet_of_String *in_texture_set,
	CKvPmatrix3D *in_pmat_or_NULL,
	CKvMesh_of_TriImage *in_mesh_tri_image)
//******************************************************************************
{
	CKvDepot_of_String texture_set;
	texture_set.im_Import(in_texture_set);
	in_scr->g_sp_Set_display_Parameters(true, Kv_RgbF(255.0f, 255.0f, 255.0f));
	in_scr->g_id_Initialize_Depots(
		in_dp,         //CKvDepot_of_Point3D *in_set_of_points_or_NULL,
		NULL,          //CKvDepot_of_RgbaF   *in_set_of_colors_or_NULL,
		&texture_set,  //CKvDepot_of_String  *in_set_of_images_or_NULL,
		in_dp_img,     //CKvDepot_of_Point   *in_set_of_image_points_or_NULL,
		NULL,          //CKvDepot_of_Pmatrix3D *in_set_of_P_matrices_or_NULL,
		NULL,          //CKvDepot_of_Font *in_set_of_fonts_or_NULL,
		NULL);         //CKvDepot_of_String *in_set_of_text_strings_or_NULL);

	in_scr->g_pti_Plot_TriImage(in_pmat_or_NULL,in_sn_or_NULL,in_mesh_tri_image,true);

	return;
}

//////////////////////////////////////////////////////////////////////////////////
////////////////////2014.02.05. by Yooji.//////////////////////////////////////
void Util_Display::pmosvd_Plot_Mesh_Object_Sequence_View_Dependent(
	CKvGraph3D *in_scr,
	int in_ww, int in_hh,
	int in_max_frame_num,
	float in_frame_rate,
	CKvSet_of_Pmatrix3D *in_set_of_pmats,
	CKvDepot_of_Point3D *in_set_of_depot_points,
	CKvDepot_of_RgbaF *in_depot_colors,
	CKvMesh_of_Triangle *in_set_of_mesh_triangle,
	CKvSet_of_VectorBool *in_set_of_per_mesh_vis,
	CKvSet_of_VectorUcharRgb *in_set_of_vertex_colors_for_all_cam)
{	
	//For texture mapping.
	LCKvUtility_for_Windows uw;
	CKvVectorFloat color_weights;
	CKvSet_of_RgbaF set_color;
	CKvPmatrix3D virtual_view;
	CKvMatrixFloat virtual_view_mat;
	CKvStopWatch zz_sw;

	int num_cameras;
	bool play_button=true, init_flag=false;
	int prev_idx=-1, tmp;
	int num_disp_per_frame=0, frame_counter=0;
	float elapsed_time_total=0.0f, elapsed_time_disp=0.0f;
	float *p_virtual_view;

	virtual_view.cc_Create_Canonical(false);
	p_virtual_view=virtual_view_mat.c_Create(3, 4, 0.0f)[0];
	num_cameras=in_set_of_pmats->vs();

	/// ////////////////////////////////////////////////////////////////////////
	/// Display 3D object sequence  //////
	/// ////////////////////////////////////////////////////////////////////////
	for(int k=0;k<in_max_frame_num;k++){	
		// initialize frame rate.
		if(!init_flag){
			zz_sw.c_Create(1);		zz_sw.r_Reset(0);		// for total time.

			virtual_view.cp_Copy(in_set_of_pmats->gpe_Get_Pointer_of_Element(0));
			//Preparing 3D point object display
			in_scr->g_sp_Set_display_Parameters(true,  Kv_RgbF(255.0f, 255.0f, 255.0f), false, Kv_Hpoint3D(0.0, 0.0, 0.0, 1.0), 0.0f, 0.0f, false, true, 0);		//in_graph_3d->g_sp_Set_display_Parameters(true,  Kv_RgbF(255.0f, 255.0f, 255.0f));					
		}
		else{
			in_scr->g_gpm_Get_P_matrix(true, &virtual_view_mat);		//for(int i=0; i<12; i++)		printf("%5.2f ", p_virtual_view[i]);					printf("\n");
			virtual_view.i_Import(&virtual_view_mat);
		}

		if(k!=prev_idx){
			set_color.c_Create(in_set_of_depot_points[k].ne_Number_of_Elements());
			set_color.se_Set_Element   (0, &Kv_RgbaF(1.0, 0.0, 0.0, 0.0));
			in_depot_colors->in_Initialize(in_set_of_depot_points[k].ne_Number_of_Elements());
			in_depot_colors->ap_Append(true,&set_color,tmp,tmp);
		}

		//View-dependent texture mapping.
		//update depot color.
		ccwac_Compute_Color_Weights_for_All_Cameras(in_set_of_pmats, &virtual_view, &color_weights);
		// view-dependent texture mapping.	
		vdtmm_View_Dependent_Texture_Mapping_on_Mesh(in_set_of_pmats, &in_set_of_per_mesh_vis[k], &in_set_of_vertex_colors_for_all_cam[k], &color_weights, &virtual_view, &in_set_of_mesh_triangle[k], in_depot_colors);					
		// set depots for display.			
		in_scr->g_id_Initialize_Depots(&in_set_of_depot_points[k],	in_depot_colors,	NULL,	NULL,	NULL,	NULL,	NULL);
		// display 3D object.
		in_scr->g_p_Plot(NULL, &in_set_of_depot_points[k], NULL, in_depot_colors, &in_set_of_mesh_triangle[k], NULL, NULL, NULL,NULL,NULL,true);

		// Object sequence player control.
		if(!init_flag){
			// calculate pause time for set frame rate to user-intended value.
			elapsed_time_total=zz_sw.get_Get_Elapsed_Time(0);
			num_disp_per_frame=(int)((1.0f/in_frame_rate - elapsed_time_total)/elapsed_time_total);		// sec

			if(!Kv_Printf("User-intended fps: %f fps \n Time per display: %f ms\nNumber of display per frame: %d frames", 
				in_frame_rate, elapsed_time_total, num_disp_per_frame))		exit(0);

			uw.ss_Set_Size(in_scr, false, in_ww, in_hh);
			init_flag=true;
		}			

		// keyboard control.
		prev_idx=k;
		if(_kbhit() == 1){
			char ch=_getch();

			// pause/play button.
			if(ch== 'p'){				
				if(!play_button)	{	play_button=true;		printf("[Play] # %d frame (%% %5.2f)\n", k+1, 100.0f*(float)(k+1)/(float)in_max_frame_num);		}
				else						{	play_button=false;		printf("[Pause] # %d frame (%% %5.2f)\n", k+1, 100.0f*(float)(k+1)/(float)in_max_frame_num);		}
			}
			// back button (-5 frames).
			else if(ch== 'b'){		k=max(0, k-5);				printf("[Back] # %d frame (%% %5.2f)\n", k+1, 100.0f*(float)(k+1)/(float)in_max_frame_num);		}
			// next button (+5 frames).
			else if(ch== 'n'){		k=min(in_max_frame_num-1, k+5);			printf("[Next] # %d frame (%% %5.2f)\n", k+1, 100.0f*(float)(k+1)/(float)in_max_frame_num);		}
			// stop button.
			else if(ch=='s'){			break;		}
			// exit player.
			else if(ch==27){		exit(0);	}
		}
		if(play_button)		frame_counter++;

		/// /////////////////////////////////////////////////////		
		if(k==in_max_frame_num-1){		play_button=false;		k--;	}
		else if(frame_counter%num_disp_per_frame || !play_button)		k--;		
		/// /////////////////////////////////////////////////////

		Kv_Pause(10);
	}


}

void Util_Display::pmosvd_Plot_Mesh_Object_Sequence_View_Dependent_NEW(
	CKvGraph3D *in_scr,
	int in_ww, int in_hh,
	int in_max_frame_num,
	float in_frame_rate,
	CKvSet_of_Pmatrix3D *in_set_of_pmats,
	CKvDepot_of_Point3D *in_set_of_depot_points,
	CKvDepot_of_RgbaF *in_depot_colors,
	CKvMesh_of_Triangle *in_set_of_mesh_triangle,
	CKvVectorUcharRgb *in_color_palette,
	CKvSet_of_VectorInt *in_vertex_color_indices_for_all_cam)
{	
	//For texture mapping.
	LCKvUtility_for_Windows uw;
	CKvVectorFloat color_weights;
	CKvSet_of_RgbaF set_color;
	CKvPmatrix3D virtual_view;
	CKvMatrixFloat virtual_view_mat;
	CKvStopWatch zz_sw;

	int num_cameras;
	bool play_button=true, init_flag=false;
	int prev_idx=-1, tmp;
	int num_disp_per_frame=0, frame_counter=0;
	float elapsed_time_total=0.0f, elapsed_time_disp=0.0f;
	float *p_virtual_view;

	virtual_view.cc_Create_Canonical(false);
	p_virtual_view=virtual_view_mat.c_Create(3, 4, 0.0f)[0];
	num_cameras=in_set_of_pmats->vs();

	/// ////////////////////////////////////////////////////////////////////////
	/// Display 3D object sequence  //////
	/// ////////////////////////////////////////////////////////////////////////
	for(int k=0;k<in_max_frame_num;k++){	
		// initialize frame rate.
		if(!init_flag){
			//zz_sw.c_Create(1);		zz_sw.r_Reset(0);		// for total time.
			virtual_view.cp_Copy(in_set_of_pmats->gpe_Get_Pointer_of_Element(0));
			//Preparing 3D point object display
			in_scr->g_sp_Set_display_Parameters(true,  Kv_RgbF(255.0f, 255.0f, 255.0f), false, Kv_Hpoint3D(0.0, 0.0, 0.0, 1.0), 0.0f, 0.0f, false, true, 0);		//in_graph_3d->g_sp_Set_display_Parameters(true,  Kv_RgbF(255.0f, 255.0f, 255.0f));					
		}
		else{
			in_scr->g_gpm_Get_P_matrix(true, &virtual_view_mat);		//for(int i=0; i<12; i++)		printf("%5.2f ", p_virtual_view[i]);					printf("\n");
			virtual_view.i_Import(&virtual_view_mat);
		}
		if(k!=prev_idx){
			set_color.c_Create(in_set_of_depot_points[k].ne_Number_of_Elements());
			set_color.se_Set_Element   (0, &Kv_RgbaF(1.0, 0.0, 0.0, 0.0));
			in_depot_colors->in_Initialize(in_set_of_depot_points[k].ne_Number_of_Elements());
			in_depot_colors->ap_Append(true,&set_color,tmp,tmp);
		}
		//View-dependent texture mapping.
		//update depot color.
		ccwac_Compute_Color_Weights_for_All_Cameras(in_set_of_pmats, &virtual_view, &color_weights);
		// view-dependent texture mapping.	
		vdtmm_View_Dependent_Texture_Mapping_on_Mesh_NEW(in_set_of_pmats, in_color_palette, &in_vertex_color_indices_for_all_cam[k], &color_weights, &virtual_view, &in_set_of_mesh_triangle[k], in_depot_colors);				
		// set depots for display.			
		in_scr->g_id_Initialize_Depots(&in_set_of_depot_points[k],	in_depot_colors,	NULL,	NULL,	NULL,	NULL,	NULL);	
		//printf("set depots for display\n");
		// display 3D object.
		in_scr->g_p_Plot(NULL, &in_set_of_depot_points[k], NULL, in_depot_colors, &in_set_of_mesh_triangle[k], NULL, NULL, NULL,NULL,NULL,true);
		//printf("display 3D object\n");
		// Object sequence player control.
		if(!init_flag){
			//calculate pause time for set frame rate to user-intended value.
			//elapsed_time_total=zz_sw.get_Get_Elapsed_Time(0);
			//num_disp_per_frame=(int)((1.0f/in_frame_rate - elapsed_time_total)/elapsed_time_total);		// sec
				
			num_disp_per_frame=10;
			/// 이거 없애면 display 들어갈 때 P matrix error 나면서 죽음.
			if(!Kv_Printf("User-intended fps: %f fps \n Time per display: %f ms\nNumber of display per frame: %d frames", 
				in_frame_rate, elapsed_time_total, num_disp_per_frame))		exit(0);

			uw.ss_Set_Size(in_scr, false, in_ww, in_hh);
			init_flag=true;
		}			
		// keyboard control.
		prev_idx=k;
		if(play_button)		printf("  [Play] # %d frame (%% %5.2f) \r", k+1, 100.0f*(float)(k+1)/(float)in_max_frame_num);
		if(_kbhit() == 1){
			char ch=_getch();

			// pause/play button.
			if(ch== 'p'){				
				if(!play_button)	{	play_button=true;		}
				else						{	play_button=false;		printf("  [Pause] # %d frame (%% %5.2f)\r", k+1, 100.0f*(float)(k+1)/(float)in_max_frame_num);		}
			}
			// back button (-5 frames).
			else if(ch== 'b'){		k=max(0, k-5);				/*printf("  [Back] # %d frame (%% %5.2f)\r", k+1, 100.0f*(float)(k+1)/(float)in_max_frame_num);*/		}
			// next button (+5 frames).
			else if(ch== 'n'){		k=min(in_max_frame_num-1, k+5);			/*printf("  [Next] # %d frame (%% %5.2f)\r", k+1, 100.0f*(float)(k+1)/(float)in_max_frame_num);*/		}
			// stop button.
			else if(ch=='s'){		printf("  [Stop] \r\n");	break;		}
			// exit player.
			else if(ch==27){		exit(0);	}
		}
		if(play_button)		frame_counter++;

		/// /////////////////////////////////////////////////////		
		if(k==in_max_frame_num-1){		play_button=false;		k--;	}
		else if(frame_counter%num_disp_per_frame || !play_button)		k--;		
		/// /////////////////////////////////////////////////////

		Kv_Pause(10);
	}
	printf("\n");

}

//****************************************************
void Util_Display::pmosvd_Plot_Mesh_Object_Sequence_View_Dependent_NEW_TOTAL(
	CKvGraph3D *in_scr,
	int in_ww, int in_hh,
	int in_max_frame_num,
	int in_num_of_GOP,
	float in_frame_rate,
	CKvSet_of_Pmatrix3D *in_set_of_pmats,
	CKvDepot_of_Point3D *in_set_of_depot_points,
	CKvDepot_of_RgbaF *in_depot_colors,
	CKvMesh_of_Triangle *in_set_of_mesh_triangle,
	CKvVectorUcharRgb *in_set_of_color_palette,
	CKvSet_of_VectorInt *in_vertex_color_indices_for_all_cam)
//****************************************************
{	
	//For texture mapping.
	LCKvUtility_for_Windows uw;
	CKvVectorFloat color_weights;
	CKvSet_of_RgbaF set_color;
	CKvPmatrix3D virtual_view;
	CKvMatrixFloat virtual_view_mat;
	CKvStopWatch zz_sw;

	int num_cameras;
	bool play_button=true, init_flag=false;
	int prev_idx=-1, tmp;
	int num_disp_per_frame=0, frame_counter=0;
	float elapsed_time_total=0.0f, elapsed_time_disp=0.0f;
	float *p_virtual_view;

	virtual_view.cc_Create_Canonical(false);
	p_virtual_view=virtual_view_mat.c_Create(3, 4, 0.0f)[0];
	num_cameras=in_set_of_pmats->vs();

	////////////////////////////////////////////////////////////////////////
	// Display 3D object sequence  //////
	////////////////////////////////////////////////////////////////////////
	for(int k=0;k<in_max_frame_num;k++){	
		// initialize frame rate.
		if(!init_flag){
			virtual_view.cp_Copy(in_set_of_pmats->gpe_Get_Pointer_of_Element(0));
			//Preparing 3D point object display
			/////////////////////////////////////////////////////////////////
			///////현재 Geometry만 테스트 중임...................///////////
			////////////////////////////////////////////////////////////////
			//in_scr->g_sp_Set_display_Parameters(true, Kv_RgbF(255.0f, 255.0f, 255.0f));
			in_scr->g_sp_Set_display_Parameters(true,  Kv_RgbF(255.0f, 255.0f, 255.0f), false, Kv_Hpoint3D(0.0, 0.0, 0.0, 1.0), 0.0f, 0.0f, false,true,0);		
			/////////////////////////////////////////////////////////////////
			///////현재 Geometry만 테스트 중임...................///////////
			////////////////////////////////////////////////////////////////
		}
		else{
			in_scr->g_gpm_Get_P_matrix(true, &virtual_view_mat);		
			virtual_view.i_Import(&virtual_view_mat);
		}
		if(k!=prev_idx){
			set_color.c_Create(in_set_of_depot_points[k].ne_Number_of_Elements());
			set_color.se_Set_Element   (0, &Kv_RgbaF(1.0, 0.0, 0.0, 0.0));
			in_depot_colors->in_Initialize(in_set_of_depot_points[k].ne_Number_of_Elements());
			in_depot_colors->ap_Append(true,&set_color,tmp,tmp);
		}
		//View-dependent texture mapping.
		/////////////////////////////////////////////////////////////////
		///////현재 Geometry만 테스트 중임...................///////////
		////////////////////////////////////////////////////////////////
		//update depot color.
		ccwac_Compute_Color_Weights_for_All_Cameras(in_set_of_pmats, &virtual_view, &color_weights);
		// view-dependent texture mapping.	
		vdtmm_View_Dependent_Texture_Mapping_on_Mesh_NEW(in_set_of_pmats, &in_set_of_color_palette[k/in_num_of_GOP], &in_vertex_color_indices_for_all_cam[k], &color_weights, &virtual_view, &in_set_of_mesh_triangle[k], in_depot_colors);				
		// set depots for display.			
		in_scr->g_id_Initialize_Depots(&in_set_of_depot_points[k],	in_depot_colors,	NULL,	NULL,	NULL,	NULL,	NULL);	
		/////////////////////////////////////////////////////////////////
		///////현재 Geometry만 테스트 중임...................///////////
		////////////////////////////////////////////////////////////////
		
		// display 3D object.
		in_scr->g_p_Plot(NULL, &in_set_of_depot_points[k], NULL, in_depot_colors, &in_set_of_mesh_triangle[k], NULL, NULL, NULL,NULL,NULL,true);
		
		// Object sequence player control.
		if(!init_flag){
			//calculate pause time for set frame rate to user-intended value.
			//elapsed_time_total=zz_sw.get_Get_Elapsed_Time(0);
			//num_disp_per_frame=(int)((1.0f/in_frame_rate - elapsed_time_total)/elapsed_time_total);		// sec

			num_disp_per_frame=10;
			/// 이거 없애면 display 들어갈 때 P matrix error 나면서 죽음.
			if(!Kv_Printf("User-intended fps: %f fps \n Time per display: %f ms\nNumber of display per frame: %d frames", 
				in_frame_rate, elapsed_time_total, num_disp_per_frame))		exit(0);

			uw.ss_Set_Size(in_scr, false, in_ww, in_hh);
			init_flag=true;
		}			
		// keyboard control.
		prev_idx=k;
		if(play_button)		printf("  [Play] # %d frame (%% %5.2f) \r", k+1, 100.0f*(float)(k+1)/(float)in_max_frame_num);
		if(_kbhit() == 1){
			char ch=_getch();
			// pause/play button.
			if(ch== 'p'){				
				if(!play_button)	{	play_button=true;		}
				else						{	play_button=false;		printf("  [Pause] # %d frame (%% %5.2f)\r", k+1, 100.0f*(float)(k+1)/(float)in_max_frame_num);		}
			}
			// back button (-5 frames).
			else if(ch== 'b'){		k=max(0, k-5);							/*printf("  [Back] # %d frame (%% %5.2f)\r", k+1, 100.0f*(float)(k+1)/(float)in_max_frame_num);*/		}
			// next button (+5 frames).
			else if(ch== 'n'){		k=min(in_max_frame_num-1, k+5);			/*printf("  [Next] # %d frame (%% %5.2f)\r", k+1, 100.0f*(float)(k+1)/(float)in_max_frame_num);*/		}
			// stop button.
			else if(ch=='s'){		printf("  [Stop] \r\n");	break;		}
			// exit player.
			else if(ch==27){		exit(0);	}
		}
		if(play_button)		frame_counter++;

		/// /////////////////////////////////////////////////////		
		if(k==in_max_frame_num-1)                                {play_button=false;k--;}
		else if(frame_counter%num_disp_per_frame || !play_button){k--;		            }
		/// /////////////////////////////////////////////////////
		Kv_Pause(10);
	}
	printf("\n");
}

//****************************************************
void Util_Display::gi_Get_Images(
	CKvGraph3D *in_scr,
	int in_ww, 
	int in_hh,
	int in_current_frame,
	int in_num_of_GOP,
	float in_frame_rate,
	CKvSet_of_Pmatrix3D *in_set_of_pmats,
	CKvDepot_of_Point3D *in_set_of_depot_points,
	CKvDepot_of_RgbaF *in_depot_colors,
	CKvMesh_of_Triangle *in_set_of_mesh_triangle,
	CKvVectorUcharRgb *in_set_of_color_palette,
	CKvSet_of_VectorInt *in_vertex_color_indices_for_all_cam,
	CKvSet_of_MatrixUcharRgb *out_generated_imgs,
	CKvSet_of_MatrixUchar    *out_generated_depth)
//****************************************************
{	
	//For texture mapping.
	LCKvUtility_for_Windows uw;
	CKvVectorFloat color_weights;
	CKvSet_of_RgbaF set_color;
	CKvPmatrix3D virtual_view;
	CKvMatrixFloat virtual_view_mat;
	CKvStopWatch zz_sw;
	CKvMatrixUcharRgb *p_generated_imgs;
	CKvMatrixUchar    *p_generated_depth;

	int num_cameras;
	bool play_button=true, init_flag=false;
	int prev_idx=-1, tmp;
	int num_disp_per_frame=0, frame_counter=0;
	float elapsed_time_total=0.0f, elapsed_time_disp=0.0f;
	float *p_virtual_view;

	
	//Initialization	
	virtual_view.cc_Create_Canonical(false);
	p_virtual_view   = virtual_view_mat.c_Create(3, 4, 0.0f)[0];
	num_cameras      = in_set_of_pmats->vs();
	p_generated_imgs = out_generated_imgs->c_Create(num_cameras);
	p_generated_depth= out_generated_depth->c_Create(num_cameras);

	in_scr->g_sp_Set_display_Parameters(true,  Kv_RgbF(255.0f, 255.0f, 255.0f), false, Kv_Hpoint3D(0.0, 0.0, 0.0, 1.0), 0.0f, 0.0f, false,true,0);		
	set_color.c_Create(in_set_of_depot_points[in_current_frame].ne_Number_of_Elements());
	set_color.se_Set_Element   (0, &Kv_RgbaF(1.0, 0.0, 0.0, 0.0));
	in_depot_colors->in_Initialize(in_set_of_depot_points[in_current_frame].ne_Number_of_Elements());
	in_depot_colors->ap_Append(true,&set_color,tmp,tmp);
	

	for(int k=0;k<num_cameras;k++){
		virtual_view.cp_Copy(in_set_of_pmats->gpe_Get_Pointer_of_Element(k));

		//View-dependent texture mapping.
		//update depot color.
		ccwac_Compute_Color_Weights_for_All_Cameras(in_set_of_pmats, &virtual_view, &color_weights);
		// view-dependent texture mapping.	
		vdtmm_View_Dependent_Texture_Mapping_on_Mesh_NEW(
			in_set_of_pmats, 
			&in_set_of_color_palette[in_current_frame/in_num_of_GOP],
			&in_vertex_color_indices_for_all_cam[in_current_frame],
			&color_weights,
			&virtual_view,
			&in_set_of_mesh_triangle[in_current_frame],
			in_depot_colors);				
		
		// set depots for display.			
		in_scr->g_id_Initialize_Depots(&in_set_of_depot_points[in_current_frame],in_depot_colors,NULL,NULL,NULL,NULL,NULL);	

		// display 3D object.
		p_generated_imgs[k] .c_Create(in_hh,in_ww);
		p_generated_depth[k].c_Create(in_hh,in_ww);

		in_scr->g_p_Plot(&virtual_view,&in_set_of_depot_points[in_current_frame],NULL,in_depot_colors,&in_set_of_mesh_triangle[in_current_frame],NULL,NULL,NULL,NULL,NULL,true);
		uw.ss_Set_Size(in_scr, false,in_ww,in_hh);
		Kv_Pause(100);
		in_scr->g_gcid_Get_Captured_Image_and_Depth_map(&p_generated_imgs[k],NULL,&p_generated_depth[k]);
	}
	printf("\n");
}



//****************************************************
void Util_Display::ccwac_Compute_Color_Weights_for_All_Cameras(
	CKvSet_of_Pmatrix3D *in_P_matrix_set,	
	CKvPmatrix3D *in_virtual_viewpoint,
	CKvVectorFloat *out_color_weights_for_all_cameras)
//****************************************************
{
	CKvPoint3D norm_vv, norm_tv;
	float *p_cw, p_norm_vv[3], p_norm_tv[3];
	int nc;
	int k; 
	float mag1, mag2, ip, gamma;

	/// //////////////////////////////
	gamma=30.0f;
	/// //////////////////////////////
	nc=in_P_matrix_set->vs();
	p_cw=out_color_weights_for_all_cameras->c_Create(nc, 0.0f);

	norm_vv=in_virtual_viewpoint->dvo_Direction_Vector_of_Optical_axis();		norm_vv.g_Get(p_norm_vv);

	//printf("[Color weights for all cameras]\n");
	for(k=0;k<nc;k++){
		norm_tv=in_P_matrix_set->gpe_Get_Pointer_of_Element(k)->dvo_Direction_Vector_of_Optical_axis();		
		norm_tv.g_Get(p_norm_tv);

		mag1=sqrt(pow(p_norm_tv[0], 2)+ pow(p_norm_tv[1], 2)+ pow(p_norm_tv[2], 2));
		mag2=sqrt(pow(p_norm_vv[0], 2)+ pow(p_norm_vv[1], 2)+ pow(p_norm_vv[2], 2));
		ip=p_norm_tv[0] * p_norm_vv[0] + p_norm_tv[1] * p_norm_vv[1] + p_norm_tv[2] * p_norm_vv[2];	ip=ip/(mag1*mag2);		
		p_cw[k]=pow((ip+1.0f)/2.0f, gamma);				

	}	

}

//****************************************************
void Util_Display::vdtmm_View_Dependent_Texture_Mapping_on_Mesh(
	//CKvDepot_of_Point3D *in_depot_point, 
	//CKvSet_of_MatrixUcharRgb *in_texture_set,
	CKvSet_of_Pmatrix3D *in_P_matrix_set,				
	CKvSet_of_VectorBool *in_per_mesh_visibility,
	CKvSet_of_VectorUcharRgb *in_vertex_colors_for_all_cameras,		
	CKvVectorFloat *in_color_weights_for_all_cameras,				
	CKvPmatrix3D *in_virtual_viewpoint,		//int in_view_index, //
	CKvMesh_of_Triangle *io_mesh_triangle,
	CKvDepot_of_RgbaF *io_depot_color)
//****************************************************
{
	// Using interpolation, knows triangle normal info. (Ver.1.3)
	CKvPoint3D point1, point2, point3;
	CKvPoint   point1_img, point2_img, point3_img;
	CKvVectorInt *idx_tri  ,*f_color_tri  ,*b_color_tri;
	CKvVectorInt *idx_tetra,*f_color_tetra,*b_color_tetra;
	CKvSet_of_Point3D t_point;

	double *t_color_p1p2p3;
	double t_color[4];
	int num_of_cam,num_of_points;
	int p,k;
	int *p_idx_tri  , *p_f_color_tri  , *p_b_color_tri;
	int *p_idx_tetra, *p_f_color_tetra, *p_b_color_tetra;
	int *t_idx,*t_f_color;
	int sz_tri, sz_tetra,sz_total;

	//Initialization
	io_mesh_triangle->gep_Get_Element_Pointers(0,idx_tri  ,f_color_tri  ,b_color_tri);
	io_mesh_triangle->gep_Get_Element_Pointers(1,idx_tetra,f_color_tetra,b_color_tetra);
	num_of_points   = io_depot_color->ne_Number_of_Elements();
	p_idx_tri       = idx_tri  ->vps(sz_tri);
	p_idx_tetra     = idx_tetra->vps(sz_tetra);
	sz_total = sz_tri+sz_tetra;

	//For triangle & tetragon meshes...
	p_f_color_tri   = f_color_tri  ->c_Create(sz_tri  ,0);
	p_b_color_tri   = b_color_tri  ->c_Create(sz_tri  ,0);
	p_f_color_tetra = f_color_tetra->c_Create(sz_tetra,0);
	p_b_color_tetra = b_color_tetra->c_Create(sz_tetra,0);

	//Memory allocation
	t_idx = new int[sz_total];
	t_f_color      = new int[sz_total];
	t_color_p1p2p3 = new double[num_of_points*4];							for(int i=0; i<num_of_points; i++){	t_color_p1p2p3[4*i]=0.0f;		t_color_p1p2p3[4*i+1]=1.0f;		t_color_p1p2p3[4*i+2]=0.0f;		t_color_p1p2p3[4*i+3]=0.0f;		}

	for(p=0;p<sz_tri  ;p++) t_idx[p]        = p_idx_tri  [p];
	for(p=0;p<sz_tetra;p++) t_idx[sz_tri+p] = p_idx_tetra[p];

	/// /////////////////////////////////////////////////////////
	CKvRgbaF w_color;
	bool *p_t_vis;
	unsigned char *p_t_colors;
	float *p_color_w;
	
	num_of_cam=in_P_matrix_set->vs();
	p_color_w=in_color_weights_for_all_cameras->vp();
	/// /////////////////////////////////////////////////////////

	//Find optimal view for each triangle by normal vector
	for(p=0;p<sz_total;p+=3){		

		//Get image pointer
		p_t_vis=in_per_mesh_visibility->gpe(p/3)->vp();

		for(k=p; k<p+3; k++){
			
			p_t_colors=in_vertex_colors_for_all_cameras->gpe(k)->vp();
			cvc_Compute_Vertex_Color(	p_t_vis, p_t_colors, p_color_w, num_of_cam, w_color);
			//Get colors of the projected points
			t_color_p1p2p3[4*t_idx[k]  ]  = w_color.r;
			t_color_p1p2p3[4*t_idx[k]+1]  = w_color.g;
			t_color_p1p2p3[4*t_idx[k]+2]  = w_color.b;
			t_color_p1p2p3[4*t_idx[k]+3]  = w_color.a;

			//if(w_color.r==1.0f && w_color.g==1.0f && w_color.b==1.0f)		if(!Kv_Printf("%d", k))		exit(0);

			//Update idx of depot of colors
			t_f_color[k]   = t_idx[k];
		}

	}

	//Update depot of colors
	for(p=0;p<num_of_points;p++){
		for(k=0; k<4; k++)	t_color[k] = t_color_p1p2p3[4*p+k];
		io_depot_color->se_Set_Element(p ,t_color);
	}

	//Copy to mesh_triangle
	for(p=0;p<sz_tri  ;p++){p_f_color_tri  [p] = p_b_color_tri  [p] = t_f_color[p];       }
	for(p=0;p<sz_tetra;p++){p_f_color_tetra[p] = p_b_color_tetra[p] = t_f_color[sz_tri+p];}

	delete []t_f_color;
	delete []t_color_p1p2p3;

}

//****************************************************
void Util_Display::vdtmm_View_Dependent_Texture_Mapping_on_Mesh_NEW(
	CKvSet_of_Pmatrix3D *in_P_matrix_set,				
	CKvVectorUcharRgb *in_color_palette,
	CKvSet_of_VectorInt *in_vertex_color_indices_for_all_cameras,		
	CKvVectorFloat *in_color_weights_for_all_cameras,				
	CKvPmatrix3D *in_virtual_viewpoint,		//int in_view_index, //
	CKvMesh_of_Triangle *io_mesh_triangle,
	CKvDepot_of_RgbaF *io_depot_color)
//****************************************************
{
	// Using interpolation, knows triangle normal info. (Ver.1.3)
	CKvPoint3D point1, point2, point3;
	CKvPoint   point1_img, point2_img, point3_img;
	CKvVectorInt *idx_tri  ,*f_color_tri  ,*b_color_tri;
	CKvVectorInt *idx_tetra,*f_color_tetra,*b_color_tetra;
	CKvSet_of_Point3D t_point;

	double t_color[4];
	int num_of_cam,num_of_points;
	int p,k, kk;
	int *p_idx_tri  , *p_f_color_tri  , *p_b_color_tri;
	int *p_idx_tetra, *p_f_color_tetra, *p_b_color_tetra;
	int sz_tri, sz_tetra,sz_total;
	int tmp_idx;

	//Initialization
	io_mesh_triangle->gep_Get_Element_Pointers(0,idx_tri  ,f_color_tri  ,b_color_tri);
	io_mesh_triangle->gep_Get_Element_Pointers(1,idx_tetra,f_color_tetra,b_color_tetra);
	num_of_points   = io_depot_color->ne_Number_of_Elements();
	p_idx_tri       = idx_tri  ->vps(sz_tri);
	p_idx_tetra     = idx_tetra->vps(sz_tetra);
	sz_total = sz_tri+sz_tetra;

	//For triangle & tetragon meshes...
	p_f_color_tri   = f_color_tri  ->c_Create(sz_tri  ,0);
	p_b_color_tri   = b_color_tri  ->c_Create(sz_tri  ,0);
	p_f_color_tetra = f_color_tetra->c_Create(sz_tetra,0);
	p_b_color_tetra = b_color_tetra->c_Create(sz_tetra,0);

	/// /////////////////////////////////////////////////////////
	CKvRgbaF w_color;
	bool *p_t_vis;
	int *p_color_indices;
	unsigned char *p_t_colors;
	float *p_color_w;
	
	num_of_cam=in_P_matrix_set->vs();
	p_color_w=in_color_weights_for_all_cameras->vp();
	/// /////////////////////////////////////////////////////////

	//Find optimal view for each triangle by normal vector
	for(p=0;p<sz_total;p+=3){	
		//Get image pointer
		//p_color_indices=in_vertex_color_indices_for_all_cameras->gpe()->vp();
		for(k=p; k<p+3; k++){
			if(k<sz_tri)	tmp_idx=p_idx_tri[k];
			else				tmp_idx=p_idx_tetra[k-sz_tri];

			p_color_indices=in_vertex_color_indices_for_all_cameras->gpe(tmp_idx)->vp();
			cvc_Compute_Vertex_Color_NEW(	p_color_indices, in_color_palette, p_color_w, num_of_cam, w_color);			

			//Get colors of the projected points
			t_color[0]  = w_color.r;	t_color[1]  = w_color.g;	t_color[2]  = w_color.b;	t_color[3]  = w_color.a;
			io_depot_color->se_Set_Element(tmp_idx ,t_color);

			//Update idx of depot of colors
			if(k<sz_tri)	p_f_color_tri[k]		  = p_b_color_tri[k]		  = tmp_idx;
			else			p_f_color_tetra[k-sz_tri] = p_b_color_tetra[k-sz_tri] = tmp_idx;
		}

	}
}

//****************************************************
void Util_Display::cvc_Compute_Vertex_Color(
	bool *in_vertex_visibility,
	unsigned char *in_vertex_colors,
	float *in_color_weights,
	int in_camera_number,
	CKvRgbaF &out_vertex_color)
//****************************************************
{
	int cn;
	float total_w, w_color[3];

	cn=in_camera_number;	total_w=0.0f;		for(int i=0; i<3; i++)	w_color[i]=0.0f;
	for(int i=0; i<cn; i++){	if(in_vertex_visibility[i])	total_w+=in_color_weights[i];		}
	for(int i=0; i<cn; i++){	
		if(in_vertex_visibility[i]){
			for(int n=0; n<3; n++){
				w_color[n]+=(in_color_weights[i]/total_w)*(float)in_vertex_colors[i+n*cn]/255.0f;
			}
		}
	}

	out_vertex_color.r=w_color[0];
	out_vertex_color.g=w_color[1];
	out_vertex_color.b=w_color[2];
	out_vertex_color.a=0.0f;

}

//****************************************************
void Util_Display::cvc_Compute_Vertex_Color_NEW(
	int *in_vertex_color_indices,
	CKvVectorUcharRgb *in_vertex_color_depot,
	float *in_color_weights,
	int in_camera_number,
	CKvRgbaF &out_vertex_color)
//****************************************************
{
	int cn;
	float total_w, w, w_color[3];

	cn=in_camera_number;	total_w=0.0f;		for(int i=0; i<3; i++)	w_color[i]=0.0f;
	for(int i=0; i<cn; i++){	if(in_vertex_color_indices[i]>=0)	total_w+=in_color_weights[i];		}
	for(int i=0; i<cn; i++){	
		if(in_vertex_color_indices[i]>=0){
			for(int n=0; n<3; n++){
				w=in_color_weights[i]/total_w;
				w_color[n]+=w*(float)(in_vertex_color_depot->vp()[3*in_vertex_color_indices[i]+n]);
			}
		}
	}

	out_vertex_color.r=w_color[0]/255.0f;
	out_vertex_color.g=w_color[1]/255.0f;
	out_vertex_color.b=w_color[2]/255.0f;
	out_vertex_color.a=0.0f;

}


///////////////////////////////////////////////////////////////////
/////////Get depth maps and silhouettes by using OpenGL////////////
//////////////////////////////////////////////////////////////////
//******************************************************************************
void Util_Display::gdi_Get_Depth_Images(
	CKvGraph3D *in_scr,
	CKvDepot_of_Point3D *in_depot_point,
	CKvDepot_of_RgbaF *in_depot_color,
	CKvMesh_of_Triangle *in_mesh_triagnle,
	CKvSet_of_Pmatrix3D *in_pmat,
	int in_width,
	int in_height,
	CKvSet_of_MatrixFloat *out_depth_map)
//******************************************************************************
{
	int i,sz;
	CKvPmatrix3D *p_pmat;
	CKvMatrixFloat *p_out;

	p_pmat = in_pmat->vps(sz);
	p_out  = out_depth_map->c_Create(sz);

	for(i=0;i<sz;i++){
		gdii_Get_Depth_Image_for_an_Image(
			in_scr,
			in_depot_point,
			in_depot_color,
			in_mesh_triagnle,	
			&p_pmat[i],			//CKvPmatrix3D *in_pmat,
			in_width,			//int in_width,
			in_height,			//int in_height,
			&p_out[i]);			//CKvMatrixFloat *out_depth_map);
	}	
	return;
}
//******************************************************************************
void Util_Display::gdi_Get_Depth_Images(
	CKvGraph3D *in_scr,
	CKvDepot_of_Point3D *in_depot_point,
	CKvDepot_of_RgbaF *in_depot_color,
	CKvMesh_of_Triangle *in_mesh_triagnle,
	CKvSet_of_Pmatrix3D *in_pmat,
	CKvSet_of_MatrixUcharRgb *in_imgs,
	CKvSet_of_MatrixFloat *out_depth_map)
//******************************************************************************
{
	int i,sz,ww,hh;
	CKvPmatrix3D *p_pmat;
	CKvMatrixFloat *p_out;
	CKvMatrixUcharRgb *p_img;

	//Initialization
	p_pmat = in_pmat->vps(sz);
	p_out  = out_depth_map->c_Create(sz);
	p_img  = in_imgs->vp();
	
	for(i=0;i<sz;i++){
		p_img[i].ms(ww,hh);
		gdii_Get_Depth_Image_for_an_Image(
			in_scr,
			in_depot_point,
			in_depot_color,
			in_mesh_triagnle,			
			&p_pmat[i],			//CKvPmatrix3D *in_pmat,
			ww,					//int in_width,
			hh,					//int in_height,
			&p_out[i]);			//CKvMatrixFloat *out_depth_map);
	}	
	return;
}
//******************************************************************************
void Util_Display::gdii_Get_Depth_Image_for_an_Image(
	CKvGraph3D *in_scr,
	CKvDepot_of_Point3D *in_depot_point,
	CKvDepot_of_RgbaF *in_depot_color,
	CKvMesh_of_Triangle *in_mesh_triagnle,
	CKvPmatrix3D *in_pmat,
	int in_width,
	int in_height,
	CKvMatrixFloat *out_depth_map)
//******************************************************************************
{
	//Plot mesh object
	pmo_Plot_Mesh_Object(in_scr,in_depot_point,in_depot_color,in_pmat,in_mesh_triagnle);
	zz_util_window.ss_Set_Size(in_scr,true,in_width,in_height); 
	Kv_Pause(10);
	in_scr->g_gcid_Get_Captured_Image_and_Depth_map(NULL,out_depth_map,NULL);  
	return;
}
//******************************************************************************
void Util_Display::gsp_Get_Silhouettes(
	CKvGraph3D *in_scr,
	CKvDepot_of_Point3D *in_dp,
	CKvDepot_of_RgbaF *in_dc,
	CKvMesh_of_Triangle *in_mesh_triangle,
	CKvSet_of_Pmatrix3D *in_pmat_set,
	int *in_ww_hh,
	int in_dilation,
	CKvSet_of_SdkCode *out_sdk_set)
//******************************************************************************
{
	CKvPmatrix3D *p_pmat_set;
	CKvSdkCode   *p_sdk_set;
	int number_camera, k;

	//Initialization
	p_pmat_set = in_pmat_set->vps(number_camera);
	p_sdk_set  = out_sdk_set->c_Create(number_camera);
	
	//Silhouette generation by multi-view projection
	for(k=0; k<number_camera; k++){
		gspi_Get_Silhouette_for_an_Image(
			in_scr,
			in_dp,
			in_dc,
			in_mesh_triangle,
			&p_pmat_set[k],
			in_ww_hh,
			in_dilation,
			&p_sdk_set[k]);
	}
}
//*******************************************************************************************
void Util_Display::gspi_Get_Silhouette_for_an_Image(
	CKvGraph3D *in_scr,
	CKvDepot_of_Point3D *in_dp,
	CKvDepot_of_RgbaF *in_dc,
	CKvMesh_of_Triangle *in_mesh_triangle,
	CKvPmatrix3D *in_pmat,
	int *in_ww_hh,	
	int in_dilation,
	CKvSdkCode *out_sdk_set)
//*******************************************************************************************
{
	CKvMatrixBool  B_img;
	CKvMatrixUchar d_img;

	B_img.c_Create(in_ww_hh[0], in_ww_hh[1], false);

	//Preparing 3D point object display
	pmo_Plot_Mesh_Object(in_scr,in_dp,in_dc,in_pmat,in_mesh_triangle);

	//zz_graph3D.g_sp_Set_display_Parameters(true, Kv_RgbF(255.0f, 255.0f, 255.0f));

	////Preparing 3D mesh object display
	//zz_graph3D.g_id_Initialize_Depots(
	//	in_dp, //CKvDepot_of_Point3D *in_set_of_points_or_NULL,
	//	in_dc, //CKvDepot_of_RgbaF *in_set_of_colors_or_NULL,
	//	NULL, //&zz_depot_image, //CKvDepot_of_String *in_set_of_images_or_NULL,
	//	NULL, //&zz_depot_image_point, //CKvDepot_of_Point *in_set_of_image_points_or_NULL,
	//	NULL, //&depot_Pmat, //CKvDepot_of_Pmatrix3D *in_set_of_P_matrices_or_NULL,
	//	NULL, //CKvDepot_of_Font *in_set_of_fonts_or_NULL,
	//	NULL); //CKvDepot_of_String *in_set_of_text_strings_or_NULL);
	//
	//zz_graph3D.g_p_Plot(in_pmat,in_dp,NULL,in_dc,in_mesh_triangle,NULL,NULL,NULL,NULL,NULL,true);

	zz_util_window.ss_Set_Size(in_scr,true,in_ww_hh[0],in_ww_hh[1]);
	
	Kv_Pause(100); 
	
	in_scr->g_gcid_Get_Captured_Image_and_Depth_map(NULL,NULL,&d_img);
	zz_util_import.mucb_Matrix_Uchar_to_Bool(&d_img,1,&B_img);

	zz_runset.i_Import(&B_img);
	if(in_dilation>0) zz_runset.d_Dilation(&zz_runset,in_dilation,in_dilation); //2일 경우 삐져나오는 경우도 발생..
	out_sdk_set->i_Import(&zz_runset, true);
}
//******************************************************************************
void Util_Display::ss_Set_Size(
	CKvGraph3D *in_scr,
	int in_ww,
	int in_hh)
//******************************************************************************
{
	zz_util_window.ss_Set_Size(in_scr,true,in_ww,in_hh); 
}
