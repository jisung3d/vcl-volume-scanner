#include "../../stdafx.h"
//#include "VCL_Inhand_Object_Scanning.h"
//#include "VCL_Inhand_Object_ScanningDlg.h"
//#include "_Utility_VCL_3DO.h"
#include "../_VCL_2014_3DO.h"

Util_VCL_3DO::Util_VCL_3DO()
{
}
Util_VCL_3DO::~Util_VCL_3DO()
{
}
//*******************************************************************************************
void Util_VCL_3DO::gd2dpt_Get_Depth_and_2D_Position_of_a_Triangle(
	float *in_pt, 
	float *in_p_mat,
	float *out_position,
	float *out_depth)
//*******************************************************************************************
{
	int k;
	for(k=0;k<3;k++){gd2dpp_Get_Depth_and_2D_Position_of_a_Point(&in_pt[3*k],in_p_mat,&out_position[2*k],out_depth[k]);}
}
//*******************************************************************************************
void Util_VCL_3DO::gd2dpp_Get_Depth_and_2D_Position_of_a_Point(
	float *in_pt, 
	float *in_p_mat,
	float *out_position,
	float &out_depth)
//*******************************************************************************************
{
	double x,y,w;
	double detM,m_3;

	//x= PX
	x = in_p_mat[0]*in_pt[0]+in_p_mat[1]*in_pt[1]+in_p_mat[2] *in_pt[2]+in_p_mat[3] *1;
	y = in_p_mat[4]*in_pt[0]+in_p_mat[5]*in_pt[1]+in_p_mat[6] *in_pt[2]+in_p_mat[7] *1;
	w = in_p_mat[8]*in_pt[0]+in_p_mat[9]*in_pt[1]+in_p_mat[10]*in_pt[2]+in_p_mat[11]*1;
	
	out_position[0] = (float)(x/w);
	out_position[1] = (float)(y/w);

	//Get depth
	detM =	in_p_mat[0]*(in_p_mat[5]*in_p_mat[10] - in_p_mat[6]*in_p_mat[9])-
		    in_p_mat[1]*(in_p_mat[4]*in_p_mat[10] - in_p_mat[6]*in_p_mat[8])+
			in_p_mat[2]*(in_p_mat[4]*in_p_mat[9 ] - in_p_mat[5]*in_p_mat[8]);
	m_3  =  sqrt(in_p_mat[8]*in_p_mat[8] + in_p_mat[9]*in_p_mat[9] + in_p_mat[10]*in_p_mat[10]);

	if(detM>0)	out_depth =    w/m_3;
	else		out_depth = -1*w/m_3;
}
//*******************************************************************************************
void Util_VCL_3DO::gd2dpt_Get_Depth_and_2D_Position_of_a_Triangle_S_Series_Data(
	float *in_pt, 
	float *in_p_mat,
	float *out_position,
	float *out_depth)
//*******************************************************************************************
{
	int k;
	for(k=0;k<3;k++){gd2dpp_Get_Depth_and_2D_Position_of_a_Point_S_Series_Data(&in_pt[3*k],in_p_mat,&out_position[2*k],out_depth[k]);}
}
//*******************************************************************************************
void Util_VCL_3DO::gd2dpp_Get_Depth_and_2D_Position_of_a_Point_S_Series_Data(
	float *in_pt, 
	float *in_p_mat,
	float *out_position,
	float &out_depth)
//*******************************************************************************************
{
	double x,y,w;
	double detM,m_3;

	//x= PX
	x = in_p_mat[0]*in_pt[0]+in_p_mat[1]*in_pt[1]+in_p_mat[2] *in_pt[2]+in_p_mat[3] *1;
	y = in_p_mat[4]*in_pt[0]+in_p_mat[5]*in_pt[1]+in_p_mat[6] *in_pt[2]+in_p_mat[7] *1;
	w = in_p_mat[8]*in_pt[0]+in_p_mat[9]*in_pt[1]+in_p_mat[10]*in_pt[2]+in_p_mat[11]*1;
	
	out_position[0] = x/w;
	out_position[1] = y/w;

	//Get depth
	detM =	in_p_mat[0]*(in_p_mat[5]*in_p_mat[10] - in_p_mat[6]*in_p_mat[9])-
		    in_p_mat[1]*(in_p_mat[4]*in_p_mat[10] - in_p_mat[6]*in_p_mat[8])+
			in_p_mat[2]*(in_p_mat[4]*in_p_mat[9 ] - in_p_mat[5]*in_p_mat[8]);
	m_3  =  sqrt(in_p_mat[8]*in_p_mat[8] + in_p_mat[9]*in_p_mat[9] + in_p_mat[10]*in_p_mat[10]);

	/// ///////////////////////////////////////////////////////
	/// S-series data의 camera matrix Z-axis가 뒤집어져 있는 듯...
	if(detM<0)		out_depth = w/m_3;
	else					out_depth = -1.0*w/m_3;
	/// ///////////////////////////////////////////////////////
}
//*******************************************************************************************
void Util_VCL_3DO::id_Interpolate_Depth(
	float *in_position,
	float *in_depth,
	int in_x,
	int in_y,
	float &out_depth)
//*******************************************************************************************
{
	float weight[3];
	bool is_inside;
	cw_Compute_Weight(in_position,in_x,in_y,weight,is_inside);
	if(is_inside){out_depth = weight[0]*in_depth[0] + weight[1]*in_depth[1] + weight[2]*in_depth[2];}
	else		 {out_depth = 255.0f;}
	/*if(weight[0]+weight[1]+weight[2]!=1.0f){
		if(is_inside) printf("weight %f!\n",weight[0]+weight[1]+weight[2]);
		out_depth = 255.0f;
	}
	else{
		if(!is_inside)	printf("????\n");
		out_depth = weight[0]*in_depth[0] + weight[1]*in_depth[1] + weight[2]*in_depth[2];
	}*/
}
//*******************************************************************************************
void Util_VCL_3DO::cw_Compute_Weight(
	float *in_position,
	int in_x,
	int in_y,
	float *out_weight,
	bool &out_is_inside)
//*******************************************************************************************
{
	float area_triangle,area_A,area_B,area_C;
	float position[9];

	position[0] = in_position[0]; position[1] = in_position[1]; position[2] =0.0f;
	position[3] = in_position[2]; position[4] = in_position[3]; position[5] =0.0f;
	position[6] = in_position[4]; position[7] = in_position[5]; position[8] =0.0f;

	mat_Measure_Area_of_a_Triangle(position,area_triangle);

	position[0] = (float)in_x;	  position[1] = (float)in_y;	position[2] =0.0f;
	position[3] = in_position[2]; position[4] = in_position[3]; position[5] =0.0f;
	position[6] = in_position[4]; position[7] = in_position[5]; position[8] =0.0f;

	mat_Measure_Area_of_a_Triangle(position,area_A);

	position[0] = in_position[0]; position[1] = in_position[1]; position[2] =0.0f;
	position[3] = (float)in_x;	  position[4] = (float)in_y;	position[5] =0.0f;
	position[6] = in_position[4]; position[7] = in_position[5]; position[8] =0.0f;
	
	mat_Measure_Area_of_a_Triangle(position,area_B);

	position[0] = in_position[0]; position[1] = in_position[1]; position[2] =0.0f;
	position[3] = in_position[2]; position[4] = in_position[3]; position[5] =0.0f;
	position[6] = (float)in_x;	  position[7] = (float)in_y;	position[8] =0.0f;

	mat_Measure_Area_of_a_Triangle(position,area_C);

	out_weight[0] = area_A/area_triangle;
	out_weight[1] = area_B/area_triangle;
	out_weight[2] = area_C/area_triangle;

	/*Kv_Printf("x:%d y:%d\n%f %f %f %f %f %f\n%f %f %f total%f\n%f",
		in_x,in_y,
		in_position[0],in_position[1],in_position[2],in_position[3],in_position[4],in_position[5],
		area_A,area_B,area_C,area_triangle,
		out_weight[0]+out_weight[1]+out_weight[2]);*/
	/*Kv_Printf("x:5 y:5\n%f %f %f total%f\n%f",
		area_A,area_B,area_C,area_triangle,
		out_weight[0]+out_weight[1]+out_weight[2]);*/

	double AB[3],AP[3],BC[3],BP[3],CA[3],CP[3];

	AB[0] = in_position[2] - in_position[0];
	AB[1] = in_position[3] - in_position[1];

	AP[0] = in_x           - in_position[0];
	AP[1] = in_y           - in_position[1];

	BC[0] = in_position[4] - in_position[2];
	BC[1] = in_position[5] - in_position[3];

	BP[0] = in_x           - in_position[2];
	BP[1] = in_y           - in_position[3];

	CA[0] = in_position[0] - in_position[4];
	CA[1] = in_position[1] - in_position[5];

	CP[0] = in_x           - in_position[4];
	CP[1] = in_y           - in_position[5];

	double det1 = AB[0]*AP[1]-AB[1]*AP[0];
	double det2 = BC[0]*BP[1]-BC[1]*BP[0];
	double det3 = CA[0]*CP[1]-CA[1]*CP[0];

	if( (det1>0 && det2>0 && det3>0) || (det1<0 && det2<0 && det3 <0)) out_is_inside = true;
	else															   out_is_inside = false;
}
//*******************************************************************************************
void Util_VCL_3DO::mat_Measure_Area_of_a_Triangle(
	float *in_coordinates_of_3_points,
	float &area_or_a_triangle)
//*******************************************************************************************
{
	float AB[3],AC[3],cross[3];

	AB[0] = in_coordinates_of_3_points[3] - in_coordinates_of_3_points[0];
	AB[1] = in_coordinates_of_3_points[4] - in_coordinates_of_3_points[1];
	AB[2] = in_coordinates_of_3_points[5] - in_coordinates_of_3_points[2];
	
	AC[0] = in_coordinates_of_3_points[6] - in_coordinates_of_3_points[0];
	AC[1] = in_coordinates_of_3_points[7] - in_coordinates_of_3_points[1];
	AC[2] = in_coordinates_of_3_points[8] - in_coordinates_of_3_points[2];

	cross[0] = AB[1]*AC[2] - AB[2]*AC[1];
	cross[1] = AB[2]*AC[0] - AB[0]*AC[2];
	cross[2] = AB[0]*AC[1] - AB[1]*AC[0];

	area_or_a_triangle = 0.5* sqrt(cross[0]*cross[0] + cross[1]*cross[1] +cross[2]*cross[2]);
}

//*******************************************************************************************
void Util_VCL_3DO::gbi_Get_Block_for_Interpolation(
	float *in_position,
	int *out_LTRB)
//*******************************************************************************************
{
	//in_position: 0:x 1:y 2:x 3:y 4:x 5:y
	//LTRB: 0:LT.x 1:LT.y 2:RB.x 3:RB.y
	int k;
	out_LTRB[0] = 100000.0f;
	out_LTRB[1] = 100000.0f;
	out_LTRB[2] = -100000.0f;
	out_LTRB[3] = -100000.0f;

	for(k=0;k<3;k++){
		if(in_position[2*k  ]<out_LTRB[0]) out_LTRB[0] = nint(in_position[2*k  ]  );
		if(in_position[2*k  ]>out_LTRB[2]) out_LTRB[2] = nint(in_position[2*k  ]+1);
		if(in_position[2*k+1]<out_LTRB[1]) out_LTRB[1] = nint(in_position[2*k+1]  );
		if(in_position[2*k+1]>out_LTRB[3]) out_LTRB[3] = nint(in_position[2*k+1]+1);
	}
}

//******************************************************************************
void Util_VCL_3DO::gac_Get_Angle_of_Camera(int *out_angle_of_camera)
//******************************************************************************
{
	//Angle of camera 0: longitude(경도) 1:latitude(위도)
	out_angle_of_camera[0 ] =0;		out_angle_of_camera[1 ] =90;	

	out_angle_of_camera[2 ] = 18;	out_angle_of_camera[3 ] = 58;
	out_angle_of_camera[4 ] = 90;	out_angle_of_camera[5 ] = 58;
	out_angle_of_camera[6 ] = 162;	out_angle_of_camera[7 ] = 58;
	out_angle_of_camera[8 ] =-126;  out_angle_of_camera[9 ] = 58;	
	out_angle_of_camera[10] =-54;	out_angle_of_camera[11] = 58;	

	out_angle_of_camera[12] = 54;	out_angle_of_camera[13] = 32;
	out_angle_of_camera[14] = 126;	out_angle_of_camera[15] = 32;
	out_angle_of_camera[16] =-162;	out_angle_of_camera[17] = 32;	
	out_angle_of_camera[18] =-90;   out_angle_of_camera[19] =32;	
	out_angle_of_camera[20] =-18;	out_angle_of_camera[21] =32;	

	out_angle_of_camera[22] = 18;	out_angle_of_camera[23] = 27;
	out_angle_of_camera[24] = 90;	out_angle_of_camera[25] = 27;
	out_angle_of_camera[26] = 162;  out_angle_of_camera[27] = 27;	
	out_angle_of_camera[28] =-126;  out_angle_of_camera[29] = 27;	
	out_angle_of_camera[30] =-54;	out_angle_of_camera[31] =27;	
	
	out_angle_of_camera[32] = 36;	out_angle_of_camera[33] = 0;
	out_angle_of_camera[34] = 72;	out_angle_of_camera[35] = 0;
	out_angle_of_camera[36] = 108;	out_angle_of_camera[37] = 0;
	out_angle_of_camera[38] = 144;	out_angle_of_camera[39] = 0;
	out_angle_of_camera[40] = 180;	out_angle_of_camera[41] = 0;	
	out_angle_of_camera[42] = -144; out_angle_of_camera[43] = 0;	
	out_angle_of_camera[44] =-108;  out_angle_of_camera[45] = 0;	
	out_angle_of_camera[46] =-72;	out_angle_of_camera[47] = 0;	
	out_angle_of_camera[48] =-36;	out_angle_of_camera[49] =0;		
	out_angle_of_camera[50] = 0;	out_angle_of_camera[51] =0;		

	out_angle_of_camera[52] =18;	out_angle_of_camera[53] =-27;	
	out_angle_of_camera[54] =90;	out_angle_of_camera[55] =-27;	
	out_angle_of_camera[56] =162;	out_angle_of_camera[57] =-27;	
	out_angle_of_camera[58] =-126;	out_angle_of_camera[59] =-27;	
	out_angle_of_camera[60] =-54;	out_angle_of_camera[61] =-27;	

	out_angle_of_camera[62] =54;	out_angle_of_camera[63] =-32;	
	out_angle_of_camera[64] =126;	out_angle_of_camera[65] =-32;	
	out_angle_of_camera[66] =-162;	out_angle_of_camera[67] =-32;	
	out_angle_of_camera[68] =-90;	out_angle_of_camera[69] =-32;	
	out_angle_of_camera[70] =-18;	out_angle_of_camera[71] =-32;	

	out_angle_of_camera[72] =18;	out_angle_of_camera[73] =-58;	
	out_angle_of_camera[74] =90;	out_angle_of_camera[75] =-58;	
	out_angle_of_camera[76] =162;	out_angle_of_camera[77] =-58;	
	out_angle_of_camera[78] =-126;	out_angle_of_camera[79] =-58;	
	out_angle_of_camera[80] =-54;	out_angle_of_camera[81] =-58;	

	out_angle_of_camera[82] =0;		out_angle_of_camera[83] =-90;
}	
//*******************************************************************************************
void Util_VCL_3DO::cc_Create_Cameras(
	int *in_angle_of_camera,
	int in_num_camera,
	int *in_ww_hh,
	float in_reciprocal_of_f,
	float in_scale,
	CKvSet_of_Pmatrix3D *out_p_mat_set)
//*******************************************************************************************
{
	int k;
	CKvPmatrix3D *p_mat;
	//Create Orthographic Cameras
	p_mat = out_p_mat_set->c_Create(in_num_camera);

	for(k=0;k<in_num_camera;k++){
		cci_Create_Camera_for_an_Image(
			&in_angle_of_camera[2*k],
			in_ww_hh,
			in_reciprocal_of_f,
			in_scale,
			&p_mat[k]);
	}
}
//*******************************************************************************************
void Util_VCL_3DO::cci_Create_Camera_for_an_Image(
	int *in_angle_of_camera,
	int *in_ww_hh,
	float in_reciprocal_of_f,
	float in_scale,
	CKvPmatrix3D *out_p_mat)
//*******************************************************************************************
{
	//P matrix generation 
	//angle of camera 0: longitude(경도) 1:latitude(위도)
	LCKvUtility_for_Pmatrix3D util_pmat3d;
	CKvMatrix rotation_temp, p_matrix;
	CKvPoint3D t_y_axis, t_optic_c, dv_t_ref_pt, dummy_point3D,optic_c,ref_pt,t_ref_pt,ori_y_axis;
	CKvPoint pp;
	double angle_x, angle_y, angle_z, **p_rotation_temp;
	bool dummy;

	//Initialization	
	ref_pt.x	  = 0.0;	
	ref_pt.y	  = 0.0;	
	//ref_pt.z	  = 5.0f;
	ref_pt.z	  = 10000000.0f;
	ori_y_axis.x  = 0.0;	
	ori_y_axis.y  = 1.0;	
	ori_y_axis.z  = 0.0;	
	pp.x		  = in_ww_hh[0]/2;
	pp.y		  = in_ww_hh[1]/2;	
	rotation_temp.c_Create(3, 3, 0.0f);
	
	//P matrix generation
	p_rotation_temp = rotation_temp.mp();
	p_rotation_temp[0][0] = p_rotation_temp[1][1] = p_rotation_temp[2][2] = 1.0f;
	p_rotation_temp[0][1] = p_rotation_temp[0][2] = p_rotation_temp[1][0] =
	p_rotation_temp[1][2] = p_rotation_temp[2][0] = p_rotation_temp[2][1] = 0.0f;
	
	//Create rotation matrix
	util_pmat3d.u_crm_Create_Rotation_Matrix(
		(in_angle_of_camera[1]/180.0f)*PI, 
		(in_angle_of_camera[0]/180.0f)*PI, 
		0.0, 
		&rotation_temp);
				
	//Rotate reference point
	t_ref_pt.x = p_rotation_temp[0][0] *ref_pt.x + p_rotation_temp[0][1] *ref_pt.y + p_rotation_temp[0][2] *ref_pt.z;
	t_ref_pt.y = p_rotation_temp[1][0] *ref_pt.x + p_rotation_temp[1][1] *ref_pt.y + p_rotation_temp[1][2] *ref_pt.z;
	t_ref_pt.z = p_rotation_temp[2][0] *ref_pt.x + p_rotation_temp[2][1] *ref_pt.y + p_rotation_temp[2][2] *ref_pt.z;
		
	//Rotate the y axis of image plane
	t_y_axis.x = p_rotation_temp[0][0] *ori_y_axis.x +p_rotation_temp[0][1] *ori_y_axis.y +p_rotation_temp[0][2] *ori_y_axis.z;	
	t_y_axis.y = p_rotation_temp[1][0] *ori_y_axis.x +p_rotation_temp[1][1] *ori_y_axis.y +p_rotation_temp[1][2] *ori_y_axis.z;	
	t_y_axis.z = p_rotation_temp[2][0] *ori_y_axis.x +p_rotation_temp[2][1] *ori_y_axis.y +p_rotation_temp[2][2] *ori_y_axis.z;	
		
	//Calculate direction vector of rotated reference point 
	dv_t_ref_pt.x = -1*t_ref_pt.x; 
	dv_t_ref_pt.y = -1*t_ref_pt.y;  
	dv_t_ref_pt.z = -1*t_ref_pt.z;	
			
	//Get angles between image plane and origin 
	util_pmat3d.u_ga_Get_Angles(dv_t_ref_pt,t_y_axis,dummy_point3D,dummy_point3D,angle_x,angle_y,angle_z);
			
	//Determine optical_center_or_a_point_on_principal_line_for_affine_mode  
	if(in_reciprocal_of_f ==0.0f){optic_c.x = optic_c.y = optic_c.z = 0.0f;}
	else						 {optic_c.x = t_ref_pt.x; optic_c.y = t_ref_pt.y; optic_c.z = t_ref_pt.z;}

	//Create projection matrix
	util_pmat3d.u_cpm_Create_Projection_Matrix(
		in_reciprocal_of_f, //in_reciprocal_of_focal_length_or_zero_for_affine_mode,
		optic_c,		    //&in_optical_center_or_a_point_on_principal_line_for_affine_mode,
		angle_x,		    //in_angle_in_radian_x
		angle_y,		    //in_angle_in_radian_y
		angle_z,		    //in_angle_in_radian_z
		in_scale,		    //in_scale_in_pixels_per_meter_xx
		in_scale,		    //in_scale_in_pixels_per_meter_yy
		0.0f,			    //in_scale_in_pixels_per_meter_xy_for_skew
		pp,				    //&in_principal_point,
		dummy,			    //&out_affine_mode,
		&p_matrix);		    //*out_P_matrix
	out_p_mat->i_Import(&p_matrix);
}






//Get point-wise normal vector
//******************************************************************************
void Util_VCL_3DO::fnv_find_normal_vector(
	Util_Mesh_Generation *in_initial_volume_sp,
	CKvSet_of_Point3D *in_set_of_points,
	CKvSet_of_Point3D *out_set_of_normal_vectors)
//******************************************************************************
{
	int num_of_pts, passing[3], sum_pass, comb_pass, **NN, i, kk, m_index, first_i, second_i, ii, jj;
	CKvPoint3D *PP, *out_NV, seg1, seg2, seg3, out_cp, sum_cp;
	CKvMatrixInt neighbor_points;
	bool is_exist;
	CKvVector mag_vec;	double *m, magnitude;
	CKvSet_of_Point3D out_vec, seg_vec;	CKvPoint3D *o, *seg;


	PP = in_set_of_points->vps(num_of_pts);
	out_NV = out_set_of_normal_vectors->c_Create(num_of_pts);

	for(i=0; i<num_of_pts; i++)
	{
		in_initial_volume_sp->gnpt_Get_list_of_Neighbor_Points_Total(
			i, //int in_index_of_a_point,
			passing, //int *out_number_of_passings,
			&neighbor_points); //CKvMatrixInt *out_list_of_previous_and_next_point_indices_in_2xN_matrix)
		sum_pass = passing[0]+passing[1]+passing[2];
		NN = neighbor_points.mp();
		if(sum_pass==0)
		{
			out_NV[i].x = out_NV[i].y = out_NV[i].z = 0.0;
		}
		else if(sum_pass==1) 
		{
			out_NV[i].x = out_NV[i].y = out_NV[i].z = 0.0;
		}
		else if(sum_pass==2)
		{
			is_exist = true;
			if(passing[0]==1 && passing[1]==1)
			{
				seg1 = PP[NN[1][0]]-PP[NN[0][0]];	// X - YZ
				if(PP[NN[0][1]].x>PP[NN[1][1]].x)          
					seg2 = PP[NN[1][1]]-PP[NN[0][1]];	// Y - ZX
				else if(PP[NN[0][1]].x<PP[NN[1][1]].x)
					seg2 = PP[NN[0][1]]-PP[NN[1][1]];
				else is_exist=false;
			}
			else if(passing[1]==1 && passing[2]==1)
			{
				seg1 = PP[NN[1][0]]-PP[NN[0][0]];	// Y - ZX
				if(PP[NN[0][1]].y>PP[NN[1][1]].y)
					seg2 = PP[NN[1][1]]-PP[NN[0][1]];	// Z - XY
				else if(PP[NN[0][1]].y<PP[NN[1][1]].y)
					seg2 = PP[NN[0][1]]-PP[NN[1][1]];
				else is_exist = false;
			}
			else if(passing[2]==1 && passing[0]==1)
			{
				seg1 = PP[NN[1][1]]-PP[NN[0][1]];	// Z - XY
				if(PP[NN[0][0]].z>PP[NN[1][0]].z)
					seg2 = PP[NN[1][0]]-PP[NN[0][0]];	// X - YZ
				else if((PP[NN[0][0]].z<PP[NN[1][0]].z))
					seg2 = PP[NN[0][0]]-PP[NN[1][0]];
				else is_exist=false;
			}
			if(is_exist)
			{
				cp_cross_product(seg1,seg2,out_NV[i],magnitude);	
			}
			else
			{
				out_NV[i].x = out_NV[i].y = out_NV[i].z = 0.0;
			}
		}
		else if(sum_pass==3)
		{
			seg = seg_vec.c_Create(sum_pass);
			m = mag_vec.c_Create(sum_pass);
			o = out_vec.c_Create(sum_pass);

			if((NN[0][0]==NN[0][1] && NN[1][0]==NN[1][1]) || (NN[0][0]==NN[1][1] && NN[1][0]==NN[0][1])) //X==Y
			{
				seg1 = PP[NN[1][1]]-PP[NN[0][1]];	//YZ
				if(PP[NN[0][2]].y>PP[NN[1][2]].y)
				{
					seg2 = PP[NN[1][2]]-PP[NN[0][2]];
					cp_cross_product(seg1,seg2,out_NV[i],magnitude);
				}
				else if(PP[NN[0][2]].y<PP[NN[1][2]].y)
				{
					seg2 = PP[NN[0][2]]-PP[NN[1][2]];
					cp_cross_product(seg1,seg2,out_NV[i],magnitude);
				}
				else
				{
					//if(!Kv_Printf("... pass==3[1].. zero ")) exit(0);
					out_NV[i].x = out_NV[i].y = out_NV[i].z = 0.0;
				}
			}
			else if((NN[0][0]==NN[0][2] && NN[1][0]==NN[1][2]) || (NN[0][0]==NN[1][2] && NN[1][0]==NN[0][2])) //X==Z
			{
				seg1 = PP[NN[1][0]]-PP[NN[0][0]];	//XY
				if(PP[NN[0][1]].x>PP[NN[1][1]].x)
				{
					seg2 = PP[NN[1][1]]-PP[NN[0][1]];
					cp_cross_product(seg1,seg2,out_NV[i],magnitude);
				}
				else if(PP[NN[0][1]].x<PP[NN[1][1]].x)
				{
					seg2 = PP[NN[0][1]]-PP[NN[1][1]];
					cp_cross_product(seg1,seg2,out_NV[i],magnitude);
				}
				else
				{
					out_NV[i].x = out_NV[i].y = out_NV[i].z = 0.0;
				}
			}
			else if((NN[0][1]==NN[0][2] && NN[1][1]==NN[1][2]) || (NN[0][1]==NN[1][2] && NN[1][1]==NN[0][2])) //Y==Z
			{
				seg1 = PP[NN[1][0]]-PP[NN[0][0]];	//XY
				if(PP[NN[0][1]].x>PP[NN[1][1]].x)
				{
					seg2 = PP[NN[1][1]]-PP[NN[0][1]];
					cp_cross_product(seg1,seg2,out_NV[i],magnitude);
				}
				else if(PP[NN[0][1]].x<PP[NN[1][1]].x)
				{
					seg2 = PP[NN[0][1]]-PP[NN[1][1]];
					cp_cross_product(seg1,seg2,out_NV[i],magnitude);
				}
				else
				{
					out_NV[i].x = out_NV[i].y = out_NV[i].z = 0.0;
				}
			}
			else
			{
				seg1 = PP[NN[1][0]]-PP[NN[0][0]];
				seg2 = PP[NN[1][1]]-PP[NN[0][1]];
				seg3 = PP[NN[1][2]]-PP[NN[0][2]];

				if(PP[NN[0][1]].x<PP[NN[1][1]].x && passing[0]==1 && passing[1]==1)
				{
					seg2 *= -1.0;
					cp_cross_product(seg1,seg2,out_cp,magnitude);
				}
				else if(PP[NN[0][1]].x>PP[NN[1][1]].x && passing[0]==1 && passing[1]==1)
				{
					cp_cross_product(seg1,seg2,out_cp,magnitude);
				}
				else if(PP[NN[0][1]].x==PP[NN[1][1]].x && passing[0]==1 && passing[1]==1)
				{
					if(PP[NN[0][2]].y<PP[NN[1][2]].y)
					{
						seg3 *= -1.0;
						cp_cross_product(seg2,seg3,out_cp,magnitude);
					}
					else if(PP[NN[0][2]].y>PP[NN[1][2]].y)
						cp_cross_product(seg2,seg3,out_cp,magnitude);
					else if(PP[NN[0][2]].y==PP[NN[1][2]].y)
					{
						if(PP[NN[0][0]].z<PP[NN[1][0]].z)
							seg1 *= -1.0;
						cp_cross_product(seg3,seg1,out_cp,magnitude);
					}
				}

				cp_cross_product(seg1,seg2,o[0],m[0]);
				cp_cross_product(seg2,seg3,o[1],m[1]);
				cp_cross_product(seg3,seg1,o[2],m[2]);

				m_index = 0;
				sum_cp.x = sum_cp.y = sum_cp.z = 0;
				for(kk=0; kk<sum_pass; kk++)
				{
					if(out_cp.x*o[kk].x<0) o[kk].x *= -1.;
					if(out_cp.y*o[kk].y<0) o[kk].y *= -1.;
					if(out_cp.z*o[kk].z<0) o[kk].z *= -1.;
					if(m[kk]!=0.0)
					{
						m_index++;
						sum_cp += o[kk];
					}
				}

				if(m_index!=0)
					out_NV[i] = sum_cp / m_index;
			}
		}
		else
		{
			comb_pass = sum_pass*(sum_pass-1)/2;
			m = mag_vec.c_Create(comb_pass);
			o = out_vec.c_Create(comb_pass);
			seg = seg_vec.c_Create(sum_pass);

			for(kk=0; kk<sum_pass; kk++)
			{
				seg[kk] = PP[NN[1][kk]]-PP[NN[0][kk]];
			}

			is_exist = true;
			first_i = 0;
			second_i = passing[0];
			if(passing[0]>0) // start X
			{
				for(kk=0; kk<passing[0]; kk++)
				{
					if(NN[0][first_i]==NN[1][first_i]) first_i++;
					else break;
				}
				if(first_i>=passing[0])
				{
					first_i = passing[0];
					goto loop;
				}

				for(kk=passing[0]; kk<sum_pass; kk++)
				{
					if(NN[0][second_i]==NN[0][first_i] || NN[1][second_i]==NN[1][first_i] 
					|| NN[0][second_i]==NN[1][first_i] || NN[1][second_i]==NN[0][first_i] 
					|| NN[0][second_i]==NN[1][second_i]) second_i++;
					else break;
				}	
				if(second_i>=sum_pass)
				{
					second_i = passing[0];
					for(kk=passing[0]; kk<sum_pass; kk++)
					{
						if((NN[0][second_i]==NN[0][first_i] && NN[1][second_i]==NN[1][first_i]) 
							|| (NN[0][second_i]==NN[1][first_i] && NN[1][second_i]==NN[0][first_i]) 
							|| (NN[0][second_i]==NN[1][second_i])) second_i++;
						else break;							
					}
				}

				if(second_i<passing[0]+passing[1]) // second: Y
				{
					seg1 = PP[NN[1][first_i]]-PP[NN[0][first_i]];  // XY

					if(PP[NN[0][second_i]].x>PP[NN[1][second_i]].x)
					{
						seg2 = PP[NN[1][second_i]]-PP[NN[0][second_i]];
						cp_cross_product(seg1,seg2,out_cp,magnitude);
					}
					else if(PP[NN[0][second_i]].x<PP[NN[1][second_i]].x)
					{
						seg2 = PP[NN[0][second_i]]-PP[NN[1][second_i]];
						cp_cross_product(seg1,seg2,out_cp,magnitude);
					}
					else is_exist=false;
				}
				else if(second_i<sum_pass)// second: Z
				{
					seg1 = PP[NN[1][second_i]]-PP[NN[0][second_i]];  // ZX

					if(PP[NN[0][first_i]].z>PP[NN[1][first_i]].z)
					{
						seg2 = PP[NN[1][first_i]]-PP[NN[0][first_i]];
						cp_cross_product(seg1,seg2,out_cp,magnitude);
					}
					else if(PP[NN[0][first_i]].z<PP[NN[1][first_i]].z)
					{
						seg2 = PP[NN[0][first_i]]-PP[NN[1][first_i]];
						cp_cross_product(seg1,seg2,out_cp,magnitude);
					}
					else is_exist=false;
				}
				else is_exist=false;
			}
			else // start Y
			{
loop:			for(kk=passing[0]; kk<passing[1]; kk++)
				{
					if(NN[0][first_i]==NN[1][first_i]) first_i++;
					else break;
				}
				if(first_i>=passing[1]+passing[0]) go_error("3"); //error...

				second_i = passing[1]+passing[0];
				for(kk=passing[1]+passing[0]; kk<sum_pass; kk++)
				{
					if(NN[0][second_i]==NN[0][first_i] || NN[1][second_i]==NN[1][first_i] 
					|| NN[0][second_i]==NN[1][first_i] || NN[1][second_i]==NN[0][first_i] 
					|| NN[0][second_i]==NN[1][second_i]) second_i++;
					else break;
				}	
				if(second_i>=sum_pass)
				{
					second_i = passing[1]+passing[0];
					for(kk=passing[1]+passing[0]; kk<sum_pass; kk++)
					{
						if((NN[0][second_i]==NN[0][first_i] && NN[1][second_i]==NN[1][first_i]) 
							|| (NN[0][second_i]==NN[1][first_i] && NN[1][second_i]==NN[0][first_i]) 
							|| (NN[0][second_i]==NN[1][second_i])) second_i++;
						else break;
					}
				}

				if(second_i<sum_pass)
				{
					seg1 = PP[NN[1][first_i]]-PP[NN[0][first_i]];  // YZ
					if(PP[NN[0][second_i]].y>PP[NN[1][second_i]].y)
					{
						seg2 = PP[NN[1][second_i]]-PP[NN[0][second_i]];
						cp_cross_product(seg1,seg2,out_cp,magnitude);
					}
					else if(PP[NN[0][second_i]].y<PP[NN[1][second_i]].y)
					{
						seg2 = PP[NN[0][second_i]]-PP[NN[1][second_i]];
						cp_cross_product(seg1,seg2,out_cp,magnitude);
					}
					else is_exist=false;
				}
				else is_exist=false;
			}


			if(is_exist)
			{
				m_index = jj = 0;
				sum_cp.x = sum_cp.y = sum_cp.z = 0;
				for(kk=0; kk<(sum_pass-1); kk++)
				{
					for(ii=kk+1; ii<sum_pass; ii++)
					{
						cp_cross_product(seg[kk],seg[ii],o[jj],m[jj]);
						if(out_cp.x*o[jj].x<0)	o[jj].x *= -1.0;
						if(out_cp.y*o[jj].y<0)	o[jj].y *= -1.0;
						if(out_cp.z*o[jj].z<0)	o[jj].z *= -1.0;
						if(m[jj]!=0.0)
						{
							m_index++;
							sum_cp += o[jj];
						}
						jj++;
					}
				}

				if(m_index!=0)
					out_NV[i] = sum_cp / m_index;
			}
			else
			{
				out_NV[i].x = out_NV[i].y = out_NV[i].z = 0.0;
			}
		}
	}

	return;
error:
	zpme("Find_Normal_Vector");		
}
//******************************************************************************
void Util_VCL_3DO::cp_cross_product(
	CKvPoint3D in1, 
	CKvPoint3D in2, 
	CKvPoint3D &out_normalized, 
	double &magnitude)
//******************************************************************************
{
	CKvPoint3D tmp;

	tmp.x = in1.y*in2.z-in1.z*in2.y;
	tmp.y = in1.z*in2.x-in1.x*in2.z;
	tmp.z = in1.x*in2.y-in1.y*in2.x;

	magnitude = sqrt(tmp.x*tmp.x+tmp.y*tmp.y+tmp.z*tmp.z);

	if(magnitude==0)
	{
		out_normalized.x = 0;
		out_normalized.y = 0;
		out_normalized.z = 0;
	}
	else
	{
		out_normalized.x = tmp.x/magnitude;
		out_normalized.y = tmp.y/magnitude;
		out_normalized.z = tmp.z/magnitude;
	}
}
//////////////////////////////////////////////////////////////////////////
//
//	For Grobal function 2012.11.19
//						made by h.d.
//
//		Add _Color_Conversion_Rgb_to_Yuv : 2012.11.30
//
//******************************************************************************
void Util_VCL_3DO::_Cross_Product(
	CKvVector *in_data1,		
	CKvVector *in_data2,		
	CKvVector *out_data)		// (size=3)
//******************************************************************************
{
	int sz;
	double *p_1, *p_2, *p_o;
	p_1=in_data1->vps(sz);
	p_2=in_data2->vp();
	p_o=out_data->c_Create(sz);

	p_o[0]=p_1[1]*p_2[2]-p_1[2]*p_2[1];
	p_o[1]=p_1[2]*p_2[0]-p_1[0]*p_2[2];
	p_o[2]=p_1[0]*p_2[1]-p_1[1]*p_2[0];
}
//******************************************************************************
void Util_VCL_3DO::_Color_Conversion_Rgb_to_Yuv(
	CKvMatrixUcharRgb *in_img,
	CKvSet_of_Matrix *out_img) // Y, U, V
//******************************************************************************
{
	//	R		[1	0		1.402	][Y]
	//	G	=	[1	-0.344	-0.714	][U]
	//	B		[1	1.772	0		][V]

	int i, j, w, h;
	CKvMatrix H, inv_H;
	CKvMatrix *p_out_img;
	LCKvUtility_for_Linear_Equation util_le;
	unsigned char **p_r, **p_g, **p_b;
	double **p_H, **p_y, **p_u, **p_v;

	p_H=H.c_Create(3,3);
	p_H[0][0]=1;	p_H[0][1]=0;		p_H[0][2]=1.402;
	p_H[1][0]=1;	p_H[1][1]=-0.344;	p_H[1][2]=-0.714;
	p_H[2][0]=1;	p_H[2][1]=1.772;	p_H[2][2]=0;

	if(!util_le.im_Inverse_Matrix(1, &H, &inv_H)) Kv_Printf("im_Inverse_Matrix error");
	p_H=inv_H.mp();

	p_r=in_img->mps(p_g, p_b, w, h);
	p_out_img=out_img->c_Create(3);
	p_y=p_out_img[0].c_Create(h, w);
	p_u=p_out_img[1].c_Create(h, w);
	p_v=p_out_img[2].c_Create(h, w);

	for(j=0; j<h; j++)
	{
		for(i=0; i<w; i++)
		{
			p_y[j][i]=p_H[0][0]*p_r[j][i]+p_H[0][1]*p_g[j][i]+p_H[0][2]*p_b[j][i];
			p_u[j][i]=p_H[1][0]*p_r[j][i]+p_H[1][1]*p_g[j][i]+p_H[1][2]*p_b[j][i];
			p_v[j][i]=p_H[2][0]*p_r[j][i]+p_H[2][1]*p_g[j][i]+p_H[2][2]*p_b[j][i];
		}
	}
	return;
}
//******************************************************************************
void Util_VCL_3DO::_Color_Conversion_Rgb_to_Yuv(
	CKvSet_of_MatrixUcharRgb *in_img,
	CKvSet2d_of_Matrix *out_img) // height=3: Y, U, V; width=num_img
//******************************************************************************
{
	//	R		[1	0		1.402	][Y]
	//	G	=	[1	-0.344	-0.714	][U]
	//	B		[1	1.772	0		][V]

	int i, j, k, w, h, num_img;
	CKvMatrix H, inv_H;
	CKvMatrixUcharRgb *p_in;
	CKvMatrix **p_out_img;
	LCKvUtility_for_Linear_Equation util_le;
	unsigned char **p_r, **p_g, **p_b;
	double **p_H, **p_y, **p_u, **p_v;

	p_H=H.c_Create(3,3);
	p_H[0][0]=1;	p_H[0][1]=0;		p_H[0][2]=1.402;
	p_H[1][0]=1;	p_H[1][1]=-0.344;	p_H[1][2]=-0.714;
	p_H[2][0]=1;	p_H[2][1]=1.772;	p_H[2][2]=0;

	if(!util_le.im_Inverse_Matrix(1, &H, &inv_H)) Kv_Printf("im_Inverse_Matrix error");
	p_H=inv_H.mp();

	p_in=in_img->vps(num_img);
	p_out_img=out_img->c_Create(3, num_img);
	for(k=0; k<num_img; k++)
	{
		p_r=p_in[k].mps(p_g, p_b, w, h);
		p_y=p_out_img[0][k].c_Create(h, w);
		p_u=p_out_img[1][k].c_Create(h, w);
		p_v=p_out_img[2][k].c_Create(h, w);

		for(j=0; j<h; j++)
		{
			for(i=0; i<w; i++)
			{
				p_y[j][i]=p_H[0][0]*p_r[j][i]+p_H[0][1]*p_g[j][i]+p_H[0][2]*p_b[j][i];
				p_u[j][i]=p_H[1][0]*p_r[j][i]+p_H[1][1]*p_g[j][i]+p_H[1][2]*p_b[j][i];
				p_v[j][i]=p_H[2][0]*p_r[j][i]+p_H[2][1]*p_g[j][i]+p_H[2][2]*p_b[j][i];
			}
		}
	}
	return;
}




//******************************************************************************
void Util_VCL_3DO::gdizb_Get_Depth_Images_using_Z_Buffering(
	CKvDepot_of_Point3D *in_depot_point,
	CKvDepot_of_RgbaF *in_depot_color,
	CKvMesh_of_Triangle *in_mesh_triagnle,
	CKvSet_of_Pmatrix3D *in_pmat,
	CKvSet_of_MatrixUcharRgb *in_imgs,
	CKvSet_of_MatrixFloat *out_depth_map,
	CKvSet_of_MatrixInt *out_buffer)
//******************************************************************************
{
	int i,sz,ww,hh;
	CKvPmatrix3D *p_pmat;
	CKvMatrixFloat *p_out;
	CKvMatrixUcharRgb *p_img;
	CKvVectorInt idx_mesh;
	CKvMatrixInt *p_buffer;

	//Initialization
	p_pmat   = in_pmat->vps(sz);
	p_img    = in_imgs->vp();
	p_out    = out_depth_map->c_Create(sz);
	p_buffer = out_buffer->c_Create(sz);
	

	//Make mesh
	gim_Get_Indices_of_Mesh(in_mesh_triagnle,0,&idx_mesh);

	
	for(i=0;i<sz;i++){
		p_img[i].ms(ww,hh);
		p_out[i].c_Create(hh,ww,255.0f);
		p_buffer[i].c_Create(hh,ww,-1);

		gdmzbi_Generate_Depth_Map_using_Z_buffering_for_an_Image(
			in_depot_point, 
			&idx_mesh,
			&p_pmat[i],
			&p_out[i],
			&p_buffer[i]);
	}	
	return;
}
//*******************************************************************************************
void Util_VCL_3DO::gim_Get_Indices_of_Mesh(
	CKvMesh_of_Triangle *in_mesh_triangle,
	int in_0_pt_1_front_color_2_back_color,
	CKvVectorInt *out_idx_mesh)
//*******************************************************************************************
{
	CKvSet_of_VectorInt element_tri,element_tetra;
	CKvVectorInt *p_element_tri,*p_element_tetra;
	int *pp_element_tri, *pp_element_tetra,total_vertices_tri,total_vertices_tetra,*p_idx_mesh,k;

	in_mesh_triangle->ge_Get_Element(0, &element_tri);
	in_mesh_triangle->ge_Get_Element(1, &element_tetra);
	p_element_tri   = element_tri  .vp();
	p_element_tetra = element_tetra.vp();
	pp_element_tri  = p_element_tri  [in_0_pt_1_front_color_2_back_color].vps(total_vertices_tri);
	pp_element_tetra= p_element_tetra[in_0_pt_1_front_color_2_back_color].vps(total_vertices_tetra);
	p_idx_mesh      = out_idx_mesh->c_Create(total_vertices_tri+total_vertices_tetra);

	//Merge index of mesh points...
	for(k=0;k<total_vertices_tri;k++)  {p_idx_mesh[k]					 = pp_element_tri[k];  }
	for(k=0;k<total_vertices_tetra;k++){p_idx_mesh[total_vertices_tri+k] = pp_element_tetra[k];}
}
//*******************************************************************************************
void Util_VCL_3DO::gdmzbi_Generate_Depth_Map_using_Z_buffering_for_an_Image(
	CKvDepot_of_Point3D *in_depot_point, 
	CKvVectorInt *in_idx_mesh,
	CKvPmatrix3D *in_p_mat,
	CKvMatrixFloat *out_d_map,
	CKvMatrixInt *out_f_buffer)
//*******************************************************************************************
{
	int *p_idx_mesh, total_vertices, num_triangle,k;
	CKvSet_of_Point3D pt;
	CKvPoint3D *p_pt;
	float t_pt[9],p_mat[12],t_depth[3],t_position[6];

	//Initialization
	p_idx_mesh     = in_idx_mesh->vps(total_vertices);
	num_triangle   = total_vertices/3;
	in_depot_point->e_Export(&pt);
	in_p_mat      ->e_Export(true,p_mat);
	p_pt		   = pt.vp();
	
	//if(!Kv_Printf("p_pt: %d, total_vertices: %d", in_depot_point->ne_Number_of_Elements(), total_vertices))	exit(0);

	//Perform z-buffering 
	for(k=0;k<num_triangle;k++){
		t_pt[0] = (float)p_pt[p_idx_mesh[3*k]  ].x;
		t_pt[1] = (float)p_pt[p_idx_mesh[3*k]  ].y;
		t_pt[2] = (float)p_pt[p_idx_mesh[3*k]  ].z;
		t_pt[3] = (float)p_pt[p_idx_mesh[3*k+1]].x;
		t_pt[4] = (float)p_pt[p_idx_mesh[3*k+1]].y;
		t_pt[5] = (float)p_pt[p_idx_mesh[3*k+1]].z;
		t_pt[6] = (float)p_pt[p_idx_mesh[3*k+2]].x;
		t_pt[7] = (float)p_pt[p_idx_mesh[3*k+2]].y;
		t_pt[8] = (float)p_pt[p_idx_mesh[3*k+2]].z;

		gd2dpt_Get_Depth_and_2D_Position_of_a_Triangle(t_pt,p_mat,t_position,t_depth);
		ub_Update_Buffer(t_position,t_depth,k,out_d_map,out_f_buffer);
	}

	// Remove invalid depth value.
	int ww, hh;
	float *p_out_d;
	p_out_d = out_d_map->mps(ww, hh)[0];
	
	for(int i = 0; i<ww*hh; i++){
		if(p_out_d[i] < 0.0f || p_out_d[i] > 5.0f)	p_out_d[i] = 0.0f;
	}
	

	//float **p_d_map;
	//int i,j,ww,hh;

	//int **p_f_buffer;

	//p_d_map    = out_d_map->mps(ww,hh);
	//p_f_buffer = out_f_buffer->mp();

	//float max_value,min_value;
	//max_value = -100000.0f;
	//min_value = 100000.0f;

	//for(j=0;j<hh;j++){
	//	for(i=0;i<ww;i++){
	//		if(p_f_buffer[j][i] == -1)continue;
	//		if(p_d_map[j][i]>max_value) max_value = p_d_map[j][i];
	//		if(p_d_map[j][i]<min_value) min_value = p_d_map[j][i];
	//	}
	//}

	//for(j=0;j<hh;j++){
	//	for(i=0;i<ww;i++){
	//		if(p_f_buffer[j][i] == -1)continue;
	//		p_d_map[j][i] = 255.0f*(p_d_map[j][i]-min_value)/(float)(max_value-min_value);
	//	}
	//}

	//CKvScreen aaa;
	//aaa.s_d_Display(1.0f,0.0f,out_d_map);
	//Kv_Printf("it works!!!");

}

//*******************************************************************************************
void Util_VCL_3DO::gdmzbibe_Generate_Depth_Map_using_Z_buffering_for_an_Image_Boundary_Erosion(
	CKvDepot_of_Point3D *in_depot_point, 
	CKvVectorInt *in_idx_mesh,
	CKvPmatrix3D *in_p_mat,
	CKvMatrixFloat *out_d_map,
	CKvMatrixInt *out_f_buffer)
//*******************************************************************************************
{
	int *p_idx_mesh, **p_out_f_buffer, **p_tmp_f_buffer;
	int total_vertices, num_triangle,i,j,k,m,ww,hh,thick;
	bool break_flag;
	CKvSet_of_Point3D pt;
	CKvMatrixInt tmp_f_buffer;
	CKvPoint3D *p_pt;
	float t_pt[9],p_mat[12],t_depth[3],t_position[6];

	//Initialization
	p_idx_mesh     = in_idx_mesh->vps(total_vertices);
	num_triangle   = total_vertices/3;
	in_depot_point->e_Export(&pt);
	in_p_mat      ->e_Export(true,p_mat);
	p_pt		   = pt.vp();
	
	//Perform z-buffering 
	for(k=0;k<num_triangle;k++){
		t_pt[0] = (float)p_pt[p_idx_mesh[3*k]  ].x;
		t_pt[1] = (float)p_pt[p_idx_mesh[3*k]  ].y;
		t_pt[2] = (float)p_pt[p_idx_mesh[3*k]  ].z;
		t_pt[3] = (float)p_pt[p_idx_mesh[3*k+1]].x;
		t_pt[4] = (float)p_pt[p_idx_mesh[3*k+1]].y;
		t_pt[5] = (float)p_pt[p_idx_mesh[3*k+1]].z;
		t_pt[6] = (float)p_pt[p_idx_mesh[3*k+2]].x;
		t_pt[7] = (float)p_pt[p_idx_mesh[3*k+2]].y;
		t_pt[8] = (float)p_pt[p_idx_mesh[3*k+2]].z;

		gd2dpt_Get_Depth_and_2D_Position_of_a_Triangle(t_pt,p_mat,t_position,t_depth);
		ub_Update_Buffer(t_position,t_depth,k,out_d_map,out_f_buffer);
	}

	// Erode depth indices.
	thick=4;
	p_out_f_buffer=out_f_buffer->mps(ww, hh);
	p_tmp_f_buffer=tmp_f_buffer.cp_Copy(out_f_buffer);
	for(j=thick; j<hh-thick; j++){	
		for(i=thick; i<ww-thick; i++){
			
			if(p_out_f_buffer[j][i]>=0){

				break_flag=false;

				for(m=-thick; m<=thick; m++){	
					for(k=-thick; k<=thick; k++){
						if(p_tmp_f_buffer[j+m][i+k]<0){
							p_out_f_buffer[j][i]=-p_out_f_buffer[j][i]-1;		// erosion flag.
							break_flag=true;
							break;
						}
					}
					if(break_flag)		break;
				}

			}

		}
	}

	//float **p_d_map;
	//int i,j,ww,hh;

	//int **p_f_buffer;

	//p_d_map    = out_d_map->mps(ww,hh);
	//p_f_buffer = out_f_buffer->mp();

	//float max_value,min_value;
	//max_value = -100000.0f;
	//min_value = 100000.0f;

	//for(j=0;j<hh;j++){
	//	for(i=0;i<ww;i++){
	//		if(p_f_buffer[j][i] == -1)continue;
	//		if(p_d_map[j][i]>max_value) max_value = p_d_map[j][i];
	//		if(p_d_map[j][i]<min_value) min_value = p_d_map[j][i];
	//	}
	//}

	//for(j=0;j<hh;j++){
	//	for(i=0;i<ww;i++){
	//		if(p_f_buffer[j][i] == -1)continue;
	//		p_d_map[j][i] = 255.0f*(p_d_map[j][i]-min_value)/(float)(max_value-min_value);
	//	}
	//}

	//CKvScreen aaa;
	//aaa.s_d_Display(1.0f,0.0f,out_d_map);
	//Kv_Printf("it works!!!");

}

//*******************************************************************************************
void Util_VCL_3DO::gszbi_Generat_Silhouette_using_Z_buffering_for_an_Image(
	CKvDepot_of_Point3D *in_depot_point, 
	CKvVectorInt *in_idx_mesh,
	CKvPmatrix3D *in_p_mat,
	CKvMatrixBool *out_silhouette)
//*******************************************************************************************
{
	int *p_idx_mesh, total_vertices, num_triangle,k;
	CKvSet_of_Point3D pt;
	CKvPoint3D *p_pt;
	float t_pt[9],p_mat[12],t_depth[3],t_position[6];

	//Initialization
	p_idx_mesh     = in_idx_mesh->vps(total_vertices);
	num_triangle   = total_vertices/3;
	in_depot_point->e_Export(&pt);
	in_p_mat      ->e_Export(true,p_mat);
	p_pt		   = pt.vp();
	
	//Perform z-buffering 
	for(k=0;k<num_triangle;k++){
		t_pt[0] = (float)p_pt[p_idx_mesh[3*k]  ].x;
		t_pt[1] = (float)p_pt[p_idx_mesh[3*k]  ].y;
		t_pt[2] = (float)p_pt[p_idx_mesh[3*k]  ].z;
		t_pt[3] = (float)p_pt[p_idx_mesh[3*k+1]].x;
		t_pt[4] = (float)p_pt[p_idx_mesh[3*k+1]].y;
		t_pt[5] = (float)p_pt[p_idx_mesh[3*k+1]].z;
		t_pt[6] = (float)p_pt[p_idx_mesh[3*k+2]].x;
		t_pt[7] = (float)p_pt[p_idx_mesh[3*k+2]].y;
		t_pt[8] = (float)p_pt[p_idx_mesh[3*k+2]].z;

		gd2dpt_Get_Depth_and_2D_Position_of_a_Triangle(t_pt,p_mat,t_position,t_depth);
		ub_Update_Buffer(t_position,t_depth,k,out_silhouette);
	}
}

//*******************************************************************************************
void Util_VCL_3DO::gdmzbi_Generate_Depth_Map_using_Z_buffering_for_an_Image_S_Series_Data(
	CKvDepot_of_Point3D *in_depot_point, 
	CKvVectorInt *in_idx_mesh,
	CKvPmatrix3D *in_p_mat,
	CKvMatrixFloat *out_d_map,
	CKvMatrixInt *out_f_buffer)
//*******************************************************************************************
{
	int *p_idx_mesh, total_vertices, num_triangle,k;
	CKvSet_of_Point3D pt;
	CKvPoint3D *p_pt;
	float t_pt[9],p_mat[12],t_depth[3],t_position[6];

	//Initialization
	p_idx_mesh     = in_idx_mesh->vps(total_vertices);
	num_triangle   = total_vertices/3;
	in_depot_point->e_Export(&pt);
	in_p_mat      ->e_Export(true,p_mat);
	p_pt		   = pt.vp();

	//Perform z-buffering 
	for(k=0;k<num_triangle;k++){

		t_pt[0] = (float)p_pt[p_idx_mesh[3*k]  ].x;
		t_pt[1] = (float)p_pt[p_idx_mesh[3*k]  ].y;
		t_pt[2] = (float)p_pt[p_idx_mesh[3*k]  ].z;
		t_pt[3] = (float)p_pt[p_idx_mesh[3*k+1]].x;
		t_pt[4] = (float)p_pt[p_idx_mesh[3*k+1]].y;
		t_pt[5] = (float)p_pt[p_idx_mesh[3*k+1]].z;
		t_pt[6] = (float)p_pt[p_idx_mesh[3*k+2]].x;
		t_pt[7] = (float)p_pt[p_idx_mesh[3*k+2]].y;
		t_pt[8] = (float)p_pt[p_idx_mesh[3*k+2]].z;

		gd2dpt_Get_Depth_and_2D_Position_of_a_Triangle_S_Series_Data(t_pt,p_mat,t_position,t_depth);
		ub_Update_Buffer(t_position,t_depth,k,out_d_map,out_f_buffer);

	}

	/// ////////////////////////////////////////////////////////////
 	float **p_d_map;
 	int i,j,ww,hh; 	 
 	int **p_f_buffer;
 
	float max_value,min_value;
	max_value = -1000000.0f;
	min_value = 1000000.0f;

 	p_d_map    = out_d_map->mps(ww,hh);
 	p_f_buffer = out_f_buffer->mp();

 	for(j=0;j<hh;j++){
 	 	for(i=0;i<ww;i++){
 	 		if(p_f_buffer[j][i] == -1)		continue;
 	 		if(p_d_map[j][i]>max_value) max_value = p_d_map[j][i];
 	 		if(p_d_map[j][i]<min_value) min_value = p_d_map[j][i];
 	 	}
 	}
 	 
 	for(j=0;j<hh;j++){ 	for(i=0;i<ww;i++){
		if(p_f_buffer[j][i] == -1 || p_d_map[j][i]==255.0f)	p_d_map[j][i]=max_value*1.3f;	 	 		
 	}}

	//if(!Kv_Printf("Max depth: %f, min depth: %f", max_value, min_value))		exit(0);
	/// ////////////////////////////////////////////////////////////
// 	float **p_d_map;
// 	int i,j,ww,hh;
// 
// 	int **p_f_buffer;
// 
// 	p_d_map    = out_d_map->mps(ww,hh);
// 	p_f_buffer = out_f_buffer->mp();
// 
// 	float max_value,min_value;
// 	max_value = -100000.0f;
// 	min_value = 100000.0f;
// 
// 	for(j=0;j<hh;j++){
// 		for(i=0;i<ww;i++){
// 			if(p_f_buffer[j][i] == -1)continue;
// 			if(p_d_map[j][i]>max_value) max_value = p_d_map[j][i];
// 			if(p_d_map[j][i]<min_value) min_value = p_d_map[j][i];
// 		}
// 	}
// 
// 	for(j=0;j<hh;j++){
// 		for(i=0;i<ww;i++){
// 			if(p_f_buffer[j][i] == -1)continue;
// 			p_d_map[j][i] = 255.0f*(p_d_map[j][i]-min_value)/(float)(max_value-min_value);
// 		}
// 	}

// 	CKvScreen aaa;
// 	dmfpc_Display_Matrix_Float_using_Pseudo_Color(&aaa, *out_d_map, true);
// 	//aaa.s_d_Display(1.0f,0.0f,out_d_map);
// 	if(!Kv_Printf("it works!!!")) exit(0);
}


//*******************************************************************************************
void Util_VCL_3DO::ub_Update_Buffer(
	float *in_position,
	float *in_depth,
	int in_idx_mesh,
	CKvMatrixFloat *io_d_map,
	CKvMatrixInt *io_f_buffer)
//*******************************************************************************************
{
	//Depth interpolation 영역 찾기 
	//투영된 영역의 depth값 비교
	//비교하여 더 가까우면 값 치환 
	//frame buffer에 triangle index 넣음 
	//in_position: 0:x 1:y 2:x 3:y 4:x 5:y
	//LTRB: 0:LT.x 1:LT.y 2:RB.x 3:RB.y
	float t_depth,**p_d_map;
	int LTRB[4],j,i,**p_f_buffer;
	int x,y,k;
	/// ///////////////////////////////////////
	int ww, hh;
	/// ///////////////////////////////////////

	p_d_map   = io_d_map->mps(ww, hh);
	p_f_buffer = io_f_buffer->mp();

	//Apply depth directly
	for(k=0;k<3;k++){
		// bounding position.
		x = min(max((int)in_position[2*k], 0), ww-1); 
		y = min(max((int)in_position[2*k+1], 0), hh-1);
		
		if(p_d_map[y][x]> in_depth[k]){
			p_d_map[y][x]    = in_depth[k];
			p_f_buffer[y][x] = in_idx_mesh;
		}
	}
	
	//Interpolate  depth 
	gbi_Get_Block_for_Interpolation(in_position,LTRB);
	// bounding position.
	LTRB[0]=min(max(LTRB[0], 0), ww-1);		LTRB[1]=min(max(LTRB[1], 0), hh-1);		
	LTRB[2]=min(max(LTRB[2], 0), ww-1);		LTRB[3]=min(max(LTRB[3], 0), hh-1);

	for(j=LTRB[1];j<LTRB[3];j++){
		for(i=LTRB[0];i<LTRB[2];i++){
			id_Interpolate_Depth(in_position,in_depth,i,j,t_depth);	
			if(p_d_map[j][i]>t_depth){
				p_d_map   [j][i] = t_depth;
				p_f_buffer[j][i] = in_idx_mesh;
			}
		}
	}
}

//*******************************************************************************************
void Util_VCL_3DO::ub_Update_Buffer(
	float *in_position,
	float *in_depth,
	int in_idx_mesh,
	CKvMatrixBool *io_silhouette)
//*******************************************************************************************
{
	//Depth interpolation 영역 찾기 
	//투영된 영역의 depth값 비교
	//비교하여 더 가까우면 값 치환 
	//frame buffer에 triangle index 넣음 
	//in_position: 0:x 1:y 2:x 3:y 4:x 5:y
	//LTRB: 0:LT.x 1:LT.y 2:RB.x 3:RB.y
	bool **p_sil;
	int LTRB[4],j,i;
	int x,y,k;
	/// ///////////////////////////////////////
	int ww, hh;
	/// ///////////////////////////////////////
	p_sil   = io_silhouette->mps(ww, hh);
	
	//Apply depth directly
	for(k=0;k<3;k++){
		// bounding position.
		x = min(max((int)in_position[2*k], 0), ww-1); 
		y = min(max((int)in_position[2*k+1], 0), hh-1);		
		
		p_sil[y][x]    = true;		
	}	
	//Interpolate  depth 
	gbi_Get_Block_for_Interpolation(in_position,LTRB);
	// bounding position.
	LTRB[0]=min(max(LTRB[0], 0), ww-1);		LTRB[1]=min(max(LTRB[1], 0), hh-1);		
	LTRB[2]=min(max(LTRB[2], 0), ww-1);		LTRB[3]=min(max(LTRB[3], 0), hh-1);

	for(j=LTRB[1];j<LTRB[3];j++){
		for(i=LTRB[0];i<LTRB[2];i++){
			if(!p_sil[j][i]){		p_sil[j][i] = true;	}
		}
	}
}


//******************************************************************************
void Util_VCL_3DO::_Get_Depth_Image_and_visibility(
	CKvDoCubeShort *in,
	CKvPmatrix3D *in_pmat,
	int in_width,
	int in_height,
	int in_id_pmat,
	float in_th_for_visibility,
	CKvVolumeBool *in_voxel,
	CKvMatrixBool *io_visibility, // width=num_point, height=visible:true, invisible:false
	CKvMatrixFloat *out_depth_map)
//******************************************************************************
{
	//int i,j,k, w,h,d, count;
	//CKvPoint3D p3d;
	//CKvPoint p2d;
	//CKvDepot_of_Point3D depot_point;
	//CKvDepot_of_RgbaF depot_color;
	//CKvMesh_of_Triangle mesh_triangle;

	//bool ***p_v, **p_visible;
	//double dx,dy,dz;
	//float depth_value;

	//LCKvUtility_for_Windows util_window;

	////Make mesh
	//mm_Make_Meshes(in,&depot_point,&depot_color,&mesh_triangle);
	////Plot mesh object
	//pmo_Plot_Mesh_Object(&depot_point,&depot_color,in_pmat,&mesh_triangle);

	//util_window.ss_Set_Size(&zz_graph3D,true,in_width,in_height); 
	//Kv_Pause(100);
	//zz_graph3D.g_gcid_Get_Captured_Image_and_Depth_map(NULL,out_depth_map,NULL);  

	//p_visible=io_visibility->mp();
	//p_v=in_voxel->tps(w,h,d);
	//dx=2./w;
	//dy=2./h;
	//dz=2./d;
	//count=0;
	//for(k=0;k<d;k++)
	//{
	//	for(j=0;j<h;j++)
	//	{
	//		for(i=0;i<w;i++)
	//		{
	//			if(p_v[k][j][i])
	//			{
	//				p3d.x=-1+i*dx;
	//				p3d.y=-1+j*dy;
	//				p3d.z=-1+k*dz;
	//				zz_graph3D.g_gdp_Get_Depth_of_a_Point(p3d, depth_value);
	//				in_pmat->tp_Transform_Point(p3d, p2d);
	//				if((depth_value<=out_depth_map->ge_Get_Element((float)p2d.x, (float)p2d.y)) || out_depth_map->ge_Get_Element((float)p2d.x, (float)p2d.y)==1)
	//				{
	//					p_visible[in_id_pmat][count]=true;
	//					count++;
	//				}
	//			}
	//		}
	//	}
	//}
	//return;
}
//******************************************************************************
void Util_VCL_3DO::_Get_Depth_Image_and_visibility(
	CKvDoCubeShort *in,
	CKvSet_of_Pmatrix3D *in_pmat,
	int in_width,
	int in_height,
	float in_th_for_visibility,
	CKvVolumeBool *in_voxel,
	int num_voxel,
	CKvMatrixBool *out_visibility, // width=num_point, height=visible:true, invisible:false
	CKvSet_of_MatrixFloat *out_depth_map)
//******************************************************************************
{
	int i,sz;
	CKvPmatrix3D *p_pmat;
	CKvMatrixFloat *p_out;
	LCKvUtility_for_Windows util_window;

	p_pmat=in_pmat->vps(sz);
	p_out=out_depth_map->c_Create(sz);
	out_visibility->c_Create(sz, num_voxel, false);
	for(i=0;i<sz;i++)
	{
		_Get_Depth_Image_and_visibility(
			in,						//CKvDoCubeShort *in,
			&p_pmat[i],				//CKvPmatrix3D *in_pmat,
			in_width,				//int in_width,
			in_height,				//int in_height,
			i,						//int in_id_pmat,
			in_th_for_visibility,	//float in_th_for_visibility,
			in_voxel,				//CKvVolume *in_voxel,
			out_visibility,			//CKvMatrixBool *in_visibility, // width=num_point, height=visible:true, invisible:false
			&p_out[i]);				//CKvMatrixFloat *out_depth_map)
	}	
	return;
}

//******************************************************************************
void Util_VCL_3DO::cip_Create_Image_Planes(
	int in_num_images,
	CKvSet_of_MatrixUcharRgb *in_background_img_set,
	CKvSet_of_MatrixBool *out_mask,
	CKvSet_of_SdkCode *out_mask_sdkcode)
//******************************************************************************
{
	int width, height,i;
	CKvMatrixBool *binary_img;
	CKvSdkCode *sdk;
	CKvRunSet rset;

	binary_img = out_mask->c_Create(in_num_images);
	sdk        = out_mask_sdkcode->c_Create(in_num_images);
	in_background_img_set->gpe(0)->mps(width,height);	

	for(i=0; i<in_num_images;i++){
		binary_img[i].c_Create(in_background_img_set->gpe(i)->mh(), in_background_img_set->gpe(i)->mw(), true);
 		rset.i_Import(&binary_img[i]);
 		sdk[i].i_Import(&rset,true);
	}
}

//******************************************************************************
void Util_VCL_3DO::gvz_Get_Visibility_Zone(
	CKvDepot_of_Point3D *in_depot_point,
	CKvDepot_of_RgbaF *in_depot_color,
	CKvMesh_of_Triangle *in_mesh_triangle,
	CKvSet_of_Pmatrix3D *in_set_of_pmat,
	CKvSet_of_MatrixBool *io_mask,
	CKvSet_of_SdkCode *io_mask_sdkcode)
//******************************************************************************
{	
	//Get visibility zone for each view
	printf("    [gvz_Get_Visibility_Zone]\n");
	gvzi_Get_Visibility_Zone_For_an_Image(in_depot_point,in_depot_color,in_mesh_triangle,in_set_of_pmat,io_mask,io_mask_sdkcode);
}

//******************************************************************************
void Util_VCL_3DO::gvzi_Get_Visibility_Zone_For_an_Image(
	CKvDepot_of_Point3D *in_depot_point,
	CKvDepot_of_RgbaF *in_depot_color,
	CKvMesh_of_Triangle *in_mesh_triangle,
	CKvSet_of_Pmatrix3D *in_set_of_pmat,
	CKvSet_of_MatrixBool *io_mask,
	CKvSet_of_SdkCode *io_mask_sdkcode)
//******************************************************************************
{
	CKvRunSet runset;
	CKvSdkCode *p_mask_sdkcode;
	CKvMatrixBool* p_mask;
	CKvMatrixUchar d_img;
	CKvPmatrix3D *p_pmat;
	CKvVectorInt mesh_idx;
	LCKvUtility_for_Windows util_window;
	LCKvUtility_for_Import util_import;
	int number_camera,i;

	//Initialization
	p_pmat		   = in_set_of_pmat->vps(number_camera);
	p_mask		   = io_mask->vp();
	p_mask_sdkcode = io_mask_sdkcode->vp();

	//Silhouette generation by multi-view projection
	gim_Get_Indices_of_Mesh(in_mesh_triangle, 0, &mesh_idx);
	for(i=0; i<number_camera; i++){		
		/// ////////////////////////////////////////////////////////////		
		printf("      #%d camera... ", i+1);
		p_mask[i].sm_Set_Matrix(false);
		gszbi_Generat_Silhouette_using_Z_buffering_for_an_Image(in_depot_point, &mesh_idx, &p_pmat[i],  &p_mask[i]);
		printf(" Complete!\r", i+1);
		/// ////////////////////////////////////////////////////////////
		runset.i_Import(&p_mask[i]);
		p_mask_sdkcode[i].i_Import(&runset, true);

		//CKvScreen aaa;
		//aaa.s_d_Display(&p_mask[i]);
		//Kv_Printf("dd");

	}
	printf("\n");
}

//******************************************************************************
void Util_VCL_3DO::gszbi_Generat_Silhouettes_using_Z_buffering(
	CKvDepot_of_Point3D *in_depot_point,
	CKvDepot_of_RgbaF *in_depot_color,
	CKvMesh_of_Triangle *in_mesh_triangle,
	CKvSet_of_Pmatrix3D *in_set_of_pmat,
	CKvSet_of_MatrixBool *io_mask,
	CKvSet_of_SdkCode *io_mask_sdkcode)
//******************************************************************************
{
	CKvRunSet runset;
	CKvSdkCode *p_mask_sdkcode;
	CKvMatrixBool* p_mask;
	CKvMatrixUchar d_img;
	CKvPmatrix3D *p_pmat;
	CKvVectorInt mesh_idx;
	LCKvUtility_for_Windows util_window;
	LCKvUtility_for_Import util_import;
	int number_camera,i;

	//Initialization
	p_pmat		   = in_set_of_pmat->vps(number_camera);
	p_mask		   = io_mask->vp();
	p_mask_sdkcode = io_mask_sdkcode->vp();

	//Silhouette generation by multi-view projection
	gim_Get_Indices_of_Mesh(in_mesh_triangle, 0, &mesh_idx);
	for(i=0; i<number_camera; i++){		
		printf("    #%d camera... ", i+1);
		p_mask[i].sm_Set_Matrix(false);
		gszbi_Generat_Silhouette_using_Z_buffering_for_an_Image(in_depot_point, &mesh_idx, &p_pmat[i],  &p_mask[i]);
		printf(" Complete!\n", i+1);
		runset.i_Import(&p_mask[i]);
		p_mask_sdkcode[i].i_Import(&runset, true);
	}
}
//******************************************************************************
void Util_VCL_3DO::gszbi_Generat_Silhouettes_using_Z_buffering(
	CKvDepot_of_Point3D *in_depot_point,
	CKvDepot_of_RgbaF *in_depot_color,
	CKvMesh_of_Triangle *in_mesh_triangle,
	CKvSet_of_Pmatrix3D *in_set_of_pmat,
	int *in_ww_hh,
	int in_dilation,
	CKvSet_of_SdkCode *out_sil_sdkcode)
//******************************************************************************
{
	CKvSdkCode *p_sil_sdkcode;
	CKvMatrixBool* p_sil;
	CKvMatrixUchar d_img;
	CKvPmatrix3D *p_pmat;
	CKvVectorInt mesh_idx;
	CKvSet_of_MatrixBool mask;
	int number_camera,i;

	//Initialization
	p_pmat		   = in_set_of_pmat->vps(number_camera);
	p_sil		   = mask.c_Create(number_camera);
	p_sil_sdkcode  = out_sil_sdkcode->c_Create(number_camera);

	//Silhouette generation by multi-view projection
	gim_Get_Indices_of_Mesh(in_mesh_triangle, 0, &mesh_idx);
	
	for(i=0; i<number_camera; i++){		
		printf("    #%d camera... ", i+1);
		p_sil[i].c_Create(in_ww_hh[1],in_ww_hh[0],false);
		gszbi_Generat_Silhouette_using_Z_buffering_for_an_Image(in_depot_point, &mesh_idx, &p_pmat[i],&p_sil[i]);
		printf(" Complete!\r", i+1);
		zz_runset.i_Import(&p_sil[i]);
		if(in_dilation!=0) zz_runset.d_Dilation(&zz_runset,1,1);
		p_sil_sdkcode[i].i_Import(&zz_runset, true);
	}
}

//******************************************************************************
void Util_VCL_3DO::cmi_Convert_Mask_Image(
	CKvSet_of_SdkCode *in_mask,
	CKvSet_of_MatrixBool *out_mask)
//******************************************************************************
{
	int k,num_img;
	bool dummy;
	CKvSdkCode *p_in_mask;
	CKvMatrixBool *p_out_mask;
	CKvRunSet runset;
	
	p_in_mask  = in_mask->vps(num_img);
	p_out_mask = out_mask->c_Create(num_img);

	for(k=0;k<num_img;k++){
		p_in_mask[k].e_Export(&runset,dummy);
		runset.e_Export(true,false,&p_out_mask[k]);
	}
}

//*********************************************************************************************************
void Util_VCL_3DO::dmfpc_Display_Matrix_Float_using_Pseudo_Color(CKvScreen *in_screen,
						  CKvMatrixFloat &in_matrix,
						  int in_normalized_mode)
//*********************************************************************************************************
{
	CKvMatrixUcharRgb disp_img;
	int ww, hh, idx;
	int pseudo_r, pseudo_g, pseudo_b;
	unsigned char *p_disp_img, temp_val;
	float *p_in_matrix, max_val, min_val, offset, scale;
	
	p_in_matrix = in_matrix.mps(ww, hh)[0];
	p_disp_img = disp_img.c_Create(hh, ww)[0];
	if(in_normalized_mode){
		min_val = 1.0e7;
		max_val = -1.0e7;
		for(int j=0; j<hh; j++){
			for(int i=0; i<ww; i++){
				int idx = j*ww+i;
				if(max_val < p_in_matrix[idx])	max_val = p_in_matrix[idx];
				if(min_val > p_in_matrix[idx])		min_val = p_in_matrix[idx];
			}	
		}
	}
	offset = -min_val;
	scale = 255.0f/(max_val-min_val);

	for(int j=0; j<hh; j++){
		for(int i=0; i<ww; i++){			
			idx = j*ww+i;
			if(in_normalized_mode)	temp_val = (unsigned char)(scale*(p_in_matrix[idx]+offset));
			else					temp_val = (unsigned char)max(min(255.0f, p_in_matrix[idx]), 0.0f);
			
			// original pseudo coloring.
// 			pseudo_r = 128*((-sin((float)temp_val*2*PI/256.0f)+1.0f))-1;
// 			pseudo_g = 128*((-cos((float)temp_val*2*PI/256.0f)+1.0f))-1;
// 			pseudo_b = 128*((sin((float)temp_val*2*PI/256.0f)+1.0f))-1;
			// new pseudo coloring.
			if(temp_val>=0 && temp_val<=63)				{		pseudo_r=0;		pseudo_g=(int)(255.0f/63.0f*(float)temp_val);		pseudo_b=255;	}
			else if(temp_val>=63 && temp_val<=127)	{		pseudo_r=0;		pseudo_g=255;		pseudo_b=255-(int)(255.0f/(127.0f-63.0f)*((float)temp_val-63.0f));	}
			else if(temp_val>=127 && temp_val<=191)	{		pseudo_r=(int)(255.0f/(191.0f-127.0f)*((float)temp_val-127.0f));		pseudo_g=255;		pseudo_b=0;	}
			else																			{		pseudo_r=255;		pseudo_g=255-(int)(255.0f/(255.0f-191.0f)*((float)temp_val-191.0f));		pseudo_b=0;	}
	

			p_disp_img[idx] = (unsigned char)pseudo_r;			idx+=ww*hh;
			p_disp_img[idx] = (unsigned char)pseudo_g;		idx+=ww*hh;
			p_disp_img[idx] = (unsigned char)pseudo_b;		idx+=ww*hh;
		}
	}

	in_screen->s_d_Display(&disp_img);
}



//Refinement and Dilation
//******************************************************************************
void Util_VCL_3DO::rvh_Refine_Visual_Hull(CKvSet2d_of_VectorShort *io_vh)
//******************************************************************************
{
	CKvVectorShort **p_vh;
	short *pp_vh,tt[100];
	int ww,hh,i,j,k,num_layer,num_layer_new;

	p_vh = io_vh->mps(ww,hh);

	for(j=0;j<hh;j++){
		for(i=0;i<ww;i++){
			pp_vh = p_vh[j][i].vps(num_layer);
			if(pp_vh[0] == NOPOINTVALDoCube) continue;
			num_layer_new = 0;
			for(k=0;k<num_layer;k+=2){
				if(pp_vh[k]==pp_vh[k+1] || pp_vh[k]+1 == pp_vh[k+1]){continue;}
				tt[num_layer_new++] = pp_vh[k];
				tt[num_layer_new++] = pp_vh[k+1];
			}
			if(num_layer_new ==0){ p_vh[j][i].c_Create(2,NOPOINTVALDoCube);}
			else{
				pp_vh = p_vh[j][i].c_Create(num_layer_new,NOPOINTVALDoCube);
				for(int kk=0;kk<num_layer_new;kk++){pp_vh[kk] = tt[kk];}
			}
		}
	}
}
