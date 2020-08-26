#include "../../stdafx.h"
//#include "VCL_Inhand_Object_Scanning.h"
//#include "VCL_Inhand_Object_ScanningDlg.h"
//#include "_VCL_Mesh_to_DoS.h"
#include "../_VCL_2014_3DO.h"


VCL_Mesh_to_DoS::VCL_Mesh_to_DoS()
{
}

VCL_Mesh_to_DoS::~VCL_Mesh_to_DoS()
{
}
//******************************************************************************
void VCL_Mesh_to_DoS::sp_Set_Parameters(
	int in_width,
	int in_height,
	float in_angle_x,
	float in_angle_y,
	float in_angle_z,
	CKvHmatrix3D *out_homo)
//******************************************************************************
{
	CKvSet_of_Point3D Hp3d;
	CKvMatrix H;
	CKvMatrixFloat tt;
	float **p_tt,pi180;
	double *pp_DoCube;

	//Initialization
	pi180 = (float)PI/180.0f;

	zz_width   = in_width;
	zz_height  = in_height;

	zz_angle_x = in_angle_x*pi180;
	zz_angle_y = in_angle_y*pi180;
	zz_angle_z = in_angle_z*pi180;

	p_tt      =tt.c_Create(4,4,0.0f);	
	p_tt[0][0]=p_tt[1][1]=p_tt[2][2]=p_tt[3][3]=1.0f;
	out_homo ->c_Create(&tt);
}
//********************************************************************************
void VCL_Mesh_to_DoS::idm_Initialize_Depth_Map(
	int in_width_of_depth_map,
	int in_height_of_depth_map,
	CKvSet2d_of_VectorFloat *out_analog_depth_map)
//********************************************************************************
{
	out_analog_depth_map->c_Create(in_height_of_depth_map,in_width_of_depth_map);
}


//******************************************************************************
void VCL_Mesh_to_DoS::cimrs_Compute_Intersections_between_Mesh_and_Ray_Segments(
	CKvSet_of_Point3D *in_set_of_p3d_world,
	CKvMatrixInt      *in_mesh_of_depth_surface,
	CKvHmatrix3D *in_homography_rotation_or_NULL,
	CKvHmatrix3D *in_homography_norm,
	CKvSet2d_of_VectorFloat *io_analog_depth_map,
	float in_theta_z_axis_and_opt_axis_depth)
//******************************************************************************
{
	CKvSet2d_of_Vector convert_to_DoCube;
	CKvSet_of_Point3D set_of_p3d_normalized;
	
	CKvHmatrix3D homo_rotation;

	CKvMatrixInt mat_num_layer;
	CKvMatrixFloat tt;
	CKvVector **p_DoCube;
	CKvRanker rank;
	CKvVectorFloat **p_analog_depth_map;

	CKvPoint3D *p3d_world, *p3d_norm;

	float angle[3], *pp_analog_depth_map;
	double *pp_DoCube;
	int **p_num_layer,i,j,k;//,num_layers;

	int num_points, ray_dim_x, ray_dim_y, mode_intersection;
	bool odd_flag, ray_direction_flag;
		
	// ===============================================================================
	// Initialize parameters. ========================================================
	// ===============================================================================
	num_points = (*in_set_of_p3d_world).vs();
	(*io_analog_depth_map).ms(ray_dim_x, ray_dim_y);
	// Check relative angle between ray direction(z-axis) and optical axis of depth camera.
	// + If two directions are similar, set flag to true, otherwise to false.
	ray_direction_flag = (in_theta_z_axis_and_opt_axis_depth < 90.0) ? true : false;
	// Select mode for updating mesh intersections.
	// + 0: using only first intersection.
	// + 1: using all intersections.
	mode_intersection = 0;
	
	// ===============================================================================
	// Set structures and pointers. ==================================================
	// ===============================================================================
	p3d_world = (*in_set_of_p3d_world).vp();
	p3d_norm = set_of_p3d_normalized.c_Create(num_points);	

	for(k=0; k<set_of_p3d_normalized.vs(); k++){		

		// Normalize DoCube.
		(*in_homography_norm).tp_Transform_Point(
			true,
			p3d_world[k],
			p3d_norm[k]);

		// Rotate DoCube.
		if(in_homography_rotation_or_NULL){
			(*in_homography_rotation_or_NULL).tp_Transform_Point(
				true,
				p3d_norm[k],
				p3d_norm[k]);
		}
	}

	// ===============================================================================
	// Refine initial visual hull using mesh surface of object depth.
	// ===============================================================================
	// Compute intersection points between mesh faces and DoCube rays.
	z_cmdc_Convert_Mesh_to_DoCube(
		set_of_p3d_normalized,
		*in_mesh_of_depth_surface,
		ray_dim_x, ray_dim_y,
		convert_to_DoCube,
		mat_num_layer);	
	// Generate DoCube of mesh surface from pre-computed intersection points.
	// + Initialize parameters.
	odd_flag=false;
	// + Set structures and pointers.
	p_DoCube		   = convert_to_DoCube.mp();
	p_num_layer		   = mat_num_layer.mp();
	p_analog_depth_map = io_analog_depth_map->mp();
	for(j=0; j<ray_dim_y; j++){
		for(i=0; i<ray_dim_x; i++){
			// Edited by Yooji at 2016.08.29.

			switch(mode_intersection){

			// Mode 1 : Using first intersection only ==================================================================
			// =========================================================================================================
			case 0:
				// Update only first intersection.
				//if(0)
				if( p_num_layer[j][i]> 0 && p_num_layer[j][i] <= (int)Do_G_INIT_SIZE_DEPTH)
				{
					//p_analog_depth_map[j][i].c_Create(2, (float)NOPOINTVALDoCube);
					//p_analog_depth_map[j][i].vp()[0] = -1.0;		p_analog_depth_map[j][i].vp()[1] = 1.0;

					// ===============================================================================
					// Check which ray intersects on the front face of mesh or on the back face.
					// + If ray intersects on the back fate of mesh, encode offset (100.0) on depth value.
					// ===============================================================================
					double td = (ray_direction_flag) ? p_DoCube[j][i].vp()[0] : p_DoCube[j][i].vp()[p_num_layer[j][i]-1];

					p_analog_depth_map[j][i].c_Create(2); 
					if(td > 1.0){	// intersection with back face of mesh.
						p_analog_depth_map[j][i].vp()[0] = -1.0;  
						p_analog_depth_map[j][i].vp()[1] = td - 100.0;						
					}
					else{			// intersection with front face of mesh.
						p_analog_depth_map[j][i].vp()[0] = td;
						p_analog_depth_map[j][i].vp()[1] = 1.0;
					}
				}
				else
				{
					p_analog_depth_map[j][i].c_Create(2, (float)NOPOINTVALDoCube);
					p_analog_depth_map[j][i].vp()[0] = -1.0;		p_analog_depth_map[j][i].vp()[1] = 1.0;
				}

				break;

			// Mode 2 : Using all intersections  =======================================================================
			// =========================================================================================================
			case 1:
				// Update all intersections.
// 				if(p_num_layer[j][i]> 0 && p_num_layer[j][i] <= (int)Do_G_INIT_SIZE_DEPTH)
// 				{
// 
// 					// Finds sequence of mesh orientations at each mesh-ray intersection.
// 					CKvVectorBool seq_ori_mesh;
// 					bool *p_seq = seq_ori_mesh.c_Create(p_num_layer[j][i], false);
// 
// 
// 
// 					// ===============================================================================
// 					// Check which ray intersects on the front face of mesh or on the back face.
// 					// + If ray intersects on the back fate of mesh, encode offset (100.0) on depth value.
// 					// ===============================================================================
// 					double td = p_DoCube[j][i].vp()[0];
// 
// 					p_analog_depth_map[j][i].c_Create(2);
// 					if(td > 1.0){
// 						p_analog_depth_map[j][i].vp()[0] = -1.0;
// 						p_analog_depth_map[j][i].vp()[1] = td - 100.0;
// 					}
// 					else{
// 						p_analog_depth_map[j][i].vp()[0] = td;
// 						p_analog_depth_map[j][i].vp()[1] = 1.0;
// 					}
// 				}
// 				else
// 				{
// 					p_analog_depth_map[j][i].c_Create(2, (float)NOPOINTVALDoCube);
// 
// 					p_analog_depth_map[j][i].vp()[0] = -1.0;		p_analog_depth_map[j][i].vp()[1] = 1.0;
// 				}
				break;
				
				
			}
//				if((p_num_layer[j][i]%2)!=0)
//				{ 
//					printf("%d %d] %f\n", p_num_layer[j][i], p_DoCube[j][i].vs(), (p_DoCube[j][i]).vp()[0]);
//					odd_flag=true;
//
//					//if(0)
//					if(p_num_layer[j][i]==1)
//					{						
//						// Add surface point on the XY-plane of which z value is 1.0 (maximum value).
//						// ===============================================================================
//						// Check which ray intersects on the front face of mesh or on the back face.
//						// + If ray intersects on the back fate of mesh, encode offset (100.0) on depth value.
//						// ===============================================================================
//						double td = p_DoCube[j][i].vp()[0];
//
//						p_analog_depth_map[j][i].c_Create(2);
//						if(td > 1.0){
//							p_analog_depth_map[j][i].vp()[0] = -1.0;
//							p_analog_depth_map[j][i].vp()[1] = td - 100.0;
//						}
//						else{
//							p_analog_depth_map[j][i].vp()[0] = td;
//							p_analog_depth_map[j][i].vp()[1] = 1.0;
//						}
//
//
//// 						rank.s_Sort(&p_DoCube[j][i], false, 0, p_num_layer[j][i],NULL);
//// 						pp_analog_depth_map = p_analog_depth_map[j][i].c_Create(p_num_layer[j][i]);
//// 						pp_DoCube = p_DoCube[j][i].vp();
//// 						for(k=0; k<(p_num_layer[j][i]); k++){pp_analog_depth_map[k] = (float)pp_DoCube[k];}
//					}
//					else
//					{
//						p_analog_depth_map[j][i].c_Create(2, (float)NOPOINTVALDoCube);
//
// 						p_analog_depth_map[j][i].vp()[0] = -1.0;		p_analog_depth_map[j][i].vp()[1] = 1.0;
//					}
//				}
//				else
//				{
//					//if(!Kv_Printf("%d] %f", p_num_layer[j][i], (p_DoCube[j][i]).vp()[0]))	exit(0);
//					printf("%d %d] %f\n", p_num_layer[j][i], p_DoCube[j][i].vs(), (p_DoCube[j][i]).vp()[0]);
//
//					// ===============================================================================
//					// Check which ray intersects on the front face of mesh or on the back face.
//					// + If ray intersects on the back fate of mesh, encode offset (100.0) on depth value.
//					// ===============================================================================
//					double td;
//					
//					pp_DoCube = p_DoCube[j][i].vp();
//
//					for(k=0; k<p_num_layer[j][i]; k++){						
//						td = pp_DoCube[k];
//						if(td > 1.0)	pp_DoCube[k] = td - 100.0;
//					}
//
//					rank.s_Sort(&p_DoCube[j][i], false, 0, p_num_layer[j][i],NULL);
//					pp_analog_depth_map = p_analog_depth_map[j][i].c_Create(p_num_layer[j][i]);					
//					for(k=0; k<p_num_layer[j][i]; k++){pp_analog_depth_map[k] = (float)pp_DoCube[k];}
//				}
//			}
//			else
//			{
//				p_analog_depth_map[j][i].c_Create(2, (float)NOPOINTVALDoCube);
//
// 				p_analog_depth_map[j][i].vp()[0] = -1.0;		p_analog_depth_map[j][i].vp()[1] = 1.0;
//			}
		}
	}
	//if(odd_flag) Kv_Printf("the nuber of the points at a ray = ODD");
	return;
}


//******************************************************************************
bool VCL_Mesh_to_DoS::cimrs_Compute_Intersections_between_Mesh_and_Ray_Segments(
	CKvSet_of_Point3D *in_set_of_p3d_world,
	CKvMatrixInt      *in_mesh_of_depth_surface,
	int in_num_valid_p3ds,
	int in_num_valid_mesh_tris,
	CKvPmatrix3D *in_pmat,
	CKvHmatrix3D *in_homography_norm,
	CKvSet2d_of_VectorFloat *io_analog_depth_map,
	int in_ray_direction,
	float in_shield_margin)		// 0: x-axis | 1: y-axis | 2: z-axis.		
//******************************************************************************
{
	CKvSet2d_of_Vector convert_to_DoCube;
	//CKvSet_of_Point3D zz_set_of_p3d_normalized;
	
	CKvHmatrix3D homo_rotation;

	CKvMatrixFloat T_mat;
	CKvPoint3D opt_axis; float ip, theta;

	//CKvMatrixInt mat_num_layer;
	CKvMatrixFloat tt;
	CKvVector **p_DoCube;
	CKvRanker rank;
	CKvVectorFloat **p_analog_depth_map;

	CKvPoint3D *p3d_world, *p3d_norm;

	float angle[3], *pp_analog_depth_map, margin_shield;
	double *pp_DoCube;
	int **p_num_layer,i,j,k, num_layers;

	int num_points_total, num_points_valid, ray_dim_x, ray_dim_y, mode_intersection;
	bool odd_flag, ray_direction_flag;
		
	// ===============================================================================
	// Initialize parameters. ========================================================
	// ===============================================================================
	num_points_total = (*in_set_of_p3d_world).vs();
	num_points_valid = in_num_valid_p3ds;
	//num_points = (*in_set_of_p3d_world).vs();
	(*io_analog_depth_map).ms(ray_dim_x, ray_dim_y);
	if(num_points_total != zz_set_of_p3d_normalized.vs()) zz_set_of_p3d_normalized.c_Create(num_points_total);

	// optical axis of input camera.
	opt_axis = (*in_pmat).dvo_Direction_Vector_of_Optical_axis();
	
	// Select mode for updating mesh intersections.
	// + 0: using only first intersection.
	// + 1: using all intersections.
	mode_intersection = 0;
	margin_shield = in_shield_margin;

	// ===============================================================================
	// Set structures and pointers. ==================================================
	// ===============================================================================
	p3d_world = (*in_set_of_p3d_world).vp();
	p3d_norm = zz_set_of_p3d_normalized.vp();
	//p3d_norm = zz_set_of_p3d_normalized.c_Create(num_points_valid);	

	for(k=0; k<num_points_total; k++){		

		// Normalize DoCube.
		(*in_homography_norm).tp_Transform_Point(
			true,
			p3d_world[k],
			p3d_norm[k]);
	}

	// ===============================================================================
	// Refine initial visual hull using mesh surface of object depth.
	// ===============================================================================
	if(in_ray_direction == 0){
		
		// + Compute angle between x-axis and plane normal (theta: 0 ~ 180).
		ip = opt_axis.x;	// inner product of plane normal and x-axis.		
		theta = acos(ip)*180.0/PI;	

		// Check relative angle between ray direction(z-axis) and optical axis of depth camera.
		// + If two directions are similar, set flag to true, otherwise to false.
		ray_direction_flag = (theta < 90.0) ? true : false;
		printf("theta: %f\n\n",theta);

		// Compute intersection points between mesh faces and DoCube rays.
		z_cmdc_Convert_Mesh_to_DoCube_for_X_axis(
			zz_set_of_p3d_normalized,
			*in_mesh_of_depth_surface,
			in_num_valid_p3ds,
			in_num_valid_mesh_tris,
			ray_dim_x, ray_dim_y,
			ray_direction_flag,
			convert_to_DoCube);	

	}
	else if(in_ray_direction == 1){

		// + Compute angle between y-axis and plane normal (theta: 0 ~ 180).
		ip = opt_axis.y;	// inner product of plane normal and y-axis.		
		theta = acos(ip)*180.0/PI;

		// Check relative angle between ray direction(z-axis) and optical axis of depth camera.
		// + If two directions are similar, set flag to true, otherwise to false.
		ray_direction_flag = (theta < 90.0) ? true : false;
		printf("theta: %f\n\n",theta);

		// Compute intersection points between mesh faces and DoCube rays.
		z_cmdc_Convert_Mesh_to_DoCube_for_Y_axis(
			zz_set_of_p3d_normalized,
			*in_mesh_of_depth_surface,
			in_num_valid_p3ds,
			in_num_valid_mesh_tris,
			ray_dim_x,ray_dim_y,
			ray_direction_flag,
			convert_to_DoCube);
	}
	else if(in_ray_direction == 2){

		// + Compute angle between z-axis and plane normal (theta: 0 ~ 180).
		ip = opt_axis.z;	// inner product of plane normal and z-axis.		
		theta = acos(ip)*180.0/PI;

		// Check relative angle between ray direction(z-axis) and optical axis of depth camera.
		// + If two directions are similar, set flag to true, otherwise to false.
		ray_direction_flag = (theta < 90.0) ? true : false;
		printf("theta: %f\n\n",theta);

		// Compute intersection points between mesh faces and DoCube rays.
		z_cmdc_Convert_Mesh_to_DoCube_for_Z_axis(
			zz_set_of_p3d_normalized,
			*in_mesh_of_depth_surface,
			in_num_valid_p3ds,
			in_num_valid_mesh_tris,
			ray_dim_x,ray_dim_y,
			ray_direction_flag,
			convert_to_DoCube);
	}

	//////////////////////////////////////////////////////////////////////////
	// 이 부분 각도에 의해 뚫린 부분이 있고 없고가 결정됨...
	// 이 부분 각도에 의해 뚫린 부분이 있고 없고가 결정됨...
	// 이 부분 각도에 의해 뚫린 부분이 있고 없고가 결정됨...
	//if(theta > 30 && theta < 150) return false;
	// 이 부분 각도에 의해 뚫린 부분이 있고 없고가 결정됨...
	// 이 부분 각도에 의해 뚫린 부분이 있고 없고가 결정됨...
	// 이 부분 각도에 의해 뚫린 부분이 있고 없고가 결정됨...
	//////////////////////////////////////////////////////////////////////////

	// Generate DoCube of mesh surface from pre-computed intersection points.
	// + Initialize parameters.
	odd_flag=false;
	// + Set structures and pointers.
	p_DoCube		   = convert_to_DoCube.mp();
	//p_num_layer		   = mat_num_layer.mp();
	p_analog_depth_map = io_analog_depth_map->mp();
	for(j=0; j<ray_dim_y; j++){
		for(i=0; i<ray_dim_x; i++){
			// Edited by Yooji at 2016.08.29.

			// Update only first intersection.
			//if(0)
// 			num_layers = p_num_layer[j][i];
// 			if(num_layers>0)
// 			{
				//p_analog_depth_map[j][i].c_Create(2, (float)NOPOINTVALDoCube);
				//p_analog_depth_map[j][i].vp()[0] = -1.0;		p_analog_depth_map[j][i].vp()[1] = 1.0;

				// ===============================================================================
				// Check which ray intersects on the front face of mesh or on the back face.
				// + If ray intersects on the back fate of mesh, encode offset (100.0) on depth value.
				// ===============================================================================
 				double td = p_DoCube[j][i].vp()[0];
 				
 				p_analog_depth_map[j][i].c_Create(2);
				//////////////////////////////////////////////////////////////////////////
//  				p_analog_depth_map[j][i].vp()[0] = -1.0;		// related with homography scale.
//  				p_analog_depth_map[j][i].vp()[1] = 1.0;			// related with homography scale.
				//////////////////////////////////////////////////////////////////////////

  				if(ray_direction_flag){	// intersection with front face of mesh.
					//if(td==1.0f) td = -1.0f;	// for invalid rays.
					p_analog_depth_map[j][i].vp()[0] = (td==1.0f) ? -1.0f : fmax(-1.0f, td - margin_shield);  
  					//p_analog_depth_map[j][i].vp()[0] = fmax(-1.0f, td - margin_shield);  
  					p_analog_depth_map[j][i].vp()[1] = 1.0;						
  				}
  				else{			// intersection with back face of mesh.
					//if(td==-1.0f) td = 1.0f;	// for invalid rays.
 					p_analog_depth_map[j][i].vp()[0] = -1.0f;
 					//p_analog_depth_map[j][i].vp()[1] = fmin(1.0f, td + margin_shield);
					p_analog_depth_map[j][i].vp()[1] = (td==-1.0f) ? 1.0f : fmin(1.0f, td - margin_shield);
 				}

//				p_analog_depth_map[j][i].c_Create(2);

// 				if(ray_direction_flag){
// 					p_analog_depth_map[j][i].vp()[0] = p_DoCube[j][i].vp()[0];
// 					p_analog_depth_map[j][i].vp()[1] = 1.0;
// 				}
// 				else{
// 					p_analog_depth_map[j][i].vp()[0] = -1.0;
// 					p_analog_depth_map[j][i].vp()[1] = p_DoCube[j][i].vp()[num_layers - 1];
// 				}
// 			}
// 			else
// 			{
// 				p_analog_depth_map[j][i].c_Create(2, (float)NOPOINTVALDoCube);
// 				p_analog_depth_map[j][i].vp()[0] = -1.0;		p_analog_depth_map[j][i].vp()[1] = 1.0;
// 			}
			
		}
	}
	//if(odd_flag) Kv_Printf("the number of the points at a ray = ODD");
	return true;
}


//******************************************************************************
//CAUTION: Starting points at the DoS(x-y plane of the initial bound box: z=-1).
bool VCL_Mesh_to_DoS::z_cmdc_Convert_Mesh_to_DoCube(
		CKvSet_of_Point3D &in_set_of_p3d_norm,	//3D points set
		CKvMatrixInt &in_mesh_indices,			//mesh index (3xN)
		int in_dim_x, int in_dim_y,				// width and height of reference points. (dimension of XY-plane in DoCube).
		CKvSet2d_of_Vector &out_depth,			//Depth of the each ray(range=-1 to 1)
		CKvMatrixInt &out_num_of_layer)			//number of the layer
//******************************************************************************
{
	CKvPoint3D v13, v12;		// norm : surface normal, v13 x v12.

	CKvPoint3D *p_p3d;
	CKvVector **p_sp, **p_d;
	
	double *pp_sp;
	int **p_mesh;	
	int **p_num_d;

	double pd_min[2], pd_max[2];
	double x[3], y[3], z[3], norm[3], depth;
	int pi_min[2], pi_max[2];

	double rp_x, rp_y;
	double dx,dy;

	int i,j,k,sz_p3d,sz_mesh,ww_ray,hh_ray,temp;
		
	ww_ray = in_dim_x;
	hh_ray = in_dim_y;

	//if(!Kv_Printf("%d %d", ww_ray, hh_ray))	exit(0);

	// Initialize parameters and pointers.
	// + Step size of reference points.
	dx = 2./ww_ray;	// step size along X-axis in normalized DoCube.
	dy = 2./hh_ray;	// step size along Y-axis in normalized DoCube.
	// + Pointers.
	p_p3d=in_set_of_p3d_norm.vps(sz_p3d);
	p_d=out_depth.c_Create(hh_ray, ww_ray);
	p_num_d=out_num_of_layer.c_Create(hh_ray, ww_ray, 0);
	p_mesh=in_mesh_indices.mps(sz_mesh, temp);

	for(int j=0; j<hh_ray; j++){	
		for(int i=0; i<ww_ray; i++){	
			p_d[j][i].c_Create(Do_G_INIT_SIZE_DEPTH, -1.0);	
		}
	}
	// Compute intersection points of mesh faces and DoCube rays.
	for(k=0;k<sz_mesh;k++)
	{
		// Get x value of 3 points included in current mesh triangle.
		x[0]=p_p3d[p_mesh[0][k]].x;	x[1]=p_p3d[p_mesh[1][k]].x;	x[2]=p_p3d[p_mesh[2][k]].x;
		pd_max[0]=_Min_Max(x,3,pd_min[0]);
		// Get y value of 3 points included in current mesh triangle.
		y[0]=p_p3d[p_mesh[0][k]].y;	y[1]=p_p3d[p_mesh[1][k]].y;	y[2]=p_p3d[p_mesh[2][k]].y;
		pd_max[1]=_Min_Max(y,3,pd_min[1]);				
		// Set ROI rectangle which bounds current mesh triangle.
		pi_min[0] = min( max( (int)((pd_min[0]+1)/dx)+1, 0 ), ww_ray - 1);
		pi_min[1] = min( max( (int)((pd_min[1]+1)/dy)+1, 0 ), hh_ray - 1);
		pi_max[0] = min( max( (int)((pd_max[0]+1)/dx)+1, 0 ), ww_ray - 1);
		pi_max[1] = min( max( (int)((pd_max[1]+1)/dy)+1, 0 ), hh_ray - 1);

		// Find intersections between DoCube rays and mesh of depth surface.
 		for(j=pi_min[1]; j<pi_max[1]; j++)
 		{
 			for(i=pi_min[0]; i<pi_max[0]; i++)
 			{
				if(p_num_d[j][i]>=Do_G_INIT_SIZE_DEPTH)	continue;
				// Set current reference point.
 				rp_x = -1.0 + (double)i*dx;
 				rp_y = -1.0 + (double)j*dy;
 
 				// Check this reference point in valid region of mesh triangle.
 				if(_Is_inlier_of_the_mesh(rp_x, rp_y, x, y))
 				{
					// Get z value of 3 points included in current mesh triangle.
 					z[0]=p_p3d[p_mesh[0][k]].z;
 					z[1]=p_p3d[p_mesh[1][k]].z;
 					z[2]=p_p3d[p_mesh[2][k]].z;

 					// Edited by Yooji at 2016.08.29.
 					//_Intersection_ray_and_mesh(-1+i*dx, -1+j*dy, x,y,z, depth);
 					if(!_Intersection_ray_and_mesh_with_normal(rp_x, rp_y, x,y,z, depth, norm))	continue;

					// ===============================================================================
					// Check which ray intersects on the front face of mesh or on the back face.
					// + If ray intersects on the back face of mesh, encode offset (100.0) on depth value.
					if(norm[2] > 0.0){	depth = depth + 100.0;	}
					// ===============================================================================

  					p_d[j][i].se_Set_Element(p_num_d[j][i], depth);
  					p_num_d[j][i]++;
 
 				}
 			}
 		}
	}

	return true;
}


//******************************************************************************
//CAUTION: Starting points at the DoS(x-y plane of the initial bound box: x=-1).
//Reference plane is YZ plane.
bool VCL_Mesh_to_DoS::z_cmdc_Convert_Mesh_to_DoCube_for_X_axis(
	CKvSet_of_Point3D &in_set_of_p3d_norm,	//3D points set
	CKvMatrixInt &in_mesh_indices,			//mesh index (3xN)
	int in_num_valid_p3ds,
	int in_num_valid_mesh_tris,
	int in_dim_x, int in_dim_y,				// width and height of reference points. (dimension of XY-plane in DoCube).
	bool in_ray_direction,
	CKvSet2d_of_Vector &out_depth)			//Depth of the each ray(range=-1 to 1)
//******************************************************************************
{
	CKvPoint3D v13, v12;		// norm : surface normal, v13 x v12.

	CKvPoint3D *p_p3d;
	CKvVector **p_sp, **p_d;
	
	double *pp_sp;
	int **p_mesh;	
//	int **p_num_d;

	double pd_min[2], pd_max[2];
	double x[3], y[3], z[3], norm[3], depth;
	int pi_min[2], pi_max[2];

	double rp_x, rp_y;
	double dx,dy;

	int i,j,k,sz_p3d,sz_mesh,ww_ray,hh_ray,temp;
		
	ww_ray = in_dim_x;
	hh_ray = in_dim_y;

	//if(!Kv_Printf("%d %d", ww_ray, hh_ray))	exit(0);

	// Initialize parameters and pointers.
	// + Step size of reference points.
	dx = 2./ww_ray;	// step size along X-axis in normalized DoCube.
	dy = 2./hh_ray;	// step size along Y-axis in normalized DoCube.
	// + Pointers.
	p_d=out_depth.c_Create(hh_ray, ww_ray);
	
	p_p3d=in_set_of_p3d_norm.vp();	sz_p3d = in_num_valid_p3ds;
	p_mesh=in_mesh_indices.mp();	sz_mesh = in_num_valid_mesh_tris;

	//p_p3d=in_set_of_p3d_norm.vps(sz_p3d);	
	//p_mesh=in_mesh_indices.mps(sz_mesh, temp);

	for(int j=0; j<hh_ray; j++){	
		for(int i=0; i<ww_ray; i++){	
			//p_d[j][i].c_Create(Do_G_INIT_SIZE_DEPTH, -1.0);	
			if(in_ray_direction) p_d[j][i].c_Create(2,1.0);
			else				 p_d[j][i].c_Create(2,-1.0);			
		}
	}
	// Compute intersection points of mesh faces and DoCube rays.
	for(k=0;k<sz_mesh;k++)
	{
		// Get x value of 3 points included in current mesh triangle.
		x[0]=p_p3d[p_mesh[0][k]].y;	x[1]=p_p3d[p_mesh[1][k]].y;	x[2]=p_p3d[p_mesh[2][k]].y;
		pd_max[0]=_Min_Max(x,3,pd_min[0]);
		// Get y value of 3 points included in current mesh triangle.
		y[0]=p_p3d[p_mesh[0][k]].z;	y[1]=p_p3d[p_mesh[1][k]].z;	y[2]=p_p3d[p_mesh[2][k]].z;
		pd_max[1]=_Min_Max(y,3,pd_min[1]);				
		// Set ROI rectangle which bounds current mesh triangle.
		pi_min[0] = min( max( (int)((pd_min[0]+1)/dx)+1, 0 ), ww_ray - 1);
		pi_min[1] = min( max( (int)((pd_min[1]+1)/dy)+1, 0 ), hh_ray - 1);
		pi_max[0] = min( max( (int)((pd_max[0]+1)/dx)+1, 0 ), ww_ray - 1);
		pi_max[1] = min( max( (int)((pd_max[1]+1)/dy)+1, 0 ), hh_ray - 1);

		// Find intersections between DoCube rays and mesh of depth surface.
 		for(j=pi_min[1]; j<pi_max[1]; j++)
 		{
 			for(i=pi_min[0]; i<pi_max[0]; i++)
 			{
				//if(p_num_d[j][i]>=Do_G_INIT_SIZE_DEPTH)	continue;
				// Set current reference point.
 				rp_x = -1.0 + (double)i*dx;
 				rp_y = -1.0 + (double)j*dy;
 
 				// Check this reference point in valid region of mesh triangle.
 				if(_Is_inlier_of_the_mesh(rp_x, rp_y, x, y))
 				{
					// Get z value of 3 points included in current mesh triangle.
 					z[0]=p_p3d[p_mesh[0][k]].x;
 					z[1]=p_p3d[p_mesh[1][k]].x;
 					z[2]=p_p3d[p_mesh[2][k]].x;

 					// Edited by Yooji at 2016.08.29.
 					//_Intersection_ray_and_mesh(-1+i*dx, -1+j*dy, x,y,z, depth);
 					if(!_Intersection_ray_and_mesh_with_normal(rp_x, rp_y, x,y,z, depth, norm))	continue;

					// ===============================================================================
					// Check which ray intersects on the front face of mesh or on the back face.
					// + If ray intersects on the back face of mesh, encode offset (100.0) on depth value.
					//if(norm[2] > 0.0){	depth = depth + 100.0;	}
					// ===============================================================================

					if(in_ray_direction){
						if(depth < p_d[j][i].vp()[0])	p_d[j][i].se_Set_Element(0,depth);
					} else if(depth > p_d[j][i].vp()[0])	p_d[j][i].se_Set_Element(0,depth);
 
 				}
 			}
 		}
	}

	return true;
}

//******************************************************************************
//CAUTION: Starting points at the DoS(x-y plane of the initial bound box: y=-1).
//Reference plane is ZX plane.
bool VCL_Mesh_to_DoS::z_cmdc_Convert_Mesh_to_DoCube_for_Y_axis(
	CKvSet_of_Point3D &in_set_of_p3d_norm,	//3D points set
	CKvMatrixInt &in_mesh_indices,			//mesh index (3xN)
	int in_num_valid_p3ds,
	int in_num_valid_mesh_tris,
	int in_dim_x, int in_dim_y,				// width and height of reference points. (dimension of XY-plane in DoCube).
	bool in_ray_direction,
	CKvSet2d_of_Vector &out_depth)			//Depth of the each ray(range=-1 to 1)	
//******************************************************************************
{
	CKvPoint3D v13, v12;		// norm : surface normal, v13 x v12.

	CKvPoint3D *p_p3d;
	CKvVector **p_sp, **p_d;
	
	double *pp_sp;
	int **p_mesh;	
	//int **p_num_d;

	double pd_min[2], pd_max[2];
	double x[3], y[3], z[3], norm[3], depth;
	int pi_min[2], pi_max[2];

	double rp_x, rp_y;
	double dx,dy;

	int i,j,k,sz_p3d,sz_mesh,ww_ray,hh_ray,temp;
		
	ww_ray = in_dim_x;
	hh_ray = in_dim_y;

	//if(!Kv_Printf("%d %d", ww_ray, hh_ray))	exit(0);

	// Initialize parameters and pointers.
	// + Step size of reference points.
	dx = 2./ww_ray;	// step size along X-axis in normalized DoCube.
	dy = 2./hh_ray;	// step size along Y-axis in normalized DoCube.
	// + Pointers.
	p_d=out_depth.c_Create(hh_ray, ww_ray);

	p_p3d=in_set_of_p3d_norm.vp();	sz_p3d = in_num_valid_p3ds;
	p_mesh=in_mesh_indices.mp();	sz_mesh = in_num_valid_mesh_tris;

	//p_p3d=in_set_of_p3d_norm.vps(sz_p3d);
	//p_mesh=in_mesh_indices.mps(sz_mesh, temp);

	for(int j=0; j<hh_ray; j++){	
		for(int i=0; i<ww_ray; i++){	
			//p_d[j][i].c_Create(Do_G_INIT_SIZE_DEPTH, -1.0);	
			if(in_ray_direction) p_d[j][i].c_Create(2,1.0);
			else				 p_d[j][i].c_Create(2,-1.0);
		}
	}
	// Compute intersection points of mesh faces and DoCube rays.
	for(k=0;k<sz_mesh;k++)
	{
		// Get x value of 3 points included in current mesh triangle.
		x[0]=p_p3d[p_mesh[0][k]].z;	x[1]=p_p3d[p_mesh[1][k]].z;	x[2]=p_p3d[p_mesh[2][k]].z;
		pd_max[0]=_Min_Max(x,3,pd_min[0]);
		// Get y value of 3 points included in current mesh triangle.
		y[0]=p_p3d[p_mesh[0][k]].x;	y[1]=p_p3d[p_mesh[1][k]].x;	y[2]=p_p3d[p_mesh[2][k]].x;
		pd_max[1]=_Min_Max(y,3,pd_min[1]);				
		// Set ROI rectangle which bounds current mesh triangle.
		pi_min[0] = min( max( (int)((pd_min[0]+1)/dx)+1, 0 ), ww_ray - 1);
		pi_min[1] = min( max( (int)((pd_min[1]+1)/dy)+1, 0 ), hh_ray - 1);
		pi_max[0] = min( max( (int)((pd_max[0]+1)/dx)+1, 0 ), ww_ray - 1);
		pi_max[1] = min( max( (int)((pd_max[1]+1)/dy)+1, 0 ), hh_ray - 1);

		// Find intersections between DoCube rays and mesh of depth surface.
 		for(j=pi_min[1]; j<pi_max[1]; j++)
 		{
 			for(i=pi_min[0]; i<pi_max[0]; i++)
 			{
				//if(p_num_d[j][i]>=Do_G_INIT_SIZE_DEPTH)	continue;
				// Set current reference point.
 				rp_x = -1.0 + (double)i*dx;
 				rp_y = -1.0 + (double)j*dy;
 
 				// Check this reference point in valid region of mesh triangle.
 				if(_Is_inlier_of_the_mesh(rp_x, rp_y, x, y))
 				{
					// Get z value of 3 points included in current mesh triangle.
 					z[0]=p_p3d[p_mesh[0][k]].y;
 					z[1]=p_p3d[p_mesh[1][k]].y;
 					z[2]=p_p3d[p_mesh[2][k]].y;

 					// Edited by Yooji at 2016.08.29.
 					//_Intersection_ray_and_mesh(-1+i*dx, -1+j*dy, x,y,z, depth);
 					if(!_Intersection_ray_and_mesh_with_normal(rp_x, rp_y, x,y,z, depth, norm))	continue;

					// ===============================================================================
					// Check which ray intersects on the front face of mesh or on the back face.
					// + If ray intersects on the back face of mesh, encode offset (100.0) on depth value.
					//if(norm[2] > 0.0){	depth = depth + 100.0;	}
					// ===============================================================================

					if(in_ray_direction){
						if(depth < p_d[j][i].vp()[0])	p_d[j][i].se_Set_Element(0,depth);
					} else if(depth > p_d[j][i].vp()[0])	p_d[j][i].se_Set_Element(0,depth);
 
 				}
 			}
 		}
	}

	return true;
}

//******************************************************************************
//CAUTION: Starting points at the DoS(x-y plane of the initial bound box: z=-1).
//Reference plane is XY plane.
bool VCL_Mesh_to_DoS::z_cmdc_Convert_Mesh_to_DoCube_for_Z_axis(
	CKvSet_of_Point3D &in_set_of_p3d_norm,	//3D points set
	CKvMatrixInt &in_mesh_indices,			//mesh index (3xN)
	int in_num_valid_p3ds,
	int in_num_valid_mesh_tris,
	int in_dim_x, int in_dim_y,				// width and height of reference points. (dimension of XY-plane in DoCube).
	bool in_ray_direction,
	CKvSet2d_of_Vector &out_depth)			//Depth of the each ray(range=-1 to 1)
//******************************************************************************
{
	CKvPoint3D v13, v12;		// norm : surface normal, v13 x v12.

	CKvPoint3D *p_p3d;
	CKvVector **p_sp, **p_d;
	
	double *pp_sp;
	int **p_mesh;	
	//int **p_num_d;

	double pd_min[2], pd_max[2];
	double x[3], y[3], z[3], norm[3], depth;
	int pi_min[2], pi_max[2];

	double rp_x, rp_y;
	double dx,dy;

	int i,j,k,/*sz_p3d,*/sz_mesh,ww_ray,hh_ray,temp;
		
	ww_ray = in_dim_x;
	hh_ray = in_dim_y;

	//if(!Kv_Printf("%d %d", ww_ray, hh_ray))	exit(0);

	// 이 부분 매 프레임 수행하는데 현재 상황에서는 크기 불변이므로 creation 은 처음 프레임에서 한 번만 해 주면 될 듯 하다.
	// Initialize parameters and pointers.
	// + Step size of reference points.
	dx = 2./ww_ray;	// step size along X-axis in normalized DoCube.
	dy = 2./hh_ray;	// step size along Y-axis in normalized DoCube.
	// + Pointers.
	p_d=out_depth.c_Create(hh_ray, ww_ray);
		
	p_p3d=in_set_of_p3d_norm.vp();	//sz_p3d = in_num_valid_p3ds;
	p_mesh=in_mesh_indices.mp();	sz_mesh = in_num_valid_mesh_tris;

	//p_p3d=in_set_of_p3d_norm.vps(sz_p3d);
	//p_mesh=in_mesh_indices.mps(sz_mesh, temp);

	for(int j=0; j<hh_ray; j++){	
		for(int i=0; i<ww_ray; i++){	
			// in case of mesh-to-DoCube of depth surface, we needs two surface point only.
			// because number of segments in each ray direction is always 1.
			//p_d[j][i].c_Create(Do_G_INIT_SIZE_DEPTH, -1.0);	
			if(in_ray_direction) p_d[j][i].c_Create(2, 1.0);	
			else				 p_d[j][i].c_Create(2, -1.0);	
		}
	}
	// 이 부분 매 프레임 수행하는데 현재 상황에서는 크기 불변이므로 creation 은 처음 프레임에서 한 번만 해 주면 될 듯 하다.

	// Compute intersection points of mesh faces and DoCube rays.
	for(k=0;k<sz_mesh;k++)
	{
		// Get x value of 3 points included in current mesh triangle.
		x[0]=p_p3d[p_mesh[0][k]].x;	x[1]=p_p3d[p_mesh[1][k]].x;	x[2]=p_p3d[p_mesh[2][k]].x;
		pd_max[0]=_Min_Max(x,3,pd_min[0]);
		// Get y value of 3 points included in current mesh triangle.
		y[0]=p_p3d[p_mesh[0][k]].y;	y[1]=p_p3d[p_mesh[1][k]].y;	y[2]=p_p3d[p_mesh[2][k]].y;
		pd_max[1]=_Min_Max(y,3,pd_min[1]);				
		// Set ROI rectangle which bounds current mesh triangle.
		pi_min[0] = min( max( (int)((pd_min[0]+1)/dx)+1, 0 ), ww_ray - 1);
		pi_min[1] = min( max( (int)((pd_min[1]+1)/dy)+1, 0 ), hh_ray - 1);
		pi_max[0] = min( max( (int)((pd_max[0]+1)/dx)+1, 0 ), ww_ray - 1);
		pi_max[1] = min( max( (int)((pd_max[1]+1)/dy)+1, 0 ), hh_ray - 1);

	//	printf("%d %d %d %d\n", pi_min[0], pi_max[0], pi_min[1], pi_max[1]);

		// Find intersections between DoCube rays and mesh of depth surface.
 		for(j=pi_min[1]; j<pi_max[1]; j++)
 		{
 			for(i=pi_min[0]; i<pi_max[0]; i++)
 			{
				//if(p_num_d[j][i]>=Do_G_INIT_SIZE_DEPTH)	continue;
				/// 교점 구할 때 ray repference point 가 잘못 생성 되는가?
				/// 교점 구할 때 ray repference point 가 잘못 생성 되는가?
				// Set current reference point.
 				rp_x = -1.0 + (double)i*dx;
 				rp_y = -1.0 + (double)j*dy;
				/// 교점 구할 때 ray repference point 가 잘못 생성 되는가?
				/// 교점 구할 때 ray repference point 가 잘못 생성 되는가?
 
 				// Check this reference point in valid region of mesh triangle.
 				if(_Is_inlier_of_the_mesh(rp_x, rp_y, x, y))
 				{
					// Get z value of 3 points included in current mesh triangle.
 					z[0]=p_p3d[p_mesh[0][k]].z;
 					z[1]=p_p3d[p_mesh[1][k]].z;
 					z[2]=p_p3d[p_mesh[2][k]].z;

 					// Edited by Yooji at 2016.08.29.
 					//_Intersection_ray_and_mesh(-1+i*dx, -1+j*dy, x,y,z, depth);
 					if(!_Intersection_ray_and_mesh_with_normal(rp_x, rp_y, x,y,z, depth, norm))	continue;

					// ===============================================================================
					// Check which ray intersects on the front face of mesh or on the back face.
					// + If ray intersects on the back face of mesh, encode offset (100.0) on depth value.
					//if(norm[2] > 0.0){	depth = depth + 100.0;	}
					// ===============================================================================
					if(in_ray_direction){
						if(depth < p_d[j][i].vp()[0])	p_d[j][i].se_Set_Element(0, depth);
					}
					else if(depth > p_d[j][i].vp()[0])	p_d[j][i].se_Set_Element(0, depth);

					//printf("depth: %f\n", depth);
  					//p_num_d[j][i]++;
 
 				}
 			}
 		}
	}

	return true;
}


//******************************************************************************
void VCL_Mesh_to_DoS::lfm_Load_From_Mesh(
	CKvSet_of_Point3D *in_ori_p3d,
	CKvMatrixInt      *in_ori_mesh,
	CKvSet2d_of_VectorFloat *io_analog_depth_map)
//******************************************************************************
{
	CKvSet2d_of_Vector convert_to_DoCube,starting_points;
	CKvSet_of_Point3D Hp3d;
	CKvMatrix H;
	CKvMatrixInt mat_num_layer;
	CKvMatrixFloat tt;
	CKvVector **p_DoCube;
	CKvRanker rank;
	CKvVectorFloat **p_analog_depth_map;
	float angle[3], *pp_analog_depth_map;
	double *pp_DoCube;
	int **p_num_layer,i,j,k;//,num_layers;
	bool odd_flag;

	//Convert from mesh to DoS
	angle[0] = zz_angle_x;
	angle[1] = zz_angle_y;
	angle[2] = zz_angle_z;

	_Center_and_Radius_of_3d_points_with_Homography(
		in_ori_p3d, 
		angle, 
		&Hp3d, 
		&H);
	
	_Converting_to_DoS(
		zz_width,
		zz_height,
		&Hp3d,
		in_ori_mesh,
		&starting_points,
		&convert_to_DoCube,
		&mat_num_layer);
	
	//Depth value sorting
	//Load analog depth data from converting to DoS
	odd_flag=false;
	p_DoCube		   = convert_to_DoCube.mp();
	p_num_layer		   = mat_num_layer.mp();
	p_analog_depth_map = io_analog_depth_map->mp();
	for(j=0; j<zz_height; j++){
		for(i=0; i<zz_width; i++){
			if( p_num_layer[j][i]!=0)
			{
				if((p_num_layer[j][i]%2)!=0)
				{ 
					odd_flag=true;
					if((p_num_layer[j][i]-1)!=0)
					{						
						//rank.s_Sort(&p_DoCube[j][i], false, (int)0, p_num_layer[j][i]-1,NULL);
						//pp_analog_depth_map = p_analog_depth_map[j][i].c_Create(p_num_layer[j][i]-1);
						//pp_DoCube = p_DoCube[j][i].vp();
						//for(k=0; k<(p_num_layer[j][i]-1); k++){pp_analog_depth_map[k] = (float)pp_DoCube[k];}
						p_num_layer[j][i]--;
						rank.s_Sort(&p_DoCube[j][i], false, (int)0, p_num_layer[j][i],NULL);
						pp_analog_depth_map = p_analog_depth_map[j][i].c_Create(p_num_layer[j][i]);
						pp_DoCube = p_DoCube[j][i].vp();
						for(k=0; k<(p_num_layer[j][i]); k++){pp_analog_depth_map[k] = (float)pp_DoCube[k];}
					}
					else
					{
						p_analog_depth_map[j][i].c_Create(2, (float)NOPOINTVALDoCube);
					}
				}
				else
				{
					rank.s_Sort(&p_DoCube[j][i], false, (int)0, p_num_layer[j][i],NULL);
					pp_analog_depth_map = p_analog_depth_map[j][i].c_Create(p_num_layer[j][i]);
					pp_DoCube = p_DoCube[j][i].vp();
					for(k=0; k<p_num_layer[j][i]; k++){pp_analog_depth_map[k] = (float)pp_DoCube[k];}
				}
			}
			else
			{
				p_analog_depth_map[j][i].c_Create(2, (float)NOPOINTVALDoCube);
			}
		}
	}
	if(odd_flag) Kv_Printf("the nuber of the points at a ray = ODD");
	return;
}
//******************************************************************************
bool VCL_Mesh_to_DoS::_Center_and_Radius_of_3d_points_with_Homography(
	CKvSet_of_Point3D *in_points,			//3D point set
	float *in_angle,						//Angle of the direction vector, 3 elements
	CKvSet_of_Point3D *out_Hpoints_or_NULL,	//3D point set with homography
	CKvMatrix *out_H)						//Homography (4x4, scale X Rotation)
//******************************************************************************
{
	LCKvUtility_for_Pmatrix3D util_p;
	LCKvAlgebra_for_Matrix alg_m;
	CKvMatrix R33, R44, S;
	int i,sz;
	CKvPoint3D *p_in, *p_out;
	double **p_R, **p_S, min[3], max[3], center[3], temp, buffer[3], radius;


	util_p.u_crm_Create_Rotation_Matrix((double)in_angle[0], (double)in_angle[1], (double)in_angle[2], &R33);

	p_R=R44.c_Create(4,4,0.);
	p_R[3][3]=1;
	R44.sb_Set_Block(0,0,&R33);

	p_in=in_points->vps(sz);
	p_out=out_Hpoints_or_NULL->c_Create(sz);

	// x,y,z scale 같게 하고자 할때
	min[0]=min[1]=min[2]=Do_G_MAXIMUM;
	max[0]=max[1]=max[2]=Do_G_MINIMUM;
	for(i=0;i<sz;i++)
	{
		p_out[i].x=p_R[0][0]*p_in[i].x+p_R[0][1]*p_in[i].y+p_R[0][2]*p_in[i].z;
		p_out[i].y=p_R[1][0]*p_in[i].x+p_R[1][1]*p_in[i].y+p_R[1][2]*p_in[i].z;
		p_out[i].z=p_R[2][0]*p_in[i].x+p_R[2][1]*p_in[i].y+p_R[2][2]*p_in[i].z;

		min[0]=(min[0]>p_out[i].x)?p_out[i].x:min[0];
		max[0]=(max[0]<p_out[i].x)?p_out[i].x:max[0];

		min[1]=(min[1]>p_out[i].y)?p_out[i].y:min[1];
		max[1]=(max[1]<p_out[i].y)?p_out[i].y:max[1];

		min[2]=(min[2]>p_out[i].z)?p_out[i].z:min[2];
		max[2]=(max[2]<p_out[i].z)?p_out[i].z:max[2];
	}

	center[0]=(max[0]+min[0])/2.;
	center[1]=(max[1]+min[1])/2.;
	center[2]=(max[2]+min[2])/2.;

	radius=max[0]-min[0];
	radius=((max[1]-min[1])<radius)?radius:(max[1]-min[1]);
	radius=((max[2]-min[2])<radius)?radius:(max[2]-min[2]);

	radius=radius*1.01;
	min[0]=center[0]-radius/2;
	max[0]=center[0]+radius/2;
	min[1]=center[1]-radius/2;
	max[1]=center[1]+radius/2;
	min[2]=center[2]-radius/2;
	max[2]=center[2]+radius/2;

	// min-max --> -1~1
	p_S=S.c_Create(4,4,0.);
	p_S[0][0]=2./(max[0]-min[0]);
	p_S[1][1]=2./(max[1]-min[1]);
	p_S[2][2]=2./(max[2]-min[2]);
	p_S[0][3]=-1*p_S[0][0]*center[0];
	p_S[1][3]=-1*p_S[1][1]*center[1];
	p_S[2][3]=-1*p_S[2][2]*center[2];
	p_S[3][3]=1.;

	alg_m.mmm_Multiply_Matrix_Matrix(&S, &R44, out_H);

	if(out_Hpoints_or_NULL!=NULL)
	{
		for(i=0;i<sz;i++)
		{
			buffer[0]=p_out[i].x;
			buffer[1]=p_out[i].y;
			buffer[2]=p_out[i].z;

			temp=p_S[3][0]*buffer[0]+p_S[3][1]*buffer[1]+p_S[3][2]*buffer[2]+p_S[3][3];
			if(temp==0) Kv_Printf("error~!");
			p_out[i].x=(p_S[0][0]*buffer[0]+p_S[0][1]*buffer[1]+p_S[0][2]*buffer[2]+p_S[0][3])/temp;
			p_out[i].y=(p_S[1][0]*buffer[0]+p_S[1][1]*buffer[1]+p_S[1][2]*buffer[2]+p_S[1][3])/temp;
			p_out[i].z=(p_S[2][0]*buffer[0]+p_S[2][1]*buffer[1]+p_S[2][2]*buffer[2]+p_S[2][3])/temp;		
		}
	}
	
	return true;
}
//******************************************************************************
bool VCL_Mesh_to_DoS::_Converting_to_DoS(
	int num_Dw,								//Number of the width ray
	int num_Dh,								//Number of the height ray
	CKvSet_of_Point3D *in_p3d,				//3D points set
	CKvMatrixInt *in_mesh,					//mesh index (3xN)
	CKvSet2d_of_Vector *out_starting_points,//Starting points at the DoS(x-y plane of the initial bound box: z=-1)
	CKvSet2d_of_Vector *out_depth,			//Depth of the each ray(range=-1 to 1)
	CKvMatrixInt *num_layer)				//number of the layer
//******************************************************************************
{
	int i,j,k,sz_p3d,sz_mesh,temp;
	CKvPoint3D *p_p3d;
	CKvVector **p_sp, **p_d;
	double *pp_sp;
	int **p_mesh;
	double dx,dy;
	int **p_num_d;

	p_p3d=in_p3d->vps(sz_p3d);
	p_sp=out_starting_points->c_Create(num_Dh, num_Dw);
	p_d=out_depth->c_Create(num_Dh, num_Dw);
	p_num_d=num_layer->c_Create(num_Dh, num_Dw, (int)0);
	p_mesh=in_mesh->mps(sz_mesh, temp);

	dx=2./num_Dw;
	dy=2./num_Dh;
	for(j=0;j<num_Dh;j++)
	{
		for(i=0;i<num_Dw;i++)
		{
			// Starting points
			pp_sp=p_sp[j][i].c_Create(3,1.);
			pp_sp[0]=(-1+dx*i);
			pp_sp[1]=(-1+dy*j);


			// Need to modify!!!!!!!!!!!!
			// Need to modify!!!!!!!!!!!!
			// Need to modify!!!!!!!!!!!!
			// Need to modify!!!!!!!!!!!!
			// Need to modify!!!!!!!!!!!!
			p_d[j][i].c_Create(Do_G_INIT_SIZE_DEPTH, -1); // Need to modify!!!!!!!!!!!!
		}
	}

	double min[2], max[2];
	double x[3],y[3],z[3],depth;
	int intmin[2], intmax[2];

	for(k=0;k<sz_mesh;k++)
	{
		x[0]=p_p3d[p_mesh[0][k]].x;
		x[1]=p_p3d[p_mesh[1][k]].x;
		x[2]=p_p3d[p_mesh[2][k]].x;
		max[0]=_Min_Max(x,3,min[0]);

		y[0]=p_p3d[p_mesh[0][k]].y;
		y[1]=p_p3d[p_mesh[1][k]].y;
		y[2]=p_p3d[p_mesh[2][k]].y;
		max[1]=_Min_Max(y,3,min[1]);

		intmin[0]=(int)((min[0]+1)/dx)+1;
		intmin[1]=(int)((min[1]+1)/dy)+1;
		intmax[0]=(int)((max[0]+1)/dx)+1;
		intmax[1]=(int)((max[1]+1)/dy)+1;

		for(j=intmin[1];j<intmax[1];j++)
		{
			for(i=intmin[0];i<intmax[0];i++)
			{
				if(_Is_inlier_of_the_mesh(-1+i*dx, -1+j*dy, x, y))
				{
					z[0]=p_p3d[p_mesh[0][k]].z;
					z[1]=p_p3d[p_mesh[1][k]].z;
					z[2]=p_p3d[p_mesh[2][k]].z;
					_Intersection_ray_and_mesh(-1+i*dx, -1+j*dy, x,y,z, depth);

					p_d[j][i].se_Set_Element(p_num_d[j][i], depth);
					p_num_d[j][i]++;

				}
			}
		}
	}
	return true;
}

//******************************************************************************
double VCL_Mesh_to_DoS::_Min_Max(
	double *in_data,
	int in_data_size,
	double &out_min)
//******************************************************************************
{
	int i;
	double max;
	max=Do_G_MINIMUM;
	out_min=Do_G_MAXIMUM;

	for(i=0;i<in_data_size;i++){
		max=(max<in_data[i])?in_data[i]:max;
		out_min=(out_min>in_data[i])?in_data[i]:out_min;
	}
	return max;	
}

//******************************************************************************
bool VCL_Mesh_to_DoS::_Is_inlier_of_the_mesh(
	double in_x,		// sample point : X value
	double in_y,		// sample point : Y value
	double *in_px,		// mesh point set : X set (size=3)
	double *in_py)		// mesh point set : Y set (size=3)
//******************************************************************************
{
	double v0[2], v1[2], v2[2], v0v0, v0v1, v1v1, v2v1, v2v0;
	double u,v,t;

	v2[0]=in_x-in_px[0];
	v2[1]=in_y-in_py[0];

	v1[0]=in_px[1]-in_px[0];
	v1[1]=in_py[1]-in_py[0];

	v0[0]=in_px[2]-in_px[0];
	v0[1]=in_py[2]-in_py[0];

	v0v0=v0[0]*v0[0]+v0[1]*v0[1];
	v0v1=v0[0]*v1[0]+v0[1]*v1[1];
	v1v1=v1[0]*v1[0]+v1[1]*v1[1];
	v2v1=v2[0]*v1[0]+v2[1]*v1[1];
	v2v0=v2[0]*v0[0]+v2[1]*v0[1];

	t=v0v0*v1v1-v0v1*v0v1;
	u=(v1v1*v2v0-v0v1*v2v1)/t;
	v=(v0v0*v2v1-v0v1*v2v0)/t;

	if(u>=0 && u<=1 && v>=0 && v<=1 && (u+v)<=1) return true;
	else return false;
}
//******************************************************************************
bool VCL_Mesh_to_DoS::_Intersection_ray_and_mesh(
	double in_x,		// sample point : X value
	double in_y,		// sample point : Y value (ray direction == z axis(0,0,1))
	double *in_px,		// mesh point set : X set (size=3)
	double *in_py,		// mesh point set : Y set (size=3)
	double *in_pz,		// mesh point set : Y set (size=3)
	double &out_z)		// Depth value
//******************************************************************************
{
	double v0[3],v1[3], p[3], d;

	v0[0]=in_px[1]-in_px[0];
	v0[1]=in_py[1]-in_py[0];
	v0[2]=in_pz[1]-in_pz[0];

	v1[0]=in_px[2]-in_px[0];
	v1[1]=in_py[2]-in_py[0];
	v1[2]=in_pz[2]-in_pz[0];

	_Cross_Product(v0,v1,p);

	// d=-1*(ax+by+cz)
	d=-1*(p[0]*in_px[0]+p[1]*in_py[0]+p[2]*in_pz[0]);

	// z=-1*(d+ax+by)/c
	if(p[2]==0) return false;
	out_z=-1*(d+p[0]*in_x+p[1]*in_y)/p[2];
	return true;
}

//******************************************************************************
bool VCL_Mesh_to_DoS::_Intersection_ray_and_mesh_with_normal(
		double in_x,		// sample point : X value
		double in_y,		// sample point : Y value (ray direction == z axis(0,0,1))
		double *in_px,		// mesh point set : X set (size=3)
		double *in_py,		// mesh point set : Y set (size=3)
		double *in_pz,		// mesh point set : Y set (size=3)
		double &out_z,		// Depth value
		double *out_norm)	// Surface normal
//******************************************************************************
{
	double v13[3],v12[3], p[3], d;
	double ip, theta;

	out_z = -1.0;

	v13[0]=in_px[2]-in_px[0];
	v13[1]=in_py[2]-in_py[0];
	v13[2]=in_pz[2]-in_pz[0];

	v12[0]=in_px[1]-in_px[0];
	v12[1]=in_py[1]-in_py[0];
	v12[2]=in_pz[1]-in_pz[0];

	// Compute normal vector of mesh triangle.
	_Cross_Product(v13,v12,p);	
	// Skip further process when plane normal of mesh triangle is perpendicular to z axis.
	d = sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2]);
	// Normalize surface normal.
	for(int i=0; i<3; i++)	p[i] = p[i]/d;
	ip = p[2];	// inner product of plane normal and z-axis.
	theta = acos(abs(ip))*180.0/PI;	// compute angle between z-axis and plane normal (theta: 0 ~ 90).
	
	if(theta > 75.0) return false;

	// Compute plane offset(?) d. ax+by+cz+d = 0.
	// d=-1*(ax+by+cz)
	d=-1*(p[0]*in_px[0]+p[1]*in_py[0]+p[2]*in_pz[0]);

	// Compute z value of intersection between ray and mesh triangle.
	// z=-1*(d+ax+by)/c	
	out_z=-1*(d+p[0]*in_x+p[1]*in_y)/p[2];	
	// Save surface normal.
	for(int i = 0; i<3; i++)	out_norm[i] = p[i];

	return true;
}

//******************************************************************************
void VCL_Mesh_to_DoS::_Cross_Product(
	double *in_data1,		
	double *in_data2,		
	double *out_data)		// (size=3)
//******************************************************************************
{
	out_data[0]=in_data1[1]*in_data2[2]-in_data1[2]*in_data2[1];
	out_data[1]=in_data1[2]*in_data2[0]-in_data1[0]*in_data2[2];
	out_data[2]=in_data1[0]*in_data2[1]-in_data1[1]*in_data2[0];
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//******************************************************************************
void VCL_Mesh_to_DoS::_Voxelization_of_DoCubeShort(
	CKvDoCubeShort *in_docube,
	int in_quantization_level,
	CKvVolumeBool *out_voxel) 
//******************************************************************************
{
	int i, j, k, l, w, h, sz;
	bool ***p_out;
	CKvSet2d_of_VectorShort *depth_value;
	CKvVectorShort **p_depth;
	short *pp_depth, sp, ep;

	depth_value=in_docube->gp_Get_Pointer();
	p_depth=depth_value->mps(w,h);
	p_out=out_voxel->c_Create(in_quantization_level, h, w, false);

	for(j=0;j<h;j++)
	{
		for(i=0;i<w;i++)
		{
			pp_depth=p_depth[j][i].vps(sz);
			if(pp_depth[0]!=NOPOINTVALDoCube)
			{
				for(k=0;k<sz;k+=2)
				{
					sp=pp_depth[k];
					ep=pp_depth[k+1];
					for(l=sp; l<=ep; l++)
						p_out[l][j][i]=true;					
				}
			}
		}
	}
	return;
}
//******************************************************************************
void VCL_Mesh_to_DoS::_Volume_to_CKvSet_of_Point3D(
	CKvVolumeBool *in_volume,
	CKvSet_of_Point3D *out_point3D) 
//******************************************************************************
{
	int i,j,k, w,h,d, num_point;
	bool ***p_v;
	double dx,dy,dz;
	CKvPoint3D *p_p3d;

	p_v=in_volume->tps(w,h,d);

	num_point=0;
	for(k=0;k<d;k++)
	{
		for(j=0;j<h;j++)
		{
			for(i=0;i<w;i++)
			{
				if(p_v[k][j][i])
				{
					num_point++;
				}
			}
		}
	}

	p_p3d=out_point3D->c_Create(num_point);
	dx=2./w;
	dy=2./h;
	dz=2./d;
	num_point=0;
	for(k=0;k<d;k++)
	{
		for(j=0;j<h;j++)
		{
			for(i=0;i<w;i++)
			{
				if(p_v[k][j][i])
				{
					p_p3d[num_point].x=-1+i*dx;
					p_p3d[num_point].y=-1+j*dy;
					p_p3d[num_point].z=-1+k*dz;
					num_point++;
				}
			}
		}
	}
	return;
}