/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_cube_tsdf_float.cpp 
/////////////////////////////////////////////////////////////////////////////////////////////

#include "../../../stdafx.h"
#include "../../_yooji_2016_object_scanning.h"

/////////////////////////////////////////////////////////////////////////////////////////////
// CKvYooji_Cube_TSDF_Float 
/////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
CKvYooji_Cube_TSDF_Float::CKvYooji_Cube_TSDF_Float(void)
//********************************************************************************************
{
	zz_classname="CKvYooji_Cube_TSDF_Float";
	//c_Create(Kv_Point3Df(0.0f,0.0f,0.0f),128,128,128,8);
}

//********************************************************************************************
CKvYooji_Cube_TSDF_Float::~CKvYooji_Cube_TSDF_Float(void)
//********************************************************************************************
{
}

//***********************************************************************************************************************
CKvYooji_Cube_TSDF_Float::CKvYooji_Cube_TSDF_Float(CKvYooji_Cube_TSDF_Float &a)
//***********************************************************************************************************************
{
	cp_Copy(&a);
}

//***********************************************************************************************************************
void CKvYooji_Cube_TSDF_Float::c_Create(
			CKvPoint3Df in_origin_in_world,
			int in_depth,
			int in_height,
			int in_width,
			int in_edge_length_of_sub_cube_in_voxel,
			float in_size_of_voxel)
//***********************************************************************************************************************
{
	CKvPoint3Df p3d, tp3d;
	int sz=in_edge_length_of_sub_cube_in_voxel;
	int i, j;

	float *p_vec_f;
	SHORT *tvec;

	if(sz<4) gerr("Edge length of sub-cube < 4");
	if((in_depth%sz)||(in_height%sz)||(in_width%sz)) gerr("Invalid cube dimension");
	if(in_size_of_voxel<=0.0f)	gerr("Invalid voxel size");

	// set origin point in the world coordinates.
	zz_org_point_in_world=in_origin_in_world;
	// set volume data.
	zz_vol_rgb.c_Create(in_depth,in_height,in_width);		// --> creation error...
	//////////////////////////////////////////////////////////////////////////
	// initialize TSDF values to 1.0f.
	zz_vol_depth.c_Create(in_depth,in_height,in_width,1.0f);	
	//////////////////////////////////////////////////////////////////////////
	zz_vol_w_depth.c_Create(in_depth,in_height,in_width,(UCHAR)0);
	zz_vol_w_rgb.c_Create(in_depth,in_height,in_width,(UCHAR)0);
	zz_vol_flag_of_sub_cube.c_Create(in_depth/sz,in_height/sz,in_width/sz,(char)KV_FLAG_SUBCUBE_INACTIVATED);
	
	// set parameters.
	zz_edge_length_of_sub_cube = in_edge_length_of_sub_cube_in_voxel;
	zz_sz_voxel = in_size_of_voxel;
	
	//if(!Kv_Printf("%d %d %d\n", in_depth, in_depth, in_depth)) exit(0);

	// set ray information for SFS.
	// + XY plane.
	// set initial visual hull segments.
 	zz_set_of_visual_hull_segments.c_Create(in_height, in_width);	
 	for(i=in_height*in_width-1; i>=0; i--){	
 		tvec=zz_set_of_visual_hull_segments.vp()[i].c_Create(2);
 		tvec[0]=(SHORT)0;	tvec[1]=(SHORT)(in_depth-1);
 	}

	// set reference points and direction vectors of DoCube.
	
 	zz_set_of_ref_pt_and_dir_vec.c_Create(in_height*in_width, 6);	// Nx6 matrix.
 	for(j=0; j<in_height; j++){
 		for(i=0; i<in_width; i++){

			tp3d.x = (float)i; tp3d.y = (float)j; tp3d.z = (float)in_depth/2.0f;
			//tp3d.x = (float)i; tp3d.y = (float)j; tp3d.z = 0.0f;
			// ======================================================================
			gpw_Get_Position_in_World(tp3d, p3d);
			//printf("p3d.z: %f\n", p3d.z);
			// ======================================================================
 			//gpw_Get_Position_in_World(Kv_Point3Df((float)i, (float)j, 0.0f), p3d);
 			p_vec_f=zz_set_of_ref_pt_and_dir_vec.mp()[j*in_width+i];
 			p_vec_f[0]=p3d.x;	p_vec_f[1]=p3d.y;	p_vec_f[2]=p3d.z;	// reference point.
 			p_vec_f[3]=0.0f;	p_vec_f[4]=0.0f;	p_vec_f[5]=1.0f;	// direction vector.
 		}
 	}
	// + ZY plane.
// 	zz_set_of_visual_hull_segments.c_Create(in_height, in_depth);	
// 	for(i=in_height*in_depth-1; i>=0; i--){	
// 		tvec=zz_set_of_visual_hull_segments.vp()[i].c_Create(2);
// 		tvec[0]=(SHORT)0;	tvec[1]=(SHORT)(in_width-1);
// 	}
// 
// 	zz_set_of_ref_pt_and_dir_vec.c_Create(in_height*in_depth, 6);	// Nx6 matrix.
// 	for(j=0; j<in_height; j++){
// 		for(i=0; i<in_depth; i++){
// 
// 			tp3d.x = (float)in_width/2.0f; tp3d.y = float(j); tp3d.z = float(i);
// 			// ======================================================================
// 			gpw_Get_Position_in_World(tp3d, p3d);
// 			// ======================================================================
// 			p_vec_f=zz_set_of_ref_pt_and_dir_vec.mp()[j*in_depth+i];
// 			p_vec_f[0]=p3d.x;	p_vec_f[1]=p3d.y;	p_vec_f[2]=p3d.z;	// reference point.
// 			p_vec_f[3]=1.0f;	p_vec_f[4]=0.0f;	p_vec_f[5]=0.0f;	// direction vector.
// 		}
// 	}

	return;
error:
	zpme("c_Create");
}

//***********************************************************************************************************************
void CKvYooji_Cube_TSDF_Float::ts_Volume_Width_Height_Depth(
			int &out_number_of_columns,
			int &out_number_of_rows,
			int &out_number_of_matrices)
//***********************************************************************************************************************
{
	zz_vol_depth.ts(out_number_of_columns,out_number_of_rows,out_number_of_matrices);
}

//***********************************************************************************************************************
void CKvYooji_Cube_TSDF_Float::ts(
			int &out_number_of_columns,
			int &out_number_of_rows,
			int &out_number_of_matrices)
//***********************************************************************************************************************
{
	zz_vol_depth.ts(out_number_of_columns,out_number_of_rows,out_number_of_matrices);
}

//***********************************************************************************************************************
//***********************************************************************************************************************
//***********************************************************************************************************************
//***********************************************************************************************************************
//***********************************************************************************************************************
void CKvYooji_Cube_TSDF_Float::cp_Copy( CKvYooji_Cube_TSDF_Float *in_set_of_elements)
//***********************************************************************************************************************
{
	zz_org_point_in_world=in_set_of_elements->zz_org_point_in_world;

	zz_vol_rgb.cp_Copy(&in_set_of_elements->zz_vol_rgb);
	zz_vol_depth.cp_Copy(&in_set_of_elements->zz_vol_depth);
	
	zz_set_of_visual_hull_segments.cp_Copy(&in_set_of_elements->zz_set_of_visual_hull_segments);
	zz_set_of_ref_pt_and_dir_vec.cp_Copy(&in_set_of_elements->zz_set_of_ref_pt_and_dir_vec);
	zz_vol_w_rgb.cp_Copy(&in_set_of_elements->zz_vol_w_rgb);
	zz_vol_w_depth.cp_Copy(&in_set_of_elements->zz_vol_w_depth);

	zz_vol_flag_of_sub_cube.cp_Copy(&in_set_of_elements->zz_vol_flag_of_sub_cube);
}

//***********************************************************************************************************************
void CKvYooji_Cube_TSDF_Float::gcciw_Get_Cube_Center_In_World(CKvPoint3Df &out_center_in_world) 
//***********************************************************************************************************************
{  
	int ww, hh, dd;
	float offset_x, offset_y, offset_z;

	ts(ww, hh, dd);
	
	offset_x = (float)ww*zz_sz_voxel/2.0f;
	offset_y = (float)hh*zz_sz_voxel/2.0f;
	offset_z = (float)dd*zz_sz_voxel/2.0f;

	out_center_in_world.x = zz_org_point_in_world.x + offset_x;
	out_center_in_world.y = zz_org_point_in_world.y + offset_y;
	out_center_in_world.z = zz_org_point_in_world.z + offset_z;
}


//***********************************************************************************************************************
bool CKvYooji_Cube_TSDF_Float::gpv_Get_Position_in_Voxel(
	CKvPoint3Df &in_point_in_world,
	CKvPoint3Df &out_point_in_voxel)
//***********************************************************************************************************************
{
	int ww, hh, dd;
	float x, y, z;
	bool flag_in_cube;
	flag_in_cube = true;

	// assume that position of origin point in the world is (-0.5, -0.5, -0.5) in voxel coordinates..
	x = (in_point_in_world.x - zz_org_point_in_world.x)/zz_sz_voxel - 0.5f;
	y = (in_point_in_world.y - zz_org_point_in_world.y)/zz_sz_voxel - 0.5f;
	z = (in_point_in_world.z - zz_org_point_in_world.z)/zz_sz_voxel - 0.5f;

	zz_vol_depth.ts(ww, hh ,dd);

	if(x<-0.5f || x>=(float)ww-0.5f || y<-0.5f || y>=(float)hh-0.5f || z<-0.5f || z>=(float)dd-0.5f)	
		flag_in_cube = false;

	out_point_in_voxel.x = x;	
	out_point_in_voxel.y = y;
	out_point_in_voxel.z = z;

	return flag_in_cube;
}

//***********************************************************************************************************************
bool CKvYooji_Cube_TSDF_Float::gpw_Get_Position_in_World(
	CKvPoint3Df &in_point_in_voxel,
	CKvPoint3Df &out_point_in_world)
//***********************************************************************************************************************
{
	int ww, hh, dd;
	float x, y, z;
	bool flag_in_cube;
	flag_in_cube = true;

	// assume that position of origin point in the world is (-0.5, -0.5, -0.5) in voxel coordinates..
	x = in_point_in_voxel.x; y = in_point_in_voxel.y; z = in_point_in_voxel.z;
	zz_vol_depth.ts(ww, hh ,dd);
	
	if(x<-0.5f || x>=(float)ww-0.5f || y<-0.5f || y>=(float)hh-0.5f || z<-0.5f || z>=(float)dd-0.5f)	
		flag_in_cube = false;

	out_point_in_world.x = (x + 0.5f)*zz_sz_voxel + zz_org_point_in_world.x;
	out_point_in_world.y = (y + 0.5f)*zz_sz_voxel + zz_org_point_in_world.y;
	out_point_in_world.z = (z + 0.5f)*zz_sz_voxel + zz_org_point_in_world.z;

	return flag_in_cube;
}

//********************************************************************************************
bool CKvYooji_Cube_TSDF_Float::grvu_Get_RGB_Value_Uninterpolated(
	CKvPoint3Df &in_point_in_voxel,
	CKvRgb &out_rgb)
//********************************************************************************************
{
	int ww, hh, dd;
	int x, y, z, tidx;

	out_rgb.r = out_rgb.g = out_rgb.b = (UCHAR)0;	// set default value.

	// calculate local indices.		
	x = ROUNDF(in_point_in_voxel.x);	y = ROUNDF(in_point_in_voxel.y);	z = ROUNDF(in_point_in_voxel.z);
	zz_vol_rgb.ts(ww, hh, dd);

	if(x<0 || y<0 || z<0 || x>ww-1 || y>hh-1 || z>dd-1)		return false;

	tidx = z*ww*hh + y*ww + x;

	if(zz_vol_w_depth.vp()[tidx] == (UCHAR)0)	return false;

	out_rgb.r = zz_vol_rgb.vp()[tidx];
	out_rgb.g = zz_vol_rgb.vp()[tidx + ww*hh*dd];
	out_rgb.b = zz_vol_rgb.vp()[tidx + ww*hh*dd*2];

	return true;
}

//********************************************************************************************
bool CKvYooji_Cube_TSDF_Float::grvi_Get_RGB_Value_Interpolated(
	CKvPoint3Df &in_point_in_voxel,
	CKvRgb &out_rgb)
//********************************************************************************************
{
	int ww_sc, hh_sc, dd_sc;

	//     5-----6
	//  1=====2
	//     7-----8
	//  3=====4
	/// Get TSDF values of 8 neighbor voxels.
	out_rgb.r = out_rgb.g = out_rgb.b = (UCHAR)0;	// set default value.
	// get offset voxel index and float residual vector of input 3d point.
	zz_offset[0] = in_point_in_voxel.x;	zz_residu[0] = in_point_in_voxel.x-zz_offset[0];
	zz_offset[1] = in_point_in_voxel.y;	zz_residu[1] = in_point_in_voxel.y-zz_offset[1];
	zz_offset[2] = in_point_in_voxel.z;	zz_residu[2] = in_point_in_voxel.z-zz_offset[2];

	zz_vol_flag_of_sub_cube.ts(ww_sc, hh_sc, dd_sc);

	// get TSDF values of neighbors on the front plane.
	zz_tpos.z = zz_offset[2];	// set Z value.

	zz_tpos.x = zz_offset[0];	zz_tpos.y = zz_offset[1];
	if(!grvu_Get_RGB_Value_Uninterpolated(zz_tpos, zz_front_rgb[0]))	return false;		// 1
	zz_tpos.x = zz_offset[0]+1;	zz_tpos.y = zz_offset[1];
	if(!grvu_Get_RGB_Value_Uninterpolated(zz_tpos, zz_front_rgb[1]))	return false;		// 2	
	zz_tpos.x = zz_offset[0];	zz_tpos.y = zz_offset[1]+1;
	if(!grvu_Get_RGB_Value_Uninterpolated(zz_tpos, zz_front_rgb[2]))	return false;		// 3
	zz_tpos.x = zz_offset[0]+1;	zz_tpos.y = zz_offset[1]+1;
	if(!grvu_Get_RGB_Value_Uninterpolated(zz_tpos, zz_front_rgb[3]))	return false;		// 4

	// get TSDF values of neighbors on the back plane.
	zz_tpos.z = zz_offset[2]+1;	// set Z value.

	zz_tpos.x = zz_offset[0];	zz_tpos.y = zz_offset[1];
	if(!grvu_Get_RGB_Value_Uninterpolated(zz_tpos, zz_back_rgb[0]))	return false;		// 5	
	zz_tpos.x = zz_offset[0]+1;	zz_tpos.y = zz_offset[1];
	if(!grvu_Get_RGB_Value_Uninterpolated(zz_tpos, zz_back_rgb[1]))	return false;		// 6
	zz_tpos.x = zz_offset[0];	zz_tpos.y = zz_offset[1]+1;
	if(!grvu_Get_RGB_Value_Uninterpolated(zz_tpos, zz_back_rgb[2]))	return false;		// 7
	zz_tpos.x = zz_offset[0]+1;	zz_tpos.y = zz_offset[1]+1;
	if(!grvu_Get_RGB_Value_Uninterpolated(zz_tpos, zz_back_rgb[3]))	return false;		// 8

	// interpolates TSDF value on XY plane.
	// + R
	zz_inter_f_rgb.r = (1.0f-zz_residu[1])*((1.0f-zz_residu[0])*(float)zz_front_rgb[0].r +zz_residu[0]*(float)zz_front_rgb[1].r)
		+zz_residu[1]*((1.0f-zz_residu[0])*(float)zz_front_rgb[2].r +zz_residu[0]*(float)zz_front_rgb[3].r);
	zz_inter_b_rgb.r = (1.0f-zz_residu[1])*((1.0f-zz_residu[0])*(float)zz_back_rgb[0].r +zz_residu[0]*(float)zz_back_rgb[1].r)
		+zz_residu[1]*((1.0f-zz_residu[0])*(float)zz_back_rgb[2].r +zz_residu[0]*(float)zz_back_rgb[3].r);

	out_rgb.r = (UCHAR)((1.0f-zz_residu[2])*zz_inter_f_rgb.r +zz_residu[2]*zz_inter_b_rgb.r);
	// + G
	zz_inter_f_rgb.g = (1.0f-zz_residu[1])*((1.0f-zz_residu[0])*(float)zz_front_rgb[0].g +zz_residu[0]*(float)zz_front_rgb[1].g)
		+zz_residu[1]*((1.0f-zz_residu[0])*(float)zz_front_rgb[2].g +zz_residu[0]*(float)zz_front_rgb[3].g);
	zz_inter_b_rgb.g = (1.0f-zz_residu[1])*((1.0f-zz_residu[0])*(float)zz_back_rgb[0].g +zz_residu[0]*(float)zz_back_rgb[1].g)
		+zz_residu[1]*((1.0f-zz_residu[0])*(float)zz_back_rgb[2].g +zz_residu[0]*(float)zz_back_rgb[3].g);

	out_rgb.g = (UCHAR)((1.0f-zz_residu[2])*zz_inter_f_rgb.g +zz_residu[2]*zz_inter_b_rgb.g);
	// + B.
	zz_inter_f_rgb.b = (1.0f-zz_residu[1])*((1.0f-zz_residu[0])*(float)zz_front_rgb[0].b +zz_residu[0]*(float)zz_front_rgb[1].b)
		+zz_residu[1]*((1.0f-zz_residu[0])*(float)zz_front_rgb[2].b +zz_residu[0]*(float)zz_front_rgb[3].b);
	zz_inter_b_rgb.b = (1.0f-zz_residu[1])*((1.0f-zz_residu[0])*(float)zz_back_rgb[0].b +zz_residu[0]*(float)zz_back_rgb[1].b)
		+zz_residu[1]*((1.0f-zz_residu[0])*(float)zz_back_rgb[2].b +zz_residu[0]*(float)zz_back_rgb[3].b);

	out_rgb.b = (UCHAR)((1.0f-zz_residu[2])*zz_inter_f_rgb.b +zz_residu[2]*zz_inter_b_rgb.b);

	return true;
}

//********************************************************************************************
bool CKvYooji_Cube_TSDF_Float::gtvu_Get_TSDF_Value_Uninterpolated(
	CKvPoint3Df &in_point_in_voxel, 
	float &out_tsdf)
//********************************************************************************************
{
	int ww, hh, dd;
	int x, y, z, tidx;

	out_tsdf=1.0f;	// set default value.
	// calculate local indices.		
	x=ROUNDF(in_point_in_voxel.x);	y=ROUNDF(in_point_in_voxel.y);	z=ROUNDF(in_point_in_voxel.z);
	zz_vol_depth.ts(ww, hh ,dd);

	if(x<0 || y<0 || z<0 || x>ww-1 || y>hh-1 || z>dd-1)		return false;

	tidx=z*ww*hh + y*ww + x;

	if(zz_vol_w_depth.vp()[tidx] == (UCHAR)0)	return false;
	out_tsdf=zz_vol_depth.vp()[tidx];

	return true;
}


//********************************************************************************************
bool CKvYooji_Cube_TSDF_Float::gtvi_Get_TSDF_Value_Interpolated(	
	CKvPoint3Df &in_point_in_voxel, 
	float &out_tsdf)
//********************************************************************************************
{
	int ww_sc, hh_sc, dd_sc;	

	//     5-----6
	//  1=====2
	//     7-----8
	//  3=====4
	/// Get TSDF values of 8 neighbor voxels.
	out_tsdf=1.0f;
	// get offset voxel index and float residual vector of input 3d point.
	zz_offset[0]=in_point_in_voxel.x;	zz_residu[0]=in_point_in_voxel.x-zz_offset[0];
	zz_offset[1]=in_point_in_voxel.y;	zz_residu[1]=in_point_in_voxel.y-zz_offset[1];
	zz_offset[2]=in_point_in_voxel.z;	zz_residu[2]=in_point_in_voxel.z-zz_offset[2];

	zz_vol_flag_of_sub_cube.ts(ww_sc, hh_sc, dd_sc);

	// get TSDF values of neighbors on the front plane.
	zz_tpos.z=zz_offset[2];	// set Z value.

	zz_tpos.x=zz_offset[0];	zz_tpos.y=zz_offset[1];	
	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_front[0]))	return false;		// 1
	zz_tpos.x=zz_offset[0]+1;	zz_tpos.y=zz_offset[1];	
	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_front[1]))	return false;		// 2	
	zz_tpos.x=zz_offset[0];	zz_tpos.y=zz_offset[1]+1;
	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_front[2]))	return false;		// 3
	zz_tpos.x=zz_offset[0]+1;	zz_tpos.y=zz_offset[1]+1;	
	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_front[3]))	return false;		// 4

	// get TSDF values of neighbors on the back plane.
	zz_tpos.z=zz_offset[2]+1;	// set Z value.

	zz_tpos.x=zz_offset[0];	zz_tpos.y=zz_offset[1];	
	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_back[0]))	return false;		// 5	
	zz_tpos.x=zz_offset[0]+1;	zz_tpos.y=zz_offset[1];	
	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_back[1]))	return false;		// 6
	zz_tpos.x=zz_offset[0];	zz_tpos.y=zz_offset[1]+1;	
	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_back[2]))	return false;		// 7
	zz_tpos.x=zz_offset[0]+1;	zz_tpos.y=zz_offset[1]+1;	
	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_back[3]))	return false;		// 8
	
	// interpolates TSDF value on XY plane.
	zz_inter_f=(1.0f-zz_residu[1])*( (1.0f-zz_residu[0])*zz_front[0] +zz_residu[0]*zz_front[1] ) 
		+zz_residu[1]*( (1.0f-zz_residu[0])*zz_front[2] +zz_residu[0]*zz_front[3] );
	zz_inter_b=(1.0f-zz_residu[1])*( (1.0f-zz_residu[0])*zz_back[0] +zz_residu[0]*zz_back[1] ) 
		+zz_residu[1]*( (1.0f-zz_residu[0])*zz_back[2] +zz_residu[0]*zz_back[3] );

	out_tsdf=(1.0f-zz_residu[2])*zz_inter_f +zz_residu[2]*zz_inter_b;

	return true;
}

//********************************************************************************************
bool CKvYooji_Cube_TSDF_Float::gwdu_Get_Weight_Depth_Uninterpolated(
	CKvPoint3Df &in_point_in_voxel, 
	float &out_w_depth)
//********************************************************************************************
{
	int ww, hh, dd;
	int x, y, z, tidx;

	out_w_depth=0.0f;	// set default value.
	// calculate local indices.		
	x=ROUNDF(in_point_in_voxel.x);	y=ROUNDF(in_point_in_voxel.y);	z=ROUNDF(in_point_in_voxel.z);
	zz_vol_w_depth.ts(ww, hh ,dd);

	if(x<0 || y<0 || z<0 || x>ww-1 || y>hh-1 || z>dd-1)		return false;

	tidx=z*ww*hh + y*ww + x;

	if(zz_vol_w_depth.vp()[tidx] == (UCHAR)0)	return false;
	out_w_depth=zz_vol_w_depth.vp()[tidx];

	return true;
}


//********************************************************************************************
bool CKvYooji_Cube_TSDF_Float::gwdi_Get_Weight_Depth_Interpolated(	
	CKvPoint3Df &in_point_in_voxel, 
	float &out_w_depth)
//********************************************************************************************
{
	int ww_sc, hh_sc, dd_sc;	

	//     5-----6
	//  1=====2
	//     7-----8
	//  3=====4
	/// Get TSDF values of 8 neighbor voxels.
	out_w_depth=0.0f;
	// get offset voxel index and float residual vector of input 3d point.
	zz_offset[0]=(int)in_point_in_voxel.x;	zz_residu[0]=in_point_in_voxel.x-zz_offset[0];
	zz_offset[1]=(int)in_point_in_voxel.y;	zz_residu[1]=in_point_in_voxel.y-zz_offset[1];
	zz_offset[2]=(int)in_point_in_voxel.z;	zz_residu[2]=in_point_in_voxel.z-zz_offset[2];

	zz_vol_flag_of_sub_cube.ts(ww_sc, hh_sc, dd_sc);

	// get TSDF values of neighbors on the front plane.
	zz_tpos.z=zz_offset[2];	// set Z value.

	zz_tpos.x=zz_offset[0];	zz_tpos.y=zz_offset[1];	
	if(!gwdu_Get_Weight_Depth_Uninterpolated(zz_tpos, zz_front[0]))	return false;	// 1
	zz_tpos.x=zz_offset[0]+1;	zz_tpos.y=zz_offset[1];	
	if(!gwdu_Get_Weight_Depth_Uninterpolated(zz_tpos, zz_front[1]))	return false;	// 2	
	zz_tpos.x=zz_offset[0];	zz_tpos.y=zz_offset[1]+1;
	if(!gwdu_Get_Weight_Depth_Uninterpolated(zz_tpos, zz_front[2]))	return false;	// 3
	zz_tpos.x=zz_offset[0]+1;	zz_tpos.y=zz_offset[1]+1;	
	if(!gwdu_Get_Weight_Depth_Uninterpolated(zz_tpos, zz_front[3]))	return false;	// 4

	// get TSDF values of neighbors on the back plane.
	zz_tpos.z=zz_offset[2]+1;	// set Z value.

	zz_tpos.x=zz_offset[0];	zz_tpos.y=zz_offset[1];	
	if(!gwdu_Get_Weight_Depth_Uninterpolated(zz_tpos, zz_back[0]))	return false;	// 5	
	zz_tpos.x=zz_offset[0]+1;	zz_tpos.y=zz_offset[1];	
	if(!gwdu_Get_Weight_Depth_Uninterpolated(zz_tpos, zz_back[1]))	return false;	// 6
	zz_tpos.x=zz_offset[0];	zz_tpos.y=zz_offset[1]+1;	
	if(!gwdu_Get_Weight_Depth_Uninterpolated(zz_tpos, zz_back[2]))	return false;	// 7
	zz_tpos.x=zz_offset[0]+1;	zz_tpos.y=zz_offset[1]+1;	
	if(!gwdu_Get_Weight_Depth_Uninterpolated(zz_tpos, zz_back[3]))	return false;	// 8
	
	// interpolates TSDF value on XY plane.
	zz_inter_f=(1.0f-zz_residu[1])*( (1.0f-zz_residu[0])*zz_front[0] +zz_residu[0]*zz_front[1] ) 
		+zz_residu[1]*( (1.0f-zz_residu[0])*zz_front[2] +zz_residu[0]*zz_front[3] );
	zz_inter_b=(1.0f-zz_residu[1])*( (1.0f-zz_residu[0])*zz_back[0] +zz_residu[0]*zz_back[1] ) 
		+zz_residu[1]*( (1.0f-zz_residu[0])*zz_back[2] +zz_residu[0]*zz_back[3] );

	out_w_depth=(1.0f-zz_residu[2])*zz_inter_f +zz_residu[2]*zz_inter_b;

	return true;
}

//********************************************************************************************
bool CKvYooji_Cube_TSDF_Float::csnt_Compute_Surface_Normal_from_TSDF(CKvPoint3Df &in_p3d,	
	CKvPoint3Df &out_surf_norm)
//********************************************************************************************
{
	// we need total 32 neighbors for computing a single surface normal of input 3D point. (refer following pictures.)
	// the offset voxel index is 4 in front XY plane.
	// + the foremost XY slice (Z=-1)
	//    -- X      
	// Y |
	//       1     2      
	//          X
	//       3     4     
	//
	//            
	//float XY_foremost[4];
	// + front XY slice (Z=0)
	//       1     2
	//
	// 3     4     5     6
	//          X
	// 7     8     9     10
	//
	//       11    12
	//float XY_front[12];
	// + back XY slice (Z=1)
	//       1     2
	//
	// 3     4     5     6
	//          X
	// 7     8     9     10
	//
	//       11    12
	//float XY_back[12];
	// + the backmost XY slice (Z=2)
	//       
	//
	//       1     2      
	//          X
	//       3     4     
	//
	//            
	//float XY_backmost[4];

	/// Get TSDF values of 32 neighbor voxels.
	float sz_voxel;
	// coverts input 3D point to position of voxel coordinates.
	goiw_Get_Origin_In_World(zz_origin);
	sz_voxel = svm_Size_of_Voxel_in_Meter();
	zz_pos_vox[0]=(in_p3d.x-zz_origin.x)/sz_voxel -0.5f;
	zz_pos_vox[1]=(in_p3d.y-zz_origin.y)/sz_voxel -0.5f;
	zz_pos_vox[2]=(in_p3d.z-zz_origin.z)/sz_voxel -0.5f;
	// get offset voxel index and float residual vector of input 3d point.
	for(int i=0; i<3; i++){	zz_offset[i]=(int)zz_pos_vox[i];	zz_residu[i]=zz_pos_vox[i]-(float)zz_offset[i]; }

	// get TSDF values of neighbors on the foremost XY plane.
	zz_tpos.z=zz_offset[2]-1;	// set Z value.

	zz_tpos.x=zz_offset[0];	zz_tpos.y=zz_offset[1];	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_foremost[0]))	return false;	// 1
	zz_tpos.x=zz_offset[0]+1;	zz_tpos.y=zz_offset[1];	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_foremost[1]))	return false;	// 2
	zz_tpos.x=zz_offset[0];	zz_tpos.y=zz_offset[1]+1;	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_foremost[2]))	return false;	// 3
	zz_tpos.x=zz_offset[0]+1;	zz_tpos.y=zz_offset[1]+1;	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_foremost[3]))	return false;	// 4

	// get TSDF values of neighbors on the front XY plane.
	zz_tpos.z=zz_offset[2];	// set Z value.

	zz_tpos.x=zz_offset[0];	zz_tpos.y=zz_offset[1]-1;	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_front[0]))	return false;	// 1
	zz_tpos.x=zz_offset[0]+1;	zz_tpos.y=zz_offset[1]-1;	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_front[1]))	return false;	// 2

	zz_tpos.x=zz_offset[0]-1;	zz_tpos.y=zz_offset[1];	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_front[2]))	return false;	// 3	
	zz_tpos.x=zz_offset[0];	zz_tpos.y=zz_offset[1];	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_front[3]))	return false;		// 4
	zz_tpos.x=zz_offset[0]+1;	zz_tpos.y=zz_offset[1];	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_front[4]))	return false;	// 5
	zz_tpos.x=zz_offset[0]+2;	zz_tpos.y=zz_offset[1];	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_front[5]))	return false;	// 6

	zz_tpos.x=zz_offset[0]-1;	zz_tpos.y=zz_offset[1]+1;	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_front[6]))	return false;	// 7
	zz_tpos.x=zz_offset[0];	zz_tpos.y=zz_offset[1]+1;	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_front[7]))	return false;	// 8
	zz_tpos.x=zz_offset[0]+1;	zz_tpos.y=zz_offset[1]+1;	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_front[8]))	return false;	// 9
	zz_tpos.x=zz_offset[0]+2;	zz_tpos.y=zz_offset[1]+1;	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_front[9]))	return false;	// 10

	zz_tpos.x=zz_offset[0];	zz_tpos.y=zz_offset[1]+2;	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_front[10]))	return false;	// 11
	zz_tpos.x=zz_offset[0]+1;	zz_tpos.y=zz_offset[1]+2;	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_front[11]))	return false;	// 12

	// get TSDF values of neighbors on the back XY plane.
	zz_tpos.z=zz_offset[2]+1;	// set Z value.

	zz_tpos.x=zz_offset[0];	zz_tpos.y=zz_offset[1]-1;	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_back[0]))	return false;	// 1
	zz_tpos.x=zz_offset[0]+1;	zz_tpos.y=zz_offset[1]-1;	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_back[1]))	return false;	// 2

	zz_tpos.x=zz_offset[0]-1;	zz_tpos.y=zz_offset[1];	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_back[2]))	return false;	// 3	
	zz_tpos.x=zz_offset[0];	zz_tpos.y=zz_offset[1];	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_back[3]))	return false;	// 4
	zz_tpos.x=zz_offset[0]+1;	zz_tpos.y=zz_offset[1];	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_back[4]))	return false;	// 5
	zz_tpos.x=zz_offset[0]+2;	zz_tpos.y=zz_offset[1];	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_back[5]))	return false;	// 6

	zz_tpos.x=zz_offset[0]-1;	zz_tpos.y=zz_offset[1]+1;	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_back[6]))	return false;	// 7
	zz_tpos.x=zz_offset[0];	zz_tpos.y=zz_offset[1]+1;	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_back[7]))	return false;	// 8
	zz_tpos.x=zz_offset[0]+1;	zz_tpos.y=zz_offset[1]+1;	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_back[8]))	return false;	// 9
	zz_tpos.x=zz_offset[0]+2;	zz_tpos.y=zz_offset[1]+1;	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_back[9]))	return false;	// 10

	zz_tpos.x=zz_offset[0];	zz_tpos.y=zz_offset[1]+2;	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_back[10]))	return false;	// 11
	zz_tpos.x=zz_offset[0]+1;	zz_tpos.y=zz_offset[1]+2;	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_back[11]))	return false;	// 12

	// get TSDF values of neighbors on the backmost XY plane.
	zz_tpos.z=zz_offset[2]+2;	// set Z value.

	zz_tpos.x=zz_offset[0];	zz_tpos.y=zz_offset[1];	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_backmost[0]))	return false;	// 1
	zz_tpos.x=zz_offset[0]+1;	zz_tpos.y=zz_offset[1];	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_backmost[1]))	return false;	// 2
	zz_tpos.x=zz_offset[0];	zz_tpos.y=zz_offset[1]+1;	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_backmost[2]))	return false;	// 3
	zz_tpos.x=zz_offset[0]+1;	zz_tpos.y=zz_offset[1]+1;	if(!gtvu_Get_TSDF_Value_Uninterpolated(zz_tpos, zz_XY_backmost[3]))	return false;	// 4

	/// Calculate surface normal.
	float inter2a[2], inter4a[4], inter2b[2], inter4b[4];
	float dx, dy, dz;
	// compute x-direction SDF gradient at point.
	// + get z-direction interpolated SDF values of (3 4 5 6) and (7 8 9 10).
	inter4a[0]=(1.0f-zz_residu[2])*zz_XY_front[2] +zz_residu[2]*zz_XY_back[2];	
	inter4a[1]=(1.0f-zz_residu[2])*zz_XY_front[3] +zz_residu[2]*zz_XY_back[3];
	inter4a[2]=(1.0f-zz_residu[2])*zz_XY_front[4] +zz_residu[2]*zz_XY_back[4];	
	inter4a[3]=(1.0f-zz_residu[2])*zz_XY_front[5] +zz_residu[2]*zz_XY_back[5];

	inter4b[0]=(1.0f-zz_residu[2])*zz_XY_front[6] +zz_residu[2]*zz_XY_back[6];	
	inter4b[1]=(1.0f-zz_residu[2])*zz_XY_front[7] +zz_residu[2]*zz_XY_back[7];
	inter4b[2]=(1.0f-zz_residu[2])*zz_XY_front[8] +zz_residu[2]*zz_XY_back[8];	
	inter4b[3]=(1.0f-zz_residu[2])*zz_XY_front[9] +zz_residu[2]*zz_XY_back[9];	
	
	// + dF(x, y, z)/dx = F(x+1, y, z)-F(x-1, y, z).
	inter2a[0]=(1.0f-zz_residu[0])*inter4a[0] +zz_residu[0]*inter4a[1];	// x-direction interpolation of 3 and 4.
	inter2a[1]=(1.0f-zz_residu[0])*inter4b[0] +zz_residu[0]*inter4b[1];	// x-direction interpolation of 7 and 8.
	inter2b[0]=(1.0f-zz_residu[0])*inter4a[2] +zz_residu[0]*inter4a[3];	// x-direction interpolation of 5 and 6.
	inter2b[1]=(1.0f-zz_residu[0])*inter4b[2] +zz_residu[0]*inter4b[3];	// x-direction interpolation of 9 and 10.


	dx=(1.0f-zz_residu[1])*inter2b[0]+zz_residu[1]*inter2b[1];			// y-direction interpolation of (5 6) and (9 10).
	dx-=(1.0f-zz_residu[1])*inter2a[0]+zz_residu[1]*inter2a[1];		// y-direction interpolation of (3 4) and (5 6).


	// compute y-direction SDF gradient at point.
	// + get z-direction interpolated SDF values of (1 4 8 11) and (2 5 9 12).
	inter4a[0]=(1.0f-zz_residu[2])*zz_XY_front[0]	+zz_residu[2]*zz_XY_back[0];	
	inter4a[1]=(1.0f-zz_residu[2])*zz_XY_front[3]	+zz_residu[2]*zz_XY_back[3];
	inter4a[2]=(1.0f-zz_residu[2])*zz_XY_front[7]	+zz_residu[2]*zz_XY_back[7];	
	inter4a[3]=(1.0f-zz_residu[2])*zz_XY_front[10]	+zz_residu[2]*zz_XY_back[10];

	inter4b[0]=(1.0f-zz_residu[2])*zz_XY_front[1]	+zz_residu[2]*zz_XY_back[1];	
	inter4b[1]=(1.0f-zz_residu[2])*zz_XY_front[4]	+zz_residu[2]*zz_XY_back[4];
	inter4b[2]=(1.0f-zz_residu[2])*zz_XY_front[8]	+zz_residu[2]*zz_XY_back[8];	
	inter4b[3]=(1.0f-zz_residu[2])*zz_XY_front[11]	+zz_residu[2]*zz_XY_back[11];

	// + dF(x, y, z)/dx = F(x, y+1, z)-F(x, y-1, z).
	inter2a[0]=(1.0f-zz_residu[1])*inter4a[0] +zz_residu[1]*inter4a[1];	// y-direction interpolation of 1 and 4.
	inter2a[1]=(1.0f-zz_residu[1])*inter4b[0] +zz_residu[1]*inter4b[1];	// y-direction interpolation of 2 and 5.
	inter2b[0]=(1.0f-zz_residu[1])*inter4a[2] +zz_residu[1]*inter4a[3];	// y-direction interpolation of 8 and 11.
	inter2b[1]=(1.0f-zz_residu[1])*inter4b[2] +zz_residu[1]*inter4b[3];	// y-direction interpolation of 9 and 12.


	dy=(1.0f-zz_residu[0])*inter2b[0]	+zz_residu[0]*inter2b[1];			// x-direction interpolation of (8 11) and (9 12).
	dy-=(1.0f-zz_residu[0])*inter2a[0]	+zz_residu[0]*inter2a[1];			// x-direction interpolation of (1 4) and (2 5).

	// compute z-direction SDF gradient at point.
	// + get y-direction interpolated SDF values of (X=0|Z=-1 0 1 2) and (X=1|Z=-1 0 1 2).
	inter4a[0]=(1.0f-zz_residu[1])*zz_XY_foremost[0]	+zz_residu[1]*zz_XY_foremost[2];		
	inter4a[1]=(1.0f-zz_residu[1])*zz_XY_front[3]		+zz_residu[1]*zz_XY_front[7];
	inter4a[2]=(1.0f-zz_residu[1])*zz_XY_back[3]		+zz_residu[1]*zz_XY_back[7];				
	inter4a[3]=(1.0f-zz_residu[1])*zz_XY_backmost[0]	+zz_residu[1]*zz_XY_backmost[2];

	inter4b[0]=(1.0f-zz_residu[1])*zz_XY_foremost[1]	+zz_residu[1]*zz_XY_foremost[3];		
	inter4b[1]=(1.0f-zz_residu[1])*zz_XY_front[4]		+zz_residu[1]*zz_XY_front[8];
	inter4b[2]=(1.0f-zz_residu[1])*zz_XY_back[4]		+zz_residu[1]*zz_XY_back[8];				
	inter4b[3]=(1.0f-zz_residu[1])*zz_XY_backmost[1]	+zz_residu[1]*zz_XY_backmost[3];
	
	// + dF(x, y, z)/dx = F(x, y, z+1)-F(x, y, z-1).
	inter2a[0]=(1.0f-zz_residu[2])*inter4a[0] +zz_residu[2]*inter4a[1];	// z-direction interpolation of (X=0|Z=-1) and (X=0|Z=0).
	inter2a[1]=(1.0f-zz_residu[2])*inter4b[0] +zz_residu[2]*inter4b[1];	// z-direction interpolation of (X=1|Z=-1) and (X=1|Z=0).
	inter2b[0]=(1.0f-zz_residu[2])*inter4a[2] +zz_residu[2]*inter4a[3];	// z-direction interpolation of (X=0|Z=1) and (X=0|Z=2).
	inter2b[1]=(1.0f-zz_residu[2])*inter4b[2] +zz_residu[2]*inter4b[3];	// z-direction interpolation of (X=1|Z=1) and (X=1|Z=2).


	dz=(1.0f-zz_residu[0])*inter2b[0]+zz_residu[0]*inter2b[1];			// x-direction interpolation of (X=0|Z= 1 2) and (X=1|Z= 1 2).
	dz-=(1.0f-zz_residu[0])*inter2a[0]+zz_residu[0]*inter2a[1];			// x-direction interpolation of (X=0|Z=-1 0) and (X=1|Z=-1 0).

	if(dx==0.0f && dy==0.0f && dz==0.0f)	return false;

	// save calculation result of surface normal.
	out_surf_norm.x=dx;
	out_surf_norm.y=dy;
	out_surf_norm.z=dz;
		
	out_surf_norm.n_Normalize();

	return true;
	
}