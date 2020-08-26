#include "../../stdafx.h"
//#include "VCL_Inhand_Object_Scanning.h"
//#include "VCL_Inhand_Object_ScanningDlg.h"
//#include "_VCL_DoCube.h"
#include "../_VCL_2014_3DO.h"

VCL_DoCube::VCL_DoCube()
{
}
VCL_DoCube::~VCL_DoCube()
{
}
//****************************************************************************//
//****************************************************************************//
//****************************************************************************//
//****************************************************************************//
//**********************DoCube Declaration************************************//
//****************************************************************************//
//****************************************************************************//
//****************************************************************************//
//****************************************************************************//
//****************************************************************************//
void VCL_DoCube::sc_Set_Cube(
	int in_width,
	int in_height,
	int in_depth)
//******************************************************************************
{
	cn_Create_Null_depth_map(in_height,in_width);
	sr_Set_Resolution(in_width,in_height,in_depth);
	srpdv_Set_Reference_Points_and_Direction_Vectors(in_width,in_height);
}
//******************************************************************************
void VCL_DoCube::sr_Set_Resolution(
	int in_width,
	int in_height,
	int in_depth)
//******************************************************************************
{
	zz_width			= clip(4,2048, in_width/2*2);
	zz_height			= clip(4,2048, in_height/2*2);
	zz_number_of_levels = clip(4,2048, in_depth/2*2);
}
//******************************************************************************
void VCL_DoCube::srpdv_Set_Reference_Points_and_Direction_Vectors(
	int in_width,
	int in_height)
//******************************************************************************
{
	mrd_Make_set_of_Reference_points_and_Direction_vectors(
		in_width,
		in_height,
		&zz_set_of_reference_and_direction_vectors);
}
//******************************************************************************
void VCL_DoCube::gr_Get_Resolution(
	int &out_width,
	int &out_height,
	int &out_depth)
//******************************************************************************
{
	out_width  = zz_width;
	out_height = zz_height;
	out_depth  = zz_number_of_levels;
}
//******************************************************************************
CKvMatrixFloat * VCL_DoCube::grpdv_Get_Reference_Points_and_Direction_Vectors()
//******************************************************************************
{
	return &zz_set_of_reference_and_direction_vectors;
}
//******************************************************************************
void VCL_DoCube::gvr_Get_Valid_Rays(int &out_valid_rays)
//******************************************************************************
{
	CKvSet2d_of_VectorShort *depth_map;
	CKvVectorShort **p_depth_map;
	short *pp_depth_map;
	int i,j,ww,hh;

	depth_map      = this->gp_Get_Pointer();
	p_depth_map    = depth_map->mps(ww,hh);
	out_valid_rays = 0;

	for(j=0;j<hh;j++){
		for(i=0;i<ww;i++){
			pp_depth_map = p_depth_map[j][i].vp();
			if(pp_depth_map[0] == NOPOINTVALDoCube) continue;
			out_valid_rays ++;
		}
	}
}

//For Mesh to DoS
//******************************************************************************
void VCL_DoCube::sr_Set_Rays(
	float in_angle_x,
	float in_angle_y,
	float in_angle_z)
//******************************************************************************
{
	zz_mesh_to_DoS.sp_Set_Parameters(
		zz_width,
		zz_height,
		in_angle_x,
		in_angle_y,
		in_angle_z,
		&zz_homo);
}

//****************************************************************************//
//****************************************************************************//
//****************************************************************************//
//****************************************************************************//
//**********************Contents Generation from 3D Mesh**********************//
//****************************************************************************//
//****************************************************************************//
//****************************************************************************//
//****************************************************************************//
//******************************************************************************
void VCL_DoCube::lfm_Load_From_Mesh(
	CKvSet_of_Point3D *in_ori_p3d,
	CKvMatrixInt      *in_ori_mesh)
//******************************************************************************
{
	CKvSet2d_of_VectorFloat analog_map; //For mesh to DoCube
	zz_mesh_to_DoS.idm_Initialize_Depth_Map(zz_width,zz_height,&analog_map);
	zz_mesh_to_DoS.lfm_Load_From_Mesh(in_ori_p3d,in_ori_mesh,&analog_map);
	i_Import(&analog_map,&zz_homo);
}


//******************************************************************************
void VCL_DoCube::cmdc_Convert_Mesh_to_DoCube(
	CKvSet_of_Point3D *in_set_of_p3d_world,
	CKvHmatrix3D *in_homo_rotation_or_NULL,
	CKvHmatrix3D *in_homo_norm,
	CKvMatrixInt *in_mesh_of_depth_surface,
	float in_theta_z_axis_and_opt_axis_depth)
//******************************************************************************
{
	CKvSet2d_of_VectorFloat analog_map; //For mesh to DoCube
	zz_mesh_to_DoS.idm_Initialize_Depth_Map(zz_width,zz_height,&analog_map);
	zz_mesh_to_DoS.cimrs_Compute_Intersections_between_Mesh_and_Ray_Segments(
		in_set_of_p3d_world,
		in_mesh_of_depth_surface,
		in_homo_rotation_or_NULL,
		in_homo_norm,
		&analog_map,
		in_theta_z_axis_and_opt_axis_depth);
	i_Import(&analog_map, in_homo_norm);
}


//******************************************************************************
bool VCL_DoCube::cmdc_Convert_Mesh_to_DoCube(
	CKvSet_of_Point3D *in_set_of_p3d_world,
	CKvMatrixInt *in_mesh_of_depth_surface,
	int in_num_valid_p3ds,
	int in_num_valid_mesh_tris,
	CKvPmatrix3D *in_pmat,
	CKvHmatrix3D *in_homo_norm,
	int in_ray_direction,
	float in_shield_margin)
//******************************************************************************
{
	CKvSet2d_of_VectorFloat analog_map; //For mesh to DoCube
	zz_mesh_to_DoS.idm_Initialize_Depth_Map(zz_width,zz_height,&analog_map);
	if(!zz_mesh_to_DoS.cimrs_Compute_Intersections_between_Mesh_and_Ray_Segments(
		in_set_of_p3d_world,
		in_mesh_of_depth_surface,
		in_num_valid_p3ds,
		in_num_valid_mesh_tris,
		in_pmat,
		in_homo_norm,
		&analog_map,
		in_ray_direction,
		in_shield_margin)) return false;
	i_Import(&analog_map, in_homo_norm);

	return true;
}

//****************************************************************************//
//****************************************************************************//
//****************************************************************************//
//****************************************************************************//
//**********************Quantization from analog depth map********************//
//****************************************************************************//
//****************************************************************************//
//****************************************************************************//
//****************************************************************************//
//******************************************************************************
void VCL_DoCube::gidd_Get_Information_of_Depth_Data(
	CKvSet2d_of_VectorShort *in_depth_data,
	float in_maximum_magnitude_of_depth,
	float &out_quantization_step_size,
	short &out_max_depth_value,
	short &out_min_depth_value,
	int &out_number_of_layers,
	int &out_number_of_points,
	float &out_dynamic_range_of_input,
	CKvMatrixInt *out_start_point_index_map)
//******************************************************************************
{
	CKvVectorShort **p_depth_data;
	int **p_start_point_index_map;
	short *pp_depth_data;
	p_start_point_index_map = out_start_point_index_map->c_Create(zz_height,zz_width,0);

	int i,j;
	p_depth_data = in_depth_data->mp();

	int ndx=0;
	int sz;

	out_min_depth_value  = -1*(Kv2008_DO_SURFACE_INVALID_MARKER+1);
	out_max_depth_value  = Kv2008_DO_SURFACE_INVALID_MARKER;
	out_number_of_layers = out_number_of_points =  0;

	for(j=0;j<zz_height;j++){
		for(i=0;i<zz_width;i++){
			p_start_point_index_map[j][i] = ndx;

			pp_depth_data = p_depth_data[j][i].vps(sz);

			if(pp_depth_data[0] == Kv2008_DO_SURFACE_INVALID_MARKER)continue;
			
			if(pp_depth_data[0   ]<out_min_depth_value) out_min_depth_value = pp_depth_data[0];
			if(pp_depth_data[sz-1]>out_max_depth_value) out_max_depth_value = pp_depth_data[sz-1];
			if(sz> out_number_of_layers)                out_number_of_layers= sz;
			ndx +=sz;
		}
	}
	out_number_of_points	   = ndx;
	out_quantization_step_size = 2.0f*in_maximum_magnitude_of_depth/(float)zz_number_of_levels;  
	out_dynamic_range_of_input = in_maximum_magnitude_of_depth;
}

//******************************************************************************
void VCL_DoCube::i_Import(CKvSet2d_of_VectorShort *in_quantized_depth_map)
//******************************************************************************
{
	zz_depth_data.cp_Copy(in_quantized_depth_map);
	gidd_Get_Information_of_Depth_Data(
		&zz_depth_data,
		1.0f,
		zz_step_size,
		zz_val_max,
		zz_val_min,
		zz_number_of_layers,
		zz_number_of_points,
		zz_dynamic_range_of_input,
		&zz_point_index_map);
}

//******************************************************************************
void VCL_DoCube::i_Import(
	CKvSet2d_of_VectorFloat *in_analog_depth_map,
	CKvHmatrix3D *in_homo_or_NULL)
//******************************************************************************
{
	if(in_homo_or_NULL){zz_homo.cp_Copy(in_homo_or_NULL);}

	//if(!Kv_Printf("%d %d", in_analog_depth_map->mw(), in_analog_depth_map->mh()))	exit(0);

  	ian_Import_from_Analog_depth_map_NEW(
  		in_analog_depth_map,//CKvSet2d_of_VectorFloat *in_depth_map_in_increasing_order,
  		true,//true,//bool in_mode_for_testing_input_depth_map,
  		1.0f,//float in_maximum_magnitude_of_depth,
  		zz_number_of_levels,//short in_number_of_quantization_levels,
  		&zz_set_of_reference_and_direction_vectors,//CKvMatrixFloat *in_set_of_reference_and_direction_vectors,
  		&zz_homo);//CKvHmatrix3D *in_homography_for_transforming_object
	
	//if(!Kv_Printf("%d %d", zz_number_of_levels, zz_number_of_points))	exit(0);

 	u_cdam_Convert_Depth_Array_to_Map(
 		zz_number_of_points,//int in_number_of_points,
 		&zz_depth_array,//CKvVectorShort *in_depth_array,
 		&zz_point_index_map,//CKvMatrixInt *in_start_point_index_map,
 		&zz_depth_data);//CKvSet2d_of_VectorShort *out_depth_map

	//if(!Kv_Printf("%d %d", zz_number_of_levels, zz_number_of_points))	exit(0);
}

//**************************************************************************************
void VCL_DoCube::ian_Import_from_Analog_depth_map_NEW(
	CKvSet2d_of_VectorFloat *in_depth_map_in_increasing_order,
	bool in_mode_for_testing_input_depth_map,
	float in_maximum_magnitude_of_depth,
	short in_number_of_quantization_levels,
	CKvMatrixFloat *in_set_of_reference_and_direction_vectors,
	CKvHmatrix3D *in_homography_for_transforming_object)
//**************************************************************************************
{
	int i,nb_ray;

	in_set_of_reference_and_direction_vectors->mps(i,nb_ray);
	if(i!=6) go_error("1");
	zz_set_of_reference_and_direction_vectors.cp_Copy(
		in_set_of_reference_and_direction_vectors);

	in_depth_map_in_increasing_order->ms(zz_width,zz_height);

	if((zz_width*zz_height)!=nb_ray) go_error("2");

	zz_homo.cp_Copy(in_homography_for_transforming_object);
	zz_dynamic_range_of_input=in_maximum_magnitude_of_depth;
	zz_number_of_levels=in_number_of_quantization_levels;

 	if(!u_qdm_Quantize_Depth_Map_2d_NEW(
 		in_maximum_magnitude_of_depth,
 		in_number_of_quantization_levels,
 		in_depth_map_in_increasing_order,
 		in_mode_for_testing_input_depth_map,
 		&zz_depth_array, // CKvVectorShort *out_depth_index_array,
 		zz_width, // ww, // 	int &out_width_of_depth_map,
 		zz_height, // hh, // 	int &out_height_of_depth_map,
 		zz_step_size, // float &out_quantization_step_size,
 		zz_val_max, // zz_max_depth_value, // int &out_max_depth_value,
 		zz_val_min, // zz_min_depth_value, // int &out_min_depth_value,
 		zz_number_of_layers, /// int &out_number_of_layers,
 		zz_number_of_points, // int &out_number_of_points)
 		&zz_point_index_map) // CKvMatrixInt *out_start_point_index_map)
 		) go_error("3");

// 	if(!Kv_Printf("[%d] %d %d, %d\n%f %d %d\n%d %d",
// 	zz_width*zz_height, zz_width, zz_height, nb_ray,
// 	zz_step_size, zz_val_max, zz_val_min,
// 	zz_number_of_layers, zz_number_of_points))	exit(0);

	return;
error:
	zpme("ian_Import_from_Analog_depth_map_NEW");
}
//********************************************************************************
bool VCL_DoCube::u_qdm_Quantize_Depth_Map_2d_NEW(
	float in_maximum_of_depth_magnitude,
	short in_number_of_levels,
	CKvSet2d_of_VectorFloat *in_depth_map_in_increasing_order,
	bool in_mode_for_testing_input_depth_map,
	CKvVectorShort *out_depth_index_array,
	int &out_width_of_depth_map,
	int &out_height_of_depth_map,
	float &out_quantization_step_size,
	short &out_max_depth_value,
	short &out_min_depth_value,
	int &out_number_of_layers,
	int &out_number_of_points,
	CKvMatrixInt *out_start_point_index_map)
//********************************************************************************
{
	CKvVectorFloat *DOC; int ww,hh,sz,*I;

	DOC=in_depth_map_in_increasing_order->mps(ww,hh)[0]; sz=ww*hh;
	I=out_start_point_index_map->c_Create(hh,ww)[0];

	out_width_of_depth_map=ww;
	out_height_of_depth_map=hh;

	return zu_qdm_Quantize_Depth_Map_NEW(
		in_maximum_of_depth_magnitude,
		in_number_of_levels,
		sz,//int in_number_of_rays,
		DOC,
		in_mode_for_testing_input_depth_map,
		out_depth_index_array, // CKvVectorShort *out_depth_index_map,
		out_quantization_step_size,
		out_max_depth_value,
		out_min_depth_value,
		out_number_of_layers,
		out_number_of_points,
		I); // int *out_set_of_start_point_indices)
}
//********************************************************************************
bool VCL_DoCube::zu_qdm_Quantize_Depth_Map_NEW(
	float in_maximum_of_depth_magnitude,
	short in_number_of_levels,
	int in_number_of_rays,
	CKvVectorFloat *in_set_of_depth_jets,
	bool in_mode_for_testing_input_depth_map,
	CKvVectorShort *out_depth_index_array,
	float &out_quantization_step_size,
	short &out_max_depth_value,
	short &out_min_depth_value,
	int &out_number_of_layers,
	int &out_number_of_points,
	int *out_set_of_start_point_indices)
//********************************************************************************
{
	static CKvVectorShort tmp_DOC;
	CKvVectorFloat *DOC; 
	CKvVectorShort *DOC_INT; 
	float d_max,step;
	int nb_ray,nb,i,k,nb1,nb_point,ndx,buf_size;
	short *I,*T;

	nb_ray=in_number_of_rays; if(nb_ray<=0) go_error("000");
	DOC=in_set_of_depth_jets;
	DOC_INT=out_depth_index_array;

	d_max=in_maximum_of_depth_magnitude;
	nb=in_number_of_levels; nb1=nb-1;
	if((d_max<=0.0)||(nb<2)){go_error("1");}
	out_quantization_step_size=step=2.0f*d_max/(float)nb;

	//if(!Kv_Printf("[zu_igm_Is_Good_Map] 1"))	exit(0);

	if(!zu_igm_Is_Good_Map(
		nb_ray,DOC,
		in_maximum_of_depth_magnitude,
		buf_size)) go_error("2");

	//if(!Kv_Printf("[zu_igm_Is_Good_Map] 2 (%d)", buf_size))	exit(0);

	buf_size <<= 1; 
	I=DOC_INT->c_Create(buf_size);
	
	out_min_depth_value=nb-1;
	out_max_depth_value=0;	
	out_number_of_layers=out_number_of_points=0;

	ndx=0;
	for(i=0;i<nb_ray;i++) 
	{
		out_set_of_start_point_indices[i]=ndx;

		//if(!Kv_Printf("[zu_qdj_Quantize_Depth_Jet_NEW] 1"))	exit(0);

		if(!zu_qdj_Quantize_Depth_Jet_NEW(
			in_maximum_of_depth_magnitude,
			in_number_of_levels,
			&DOC[i], // CKvVectorFloat *in_depth_jet,
			in_mode_for_testing_input_depth_map,
			nb_point, // int &out_number_of_points,
			&tmp_DOC, // CKvVectorShort *out_depth_index_jet,
			out_quantization_step_size,
			out_max_depth_value,
			out_min_depth_value,
			out_number_of_layers,
			out_number_of_points)
			) gerr("3");
				
		//if(!Kv_Printf("[zu_qdj_Quantize_Depth_Jet_NEW] 2"))	exit(0);

		if(DOC[i].vs()<nb_point) gerr("4");
		if(buf_size<out_number_of_points) gerr("111");
		T=tmp_DOC.vp(); for(k=0;k<nb_point;k++){I[ndx++]=T[k];}
		if(ndx!=out_number_of_points)  gerr("111");
	}
	if(out_min_depth_value>out_max_depth_value) gerr("5");

	return true;
error:
	zsm("zu_qdm_Quantize_Depth_Map_NEW");
	return false;
}
//********************************************************************************
bool VCL_DoCube::zu_qdj_Quantize_Depth_Jet_NEW(
	float in_maximum_of_depth_magnitude,
	short in_number_of_levels,
	CKvVectorFloat *in_depth_jet,
	bool in_mode_for_testing_input_depth_map,
	int &out_number_of_points,
	CKvVectorShort *out_depth_index_jet,
	float &out_quantization_step_size,
	short &io_max_depth_value,
	short &io_min_depth_value,
	int &io_number_of_layers,
	int &io_total_number_of_points)
//********************************************************************************
{
	CKvVectorFloat *DOC; 
	CKvVectorShort *DOC_INT; 
	float *D; short *I; 
	float d_max,step,tmp;
	int nb,sz,nb_del,i,k,kk,nb1,ns; 

	d_max=in_maximum_of_depth_magnitude;
	nb=in_number_of_levels; nb1=nb-1;

	if((d_max<=0.0)||(nb<2))	go_error("1");

	out_quantization_step_size=step=2.0f*d_max/(float)nb;

	DOC=in_depth_jet;
	DOC_INT=out_depth_index_jet;
	
	if(!u_igj_Is_Good_Jet(DOC,ns))	go_error("2");

	if(io_max_depth_value>=nb) gerr("11");
	if(io_min_depth_value<0) gerr("12");
	if(io_number_of_layers<0) gerr("14");
	if(io_total_number_of_points<0) gerr("15");

	D=DOC->vps(sz); 
	I=DOC_INT->vps(i);
	if(i<sz)	I=DOC_INT->c_Create(sz); 

	//	if(u_inj_Is_NULL_Jet(DOC)) out_number_of_points=0;
	if(D[0]==Kv2008_DO_SURFACE_INVALID_MARKER) out_number_of_points=0;
	else
	{
		for(k=0;k<sz;k++) 
		{ 
			tmp=(D[k]+d_max)/step-0.5f;
			I[k]=clip_fnint(0,nb1,tmp); 
			//printf("%f %f %f %f %d\n",D[k],d_max,step,tmp,fnint(tmp));
			//printf("%d\n",I[k]);
		}
		
		for(nb_del=0,k=2,kk=2;k<sz;k+=2)
		{
			if((I[k]-I[k-1])>=2) 
			{
				I[kk++]=I[k];
				I[kk++]=I[k+1];
			}
			else
			{ 
				nb_del++; 
				I[kk-1]=I[k+1];
			}
		}
		out_number_of_points=sz-(2*nb_del); 

		if(out_number_of_points>0)
		{
			if(io_number_of_layers<out_number_of_points) io_number_of_layers=out_number_of_points;
			if(io_min_depth_value>I[0]) io_min_depth_value=I[0];
			if(io_max_depth_value<I[out_number_of_points-1]) io_max_depth_value=I[out_number_of_points-1];
			io_total_number_of_points += out_number_of_points;
			if(io_min_depth_value>io_max_depth_value) gerr("17");
		}
		if(out_number_of_points<0) gerr("18");
		//system("pause");
	}
//	if(out_number_of_points<=0)  out_depth_index_jet->c_Create(2,Kv2008_DO_SURFACE_INVALID_MARKER);
//	else D_tmp.g_Get(0,out_number_of_points,DOC_INT);

	return true;
error:
	zsm("zu_qdj_Quantize_Depth_Jet_NEW");
	return false;
}
