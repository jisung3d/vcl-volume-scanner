#include "../../stdafx.h"
// #include "VCL_Inhand_Object_Scanning.h"
// #include "VCL_Inhand_Object_ScanningDlg.h"
//#include "_Utility_Mesh_Generation.h"
#include "../_VCL_2014_3DO.h"

Util_Mesh_Generation::Util_Mesh_Generation(void)
{
}

Util_Mesh_Generation::~Util_Mesh_Generation(void)
{
}
//Make mesh from DoCube
//******************************************************************************
void Util_Mesh_Generation::mm_Make_Meshes(
	CKvDoCubeShort *in_do_cube,
	bool is_using_imported_homography,
	CKvDepot_of_Point3D *out_depot_point,
	CKvDepot_of_RgbaF *out_depot_color,
	CKvMesh_of_Triangle *out_mesh_triangle)
//******************************************************************************
{
	CKvSet_of_RgbaF set_color;
	int i, color_index[6];
	//DoCube import
	im_Import(in_do_cube);
	//Preparing point depot
	mdsp_Make_Depot_of_Surface_Points(in_do_cube,is_using_imported_homography,out_depot_point);
	//Preparing color depot
	for(i=0; i<6; i++) color_index[i] = 0;
	set_color.c_Create(out_depot_point->ne_Number_of_Elements());
	set_color.se_Set_Element   (0, &Kv_RgbaF(1.0, 0.0, 0.0, 0.0));
	out_depot_color->ap_Append(true,&set_color,i,i);
	//Mesh generation
	mm_Make_Meshes(
		3, //int in_point_size,
		0x0110f0f, //int in_segment_style_4bit_width_4bit_factor_16bit_pattern,
		color_index, //int *in_array_of_color_indices_in_6_values_1_point_3_segment_2_triangle,
		NULL, //CKvMesh_of_Point *out_mesh_of_points_or_NULL,
		NULL, // CKvMesh_of_Segment *out_mesh_of_segments_or_NULL,
		out_mesh_triangle); //CKvMesh_of_Triangle *out_mesh_of_triangles_or_NULL)
}

//******************************************************************************
void Util_Mesh_Generation::mm_Make_Meshes(
	CKvDoCubeShort *in_do_cube,
	CKvSet2d_of_VectorUchar *in_color_volume,
	CKvDepot_of_Point3D *out_depot_point,
	CKvDepot_of_RgbaF *out_depot_color,
	CKvMesh_of_Triangle *out_mesh_triangle)
//******************************************************************************
{
	//printf("	[mm_Make_Meshes]\n");
	CKvSet2d_of_VectorInt set_indices;
	CKvSet_of_RgbaF set_color;
	CKvVectorUchar *p_color_vol;
	int i, ww, hh, dd, color_index[6];
	unsigned char *p_color_vec;

	p_color_vol=in_color_volume->vp();
	ww=in_color_volume->mw();
	hh=in_color_volume->mh();
	dd=p_color_vol[0].vs();

	//DoCube import
	im_Import(in_do_cube);
	//Preparing point depot
	CKvSet_of_Point3D set_point;
	mdsp_Make_Depot_of_Surface_Points(in_do_cube,false,/*true,*/out_depot_point);

	//Preparing color depot.
	for(i=0; i<6; i++) color_index[i] = 0;
	set_color.c_Create(out_depot_point->ne_Number_of_Elements());
	//set_color.se_Set_Element(0, &Kv_RgbaF(1.0, 0.0, 0.0, 0.0));
	//out_depot_color->ap_Append(true,&set_color,i,i);

	for(i=0; i<out_depot_point->ne_Number_of_Elements(); i++){
		CKvRgbaF trgb;
		float *tp;
		int x, y, z;
		out_depot_point->gep_Get_Element_Pointer(i, tp);
		x=(int)((tp[0]+1.0f)*128.0f);		y=(int)((tp[1]+1.0f)*128.0f);		z=(int)((tp[2]+1.0f)*128.0f);
		
		p_color_vec=p_color_vol[y*ww+x].vp();
		trgb.r=((float)p_color_vec[3*z])/255.0f;
		trgb.g=((float)p_color_vec[3*z+1])/255.0f;
		trgb.b=((float)p_color_vec[3*z+2])/255.0f;
		trgb.a=1.0f;

		// set default color to blue.
		if(trgb.r==0.0f && trgb.g==0.0f && trgb.b==0.0f){
			trgb.r=0.0f;		trgb.g=0.0f;		trgb.b=1.0f;
		}

		set_color.se_Set_Element(i, &trgb);

		//printf("%d %d %d (%f %f %f)\n", x, y, z, tp[0], tp[1], tp[2]);		
// 		printf("%f %f %f (%d %d %d)\n", trgb.r, trgb.g, trgb.b, p_color_vec[3*z], p_color_vec[3*z+1], p_color_vec[3*z+2]);		
// 		system("pause");
	}
	out_depot_color->ap_Append(true,&set_color,i,i);





	//Mesh generation
	mm_Make_Meshes(
		3, //int in_point_size,
		0x0110f0f, //int in_segment_style_4bit_width_4bit_factor_16bit_pattern,
		NULL, //CKvMesh_of_Point *out_mesh_of_points_or_NULL,
		NULL, // CKvMesh_of_Segment *out_mesh_of_segments_or_NULL,
		out_mesh_triangle); //CKvMesh_of_Triangle *out_mesh_of_triangles_or_NULL)
}

//******************************************************************************
void Util_Mesh_Generation::mm_Make_Meshes(
	CKvDoCubeShort *in_do_cube,
	CKvDepot_of_Point3D *out_depot_point,
	CKvDepot_of_RgbaF *out_depot_color,
	CKvMesh_of_Triangle *out_mesh_triangle,
	CKvRgbaF *in_face_color)
//******************************************************************************
{
	//printf("	[mm_Make_Meshes]\n");
	CKvSet_of_RgbaF set_color;
	int i, color_index[6];
	//DoCube import
	im_Import(in_do_cube);
	//Preparing point depot
	mdsp_Make_Depot_of_Surface_Points(in_do_cube,false,/*true,*/out_depot_point);
	//mdsp_Make_Depot_of_Surface_Points(in_do_cube,true,out_depot_point);

	//Preparing color depot
	for(i=0; i<6; i++) color_index[i] = 0;
	set_color.c_Create(out_depot_point->ne_Number_of_Elements()); 
	if(in_face_color)	set_color.se_Set_Element(0, in_face_color);
	else				set_color.se_Set_Element(0, &Kv_RgbaF(1.0, 0.0, 0.0, 0.0));
	out_depot_color->ap_Append(true,&set_color,i,i);

	//Mesh generation
	mm_Make_Meshes(
		3, //int in_point_size,
		0x0110f0f, //int in_segment_style_4bit_width_4bit_factor_16bit_pattern,
		color_index, //int *in_array_of_color_indices_in_6_values_1_point_3_segment_2_triangle,
		NULL, //CKvMesh_of_Point *out_mesh_of_points_or_NULL,
		NULL, // CKvMesh_of_Segment *out_mesh_of_segments_or_NULL,
		out_mesh_triangle); //CKvMesh_of_Triangle *out_mesh_of_triangles_or_NULL)
}

//*******************************************************************************************
void Util_Mesh_Generation::gimwt_Get_Indices_of_Mesh_without_Tetragon(
	CKvMesh_of_Triangle *in_mesh_triangle,
	int in_0_pt_1_front_color_2_back_color,
	CKvVectorInt *out_idx_mesh)
//*******************************************************************************************
{
	CKvSet_of_VectorInt element_tri;
	CKvVectorInt *p_element_tri;
	int *pp_element_tri,total_vertices_tri,*p_idx_mesh,k;

	in_mesh_triangle->ge_Get_Element(0, &element_tri);
	p_element_tri   = element_tri  .vp();
	pp_element_tri  = p_element_tri  [in_0_pt_1_front_color_2_back_color].vps(total_vertices_tri);
	p_idx_mesh      = out_idx_mesh->c_Create(total_vertices_tri);

	//Merge index of mesh points...
	for(k=0;k<total_vertices_tri;k++)  {p_idx_mesh[k]					 = pp_element_tri[k];  }	
}

//*******************************************************************************************
void Util_Mesh_Generation::gimt_Get_Indices_of_Mesh_with_Tetragon(
	CKvMesh_of_Triangle *in_mesh_triangle,
	int in_0_pt_1_front_color_2_back_color,
	CKvVectorInt *out_idx_mesh)
//*******************************************************************************************
{
	CKvSet_of_VectorInt element_tri, element_tetra;
	CKvVectorInt *p_element_tri, *p_element_tetra;
	int *pp_element_tri, *pp_element_tetra,*p_idx_mesh;
	int num_vertices_total, num_vertices_tri, num_vertices_tetra, k;

	in_mesh_triangle->ge_Get_Element(0, &element_tri);	// triangles.
	in_mesh_triangle->ge_Get_Element(1, &element_tetra);	// tetragons.

	p_element_tri   = element_tri  .vp();
	pp_element_tri  = p_element_tri  [in_0_pt_1_front_color_2_back_color].vps(num_vertices_tri);

	p_element_tetra = element_tetra.vp();
	pp_element_tetra = p_element_tetra[in_0_pt_1_front_color_2_back_color].vps(num_vertices_tetra);

	num_vertices_total = num_vertices_tri + num_vertices_tetra;
	p_idx_mesh      = out_idx_mesh->c_Create(num_vertices_total);

	//Merge index of mesh points...
	for(k=0;k<num_vertices_tri;k++)		{p_idx_mesh[k]					 = pp_element_tri[k];  }	
	for(k=0;k<num_vertices_tetra;k++)   {p_idx_mesh[k+num_vertices_tri]	 = pp_element_tetra[k];  }	
}

//*******************************************************************************************
bool Util_Mesh_Generation::mm_Make_Meshes(
	int in_point_size,
	int in_segment_style_4bit_width_4bit_factor_16bit_pattern,
	int *in_array_of_color_indices_in_6_values_1_point_3_segment_2_triangle,
	CKvMesh_of_Point *out_mesh_of_points_or_NULL,
	CKvMesh_of_Segment *out_mesh_of_segments_or_NULL,
	CKvMesh_of_Triangle *out_mesh_of_triangles_or_NULL)
//*******************************************************************************************
{
	int np,nc,k,offset[3],*I,*c,dum,*FI,*PN,i,j,nc_per_plane,np_per_plane,start,last,ndx,nb_region;
	CKvVectorInt list;
	CKvSet_of_VectorInt element;
	CKvVectorInt *pi_set,*fi_set;

	c=in_array_of_color_indices_in_6_values_1_point_3_segment_2_triangle;
	np=zz_number_of_voxels;
	nc=zz_number_of_contours[0]+zz_number_of_contours[1]+zz_number_of_contours[2];

	for(k=0;k<3;k++) offset[k]=0;

	if(out_mesh_of_points_or_NULL)
	{
		out_mesh_of_points_or_NULL->in_Initialize();
		if(np>0)
		{
			I=list.c_Create(np);
			for(k=0;k<np;k++) I[k]=k;
			out_mesh_of_points_or_NULL->u_me_Make_Element(&list,c[0],in_point_size,&element);
			out_mesh_of_points_or_NULL->ap_Append(false,&element,offset,dum);
		}
	}

	if(out_mesh_of_segments_or_NULL)
	{
		pi_set=zz_voxel_index_sequences.vps(i); //if(i!=3) go_error("5");
		fi_set=zz_contour_start_voxel_index_sets.vps(i); //if(i!=3) go_error("6");

		out_mesh_of_segments_or_NULL->in_Initialize(nc+4);

		if(nc>0)
		{
			for(i=0;i<3;i++)
			{
				FI=fi_set[i].vps(nc_per_plane);
				PN=pi_set[i].vps(np_per_plane);

				nc_per_plane--;
				for(k=0;k<=nc_per_plane;k++)
				{
					start=FI[k];
					if(k<nc_per_plane) last=FI[k+1]; 
					else last=np_per_plane;

					if(last>start)
					{
						I=list.c_Create((last-start)<<1); last--;
						ndx=0; 
						for(j=start;j<=last;j++)
						{
							I[ndx++]=PN[j];
							if(j<last) I[ndx++]=PN[j+1];
							else I[ndx++]=PN[start];
						}
					}
					else if(last==start)
					{
						I=list.c_Create(2); 
						I[0]=I[1]=PN[start];

					}
					else Kv_Printf("[error:mm_Make_Meshes] 1");

					out_mesh_of_segments_or_NULL->u_me_Make_Element(
						&list,c[i+1],
						in_segment_style_4bit_width_4bit_factor_16bit_pattern,
						&element);
					out_mesh_of_segments_or_NULL->ap_Append(false,&element,offset,dum);
				}
			}
		}
	}
	if(out_mesh_of_triangles_or_NULL)
	{
		out_mesh_of_triangles_or_NULL->in_Initialize();

		ftr_Find_set_of_Triangles(nb_region,&list);
		if(nb_region>0)
		{
			out_mesh_of_triangles_or_NULL->u_me_Make_Element(&list,c[4],c[2],&element);
			out_mesh_of_triangles_or_NULL->ap_Append(false,&element,offset,dum);
		}

		ftt_Find_set_of_Tetragons(nb_region,&list);
		if(nb_region)
		{
			out_mesh_of_triangles_or_NULL->u_me_Make_Element(&list,c[4],c[5],&element);
			out_mesh_of_triangles_or_NULL->ap_Append(false,&element,offset,dum);
		}
	}

	return true;
}

	//*******************************************************************************************
bool Util_Mesh_Generation::mm_Make_Meshes(
	int in_point_size,
	int in_segment_style_4bit_width_4bit_factor_16bit_pattern,
	CKvMesh_of_Point *out_mesh_of_points_or_NULL,
	CKvMesh_of_Segment *out_mesh_of_segments_or_NULL,
	CKvMesh_of_Triangle *out_mesh_of_triangles_or_NULL)
//*******************************************************************************************
{
	int np,nc,k,offset[3],*I,*c,dum,*FI,*PN,i,j,nc_per_plane,np_per_plane,start,last,ndx,nb_region;
	CKvVectorInt list;
	CKvSet_of_VectorInt element;
	CKvVectorInt *pi_set,*fi_set;

	np=zz_number_of_voxels;
	nc=zz_number_of_contours[0]+zz_number_of_contours[1]+zz_number_of_contours[2];

	for(k=0;k<3;k++) offset[k]=0;

	if(out_mesh_of_points_or_NULL)
	{
		out_mesh_of_points_or_NULL->in_Initialize();
		if(np>0)
		{
			I=list.c_Create(np);
			for(k=0;k<np;k++) I[k]=k;
			out_mesh_of_points_or_NULL->u_me_Make_Element(&list, &list, &list,&element);
			out_mesh_of_points_or_NULL->ap_Append(false,&element,offset,dum);
		}
	}

	if(out_mesh_of_segments_or_NULL)
	{
		pi_set=zz_voxel_index_sequences.vps(i); //if(i!=3) go_error("5");
		fi_set=zz_contour_start_voxel_index_sets.vps(i); //if(i!=3) go_error("6");

		out_mesh_of_segments_or_NULL->in_Initialize(nc+4);

		if(nc>0)
		{
			for(i=0;i<3;i++)
			{
				FI=fi_set[i].vps(nc_per_plane);
				PN=pi_set[i].vps(np_per_plane);

				nc_per_plane--;
				for(k=0;k<=nc_per_plane;k++)
				{
					start=FI[k];
					if(k<nc_per_plane) last=FI[k+1]; 
					else last=np_per_plane;

					if(last>start)
					{
						I=list.c_Create((last-start)<<1); last--;
						ndx=0; 
						for(j=start;j<=last;j++)
						{
							I[ndx++]=PN[j];
							if(j<last) I[ndx++]=PN[j+1];
							else I[ndx++]=PN[start];
						}
					}
					else if(last==start)
					{
						I=list.c_Create(2); 
						I[0]=I[1]=PN[start];

					}
					else Kv_Printf("[error:mm_Make_Meshes] 1");

					out_mesh_of_segments_or_NULL->u_me_Make_Element(
						&list, &list, &list,
						&element);
					out_mesh_of_segments_or_NULL->ap_Append(false,&element,offset,dum);
				}
			}
		}
	}
	if(out_mesh_of_triangles_or_NULL)
	{
		out_mesh_of_triangles_or_NULL->in_Initialize();

		ftr_Find_set_of_Triangles(nb_region,&list);
		if(nb_region>0)
		{
			out_mesh_of_triangles_or_NULL->u_me_Make_Element(&list, &list, &list, &element);
			out_mesh_of_triangles_or_NULL->ap_Append(false,&element,offset,dum);
		}

		ftt_Find_set_of_Tetragons(nb_region,&list);
		if(nb_region)
		{
			out_mesh_of_triangles_or_NULL->u_me_Make_Element(&list, &list, &list, &element);
			out_mesh_of_triangles_or_NULL->ap_Append(false,&element,offset,dum);
		}
	}

	return true;
}
//*******************************************************************************************
bool Util_Mesh_Generation::ovte_Ordering_of_Verteces_on_Tetragon(
			int in_1st_point_index,
			int in_2nd_point_index,
			int in_3rd_point_index,
			int in_4th_point_index,
			bool &out_ordering)
//*******************************************************************************************
{
	int p[6], sz, i, k;
	bool validity;
	CKvVectorInt id_side;
	CKvVectorBool direction_seg;
	int *p_id_side;
	bool *p_direction_seg;

	int count=0;
	p[0]=in_1st_point_index;
	p[1]=in_2nd_point_index;
	p[2]=in_3rd_point_index;
	p[3]=in_4th_point_index;
	p[4]=in_1st_point_index;
	p[5]=in_2nd_point_index;

	validity=false;
	out_ordering=true;
	k=0;
	for(k=0;k<4;k++)
	{
		fss_Find_set_of_Slices_including_a_linked_Segment(
			p[k],				//int in_1st_point_index,
			p[k+1],				//int in_2nd_point_index,
			&id_side,			//CKvVectorInt *out_passings_0X_1Y_2Z,
			&direction_seg);	//CKvVectorBool *out_direction_of_segment)
		p_id_side=id_side.vps(sz);
		p_direction_seg=direction_seg.vp();

		for(i=0;i<sz;i++)
		{
			if(p_id_side[i]<3)
			{
				validity=ovtr_Ordering_of_Verteces_on_Triangle(
					p[k],
					p[k+1],
					p[k+2],
					p_id_side[i],
					p_direction_seg[i],
					out_ordering);
			}
			if(validity) break;
		}
		if(validity) break;
	}

	return validity;
}
//*******************************************************************************************
bool Util_Mesh_Generation::ovtr_Ordering_of_Verteces_on_Triangle(
			int in_1st_point_index,
			int in_2nd_point_index,
			int in_3rd_point_index,
			bool &out_ordering)
//*******************************************************************************************
{
	int p[5], sz, i, k;
	bool validity;
	CKvVectorInt id_side;
	CKvVectorBool direction_seg;
	int *p_id_side;
	bool *p_direction_seg;

	p[0]=in_1st_point_index;
	p[1]=in_2nd_point_index;
	p[2]=in_3rd_point_index;
	p[3]=in_1st_point_index;
	p[4]=in_2nd_point_index;

	validity=false;
	out_ordering=true;
	k=0;
	for(k=0;k<3;k++)
	{
		fss_Find_set_of_Slices_including_a_linked_Segment(
			p[k],				//int in_1st_point_index,
			p[k+1],				//int in_2nd_point_index,
			&id_side,			//CKvVectorInt *out_passings_0X_1Y_2Z,
			&direction_seg);	//CKvVectorBool *out_direction_of_segment)
		p_id_side=id_side.vps(sz);
		p_direction_seg=direction_seg.vp();

		for(i=0;i<sz;i++)
		{
			if(p_id_side[i]<3)
			{
				validity=ovtr_Ordering_of_Verteces_on_Triangle(
					p[k],
					p[k+1],
					p[k+2],
					p_id_side[i],
					p_direction_seg[i],
					out_ordering);
			}
			if(validity) break;
		}
		if(validity) break;
	}

	return validity;
}
//*******************************************************************************************
void Util_Mesh_Generation::fss_Find_set_of_Slices_including_a_linked_Segment(
			int in_1st_point_index,
			int in_2nd_point_index,
			CKvVectorInt *out_passings_0X_1Y_2Z,
			CKvVectorBool *out_direction_of_segment)
//out_direction_of_segment==true : p1-->p2
//out_passings_0X_2Y_3Z : 0-yz, 1-zx, 2-xy, 3-not valid
//*******************************************************************************************
{
	int **F,**G,k,start,i,j,nn[3],nn_total, *NN, count; 
	bool *A, pre_direction, flag, first;

	k=in_1st_point_index;
	F=zz_info_start_index_and_number_code_sets.mp();

	nn[0]=((F[1][k]>>16) & 0xff);
	nn[1]=((F[1][k]>>8) & 0xff);
	nn[2]=(F[1][k] & 0xff);
	nn_total=nn[0]+nn[1]+nn[2]; if(nn_total==0) Kv_Printf("[error:fss_Find_set_of_Slices_including_a_linked_Segment] 2");

	A=out_direction_of_segment->c_Create(nn_total, false);
	NN=out_passings_0X_1Y_2Z->c_Create(nn_total, 3);

	start=F[0][k];

	G=zz_prev_and_next_voxel_index_set.mp();

	for(j=start,i=0;i<nn_total;i++,j++)
	{
		count=0;
		if(in_2nd_point_index==G[j][0])
		{
			if(i<nn[0]) {NN[i]=0;}
			else if(i<(nn[0]+nn[1])) {NN[i]=1;}
			else {NN[i]=2;}
			A[i]=false;
			count++;
		}
		if(in_2nd_point_index==G[j][1])
		{
			if(i<nn[0]) {NN[i]=0;}
			else if(i<(nn[0]+nn[1])) {NN[i]=1;}
			else {NN[i]=2;}
			A[i]=true;
			count++;
		}
		if(count==2) NN[i]=3;
	}
	
	//error check
	// Passing X
	flag=false;
	first=true;
	for(i=0;i<nn[0];i++)
	{
		if(NN[i]==0)
		{
			if(first) {pre_direction=A[i]; first=false;}
			else
			{
				if(pre_direction!=A[i]) {flag=true; break;}				
			}
		}
	}
	if(flag){for(i=0;i<nn[0];i++){NN[i]=3;}}
	// Passing Y
	flag=false;
	first=true;
	for(i=nn[0];i<(nn[0]+nn[1]);i++)
	{
		if(NN[i]==1)
		{
			if(first) {pre_direction=A[i]; first=false;}
			else
			{
				if(pre_direction!=A[i]) {flag=true; break;}				
			}
		}
	}
	if(flag){for(i=nn[0];i<(nn[0]+nn[1]);i++){NN[i]=3;}}
	// Passing Z
	flag=false;
	first=true;
	for(i=(nn[0]+nn[1]);i<nn_total;i++)
	{
		if(NN[i]==2)
		{
			if(first) {pre_direction=A[i]; first=false;}
			else
			{
				if(pre_direction!=A[i]) {flag=true; break;}				
			}
		}
	}
	if(flag){for(i=(nn[0]+nn[1]);i<nn_total;i++){NN[i]=3;}}

	return;
}
//*******************************************************************************************
bool Util_Mesh_Generation::ovtr_Ordering_of_Verteces_on_Triangle(
			int in_1st_point_index,
			int in_2nd_point_index,
			int in_3rd_point_index,
			int in_slice_mode_0X_1Y_2Z,
			bool in_direction_of_segment_p1_to_p2,
			bool &out_ordering)
//*******************************************************************************************
{
	int i, np, three, a, b, c;
	int **p_voxel;
	int p1[3], p2[3], p3[3], p21[3], p31[3];
	int v[3], n[3], nv;
	bool validity;

	p_voxel=zz_table_of_voxels_3xNP.mps(np, three);
	for(i=0;i<three;i++)
	{
		p1[i]=p_voxel[i][in_1st_point_index];
		p2[i]=p_voxel[i][in_2nd_point_index];
		p3[i]=p_voxel[i][in_3rd_point_index];
		p21[i]=p2[i]-p1[i];
		p31[i]=p3[i]-p1[i];
	}
	switch(in_slice_mode_0X_1Y_2Z)
	{
	case 0: a=1; b=2; c=0; break;
	case 1: a=2; b=0; c=1; break;
	case 2: a=0; b=1; c=2; break;
	}
	if(p21[c]!=0) if(!Kv_Printf("[error:ovtr_Ordering_of_Verteces_on_Triangle]	in_slice_mode_0X_1Y_2Z=%d P1[c]=%d, %d, %d, p2[c]=%d, %d, %d"
		, in_slice_mode_0X_1Y_2Z, p1[0], p1[1], p1[2], p2[0], p2[1], p2[2])) exit(0);

	v[c]=p21[c];

	if(in_direction_of_segment_p1_to_p2)
	{
		v[a]=-p21[b];
		v[b]=p21[a];
	}
	else
	{
		v[a]=p21[b];
		v[b]=-p21[a];
	}
	
	n[0]=p21[1]*p31[2] - p21[2]*p31[1];
	n[1]=p21[2]*p31[0] - p21[0]*p31[2];
	n[2]=p21[0]*p31[1] - p21[1]*p31[0];

	nv=n[0]*v[0]+n[1]*v[1]+n[2]*v[2];
	if(nv<0)
	{
		validity=true;
		out_ordering=false;
	}
	else if(nv>0)
	{
		validity=true;
		out_ordering=true;
	}
	else
	{
		validity=false;
		out_ordering=false;
	}
	return validity;
}
//*******************************************************************************************
void Util_Mesh_Generation::ftr_Find_set_of_Triangles(
			int &out_number_of_triangles,
			CKvVectorInt *out_set_of_1st_2nd_and_3rd_point_indices)
//*******************************************************************************************
{
	static CKvVectorInt vec[2];
	int k,i,j,ndx,*a,*aa, *b,nb,np=zz_number_of_voxels; 
	int id_m[3];
	out_number_of_triangles=0;
	bool ordering;
	CKvVectorInt temp;

	for(k=0;k<np;k++)
	{
		z_ftrv_Find_set_of_Triangles_including_a_Vertex(
			true,
			k,
			nb,
			&vec[0],
			NULL,
			&vec[1]);

		out_number_of_triangles += nb;
	}

	if(out_number_of_triangles<=0) return;

	a=temp.c_Create(out_number_of_triangles*3);

	ndx=0;
	out_number_of_triangles=0;
	for(k=0;k<np;k++)
	{
		z_ftrv_Find_set_of_Triangles_including_a_Vertex(
			true,
			k,
			nb,
			&vec[0],
			NULL,
			&vec[1]);

		b=vec[0].vp();

		for(i=j=0;i<nb;i++)
		{
			id_m[0]=k;
			id_m[1]=b[j++];
			id_m[2]=b[j++];
			if(ovtr_Ordering_of_Verteces_on_Triangle(id_m[0], id_m[1], id_m[2], ordering))
			{
				if(ordering)
				{
					a[ndx++]=id_m[0];
					a[ndx++]=id_m[1];
					a[ndx++]=id_m[2];
				}
				else
				{
					a[ndx++]=id_m[0];
					a[ndx++]=id_m[2];
					a[ndx++]=id_m[1];
				}
				out_number_of_triangles++;
			}
		}
	}
	if(ndx!=(3*out_number_of_triangles)) Kv_Printf("[error:ftr_Find_set_of_Triangles] 88");
	aa=out_set_of_1st_2nd_and_3rd_point_indices->c_Create(out_number_of_triangles*3);
	for(i=0;i<ndx;i++) aa[i]=a[i];

	return;
}
//*******************************************************************************************
void Util_Mesh_Generation::ftt_Find_set_of_Tetragons(
			int &out_number_of_tetragons,
			CKvVectorInt *out_set_of_1st_2nd_and_3rd_point_indices)
//*******************************************************************************************
{
	static CKvVectorInt vec[2];
	int k,i,j,ndx,*a,*aa, *b,nb,np=zz_number_of_voxels; 
	int id_m[4];
	bool ordering;
	CKvVectorInt temp;

	out_number_of_tetragons=0;

	for(k=0;k<np;k++)
	{
		z_fttv_Find_set_of_Tetragons_including_a_Vertex(
			true,
			k,
			nb,
			&vec[0],
			NULL,
			&vec[1]);

		out_number_of_tetragons += nb;
	}

	if(out_number_of_tetragons<=0) return;

	a=temp.c_Create(out_number_of_tetragons*6);

	ndx=out_number_of_tetragons=0;
	for(k=0;k<np;k++)
	{
		z_fttv_Find_set_of_Tetragons_including_a_Vertex(
			true,
			k,
			nb,
			&vec[0],
			NULL,
			&vec[1]);

		b=vec[0].vp();

		for(i=j=0;i<nb;i++,j+=3)
		{
			id_m[0]=k;
			id_m[1]=b[j];
			id_m[2]=b[j+1];
			id_m[3]=b[j+2];

			if(ovte_Ordering_of_Verteces_on_Tetragon(id_m[0], id_m[1], id_m[2], id_m[3], ordering))
			{
				if(ordering)
				{
					a[ndx++]=id_m[0];
					a[ndx++]=id_m[1];
					a[ndx++]=id_m[2];

					a[ndx++]=id_m[2];
					a[ndx++]=id_m[3];
					a[ndx++]=id_m[0];
				}
				else
				{
					a[ndx++]=id_m[0];
					a[ndx++]=id_m[2];
					a[ndx++]=id_m[1];

					a[ndx++]=id_m[2];
					a[ndx++]=id_m[0];
					a[ndx++]=id_m[3];
				}
				out_number_of_tetragons++;
			}
		}
	}
	if(ndx!=(6*out_number_of_tetragons)) Kv_Printf("[error:ftt_Find_set_of_Tetragons] 88");
	aa=out_set_of_1st_2nd_and_3rd_point_indices->c_Create(out_number_of_tetragons*6);
	for(i=0;i<ndx;i++) aa[i]=a[i];

	return;
}

//////////////////////////////////////
//Marching Cube method////////////////
/////////////////////////////////////
//***********************************************************************************//
bool Util_Mesh_Generation::Marching_Cube_Docube(
	CKvDoCubeShort *in,
	int in_width,
	int in_height,
	int in_depth,
	unsigned char in_isovalue_0_to_255,
	CKvSet_of_Point3D *out_p3d,
	CKvMatrixInt *out_mash_3XN,
	float &out_time)
//***********************************************************************************//
{
	CKvSet2d_of_VectorShort *depth;
	CKvVectorUchar volume;
	depth=in->gp_Get_Pointer();
	
	Convert_from_CKvSet2d_of_VectorShort_to_CKvVectorUchar(
		depth,			//CKvSet2d_of_VectorShort *in,
		in_depth,
		&volume);		//CKvVectorUchar *out)

	Marching_Cube(
		&volume,		//CKvVectorUchar *in_voxel,
		in_width,
		in_height,
		in_depth,
		in_isovalue_0_to_255,
		out_p3d,
		out_mash_3XN,
		out_time);

	
	return true;		 
}

//***********************************************************************************//
bool Util_Mesh_Generation::Marching_Cube(
	CKvVectorUchar *in_voxel,
	int in_width,
	int in_height,
	int in_depth,
	unsigned char in_isovalue_0_to_255,
	CKvSet_of_Point3D *out_p3d,
	CKvMatrixInt *out_mash_3XN,
	float &out_time)
//***********************************************************************************//
{
	int i,j,k, num_tri_cell, num_total_triangle, l, n, L;
	CKvSet_of_Point3D cell_p3d, tri_p3d;
	CKvVectorUchar cell_value;
	CKvPoint3D *o_p3d, *p_tri_p3d;
	int **p_t;
	int MC_edgeTable[256]={
		0x0  , 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
		0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
		0x190, 0x99 , 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
		0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
		0x230, 0x339, 0x33 , 0x13a, 0x636, 0x73f, 0x435, 0x53c,
		0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
		0x3a0, 0x2a9, 0x1a3, 0xaa , 0x7a6, 0x6af, 0x5a5, 0x4ac,
		0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
		0x460, 0x569, 0x663, 0x76a, 0x66 , 0x16f, 0x265, 0x36c,
		0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
		0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff , 0x3f5, 0x2fc,
		0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
		0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55 , 0x15c,
		0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
		0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc ,
		0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
		0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
		0xcc , 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
		0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
		0x15c, 0x55 , 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
		0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
		0x2fc, 0x3f5, 0xff , 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
		0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
		0x36c, 0x265, 0x16f, 0x66 , 0x76a, 0x663, 0x569, 0x460,
		0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac,
		0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa , 0x1a3, 0x2a9, 0x3a0,
		0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
		0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33 , 0x339, 0x230,
		0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
		0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99 , 0x190,
		0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
		0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0   };
		int MC_triTable[256][16] =
		{{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1},
		{3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1},
		{3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
		{3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
		{9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1},
		{1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1},
		{9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
		{2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1},
		{8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1},
		{9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
		{4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1},
		{3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1},
		{1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1},
		{4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1},
		{4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
		{9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1},
		{1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
		{5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1},
		{2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1},
		{9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
		{0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
		{2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1},
		{10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1},
		{4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1},
		{5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1},
		{5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1},
		{9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1},
		{0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1},
		{1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1},
		{10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1},
		{8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1},
		{2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1},
		{7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1},
		{9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1},
		{2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1},
		{11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1},
		{9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1},
		{5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1},
		{11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1},
		{11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
		{1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1},
		{9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1},
		{5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1},
		{2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
		{0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
		{5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1},
		{6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1},
		{0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1},
		{3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1},
		{6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1},
		{5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1},
		{1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
		{10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1},
		{6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1},
		{1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1},
		{8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1},
		{7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1},
		{3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
		{5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1},
		{0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1},
		{9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1},
		{8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1},
		{5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1},
		{0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1},
		{6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1},
		{10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1},
		{10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1},
		{8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1},
		{1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1},
		{3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1},
		{0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1},
		{10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1},
		{0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1},
		{3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1},
		{6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1},
		{9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1},
		{8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1},
		{3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1},
		{6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1},
		{0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1},
		{10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1},
		{10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1},
		{1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1},
		{2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1},
		{7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1},
		{7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1},
		{2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1},
		{1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1},
		{11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1},
		{8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1},
		{0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1},
		{7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
		{10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
		{2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
		{6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1},
		{7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1},
		{2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1},
		{1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1},
		{10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1},
		{10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1},
		{0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1},
		{7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1},
		{6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1},
		{8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1},
		{9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1},
		{6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1},
		{1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1},
		{4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1},
		{10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1},
		{8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1},
		{0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1},
		{1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1},
		{8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1},
		{10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1},
		{4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1},
		{10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
		{5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
		{11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1},
		{9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
		{6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1},
		{7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1},
		{3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1},
		{7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1},
		{9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1},
		{3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1},
		{6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1},
		{9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1},
		{1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1},
		{4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1},
		{7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1},
		{6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1},
		{3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1},
		{0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1},
		{6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1},
		{1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1},
		{0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1},
		{11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1},
		{6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1},
		{5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1},
		{9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1},
		{1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1},
		{1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1},
		{10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1},
		{0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1},
		{5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1},
		{10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1},
		{11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1},
		{0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1},
		{9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1},
		{7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1},
		{2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1},
		{8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1},
		{9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1},
		{9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1},
		{1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1},
		{9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1},
		{9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1},
		{5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1},
		{0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1},
		{10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1},
		{2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1},
		{0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1},
		{0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1},
		{9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1},
		{5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1},
		{3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1},
		{5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1},
		{8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1},
		{0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1},
		{9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
		{0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1},
		{1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1},
		{3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1},
		{4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1},
		{9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1},
		{11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1},
		{11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1},
		{2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1},
		{9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1},
		{3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1},
		{1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1},
		{4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1},
		{4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1},
		{0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
		{3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1},
		{3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1},
		{0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1},
		{9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1},
		{1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}};

		CKvStopWatch stop;
		stop.c_Create(1);

		num_total_triangle=0;
		for(i=0; i<in_width-1; i++)
		{
			for(j=0; j<in_height-1; j++)
			{
				for(k=0; k<in_depth-1; k++)
				{
					Get_Cell_data(
						in_voxel,						//CKvVectorUchar *in_voxel,
						in_width, in_height, in_depth,	//int in_sz_x, int in_sz_y, int in_sz_z,
						i, j, k,						//int in_x, int in_y, int in_z,
						&cell_p3d,						//CKvSet_of_Point3D *out_grid_vertex_8,
						&cell_value);					//CKvVector *out_grid_value_8)
					num_tri_cell=Polygonise(
						&cell_p3d,						//CKvSet_of_Point3D *grid_vertex_8,
						&cell_value,					//CKvVector *grid_value_8,
						in_isovalue_0_to_255,				//double isolevel,
						MC_edgeTable,
						MC_triTable,
						&tri_p3d);						
					num_total_triangle+=num_tri_cell;
				}
			}
		}
		//Kv_Printf("num_tri=%d", num_total_triangle);
		o_p3d=out_p3d->c_Create(num_total_triangle*3);
		p_t=out_mash_3XN->c_Create(3, num_total_triangle);

		stop.r_Reset(0); 
		num_total_triangle=0;
		for(i=0; i<in_width-1; i++)
		{
			for(j=0; j<in_height-1; j++)
			{
				for(k=0; k<in_depth-1; k++)
				{
					Get_Cell_data(
						in_voxel,						//CKvVectorUchar *in_voxel,
						in_width, in_height, in_depth,	//int in_sz_x, int in_sz_y, int in_sz_z,
						i, j, k,						//int in_x, int in_y, int in_z,
						&cell_p3d,						//CKvSet_of_Point3D *out_grid_vertex_8,
						&cell_value);					//CKvVector *out_grid_value_8)

					num_tri_cell=Polygonise(
						&cell_p3d,						//CKvSet_of_Point3D *grid_vertex_8,
						&cell_value,					//CKvVector *grid_value_8,
						in_isovalue_0_to_255,				//double isolevel,
						MC_edgeTable,
						MC_triTable,
						&tri_p3d);						
										
					if(num_tri_cell!=0)
					{
						p_tri_p3d=tri_p3d.vp();

						for(l=0;l<num_tri_cell;l++)
						{
							n=3*num_total_triangle;
							L=3*l;
							o_p3d[n+L]=p_tri_p3d[L];
							o_p3d[n+L+1]=p_tri_p3d[L+1];
							o_p3d[n+L+2]=p_tri_p3d[L+2];
							p_t[0][num_total_triangle+l]=n+L;
							p_t[1][num_total_triangle+l]=n+L+1;
							p_t[2][num_total_triangle+l]=n+L+2;
						}
						num_total_triangle+=num_tri_cell;
					}
				}
			}
		}
		out_time=stop.get_Get_Elapsed_Time(0);
		return true;
}

//***********************************************************************************//
void Util_Mesh_Generation::Get_Cell_data(
	CKvVectorUchar *in_voxel,
	int in_sz_x, int in_sz_y, int in_sz_z,
	int in_x, int in_y, int in_z,
	CKvSet_of_Point3D *out_grid_vertex_8,
	CKvVectorUchar *out_grid_value_8)
	//***********************************************************************************//
{
	int	x, y, z, k, sz;
	CKvPoint3D *p_p3d;
	unsigned char *p_v;
	unsigned char *p_voxel;
	p_p3d=out_grid_vertex_8->c_Create(8);
	p_v=out_grid_value_8->c_Create(8);
	p_voxel=in_voxel->vps(sz);

	k=0; 
	x=in_x; y=in_y; z=in_z;
	p_p3d[k].x=x;	p_p3d[k].y=y;	p_p3d[k].z=z;
	p_v[k]=p_voxel[z*in_sz_y*in_sz_x+y*in_sz_x+x];
	//	if((z*in_sz_y*in_sz_x+y*in_sz_x+x)>=sz) Kv_Printf("size error");
	k++; 
	x=in_x+1; y=in_y; z=in_z;
	p_p3d[k].x=x;	p_p3d[k].y=y;	p_p3d[k].z=z;
	p_v[k]=p_voxel[z*in_sz_y*in_sz_x+y*in_sz_x+x];
	//	if((z*in_sz_y*in_sz_x+y*in_sz_x+x)>=sz) Kv_Printf("size error");
	k++; 
	x=in_x+1; y=in_y+1; z=in_z;
	p_p3d[k].x=x;	p_p3d[k].y=y;	p_p3d[k].z=z;
	p_v[k]=p_voxel[z*in_sz_y*in_sz_x+y*in_sz_x+x];
	//	if((z*in_sz_y*in_sz_x+y*in_sz_x+x)>=sz) Kv_Printf("size error");
	k++; x=in_x; y=in_y+1; z=in_z;
	p_p3d[k].x=x;	p_p3d[k].y=y;	p_p3d[k].z=z;
	p_v[k]=p_voxel[z*in_sz_y*in_sz_x+y*in_sz_x+x];
	//	if((z*in_sz_y*in_sz_x+y*in_sz_x+x)>=sz) Kv_Printf("size error");
	k++;
	x=in_x; y=in_y; z=in_z+1;
	p_p3d[k].x=x;	p_p3d[k].y=y;	p_p3d[k].z=z;
	p_v[k]=p_voxel[z*in_sz_y*in_sz_x+y*in_sz_x+x];
	//	if((z*in_sz_y*in_sz_x+y*in_sz_x+x)>=sz) Kv_Printf("size error");
	k++;
	x=in_x+1; y=in_y; z=in_z+1;
	p_p3d[k].x=x;	p_p3d[k].y=y;	p_p3d[k].z=z;
	p_v[k]=p_voxel[z*in_sz_y*in_sz_x+y*in_sz_x+x];
	//	if((z*in_sz_y*in_sz_x+y*in_sz_x+x)>=sz) Kv_Printf("size error");
	k++;
	x=in_x+1; y=in_y+1; z=in_z+1;
	p_p3d[k].x=x;	p_p3d[k].y=y;	p_p3d[k].z=z;
	p_v[k]=p_voxel[z*in_sz_y*in_sz_x+y*in_sz_x+x];
	//	if((z*in_sz_y*in_sz_x+y*in_sz_x+x)>=sz) Kv_Printf("size error");
	k++;
	x=in_x; y=in_y+1; z=in_z+1;
	p_p3d[k].x=x;	p_p3d[k].y=y;	p_p3d[k].z=z;
	p_v[k]=p_voxel[z*in_sz_y*in_sz_x+y*in_sz_x+x];
	//	if((z*in_sz_y*in_sz_x+y*in_sz_x+x)>=sz) Kv_Printf("size error");
	//if(p_v[0]!=0)
	//	if(!Kv_Printf("%lf, %lf, %lf, %lf\n, %lf, %lf, %lf, %lf",
	//	p_v[0],p_v[1],p_v[2],p_v[3],p_v[4],p_v[5],p_v[6],p_v[7])) exit(0);
	return;
}

//***********************************************************************************//
int Util_Mesh_Generation::Polygonise(
	CKvSet_of_Point3D *grid_vertex_8,
	CKvVectorUchar *grid_value_8,
	unsigned char isolevel,
	int *in_edgetable,
	int in_triTable[][16],
	CKvSet_of_Point3D *triangles_3xN)
//***********************************************************************************//
{
   int i,ntriang;
   int cubeindex;
   unsigned char  *p_grid_value;
   CKvPoint3D vertlist[12], *p_triangle, *p_grid_point;
   p_grid_point=grid_vertex_8->vp();
   p_grid_value=grid_value_8->vp();

   /*
      Determine the index into the edge table which
      tells us which vertices are inside of the surface
   */
   cubeindex = 0;
   if (p_grid_value[0] < isolevel) cubeindex |= 1;
   if (p_grid_value[1] < isolevel) cubeindex |= 2;
   if (p_grid_value[2] < isolevel) cubeindex |= 4;
   if (p_grid_value[3] < isolevel) cubeindex |= 8;
   if (p_grid_value[4] < isolevel) cubeindex |= 16;
   if (p_grid_value[5] < isolevel) cubeindex |= 32;
   if (p_grid_value[6] < isolevel) cubeindex |= 64;
   if (p_grid_value[7] < isolevel) cubeindex |= 128;

   //if(in_edgetable[cubeindex]!=0)
	  // if(!Kv_Printf("grid_value=%lf, %lf, %lf, %lf\n%lf, %lf, %lf, %lf", p_grid_value[0], p_grid_value[1], p_grid_value[2], p_grid_value[3], p_grid_value[4], p_grid_value[5], p_grid_value[6], p_grid_value[7])) exit(0);
   //if(cubeindex!=255) if(!Kv_Printf("%d/index=%d", in_edgetable[cubeindex], cubeindex)) exit(0);

/* Cube is entirely in/out of the surface */
   if (in_edgetable[cubeindex] == 0)
      return(0);

   /* Find the vertices where the surface intersects the cube */
   if (in_edgetable[cubeindex] & 1)
	   VertexInterp(isolevel,p_grid_point[0],p_grid_point[1],p_grid_value[0],p_grid_value[1], vertlist[0]);
   if (in_edgetable[cubeindex] & 2)
	   VertexInterp(isolevel,p_grid_point[1],p_grid_point[2],p_grid_value[1],p_grid_value[2], vertlist[1]);
   if (in_edgetable[cubeindex] & 4)
	   VertexInterp(isolevel,p_grid_point[2],p_grid_point[3],p_grid_value[2],p_grid_value[3], vertlist[2]);
   if (in_edgetable[cubeindex] & 8)
	   VertexInterp(isolevel,p_grid_point[3],p_grid_point[0],p_grid_value[3],p_grid_value[0], vertlist[3]);
   if (in_edgetable[cubeindex] & 16)
	   VertexInterp(isolevel,p_grid_point[4],p_grid_point[5],p_grid_value[4],p_grid_value[5], vertlist[4]);
   if (in_edgetable[cubeindex] & 32)
	   VertexInterp(isolevel,p_grid_point[5],p_grid_point[6],p_grid_value[5],p_grid_value[6], vertlist[5]);
   if (in_edgetable[cubeindex] & 64)
	   VertexInterp(isolevel,p_grid_point[6],p_grid_point[7],p_grid_value[6],p_grid_value[7], vertlist[6]);
   if (in_edgetable[cubeindex] & 128)
	   VertexInterp(isolevel,p_grid_point[7],p_grid_point[4],p_grid_value[7],p_grid_value[4], vertlist[7]);
   if (in_edgetable[cubeindex] & 256)
	   VertexInterp(isolevel,p_grid_point[0],p_grid_point[4],p_grid_value[0],p_grid_value[4], vertlist[8]);
   if (in_edgetable[cubeindex] & 512)
	   VertexInterp(isolevel,p_grid_point[1],p_grid_point[5],p_grid_value[1],p_grid_value[5], vertlist[9]);
   if (in_edgetable[cubeindex] & 1024)
	   VertexInterp(isolevel,p_grid_point[2],p_grid_point[6],p_grid_value[2],p_grid_value[6], vertlist[10]);
   if (in_edgetable[cubeindex] & 2048)
	   VertexInterp(isolevel,p_grid_point[3],p_grid_point[7],p_grid_value[3],p_grid_value[7], vertlist[11]);

   /* Create the triangle */
   ntriang = 0;   
   for (i=0;in_triTable[cubeindex][i]!=-1;i+=3) {
	//   Kv_Printf("i=%d, tri=%d", i, in_triTable[cubeindex][i]);
	   ntriang++;
   }

   p_triangle=triangles_3xN->c_Create(3*ntriang);

   ntriang = 0;   
   for (i=0;in_triTable[cubeindex][i]!=-1;i+=3) {
      p_triangle[3*ntriang	] = vertlist[in_triTable[cubeindex][i  ]];
      p_triangle[3*ntriang+1] = vertlist[in_triTable[cubeindex][i+1]];
      p_triangle[3*ntriang+2] = vertlist[in_triTable[cubeindex][i+2]];
      ntriang++;
   }
   return(ntriang);
}

//***********************************************************************************//
void Util_Mesh_Generation::VertexInterp(
	unsigned char isolevel,
	CKvPoint3D &p1,
	CKvPoint3D &p2,
	unsigned char valp1,
	unsigned char valp2,
	CKvPoint3D &out_p3d)
{
	double mu;
	CKvPoint3D p;

	if (abs(isolevel-valp1) < 0.00001)
		out_p3d=p1;
	if (abs(isolevel-valp2) < 0.00001)
		out_p3d=p2;
	if (abs(valp1-valp2) < 0.00001)
		out_p3d=p1;

	//	if(!Kv_Printf("ddddd=%lf", isolevel - valp1)) exit(0);
	mu = (isolevel - valp1) / (valp2 - valp1);
	out_p3d.x = p1.x + mu * (p2.x - p1.x);
	out_p3d.y = p1.y + mu * (p2.y - p1.y);
	out_p3d.z = p1.z + mu * (p2.z - p1.z);
	//	if(!Kv_Printf("isolevel=%lf, valp=%lf, %lf, mu=%lf, p1=%lf, %lf, %lf\np2=%lf, %lf, %lf\np=%lf, %lf, %lf", isolevel, valp1, valp2, mu, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, p.x, p.y, p.z)) exit(0);

	return;
}

//***********************************************************************************//
bool Util_Mesh_Generation::Convert_from_CKvSet2d_of_VectorShort_to_CKvVectorUchar(
	CKvSet2d_of_VectorShort *in,
	int in_depth,
	CKvVectorUchar *out)
//***********************************************************************************//
{
	int i,j,x,y,w,h,vs,max_vs;
	CKvVectorShort **p_in;
	short *p_p_in;
	unsigned char *p_out;

	p_in=in->mps_Matrix_Pointer_and_Width_Height(w,h);

	p_out=out->c_Create(in_depth*h*w,(unsigned char)0);

	for (x=0;x<w;x++){
		for (y=0;y<h;y++){
			p_p_in=p_in[y][x].vps_Vector_Pointer_and_Size(vs);
			if(p_p_in[0]!=(short)NOPOINTVALDoCube)
			
			{
				for (i=0;i<vs;i+=2)
				{
					for(j=p_p_in[i];j<=p_p_in[i+1];j++)
					{
						p_out[j*w*h+y*w+x]=255;
					}
				}
			}
		}
	}
	return true;
}