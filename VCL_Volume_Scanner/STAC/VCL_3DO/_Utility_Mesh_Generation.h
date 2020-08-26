#pragma once
#define  NOPOINTVALDoCube -32768

#include <_sdkim_2008_depth.h>

///////////////////////////////////////////////////
//Utility for Mesh generation Ver.1.0, 2013.12.11//
///////////////////////////////////////////////////
class Util_Mesh_Generation: public CKvVolume_by_Surface_Points
{
public:
	///////////////////////Modified by Dreamerjoe, 2013.12.12///////////////////
	void mm_Make_Meshes(CKvDoCubeShort *in_do_cube,CKvDepot_of_Point3D *out_depot_point,CKvDepot_of_RgbaF *out_depot_color,CKvMesh_of_Triangle *out_mesh_triangle, CKvRgbaF *in_face_color = NULL);
	void mm_Make_Meshes(CKvDoCubeShort *in_do_cube, CKvSet2d_of_VectorUchar *in_color_volume, CKvDepot_of_Point3D *out_depot_point,CKvDepot_of_RgbaF *out_depot_color,CKvMesh_of_Triangle *out_mesh_triangle);
	void mm_Make_Meshes(CKvDoCubeShort *in_do_cube,bool is_using_imported_homography,CKvDepot_of_Point3D *out_depot_point,CKvDepot_of_RgbaF *out_depot_color,CKvMesh_of_Triangle *out_mesh_triangle);
	void gimwt_Get_Indices_of_Mesh_without_Tetragon(CKvMesh_of_Triangle *in_mesh_triangle,int in_0_pt_1_front_color_2_back_color,CKvVectorInt *out_idx_mesh);
	void gimt_Get_Indices_of_Mesh_with_Tetragon(CKvMesh_of_Triangle *in_mesh_triangle,int in_0_pt_1_front_color_2_back_color,CKvVectorInt *out_idx_mesh);
	///////////////////////Modified by Dreamerjoe, 2013.12.12///////////////////


	bool mm_Make_Meshes(
		int in_point_size,
		int in_segment_style_4bit_width_4bit_factor_16bit_pattern,
		int *in_array_of_color_indices_in_6_values_1_point_3_segment_2_triangle,
		CKvMesh_of_Point *out_mesh_of_points_or_NULL,
		CKvMesh_of_Segment *out_mesh_of_segments_or_NULL,
		CKvMesh_of_Triangle *out_mesh_of_triangles_or_NULL);
	/// //////////////////////////////////////////////////////////////////////
	bool mm_Make_Meshes(
		int in_point_size,
		int in_segment_style_4bit_width_4bit_factor_16bit_pattern,		
		CKvMesh_of_Point *out_mesh_of_points_or_NULL,
		CKvMesh_of_Segment *out_mesh_of_segments_or_NULL,
		CKvMesh_of_Triangle *out_mesh_of_triangles_or_NULL);
	/// //////////////////////////////////////////////////////////////////////
	void ftt_Find_set_of_Tetragons(
		int &out_number_of_tetragons,
		CKvVectorInt *out_set_of_1st_2nd_and_3rd_point_indices);
	void ftr_Find_set_of_Triangles(
		int &out_number_of_triangles,
		CKvVectorInt *out_set_of_1st_2nd_and_3rd_point_indices);
	bool ovtr_Ordering_of_Verteces_on_Triangle(
		int in_1st_point_index,
		int in_2nd_point_index,
		int in_3rd_point_index,
		bool &out_ordering);
	bool ovte_Ordering_of_Verteces_on_Tetragon(
		int in_1st_point_index,
		int in_2nd_point_index,
		int in_3rd_point_index,
		int in_4th_point_index,
		bool &out_ordering);
	bool ovtr_Ordering_of_Verteces_on_Triangle(
		int in_1st_point_index,
		int in_2nd_point_index,
		int in_3rd_point_index,
		int in_slice_mode_0X_1Y_2Z,
		bool in_direction_of_segment_p1_to_p2,
		bool &out_ordering);
	void fss_Find_set_of_Slices_including_a_linked_Segment(
		int in_1st_point_index,
		int in_2nd_point_index,
		CKvVectorInt *out_passings_0X_1Y_2Z,
		CKvVectorBool *out_direction_of_segment);


	//////////////////////////////////////
	//Marching Cube method////////////////
	/////////////////////////////////////
	bool Marching_Cube_Docube(
		CKvDoCubeShort *in,
		int in_width,
		int in_height,
		int in_depth,
		unsigned char in_isovalue_0_to_255,
		CKvSet_of_Point3D *out_p3d,
		CKvMatrixInt *out_mash_3XN,
		float &out_time);

	bool Marching_Cube(
		CKvVectorUchar *in_voxel,
		int in_width,
		int in_height,
		int in_depth,
		unsigned char in_isovalue_0_to_255,
		CKvSet_of_Point3D *out_p3d,
		CKvMatrixInt *out_mash_3XN,
		float &out_time);
	int Polygonise(
		CKvSet_of_Point3D *grid_vertex_8,
		CKvVectorUchar *grid_value_8,
		unsigned char isolevel,
		int *in_edgetable,
		int in_triTable[][16],
		CKvSet_of_Point3D *triangles_3xN);
	void VertexInterp(
		unsigned char isolevel,
		CKvPoint3D &p1,
		CKvPoint3D &p2,
		unsigned char valp1,
		unsigned char valp2,
		CKvPoint3D &out_p3d);
	void Get_Cell_data(
		CKvVectorUchar *in_voxel,
		int in_sz_x, int in_sz_y, int in_sz_z,
		int in_x, int in_y, int in_z,
		CKvSet_of_Point3D *out_grid_vertex_8,
		CKvVectorUchar *out_grid_value_8);
	bool Convert_from_CKvSet2d_of_VectorShort_to_CKvVectorUchar(
		CKvSet2d_of_VectorShort *in,
		int in_depth,
		CKvVectorUchar *out);

	Util_Mesh_Generation(void);
	~Util_Mesh_Generation(void);
};
