/////////////////////////////////////////
//Load from Mesh Ver.1.0, 2013.12.11/////
//Modified by Dreamerjoe 2013.12.12//////
/////////////////////////////////////////
class VCL_Mesh_to_DoS
{
public:

	VCL_Mesh_to_DoS();
	~VCL_Mesh_to_DoS();

	//Load from Mesh Object
	void sp_Set_Parameters(
		int in_width,
		int in_height,
		float in_angle_x,
		float in_angle_y,
		float in_angle_z,
		CKvHmatrix3D *out_homo);
	void idm_Initialize_Depth_Map(
		int in_width_of_depth_map,
		int in_height_of_depth_map,
		CKvSet2d_of_VectorFloat *out_analog_depth_map);	

	void cimrs_Compute_Intersections_between_Mesh_and_Ray_Segments(
		CKvSet_of_Point3D *in_set_of_p3d_world,
		CKvMatrixInt      *in_mesh_of_depth_surface,
		CKvHmatrix3D *in_homography_rotation_or_NULL,
		CKvHmatrix3D *in_homography_norm,
		CKvSet2d_of_VectorFloat *io_analog_depth_map,
		float in_theta_z_axis_and_opt_axis_depth);

	bool cimrs_Compute_Intersections_between_Mesh_and_Ray_Segments(
		CKvSet_of_Point3D *in_set_of_p3d_world,
		CKvMatrixInt      *in_mesh_of_depth_surface,
		int in_num_valid_p3ds,
		int in_num_valid_mesh_tris,
		CKvPmatrix3D *in_pmat,
		CKvHmatrix3D *in_homography_norm,
		CKvSet2d_of_VectorFloat *io_analog_depth_map,
		int in_ray_direction,
		float in_shield_margin);		// 0: x-axis | 1: y-axis | 2: z-axis.	

	bool z_cmdc_Convert_Mesh_to_DoCube(
		CKvSet_of_Point3D &in_set_of_p3d_norm,	//3D points set
		CKvMatrixInt &in_mesh_indices,			//mesh index (3xN)
		int in_dim_x,int in_dim_y,				// width and height of reference points. (dimension of XY-plane in DoCube).
		CKvSet2d_of_Vector &out_depth,			//Depth of the each ray(range=-1 to 1)
		CKvMatrixInt &out_num_of_layer);		//number of the layer

	//////////////////////////////////////////////////////////////////////////
	bool z_cmdc_Convert_Mesh_to_DoCube_for_X_axis(
		CKvSet_of_Point3D &in_set_of_p3d_norm,	//3D points set in normalized coordinates.
		CKvMatrixInt &in_mesh_indices,			//mesh index (3xN)
		int in_num_valid_p3ds,
		int in_num_valid_mesh_tris,
		int in_dim_x,int in_dim_y,				// width and height of reference points. (dimension of XY-plane in DoCube).
		bool in_ray_direction,
		CKvSet2d_of_Vector &out_depth);			//Depth of the each ray(range=-1 to 1)

	bool z_cmdc_Convert_Mesh_to_DoCube_for_Y_axis(
		CKvSet_of_Point3D &in_set_of_p3d_norm,	//3D points set in normalized coordinates.
		CKvMatrixInt &in_mesh_indices,			//mesh index (3xN)
		int in_num_valid_p3ds,
		int in_num_valid_mesh_tris,
		int in_dim_x,int in_dim_y,				// width and height of reference points. (dimension of XY-plane in DoCube).
		bool in_ray_direction,
		CKvSet2d_of_Vector &out_depth);			//Depth of the each ray(range=-1 to 1)

	bool z_cmdc_Convert_Mesh_to_DoCube_for_Z_axis(
		CKvSet_of_Point3D &in_set_of_p3d_norm,	//3D points set in normalized coordinates.
		CKvMatrixInt &in_mesh_indices,			//mesh index (3xN)
		int in_num_valid_p3ds,
		int in_num_valid_mesh_tris,
		int in_dim_x,int in_dim_y,				// width and height of reference points. (dimension of XY-plane in DoCube).
		bool in_ray_direction,
		CKvSet2d_of_Vector &out_depth);			//Depth of the each ray(range=-1 to 1)
	//////////////////////////////////////////////////////////////////////////

	void lfm_Load_From_Mesh(
		CKvSet_of_Point3D *in_ori_p3d,
		CKvMatrixInt      *in_ori_mesh,
		CKvSet2d_of_VectorFloat *io_analog_depth_map);

	void lfm_Load_From_Mesh(
		CKvSet_of_Point3D *in_ori_p3d,
		CKvMatrixInt      *in_ori_mesh,
		int				   in_width_of_depth_map,
		int                in_height_of_depth_map,
		CKvSet2d_of_VectorFloat *out_analog_depth_map,
		CKvHmatrix3D *out_homo);
	
	bool _Center_and_Radius_of_3d_points_with_Homography(
		CKvSet_of_Point3D *in_points,			//3D point set
		float *in_angle,						//Angle of the direction vector, 3 elements
		CKvSet_of_Point3D *out_Hpoints_or_NULL,	//3D point set with homography
		CKvMatrix *out_H);						//Homography (4x4, scale X Rotation)
	bool _Converting_to_DoS(
		int num_Dw,								//Number of the width ray
		int num_Dh,								//Number of the height ray
		CKvSet_of_Point3D *in_p3d,				//3D points set
		CKvMatrixInt *in_mesh,					//mesh index (3xN)
		CKvSet2d_of_Vector *out_starting_points,//Starting points at the DoS(x-y plane of the initial bound box: z=-1)
		CKvSet2d_of_Vector *out_depth,			//Depth of the each ray(range=-1 to 1)
		CKvMatrixInt *num_layer);
	bool _Is_inlier_of_the_mesh(
		double in_x,		// sample point : X value
		double in_y,		// sample point : Y value
		double *in_px,		// mesh point set : X set (size=3)
		double *in_py);		// mesh point set : Y set (size=3)
	bool _Intersection_ray_and_mesh(
		double in_x,		// sample point : X value
		double in_y,		// sample point : Y value (ray direction == z axis(0,0,1))
		double *in_px,		// mesh point set : X set (size=3)
		double *in_py,		// mesh point set : Y set (size=3)
		double *in_pz,		// mesh point set : Y set (size=3)
		double &out_z);		// Depth value
	bool _Intersection_ray_and_mesh_with_normal(
		double in_x,		// sample point : X value
		double in_y,		// sample point : Y value (ray direction == z axis(0,0,1))
		double *in_px,		// mesh point set : X set (size=3)
		double *in_py,		// mesh point set : Y set (size=3)
		double *in_pz,		// mesh point set : Y set (size=3)
		double &out_z,		// Depth value
		double *out_norm);	// Surface normal
	void _Cross_Product(
		double *in_data1,		
		double *in_data2,		
		double *out_data);		// (size=3)
	double _Min_Max(
		double *in_data,
		int in_data_size,
		double &out_min);
	//Add h.d.
	void _Voxelization_of_DoCubeShort(
		CKvDoCubeShort *in_docube,
		int in_quantization_level,
		CKvVolumeBool *out_voxel); 
	void _Volume_to_CKvSet_of_Point3D(
		CKvVolumeBool *in_volume,
		CKvSet_of_Point3D *out_point3D);

	CKvSet_of_Point3D zz_set_of_p3d_normalized;

	int zz_width;
	int zz_height;
	float zz_angle_x, zz_angle_y, zz_angle_z;
};