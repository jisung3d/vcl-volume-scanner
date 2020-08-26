// object cube class for cuda.
#define GKvObjectCubeFloat GKvObjectCube<float>
#define GKvObjectCubeShort GKvObjectCube<short>

// ==============================================================================
// Object cube class for device memory.
// ==============================================================================
template<typename T>
class GKvObjectCube
{
public:

	// /////////////////////////////////////////////////
	// Host functions
	// /////////////////////////////////////////////////
	bool from_host(
		T *in_vol_tsdf,
		uchar *in_vol_w,
		uchar *in_vol_color,
		Vector3i in_dim, //int in_ww, int in_hh, int in_dd,
		Vector3f in_origin,
		Vector3f in_center,
		float in_sz_voxel);

	bool to_host(
		T *out_vol_tsdf,
		uchar *out_vol_w,
		uchar *out_vol_color,
		Vector3i &out_dim, //int in_ww, int in_hh, int in_dd,
		Vector3f &out_origin,
		Vector3f &out_center,
		float &out_sz_voxel);
	// 
	void copy(GKvObjectCube<T> *a);

	// create object cube.
	bool create(
		Vector3i in_dim, //int in_ww, int in_hh, int in_dd,
		Vector3f in_origin,
		Vector3f in_center,
		float in_sz_voxel,
		bool in_flag_color);
	void release(){
		z_vol_tsdf.release(); z_vol_w.release(); z_vol_sc.release(); z_vol_color.release();
		z_ww = z_hh = z_dd = 1;	z_flag_color = false;
	}

	// /////////////////////////////////////////////////
	// Pointers.
	// /////////////////////////////////////////////////	
	// volume data pointers.
	T* vp_tsdf(){ return z_vol_tsdf.vp(); }
	uchar* vp_w(){ return z_vol_w.vp(); }
	bool* vp_valid(){ return z_vol_sc.vp(); }
	uchar* vp_rgb(){ return z_vol_color.vp(); }	// [r---r|g---g|b---b]

	// volume offset.
	Vector3f origin(){ return z_origin; }
	Vector3f center(){ return z_center; }

	// volume dimension.	
	void ts(int &out_ww, int &out_hh, int &out_dd){ out_ww = z_ww;	out_hh = z_hh; out_dd = z_dd; }
	int tw(){ return z_ww; }
	int th(){ return z_hh; }
	int td(){ return z_dd; }
	int dim_sc(){ return z_dim_sc; }

	// voxel size.
	float sz_vox(){ return z_sz_vox; }

	// 
	~GKvObjectCube(){ release(); }
	GKvObjectCube(){
		z_origin = z_center = 0.0f;

		z_ww = z_hh = z_dd = 1;	z_flag_color = false;
		z_dim_sc = 8;
		z_sz_vox = 1.0f;

		z_vol_tsdf.create(1, 1, 1, 1, T(0));
		z_vol_w.create(1, 1, 1, 1, uchar(0));
		z_vol_sc.create(1, 1, 1, 1, false);
		z_vol_color.create(1, 1, 1, 1, uchar(0));
	}


private:
	Vector3f z_origin;		// left-top position of cube in global 3D coordinates.
	Vector3f z_center;		// center position of cube in global 3D coordinates.

	bool z_flag_color;
	int z_ww, z_hh, z_dd;	// cube size.
	int z_dim_sc;			// sub-cube size.
	float z_sz_vox;			// voxel size.

	GKvVolume<T> z_vol_tsdf;	// TSDF cube.
	GKvVolumeUchar z_vol_w;		// weight cube.
	GKvVolumeUchar z_vol_color;	// color cube.
	GKvVolumeBool z_vol_sc;		// Sub-cube flags. Each elements represents validity of 8x8 sub-cube.

};


#include "_yooji_cuda_object_cube.hpp"