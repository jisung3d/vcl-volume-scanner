// ==============================================================================
// Object cube class for device memory.
// ==============================================================================
// *******************************************************************************
template<typename T>
bool GKvObjectCube<T>::from_host(
	T *in_vol_tsdf,
	uchar *in_vol_w,
	uchar *in_vol_color,
	Vector3i in_dim, //int in_ww, int in_hh, int in_dd,
	Vector3f in_origin,
	Vector3f in_center,
	float in_sz_voxel)
// *******************************************************************************
{
	if(in_dim.x <= 0 || in_dim.y <= 0 || in_dim.z <= 0){
		return false;
	}

	int ww_sc, hh_sc, dd_sc;

	z_origin = in_origin;
	z_center = in_center;
	z_sz_vox = in_sz_voxel;

 	if(z_ww != in_dim.x || z_hh != in_dim.y || z_dd != in_dim.z){

		z_ww = in_dim.x;	z_hh = in_dim.y;  z_dd = in_dim.z;

		z_vol_tsdf.create(z_ww, z_hh, z_dd, 1, T(CV_CUDA_TSDF_INVALID));
		if(in_vol_w)        z_vol_w.create(z_ww, z_hh, z_dd, 1, uchar(0));
		if(in_vol_color)	z_vol_color.create(z_ww, z_hh, z_dd, 3, uchar(0));

		//ww_sc = iDivUp(z_ww, z_dim_sc); hh_sc = iDivUp(z_hh, z_dim_sc); dd_sc = iDivUp(z_dd, z_dim_sc);
		//z_vol_sc.create(ww_sc, hh_sc, dd_sc, 1, bool(0));
	}

	z_vol_tsdf.from_host(in_vol_tsdf, z_ww, z_hh, z_dd, 1);
	if(in_vol_w)		z_vol_w.from_host(in_vol_w, z_ww, z_hh, z_dd, 1);
	if(in_vol_color)	z_vol_color.from_host(in_vol_color, z_ww, z_hh, z_dd, 3);	

	return true;
}

// *******************************************************************************
template<typename T>
bool GKvObjectCube<T>::to_host(
	T *out_vol_tsdf,
	uchar *out_vol_w,
	uchar *out_vol_color,
	Vector3i &out_dim, //int in_ww, int in_hh, int in_dd,
	Vector3f &out_origin,
	Vector3f &out_center,
	float &out_sz_voxel)
// *******************************************************************************
{
	if(!out_vol_tsdf || z_ww <= 0 || z_hh <= 0 || z_dd <= 0){
		return false;
	}

	out_origin = z_origin;
	out_center = z_center;
	out_sz_voxel = z_sz_vox;

	out_dim.x = z_ww; out_dim.y = z_hh; out_dim.z = z_dd;

 	int len = z_ww*z_hh*z_dd;

	cudaMemcpy(out_vol_tsdf,z_vol_tsdf.vp(),sizeof(T)*len,cudaMemcpyDeviceToHost);
	if(z_vol_w.vp())
		cudaMemcpy(out_vol_w,z_vol_w.vp(),sizeof(uchar)*len,cudaMemcpyDeviceToHost);
	if(z_vol_color.vp())
		cudaMemcpy(out_vol_color,z_vol_color.vp(),sizeof(uchar)*3*len,cudaMemcpyDeviceToHost);

	// 요놈이 이상하게 복사해!!! ㅅㅂㄻ 내부에서 host array creation 하면 안되나????
	// 요놈이 이상하게 복사해!!! ㅅㅂㄻ 내부에서 host array creation 하면 안되나????
	// 요놈이 이상하게 복사해!!! ㅅㅂㄻ 내부에서 host array creation 하면 안되나????
	// 요놈이 이상하게 복사해!!! ㅅㅂㄻ 내부에서 host array creation 하면 안되나????
	//int ww, hh, dd, ch;
	//z_vol_tsdf.to_host(out_vol_tsdf,ww,hh,dd,ch);
	//if(z_vol_w.vp()) z_vol_w.to_host(out_vol_w,ww,hh,dd,ch);
	//if(z_vol_color.vp()) z_vol_color.to_host(out_vol_color,ww,hh,dd,ch);

	return true;
}

// *******************************************************************************
template<typename T>
void GKvObjectCube<T>::copy(GKvObjectCube<T> *a)
// *******************************************************************************
{
	z_origin = a->z_origin;
	z_center = a->z_center;

	z_flag_color = a->z_flag_color;
	z_ww = a->z_ww;	z_hh = a->z_hh;	z_dd = a->z_dd;
	z_dim_sc = a->z_dim_sc;
	z_sz_vox = a->z_sz_vox;

	z_vol_tsdf.copy(&a->z_vol_tsdf);
	z_vol_w.copy(&a->z_vol_w);
	z_vol_sc.copy(&a->z_vol_sc);
	if(z_flag_color)	z_vol_color.copy(&a->z_vol_color);
}

// *******************************************************************************
template<typename T>
bool GKvObjectCube<T>::create(
	Vector3i in_dim, //int in_ww, int in_hh, int in_dd,
	Vector3f in_origin,
	Vector3f in_center,
	float in_sz_voxel,
	bool in_flag_color)
// *******************************************************************************
{
	if(in_dim.x <= 0 || in_dim.y <= 0 || in_dim.z <= 0)	return false;

	z_ww = in_dim.x;	z_hh = in_dim.y;  z_dd = in_dim.z;
	z_flag_color = in_flag_color;

	// create tsdf volume.
	z_vol_tsdf.create(z_ww, z_hh, z_dd, 1, T(0));
	// create weight volume.
	z_vol_w.create(z_ww, z_hh, z_dd, 1, uchar(0));
	// create sub-cubes.
	z_vol_sc.create(iDivUp(z_ww, z_dim_sc), iDivUp(z_hh, z_dim_sc), iDivUp(z_dd, z_dim_sc), 1, false);
	// create color volume.
	if(in_flag_color)	z_vol_color.create(z_ww,z_hh,z_dd,3,uchar(0));

	// set origin and center.
	z_origin = in_origin;
	z_center = in_center;
	z_sz_vox = in_sz_voxel;

	return true;
}