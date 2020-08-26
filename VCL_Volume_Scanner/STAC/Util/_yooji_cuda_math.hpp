//// *******************************************************************************
//bool GKvMatrixFloat1::import(
//	float *in_mat,
//	int in_ww,int in_hh,
//	int in_ch)
//// *******************************************************************************
//{
//	if(in_ww <= 0 || in_hh <= 0 || in_ch <= 0){
//		return false;
//	}
//
//	z_ww = in_ww;	z_hh = in_hh;  z_ch = in_ch;
//
//	if(z_ww != in_ww || z_hh != in_hh || z_ch != in_ch){
//		if(z_p_mat)	cudaFree(z_p_mat);
//		cudaMalloc((void**)&z_p_mat,sizeof(float)*(z_ww*z_hh*z_ch));
//	}
//
//	cudaMemcpy(z_p_mat,in_mat,sizeof(float)*(z_ww*z_hh*z_ch),cudaMemcpyHostToDevice);
//	//printf("%d %d %d\n", z_ww, z_hh, z_ch);
//
//	return true;
//}
//
//// *******************************************************************************
//float* GKvMatrixFloat1::copy(GKvMatrixFloat1 *a)
//// *******************************************************************************
//{
//	z_ww = a->z_ww;	z_hh = a->z_hh;	z_ch = a->z_ch;
//
//	if(z_p_mat){
//		cudaFree(z_p_mat);
//		cudaMalloc((void**)&z_p_mat,sizeof(float)*(z_ww*z_hh)*z_ch);
//	}
//
//	cudaMemcpy(z_p_mat,a->vp(),sizeof(float)*(z_ww*z_hh)*z_ch,cudaMemcpyDeviceToDevice);
//
//	return z_p_mat;
//}
//
//// *******************************************************************************
//float* GKvMatrixFloat1::create(
//	int in_hh,int in_ww,
//	int in_ch)
//// *******************************************************************************
//{
//	if(in_ww <= 0 || in_hh <= 0 || in_ch <= 0){
//		return NULL;
//	}
//
//	z_ww = in_ww;	z_hh = in_hh;  z_ch = in_ch;
//	if(z_p_mat)		cudaFree(z_p_mat);
//	cudaMalloc((void**)&z_p_mat,sizeof(float)*(z_ww*z_hh)*z_ch);
//
//	return z_p_mat;
//}

// ==============================================================================
// Matrix class for device memory.
// ==============================================================================
// *******************************************************************************
template<typename T>
bool GKvMatrix<T>::from_host(
	T *in_mat,
	int in_ww, int in_hh,
	int in_ch)
// *******************************************************************************
{
	if(in_ww <= 0 || in_hh <= 0 || in_ch <= 0){
		return false;
	}

	z_ww = in_ww;	z_hh = in_hh;  z_ch = in_ch;

	if(z_ww != in_ww || z_hh != in_hh || z_ch != in_ch){
		if(z_p_mat)	cudaFree(z_p_mat);
		cudaMalloc((void**)&z_p_mat, sizeof(T)*(z_ww*z_hh*z_ch));
	}

	cudaMemcpy(z_p_mat, in_mat, sizeof(T)*(z_ww*z_hh*z_ch), cudaMemcpyHostToDevice);
	//printf("%d %d %d\n", z_ww, z_hh, z_ch);

	return true;
}

// *******************************************************************************
template<typename T>
bool GKvMatrix<T>::to_host(
	T *out_mat,
	int &out_ww,int &out_hh,
	int &out_ch)
// *******************************************************************************
{
	if(!z_p_mat || z_ww <= 0 || z_hh <= 0)	return false;

	out_ww = z_ww;	out_hh = z_hh;	out_ch = z_ch;

	if(out_vol) delete[] out_vol;	out_vol = new T[z_ww*z_hh*z_ch];

	cudaMemcpy(out_mat,z_p_mat,sizeof(T)*(z_ww*z_hh*z_ch),cudaMemcpyDeviceToHost);

	return true;
}

//// *******************************************************************************
//template<typename T>
//bool GKvVolume<T>::export(
//	T *out_vol,
//	int &out_ww, int &out_hh, int &out_dd,
//	int &out_ch)
//// *******************************************************************************
//{
//	if(!z_p_vol)	return false;
//
//	out_ww = z_ww;	out_hh = z_hh;	out_dd = z_dd;	out_ch = z_ch;
//
//	if(out_vol)	delete out_vol;
//	out_vol = new T[z_ww*z_hh*z_dd*z_ch];
//
//	cudaMemcpy(out_vol, z_p_vol, sizeof(z_ww*z_hh*z_dd*z_ch), cudaMemcpyDeviceToHost);
//
//	return true;
//}

// *******************************************************************************
template<typename T>
T* GKvMatrix<T>::copy(GKvMatrix<T> *a)
// *******************************************************************************
{
	z_ww = a->z_ww;	z_hh = a->z_hh;	z_ch = a->z_ch;

	if(z_p_mat){
		cudaFree(z_p_mat);
		cudaMalloc((void**)&z_p_mat, sizeof(T)*(z_ww*z_hh)*z_ch);
	}

	cudaMemcpy(z_p_mat,	a->vp(), sizeof(T)*(z_ww*z_hh)*z_ch, cudaMemcpyDeviceToDevice);

	return z_p_mat;
}

// *******************************************************************************
template<typename T>
T* GKvMatrix<T>::create(
	int in_hh, int in_ww,
	int in_ch)
// *******************************************************************************
{
	if(in_ww <= 0 || in_hh <= 0 || in_ch <= 0){
		return NULL;
	}

	z_ww = in_ww;	z_hh = in_hh;  z_ch = in_ch;
	if(z_p_mat)		cudaFree(z_p_mat);
	cudaMalloc((void**)&z_p_mat, sizeof(T)*(z_ww*z_hh)*z_ch);
	cudaMemset(z_p_mat,0,sizeof(T)*(z_ww*z_hh)*z_ch);

	return z_p_mat;
}


// ==============================================================================
// Volume class for device memory.
// ==============================================================================
// *******************************************************************************
template<typename T>
bool GKvVolume<T>::from_host(
	T *in_vol,
	int in_ww, int in_hh, int in_dd,
	int in_ch)
// *******************************************************************************
{
	if(in_ww <= 0 || in_hh <= 0 || in_dd <= 0 || in_ch <= 0){
		return false;
	}

	z_ww = in_ww;	z_hh = in_hh;  z_dd = in_dd;	z_ch = in_ch;

	if(z_ww != in_ww || z_hh != in_hh || z_dd != in_dd || z_ch != in_ch){
		if(z_p_vol)	cudaFree(z_p_vol);
		cudaMalloc((void**)&z_p_vol, sizeof(T)*(z_ww*z_hh*z_dd*z_ch));
	}

	cudaMemcpy(z_p_vol, in_vol, sizeof(T)*(z_ww*z_hh*z_dd*z_ch), cudaMemcpyHostToDevice);


	return true;
}

// *******************************************************************************
template<typename T>
bool GKvVolume<T>::to_host(
	T *out_vol,
	int &out_ww, int &out_hh, int &out_dd,
	int &out_ch)
// *******************************************************************************
{
	if(!z_p_vol || z_ww <= 0 || z_hh <= 0 || z_dd <= 0)	return false;

	out_ww = z_ww;	out_hh = z_hh;	out_dd = z_dd;	out_ch = z_ch;

	if(out_vol) delete[] out_vol;	out_vol = new T[z_ww*z_hh*z_dd*z_ch];

	cudaMemcpy(out_vol, z_p_vol, sizeof(T)*(z_ww*z_hh*z_dd*z_ch), cudaMemcpyDeviceToHost);

	return true;
}

// *******************************************************************************
template<typename T>
T* GKvVolume<T>::copy(GKvVolume<T> *a)
// *******************************************************************************
{
	z_ww = a->z_ww;	z_hh = a->z_hh;	z_dd = a->z_dd;	z_ch = a->z_ch;

	if(z_p_vol){
		cudaFree(z_p_vol);
		cudaMalloc((void**)&z_p_vol, sizeof(T)*(z_ww*z_hh*z_dd)*z_ch);
	}

	cudaMemcpy(z_p_vol,
		a->vp(),
		sizeof(T)*(z_ww*z_hh*z_dd)*z_ch,
		cudaMemcpyDeviceToDevice);

	return z_p_vol;
}

// *******************************************************************************
template<typename T>
T* GKvVolume<T>::create(
	int in_ww, int in_hh, int in_dd,
	int in_ch,
	T in_value)
// *******************************************************************************
{
	int block_sz, grid_sz;
	if(in_ww <= 0 || in_hh <= 0 || in_dd <= 0 || in_ch <= 0){
		return NULL;
	}

	z_ww = in_ww;	z_hh = in_hh;  z_dd = in_dd;	z_ch = in_ch;
	if(z_p_vol)		cudaFree(z_p_vol);
	cudaMalloc((void**)&z_p_vol, sizeof(T)*(z_ww*z_hh*z_dd)*z_ch);
	cudaMemset(z_p_vol,0,sizeof(T)*(z_ww*z_hh*z_dd)*z_ch);

	return z_p_vol;
}