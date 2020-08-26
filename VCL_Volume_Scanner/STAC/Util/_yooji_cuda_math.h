#define SQUARE(x) ((x)*(x))

// Matrix class for cuda.
#define GKvMatrixFloat GKvMatrix<float>
#define GKvMatrixInt GKvMatrix<int>
#define GKvMatrixShort GKvMatrix<short>
#define GKvMatrixUchar GKvMatrix<uchar>
#define GKvMatrixBool GKvMatrix<bool>

// Volume class for cuda.
#define GKvVolumeFloat GKvVolume<float>
#define GKvVolumeInt GKvVolume<int>
#define GKvVolumeShort GKvVolume<short>
#define GKvVolumeUchar GKvVolume<uchar>
#define GKvVolumeBool GKvVolume<bool>

// ==============================================================================
// Kernel template 화를 못하겠음.... 추후에 찾아보자.
__host__ void setDeviceMem1D(float* in_vec, int in_dim, float in_value);
__host__ void setDeviceMem1D(int* in_vec, int in_dim, int in_value);
__host__ void setDeviceMem1D(short* in_vec, int in_dim, short in_value);
__host__ void setDeviceMem1D(uchar* in_vec, int in_dim, uchar in_value);
__host__ void setDeviceMem1D(bool* in_vec, int in_dim, bool in_value);

__host__ void setDeviceMem2D(float* in_vec,int2 in_dim,float in_value);
__host__ void setDeviceMem2D(int* in_vec,int2 in_dim,int in_value);
__host__ void setDeviceMem2D(short* in_vec,int2 in_dim,short in_value);
__host__ void setDeviceMem2D(uchar* in_vec,int2 in_dim,uchar in_value);
__host__ void setDeviceMem2D(bool* in_vec,int2 in_dim,bool in_value);

__host__ void setDeviceMem3D(float* in_vec,int3 in_dim,float in_value);
__host__ void setDeviceMem3D(int* in_vec,int3 in_dim,int in_value);
__host__ void setDeviceMem3D(short* in_vec,int3 in_dim,short in_value);
__host__ void setDeviceMem3D(uchar* in_vec,int3 in_dim,uchar in_value);
__host__ void setDeviceMem3D(bool* in_vec,int3 in_dim,bool in_value);
// ==============================================================================
//
//class GKvMatrixFloat1
//{
//public:
//	// import volume data from host to device.
//	bool import(
//		float *in_mat,
//		int in_ww,int in_hh,
//		int in_ch);
//
//	// export volume data from device to host.
//	// 	bool export(
//	// 		T *out_vol,
//	// 		int &out_ww, int &out_hh, int &out_dd,
//	// 		int &out_ch);
//
//	// copy volume data class.
//	float* copy(GKvMatrixFloat1 *a);
//
//	// create and release volume data.
//	float* create(
//		int in_hh,int in_ww,
//		int in_ch);
//
//	void release(){ if(z_p_mat) cudaFree(z_p_mat);	z_ww = z_hh = z_ch = 1; }
//
//	// volume data pointer.
//	float* vp(){ return z_p_mat; }
//	// volume length.
//	int vs(){ return z_ww*z_hh; }
//
//	// Matrix dimension.
//	void ms(int &out_ww,int &out_hh){ out_ww = z_ww;	out_hh = z_hh; }
//	// Matrix width. (x)
//	int mw(){ return z_ww; }
//	// Matrix height. (y)
//	int mh(){ return z_hh; }
//	// Matrix channel.
//	int nch(){ return z_ch; }
//
//	~GKvMatrixFloat1(){ release(); }
//	GKvMatrixFloat1(){ z_ww = z_hh = z_ch = 1;	z_p_mat = NULL; }
//
//
//private:
//
//	int z_ww,z_hh,z_ch;
//	float *z_p_mat;
//
//};


// ==============================================================================
// Matrix class for device memory.
// ==============================================================================
template<typename T>
class GKvMatrix
{
public:
	// import volume data from host to device.
	bool from_host(
		T *in_mat,
		int in_ww, int in_hh,
		int in_ch);

	// export volume data from device to host.
	bool to_host(
		T *out_mat,
		int &out_ww,int &out_hh,
		int &out_ch);
	
	// copy volume data class.
	T* copy(GKvMatrix<T> *a);

	// create and release volume data.
	T* create(
		int in_hh, int in_ww,
		int in_ch);

	void release(){ if(z_p_mat) cudaFree(z_p_mat);	z_ww = z_hh = z_ch = 1; }

	// volume data pointer.
	T* vp(){ return z_p_mat; }
	// volume length.
	int vs(){ return z_ww*z_hh; }

	// Matrix dimension.
	void ms(int &out_ww, int &out_hh){ out_ww = z_ww;	out_hh = z_hh; }
	// Matrix width. (x)
	int mw(){ return z_ww; }
	// Matrix height. (y)
	int mh(){ return z_hh; }
	// Matrix channel.
	int nch(){ return z_ch; }

	~GKvMatrix(){ release(); }
	GKvMatrix(){ z_ww = z_hh = z_ch = 1;	z_p_mat = NULL; }


private:

	int z_ww, z_hh, z_ch;
	T *z_p_mat;

};

// ==============================================================================
// Volume class for device memory.
// ==============================================================================
template<typename T>
class GKvVolume
{
public:
	// import volume data from host to device.
	bool from_host(
		T *in_vol,
		int in_ww, int in_hh, int in_dd,
		int in_ch);

	//export volume data from device to host.
	bool to_host(
		T *out_vol,
		int &out_ww,int &out_hh,int &out_dd,
		int &out_ch);

	// copy volume data class.
	T* copy(GKvVolume<T> *a);

	// create and release volume data.
	T* create(
		int in_ww, int in_hh, int in_dd,
		int in_ch,
		T in_value);

	void release(){ if(z_p_vol) cudaFree(z_p_vol);	z_ww = z_hh = z_dd = z_ch = 1; }

	// volume data pointer.
	T* vp(){ return z_p_vol; }
	// volume length.
	int vs(){ return z_ww*z_hh*z_dd; }

	// volume dimension.
	void ts(int &out_ww, int &out_hh, int &out_dd){ out_ww = z_ww;	out_hh = z_hh; out_dd = z_dd; }
	// volume width. (x)
	int tw(){ return z_ww; }
	// volume height. (y)
	int th(){ return z_hh; }
	// volume depth. (z)
	int td(){ return z_dd; }
	// volume channel.
	int tc(){ return z_ch; }

	~GKvVolume(){ release(); }
	GKvVolume(){ z_ww = z_hh = z_dd = z_ch = 1;	z_p_vol = NULL; }


private:

	int z_ww, z_hh, z_dd, z_ch;
	T *z_p_vol;

};


///////////////////////////////////////////////////////////////////////////////
/// Basic device inline functions.
///////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
_CPU_AND_GPU_CODE_ inline void d_p_Project(Vector3f in_p3d, const float *K, Vector2f &out_p2d)
//********************************************************************************************
 {
 	out_p2d.x = K[0]*in_p3d.x/in_p3d.z+ K[2];
 	out_p2d.y = K[1]*in_p3d.y/in_p3d.z+ K[3];
 }
//********************************************************************************************
_CPU_AND_GPU_CODE_ inline void d_bp_Back_Project(Vector2f in_p2d, const float *K, float in_d, Vector3f &out_p3d)
//********************************************************************************************
{
	out_p3d.x = in_d*(in_p2d.x-K[2])/K[0];
	out_p3d.y = in_d*(in_p2d.y-K[3])/K[1];
	out_p3d.z = in_d;
}
//********************************************************************************************
_CPU_AND_GPU_CODE_ inline void d_bp_Back_Project(Vector2i in_pix, const float *K, float in_d, Vector3f &out_p3d)
//********************************************************************************************
{
	out_p3d.x = in_d*(in_pix.x-K[2])/K[0];
	out_p3d.y = in_d*(in_pix.y-K[3])/K[1];
	out_p3d.z = in_d;
}

//********************************************************************************************
_CPU_AND_GPU_CODE_ inline void d_t_Transform(Vector3f in_p3d, const float *T, Vector3f &out_p3d)
//********************************************************************************************
{
	out_p3d.x = T[0]*in_p3d.x	+T[1]*in_p3d.y	+T[2]*in_p3d.z	+T[3];
	out_p3d.y = T[4]*in_p3d.x	+T[5]*in_p3d.y	+T[6]*in_p3d.z	+T[7];
	out_p3d.z = T[8]*in_p3d.x	+T[9]*in_p3d.y	+T[10]*in_p3d.z	+T[11];
}

//********************************************************************************************
_CPU_AND_GPU_CODE_ inline void d_r_Rotate(Vector3f in_p3d, const float *T, Vector3f &out_p3d)
//********************************************************************************************
{
	out_p3d.x = T[0]*in_p3d.x	+T[1]*in_p3d.y	+T[2]*in_p3d.z;
	out_p3d.y = T[4]*in_p3d.x	+T[5]*in_p3d.y	+T[6]*in_p3d.z;
	out_p3d.z = T[8]*in_p3d.x	+T[9]*in_p3d.y	+T[10]*in_p3d.z;
}

//********************************************************************************************
_CPU_AND_GPU_CODE_ inline void d_gcc_Get_Camera_Center(const float *T, Vector3f &out_center)
//********************************************************************************************
{
	// pose = | R t |
	//		  | 0 1 |
	// camera center in global coordinates,
	// C=-R^-1*t from T=-RC.
	const float *p_mat= T;
	float tx, ty, tz;

	tx=p_mat[4*0+3];	ty=p_mat[4*1+3];	tz=p_mat[4*2+3];
	out_center.x=-p_mat[4*0+0]*tx -p_mat[4*1+0]*ty -p_mat[4*2+0]*tz;
	out_center.y=-p_mat[4*0+1]*tx -p_mat[4*1+1]*ty -p_mat[4*2+1]*tz;
	out_center.z=-p_mat[4*0+2]*tx -p_mat[4*1+2]*ty -p_mat[4*2+2]*tz;	

}

//********************************************************************************************
_CPU_AND_GPU_CODE_ inline bool d_gpv_Get_Position_in_Voxel(Vector3f in_p3d, 
	const float *origin, const int *dim_cube, const float sz_vox_inv,
	Vector3f &out_vox)
//********************************************************************************************
{
	bool valid = true;

	// assume that position of origin point in the world is (-0.5, -0.5, -0.5) in voxel coordinates..
	out_vox.x = (in_p3d.x - origin[0])*sz_vox_inv - 0.5f;
	out_vox.y = (in_p3d.y - origin[1])*sz_vox_inv - 0.5f;
	out_vox.z = (in_p3d.z - origin[2])*sz_vox_inv - 0.5f;

	if(out_vox.x < -0.5f || out_vox.x >= (float)dim_cube[0] - 0.5f ||
		out_vox.y < -0.5f || out_vox.y >= (float)dim_cube[1] - 0.5f ||
		out_vox.z < -0.5f || out_vox.z >= (float)dim_cube[2] - 0.5f)
		valid = false;

	return valid;
}
//********************************************************************************************
_CPU_AND_GPU_CODE_ inline bool d_gpw_Get_Position_in_World(Vector3f in_vox, 
	const float *origin, const int *dim_cube, const float sz_vox_inv,
	Vector3f &out_p3d)
//********************************************************************************************
{
	bool valid = true;

	if(in_vox.x < -0.5f || in_vox.x >= (float)dim_cube[0] - 0.5f ||
		in_vox.y < -0.5f || in_vox.y >= (float)dim_cube[1] - 0.5f ||
		in_vox.z < -0.5f || in_vox.z >= (float)dim_cube[2] - 0.5f)
		valid = false;

	out_p3d.x = (in_vox.x + 0.5f)/sz_vox_inv + origin[0];
	out_p3d.y = (in_vox.y + 0.5f)/sz_vox_inv + origin[1];
	out_p3d.z = (in_vox.z + 0.5f)/sz_vox_inv + origin[2];

	return valid;
}

//********************************************************************************************
__device__ inline bool d_gii_Get_Interpolated_Intensity(Vector2f in_p2d, int in_ww, int in_hh,
	const uchar *in_map_inten, float &out_inten)
//********************************************************************************************
{
	// d1   d2
	//    x
	// d3   d4
	float d1, d2, d3, d4;
	float xf, yf;
	int x, y;
	float resi_x, resi_y;

	xf = in_p2d.x; yf = in_p2d.y;
	if(xf<0.0f || xf>float(in_ww-1) ||
	   yf<0.0f || yf>float(in_hh-1))	return false;

	d2=d3=d4=0.0f;

	x=(int)xf;	y=(int)yf;	resi_x=xf-(float)x;	resi_y=yf-(float)y;
	d1=(float)in_map_inten[y*in_ww+x];			if(d1<=0.0f)	return false;
	if(resi_x>0.0f){					d2=(float)in_map_inten[y*in_ww+x+1];		if(d2<=0.0f)	return false;	}
	if(resi_y>0.0f){					d3=(float)in_map_inten[(y+1)*in_ww+x];		if(d3<=0.0f)	return false;	}
	if(resi_x>0.0f && resi_y>0.0f){		d4=(float)in_map_inten[(y+1)*in_ww+x+1];	if(d4<=0.0f)	return false;	}

	out_inten=(1.0f-resi_y)*( (1.0f-resi_x)*d1+resi_x*d2 ) + resi_y*( (1.0f-resi_x)*d3+resi_x*d4 );	

	return true;
}

//********************************************************************************************
__device__ inline bool d_gid_Get_Interpolated_Depth(Vector2f in_p2d, int in_ww, int in_hh,
	const float *in_map_depth, float &out_depth)
//********************************************************************************************
{
	// d1   d2
	//    x
	// d3   d4
	float d1, d2, d3, d4;
	float xf, yf;
	int x, y;
	float resi_x, resi_y;

	xf = in_p2d.x; yf = in_p2d.y;
	if(xf<0.0f || xf>float(in_ww-1) ||
	   yf<0.0f || yf>float(in_hh-1))	return false;

	d2=d3=d4=0.0f;

	x=(int)xf;	y=(int)yf;	resi_x=xf-(float)x;	resi_y=yf-(float)y;
	d1=in_map_depth[y*in_ww+x];			if(d1<=0.0f)	return false;
	if(resi_x>0.0f){					d2=in_map_depth[y*in_ww+x+1];		if(d2<=0.0f)	return false;	}
	if(resi_y>0.0f){					d3=in_map_depth[(y+1)*in_ww+x];		if(d3<=0.0f)	return false;	}
	if(resi_x>0.0f && resi_y>0.0f){		d4=in_map_depth[(y+1)*in_ww+x+1];	if(d4<=0.0f)	return false;	}

	out_depth=(1.0f-resi_y)*( (1.0f-resi_x)*d1+resi_x*d2 ) + resi_y*( (1.0f-resi_x)*d3+resi_x*d4 );	

	return true;
}

//********************************************************************************************
__device__ __forceinline__ bool d_giv_Get_Interpolated_Vertex(Vector2f in_p2d, int in_ww, int in_hh,
	const float *in_map_vertex, Vector3f &out_vertex)
//********************************************************************************************
{
	// n1   n2
	//    x
	// n3   n4	
	Vector3f v1, v2, v3, v4;
	const float *p_vertex = in_map_vertex;
	int x, y, tidx;	float xf, yf;
	float res_x, res_y, ires_x, ires_y;

	xf = in_p2d.x; yf = in_p2d.y;
	if(xf<0.0f || xf>float(in_ww-1) ||
	   yf<0.0f || yf>float(in_hh-1))	return false;

	x = (int)xf;	y = (int)yf;	res_x = xf-(float)x;	res_y = yf-(float)y;

	//zz_n2.s_Set(0.0f, 0.0f, 0.0f);	zz_n3.s_Set(0.0f, 0.0f, 0.0f);	zz_n4.s_Set(0.0f, 0.0f, 0.0f);

	// default value of invalid normal is (-100.0f, -100.0f, -100.0f). 
	// + n1.
	tidx = 3*(y*in_ww + x);	
	//if(isnan(v1.x=p_vertex[tidx++])) return false;
	if((v1.x=p_vertex[tidx++])==-100.0f)	return false; 
	//n1.x=p_norm[tidx++];
	v1.y=p_vertex[tidx++]; v1.z=p_vertex[tidx];
	// + n2.
	if(res_x>0.0f){	
		tidx = 3*(y*in_ww + x+1); 
		//if(isnan(v2.x=p_vertex[tidx++])) return false;
		if((v2.x=p_vertex[tidx++])==-100.0f)	return false; 
		//n2.x=p_norm[tidx++]; 
		v2.y=p_vertex[tidx++]; v2.z=p_vertex[tidx];
	}
	// + n3.
	if(res_y>0.0f){			
		tidx = 3*((y+1)*in_ww + x);
		//if(isnan(v3.x=p_vertex[tidx++])) return false;
		if((v3.x=p_vertex[tidx++])==-100.0f)	return false; 
		//n3.x = p_norm[tidx++]; 
		v3.y = p_vertex[tidx++]; v3.z = p_vertex[tidx];
	}
	// + n4.
	if(res_x>0.0f && res_y>0.0f){	
		tidx = 3*((y+1)*in_ww + x+1);
		//if(isnan(v4.x=p_vertex[tidx++])) return false;
		if((v4.x=p_vertex[tidx++])==-100.0f)	return false; 
		//n4.x = p_norm[tidx++]; 
		v4.y = p_vertex[tidx++]; v4.z = p_vertex[tidx];
	}

	ires_x = 1.0f - res_x;	ires_y = 1.0f - res_y;
	out_vertex = (ires_y*(ires_x*v1 + res_x*v2) + res_y*(ires_x*v3 + res_x*v4));

	return true;
}


//********************************************************************************************
__device__ __forceinline__ bool d_gin_Get_Interpolated_Normal(Vector2f in_p2d, int in_ww, int in_hh,
	const float *in_normal_map, Vector3f &out_norm)
//********************************************************************************************
{
	// n1   n2
	//    x
	// n3   n4	
	Vector3f n1, n2, n3, n4;
	const float *p_norm = in_normal_map;
	int x, y, tidx;	float xf, yf;
	float res_x, res_y, ires_x, ires_y;

	xf = in_p2d.x; yf = in_p2d.y;
	if(xf<0.0f || xf>float(in_ww-1) ||
	   yf<0.0f || yf>float(in_hh-1))	return false;

	x = (int)xf;	y = (int)yf;	res_x = xf-(float)x;	res_y = yf-(float)y;

	//zz_n2.s_Set(0.0f, 0.0f, 0.0f);	zz_n3.s_Set(0.0f, 0.0f, 0.0f);	zz_n4.s_Set(0.0f, 0.0f, 0.0f);

	// default value of invalid normal is (-100.0f, -100.0f, -100.0f). 
	// + n1.
	tidx = 3*(y*in_ww + x);	
	//if(isnan(n1.x=p_norm[tidx++])) return false;
	if((n1.x=p_norm[tidx++])==-100.0f)	return false; 
	//n1.x=p_norm[tidx++];
	n1.y=p_norm[tidx++]; n1.z=p_norm[tidx];
	// + n2.
	if(res_x>0.0f){	
		tidx = 3*(y*in_ww + x+1); 
		//if(isnan(n2.x=p_norm[tidx++])) return false;
		if((n2.x=p_norm[tidx++])==-100.0f)	return false; 
		//n2.x=p_norm[tidx++]; 
		n2.y=p_norm[tidx++]; n2.z=p_norm[tidx];
	}
	// + n3.
	if(res_y>0.0f){			
		tidx = 3*((y+1)*in_ww + x);
		//if(isnan(n3.x=p_norm[tidx++])) return false;
		if((n3.x=p_norm[tidx++])==-100.0f)	return false; 
		//n3.x = p_norm[tidx++]; 
		n3.y = p_norm[tidx++]; n3.z = p_norm[tidx];
	}
	// + n4.
	if(res_x>0.0f && res_y>0.0f){	
		tidx = 3*((y+1)*in_ww + x+1);
		//if(isnan(n4.x=p_norm[tidx++])) return false;
		if((n4.x=p_norm[tidx++])==-100.0f)	return false; 
		//n4.x = p_norm[tidx++]; 
		n4.y = p_norm[tidx++]; n4.z = p_norm[tidx];
	}

	ires_x = 1.0f - res_x;	ires_y = 1.0f - res_y;
	out_norm = (ires_y*(ires_x*n1 + res_x*n2) + res_y*(ires_x*n3 + res_x*n4)).normalised();

	return true;
}

//********************************************************************************************
// Every symmetric, positive definite matrix A can be decomposed into 
// a product of a unique lower triangular matrix L and its transpose
// Cholesky decomposition.
// Ljj = sqrt( Ajj - sum(Ljk^2)(k=0:j-1) )
// Lij = ( Aij - sum(Lik*Ljk)(k=0:j-1) )/Ljj
template<typename T>
_CPU_AND_GPU_CODE_ inline bool d_lld_LL_Decomposition(const T *A, int dim, T *L)
//********************************************************************************************
{
	T tsum;

	for(int i=0; i<dim*dim; i++) L[i] = 0.0f;
	
	for(int i=0; i<dim; i++){
		for(int j=0; j<dim; j++){

			// for elements above the diagonal.
			if(i<j) continue;
			// for diagonal elements.
			// Ljj = sqrt( Ajj - sum(Ljk^2)(k=0:j-1) )
			else if(i==j){			
				tsum = 0.0f;
				for(int k=0; k<j; k++) tsum += SQUARE(L[j*dim + k]);
				L[j*dim + j] = sqrtf(A[j*dim + j] - tsum);
			}
			// for elements below the diagonal.
			// Lij = ( Aij - sum(Lik*Ljk)(k=0:j-1) )/Ljj
			else if(abs(L[j*dim + j]) >= 1e-8f){
				tsum = 0.0f;
				for(int k=0; k<j; k++) tsum += L[i*dim + k]*L[j*dim + k];
				L[i*dim + j] = (A[i*dim + j] - tsum)/L[j*dim + j];
			} 
			else L[i*dim + j] = 0.0f;
// 			else{
// 				tsum = 0.0f;
// 				for(int k=0; k<j; k++) tsum += L[i*dim + k]*L[j*dim + k];
// 				L[i*dim + j] = (A[i*dim + j] - tsum)/L[j*dim + j];
// 			}
		}
	}

	// 	for(int i=0; i<dim; i++){
	// 
	// 		// compute diagonal term.
	// 		// Ljj = sqrt( Ajj - sum(Ljk^2)(k=0:j-1) )
	// 		tsum = 0.0f;
	// 		for(int k=0; k<i; k++) tsum += SQUARE(L[i*dim + k]);
	// 		L[i*dim + i] = sqrtf(A[i*dim + i] - tsum);
	// 
	// 		// check diagonal term.
	// 		if(abs(L[i*dim + i]) < 0.0f){
	// 			for(int j=i+1; j<dim; j++) L[i*dim + j] = 0.0f;
	// 		}
	// 		else{
	// 			for(int j=i+1; j<dim; j++){
	// 
	// 				// for elements above the diagonal.			
	// 				// Lij = ( Aij - sum(Lik*Ljk)(k=0:j-1) )/Ljj
	// 				tsum = 0.0f;
	// 				for(int k=0; k<j; k++) tsum += L[i*dim + k]*L[j*dim + k];
	// 				L[i*dim + j] = (A[i*dim + j] - tsum)/L[j*dim + j];
	// 
	// 				//////////////////////////////////////////////////////////////////////////
	// 			}
	// 		}
	// 
	// 		
	// 	}

	return true;
}

//********************************************************************************************
// Ax = b -> LL^Tx = b -> Ly = b -> compute y -> L^Tx = y -> compute x.
template<typename T>
_CPU_AND_GPU_CODE_ inline bool d_sls_Solve_Linear_System_using_LLD(const T *L, const T *b, 
	T *y, int dim, T *sol_x)
//********************************************************************************************
{
	float tsum;

	for(int i=0; i<dim; i++) 
		if(abs(L[i*dim + i]) < 1.0e-8f){
			printf("------------------- L: %f\n -----------------------\n",L[i*dim + i]); return false;
		}

	// compute y in ascending order.
	// y(i) = [b(i) - (L(i,1)*y(1) + ... + L(i,i-1)*y(i-1))]/L(i,i)
	for(int i=0; i<dim; i++){
		tsum = 0.0f;
		for(int k=0; k<i; k++) tsum += L[i*dim + k]*y[k];
		y[i] = (b[i] - tsum)/L[i*dim + i];
	}
	// compute x in descending order.
	// x(i) = [y(i) - (L(i+1,i)*x(i+1) + ... L(N,i)*x(N))]/L(i,i)	
	for(int i = dim-1; i>=0; i--){
		tsum = 0.0f;
		for(int k = i+1; k<dim; k++) tsum += L[k*dim + i]*sol_x[k];
		sol_x[i] = (y[i] - tsum)/L[i*dim + i];
	}
// 	// x(N-i) = [y(N-i) - (L(N-i+1,N-i)*x(N-i+1) + ... L(N,N-i)*x(N))]/L(N-i,N-i)
// 	for(int i=dim-1; i>=0; i--){
// 		tsum = 0.0f;
// 		for(int k=dim-1; k>i; k--) tsum += L[k*dim + i]*sol_x[k];
// 		sol_x[i] = (y[i] - tsum)/L[i*dim + i];
// 	}

	return true;
}

//********************************************************************************************
// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM
// Reference: http://crrl.poly.edu/522/rotation_matrix_representations.pdf
// Axis-angle representation:
inline bool d_gtp_Get_Transformation_from_Params(const float *in_params, float *out_mat)
//********************************************************************************************
{
	//float *p_mat = zz_mat.vp();
	float p_R[9], p_t[3];

	float one_6th = 1.0f/6.0f;
	float one_20th = 1.0f/20.0f;

	Vector3f w; w.x = in_params[0]; w.y = in_params[1]; w.z = in_params[2];
	Vector3f t; t.x = in_params[3]; t.y = in_params[4]; t.z = in_params[5];

	//float theta_sq = VectorDotProduct_3(&w, &w);
	float theta_sq = dot(w,w);
	float theta = sqrtf(theta_sq);

	float A,B;

	Vector3f crossV = cross(w,t);// , buffV3; VectorCrossProduct_3(&crossV, &w, &t, &buffV3);
	if(theta_sq < 1e-8f)
	{
		A = 1.0f - one_6th * theta_sq; B = 0.5f;
		p_t[0] = t.x + 0.5f * crossV.x; p_t[1] = t.y + 0.5f * crossV.y; p_t[2] = t.z + 0.5f * crossV.z;
	} else
	{
		float C;
		if(theta_sq < 1e-6f)
		{
			C = one_6th * (1.0f - one_20th * theta_sq);
			A = 1.0f - theta_sq * C;
			B = 0.5f - 0.25f * one_6th * theta_sq;
		} else
		{
			float inv_theta = 1.0f / theta;
			A = sinf(theta) * inv_theta;
			B = (1.0f - cosf(theta)) * (inv_theta * inv_theta);
			C = (1.0f - A) * (inv_theta * inv_theta);
		}

		Vector3f cross2 = cross(w,crossV);
		//VectorCrossProduct_3(&cross2, &w, &crossV, &buffV3);

		p_t[0] = t.x + B * crossV.x + C * cross2.x; p_t[1] = t.y + B * crossV.y + C * cross2.y; p_t[2] = t.z + B * crossV.z + C * cross2.z;
	}

	float wx2 = w.x * w.x,wy2 = w.y * w.y,wz2 = w.z * w.z;
	p_R[0*3 + 0] = 1.0f - B*(wy2 + wz2);
	p_R[1*3 + 1] = 1.0f - B*(wx2 + wz2);
	p_R[2*3 + 2] = 1.0f - B*(wx2 + wy2);

	float a,b;
	a = A * w.z,b = B * (w.x * w.y);
	p_R[0*3 + 1] = b - a;
	p_R[1*3 + 0] = b + a;

	a = A * w.y,b = B * (w.x * w.z);
	p_R[0*3 + 2] = b + a;
	p_R[2*3 + 0] = b - a;

	a = A * w.x,b = B * (w.y * w.z);
	p_R[1*3 + 2] = b - a;
	p_R[2*3 + 1] = b + a;

	// set output trasformation matrix.
	for(int i=0; i<3; i++) for(int j=0; j<3; j++) out_mat[i*4+j] = p_R[i*3+j];
	for(int i=0; i<3; i++) out_mat[i*4+3] = p_t[i];

	return true;
}

//********************************************************************************************
// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM
// Reference: http://crrl.poly.edu/522/rotation_matrix_representations.pdf
// Axis-angle representation:
inline bool d_gpt_Get_Params_from_Transformation(const float *in_mat, float *out_params)
//********************************************************************************************
{
	float R[9], t[3];

	// get rotation matrix.
	for(int j=0; j<3; j++)	for(int i=0; i<3; i++)	R[j*3+i]=in_mat[j*4+i];
	// get translation vector.
	for(int j=0; j<3; j++)	t[j]=in_mat[j*4+3];

	Vector3f resultRot;

	float cos_angle = (R[0*3 + 0]  + R[1*3 + 1] + R[2*3 + 2] - 1.0f) * 0.5f;
	resultRot.x = (R[2*3 + 1] - R[1*3 + 2]) * 0.5f;
	resultRot.y = (R[0*3 + 2] - R[2*3 + 0]) * 0.5f;
	resultRot.z = (R[1*3 + 0] - R[0*3 + 1]) * 0.5f;

// 	if(!Kv_Printf("%f %f %f %f %f %f\n",resultRot.x,resultRot.y,resultRot.z,
// 	t[0],t[1],t[2])) exit(0);

	float sin_angle_abs = sqrtf(dot(resultRot,resultRot));

	// Compute rotation axis.
	if(cos_angle > M_SQRT1_2)
	{
		if(sin_angle_abs)
		{
			float p = asinf(sin_angle_abs) / sin_angle_abs;
			resultRot.x *= p; resultRot.y *= p; resultRot.z *= p;
		}
	} else
	{
		if(cos_angle > -M_SQRT1_2)
		{
			float p = acosf(cos_angle) / sin_angle_abs;
			resultRot.x *= p; resultRot.y *= p; resultRot.z *= p;
		} else
		{
			float angle = (float)M_PI - asinf(sin_angle_abs);
			float d0 = R[0*3 + 0] - cos_angle;
			float d1 = R[1*3 + 1] - cos_angle;
			float d2 = R[2*3 + 2] - cos_angle;

			Vector3f r2;

			if(fabsf(d0) > fabsf(d1) && fabsf(d0) > fabsf(d2))
			{
				r2.x = d0; r2.y = (R[1*3 + 0] + R[0*3 + 1]) * 0.5f; r2.z = (R[0*3 + 2] + R[2*3 + 0]) * 0.5f;
			} else
			{
				if(fabsf(d1) > fabsf(d2))
				{
					r2.x = (R[1*3 + 0] + R[0*3 + 1]) * 0.5f; r2.y = d1; r2.z = (R[2*3 + 1] + R[1*3 + 2]) * 0.5f;
				} else { r2.x = (R[0*3 + 2] + R[2*3 + 0]) * 0.5f; r2.y = (R[2*3 + 1] + R[1*3 + 2]) * 0.5f; r2.z = d2; }
			}

			//if (VectorDotProduct_3(&r2, &resultRot) < 0.0f)
			if(dot(r2,resultRot) < 0.0f) { r2.x *= -1.0f; r2.y *= -1.0f; r2.z *= -1.0f; }

			//VectorNormalize_3(&r2, &r2);
			r2 = normalize(r2);

			resultRot.x = angle * r2.x; resultRot.y = angle * r2.y; resultRot.z = angle * r2.z;
		}
	}

	float shtot = 0.5f;
	//float theta = sqrtf(VectorDotProduct_3(&resultRot, &resultRot));
	float theta = sqrtf(dot(resultRot,resultRot));

	if(theta > 0.00001f) shtot = sinf(theta * 0.5f) / theta;

	float halfrotorparams[6];
	float rott[3], matT[16];
	halfrotorparams[0] = resultRot.x * -0.5f; halfrotorparams[1] = resultRot.y * -0.5f; halfrotorparams[2] = resultRot.z * -0.5f;
	halfrotorparams[3] = 0.0f; halfrotorparams[4] = 0.0f; halfrotorparams[5] = 0.0f;
	
	d_gtp_Get_Transformation_from_Params(halfrotorparams, matT);
	for(int j=0; j<3; j++)	for(int i=0; i<3; i++)	R[j*3+i]=matT[j*4+i];	
	d_mmsv_Multiply_Matrix_Square_Vector(R, t, 3, rott);
	//Vector3f rottrans, buffV3;
	//MatrixVectorMultiply_3(&rottrans, &halfrotor.R, &this->T, &buffV3);
	
	Vector3f rottrans, T;// = halfrotor.R * T;
	rottrans.x = rott[0]; rottrans.y = rott[1]; rottrans.z = rott[2];
	T.x = t[0]; T.y = t[1]; T.z = t[2];

// 	if(!Kv_Printf("%f %f %f %f %f %f\n", rottrans.x, rottrans.y, rottrans.z, 
// 		T.x, T.y, T.z)) exit(0);

	if(theta > 0.001f)
	{
		//float denom = VectorDotProduct_3(&resultRot, &resultRot);
		//float param = VectorDotProduct_3(&this->T, &resultRot) * (1 - 2 * shtot) / denom;
		float denom = dot(resultRot,resultRot);
		float param = dot(T,resultRot) * (1 - 2 * shtot) / denom;

		rottrans.x -= resultRot.x * param; rottrans.y -= resultRot.y * param; rottrans.z -= resultRot.z * param;
	} else
	{
		//float param = VectorDotProduct_3(&this->T, &resultRot) / 24;
		float param = dot(T,resultRot) / 24;
		rottrans.x -= resultRot.x * param; rottrans.y -= resultRot.y * param; rottrans.z -= resultRot.z * param;
	}

	rottrans.x /= 2 * shtot; rottrans.y /= 2 * shtot; rottrans.z /= 2 * shtot;

	out_params[0] = resultRot.x; out_params[1] = resultRot.y; out_params[2] = resultRot.z;
	out_params[3] = rottrans.x; out_params[4] = rottrans.y; out_params[5] = rottrans.z;


	return true;
}
//
///////
//
////********************************************************************************************
//_CPU_AND_GPU_CODE_ inline bool d_gtvu_Get_TSDF_Value_Uninterpolated(
//	Vector3f in_vox, 
//	const float *vol_tsdf,
//	const uchar *vol_w,
//	const int *dim_cube_dev,
//	float &out_tsdf)
////********************************************************************************************
//{
//	int ww, hh, dd;
//	int x, y, z, tidx;
//
//	out_tsdf = 1.0f;	// set default value.
//
//	// calculate local indices.		
//	x = ROUNDF(in_vox.x);	y = ROUNDF(in_vox.y);	z = ROUNDF(in_vox.z);
//	//x = int(in_vox.x);	y = int(in_vox.y);	z = int(in_vox.z);
//
//	if(x<0 || x>=dim_cube_dev[0] || 
//	   y<0 || y>=dim_cube_dev[1] || 
//	   z<0 || z>=dim_cube_dev[2]) 
//	   return false; 
//	
// 	tidx = z*dim_cube_dev[0]*dim_cube_dev[1] + y*dim_cube_dev[0] + x;
//	 
// 	if(vol_w[tidx] <= uchar(0)) return false;
// 
// 	out_tsdf = vol_tsdf[tidx];
//
//	return true;
//	
//}
//
////********************************************************************************************
//_CPU_AND_GPU_CODE_ inline bool d_gtvi_Get_TSDF_Value_Interpolated(	
//	Vector3f in_vox,
//	const float *vol_tsdf,
//	const uchar *vol_w,
//	const int *dim_cube_dev,
//	float &out_tsdf)
////********************************************************************************************
//{
//	Vector3i offset;
//	Vector3f residu;
//	Vector3f tpos;
//
//	float inter_f, inter_b;
//	float front[4], back[4];
//	float dx1[4], dy1[4];
//	int i;
//
//	//     5-----6
//	//  1=====2
//	//     7-----8
//	//  3=====4
//	/// Get TSDF values of 8 neighbor voxels.
//	out_tsdf = 1.0f;
//	// get offset voxel index and float residual vector of input 3d point.
//	offset = in_vox.toInt(residu);
//
//	// set delta x, y for slice access.
//	dx1[0] = 0.0f; dx1[1] = 1.0f; dx1[2] = 0.0f; dx1[3] = 1.0f;
//	dy1[0] = 0.0f; dy1[1] = 0.0f; dy1[2] = 1.0f; dy1[3] = 1.0f;
//
//	// get TSDF values of neighbors on the front plane.
//	tpos.z = offset.z;	// set Z value.
//	for(i = 0; i<4; i++){
//		tpos.x = offset.x + dx1[i]; tpos.y = offset.y + dy1[i];
//		if(!d_gtvu_Get_TSDF_Value_Uninterpolated(tpos, vol_tsdf, vol_w, dim_cube_dev, front[i])) return false;
//	}
//
//	//// get TSDF values of neighbors on the back plane.
//	tpos.z = offset.z + 1.0f;	// set Z value.
//	for(i = 0; i<4; i++){
//		tpos.x = offset.x + dx1[i]; tpos.y = offset.y + dy1[i];
//		if(!d_gtvu_Get_TSDF_Value_Uninterpolated(tpos, vol_tsdf, vol_w, dim_cube_dev, back[i])) return false;
//	}
//
//	
//	// interpolates TSDF value on XY plane.
//	inter_f = (1.0f - residu.y)*((1.0f - residu.x)*front[0] + residu.x*front[1])
//		+ residu.y*((1.0f - residu.x)*front[2] + residu.x*front[3]);
//	inter_b = (1.0f - residu.y)*((1.0f - residu.x)*back[0] + residu.x*back[1])
//		+ residu.y*((1.0f - residu.x)*back[2] + residu.x*back[3]);
// 
// 	out_tsdf = (1.0f - residu.z)*inter_f + residu.z*inter_b;
//
//	return true;
//}
//
//
////********************************************************************************************
//_CPU_AND_GPU_CODE_ inline bool d_csnt_Compute_Surface_Normal_from_TSDF(
//	Vector3f in_p3d, 
//	const float *vol_tsdf,
//	const uchar *vol_w,
//	const float *origin_dev, 
//	const int *dim_cube_dev, 
//	const int sz_vox_inv_dev,
//	Vector3f &out_surf_norm)
////********************************************************************************************
//{
//	// we need total 32 neighbors for computing a single surface normal of input 3D point. (refer following pictures.)
//	// the offset voxel index is 4 in front XY plane.
//	// + the foremost XY slice (Z=-1)
//	//    -- X      
//	// Y |
//	//       1     2      
//	//          X
//	//       3     4     
//	//
//	//            
//	//float XY_foremost[4];
//	// + front XY slice (Z=0)
//	//       1     2
//	//
//	// 3     4     5     6
//	//          X
//	// 7     8     9     10
//	//
//	//       11    12
//	//float XY_front[12];
//	// + back XY slice (Z=1)
//	//       1     2
//	//
//	// 3     4     5     6
//	//          X
//	// 7     8     9     10
//	//
//	//       11    12
//	//float XY_back[12];
//	// + the backmost XY slice (Z=2)
//	//       
//	//
//	//       1     2      
//	//          X
//	//       3     4     
//	//
//	//            
//	//float XY_backmost[4];
//
//	/// Get TSDF values of 32 neighbor voxels.
// 	Vector3i offset;
// 	Vector3f residu;
// 	Vector3f tpos, vox, norm;
// 	
// 	float sz_voxel, gx, gy, gz;
// 	float foremost[4], front[12], back[12], backmost[4];
// 	float dx1[4], dy1[4], dx2[12], dy2[12];
//
//	// coverts input 3D point to position of voxel coordinates.
// 	if(!d_gpv_Get_Position_in_Voxel(in_p3d, origin_dev, dim_cube_dev, sz_vox_inv_dev, vox)) return false;
// 
// 	// get offset voxel index and float residual vector of input 3d point.
// 	offset = vox.toInt(residu);
// 
// 	// set delta x, y for slice access.
// 	dx1[0] = 0.0f; dx1[1] = 1.0f; dx1[2] = 0.0f; dx1[3] = 1.0f;
// 	dy1[0] = 0.0f; dy1[1] = 0.0f; dy1[2] = 1.0f; dy1[3] = 1.0f;
// 
// 	dx2[0] = 0.0f;   dx2[1] = 1.0f;
// 	dx2[2] = -1.0f;  dx2[3] = 0.0f; dx2[4] = 1.0f; dx2[5] = 2.0f;
// 	dx2[6] = -1.0f;  dx2[7] = 0.0f; dx2[8] = 1.0f; dx2[9] = 2.0f;
// 	dx2[10] = 0.0f;  dx2[11] = 1.0f;
// 
// 	dy2[0] = -1.0f;  dy2[1] = -1.0f;
// 	dy2[2] = 0.0f;  dy2[3] = 0.0f;  dy2[4] = 0.0f; dy2[5] = 0.0f;
// 	dy2[6] = 1.0f;  dy2[7] = 1.0f;  dy2[8] = 1.0f; dy2[9] = 1.0f;
// 	dy2[10] = 2.0f; dy2[11] = 2.0f;
//
//	// get TSDF values of neighbors on the foremost XY plane. (Z = -1)
//	tpos.z = offset.z - 1.0f;
//	for(int i=0; i<4; i++){
//		tpos.x = offset.x + dx1[i]; tpos.y = offset.y + dy1[i];
//		if(!d_gtvu_Get_TSDF_Value_Uninterpolated(tpos, vol_tsdf, vol_w, dim_cube_dev, foremost[i])) return false;
//	}
//	// get TSDF values of neighbors on the front XY plane. (Z = 0)
//	tpos.z = offset.z;
//	for(int i = 0; i<12; i++){
//		tpos.x = offset.x + dx2[i]; tpos.y = offset.y + dy2[i];
//		if(!d_gtvu_Get_TSDF_Value_Uninterpolated(tpos, vol_tsdf, vol_w, dim_cube_dev, front[i])) return false;
//	}
//	// get TSDF values of neighbors on the back XY plane. (Z = 1)
//	tpos.z = offset.z + 1.0f;
//	for(int i = 0; i<12; i++){
//		tpos.x = offset.x + dx2[i]; tpos.y = offset.y + dy2[i];
//		if(!d_gtvu_Get_TSDF_Value_Uninterpolated(tpos, vol_tsdf, vol_w, dim_cube_dev, back[i])) return false;
//	}
//	// get TSDF values of neighbors on the foremost XY plane. (Z = 2)
//	tpos.z = offset.z + 2.0f;
//	for(int i = 0; i<4; i++){
//		tpos.x = offset.x + dx1[i]; tpos.y = offset.y + dy1[i];
//		if(!d_gtvu_Get_TSDF_Value_Uninterpolated(tpos, vol_tsdf, vol_w, dim_cube_dev, backmost[i])) return false;
//	}
//
//	
//	/// Calculate surface normal.
//	float inter2a[2], inter4a[4], inter2b[2], inter4b[4];
//	// compute x-direction SDF gradient at point.
//	// + get z-direction interpolated SDF values of (3 4 5 6) and (7 8 9 10).
//	inter4a[0]=(1.0f-residu.z)*front[2] +residu.z*back[2];	
//	inter4a[1]=(1.0f-residu.z)*front[3] +residu.z*back[3];
//	inter4a[2]=(1.0f-residu.z)*front[4] +residu.z*back[4];	
//	inter4a[3]=(1.0f-residu.z)*front[5] +residu.z*back[5];
//
//	inter4b[0]=(1.0f-residu.z)*front[6] +residu.z*back[6];	
//	inter4b[1]=(1.0f-residu.z)*front[7] +residu.z*back[7];
//	inter4b[2]=(1.0f-residu.z)*front[8] +residu.z*back[8];	
//	inter4b[3]=(1.0f-residu.z)*front[9] +residu.z*back[9];	
//	
//	// + dF(x, y, z)/dx = F(x+1, y, z)-F(x-1, y, z).
//	inter2a[0]=(1.0f-residu.x)*inter4a[0] +residu.x*inter4a[1];	// x-direction interpolation of 3 and 4.
//	inter2a[1]=(1.0f-residu.x)*inter4b[0] +residu.x*inter4b[1];	// x-direction interpolation of 7 and 8.
//	inter2b[0]=(1.0f-residu.x)*inter4a[2] +residu.x*inter4a[3];	// x-direction interpolation of 5 and 6.
//	inter2b[1]=(1.0f-residu.x)*inter4b[2] +residu.x*inter4b[3];	// x-direction interpolation of 9 and 10.
//
//	gx=(1.0f-residu.y)*inter2b[0]+residu.y*inter2b[1];		// y-direction interpolation of (5 6) and (9 10).
//	gx-=(1.0f-residu.y)*inter2a[0]+residu.y*inter2a[1];		// y-direction interpolation of (3 4) and (5 6).
//
//
//	// compute y-direction SDF gradient at point.
//	// + get z-direction interpolated SDF values of (1 4 8 11) and (2 5 9 12).
//	inter4a[0]=(1.0f-residu.z)*front[0]	 +residu.z*back[0];	
//	inter4a[1]=(1.0f-residu.z)*front[3]	 +residu.z*back[3];
//	inter4a[2]=(1.0f-residu.z)*front[7]	 +residu.z*back[7];	
//	inter4a[3]=(1.0f-residu.z)*front[10] +residu.z*back[10];
//
//	inter4b[0]=(1.0f-residu.z)*front[1]	 +residu.z*back[1];	
//	inter4b[1]=(1.0f-residu.z)*front[4]	 +residu.z*back[4];
//	inter4b[2]=(1.0f-residu.z)*front[8]	 +residu.z*back[8];	
//	inter4b[3]=(1.0f-residu.z)*front[11] +residu.z*back[11];
//
//	// + dF(x, y, z)/dx = F(x, y+1, z)-F(x, y-1, z).
//	inter2a[0]=(1.0f-residu.y)*inter4a[0] +residu.y*inter4a[1];	// y-direction interpolation of 1 and 4.
//	inter2a[1]=(1.0f-residu.y)*inter4b[0] +residu.y*inter4b[1];	// y-direction interpolation of 2 and 5.
//	inter2b[0]=(1.0f-residu.y)*inter4a[2] +residu.y*inter4a[3];	// y-direction interpolation of 8 and 11.
//	inter2b[1]=(1.0f-residu.y)*inter4b[2] +residu.y*inter4b[3];	// y-direction interpolation of 9 and 12.
//
//
//	gy=(1.0f-residu.x)*inter2b[0]	+residu.x*inter2b[1];			// x-direction interpolation of (8 11) and (9 12).
//	gy-=(1.0f-residu.x)*inter2a[0]	+residu.x*inter2a[1];			// x-direction interpolation of (1 4) and (2 5).
//
//	// compute z-direction SDF gradient at point.
//	// + get y-direction interpolated SDF values of (X=0|Z=-1 0 1 2) and (X=1|Z=-1 0 1 2).
//	inter4a[0]=(1.0f-residu.y)*foremost[0]	+residu.y*foremost[2];		
//	inter4a[1]=(1.0f-residu.y)*front[3]		+residu.y*front[7];
//	inter4a[2]=(1.0f-residu.y)*back[3]		+residu.y*back[7];				
//	inter4a[3]=(1.0f-residu.y)*backmost[0]	+residu.y*backmost[2];
//
//	inter4b[0]=(1.0f-residu.y)*foremost[1]	+residu.y*foremost[3];		
//	inter4b[1]=(1.0f-residu.y)*front[4]		+residu.y*front[8];
//	inter4b[2]=(1.0f-residu.y)*back[4]		+residu.y*back[8];				
//	inter4b[3]=(1.0f-residu.y)*backmost[1]	+residu.y*backmost[3];
//	
//	// + dF(x, y, z)/dx = F(x, y, z+1)-F(x, y, z-1).
//	inter2a[0]=(1.0f-residu.z)*inter4a[0] +residu.z*inter4a[1];	// z-direction interpolation of (X=0|Z=-1) and (X=0|Z=0).
//	inter2a[1]=(1.0f-residu.z)*inter4b[0] +residu.z*inter4b[1];	// z-direction interpolation of (X=1|Z=-1) and (X=1|Z=0).
//	inter2b[0]=(1.0f-residu.z)*inter4a[2] +residu.z*inter4a[3];	// z-direction interpolation of (X=0|Z=1) and (X=0|Z=2).
//	inter2b[1]=(1.0f-residu.z)*inter4b[2] +residu.z*inter4b[3];	// z-direction interpolation of (X=1|Z=1) and (X=1|Z=2).
//
//	gz=(1.0f-residu.x)*inter2b[0]  +residu.x*inter2b[1];			// x-direction interpolation of (X=0|Z= 1 2) and (X=1|Z= 1 2).
//	gz-=(1.0f-residu.x)*inter2a[0] +residu.x*inter2a[1];			// x-direction interpolation of (X=0|Z=-1 0) and (X=1|Z=-1 0).
//
//	if(gx==0.0f && gy==0.0f && gz==0.0f) return false;
//
//	// save calculation result of surface normal.
//	norm.x = gx; norm.y = gy; norm.z = gz;
//	out_surf_norm = norm.normalised();
//
//	return true;
//	
//}
//
//_CPU_AND_GPU_CODE_ inline bool d_crtc_Cast_Ray_on_TSDF_Cube(
//	// variable parameters.
//	Vector2f p2d,
//	const float *T_gc, const float *T_cg,
//	Vector3f cam_cen, Vector3f light, Vector3f norm_cc, float dist_cg_cen, float theta_max,
//	// fixed parameters.
//	const float *cube_tsdf,
//	const uchar *cube_w,
//	const float *K_dev,
//	const float *origin_dev,
//	const int *dim_cube_dev,
//	const int sz_vox_inv_dev,
//	int ww, int hh,
//	float r_cube, float mu, float sz_vox_inv, int dim_sc,
//	// for output cross point.
//	Vector3f &p3d)
//{
//	// RENDERING NOW!!!!!
//	Vector3f p3d_c, p3d_s, p3d_e, vox_s, vox_e, rd_g, rd_vox;
//	float cos_val, theta, mag, dist_min, dist_max;
//
//	bool flag_valid = false;
//	bool flag_result = false;
//	int cnt = 0;
//	float step_sz, step_sz_coarse, step_scale, sz_sub_cube, total_step, total_step_max, tsdf;
//
//	enum { SEARCH_COARSE, SEARCH_FINE, BEHIND_SURFACE, SEARCH_FINE_BACK } state;
//
//	dist_min = dist_cg_cen - r_cube;
//	dist_max = dist_cg_cen + r_cube;
//
//	// compute the end point of the pixel ray for the input pixel.
//	d_bp_Back_Project(p2d, K_dev, dist_max, p3d_c);
//	d_t_Transform(p3d_c, T_cg, p3d_e);
//
//	// ==================================================================
//	// Ray validity test
//	// ==================================================================
//	// compute the direction of the pixel ray for the input pixel using camera center and far plane of view frustum.
//	rd_g = (p3d_e - cam_cen).normalised();
//
//	// compute angle between the pixel ray and direction vector between the two centers.
//	// check whether the ray crossed the object cube.
//	//cos_val = rd.x*norm_cc.x + rd.y*norm_cc.y + rd.z*norm_cc.z;
//	theta = acosf(dot(rd_g, norm_cc));
//
//	if(theta > theta_max){ return false; }
//
//	// ==================================================================
//	// Ray casting in voxel coordinates.
//	// ==================================================================
//	// calculates start point and ray direction for ray-casting in voxel coordinates.	
//	p3d_s = cam_cen + dist_min*rd_g;
//	// converts points and parameter coordinates from the real global coordinates to the cube voxel coordinates. (mm->voxel)
//	// CAUTION: the offset position of global voxel cube is (-0.5f, -0.5f, -0.5f) in the cube voxel coordinates.
//	// compute scale factor for converting TSDF value to 'mm' unit.	
//	step_scale = mu*sz_vox_inv;
//	// ======================================================
//	// 현재 coarse search 는 step_scale 로 진행해야 함.
//	// 현재 ray 가 invalid 한 곳을 지나는 중인 경우 (tsdf 값이 없음) 
//	// 현재 coarse search 보다 한 단계 위의 coarse search (step size = dim_sc) 로 진행해도 될 듯.
//	step_sz = step_sz_coarse = step_scale; //fmaxf(step_scale+0.1f, float(dim_sc));
//	// ======================================================
//	// set maximum search length of the pixel ray to 2*in_radius_cube.
//	total_step_max = 2.0f*r_cube*sz_vox_inv;		//	 total_step_max=pe_g.d_Distance(ps_g)*in_voxel_sz_inv;
//
//	// starts casting ray.
//	total_step = 0.0f;
//	// Validity check 안해도 되나??
//	d_gpv_Get_Position_in_Voxel(p3d_s, origin_dev, dim_cube_dev, sz_vox_inv_dev, vox_s);
//	//d_gpv_Get_Position_in_Voxel(p3d_s, origin_dev, dim_cube_dev, sz_vox_inv_dev[0], vox_s);
//	// Validity check 안해도 되나??
//	flag_valid = d_gtvu_Get_TSDF_Value_Uninterpolated(vox_s, cube_tsdf, cube_w, dim_cube_dev, tsdf);
//	if(!flag_valid)		state = SEARCH_COARSE;
//	else if(tsdf<=0.0f)	state = SEARCH_FINE_BACK;
//	else				state = SEARCH_FINE;
//
//	// ===================================================
//	// Casts ray.
//	// ===================================================
//	while(state!=BEHIND_SURFACE){
//
//		// ===================================================
//		// Sets step size.
//		// ===================================================
//		if(state == SEARCH_COARSE)    step_sz = step_sz_coarse;
//		else if(state == SEARCH_FINE) step_sz = fmaxf(step_scale*tsdf, 1.0f);	// in front of surface.  
//		else if(state == SEARCH_FINE_BACK)  step_sz = fminf(step_scale*tsdf, -1.0f);  // behind surface.
//
//		// ===================================================
//		// Proceeds ray casting.
//		// ===================================================
//		vox_s += step_sz*rd_g;	//vox_s.x += step_sz*rd.x; vox_s.y += step_sz*rd.y; vox_s.z += step_sz*rd.z;
//		total_step += step_sz;
//		// if total step length exceeds maximum step size, break while loop.
//		if(total_step > total_step_max)	break;
//
//		flag_valid = d_gtvu_Get_TSDF_Value_Uninterpolated(vox_s, cube_tsdf, cube_w, dim_cube_dev, tsdf);
//
//		// approached near the surface.
//		if(tsdf>-1.0f && tsdf<1.0f){
//			// add interpolation version of TSDF value calculator.
//			flag_valid = d_gtvi_Get_TSDF_Value_Interpolated(vox_s, cube_tsdf, cube_w, dim_cube_dev, tsdf);
//		}
//
//		// ===================================================
//		// Checks casting mode.
//		// ===================================================
//		if(state == SEARCH_COARSE){
//			if(tsdf<=0.0f) state = SEARCH_FINE_BACK; // go back.
//			else           state = SEARCH_FINE;		 // start fine search.
//		}
//		else if(state == SEARCH_FINE){
//			// ================================================================
//			// Infinite Loop Cause 1
//			// : In case of noisy TSDF data processing.
//			// Solution: Remove noisy data processing part.
//			// if(tsdf >= 1.0f || tsdf <= -1.0f) state = SEARCH_COARSE;  // convert to coarse search. (ray met noise voxel data)
//			// ================================================================
//			if(tsdf<=0.0f)  state = BEHIND_SURFACE; // break while loop, and find intersection.
//		}
//		else if(state == SEARCH_FINE_BACK){
//			if(tsdf>0.0f) state = SEARCH_FINE;     // start fine search.
//		}
//
//		// for debugging.
//		//if(cnt++ > 100)	break;
//		// for debugging.
//	}
//
//	// ===================================================
//	// Find intersection between ray and surface.
//	// ===================================================
//	if(state==BEHIND_SURFACE){
//
//		step_sz = fminf(step_scale*tsdf, -1.0f);	vox_s += step_sz*rd_g;
//		d_gtvi_Get_TSDF_Value_Interpolated(vox_s, cube_tsdf, cube_w, dim_cube_dev, tsdf);
//		step_sz = step_scale*tsdf;	vox_s += step_sz*rd_g;
//
//		// converts points coordinates from the cube voxel coordinates to the real global coordinates . (voxel->mm)
//		d_gpw_Get_Position_in_World(vox_s, origin_dev, dim_cube_dev, sz_vox_inv_dev, p3d);
//
//		flag_result = true;
//	}
//
//	return flag_result;
//
//}
//
///////

#include "_yooji_cuda_math.hpp"