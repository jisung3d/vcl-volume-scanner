// ==============================================================================
// Tracking state class for device memory.
// ==============================================================================
class GKvTrackingState
{
public:

	// /////////////////////////////////////////////////
	// Host functions
	// /////////////////////////////////////////////////	
	bool create(Vector2i in_dim){
		if(in_dim.x <= 0 || in_dim.y <= 0)	return false;
		sz_map = in_dim;
		center = 0.0f;

		z_T_gc.create(4, 4, 1);	z_T_cg.create(4, 4, 1); // fixed size.
		cudaMemcpy(z_T_gc.vp(), z_eye4f, 16*sizeof(float), cudaMemcpyHostToDevice);
		cudaMemcpy(z_T_cg.vp(), z_eye4f, 16*sizeof(float), cudaMemcpyHostToDevice);
		
		Vector2i dim_lev = in_dim;
		for(int i=0; i<GK_LEVEL_OF_IMAGE_PYRAMID; i++){
			z_map_depth[i].create(dim_lev.y, dim_lev.x, 1);
			z_map_vertex[i].create(dim_lev.y, dim_lev.x, 3);
			z_map_normal[i].create(dim_lev.y, dim_lev.x, 3);

			z_img_texture[i].create(dim_lev.y,dim_lev.x,3);
			z_img_normal[i].create(dim_lev.y,dim_lev.x,1);

			dim_lev /= 2;
		}
		
		return true;
	}
	void copy(GKvTrackingState *a){
		sz_map = a->sz_map;	
		center = a->center;

		z_T_gc.copy(&a->z_T_gc); z_T_cg.copy(&a->z_T_cg);	
		for(int i=0; i<GK_LEVEL_OF_IMAGE_PYRAMID; i++){
			z_map_depth[i].copy(&a->z_map_depth[i]);
			z_map_vertex[i].copy(&a->z_map_vertex[i]);		
			z_map_normal[i].copy(&a->z_map_normal[i]);

			z_img_texture[i].copy(&a->z_img_texture[i]);
			z_img_normal[i].copy(&a->z_img_normal[i]);
		}

		
	}
	void release(){
		z_T_gc.release();	z_T_cg.release();
		for(int i=0; i<GK_LEVEL_OF_IMAGE_PYRAMID; i++){
			z_map_depth[i].release();  z_map_vertex[i].release();	z_map_normal[i].release(); 
			z_img_texture[i].release(); z_img_normal[i].release();	
		}
		
		sz_map = 1;
	}
	bool set_transform(GKvMatrixFloat *in_mat,bool flg_from_inv = false){

		int mw = in_mat->mw(),mh = in_mat->mh();
		int len_byte = 16*sizeof(float);
		float R[9],t[3],T[16],Tinv[16],params[6];

		if(mw!=4 || mh!=4)	return false;

		// copy matrix from device to host.
		if(!flg_from_inv) cudaMemcpy(T,in_mat->vp(),len_byte,cudaMemcpyDeviceToHost);
		else{
			cudaMemcpy(Tinv,in_mat->vp(),len_byte,cudaMemcpyDeviceToHost);
			d_im_Inverse_Matrix_4x4(Tinv,T);
		}

// 		// extract rotation parameters(Euler angles) from R matrix.
// 		d_gpt_Get_Params_from_Transformation(T,params);
// 
// 		//if(!Kv_Printf("%f %f %f %f %f %f\n", zz_rx, zz_ry, zz_rz, zz_tx, zz_ty, zz_tz)) exit(0);
// 
// 		// set transformation matrix.
// 		d_gtp_Get_Transformation_from_Params(params,T);

		// set transformation matrix.
		
		// set inverse transformation.
		d_im_Inverse_Matrix_4x4(T,Tinv);

		// copy matrix from host to device.
		cudaMemcpy(z_T_gc.vp(),T,len_byte,cudaMemcpyHostToDevice);
		cudaMemcpy(z_T_cg.vp(),Tinv,len_byte,cudaMemcpyHostToDevice);
		// set camera center.
		d_gcc_Get_Camera_Center(T,center);

		return true;
	}

	// /////////////////////////////////////////////////
	// Pointers.
	// /////////////////////////////////////////////////	
	// tracking state pointers.
	float* vp_map_depth(int in_lev){ return z_map_depth[in_lev].vp(); }
	float* vp_map_vertex(int in_lev){ return z_map_vertex[in_lev].vp(); }
	float* vp_map_normal(int in_lev){ return z_map_normal[in_lev].vp(); }

	uchar* vp_img_texture(int in_lev){ return z_img_texture[in_lev].vp(); }
	uchar* vp_img_norm(int in_lev){ return z_img_normal[in_lev].vp(); }

	float *vp_T_gc(){ return z_T_gc.vp(); }
	float *vp_T_cg(){ return z_T_cg.vp(); }
	
	// matrix size of tracking states.
// 	Vector2i ms(){ return z_sz; }
// 	Vector3f cc(){ return z_center; }

	~GKvTrackingState(){ release(); }
	GKvTrackingState(){
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// Identity matrices for initializing pose matrices.
		float eye4f[4*4] =
			{float(1), float(0), float(0), float(0),
			float(0), float(1), float(0), float(0),
			float(0), float(0), float(1), float(0),
			float(0), float(0), float(0), float(1)};
		for(int i=0; i<16; i++) z_eye4f[i] = eye4f[i];
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		sz_map = 1;
		center = 0.0f;

		z_T_gc.create(4, 4, 1);	z_T_cg.create(4, 4, 1); // fixed size.
		cudaMemcpy(z_T_gc.vp(), z_eye4f, 16*sizeof(float), cudaMemcpyHostToDevice);
		cudaMemcpy(z_T_cg.vp(), z_eye4f, 16*sizeof(float), cudaMemcpyHostToDevice);

		for(int i=0; i<GK_LEVEL_OF_IMAGE_PYRAMID; i++){
			z_map_depth[i].create(1, 1, 1);
			z_map_vertex[i].create(1, 1, 3);
			z_map_normal[i].create(1, 1, 3);

			z_img_texture[i].create(1,1,3);
			z_img_normal[i].create(1,1,1);
		}

		
	}

	// /////////////////////////////////////////////////
	// HOST //////////////////////////////////////////
	// /////////////////////////////////////////////////
	Vector2i sz_map;      // matrix size.
	Vector3f center;  // camera center.

private:

	// /////////////////////////////////////////////////
	// DEVICE //////////////////////////////////////////
	// /////////////////////////////////////////////////
	GKvMatrixFloat z_T_gc, z_T_cg;	// estimated pose.

	GKvMatrixFloat z_map_depth[GK_LEVEL_OF_IMAGE_PYRAMID];		// rendered depth map.
	GKvMatrixFloat z_map_vertex[GK_LEVEL_OF_IMAGE_PYRAMID];	// rendered vertex map. (3 ch)	
	GKvMatrixFloat z_map_normal[GK_LEVEL_OF_IMAGE_PYRAMID];	// rendered normal map. (3 ch)

	GKvMatrixUchar z_img_texture[GK_LEVEL_OF_IMAGE_PYRAMID];	// rendered texture image. (3 ch)
	GKvMatrixUchar z_img_normal[GK_LEVEL_OF_IMAGE_PYRAMID];	// rendered normal image.

	////////////////////////////////////////////////////
	// Identity matrix.
	float z_eye4f[4*4];

};