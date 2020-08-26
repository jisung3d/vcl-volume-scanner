// ==============================================================================
// Tracking state class for device memory.
// ==============================================================================
class GKvRgbdFrame
{
public:

	// /////////////////////////////////////////////////
	// Host functions
	// /////////////////////////////////////////////////	
	bool create(Vector2i in_dim){
		if(in_dim.x <= 0 || in_dim.y <= 0)	return false;
		sz_map = in_dim;

		Vector2i dim_lev = in_dim;
		for(int i=0; i<GK_LEVEL_OF_IMAGE_PYRAMID; i++){
			z_map_depth[i].create(dim_lev.y,dim_lev.x,1);
			z_map_vertex[i].create(dim_lev.y,dim_lev.x,3);
			z_map_normal[i].create(dim_lev.y,dim_lev.x,3);

			z_img_texture[i].create(dim_lev.y,dim_lev.x,3);

			dim_lev /= 2;
		}

		return true;
	}
	void copy(GKvRgbdFrame *a){
		sz_map = a->sz_map;

		for(int i=0; i<GK_LEVEL_OF_IMAGE_PYRAMID; i++){
			z_map_depth[i].copy(&a->z_map_depth[i]);
			z_map_vertex[i].copy(&a->z_map_vertex[i]);
			z_map_normal[i].copy(&a->z_map_normal[i]);

			z_img_texture[i].copy(&a->z_img_texture[i]);
		}


	}
	void release(){
		for(int i=0; i<GK_LEVEL_OF_IMAGE_PYRAMID; i++){
			z_map_depth[i].release();  z_map_vertex[i].release();	z_map_normal[i].release();
			z_img_texture[i].release(); 
		}

		sz_map = 1;
	}

	// /////////////////////////////////////////////////
	// Pointers.
	// /////////////////////////////////////////////////	
	// tracking state pointers.
	float* vp_map_depth(int in_lev){ return z_map_depth[in_lev].vp(); }
	float* vp_map_vertex(int in_lev){ return z_map_vertex[in_lev].vp(); }
	float* vp_map_normal(int in_lev){ return z_map_normal[in_lev].vp(); }

	uchar* vp_img_texture(int in_lev){ return z_img_texture[in_lev].vp(); }

	~GKvRgbdFrame(){ release(); }
	GKvRgbdFrame(){

		for(int i=0; i<GK_LEVEL_OF_IMAGE_PYRAMID; i++){
			z_map_depth[i].create(1,1,1);
			z_map_vertex[i].create(1,1,3);
			z_map_normal[i].create(1,1,3);

			z_img_texture[i].create(1,1,3);
		}


	}

	// /////////////////////////////////////////////////
	// HOST //////////////////////////////////////////
	// /////////////////////////////////////////////////
	Vector2i sz_map;  // matrix size.

private:

	// /////////////////////////////////////////////////
	// DEVICE //////////////////////////////////////////
	// /////////////////////////////////////////////////
	GKvMatrixFloat z_map_depth[GK_LEVEL_OF_IMAGE_PYRAMID];		// rendered depth map.
	GKvMatrixFloat z_map_vertex[GK_LEVEL_OF_IMAGE_PYRAMID];	// rendered vertex map. (3 ch)	
	GKvMatrixFloat z_map_normal[GK_LEVEL_OF_IMAGE_PYRAMID];	// rendered normal map. (3 ch)

	GKvMatrixUchar z_img_texture[GK_LEVEL_OF_IMAGE_PYRAMID];	// rendered texture image. (3 ch)

	////////////////////////////////////////////////////

};