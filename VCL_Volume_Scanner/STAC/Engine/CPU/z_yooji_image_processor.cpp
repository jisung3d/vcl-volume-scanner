/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_image_processor.cpp 
/////////////////////////////////////////////////////////////////////////////////////////////

#include "../../../stdafx.h"
#include "../../_yooji_2016_object_scanning.h"

/////////////////////////////////////////////////////////////////////////////////////////////
// LCKvYooji_Scanner_Display 
/////////////////////////////////////////////////////////////////////////////////////////////
//********************************************************************************************
LCKvYooji_Image_Processor::LCKvYooji_Image_Processor()
//********************************************************************************************
{
	// set default value of projected camera center.
	zz_classname = "LCKvYooji_Image_Processor";

}

//********************************************************************************************
LCKvYooji_Image_Processor::~LCKvYooji_Image_Processor()
//********************************************************************************************
{
}

//********************************************************************************************
void LCKvYooji_Image_Processor::dec_Detect_Edges_using_Canny(
	CKvMatrixUchar *in_img,
	CKvYooji_Edge_Map *out_edge_map,
	float in_th_for_edge_mask,
	CKvMatrixBool *in_roi_mask)
//********************************************************************************************
{

	CKvMatrixShort *mp_grad[2];

	float *p_grad_mag, *p_grad_ori;
	short *p_grad[2];
	bool *p_mask;

	float gh, gv, theta, mag;
	int ww, hh;
		
	zz_canny.im_Import(in_img, in_roi_mask);	

	mp_grad[0] = zz_canny.pgh();		// horizontal edge magnitude.
	mp_grad[1] = zz_canny.pgv();		// vertical edge magnitude.

	p_grad[0] = mp_grad[0]->mps(ww, hh)[0];
	p_grad[1] = mp_grad[1]->mps(ww, hh)[0];
	
	// Remain ROI pixels only.
	if(in_roi_mask){
		int ww_mask, hh_mask;
		p_mask = in_roi_mask->mps(ww_mask, hh_mask)[0];

		if(ww_mask == ww && hh_mask == hh){
			for(int i=0; i<ww*hh; i++){

				if(!p_mask[i]){

					p_grad[0][i] = p_grad[1][i] = 0;
					zz_canny.pe()->vp()[i] = 0;

				}

			}
		}
	}

	// Get gradient information.
	if(ww != out_edge_map->ww || hh != out_edge_map->hh)	out_edge_map->c_Create(hh, ww);

	p_grad_mag = out_edge_map->grad_mag.vp();
	p_grad_ori = out_edge_map->grad_ori.vp();
	p_mask = out_edge_map->edge_mask.vp();

	for(int i = 0; i<ww*hh; i++){

		gh = (float)p_grad[0][i];	gv = (float)p_grad[1][i];

		mag = sqrt(SQUARE((float)gh) + SQUARE((float)gv));
		theta = atan2(float(gv), float(gh));

		p_grad_mag[i] = mag;
		//p_grad_mag[i] = (mag < 150.0f) ? mag : 150.0f;				// Maximum gradient suppression.			
		p_grad_ori[i] = (theta < 0.0f) ? theta + 360.0f : theta;	// Orientation range conversion.

		p_mask[i] = (mag < in_th_for_edge_mask) ? false : true;		// Edge mask.

// 		if(abs(gh) > 0.0f || abs(gv) > 0.0f){
// 
// 			mag = sqrt(SQUARE((float)gh) + SQUARE((float)gv));
// 			theta = atan2(float(gv), float(gh));
// 
// 			p_grad_mag[i] = mag;
// 			//p_grad_mag[i] = (mag < 150.0f) ? mag : 150.0f;				// Maximum gradient suppression.			
// 			p_grad_ori[i] = (theta < 0.0f) ? theta + 360.0f : theta;	// Orientation range conversion.
// 			 
// 			p_mask[i] = (mag < in_th_for_edge_mask) ? false : true;		// Edge mask.
// 		}
// 		else{
// 			p_grad_mag[i] = p_grad_ori[i] = 0.0f;
// 			p_mask[i] = false;
// 		}
	}

// 	CKvScreen sc[3];
// 	//zz_canny.de_Display_Edges(&sc[0], NULL);
// 	sc[0].s_d_Display(1.0f, 0.0f, mp_grad[0]);	
// 	sc[1].s_d_Display(1.0f, 0.0f, mp_grad[1]);
//  	sc[2].s_d_Display(&out_edge_map->edge_mask);
// // 	sc[2].s_d_Display(zz_canny.pe());
// 
// 	if(!Kv_Printf("Edge!"))	exit(0);
}

//********************************************************************************************
void LCKvYooji_Image_Processor::cpm_Compute_Point_Map(
	CKvMatrixFloat *in_map_depth,
	CKvYooji_Intrinsics *in_intrinsics,
	CKvSet2d_of_Point3Df *out_map_p3d)
//********************************************************************************************
{
	CKvPoint3Df *p_p3d; float *p_depth;
	CKvPixel xy;
	int ww,hh;

	in_map_depth->ms(ww,hh);
	if(ww != out_map_p3d->mw() || hh!=out_map_p3d->mh()) out_map_p3d->c_Create(hh,ww);

	// get pointers.
	p_depth = in_map_depth->vp();
	p_p3d = out_map_p3d->vp();

	// initialize point maps.
	for(int len=0; len<ww*hh; len++) p_p3d[len].x = p_p3d[len].y = p_p3d[len].z = -100.0f;

	// compute 3d point using back projection.
	for(int i=0; i<hh; i++){
		for(int j=0; j<ww; j++){

			int tidx = i*ww + j;
			float td = p_depth[tidx];

			if(td<=0.0f) continue;
			
			xy.x = j; xy.y = i;
			in_intrinsics->back_project(xy, td, p_p3d[tidx]);
		}
	}
}

//********************************************************************************************
void LCKvYooji_Image_Processor::enm_Estimate_Normal_Map(
	CKvSet2d_of_Point3Df *in_map_p3d,
	CKvSet2d_of_Point3Df *out_map_normal,
	int in_sz_mask_half)
//********************************************************************************************
{
	CKvPoint3Df *p_p3d, *p_normal;
	int ww, hh;

	in_map_p3d->ms(ww, hh);
	if(ww != out_map_normal->mw() || hh!=out_map_normal->mh()) out_map_normal->c_Create(hh, ww);

	// get pointers.
	p_p3d = in_map_p3d->vp();
	p_normal = out_map_normal->vp();

	// initialize normal maps.
	for(int len=0; len<ww*hh; len++) p_normal[len].x = p_normal[len].y = p_normal[len].z = -100.0f;

	// compute normals using fixed size window.
	// normal = (dz/dx, dz/dy, -1).normalize
	// default normal vector is (0, 0, -1).
	int szh = in_sz_mask_half;
	for(int i=szh; i<hh-szh; i++){
		for(int j=szh; j<ww-szh; j++){

			CKvPoint3Df *px1, *px2, *py1, *py2;
			float dzdx, dzdy, dx, dy;			
			float x1, x2, y1, y2;

			// get 4-neighborhoods.
			int tidx = i*ww + j;
			px1 = &p_p3d[tidx - 1];  px2 = &p_p3d[tidx + 1];
			py1 = &p_p3d[tidx - ww]; py2 = &p_p3d[tidx + ww];

			// check NaN pixels. In point map, NaN is -100.0f.
			if(p_p3d[tidx].x == -100.0f ||
				px1->x == -100.0f || px2->x == -100.0f ||
				py1->x == -100.0f || py2->x == -100.0f) 
				continue;			

			// compute dz/dx and dz/dy.
			// if dx or dy is zero? 
			//////////////////////////////////////////////////////////////////////////
			dx = px2->x - px1->x;	if(abs(dx)<1.0e-7f) continue;
			dy = py2->y - py1->y;	if(abs(dy)<1.0e-7f) continue;
			//////////////////////////////////////////////////////////////////////////

			dzdx = (px2->z - px1->z)/dx;
			dzdy = (py2->z - py1->z)/dy;

			// set computed normal.
			p_normal[tidx].x = dzdx;
			p_normal[tidx].y = dzdy;
			p_normal[tidx].z = -1.0f;

			// normalize normal.
			p_normal[tidx].n_Normalize();

			//////////////////////////////////////////////////////////////////////////
			// normal filtering.
			if(-p_normal[tidx].z < cos(PI*60.0f/180.0f)){
				p_p3d[tidx].x = p_p3d[tidx].y = p_p3d[tidx].z = -100.0f; 
				p_normal[tidx].x = p_normal[tidx].y = p_normal[tidx].z = -100.0f;
			}
			//////////////////////////////////////////////////////////////////////////

			//printf("%f %f %f\n", p_normal[tidx].x, p_normal[tidx].y, p_normal[tidx].z);
		}
	}


}

//********************************************************************************************
void LCKvYooji_Image_Processor::rsd_Remove_Specular_Depth(
	CKvSet2d_of_Point3Df *in_map_p3d,
	CKvSet2d_of_Point3Df *in_map_normal,
	CKvMatrixBool *out_map_specular,
	float in_th_angle_with_Z)
//********************************************************************************************
{
	int ww, hh;

	CKvPoint3Df *p_p3d = in_map_p3d->mps(ww, hh)[0];
	CKvPoint3Df *p_normals = in_map_normal->mps(ww, hh)[0];

	Vector3f V, n, p_refl, _z_axis(0,0, 1.0f);	

	out_map_specular->c_Create(hh, ww, false);

	for(int i=0; i<hh; i++){
		for(int j=0; j<ww; j++){

			int tidx = i*ww + j;
			float alpha, theta;

			if(p_p3d[tidx].x == -100.0f || p_normals[tidx].x == -100.0f) continue;

			V.x = p_p3d[tidx].x;		V.y = p_p3d[tidx].y;		V.z = p_p3d[tidx].z;
			n.x = p_normals[tidx].x;	n.y = p_normals[tidx].y;	n.z = p_normals[tidx].z;

			alpha = -dot(V, n);
			p_refl = 2*alpha*n + V;

			p_refl.normalised();

			theta = acos(dot(p_refl, _z_axis));

// 			cout << V << endl;
// 			cout << n << endl;

			theta = 180.0f*theta/PI;

//			printf("theta: %f alpha: %f\n", theta, alpha);

			if(theta < 30/*in_th_angle_with_Z*/){	out_map_specular->vp()[tidx] = true;	}


		}
	}

	CKvScreen sc;
	sc.s_d_Display(out_map_specular); 
	if(!Kv_Printf("Specular!!!")) exit(0);

}

//********************************************************************************************
// CAUTION: WHEN BUILD UPPER LEVEL, SMOOTHING LOWER LEVEL IMAGE FIRST AND DOWNSAMPLING.
// THIS PROCEDURE IS NOT EFFICIENT. SMOOTHING ONLY NEEDS TO DOWNSAMPLED PIXELS.
void LCKvYooji_Image_Processor::cpd_Construct_Pyramids_Down(
		CKvMatrixBool *in_img_bool,
		CKvYooji_Pyramid_Mat_Bool *out_pyrams_img_bool,
		int in_level_of_pyrams)
//********************************************************************************************
{
	int ww, hh;

	in_img_bool->ms(ww, hh);

	if(out_pyrams_img_bool->levOfScale != in_level_of_pyrams){
		out_pyrams_img_bool->c_Create(ww, hh, in_level_of_pyrams);
	}	

	// Set bottom level of pyramid.
	out_pyrams_img_bool->imgs[0].cp_Copy(in_img_bool);
	// Build upper levels of pyramid.
	// CAUTION: WHEN BUILD UPPER LEVEL, SMOOTHING LOWER LEVEL IMAGE FIRST AND DOWNSAMPLING.
	// THIS PROCEDURE IS NOT EFFICIENT. SMOOTHING ONLY NEEDS TO DOWNSAMPLED PIXELS.
	for(int i=1; i<in_level_of_pyrams; i++){		
		dih_Downsample_Image_Half(&out_pyrams_img_bool->imgs[i-1], &out_pyrams_img_bool->imgs[i]);		
	}
// 
// 	CKvScreen sc[3];
// 
// 	sc[0].s_d_Display(&(*out_pyrams_img_rgb)[0]);
// 	sc[1].s_d_Display(&(*out_pyrams_img_rgb)[1]);
// 	sc[2].s_d_Display(&(*out_pyrams_img_rgb)[2]);
// 
// 	if(!Kv_Printf("Pyrams"))	exit(0);

}

//********************************************************************************************
// CAUTION: WHEN BUILD UPPER LEVEL, SMOOTHING LOWER LEVEL IMAGE FIRST AND DOWNSAMPLING.
// THIS PROCEDURE IS NOT EFFICIENT. SMOOTHING ONLY NEEDS TO DOWNSAMPLED PIXELS.
void LCKvYooji_Image_Processor::cpd_Construct_Pyramids_Down(
		CKvMatrixUchar *in_img_gray,
		CKvYooji_Pyramid_Mat_Uchar *out_pyrams_img_rgb,
		int in_level_of_pyrams)
//********************************************************************************************
{
	int ww, hh;

	in_img_gray->ms(ww, hh);

	if(out_pyrams_img_rgb->levOfScale != in_level_of_pyrams){
		out_pyrams_img_rgb->c_Create(ww, hh, in_level_of_pyrams);
	}	

	// Set bottom level of pyramid.
	out_pyrams_img_rgb->imgs[0].cp_Copy(in_img_gray);
	//////////////////////////////////////////////////////////////////////////
	//sim5_Smooth_Image_Mean_5x5(in_img_gray,&zz_img_tmp[0],false,&zz_img_padding[0]);
	//sim5_Smooth_Image_Mean_5x5(&zz_img_tmp[0],&out_pyrams_img_rgb->imgs[0],false,&zz_img_padding[0]);
	//sig5_Smooth_Image_Gaussian_5x5(in_img_gray,&out_pyrams_img_rgb->imgs[0],false,&zz_img_padding[0]);
	//////////////////////////////////////////////////////////////////////////
	// Build upper levels of pyramid.
	// CAUTION: WHEN BUILD UPPER LEVEL, SMOOTHING LOWER LEVEL IMAGE FIRST AND DOWNSAMPLING.
	// THIS PROCEDURE IS NOT EFFICIENT. SMOOTHING ONLY NEEDS TO DOWNSAMPLED PIXELS.
	for(int i=1; i<in_level_of_pyrams; i++){
		sig5_Smooth_Image_Gaussian_5x5(&out_pyrams_img_rgb->imgs[i-1], &zz_img_tmp[i-1], false, &zz_img_padding[i-1]);
		dih_Downsample_Image_Half(&zz_img_tmp[i-1], &out_pyrams_img_rgb->imgs[i]);	

// 		out_pyrams_img_rgb->imgs[i].ms(ww, hh);
// 		printf("%d] %d %d\n", i, ww, hh);
	}
// 
//   	CKvScreen sc[3];
//  // 
//   	sc[0].s_d_Display(&(*out_pyrams_img_rgb).imgs[0]);
//   	sc[1].s_d_Display(&(*out_pyrams_img_rgb).imgs[1]);
//   	sc[2].s_d_Display(&(*out_pyrams_img_rgb).imgs[2]);
//  // 
//   	if(!Kv_Printf("Pyrams"))	exit(0);

}


//********************************************************************************************
// CAUTION: WHEN BUILD UPPER LEVEL, SMOOTHING LOWER LEVEL IMAGE FIRST AND DOWNSAMPLING.
// THIS PROCEDURE IS NOT EFFICIENT. SMOOTHING ONLY NEEDS TO DOWNSAMPLED PIXELS.
void LCKvYooji_Image_Processor::cpdd_Construct_Pyramids_Down_Depth(
		CKvMatrixFloat *in_map_depth,
		CKvYooji_Pyramid_Mat_Float *out_pyrams_map,
		int in_level_of_pyrams)
//********************************************************************************************
{
	int ww, hh;

	in_map_depth->ms(ww, hh);

	if(out_pyrams_map->levOfScale != in_level_of_pyrams){
		out_pyrams_map->c_Create(ww, hh, in_level_of_pyrams);
	}
	// Set bottom level of pyramid.
	out_pyrams_map->imgs[0].cp_Copy(in_map_depth);
	//////////////////////////////////////////////////////////////////////////
	//sdg5_Smooth_Depthmap_Gaussian_5x5(in_map_depth,&out_pyrams_map->imgs[0],false,&zz_map_tmp[0]);
	//////////////////////////////////////////////////////////////////////////

	// Build upper levels of pyramid.
	// CAUTION: WHEN BUILD UPPER LEVEL, SMOOTHING LOWER LEVEL IMAGE FIRST AND DOWNSAMPLING.
	// THIS PROCEDURE IS NOT EFFICIENT. SMOOTHING ONLY NEEDS TO DOWNSAMPLED PIXELS.
	for(int i=1; i<in_level_of_pyrams; i++){
		sdg5_Smooth_Depthmap_Gaussian_5x5(&out_pyrams_map->imgs[i-1], &zz_map_tmp[i-1], false, &zz_map_padding[i-1]);
		ddh_Downsample_Depthmap_Half(&zz_map_tmp[i-1], &out_pyrams_map->imgs[i]);	
		//ddh_Downsample_Depthmap_Half(&out_pyrams_map->imgs[i-1], &out_pyrams_map->imgs[i]);

		//out_pyrams_map->imgs[i].ms(ww, hh);
 		//printf("%d] %d %d\n", i, ww, hh);
	}

// 	CKvScreen sc[3];
// 
// 	sc[0].s_d_Display(100.0f, 0.0f, &out_pyrams_map->imgs[0]);
// 	sc[1].s_d_Display(100.0f, 0.0f, &out_pyrams_map->imgs[1]);
// 	sc[2].s_d_Display(100.0f, 0.0f, &out_pyrams_map->imgs[2]);
// 
// 	if(!Kv_Printf("Pyrams Depth"))	exit(0);

}

//********************************************************************************************
void LCKvYooji_Image_Processor::cpde_Construct_Pyramids_Down_Edge(
	CKvYooji_Pyramid_Mat_Uchar *in_pyram_imgs,
	CKvYooji_Pyramid_Map_Edge *out_pyram_edges,
	float in_th_for_edge_mask)
//********************************************************************************************
{
//	int level = in_pyram_imgs->levOfScale;
//	int ww, hh;
//
//	float *p_grad_mag, *p_grad_ori;
//	short *p_grad_x, *p_grad_y;
//	bool *p_mask;
//
//	float gh, gv, mag, theta;
//	
//	in_pyram_imgs->imgs[0].ms(ww, hh);
//	out_pyram_edges->c_Create(ww, hh, level);
//
//	// if(!Kv_Printf("%d %d %d", ww, hh, level))	exit(0);
//
//	for(int k=0; k<level; k++){
//		
//		//CKvEdgeDetector_Canny canny;
//
//	//	printf("level: %d (%d %d)\n", k, in_pyram_imgs->imgs[k].mw(), in_pyram_imgs->imgs[k].mh());
//
//	//	CKvScreen sc;	sc.s_d_Display(&(in_pyram_imgs->imgs[k]));	if(!Kv_Printf("%d", k)) exit(0);
//		
//		zz_canny.im_Import(&(in_pyram_imgs->imgs[k]), NULL);
//	
//
//	//	printf("canny: %d (%d %d)\n", k, zz_canny.pgh()->mw(), zz_canny.pgh()->mh());
//
//		p_grad_x = out_pyram_edges->grad_x[k].cp_Copy(zz_canny.pgh())[0];
//		p_grad_y = out_pyram_edges->grad_y[k].cp_Copy(zz_canny.pgv())[0];
//
//	//	printf("aaaa\n");
//		                                       
//		// Get gradient information.
//		p_grad_mag = out_pyram_edges->grad_mag[k].vp();
//		p_grad_ori = out_pyram_edges->grad_ori[k].vp();
//		p_mask = out_pyram_edges->edge_mask[k].vp();
//
//	//	printf("bbbbb: %d %d\n", out_pyram_edges->grad_mag[k].mw(), out_pyram_edges->grad_mag[k].mh());
//
//		for(int i = 0; i<ww*hh; i++){
//
//			gh = (float)p_grad_x[i];	gv = (float)p_grad_y[i];
//
//			mag = sqrt(SQUARE((float)gh) + SQUARE((float)gv));
//			theta = atan2(float(gv), float(gh));
//
// 			p_grad_mag[i] = mag;
// 			//p_grad_mag[i] = (mag < 150.0f) ? mag : 150.0f;				// Maximum gradient suppression.			
// 			p_grad_ori[i] = (theta < 0.0f) ? theta + 360.0f : theta;	// Orientation range conversion.
// 
// 			p_mask[i] = (mag < in_th_for_edge_mask) ? false : true;		// Edge mask.
//
//		}
//	//	printf("level: %d\n", k);
//
//		
//	}
//// 
//// 	CKvScreen sc;
//// 
//// 	sc.s_d_Display(&out_pyram_edges->edge_mask[0]);
//// 	if(!Kv_Printf("Edge mask"))	exit(0);
//
//
}

//********************************************************************************************
void LCKvYooji_Image_Processor::cpdi_Construct_Pyramids_Down_Intrinsics(
	CKvYooji_Intrinsics *in_intrinsics,
	CKvYooji_Pyramid_Intrinsics *out_pyrams_intrinsics,
	int in_level_of_pyrams)
//********************************************************************************************
{
	if(out_pyrams_intrinsics->levOfScale != in_level_of_pyrams){
		out_pyrams_intrinsics->c_Create(in_level_of_pyrams);
	}

	out_pyrams_intrinsics->intrins[0].copy(in_intrinsics);

	for(int i=1; i<in_level_of_pyrams; i++){
		dih_Downsample_Intrinsics_Half(&out_pyrams_intrinsics->intrins[i-1], &out_pyrams_intrinsics->intrins[i]);
	}
}


//********************************************************************************************
void LCKvYooji_Image_Processor::dih_Downsample_Image_Half(
	CKvMatrixBool *in_img_bool,
	CKvMatrixBool *out_img_downsampled)
//********************************************************************************************
{
	bool *p_in_img, *p_out_img;
	int ww, hh, ww2, hh2, wwp, hhp;	

	// Initialization.
	p_in_img = in_img_bool->mps(ww, hh)[0];
	ww2 = ww/2;	hh2 = hh/2;

	// + create output image.
	if(out_img_downsampled->mw() != ww2 || out_img_downsampled->mh() != hh2)
		out_img_downsampled->c_Create(hh2, ww2, false);
	p_out_img = out_img_downsampled->vp();

	for(int j2=0; j2<hh2; j2++){

		int j = 2*j2 + 1;

		for(int i2=0; i2<ww2; i2++){

			int i = 2*i2 + 1;
			
			p_out_img[j2*ww2 + i2] = p_in_img[j*ww + i];
		}
	}
	
}


//********************************************************************************************
void LCKvYooji_Image_Processor::dih_Downsample_Image_Half(
	CKvMatrixUchar *in_img_gray,
	CKvMatrixUchar *out_img_downsampled)
//********************************************************************************************
{
	unsigned char *p_in_img, *p_out_img;
	int ww, hh, ww2, hh2, wwp, hhp;	

	// Initialization.
	p_in_img = in_img_gray->mps(ww, hh)[0];
	ww2 = ww/2;	hh2 = hh/2;

	// + create output image.
	if(out_img_downsampled->mw() != ww2 || out_img_downsampled->mh() != hh2)
		out_img_downsampled->c_Create(hh2, ww2, (unsigned char)0);
	p_out_img = out_img_downsampled->vp();

	for(int j2=0; j2<hh2; j2++){

		int j = 2*j2 + 1;

		for(int i2=0; i2<ww2; i2++){

			int i = 2*i2 + 1;
			
			p_out_img[j2*ww2 + i2] = p_in_img[j*ww + i];
		}
	}
	
}

//********************************************************************************************
void LCKvYooji_Image_Processor::ddh_Downsample_Depthmap_Half(
	CKvMatrixFloat *in_map_depth,
	CKvMatrixFloat *out_map_downsampled)
//********************************************************************************************
{
	float *p_in_map, *p_out_map;
	int ww, hh, ww2, hh2, wwp, hhp;	

	// Initialization.
	p_in_map = in_map_depth->mps(ww, hh)[0];
	ww2 = ww/2;	hh2 = hh/2;

	// + create output image.
	if(out_map_downsampled->mw() != ww2 || out_map_downsampled->mh() != hh2)
		out_map_downsampled->c_Create(hh2, ww2, 0.0f);
	p_out_map = out_map_downsampled->vp();

	for(int j2=0; j2<hh2; j2++){

		int j = 2*j2 + 1;

		for(int i2=0; i2<ww2; i2++){

			int i = 2*i2 + 1;
			
			p_out_map[j2*ww2 + i2] = p_in_map[j*ww + i];
		}
	}
	
}

//********************************************************************************************
void LCKvYooji_Image_Processor::dih_Downsample_Intrinsics_Half(
	CKvYooji_Intrinsics *in_intrinsics,
	CKvYooji_Intrinsics *out_intrinsics_downsampled)
//********************************************************************************************
{
	float fx, fy, px, py, dmin, dmax;
	
	in_intrinsics->get_params(fx, fy, px, py, dmin, dmax);
	
	out_intrinsics_downsampled->set_from_params(
		0.5f*fx, 0.5f*fy,
		0.5f*(px - 0.5f), 0.5f*(py - 0.5f),
		dmin, dmax);
}

//********************************************************************************************
// CAUTION: CURRENTLY, WE DO NOT CONSIDER BOUNDARY PADDING.
void LCKvYooji_Image_Processor::sig5_Smooth_Image_Gaussian_5x5(
	CKvMatrixUchar *in_img_rgb,
	CKvMatrixUchar *out_img_smooth,
	bool in_flag_boundary_padding,
	CKvMatrixInt *in_img_for_padding)
//********************************************************************************************
{
	CKvMatrixInt img_pad;
	static int filt1d[3] = {1, 4, 6};
// 	static int filter[5][5] = 
// 	{
// 		{1, 4, 6, 4, 1},
// 		{4, 16, 24, 16, 4},
// 		{6, 24, 36, 24, 6},
// 		{4, 16, 24, 16, 4},
// 		{1, 4, 6, 4, 1}
// 	};

	unsigned char *p_in_img, *p_out_img;
	int *p_pad_img;
	int ww, hh, wwp, hhp;	

	// Initialization.
	p_in_img = in_img_rgb->mps(ww, hh)[0];

	// + create output image.
	if(out_img_smooth->mw() != ww || out_img_smooth->mh() != hh)
		out_img_smooth->c_Create(hh, ww, (unsigned char)0);
	p_out_img = out_img_smooth->vp();

	// + check boundary padding flag.
	if(in_flag_boundary_padding){	wwp = ww + 1;	hhp = hh + 1;	}
	else{		wwp = ww;	hhp = hh;	}
	
	// + create temporary image.
	if(!in_img_for_padding)	p_pad_img = img_pad.c_Create(hhp, wwp)[0];
	else{
		
		if(wwp != in_img_for_padding->mw() || hhp != in_img_for_padding->mh()){			
			p_pad_img = in_img_for_padding->c_Create(hhp, wwp)[0];
		}
		else p_pad_img = in_img_for_padding->vp();
	}	

	

	// compute interpolated values of boundary pixels.
//	// + 4 corners (left top, right top, left bottom, right bottom).
// 	for(int m=-1; m<=2; m++){for(int k=-1; k<=2; k++){	;	}}	// LT
// 	for(int m=-1; m<=2; m++){for(int k=-2; k<=1; k++){	;	}}	// RT
// 	for(int m=-2; m<=1; m++){for(int k=-1; k<=2; k++){	;	}}	// LB
// 	for(int m=-2; m<=1; m++){for(int k=-2; k<=1; k++){	;	}}	// RB
// 	// + 4 edges.
// 	for(int j2=1; j2<hh2-1; j2++){
// 		for(int m=-2; m<=2; m++){for(int k=-1; k<=2; k++){	;	}}	// L
// 
// 		for(int m=-2; m<=2; m++){for(int k=-2; k<=1; k++){	;	}}	// R
// 	}
// 	for(int i2=1; i2<ww2-1; i2++){
// 		for(int m=-1; m<=2; m++){for(int k=-2; k<=2; k++){	;	}}	// T
// 
// 		for(int m=-2; m<=1; m++){for(int k=-2; k<=2; k++){	;	}}	// B
// 	}

	// compute interpolated values of pixels in boundary. 

	if(in_flag_boundary_padding)
	{

	}
	else{
		// perform horizontal filtering.
		// --->	---> ---> ---> --->
		// --->	---> ---> ---> --->
		for(int j=0; j<hh; j++){
			for(int i=2; i<ww-2; i++){
				p_pad_img[j*ww + i] = 
					filt1d[0]*(int)p_in_img[j*ww + (i-2)] + filt1d[0]*(int)p_in_img[j*ww + (i+2)]
				+	filt1d[1]*(int)p_in_img[j*ww + (i-1)] + filt1d[1]*(int)p_in_img[j*ww + (i+1)]
				+	filt1d[2]*(int)p_in_img[j*ww + i];

			}
		}

		// perform vertical filtering.
		// |  |  |  |  |
		// v  v  v  v  v
		for(int i = 2; i<ww-2; i++){
			for(int j = 2; j<hh-2; j++){
				p_out_img[j*ww + i] =
					(unsigned char)((filt1d[0]*(int)p_pad_img[(j-2)*ww + i] + filt1d[0]*(int)p_pad_img[(j+2)*ww + i]
				+	filt1d[1]*(int)p_pad_img[(j-1)*ww + i] + filt1d[1]*(int)p_pad_img[(j+1)*ww + i]
				+	filt1d[2]*(int)p_pad_img[j*ww + i])/(16*16));
			}
		}
	}

// 	for(int j2=1; j2<hh2-1; j2++){
// 
// 		int j = 2*j2 + 1;
// 
// 		for(int i2=1; i2<ww2-1; i2++){
// 
// 			int i = 2*i2 + 1;
// 			int tval = 0;
// 
// 			// Gaussian filtering.
// 			for(int m=-2; m<=2; m++){for(int k=-2; k<=2; k++){
// 				tval += filter[m][k]*(int)p_in_img[(j + m)*ww + (i + k)];
// 			}}
// 			
// 			p_out_img[j2*ww2 + i2] = (int)((tval + 0.5f)/16);
// 		}
// 	}


}

//********************************************************************************************
// CAUTION: CURRENTLY, WE DO NOT CONSIDER BOUNDARY PADDING.
void LCKvYooji_Image_Processor::sim5_Smooth_Image_Mean_5x5(
	CKvMatrixUchar *in_img_rgb,
	CKvMatrixUchar *out_img_smooth,
	bool in_flag_boundary_padding,
	CKvMatrixInt *in_img_for_padding)
//********************************************************************************************
{
	CKvMatrixInt img_pad;
	static int filt1d[3] = {1, 1, 1};
// 	static int filter[5][5] = 
// 	{
// 		{1, 4, 6, 4, 1},
// 		{4, 16, 24, 16, 4},
// 		{6, 24, 36, 24, 6},
// 		{4, 16, 24, 16, 4},
// 		{1, 4, 6, 4, 1}
// 	};

	unsigned char *p_in_img, *p_out_img;
	int *p_pad_img;
	int ww, hh, wwp, hhp;	

	// Initialization.
	p_in_img = in_img_rgb->mps(ww, hh)[0];

	// + create output image.
	if(out_img_smooth->mw() != ww || out_img_smooth->mh() != hh)
		out_img_smooth->c_Create(hh, ww, (unsigned char)0);
	p_out_img = out_img_smooth->vp();

	// + check boundary padding flag.
	if(in_flag_boundary_padding){	wwp = ww + 1;	hhp = hh + 1;	}
	else{		wwp = ww;	hhp = hh;	}
	
	// + create temporary image.
	if(!in_img_for_padding)	p_pad_img = img_pad.c_Create(hhp, wwp)[0];
	else{
		
		if(wwp != in_img_for_padding->mw() || hhp != in_img_for_padding->mh()){			
			p_pad_img = in_img_for_padding->c_Create(hhp, wwp)[0];
		}
		else p_pad_img = in_img_for_padding->vp();
	}	

	

	// compute interpolated values of boundary pixels.
//	// + 4 corners (left top, right top, left bottom, right bottom).
// 	for(int m=-1; m<=2; m++){for(int k=-1; k<=2; k++){	;	}}	// LT
// 	for(int m=-1; m<=2; m++){for(int k=-2; k<=1; k++){	;	}}	// RT
// 	for(int m=-2; m<=1; m++){for(int k=-1; k<=2; k++){	;	}}	// LB
// 	for(int m=-2; m<=1; m++){for(int k=-2; k<=1; k++){	;	}}	// RB
// 	// + 4 edges.
// 	for(int j2=1; j2<hh2-1; j2++){
// 		for(int m=-2; m<=2; m++){for(int k=-1; k<=2; k++){	;	}}	// L
// 
// 		for(int m=-2; m<=2; m++){for(int k=-2; k<=1; k++){	;	}}	// R
// 	}
// 	for(int i2=1; i2<ww2-1; i2++){
// 		for(int m=-1; m<=2; m++){for(int k=-2; k<=2; k++){	;	}}	// T
// 
// 		for(int m=-2; m<=1; m++){for(int k=-2; k<=2; k++){	;	}}	// B
// 	}

	// compute interpolated values of pixels in boundary. 

	if(in_flag_boundary_padding)
	{

	}
	else{
		// perform horizontal filtering.
		// --->	---> ---> ---> --->
		// --->	---> ---> ---> --->
		for(int j=0; j<hh; j++){
			for(int i=2; i<ww-2; i++){
				p_pad_img[j*ww + i] = 
					filt1d[0]*(int)p_in_img[j*ww + (i-2)] + filt1d[0]*(int)p_in_img[j*ww + (i+2)]
				+	filt1d[1]*(int)p_in_img[j*ww + (i-1)] + filt1d[1]*(int)p_in_img[j*ww + (i+1)]
				+	filt1d[2]*(int)p_in_img[j*ww + i];

			}
		}

		// perform vertical filtering.
		// |  |  |  |  |
		// v  v  v  v  v
		for(int i = 2; i<ww-2; i++){
			for(int j = 2; j<hh-2; j++){
				p_out_img[j*ww + i] =
					(unsigned char)((filt1d[0]*(int)p_pad_img[(j-2)*ww + i] + filt1d[0]*(int)p_pad_img[(j+2)*ww + i]
				+	filt1d[1]*(int)p_pad_img[(j-1)*ww + i] + filt1d[1]*(int)p_pad_img[(j+1)*ww + i]
				+	filt1d[2]*(int)p_pad_img[j*ww + i])/(5*5));
			}
		}
	}

// 	for(int j2=1; j2<hh2-1; j2++){
// 
// 		int j = 2*j2 + 1;
// 
// 		for(int i2=1; i2<ww2-1; i2++){
// 
// 			int i = 2*i2 + 1;
// 			int tval = 0;
// 
// 			// Gaussian filtering.
// 			for(int m=-2; m<=2; m++){for(int k=-2; k<=2; k++){
// 				tval += filter[m][k]*(int)p_in_img[(j + m)*ww + (i + k)];
// 			}}
// 			
// 			p_out_img[j2*ww2 + i2] = (int)((tval + 0.5f)/16);
// 		}
// 	}


}


//********************************************************************************************
// CAUTION: CURRENTLY, WE DO NOT CONSIDER BOUNDARY PADDING.
void LCKvYooji_Image_Processor::sdg5_Smooth_Depthmap_Gaussian_5x5(
	CKvMatrixFloat *in_map_depth,
	CKvMatrixFloat *out_map_smooth,
	bool in_flag_boundary_padding,
	CKvMatrixFloat *in_map_for_padding)
//********************************************************************************************
{
	CKvMatrixFloat map_pad;
	static float filt1d[3] = {1.0f, 4.0f, 6.0f};

	float *p_in_map, *p_out_map;
	float *p_pad_map;
	int ww, hh, wwp, hhp;	

	// Initialization.
	p_in_map = in_map_depth->mps(ww, hh)[0];

	// + create output image.
	if(out_map_smooth->mw() != ww || out_map_smooth->mh() != hh)
		out_map_smooth->c_Create(hh, ww, 0.0f);
	p_out_map = out_map_smooth->vp();

	// + check boundary padding flag.
	if(in_flag_boundary_padding){ wwp = ww + 1;	hhp = hh + 1; }
	else{ wwp = ww;	hhp = hh; }
	
	// + create temporary image.
	if(!in_map_for_padding)	p_pad_map = map_pad.c_Create(hhp, wwp)[0];
	else{		
		if(wwp != in_map_for_padding->mw() || hhp != in_map_for_padding->mh()){			
			p_pad_map = map_pad.c_Create(hhp, wwp)[0];
		}
		else p_pad_map = in_map_for_padding->vp();
	}		
	
	if(in_flag_boundary_padding)
	{

	}
	else{

		// perform horizontal filtering.
		// --->	---> ---> ---> --->
		// --->	---> ---> ---> --->
		for(int j=0; j<hh; j++){
			for(int i=2; i<ww-2; i++){

// 				if(p_in_map[j*ww + (i-2)] <= 0.0f)	continue;
// 				if(p_in_map[j*ww + (i-1)] <= 0.0f)	continue;
// 				if(p_in_map[j*ww + (i  )] <= 0.0f)	continue;
// 				if(p_in_map[j*ww + (i+1)] <= 0.0f)	continue;
// 				if(p_in_map[j*ww + (i+2)] <= 0.0f)	continue;

				p_pad_map[j*ww + i] = 
					filt1d[0]*p_in_map[j*ww + (i-2)] + filt1d[0]*p_in_map[j*ww + (i+2)]
				+	filt1d[1]*p_in_map[j*ww + (i-1)] + filt1d[1]*p_in_map[j*ww + (i+1)]
				+	filt1d[2]*p_in_map[j*ww + i];

			}
		}

		// perform vertical filtering.
		// |  |  |  |  |
		// v  v  v  v  v
		for(int i = 2; i<ww-2; i++){
			for(int j = 2; j<hh-2; j++){

// 				if(p_pad_map[(j-2)*ww + i] <= 0.0f)	continue;
// 				if(p_pad_map[(j-1)*ww + i] <= 0.0f)	continue;
// 				if(p_pad_map[(j  )*ww + i] <= 0.0f)	continue;
// 				if(p_pad_map[(j+1)*ww + i] <= 0.0f)	continue;
// 				if(p_pad_map[(j+2)*ww + i] <= 0.0f)	continue;

				p_out_map[j*ww + i] =
					((filt1d[0]*p_pad_map[(j-2)*ww + i] + filt1d[0]*p_pad_map[(j+2)*ww + i]
				+	filt1d[1]*p_pad_map[(j-1)*ww + i] + filt1d[1]*p_pad_map[(j+1)*ww + i]
				+	filt1d[2]*p_pad_map[j*ww + i])/(16.0f*16.0f));
			}
		}

	}



}