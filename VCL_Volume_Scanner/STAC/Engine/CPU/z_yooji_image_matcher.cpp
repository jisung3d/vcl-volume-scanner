//#pragma once
//#include "stdafx.h"
//#include "JM_Contour_Detection.h"

#include "../../../stdafx.h"
#include "../../_yooji_2016_object_scanning.h"

//********************************************************************************
LCKvImageMatcher:: LCKvImageMatcher()
//********************************************************************************
{
	zz_max_feat_num = 0;
	zz_match_buff = NULL;
// 	zz_max_feat_num=2000;
// 	// VLFeat.
// // 	for(int i=0; i<2; i++){
// // 		zz_DATAdescr[i]=new vl_uint8[128*zz_max_feat_num];
// // 		zz_DATAframes[i]=new double[4*zz_max_feat_num];
// // 		zz_nframe[i]=0;
// // 	}
// // 	zz_p_matches=new double[2*zz_max_feat_num];
// 
// 	//////////////////////////////////////////////////////////////////////////
// 	// SIFTGPU.
//  	vKeysS.resize(zz_max_feat_num);			vKeysD.resize(zz_max_feat_num);
//  	vDescriptorS.resize(128*zz_max_feat_num);	vDescriptorD.resize(128*zz_max_feat_num);
//  	match_buff = new int[zz_max_feat_num][2];
//  
//  	///
//  	// GPUSIFT INIT
//  	zz_Sift = CreateNewSiftGPU();
//  	zz_SiftMat = CreateNewSiftMatchGPU();//zz_SiftMat = new SiftMatchGPU[4096];
//  
//  	char * argv[] = {"-fo", "-1", "-v", "1"};
//  	//char * argv[] = {"-m", "-v", "1"};
//  	//-fo -1	use -1 octave 
//  	//-m,		up to 2 orientations for each feature
//  	//-s		enable subpixel subscale
//  	//-v 1		will invoke calling SiftGPU::SetVerbose(1),(only print out # feature and overall time)
//  	//-loweo	add a (.5, .5) offset
//  
//  	int argc = sizeof(argv)/sizeof(char*);
//  	zz_Sift->ParseParam(argc, argv);
//  
//  	//create an OpenGL context for computation
// 	//int support = zz_Sift->VerifyContextGL();
// 	int support = zz_Sift->CreateContextGL();
// 
//  	if(support != SiftGPU::SIFTGPU_FULL_SUPPORTED) return ;
//  	//run sift on a set of images
// 	zz_SiftMat->VerifyContextGL();

}

//********************************************************************************
LCKvImageMatcher:: ~LCKvImageMatcher()
//********************************************************************************
{
// 	for(int i=0; i<2; i++){
// 		delete[] zz_DATAdescr[i];
// 		delete[] zz_DATAframes[i];		
// 	}
// 	delete[] zz_p_matches;

	zz_vKeysS.clear();			zz_vKeysD.clear();
	printf("1\n");
	zz_vDescriptorS.clear();	zz_vDescriptorD.clear();
	printf("2\n");
	if(!zz_match_buff) delete[] zz_match_buff;
	printf("3\n");
	///
// 	delete[] zz_Sift;
// 	delete[] zz_SiftMat;
}

//********************************************************************************
void LCKvImageMatcher::i_Initialize(int in_max_num_features)
//********************************************************************************
{
	zz_max_feat_num = in_max_num_features;
	// VLFeat.
// 	for(int i=0; i<2; i++){
// 		zz_DATAdescr[i]=new vl_uint8[128*zz_max_feat_num];
// 		zz_DATAframes[i]=new double[4*zz_max_feat_num];
// 		zz_nframe[i]=0;
// 	}
// 	zz_p_matches=new double[2*zz_max_feat_num];

	//////////////////////////////////////////////////////////////////////////
	// SIFTGPU.
	zz_vKeysS.clear();			zz_vKeysD.clear();
	zz_vDescriptorS.clear();	zz_vDescriptorD.clear();
	if(zz_match_buff) delete[] zz_match_buff;

//  	zz_vKeysS.resize(zz_max_feat_num);			zz_vKeysD.resize(zz_max_feat_num);
//  	zz_vDescriptorS.resize(128*zz_max_feat_num);	zz_vDescriptorD.resize(128*zz_max_feat_num);

	zz_vKeysS.reserve(zz_max_feat_num);			zz_vKeysD.reserve(zz_max_feat_num);
	zz_vDescriptorS.reserve(128*zz_max_feat_num);	zz_vDescriptorD.reserve(128*zz_max_feat_num);
 	zz_match_buff = new int[zz_max_feat_num][2];
 
 	///
 	// GPUSIFT INIT
 	zz_Sift = CreateNewSiftGPU();
 	zz_SiftMat = CreateNewSiftMatchGPU();//zz_SiftMat = new SiftMatchGPU[4096];
 
 	char * argv[] = {"-fo", "-1", "-v", "1"};
 	//char * argv[] = {"-m", "-v", "1"};
 	//-fo -1	use -1 octave 
 	//-m,		up to 2 orientations for each feature
 	//-s		enable subpixel subscale
 	//-v 1		will invoke calling SiftGPU::SetVerbose(1),(only print out # feature and overall time)
 	//-loweo	add a (.5, .5) offset
 
 	int argc = sizeof(argv)/sizeof(char*);
 	zz_Sift->ParseParam(argc, argv);
 
 	//create an OpenGL context for computation
	//int support = zz_Sift->VerifyContextGL();
	int support = zz_Sift->CreateContextGL();

 	if(support != SiftGPU::SIFTGPU_FULL_SUPPORTED) return ;
 	//run sift on a set of images
	zz_SiftMat->VerifyContextGL();

}


//********************************************************************************
void LCKvImageMatcher::emsf_Extract_and_Match_SIFT_Features(CKvMatrixUchar *in_src_image,
											 CKvMatrixUchar *in_dst_image,
											 int &out_num_matched,
											 float *out_matched_src_points,
											 float *out_matched_dst_points)
//********************************************************************************
{
	/// extract SIFT features.	
	zz_nframe[0]=zz_nframe[1]=zz_matched_num=0;
	// extract SIFT features.
	esf_Extract_SIFT_Features_VLFeat(*in_src_image, zz_DATAdescr[0], zz_DATAframes[0], zz_nframe[0]);
	esf_Extract_SIFT_Features_VLFeat(*in_dst_image, zz_DATAdescr[1], zz_DATAframes[1], zz_nframe[1]);
	// match extracted SIFT features.
	zz_matched_num=0;
	if(zz_nframe[0]>0 && zz_nframe[1]>0){		
		mesf_Match_Extracted_SIFT_Features_VLFeat(zz_DATAdescr[0], zz_DATAdescr[1], zz_nframe[0], zz_nframe[1], 
			4.0f, zz_matched_num, zz_p_matches);
	}
	// update output.
	out_num_matched=zz_matched_num;
	for(int i=0; i<out_num_matched; i++){
		out_matched_src_points[2*i]=(float)zz_DATAframes[0][(int)zz_p_matches[2*i]*4];			// src_x
		out_matched_src_points[2*i+1]=(float)zz_DATAframes[0][(int)zz_p_matches[2*i]*4+1];		// scr_y

		out_matched_dst_points[2*i]=(float)zz_DATAframes[1][(int)zz_p_matches[2*i+1]*4];			// dst_x
		out_matched_dst_points[2*i+1]=(float)zz_DATAframes[1][(int)zz_p_matches[2*i+1]*4+1];		// dst_y
	}

}

//********************************************************************************
void LCKvImageMatcher::emsf_Extract_and_Match_SIFT_Features(CKvMatrixUchar *in_src_image,
											 CKvMatrixUchar *in_dst_image,
											 int &out_num_matched,
											 Vector2f *out_matched_src_points,
											 Vector2f *out_matched_dst_points)
//********************************************************************************
{
	/// extract SIFT features.	
	zz_nframe[0]=zz_nframe[1]=zz_matched_num=0;
	// extract SIFT features.
	esf_Extract_SIFT_Features_VLFeat(*in_src_image,zz_DATAdescr[0],zz_DATAframes[0],zz_nframe[0]);
	esf_Extract_SIFT_Features_VLFeat(*in_dst_image,zz_DATAdescr[1],zz_DATAframes[1],zz_nframe[1]);
	// match extracted SIFT features.
	zz_matched_num=0;
	if(zz_nframe[0]>0 && zz_nframe[1]>0){		
		mesf_Match_Extracted_SIFT_Features_VLFeat(zz_DATAdescr[0], zz_DATAdescr[1], zz_nframe[0], zz_nframe[1], 
			4.0f, zz_matched_num, zz_p_matches);
	}
	// update output.
	out_num_matched=zz_matched_num;
	for(int i=0; i<out_num_matched; i++){
		out_matched_src_points[i].x=(float)zz_DATAframes[0][(int)zz_p_matches[2*i]*4];			// src_x
		out_matched_src_points[i].y=(float)zz_DATAframes[0][(int)zz_p_matches[2*i]*4+1];		// scr_y

		out_matched_dst_points[i].x=(float)zz_DATAframes[1][(int)zz_p_matches[2*i+1]*4];			// dst_x
		out_matched_dst_points[i].y=(float)zz_DATAframes[1][(int)zz_p_matches[2*i+1]*4+1];		// dst_y
	}

}


//********************************************************************************
void LCKvImageMatcher::emsf_Extract_and_Match_SIFT_Features(CKvMatrixUchar *in_src_image,
											 CKvMatrixUchar *in_dst_image,
											 int &out_num_matched,
											 vector<Vector2f> &out_matched_src_points,
										  	 vector<Vector2f> &out_matched_dst_points)
//********************************************************************************
{
	/// extract SIFT features.	
	zz_nframe[0]=zz_nframe[1]=zz_matched_num=0;
	// extract SIFT features.
	esf_Extract_SIFT_Features_VLFeat(*in_src_image,zz_DATAdescr[0],zz_DATAframes[0],zz_nframe[0]);
	esf_Extract_SIFT_Features_VLFeat(*in_dst_image,zz_DATAdescr[1],zz_DATAframes[1],zz_nframe[1]);
	// match extracted SIFT features.
	zz_matched_num=0;
	if(zz_nframe[0]>0 && zz_nframe[1]>0){		
		mesf_Match_Extracted_SIFT_Features_VLFeat(zz_DATAdescr[0], zz_DATAdescr[1], zz_nframe[0], zz_nframe[1], 
			4.0f, zz_matched_num, zz_p_matches);
	}
	// update output.
	out_num_matched=zz_matched_num;
	out_matched_src_points.resize(out_num_matched);
	out_matched_dst_points.resize(out_num_matched);
	for(int i=0; i<out_num_matched; i++){
		out_matched_src_points[i].x=(float)zz_DATAframes[0][(int)zz_p_matches[2*i]*4];			// src_x
		out_matched_src_points[i].y=(float)zz_DATAframes[0][(int)zz_p_matches[2*i]*4+1];		// scr_y

		out_matched_dst_points[i].x=(float)zz_DATAframes[1][(int)zz_p_matches[2*i+1]*4];			// dst_x
		out_matched_dst_points[i].y=(float)zz_DATAframes[1][(int)zz_p_matches[2*i+1]*4+1];		// dst_y
	}

}

//********************************************************************************
void LCKvImageMatcher::emsf_Extract_and_Match_SIFT_Features(CKvMatrixUcharRgb *in_src_image,
	CKvMatrixUcharRgb *in_dst_image,
	int &out_num_matched,
	float *out_matched_src_points,
	float *out_matched_dst_points)
//********************************************************************************
{
	/// extract SIFT features.	
	zz_nframe[0]=zz_nframe[1]=zz_matched_num=0;
	aa_ycc.mrg_Matrix_Rgb_to_Gray(in_src_image, false, &zz_img_g_src);
	aa_ycc.mrg_Matrix_Rgb_to_Gray(in_dst_image, false, &zz_img_g_dst);
	// extract SIFT features.
	esf_Extract_SIFT_Features_VLFeat(zz_img_g_src, zz_DATAdescr[0], zz_DATAframes[0], zz_nframe[0]);
	esf_Extract_SIFT_Features_VLFeat(zz_img_g_dst, zz_DATAdescr[1], zz_DATAframes[1], zz_nframe[1]);
	// match extracted SIFT features.
	zz_matched_num=0;
	if(zz_nframe[0]>0 && zz_nframe[1]>0){		
		mesf_Match_Extracted_SIFT_Features_VLFeat(zz_DATAdescr[0], zz_DATAdescr[1], zz_nframe[0], zz_nframe[1], 
			4.0f, zz_matched_num, zz_p_matches);
	}
	// update output.
	out_num_matched=zz_matched_num;
	for(int i=0; i<out_num_matched; i++){
		out_matched_src_points[2*i]=(float)zz_DATAframes[0][(int)zz_p_matches[2*i]*4];			// src_x
		out_matched_src_points[2*i+1]=(float)zz_DATAframes[0][(int)zz_p_matches[2*i]*4+1];		// scr_y

		out_matched_dst_points[2*i]=(float)zz_DATAframes[1][(int)zz_p_matches[2*i+1]*4];			// dst_x
		out_matched_dst_points[2*i+1]=(float)zz_DATAframes[1][(int)zz_p_matches[2*i+1]*4+1];		// dst_y
	}

}

//********************************************************************************
bool LCKvImageMatcher::emf_Extract_and_Match_Features_using_SiftGPU(
	CKvMatrixUchar &in_src_image,
	CKvMatrixUchar &in_dst_image,
	vector<Point2f> &out_p2d_matched_src,
	vector<Point2f> &out_p2d_matched_dst,
	CKvMatrixFloat *in_map_depth_src,
	CKvMatrixFloat *in_map_depth_dst)
//********************************************************************************
{
	/// extract SIFT features.	
	uchar *p_src, *p_dst;
	int ww_s, hh_s, ww_d, hh_d;
	zz_nframe[0]=zz_nframe[1]=zz_matched_num=0;

	p_src=in_src_image.mps(ww_s, hh_s)[0];
	p_dst=in_dst_image.mps(ww_d, hh_d)[0];

	// SIFT matcher setting.
	int numFeatS, numFeatD, numFeatMax, matPairNum;
	float temp_dist;

	printf("Image size! %d %d %d %d", ww_s, hh_s, ww_d, hh_d);

	// extract SIFT features.
	if(zz_Sift->RunSIFT(ww_s, hh_s, p_src, GL_LUMINANCE, GL_UNSIGNED_BYTE)){
		numFeatS = zz_Sift->GetFeatureNum();
		//printf("numFeatS: %d\n",numFeatS);

		//vKeysS.resize(numFeatS);							vDescriptorS.resize(128*numFeatS);
		zz_Sift->GetFeatureVector(&zz_vKeysS[0], &zz_vDescriptorS[0]);
	}
	else{
		Kv_Printf("[LCKvImageMatcher::emsfg_Extract_and_Match_SIFT_Features_on_GPU]\n SIFT feature extraction of source image was failed.");
		system("pause");
		return false;
	}
	
	if(zz_Sift->RunSIFT(ww_d, hh_d, p_dst, GL_LUMINANCE, GL_UNSIGNED_BYTE)){
		numFeatD = zz_Sift->GetFeatureNum();
		//vKeysD.resize(numFeatD);							vDescriptorD.resize(128*numFeatD);
		zz_Sift->GetFeatureVector(&zz_vKeysD[0], &zz_vDescriptorD[0]);		
	}
	else{
		Kv_Printf("[LCKvImageMatcher::emsfg_Extract_and_Match_SIFT_Features_on_GPU]\n SIFT feature extraction of destination image was failed.");
		system("pause");
		return false;
	}
	// match SIFT features.
	numFeatMax = min(numFeatS, numFeatD);
	
	zz_SiftMat->SetDescriptors(0, numFeatS, &zz_vDescriptorS[0]);
	zz_SiftMat->SetDescriptors(1, numFeatD, &zz_vDescriptorD[0]);

	//vDescriptorS.clear();
	//vDescriptorD.clear();

	//matPairNum = zz_SiftMat->GetSiftMatch(numFeatMax, 
	//	match_buff, 
	//	in_maximum_distance,			// default value: 0.7
	//	in_maximum_distance_ratio,		// default value: 0.8
	//	1);
	matPairNum = zz_SiftMat->GetSiftMatch(numFeatMax, zz_match_buff);

	// update output.
	//out_p2d_matched_src.resize(matPairNum);		out_p2d_matched_dst.resize(matPairNum);
	Point2f tp2d_src, tp2d_dst;	Vector2f tp2d_v;	
	int ww, hh;	float td;

	if(in_map_depth_src) in_map_depth_src->ms(ww, hh);

	out_p2d_matched_src.clear(); out_p2d_matched_dst.clear();
	for(int i=0; i<matPairNum; i++){
// 		Point2f &p2d_src = out_p2d_matched_src[i];
// 		Point2f &p2d_dst = out_p2d_matched_dst[i];

		tp2d_src.x = zz_vKeysS[zz_match_buff[i][0]].x;		// src_x
		tp2d_src.y = zz_vKeysS[zz_match_buff[i][0]].y;		// scr_y
		
		tp2d_dst.x = zz_vKeysD[zz_match_buff[i][1]].x;		// dst_x
		tp2d_dst.y = zz_vKeysD[zz_match_buff[i][1]].y;		// dst_y

		// check depth validity of source points.
		if(in_map_depth_src){
			tp2d_v.x = tp2d_src.x;		// src_x
			tp2d_v.y = tp2d_src.y;		// scr_y

			if(!d_gid_Get_Interpolated_Depth(tp2d_v, ww, hh, (const float*)in_map_depth_src->vp(), td)) continue;
		}

		if(in_map_depth_dst){
			tp2d_v.x = tp2d_dst.x;		// src_x
			tp2d_v.y = tp2d_dst.y;		// scr_y

			if(!d_gid_Get_Interpolated_Depth(tp2d_v,ww,hh,(const float*)in_map_depth_dst->vp(),td)) continue;
		}

		// append valid matched points.
		out_p2d_matched_src.push_back(tp2d_src);
		out_p2d_matched_dst.push_back(tp2d_dst);
	}

	return true;

}

//********************************************************************************
bool LCKvImageMatcher::emf_Extract_and_Match_Features_using_SiftGPU(
	CKvMatrixUchar &in_src_image,
	CKvMatrixUchar &in_dst_image,
	int &out_num_matched,
	float *out_matched_src_points,
	float *out_matched_dst_points)
//********************************************************************************
{
	/// extract SIFT features.	
	uchar *p_src, *p_dst;
	int ww_s, hh_s, ww_d, hh_d;
	zz_nframe[0]=zz_nframe[1]=zz_matched_num=0;

	p_src=in_src_image.mps(ww_s, hh_s)[0];
	p_dst=in_dst_image.mps(ww_d, hh_d)[0];

	// SIFT matcher setting.
	int numFeatS, numFeatD, numFeatMax, matPairNum;
	float temp_dist;

	//if(!Kv_Printf("Image size! %d %d %d %d", ww_s, hh_s, ww_d, hh_d)) exit(0);

	// extract SIFT features.
	if(zz_Sift->RunSIFT(ww_s, hh_s, p_src, GL_LUMINANCE, GL_UNSIGNED_BYTE)){
		numFeatS = zz_Sift->GetFeatureNum();
		printf("numFeatS: %d\n",numFeatS);

		//vKeysS.resize(numFeatS);							vDescriptorS.resize(128*numFeatS);
		zz_Sift->GetFeatureVector(&zz_vKeysS[0], &zz_vDescriptorS[0]);
	}
	else{
		Kv_Printf("[LCKvImageMatcher::emsfg_Extract_and_Match_SIFT_Features_on_GPU]\n SIFT feature extraction of source image was failed.");
		system("pause");
		return false;
	}
	
	if(zz_Sift->RunSIFT(ww_d, hh_d, p_dst, GL_LUMINANCE, GL_UNSIGNED_BYTE)){
		numFeatD = zz_Sift->GetFeatureNum();
		//vKeysD.resize(numFeatD);							vDescriptorD.resize(128*numFeatD);
		zz_Sift->GetFeatureVector(&zz_vKeysD[0], &zz_vDescriptorD[0]);		
	}
	else{
		Kv_Printf("[LCKvImageMatcher::emsfg_Extract_and_Match_SIFT_Features_on_GPU]\n SIFT feature extraction of destination image was failed.");
		system("pause");
		return false;
	}
	// match SIFT features.
	numFeatMax = min(numFeatS, numFeatD);
	
	zz_SiftMat->SetDescriptors(0, numFeatS, &zz_vDescriptorS[0]);
	zz_SiftMat->SetDescriptors(1, numFeatD, &zz_vDescriptorD[0]);

	//vDescriptorS.clear();
	//vDescriptorD.clear();

	//matPairNum = zz_SiftMat->GetSiftMatch(numFeatMax, 
	//	match_buff, 
	//	in_maximum_distance,			// default value: 0.7
	//	in_maximum_distance_ratio,		// default value: 0.8
	//	1);
	matPairNum = zz_SiftMat->GetSiftMatch(numFeatMax, zz_match_buff);


	printf("Matched Num: %d (%d %d)\n", matPairNum, numFeatS, numFeatD);


	// update output.
	out_num_matched=matPairNum;
	for(int i=0; i<out_num_matched; i++){
		out_matched_src_points[2*i]=zz_vKeysS[zz_match_buff[i][0]].x;			// src_x
		out_matched_src_points[2*i+1]=zz_vKeysS[zz_match_buff[i][0]].y;		// scr_y

		out_matched_dst_points[2*i]=zz_vKeysD[zz_match_buff[i][1]].x;			// dst_x
		out_matched_dst_points[2*i+1]=zz_vKeysD[zz_match_buff[i][1]].y;		// dst_y
	}

	return true;

}

/// //////////////////////////////////////////////////////////////////////
/** ------------------------------------------------------------------
 ** @internal
 ** @brief Transpose desriptor
 **
 ** @param dst destination buffer.
 ** @param src source buffer.
 **
 ** The function writes to @a dst the transpose of the SIFT descriptor
 ** @a src. The tranpsose is defined as the descriptor that one
 ** obtains from computing the normal descriptor on the transposed
 ** image.
 **/

VL_INLINE void
transpose_descriptor (vl_sift_pix* dst, vl_sift_pix* src)
{
  int const BO = 8 ;  /* number of orientation bins */
  int const BP = 4 ;  /* number of spatial bins     */
  int i, j, t ;

  for (j = 0 ; j < BP ; ++j) {
    int jp = BP - 1 - j ;
    for (i = 0 ; i < BP ; ++i) {
      int o  = BO * i + BP*BO * j  ;
      int op = BO * i + BP*BO * jp ;
      dst [op] = src[o] ;
      for (t = 1 ; t < BO ; ++t)
        dst [BO - t + op] = src [t + o] ;
    }
  }
}

/** ------------------------------------------------------------------
 ** @internal
 ** @brief Ordering of tuples by increasing scale
 **
 ** @param a tuple.
 ** @param b tuple.
 **
 ** @return @c a[2] < b[2]
 **/

static int
korder (void const* a, void const* b) {
  double x = ((double*) a) [2] - ((double*) b) [2] ;
  if (x < 0) return -1 ;
  if (x > 0) return +1 ;
  return 0 ;
}

/** ------------------------------------------------------------------
 ** @internal
 ** @brief Check for sorted keypoints
 **
 ** @param keys keypoint list to check
 ** @param nkeys size of the list.
 **
 ** @return 1 if the keypoints are storted.
 **/

vl_bool
check_sorted (double const * keys, vl_size nkeys)
{
  vl_uindex k ;
  for (k = 0 ; k + 1 < nkeys ; ++ k) {
    if (korder(keys, keys + 4) > 0) {
      return VL_FALSE ;
    }
    keys += 4 ;
  }
  return VL_TRUE ;
}


void LCKvImageMatcher::esf_Extract_SIFT_Features_VLFeat(
	CKvMatrixUchar &in_image, 	
	vl_uint8 *DATAdescr,
	double *DATAframes,
	int &nframes)
{
  enum {IN_I=0,IN_END} ;
  enum {OUT_FRAMES=0, OUT_DESCRIPTORS} ;

  int                verbose = 0 ;
  int                opt ;
  int                next = IN_END ;

  vl_uint8 *p_in_image;
  vl_sift_pix const *data ;
  int                M, N ;

  /// ********************************************************************************* ///
  // Default values of parameters.
  // O       = VL_MAX (floor (log2 (VL_MIN(width, height))) - o_min - 3, 1) ;

  // sigman  = 0.5 ;
  // sigmak  = pow (2.0, 1.0 / nlevels) ;
  // sigma0  = 1.6 * f->sigmak ;
  // dsigma0 = f->sigma0 * sqrt (1.0 - 1.0 / (f->sigmak*f->sigmak)) ;

  // peak_thresh = 0.0 ;
  // edge_thresh = 10.0 ;
  // norm_thresh = 0.0 ;
  // magnif      = 3.0 ;
  // windowSize  = NBP / 2 ;
  int                O     = - 1 ;
  int                S     =   3 ;
  int                o_min =   0 ;

  double             edge_thresh = -1 ;
  double             peak_thresh = -1 ;
  double             norm_thresh = -1 ;
  double             magnif      = -1 ;
  double             window_size = -1 ;
  /// ********************************************************************************* ///
  
  float *frame;
  double            *ikeys = 0 ;
  int                nikeys = -1 ;
  vl_bool            force_orientations = 0 ;
  vl_bool            floatDescriptors = 0 ;

  p_in_image=in_image.mps(N, M)[0];
  if(zz_frame_float.mw()!=N || zz_frame_float.mh()!=M)	zz_frame_float.c_Create(M, N, 0.0f);
  frame=zz_frame_float.vp();

  //for(int j=0; j<M; j++){
	 // for(int i=0; i<N; i++){
		//  frame[j*N+i]=(float)p_in_image[j*N+i];
	 // }
  //}
  // transpose input image for matlab.
  for(int j=0; j<N; j++){
	  for(int i=0; i<M; i++){
		  frame[j*M+i]=(float)p_in_image[i*N+j];
	  }
  }

  data = (vl_sift_pix*)frame;

/* -----------------------------------------------------------------
   *                                                            Do job
   * -------------------------------------------------------------- */
  {
    VlSiftFilt        *filt ;
    vl_bool            first ;
    double            *frames = 0 ;
    void              *descr  = 0 ;
    int                reserved = 0, i,j,q ;

    /* create a filter to process the image */
    filt = vl_sift_new (M, N, O, S, o_min) ;

    if (peak_thresh >= 0) vl_sift_set_peak_thresh (filt, peak_thresh) ;
    if (edge_thresh >= 0) vl_sift_set_edge_thresh (filt, edge_thresh) ;
    if (norm_thresh >= 0) vl_sift_set_norm_thresh (filt, norm_thresh) ;
    if (magnif      >= 0) vl_sift_set_magnif      (filt, magnif) ;
    if (window_size >= 0) vl_sift_set_window_size (filt, window_size) ;

    if (verbose) {
      printf("vl_sift: filter settings:\n") ;
      printf("vl_sift:   octaves      (O)      = %d\n",
                vl_sift_get_noctaves      (filt)) ;
      printf("vl_sift:   levels       (S)      = %d\n",
                vl_sift_get_nlevels       (filt)) ;
      printf("vl_sift:   first octave (o_min)  = %d\n",
                vl_sift_get_octave_first  (filt)) ;
      printf("vl_sift:   edge thresh           = %g\n",
                vl_sift_get_edge_thresh   (filt)) ;
      printf("vl_sift:   peak thresh           = %g\n",
                vl_sift_get_peak_thresh   (filt)) ;
      printf("vl_sift:   norm thresh           = %g\n",
                vl_sift_get_norm_thresh   (filt)) ;
      printf("vl_sift:   window size           = %g\n",
                vl_sift_get_window_size   (filt)) ;
      printf("vl_sift:   float descriptor      = %d\n",
                floatDescriptors) ;

      printf((nikeys >= 0) ?
                "vl_sift: will source frames? yes (%d read)\n" :
                "vl_sift: will source frames? no\n", nikeys) ;
      printf("vl_sift: will force orientations? %s\n",
                force_orientations ? "yes" : "no") ;
    }

    /* ...............................................................
     *                                             Process each octave
     * ............................................................ */
    i     = 0 ;
    first = 1 ;
    while (1) {
      int                   err ;
      VlSiftKeypoint const *keys  = 0 ;
      int                   nkeys = 0 ;

      if (verbose) {
        printf ("vl_sift: processing octave %d\n",
                   vl_sift_get_octave_index (filt)) ;
      }

      /* Calculate the GSS for the next octave .................... */
      if (first) {
        err   = vl_sift_process_first_octave (filt, data) ;
        first = 0 ;
      } else {
        err   = vl_sift_process_next_octave  (filt) ;
      }

      if (err) break ;

      if (verbose > 1) {
        printf("vl_sift: GSS octave %d computed\n",
                  vl_sift_get_octave_index (filt));
      }

      /* Run detector ............................................. */
      if (nikeys < 0) {
        vl_sift_detect (filt) ;

        keys  = vl_sift_get_keypoints  (filt) ;
        nkeys = vl_sift_get_nkeypoints (filt) ;
        i     = 0 ;

        if (verbose > 1) {
          printf ("vl_sift: detected %d (unoriented) keypoints\n", nkeys) ;
        }
      } else {
        nkeys = nikeys ;
      }

      /* For each keypoint ........................................ */
      for (; i < nkeys ; ++i) {
        double                angles [4] ;
        int                   nangles ;
        VlSiftKeypoint        ik ;
        VlSiftKeypoint const *k ;

        /* Obtain keypoint orientations ........................... */
        if (nikeys >= 0) {
          vl_sift_keypoint_init (filt, &ik,
                                 ikeys [4 * i + 1] - 1,
                                 ikeys [4 * i + 0] - 1,
                                 ikeys [4 * i + 2]) ;

          if (ik.o != vl_sift_get_octave_index (filt)) {
            break ;
          }

          k = &ik ;

          /* optionally compute orientations too */
          if (force_orientations) {
            nangles = vl_sift_calc_keypoint_orientations
              (filt, angles, k) ;
          } else {
            angles [0] = PI / 2 - ikeys [4 * i + 3] ;
            nangles    = 1 ;
          }
        } else {
          k = keys + i ;
          nangles = vl_sift_calc_keypoint_orientations
            (filt, angles, k) ;
        }

        /* For each orientation ................................... */
        for (q = 0 ; q < nangles ; ++q) {
          vl_sift_pix  buf [128] ;
          vl_sift_pix rbuf [128] ;

          /* compute descriptor (if necessary) */
		  vl_sift_calc_keypoint_descriptor (filt, buf, k, angles [q]) ;
		  transpose_descriptor (rbuf, buf) ;


          /* make enough room for all these keypoints and more */
          if (reserved < nframes + 1) {
            reserved += 2 * nkeys ;
            frames = (double*)realloc (frames, 4 * sizeof(double) * reserved) ;
            descr  = (vl_uint8*)realloc (descr,  128 * sizeof(vl_uint8) * reserved) ;
          }

          /* Save back with MATLAB conventions. Notice that the input
           * image was the transpose of the actual image. */
          frames [4 * nframes + 0] = k -> y + 1 ;
          frames [4 * nframes + 1] = k -> x + 1 ;
          frames [4 * nframes + 2] = k -> sigma ;
          frames [4 * nframes + 3] = PI / 2 - angles [q] ;
		  
		  for (j = 0 ; j < 128 ; ++j) {
			  float x = 512.0F * rbuf [j] ;
			  x = (x < 255.0F) ? x : 255.0F ;
			  ((vl_uint8*)descr) [128 * nframes + j] = (vl_uint8) x ;
		  }
		  
          ++ nframes ;
        } /* next orientation */
      } /* next keypoint */
    } /* next octave */

    if (verbose) {
      printf ("vl_sift: found %d keypoints\n", nframes) ;
    }

	// save variables:
	memcpy(DATAframes, frames, 4 * (nframes ) * sizeof(double));
	memcpy(DATAdescr, descr, 128 * (nframes ) * sizeof(vl_uint8));

	/* cleanup */

	vl_sift_delete (filt) ;

	} /* end: do job */
}

void LCKvImageMatcher::mesf_Match_Extracted_SIFT_Features_VLFeat(
	vl_uint8* L1_pt,
	vl_uint8* L2_pt, 
	int KK1, 
	int KK2, 
	double thresh, 
	int &nMatches, 
	double* MATCHES){

	//Match descriptors!
	int ND = 128;
	
	Pair* pairs_begin = (Pair*) malloc(sizeof(Pair) * (KK1+KK2));
	Pair* pairs_iterator = pairs_begin ;

	int k1, k2 ;
	const int maxval = 0x7fffffff ;  

	for(k1 = 0 ; k1 < KK1 ; ++k1, L1_pt += ND) {    //kalooo!  
		
		int best = maxval;
		int second_best = maxval;
		int bestk = -1;

		/* For each point P2[k2] in the second image... */
		for(k2 =  0 ; k2 < KK2 ; ++k2, L2_pt += ND) {                     
			int bin;
			int acc = 0;
			for(bin = 0 ; bin < ND ; ++bin){
				int delta =  ((int) L1_pt[bin]) - ((int) L2_pt[bin]);
				acc += delta*delta ;
			}

			/* Filter the best and second best matching point. */
			if(acc < best) {
				second_best = best;
				best = acc ;
				bestk = k2 ;
			} else if(acc < second_best){		second_best = acc ;			}
		}
		L2_pt -= ND*KK2;

		/* Lowe's method: accept the match only if unique. */
		if(thresh * (float) best < (float) second_best &&	bestk != -1) { 
				pairs_iterator->k1 = k1;
				pairs_iterator->k2 = bestk;
				pairs_iterator->score = best;
				pairs_iterator++;
				(nMatches)++;
		}
	}

	Pair* pairs_end = pairs_iterator;
	double* M_pt = (double*)calloc((nMatches)*2,sizeof(double));

	for(pairs_iterator = pairs_begin;
		pairs_iterator < pairs_end ;
		++pairs_iterator){
		*M_pt++ = pairs_iterator->k1;
		*M_pt++ = pairs_iterator->k2;
		//*M_pt++ = pairs_iterator->k1+1;
		//*M_pt++ = pairs_iterator->k2+1;

	}
	M_pt -= (nMatches)*2;

	memcpy(MATCHES,M_pt,(nMatches) * 2 * sizeof(double));

	free(pairs_begin);
	free(M_pt);
	
	return;
}
