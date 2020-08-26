//#pragma once
//
//#include <vector>
//using namespace std;
//
//// SiftGPU
//// #include <glew.h>
//// #include <glut.h>
//// #include <wglew.h>
//// #include <il.h>
//// 
//// #include "SiftGPU.h"
//
//// VLFeat
////#include <generic.h>
////#include <sift.h>
////
////typedef struct
////{
////	int k1;
////	int k2;
////	double score;
////} Pair;
////
////// re-define the data type of which name is long.
////typedef unsigned char uchar;
////typedef unsigned short ushort;
////typedef unsigned int uint;
////typedef unsigned long ulong;
//
//#include "_sdkim_2008_hello.h"
//#include "_sdkim_2008_screen.h"

// matching mode.
#define KV_MATCHING_HIST_OF_INTENSITIES 0
#define KV_MATCHING_HIST_OF_ORIENTED_GRADIENTS 1
#define KV_MATCHING_HIST_OF_COLORS 2
#define KV_MATCHING_HIST_OF_DOMINANT_GRADIENTS 3

//********************************************************************************************
//class AFX_EXT_CLASS LCKvYooji_Image_Processor : public CKvClass
class LCKvImageMatcher: public CKvClass
//********************************************************************************************
{
public:
	LCKvImageMatcher();
	virtual ~LCKvImageMatcher();

	void i_Initialize(int in_max_num_features = 4096);

	// Local descriptor-based matching
	void emsf_Extract_and_Match_SIFT_Features(CKvMatrixUchar *in_src_image,
		CKvMatrixUchar *in_dst_image,
		int &out_num_matched,
		float *out_matched_src_points,
		float *out_matched_dst_points);

	void emsf_Extract_and_Match_SIFT_Features(CKvMatrixUchar *in_src_image,
		CKvMatrixUchar *in_dst_image,
		int &out_num_matched,
		Vector2f *out_matched_src_points,
		Vector2f *out_matched_dst_points);

	void emsf_Extract_and_Match_SIFT_Features(CKvMatrixUchar *in_src_image,
		CKvMatrixUchar *in_dst_image,
		int &out_num_matched,
		vector<Vector2f> &out_matched_src_points,
		vector<Vector2f> &out_matched_dst_points);

	void emsf_Extract_and_Match_SIFT_Features(CKvMatrixUcharRgb *in_src_image,
		CKvMatrixUcharRgb *in_dst_image,
		int &out_num_matched,
		float *out_matched_src_points,
		float *out_matched_dst_points);
	
	bool emf_Extract_and_Match_Features_using_SiftGPU(CKvMatrixUchar &in_src_image,
		CKvMatrixUchar &in_dst_image,
		vector<Point2f> &out_p2d_matched_src,
		vector<Point2f> &out_p2d_matched_dst,
		CKvMatrixFloat *in_map_depth_src = NULL,
		CKvMatrixFloat *in_map_depth_dst = NULL);

	bool emf_Extract_and_Match_Features_using_SiftGPU(CKvMatrixUchar &in_src_image,
		CKvMatrixUchar &in_dst_image,
		int &out_num_matched,
		float *out_matched_src_points,
		float *out_matched_dst_points);

	// Display matched results.
	void esf_Extract_SIFT_Features_VLFeat(
		CKvMatrixUchar &in_image, 
		vl_uint8 *DATAdescr,
		double *DATAframes,
		int &nframes);

	void mesf_Match_Extracted_SIFT_Features_VLFeat(
		vl_uint8* L1_pt,
		vl_uint8* L2_pt, 
		int KK1, 
		int KK2, 
		double thresh, 
		int &nMatches, 
		double *MATCHES);

protected:

	LCKvUtility_for_YCbCr aa_ycc;

	VlSiftKeypoint zz_sift_keys;
	VlSiftFilt zz_sift_filt;

	CKvMatrixFloat zz_frame_float;
	CKvMatrixUchar zz_img_g_src, zz_img_g_dst;
	
	///
	SiftGPU *zz_Sift;
	SiftMatchGPU *zz_SiftMat;
	vector<SiftGPU::SiftKeypoint> zz_vKeysS, zz_vKeysD;
	vector<float> zz_vDescriptorS, zz_vDescriptorD;
	int (*zz_match_buff)[2];

	///

	// VL_Feat
	vl_uint8 *zz_DATAdescr[2];
	double *zz_DATAframes[2], *zz_p_matches;
	int zz_nframe[2], zz_max_feat_num, zz_matched_num;
};

