#pragma once

// KAISION
#include <_sdkim_2008_hello.h>

// defines.
#define MAX_DEPTH_VAL 3000				// mm
#define MIN_DEPTH_VAL 300				// mm

//********************************************************************************************
class LCKvOpenNI  : public CKvClass
//********************************************************************************************
{
private:
	class PrivateData;
	PrivateData *data;
	bool colorAvailable, depthAvailable;

public:
	LCKvOpenNI(const char *deviceURI = NULL, const bool useInternalCalibration = false);
	//LCKvOpenNI(const char *calibFilename, const char *deviceURI = NULL, const bool useInternalCalibration = true);
	virtual ~LCKvOpenNI();

	bool gi_Get_Images(CKvMatrixUcharRgb *out_img_rgb, CKvMatrixUshort *out_img_d);
	bool gi_Get_Images(CKvMatrixUcharRgb *out_img_rgb, CKvMatrixFloat *out_img_d);
	bool gi_Get_Images(CKvMatrixUshort *out_img_IR, CKvMatrixFloat *out_img_d);
	
	void sroi_Set_Region_Of_Interest(CKvMatrixUshort *io_img_d, int x_tl, int x_br, int y_tl, int y_br);	
	void cid8_Convert_Image_Depth_to_8bit(CKvMatrixUshort *in_img_d, CKvMatrixUchar *out_img_d);
	void cid8_Convert_Image_Depth_to_8bit(CKvMatrixUshort *in_img_d, CKvMatrixUchar *out_img_d,
		int in_min_d, int in_max_d);
	int cid8_Convert_Image_Depth_to_8bit(CKvMatrixUshort *in_img_d, CKvMatrixUchar *out_img_d,
		int x_tl, int x_br, int y_tl, int y_br);

	/// /////////////////////////////
	void cid8_ConvertImageDepth8bit(CKvMatrixUshort *in_imgDepth,
		int in_modeConversion,
		CKvMatrixUchar *out_imgDepth);
	void cid8_ConvertImageDepth8bit(CKvMatrixFloat *in_imgDepth,
		int in_modeConversion,
		CKvMatrixUchar *out_imgDepth);
};