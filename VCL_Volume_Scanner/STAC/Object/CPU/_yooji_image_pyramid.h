//********************************************************************************************
//class AFX_EXT_CLASS CKvYooji_Image_Pyramid : public CKvClass
// + This class contains the information of camera(object) pose tracking.
class CKvYooji_Pyramid_Mat_Bool : public CKvClass
//********************************************************************************************
{
public:

	CKvYooji_Pyramid_Mat_Bool(void);
	~CKvYooji_Pyramid_Mat_Bool(void);
 	CKvYooji_Pyramid_Mat_Bool(CKvYooji_Pyramid_Mat_Bool &a);
 
 	void cp_Copy(CKvYooji_Pyramid_Mat_Bool *a);
 	void c_Create(
 		int in_ww_bottom, int in_hh_bottom,
 		int in_level_of_scale = 3);
 
 public:
 
 	int levOfScale;			// the number of scales of image pyramid.	
 
 	vector<CKvMatrixBool> imgs;
	//vector<CKvYooji_Intrinsics> intrins;
};

//********************************************************************************************
//class AFX_EXT_CLASS CKvYooji_Image_Pyramid : public CKvClass
// + This class contains the information of camera(object) pose tracking.
class CKvYooji_Pyramid_Mat_Uchar : public CKvClass
//********************************************************************************************
{
public:

	CKvYooji_Pyramid_Mat_Uchar(void);
	~CKvYooji_Pyramid_Mat_Uchar(void);
 	CKvYooji_Pyramid_Mat_Uchar(CKvYooji_Pyramid_Mat_Uchar &a);
 
 	void cp_Copy(CKvYooji_Pyramid_Mat_Uchar *a);
 	void c_Create(
 		int in_ww_bottom, int in_hh_bottom,
 		int in_level_of_scale = 3);
 
 public:
 
 	int levOfScale;			// the number of scales of image pyramid.	
 
 	vector<CKvMatrixUchar> imgs;
	//vector<CKvYooji_Intrinsics> intrins;
};

//********************************************************************************************
//class AFX_EXT_CLASS CKvYooji_Image_Pyramid : public CKvClass
// + This class contains the information of camera(object) pose tracking.
class CKvYooji_Image_Pyramid_Short : public CKvClass
//********************************************************************************************
{
public:

	CKvYooji_Image_Pyramid_Short(void);
	~CKvYooji_Image_Pyramid_Short(void);
 	CKvYooji_Image_Pyramid_Short(CKvYooji_Image_Pyramid_Short &a);
 
 	void cp_Copy(CKvYooji_Image_Pyramid_Short *a);
 	void c_Create(
 		int in_ww_bottom, int in_hh_bottom,
 		int in_level_of_scale = 3);
 
 public:
 
 	int levOfScale;			// the number of scales of image pyramid.	
 
 	vector<CKvMatrixShort> imgs;
	//vector<CKvYooji_Intrinsics> intrins;
};

//********************************************************************************************
//class AFX_EXT_CLASS CKvYooji_Image_Pyramid : public CKvClass
// + This class contains the information of camera(object) pose tracking.
class CKvYooji_Pyramid_Mat_Float : public CKvClass
//********************************************************************************************
{
public:

	CKvYooji_Pyramid_Mat_Float(void);
	~CKvYooji_Pyramid_Mat_Float(void);
 	CKvYooji_Pyramid_Mat_Float(CKvYooji_Pyramid_Mat_Float &a);
 
 	void cp_Copy(CKvYooji_Pyramid_Mat_Float *a);
 	void c_Create(
 		int in_ww_bottom, int in_hh_bottom,
 		int in_level_of_scale = 3);
 
 public:
 
 	int levOfScale;			// the number of scales of image pyramid.	
 
	vector<CKvMatrixFloat> imgs;
	//vector<CKvYooji_Intrinsics> intrins;
};

//********************************************************************************************
//class AFX_EXT_CLASS CKvYooji_Image_Pyramid : public CKvClass
// + This class contains the information of camera(object) pose tracking.
class CKvYooji_Image_Pyramid_Point3Df : public CKvClass
//********************************************************************************************
{
public:

	CKvYooji_Image_Pyramid_Point3Df(void);
	~CKvYooji_Image_Pyramid_Point3Df(void);
 	CKvYooji_Image_Pyramid_Point3Df(CKvYooji_Image_Pyramid_Point3Df &a);
 
 	void cp_Copy(CKvYooji_Image_Pyramid_Point3Df *a);
 	void c_Create(
 		int in_ww_bottom, int in_hh_bottom,
 		int in_level_of_scale = 3);
 
 public:
 
 	int levOfScale;			// the number of scales of image pyramid.	
 
 	vector<CKvSet2d_of_Point3Df> imgs;
	//vector<CKvYooji_Intrinsics> intrins;

};

//********************************************************************************************
//class AFX_EXT_CLASS CKvYooji_Image_Pyramid : public CKvClass
// + This class contains the information of camera(object) pose tracking.
class CKvYooji_Pyramid_Intrinsics : public CKvClass
//********************************************************************************************
{
public:

	CKvYooji_Pyramid_Intrinsics(void);
	~CKvYooji_Pyramid_Intrinsics(void);
 	CKvYooji_Pyramid_Intrinsics(CKvYooji_Pyramid_Intrinsics &a);
 
 	void cp_Copy(CKvYooji_Pyramid_Intrinsics *a);
 	void c_Create(int in_level_of_scale = 3);
 
 public:
 
 	int levOfScale;			// the number of scales of image pyramid.	
	vector<CKvYooji_Intrinsics> intrins;
};