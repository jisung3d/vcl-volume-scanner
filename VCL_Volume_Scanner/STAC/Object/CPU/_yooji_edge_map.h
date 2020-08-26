//********************************************************************************************
//class AFX_EXT_CLASS CKvYooji_Edge_Map : public CKvClass
// + This class contains the information of camera(object) pose tracking.
class CKvYooji_Edge_Map : public CKvClass
//********************************************************************************************
{
public:

	CKvYooji_Edge_Map(void);
	~CKvYooji_Edge_Map(void);
 	CKvYooji_Edge_Map(CKvYooji_Edge_Map &a);
 
 	void cp_Copy(CKvYooji_Edge_Map *a);
 	void c_Create(int in_hh, int in_ww);
 
 public:
 
 	//int levOfScale;			// the number of scales of image pyramid.	
	int ww, hh;

	CKvMatrixFloat grad_x;
	CKvMatrixFloat grad_y;

	CKvMatrixFloat grad_xx;
	CKvMatrixFloat grad_yy;
	CKvMatrixFloat grad_xy;
	CKvMatrixFloat grad_yx;

	CKvMatrixFloat grad_mag;
	CKvMatrixFloat laplacian;
	CKvMatrixFloat grad_ori;		// 0 ~ 2PI
	CKvMatrixBool edge_mask;
};

//********************************************************************************************
//class AFX_EXT_CLASS CKvYooji_Edge_Map_Pyramid : public CKvClass
// + This class contains the information of camera(object) pose tracking.
class CKvYooji_Pyramid_Map_Edge : public CKvClass
//********************************************************************************************
{
public:

	CKvYooji_Pyramid_Map_Edge(void);
	~CKvYooji_Pyramid_Map_Edge(void);
 	CKvYooji_Pyramid_Map_Edge(CKvYooji_Pyramid_Map_Edge &a);
 
 	void cp_Copy(CKvYooji_Pyramid_Map_Edge *a);
	void c_Create(int in_ww_bottom, int in_hh_bottom,
		int in_level_of_scale = 3);
 
 public:
 
 	int levOfScale;			// the number of scales of image pyramid.	
	int ww, hh;

	vector<CKvYooji_Edge_Map> maps;

// 	vector<CKvMatrixFloat> grad_x;
// 	vector<CKvMatrixFloat> grad_y;
// 
// 	vector<CKvMatrixFloat> grad_mag;
// 	vector<CKvMatrixFloat> grad_ori;		// 0 ~ 2PI
// 	vector<CKvMatrixBool> edge_mask;

	//vector<CKvYooji_Intrinsics> intrins;
};

//********************************************************************************************
//class AFX_EXT_CLASS CKvYooji_Edge_Map_Pyramid : public CKvClass
// + This class contains the information of camera(object) pose tracking.
class CKvYooji_Pyramid_Map_Depth_Edge : public CKvClass
//********************************************************************************************
{
public:

	CKvYooji_Pyramid_Map_Depth_Edge(void);
	~CKvYooji_Pyramid_Map_Depth_Edge(void);
 	CKvYooji_Pyramid_Map_Depth_Edge(CKvYooji_Pyramid_Map_Depth_Edge &a);
 
 	void cp_Copy(CKvYooji_Pyramid_Map_Depth_Edge *a);
	void c_Create(int in_ww_bottom, int in_hh_bottom,
		int in_level_of_scale = 3);
 
 public:
 
 	int levOfScale;			// the number of scales of image pyramid.	
	int ww, hh;

	vector<CKvYooji_Edge_Map> maps;

// 	vector<CKvMatrixFloat> grad_x;
// 	vector<CKvMatrixFloat> grad_y;
// 
// 	vector<CKvMatrixFloat> grad_mag;
// 	vector<CKvMatrixFloat> grad_ori;		// 0 ~ 2PI
// 	vector<CKvMatrixBool> edge_mask;

	//vector<CKvYooji_Intrinsics> intrins;
};

