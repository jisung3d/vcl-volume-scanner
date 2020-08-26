
// VCL_Object_Scanner_SDIView.h : CVCL_Object_Scanner_SDIView 클래스의 인터페이스
//
#pragma once

class CVCL_Volume_Scanner_View : public CView, public CKvClass
{
protected: // serialization에서만 만들어집니다.
	CVCL_Volume_Scanner_View();
	DECLARE_DYNCREATE(CVCL_Volume_Scanner_View)

// 특성입니다.
public:
	CVCL_Volume_Scanner_Doc* GetDocument() const;
	

// 작업입니다.
public:

	// =================================================
 	LCKvIO zz_io;
 	LCKvIO_FileJpg zz_iof;
 
//  	// 0: single bmp image.
//  	// 1: RGB-D image sequence.
//  	// 	// 2: KINECT2 video stream.
//  	int zz_input_type;
// 	// 0: Basic KinectFusion.
// 	// 1: KinFu + Color glove.
// 	// 2: KinFu + Visual-hull shield.
// 	int zz_scanning_type;
 
 	// for playing RGB-D video stream and loading RGB-D image sequence.
 	//CKvYooji_FrameCapture zz_capture;
 
 	int zz_num_frame;
 
 	// for 3D scanning.
 	//CKvYooji_3D_Object_Scanner zz_3dos;
 
 	// For display.
	LCKvYooji_Scanner_Display zz_display;
	CKvYooji_ContourDetection zz_cd;
	LCKvUtility_for_Windows zz_uw;

	CKvData_Color_x_Depth zz_mat_cxd;
 	CKvYooji_MatrixRgbD zz_mat_rgbd;
 	CKvYooji_Tracking_State *zz_p_state;
 	// 
 	CKvMatrixUcharRgb zz_cimg[4];
	CKvMatrixUcharRgb zz_img_gt;
 	CKvMatrixUchar zz_img[4];
 
 	//CKvScreen zz_sc[4];
 
 	int zz_idx_frame;
 	int zz_flag_player;
 	bool zz_flag_scanning;

	// ===========================================
	CKvPoint3Df zz_cube_origin;
	float zz_cube_size;
	int zz_cube_dim;
	//int zz_cnt;
	// ===========================================


// 재정의입니다.
public:
	virtual void OnDraw(CDC* pDC);  // 이 뷰를 그리기 위해 재정의되었습니다.
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:
	virtual BOOL OnPreparePrinting(CPrintInfo* pInfo);
	virtual void OnBeginPrinting(CDC* pDC, CPrintInfo* pInfo);
	virtual void OnEndPrinting(CDC* pDC, CPrintInfo* pInfo);

// 구현입니다.
public:
	virtual ~CVCL_Volume_Scanner_View();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// 생성된 메시지 맵 함수
protected:
	DECLARE_MESSAGE_MAP()

public:
	
	// Toolbar functions.
	afx_msg void OnButtonPlay();
	afx_msg void OnButtonPause();
	afx_msg void OnButtonStop();
	afx_msg void OnButton3dScanning();
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);

	// Menu functions.
	afx_msg void OnRenderTSDFwithPCL();
	afx_msg void OnRenderScene();
	afx_msg void OnRenderObject();


	afx_msg void OnScanningKinectfusion();
	afx_msg void OnScanningKinFuWithGlove();
	afx_msg void OnScanningPhotoMax();
	afx_msg void OnTestDepthflow();
	afx_msg void OnTestRgbdOdometryOpencv();
	afx_msg void OnTestPoseEstimationWithArucoMarkersOpenCV();
	afx_msg void OnTestGradientFieldDisplay();
	afx_msg void OnTestSiftGpuTest();
	afx_msg void OnTestSinglePoseEstimationWithArucoMarkersOpencv();
	afx_msg void OnTestSequenceposeestimationwithcharucomarkersopencv();
	afx_msg void OnTestSingleposeestimationwithcharucomarkersopencv();
	afx_msg void OnCalibrationCapturergb();
	afx_msg void OnTestChessboardremovalfromdepthmaps();
	afx_msg void OnTestAverageimage();
};

#ifndef _DEBUG  // VCL_Object_Scanner_SDIView.cpp의 디버그 버전
inline CVCL_Volume_Scanner_Doc* CVCL_Volume_Scanner_View::GetDocument() const
   { return reinterpret_cast<CVCL_Volume_Scanner_Doc*>(m_pDocument); }
#endif

