
// VCL_Object_Scanner_SDIDoc.h : CVCL_Object_Scanner_SDIDoc 클래스의 인터페이스
//


#pragma once


class CVCL_Volume_Scanner_Doc : public CDocument
{
protected: // serialization에서만 만들어집니다.
	CVCL_Volume_Scanner_Doc();
	DECLARE_DYNCREATE(CVCL_Volume_Scanner_Doc)

// 특성입니다.
public:
	LCKvOpenNI zz_prime;
	CKvYooji_FrameCapture zz_capture;
	CKvYooji_3D_Object_Scanner zz_3dos;

	// 0: Saved RGB-D image sequence.
	// 1: KINECTV1 video stream.
	// 2: KINECTV2 video stream.
	int zz_input_type;
	// 0: Basic KinectFusion.
	// 1: KinFu + Color glove.
	// 2: KinFu + Visual-hull shield.
	int zz_scan_type;

	CKvYooji_Cube_TSDF_Float zz_cube;

 
// 작업입니다.
public:

// 재정의입니다.
public:
	virtual BOOL OnNewDocument();
	virtual void Serialize(CArchive& ar);
#ifdef SHARED_HANDLERS
	virtual void InitializeSearchContent();
	virtual void OnDrawThumbnail(CDC& dc, LPRECT lprcBounds);
#endif // SHARED_HANDLERS

// 구현입니다.
public:
	virtual ~CVCL_Volume_Scanner_Doc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// 생성된 메시지 맵 함수
protected:
	DECLARE_MESSAGE_MAP()

#ifdef SHARED_HANDLERS
	// 검색 처리기에 대한 검색 콘텐츠를 설정하는 도우미 함수
	void SetSearchContent(const CString& value);
#endif // SHARED_HANDLERS
public:

//	afx_msg void OnInputFile();
	afx_msg void OnLoadTSDFasVCL();
	afx_msg void OnInputImageSequenceVCL();
	afx_msg void OnInputVideoStreamKinectV1();

	afx_msg void OnSaveTSDFasBin();
	afx_msg void OnSaveObjectVCL();
	afx_msg void OnSaveInputSequenceAsVCL();
	afx_msg void OnSaveInputSequenceAsCXD();
	afx_msg void OnConvertVcLtoCxD();
	afx_msg void OnInputImageSequenceCxD();
	afx_msg void OnFileExit();
	afx_msg void OnSaveEstimatedTrajactoryQuarternion();

	void writeMatToTextFile(Mat& m,const char* filename);
	void readTextFileToMat(Mat& m,const char* filename);
	void writeMatSetToTextFile(vector<Mat>& m, const char* filename);
	afx_msg void OnSaveSavesilhouettesequence();
	afx_msg void OnSaveSaveestimatedtrajactory4x4();
	afx_msg void OnConvertTransformtoquarternion();
};
