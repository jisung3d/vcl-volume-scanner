//********************************************************************************************
//class AFX_EXT_CLASS CKvYooji_FrameCapture : public CKvClass
class CKvYooji_InterLib_Convert : public CKvClass
//********************************************************************************************
{
public:
	CKvYooji_InterLib_Convert(void);
	virtual ~CKvYooji_InterLib_Convert(void);

	// KAISION - OPENCV
	void cfko_Convert_Format_from_KAISION_to_Opencv(
		CKvMatrixUchar &in_img,
		Mat &out_img);
	void cfko_Convert_Format_from_KAISION_to_Opencv(
		CKvMatrixUcharRgb &in_img,
		Mat &out_img);
	void cfko_Convert_Format_from_KAISION_to_Opencv(
		CKvMatrixFloat &in_mat,
		Mat &out_mat,
		int in_output_type = CV_64F);
	void cfko_Convert_Format_from_KAISION_to_Opencv(
		CKvMatrix &in_mat,
		Mat &out_mat,
		int in_output_type = CV_64F);

	void cfok_Convert_Format_from_Opencv_to_KAISION(
		Mat &in_img,
		CKvMatrixUcharRgb &out_img);
	void cfok_Convert_Format_from_Opencv_to_KAISION(
		Mat &in_mat,
		CKvMatrixFloat &out_mat);
};