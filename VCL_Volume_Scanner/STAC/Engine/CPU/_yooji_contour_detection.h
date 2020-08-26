//********************************************************************************************
class CKvYooji_ContourDetection : public CKvClass
//********************************************************************************************
{
public:
	CKvYooji_ContourDetection(void);
	virtual ~CKvYooji_ContourDetection(void);

	// Gradient computation.
	bool cmm_Compute_Magnitude_Map(
		CKvMatrixFloat &in_map_grad_x,
		CKvMatrixFloat &in_map_grad_y,
		CKvMatrixFloat &out_map_grad_mag);

	bool clm_Compute_Laplacian_Map(
		CKvMatrixFloat &in_map_grad_xx,
		CKvMatrixFloat &in_map_grad_yy,
		CKvMatrixFloat &out_map_laplacian);

	bool clm_Compute_Laplacian_Map(
		CKvMatrixUchar &in_gray_image,
		CKvMatrixFloat &out_map_laplacian);

	// + Simple operator.
	int cig_Compute_Image_Gradients(
		CKvMatrixUchar &in_gray_image,
		CKvMatrixFloat &out_map_grad_x,
		CKvMatrixFloat &out_map_grad_y);

	int cig_Compute_Image_Gradients(
		CKvMatrixFloat &in_image_gray,
		CKvMatrixFloat &out_map_grad_x,
		CKvMatrixFloat &out_map_grad_y);
	
	// + Sobel operator.
	int cigs_Compute_Image_Gradients_using_Sobel(
		CKvMatrixUchar &in_gray_image,
		CKvMatrixFloat &out_map_grad_x,
		CKvMatrixFloat &out_map_grad_y);

	int cigs_Compute_Image_Gradients_using_Sobel(
		CKvMatrixFloat &in_image_gray,
		CKvMatrixFloat &out_map_grad_x,
		CKvMatrixFloat &out_map_grad_y,
		float in_scale = 1.0f);

	int cigs_Compute_Image_Gradients_using_Scharr(
		CKvMatrixUchar &in_gray_image,
		CKvMatrixFloat &out_map_grad_x,
		CKvMatrixFloat &out_map_grad_y);

	int cigs_Compute_Image_Gradients_using_Scharr(
		CKvMatrixFloat &in_image_gray,
		CKvMatrixFloat &out_map_grad_x,
		CKvMatrixFloat &out_map_grad_y);

	int ees_Extract_Edges_using_Sobel(
		CKvMatrixUchar &in_gray_image,
		CKvMatrixFloat &out_map_gradient_magnitude,
		CKvMatrixFloat &out_map_gradient_orientation);	// orientation range: 0 ~ 2*PI radian



	void ribi_Resize_Image_using_Bilinear_Interpolation(CKvMatrixUchar &in_img_gray,
		int in_width_new,
		int in_height_new,
		CKvMatrixUchar &out_img_resized);

	void ribi_Resize_Image_using_Bilinear_Interpolation(CKvMatrixUcharRgb &in_img_rgb,
		int in_width_new,
		int in_height_new,
		CKvMatrixUcharRgb &out_img_resized);

	// Statistics.
	void mii_Make_Integral_Image(CKvMatrixUchar *in_image_gray,
		CKvMatrixFloat *out_sum_image,
		CKvMatrixFloat *out_sum_of_square_image_or_NULL = NULL);
	void mii_Make_Integral_Image(
		CKvMatrixFloat *in_image,
		CKvMatrixFloat *out_sum_image,
		CKvMatrixFloat *out_sum_of_square_image_or_NULL = NULL);
	
	bool glsii_Get_Local_Sum_from_Integral_Image(CKvMatrixFloat *in_integ_image,
		const int in_x,const int in_y,
		const int in_dx,const int in_dy,
		float &out_sum);
	void gmii_Get_Mean_from_Integral_Image(CKvMatrixFloat *in_sum_image,
		const int in_x,const int in_y,
		const int in_mask_width,const int in_mask_height,
		float &out_mean_value);
	void gvii_Get_Variance_from_Integral_Image(CKvMatrixFloat *in_sum_image,
		CKvMatrixFloat *in_sum_of_square_image,
		const int in_x,const int in_y,
		const int in_mask_width,const int in_mask_height,
		float &out_variance_value);
};