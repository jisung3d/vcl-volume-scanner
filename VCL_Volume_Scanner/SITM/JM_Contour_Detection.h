#pragma  once

#include <vector>
using namespace std;
using std::vector;

// KAISION
#include <_sdkim_2008_hello.h>
#include <_sdkim_2008_io.h>
#include <_sdkim_2008_bootstrap.h>
#include <_sdkim_2008_algebra.h>
#include <_sdkim_2008_array.h>
#include <_sdkim_2008_geometry.h>
#include <_sdkim_2008_object_geometric.h>
#include <_sdkim_2008_graph_3d.h>
#include <_sdkim_2008_jpeg.h>
#include <_sdkim_2008_screen.h>
#include <_sdkim_2008_color.h>
#include <_sdkim_2008_tempo.h>

// MACROS
#define SQUARE(x) (x)*(x)


// histogram
#define DEFAULT_IMAGE_WIDTH_FOR_HISTOGRAM_IMAGE 256
#define MINIMUM_SUM_OF_GRADIENT_MAGNITUDE 300

#define MAX_TMPL_NUM 20

#define KV_HIST_SIMILARITY_OVERLAP 0
#define KV_HIST_DISTANCE_L1_NORM 1
#define KV_HIST_DISTANCE_L2_NORM 2

#define KV_HIST_NORMALIZE_L1_NORM 1
#define KV_HIST_NORMALIZE_L2_NORM 2

#define KV_HIST_DOMINANT_GRADIENT_REDUCTION_RATIO 0.2f

#define KV_HIST_FILTER_TYPE_HOI 0
#define KV_HIST_FILTER_TYPE_HOG 1

// contour detection.
#define MARGIN_OF_Y_POSITION_FOR_CONTOUR_DETECTION 20

//********************************************************************************************
class CKvHistogram
//********************************************************************************************
{
public:
	CKvHistogram();
	virtual ~CKvHistogram();
	
	// Histogram
	float* c_Create(int in_bin_number);
	float* im_Import(float *in_histogram_pointer,
		int in_bin_number);
	float* hp_Histogram_Pointer();

	float* cnhoi_Compute_Normalized_Histogram_Of_Intensity(
		CKvMatrixUchar *in_gray_image,
		int in_number_of_bins);

	float* cnhog_Compute_Normalized_Histogram_Of_Gradient(
		CKvMatrixFloat *in_edge_mag_image,
		CKvMatrixFloat *in_edge_grad_image,
		int in_number_of_bins);

	float* cnhogs_Compute_Normalized_Histogram_Of_Gradient_in_SIFT(
		CKvMatrixFloat *in_edge_mag_image,
		CKvMatrixFloat *in_edge_ori_image,
		int in_number_of_bins);

	float* cnhogsb_Compute_Normalized_Histogram_Of_Gradient_of_Sub_Blocks(
		CKvMatrixFloat *in_edge_mag_image,
		CKvMatrixFloat *in_edge_ori_image,
		CKvMatrixFloat *in_sub_block_position_ratios,				// 4xn matrix, position ratio: the top left and the bottom right / image width or height.
		int in_number_of_bins);

	float* cnhogsbs_Compute_Normalized_Histogram_Of_Gradient_of_Sub_Blocks_in_SIFT(
		CKvMatrixFloat *in_edge_mag_image,
		CKvMatrixFloat *in_edge_ori_image,
		CKvMatrixFloat *in_sub_block_position_ratios,				// 4xn matrix, position ratio: the top left and the bottom right / image width or height.
		int in_number_of_bins);

	float* csbiqt_Compose_Sub_Blocks_of_Image_using_Quad_Tree(
		CKvMatrixFloat *in_edge_mag_image,
		CKvMatrixFloat *in_edge_ori_image,
		CKvMatrixFloat *io_sub_block_position_ratios,			// 4xn matrix, position ratio: the top left and the bottom right / image width or height.
		int mask_size_of_hist_filter);				

	void csbiqt_Compose_Sub_Blocks_of_Image_using_Quad_Tree(
		CKvMatrixFloat *in_edge_mag_image,
		CKvMatrixFloat *in_edge_ori_image,
		int in_minimum_block_size,	
		int in_maximum_tree_level,
		CKvSet_of_MatrixFloat *io_set_of_sub_block_position_ratios,
		int mask_size_of_hist_filter);


	int* svsb_Select_Valid_Sub_Blocks(
		CKvMatrixFloat *in_edge_mag_image,
		CKvMatrixFloat *in_edge_ori_image,
		CKvMatrixFloat *in_sub_block_position_ratios,				// 4xn matrix, position ratio: the top left and the bottom right / image width or height.
		CKvVectorInt *out_valid_block_indices);

	float* cnhogvsb_Compute_Normalized_Histogram_of_Oriented_Gradient_of_Valid_Sub_Blocks(
		CKvMatrixFloat *in_edge_mag_image,
		CKvMatrixFloat *in_edge_ori_image,
		CKvMatrixFloat *in_sub_block_position_ratios,				// 4xn matrix, position ratio: the top left and the bottom right / image width or height.
		int in_number_of_bins,
		int in_normalize_mode,
		int in_dominant_gradient_reduction_mode,
		CKvVectorInt *out_valid_block_indices);

	float* cnhogsbt_Compute_Normalized_Histogram_of_Oriented_Gradient_of_Sub_Block_Tree(
		CKvMatrixFloat *in_edge_mag_image,
		CKvMatrixFloat *in_edge_ori_image,
		CKvSet_of_MatrixFloat *in_set_of_sub_block_position_ratios,				// 4xn matrix, position ratio: the top left and the bottom right / image width or height.
		int in_number_of_bins,
		int in_normalize_mode,
		int in_dominant_gradient_reduction_mode,
		CKvSet_of_VectorInt *out_set_of_valid_block_indices);

	float* cnhogwsbt_Compute_Normalized_Histogram_of_Oriented_Gradient_of_Weighted_Sub_Block_Tree(
		CKvMatrixFloat *in_edge_mag_image,
		CKvMatrixFloat *in_edge_ori_image,
		CKvSet_of_MatrixFloat *in_set_of_sub_block_position_ratios,				// 4xn matrix, position ratio: the top left and the bottom right / image width or height.
		int in_number_of_bins,
		int in_normalize_mode,
		int in_dominant_gradient_reduction_mode,
		CKvSet_of_VectorFloat *out_set_of_weight_vector);

	// Integral histogram.
	void mihoi_Make_Integral_Histogram_Of_Intensity(	
		CKvMatrixUchar *in_gray_image,
		int in_number_of_bins,
		CKvSet_of_MatrixInt *out_integral_Histogram);

	void mihoie_Make_Integral_Histogram_Of_Intensity_Efficeint(	
		CKvMatrixUchar *in_gray_image,
		int in_max_window_area,
		int in_number_of_bins,
		CKvSet_of_MatrixInt *out_integral_Histogram);
	
	void mihog_Make_Integral_Histogram_Of_Gradient(	
		CKvMatrixFloat *in_edge_mag_image,
		CKvMatrixFloat *in_edge_grad_image,
		int in_number_of_bins,
		CKvSet_of_MatrixInt *out_integral_Histogram);

	void msirhoi_Make_Set_of_Rotated_Integral_Histograms_Of_Intensity(	
		CKvMatrixUchar *in_gray_image,
		int in_number_of_bins,
		int in_number_of_rotations,
		vector<CKvSet_of_MatrixInt> &out_integral_Histogram,
		CKvSet_of_Pixel &out_offsets);

	void gnhih_Get_Normalized_Histogram_from_Integral_Histogram(
		CKvSet_of_MatrixInt *in_integral_hist,
		int in_x,
		int in_y,
		int in_mask_size_ww,
		int in_mask_size_hh,
		float *out_hist_from_integral);

	void gnhihs_Get_Normalized_Histogram_from_Integral_Histogram_in_SIFT(
		CKvSet_of_MatrixInt *in_integral_hist,
		int in_x,
		int in_y,
		int in_mask_size_ww,
		int in_mask_size_hh,
		float *out_hist_from_integral);

	void gnhsbih_Get_Normalized_Histogram_of_Sub_Blocks_from_Integral_Histogram(
		CKvSet_of_MatrixInt *in_integral_hist,
		int in_x,
		int in_y,
		int in_mask_size_ww,
		int in_mask_size_hh,
		CKvMatrixFloat *in_sub_block_position_ratios,
		float *out_hist_from_integral);

	void gnhsbihs_Get_Normalized_Histogram_of_Sub_Blocks_from_Integral_Histogram_in_SIFT(
		CKvSet_of_MatrixInt *in_integral_hist,
		int in_x,
		int in_y,
		int in_mask_size_ww,
		int in_mask_size_hh,
		CKvMatrixFloat *in_sub_block_position_ratios,
		float *out_hist_from_integral);

	void gnhvsbih_Get_Normalized_Histogram_of_Valid_Sub_Blocks_from_Integral_Histogram(
		CKvSet_of_MatrixInt *in_integral_hist,
		int in_x,
		int in_y,
		int in_mask_size_ww,
		int in_mask_size_hh,
		CKvMatrixFloat *in_sub_block_position_ratios,
		CKvVectorInt *in_valid_block_indices,
		int in_normalize_mode,
		int in_dominant_gradient_reduction_mode,
		float *out_hist_from_integral);

	/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void gnhwsbih_Get_Normalized_Histogram_of_Weighted_Sub_Blocks_from_Integral_Histogram(
		CKvSet_of_MatrixInt *in_integral_hist,
		int in_x,
		int in_y,
		int in_mask_size_ww,
		int in_mask_size_hh,
		CKvMatrixFloat *in_sub_block_position_ratios,
		int in_normalize_mode,
		int in_dominant_gradient_reduction_mode,
		float *out_hist_from_integral);
	/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void gnhvsbihe_Get_Normalized_Histogram_of_Valid_Sub_Blocks_from_Integral_Histogram_Efficient(
		CKvSet_of_MatrixInt *in_integral_hist,
		int in_x,
		int in_y,
		int in_mask_size_ww,
		int in_mask_size_hh,
		int in_max_window_area,
		CKvMatrixFloat *in_sub_block_position_ratios,
		CKvVectorInt *in_valid_block_indices,
		int in_normalize_mode,
		int in_dominant_gradient_reduction_mode,
		float *out_hist_from_integral);

	// Display histogram
	void ghi_Get_Histogram_Image(
		CKvMatrixUcharRgb *out_histogram_image,
		bool in_streched_mode,
		int in_color_mode);

	void ghi_Get_Histogram_Image(
		CKvMatrixUcharRgb* out_histogram_image,
		int in_ww, int in_hh,
		int in_scaling,
		bool in_streched_mode,
		int in_color_mode);

	void ghi_Get_Histogram_Image(
		float *in_hist,
		int in_num_bins,
		CKvMatrixUcharRgb* out_histogram_image,
		bool in_streched_mode,
		int in_color_mode);

	void ghi_Get_Histogram_Image(
		float *in_hist,
		int in_num_bins,
		CKvMatrixUcharRgb* out_histogram_image,
		int in_ww, int in_hh,
		int in_scaling,
		bool in_streched_mode,
		int in_color_mode);

	void ghgi_Get_Histogram_Grid_Image(
		CKvMatrixUcharRgb *out_histogram_grid_image,
		int in_grid_width,int in_grid_height,
		int in_grid_num_x, int in_grid_num_y,
		CKvMatrixFloat *in_sub_block_position_ratios,
		bool in_streched_mode,
		int in_color_mode);

	void ghgi_Get_Histogram_Grid_Image(
		CKvMatrixUcharRgb *out_histogram_grid_image,
		CKvMatrixFloat *in_sub_block_position_ratios,
		int in_ww, int in_hh,
		int in_bin_number,
		bool in_streched_mode,
		int in_color_mode);

	void dgi_Display_Grid_Image(
		CKvScreen *in_screen,
		CKvMatrixUcharRgb *in_image,
		CKvMatrixFloat *in_sub_block_position_ratios);

protected:
	int zz_hist_bin_num;
	float *zz_histogram;

};

//********************************************************************************************
class CKvSplittedDiscJM
//********************************************************************************************
{
public:
	CKvSplittedDiscJM();
	virtual ~CKvSplittedDiscJM();

	int c_Create(CKvMatrixUchar &in_gray_image,
		int in_x,
		int in_y,
		float in_split_angle,
		float in_disc_radius,
		int in_hist_bin_num);			// contains csh_Compute_Splitted_Histogram function.

	int cshoi_Compute_Splitted_Histogram_of_Intensity(CKvMatrixUchar &in_gray_image,
		int in_x,
		int in_y,
		float in_split_angle,
		float in_disc_radius,
		int in_hist_bin_num);			// just compute histograms of splitted regions.

	int cshog_Compute_Splitted_Histogram_of_Gradient(CKvMatrixFloat &in_edge_mag_image,
		CKvMatrixFloat &in_edge_ori_image,
		int in_x,
		int in_y,
		float in_disc_radius,
		int in_hist_bin_num);			// just compute histograms of splitted regions.

	int ssd_Show_Splitted_Disc(CKvMatrixUcharRgb &in_image,
		CKvRgbaF in_color_of_upper_disc,
		CKvRgbaF in_color_of_lower_disc);

	float *hp_Histogram_Pointer(int in_disc_index);		// upper disc: 0, lower disc: 1

protected:
	int zz_disc_radius;
	float zz_edge_gradient;
	int zz_pixel_num_in_upper_disc, zz_pixel_num_in_lower_disc;
	int *zz_pixels_in_upper_disc, *zz_pixels_in_lower_disc;		// vector size: zz_pixelNum*2 ------> (x, y)
	
	int zz_hist_bin_num;
	float *zz_intensity_hist_of_upper_disc, *zz_intensity_hist_of_lower_disc;

};

//********************************************************************************************
class CKvContourDetectionJM
//********************************************************************************************
{
public:
	CKvContourDetectionJM();
	virtual ~CKvContourDetectionJM();

	// Filtering.
	void siff_Smooth_Image_with_Filter_Float(CKvMatrixUchar &in_image,
		CKvMatrixFloat &in_filter_float,
		CKvMatrixUchar &out_filtered_image);
	void siff_Smooth_Image_with_Filter_Float(CKvMatrixUcharRgb &in_image,
		CKvMatrixFloat &in_filter_float,
		CKvMatrixUcharRgb &out_filtered_image);
	void siff_Smooth_Image_with_Filter_Float(CKvMatrixShort &in_image,
		CKvMatrixFloat &in_filter_float,
		CKvMatrixShort &out_filtered_image);
	void siff_Smooth_Image_with_Filter_Float(CKvMatrixShortRgb &in_image,
		CKvMatrixFloat &in_filter_float,
		CKvMatrixShortRgb &out_filtered_image);

	// Edge extraction.
	int exso_Edge_Extraction_using_Sobel_Operator(CKvMatrixUchar &in_gray_image,
		CKvMatrixFloat &out_edge_mag_image,
		CKvMatrixFloat &out_edge_grad_image);	// gradient range: 0 ~ 2*PI radian

	int exso_Edge_Extraction_using_Sobel_Operator(CKvMatrixFloat &in_gray_image,
		CKvMatrixFloat &out_edge_mag_image,
		CKvMatrixFloat &out_edge_grad_image);	// gradient range: 0 ~ 2*PI radian

	int cexlab_Color_Edge_Extraction_in_Lab_Color_Space(CKvMatrixUcharRgb &in_image,
		CKvMatrixFloat &out_edge_mag_image,
		CKvMatrixFloat &out_edge_grad_image);	// gradient range: 0 ~ 2*PI radian
	
	int efhs_Edge_Filtering_using_Histogram_Similarity(CKvMatrixUchar &in_gray_image,
		CKvMatrixFloat &in_edge_mag_image,
		CKvMatrixFloat &in_edge_ori_image,
		int in_histogram_filter_type,
		int in_histogram_distance_type,
		int in_disk_size,
		int in_hist_bin_num,
		float &out_hist_dist_max,
		CKvMatrixFloat &out_edge_mag_image);		// constructing...

	int efhs_Edge_Filtering_using_Histogram_Similarity(CKvMatrixUchar &in_gray_image,
		CKvMatrixFloat &in_edge_mag_image,
		CKvMatrixFloat &in_edge_ori_image,
		CKvMatrixFloat &in_gaussian_filter,
		int in_histogram_filter_type,
		int in_histogram_distance_type,
		int in_disk_size,
		int in_hist_bin_num,
		float &out_hist_dist_max,
		CKvMatrixFloat &out_edge_mag_image);		// constructing...

	int efhsrih_Edge_Filtering_using_Histogram_Similarity_using_Rotated_Integral_Histograms(CKvMatrixUchar &in_gray_image,
		CKvMatrixFloat &in_edge_mag_image,
		CKvMatrixFloat &in_edge_ori_image,
		int in_histogram_filter_type,
		int in_histogram_distance_type,
		int in_disk_size,
		int in_hist_bin_num,
		float &out_hist_dist_max,
		CKvMatrixFloat &out_edge_mag_image);

	int exhs_Edge_Extraction_using_Histogram_Similarity(CKvMatrixUchar &in_gray_image,
		int in_disk_size,
		int in_hist_bin_num,
		float &out_max_mag,
		CKvMatrixFloat &out_edge_mag_image,
		CKvMatrixFloat &out_edge_grad_image);		// constructing...

	int dms_Determine_Mask_Size_for_Histogram_Filtering(const int in_x, const int in_y,
		const int in_width, const int in_height,
		const float *in_edge_mag_image,
		const float *in_edge_ori_image,
		int &out_mask_size);
	
	// Statistics.
	void mii_Make_Integral_Image(CKvMatrixUchar *in_image_gray,
		CKvMatrixFloat *out_sum_image,
		CKvMatrixFloat *out_sum_of_square_image_or_NULL);
	void gmii_Get_Mean_from_Integral_Image(CKvMatrixFloat *in_sum_image,
		const int in_x, const int in_y,
		const int in_mask_width, const int in_mask_height,
		float &out_mean_value);
	void gvii_Get_Variance_from_Integral_Image(CKvMatrixFloat *in_sum_image,
		CKvMatrixFloat *in_sum_of_square_image,
		const int in_x, const int in_y,
		const int in_mask_width, const int in_mask_height,
		float &out_variance_value);

	void gtfm_Get_Threshold_from_Matrix(CKvMatrixFloat *in_matrix,
		float in_percentage_for_threshold,
		float &out_threshold);

	void gtfm_Get_Threshold_from_Matrix(CKvMatrixFloat *in_matrix,
		float in_min_value_NULL, float in_max_value_or_NULL,
		float in_percentage_for_threshold,
		float &out_threshold);

	
	// Calculate histogram similarity
	float cwhvs_Caculate_Weighted_Histogram_Vector_Similarity_using_Overlapped_Intensity(float *in_histogram1,
		float *in_histogram2,
		float *in_weight_vector,
		int in_block_number,
		int in_hist_bin_size);

	float chs_Caculate_Histogram_Similarity_using_Overlapped_Intensity(float *in_histogram1,
		float *in_histogram2,
		int in_hist_bin_size);

	float chd_Caculate_Histogram_Distance_using_L1_Norm(float *in_histogram1,
		float *in_histogram2,
		int in_hist_bin_size);

	float chd_Caculate_Histogram_Distance_using_L2_Norm(float *in_histogram1,
		float *in_histogram2,
		int in_hist_bin_size);

	float chd_Caculate_Histogram_Dissimilarity_using_Divergence(float *in_histogram1,
		float *in_histogram2,
		int in_hist_bin_size);
	
	// Etc.
	void ccirl_Convert_Color_Image_RGB_to_LAB(CKvMatrixUcharRgb &in_img_RGB,
		CKvMatrixFloat &out_L_image,
		CKvMatrixFloat &out_A_image,
		CKvMatrixFloat &out_B_image);
	void ccvrl_Convert_Color_Value_RGB_to_LAB(unsigned char in_sR, unsigned char in_sG, unsigned char in_sB,
		double &out_l_star, double &out_a_star, double &out_b_star);
	void ccvrx_Convert_Color_Value_RGB_to_XYZ(unsigned char in_sR, unsigned char in_sG, unsigned char in_sB,
		double &out_X, double &out_Y, double &out_Z);

	void ribi_Resize_Image_using_Bilinear_Interpolation(CKvMatrixUchar &in_img_gray,
		int in_new_ww,
		int in_new_hh,
		CKvMatrixUchar &out_resized_img_gray);

	void ribi_Resize_Image_using_Bilinear_Interpolation(CKvMatrixUcharRgb &in_img,
		int in_new_ww,
		int in_new_hh,
		CKvMatrixUcharRgb &out_resized_img);

	void gplgm_Get_Pixel_with_Largest_Gradient_Magnitudes(CKvMatrixFloat &in_grad_mag_image,
		int in_x, int in_y,
		int in_size_of_roi,
		int &out_x, int &out_y);

	void hfac_Hole_Filling_using_Average_Color(CKvMatrixUcharRgb &io_image_color);

	void dmf_Display_Matrix_Float(CKvScreen *in_screen,
		CKvMatrixFloat &in_matrix,
		int in_normalized_mode);

	void dmf_Display_Matrix_Float(CKvScreen *in_screen,
		CKvMatrixFloat &in_matrix,
		float in_threshold);

	void dmfpc_Display_Matrix_Float_using_Pseudo_Color(CKvScreen *in_screen,
		CKvMatrixFloat &in_matrix,
		int in_normalized_mode);

	void deh_Display_Edge_Histogram_XY(CKvScreen *in_screen,
		CKvMatrixFloat &in_edge_mag_image,
		float in_edge_mag_threshold);

	
};