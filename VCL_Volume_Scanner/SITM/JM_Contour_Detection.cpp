#pragma once
#include "stdafx.h"
#include "JM_Contour_Detection.h"

//********************************************************************************************
CKvHistogram::CKvHistogram()
//********************************************************************************************
{
	zz_hist_bin_num = 0;
	zz_histogram = NULL;
}

//********************************************************************************************
CKvHistogram::~CKvHistogram()
//********************************************************************************************
{
	if(zz_histogram)		delete[] zz_histogram;
}

//********************************************************************************************
float* CKvHistogram::c_Create(int in_bin_number)
//********************************************************************************************
{
	if(zz_histogram)		delete[] zz_histogram;

	zz_hist_bin_num = in_bin_number;
	zz_histogram = new float[in_bin_number];
	for(int i=0; i<in_bin_number; i++)		
		zz_histogram[i] = 0.0f;

	return zz_histogram;
}

//********************************************************************************************
float* CKvHistogram::im_Import(float *in_histogram_pointer,
							   int in_bin_number)
//********************************************************************************************
{
	if(zz_histogram)		delete[] zz_histogram;
	
	zz_hist_bin_num = in_bin_number;
	zz_histogram = new float[in_bin_number];
	for(int i=0; i<in_bin_number; i++)		
		zz_histogram[i] = in_histogram_pointer[i];

	return zz_histogram;
}

//********************************************************************************************
float* CKvHistogram::hp_Histogram_Pointer()
//********************************************************************************************
{
	if(zz_hist_bin_num==0){
		Kv_Printf("[CKvHistogram::hp_Histogram_Pointer]\nNon-positive vector size");
		exit(0);
	}
	else		return zz_histogram;
}

//*********************************************************************************************************
float* CKvHistogram::cnhoi_Compute_Normalized_Histogram_Of_Intensity(
	CKvMatrixUchar *in_gray_image,
	int in_number_of_bins)
//*********************************************************************************************************
{
 	int i,k,ww, hh;
 	unsigned char *p_in_gray_image;
 	float step_bins, lower_bound, upper_bound, temp_inten;
  	//Initialization
 	if(in_number_of_bins <= 0){Kv_Printf("[CKvSuperPixel::cnih_Compute_Normalized_Intensity_Histogram]\nNon-positive vector size");exit(0);}
 	if(zz_hist_bin_num > 0){delete[] zz_histogram;zz_hist_bin_num = 0;}
 	zz_hist_bin_num = in_number_of_bins;
 	step_bins       = 256.0f/(float)zz_hist_bin_num;
 	zz_histogram    = new float[zz_hist_bin_num];		for(i=0; i<zz_hist_bin_num; i++)		zz_histogram[i] = 0.0f;
 	p_in_gray_image = in_gray_image->mps(ww, hh)[0];
 	// Calculate histogram for gray image.
	for(i=0; i<ww*hh; i++){
		k=p_in_gray_image[i]/step_bins;
		zz_histogram[k] += 1.0f;
	}
// 	for(k=0; k<zz_hist_bin_num; k++){
// 		lower_bound = k*step_bins;		upper_bound = lower_bound+step_bins;
// 		if(k == zz_hist_bin_num-1)		upper_bound = 256.0f;
// 		for(i=0; i<ww*hh; i++){
// 			temp_inten = p_in_gray_image[i];
// 			if(temp_inten >= lower_bound && temp_inten < upper_bound){
// 				zz_histogram[k] += 1.0f;
// 			}
// 		}
// 	}
	// normalization.
	for(i=0; i<zz_hist_bin_num; i++)	zz_histogram[i] = zz_histogram[i]/(float)(ww*hh);
	return zz_histogram;
}

//*********************************************************************************************************
float* CKvHistogram::cnhog_Compute_Normalized_Histogram_Of_Gradient(
	CKvMatrixFloat *in_edge_mag_image,
	CKvMatrixFloat *in_edge_ori_image,
	int in_number_of_bins)
//*********************************************************************************************************
{
	int i,k,ww, hh;
	float *p_in_edge_mag_image, *p_in_edge_ori_image;
	float step_bins, lower_bound, upper_bound, temp_ori, temp_mag, total_sum_of_mag;
	//Initialization
 	if(in_number_of_bins <= 0){Kv_Printf("[CKvHistogram::cnhog_Compute_Normalized_Histogram_Of_Gradient]\nNon-positive vector size");exit(0);}
 	if(zz_hist_bin_num <= 0)												zz_histogram    = new float[in_number_of_bins];		
 	else	if(zz_hist_bin_num!=in_number_of_bins){		delete[] zz_histogram;		zz_histogram    = new float[in_number_of_bins];		}

 	zz_hist_bin_num = in_number_of_bins;
 	step_bins       = 2.0f*PI/(float)zz_hist_bin_num;	
 	for(i=0; i<zz_hist_bin_num; i++)		zz_histogram[i] = 0.0f;
	
	p_in_edge_mag_image = in_edge_mag_image->mps(ww, hh)[0];
	p_in_edge_ori_image = in_edge_ori_image->vp();
	
	// Calculate histogram for gray image.
	total_sum_of_mag = 0.0f;
	for(i=0; i<ww*hh; i++){
		k=p_in_edge_ori_image[i]/step_bins;
		zz_histogram[k] += p_in_edge_mag_image[i];
		total_sum_of_mag += p_in_edge_mag_image[i];
	}

// 	total_sum_of_mag = 0.0f;
// 	for(i=0; i<ww*hh; i++){
// 		temp_ori = p_in_edge_ori_image[i];			
// 		temp_mag = p_in_edge_mag_image[i];	
// 
// 		for(k=0; k<zz_hist_bin_num; k++){
// 			if(k==0)		lower_bound = 0.0f;		
// 			else				lower_bound += step_bins;					
// 			
// 			if(k == zz_hist_bin_num-1)		upper_bound = 2.0f*PI;
// 			else												upper_bound = lower_bound+step_bins;
// 			
// 			if(temp_ori >= lower_bound){
// 				if(temp_ori < upper_bound){
// 					zz_histogram[k] += temp_mag;
// 					total_sum_of_mag += temp_mag;					
// 					break;
// 				}
// 			}
// 		}
// 
// 	}

	// normalization step by me.
	if(total_sum_of_mag < 10)				for(i=0; i<zz_hist_bin_num; i++)	zz_histogram[i] = 0.0f;
	else														for(i=0; i<zz_hist_bin_num; i++)	zz_histogram[i] = zz_histogram[i]/total_sum_of_mag;

	return zz_histogram;
}

//*********************************************************************************************************
float* CKvHistogram::cnhogs_Compute_Normalized_Histogram_Of_Gradient_in_SIFT(
	CKvMatrixFloat *in_edge_mag_image,
	CKvMatrixFloat *in_edge_ori_image,
	int in_number_of_bins)
//*********************************************************************************************************
{
	int i,k,ww, hh;
	float *p_in_edge_mag_image, *p_in_edge_ori_image;
	float step_bins, lower_bound, upper_bound, temp_ori, temp_mag, total_sum_of_mag_square;
	//Initialization
 	if(in_number_of_bins <= 0){Kv_Printf("[CKvHistogram::cnhog_Compute_Normalized_Histogram_Of_Gradient]\nNon-positive vector size");exit(0);}
 	if(zz_hist_bin_num <= 0)												zz_histogram    = new float[in_number_of_bins];		
 	else	if(zz_hist_bin_num!=in_number_of_bins){		delete[] zz_histogram;		zz_histogram    = new float[in_number_of_bins];		}

 	zz_hist_bin_num = in_number_of_bins;
 	step_bins       = 2.0f*PI/(float)zz_hist_bin_num;	
 	for(i=0; i<zz_hist_bin_num; i++)		zz_histogram[i] = 0.0f;
	
	p_in_edge_mag_image = in_edge_mag_image->mps(ww, hh)[0];
	p_in_edge_ori_image = in_edge_ori_image->vp();
	
	// Calculate histogram for gray image.
	for(i=0; i<ww*hh; i++){
		temp_ori = p_in_edge_ori_image[i];			
		temp_mag = p_in_edge_mag_image[i];	

		for(k=0; k<zz_hist_bin_num; k++){
			if(k==0)		lower_bound = 0.0f;		
			else				lower_bound += step_bins;					
			
			if(k == zz_hist_bin_num-1)		upper_bound = 2.0f*PI;
			else												upper_bound = lower_bound+step_bins;
			
			if(temp_ori >= lower_bound){
				if(temp_ori < upper_bound){
					zz_histogram[k] += temp_mag;				
					break;
				}
			}
		}

	}

	// normalization step by Lowe.
	total_sum_of_mag_square = 0.0f;
	for(i=0; i<zz_hist_bin_num; i++)	total_sum_of_mag_square += SQUARE(zz_histogram[i]);
	total_sum_of_mag_square = sqrt(total_sum_of_mag_square);
	for(i=0; i<zz_hist_bin_num; i++)	zz_histogram[i] = zz_histogram[i]/total_sum_of_mag_square;
	
	total_sum_of_mag_square = 0.0f;
	for(i=0; i<zz_hist_bin_num; i++){
		if(zz_histogram[i] >0.2f)		zz_histogram[i] = 0.2f;
		total_sum_of_mag_square += SQUARE(zz_histogram[i]);
	}	
	total_sum_of_mag_square = sqrt(total_sum_of_mag_square);
	for(i=0; i<zz_hist_bin_num; i++)	zz_histogram[i] = zz_histogram[i]/total_sum_of_mag_square;
	
// 	total_sum_of_mag_square = 0.0f;
// 	for(i=0; i<zz_hist_bin_num; i++)	total_sum_of_mag_square+=SQUARE(zz_histogram[i]);
// 	printf("+Vector length: %f\n", sqrt(total_sum_of_mag_square));
	return zz_histogram;
}

//*********************************************************************************************************
float* CKvHistogram::cnhogsb_Compute_Normalized_Histogram_Of_Gradient_of_Sub_Blocks(
	CKvMatrixFloat *in_edge_mag_image,
	CKvMatrixFloat *in_edge_ori_image,
	CKvMatrixFloat *in_sub_block_position_ratios,				// 4xn matrix, position ratio: the top left and the bottom right / image width or height.
	int in_number_of_bins)
//*********************************************************************************************************
{
	int i, j, k, n, ww, hh, idx;
	int block_num, tmp, tl_x, tl_y, br_x, br_y;
	float *p_in_edge_mag_image, *p_in_edge_ori_image;
	float **p_in_sub_block_position_ratios;
	float step_bins, lower_bound, upper_bound, temp_ori, temp_mag, total_sum_of_mag;

	//Initialization
 	if(in_number_of_bins <= 0){Kv_Printf("[CKvHistogram::cnhog_Compute_Normalized_Histogram_Of_Gradient]\nNon-positive vector size");exit(0);}
	
	p_in_edge_mag_image = in_edge_mag_image->mps(ww, hh)[0];
	p_in_edge_ori_image = in_edge_ori_image->vp();
	p_in_sub_block_position_ratios = in_sub_block_position_ratios->mps(block_num, tmp);

 	if(zz_hist_bin_num <= 0)												zz_histogram    = new float[in_number_of_bins*block_num];		
 	else	if(zz_hist_bin_num!=in_number_of_bins*block_num){		delete[] zz_histogram;		zz_histogram    = new float[in_number_of_bins*block_num];		}

 	zz_hist_bin_num = in_number_of_bins*block_num;
 	step_bins       = 2.0f*PI/(float)in_number_of_bins;	
 	for(i=0; i<zz_hist_bin_num; i++)		zz_histogram[i] = 0.0f;	

	// Calculate histogram for gray image.
// 	CKvMatrixUcharRgb temp_img;
// 	CKvHistogram temp_hist;
// 	CKvScreen sc;
// 	temp_hist.c_Create(in_number_of_bins);
	for(n=0; n<block_num; n++){

		total_sum_of_mag = 0.0f;
		tl_x = (int)(p_in_sub_block_position_ratios[0][n]*ww);			tl_y = (int)(p_in_sub_block_position_ratios[1][n]*hh);
		br_x = (int)(p_in_sub_block_position_ratios[2][n]*ww);		br_y = (int)(p_in_sub_block_position_ratios[3][n]*hh);
		//printf("(%d, %d) (%d, %d)\n", tl_x, tl_y, br_x, br_y);
		for(j=tl_y; j<br_y; j++){
			for(i=tl_x; i<br_x; i++){
				idx = j*ww+i;
				temp_ori = p_in_edge_ori_image[idx];			
				temp_mag = p_in_edge_mag_image[idx];

				for(k=0; k<in_number_of_bins; k++){
					if(k==0)		lower_bound = 0.0f;		
					else				lower_bound += step_bins;					
					
					if(k == in_number_of_bins-1)		upper_bound = 2.0f*PI;
					else														upper_bound = lower_bound+step_bins;
					
					if(temp_ori >= lower_bound){
						if(temp_ori < upper_bound){
							zz_histogram[n*in_number_of_bins+k] += temp_mag;
							total_sum_of_mag += temp_mag;
							break;
						}
					}
				}

			}

		}
		// normalization.
		if(total_sum_of_mag < 100)			for(i=0; i<in_number_of_bins; i++)	zz_histogram[n*in_number_of_bins+i] = 0.0f;
		else								for(i=0; i<in_number_of_bins; i++)	zz_histogram[n*in_number_of_bins+i] = zz_histogram[n*in_number_of_bins+i]/total_sum_of_mag;

// 		for(i=0; i<in_number_of_bins; i++)	temp_hist.hp_Histogram_Pointer()[i] = zz_histogram[n*in_number_of_bins+i];
// 		temp_hist.ghi_Get_Histogram_Image(&temp_img, false, false);
// 		sc.s_d_Display(&temp_img);
// 		if(!Kv_Printf("#%d block", n+1))	exit(0);
	}
	return zz_histogram;
}

//*********************************************************************************************************
float* CKvHistogram::cnhogsbs_Compute_Normalized_Histogram_Of_Gradient_of_Sub_Blocks_in_SIFT(
	CKvMatrixFloat *in_edge_mag_image,
	CKvMatrixFloat *in_edge_ori_image,
	CKvMatrixFloat *in_sub_block_position_ratios,				// 4xn matrix, position ratio: the top left and the bottom right / image width or height.
	int in_number_of_bins)
//*********************************************************************************************************
{
	int i, j, k, n, ww, hh, idx;
	int block_num, tmp, tl_x, tl_y, br_x, br_y;
	float *p_in_edge_mag_image, *p_in_edge_ori_image;
	float **p_in_sub_block_position_ratios;
	float step_bins, lower_bound, upper_bound, temp_ori, temp_mag, total_sum_of_mag_square;

	//Initialization
 	if(in_number_of_bins <= 0){Kv_Printf("[CKvHistogram::cnhog_Compute_Normalized_Histogram_Of_Gradient]\nNon-positive vector size");exit(0);}
	
	p_in_edge_mag_image = in_edge_mag_image->mps(ww, hh)[0];
	p_in_edge_ori_image = in_edge_ori_image->vp();
	p_in_sub_block_position_ratios = in_sub_block_position_ratios->mps(block_num, tmp);

 	if(zz_hist_bin_num <= 0)												zz_histogram    = new float[in_number_of_bins*block_num];		
 	else	if(zz_hist_bin_num!=in_number_of_bins*block_num){		delete[] zz_histogram;		zz_histogram    = new float[in_number_of_bins*block_num];		}

 	zz_hist_bin_num = in_number_of_bins*block_num;
 	step_bins       = 2.0f*PI/(float)in_number_of_bins;	
 	for(i=0; i<zz_hist_bin_num; i++)		zz_histogram[i] = 0.0f;	

	// Calculate histogram for gray image.
// 	CKvMatrixUcharRgb temp_img;
// 	CKvHistogram temp_hist;
// 	CKvScreen sc;
// 	temp_hist.c_Create(in_number_of_bins);
	for(n=0; n<block_num; n++){

		tl_x = (int)(p_in_sub_block_position_ratios[0][n]*ww);			tl_y = (int)(p_in_sub_block_position_ratios[1][n]*hh);
		br_x = (int)(p_in_sub_block_position_ratios[2][n]*ww);		br_y = (int)(p_in_sub_block_position_ratios[3][n]*hh);
		//printf("(%d, %d) (%d, %d)\n", tl_x, tl_y, br_x, br_y);
		for(j=tl_y; j<br_y; j++){
			for(i=tl_x; i<br_x; i++){
				idx = j*ww+i;
				temp_ori = p_in_edge_ori_image[idx];			
				temp_mag = p_in_edge_mag_image[idx];

				for(k=0; k<in_number_of_bins; k++){
					if(k==0)		lower_bound = 0.0f;		
					else				lower_bound += step_bins;					
					
					if(k == in_number_of_bins-1)		upper_bound = 2.0f*PI;
					else														upper_bound = lower_bound+step_bins;
					
					if(temp_ori >= lower_bound){
						if(temp_ori < upper_bound){
							zz_histogram[n*in_number_of_bins+k] += temp_mag;
							break;
						}
					}
				}

			}

		}
		// normalization.
		total_sum_of_mag_square = 0.0f;
		for(i=0; i<in_number_of_bins; i++)	total_sum_of_mag_square += SQUARE(zz_histogram[n*in_number_of_bins+i]);
		total_sum_of_mag_square = sqrt(total_sum_of_mag_square);
		for(i=0; i<in_number_of_bins; i++)	zz_histogram[n*in_number_of_bins+i] = zz_histogram[n*in_number_of_bins+i]/total_sum_of_mag_square;

		total_sum_of_mag_square = 0.0f;
		for(i=0; i<in_number_of_bins; i++){
			if(zz_histogram[n*in_number_of_bins+i] >0.2f)		zz_histogram[n*in_number_of_bins+i] = 0.2f;
			total_sum_of_mag_square += SQUARE(zz_histogram[n*in_number_of_bins+i]);
		}	
		total_sum_of_mag_square = sqrt(total_sum_of_mag_square);
		for(i=0; i<in_number_of_bins; i++)	zz_histogram[n*in_number_of_bins+i] = zz_histogram[n*in_number_of_bins+i]/total_sum_of_mag_square;

// 		for(i=0; i<in_number_of_bins; i++)	temp_hist.hp_Histogram_Pointer()[i] = zz_histogram[n*in_number_of_bins+i];
// 		temp_hist.ghi_Get_Histogram_Image(&temp_img, false, false);
// 		sc.s_d_Display(&temp_img);
// 		if(!Kv_Printf("#%d block", n+1))	exit(0);
	}
	return zz_histogram;
}

//*********************************************************************************************************
float* CKvHistogram::cnhogvsb_Compute_Normalized_Histogram_of_Oriented_Gradient_of_Valid_Sub_Blocks(
	CKvMatrixFloat *in_edge_mag_image,
	CKvMatrixFloat *in_edge_ori_image,
	CKvMatrixFloat *in_sub_block_position_ratios,				// 4xn matrix, position ratio: the top left and the bottom right / image width or height.
	int in_number_of_bins,
	int in_normalize_mode,
	int in_dominant_gradient_reduction_mode,
	CKvVectorInt *out_valid_block_indices)
//*********************************************************************************************************
{
	int i, j, k, n, ww, hh, idx, cnt, dg_cnt;
	int block_num, tmp, threshold, tl_x, tl_y, br_x, br_y;
	int temp_idx[100];
	float temp_hist[100];
	float *p_in_edge_mag_image, *p_in_edge_ori_image;
	float **p_in_sub_block_position_ratios;
	float step_bins, lower_bound, upper_bound, temp_ori, temp_mag, total_sum_of_mag, total_sum_of_mag_squared, dg_th;

	//Initialization
	if(in_number_of_bins <= 0){Kv_Printf("[CKvHistogram::cnhog_Compute_Normalized_Histogram_Of_Gradient]\nNon-positive vector size");exit(0);}

	p_in_edge_mag_image = in_edge_mag_image->mps(ww, hh)[0];
	p_in_edge_ori_image = in_edge_ori_image->vp();
	p_in_sub_block_position_ratios = in_sub_block_position_ratios->mps(block_num, tmp);

	if(zz_hist_bin_num <= 0)	{	zz_histogram = new float[in_number_of_bins*block_num];		}
	else	if(zz_hist_bin_num!=in_number_of_bins*block_num){		delete[] zz_histogram;		zz_histogram    = new float[in_number_of_bins*block_num];		}
	zz_hist_bin_num = in_number_of_bins*block_num;

	step_bins       = 2.0f*PI/(float)in_number_of_bins;	
	for(i=0; i<zz_hist_bin_num; i++)		zz_histogram[i] = 0.0f;	

	// Calculate histogram for gray image.
	cnt = 0;		
	/// //////////////////////////////////////////////////////////////////////////
	//threshold = (int)(((float)(ww*hh)/(float)block_num)*0.0f);
	dg_th=100.0f;
	/// /////////////////////////////////////////////////////////////////////////
	for(n=0; n<block_num; n++){

		total_sum_of_mag = 0.0f;
		tl_x = (int)(p_in_sub_block_position_ratios[0][n]*ww);		tl_y = (int)(p_in_sub_block_position_ratios[1][n]*hh);
		br_x = (int)(p_in_sub_block_position_ratios[2][n]*ww);	br_y = (int)(p_in_sub_block_position_ratios[3][n]*hh);
		/// //////////////////////////////////////////////////////////////////////////////
		threshold=(int)((float)((br_x-tl_x)*(br_y-tl_y))*0.01f);
		/// //////////////////////////////////////////////////////////////////////////////

		for(k=0; k<in_number_of_bins; k++)		temp_hist[k] = 0.0f;
/// /////////////////////////////////////////////////////////////////////////////////////////
		total_sum_of_mag = 0.0f;			dg_cnt=0;		
		for(j=tl_y; j<br_y; j++){
			for(i=tl_x; i<br_x; i++){
				idx = j*ww+i;
				temp_mag = p_in_edge_mag_image[idx];

				k=p_in_edge_ori_image[idx]/step_bins;
				temp_hist[k] += temp_mag;
				total_sum_of_mag += temp_mag;
				if(temp_mag>=dg_th)	dg_cnt++;
			}
		}		
//		printf("dg_cnt: %d\n", dg_cnt);
// 		for(j=tl_y; j<br_y; j++){
// 			for(i=tl_x; i<br_x; i++){
// 				idx = j*ww+i;
// 				temp_ori = p_in_edge_ori_image[idx];			
// 				temp_mag = p_in_edge_mag_image[idx];
// 
// 				for(k=0; k<in_number_of_bins; k++){
// 					if(k==0)		lower_bound = 0.0f;		
// 					else				lower_bound += step_bins;					
// 
// 					if(k == in_number_of_bins-1)		upper_bound = 2.0f*PI;
// 					else														upper_bound = lower_bound+step_bins;
// 
// 					if(temp_ori >= lower_bound){
// 						if(temp_ori < upper_bound){
// 							temp_hist[k] += temp_mag;
// 							total_sum_of_mag += temp_mag;
// 							break;
// 						}
// 					}
// 				}
// 
// 			}
// 
// 		}

		/// /////////////////////////////////////////////////////////////////////////////////////////
		//if(!Kv_Printf("sum_edge: %f / th: %d", total_sum_of_mag, threshold))		exit(0);
		if(!out_valid_block_indices){	
			/// Validity check.
			if(total_sum_of_mag < MINIMUM_SUM_OF_GRADIENT_MAGNITUDE)
				for(k=0;k<in_number_of_bins;k++)		zz_histogram[n*in_number_of_bins+k] = 0.0f;
			else{
				/// Normalize histogram.
				// compute total sum.
				total_sum_of_mag_squared = 0.0f;
				if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM)	for(k=0;k<in_number_of_bins;k++)		total_sum_of_mag_squared += SQUARE(temp_hist[k]);
				// normalize to 1.
				if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)
					for(k=0;k<in_number_of_bins;k++)		temp_hist[k] = temp_hist[k]/total_sum_of_mag;
				else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM){
					total_sum_of_mag_squared = sqrt(total_sum_of_mag_squared);
					for(k=0;k<in_number_of_bins;k++)		temp_hist[k] = temp_hist[k]/total_sum_of_mag_squared;
				}				
				/// Dominant gradient reduction.
				if(!in_dominant_gradient_reduction_mode){
					for(k=0;k<in_number_of_bins;k++)		zz_histogram[n*in_number_of_bins+k] = temp_hist[k];
				}
				else{				
					total_sum_of_mag = total_sum_of_mag_squared = 0.0f;
					// reduction.
					for(k=0; k<in_number_of_bins; k++){
						if(temp_hist[k] >KV_HIST_DOMINANT_GRADIENT_REDUCTION_RATIO)		
							temp_hist[k] = KV_HIST_DOMINANT_GRADIENT_REDUCTION_RATIO;
						if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)			total_sum_of_mag+=temp_hist[k];
						else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM)	total_sum_of_mag_squared+=SQUARE(temp_hist[k]);
					}					
					// normalization.
					if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)
						for(k=0;k<in_number_of_bins;k++)		zz_histogram[n*in_number_of_bins+k] = temp_hist[k]/total_sum_of_mag;
					else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM){
						total_sum_of_mag_squared = sqrt(total_sum_of_mag_squared);
						for(k=0;k<in_number_of_bins;k++)		zz_histogram[n*in_number_of_bins+k] = temp_hist[k]/total_sum_of_mag_squared;
					}
				}
			}

		}
		else{
			// normalization and validity test.
			if(dg_cnt >= threshold){//if(total_sum_of_mag > threshold){

				/// Normalize histogram.
				// compute total sum.
				total_sum_of_mag_squared = 0.0f;
				if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM)	for(k=0;k<in_number_of_bins;k++)		total_sum_of_mag_squared += SQUARE(temp_hist[k]);
				// normalize to 1.
				if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)
					for(k=0;k<in_number_of_bins;k++)		temp_hist[k] = temp_hist[k]/total_sum_of_mag;
				else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM){
					total_sum_of_mag_squared = sqrt(total_sum_of_mag_squared);
					for(k=0;k<in_number_of_bins;k++)		temp_hist[k] = temp_hist[k]/total_sum_of_mag_squared;
				}
				/// Dominant gradient reduction.
				if(!in_dominant_gradient_reduction_mode){
					for(k=0;k<in_number_of_bins;k++)		zz_histogram[cnt*in_number_of_bins+k] = temp_hist[k];
				}
				else{		
					total_sum_of_mag = total_sum_of_mag_squared = 0.0f;
					// reduction.
					for(k=0; k<in_number_of_bins; k++){
						if(temp_hist[k] >KV_HIST_DOMINANT_GRADIENT_REDUCTION_RATIO)		
							temp_hist[k] = KV_HIST_DOMINANT_GRADIENT_REDUCTION_RATIO;
						if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)			total_sum_of_mag+=temp_hist[k];
						else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM)	total_sum_of_mag_squared+=SQUARE(temp_hist[k]);
					}	
					// normalization.
					if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)
						for(k=0;k<in_number_of_bins;k++)		zz_histogram[cnt*in_number_of_bins+k] = temp_hist[k]/total_sum_of_mag;
					else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM){
						total_sum_of_mag_squared = sqrt(total_sum_of_mag_squared);
						for(k=0;k<in_number_of_bins;k++)		zz_histogram[cnt*in_number_of_bins+k] = temp_hist[k]/total_sum_of_mag_squared;
					}
				}

				temp_idx[cnt] = n;
				cnt++;
			}
		}
	}
printf("- valid block cnt: %d\n", cnt);
printf(" + (");
	if(out_valid_block_indices){
		out_valid_block_indices->c_Create(cnt, 0);
		for(i=0; i<cnt; i++){
			out_valid_block_indices->vp()[i] = temp_idx[i];
			printf("%d, ", temp_idx[i]);
		}
	}
	printf(")\n");

	return zz_histogram;
}

//*********************************************************************************************************
float* CKvHistogram::cnhogsbt_Compute_Normalized_Histogram_of_Oriented_Gradient_of_Sub_Block_Tree(
	CKvMatrixFloat *in_edge_mag_image,
	CKvMatrixFloat *in_edge_ori_image,
	CKvSet_of_MatrixFloat *in_set_of_sub_block_position_ratios,				// 4xn matrix, position ratio: the top left and the bottom right / image width or height.
	int in_number_of_bins,
	int in_normalize_mode,
	int in_dominant_gradient_reduction_mode,
	CKvSet_of_VectorInt *out_set_of_valid_block_indices)
//*********************************************************************************************************
{
	int i, j, k, n, m, ww, hh, idx, cnt, total_cnt, dg_cnt;
	int block_num, max_tree_level, total_block_num, tmp, threshold, tl_x, tl_y, br_x, br_y;
	int temp_idx[100];
	float temp_hist[100];
	float *p_in_edge_mag_image, *p_in_edge_ori_image;
	float **p_in_sub_block_position_ratios;
	float step_bins, lower_bound, upper_bound, temp_ori, temp_mag, total_sum_of_mag, total_sum_of_mag_squared, dg_th;

	//Initialization
	if(in_number_of_bins <= 0){Kv_Printf("[CKvHistogram::cnhog_Compute_Normalized_Histogram_Of_Gradient]\nNon-positive vector size");exit(0);}

	p_in_edge_mag_image = in_edge_mag_image->mps(ww, hh)[0];
	p_in_edge_ori_image = in_edge_ori_image->vp();
	max_tree_level = in_set_of_sub_block_position_ratios->vs();
	out_set_of_valid_block_indices->c_Create(max_tree_level);
	total_block_num=0;	for(i=0; i<max_tree_level; i++)	total_block_num+=in_set_of_sub_block_position_ratios->gpe(i)->mw();	


	//p_in_sub_block_position_ratios = in_sub_block_position_ratios->mps(block_num, tmp);

	if(zz_hist_bin_num <= 0)	{	zz_histogram = new float[in_number_of_bins*total_block_num];		}
	else	if(zz_hist_bin_num!=in_number_of_bins*total_block_num){		delete[] zz_histogram;		zz_histogram    = new float[in_number_of_bins*total_block_num];		}
	zz_hist_bin_num = in_number_of_bins*total_block_num;

	step_bins  = 2.0f*PI/(float)in_number_of_bins;	
	for(i=0; i<zz_hist_bin_num; i++)		zz_histogram[i] = 0.0f;	

	// Calculate histogram for gray image.
	total_cnt = 0;
	/// //////////////////////////////////////////////////////////////////////////
	//threshold = (int)(((float)(ww*hh)/(float)block_num)*0.0f);
	dg_th=30.0f;		/// ////////////// Æ©´× ÇÊ¿ä
	// for display binary edge map.
// 	for(i=0; i<ww*hh; i++){
// 		if(	p_in_edge_mag_image[i]>=dg_th)		p_in_edge_mag_image[i]=255.0f;
// 		else																	p_in_edge_mag_image[i]=0.0f;
// 	}
// 	CKvScreen sc;
// 	sc.s_d_Display(1.0f, 0.0f, in_edge_mag_image);
// 	if(!Kv_Printf("Edge"))		exit(0);
	/// /////////////////////////////////////////////////////////////////////////

	

	for(m=0; m<max_tree_level; m++){

		cnt=0;
		
		block_num = in_set_of_sub_block_position_ratios->gpe(m)->mw();
		p_in_sub_block_position_ratios=in_set_of_sub_block_position_ratios->gpe(m)->mp();

		for(n=0; n<block_num; n++){

			total_sum_of_mag = 0.0f;
			tl_x = (int)(p_in_sub_block_position_ratios[0][n]*ww);		tl_y = (int)(p_in_sub_block_position_ratios[1][n]*hh);
			br_x = (int)(p_in_sub_block_position_ratios[2][n]*ww);	br_y = (int)(p_in_sub_block_position_ratios[3][n]*hh);
			/// //////////////////////////////////////////////////////////////////////////////
			threshold=(int)((float)((br_x-tl_x)*(br_y-tl_y))*0.01f);
			/// //////////////////////////////////////////////////////////////////////////////

			for(k=0; k<in_number_of_bins; k++)		temp_hist[k] = 0.0f;
			/// /////////////////////////////////////////////////////////////////////////////////////////
			total_sum_of_mag = 0.0f;			dg_cnt=0;		
			for(j=tl_y; j<br_y; j++){
				for(i=tl_x; i<br_x; i++){
					idx = j*ww+i;
					temp_mag = p_in_edge_mag_image[idx];

					k=p_in_edge_ori_image[idx]/step_bins;
					temp_hist[k] += temp_mag;
					total_sum_of_mag += temp_mag;
					if(temp_mag>=dg_th)	dg_cnt++;
				}
			}		

			/// /////////////////////////////////////////////////////////////////////////////////////////
			//if(!Kv_Printf("sum_edge: %f / th: %d", total_sum_of_mag, threshold))		exit(0);
			if(!out_set_of_valid_block_indices)		threshold=0;	
		
			// normalization and validity test.
			if(dg_cnt >= threshold){//if(total_sum_of_mag > threshold){

				/// Normalize histogram.
				// compute total sum.
				total_sum_of_mag_squared = 0.0f;
				if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM)	for(k=0;k<in_number_of_bins;k++)		total_sum_of_mag_squared += SQUARE(temp_hist[k]);
				// normalize to 1.
				if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)
					for(k=0;k<in_number_of_bins;k++)		temp_hist[k] = temp_hist[k]/total_sum_of_mag;
				else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM){
					total_sum_of_mag_squared = sqrt(total_sum_of_mag_squared);
					for(k=0;k<in_number_of_bins;k++)		temp_hist[k] = temp_hist[k]/total_sum_of_mag_squared;
				}
				/// Dominant gradient reduction.
				if(!in_dominant_gradient_reduction_mode){
					for(k=0;k<in_number_of_bins;k++)		zz_histogram[total_cnt*in_number_of_bins+k] = temp_hist[k];
				}
				else{		
					total_sum_of_mag = total_sum_of_mag_squared = 0.0f;
					// reduction.
					for(k=0; k<in_number_of_bins; k++){
						if(temp_hist[k] >KV_HIST_DOMINANT_GRADIENT_REDUCTION_RATIO)		
							temp_hist[k] = KV_HIST_DOMINANT_GRADIENT_REDUCTION_RATIO;
						if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)			total_sum_of_mag+=temp_hist[k];
						else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM)	total_sum_of_mag_squared+=SQUARE(temp_hist[k]);
					}	
					// normalization.
					if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)
						for(k=0;k<in_number_of_bins;k++)		zz_histogram[total_cnt*in_number_of_bins+k] = temp_hist[k]/total_sum_of_mag;
					else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM){
						total_sum_of_mag_squared = sqrt(total_sum_of_mag_squared);
						for(k=0;k<in_number_of_bins;k++)		zz_histogram[total_cnt*in_number_of_bins+k] = temp_hist[k]/total_sum_of_mag_squared;
					}
				}

				temp_idx[cnt] = n;
				cnt++;	total_cnt++;
			}

		}

	 printf("- valid block cnt: %d\n", cnt);
	 printf(" + (");
	 	if(out_set_of_valid_block_indices){
	 		out_set_of_valid_block_indices->gpe(m)->c_Create(cnt, 0);
	 		for(i=0; i<cnt; i++){
	 			out_set_of_valid_block_indices->gpe(m)->vp()[i] = temp_idx[i];
	 			printf("%d, ", temp_idx[i]);
	 		}
	 	}
	 	printf(")\n");

	}

	
// printf("- valid block cnt: %d\n", cnt);
// printf(" + (");
// 	if(out_valid_block_indices){
// 		out_valid_block_indices->c_Create(cnt, 0);
// 		for(i=0; i<cnt; i++){
// 			out_valid_block_indices->vp()[i] = temp_idx[i];
// 			printf("%d, ", temp_idx[i]);
// 		}
// 	}
// 	printf(")\n");

	return zz_histogram;
}

//*********************************************************************************************************
float* CKvHistogram::cnhogwsbt_Compute_Normalized_Histogram_of_Oriented_Gradient_of_Weighted_Sub_Block_Tree(
	CKvMatrixFloat *in_edge_mag_image,
	CKvMatrixFloat *in_edge_ori_image,
	CKvSet_of_MatrixFloat *in_set_of_sub_block_position_ratios,				// 4xn matrix, position ratio: the top left and the bottom right / image width or height.
	int in_number_of_bins,
	int in_normalize_mode,
	int in_dominant_gradient_reduction_mode,
	CKvSet_of_VectorFloat *out_set_of_weight_vector)
//*********************************************************************************************************
{
	int i, j, k, n, m, ww, hh, idx, cnt, total_cnt, dg_cnt, total_dg_cnt;
	int block_num, max_tree_level, total_block_num, tmp, threshold, tl_x, tl_y, br_x, br_y;
	int temp_dg_cnt[100];
	float temp_hist[100];
	float temp_dg[100], dg, total_dg;
	float *p_in_edge_mag_image, *p_in_edge_ori_image;
	float **p_in_sub_block_position_ratios, *p_out_weight_vector;
	float step_bins, lower_bound, upper_bound, temp_ori, temp_mag, total_sum_of_mag, total_sum_of_mag_squared, dg_th;

	//Initialization
	if(in_number_of_bins <= 0){Kv_Printf("[CKvHistogram::cnhog_Compute_Normalized_Histogram_Of_Gradient]\nNon-positive vector size");exit(0);}

	p_in_edge_mag_image = in_edge_mag_image->mps(ww, hh)[0];
	p_in_edge_ori_image = in_edge_ori_image->vp();
	max_tree_level = in_set_of_sub_block_position_ratios->vs();
	total_block_num=0;	for(i=0; i<max_tree_level; i++)	total_block_num+=in_set_of_sub_block_position_ratios->gpe(i)->mw();	
	out_set_of_weight_vector->c_Create(max_tree_level);
	for(i=0; i<max_tree_level; i++)	out_set_of_weight_vector->gpe(i)->c_Create(in_set_of_sub_block_position_ratios->gpe(i)->mw(), 0.0f);	

	//p_in_sub_block_position_ratios = in_sub_block_position_ratios->mps(block_num, tmp);

	if(zz_hist_bin_num <= 0)	{	zz_histogram = new float[in_number_of_bins*total_block_num];		}
	else	if(zz_hist_bin_num!=in_number_of_bins*total_block_num){		delete[] zz_histogram;		zz_histogram    = new float[in_number_of_bins*total_block_num];		}
	zz_hist_bin_num = in_number_of_bins*total_block_num;

	step_bins  = 2.0f*PI/(float)in_number_of_bins;	
	for(i=0; i<zz_hist_bin_num; i++)		zz_histogram[i] = 0.0f;	

	// Calculate histogram for gray image.
	total_cnt=total_dg_cnt = 0;
	/// //////////////////////////////////////////////////////////////////////////
	//threshold = (int)(((float)(ww*hh)/(float)block_num)*0.0f);
	dg_th=0.0f;		/// ////////////// Æ©´× ÇÊ¿ä
	// for display binary edge map.
// 	for(i=0; i<ww*hh; i++){
// 		if(	p_in_edge_mag_image[i]>=dg_th)		p_in_edge_mag_image[i]=255.0f;
// 		else																	p_in_edge_mag_image[i]=0.0f;
// 	}
// 	CKvScreen sc;
// 	sc.s_d_Display(1.0f, 0.0f, in_edge_mag_image);
// 	if(!Kv_Printf("Edge"))		exit(0);
	/// /////////////////////////////////////////////////////////////////////////

	

	for(m=0; m<max_tree_level; m++){

		cnt=total_dg_cnt=0;		total_dg=0.0f;
		block_num = in_set_of_sub_block_position_ratios->gpe(m)->mw();
		p_in_sub_block_position_ratios=in_set_of_sub_block_position_ratios->gpe(m)->mp();
		p_out_weight_vector=out_set_of_weight_vector->gpe(m)->vp();

		for(n=0; n<block_num; n++){

			total_sum_of_mag = 0.0f;
			tl_x = (int)(p_in_sub_block_position_ratios[0][n]*ww);		tl_y = (int)(p_in_sub_block_position_ratios[1][n]*hh);
			br_x = (int)(p_in_sub_block_position_ratios[2][n]*ww);	br_y = (int)(p_in_sub_block_position_ratios[3][n]*hh);
			/// //////////////////////////////////////////////////////////////////////////////
			threshold=(int)((float)((br_x-tl_x)*(br_y-tl_y))*0.01f);
			/// //////////////////////////////////////////////////////////////////////////////

			for(k=0; k<in_number_of_bins; k++)		temp_hist[k] = 0.0f;
			/// /////////////////////////////////////////////////////////////////////////////////////////
			total_sum_of_mag = 0.0f;			dg=0.0f;
			for(j=tl_y; j<br_y; j++){
				for(i=tl_x; i<br_x; i++){
					idx = j*ww+i;
					temp_mag = p_in_edge_mag_image[idx];

					k=p_in_edge_ori_image[idx]/step_bins;
					temp_hist[k] += temp_mag;
					total_sum_of_mag += temp_mag;

					if(temp_mag>=dg_th)	dg+=temp_mag;		// dg+=1;//
				}
			}		
			/// /////////////////////////////////////////////////////////////
			temp_dg[n]=dg;							total_dg+=dg;		//		temp_dg[n]=dg_cnt;					total_dg+=dg_cnt;
			/// /////////////////////////////////////////////////////////////////////////////////////////

			/// Normalize histogram.
			// compute total sum.
			total_sum_of_mag_squared = 0.0f;
			if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM)	for(k=0;k<in_number_of_bins;k++)		total_sum_of_mag_squared += SQUARE(temp_hist[k]);
			// normalize to 1.
			if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)
				for(k=0;k<in_number_of_bins;k++){
					temp_hist[k] = temp_hist[k]/total_sum_of_mag;
					/// debugging....
					if(temp_hist[k]<0.0f || temp_hist[k]>1.0f	){
						printf("%f\n", temp_hist[k]);
						system("pause");
					}
					/// debugging....
				}
			else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM){
				total_sum_of_mag_squared = sqrt(total_sum_of_mag_squared);
				for(k=0;k<in_number_of_bins;k++)		temp_hist[k] = temp_hist[k]/total_sum_of_mag_squared;
			}
			/// Dominant gradient reduction.
			if(!in_dominant_gradient_reduction_mode){
				for(k=0;k<in_number_of_bins;k++)		zz_histogram[total_cnt*in_number_of_bins+k] = temp_hist[k];
			}
			else{		
				total_sum_of_mag = total_sum_of_mag_squared = 0.0f;
				// reduction.
				for(k=0; k<in_number_of_bins; k++){
					if(temp_hist[k] >KV_HIST_DOMINANT_GRADIENT_REDUCTION_RATIO)		
						temp_hist[k] = KV_HIST_DOMINANT_GRADIENT_REDUCTION_RATIO;
					if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)			total_sum_of_mag+=temp_hist[k];
					else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM)	total_sum_of_mag_squared+=SQUARE(temp_hist[k]);
				}	
				// normalization.
				if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)
					for(k=0;k<in_number_of_bins;k++)		zz_histogram[total_cnt*in_number_of_bins+k] = temp_hist[k]/total_sum_of_mag;
				else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM){
					total_sum_of_mag_squared = sqrt(total_sum_of_mag_squared);
					for(k=0;k<in_number_of_bins;k++)		zz_histogram[total_cnt*in_number_of_bins+k] = temp_hist[k]/total_sum_of_mag_squared;
				}
			}
			
			total_cnt++;
		}
		//printf("block %d\n", block_num);
		for(i=0; i<block_num; i++){					
			p_out_weight_vector[i]=(float)temp_dg[i]/(float)total_dg;	
			//printf("%5.3f,\n", p_out_weight_vector[i]);
		}
		//printf("\n");

	}

	// Normalize weight vector.
// 	printf("- weight vector cnt: %d (%d)\n", total_cnt, total_block_num);
// 	printf(" + (");
// 	for(i=0; i<total_cnt; i++){		
// 		p_out_weight_vector[i]=(float)temp_dg_cnt[i]/(float)total_dg_cnt;	
// 		printf("%5.2f, ", p_out_weight_vector[i]);
// 	}
// 	printf(")\n");

	
// printf("- valid block cnt: %d\n", cnt);
// printf(" + (");
// 	if(out_valid_block_indices){
// 		out_valid_block_indices->c_Create(cnt, 0);
// 		for(i=0; i<cnt; i++){
// 			out_valid_block_indices->vp()[i] = temp_idx[i];
// 			printf("%d, ", temp_idx[i]);
// 		}
// 	}
// 	printf(")\n");

	return zz_histogram;
}


/// /////////////////////////////////////////// 131016 //////////////////////////////////////////////////////////////////
//*********************************************************************************************************
float* CKvHistogram::csbiqt_Compose_Sub_Blocks_of_Image_using_Quad_Tree(
	CKvMatrixFloat *in_edge_mag_image,
	CKvMatrixFloat *in_edge_ori_image,
	CKvMatrixFloat *io_sub_block_position_ratios,
	int mask_size_of_hist_filter)
//*********************************************************************************************************
{
	CKvMatrixFloat temp_sbpr[3];
	CKvVectorBool sbpr_flag[3];

	int ww, hh;
	int bn, new_bn, tl_x, tl_y, br_x, br_y;
	float tl_xr, tl_yr, br_xr, br_yr;
	float temp_sum[64], th_ratio, edge_mag_th;

	int grid_ww, split_level, tree_level, max_tr_lev, xx ,yy, min_blk_sz, cnt;
	double offset_x, offset_y, step_x, step_y, step_xx, step_yy;
	float mean, var;

	bool *p_sbpr_flag[3];
	float **p_io_sbpr, *p_in_edge_mag_image, *p_temp_sbpr[3];
		
	p_in_edge_mag_image = in_edge_mag_image->mps(ww, hh)[0];
/// //////////////////////////////////////////////////////////////////////////////////////////
		
	// Make sub blocks using quad tree.
	// decide tree level.
	tree_level=1;
	xx=ww/2;	yy=hh/2;
	max_tr_lev=2;
	min_blk_sz=50;
	while(xx>min_blk_sz && yy>min_blk_sz && tree_level<max_tr_lev){
		xx/=2;		yy/=2;
		tree_level++;
	}	
	// create sub block tree.
	//p_sbpr_flag = new bool*[3];		p_temp_sbpr = new float*[3];
	for(int i=0; i<tree_level; i++){
		grid_ww = (int)pow(2.0f, i+1);
		p_temp_sbpr[i] = temp_sbpr[i].c_Create(4, grid_ww*grid_ww)[0];
		p_sbpr_flag[i] = sbpr_flag[i].c_Create(grid_ww*grid_ww, false);
	}
	// set quad tree prameters.
	grid_ww=2;		bn=grid_ww*grid_ww;
	offset_x=(double)mask_size_of_hist_filter/(double)ww;								
	offset_y=(double)mask_size_of_hist_filter/(double)hh;
	step_x =(double)(ww-2*mask_size_of_hist_filter)/(double)ww/(double)grid_ww;		
	step_y =(double)(hh-2*mask_size_of_hist_filter)/(double)hh/(double)grid_ww;

	// set 1,2,3 level sbpr.
	for(int n=0; n<tree_level; n++){
		grid_ww=(int)pow(2.0f, n+1);		bn=grid_ww*grid_ww;
		for(int i=0; i<bn; i++){		

			step_xx = step_x/(double)pow(2.0f, n);
			step_yy = step_y/(double)pow(2.0f, n);

			//printf("%f %f %f %f\n", step_x, step_xx, step_y, step_yy);
			
			p_temp_sbpr[n][0*bn+i]=(double)(i%grid_ww)*step_xx+offset_x;				
			p_temp_sbpr[n][1*bn+i]=(double)(i/grid_ww)*step_yy+offset_y;		
			p_temp_sbpr[n][2*bn+i]=(double)(i%grid_ww+1)*step_xx+offset_x;		
			p_temp_sbpr[n][3*bn+i]=(double)(i/grid_ww+1)*step_yy+offset_y;			

		}
	}

	// tree root initialization.
	split_level = 0;		mean = var = 0.0f;
	/// //////////////////////////////////////////////
	//th_ratio=ww*hh*0.005f;
	th_ratio=ww*hh*2.0f;
	/// //////////////////////////////////////////////
	for(int i=0; i<4; i++)	p_sbpr_flag[0][i] = true;	

 	// start splitting.
 	while(split_level<tree_level-1){
		bn=sbpr_flag[split_level].vs();			grid_ww=(int)sqrt((float)bn);				//grid_ww=(int)pow(2.0f, split_level+1);		bn=grid_ww*grid_ww;		--> Why dead????
 		
 		for(int n=0; n<bn; n++){
 
 			if(p_sbpr_flag[split_level][n]){
 
 				temp_sum[n] =0.0f;
 				tl_xr = p_temp_sbpr[split_level][0*bn+n];		
 				tl_yr = p_temp_sbpr[split_level][1*bn+n];			
 				br_xr = p_temp_sbpr[split_level][2*bn+n];	
 				br_yr = p_temp_sbpr[split_level][3*bn+n];	
 
 				tl_x = (int)(tl_xr*ww);			tl_y = (int)(tl_yr*hh);				br_x = (int)(br_xr*ww);		br_y = (int)(br_yr*hh);
 				for(int j=tl_y; j<br_y; j++){
 					for(int i=tl_x; i<br_x; i++){
 						//if(p_in_edge_mag_image[j*ww+i]>120)		temp_sum[n]+=1.0f;
						temp_sum[n]+=p_in_edge_mag_image[j*ww+i];
 					}
 				}
 				// decision whether to split sub-block or not.
 				edge_mag_th = th_ratio*(br_xr-tl_xr)*(br_xr-tl_xr);
 				if(temp_sum[n]>edge_mag_th){
 					xx = n%grid_ww;		yy = n/grid_ww;
 					p_sbpr_flag[split_level][n] = false;
 					p_sbpr_flag[split_level+1][(2*yy)*(2*grid_ww)+(2*xx)] = true;
 					p_sbpr_flag[split_level+1][(2*yy)*(2*grid_ww)+(2*xx)+1] = true;
 					p_sbpr_flag[split_level+1][(2*yy+1)*(2*grid_ww)+(2*xx)] = true;
 					p_sbpr_flag[split_level+1][(2*yy+1)*(2*grid_ww)+(2*xx)+1] = true;
 				}
  			}		
  		} 
 		split_level++;
 	}

	// count number of blocks.
	new_bn=0;
	for(int n=0; n<tree_level; n++){
		bn=sbpr_flag[n].vs();		
		for(int i=0; i<bn; i++){
			if(p_sbpr_flag[n][i])		new_bn++;
		}
	}
	// create new sbpr.
	p_io_sbpr = io_sub_block_position_ratios->c_Create(4, new_bn);
	cnt=0;
	for(int n=0; n<tree_level; n++){
		bn=sbpr_flag[n].vs();		grid_ww=(int)sqrt((float)bn);
		for(int i=0; i<bn; i++){
			if(p_sbpr_flag[n][i]){
				
				for(int j=0; j<4; j++){
					p_io_sbpr[j][cnt]=p_temp_sbpr[n][j*bn+i];
				}				
				cnt++;
			}
		}
	}

	return p_io_sbpr[0];
}

//*********************************************************************************************************
void CKvHistogram::csbiqt_Compose_Sub_Blocks_of_Image_using_Quad_Tree(
	CKvMatrixFloat *in_edge_mag_image,
	CKvMatrixFloat *in_edge_ori_image,
	int in_minimum_block_size,	
	int in_maximum_tree_level,
	CKvSet_of_MatrixFloat *io_set_of_sub_block_position_ratios,
	int mask_size_of_hist_filter)
//*********************************************************************************************************
{
	CKvMatrixFloat temp_sbpr[3], *temp_out_sbpr;
	CKvVectorBool sbpr_flag[3];

	int ww, hh;
	int bn, new_bn, tl_x, tl_y, br_x, br_y;
	float tl_xr, tl_yr, br_xr, br_yr;
	float temp_sum[64], th_ratio, edge_mag_th;

	int grid_ww, grid_hh, split_level, tree_level, max_tr_lev, xx ,yy, min_blk_sz, cnt;
	double offset_x, offset_y, step_x, step_y, step_xx, step_yy;
	float mean, var;

	bool *p_sbpr_flag[3];
	float **p_io_sbpr, *p_in_edge_mag_image, *p_temp_sbpr[3];
		
	p_in_edge_mag_image = in_edge_mag_image->mps(ww, hh)[0];
/// //////////////////////////////////////////////////////////////////////////////////////////
	
	// Make sub blocks using quad tree.
	// decide tree level.
	tree_level=1;
	if(ww>2*hh)			{		xx=ww/4;	yy=hh/2;	}
	else if(hh>2*ww)	{		xx=ww/2;	yy=hh/4;	}
	else							{		xx=ww/4;	yy=hh/4;	}

	max_tr_lev=in_maximum_tree_level;
	min_blk_sz=in_minimum_block_size;
	while(xx>min_blk_sz && yy>min_blk_sz && tree_level<max_tr_lev){
		xx/=2;		yy/=2;
		tree_level++;
	}	
	// create sub block tree.
	io_set_of_sub_block_position_ratios->c_Create(tree_level);
	for(int i=0; i<tree_level; i++){

		if(ww>2*hh)			{		grid_ww=(int)pow(2.0f, i+1);		grid_hh=(int)pow(2.0f, i);		}
		else if(hh>2*ww)	{		grid_ww=(int)pow(2.0f, i);			grid_hh=(int)pow(2.0f, i+1);		}
		else							{		grid_ww=grid_hh=(int)pow(2.0f, i+1);	}

		p_temp_sbpr[i] = temp_sbpr[i].c_Create(4, grid_ww*grid_hh)[0];
		p_sbpr_flag[i] = sbpr_flag[i].c_Create(grid_ww*grid_hh, false);
	}
	// set quad tree prameters.
	if(ww>2*hh)			{		grid_ww=2;		grid_hh=1;		}
	else if(hh>2*ww)	{		grid_ww=1;		grid_hh=2;		}
	else							{		grid_ww=2;		grid_hh=2;		}

	bn=grid_ww*grid_hh;
	offset_x=(double)mask_size_of_hist_filter/(double)ww;								
	offset_y=(double)mask_size_of_hist_filter/(double)hh;
	step_x =(double)(ww-2*mask_size_of_hist_filter)/(double)ww/(double)grid_ww;		
	step_y =(double)(hh-2*mask_size_of_hist_filter)/(double)hh/(double)grid_hh;

	// set 1,2,3 level sbpr.
	for(int n=0; n<tree_level; n++){

		if(ww>2*hh)			{		grid_ww=(int)pow(2.0f, n+1);		grid_hh=(int)pow(2.0f, n);		}
		else if(hh>2*ww)	{		grid_ww=(int)pow(2.0f, n);			grid_hh=(int)pow(2.0f, n+1);		}
		else							{		grid_ww=grid_hh=(int)pow(2.0f, n+1);	}
		
		bn=grid_ww*grid_hh;
		step_xx = step_x/(double)pow(2.0f, n);	step_yy = step_y/(double)pow(2.0f, n);
		for(int i=0; i<bn; i++){		
			p_temp_sbpr[n][0*bn+i]=(double)(i%grid_ww)*step_xx+offset_x;				
			p_temp_sbpr[n][1*bn+i]=(double)(i/grid_ww)*step_yy+offset_y;		
			p_temp_sbpr[n][2*bn+i]=(double)(i%grid_ww+1)*step_xx+offset_x;		
			p_temp_sbpr[n][3*bn+i]=(double)(i/grid_ww+1)*step_yy+offset_y;			

		}
	}

	// tree root initialization.
	while(tree_level>0){
		split_level = 0;		mean = var = 0.0f;
		/// //////////////////////////////////////////////
		//th_ratio=ww*hh*0.005f;
		th_ratio=ww*hh*2.0f;
		/// //////////////////////////////////////////////
		for(int i=0; i<4; i++)	p_sbpr_flag[0][i] = true;	

		// start splitting.
		while(split_level<tree_level-1){
			bn=sbpr_flag[split_level].vs();			grid_ww=(int)sqrt((float)bn);				//grid_ww=(int)pow(2.0f, split_level+1);		bn=grid_ww*grid_ww;		--> Why dead????

			for(int n=0; n<bn; n++){

				if(p_sbpr_flag[split_level][n]){

					temp_sum[n] =0.0f;
					tl_xr = p_temp_sbpr[split_level][0*bn+n];		
					tl_yr = p_temp_sbpr[split_level][1*bn+n];			
					br_xr = p_temp_sbpr[split_level][2*bn+n];	
					br_yr = p_temp_sbpr[split_level][3*bn+n];	

					tl_x = (int)(tl_xr*ww);			tl_y = (int)(tl_yr*hh);				br_x = (int)(br_xr*ww);		br_y = (int)(br_yr*hh);
					for(int j=tl_y; j<br_y; j++){
						for(int i=tl_x; i<br_x; i++){
							//if(p_in_edge_mag_image[j*ww+i]>120)		temp_sum[n]+=1.0f;
							temp_sum[n]+=p_in_edge_mag_image[j*ww+i];
						}
					}
					// decision whether to split sub-block or not.
					edge_mag_th = th_ratio*(br_xr-tl_xr)*(br_xr-tl_xr);
					if(temp_sum[n]>edge_mag_th){
						xx = n%grid_ww;		yy = n/grid_ww;
						p_sbpr_flag[split_level][n] = false;
						p_sbpr_flag[split_level+1][(2*yy)*(2*grid_ww)+(2*xx)] = true;
						p_sbpr_flag[split_level+1][(2*yy)*(2*grid_ww)+(2*xx)+1] = true;
						p_sbpr_flag[split_level+1][(2*yy+1)*(2*grid_ww)+(2*xx)] = true;
						p_sbpr_flag[split_level+1][(2*yy+1)*(2*grid_ww)+(2*xx)+1] = true;
					}
				}		
			} 
			split_level++;
		}

		// count number of blocks.
		new_bn=0;
		for(int n=0; n<tree_level; n++){
			bn=sbpr_flag[n].vs();		
			for(int i=0; i<bn; i++){
				if(p_sbpr_flag[n][i])		new_bn++;
			}
		}
		// create new sbpr.
		p_io_sbpr = io_set_of_sub_block_position_ratios->gpe(tree_level-1)->c_Create(4, new_bn);
		cnt=0;
		for(int n=0; n<tree_level; n++){
			bn=sbpr_flag[n].vs();		grid_ww=(int)sqrt((float)bn);
			for(int i=0; i<bn; i++){
				if(p_sbpr_flag[n][i]){

					for(int j=0; j<4; j++){
						p_io_sbpr[j][cnt]=p_temp_sbpr[n][j*bn+i];
					}				
					cnt++;
				}
			}
		}

		tree_level--;

	}
/// //////////////////////////////////////////////////////////////////////////////////////////
}

/// /////////////////////////////////////////// 131016 //////////////////////////////////////////////////////////////////

//*********************************************************************************************************
void CKvHistogram::mihoi_Make_Integral_Histogram_Of_Intensity(	
	CKvMatrixUchar *in_gray_image,
	int in_number_of_bins,
	CKvSet_of_MatrixInt *out_integral_Histogram)
//*********************************************************************************************************
{
	CKvMatrixInt *p_intgral_Hist;
	int **pp_integral_Hist,cnt,i,j,k,ww, hh;
	unsigned char **p_in_gray_image;
	float step_bins, lower_bound, upper_bound, temp_inten;

	//Initialization
	step_bins            = 256.0f/(float)in_number_of_bins;
	p_intgral_Hist       = out_integral_Histogram->c_Create(in_number_of_bins);
	p_in_gray_image      = in_gray_image->mps(ww, hh);
	for(k=0;k<in_number_of_bins;k++) pp_integral_Hist = p_intgral_Hist[k].c_Create(hh,ww,(int)0);

	//Designate each pixel into a certain bin
	for(k=0; k<in_number_of_bins; k++){
		pp_integral_Hist= p_intgral_Hist[k].mp();
		lower_bound = k*step_bins;		upper_bound = lower_bound+step_bins;
		if(k == in_number_of_bins-1)	upper_bound = 256.0f;
		for(j=0; j<hh;j++){
			for(i=0;i<ww;i++){
				temp_inten = p_in_gray_image[j][i];
				if(temp_inten >= lower_bound && temp_inten < upper_bound){pp_integral_Hist[j][i] = 1;}
			}
		}
	}
	//Make Integral Image of each bin
	for(k=0;k<in_number_of_bins;k++){
		pp_integral_Hist= p_intgral_Hist[k].mp();
		cnt = 0;
		for(i=0; i<ww; i++){cnt = cnt + pp_integral_Hist[0][i];	pp_integral_Hist[0][i] = cnt;}
		cnt = 0;
		for(i=0; i<hh; i++){cnt += pp_integral_Hist[i][0];pp_integral_Hist[i][0] = cnt;}
		for(j=1; j<hh; j++){
			for(i=1; i<ww; i++){
				pp_integral_Hist[j][i] = pp_integral_Hist[j-1][i]+ pp_integral_Hist[j][i-1]- pp_integral_Hist[j-1][i-1]+ pp_integral_Hist[j][i];	
			}
		}
	}
}

//*********************************************************************************************************
void CKvHistogram::mihoie_Make_Integral_Histogram_Of_Intensity_Efficeint(	
	CKvMatrixUchar *in_gray_image,
	int in_max_window_area,
	int in_number_of_bins,
	CKvSet_of_MatrixInt *out_integral_Histogram)
//*********************************************************************************************************
{
	CKvMatrixInt *p_intgral_Hist;
	int **pp_integral_Hist,cnt,i,j,k,ww, hh;
	unsigned char **p_in_gray_image;
	float step_bins, lower_bound, upper_bound, temp_inten;

	//Initialization
	step_bins            = 256.0f/(float)in_number_of_bins;
	p_intgral_Hist       = out_integral_Histogram->c_Create(in_number_of_bins);
	p_in_gray_image      = in_gray_image->mps(ww, hh);
	for(k=0;k<in_number_of_bins;k++) pp_integral_Hist = p_intgral_Hist[k].c_Create(hh,ww,(int)0);

	//Designate each pixel into a certain bin
	for(k=0; k<in_number_of_bins; k++){
		pp_integral_Hist= p_intgral_Hist[k].mp();
		lower_bound = k*step_bins;		upper_bound = lower_bound+step_bins;
		if(k == in_number_of_bins-1)	upper_bound = 256.0f;
		for(j=0; j<hh;j++){
			for(i=0;i<ww;i++){
				temp_inten = p_in_gray_image[j][i];
				if(temp_inten >= lower_bound && temp_inten < upper_bound){pp_integral_Hist[j][i] = 1;}
			}
		}
	}
	//Make Integral Image of each bin
	for(k=0;k<in_number_of_bins;k++){
		pp_integral_Hist= p_intgral_Hist[k].mp();
		cnt = 0;
		for(i=0; i<ww; i++){cnt = cnt + pp_integral_Hist[0][i];	pp_integral_Hist[0][i] = cnt;}
		cnt = 0;
		for(i=0; i<hh; i++){cnt += pp_integral_Hist[i][0];pp_integral_Hist[i][0] = cnt;}
		for(j=1; j<hh; j++){
			for(i=1; i<ww; i++){
				pp_integral_Hist[j][i] = (pp_integral_Hist[j-1][i]+ pp_integral_Hist[j][i-1]- pp_integral_Hist[j-1][i-1]+ pp_integral_Hist[j][i])%in_max_window_area;	
			}
		}
	}
}

//*********************************************************************************************************
void CKvHistogram::mihog_Make_Integral_Histogram_Of_Gradient(	
	CKvMatrixFloat *in_edge_mag_image,
	CKvMatrixFloat *in_edge_ori_image,
	int in_number_of_bins,
	CKvSet_of_MatrixInt *out_integral_Histogram)
//*********************************************************************************************************
{
	CKvMatrixInt *p_intgral_Hist;
	int *pp_integral_Hist,cnt,i,j,k,ww, hh;
	float *p_in_edge_mag_image, *p_in_edge_ori_image;
	float step_bins, lower_bound, upper_bound, temp_ori;

	//Initialization
	step_bins            = 2.0f*PI/(float)in_number_of_bins;
	p_intgral_Hist       = out_integral_Histogram->c_Create(in_number_of_bins);
	p_in_edge_mag_image      = in_edge_mag_image->mps(ww, hh)[0];
	p_in_edge_ori_image			= in_edge_ori_image->mp()[0];
	for(k=0;k<in_number_of_bins;k++) p_intgral_Hist[k].c_Create(hh,ww,(int)0);
/// //////////////////////////////////////////////////////////////////////////////////////////////
	//Designate each pixel into a certain bin
	for(i=0; i<ww*hh; i++){
		k=p_in_edge_ori_image[i]/step_bins;
		p_intgral_Hist[k].mp()[0][i] += (int)p_in_edge_mag_image[i];
	}
// 	for(k=0; k<in_number_of_bins; k++){
// 		pp_integral_Hist= p_intgral_Hist[k].mp()[0];
// 		lower_bound = k*step_bins;		upper_bound = lower_bound+step_bins;
// 		if(k == in_number_of_bins-1)	upper_bound = 2.0f*PI;
// 		for(i=0; i<ww*hh; i++){
// 				temp_ori = p_in_edge_ori_image[i];
// 				if(temp_ori >= lower_bound && temp_ori < upper_bound){pp_integral_Hist[i] = (int)p_in_edge_mag_image[i];}
// 		}
// 	}
/// //////////////////////////////////////////////////////////////////////////////////////////////
	//Make Integral Image of each bin
	for(k=0;k<in_number_of_bins;k++){
		pp_integral_Hist= p_intgral_Hist[k].mp()[0];
		cnt = 0;
		for(i=0; i<ww; i++){cnt += pp_integral_Hist[i];		pp_integral_Hist[i] = cnt;}
		cnt = 0;
		for(i=0; i<hh; i++){cnt += pp_integral_Hist[i*ww];		pp_integral_Hist[i*ww] = cnt;}
		for(j=1; j<hh; j++){
			for(i=1; i<ww; i++){
				pp_integral_Hist[j*ww+i] = pp_integral_Hist[(j-1)*ww+i]+ pp_integral_Hist[j*ww+i-1]- pp_integral_Hist[(j-1)*ww+i-1]+ pp_integral_Hist[j*ww+i];	
			}
		}
	}

}

//*********************************************************************************************************
void CKvHistogram::msirhoi_Make_Set_of_Rotated_Integral_Histograms_Of_Intensity(	
	CKvMatrixUchar *in_gray_image,
	int in_number_of_bins,
	int in_number_of_rotations,
	vector<CKvSet_of_MatrixInt> &out_integral_Histogram,
	CKvSet_of_Pixel &out_offsets)
//*********************************************************************************************************
{
	CKvMatrixInt *p_intgral_Hist;
	CKvMatrixUchar rotated_gray_image;
	int *pp_integral_Hist;
	int cnt, i, j, k, n;
	int ww, hh, ww_r, hh_r;
	unsigned char *p_in_gray_image, *p_rotated_gray_image;
	float step_bins, lower_bound, upper_bound, temp_inten;
	float cos_theta, sin_theta, theta;
	int x, y, max_x, max_y, min_x, min_y;
	int offset_x, offset_y;

	out_integral_Histogram.clear();
	out_integral_Histogram.resize(in_number_of_rotations);
	out_offsets.c_Create(in_number_of_rotations);

	step_bins            = 256.0f/(float)in_number_of_bins;
	p_in_gray_image      = in_gray_image->mps(ww, hh)[0];
	for(n=0; n<in_number_of_rotations; n++){

		/// Initialization
		// find warping region.
		if(n>0){
			theta = PI*(float)n/(float)in_number_of_rotations;			// clockwise.
			cos_theta = cos(theta);		sin_theta = sin(theta);
			max_x = max_y = min_x = min_y = 0;
			x = (int)((ww-1)*cos_theta);		y = (int)((ww-1)*sin_theta);		if(max_x < x)  max_x = x;		if(max_y < y)	max_y = y;		if(min_x > x)	min_x = x;		if(min_y > y)	min_y = y;
			x = (int)(-(hh-1)*sin_theta);			y = (int)((hh-1)*cos_theta);		if(max_x < x)  max_x = x;		if(max_y < y)	max_y = y;		if(min_x > x)	min_x = x;		if(min_y > y)	min_y = y;
			x = (int)((ww-1)*cos_theta - (hh-1)*sin_theta);		y = (int)((ww-1)*sin_theta+(hh-1)*cos_theta);		if(max_x < x)  max_x = x;		if(max_y < y)	max_y = y;		if(min_x > x)	min_x = x;		if(min_y > y)	min_y = y;

			ww_r = max_x - min_x+1;	hh_r = max_y - min_y+1;
			offset_x = -min_x;		offset_y = -min_y;

			out_offsets.gpe_Get_Pointer_of_Element(n)->x = offset_x;
			out_offsets.gpe_Get_Pointer_of_Element(n)->y = offset_y;
			//Kv_Printf("%f %f %f %f", max_x, min_x, max_y, min_y);			
		}
		else{		
			ww_r = ww;	hh_r = hh;		
			out_offsets.gpe_Get_Pointer_of_Element(n)->x = 0;
			out_offsets.gpe_Get_Pointer_of_Element(n)->y = 0;
		}

 		p_intgral_Hist       = out_integral_Histogram[n].c_Create(in_number_of_bins);		
 		p_rotated_gray_image = rotated_gray_image.c_Create(hh_r, ww_r, (unsigned char)0)[0];
		for(k=0;k<in_number_of_bins;k++) p_intgral_Hist[k].c_Create(hh_r,ww_r,(int)0);

		/// rotate input gray image.						== 40 ms
		// inverse mapping.
		if(n>0){
			for(j=0; j<hh_r; j++){
				for(i=0; i<ww_r; i++){

///////////////////////////////////////////////////////////////////////////////////////////
					i -= offset_x;		j -= offset_y;
					x = (int)(i*cos_theta + j*sin_theta);		
					y = (int)(-i*sin_theta + j*cos_theta);
					i += offset_x;		j += offset_y;
///////////////////////////////////////////////////////////////////////////////////////////

					if(x >= 0 && x <= ww-1 && y >= 0 && y <= hh-1)
						p_rotated_gray_image[j*ww_r+i] = p_in_gray_image[y*ww+x];
				}
			}
		}
		else		rotated_gray_image.cp_Copy(in_gray_image);

		///Designate each pixel into a certain bin		== 90 ms
		for(j=0; j<hh_r;j++){
			for(i=0;i<ww_r;i++){
				temp_inten = p_rotated_gray_image[j*ww_r+i];
				if(temp_inten){
					for(k=0; k<in_number_of_bins; k++){
						lower_bound = k*step_bins;		upper_bound = lower_bound+step_bins;
						if(k == in_number_of_bins-1)	upper_bound = 256.0f;
						if(temp_inten >= lower_bound && temp_inten < upper_bound){p_intgral_Hist[k].vp()[j*ww_r+i] = 1;		break;}
					}
				}
			}
		}
		///Make Integral Image of each bin			== 180 ms
		for(k=0;k<in_number_of_bins;k++){
			pp_integral_Hist= p_intgral_Hist[k].vp();
			cnt = 0;		for(i=0; i<ww_r; i++){cnt = cnt + pp_integral_Hist[i];		pp_integral_Hist[i] = cnt;}
			cnt = 0;		for(j=0; j<hh_r; j++){cnt += pp_integral_Hist[j*ww_r];		pp_integral_Hist[j*ww_r] = cnt;}
			for(j=1; j<hh_r; j++){
				for(i=1; i<ww_r; i++){
					pp_integral_Hist[j*ww_r+i] = pp_integral_Hist[(j-1)*ww_r+i]+ pp_integral_Hist[j*ww_r+i-1]- pp_integral_Hist[(j-1)*ww_r+i-1]+ pp_integral_Hist[j*ww_r+i];	
				}
			}
		}

// 		CKvScreen zz_sc;
// 		zz_sc.s_d_Display(&rotated_gray_image);
// 		if(!Kv_Printf("KKK")) exit(0);

	}
	
}

//*********************************************************************************************************
void CKvHistogram::gnhih_Get_Normalized_Histogram_from_Integral_Histogram(
	CKvSet_of_MatrixInt *in_integral_hist,
	int in_x,
	int in_y,
	int in_mask_size_ww,
	int in_mask_size_hh,
	float *out_hist_from_integral)
//*********************************************************************************************************
{
	CKvMatrixInt *p_integral_hist;
	int *vp_integral_hist, **pp_integral_hist, num_bins, k, ww, hh;
	int idx1, idx2, idx3, idx4;
	float total_area, temp_val, temp_hist[100];
	//Initialization
	p_integral_hist = in_integral_hist->vps(num_bins); 
	if(num_bins <= 0){Kv_Printf("[CKvHistogram::gnhih_Get_Normalized_Histogram_from_Integral_Histogram]\nNon-positive vector size");exit(0);}
	if(num_bins > 100){Kv_Printf("[CKvHistogram::gnhih_Get_Normalized_Histogram_from_Integral_Histogram]\nMax size of histogram bin is 100");exit(0);}
	//Get normalized histogram from integral histogram
	p_integral_hist[0].ms(ww, hh);
	idx1 = in_y*ww+in_x;
	idx2 = idx1 + in_mask_size_ww;
	idx3 = idx1 + in_mask_size_hh*ww;
	idx4 = idx3 + in_mask_size_ww;
// ****************************************************************************
	total_area=0.0f;
 	for(k=0;k<num_bins;k++){
   		vp_integral_hist = p_integral_hist[k].vp();
   		temp_hist[k] = (float)(vp_integral_hist[idx4]
   						             -vp_integral_hist[idx2]
   						             -vp_integral_hist[idx3]
   						             +vp_integral_hist[idx1]);
 		total_area+=temp_hist[k];
 	}
// ****************************************************************************
	//Normalize histogram.
	if(total_area < 10)				for(k=0;k<num_bins;k++)		out_hist_from_integral[k] = 0.0f;
	else											for(k=0;k<num_bins;k++)		out_hist_from_integral[k] = temp_hist[k]/total_area;
	
}

//*********************************************************************************************************
void CKvHistogram::gnhihs_Get_Normalized_Histogram_from_Integral_Histogram_in_SIFT(
	CKvSet_of_MatrixInt *in_integral_hist,
	int in_x,
	int in_y,
	int in_mask_size_ww,
	int in_mask_size_hh,
	float *out_hist_from_integral)
//*********************************************************************************************************
{
	CKvMatrixInt *p_integral_hist;
	int *vp_integral_hist, **pp_integral_hist, num_bins, i, k, ww, hh;
	int idx1, idx2, idx3, idx4;
	float total_sum_of_mag_square, temp_val, temp_hist[100];
	//Initialization
	p_integral_hist = in_integral_hist->vps(num_bins); 
	if(num_bins <= 0){Kv_Printf("[CKvHistogram::gnhih_Get_Normalized_Histogram_from_Integral_Histogram]\nNon-positive vector size");exit(0);}
	if(num_bins > 100){Kv_Printf("[CKvHistogram::gnhih_Get_Normalized_Histogram_from_Integral_Histogram]\nMax size of histogram bin is 100");exit(0);}
	//Get normalized histogram from integral histogram
	p_integral_hist[0].ms(ww, hh);
	idx1 = in_y*ww+in_x;
	idx2 = idx1 + in_mask_size_ww;
	idx3 = idx1 + in_mask_size_hh*ww;
	idx4 = idx3 + in_mask_size_ww;
// ****************************************************************************
	total_sum_of_mag_square=0.0f;
 	for(k=0;k<num_bins;k++){
   		vp_integral_hist = p_integral_hist[k].vp();
   		temp_hist[k] = (float)(vp_integral_hist[idx4]
   						             -vp_integral_hist[idx2]
   						             -vp_integral_hist[idx3]
   						             +vp_integral_hist[idx1]);
 		total_sum_of_mag_square+=SQUARE(temp_hist[k]);
 	}
// ****************************************************************************

	// normalization step by Lowe.
	total_sum_of_mag_square = sqrt(total_sum_of_mag_square);
	for(i=0; i<num_bins; i++)	temp_hist[i] = temp_hist[i]/total_sum_of_mag_square;

	total_sum_of_mag_square = 0.0f;
	for(i=0; i<num_bins; i++){
		if(temp_hist[i] >0.2f)		temp_hist[i] = 0.2f;
		total_sum_of_mag_square += SQUARE(temp_hist[i]);
	}	
	total_sum_of_mag_square = sqrt(total_sum_of_mag_square);
	for(i=0; i<num_bins; i++)	out_hist_from_integral[i] = temp_hist[i]/total_sum_of_mag_square;
	
}

//*********************************************************************************************************
void CKvHistogram::gnhsbih_Get_Normalized_Histogram_of_Sub_Blocks_from_Integral_Histogram(
	CKvSet_of_MatrixInt *in_integral_hist,
	int in_x,
	int in_y,
	int in_mask_size_ww,
	int in_mask_size_hh,
	CKvMatrixFloat *in_sub_block_position_ratios,
	float *out_hist_from_integral)
//*********************************************************************************************************
{
	CKvMatrixInt *p_integral_hist;
	int *vp_integral_hist, **pp_integral_hist, num_bins, k, n, ww, hh;
	int idx1, idx2, idx3, idx4;
	int block_num, tmp, tl_x, tl_y, br_x, br_y;
	float **p_in_sub_block_position_ratios;
	float total_area, temp_val, temp_hist[100];
	//Initialization
	p_integral_hist = in_integral_hist->vps(num_bins); 
	if(num_bins <= 0){Kv_Printf("[CKvHistogram::gnhih_Get_Normalized_Histogram_from_Integral_Histogram]\nNon-positive vector size");exit(0);}
	if(num_bins > 100){Kv_Printf("[CKvHistogram::gnhih_Get_Normalized_Histogram_from_Integral_Histogram]\nMax size of histogram bin is 100");exit(0);}
	//Get normalized histogram from integral histogram
	p_integral_hist[0].ms(ww, hh);
	p_in_sub_block_position_ratios = in_sub_block_position_ratios->mps(block_num, tmp);

// 	idx1 = in_y*ww+in_x;
// 	idx2 = idx1 + in_mask_size_ww;
// 	idx3 = idx1 + in_mask_size_hh*ww;
// 	idx4 = idx3 + in_mask_size_ww;
// ****************************************************************************
	for(n=0; n<block_num; n++){
		total_area=0.0f;
		tl_x = in_x+(int)(p_in_sub_block_position_ratios[0][n]*in_mask_size_ww);			tl_y = in_y+(int)(p_in_sub_block_position_ratios[1][n]*in_mask_size_hh);
		br_x = in_x+(int)(p_in_sub_block_position_ratios[2][n]*in_mask_size_ww);			br_y = in_y+(int)(p_in_sub_block_position_ratios[3][n]*in_mask_size_hh);

		idx1 = tl_y*ww+tl_x;
		idx2 = tl_y*ww+br_x;
		idx3 = br_y*ww+tl_x;
		idx4 = br_y*ww+br_x;

 		for(k=0;k<num_bins;k++){
   			vp_integral_hist = p_integral_hist[k].vp();
   			temp_hist[k] = (float)(vp_integral_hist[idx4]
   										 -vp_integral_hist[idx2]
   										 -vp_integral_hist[idx3]
   										 +vp_integral_hist[idx1]);
 			total_area+=temp_hist[k];
 		}
	// ****************************************************************************
		//Normalize histogram.
		if(total_area < 100)				for(k=0;k<num_bins;k++)		out_hist_from_integral[n*num_bins+k] = 0.0f;
		else											for(k=0;k<num_bins;k++)		out_hist_from_integral[n*num_bins+k] = temp_hist[k]/total_area;
	}
	
}

//*********************************************************************************************************
void CKvHistogram::gnhsbihs_Get_Normalized_Histogram_of_Sub_Blocks_from_Integral_Histogram_in_SIFT(
	CKvSet_of_MatrixInt *in_integral_hist,
	int in_x,
	int in_y,
	int in_mask_size_ww,
	int in_mask_size_hh,
	CKvMatrixFloat *in_sub_block_position_ratios,
	float *out_hist_from_integral)
//*********************************************************************************************************
{
	CKvMatrixInt *p_integral_hist;
	int *vp_integral_hist, **pp_integral_hist, num_bins, i, k, n, ww, hh;
	int idx1, idx2, idx3, idx4;
	int block_num, tmp, tl_x, tl_y, br_x, br_y;
	float **p_in_sub_block_position_ratios;
	float total_sum_of_mag_square, temp_val, temp_hist[100];
	//Initialization
	p_integral_hist = in_integral_hist->vps(num_bins); 
	if(num_bins <= 0){Kv_Printf("[CKvHistogram::gnhih_Get_Normalized_Histogram_from_Integral_Histogram]\nNon-positive vector size");exit(0);}
	if(num_bins > 100){Kv_Printf("[CKvHistogram::gnhih_Get_Normalized_Histogram_from_Integral_Histogram]\nMax size of histogram bin is 100");exit(0);}
	//Get normalized histogram from integral histogram
	p_integral_hist[0].ms(ww, hh);
	p_in_sub_block_position_ratios = in_sub_block_position_ratios->mps(block_num, tmp);

// 	idx1 = in_y*ww+in_x;
// 	idx2 = idx1 + in_mask_size_ww;
// 	idx3 = idx1 + in_mask_size_hh*ww;
// 	idx4 = idx3 + in_mask_size_ww;
// ****************************************************************************
	for(n=0; n<block_num; n++){
		total_sum_of_mag_square=0.0f;
		tl_x = in_x+(int)(p_in_sub_block_position_ratios[0][n]*in_mask_size_ww);			tl_y = in_y+(int)(p_in_sub_block_position_ratios[1][n]*in_mask_size_hh);
		br_x = in_x+(int)(p_in_sub_block_position_ratios[2][n]*in_mask_size_ww);			br_y = in_y+(int)(p_in_sub_block_position_ratios[3][n]*in_mask_size_hh);

		idx1 = tl_y*ww+tl_x;
		idx2 = tl_y*ww+br_x;
		idx3 = br_y*ww+tl_x;
		idx4 = br_y*ww+br_x;

 		for(k=0;k<num_bins;k++){
   			vp_integral_hist = p_integral_hist[k].vp();
   			temp_hist[k] = (float)(vp_integral_hist[idx4]
   										 -vp_integral_hist[idx2]
   										 -vp_integral_hist[idx3]
   										 +vp_integral_hist[idx1]);
 			total_sum_of_mag_square+=SQUARE(temp_hist[k]);
 		}
	// ****************************************************************************
		// normalization step by Lowe.
		total_sum_of_mag_square = sqrt(total_sum_of_mag_square);
		for(i=0; i<num_bins; i++)	temp_hist[i] = temp_hist[i]/total_sum_of_mag_square;

		total_sum_of_mag_square = 0.0f;
		for(i=0; i<num_bins; i++){
			if(temp_hist[i] >0.2f)		temp_hist[i] = 0.2f;
			total_sum_of_mag_square += SQUARE(temp_hist[i]);
		}	
		total_sum_of_mag_square = sqrt(total_sum_of_mag_square);
		for(i=0; i<num_bins; i++)	out_hist_from_integral[n*num_bins+i] = temp_hist[i]/total_sum_of_mag_square;
	}
	
}

//*********************************************************************************************************
void CKvHistogram::gnhvsbih_Get_Normalized_Histogram_of_Valid_Sub_Blocks_from_Integral_Histogram(
	CKvSet_of_MatrixInt *in_integral_hist,
	int in_x,
	int in_y,
	int in_mask_size_ww,
	int in_mask_size_hh,
	CKvMatrixFloat *in_sub_block_position_ratios,
	CKvVectorInt *in_valid_block_indices,
	int in_normalize_mode,
	int in_dominant_gradient_reduction_mode,
	float *out_hist_from_integral)
//*********************************************************************************************************
{
	CKvMatrixInt *p_integral_hist;
	int i, k, n, ww, hh;
	int *vp_integral_hist, **pp_integral_hist; 
	int num_blocks, num_bins;
	int idx1, idx2, idx3, idx4;
	int block_num, tmp, tl_x, tl_y, br_x, br_y;
	float **p_in_sub_block_position_ratios;
	float total_area, temp_val, threshold, temp_hist[100];
	//Initialization
	p_integral_hist = in_integral_hist->vps(num_bins); 
	if(num_bins <= 0){Kv_Printf("[CKvHistogram::gnhih_Get_Normalized_Histogram_from_Integral_Histogram]\nNon-positive vector size");exit(0);}
	if(num_bins > 100){Kv_Printf("[CKvHistogram::gnhih_Get_Normalized_Histogram_from_Integral_Histogram]\nMax size of histogram bin is 100");exit(0);}
	//Get normalized histogram from integral histogram
	p_integral_hist[0].ms(ww, hh);
	p_in_sub_block_position_ratios = in_sub_block_position_ratios->mp();

// 	idx1 = in_y*ww+in_x;
// 	idx2 = idx1 + in_mask_size_ww;
// 	idx3 = idx1 + in_mask_size_hh*ww;
// 	idx4 = idx3 + in_mask_size_ww;
// ****************************************************************************
	if(in_valid_block_indices){		num_blocks = in_valid_block_indices->vs();				}
	else												num_blocks = in_sub_block_position_ratios->mw();

	for(i=0; i<num_blocks; i++){
		if(in_valid_block_indices)	n = in_valid_block_indices->vp()[i];
		else											n = i;
		tl_x = in_x+(int)(p_in_sub_block_position_ratios[0][n]*in_mask_size_ww);			tl_y = in_y+(int)(p_in_sub_block_position_ratios[1][n]*in_mask_size_hh);
		br_x = in_x+(int)(p_in_sub_block_position_ratios[2][n]*in_mask_size_ww);			br_y = in_y+(int)(p_in_sub_block_position_ratios[3][n]*in_mask_size_hh);

		idx1 = tl_y*ww+tl_x;
		idx2 = tl_y*ww+br_x;
		idx3 = br_y*ww+tl_x;
		idx4 = br_y*ww+br_x;

		total_area=0.0f;
 		for(k=0;k<num_bins;k++){
   			vp_integral_hist = p_integral_hist[k].vp();
   			temp_hist[k] = (float)(vp_integral_hist[idx4]
   										 -vp_integral_hist[idx2]
   										 -vp_integral_hist[idx3]
   										 +vp_integral_hist[idx1]);
 			if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)			total_area+=temp_hist[k];
			else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM)	total_area+=SQUARE(temp_hist[k]);
		}

		//printf("total area: %f\n", total_area);

	// ****************************************************************************
		/// Validity check.
		if(total_area < 50.0f*2.0f*((br_x-tl_x)+(br_y-tl_y)))			
			for(k=0;k<num_bins;k++)		out_hist_from_integral[i*num_bins+k] = 0.0f;
		else{
			/// Normalize histogram.
			if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)
				for(k=0;k<num_bins;k++)		temp_hist[k] = temp_hist[k]/total_area;
			else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM){
				total_area = sqrt(total_area);
				for(k=0;k<num_bins;k++)		temp_hist[k] = temp_hist[k]/total_area;
			}
			/// Dominant gradient reduction.
			if(!in_dominant_gradient_reduction_mode)
				for(k=0;k<num_bins;k++)		out_hist_from_integral[i*num_bins+k] = temp_hist[k];
			else{
				total_area = 0.0f;
				// reduction.
				for(k=0; k<num_bins; k++){
					if(temp_hist[k] >KV_HIST_DOMINANT_GRADIENT_REDUCTION_RATIO)		
						temp_hist[k] = KV_HIST_DOMINANT_GRADIENT_REDUCTION_RATIO;
					if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)			total_area+=temp_hist[k];
					else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM)	total_area+=SQUARE(temp_hist[k]);
				}					
				// normalization.
				if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)
					for(k=0;k<num_bins;k++)		out_hist_from_integral[i*num_bins+k] = temp_hist[k]/total_area;
				else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM){
					total_area = sqrt(total_area);
					for(k=0;k<num_bins;k++)		out_hist_from_integral[i*num_bins+k] = temp_hist[k]/total_area;
				}
			}
		}
		
	}

	//printf("%d %d %f\n", in_mask_size_ww, in_mask_size_hh, out_hist_from_integral[0]);
	
}

	//*********************************************************************************************************
void CKvHistogram::gnhwsbih_Get_Normalized_Histogram_of_Weighted_Sub_Blocks_from_Integral_Histogram(
	CKvSet_of_MatrixInt *in_integral_hist,
	int in_x,
	int in_y,
	int in_mask_size_ww,
	int in_mask_size_hh,
	CKvMatrixFloat *in_sub_block_position_ratios,
	int in_normalize_mode,
	int in_dominant_gradient_reduction_mode,
	float *out_hist_from_integral)
//*********************************************************************************************************
{
	CKvMatrixInt *p_integral_hist;
	int i, k, n, ww, hh;
	int *vp_integral_hist, **pp_integral_hist; 
	int num_blocks, num_bins;
	int idx1, idx2, idx3, idx4;
	int block_num, tmp, tl_x, tl_y, br_x, br_y;
	float **p_in_sub_block_position_ratios;
	float total_area, temp_val, threshold, temp_hist[100];
	//Initialization
	p_integral_hist = in_integral_hist->vps(num_bins); 
	if(num_bins <= 0){Kv_Printf("[CKvHistogram::gnhih_Get_Normalized_Histogram_from_Integral_Histogram]\nNon-positive vector size");exit(0);}
	if(num_bins > 100){Kv_Printf("[CKvHistogram::gnhih_Get_Normalized_Histogram_from_Integral_Histogram]\nMax size of histogram bin is 100");exit(0);}
	//Get normalized histogram from integral histogram
	p_integral_hist[0].ms(ww, hh);
	p_in_sub_block_position_ratios = in_sub_block_position_ratios->mp();

// 	idx1 = in_y*ww+in_x;
// 	idx2 = idx1 + in_mask_size_ww;
// 	idx3 = idx1 + in_mask_size_hh*ww;
// 	idx4 = idx3 + in_mask_size_ww;
// ****************************************************************************
	num_blocks = in_sub_block_position_ratios->mw();

	for(i=0; i<num_blocks; i++){
		n = i;
		tl_x = in_x+(int)(p_in_sub_block_position_ratios[0][n]*in_mask_size_ww);			tl_y = in_y+(int)(p_in_sub_block_position_ratios[1][n]*in_mask_size_hh);
		br_x = in_x+(int)(p_in_sub_block_position_ratios[2][n]*in_mask_size_ww);			br_y = in_y+(int)(p_in_sub_block_position_ratios[3][n]*in_mask_size_hh);

		idx1 = tl_y*ww+tl_x;
		idx2 = tl_y*ww+br_x;
		idx3 = br_y*ww+tl_x;
		idx4 = br_y*ww+br_x;

		total_area=0.0f;
 		for(k=0;k<num_bins;k++){
   			vp_integral_hist = p_integral_hist[k].vp();
   			temp_hist[k] = (float)(vp_integral_hist[idx4]
   										 -vp_integral_hist[idx2]
   										 -vp_integral_hist[idx3]
   										 +vp_integral_hist[idx1]);
 			if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)			total_area+=temp_hist[k];
			else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM)	total_area+=SQUARE(temp_hist[k]);
		}

	// ****************************************************************************
		/// Normalize histogram.
		if(total_area < 50.0f*2.0f*((br_x-tl_x)+(br_y-tl_y)))			
			for(k=0;k<num_bins;k++)		out_hist_from_integral[i*num_bins+k] = 0.0f;
		else{
			if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)
				for(k=0;k<num_bins;k++){
					temp_hist[k] = temp_hist[k]/total_area;
					/// debugging....
					if(temp_hist[k]<0.0f || temp_hist[k]>1.0f	){
						printf("%f\n", temp_hist[k]);
						system("pause");
					}
					/// debugging....
				}
			else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM){
				total_area = sqrt(total_area);
				for(k=0;k<num_bins;k++){
					temp_hist[k] = temp_hist[k]/total_area;
				}
			}
			/// Dominant gradient reduction.
			if(!in_dominant_gradient_reduction_mode)
				for(k=0;k<num_bins;k++)		out_hist_from_integral[i*num_bins+k] = temp_hist[k];
			else{
				total_area = 0.0f;
				// reduction.
				for(k=0; k<num_bins; k++){
					if(temp_hist[k] >KV_HIST_DOMINANT_GRADIENT_REDUCTION_RATIO)		
						temp_hist[k] = KV_HIST_DOMINANT_GRADIENT_REDUCTION_RATIO;
					if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)			total_area+=temp_hist[k];
					else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM)	total_area+=SQUARE(temp_hist[k]);
				}					
				// normalization.
				if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)
					for(k=0;k<num_bins;k++)		out_hist_from_integral[i*num_bins+k] = temp_hist[k]/total_area;
				else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM){
					total_area = sqrt(total_area);
					for(k=0;k<num_bins;k++)		out_hist_from_integral[i*num_bins+k] = temp_hist[k]/total_area;
				}
			}
		}

		
	}

	//printf("%d %d %f\n", in_mask_size_ww, in_mask_size_hh, out_hist_from_integral[0]);
	
}

//*********************************************************************************************************
void CKvHistogram::gnhvsbihe_Get_Normalized_Histogram_of_Valid_Sub_Blocks_from_Integral_Histogram_Efficient(
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
	float *out_hist_from_integral)
//*********************************************************************************************************
{
	CKvMatrixInt *p_integral_hist;
	int i, k, n, ww, hh;
	int *vp_integral_hist, **pp_integral_hist; 
	int num_blocks, num_bins;
	int idx1, idx2, idx3, idx4;
	int block_num, tmp, tl_x, tl_y, br_x, br_y;
	float **p_in_sub_block_position_ratios;
	float total_area, temp_val, temp_hist[100];
	//Initialization
	p_integral_hist = in_integral_hist->vps(num_bins); 
	if(num_bins <= 0){Kv_Printf("[CKvHistogram::gnhih_Get_Normalized_Histogram_from_Integral_Histogram]\nNon-positive vector size");exit(0);}
	if(num_bins > 100){Kv_Printf("[CKvHistogram::gnhih_Get_Normalized_Histogram_from_Integral_Histogram]\nMax size of histogram bin is 100");exit(0);}
	//Get normalized histogram from integral histogram
	p_integral_hist[0].ms(ww, hh);
	p_in_sub_block_position_ratios = in_sub_block_position_ratios->mp();

// 	idx1 = in_y*ww+in_x;
// 	idx2 = idx1 + in_mask_size_ww;
// 	idx3 = idx1 + in_mask_size_hh*ww;
// 	idx4 = idx3 + in_mask_size_ww;
// ****************************************************************************
	if(in_valid_block_indices)		num_blocks = in_valid_block_indices->vs();
	else												num_blocks = in_sub_block_position_ratios->mw();

	for(i=0; i<num_blocks; i++){
		if(in_valid_block_indices)	n = in_valid_block_indices->vp()[i];
		else											n = i;
		tl_x = in_x+(int)(p_in_sub_block_position_ratios[0][n]*in_mask_size_ww);			tl_y = in_y+(int)(p_in_sub_block_position_ratios[1][n]*in_mask_size_hh);
		br_x = in_x+(int)(p_in_sub_block_position_ratios[2][n]*in_mask_size_ww);			br_y = in_y+(int)(p_in_sub_block_position_ratios[3][n]*in_mask_size_hh);

		idx1 = tl_y*ww+tl_x;
		idx2 = tl_y*ww+br_x;
		idx3 = br_y*ww+tl_x;
		idx4 = br_y*ww+br_x;

		total_area=0.0f;
 		for(k=0;k<num_bins;k++){
   			vp_integral_hist = p_integral_hist[k].vp();
   			temp_hist[k] = (float)(vp_integral_hist[idx4]
   										 -vp_integral_hist[idx2]
   										 -vp_integral_hist[idx3]
   										 +vp_integral_hist[idx1]);
            temp_hist[k] = modulus((int)temp_hist[k], in_max_window_area);
 			if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)			total_area+=temp_hist[k];
			else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM)	total_area+=SQUARE(temp_hist[k]);
 		}

	// ****************************************************************************
		/// Validity check.
		if(total_area < MINIMUM_SUM_OF_GRADIENT_MAGNITUDE)			
			for(k=0;k<num_bins;k++)		out_hist_from_integral[i*num_bins+k] = 0.0f;
		else{
			/// Normalize histogram.
			if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)
				for(k=0;k<num_bins;k++)		temp_hist[k] = temp_hist[k]/total_area;
			else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM){
				total_area = sqrt(total_area);
				for(k=0;k<num_bins;k++)		temp_hist[k] = temp_hist[k]/total_area;
			}
			/// Dominant gradient reduction.
			if(!in_dominant_gradient_reduction_mode)
				for(k=0;k<num_bins;k++)		out_hist_from_integral[i*num_bins+k] = temp_hist[k];
			else{
				total_area = 0.0f;
				// reduction.
				for(k=0; k<num_bins; k++){
					if(temp_hist[k] >KV_HIST_DOMINANT_GRADIENT_REDUCTION_RATIO)		
						temp_hist[k] = KV_HIST_DOMINANT_GRADIENT_REDUCTION_RATIO;
					if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)			total_area+=temp_hist[k];
					else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM)	total_area+=SQUARE(temp_hist[k]);
				}					
				// normalization.
				if(in_normalize_mode==KV_HIST_NORMALIZE_L1_NORM)
					for(k=0;k<num_bins;k++)		out_hist_from_integral[i*num_bins+k] = temp_hist[k]/total_area;
				else if(in_normalize_mode==KV_HIST_NORMALIZE_L2_NORM){
					total_area = sqrt(total_area);
					for(k=0;k<num_bins;k++)		out_hist_from_integral[i*num_bins+k] = temp_hist[k]/total_area;
				}
			}
		}
		
	}
	
}

//*********************************************************************************************************
void CKvHistogram::ghi_Get_Histogram_Image(
	CKvMatrixUcharRgb* out_histogram_image,
	bool in_streched_mode,
	int in_color_mode)
//*********************************************************************************************************
{
	int i, j, k, kk, ww, hh, step_size=0, hist_height=0;
	unsigned char *p_out_histogram_image;
	float *temp_hist;

	if(zz_hist_bin_num<=0){Kv_Printf("[CKvSuperPixel::ghip_Get_Histogram_Image_Pointer]\nNon-positive bin size");exit(0);}

	temp_hist = new float[zz_hist_bin_num];

	if(out_histogram_image->mw() == 1 && out_histogram_image->mh() == 1){
		ww = hh = DEFAULT_IMAGE_WIDTH_FOR_HISTOGRAM_IMAGE;
		step_size = ww/zz_hist_bin_num;		
		ww = step_size*zz_hist_bin_num;
		if(in_color_mode != 0)		ww = 3*ww;
		p_out_histogram_image = out_histogram_image->c_Create(hh, ww, Kv_Rgb((unsigned char)255, (unsigned char)255, (unsigned char)255))[0];
	}
	else{
		ww = out_histogram_image->mw();		hh = out_histogram_image->mh();
		step_size = ww/zz_hist_bin_num;	
		if(ww%zz_hist_bin_num>0){	
			if(in_color_mode==0)		ww = step_size*zz_hist_bin_num;
			else										ww = 3*step_size*zz_hist_bin_num;		
			hh = DEFAULT_IMAGE_WIDTH_FOR_HISTOGRAM_IMAGE;
			p_out_histogram_image = out_histogram_image->c_Create(hh, ww, Kv_Rgb((unsigned char)255, (unsigned char)255, (unsigned char)255))[0];
		}		
		else p_out_histogram_image = out_histogram_image->mps(ww, hh)[0];
		for(i=0; i<ww*hh; i++){
			kk=i;
			p_out_histogram_image[kk] = (unsigned char)255;		kk+=ww*hh;
			p_out_histogram_image[kk] = (unsigned char)255;		kk+=ww*hh;
			p_out_histogram_image[kk] = (unsigned char)255;		kk+=ww*hh;
		}
	}

	if(in_streched_mode==true){
		float max_hist=-10000.0;
		for(i=0; i<zz_hist_bin_num; i++)	{
			if(zz_histogram[i] > max_hist)		max_hist = zz_histogram[i];
		}
		for(i=0; i<zz_hist_bin_num; i++)	{
			temp_hist[i] = zz_histogram[i]/max_hist;		
		}
	}
	else{
		for(i=0; i<zz_hist_bin_num; i++)	{
			temp_hist[i] = zz_histogram[i];		
		}
	}
		
	for(i=0; i<zz_hist_bin_num; i++){

		/// ///////////////////////////////////////////////////////////////////////////
		hist_height = (int)(temp_hist[i]*(float)hh);
		/// ///////////////////////////////////////////////////////////////////////////

		for(k=0; k<step_size; k++){

			for(j=hh-1; j>hh-hist_height-1; j--){
 				kk=j*ww+(i*step_size+k);

				p_out_histogram_image[kk] = (unsigned char)255;		kk+=ww*hh;
 				p_out_histogram_image[kk] = (unsigned char)		0;		kk+=ww*hh;
 				p_out_histogram_image[kk] = (unsigned char)		0;		kk+=ww*hh;
			}
		}

		if(in_color_mode != 0){
			if((i+1)%(zz_hist_bin_num/3) == 0){
				for(j=hh-1; j>=0; j--){
					kk=j*ww+(i*step_size+step_size-1);

					p_out_histogram_image[kk] = (unsigned char)		0;		kk+=ww*hh;
					p_out_histogram_image[kk] = (unsigned char)		0;		kk+=ww*hh;
					p_out_histogram_image[kk] = (unsigned char)		255;		kk+=ww*hh;
				}
			}
		}
	}

	delete[] temp_hist;
}

//*********************************************************************************************************
void CKvHistogram::ghi_Get_Histogram_Image(
	CKvMatrixUcharRgb* out_histogram_image,
	int in_ww, int in_hh,
	int in_scaling,
	bool in_streched_mode,
	int in_color_mode)
//*********************************************************************************************************
{
	int i, j, k, kk, ww, hh, step_size=0, hist_height=0;
	unsigned char *p_out_histogram_image;
	float *temp_hist;

	if(zz_hist_bin_num<=0){Kv_Printf("[CKvSuperPixel::ghip_Get_Histogram_Image_Pointer]\nNon-positive bin size");exit(0);}

	temp_hist = new float[zz_hist_bin_num];

	
	ww = in_ww;		hh = in_hh;
	step_size = ww/zz_hist_bin_num;		
	ww = step_size*zz_hist_bin_num;
	if(in_color_mode != 0)		ww = 3*ww;
	p_out_histogram_image = out_histogram_image->c_Create(hh, ww, Kv_Rgb((unsigned char)255, (unsigned char)255, (unsigned char)255))[0];

	if(in_streched_mode==true){
		float max_hist=-10000.0;
		for(i=0; i<zz_hist_bin_num; i++)	{
			if(zz_histogram[i] > max_hist)		max_hist = zz_histogram[i];
		}
		for(i=0; i<zz_hist_bin_num; i++)	{
			temp_hist[i] = zz_histogram[i]/max_hist;		
		}
	}
	else{
		for(i=0; i<zz_hist_bin_num; i++)	{
			temp_hist[i] = zz_histogram[i];		
		}
	}
		
	for(i=0; i<zz_hist_bin_num; i++){

		/// ///////////////////////////////////////////////////////////////////////////
		hist_height = min((int)(temp_hist[i]*(float)hh)*in_scaling, hh-1);
		/// ///////////////////////////////////////////////////////////////////////////

		for(k=0; k<step_size; k++){

			for(j=hh-1; j>hh-hist_height-1; j--){
 				kk=j*ww+(i*step_size+k);

				p_out_histogram_image[kk] = (unsigned char)255;		kk+=ww*hh;
 				p_out_histogram_image[kk] = (unsigned char)		0;		kk+=ww*hh;
 				p_out_histogram_image[kk] = (unsigned char)		0;		kk+=ww*hh;
			}
		}

		if(in_color_mode != 0){
			if((i+1)%(zz_hist_bin_num/3) == 0){
				for(j=hh-1; j>=0; j--){
					kk=j*ww+(i*step_size+step_size-1);

					p_out_histogram_image[kk] = (unsigned char)		0;		kk+=ww*hh;
					p_out_histogram_image[kk] = (unsigned char)		0;		kk+=ww*hh;
					p_out_histogram_image[kk] = (unsigned char)		255;		kk+=ww*hh;
				}
			}
		}
	}

	delete[] temp_hist;
}

//*********************************************************************************************************
void CKvHistogram::ghi_Get_Histogram_Image(
	float *in_hist,
	int in_num_bins,
	CKvMatrixUcharRgb* out_histogram_image,
	bool in_streched_mode,
	int in_color_mode)
//*********************************************************************************************************
{
	int i, j, k, kk, ww, hh, step_size=0, hist_height=0;
	unsigned char *p_out_histogram_image;
	float *temp_hist;

	if(in_num_bins<=0){Kv_Printf("[CKvSuperPixel::ghip_Get_Histogram_Image_Pointer]\nNon-positive bin size");exit(0);}

	temp_hist = new float[in_num_bins];

	if(out_histogram_image->mw() == 1 && out_histogram_image->mh() == 1){
		ww = hh = DEFAULT_IMAGE_WIDTH_FOR_HISTOGRAM_IMAGE;
		if(in_color_mode != 0)		ww = 3*ww;
		p_out_histogram_image = out_histogram_image->c_Create(hh, ww, Kv_Rgb((unsigned char)255, (unsigned char)255, (unsigned char)255))[0];
	}
	else{
		p_out_histogram_image = out_histogram_image->mps(ww, hh)[0];
		for(i=0; i<ww*hh; i++){
			kk=i;
			p_out_histogram_image[kk] = (unsigned char)255;		kk+=ww*hh;
			p_out_histogram_image[kk] = (unsigned char)255;		kk+=ww*hh;
			p_out_histogram_image[kk] = (unsigned char)255;		kk+=ww*hh;
		}
	}

	if(in_streched_mode==true){
		float max_hist=-10000.0;
		for(i=0; i<in_num_bins; i++) {if(in_hist[i] > max_hist)max_hist = in_hist[i];}
		for(i=0; i<in_num_bins; i++) {temp_hist[i] = in_hist[i]/max_hist;}
	}
	else{for(i=0; i<in_num_bins; i++){temp_hist[i] = in_hist[i];}}
	step_size = ww/in_num_bins;
	
	for(i=0; i<in_num_bins; i++){
		/// ///////////////////////////////////////////////////////////////////////////
		hist_height = (int)(temp_hist[i]*(float)hh);
		/// ///////////////////////////////////////////////////////////////////////////
		for(k=0; k<step_size; k++){

			for(j=hh-1; j>hh-hist_height-1; j--){
 				kk=j*ww+(i*step_size+k);
				p_out_histogram_image[kk] = (unsigned char)255;		kk+=ww*hh;
 				p_out_histogram_image[kk] = (unsigned char)0;		kk+=ww*hh;
 				p_out_histogram_image[kk] = (unsigned char)0;		kk+=ww*hh;
			}
		}
		if(in_color_mode != 0){
			if((i+1)%(in_num_bins/3) == 0){
				for(j=hh-1; j>=0; j--){
					kk=j*ww+(i*step_size+step_size-1);
					p_out_histogram_image[kk] = (unsigned char)0;		kk+=ww*hh;
					p_out_histogram_image[kk] = (unsigned char)0;		kk+=ww*hh;
					p_out_histogram_image[kk] = (unsigned char)255;		kk+=ww*hh;
				}
			}
		}
	}

	delete[] temp_hist;
}

//*********************************************************************************************************
void CKvHistogram::ghi_Get_Histogram_Image(
	float *in_hist,
	int in_num_bins,
	CKvMatrixUcharRgb* out_histogram_image,
	int in_ww, int in_hh,
	int in_scaling,
	bool in_streched_mode,
	int in_color_mode)
//*********************************************************************************************************
{
	int i, j, k, kk, ww, hh, step_size=0, hist_height=0;
	unsigned char *p_out_histogram_image;
	float *temp_hist;

	if(in_num_bins<=0){Kv_Printf("[CKvSuperPixel::ghip_Get_Histogram_Image_Pointer]\nNon-positive bin size");exit(0);}

	temp_hist = new float[in_num_bins];

	ww = in_ww;		hh = in_hh;
	if(in_color_mode)		ww = 3*ww;
	p_out_histogram_image = out_histogram_image->c_Create(hh, ww, Kv_Rgb((unsigned char)255, (unsigned char)255, (unsigned char)255))[0];

	if(in_streched_mode){
		float max_hist=-10000.0;
		for(i=0; i<in_num_bins; i++) {if(in_hist[i] > max_hist)max_hist = in_hist[i];}
		for(i=0; i<in_num_bins; i++) {temp_hist[i] = in_hist[i]/max_hist;}
	}
	else{for(i=0; i<in_num_bins; i++){temp_hist[i] = in_hist[i];}}
	step_size = ww/in_num_bins;

	for(i=0; i<in_num_bins; i++){
		/// ///////////////////////////////////////////////////////////////////////////
		hist_height = min((int)(temp_hist[i]*(float)hh), hh-1);//hist_height = min((int)(temp_hist[i]*(float)hh)*in_scaling, hh-1);
		/// ///////////////////////////////////////////////////////////////////////////
		for(k=0; k<step_size; k++){

			for(j=hh-1; j>hh-hist_height-1; j--){
 				kk=j*ww+(i*step_size+k);
				p_out_histogram_image[kk] = (unsigned char)255;		kk+=ww*hh;
 				p_out_histogram_image[kk] = (unsigned char)0;		kk+=ww*hh;
 				p_out_histogram_image[kk] = (unsigned char)0;		kk+=ww*hh;
			}
		}
		if(in_color_mode){
			if((i+1)%(in_num_bins/3) == 0){
				for(j=hh-1; j>=0; j--){
					kk=j*ww+(i*step_size+step_size-1);
					p_out_histogram_image[kk] = (unsigned char)0;		kk+=ww*hh;
					p_out_histogram_image[kk] = (unsigned char)0;		kk+=ww*hh;
					p_out_histogram_image[kk] = (unsigned char)255;		kk+=ww*hh;
				}
			}
		}
	}
	delete[] temp_hist;
}

//*********************************************************************************************************
void CKvHistogram::dgi_Display_Grid_Image(
	CKvScreen *in_screen,
	CKvMatrixUcharRgb *in_image,
	CKvMatrixFloat *in_sub_block_position_ratios)
//*********************************************************************************************************
{
	CKvMatrixUcharRgb temp_img;
	int i, j, ww, hh, block_num, tmp;
	int tl_x, tl_y, br_x, br_y;
	float **p_sub_blocks;

	p_sub_blocks = in_sub_block_position_ratios->mps(block_num, tmp);
	temp_img.cp_Copy(in_image);
	temp_img.ms(ww, hh);

	for(i=0; i<block_num; i++){
		tl_x = (int)(p_sub_blocks[0][i]*ww);		tl_y = (int)(p_sub_blocks[1][i]*hh);
		br_x = (int)(p_sub_blocks[2][i]*ww);		br_y = (int)(p_sub_blocks[3][i]*hh);
		
		temp_img.sbox_Set_Box(tl_x, tl_y, br_x-tl_x, br_y-tl_y, Kv_Rgb(255, 0, 0));
	}	

	in_screen->s_d_Display(&temp_img);


}

//*********************************************************************************************************
void CKvHistogram::ghgi_Get_Histogram_Grid_Image(
	CKvMatrixUcharRgb *out_histogram_grid_image,
	int in_grid_width,int in_grid_height,
	int in_grid_num_x, int in_grid_num_y,
	CKvMatrixFloat *in_sub_block_position_ratios,
	bool in_streched_mode,
	int in_color_mode)
//*********************************************************************************************************
{
	int i, j, k, n, kk, mm, ww, hh, step_size=0, hist_height=0;
	int block_num, temp_bin_num, grid_x, grid_y, tmp;
	CKvMatrixUcharRgb temp_hist_image;
	unsigned char *p_out_histogram_grid_image, *p_temp_hist_image;
	float *temp_hist;
	float **p_in_sub_block_position_ratios;

	if(zz_hist_bin_num<=0){Kv_Printf("[CKvSuperPixel::ghip_Get_Histogram_Image_Pointer]\nNon-positive bin size");exit(0);}

	p_in_sub_block_position_ratios = in_sub_block_position_ratios->mps(block_num, tmp);
	//block_num = in_grid_num_x*in_grid_num_y;
	temp_bin_num = zz_hist_bin_num/block_num;		
	temp_hist = new float[temp_bin_num];
	p_temp_hist_image = temp_hist_image.c_Create(in_grid_height, in_grid_width)[0];

	ww = in_grid_width*in_grid_num_x;	hh = in_grid_height*in_grid_num_y;
	if(out_histogram_grid_image->mw() != ww || out_histogram_grid_image->mh() != hh){ 		
		p_out_histogram_grid_image = out_histogram_grid_image->c_Create(hh, ww, Kv_Rgb((unsigned char)255, (unsigned char)255, (unsigned char)255))[0];
	}
	else{
		p_out_histogram_grid_image = out_histogram_grid_image->mps(ww, hh)[0];
		for(i=0; i<ww*hh; i++){
			kk=i;
			p_out_histogram_grid_image[kk] = (unsigned char)255;		kk+=ww*hh;
			p_out_histogram_grid_image[kk] = (unsigned char)255;		kk+=ww*hh;
			p_out_histogram_grid_image[kk] = (unsigned char)255;		kk+=ww*hh;
		}
	}

	for(n=0; n<block_num; n++){

		for(i=0; i<temp_bin_num; i++)	temp_hist[i] = zz_histogram[n*temp_bin_num+i];
		ghi_Get_Histogram_Image(temp_hist,
			temp_bin_num,
			&temp_hist_image,
			in_grid_width, in_grid_height,
			5,
			in_streched_mode,
			in_color_mode);
		temp_hist_image.sbox_Set_Box(0, 0, in_grid_width, in_grid_height, Kv_Rgb(0, 0, 255));
	
		grid_x = n%in_grid_num_x;	grid_y = n/in_grid_num_x;
		for(j=0; j<in_grid_height; j++){
			for(i=0; i<in_grid_width; i++){
					
				kk = (grid_y*in_grid_height+j)*ww+(grid_x*in_grid_width+i);
				mm = j*in_grid_width+i;
				p_out_histogram_grid_image[kk] = p_temp_hist_image[mm];		kk+=ww*hh;	mm+=in_grid_width*in_grid_height;
				p_out_histogram_grid_image[kk] = p_temp_hist_image[mm];		kk+=ww*hh;	mm+=in_grid_width*in_grid_height;
				p_out_histogram_grid_image[kk] = p_temp_hist_image[mm];		kk+=ww*hh;	mm+=in_grid_width*in_grid_height;
			}
		}		
	}

	delete[] temp_hist;
}

//*********************************************************************************************************
void CKvHistogram::ghgi_Get_Histogram_Grid_Image(
	CKvMatrixUcharRgb *out_histogram_grid_image,
	CKvMatrixFloat *in_sub_block_position_ratios,
	int in_ww, int in_hh,
	int in_bin_number,
	bool in_streched_mode,
	int in_color_mode)
//*********************************************************************************************************
{
	int i, j, k, n, kk, mm, step_size=0, hist_height=0;
	int block_num, grid_x, grid_y, grid_ww, grid_hh, tmp;
	CKvMatrixUcharRgb temp_hist_image;
	unsigned char *p_out_histogram_grid_image, *p_temp_hist_image;
	float *temp_hist;
	float **p_in_sub_block_position_ratios;

	if(zz_hist_bin_num<=0){Kv_Printf("[CKvSuperPixel::ghip_Get_Histogram_Image_Pointer]\nNon-positive bin size");exit(0);}

	p_in_sub_block_position_ratios = in_sub_block_position_ratios->mps(block_num, tmp);
	//block_num = in_grid_num_x*in_grid_num_y;
	temp_hist = new float[in_bin_number];	
	
	if(out_histogram_grid_image->mw() != in_ww || out_histogram_grid_image->mh() != in_hh){ 		
		p_out_histogram_grid_image = out_histogram_grid_image->c_Create(in_hh, in_ww, Kv_Rgb((unsigned char)255, (unsigned char)255, (unsigned char)255))[0];
	}
	else{
		p_out_histogram_grid_image = out_histogram_grid_image->vp();
		for(i=0; i<in_ww*in_hh; i++){
			kk=i;
			p_out_histogram_grid_image[kk] = (unsigned char)255;		kk+=in_ww*in_hh;
			p_out_histogram_grid_image[kk] = (unsigned char)255;		kk+=in_ww*in_hh;
			p_out_histogram_grid_image[kk] = (unsigned char)255;		kk+=in_ww*in_hh;
		}
	}

	for(n=0; n<block_num; n++){

		grid_x = (int)(p_in_sub_block_position_ratios[0][n]*in_ww);
		grid_y = (int)(p_in_sub_block_position_ratios[1][n]*in_hh);			
		grid_ww=(int)((p_in_sub_block_position_ratios[2][n]-p_in_sub_block_position_ratios[0][n])*in_ww);
		grid_hh=(int)((p_in_sub_block_position_ratios[3][n]-p_in_sub_block_position_ratios[1][n])*in_hh);
		p_temp_hist_image=temp_hist_image.c_Create(grid_hh, grid_ww, Kv_Rgb((unsigned char)255, (unsigned char)255, (unsigned char)255))[0];

		for(i=0; i<in_bin_number; i++)	temp_hist[i] = zz_histogram[n*in_bin_number+i];

		ghi_Get_Histogram_Image(temp_hist,
			in_bin_number,
			&temp_hist_image,
			grid_ww, grid_hh,
			5,
			in_streched_mode,
			in_color_mode);
		temp_hist_image.sbox_Set_Box(0, 0, grid_ww, grid_hh, Kv_Rgb(0, 0, 255));
	
		for(j=0; j<grid_hh; j++){
			for(i=0; i<grid_ww; i++){
					
				kk = (grid_y+j)*in_ww+(grid_x+i);
				mm = j*grid_ww+i;
				p_out_histogram_grid_image[kk] = p_temp_hist_image[mm];		kk+=in_ww*in_hh;	mm+=grid_ww*grid_hh;
				p_out_histogram_grid_image[kk] = p_temp_hist_image[mm];		kk+=in_ww*in_hh;	mm+=grid_ww*grid_hh;
				p_out_histogram_grid_image[kk] = p_temp_hist_image[mm];		kk+=in_ww*in_hh;	mm+=grid_ww*grid_hh;
			}
		}

	}

	delete[] temp_hist;
}

//********************************************************************************************
CKvSplittedDiscJM::CKvSplittedDiscJM()
//********************************************************************************************
{
	zz_disc_radius=0;
	zz_edge_gradient=0.0f;
	zz_pixels_in_upper_disc=NULL;
	zz_pixels_in_lower_disc=NULL;

	zz_hist_bin_num=0;
	zz_intensity_hist_of_upper_disc=NULL;
	zz_intensity_hist_of_lower_disc=NULL;
}

//********************************************************************************************
CKvSplittedDiscJM::~CKvSplittedDiscJM()
//********************************************************************************************
{
	if(zz_pixels_in_upper_disc!=NULL)	delete[] zz_pixels_in_upper_disc;
	if(zz_pixels_in_lower_disc!=NULL)	delete[] zz_pixels_in_lower_disc;
	if(zz_intensity_hist_of_upper_disc!=NULL)			delete[] zz_intensity_hist_of_upper_disc;
	if(zz_intensity_hist_of_lower_disc!=NULL)			delete[] zz_intensity_hist_of_lower_disc;
}

//********************************************************************************************
int CKvSplittedDiscJM::c_Create(CKvMatrixUchar &in_gray_image,
			 int in_x,
			 int in_y,
			 float in_split_angle,
			 float in_disc_radius,
			 int in_hist_bin_num)
//********************************************************************************************
{

	int ww, hh,value;
	unsigned char *p_in_gray_image;
	float grad_x,grad_y,angle,ud,distance,scale;

	
	p_in_gray_image = in_gray_image.mps_Matrix_Pointer_and_Width_Height(ww, hh)[0];

	if(in_hist_bin_num<=0){
			Kv_Printf("CKvSplittedDiscJM::c_Create\n--Non-positive histogram bin number.");
			return 0;
	}

	if(ww==1 && hh==1){
		Kv_Printf("CKvSplittedDiscJM::c_Create\n--Image size is non-positive.");
		return 0;
	}

	if(in_x-in_disc_radius<0.0f || in_x+in_disc_radius>=ww
		|| in_y-in_disc_radius<0.0f || in_y+in_disc_radius>=hh){
		Kv_Printf("CKvSplittedDiscJM::c_Create\n--Invalid pixel position.");
		return 0;
	}

	int i, j;
	int max_pixel_num;
	zz_pixel_num_in_upper_disc = zz_pixel_num_in_lower_disc = 0;
	max_pixel_num = 2*in_disc_radius*in_disc_radius;			// area of half rectangle
	if(!zz_pixels_in_upper_disc)	zz_pixels_in_upper_disc = new int[2*max_pixel_num];
	if(!zz_pixels_in_lower_disc)		zz_pixels_in_lower_disc = new int[2*max_pixel_num];
	
	if(in_split_angle < -90.0f || in_split_angle > 90.0f){
		// compute edge gradient of pixel (x, y).
		grad_x=p_in_gray_image[in_x+1+ww*(in_y-1)]+2*p_in_gray_image[in_x+1+ww*(in_y)]+p_in_gray_image[in_x+1+ww*(in_y+1)]
		-(p_in_gray_image[in_x-1+ww*(in_y-1)]+2*p_in_gray_image[in_x-1+ww*(in_y)]+p_in_gray_image[in_x-1+ww*(in_y+1)]);

		grad_y=p_in_gray_image[in_x-1+ww*(in_y+1)]+2*p_in_gray_image[in_x+ww*(in_y+1)]+p_in_gray_image[in_x+1+ww*(in_y+1)]
		-(p_in_gray_image[in_x-1+ww*(in_y-1)]+2*p_in_gray_image[in_x+ww*(in_y-1)]+p_in_gray_image[in_x+1+ww*(in_y-1)]);	
		
	}
	else{
		angle = tan(in_split_angle*PI/180.0);
		if(abs(angle)>1.0){
			grad_x = 1.0/abs(angle);
			grad_y = angle/abs(angle);
		}
		else{
			grad_x = 1.0;
			grad_y = angle;
		}
	}
	
	// find pixels included in disc.
	for(j=in_y-in_disc_radius; j<in_y+in_disc_radius; j++){
		for(i=in_x-in_disc_radius; i<in_x+in_disc_radius; i++){
	
			// Euclidean distance between pixel (i, j) and pixel (x, y)
			distance=sqrt((float)((i-in_x)*(i-in_x)+(j-in_y)*(j-in_y)));
			if(distance>in_disc_radius)
				continue;
			else
			{
				// inner product between direction vector of pixel (i-in_x, j-in_y) and gradient vector of pixel (in_x, in_y).
				ud=grad_x*(i-in_x)+grad_y*(j-in_y);		
				if (grad_x<0)
				{
					if(ud>=0){
						//up
						zz_pixels_in_upper_disc[2*zz_pixel_num_in_upper_disc] = i;
 						zz_pixels_in_upper_disc[2*zz_pixel_num_in_upper_disc+1] = j;
 						zz_pixel_num_in_upper_disc++;
					}
					else{
						zz_pixels_in_lower_disc[2*zz_pixel_num_in_lower_disc] = i;
						zz_pixels_in_lower_disc[2*zz_pixel_num_in_lower_disc+1] = j;
						zz_pixel_num_in_lower_disc++;
					}
					
				}
				else
				{
					if(ud>0){
						zz_pixels_in_lower_disc[2*zz_pixel_num_in_lower_disc] = i;
						zz_pixels_in_lower_disc[2*zz_pixel_num_in_lower_disc+1] = j;
						zz_pixel_num_in_lower_disc++;
					}
					else{						
						zz_pixels_in_upper_disc[2*zz_pixel_num_in_upper_disc] = i;
						zz_pixels_in_upper_disc[2*zz_pixel_num_in_upper_disc+1] = j;
						zz_pixel_num_in_upper_disc++;
					}				
				}
			}

		}
	}
	
	// compute histograms of each half-disc
	zz_hist_bin_num = in_hist_bin_num;
	if(!zz_intensity_hist_of_upper_disc)	zz_intensity_hist_of_upper_disc = new float[zz_hist_bin_num];
	if(!zz_intensity_hist_of_lower_disc)	zz_intensity_hist_of_lower_disc = new float[zz_hist_bin_num];
	for(i=0; i<zz_hist_bin_num; i++){
		zz_intensity_hist_of_upper_disc[i] = 0.0;
		zz_intensity_hist_of_lower_disc[i] = 0.0;
	}

	scale=(float)zz_hist_bin_num/256.0f;
	for(i=0;i<zz_pixel_num_in_upper_disc-1;i++)
	{
		value=p_in_gray_image[zz_pixels_in_upper_disc[2*i+1]*ww+zz_pixels_in_upper_disc[2*i]];
		zz_intensity_hist_of_upper_disc[(int)(scale*value)]+=1.0/(float)zz_pixel_num_in_upper_disc;
	}

	for(i=0;i<zz_pixel_num_in_lower_disc-1;i++)
	{
		value=p_in_gray_image[zz_pixels_in_lower_disc[2*i+1]*ww+zz_pixels_in_lower_disc[2*i]];
		zz_intensity_hist_of_lower_disc[(int)(scale*value)]+=1.0/(float)zz_pixel_num_in_lower_disc;
	}

	return 1;
}

//********************************************************************************************
int CKvSplittedDiscJM::cshoi_Compute_Splitted_Histogram_of_Intensity(CKvMatrixUchar &in_gray_image,
			 int in_x,
			 int in_y,
			 float in_split_angle,
			 float in_disc_radius,
			 int in_hist_bin_num)
//********************************************************************************************
{
//	CKvScreen zz_sc;
//	zz_sc.s_d_Display(&in_gray_image);
//	if(Kv_Printf("gray")==0)		exit(0);
	int ww, hh,val;
	unsigned char *p_in_gray_image;
	float grad_x,grad_y,angle,ud,distance,scale;
	
	p_in_gray_image = in_gray_image.mps_Matrix_Pointer_and_Width_Height(ww, hh)[0];

	if(in_hist_bin_num<=0){
			Kv_Printf("CKvSplittedDiscJM::c_Create\n--Non-positive histogram bin number.");
			return 0;
	}

	if(ww==1 && hh==1){
		Kv_Printf("CKvSplittedDiscJM::c_Create\n--Image size is non-positive.");
		return 0;
	}

	if(in_x-in_disc_radius<0.0f || in_x+in_disc_radius>=ww
		|| in_y-in_disc_radius<0.0f || in_y+in_disc_radius>=hh){
		Kv_Printf("CKvSplittedDiscJM::c_Create\n--Invalid pixel position.");
		return 0;
	}

	int i, j;
	int max_pixel_num, radius_square, temp_idx;
	zz_pixel_num_in_upper_disc = zz_pixel_num_in_lower_disc = 0;
	max_pixel_num = 2*in_disc_radius*in_disc_radius;			// area of half rectangle
	
	if(in_split_angle < -90.0f || in_split_angle > 90.0f){
		printf("hell!\n");
		// compute edge gradient of pixel (x, y).
		grad_x=p_in_gray_image[in_x+1+ww*(in_y-1)]+2*p_in_gray_image[in_x+1+ww*(in_y)]+p_in_gray_image[in_x+1+ww*(in_y+1)]
		-(p_in_gray_image[in_x-1+ww*(in_y-1)]+2*p_in_gray_image[in_x-1+ww*(in_y)]+p_in_gray_image[in_x-1+ww*(in_y+1)]);
		grad_y=p_in_gray_image[in_x-1+ww*(in_y+1)]+2*p_in_gray_image[in_x+ww*(in_y+1)]+p_in_gray_image[in_x+1+ww*(in_y+1)]
		-(p_in_gray_image[in_x-1+ww*(in_y-1)]+2*p_in_gray_image[in_x+ww*(in_y-1)]+p_in_gray_image[in_x+1+ww*(in_y-1)]);			
	}
	else{
		angle = tan(in_split_angle*PI/180.0);
		if(abs(angle)>1.0){
			grad_x = 1.0/abs(angle);
			grad_y = angle/abs(angle);
		}
		else{
			grad_x = 1.0;
			grad_y = angle;
		}
	}
	
	// compute histograms of each half-disc	
	if(zz_intensity_hist_of_upper_disc && zz_hist_bin_num != in_hist_bin_num){
		zz_hist_bin_num = in_hist_bin_num;
		delete[]	zz_intensity_hist_of_upper_disc;		zz_intensity_hist_of_upper_disc = new float[zz_hist_bin_num];
		delete[]	zz_intensity_hist_of_lower_disc;		zz_intensity_hist_of_lower_disc = new float[zz_hist_bin_num];
	}
	else{
		zz_hist_bin_num = in_hist_bin_num;
		if(!zz_intensity_hist_of_upper_disc){	zz_intensity_hist_of_upper_disc = new float[zz_hist_bin_num];	}
		if(!zz_intensity_hist_of_lower_disc){	zz_intensity_hist_of_lower_disc = new float[zz_hist_bin_num];	}
	}	

	for(i=0; i<zz_hist_bin_num; i++){
		zz_intensity_hist_of_upper_disc[i] = 0.0;
		zz_intensity_hist_of_lower_disc[i] = 0.0;
	}
	scale=(float)zz_hist_bin_num/256.0f;
	radius_square = SQUARE(in_disc_radius);

	for(j=in_y-in_disc_radius; j<in_y+in_disc_radius; j++){
		for(i=in_x-in_disc_radius; i<in_x+in_disc_radius; i++){
	
			// Euclidean distance between pixel (i, j) and pixel (x, y)
			distance=(float)(SQUARE(i-in_x)+SQUARE(j-in_y));
			if(distance<=radius_square){
				// inner product between direction vector of pixel (i-in_x, j-in_y) and gradient vector of pixel (in_x, in_y).
				ud = grad_x*(i-in_x)+grad_y*(j-in_y);
				val=p_in_gray_image[j*ww+i];
				temp_idx = (int)(scale*val);
				if (grad_x<0)
				{
					if(ud>=0){
						//upper
						zz_intensity_hist_of_upper_disc[temp_idx]+=1.0f;
 						zz_pixel_num_in_upper_disc++;
					}
					else{
						//lower
						zz_intensity_hist_of_lower_disc[temp_idx]+=1.0f;
						zz_pixel_num_in_lower_disc++;
					}
					
				}
				else
				{
					if(ud>0){
						//lower
						zz_intensity_hist_of_lower_disc[temp_idx]+=1.0f;
						zz_pixel_num_in_lower_disc++;
					}
					else{		
						//upper
						zz_intensity_hist_of_upper_disc[temp_idx]+=1.0f;
						zz_pixel_num_in_upper_disc++;
					}				
				}
			}

		}
	}
	
	// normalize histograms of each half-disc
	for(i=0;i<zz_hist_bin_num;i++)
	{
		zz_intensity_hist_of_upper_disc[i] /= (float)zz_pixel_num_in_upper_disc;
		zz_intensity_hist_of_lower_disc[i] /= (float)zz_pixel_num_in_lower_disc;
	}

	return 1;
}

//********************************************************************************************
int CKvSplittedDiscJM::cshog_Compute_Splitted_Histogram_of_Gradient(CKvMatrixFloat &in_edge_mag_image,
												 CKvMatrixFloat &in_edge_ori_image,
												 int in_x,
												 int in_y,												
												 float in_disc_radius,
												 int in_hist_bin_num)
//********************************************************************************************
{
//	CKvScreen zz_sc;
//	zz_sc.s_d_Display(&in_gray_image);
//	if(Kv_Printf("gray")==0)		exit(0);
	int ww, hh;
	float *p_in_edge_mag_image, *p_in_edge_ori_image;
	float grad_x, grad_y, angle, ud, distance, scale, in_split_angle;
	
	p_in_edge_mag_image = in_edge_mag_image.mps(ww, hh)[0];
	p_in_edge_ori_image = in_edge_ori_image.vp();

	if(in_hist_bin_num<=0){
			Kv_Printf("CKvSplittedDiscJM::c_Create\n--Non-positive histogram bin number.");
			return 0;
	}

	if(ww==1 && hh==1){
		Kv_Printf("CKvSplittedDiscJM::c_Create\n--Image size is non-positive.");
		return 0;
	}

	if(in_x-in_disc_radius<0.0f || in_x+in_disc_radius>=ww
		|| in_y-in_disc_radius<0.0f || in_y+in_disc_radius>=hh){
		Kv_Printf("CKvSplittedDiscJM::c_Create\n--Invalid pixel position.");
		return 0;
	}

	int i, j;
	int max_pixel_num, radius_square, temp_idx;
	float sum_of_mag_upper, sum_of_mag_lower, temp_mag, temp_ori;
	sum_of_mag_upper = sum_of_mag_lower = 0.0f;
	zz_pixel_num_in_upper_disc = zz_pixel_num_in_lower_disc = 0;
	max_pixel_num = 2*in_disc_radius*in_disc_radius;			// area of half rectangle

	// convert angle range from 0~2*PI to -90~90.
	in_split_angle = p_in_edge_ori_image[in_y*ww+in_x]*180.0f/PI;
	if(in_split_angle > 90.0f && in_split_angle <= 270.0f)				in_split_angle -= 180.0f;
	else if(in_split_angle > 270.0f && in_split_angle <= 360.0f)		in_split_angle -= 360.0f;	
	
	angle = tan(in_split_angle*PI/180.0);
	if(abs(angle)>1.0){
		grad_x = 1.0/abs(angle);
		grad_y = angle/abs(angle);
	}
	else{
		grad_x = 1.0;
		grad_y = angle;
	}

	// compute histograms of each half-disc	
	if(zz_intensity_hist_of_upper_disc && zz_hist_bin_num != in_hist_bin_num){
		zz_hist_bin_num = in_hist_bin_num;
		delete[]	zz_intensity_hist_of_upper_disc;		zz_intensity_hist_of_upper_disc = new float[zz_hist_bin_num];
		delete[]	zz_intensity_hist_of_lower_disc;		zz_intensity_hist_of_lower_disc = new float[zz_hist_bin_num];
	}
	else{
		zz_hist_bin_num = in_hist_bin_num;
		if(!zz_intensity_hist_of_upper_disc){	zz_intensity_hist_of_upper_disc = new float[zz_hist_bin_num];	}
		if(!zz_intensity_hist_of_lower_disc){	zz_intensity_hist_of_lower_disc = new float[zz_hist_bin_num];	}
	}	

	for(i=0; i<zz_hist_bin_num; i++){
		zz_intensity_hist_of_upper_disc[i] = 0.0;
		zz_intensity_hist_of_lower_disc[i] = 0.0;
	}

	scale=(float)zz_hist_bin_num/(2.0f*PI);
	radius_square = SQUARE(in_disc_radius);

	for(j=in_y-in_disc_radius; j<in_y+in_disc_radius; j++){
		for(i=in_x-in_disc_radius; i<in_x+in_disc_radius; i++){
	
			// Euclidean distance between pixel (i, j) and pixel (x, y)
			distance=(float)(SQUARE(i-in_x)+SQUARE(j-in_y));
			if(distance<=radius_square){
				// inner product between direction vector of pixel (i-in_x, j-in_y) and gradient vector of pixel (in_x, in_y).
				ud=grad_x*(i-in_x)+grad_y*(j-in_y);

				temp_mag=p_in_edge_mag_image[j*ww+i];
				temp_ori=p_in_edge_ori_image[j*ww+i];
				temp_idx = (int)(scale*temp_ori);

				if (grad_x<0)
				{
					if(ud>=0){
						//upper
						zz_intensity_hist_of_upper_disc[temp_idx]+=temp_mag;
						sum_of_mag_upper += temp_mag;
 						zz_pixel_num_in_upper_disc++;
					}
					else{
						//lower
						zz_intensity_hist_of_lower_disc[temp_idx]+=temp_mag;
						sum_of_mag_lower += temp_mag;
						zz_pixel_num_in_lower_disc++;
					}
					
				}
				else
				{
					if(ud>0){
						//lower
						zz_intensity_hist_of_lower_disc[temp_idx]+=temp_mag;
						sum_of_mag_lower += temp_mag;
						zz_pixel_num_in_lower_disc++;
					}
					else{		
						//upper
						zz_intensity_hist_of_upper_disc[temp_idx]+=temp_mag;
						sum_of_mag_upper += temp_mag;
						zz_pixel_num_in_upper_disc++;
					}				
				}
			}

		}
	}

	// normalize histograms of each half-disc
	if(sum_of_mag_upper<=10.0f || sum_of_mag_lower <= 10.0f)
		return 0;
	else{
		for(i=0;i<zz_hist_bin_num;i++)
		{
			zz_intensity_hist_of_upper_disc[i] /= sum_of_mag_upper;
			zz_intensity_hist_of_lower_disc[i] /= sum_of_mag_lower;
		}

		return 1;
	}
}


//********************************************************************************************
int CKvSplittedDiscJM::ssd_Show_Splitted_Disc(CKvMatrixUcharRgb &in_image,
	CKvRgbaF in_color_of_upper_disc,
	CKvRgbaF in_color_of_lower_disc)
//********************************************************************************************
{
	int ww, hh,i;
	unsigned char *p_in_image;
	p_in_image = in_image.mps_Matrix_Pointer_and_Width_Height(ww, hh)[0];

	if(ww==1 && hh==1){
		Kv_Printf("CKvSplittedDiscJM::ssd_Show_Splitted_Disc\n--Image size is non-positive.");
		return 0;
	}

	if(zz_pixel_num_in_upper_disc==0 || zz_pixel_num_in_lower_disc==0){
		Kv_Printf("CKvSplittedDiscJM::ssd_Show_Splitted_Disc\n--Disc was not created yet.");
		return 0;
	}

	// display splitted disc on original image.
	//{
	for(i=0;i<zz_pixel_num_in_upper_disc-1;i++){

		p_in_image[zz_pixels_in_upper_disc[2*i+1]*ww+zz_pixels_in_upper_disc[2*i]]=in_color_of_upper_disc.r;
		p_in_image[zz_pixels_in_upper_disc[2*i+1]*ww+zz_pixels_in_upper_disc[2*i]+ww*hh]=in_color_of_upper_disc.g;
		p_in_image[zz_pixels_in_upper_disc[2*i+1]*ww+zz_pixels_in_upper_disc[2*i]+2*ww*hh]=in_color_of_upper_disc.b;
	}
	for(i=0;i<zz_pixel_num_in_lower_disc-1;i++){

		p_in_image[zz_pixels_in_lower_disc[2*i+1]*ww+zz_pixels_in_lower_disc[2*i]]=in_color_of_lower_disc.r;
		p_in_image[zz_pixels_in_lower_disc[2*i+1]*ww+zz_pixels_in_lower_disc[2*i]+ww*hh]=in_color_of_lower_disc.g;
		p_in_image[zz_pixels_in_lower_disc[2*i+1]*ww+zz_pixels_in_lower_disc[2*i]+2*ww*hh]=in_color_of_lower_disc.b;
	}


	//	}

	return 1;
}

float* CKvSplittedDiscJM::hp_Histogram_Pointer(int in_disc_index)
{
	if(zz_hist_bin_num<=0){
		Kv_Printf("CKvSplittedDiscJM::hp_Histogram_Pointer\n--Non-positive histogram bin size.");
		return 0;
	}

	if(in_disc_index!=0 && in_disc_index!=1){
		Kv_Printf("CKvSplittedDiscJM::hp_Histogram_Pointer\n--Invalid disc index.");
		return 0;
	}

	if(in_disc_index==0)		return zz_intensity_hist_of_upper_disc;
	else if(in_disc_index==1)	return zz_intensity_hist_of_lower_disc;
}

// ************************************************************************************ //
// ****************************** Contour Detection ********************************* //
// ************************************************************************************ //

//********************************************************************************************
CKvContourDetectionJM::CKvContourDetectionJM()
//********************************************************************************************
{
}

//********************************************************************************************
CKvContourDetectionJM::~CKvContourDetectionJM()
//********************************************************************************************
{
}

//********************************************************************************
void CKvContourDetectionJM::siff_Smooth_Image_with_Filter_Float(CKvMatrixUchar &in_image,
										 CKvMatrixFloat &in_filter_float,
										 CKvMatrixUchar &out_filtered_image)
 //********************************************************************************
{
	int i,j,k,m;
	int ww, hh, ww_exp, hh_exp;
	int ww_f, hh_f, dx1, dy1, dx2, dy2;

	CKvMatrixUchar exp_img;
	float *p_filter, tmp_val;
	unsigned char *p_in_image, *p_out_filtered_image, *p_temp_img, *p_exp_img;

	// We do not consider the boundary region.
	p_filter = in_filter_float.mps(ww_f, hh_f)[0];
	p_in_image = in_image.mps(ww, hh)[0];
	p_out_filtered_image = out_filtered_image.vp();

	if(ww<=0 || hh<=0 || ww!=out_filtered_image.mw() || hh!=out_filtered_image.mh()){
		Kv_Printf("[CKvContourDetectionJM::siff_Smooth_Image_with_Filter_Float]\n Invalid image size.");
		exit(0);
	}
	if(ww_f<=0 || hh_f<=0 || ww_f>ww || hh_f>hh){
		Kv_Printf("[CKvContourDetectionJM::siff_Smooth_Image_with_Filter_Float]\n Invalid filter size.");
		exit(0);
	}

	dx1 = ww_f/2;		dx2 = ww_f-1-dx1;
	dy1 = hh_f/2;			dy2 = hh_f-1-dy1;
	ww_exp = ww+ww_f-1;		hh_exp = hh+hh_f-1;
	p_exp_img = exp_img.c_Create(hh_exp, ww_exp)[0];
	exp_img.sb_Set_Block(dx1, dy1, &in_image);

	// mirroring boundary pixels.
	// vertical boundary pixels.
	for(i=0; i<dx1; i++){  	for(j=dy1; j<hh_exp-dy2; j++){		
		p_exp_img[j*ww_exp+i] 
		= p_in_image[(j-dy1)*ww+(dx1-1-i)];
	}	}
	for(i=0; i<dx2; i++){  	for(j=dy1; j<hh_exp-dy2; j++){		
		p_exp_img[j*ww_exp+(ww_exp-1-i)] 
		= p_in_image[(j-dy1)*ww+(ww-1-(dx2-1-i))];
	}	}
	// horizontal boundary pixels.
	for(j=0; j<dy1; j++){  	for(i=dx1; i<ww_exp-dx2; i++){		
		p_exp_img[j*ww_exp+i] 
		= p_in_image[(dy1-1-j)*ww+(i-dx1)];
	}	}
	for(j=0; j<dy2; j++){  	for(i=dx1; i<ww_exp-dx2; i++){		
		p_exp_img[(hh_exp-1-j)*ww_exp+i] 
		= p_in_image[(hh-1-(dy2-1-j))*ww+(i-dx1)];
	}	}

	// convolution.
	for(j=0; j<hh; j++){	for(i=0; i<ww; i++){	
		tmp_val = 0.0f;
		for(m=-dy1; m<dy2+1; m++){		for(k=-dx1; k<dx2+1; k++){						
			tmp_val += p_filter[(m+dy1)*ww_f+(k+dx1)]*(float)p_exp_img[(j+m+dy1)*ww_exp+(i+k+dx1)];
		}	}
		p_out_filtered_image[j*ww+i] = (unsigned char)tmp_val;		
	}	}


}

//********************************************************************************
void CKvContourDetectionJM::siff_Smooth_Image_with_Filter_Float(CKvMatrixUcharRgb &in_image,
										 CKvMatrixFloat &in_filter_float,
										 CKvMatrixUcharRgb &out_filtered_image)
 //********************************************************************************
{
	int i,j,k,m, n;
	int ww, hh, ww_exp, hh_exp;
	int ww_f, hh_f, dx1, dy1, dx2, dy2;

	CKvMatrixUcharRgb exp_img;
	float *p_filter, tmp_val;
	unsigned char *p_in_image, *p_out_filtered_image, *p_temp_img, *p_exp_img;

	// We do not consider the boundary region.
	p_filter = in_filter_float.mps(ww_f, hh_f)[0];
	p_in_image = in_image.mps(ww, hh)[0];
	p_out_filtered_image = out_filtered_image.vp();

	if(ww<=0 || hh<=0 || ww!=out_filtered_image.mw() || hh!=out_filtered_image.mh()){
		Kv_Printf("[CKvMakePanorama::siff_Smooth_Image_with_Filter_Float]\n Invalid image size.");
		exit(0);
	}
	if(ww_f<=0 || hh_f<=0 || ww_f>ww || hh_f>hh){
		Kv_Printf("[CKvMakePanorama::siff_Smooth_Image_with_Filter_Float]\n Invalid filter size.");
		exit(0);
	}
	
	dx1 = ww_f/2;		dx2 = ww_f-1-dx1;
	dy1 = hh_f/2;			dy2 = hh_f-1-dy1;
	ww_exp = ww+ww_f-1;		hh_exp = hh+hh_f-1;
	p_exp_img = exp_img.c_Create(hh_exp, ww_exp)[0];
	exp_img.sb_Set_Block(dx1, dy1, &in_image);

	// mirroring boundary pixels.
	// vertical boundary pixels.
	for(i=0; i<dx1; i++){  	for(j=dy1; j<hh_exp-dy2; j++){		for(n=0; n<3; n++){
		p_exp_img[j*ww_exp+i+ww_exp*hh_exp*n] 
		= p_in_image[(j-dy1)*ww+(dx1-1-i)+ww*hh*n];
	}	}	}
	for(i=0; i<dx2; i++){  	for(j=dy1; j<hh_exp-dy2; j++){		for(n=0; n<3; n++){
		p_exp_img[j*ww_exp+(ww_exp-1-i)+ww_exp*hh_exp*n] 
		= p_in_image[(j-dy1)*ww+(ww-1-(dx2-1-i))+ww*hh*n];
	}	}	}
	// horizontal boundary pixels.
	for(j=0; j<dy1; j++){  	for(i=dx1; i<ww_exp-dx2; i++){		for(n=0; n<3; n++){
		p_exp_img[j*ww_exp+i+ww_exp*hh_exp*n] 
		= p_in_image[(dy1-1-j)*ww+(i-dx1)+ww*hh*n];
	}	}	}
	for(j=0; j<dy2; j++){  	for(i=dx1; i<ww_exp-dx2; i++){		for(n=0; n<3; n++){
		p_exp_img[(hh_exp-1-j)*ww_exp+i+ww_exp*hh_exp*n] 
		= p_in_image[(hh-1-(dy2-1-j))*ww+(i-dx1)+ww*hh*n];
	}	}	}

	// convolution.
	for(j=0; j<hh; j++){	for(i=0; i<ww; i++){		for(n=0; n<3; n++){
		tmp_val = 0.0f;
		for(m=-dy1; m<dy2+1; m++){		for(k=-dx1; k<dx2+1; k++){						
			tmp_val += p_filter[(m+dy1)*ww_f+(k+dx1)]*(float)p_exp_img[(j+m+dy1)*ww_exp+(i+k+dx1) + ww_exp*hh_exp*n];
		}	}
		p_out_filtered_image[j*ww+i + ww*hh*n] = (unsigned char)tmp_val;		
	}	}	}

}

//********************************************************************************
void CKvContourDetectionJM::siff_Smooth_Image_with_Filter_Float(CKvMatrixShort &in_image,
										 CKvMatrixFloat &in_filter_float,
										 CKvMatrixShort &out_filtered_image)
 //********************************************************************************
{
	int i,j,k,m;
	int cx_f, cy_f, ww_f, hh_f, ww, hh;

	CKvMatrixShort temp_img;
	float *p_filter, tmp_val;
	short *p_in_image, *p_out_filtered_image, *p_temp_img;
	
	// We do not consider the boundary region.
	p_filter = in_filter_float.mps(ww_f, hh_f)[0];
	p_in_image = in_image.mps(ww, hh)[0];
	p_out_filtered_image = out_filtered_image.vp();
	if(&in_image == &out_filtered_image)		p_temp_img = temp_img.cp_Copy(&in_image)[0];

	if(ww<=0 || hh<=0 || ww!=out_filtered_image.mw() || hh!=out_filtered_image.mh()){
		Kv_Printf("[CKvMakePanorama::siff_Smooth_Image_with_Filter_Float]\n Invalid image size.");
		exit(0);
	}
	if(ww_f<=0 || hh_f<=0 || ww_f>ww || hh_f>hh){
		Kv_Printf("[CKvMakePanorama::siff_Smooth_Image_with_Filter_Float]\n Invalid filter size.");
		exit(0);
	}

	// convolution.
	cx_f = ww_f/2;		cy_f = hh_f/2;
	for(j=cy_f; j<hh-(hh_f-cy_f-1); j++){
		for(i=cx_f; i<ww-(ww_f-cx_f-1); i++){

			tmp_val = 0.0f;
			for(m=-cy_f; m<hh_f-cy_f; m++){
				for(k=-cx_f; k<ww_f-cx_f; k++){
					if(&in_image == &out_filtered_image)		
						tmp_val += p_filter[(m+cy_f)*ww_f+(k+cx_f)]*(float)p_temp_img[(j+m)*ww+(i+k)];
					else																
						tmp_val += p_filter[(m+cy_f)*ww_f+(k+cx_f)]*(float)p_in_image[(j+m)*ww+(i+k)];
				}
			}
			p_out_filtered_image[j*ww+i] = (short)tmp_val;

		}
	}

}

//********************************************************************************
void CKvContourDetectionJM::siff_Smooth_Image_with_Filter_Float(CKvMatrixShortRgb &in_image,
										 CKvMatrixFloat &in_filter_float,
										 CKvMatrixShortRgb &out_filtered_image)
 //********************************************************************************
{
	int i,j,k,m, n;
	int cx_f, cy_f, ww_f, hh_f, ww, hh;

	CKvMatrixShortRgb temp_img;
	float *p_filter, tmp_val;
	short *p_in_image, *p_out_filtered_image, *p_temp_img;

	// We do not consider the boundary region.
	p_filter = in_filter_float.mps(ww_f, hh_f)[0];
	p_in_image = in_image.mps(ww, hh)[0];
	p_out_filtered_image = out_filtered_image.vp();
	if(&in_image == &out_filtered_image)	p_temp_img = temp_img.cp_Copy(&in_image)[0];
	
	if(ww<=0 || hh<=0 || ww!=out_filtered_image.mw() || hh!=out_filtered_image.mh()){
		Kv_Printf("[CKvMakePanorama::siff_Smooth_Image_with_Filter_Float]\n Invalid image size.");
		exit(0);
	}
	if(ww_f<=0 || hh_f<=0 || ww_f>ww || hh_f>hh){
		Kv_Printf("[CKvMakePanorama::siff_Smooth_Image_with_Filter_Float]\n Invalid filter size.");
		exit(0);
	}

	// convolution.
	cx_f = ww_f/2;		cy_f = hh_f/2;
	for(j=cy_f; j<hh-(hh_f-cy_f-1); j++){
		for(i=cx_f; i<ww-(ww_f-cx_f-1); i++){

			for(n=0; n<3; n++){

				tmp_val = 0.0f;
				for(m=-cy_f; m<hh_f-cy_f; m++){
					for(k=-cx_f; k<ww_f-cx_f; k++){
						if(&in_image == &out_filtered_image)		
							tmp_val += p_filter[(m+cy_f)*ww_f+(k+cx_f)]*(float)p_temp_img[(j+m)*ww+(i+k) + ww*hh*n];
						else																		
							tmp_val += p_filter[(m+cy_f)*ww_f+(k+cx_f)]*(float)p_in_image[(j+m)*ww+(i+k) + ww*hh*n];

					}
				}
				p_out_filtered_image[j*ww+i + ww*hh*n] = (short)tmp_val;
		
			}

		}
	}

}

//********************************************************************************************
int CKvContourDetectionJM::exso_Edge_Extraction_using_Sobel_Operator(CKvMatrixUchar &in_gray_image,
										  CKvMatrixFloat &out_edge_mag_image,
										  CKvMatrixFloat &out_edge_ori_image)	// gradient range: 0 ~ 2*PI radian
//********************************************************************************************
{
	int ww, hh;
	int i, j, tmp;
	int top_left, top_mid, top_right, mid_left, mid_mid, mid_right, bottom_left, bottom_mid, bottom_right;
	float grad_x, grad_y, grad_mag, grad_ori;
	
	unsigned char *p_in_gray_image;
	float *p_out_edge_mag_image, *p_out_edge_ori_image;
	p_in_gray_image = in_gray_image.mps_Matrix_Pointer_and_Width_Height(ww, hh)[0];
	p_out_edge_mag_image = out_edge_mag_image.vp_Vector_Pointer();
	p_out_edge_ori_image = out_edge_ori_image.vp_Vector_Pointer();

	for(j=1; j<hh-1; j++){
		for(i=1; i<ww-1; i++){

			// BASIC method.
			tmp=(j-1) *ww+i+1;		grad_x =	(float)(p_in_gray_image[tmp]+2*p_in_gray_image[tmp+ww]+p_in_gray_image[tmp+2*ww]);
			tmp=(j-1) *ww+i-1;		grad_x -=	(float)(p_in_gray_image[tmp]+2*p_in_gray_image[tmp+ww]+p_in_gray_image[tmp+2*ww]);
			tmp=(j+1) *ww+i-1;		grad_y =	(float)(p_in_gray_image[tmp]+2*p_in_gray_image[tmp+1]+p_in_gray_image[tmp+2]);
			tmp=(j-1) *ww+i-1;		grad_y -=	(float)(p_in_gray_image[tmp]+2*p_in_gray_image[tmp+1]+p_in_gray_image[tmp+2]);
			
 			// magnitude
			grad_mag = sqrt(SQUARE(grad_x)+SQUARE(grad_y));
			p_out_edge_mag_image[j*ww+i] = grad_mag;			

			// gradient
			//grad_ori = atan2(grad_y, grad_x);
			grad_ori = abs(atan2(grad_y, grad_x));
			p_out_edge_ori_image[j*ww+i] = (grad_ori<0) ? (grad_ori+2.0f*PI) : grad_ori;

		}
	}

	return 1;
}

//********************************************************************************************
int CKvContourDetectionJM::exso_Edge_Extraction_using_Sobel_Operator(CKvMatrixFloat &in_gray_image,
																	 CKvMatrixFloat &out_edge_mag_image,
																	 CKvMatrixFloat &out_edge_ori_image)	// gradient range: 0 ~ 2*PI radian
//********************************************************************************************
{
	int ww, hh;
	int i, j, tmp;
	int top_left, top_mid, top_right, mid_left, mid_mid, mid_right, bottom_left, bottom_mid, bottom_right;
	float grad_x, grad_y, grad_mag, grad_ori;
	
	float *p_in_gray_image;
	float *p_out_edge_mag_image, *p_out_edge_ori_image;
	p_in_gray_image = in_gray_image.mps_Matrix_Pointer_and_Width_Height(ww, hh)[0];
	p_out_edge_mag_image = out_edge_mag_image.vp_Vector_Pointer();
	p_out_edge_ori_image = out_edge_ori_image.vp_Vector_Pointer();

	for(j=1; j<hh-1; j++){
		for(i=1; i<ww-1; i++){

			// BASIC method.
			tmp=(j-1) *ww+i+1;		grad_x =	(float)(p_in_gray_image[tmp]+2*p_in_gray_image[tmp+ww]+p_in_gray_image[tmp+2*ww]);
			tmp=(j-1) *ww+i-1;		grad_x -=	(float)(p_in_gray_image[tmp]+2*p_in_gray_image[tmp+ww]+p_in_gray_image[tmp+2*ww]);
			tmp=(j+1) *ww+i-1;		grad_y =	(float)(p_in_gray_image[tmp]+2*p_in_gray_image[tmp+1]+p_in_gray_image[tmp+2]);
			tmp=(j-1) *ww+i-1;		grad_y -=	(float)(p_in_gray_image[tmp]+2*p_in_gray_image[tmp+1]+p_in_gray_image[tmp+2]);


			// magnitude
			grad_mag = sqrt(SQUARE(grad_x)+SQUARE(grad_y));
			p_out_edge_mag_image[j*ww+i] = grad_mag;

			// gradient
			grad_ori = atan2(grad_y, grad_x);
			p_out_edge_ori_image[j*ww+i] = (grad_ori<0) ? (grad_ori+2*PI) : grad_ori;

		}
	}

	return 1;
}

//*********************************************************************************************************
int CKvContourDetectionJM::cexlab_Color_Edge_Extraction_in_Lab_Color_Space(CKvMatrixUcharRgb &in_image,
													CKvMatrixFloat &out_edge_mag_image,
													CKvMatrixFloat &out_edge_ori_image)	// gradient range: 0 ~ 2*PI radian
//*********************************************************************************************************
{
	LCKvUtility_for_Color_Conversion aa_cc;
	int ww, hh;
	int i, j, k, idx;

	unsigned char *p_in_image;
	float *p_out_edge_mag_image, *p_out_edge_ori_image;
	float *p_L_image, *p_a_image, *p_b_image, *p_ab_image;
	float *p_L_edge_mag, *p_a_edge_mag, *p_b_edge_mag, *p_ab_edge_mag;
	float *p_L_edge_ori, *p_a_edge_ori, *p_b_edge_ori, *p_ab_edge_ori;

	CKvMatrixFloat L_image, a_image, b_image, ab_image;
	CKvMatrixFloat L_edge_mag, a_edge_mag, b_edge_mag, ab_edge_mag;
	CKvMatrixFloat L_edge_ori, a_edge_ori, b_edge_ori, ab_edge_ori;
	p_in_image = in_image.mps(ww, hh)[0];
	p_out_edge_mag_image = out_edge_mag_image.mp()[0];
	p_out_edge_ori_image = out_edge_ori_image.mp()[0];

	p_L_image = L_image.c_Create(hh, ww, 0.0f)[0];
	p_a_image = a_image.c_Create(hh, ww, 0.0f)[0];
	p_b_image = b_image.c_Create(hh, ww, 0.0f)[0];
	p_ab_image = ab_image.c_Create(hh, ww, 0.0f)[0];

	p_L_edge_mag = L_edge_mag.c_Create(hh, ww, 0.0f)[0];
	p_a_edge_mag = a_edge_mag.c_Create(hh, ww, 0.0f)[0];
	p_b_edge_mag = b_edge_mag.c_Create(hh, ww, 0.0f)[0];
	p_ab_edge_mag = ab_edge_mag.c_Create(hh, ww, 0.0f)[0];

	p_L_edge_ori = L_edge_ori.c_Create(hh, ww, 0.0f)[0];
	p_a_edge_ori = a_edge_ori.c_Create(hh, ww, 0.0f)[0];
	p_b_edge_ori = b_edge_ori.c_Create(hh, ww, 0.0f)[0];
	p_ab_edge_ori = ab_edge_ori.c_Create(hh, ww, 0.0f)[0];

	ccirl_Convert_Color_Image_RGB_to_LAB(in_image,
		L_image,
		a_image,
		b_image);

	LCKvUtility_for_Windows aa_uw;
	CKvScreen sc[3];
	CKvString caption;
	caption = "L image";		sc[0].s_d_Display(1.0f, 0.0f, &L_image);		aa_uw.spt_Set_Position_and_Title(&sc[0], 0, 0, caption);
	caption = "a image";		sc[1].s_d_Display(1.0f, 0.0f, &a_image);		aa_uw.spt_Set_Position_and_Title(&sc[1], ww, 0, caption);
	caption = "b image";		sc[2].s_d_Display(1.0f, 0.0f, &b_image);		aa_uw.spt_Set_Position_and_Title(&sc[2], 2*ww, 0, caption);
	if(!Kv_Printf("Lab")) exit(0);

	// Edge map.
	// edge of L image.
	exso_Edge_Extraction_using_Sobel_Operator(L_image,
		L_edge_mag,
		L_edge_ori);
	caption = "L image";		sc[0].s_d_Display(1.0f, 0.0f, &L_image);			aa_uw.spt_Set_Position_and_Title(&sc[0], 0, 0, caption);
	caption = "Edge mag";		sc[1].s_d_Display(1.0f, 0.0f, &L_edge_mag);	aa_uw.spt_Set_Position_and_Title(&sc[1], ww, 0, caption);
	caption = "Edge ori";		sc[2].s_d_Display(1.0f, 0.0f, &L_edge_ori);		aa_uw.spt_Set_Position_and_Title(&sc[2], 2*ww, 0, caption);
	if(!Kv_Printf("L edge")) exit(0);
	// edge of a image.
	exso_Edge_Extraction_using_Sobel_Operator(a_image,
		a_edge_mag,
		a_edge_ori);
	caption = "a image";		sc[0].s_d_Display(1.0f, 0.0f, &a_image);		aa_uw.spt_Set_Position_and_Title(&sc[0], 0, 0, caption);
	caption = "Edge mag";		sc[1].s_d_Display(1.0f, 0.0f, &a_edge_mag);		aa_uw.spt_Set_Position_and_Title(&sc[1], ww, 0, caption);
	caption = "Edge ori";		sc[2].s_d_Display(1.0f, 0.0f, &a_edge_ori);		aa_uw.spt_Set_Position_and_Title(&sc[2], 2*ww, 0, caption);
	if(!Kv_Printf("a edge")) exit(0);
	// edge of b image.
	exso_Edge_Extraction_using_Sobel_Operator(b_image,
		b_edge_mag,
		b_edge_ori);
	caption = "b image";		sc[0].s_d_Display(1.0f, 0.0f, &b_image);		aa_uw.spt_Set_Position_and_Title(&sc[0], 0, 0, caption);
	caption = "Edge mag";		sc[1].s_d_Display(1.0f, 0.0f, &b_edge_mag);		aa_uw.spt_Set_Position_and_Title(&sc[1], ww, 0, caption);
	caption = "Edge ori";		sc[2].s_d_Display(1.0f, 0.0f, &b_edge_ori);		aa_uw.spt_Set_Position_and_Title(&sc[2], 2*ww, 0, caption);
	if(!Kv_Printf("b edge")) exit(0);

	for(j=0; j<hh; j++){
		for(i=0; i<ww; i++){
			idx = j*ww+i;
			p_ab_edge_mag[idx] = sqrt(SQUARE(p_a_edge_mag[idx])+SQUARE(p_b_edge_mag[idx]));
			p_ab_edge_ori[idx] = sqrt(SQUARE(p_a_edge_ori[idx])+SQUARE(p_b_edge_ori[idx]));
		}
	}

	caption = "Color image";			sc[0].s_d_Display(&in_image);			aa_uw.spt_Set_Position_and_Title(&sc[1], 0, 0, caption);
	caption = "L edge image";			sc[1].s_d_Display(1.0f, 0.0f, &L_edge_mag);			aa_uw.spt_Set_Position_and_Title(&sc[1], ww, 0, caption);
	caption = "ab edge image";		sc[2].s_d_Display(1.0f, 0.0f, &ab_edge_mag);		aa_uw.spt_Set_Position_and_Title(&sc[2], 2*ww, 0, caption);
	//caption = "ab image";		sc[2].s_d_Display(1.0f, 0.0f, &ab_image);		aa_uw.spt_Set_Position_and_Title(&sc[2], 2*ww, 0, caption);
	if(!Kv_Printf("ab image")) exit(0);
}

//*********************************************************************************************************
int CKvContourDetectionJM::efhs_Edge_Filtering_using_Histogram_Similarity(CKvMatrixUchar &in_image_gray,
	CKvMatrixFloat &in_edge_mag_image,
	CKvMatrixFloat &in_edge_ori_image,
	int in_histogram_filter_type,
	int in_histogram_distance_type,
	int in_disk_size,
	int in_hist_bin_num,
	float &out_hist_dist_max,
	CKvMatrixFloat &out_edge_mag_image)
//*********************************************************************************************************
{
	CKvSplittedDiscJM zz_sd;
	CKvMatrixUchar imgs_gray[4];
	CKvMatrixFloat edge_mag[4], edge_ori[4];
	CKvMatrixFloat in_gaussian_filter;

	int i, j, k, n, x, y;
	int ww, hh, prev_sigma, tmp;

	float hist_sim, hist_dist, temp_ori, dist_min, dist_max, ori_max, mean, var;
	float *p_edge_mag[4], *p_edge_ori[4];
	float *p_in_edge_mag_image, *p_in_edge_ori_image, *p_out_edge_mag_image;

	p_in_edge_mag_image = in_edge_mag_image.mps_Matrix_Pointer_and_Width_Height(ww, hh)[0];
	p_in_edge_ori_image = in_edge_ori_image.vp();
	p_out_edge_mag_image = out_edge_mag_image.vp_Vector_Pointer();

	CKvStopWatch zz_sw;
	float t1, t2;
	t1 = t2 = 0.0f;
	zz_sw.c_Create(2);

	dist_min = 100000.0f;		dist_max = -100000.0f;
	for(j=in_disk_size; j<hh-in_disk_size; j++){
		for(i=in_disk_size; i<ww-in_disk_size; i++){
			
			tmp = j*ww+i;
			zz_sw.r_Reset(0);
			// adaptive mask size.	
// 			dms_Determine_Mask_Size_for_Histogram_Filtering(i, j,
// 				ww, hh,
// 				p_in_edge_mag_image,
// 				p_in_edge_ori_image,
// 				mask_sz);

			/// remove meaningless edges.
			// using magnitude.
			if(p_in_edge_mag_image[tmp] <= 5.0f){
				p_out_edge_mag_image[tmp] = 0.0f;
				continue;
			}

			// histogram filtering.
			if(in_histogram_filter_type==KV_HIST_FILTER_TYPE_HOI){
				// using Histogram of Intensity.
				temp_ori = p_in_edge_ori_image[tmp]*180.0f/PI;
				if(temp_ori > 90.0f && temp_ori <= 270.0f)					temp_ori -= 180.0f;
				else if(temp_ori > 270.0f && temp_ori <= 360.0f)		temp_ori -= 360.0f;				
				zz_sd.cshoi_Compute_Splitted_Histogram_of_Intensity(in_image_gray,
					i, j,
					temp_ori,
					(float)in_disk_size,
					in_hist_bin_num);
			}
			else if(in_histogram_filter_type==KV_HIST_FILTER_TYPE_HOG){
				//  using Histogram of Gradient.
 				if(!zz_sd.cshog_Compute_Splitted_Histogram_of_Gradient(in_edge_mag_image,
					in_edge_ori_image,
 					i, j,
 					(float)in_disk_size,
 					in_hist_bin_num))		continue;
			}

			if(in_histogram_distance_type==KV_HIST_DISTANCE_L1_NORM)		
				hist_dist = chd_Caculate_Histogram_Distance_using_L1_Norm(zz_sd.hp_Histogram_Pointer(0),
				zz_sd.hp_Histogram_Pointer(1),
				in_hist_bin_num);
			else if(in_histogram_distance_type==KV_HIST_DISTANCE_L2_NORM)
				hist_dist = chd_Caculate_Histogram_Distance_using_L2_Norm(zz_sd.hp_Histogram_Pointer(0),
				zz_sd.hp_Histogram_Pointer(1),
				in_hist_bin_num);

			if(hist_dist > dist_max)		dist_max = hist_dist;
			if(hist_dist < dist_min)			dist_min = hist_dist;

			p_out_edge_mag_image[tmp] = hist_dist*p_in_edge_mag_image[tmp];
			
			//if(Kv_Printf("grad_x: %f, grad_y: %f, grad_mag: %f\n", grad_x, grad_y, grad_mag)==0) exit(0);
		}
	}
	//if(!Kv_Printf("t1: %f	t2: %f", t1*1000.0f, t2*1000.0f))		exit(0);
	//if(!Kv_Printf("dist max: %f	dist min: %f", dist_max, dist_min))		exit(0);
	out_hist_dist_max = dist_max;

	return 1;
}

//*********************************************************************************************************
int CKvContourDetectionJM::efhs_Edge_Filtering_using_Histogram_Similarity(CKvMatrixUchar &in_image_gray,
	CKvMatrixFloat &in_edge_mag_image,
	CKvMatrixFloat &in_edge_ori_image,
	CKvMatrixFloat &in_gaussian_filter,
	int in_histogram_filter_type,
	int in_histogram_distance_type,
	int in_disk_size,
	int in_hist_bin_num,
	float &out_hist_dist_max,
	CKvMatrixFloat &out_edge_mag_image)
//*********************************************************************************************************
{
	CKvSplittedDiscJM zz_sd;
	CKvMatrixUchar imgs_gray[4];
	CKvMatrixFloat edge_mag[4], edge_ori[4];

	int i, j, k, n, x, y;
	int ww, hh, prev_sigma, tmp;

	float hist_sim, hist_dist, temp_ori, dist_min, dist_max, ori_max, mean, var;
	float *p_edge_mag[4], *p_edge_ori[4];
	float *p_in_edge_mag_image, *p_in_edge_ori_image, *p_out_edge_mag_image;

	p_in_edge_mag_image = in_edge_mag_image.mps(ww, hh)[0];
	p_in_edge_ori_image = in_edge_ori_image.vp();
	p_out_edge_mag_image = out_edge_mag_image.vp();

/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///Gaussian smoothing.	
	// canonical smoothing.
	for(i=0; i<4; i++)		imgs_gray[i].c_Create(hh, ww, (unsigned char)0);
	imgs_gray[0].cp_Copy(&in_image_gray);		
	for(i=0; i<3; i++){
		siff_Smooth_Image_with_Filter_Float(imgs_gray[i], in_gaussian_filter, imgs_gray[i+1]);
		siff_Smooth_Image_with_Filter_Float(imgs_gray[i+1], in_gaussian_filter, imgs_gray[i+1]);
	}

	//Create edge map.
	for(i=0; i<4; i++){		p_edge_mag[i] = edge_mag[i].c_Create(hh, ww, 0.0f)[0];		p_edge_ori[i] = edge_ori[i].c_Create(hh, ww, 0.0f)[0];	}
	//Make edge map.
	for(i=0; i<4; i++){		exso_Edge_Extraction_using_Sobel_Operator(imgs_gray[i],edge_mag[i],edge_ori[i]);  	}
/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	CKvStopWatch zz_sw;
	float t1, t2;
	t1 = t2 = 0.0f;
	zz_sw.c_Create(2);

	// +-5 degree: PI*PI/1296.0f
	// +-10 degree: PI*PI/324.0f
	ori_max = PI*PI/324.0f;		
	dist_min = 100000.0f;		dist_max = -100000.0f;
	for(j=in_disk_size; j<hh-in_disk_size; j++){
		for(i=in_disk_size; i<ww-in_disk_size; i++){
			
			tmp = j*ww+i;
			zz_sw.r_Reset(0);
			// adaptive mask size.	
// 			dms_Determine_Mask_Size_for_Histogram_Filtering(i, j,
// 				ww, hh,
// 				p_in_edge_mag_image,
// 				p_in_edge_ori_image,
// 				mask_sz);

			/// remove meaningless edges.
			// using magnitude.
			if(p_in_edge_mag_image[tmp] <= 5.0f){
				p_out_edge_mag_image[tmp] = 0.0f;
				continue;
			}
			// using orientation fluctuation.
			mean = var = 0.0f;
			for(k=0; k<4; k++){		mean+=p_edge_ori[k][tmp];		}								mean=mean/4.0f;
			for(k=0; k<4; k++){		var+=SQUARE(p_edge_ori[k][tmp]-mean);		}		var=var/4.0f;
			if(var>ori_max){
				p_out_edge_mag_image[tmp] = 0.0f;
				continue;
			}
			
			// histogram filtering.
			if(in_histogram_filter_type==KV_HIST_FILTER_TYPE_HOI){
				// using Histogram of Intensity.
				temp_ori = p_in_edge_ori_image[tmp]*180.0f/PI;
				if(temp_ori > 90.0f && temp_ori <= 270.0f)					temp_ori -= 180.0f;
				else if(temp_ori > 270.0f && temp_ori <= 360.0f)		temp_ori -= 360.0f;				
				zz_sd.cshoi_Compute_Splitted_Histogram_of_Intensity(in_image_gray,
					i, j,
					temp_ori,
					(float)in_disk_size,
					in_hist_bin_num);
			}
			else if(in_histogram_filter_type==KV_HIST_FILTER_TYPE_HOG){
				//  using Histogram of Gradient.
 				if(!zz_sd.cshog_Compute_Splitted_Histogram_of_Gradient(in_edge_mag_image,
					in_edge_ori_image,
 					i, j,
 					(float)in_disk_size,
 					in_hist_bin_num))		continue;
			}

			if(in_histogram_distance_type==KV_HIST_DISTANCE_L1_NORM)		
				hist_dist = chd_Caculate_Histogram_Distance_using_L1_Norm(zz_sd.hp_Histogram_Pointer(0),
				zz_sd.hp_Histogram_Pointer(1),
				in_hist_bin_num);
			else if(in_histogram_distance_type==KV_HIST_DISTANCE_L2_NORM)
				hist_dist = chd_Caculate_Histogram_Distance_using_L2_Norm(zz_sd.hp_Histogram_Pointer(0),
				zz_sd.hp_Histogram_Pointer(1),
				in_hist_bin_num);

			if(hist_dist > dist_max)		dist_max = hist_dist;
			if(hist_dist < dist_min)			dist_min = hist_dist;

			p_out_edge_mag_image[tmp] = hist_dist*p_in_edge_mag_image[tmp];
			
			//if(Kv_Printf("grad_x: %f, grad_y: %f, grad_mag: %f\n", grad_x, grad_y, grad_mag)==0) exit(0);
		}
	}
	//if(!Kv_Printf("t1: %f	t2: %f", t1*1000.0f, t2*1000.0f))		exit(0);
	//if(!Kv_Printf("dist max: %f	dist min: %f", dist_max, dist_min))		exit(0);

	// normalize histogram weight.
	for(i=0; i<ww*hh; i++)		p_out_edge_mag_image[i]/=dist_max;
	out_hist_dist_max = dist_max;

	return 1;
}

//*********************************************************************************************************
int CKvContourDetectionJM::efhsrih_Edge_Filtering_using_Histogram_Similarity_using_Rotated_Integral_Histograms(CKvMatrixUchar &in_gray_image,
	CKvMatrixFloat &in_edge_mag_image,
	CKvMatrixFloat &in_edge_ori_image,
	int in_histogram_filter_type,
	int in_histogram_distance_type,
	int in_disk_size,
	int in_hist_bin_num,
	float &out_hist_dist_max,
	CKvMatrixFloat &out_edge_mag_image)
//*********************************************************************************************************
{
	int ww, hh;
	int i, j, n, x, y;
	int diag_sz, offset_x, offset_y;
	float hist_sim, hist_dist, temp_ori, dist_min, dist_max, cos_theta, sin_theta, theta;
	float *p_in_edge_mag_image, *p_in_edge_ori_image, *p_out_edge_mag_image;
	float *p_hist1, *p_hist2;
	CKvSplittedDiscJM zz_sd;
	CKvHistogram zz_hist, hist1, hist2;
	vector<CKvSet_of_MatrixInt> int_hist_set;
	CKvSet_of_Pixel offsets;

	p_in_edge_mag_image = in_edge_mag_image.mps_Matrix_Pointer_and_Width_Height(ww, hh)[0];
	p_in_edge_ori_image = in_edge_ori_image.vp();
	p_out_edge_mag_image = out_edge_mag_image.vp_Vector_Pointer();
	// 	out_edge_mag_image.c_Create(hh, ww, 0,0f);
	// 	out_edge_grad_image.c_Create(hh, ww, 0,0f);

	CKvStopWatch zz_sw;
	float t1, t2;
	t1 = t2 = 0.0f;
	zz_sw.c_Create(2);

	//zz_sw.r_Reset(0);
	/// make set of rotated integral histograms of intensity.
	zz_hist.msirhoi_Make_Set_of_Rotated_Integral_Histograms_Of_Intensity(&in_gray_image,
		in_hist_bin_num,
		8,
		int_hist_set,
		offsets);
	//t1 = zz_sw.get_Get_Elapsed_Time(0);

	//CKvScreen zz_sc;
	//CKvMatrixUcharRgb temp_img;
	//temp_img.c_Create(hh, ww, Kv_Rgb(0, 0, 0));	

	//zz_sw.r_Reset(1);
	p_hist1 = hist1.c_Create(in_hist_bin_num);
	p_hist2 = hist2.c_Create(in_hist_bin_num);

	diag_sz = (int)(in_disk_size*sqrt(2.0f))+1;
	dist_min = 100000.0f;		dist_max = -100000.0f;
	for(j=diag_sz; j<hh-diag_sz; j++){
		for(i=diag_sz; i<ww-diag_sz; i++){			

			if(p_in_edge_mag_image[j*ww+i] < 10.0f){
				p_in_edge_mag_image[j*ww+i] = 0.0f;
				continue;
			}

			// using Histogram of Intensity.
			temp_ori = p_in_edge_ori_image[j*ww+i] - PI/2.0f;
			if(temp_ori < 0.0f)	temp_ori += 2*PI;	

			if(temp_ori >= PI && temp_ori <= 2*PI)				temp_ori -= PI;	
			n = (int)(temp_ori*8.0f/PI);			temp_ori = n*PI/8.0f;
			cos_theta = cos(temp_ori);		sin_theta = sin(temp_ori);

			offset_x = offsets.gpe_Get_Pointer_of_Element(n)->x;
			offset_y = offsets.gpe_Get_Pointer_of_Element(n)->y;

			//printf("%d %d %d\n", n, offset_x, offset_y);

			x = (int)(i*cos_theta - j*sin_theta) + offset_x;
			y = (int)(i*sin_theta + j*cos_theta) + offset_y; 
			
			//if(!Kv_Printf("%d %d %d\n%d %d %d", x, y, n, int_hist_set[n].vs(), int_hist_set[n].gpe(0)->mw(), int_hist_set[n].gpe(0)->mh())) exit(0);
			zz_hist.gnhih_Get_Normalized_Histogram_from_Integral_Histogram(&int_hist_set[n],
				x-in_disk_size, y-in_disk_size,
				2*in_disk_size+1, in_disk_size,
				p_hist1);
			zz_hist.gnhih_Get_Normalized_Histogram_from_Integral_Histogram(&int_hist_set[n],
				x-in_disk_size, y,
				2*in_disk_size+1, in_disk_size,
				p_hist2);
		
			hist_dist = chd_Caculate_Histogram_Distance_using_L1_Norm(p_hist1,
			p_hist2,
			in_hist_bin_num);
		
			if(hist_dist > dist_max)		dist_max = hist_dist;
			if(hist_dist < dist_min)			dist_min = hist_dist;

			//if(hist_dist>=1.0f)	if(!Kv_Printf("hist_dist: %f", hist_dist)) exit(0);

			p_out_edge_mag_image[j*ww+i] = hist_dist*p_in_edge_mag_image[j*ww+i];
			
			//if(Kv_Printf("grad_x: %f, grad_y: %f, grad_mag: %f\n", grad_x, grad_y, grad_mag)==0) exit(0);
		}
	}	
	//if(!Kv_Printf("dist max: %f	dist min: %f", dist_max, dist_min))		exit(0);
	out_hist_dist_max = dist_max;

// 	t2 = zz_sw.get_Get_Elapsed_Time(1);
// 	if(!Kv_Printf("t1: %f	t2: %f", t1*1000.0f, t2*1000.0f))		exit(0);

	return 1;
}


//*********************************************************************************************************
int CKvContourDetectionJM::exhs_Edge_Extraction_using_Histogram_Similarity(CKvMatrixUchar &in_gray_image,
	int in_disk_size,
	int in_hist_bin_num,
	float &out_max_mag,
	CKvMatrixFloat &out_edge_mag_image,
	CKvMatrixFloat &out_edge_grad_image)
//*********************************************************************************************************
{
	int ww, hh;
	int i, j;
	float grad_x, grad_y, grad_mag;
	float *p_out_edge_mag_image;
	CKvSplittedDiscJM zz_sd;
	in_gray_image.mps_Matrix_Pointer_and_Width_Height(ww, hh);
	p_out_edge_mag_image = out_edge_mag_image.vp_Vector_Pointer();
	// 	out_edge_mag_image.c_Create(hh, ww, 0,0f);
	// 	out_edge_grad_image.c_Create(hh, ww, 0,0f);
	
	CKvScreen zz_sc;
	CKvMatrixUcharRgb temp_img;
	temp_img.c_Create(hh, ww, Kv_Rgb(0, 0, 0));	
	
	out_max_mag = -1000000.0f;
	for(j=in_disk_size; j<hh-in_disk_size; j++){
		for(i=in_disk_size; i<ww-in_disk_size; i++){

			// x-gradient
			zz_sd.c_Create(in_gray_image,
				i, j,
				0.0f,
				(float)in_disk_size,
				in_hist_bin_num);

// 			grad_x = chd_Caculate_Histogram_Distance_using_L1_Norm(zz_sd.hp_Histogram_Pointer(0),
// 				zz_sd.hp_Histogram_Pointer(1),
// 				in_hist_bin_num);
			grad_x = chd_Caculate_Histogram_Dissimilarity_using_Divergence(zz_sd.hp_Histogram_Pointer(0),
				zz_sd.hp_Histogram_Pointer(1),
				in_hist_bin_num);

			// y-gradient
			zz_sd.c_Create(in_gray_image,
				i, j,
				90.0f,
				(float)in_disk_size,
				in_hist_bin_num);

// 			grad_y = chd_Caculate_Histogram_Distance_using_L1_Norm(zz_sd.hp_Histogram_Pointer(0),
// 				zz_sd.hp_Histogram_Pointer(1),
// 				in_hist_bin_num);
			grad_y = chd_Caculate_Histogram_Dissimilarity_using_Divergence(zz_sd.hp_Histogram_Pointer(0),
				zz_sd.hp_Histogram_Pointer(1),
				in_hist_bin_num);

// 			zz_sd.ssd_Show_Splitted_Disc(temp_img,
// 				Kv_RgbaF(255.0f, 0.0f, 0.0f, 1.0f),
// 				Kv_RgbaF(0.0f, 0.0f, 255.0f, 1.0f));
// 			zz_sc.s_d_Display(&temp_img);

			grad_mag = sqrt(grad_x*grad_x+grad_y*grad_y);
			p_out_edge_mag_image[j*ww+i] = grad_mag;

			if(out_max_mag<grad_mag)		out_max_mag = grad_mag;

			//if(Kv_Printf("grad_x: %f, grad_y: %f, grad_mag: %f\n", grad_x, grad_y, grad_mag)==0) exit(0);
		}
	}

	return 1;
}

//*********************************************************************************************************
int CKvContourDetectionJM::dms_Determine_Mask_Size_for_Histogram_Filtering(const int in_x, const int in_y,
												    const int in_width, const int in_height,
													const float *in_edge_mag_image,
													const float *in_edge_ori_image,
													int &out_mask_size)			// minimum mask size: 2, maximum mask size: 20
//*********************************************************************************************************
{
	int default_mask_size=10;
	int idx, bin_idx;

	if(in_x-default_mask_size < 0 || in_x+default_mask_size > in_width-1 || in_y-default_mask_size < 0 || in_y+default_mask_size > in_height-1){
		Kv_Printf("CKvContourDetectionJM::dms_Determine_Mask_Size_for_Histogram_Filtering\n--Invalid pixel position.");
		exit(0);
	}

	int total_freq, max_freq;
	int hog[8];		for(int i=0; i<8; i++)	hog[i] = 0;
	total_freq = 0;		max_freq = -1000000;

	while(default_mask_size > 2){
		for(int m=-default_mask_size; m<default_mask_size+1; m++){
			for(int k=-default_mask_size; k<default_mask_size+1; k++){

				idx = (in_y+m)*in_width+(in_x+k);
				bin_idx = (int)((in_edge_ori_image[idx]*180.0f/PI+27.5f)/45.0f);
				if(bin_idx>=8)	bin_idx -= 8;

				//if(!Kv_Printf("bin_idx: %d, edge_ori: %f", bin_idx, in_edge_ori_image[idx]))		exit(0);

				hog[bin_idx] += (int)in_edge_mag_image[idx];
				total_freq	+= (int)in_edge_mag_image[idx];
			}
		}

		for(int i=0; i<8; i++)	if(max_freq < hog[i])	max_freq = hog[i];
		if((float)max_freq/(float)total_freq > 0.3f)		break;
		default_mask_size-=2;
	}
	out_mask_size = default_mask_size;
	
// 	CKvMatrixUcharRgb hog_img;
// 	CKvHistogram hist;
// 	CKvScreen zz_sc;
// 	float hog_fl[8];
// 	for(int i=0; i<8; i++)	hog_fl[i] = (float)hog[i]/(float)total_freq;
// 	hist.ghi_Get_Histogram_Image(hog_fl,
// 		8,
// 		&hog_img,
// 		true,
// 		false);
// 	zz_sc.s_d_Display(&hog_img);
// 	if(!Kv_Printf("HoG"))		exit(0);


}

//*********************************************************************************************************
void CKvContourDetectionJM::mii_Make_Integral_Image(CKvMatrixUchar *in_image_gray,
							 CKvMatrixFloat *out_sum_image,
							 CKvMatrixFloat *out_sum_of_square_image_or_NULL)
//*********************************************************************************************************
{
	int ww, hh;
	unsigned char *p_in_image_gray;
	float *p_out_sum_image, *p_out_sum_of_square_image = NULL;

	p_in_image_gray = in_image_gray->mps(ww, hh)[0];
	p_out_sum_image = out_sum_image->vp();
	if(out_sum_of_square_image_or_NULL)		p_out_sum_of_square_image = out_sum_of_square_image_or_NULL->vp();

	for(int j=0; j<hh; j++){
		for(int i=0; i<ww; i++){

			if(j==0 && i==0)	
				p_out_sum_image[j*ww+i] = (float)p_in_image_gray[j*ww+i];
			else if(j==0)		
				p_out_sum_image[j*ww+i] = p_out_sum_image[j*ww+(i-1)] + (float)p_in_image_gray[j*ww+i];
			else if(i==0)		
				p_out_sum_image[j*ww+i] = p_out_sum_image[(j-1)*ww+i] + (float)p_in_image_gray[j*ww+i];
			else{				
				p_out_sum_image[j*ww+i] = p_out_sum_image[j*ww+(i-1)] + p_out_sum_image[(j-1)*ww+i]
				- p_out_sum_image[(j-1)*ww+(i-1)] + (float)p_in_image_gray[j*ww+i];
			}

			if(out_sum_of_square_image_or_NULL){
				
				if(j==0 && i==0)	
					p_out_sum_of_square_image[j*ww+i] = SQUARE((float)p_in_image_gray[j*ww+i]);
				else if(j==0)		
					p_out_sum_of_square_image[j*ww+i] = p_out_sum_of_square_image[j*ww+(i-1)] + SQUARE((float)p_in_image_gray[j*ww+i]);
				else if(i==0)		
					p_out_sum_of_square_image[j*ww+i] = p_out_sum_of_square_image[(j-1)*ww+i] + SQUARE((float)p_in_image_gray[j*ww+i]);
				else{				
					p_out_sum_of_square_image[j*ww+i] = p_out_sum_of_square_image[j*ww+(i-1)] + p_out_sum_of_square_image[(j-1)*ww+i]
					- p_out_sum_of_square_image[(j-1)*ww+(i-1)] + SQUARE((float)p_in_image_gray[j*ww+i]);
				}
				
			}

		}
	}

}

//*********************************************************************************************************
void CKvContourDetectionJM::gmii_Get_Mean_from_Integral_Image(CKvMatrixFloat *in_sum_image,
									   const int in_x, const int in_y,
									   const int in_mask_width, const int in_mask_height,
									   float &out_mean_value)
//*********************************************************************************************************
{	
	int ww, hh;
	int temp_x, temp_y;
	float *p_in_sum_image;
	p_in_sum_image = in_sum_image->mps(ww, hh)[0];

	if(in_x < 0 || in_y < 0 || in_x+in_mask_width > ww || in_y+in_mask_height > hh){
		Kv_Printf("[CKvContourDetectionJM::gmii_Get_Mean_from_Integral_Image]\nInvalid mask");
		exit(0);
	}
	
	temp_x = in_x-1;		temp_y = in_y-1;
	if(in_x==0 && in_y==0)	
		out_mean_value = p_in_sum_image[(temp_y+in_mask_height)*ww+(temp_x+in_mask_width)]/(in_mask_width*in_mask_height);
	else if(in_y==0)		
		out_mean_value = (p_in_sum_image[(temp_y+in_mask_height)*ww+(temp_x+in_mask_width)]
					- p_in_sum_image[(temp_y+in_mask_height)*ww+temp_x])/(in_mask_width*in_mask_height);
	else if(in_x==0)		
		out_mean_value = (p_in_sum_image[(temp_y+in_mask_height)*ww+(temp_x+in_mask_width)]
		- p_in_sum_image[temp_y*ww+(temp_x+in_mask_width)])/(in_mask_width*in_mask_height);
	else
		out_mean_value = (p_in_sum_image[(temp_y+in_mask_height)*ww+(temp_x+in_mask_width)]
		- p_in_sum_image[temp_y*ww+(temp_x+in_mask_width)] 
		- p_in_sum_image[(temp_y+in_mask_height)*ww+temp_x]
		+ p_in_sum_image[temp_y*ww+temp_x])/(in_mask_width*in_mask_height);

}

//*********************************************************************************************************
void CKvContourDetectionJM::gvii_Get_Variance_from_Integral_Image(CKvMatrixFloat *in_sum_image,
									   CKvMatrixFloat *in_sum_of_square_image,
									   const int in_x, const int in_y,
									   const int in_mask_width, const int in_mask_height,
									   float &out_variance_value)
//*********************************************************************************************************
{	
	int ww, hh;
	int temp_x, temp_y;
	float *p_in_sum_of_square_image, mean_value, sum_of_square;
	p_in_sum_of_square_image = in_sum_of_square_image->mps(ww, hh)[0];

	if(in_x < 0 || in_y < 0 || in_x+in_mask_width > ww || in_y+in_mask_height > hh){
		Kv_Printf("[CKvContourDetectionJM::gvii_Get_Variance_from_Integral_Image]\nInvalid mask");
		exit(0);
	}

	gmii_Get_Mean_from_Integral_Image(in_sum_image,
		in_x, in_y,
		in_mask_width, in_mask_height,
		mean_value);

	temp_x = in_x-1;		temp_y = in_y-1;
	if(in_x==0 && in_y==0)	
		sum_of_square = p_in_sum_of_square_image[(temp_y+in_mask_height)*ww+(temp_x+in_mask_width)];
	else if(in_y==0)		
		sum_of_square = p_in_sum_of_square_image[(temp_y+in_mask_height)*ww+(temp_x+in_mask_width)]
	- p_in_sum_of_square_image[(temp_y+in_mask_height)*ww+temp_x];
	else if(in_x==0)		
		sum_of_square = p_in_sum_of_square_image[(temp_y+in_mask_height)*ww+(temp_x+in_mask_width)]
	- p_in_sum_of_square_image[temp_y*ww+(temp_x+in_mask_width)];
	else
		sum_of_square = p_in_sum_of_square_image[(temp_y+in_mask_height)*ww+(temp_x+in_mask_width)]
	- p_in_sum_of_square_image[temp_y*ww+(temp_x+in_mask_width)] 
	- p_in_sum_of_square_image[(temp_y+in_mask_height)*ww+temp_x]
	+ p_in_sum_of_square_image[temp_y*ww+temp_x];
	
	out_variance_value = sum_of_square/(in_mask_width*in_mask_height) - SQUARE(mean_value);

	//if(!Kv_Printf("square mean: %f sum_of_square ORI: %f", mean_value, sqrt(sum_of_square/(in_mask_width*in_mask_height)))) exit(0);

}

//*********************************************************************************************************
void CKvContourDetectionJM::gtfm_Get_Threshold_from_Matrix(CKvMatrixFloat *in_matrix,
									 float in_percentage_for_threshold,
									 float &out_threshold)
//*********************************************************************************************************
{
	int ww, hh, idx;
	float *p_in_matrix, *p_matrix_vector;
	CKvVectorFloat matrix_vector;
	CKvRanker zz_rank;
	p_in_matrix = in_matrix->mps(ww, hh)[0];
	p_matrix_vector = matrix_vector.c_Create(ww*hh, 0.0f);

	for(int i=0; i<ww*hh; i++)	p_matrix_vector[i] = p_in_matrix[i];
	
	zz_rank.s_Sort(&matrix_vector, true, 0, ww*hh, NULL);	
	
	idx = (int)(in_percentage_for_threshold*ww*hh);
	out_threshold = p_matrix_vector[idx];
}

//*********************************************************************************************************
void CKvContourDetectionJM::gtfm_Get_Threshold_from_Matrix(CKvMatrixFloat *in_matrix,
									 float in_min_value_NULL, float in_max_value_or_NULL,
									 float in_percentage_for_threshold,
									 float &out_threshold)
//*********************************************************************************************************
{
	int ww, hh, idx, start_idx, end_idx;
	float *p_in_matrix, *p_matrix_vector;
	CKvVectorFloat matrix_vector;
	CKvRanker zz_rank;
	p_in_matrix = in_matrix->mps(ww, hh)[0];
	p_matrix_vector = matrix_vector.c_Create(ww*hh, 0.0f);

	for(int i=0; i<ww*hh; i++)	p_matrix_vector[i] = p_in_matrix[i];
	
	zz_rank.s_Sort(&matrix_vector, true, 0, ww*hh, NULL);	
	
	if(in_max_value_or_NULL)	for(int i=0; i<ww*hh; i++){	if(p_matrix_vector[i] <= in_max_value_or_NULL)	start_idx = i;	}
	else						start_idx = 0;
	if(in_min_value_NULL)		for(int i=0; i<ww*hh; i++){	if(p_matrix_vector[i] >= in_min_value_NULL)		end_idx = i;	}
	else						end_idx = ww*hh-1;
	
	idx = (int)(in_percentage_for_threshold*(end_idx-start_idx)+start_idx);
	out_threshold = p_matrix_vector[idx];
}

//*********************************************************************************************************
float CKvContourDetectionJM::cwhvs_Caculate_Weighted_Histogram_Vector_Similarity_using_Overlapped_Intensity(float *in_histogram1,
		 float *in_histogram2,
		 float *in_weight_vector,
		 int in_block_number,
		 int in_hist_bin_size)
//*********************************************************************************************************
{
	if(in_hist_bin_size<=0){
		Kv_Printf("[CKvContourDetectionJM::cwhvs_Caculate_Weighted_Histogram_Vector_Similarity_using_Overlapped_Intensity]\nNon-positive bin size");
		exit(0);
	}
	if(in_block_number<=0){
		Kv_Printf("[CKvContourDetectionJM::cwhvs_Caculate_Weighted_Histogram_Vector_Similarity_using_Overlapped_Intensity]\nNon-positive l\block number");
		exit(0);
	}

	float sim_total, sim;
	float *p_hist1, *p_hist2;
	
	sim=sim_total=0.0f;
	for(int i=0; i<in_block_number; i++){
		p_hist1=&in_histogram1[i*in_hist_bin_size];		p_hist2=&in_histogram2[i*in_hist_bin_size];
		sim=in_weight_vector[i]*chs_Caculate_Histogram_Similarity_using_Overlapped_Intensity(p_hist1, p_hist2, in_hist_bin_size);
// 		printf("%f %f\n", in_histogram1[i*in_hist_bin_size], in_histogram2[i*in_hist_bin_size]);
// 		printf("%d w: %f sim: %f\n", i, in_weight_vector[i], sim);
// 		system("pause");
		sim_total+=sim;
	}

	return sim_total;

}


//*********************************************************************************************************
float CKvContourDetectionJM::chs_Caculate_Histogram_Similarity_using_Overlapped_Intensity(float *in_histogram1,
																						 float *in_histogram2,
																						 int in_hist_bin_size)
//*********************************************************************************************************
{
	if(in_hist_bin_size<=0){
		Kv_Printf("[CKvContourDetectionJM::chs_Caculate_Histogram_Similarity_using_Overlapped_Intensity]\nNon-positive bin size");
		exit(0);
	}

	float sim=0.0f;
	for(int i=0; i<in_hist_bin_size; i++)	sim+=min(1.0f, max(0.0f, min(in_histogram1[i], in_histogram2[i])));	

	return sim;	
}

//*********************************************************************************************************
float CKvContourDetectionJM::chd_Caculate_Histogram_Distance_using_L1_Norm(float *in_histogram1,
																						 float *in_histogram2,
																						 int in_hist_bin_size)
//*********************************************************************************************************
{
	if(in_hist_bin_size<=0){
		Kv_Printf("[CKvContourDetectionJM::chs_Caculate_Histogram_Similarity_using_Overlapped_Intensity]\nNon-positive bin size");
		exit(0);
	}

	float sim=0.0f;
	for(int i=0; i<in_hist_bin_size; i++)		sim+=abs(in_histogram1[i]- in_histogram2[i]);	
	return sim;	
}


//*********************************************************************************************************
float CKvContourDetectionJM::chd_Caculate_Histogram_Distance_using_L2_Norm(float *in_histogram1,
																						 float *in_histogram2,
																						 int in_hist_bin_size)
//*********************************************************************************************************
{
	if(in_hist_bin_size<=0){
		Kv_Printf("[CKvContourDetectionJM::chs_Caculate_Histogram_Similarity_using_Overlapped_Intensity]\nNon-positive bin size");
		exit(0);
	}

	float sim=0.0f;
	for(int i=0; i<in_hist_bin_size; i++)		sim+=SQUARE(in_histogram1[i]- in_histogram2[i]);	
	sim = sqrt(sim);
	return sim;	
}

//*********************************************************************************************************
float CKvContourDetectionJM::chd_Caculate_Histogram_Dissimilarity_using_Divergence(float *in_histogram1,
															float *in_histogram2,
															int in_hist_bin_size)
//*********************************************************************************************************
{
	if(in_hist_bin_size<=0){
		Kv_Printf("[CKvSet_of_SuperPixel::chd_Caculate_Histogram_Dissimilarity_using_Divergence]\nNon-positive bin size");
		exit(0);
	}

	int i;
	float dissim, p1, p2, area1, area2, min_bound, temp, log2;
	log2  = 0.30103f;
	dissim=area1=area2=0.0f;
	for(i=0; i<in_hist_bin_size; i++){
		area1+=in_histogram1[i];	
		area2+=in_histogram2[i];
	}

	// calculate minimum bound.
	min_bound = 0.0001f;
	
	for(i=0; i<in_hist_bin_size; i++){
		p1 = (in_histogram1[i]+min_bound)/(area1+min_bound);
		p2 = (in_histogram2[i]+min_bound)/(area2+min_bound);
		temp = 2.0f*p1/(p1+p2);
		dissim+=p1*log(temp)/log2;
		//if(!Kv_Printf("p1: %f p2: %f dissim: %f", p1, p2, dissim)==0) exit(0);
	}

	return dissim;	
}

//*********************************************************************************************************
void CKvContourDetectionJM::ccirl_Convert_Color_Image_RGB_to_LAB(CKvMatrixUcharRgb &in_img_RGB,
										  CKvMatrixFloat &out_L_image,
										  CKvMatrixFloat &out_A_image,
										  CKvMatrixFloat &out_B_image)
//*********************************************************************************************************
{
	int i, j, idx;
	int ww, hh;
	unsigned char *p_in_R_image, *p_in_G_image, *p_in_B_image;
	double val_l_star, val_a_star, val_b_star;
	float *p_out_L_image, *p_out_A_image, *p_out_B_image;

	p_in_R_image = in_img_RGB.vp(p_in_G_image, p_in_B_image);
	p_out_L_image = out_L_image.mps(ww, hh)[0];
	p_out_A_image = out_A_image.vp();
	p_out_B_image = out_B_image.vp();

	for(j=0; j<hh; j++){
		for(i=0; i<ww; i++){
			idx = j*ww+i;
			ccvrl_Convert_Color_Value_RGB_to_LAB(p_in_R_image[idx], p_in_G_image[idx], p_in_B_image[idx],
				val_l_star, val_a_star, val_b_star);
				p_out_L_image[idx] = val_l_star;
				p_out_A_image[idx] = val_a_star;
				p_out_B_image[idx] = val_b_star;
		}
	}


}

//*********************************************************************************************************
void CKvContourDetectionJM::ccvrl_Convert_Color_Value_RGB_to_LAB(unsigned char in_sR, unsigned char in_sG, unsigned char in_sB,
									  double &out_l_star, double &out_a_star, double &out_b_star)
//*********************************************************************************************************
{
	//------------------------
	// sRGB to XYZ conversion
	//------------------------
	double X, Y, Z;
	ccvrx_Convert_Color_Value_RGB_to_XYZ(in_sR, in_sG, in_sB, 
		X, Y, Z);

	//------------------------
	// XYZ to LAB conversion
	//------------------------
	double epsilon = 0.008856;	//actual CIE standard
	double kappa   = 903.3;		//actual CIE standard

	double Xr = 0.950456;	//reference white
	double Yr = 1.0;		//reference white
	double Zr = 1.088754;	//reference white

	double xr = X/Xr;
	double yr = Y/Yr;
	double zr = Z/Zr;

	double fx, fy, fz;
	if(xr > epsilon)	fx = pow(xr, 1.0/3.0);
	else				fx = (kappa*xr + 16.0)/116.0;
	if(yr > epsilon)	fy = pow(yr, 1.0/3.0);
	else				fy = (kappa*yr + 16.0)/116.0;
	if(zr > epsilon)	fz = pow(zr, 1.0/3.0);
	else				fz = (kappa*zr + 16.0)/116.0;

	out_l_star = 116.0*fy-16.0;
	out_a_star = 500.0*(fx-fy);
	out_b_star = 200.0*(fy-fz);
}

//*********************************************************************************************************
void CKvContourDetectionJM::ccvrx_Convert_Color_Value_RGB_to_XYZ(unsigned char in_sR, unsigned char in_sG, unsigned char in_sB,
									  double &out_X, double &out_Y, double &out_Z)
//*********************************************************************************************************
{
	double R = in_sR/255.0;
	double G = in_sG/255.0;
	double B = in_sB/255.0;

	double r, g, b;

	if(R <= 0.04045)	r = R/12.92;
	else				r = pow((R+0.055)/1.055,2.4);
	if(G <= 0.04045)	g = G/12.92;
	else				g = pow((G+0.055)/1.055,2.4);
	if(B <= 0.04045)	b = B/12.92;
	else				b = pow((B+0.055)/1.055,2.4);

	out_X = r*0.4124564 + g*0.3575761 + b*0.1804375;
	out_Y = r*0.2126729 + g*0.7151522 + b*0.0721750;
	out_Z = r*0.0193339 + g*0.1191920 + b*0.9503041;
}

//*********************************************************************************************************
void CKvContourDetectionJM::ribi_Resize_Image_using_Bilinear_Interpolation(CKvMatrixUchar &in_img_gray,
									int in_new_ww,
									int in_new_hh,
									CKvMatrixUchar &out_resized_img_gray)
//*********************************************************************************************************
{
	int org_ww, org_hh, m, n, m1, n1, m2, n2;
	double rm, rn, p, q, temp;
	unsigned char **p_img;
	unsigned char **p_rsz_img;

	in_img_gray.ms(org_ww, org_hh);
	p_img = in_img_gray.mp();
	p_rsz_img = out_resized_img_gray.mp();

	for (m=0; m<in_new_hh; m++)
		for (n=0; n<in_new_ww; n++)
		{
			rm = (double)org_hh * m/in_new_hh;
			rn = (double)org_ww * n/in_new_ww;

			m1 = (int)rm;
			m2 = m1 + 1;
			if (m2 == org_hh)		m2 = org_hh - 1;

			n1 = (int)rn;
			n2 = n1 + 1;
			if (n2 == org_ww)	n2 = org_ww - 1;

			p = rn - n1;
			q = rm - m1;

			temp = (1.0-p)*(1.0-q)*p_img[m1][n1]
			+ p*(1.0-q)	  *p_img[m1][n2]
			+ (1.0-p)*q	  *p_img[m2][n1]
			+ p*q			  *p_img[m2][n2];

			p_rsz_img[m][n] = (unsigned char)temp;
		}
}

//*********************************************************************************************************
void CKvContourDetectionJM::ribi_Resize_Image_using_Bilinear_Interpolation(CKvMatrixUcharRgb &in_img,
									int in_new_ww,
									int in_new_hh,
									CKvMatrixUcharRgb &out_resized_img)
//*********************************************************************************************************
{
	int org_ww, org_hh, m, n, k, m1, n1, m2, n2, step_sz, step_sz_new, temp_step;
	float rm, rn, p, q, temp;
	unsigned char *p_img;
	unsigned char *p_rsz_img;

	in_img.ms(org_ww, org_hh);
	p_img = in_img.vp();
	p_rsz_img = out_resized_img.vp();
	step_sz = org_ww*org_hh;
	step_sz_new = in_new_ww*in_new_hh;
	
	for (m=0; m<in_new_hh; m++){
		for (n=0; n<in_new_ww; n++){
		
			// Find sub-pixel position.
			rm = (float)org_hh * (float)m/(float)in_new_hh;
			rn = (float)org_ww * (float)n/(float)in_new_ww;

			m1 = (int)rm;
			m2 = m1 + 1;
			if (m2 == org_hh) 	m2 = org_hh - 1;

			n1 = (int)rn;
			n2 = n1 + 1;
			if (n2 == org_ww)	n2 = org_ww - 1;

			p = rn - n1;
			q = rm - m1;

			for(k=0; k<3; k++){
				// Bilinear color interpolation.
				temp_step = k*step_sz;
				temp = (1.0f-p)*(1.0f-q)*(float)(p_img[m1*org_ww+n1+temp_step])
				+ p*(1.0f-q)	  *(float)(p_img[m1*org_ww+n2+temp_step])
				+ (1.0f-p)*q	  *(float)(p_img[m2*org_ww+n1+temp_step])
				+ p*q				 *(float)(p_img[m2*org_ww+n2+temp_step]);

				p_rsz_img[m*in_new_ww+n+k*step_sz_new] = (unsigned char)temp;
			
			}
		}
	}
}

//*********************************************************************************************************
void CKvContourDetectionJM::gplgm_Get_Pixel_with_Largest_Gradient_Magnitudes(CKvMatrixFloat &in_grad_mag_image,
													 int in_x, int in_y,
													  int in_size_of_roi,
													  int &out_x, int &out_y)
//*********************************************************************************************************
{
	int i, j, ww, hh;
	float max_grad;
	float *p_in_grad_mag_image;
	p_in_grad_mag_image = in_grad_mag_image.mps(ww, hh)[0];

	if(in_x<in_size_of_roi || in_x>=ww-in_size_of_roi || in_y<in_size_of_roi || in_y>=hh-in_size_of_roi){
		Kv_Printf("[CKvContourDetectionJM::gplgm_Get_Pixel_with_Largest_Gradient_Magnitudes]\n Invalid pixel position.");
		exit(0);
	}
	
	max_grad = -1000000.0f;
	out_x = out_y = 0;
	for(j=in_y-in_size_of_roi; j<in_y+in_size_of_roi; j++){
		for(i=in_x-in_size_of_roi; i<in_x+in_size_of_roi; i++){
			if(p_in_grad_mag_image[j*ww+hh] > max_grad){
				max_grad = p_in_grad_mag_image[j*ww+hh];
				out_x = i;		out_y = j;
			}
		}
	}

}

//*********************************************************************************************************
void CKvContourDetectionJM::hfac_Hole_Filling_using_Average_Color(CKvMatrixUcharRgb &io_image_color)
//*********************************************************************************************************
{
	int i, j, k, m, idx;
	int ww, hh;
	int sum_color[3], cnt;
	unsigned char *p_io_image_r, *p_io_image_g, *p_io_image_b, threshold;

	io_image_color.ms(ww, hh);
	p_io_image_r = io_image_color.vp(p_io_image_g, p_io_image_b);

	threshold = (unsigned char)250;
	for(j=1; j<hh-1; j++){
		for(i=1; i<ww-1; i++){

			if(p_io_image_r[j*ww+i] > threshold && p_io_image_g[j*ww+i] > threshold && p_io_image_b[j*ww+i] > threshold){
			
				cnt = sum_color[0] = sum_color[1] = sum_color[2] = 0;
				for(m=-1; m<2; m++){
					for(k=-1; k<2; k++){

						if(m==0 && k==0)		continue;

						idx=(j+m)*ww+i+k;					
						if(p_io_image_r[idx] < threshold || p_io_image_g[idx] < threshold || p_io_image_b[idx] < threshold){
							sum_color[0] += p_io_image_r[idx];
							sum_color[1] += p_io_image_g[idx];
							sum_color[2] += p_io_image_b[idx];
							cnt++;
						}
					}			
				}
				if(cnt>0){
					for(k=0; k<3; k++)	sum_color[k] = sum_color[k]/cnt;
					p_io_image_r[j*ww+i] = (unsigned char)sum_color[0];
					p_io_image_g[j*ww+i] = (unsigned char)sum_color[1];
					p_io_image_b[j*ww+i] = (unsigned char)sum_color[2];
				}
			}
			

		}
	}
}

//*********************************************************************************************************
void CKvContourDetectionJM::dmf_Display_Matrix_Float(CKvScreen *in_screen,
						  CKvMatrixFloat &in_matrix,
						  int in_normalized_mode)
//*********************************************************************************************************
{
	CKvMatrixUchar disp_img;
	int ww, hh, idx;
	unsigned char *p_disp_img;
	float *p_in_matrix, max_val, min_val, offset, scale;
	
	p_in_matrix = in_matrix.mps(ww, hh)[0];
	p_disp_img = disp_img.c_Create(hh, ww, (unsigned char)0)[0];
	if(in_normalized_mode){
		min_val = 1.0e7;
		max_val = -1.0e7;
		for(int j=0; j<hh; j++){
			for(int i=0; i<ww; i++){
				int idx = j*ww+i;
				if(max_val < p_in_matrix[idx])	max_val = p_in_matrix[idx];
				if(min_val > p_in_matrix[idx])	min_val = p_in_matrix[idx];
			}	
		}
	}
	offset = -min_val;
	scale = 255.0f/(max_val-min_val);

	for(int j=0; j<hh; j++){
		for(int i=0; i<ww; i++){			
			idx = j*ww+i;
			if(in_normalized_mode)	p_disp_img[idx] = (unsigned char)(scale*(p_in_matrix[idx]+offset));
			else					p_disp_img[idx] = (unsigned char)max(min(255.0f, p_in_matrix[idx]), 0.0f);
		}
	}

	in_screen->s_d_Display(&disp_img);
}

//*********************************************************************************************************
void CKvContourDetectionJM::dmf_Display_Matrix_Float(CKvScreen *in_screen,
						  CKvMatrixFloat &in_matrix,
						  float in_threshold)
//*********************************************************************************************************
{
	CKvMatrixUchar disp_img;
	int ww, hh, idx;
	unsigned char *p_disp_img;
	float *p_in_matrix, max_val;
	
	p_in_matrix = in_matrix.mps(ww, hh)[0];
	p_disp_img = disp_img.c_Create(hh, ww, (unsigned char)0)[0];

	for(int j=0; j<hh; j++){
		for(int i=0; i<ww; i++){			
			idx = j*ww+i;
			if(p_in_matrix[idx] > in_threshold)	p_disp_img[idx] = (unsigned char)255;
		}
	}
	in_screen->s_d_Display(&disp_img);
}

//*********************************************************************************************************
void CKvContourDetectionJM::dmfpc_Display_Matrix_Float_using_Pseudo_Color(CKvScreen *in_screen,
						  CKvMatrixFloat &in_matrix,
						  int in_normalized_mode)
//*********************************************************************************************************
{
	CKvMatrixUcharRgb disp_img;
	int ww, hh, idx;
	int pseudo_r, pseudo_g, pseudo_b;
	unsigned char *p_disp_img, temp_val;
	float *p_in_matrix, max_val, min_val, offset, scale;
	
	p_in_matrix = in_matrix.mps(ww, hh)[0];
	p_disp_img = disp_img.c_Create(hh, ww)[0];
	if(in_normalized_mode){
		min_val = 1.0e7;
		max_val = -1.0e7;
		for(int j=0; j<hh; j++){
			for(int i=0; i<ww; i++){
				int idx = j*ww+i;
				if(max_val < p_in_matrix[idx])	max_val = p_in_matrix[idx];
				if(min_val > p_in_matrix[idx])		min_val = p_in_matrix[idx];
			}	
		}
	}
	offset = -min_val;
	scale = 255.0f/(max_val-min_val);

	for(int j=0; j<hh; j++){
		for(int i=0; i<ww; i++){			
			idx = j*ww+i;
			if(in_normalized_mode)	temp_val = (unsigned char)(scale*(p_in_matrix[idx]+offset));
			else					temp_val = (unsigned char)max(min(255.0f, p_in_matrix[idx]), 0.0f);
			
			// original pseudo coloring.
// 			pseudo_r = 128*((-sin((float)temp_val*2*PI/256.0f)+1.0f))-1;
// 			pseudo_g = 128*((-cos((float)temp_val*2*PI/256.0f)+1.0f))-1;
// 			pseudo_b = 128*((sin((float)temp_val*2*PI/256.0f)+1.0f))-1;
			// new pseudo coloring.
			if(temp_val>=0 && temp_val<=63)				{		pseudo_r=0;		pseudo_g=(int)(255.0f/63.0f*(float)temp_val);		pseudo_b=255;	}
			else if(temp_val>=63 && temp_val<=127)	{		pseudo_r=0;		pseudo_g=255;		pseudo_b=255-(int)(255.0f/(127.0f-63.0f)*((float)temp_val-63.0f));	}
			else if(temp_val>=127 && temp_val<=191)	{		pseudo_r=(int)(255.0f/(191.0f-127.0f)*((float)temp_val-127.0f));		pseudo_g=255;		pseudo_b=0;	}
			else																			{		pseudo_r=255;		pseudo_g=255-(int)(255.0f/(255.0f-191.0f)*((float)temp_val-191.0f));		pseudo_b=0;	}
	

			p_disp_img[idx] = (unsigned char)pseudo_r;			idx+=ww*hh;
			p_disp_img[idx] = (unsigned char)pseudo_g;		idx+=ww*hh;
			p_disp_img[idx] = (unsigned char)pseudo_b;		idx+=ww*hh;
		}
	}

	in_screen->s_d_Display(&disp_img);
}

//*********************************************************************************************************
void CKvContourDetectionJM::deh_Display_Edge_Histogram_XY(CKvScreen *in_screen,
						  CKvMatrixFloat &in_edge_mag_image,
						  float in_edge_mag_threshold)
//*********************************************************************************************************
{
	CKvMatrixUchar disp_img;
	int ww, hh, idx, freq;
	unsigned char *p_disp_img;
	float *p_in_edge_mag_image;
	
	p_in_edge_mag_image = in_edge_mag_image.mps(ww, hh)[0];
	p_disp_img = disp_img.c_Create(hh, ww, (unsigned char)0)[0];

	// y-axis projection.
	for(int j=0; j<hh; j++){
		freq = 0;
		for(int i=0; i<ww; i++){			
			idx = j*ww+i;
			if(p_in_edge_mag_image[idx] > in_edge_mag_threshold)		freq++;
		}
		for(int i=ww-1; i>ww-1-freq; i--)		p_disp_img[j*ww+i] = (unsigned char)255;
	}

	// x-axis projection.
	for(int i=0; i<ww; i++){
		freq = 0;
		for(int j=0; j<hh; j++){			
			idx = j*ww+i;
			if(p_in_edge_mag_image[idx] > in_edge_mag_threshold)		freq++;
		}
		for(int j=hh-1; j>hh-1-freq; j--)		p_disp_img[j*ww+i] = (unsigned char)255;
	}

	in_screen->s_d_Display(&disp_img);
}