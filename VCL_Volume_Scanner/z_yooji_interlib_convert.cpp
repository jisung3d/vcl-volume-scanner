#include "stdafx.h"

//********************************************************************************************
CKvYooji_InterLib_Convert::CKvYooji_InterLib_Convert(void)
//********************************************************************************************
{

}

//********************************************************************************************
CKvYooji_InterLib_Convert::~CKvYooji_InterLib_Convert(void)
//********************************************************************************************
{

}

//********************************************************************************************
void CKvYooji_InterLib_Convert::cfko_Convert_Format_from_KAISION_to_Opencv(CKvMatrixUchar &in_img,
	Mat &out_img)
//********************************************************************************************
{
	int ww_in, hh_in, ww_out, hh_out;

	in_img.ms(ww_in, hh_in);
	ww_out = out_img.cols;	hh_out = out_img.rows;

	if (ww_in != ww_out || hh_in != hh_out){
		ww_out = ww_in;	 hh_out = hh_in;
		out_img = Mat::zeros(hh_out, ww_out, CV_8UC1);
	}

	memcpy(out_img.data, in_img.vp(), sizeof(UCHAR)*ww_in*hh_in);
}

//********************************************************************************************
void CKvYooji_InterLib_Convert::cfko_Convert_Format_from_KAISION_to_Opencv(CKvMatrixUcharRgb &in_img,
	Mat &out_img)
//********************************************************************************************
{
	UCHAR *in_r, *in_g, *in_b;
	int ww_in, hh_in, ww_out, hh_out;

	in_img.ms(ww_in, hh_in);
	ww_out = out_img.cols;	hh_out = out_img.rows;

	if (ww_in != ww_out || hh_in != hh_out){
		ww_out = ww_in;	 hh_out = hh_in;
		out_img = Mat::zeros(hh_out, ww_out, CV_8UC3);
	}

	// KAISION color image. ( [R_1 ... R_N] [G_1 ... G_N] [B_1 ... B_N] )
	in_r = in_img.vp(in_g, in_b);
	// OpenCV color image. ( [B_1 G_1 R_1] ...  [B_N G_N R_N])
	for (int j = 0; j < hh_in; j++){		
		for (int i = 0; i < ww_in; i++){

			out_img.data[3 * (j * ww_in + i) + 0] = in_b[j * ww_in + i];	// b
			out_img.data[3 * (j * ww_in + i) + 1] = in_g[j * ww_in + i];	// g
			out_img.data[3 * (j * ww_in + i) + 2] = in_r[j * ww_in + i];  // r

		}
	}
}

//********************************************************************************************
void CKvYooji_InterLib_Convert::cfko_Convert_Format_from_KAISION_to_Opencv(
	CKvMatrixFloat &in_mat,
	Mat &out_mat,
	int in_output_type)
//********************************************************************************************
{
	int ww_in, hh_in, ww_out, hh_out;

	in_mat.ms(ww_in, hh_in);
	ww_out = out_mat.cols;	hh_out = out_mat.rows;
	
	if (ww_in != ww_out || hh_in != hh_out){
		ww_out = ww_in;	 hh_out = hh_in;
		out_mat = Mat::zeros(hh_out, ww_out, in_output_type);
	}

	for (int j = 0; j < hh_in; j++){
		for (int i = 0; i < ww_in; i++){

			if (in_output_type == CV_64F) out_mat.at<double>(j, i) = in_mat.ge_Get_Element(i, j);
			else if (in_output_type == CV_32F) out_mat.at<float>(j, i) = in_mat.ge_Get_Element(i, j);

		}
	}
}

//********************************************************************************************
void CKvYooji_InterLib_Convert::cfko_Convert_Format_from_KAISION_to_Opencv(
	CKvMatrix &in_mat,
	Mat &out_mat,
	int in_output_type)
//********************************************************************************************
{
	int ww_in, hh_in, ww_out, hh_out;

	in_mat.ms(ww_in, hh_in);
	ww_out = out_mat.cols;	hh_out = out_mat.rows;
	
	if (ww_in != ww_out || hh_in != hh_out){
		ww_out = ww_in;	 hh_out = hh_in;
		out_mat = Mat::zeros(hh_out, ww_out, in_output_type);
	}

	for (int j = 0; j < hh_in; j++){
		for (int i = 0; i < ww_in; i++){

			if (in_output_type == CV_64F) out_mat.at<double>(j, i) = in_mat.ge_Get_Element(i, j);
			else if (in_output_type == CV_32F) out_mat.at<float>(j, i) = in_mat.ge_Get_Element(i, j);

		}
	}
}


//********************************************************************************************
void CKvYooji_InterLib_Convert::cfok_Convert_Format_from_Opencv_to_KAISION(
	Mat &in_img,
	CKvMatrixUcharRgb &out_img)
//********************************************************************************************
{
	UCHAR *out_r, *out_g, *out_b;
	int ww_in, hh_in, ww_out, hh_out;
		
	ww_in = in_img.cols;	hh_in = in_img.rows;
	out_img.ms(ww_out, hh_out);

	if (ww_in != ww_out || hh_in != hh_out){
		ww_out = ww_in;	 hh_out = hh_in;
		out_img.c_Create(hh_out, ww_out);
	}

	// KAISION color image. ( [R_1 ... R_N] [G_1 ... G_N] [B_1 ... B_N] )
	out_r = out_img.vp(out_g, out_b);
	// OpenCV color image. ( [B_1 G_1 R_1] ...  [B_N G_N R_N])
	for (int j = 0; j < hh_in; j++){
		for (int i = 0; i < ww_in; i++){

			out_b[j * ww_in + i] = in_img.data[3 * (j * ww_in + i) + 0];	// b
			out_g[j * ww_in + i] = in_img.data[3 * (j * ww_in + i) + 1];	// g
			out_r[j * ww_in + i] = in_img.data[3 * (j * ww_in + i) + 2];	// r

		}
	}
}

//********************************************************************************************
void CKvYooji_InterLib_Convert::cfok_Convert_Format_from_Opencv_to_KAISION(
	Mat &in_mat,
	CKvMatrixFloat &out_mat)
//********************************************************************************************
{
	int ww_in, hh_in, ww_out, hh_out;

	ww_in = in_mat.cols;	hh_in = in_mat.rows;
	out_mat.ms(ww_out, hh_out);

	if (ww_in != ww_out || hh_in != hh_out){
		ww_out = ww_in;	 hh_out = hh_in;
		out_mat.c_Create(hh_out, ww_out, 0.0f);
	}

	for (int j = 0; j < hh_in; j++){
		for (int i = 0; i < ww_in; i++){

			if (in_mat.type() == CV_64F)		out_mat.se_Set_Element(i, j, in_mat.at<double>(j, i));
			else if (in_mat.type() == CV_32F)	out_mat.se_Set_Element(i, j, in_mat.at<float>(j, i));

		}
	}
}