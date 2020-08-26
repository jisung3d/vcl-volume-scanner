/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_contour_detection.cpp 
/////////////////////////////////////////////////////////////////////////////////////////////

#include "../../../stdafx.h"
#include "../../_yooji_2016_object_scanning.h"

//********************************************************************************************
CKvYooji_ContourDetection::CKvYooji_ContourDetection(void)
//********************************************************************************************
{
	zz_classname="CKvYooji_ContourDetection";
}

//********************************************************************************************
CKvYooji_ContourDetection::~CKvYooji_ContourDetection(void)
//********************************************************************************************
{
}

//********************************************************************************************
bool CKvYooji_ContourDetection::cmm_Compute_Magnitude_Map(
	CKvMatrixFloat &in_map_grad_x,
	CKvMatrixFloat &in_map_grad_y,
	CKvMatrixFloat &out_map_grad_mag)
//********************************************************************************************
{
	int ww, hh;

	float *p_grad_x, *p_grad_y, *p_grad_mag;

	in_map_grad_x.ms(ww, hh);
	if(in_map_grad_y.mw() != ww || in_map_grad_y.mh() != hh) return false;
	if(out_map_grad_mag.mw() != ww || out_map_grad_mag.mh() != hh) out_map_grad_mag.c_Create(hh, ww, 0.0f);

	// get pointers.
	p_grad_x = in_map_grad_x.vp();
	p_grad_y = in_map_grad_y.vp();
	p_grad_mag = out_map_grad_mag.vp();

	for(int i=0; i<ww*hh; i++) p_grad_mag[i] = sqrt(SQUARE(p_grad_x[i]) + SQUARE(p_grad_y[i]));


	return true;
}

//********************************************************************************************
bool CKvYooji_ContourDetection::clm_Compute_Laplacian_Map(
	CKvMatrixFloat &in_map_grad_xx,
	CKvMatrixFloat &in_map_grad_yy,
	CKvMatrixFloat &out_map_laplacian)
//********************************************************************************************
{
	int ww,hh;

	float *p_grad_xx,*p_grad_yy,*p_laplacian;

	in_map_grad_xx.ms(ww,hh);
	if(in_map_grad_yy.mw() != ww || in_map_grad_yy.mh() != hh) return false;
	if(out_map_laplacian.mw() != ww || out_map_laplacian.mh() != hh) out_map_laplacian.c_Create(hh,ww,0.0f);

	// get pointers.
	p_grad_xx = in_map_grad_xx.vp();
	p_grad_yy = in_map_grad_yy.vp();
	p_laplacian = out_map_laplacian.vp();

	for(int i=0; i<ww*hh; i++) p_laplacian[i] = p_grad_xx[i] + p_grad_yy[i];

	return true;
}

//********************************************************************************************
bool CKvYooji_ContourDetection::clm_Compute_Laplacian_Map(
	CKvMatrixUchar &in_gray_image,
	CKvMatrixFloat &out_map_laplacian)
//********************************************************************************************
{
	int ww,hh;
	int i,j,tmp;
	float tLap,grad_y;

	unsigned char *p_img_gray;
	float *p_laplace;
	p_img_gray = in_gray_image.mps_Matrix_Pointer_and_Width_Height(ww,hh)[0];

	if(ww != out_map_laplacian.mw() || hh != out_map_laplacian.mh())	out_map_laplacian.c_Create(hh,ww,float(0));

	p_laplace = out_map_laplacian.vp();

	for(j=1; j<hh-1; j++){
		for(i=1; i<ww-1; i++){

			// BASIC method.
			tLap = 8*(float)p_img_gray[j*ww + i];
			for(int m=-1; m<=1; m++){
				for(int n=-1; n<=1; n++){

					if(m == 0 && n == 0) continue;

					tLap -= p_img_gray[(j+m)*ww + (i+n)];

				}
			}

			p_laplace[j*ww + i] = tLap/8.0f;

		}
	}

	return true;
}

//********************************************************************************************
int CKvYooji_ContourDetection::cig_Compute_Image_Gradients(
	CKvMatrixUchar &in_gray_image,
	CKvMatrixFloat &out_map_grad_x,
	CKvMatrixFloat &out_map_grad_y)
//********************************************************************************************
{
	int ww, hh;
	int i, j, p1, p2, tmp;
	float grad_x, grad_y;
	
	unsigned char *p_img_gray;
	float *p_grad_x, *p_grad_y;
	p_img_gray = in_gray_image.mps_Matrix_Pointer_and_Width_Height(ww, hh)[0];

	if(ww != out_map_grad_x.mw() || hh != out_map_grad_x.mh())	out_map_grad_x.c_Create(hh, ww, float(0));
	if(ww != out_map_grad_y.mw() || hh != out_map_grad_y.mh())	out_map_grad_y.c_Create(hh, ww, float(0));

	p_grad_x = out_map_grad_x.vp();
	p_grad_y = out_map_grad_y.vp();

	for(j=1; j<hh-1; j++){
		for(i=1; i<ww-1; i++){
			
			p2=j*ww + i+1;		p1=j*ww + i-1;		
			p_grad_x[j*ww + i] =	0.5f*((float)p_img_gray[p2] - (float)p_img_gray[p1]);

			//printf("gradient: %d %d -> %f\n", p_img_gray[p2], p_img_gray[p1], p_grad_x[j*ww + i]);

			p2=(j+1)*ww + i;		p1=(j-1)*ww + i;
			p_grad_y[j*ww + i] =	0.5f*((float)p_img_gray[p2] - (float)p_img_gray[p1]);			
	
		}
	}

	return 1;
}

//********************************************************************************************
int CKvYooji_ContourDetection::cig_Compute_Image_Gradients(
	CKvMatrixFloat &in_gray_image,
	CKvMatrixFloat &out_map_grad_x,
	CKvMatrixFloat &out_map_grad_y)
//********************************************************************************************
{
	int ww, hh;
	int i, j, p1, p2, tmp;
	float grad_x, grad_y;
	
	float *p_img_gray;
	float *p_grad_x, *p_grad_y;
	p_img_gray = in_gray_image.mps_Matrix_Pointer_and_Width_Height(ww, hh)[0];

	if(ww != out_map_grad_x.mw() || hh != out_map_grad_x.mh())	out_map_grad_x.c_Create(hh, ww, float(0));
	if(ww != out_map_grad_y.mw() || hh != out_map_grad_y.mh())	out_map_grad_y.c_Create(hh, ww, float(0));

	p_grad_x = out_map_grad_x.vp();
	p_grad_y = out_map_grad_y.vp();

	for(j=1; j<hh-1; j++){
		for(i=1; i<ww-1; i++){

			p2=j*ww + i+1;		p1=j*ww + i;
			grad_x =	(p_img_gray[p2] - p_img_gray[p1]);

			p2=(j+1)*ww + i;		p1=(j)*ww + i;
			grad_y =	(p_img_gray[p2] - p_img_gray[p1]);

			p_grad_x[j*ww + i] = grad_x;
			p_grad_y[j*ww + i] = grad_y;

		}
	}

	return 1;
}


//********************************************************************************************
int CKvYooji_ContourDetection::cigs_Compute_Image_Gradients_using_Sobel(
	CKvMatrixUchar &in_gray_image,
	CKvMatrixFloat &out_map_grad_x,
	CKvMatrixFloat &out_map_grad_y)
//********************************************************************************************
{
	int ww, hh;
	int i, j, tmp;
	float grad_x, grad_y;
	
	unsigned char *p_img_gray;
	float *p_grad_x, *p_grad_y;
	p_img_gray = in_gray_image.mps_Matrix_Pointer_and_Width_Height(ww, hh)[0];

	if(ww != out_map_grad_x.mw() || hh != out_map_grad_x.mh())	out_map_grad_x.c_Create(hh, ww, float(0));
	if(ww != out_map_grad_y.mw() || hh != out_map_grad_y.mh())	out_map_grad_y.c_Create(hh, ww, float(0));

	p_grad_x = out_map_grad_x.vp();
	p_grad_y = out_map_grad_y.vp();

	for(j=1; j<hh-1; j++){
		for(i=1; i<ww-1; i++){

			// BASIC method.
			tmp=(j-1) *ww+i+1;		grad_x =	((float)p_img_gray[tmp]+2*(float)p_img_gray[tmp+ww]+(float)p_img_gray[tmp+2*ww]);
			tmp=(j-1) *ww+i-1;		grad_x -=	((float)p_img_gray[tmp]+2*(float)p_img_gray[tmp+ww]+(float)p_img_gray[tmp+2*ww]);
			tmp=(j+1) *ww+i-1;		grad_y =	((float)p_img_gray[tmp]+2*(float)p_img_gray[tmp+1]+(float)p_img_gray[tmp+2]);
			tmp=(j-1) *ww+i-1;		grad_y -=	((float)p_img_gray[tmp]+2*(float)p_img_gray[tmp+1]+(float)p_img_gray[tmp+2]);
			
			p_grad_x[j*ww + i] = grad_x/8.0f;
			p_grad_y[j*ww + i] = grad_y/8.0f;

		}
	}

	return 1;
}

//********************************************************************************************
int CKvYooji_ContourDetection::cigs_Compute_Image_Gradients_using_Sobel(
	CKvMatrixFloat &in_image_gray,
	CKvMatrixFloat &out_map_grad_x,
	CKvMatrixFloat &out_map_grad_y,
	float in_scale)
//********************************************************************************************
{
	int ww,hh;
	int i,j,tmp;
	float grad_x,grad_y;

	float *p_img_gray;
	float *p_grad_x,*p_grad_y;
	p_img_gray = in_image_gray.mps_Matrix_Pointer_and_Width_Height(ww,hh)[0];

	if(ww != out_map_grad_x.mw() || hh != out_map_grad_x.mh())	out_map_grad_x.c_Create(hh,ww,float(0));
	if(ww != out_map_grad_y.mw() || hh != out_map_grad_y.mh())	out_map_grad_y.c_Create(hh,ww,float(0));

	p_grad_x = out_map_grad_x.vp();
	p_grad_y = out_map_grad_y.vp();

	for(j=1; j<hh-1; j++){
		for(i=1; i<ww-1; i++){

			// BASIC method.
			tmp=(j-1) *ww+i+1;		grad_x =	(p_img_gray[tmp]+2*p_img_gray[tmp+ww]+p_img_gray[tmp+2*ww]);
			tmp=(j-1) *ww+i-1;		grad_x -=	(p_img_gray[tmp]+2*p_img_gray[tmp+ww]+p_img_gray[tmp+2*ww]);
			tmp=(j+1) *ww+i-1;		grad_y =	(p_img_gray[tmp]+2*p_img_gray[tmp+1]+p_img_gray[tmp+2]);
			tmp=(j-1) *ww+i-1;		grad_y -=	(p_img_gray[tmp]+2*p_img_gray[tmp+1]+p_img_gray[tmp+2]);

			p_grad_x[j*ww + i] = (in_scale*grad_x)/8.0f;
			p_grad_y[j*ww + i] = (in_scale*grad_y)/8.0f;

			//if(in_scale != 1.0f) printf("grad: %f %f\n",grad_x,grad_y);

		}
	}

	return 1;
}

//********************************************************************************************
int CKvYooji_ContourDetection::cigs_Compute_Image_Gradients_using_Scharr(
	CKvMatrixUchar &in_gray_image,
	CKvMatrixFloat &out_map_grad_x,
	CKvMatrixFloat &out_map_grad_y)
//********************************************************************************************
{
	int ww, hh;
	int i, j, tmp;
	float grad_x, grad_y;
	
	unsigned char *p_img_gray;
	float *p_grad_x, *p_grad_y;
	p_img_gray = in_gray_image.mps_Matrix_Pointer_and_Width_Height(ww, hh)[0];

	if(ww != out_map_grad_x.mw() || hh != out_map_grad_x.mh())	out_map_grad_x.c_Create(hh, ww, float(0));
	if(ww != out_map_grad_y.mw() || hh != out_map_grad_y.mh())	out_map_grad_y.c_Create(hh, ww, float(0));

	p_grad_x = out_map_grad_x.vp();
	p_grad_y = out_map_grad_y.vp();

	for(j=1; j<hh-1; j++){
		for(i=1; i<ww-1; i++){

			// BASIC method.
			tmp=(j-1) *ww+i+1;		grad_x =	(3*(float)p_img_gray[tmp]+10*(float)p_img_gray[tmp+ww]+3*(float)p_img_gray[tmp+2*ww]);
			tmp=(j-1) *ww+i-1;		grad_x -=	(3*(float)p_img_gray[tmp]+10*(float)p_img_gray[tmp+ww]+3*(float)p_img_gray[tmp+2*ww]);
			tmp=(j+1) *ww+i-1;		grad_y =	(3*(float)p_img_gray[tmp]+10*(float)p_img_gray[tmp+1]+3*(float)p_img_gray[tmp+2]);
			tmp=(j-1) *ww+i-1;		grad_y -=	(3*(float)p_img_gray[tmp]+10*(float)p_img_gray[tmp+1]+3*(float)p_img_gray[tmp+2]);
			
			p_grad_x[j*ww + i] = grad_x/32.0f;
			p_grad_y[j*ww + i] = grad_y/32.0f;

		}
	}

	return 1;
}

//********************************************************************************************
int CKvYooji_ContourDetection::cigs_Compute_Image_Gradients_using_Scharr(
	CKvMatrixFloat &in_image_gray,
	CKvMatrixFloat &out_map_grad_x,
	CKvMatrixFloat &out_map_grad_y)
//********************************************************************************************
{
	int ww,hh;
	int i,j,tmp;
	float grad_x,grad_y;

	float *p_img_gray;
	float *p_grad_x,*p_grad_y;
	p_img_gray = in_image_gray.mps_Matrix_Pointer_and_Width_Height(ww,hh)[0];

	if(ww != out_map_grad_x.mw() || hh != out_map_grad_x.mh())	out_map_grad_x.c_Create(hh,ww,float(0));
	if(ww != out_map_grad_y.mw() || hh != out_map_grad_y.mh())	out_map_grad_y.c_Create(hh,ww,float(0));

	p_grad_x = out_map_grad_x.vp();
	p_grad_y = out_map_grad_y.vp();

	for(j=1; j<hh-1; j++){
		for(i=1; i<ww-1; i++){

			// BASIC method.
			tmp=(j-1) *ww+i+1;		grad_x =	(3*p_img_gray[tmp]+10*p_img_gray[tmp+ww]+3*p_img_gray[tmp+2*ww]);
			tmp=(j-1) *ww+i-1;		grad_x -=	(3*p_img_gray[tmp]+10*p_img_gray[tmp+ww]+3*p_img_gray[tmp+2*ww]);
			tmp=(j+1) *ww+i-1;		grad_y =	(3*p_img_gray[tmp]+10*p_img_gray[tmp+1]+3*p_img_gray[tmp+2]);
			tmp=(j-1) *ww+i-1;		grad_y -=	(3*p_img_gray[tmp]+10*p_img_gray[tmp+1]+3*p_img_gray[tmp+2]);

			p_grad_x[j*ww + i] = grad_x/32.0f;
			p_grad_y[j*ww + i] = grad_y/32.0f;

			//if(p_img_gray[j*ww + i]) printf("%f %f\n", grad_x, grad_y);

		}
	}

	return 1;
}


//********************************************************************************************
int CKvYooji_ContourDetection::ees_Extract_Edges_using_Sobel(
	CKvMatrixUchar &in_gray_image,
	CKvMatrixFloat &out_edge_mag_image,
	CKvMatrixFloat &out_edge_ori_image)	// gradient range: 0 ~ 2*PI radian
//********************************************************************************************
{
	int ww, hh;
	int i, j, tmp;
	float grad_x, grad_y, grad_mag, grad_ori;
	
	unsigned char *p_in_gray_image;
	float *p_out_edge_mag_image, *p_out_edge_ori_image;
	p_in_gray_image = in_gray_image.mps_Matrix_Pointer_and_Width_Height(ww, hh)[0];

	if(ww != out_edge_mag_image.mw() || hh != out_edge_mag_image.mh())	out_edge_mag_image.c_Create(hh, ww, 0.0f);
	if(ww != out_edge_ori_image.mw() || hh != out_edge_ori_image.mh())	out_edge_ori_image.c_Create(hh, ww, 0.0f);

	p_out_edge_mag_image = out_edge_mag_image.vp();
	p_out_edge_ori_image = out_edge_ori_image.vp();

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
			p_out_edge_ori_image[j*ww+i] = (grad_ori<0) ? (grad_ori+2.0f*(float)PI) : grad_ori;

		}
	}

	return 1;
}

//*********************************************************************************************************
void CKvYooji_ContourDetection::ribi_Resize_Image_using_Bilinear_Interpolation(CKvMatrixUchar &in_img,
									int in_width_new,
									int in_height_new,
									CKvMatrixUchar &out_img_resized)
//*********************************************************************************************************
{
	int orgW, orgH, m, n, m1, n1, m2, n2;
	double rm, rn, p, q, temp;
	unsigned char **pImg;
	unsigned char **pResizeImg;

	in_img.ms_Matrix_Width_Height(orgW, orgH);
	pImg = in_img.mp_Matrix_Pointer();
	pResizeImg = out_img_resized.mp_Matrix_Pointer();

	for (m=0; m<in_height_new; m++)
		for (n=0; n<in_width_new; n++)
		{
			rm = (double)orgH * m/in_height_new;
			rn = (double)orgW * n/in_width_new;

			m1 = (int)rm;
			m2 = m1 + 1;
			if (m2 == orgH)
				m2 = orgH - 1;

			n1 = (int)rn;
			n2 = n1 + 1;
			if (n2 == orgW)
				n2 = orgW - 1;

			p = rn - n1;
			q = rm - m1;

			temp = (1.0-p)*(1.0-q)*pImg[m1][n1]
			+ p*(1.0-q)	  *pImg[m1][n2]
			+ (1.0-p)*q	  *pImg[m2][n1]
			+ p*q			  *pImg[m2][n2];

			pResizeImg[m][n] = (unsigned char)temp;
		}
}

//*********************************************************************************************************
void CKvYooji_ContourDetection::ribi_Resize_Image_using_Bilinear_Interpolation(CKvMatrixUcharRgb &in_img_rgb,
									int in_width_new,
									int in_height_new,
									CKvMatrixUcharRgb &out_img_resized)
//*********************************************************************************************************
{
	int orgW, orgH, k, m, n, m1, n1, m2, n2;
	double rm, rn, p, q, temp;
	unsigned char *pImg;
	unsigned char *pResizeImg;
		
	pImg = in_img_rgb.mps(orgW, orgH)[0];
	pResizeImg = out_img_resized.mp()[0];

	for (m=0; m<in_height_new; m++)
		for (n=0; n<in_width_new; n++)
		{
			rm = (double)orgH * m/in_height_new;
			rn = (double)orgW * n/in_width_new;

			m1 = (int)rm;
			m2 = m1 + 1;
			if (m2 == orgH)
				m2 = orgH - 1;

			n1 = (int)rn;
			n2 = n1 + 1;
			if (n2 == orgW)
				n2 = orgW - 1;

			p = rn - n1;
			q = rm - m1;

			for(k=0; k<3; k++){
				temp = (1.0-p)*(1.0-q)*pImg[(m1 + k*orgH)*orgW + n1]
				+ p*(1.0-q)	  *pImg[(m1 + k*orgH)*orgW + n2]
				+ (1.0-p)*q	  *pImg[(m2 + k*orgH)*orgW + n1]
				+ p*q			  *pImg[(m2 + k*orgH)*orgW + n2];

				pResizeImg[(m + k*in_height_new)*in_width_new + n] = (unsigned char)temp;
			}
			
		}
}


//*********************************************************************************************************
void CKvYooji_ContourDetection::mii_Make_Integral_Image(CKvMatrixUchar *in_image_gray,
	CKvMatrixFloat *out_sum_image,
	CKvMatrixFloat *out_sum_of_square_image_or_NULL)
//*********************************************************************************************************
{
	int ww, hh;
	unsigned char *p_in_image_gray;
	float *p_out_sum_image, *p_out_sum_of_square_image;

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
void CKvYooji_ContourDetection::mii_Make_Integral_Image(
	CKvMatrixFloat *in_image,
	CKvMatrixFloat *out_sum_image,
	CKvMatrixFloat *out_sum_of_square_image_or_NULL)
//*********************************************************************************************************
{
	int ww, hh;
	float *p_in_image = NULL;
	float *p_out_sum_image = NULL, *p_out_sum_of_square_image = NULL;

	p_in_image = in_image->mps(ww, hh)[0];
	p_out_sum_image = out_sum_image->vp();
	if(out_sum_of_square_image_or_NULL)	 p_out_sum_of_square_image = out_sum_of_square_image_or_NULL->vp();

	for(int j=0; j<hh; j++){
		for(int i=0; i<ww; i++){

			if(j==0 && i==0)	
				p_out_sum_image[j*ww+i] = p_in_image[j*ww+i];
			else if(j==0)		
				p_out_sum_image[j*ww+i] = p_out_sum_image[j*ww+(i-1)] + p_in_image[j*ww+i];
			else if(i==0)		
				p_out_sum_image[j*ww+i] = p_out_sum_image[(j-1)*ww+i] + p_in_image[j*ww+i];
			else{				
				p_out_sum_image[j*ww+i] = p_out_sum_image[j*ww+(i-1)] + p_out_sum_image[(j-1)*ww+i]
				- p_out_sum_image[(j-1)*ww+(i-1)] + p_in_image[j*ww+i];
			}

			if(out_sum_of_square_image_or_NULL){
				
				if(j==0 && i==0)	
					p_out_sum_of_square_image[j*ww+i] = SQUARE(p_in_image[j*ww+i]);
				else if(j==0)		
					p_out_sum_of_square_image[j*ww+i] = p_out_sum_of_square_image[j*ww+(i-1)] + SQUARE(p_in_image[j*ww+i]);
				else if(i==0)		
					p_out_sum_of_square_image[j*ww+i] = p_out_sum_of_square_image[(j-1)*ww+i] + SQUARE(p_in_image[j*ww+i]);
				else{				
					p_out_sum_of_square_image[j*ww+i] = p_out_sum_of_square_image[j*ww+(i-1)] + p_out_sum_of_square_image[(j-1)*ww+i]
					- p_out_sum_of_square_image[(j-1)*ww+(i-1)] + SQUARE(p_in_image[j*ww+i]);
				}
				
			}

		}
	}

}

//*********************************************************************************************************
bool CKvYooji_ContourDetection::glsii_Get_Local_Sum_from_Integral_Image(
	CKvMatrixFloat *in_integ_image,
	const int in_x,const int in_y,
	const int in_dx,const int in_dy,
	float &out_sum)
//*********************************************************************************************************
{	
	int ww, hh;
	int temp_x, temp_y;
	float *p_in_sum_image;
	p_in_sum_image = in_integ_image->mps(ww, hh)[0];

	if(in_x < 0 || in_y < 0 || in_x+in_dx > ww || in_y+in_dy > hh) return false;
	
	temp_x = in_x-1;		temp_y = in_y-1;
	if(in_x==0 && in_y==0)	
		out_sum = p_in_sum_image[(temp_y + in_dy)*ww + (temp_x + in_dx)];
	else if(in_y==0)		
		out_sum = (p_in_sum_image[(temp_y + in_dy)*ww + (temp_x + in_dx)]
					- p_in_sum_image[(temp_y + in_dy)*ww + temp_x]);
	else if(in_x==0)		
		out_sum = (p_in_sum_image[(temp_y + in_dy)*ww + (temp_x + in_dx)]
		- p_in_sum_image[temp_y*ww + (temp_x + in_dx)]);
	else
		out_sum = (p_in_sum_image[(temp_y + in_dy)*ww + (temp_x + in_dx)]
		- p_in_sum_image[temp_y*ww + (temp_x + in_dx)] 
		- p_in_sum_image[(temp_y + in_dy)*ww + temp_x]
		+ p_in_sum_image[temp_y*ww + temp_x]);

}

//*********************************************************************************************************
void CKvYooji_ContourDetection::gmii_Get_Mean_from_Integral_Image(CKvMatrixFloat *in_sum_image,
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

	float sum;
	glsii_Get_Local_Sum_from_Integral_Image(in_sum_image, in_x, in_y, in_mask_width, in_mask_height, sum);
	out_mean_value = sum/float(in_mask_width*in_mask_height);

// 	temp_x = in_x-1;		temp_y = in_y-1;
// 	if(in_x==0 && in_y==0)	
// 		out_mean_value = p_in_sum_image[(temp_y+in_mask_height)*ww+(temp_x+in_mask_width)]/(in_mask_width*in_mask_height);
// 	else if(in_y==0)		
// 		out_mean_value = (p_in_sum_image[(temp_y+in_mask_height)*ww+(temp_x+in_mask_width)]
// 					- p_in_sum_image[(temp_y+in_mask_height)*ww+temp_x])/(in_mask_width*in_mask_height);
// 	else if(in_x==0)		
// 		out_mean_value = (p_in_sum_image[(temp_y+in_mask_height)*ww+(temp_x+in_mask_width)]
// 		- p_in_sum_image[temp_y*ww+(temp_x+in_mask_width)])/(in_mask_width*in_mask_height);
// 	else
// 		out_mean_value = (p_in_sum_image[(temp_y+in_mask_height)*ww+(temp_x+in_mask_width)]
// 		- p_in_sum_image[temp_y*ww+(temp_x+in_mask_width)] 
// 		- p_in_sum_image[(temp_y+in_mask_height)*ww+temp_x]
// 		+ p_in_sum_image[temp_y*ww+temp_x])/(in_mask_width*in_mask_height);

}

//*********************************************************************************************************
void CKvYooji_ContourDetection::gvii_Get_Variance_from_Integral_Image(CKvMatrixFloat *in_sum_image,
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