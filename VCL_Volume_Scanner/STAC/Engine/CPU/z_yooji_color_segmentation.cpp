/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_color_segmentation.cpp 
/////////////////////////////////////////////////////////////////////////////////////////////

#include "../../../stdafx.h"
#include "../../_yooji_2016_object_scanning.h"

//********************************************************************************
// ****************************** Color segmentation *****************************
//********************************************************************************

//********************************************************************************
CKvYooji_ColorModel::CKvYooji_ColorModel(void)
//********************************************************************************
{
	zz_classname = "CKvYooji_ColorModel";

	zz_class_num = zz_gaussian_num = zz_color_type = zz_feat_dim = -1;
	zz_flag_full_cov = true;

	zz_set_of_mean_vec.clear();
	zz_set_of_cov_mat_full.clear();			zz_set_of_cov_mat_inv_full.clear();
	zz_set_of_cov_mat_diag.clear();			zz_set_of_cov_mat_inv_diag.clear();
	zz_set_of_det_of_cov_mat.clear();

	zz_set_of_weights.clear();
}

//********************************************************************************
CKvYooji_ColorModel::~CKvYooji_ColorModel(void)
//********************************************************************************
{
	zz_set_of_mean_vec.clear();
	zz_set_of_cov_mat_full.clear();			zz_set_of_cov_mat_inv_full.clear();
	zz_set_of_cov_mat_diag.clear();			zz_set_of_cov_mat_inv_diag.clear();
	zz_set_of_det_of_cov_mat.clear();

	zz_set_of_weights.clear();
}

//***********************************************************************************************************************
CKvYooji_ColorModel::CKvYooji_ColorModel(CKvYooji_ColorModel &a)
//***********************************************************************************************************************
{
	cp_Copy(&a);
}

//********************************************************************************
void CKvYooji_ColorModel::c_Create(int in_class_num, int in_gaussian_num, int in_color_type,
	bool in_flag_full_cov)
//********************************************************************************
{
	if (in_class_num <= 0){
		printf("[CKvColorModel::c_Create] Non-positive number of classes.\n");
		system("pause");
		exit(0);
	}

	if (in_gaussian_num <= 0){
		printf("[CKvColorModel::c_Create] Non-positive number of models.\n");
		system("pause");
		exit(0);
	}

	zz_set_of_mean_vec.clear();
	zz_set_of_cov_mat_full.clear();		zz_set_of_cov_mat_inv_full.clear();
	zz_set_of_cov_mat_diag.clear();		zz_set_of_cov_mat_inv_diag.clear();
	zz_set_of_weights.clear();			zz_set_of_det_of_cov_mat.clear();
	
	zz_class_num = in_class_num;
	zz_gaussian_num = in_gaussian_num;
	zz_flag_full_cov = in_flag_full_cov;

	if (in_color_type<0 || in_color_type>2)		zz_color_type = KV_COLOR_MODEL_RGB;
	else										zz_color_type = in_color_type;

	if (zz_color_type == KV_COLOR_MODEL_HUE)		zz_feat_dim = 1;
	else											zz_feat_dim = 3;

 	printf("1111\n");
// 	CKvVectorFloat tsvf11;	tsvf11.c_Create(1, 0.0f);
// 	printf("aaaa\n");
// 	zz_test_vec.reserve(zz_class_num); 
// 	printf("bbbb\n");
// 	for(int i=0; i<zz_class_num; i++) zz_test_vec.push_back(tsvf11);
	zz_set_of_mean_vec.resize(zz_class_num);
	printf("2222\n");
	if(zz_flag_full_cov){
		zz_set_of_cov_mat_full.resize(zz_class_num);		zz_set_of_cov_mat_inv_full.resize(zz_class_num);
	}
	else{
		zz_set_of_cov_mat_diag.resize(zz_class_num);		zz_set_of_cov_mat_inv_diag.resize(zz_class_num);
	}	
	printf("3333\n");
	zz_set_of_weights.resize(zz_class_num);					zz_set_of_det_of_cov_mat.resize(zz_class_num);

	printf("4444\n");
	for (int i = 0; i<zz_class_num; i++){
		// initialize mean vector.
		zz_set_of_mean_vec[i].c_Create(zz_gaussian_num);
		for(int k = 0; k<zz_gaussian_num; k++){
			zz_set_of_mean_vec[i].gpe(k)->c_Create(zz_feat_dim,0.0f);
		}
		// initialize covariance matrix.
		if(zz_flag_full_cov){
			zz_set_of_cov_mat_full[i].c_Create(zz_gaussian_num);
			zz_set_of_cov_mat_inv_full[i].c_Create(zz_gaussian_num);
			for(int k = 0; k<zz_gaussian_num; k++){
				zz_set_of_cov_mat_full[i].gpe(k)->c_Create(zz_feat_dim,zz_feat_dim,0.0f);
				zz_set_of_cov_mat_inv_full[i].gpe(k)->c_Create(zz_feat_dim,zz_feat_dim,0.0f);
			}
		}
		else{
			zz_set_of_cov_mat_diag[i].c_Create(zz_gaussian_num);
			zz_set_of_cov_mat_inv_diag[i].c_Create(zz_gaussian_num);
			for(int k = 0; k<zz_gaussian_num; k++){
				zz_set_of_cov_mat_diag[i].gpe(k)->c_Create(zz_feat_dim,0.0f);
				zz_set_of_cov_mat_inv_diag[i].gpe(k)->c_Create(zz_feat_dim,0.0f);
			}
		}		

		// initialize weights and determinant of cov mat.
		zz_set_of_weights[i].c_Create(zz_gaussian_num, 0.0f);
		zz_set_of_det_of_cov_mat[i].c_Create(zz_gaussian_num, 0.0f);
	}

	zz_tvec1.c_Create(zz_feat_dim, 0.0f);
	zz_tvec2.c_Create(zz_feat_dim, 0.0f);
	zz_tvec3.c_Create(zz_feat_dim, 0.0f);
}

//********************************************************************************
void CKvYooji_ColorModel::cp_Copy(CKvYooji_ColorModel *in_color_model)
//********************************************************************************
{
	int in_class_num, in_gaussian_num, in_color_type;

	in_class_num = in_color_model->gnc_Get_Number_of_Classes();
	in_gaussian_num = in_color_model->gnm_Get_Number_of_Models();
	in_color_type = in_color_model->gct_Get_Color_Type();

	if (in_class_num <= 0){
		printf("[CKvColorModel::cp_Copy] Non-positive number of classes.\n");
		system("pause");
		exit(0);
	}

	if (in_gaussian_num <= 0){
		printf("[CKvColorModel::cp_Copy] Non-positive number of models.\n");
		system("pause");
		exit(0);
	}

	CKvVectorFloat *p_mean_vec = NULL, *p_cov_mat_diag = NULL, *p_cov_mat_diag_inv = NULL;
	CKvMatrixFloat *p_cov_mat = NULL, *p_cov_mat_inv = NULL;
	float *p_w, *p_det, *tmp_w, *tmp_det;

	zz_set_of_mean_vec.clear();
	zz_set_of_cov_mat_full.clear();		zz_set_of_cov_mat_inv_full.clear();
	zz_set_of_cov_mat_diag.clear();		zz_set_of_cov_mat_inv_diag.clear();
	zz_set_of_weights.clear();			zz_set_of_det_of_cov_mat.clear();

	zz_class_num = in_class_num;
	zz_gaussian_num = in_gaussian_num;
	zz_flag_full_cov = in_color_model->gffc_Get_Flag_of_Full_Covariance();

	if (in_color_type<0 || in_color_type>2)		zz_color_type = KV_COLOR_MODEL_RGB;
	else										zz_color_type = in_color_type;

	if (zz_color_type == KV_COLOR_MODEL_HUE)		zz_feat_dim = 1;
	else											zz_feat_dim = 3;

	zz_set_of_mean_vec.resize(zz_class_num);
	if(zz_flag_full_cov){
		zz_set_of_cov_mat_full.resize(zz_class_num);		zz_set_of_cov_mat_inv_full.resize(zz_class_num);
	}
	else{
		zz_set_of_cov_mat_diag.resize(zz_class_num);		zz_set_of_cov_mat_inv_diag.resize(zz_class_num);
	}	
	zz_set_of_weights.resize(zz_class_num);				zz_set_of_det_of_cov_mat.resize(zz_class_num);

	for (int i = 0; i<zz_class_num; i++){
		zz_set_of_mean_vec[i].c_Create(zz_gaussian_num);
		if(zz_flag_full_cov){
			zz_set_of_cov_mat_full[i].c_Create(zz_gaussian_num);
			zz_set_of_cov_mat_inv_full[i].c_Create(zz_gaussian_num);
		}
		else{
			zz_set_of_cov_mat_diag[i].c_Create(zz_gaussian_num);
			zz_set_of_cov_mat_inv_diag[i].c_Create(zz_gaussian_num);
		}
		// get pointers of input color model.
		p_mean_vec = in_color_model->gpmvs_Get_Pointer_of_Mean_Vector_Set(i);
		if(zz_flag_full_cov){
			p_cov_mat = in_color_model->gpcms_Get_Pointer_of_Covariance_Matrix_Set(i);
			p_cov_mat_inv = in_color_model->gpcmis_Get_Pointer_of_Covariance_Matrix_Inverse_Set(i);
		}
		else{
			p_cov_mat_diag = in_color_model->gpcmds_Get_Pointer_of_Covariance_Matrix_Diagonal_Set(i);
			p_cov_mat_diag_inv = in_color_model->gpcmids_Get_Pointer_of_Covariance_Matrix_Inverse_Diagonal_Set(i);
		}
		p_w = in_color_model->gpmw_Get_Pointer_of_Model_Weights(i);
		p_det = in_color_model->gpdcm_Get_Pointer_of_Determinant_of_Covariance_Matrix(i);
		// create.
		tmp_w = zz_set_of_weights[i].c_Create(zz_gaussian_num, 0.0f);
		tmp_det = zz_set_of_det_of_cov_mat[i].c_Create(zz_gaussian_num, 0.0f);

		// copy.
		for (int k = 0; k<zz_gaussian_num; k++){
			zz_set_of_mean_vec[i].gpe(k)->cp_Copy(&p_mean_vec[k]);//zz_set_of_mean_vec[i].gpe(k)->c_Create(zz_feat_dim, 0.0f);
			if(zz_flag_full_cov){
				zz_set_of_cov_mat_full[i].gpe(k)->cp_Copy(&p_cov_mat[k]);	//zz_set_of_cov_mat_diag[i].gpe(k)->c_Create(zz_feat_dim, 0.0f);		
				zz_set_of_cov_mat_inv_full[i].gpe(k)->cp_Copy(&p_cov_mat_inv[k]);//zz_set_of_cov_mat_inv_diag[i].gpe(k)->c_Create(zz_feat_dim, 0.0f);
			}
			else{
				zz_set_of_cov_mat_diag[i].gpe(k)->cp_Copy(&p_cov_mat_diag[k]);	//zz_set_of_cov_mat_diag[i].gpe(k)->c_Create(zz_feat_dim, 0.0f);		
				zz_set_of_cov_mat_inv_diag[i].gpe(k)->cp_Copy(&p_cov_mat_diag_inv[k]);//zz_set_of_cov_mat_inv_diag[i].gpe(k)->c_Create(zz_feat_dim, 0.0f);

// 				printf("zz_set_of_cov_mat_diag: %f %f %f (%f)\n",p_cov_mat_diag_inv[k].vp()[0]
// 					,p_cov_mat_diag_inv[k].vp()[1],p_cov_mat_diag_inv[k].vp()[2]
// 					,p_det[k]);
			}
			tmp_w[k] = p_w[k];
			tmp_det[k] = p_det[k];
		}


	}

}

//********************************************************************************
void CKvYooji_ColorModel::cnm_Change_Number_of_Models(int in_gaussian_num)
//********************************************************************************
{
	if (in_gaussian_num>zz_set_of_mean_vec[0].vs()){
		printf("[CKvColorModel::cnm_Change_Number_of_Models] Number of models is larger than maximum capacity.\n");
		system("pause");
		exit(0);
	}

	zz_gaussian_num = in_gaussian_num;
}

//********************************************************************************
CKvVectorFloat* CKvYooji_ColorModel::gpmvs_Get_Pointer_of_Mean_Vector_Set(int in_class_idx)
//********************************************************************************
{
	if (in_class_idx >= zz_class_num || in_class_idx<0){
		printf("[CKvColorModel::gpmvs_Get_Pointer_of_Mean_Vector_Set] Invalid class index.\n");
		system("pause");
		exit(0);

		return NULL;
	}

	return zz_set_of_mean_vec[in_class_idx].vp();
}

//********************************************************************************
CKvMatrixFloat* CKvYooji_ColorModel::gpcms_Get_Pointer_of_Covariance_Matrix_Set(int in_class_idx)
//********************************************************************************
{
	if (in_class_idx >= zz_class_num || in_class_idx<0){
		printf("[CKvColorModel::gpcms_Get_Pointer_of_Covariance_Matrix_Set] Invalid class index.\n");
		system("pause");
		exit(0);

		return NULL;
	}

	return zz_set_of_cov_mat_full[in_class_idx].vp();
}

//********************************************************************************
CKvMatrixFloat* CKvYooji_ColorModel::gpcmis_Get_Pointer_of_Covariance_Matrix_Inverse_Set(int in_class_idx)
//********************************************************************************
{
	if (in_class_idx >= zz_class_num || in_class_idx<0){
		printf("[CKvColorModel::gpcmis_Get_Pointer_of_Covariance_Matrix_Inverse_Set] Invalid class index.\n");
		system("pause");
		exit(0);

		return NULL;
	}

	return zz_set_of_cov_mat_inv_full[in_class_idx].vp();
}

//********************************************************************************
CKvVectorFloat* CKvYooji_ColorModel::gpcmds_Get_Pointer_of_Covariance_Matrix_Diagonal_Set(int in_class_idx)
//********************************************************************************
{
	if (in_class_idx >= zz_class_num || in_class_idx<0){
		printf("[CKvColorModel::gpcmds_Get_Pointer_of_Covariance_Matrix_Diagonal_Set] Invalid class index.\n");
		system("pause");
		exit(0);

		return NULL;
	}

	return zz_set_of_cov_mat_diag[in_class_idx].vp();
}

//********************************************************************************
CKvVectorFloat* CKvYooji_ColorModel::gpcmids_Get_Pointer_of_Covariance_Matrix_Inverse_Diagonal_Set(int in_class_idx)
//********************************************************************************
{
	if (in_class_idx >= zz_class_num || in_class_idx<0){
		printf("[CKvColorModel::gpcmids_Get_Pointer_of_Covariance_Matrix_Inverse_Diagonal_Set] Invalid class index.\n");
		system("pause");
		exit(0);

		return NULL;
	}

	return zz_set_of_cov_mat_inv_diag[in_class_idx].vp();
}

//********************************************************************************
float* CKvYooji_ColorModel::gpmw_Get_Pointer_of_Model_Weights(int in_class_idx)
//********************************************************************************
{
	if (in_class_idx >= zz_class_num || in_class_idx<0){
		printf("[CKvColorModel::gpmw_Get_Pointer_of_Model_Weights] Invalid class index.\n");
		system("pause");
		exit(0);

		return NULL;
	}

	return zz_set_of_weights[in_class_idx].vp();
}

//********************************************************************************
float* CKvYooji_ColorModel::gpdcm_Get_Pointer_of_Determinant_of_Covariance_Matrix(int in_class_idx)
//********************************************************************************
{
	if (in_class_idx >= zz_class_num || in_class_idx<0){
		printf("[CKvColorModel::gpdcm_Get_Pointer_of_Determinant_of_Covariance_Matrix] Invalid class index.\n");
		system("pause");
		exit(0);

		return NULL;
	}

	return zz_set_of_det_of_cov_mat[in_class_idx].vp();
}

//********************************************************************************
float CKvYooji_ColorModel::gpdw_Get_Probability_Density_Weighted(
	CKvVectorFloat *in_sample_feature,
	int in_class_idx)
//********************************************************************************
{
	int feat_dim;
	float mahal_dist = 0.0f, det, pdf_val, pdf_val_w;
	float *p_w;

	feat_dim = in_sample_feature->vs();
	p_w = gpmw_Get_Pointer_of_Model_Weights(in_class_idx);

	pdf_val_w = 0.0f;
 	for(int i=0; i<zz_gaussian_num; i++){
 // 		det = gpdcm_Get_Pointer_of_Determinant_of_Covariance_Matrix(in_class_idx)[i];
 // 		if(det<0.000001)	continue;
 // 		mahal_dist = gmd_Get_Mahalanobis_Distance(in_sample_feature, in_class_idx, i, in_flag_full_cov_mat);
 // 		pdf_val = exp(-0.5f*mahal_dist) / sqrt(pow(2.0f*PI, feat_dim)*det);
 // 		
 		pdf_val = gpd_Get_Probability_Density(in_sample_feature, in_class_idx, i);
 		pdf_val_w += p_w[i]*pdf_val;
 
 		if(p_w[i] > 1.0f || p_w[i] < 0.0f)	
 			printf("p_w: %f / mahal: %f / pdf_val: %f \n", p_w[i], mahal_dist, pdf_val);
 	}

	return pdf_val_w;

}

//********************************************************************************
// Mahalanobis distance가 음수가 나올 때가 간간히 존재한다...
// 절대값이 작은것으로 보아선 covariance matrix와 그 inverse를 구할 때
// 발생하는 float 연산 오차로 발생하는듯...
float CKvYooji_ColorModel::gpd_Get_Probability_Density(CKvVectorFloat *in_sample_feature,
	int in_class_idx,
	int in_model_idx)
//********************************************************************************
{
	int feat_dim;
	float mahal_dist = 0.0f, det, pdf_val;

	feat_dim = in_sample_feature->vs();
	det = gpdcm_Get_Pointer_of_Determinant_of_Covariance_Matrix(in_class_idx)[in_model_idx];
	if(det < 0.00001)	return 0.0f;
	mahal_dist = gmd_Get_Mahalanobis_Distance(in_sample_feature, in_class_idx, in_model_idx);
	//if(mahal_dist < -10e-7)	return 0.0f;
	// pdf
	pdf_val = exp(-0.5f*mahal_dist) / (float)sqrt(pow(2.0f*PI, feat_dim)*det);

	//printf("const: %f\n", (float)sqrt(pow(2.0f*PI, feat_dim)*det));
	
	if(pdf_val > 1.0f/*this threshold should be changed*/ || pdf_val < 0.0f)	return 0.0f;

	return pdf_val;

}

//********************************************************************************
float CKvYooji_ColorModel::gmd_Get_Mahalanobis_Distance(CKvVectorFloat *in_sample_feature,
	int in_class_idx,
	int in_model_idx)
//********************************************************************************
{
	int feat_dim;
	float mahal_dist = 0.0f;
	float *p_samp_feat, *p_mean_vec, *p_cov_mat_inv, *p_cov_mat_inv_diag;
	float *p_tvec1, *p_tvec2;

	p_cov_mat_inv = p_cov_mat_inv_diag = NULL;

	feat_dim = in_sample_feature->vs();

	p_samp_feat = in_sample_feature->vp();
	p_mean_vec = zz_set_of_mean_vec[in_class_idx].gpe(in_model_idx)->vp();

	p_tvec1 = zz_tvec1.vp();	p_tvec2 = zz_tvec2.vp();

	// x-mu
	//for (int k = 0; k<feat_dim; k++)	p_tvec1[k] = -100.0f;
	for (int k = 0; k<feat_dim; k++)	p_tvec1[k] = p_samp_feat[k] - p_mean_vec[k];
	// x_t*Cov_inv*x
	if(!zz_flag_full_cov){
		p_cov_mat_inv_diag = zz_set_of_cov_mat_inv_diag[in_class_idx].gpe(in_model_idx)->vp();
		for (int k = 0; k<feat_dim; k++)	mahal_dist = p_tvec1[k] * p_cov_mat_inv_diag[k] * p_tvec1[k];

		if(mahal_dist < -0.00001f){
			printf("mahal_dist: %f ",mahal_dist);
			d_pm_Printf_Matrix(p_cov_mat_inv_diag,1,3,"CovI");
		}
	}
	else{
		float tval=0.0f;

		p_cov_mat_inv = zz_set_of_cov_mat_inv_full[in_class_idx].gpe(in_model_idx)->vp();	

		// x_t*Cov_inv		
		for(int k=0; k<feat_dim; k++){
			tval=0.0f;
			//////////////////////////////////////////////////////////////////////////
			// p_tvec1 index 오류였음!!
			for(int i=0; i<feat_dim; i++)	tval += p_tvec1[i]*p_cov_mat_inv[i*feat_dim + k];
			//////////////////////////////////////////////////////////////////////////
			p_tvec2[k] = tval;			
		}
		// x_t*Cov_inv*x
		mahal_dist = 0.0f;
		d_ipv_Inner_Product_Vector(p_tvec2, p_tvec1, feat_dim, mahal_dist);

		if(mahal_dist < -0.00001f){
			float mat[9];
			printf("mahal_dist: %f (%f %f %f) \n", mahal_dist, p_tvec1[0], p_tvec1[1], p_tvec1[2]);
			
			d_mms_Multiply_Matrix_Square(p_cov_mat_inv, zz_set_of_cov_mat_full[in_class_idx].gpe(in_model_idx)->vp(), 3, mat);
			d_pm_Printf_Matrix(mat, 3, 3, "I");
		}
		else{
// 			float mat[9];
// 			printf("mahal_dist: %f (%f %f %f) \n",mahal_dist,p_tvec1[0],p_tvec1[1],p_tvec1[2]);
// 
// 			d_mms_Multiply_Matrix_Square(p_cov_mat_inv,zz_set_of_cov_mat_full[in_class_idx].gpe(in_model_idx)->vp(),3,mat);
// 			d_pm_Printf_Matrix(zz_set_of_cov_mat_full[in_class_idx].gpe(in_model_idx)->vp(),3,3,"Cov");
// 			d_pm_Printf_Matrix(p_cov_mat_inv,3,3,"CovI");
// 			d_pm_Printf_Matrix(mat,3,3,"I");
		}
// 
// 		d_pm_Printf_Matrix(p_samp_feat, 1, 3, "feat");
// 		d_pm_Printf_Matrix(p_mean_vec, 1, 3, "mean");
// 
// 		d_pm_Printf_Matrix(p_tvec1, 1, 3, "p_tvec1");
// 		d_pm_Printf_Matrix(p_tvec2, 1, 3, "p_tvec2");
// 
// 		if(!Kv_Printf("mahal_dist: %f", mahal_dist))	exit(0);
		
	}

	return mahal_dist;
}

//********************************************************************************
float CKvYooji_ColorModel::gpdc_Get_Probability_Density_Circular(CKvVectorFloat *in_sample_feature,
	int in_class_idx,
	int in_model_idx)
	//********************************************************************************
{
	int feat_dim;
	float mahal_dist, det, pdf_val;

	feat_dim = in_sample_feature->vs();
	det = gpdcm_Get_Pointer_of_Determinant_of_Covariance_Matrix(in_class_idx)[in_model_idx];
	mahal_dist = gmdc_Get_Mahalanobis_Distance_Circular(in_sample_feature, in_class_idx, in_model_idx);
	// pdf
	pdf_val = exp(-0.5f*mahal_dist) / (float)sqrt(pow(2.0f*PI, feat_dim)*det);

	return pdf_val;

}

//********************************************************************************
float CKvYooji_ColorModel::gmdc_Get_Mahalanobis_Distance_Circular(CKvVectorFloat *in_sample_feature,
	int in_class_idx,
	int in_model_idx)
	//********************************************************************************
{
	int feat_dim, k;
	float mahal_dist;
	float *p_samp_feat, *p_mean_vec, *p_cov_mat_inv_diag, *p_tmp_vec;

	feat_dim = in_sample_feature->vs();

	p_samp_feat = in_sample_feature->vp();
	p_mean_vec = zz_set_of_mean_vec[in_class_idx].gpe(in_model_idx)->vp();
	p_cov_mat_inv_diag = zz_set_of_cov_mat_inv_diag[in_class_idx].gpe(in_model_idx)->vp();
	p_tmp_vec = zz_tvec1.vp();

	// x-mu
	for (k = 0; k<feat_dim; k++){
		mahal_dist = abs(p_samp_feat[k] - p_mean_vec[k]);
		p_tmp_vec[k] = DIST_CIRCULAR(mahal_dist);
	}
	// x_t*Cov_inv*x
	for (k = 0; k<feat_dim; k++)	mahal_dist = p_tmp_vec[k] * p_cov_mat_inv_diag[k] * p_tmp_vec[k];

	return mahal_dist;

}


//********************************************************************************
void CKvYooji_ColorModel::scm_Save_Color_Model(CKvString in_save_file)
//********************************************************************************
{
	const char *bp = (const char*)in_save_file.bp();
	std::ofstream f(bp);
	
	CKvYooji_ColorModel tmp_cm;
	CKvRanker zz_rank;
	CKvVectorInt model_indices;
	CKvMatrixFloat *p_cov_mat_full = NULL, *p_tmp_cov_mat_full = NULL;
	CKvVectorFloat *p_mean_vec = NULL, *p_cov_mat_diag = NULL, *p_tmp_mean_vec = NULL, *p_tmp_cov_mat_diag = NULL;
	CKvVectorFloat tmp_vec;
	int *p_mod_ind;
	float *p_w;
	float *p_tmp_vec;

	f << zz_class_num << " " << zz_gaussian_num << " " << zz_feat_dim << " " << zz_color_type << " "
		<< zz_flag_full_cov << endl;

	if (zz_class_num <= 0) gerr("1");
	
	// sorting model sequence.
 	tmp_cm.cp_Copy(this);
 	p_mod_ind = model_indices.c_Create(zz_gaussian_num, 0);
 	p_tmp_vec = tmp_vec.c_Create(zz_gaussian_num, 0.0f);
 	for (int i = 0; i<zz_class_num; i++){
 		// target pointers.
 		p_mean_vec = gpmvs_Get_Pointer_of_Mean_Vector_Set(i);
		if(zz_flag_full_cov) p_cov_mat_full = gpcms_Get_Pointer_of_Covariance_Matrix_Set(i);
		else  p_cov_mat_diag = gpcmds_Get_Pointer_of_Covariance_Matrix_Diagonal_Set(i);
 		p_w = gpmw_Get_Pointer_of_Model_Weights(i);
 		// source pointers.
 		p_tmp_mean_vec = tmp_cm.gpmvs_Get_Pointer_of_Mean_Vector_Set(i);
		if(zz_flag_full_cov) p_tmp_cov_mat_full = tmp_cm.gpcms_Get_Pointer_of_Covariance_Matrix_Set(i);
		else p_tmp_cov_mat_diag = tmp_cm.gpcmds_Get_Pointer_of_Covariance_Matrix_Diagonal_Set(i);
 		// sort index sequence.
 		for (int k = 0; k<zz_gaussian_num; k++)	p_mod_ind[k] = k;
 		zz_rank.s_Sort(p_w, true, 0, zz_gaussian_num, p_mod_ind);
 		// update model sequence.
 		for (int k = 0; k<zz_gaussian_num; k++){
 			p_mean_vec[k].cp_Copy(&p_tmp_mean_vec[p_mod_ind[k]]);
			if(zz_flag_full_cov) p_cov_mat_full[k].cp_Copy(&p_tmp_cov_mat_full[p_mod_ind[k]]);
			else p_cov_mat_diag[k].cp_Copy(&p_tmp_cov_mat_diag[p_mod_ind[k]]);
 		}
 	}
	//printf("1111\n");
	// mean vectors.
 	for (int i = 0; i<zz_class_num; i++){
 		p_mean_vec = gpmvs_Get_Pointer_of_Mean_Vector_Set(i);
 		for (int k = 0; k<zz_gaussian_num; k++){
 			p_tmp_vec = p_mean_vec[k].vp();
 			for (int n = 0; n<zz_feat_dim; n++){
				f << p_tmp_vec[n] << " ";
 				//fprintf_s(fp, "%f ", p_tmp_vec[n]);
 				printf("%f ", p_tmp_vec[n]);
 			}
			f << endl;
 		}
 		f << endl;
 		printf("\n");
 	}
 //	printf("1111\n");
 	// covariance matrices.
	int vec_sz;
 	for (int i = 0; i<zz_class_num; i++){

		if(zz_flag_full_cov) p_cov_mat_full = gpcms_Get_Pointer_of_Covariance_Matrix_Set(i);
		else  p_cov_mat_diag = gpcmds_Get_Pointer_of_Covariance_Matrix_Diagonal_Set(i);

 		for (int k = 0; k<zz_gaussian_num; k++){
 			
			//////////////////////////////////////////////////////////////////////////
			/// SIZE!!!!!
			if(zz_flag_full_cov) p_tmp_vec = p_cov_mat_full[k].vps(vec_sz);
			else p_tmp_vec = p_cov_mat_diag[k].vps(vec_sz);

 			for (int n = 0; n<vec_sz; n++){
			//////////////////////////////////////////////////////////////////////////
 				f << p_tmp_vec[n] << " ";
				//fprintf_s(fp, "%f ", p_tmp_vec[n]);
 				printf("%f ", p_tmp_vec[n]);
 			}
			f << endl;
 		}
 		f << endl;
 		printf("\n");
 	}
 //	printf("1111\n");
 	// weight vectors.
 	for (int i = 0; i<zz_class_num; i++){
 		p_tmp_vec = gpmw_Get_Pointer_of_Model_Weights(i);
 		for (int k = 0; k<zz_gaussian_num; k++){
 			f << p_tmp_vec[k] << " ";
			//fprintf_s(fp, "%f ", p_tmp_vec[k]);
 			printf("%f ", p_tmp_vec[k]);
 		}
 		f << endl;
 		printf("\n");
 	}

//	fclose(fp);

	f.close();

	return;
error:
	zpme("scm_Save_Color_Model");
}


//********************************************************************************
void CKvYooji_ColorModel::scmp_Save_Color_Model_Pixelwise(CKvString in_save_file)
//********************************************************************************
{
	LCKvIO_FileVcl io_vcl;
	CKvMatrixFloat gmm_models;
	float **pp_models,*p_mean_vec,*p_cov_mat,*p_covi_mat,*p_det;
	int ww,hh;
	int num_pixels;

	if (zz_class_num <= 0) gerr("1");

	// model matrix is 4 x 3N. (row: 3 for cov. mat, 1 for mean vec)
	num_pixels = gnc_Get_Number_of_Classes();
	ww = 3*num_pixels; 
	if(zz_flag_full_cov)	hh = 4;
	else					hh = 2;
	pp_models = gmm_models.c_Create(hh, ww);
		
	for(int i=0; i<num_pixels; i++){

		p_mean_vec = gpmvs_Get_Pointer_of_Mean_Vector_Set(i)->vp();
		if(zz_flag_full_cov) p_cov_mat = gpcms_Get_Pointer_of_Covariance_Matrix_Set(i)->vp();
		else	p_cov_mat = gpcmds_Get_Pointer_of_Covariance_Matrix_Diagonal_Set(i)->vp();

		// mean vector.
		for(int k=0; k<3; k++) pp_models[hh-1][3*i + k] = p_mean_vec[k];
		// cov. matrix.
		if(zz_flag_full_cov){
			for(int m=0; m<3; m++)	// rows.
				for(int k=0; k<3; k++) // cols.
					pp_models[m][3*i + k] = p_cov_mat[k + 3*m];
		}
		else{
			for(int k=0; k<3; k++) // cols.
				pp_models[0][3*i + k] = p_cov_mat[k];
		}
	}

	io_vcl.svf_Save_as_Vcl_File(in_save_file, &gmm_models);

	return;
error:
	zpme("scmp_Save_Color_Model_Pixelwise");
}

//********************************************************************************
bool CKvYooji_ColorModel::lcm_Load_Color_Model(CKvString in_save_file)
//********************************************************************************
{
	const char *bp = (const char *)in_save_file.bp();
	std::ifstream f(bp);

	CKvRanker zz_rank;
	CKvMatrixFloat *p_cov_mat = NULL, *p_cov_mat_inv = NULL;
	CKvVectorFloat *p_mean_vec = NULL, *p_cov_mat_diag = NULL, *p_cov_mat_inv_diag = NULL;
	CKvVectorFloat tmp_vec;
	float *p_det;

	float *p_tmp_vec, *p_tmp_vec2, tmp_det;

//	errno_t err; 
//	FILE *fp;
// 	if((err = fopen_s(&fp, in_save_file.bp(), "rt")) != 0) gerr("1"); 
// 	fscanf_s(fp, "%d %d %d %d %d\n", &zz_class_num, &zz_gaussian_num, &zz_feat_dim, &zz_color_type, &zz_flag_full_cov);

	if(f.fail()) return false;
	f >> zz_class_num >> zz_gaussian_num >> zz_feat_dim >> zz_color_type >> zz_flag_full_cov;

	if (zz_class_num <= 0)	gerr("2");

	if (zz_gaussian_num <= 0)	gerr("3");

	if (zz_feat_dim <= 0)	gerr("4");

	if (zz_color_type<0 || zz_color_type>2)		zz_color_type = KV_COLOR_MODEL_RGB;

	//printf("%d %d %d %d %d\n", zz_class_num, zz_gaussian_num, zz_feat_dim, zz_color_type, zz_flag_full_cov);

	zz_set_of_mean_vec.resize(zz_class_num);
	zz_set_of_cov_mat_full.resize(zz_class_num);		zz_set_of_cov_mat_inv_full.resize(zz_class_num);
	zz_set_of_cov_mat_diag.resize(zz_class_num);		zz_set_of_cov_mat_inv_diag.resize(zz_class_num);
	zz_set_of_weights.resize(zz_class_num);				zz_set_of_det_of_cov_mat.resize(zz_class_num);

	// mean vectors.
	for (int i = 0; i<zz_class_num; i++){

		p_mean_vec = zz_set_of_mean_vec[i].c_Create(zz_gaussian_num);

		for (int k = 0; k<zz_gaussian_num; k++){

			p_tmp_vec = p_mean_vec[k].c_Create(zz_feat_dim, 0.0f);

			for (int n = 0; n<zz_feat_dim; n++){
				f >> p_tmp_vec[n];
				//fscanf_s(fp, "%f ", &p_tmp_vec[n]);
				//printf("%f ", p_tmp_vec[n]);	
			}
			//printf("\n");
		}
		//printf("\n");
	}
	// covariance matrices.
	int vec_sz = (zz_flag_full_cov) ? zz_feat_dim*zz_feat_dim : zz_feat_dim;

	for (int i = 0; i<zz_class_num; i++){
		// cov. mat.
		if(zz_flag_full_cov){			
			p_cov_mat = zz_set_of_cov_mat_full[i].c_Create(zz_gaussian_num);
			p_cov_mat_inv = zz_set_of_cov_mat_inv_full[i].c_Create(zz_gaussian_num);
		}
		else{			
			p_cov_mat_diag = zz_set_of_cov_mat_diag[i].c_Create(zz_gaussian_num);
			p_cov_mat_inv_diag = zz_set_of_cov_mat_inv_diag[i].c_Create(zz_gaussian_num);
		}
		// det.
		p_det = zz_set_of_det_of_cov_mat[i].c_Create(zz_gaussian_num, 0.0f);

		for (int k = 0; k<zz_gaussian_num; k++){

			if(zz_flag_full_cov) p_tmp_vec = p_cov_mat[k].c_Create(zz_feat_dim, zz_feat_dim, 0.0f)[0];
			else				 p_tmp_vec = p_cov_mat_diag[k].c_Create(zz_feat_dim, 0.0f);
			
			for (int n = 0; n<vec_sz; n++){
				f >> p_tmp_vec[n];
				//fscanf_s(fp, "%f ", &p_tmp_vec[n]);
				//printf("%f ", p_tmp_vec[n]);	
			}
			// calculate inverse and determinant.
			if(zz_flag_full_cov){ // full cov. mat.				
				p_tmp_vec2 = p_cov_mat_inv[k].c_Create(zz_feat_dim, zz_feat_dim, 0.0f)[0];
				p_det[k] = d_im3_Inverse_Matrix_3(p_tmp_vec, p_tmp_vec2);
				//printf("%f %f %f", p_tmp_vec2[0], p_tmp_vec2[1], p_tmp_vec2[2]);
				//printf("%f ", p_det[k]);
			}
			else{ // diag. cov. mat.				
				p_tmp_vec2 = p_cov_mat_inv_diag[k].c_Create(zz_feat_dim,0.0f);

				tmp_det = 1.0f;
				for(int n = 0; n<vec_sz; n++){
					p_tmp_vec2[n] = 1.0f / p_tmp_vec[n];
					tmp_det *= p_tmp_vec[n];
				}
				p_det[k] = tmp_det;

			}	

			//printf("\n");
		}
		//printf("\n");
	}
	// weight vectors.
	for (int i = 0; i<zz_class_num; i++){
		p_tmp_vec = zz_set_of_weights[i].c_Create(zz_gaussian_num, 0.0f);
		for (int k = 0; k<zz_gaussian_num; k++){
			f >> p_tmp_vec[k];
			//fscanf_s(fp, "%f ", &p_tmp_vec[k]);
			//printf("%f ", p_tmp_vec[k]);	
		}
		//printf("\n");
	}

//	fclose(fp);

	f.close();

	return true;
error:
	printf("lcm_Load_Color_Model");
	return false;
}


//********************************************************************************
bool CKvYooji_ColorModel::lcmp_Load_Color_Model_Pixelwise(CKvString in_load_file)
//********************************************************************************
{
	LCKvIO_FileVcl io_vcl;
	CKvMatrixFloat gmm_models;
	float **pp_models, *p_mean_vec, *p_cov_mat, *p_covi_mat, *p_det;
	int ww, hh;
	int num_pixels = 0;
	bool flag_full_cov;
	//if(!Kv_Printf("num_pixels: %d\n",num_pixels)) exit(0);


	// model matrix is 4 x 3N. (row: 3 for cov. mat, 1 for mean vec)
	if(!io_vcl.lvf_Load_from_Vcl_File(in_load_file, &gmm_models)) return false;
	
	pp_models = gmm_models.mps(ww, hh);
	if(hh == 2) flag_full_cov = false;
	else if(hh == 4) flag_full_cov = true;
	else return false;

	num_pixels = ww/3;

	//if(!Kv_Printf("num_pixels: %d, max_size: %d\n", num_pixels, zz_set_of_det_of_cov_mat.max_size())) exit(0);

	c_Create(num_pixels, 1, KV_COLOR_MODEL_RGB, flag_full_cov);	

	for(int i=0; i<num_pixels; i++){
		
		p_mean_vec = gpmvs_Get_Pointer_of_Mean_Vector_Set(i)->vp();

		if(flag_full_cov){
			p_cov_mat = gpcms_Get_Pointer_of_Covariance_Matrix_Set(i)->vp();
			p_covi_mat = gpcmis_Get_Pointer_of_Covariance_Matrix_Inverse_Set(i)->vp();
		}
		else{
			p_cov_mat = gpcmds_Get_Pointer_of_Covariance_Matrix_Diagonal_Set(i)->vp();
			p_covi_mat = gpcmids_Get_Pointer_of_Covariance_Matrix_Inverse_Diagonal_Set(i)->vp();
		}

		p_det = gpdcm_Get_Pointer_of_Determinant_of_Covariance_Matrix(i);
		
		// mean vector.
		for(int k=0; k<3; k++) p_mean_vec[k] = pp_models[hh-1][3*i + k];
		// cov. matrix.
		if(flag_full_cov){
			for(int m=0; m<3; m++)	// rows.
				for(int k=0; k<3; k++) // cols.
					p_cov_mat[k + 3*m] = pp_models[m][3*i + k];
			// deter. and inverse.
			p_det[0] = d_im3_Inverse_Matrix_3(p_cov_mat, p_covi_mat);
		}
		else{
			for(int k=0; k<3; k++) // cols.
				p_cov_mat[k] = pp_models[hh-1][3*i + k];
			// deter. and inverse.
			p_det[0] = 1.0f;
			for(int k=0; k<3; k++){
				p_covi_mat[k] = 1.0f/p_cov_mat[k];
				p_det[0] *= p_cov_mat[k];
			}

			//printf("p_covi_mat: %f %f %f (%f)\n", p_covi_mat[0], p_covi_mat[1], p_covi_mat[2], p_det[0]);
			

		}		
	}


	return true;
error:
	zpme("lcmp_Load_Color_Model_Pixelwise");
}



//********************************************************************************
LCKvYooji_Color_Segmentation::LCKvYooji_Color_Segmentation()
//********************************************************************************
{
	zz_classname = "LCKvYooji_Color_Segmentation";

	zz_class_num = zz_gaussian_num = zz_pcs_color_flag = 0;

	zz_set_of_mean_vector.clear();
	zz_set_of_covariance_matrix.clear();			zz_set_of_covariance_matrix_inverse.clear();
	zz_set_of_determinant_of_covariance_matrix.clear();
	zz_set_of_weights.clear();
}

//********************************************************************************
LCKvYooji_Color_Segmentation::~LCKvYooji_Color_Segmentation()
//********************************************************************************
{
	zz_set_of_mean_vector.clear();
	zz_set_of_covariance_matrix.clear();			zz_set_of_covariance_matrix_inverse.clear();
	zz_set_of_determinant_of_covariance_matrix.clear();
	zz_set_of_weights.clear();
}

//********************************************************************************
void LCKvYooji_Color_Segmentation::i_Initialize(bool in_flag_full_cov_mat)
//********************************************************************************
{

}

//
////********************************************************************************
//void LCKvYooji_Color_Segmentation::brgmm_Background_Removal_using_GMM(
//	CKvYooji_ColorModel *in_color_model,
//	CKvMatrixBool *in_mask_background_or_NULL,
//	CKvMatrixUcharRgb *in_image_color,
//	CKvMatrixFloat *in_image_depth_or_NULL,
//	CKvMatrixBool *out_mask_object)
////********************************************************************************
//{
//	CKvVectorFloat tmp_rgb, tmp_pcs, tmp_hue, tmp_vec;
//	CKvVectorFloat *p_mean_vec, *p_cov_mat_inv_diag;
//
//	int ww, hh, tidx;
//	int x_min, x_max, y_min, y_max, marg_x, marg_y;
//
//	bool *p_mask_obj=NULL, *p_mask_back=NULL;
//	float *p_in_image_d=NULL;
//	UCHAR *p_r, *p_g, *p_b;
//
//	unsigned char r_val, g_val, b_val;
//	double h_val, s_val, i_val;
//	float *p_tmp_rgb=NULL, *p_tmp_pcs=NULL, *p_tmp_hue=NULL;
//	float *p_det;
//
//	float prob;
//
//	int in_class_num, in_gauss_num, in_color_type;
//
//	in_class_num = in_color_model->gnc_Get_Number_of_Classes();
//	in_gauss_num = in_color_model->gnm_Get_Number_of_Models();
//	in_color_type = in_color_model->gct_Get_Color_Type();
//
//	if (in_image_depth_or_NULL)	p_in_image_d = in_image_depth_or_NULL->mps(ww, hh)[0];
//	else in_image_color->ms(ww, hh);
//	if (out_mask_object->mw() != ww || out_mask_object->mh() != hh)	out_mask_object->c_Create(hh, ww);
//	p_mask_obj = out_mask_object->vp();
//	if (in_mask_background_or_NULL)	p_mask_back = in_mask_background_or_NULL->vp();
//	p_r = in_image_color->vp(p_g, p_b);
//
//	if (in_color_type == KV_COLOR_MODEL_HUE){
//		p_tmp_hue = tmp_hue.c_Create(1, 0.0f);
//	}
//	else{
//		p_tmp_rgb = tmp_rgb.c_Create(3, 0.0f);
//		p_tmp_pcs = tmp_pcs.c_Create(3, 0.0f);
//	}
//
//	if (in_class_num <= 0)	gerr("1");
//
//	//printf("%d %d %d %d\n", ww, hh, in_image_color->mw(), in_image_color->mh());
//
//	if (in_image_color->mw() != ww || in_image_color->mh() != hh)	gerr("2");
//
//	/// Currently, select single model of which the weight is the largest value for each class. 
//	// get pointers of color model.
//	p_mean_vec = in_color_model->gpmvs_Get_Pointer_of_Mean_Vector_Set(0);
//	p_cov_mat_inv_diag = in_color_model->gpcmids_Get_Pointer_of_Covariance_Matrix_Inverse_Diagonal_Set(0);
//	p_det = in_color_model->gpdcm_Get_Pointer_of_Determinant_of_Covariance_Matrix(0);
//	/// ///////////////////////////////////////////////////
//	// find object ROI region. --> 추후에는 object cube를 projection 시켜서 얻자...
//	marg_x = 10;	marg_y = 10;
//	x_min = y_min = 0;		x_max = ww - 1; y_max = hh - 1;
//	for (int j = 0; j<hh; j++){
//		for (int i = 0; i<ww; i++){
//
//			tidx = j*ww + i;
//			p_mask_obj[tidx] = false;	// initialize object mask.
//			if (in_image_depth_or_NULL){
//				if (p_in_image_d[tidx]>0){
//					if (i<x_min)	x_min = i;
//					if (i>x_max)	x_max = i;
//					if (j<y_min)	y_min = j;
//					if (j>y_max)	y_max = j;
//				}
//			}
//
//		}
//	}
//
//	// remove background region.
//	x_min = max(0, x_min - marg_x);		y_min = max(0, y_min - marg_y);
//	x_max = min(ww - 1, x_max + marg_x);	y_max = min(hh - 1, y_max + marg_y);
//	for (int j = y_min; j <= y_max; j++){
//		for (int i = x_min; i <= x_max; i++){
//
//			tidx = j*ww + i;
//
//			if (in_mask_background_or_NULL){
//				if(!p_mask_back[tidx]){
//
//					// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//					p_r[tidx] = (UCHAR)(0.5f*p_r[tidx]);
//					p_g[tidx] = (UCHAR)(0.5f*p_g[tidx]);
//					p_b[tidx] = (UCHAR)(0.5f*p_b[tidx]);
//					// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//
//					continue;
//				}
//			}
//
//			r_val = p_r[tidx];		g_val = p_g[tidx];			b_val = p_b[tidx];
//
//			// convert color coordinates.
//			if (in_color_type == KV_COLOR_MODEL_RGB){
//				p_tmp_rgb[0] = (float)r_val;	p_tmp_rgb[1] = (float)g_val;	p_tmp_rgb[2] = (float)b_val;
//				prob = z_gmd_Get_Mahalanobis_Distance(&tmp_rgb, &p_mean_vec[0], &p_cov_mat_inv_diag[0], p_det[0]);
//			}
//			else if (in_color_type == KV_COLOR_MODEL_PCS){
//				p_tmp_rgb[0] = (float)r_val;	p_tmp_rgb[1] = (float)g_val;	p_tmp_rgb[2] = (float)b_val;
//				cc_Convert_Color_Rgb_to_PCS(tmp_rgb, tmp_pcs);
//				prob = z_gmd_Get_Mahalanobis_Distance(&tmp_pcs, &p_mean_vec[0], &p_cov_mat_inv_diag[0], p_det[0]);
//			}
//			else{
//				ccvrh_Convert_Color_Value_RGB_to_HSI(r_val, g_val, b_val, h_val, s_val, i_val);
//				if (s_val>0.1 && i_val <200.0f){
//					p_tmp_hue[0] = (float)h_val;
//					prob = z_gmdc_Get_Mahalanobis_Distance_Circular(&tmp_hue, &p_mean_vec[0], &p_cov_mat_inv_diag[0], p_det[0]);
//				}
//				else{
//					p_tmp_hue[0] = -1.0f;
//					prob = 100000.0f;
//				}
//
//			}
//
//			// 				printf("prob: %f\n", prob);
//			// 				system("pause");
//			// set hand mask.
//			if (prob>10.0f)	p_mask_obj[tidx] = true;
//			else{
//				p_r[tidx] = (UCHAR)(0.5f*p_r[tidx]);
//				p_g[tidx] = (UCHAR)(0.5f*p_g[tidx]);
//				p_b[tidx] = (UCHAR)(0.5f*p_b[tidx]);
//			}
//
//			//if(prob<3.0f)	p_io_image_d[tidx]=(unsigned short)0;
//		}
//
//	}
//
//	for (int j = 0; j<hh; j++){
//		for (int i = 0; i<ww; i++){
//
//			if ((j<y_min || j>y_max) || (i<x_min || i>x_max)){
//
//				tidx = j*ww + i;
//
//				// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//				p_r[tidx] = (UCHAR)(0.3f*p_r[tidx]);
//				p_g[tidx] = (UCHAR)(0.3f*p_g[tidx]);
//				p_b[tidx] = (UCHAR)(0.3f*p_b[tidx]);
//				// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//
//			}
//
//		}
//	}
//
//	return;
//error:
//	zpme("brgmm_Background_Removal_using_GMM");
//
//}

//********************************************************************************
void LCKvYooji_Color_Segmentation::scd_Skin_Color_Detection(
	CKvYooji_ColorModel *in_color_model,
	CKvMatrixUcharRgb *in_image_color,
	CKvMatrixBool *out_mask_object)
	//********************************************************************************
{
	CKvVectorFloat tmp_rgb, tmp_pcs, tmp_hue, tmp_vec;
	CKvVectorFloat *p_mean_vec, *p_cov_mat_inv_diag;

	int ww, hh, tidx;

	bool *p_mask_obj;
	UCHAR *p_r, *p_g, *p_b;

	unsigned char r_val, g_val, b_val;
	double h_val, s_val, i_val;
	float *p_tmp_rgb, *p_tmp_pcs, *p_tmp_hue;
	float *p_det;

	float prob;

	int in_class_num, in_gauss_num, in_color_type;

	in_class_num = in_color_model->gnc_Get_Number_of_Classes();
	in_gauss_num = in_color_model->gnm_Get_Number_of_Models();
	in_color_type = in_color_model->gct_Get_Color_Type();

	in_image_color->ms(ww, hh);
	if (out_mask_object->mw() != ww || out_mask_object->mh() != hh)	out_mask_object->c_Create(hh, ww);
	p_mask_obj = out_mask_object->vp();
	p_r = in_image_color->vp(p_g, p_b);

	if (in_color_type == KV_COLOR_MODEL_HUE){
		p_tmp_hue = tmp_hue.c_Create(1, 0.0f);
	}
	else{
		p_tmp_rgb = tmp_rgb.c_Create(3, 0.0f);
		p_tmp_pcs = tmp_pcs.c_Create(3, 0.0f);
	}

	if (in_class_num <= 0)	gerr("1");
	if (in_image_color->mw() != ww || in_image_color->mh() != hh)	gerr("2");

	/// Currently, select single model of which the weight is the largest value for each class. 
	/// ///////////////////////////////////////////////////

	// detect skin color region.
	for (int j = 0; j<hh; j++){
		for (int i = 0; i<ww; i++){

			tidx = j*ww + i;

			r_val = p_r[tidx];		g_val = p_g[tidx];			b_val = p_b[tidx];

			// convert color coordinates.
			if (in_color_type == KV_COLOR_MODEL_RGB){
				p_tmp_rgb[0] = (float)r_val;	p_tmp_rgb[1] = (float)g_val;	p_tmp_rgb[2] = (float)b_val;
				prob = in_color_model->gmd_Get_Mahalanobis_Distance(&tmp_rgb, 0, 0);
				//prob = z_gmd_Get_Mahalanobis_Distance(&tmp_rgb, &p_mean_vec[0], &p_cov_mat_inv_diag[0], p_det[0]);
			}
			else if (in_color_type == KV_COLOR_MODEL_PCS){
				p_tmp_rgb[0] = (float)r_val;	p_tmp_rgb[1] = (float)g_val;	p_tmp_rgb[2] = (float)b_val;
				cc_Convert_Color_Rgb_to_PCS(tmp_rgb, tmp_pcs);
				prob = in_color_model->gmd_Get_Mahalanobis_Distance(&tmp_pcs, 0, 0);
				//prob = z_gmd_Get_Mahalanobis_Distance(&tmp_pcs, &p_mean_vec[0], &p_cov_mat_inv_diag[0], p_det[0]);
			}
			else{
				ccvrh_Convert_Color_Value_RGB_to_HSI(r_val, g_val, b_val, h_val, s_val, i_val);
				if (s_val>0.1 && i_val <200.0f){
					p_tmp_hue[0] = (float)h_val;
					prob = in_color_model->gmdc_Get_Mahalanobis_Distance_Circular(&tmp_hue, 0, 0);
					//prob = z_gmdc_Get_Mahalanobis_Distance_Circular(&tmp_hue, &p_mean_vec[0], &p_cov_mat_inv_diag[0], p_det[0]);
				}
				else{
					p_tmp_hue[0] = -1.0f;
					prob = 100000.0f;
				}

			}

			// for skin color detection.
			if (prob>3.0f){
				p_mask_obj[tidx] = false;
				//p_r[tidx]=(UCHAR)(0.3f*p_r[tidx]);
				//p_g[tidx]=(UCHAR)(0.3f*p_g[tidx]);
				//p_b[tidx]=(UCHAR)(0.3f*p_b[tidx]);
			}
			else{
				p_mask_obj[tidx] = true;
			}
		}

	}

	return;
error:
	zpme("brgmm_Background_Removal_using_GMM");

}

//********************************************************************************
void LCKvYooji_Color_Segmentation::rhr_Remove_Hand_Region(CKvYooji_ColorModel &in_color_model,
	CKvMatrixUcharRgb &in_image_color,
	CKvMatrixUshort &io_image_depth)
	//********************************************************************************
{
	int in_class_num, in_gauss_num, in_color_type;

	in_class_num = in_color_model.gnc_Get_Number_of_Classes();
	in_gauss_num = in_color_model.gnm_Get_Number_of_Models();
	in_color_type = in_color_model.gct_Get_Color_Type();

	if (in_class_num <= 0){
		printf("[LCKvColorSegmentation::rhr_Remove_Hand_Region] Invalid GMM model was loaded.\n");
		system("pause");
		exit(0);
	}

	CKvMatrixBool mask_hand;
	CKvVectorFloat tmp_rgb, tmp_pcs, tmp_hue, tmp_vec;
	CKvVectorFloat *p_mean_vec, *p_cov_mat_inv_diag;

	int ww, hh, tidx;

	bool *p_mask_hand;
	unsigned short *p_io_image_d;
	unsigned char *p_r, *p_g, *p_b;

	unsigned char r_val, g_val, b_val;
	double h_val, s_val, i_val;
	float *p_tmp_rgb, *p_tmp_pcs, *p_tmp_hue;
	float *p_det;

	float *p_tmp_vec, prob;

	p_io_image_d = io_image_depth.mps(ww, hh)[0];
	p_mask_hand = mask_hand.c_Create(hh, ww, false)[0];
	p_r = in_image_color.vp(p_g, p_b);
	p_tmp_vec = tmp_vec.c_Create(3, 0.0f);

	if (in_image_color.mw() != ww || in_image_color.mh() != hh){
		printf("[LCKvColorSegmentation::rhr_Remove_Hand_Region] Dimensions of input images are different.\n");
		system("pause");
		exit(0);
	}

	if (in_color_type == KV_COLOR_MODEL_HUE){
		p_tmp_hue = tmp_hue.c_Create(1, 0.0f);
	}
	else{
		p_tmp_rgb = tmp_rgb.c_Create(3, 0.0f);
		p_tmp_pcs = tmp_pcs.c_Create(3, 0.0f);
	}

	/// Currently, select single model of which the weight is the largest value for each class. 
	/// ///////////////////////////////////////////////////
	// find hand region.
	for (int j = 0; j<hh; j++){
		for (int i = 0; i<ww; i++){
			tidx = j*ww + i;
			if (p_io_image_d[tidx]>0){

				/// find pixel on rgb camera coordinates.

				r_val = p_r[tidx];		g_val = p_g[tidx];			b_val = p_b[tidx];

				// convert color coordinates.
				if (in_color_type == KV_COLOR_MODEL_RGB){
					p_tmp_rgb[0] = (float)r_val;	p_tmp_rgb[1] = (float)g_val;	p_tmp_rgb[2] = (float)b_val;
					prob = in_color_model.gmd_Get_Mahalanobis_Distance(&tmp_rgb, 0, 0);
					//prob = z_gmd_Get_Mahalanobis_Distance(&tmp_rgb, &p_mean_vec[0], &p_cov_mat_inv_diag[0], p_det[0]);
				}
				else if (in_color_type == KV_COLOR_MODEL_PCS){
					p_tmp_rgb[0] = (float)r_val;	p_tmp_rgb[1] = (float)g_val;	p_tmp_rgb[2] = (float)b_val;
					cc_Convert_Color_Rgb_to_PCS(tmp_rgb, tmp_pcs);
					prob = in_color_model.gmd_Get_Mahalanobis_Distance(&tmp_pcs, 0, 0);
					//prob = z_gmd_Get_Mahalanobis_Distance(&tmp_pcs, &p_mean_vec[0], &p_cov_mat_inv_diag[0], p_det[0]);
				}
				else{
					ccvrh_Convert_Color_Value_RGB_to_HSI(r_val, g_val, b_val, h_val, s_val, i_val);
					if (s_val>0.1 && i_val <200.0f)	p_tmp_hue[0] = (float)h_val;
					else{ p_tmp_hue[0] = -1.0f;		continue; }
					prob = in_color_model.gmdc_Get_Mahalanobis_Distance_Circular(&tmp_pcs, 0, 0);
					//prob = z_gmdc_Get_Mahalanobis_Distance_Circular(&tmp_hue, &p_mean_vec[0], &p_cov_mat_inv_diag[0], p_det[0]);
				}

				// 				printf("prob: %f\n", prob);
				// 				system("pause");
				// set hand mask.
				if (prob<3.0f)	p_mask_hand[tidx] = true;
				//if(prob<3.0f)	p_io_image_d[tidx]=(unsigned short)0;
			}
		}
	}
	// dilate hand region.


}

//********************************************************************************
void LCKvYooji_Color_Segmentation::sibf_Smooth_Image_using_Bilateral_Filter(CKvMatrixUchar *in_img,
	CKvMatrixUchar *out_img)
	//********************************************************************************
{
	//Filter specification.
	//- Spatial(domain) filter
	// + 5x5 approximated Gaussian mask.
	// + sum of 25 filter weights is 1.
	//- Range filter
	// + sigma can be adjusted by user. (initial value is 10 in 8-bit gray image)

	CKvMatrixFloat filt_spa;
	CKvMatrixUchar exp_img;
	UCHAR *p_in, *p_out, *p_exp_img;
	float *p_filt;

	int i, j, k, m;
	int ww, hh, ww_exp, hh_exp;
	int ww_f, hh_f, dx1, dy1, dx2, dy2;
	float tmp_val, w, w_total, f, f_refer, sig_f, sig_f_sq;

	p_in = in_img->mps(ww, hh)[0];
	if (in_img == out_img)	p_out = p_in;
	else{
		if (out_img->mw() != ww || out_img->mh() != hh)	out_img->c_Create(hh, ww, (UCHAR)0);
		p_out = out_img->vp();
	}

	// get spatial Gaussian filter.
	gsfg_Get_Spatial_Filter_Gaussian(filt_spa);
	p_filt = filt_spa.mps(ww_f, hh_f)[0];

	dx1 = ww_f / 2;			dx2 = ww_f - 1 - dx1;
	dy1 = hh_f / 2;			dy2 = hh_f - 1 - dy1;
	ww_exp = ww + ww_f - 1;		hh_exp = hh + hh_f - 1;
	p_exp_img = exp_img.c_Create(hh_exp, ww_exp)[0];
	exp_img.sb_Set_Block(dx1, dy1, in_img);

	// mirroring boundary pixels.
	// vertical boundary pixels.
	for (i = 0; i<dx1; i++){
		for (j = dy1; j<hh_exp - dy2; j++){
			p_exp_img[j*ww_exp + i]
				= p_in[(j - dy1)*ww + (dx1 - 1 - i)];
		}
	}
	for (i = 0; i<dx2; i++){
		for (j = dy1; j<hh_exp - dy2; j++){
			p_exp_img[j*ww_exp + (ww_exp - 1 - i)]
				= p_in[(j - dy1)*ww + (ww - 1 - (dx2 - 1 - i))];
		}
	}
	// horizontal boundary pixels.
	for (j = 0; j<dy1; j++){
		for (i = dx1; i<ww_exp - dx2; i++){
			p_exp_img[j*ww_exp + i]
				= p_in[(dy1 - 1 - j)*ww + (i - dx1)];
		}
	}
	for (j = 0; j<dy2; j++){
		for (i = dx1; i<ww_exp - dx2; i++){
			p_exp_img[(hh_exp - 1 - j)*ww_exp + i]
				= p_in[(hh - 1 - (dy2 - 1 - j))*ww + (i - dx1)];
		}
	}

	// convolution.
	sig_f = 10.0f;	sig_f_sq = SQUARE(sig_f);
	for (j = 0; j<hh; j++){
		for (i = 0; i<ww; i++){
			tmp_val = w_total = 0.0f;
			f_refer = (float)p_exp_img[(j + dy1)*ww_exp + (i + dx1)];
			for (m = -dy1; m<dy2 + 1; m++){
				for (k = -dx1; k<dx2 + 1; k++){
					f = (float)p_exp_img[(j + m + dy1)*ww_exp + (i + k + dx1)];
					w = /*domain*/p_filt[(m + dy1)*ww_f + (k + dx1)] * /*range*/exp(-0.5f*SQUARE(f - f_refer) / sig_f_sq);

					tmp_val += w*f;
					w_total += w;
				}
			}
			p_out[j*ww + i] = (UCHAR)(tmp_val / w_total);
		}
	}


}

//********************************************************************************
void LCKvYooji_Color_Segmentation::sibf_Smooth_Image_using_Bilateral_Filter(CKvMatrixFloat *in_img,
	float in_sigma_range,
	CKvMatrixFloat *out_img)
	//********************************************************************************
{
	//Filter specification.
	//- Spatial(domain) filter
	// + 5x5 approximated Gaussian mask.
	// + sum of 25 filter weights is 1.
	//- Range filter
	// + sigma can be adjusted by user.

	CKvMatrixFloat filt_spa;
	CKvMatrixFloat exp_img;
	float *p_in, *p_out, *p_exp_img;
	float *p_filt;

	int i, j, k, m;
	int ww, hh, ww_exp, hh_exp;
	int ww_f, hh_f, dx1, dy1, dx2, dy2;
	float tmp_val, w, w_total, f, f_refer, sig_f_sq;

	p_in = in_img->mps(ww, hh)[0];
	if (in_img == out_img)	p_out = p_in;
	else{
		if (out_img->mw() != ww || out_img->mh() != hh)	out_img->c_Create(hh, ww, 0.0f);
		p_out = out_img->vp();
	}

	// get spatial Gaussian filter.
	gsfg_Get_Spatial_Filter_Gaussian(filt_spa);
	p_filt = filt_spa.mps(ww_f, hh_f)[0];

	dx1 = ww_f / 2;			dx2 = ww_f - 1 - dx1;
	dy1 = hh_f / 2;			dy2 = hh_f - 1 - dy1;
	ww_exp = ww + ww_f - 1;		hh_exp = hh + hh_f - 1;
	p_exp_img = exp_img.c_Create(hh_exp, ww_exp)[0];
	exp_img.sb_Set_Block(dx1, dy1, in_img);

	// mirroring boundary pixels.
	// vertical boundary pixels.
	for (i = 0; i<dx1; i++){
		for (j = dy1; j<hh_exp - dy2; j++){
			p_exp_img[j*ww_exp + i]
				= p_in[(j - dy1)*ww + (dx1 - 1 - i)];
		}
	}
	for (i = 0; i<dx2; i++){
		for (j = dy1; j<hh_exp - dy2; j++){
			p_exp_img[j*ww_exp + (ww_exp - 1 - i)]
				= p_in[(j - dy1)*ww + (ww - 1 - (dx2 - 1 - i))];
		}
	}
	// horizontal boundary pixels.
	for (j = 0; j<dy1; j++){
		for (i = dx1; i<ww_exp - dx2; i++){
			p_exp_img[j*ww_exp + i]
				= p_in[(dy1 - 1 - j)*ww + (i - dx1)];
		}
	}
	for (j = 0; j<dy2; j++){
		for (i = dx1; i<ww_exp - dx2; i++){
			p_exp_img[(hh_exp - 1 - j)*ww_exp + i]
				= p_in[(hh - 1 - (dy2 - 1 - j))*ww + (i - dx1)];
		}
	}

	// convolution.
	sig_f_sq = SQUARE(in_sigma_range);
	for (j = 0; j<hh; j++){
		for (i = 0; i<ww; i++){
			tmp_val = w_total = 0.0f;
			f_refer = p_exp_img[(j + dy1)*ww_exp + (i + dx1)];
			for (m = -dy1; m<dy2 + 1; m++){
				for (k = -dx1; k<dx2 + 1; k++){
					f = p_exp_img[(j + m + dy1)*ww_exp + (i + k + dx1)];
					w = /*domain*/p_filt[(m + dy1)*ww_f + (k + dx1)] * /*range*/exp(-0.5f*SQUARE(f - f_refer) / sig_f_sq);

					tmp_val += w*f;
					w_total += w;
				}
			}
			p_out[j*ww + i] = tmp_val / w_total;
		}
	}


}

//********************************************************************************
void LCKvYooji_Color_Segmentation::sisf_Smooth_Image_using_Spatial_Filter(CKvMatrixUchar *in_img,
	CKvMatrixUchar *out_img)
	//********************************************************************************
{
	CKvMatrixFloat filt_spa;
	CKvMatrixUchar exp_img;
	UCHAR *p_in, *p_out, *p_exp_img;
	float *p_filt;

	int i, j, k, m;
	int ww, hh, ww_exp, hh_exp;
	int ww_f, hh_f, dx1, dy1, dx2, dy2;
	float tmp_val;

	p_in = in_img->mps(ww, hh)[0];
	if (out_img->mw() != ww || out_img->mh() != hh)	out_img->c_Create(hh, ww, (UCHAR)0);
	p_out = out_img->vp();

	// get spatial Gaussian filter.
	gsfg_Get_Spatial_Filter_Gaussian(filt_spa);
	p_filt = filt_spa.mps(ww_f, hh_f)[0];

	dx1 = ww_f / 2;			dx2 = ww_f - 1 - dx1;
	dy1 = hh_f / 2;			dy2 = hh_f - 1 - dy1;
	ww_exp = ww + ww_f - 1;		hh_exp = hh + hh_f - 1;
	p_exp_img = exp_img.c_Create(hh_exp, ww_exp)[0];
	exp_img.sb_Set_Block(dx1, dy1, in_img);

	// mirroring boundary pixels.
	// vertical boundary pixels.
	for (i = 0; i<dx1; i++){
		for (j = dy1; j<hh_exp - dy2; j++){
			p_exp_img[j*ww_exp + i]
				= p_in[(j - dy1)*ww + (dx1 - 1 - i)];
		}
	}
	for (i = 0; i<dx2; i++){
		for (j = dy1; j<hh_exp - dy2; j++){
			p_exp_img[j*ww_exp + (ww_exp - 1 - i)]
				= p_in[(j - dy1)*ww + (ww - 1 - (dx2 - 1 - i))];
		}
	}
	// horizontal boundary pixels.
	for (j = 0; j<dy1; j++){
		for (i = dx1; i<ww_exp - dx2; i++){
			p_exp_img[j*ww_exp + i]
				= p_in[(dy1 - 1 - j)*ww + (i - dx1)];
		}
	}
	for (j = 0; j<dy2; j++){
		for (i = dx1; i<ww_exp - dx2; i++){
			p_exp_img[(hh_exp - 1 - j)*ww_exp + i]
				= p_in[(hh - 1 - (dy2 - 1 - j))*ww + (i - dx1)];
		}
	}

	// convolution.
	for (j = 0; j<hh; j++){
		for (i = 0; i<ww; i++){
			tmp_val = 0.0f;
			for (m = -dy1; m<dy2 + 1; m++){
				for (k = -dx1; k<dx2 + 1; k++){
					tmp_val += p_filt[(m + dy1)*ww_f + (k + dx1)] * (float)p_exp_img[(j + m + dy1)*ww_exp + (i + k + dx1)];
				}
			}
			p_out[j*ww + i] = (unsigned char)tmp_val;
		}
	}


}




//********************************************************************************
void LCKvYooji_Color_Segmentation::tcmgmm_Training_Color_Model_using_Gaussian_Mixture_Model(CKvString &in_load_folder,
	CKvString &in_save_file_name,
	int in_class_num,
	int in_gaussian_num,
	int in_color_type)
	//********************************************************************************
{

	CKvYooji_ColorModel zz_cm;

	vector<CKvVectorFloat> *set_of_feature_clusters = NULL;
	vector<CKvVectorInt> tmp_cluster_indices;

	// initialization.
	set_of_feature_clusters = new vector<CKvVectorFloat>[in_class_num];
	for (int i = 0; i<in_class_num; i++)		set_of_feature_clusters[i].clear();
	zz_cm.c_Create(in_class_num, in_gaussian_num, in_color_type);

	// get sample points.
	gsfis_Get_Sample_Features_from_Image_Sequence(in_load_folder, in_class_num, in_color_type, set_of_feature_clusters);
	// clustering sample points.
	for (int i = 0; i<in_class_num; i++)		cgmm_Clustering_using_Gaussian_Mixture_Model(set_of_feature_clusters[i], i, &zz_cm);
	// save gmm parameters.
	zz_cm.scm_Save_Color_Model(in_save_file_name);

	// release allocated memory.
	for (int i = 0; i<in_class_num; i++){ set_of_feature_clusters[i].clear(); }
	tmp_cluster_indices.clear();
	delete[] set_of_feature_clusters;

}

//********************************************************************************
int LCKvYooji_Color_Segmentation::cfgmm_Classify_A_Feature_using_Gaussian_Mixture_Model(
	CKvVectorFloat &in_feature_vector,
	CKvYooji_ColorModel &in_gmm_model,
	int in_class_num)
//********************************************************************************
{
	int class_result;

	float max_prob, prob;

	max_prob = -100000.0f;
	for (int m = 0; m<in_class_num; m++){
		prob = in_gmm_model.gpdw_Get_Probability_Density_Weighted(&in_feature_vector, m);
		//printf("prob: %f\n", prob);
		if(prob > 1.0f)	continue;
		if (prob>max_prob){ max_prob = prob;		class_result = m; }
	}

	return class_result;
}

//********************************************************************************************
void LCKvYooji_Color_Segmentation::do_Dilate_Object(CKvMatrixBool &io_mask_obj,
	int in_dx, int in_dy,
	bool in_largest_blob_select_mode)
	//********************************************************************************************
{
	CKvRunSet zz_rs;		/// CkvRunSet 대신 CKvSdkRunset의 dil_Dilation 함수 사용 시 죽을 때가 많음.

	zz_rs.i_Import(&io_mask_obj);
	zz_rs.d_Dilation(&zz_rs, in_dx, in_dy);
	zz_rs.e_Export(true, false, &io_mask_obj);

	if (in_largest_blob_select_mode){
		CKvSdkCodeRun zz_code_run;
		CKvSdkRunset zz_srs;
		zz_code_run.im_Import(&io_mask_obj);
		zz_code_run.eo_Extract_Objects(0, 1, &zz_srs);		//zz_code_run.eo_Extract_an_Object(0, &zz_srs);
		zz_srs.ex_Export(true, false, &io_mask_obj);
	}
}


//********************************************************************************************
void LCKvYooji_Color_Segmentation::eo_Erode_Object(CKvMatrixBool &io_mask_obj,
	int in_dx, int in_dy,
	bool in_largest_blob_select_mode)
	//********************************************************************************************
{
	CKvRunSet zz_rs;

	zz_rs.i_Import(&io_mask_obj);
	zz_rs.e_Erosion(&zz_rs, in_dx, in_dy);
	zz_rs.e_Export(true, false, &io_mask_obj);

	if (in_largest_blob_select_mode){
		CKvSdkCodeRun zz_code_run;
		CKvSdkRunset zz_srs;
		zz_code_run.im_Import(&io_mask_obj);
		zz_code_run.eo_Extract_Objects(0, 1, &zz_srs);		//zz_code_run.eo_Extract_an_Object(0, &zz_srs);
		zz_srs.ex_Export(true, false, &io_mask_obj);
	}
}

//********************************************************************************
void LCKvYooji_Color_Segmentation::cgmm_Clustering_using_Gaussian_Mixture_Model(vector<CKvVectorFloat> &in_sample_features,
	int in_class_idx,
	CKvYooji_ColorModel *io_color_model)
	//********************************************************************************
{
	CKvVectorFloat *tmp_mean_vector;
	int feat_dim, circular_mode;
	float w_init, dist, dist2, max_mean_shift, *tmp_vec1, *tmp_vec2;
	float *p_w;


	CKvVectorFloat *p_mean_vec;
	CKvVectorInt *cluster_indices;
	int num_gauss, color_type;

	feat_dim = in_sample_features[0].vs();
	if (feat_dim != io_color_model->gdf_Get_Dimension_of_Feature()){
		printf("[LCKvColorSegmentation::cgmm_Clustering_using_Gaussian_Mixture_Model] invalid feature dimension.\n");
		system("pause");
		exit(0);
	}

	num_gauss = io_color_model->gnm_Get_Number_of_Models();
	color_type = io_color_model->gct_Get_Color_Type();
	p_mean_vec = io_color_model->gpmvs_Get_Pointer_of_Mean_Vector_Set(in_class_idx);
	p_w = io_color_model->gpmw_Get_Pointer_of_Model_Weights(in_class_idx);

	cluster_indices = new CKvVectorInt[num_gauss];
	tmp_mean_vector = new CKvVectorFloat[num_gauss];

	if (color_type == KV_COLOR_MODEL_RGB){
		max_mean_shift = 3*KV_MAX_MEAN_SHIFT_RGB*KV_MAX_MEAN_SHIFT_RGB; // squared dist.
	}
	else if (color_type == KV_COLOR_MODEL_PCS){
		max_mean_shift = 3*KV_MAX_MEAN_SHIFT_PCS*KV_MAX_MEAN_SHIFT_PCS; // squared dist.
	}
	else{
		max_mean_shift = KV_MAX_MEAN_SHIFT_HUE;							// circular dist.
		circular_mode = true;
	}

	// initialization.	
	//printf("ickm_Initial_Clustering_using_K_Means_Algorithm... ");
	z_ickm_Initial_Clustering_using_K_Means_Algorithm(in_sample_features,
		in_class_idx,
		cluster_indices,
		io_color_model);
	//printf("Complete!                                                                     \r");

 	num_gauss = io_color_model->gnm_Get_Number_of_Models();
 	w_init = 1.0f / (float)num_gauss;		for (int i = 0; i<num_gauss; i++)		p_w[i] = w_init;
 
  	//printf("z_em_Expectation_and_Maximization... ");
  	int iter_cnt = 0;
  	//while (iter_cnt < 200){
  	while (iter_cnt < 5){
  
  		for (int i = 0; i<num_gauss; i++)		tmp_mean_vector[i].cp_Copy(&p_mean_vec[i]);
  				
  		z_em_Expectation_and_Maximization(in_sample_features,
  			in_class_idx,
  			io_color_model);
  		//printf("Complete!                                                                     \r");
  
  		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  		// log-likelihood 계산해서 조건 추가......
  		// log-likelihood 계산해서 조건 추가......
  		// log-likelihood 계산해서 조건 추가......
  		//printf("Check terminal condition... ");
  		dist = 0.0f;
  		for (int i = 0; i<num_gauss; i++){
  			dist2 = 0.0f;
  			tmp_vec1 = tmp_mean_vector[i].vp();		tmp_vec2 = p_mean_vec[i].vp();
  			if (circular_mode){ dist2 = abs(tmp_vec1[0] - tmp_vec2[0]);		dist2 = DIST_CIRCULAR(dist2); }
  			else{ for (int k = 0; k<feat_dim; k++)	dist2 += SQUARE(tmp_vec1[k] - tmp_vec2[k]); }
  			dist += sqrt(dist2);
  
  			//printf("p_mean_vec: %f %f %f\n", tmp_vec2[0], tmp_vec2[1], tmp_vec2[2]);
  		}
  		//printf("Complete!                                                                     \r");
  		/// ////////////////////
  		//printf("Param. change: %f (th: %f)\n", dist, max_mean_shift*num_gauss);
  		if (dist<max_mean_shift*num_gauss)	break;		// for RGB
  		/// ////////////////////
  
  		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  
  		iter_cnt++;
  	}
	//printf("Complete!                                                                     \r");

	//printf("\n");

// 	float *pd = io_color_model->gpdcm_Get_Pointer_of_Determinant_of_Covariance_Matrix(in_class_idx);
// 	int ng = io_color_model->gnm_Get_Number_of_Models();
	//printf("Determinant of covariance matrices [#%d class]\n", in_class_idx);
	//for(int i=0; i<ng; i++)	printf("%f\n", pd[i]);
	//if(!Kv_Printf("Det"))	exit(0);

// 	printf("Covariance matrices [#%d class]\n", in_class_idx);
// 	d_pm_Printf_Matrix(io_color_model->gpcmds_Get_Pointer_of_Covariance_Matrix_Diagonal_Set(in_class_idx)[0].vp(), 1, 3, "C1");
// 	d_pm_Printf_Matrix(io_color_model->gpcms_Get_Pointer_of_Covariance_Matrix_Set(in_class_idx)[1].vp(), 3, 3, "C2");
// 	d_pm_Printf_Matrix(io_color_model->gpcms_Get_Pointer_of_Covariance_Matrix_Set(in_class_idx)[2].vp(), 3, 3, "C3");
// 	d_pm_Printf_Matrix(io_color_model->gpcms_Get_Pointer_of_Covariance_Matrix_Set(in_class_idx)[3].vp(), 3, 3, "C4");
// 	d_pm_Printf_Matrix(io_color_model->gpcms_Get_Pointer_of_Covariance_Matrix_Set(in_class_idx)[4].vp(), 3, 3, "C5");
// 	d_pm_Printf_Matrix(io_color_model->gpcmis_Get_Pointer_of_Covariance_Matrix_Inverse_Set(in_class_idx)[0].vp(), 3, 3, "C1");
// 	d_pm_Printf_Matrix(io_color_model->gpcmis_Get_Pointer_of_Covariance_Matrix_Inverse_Set(in_class_idx)[1].vp(), 3, 3, "C2");
// 	d_pm_Printf_Matrix(io_color_model->gpcmis_Get_Pointer_of_Covariance_Matrix_Inverse_Set(in_class_idx)[2].vp(), 3, 3, "C3");
// 	d_pm_Printf_Matrix(io_color_model->gpcmis_Get_Pointer_of_Covariance_Matrix_Inverse_Set(in_class_idx)[3].vp(), 3, 3, "C4");
// 	d_pm_Printf_Matrix(io_color_model->gpcmis_Get_Pointer_of_Covariance_Matrix_Inverse_Set(in_class_idx)[4].vp(), 3, 3, "C5");

//	if(!Kv_Printf("Cov!"))	exit(0);

	delete[] tmp_mean_vector;
	delete[] cluster_indices;
}

//********************************************************************************
void LCKvYooji_Color_Segmentation::z_ickm_Initial_Clustering_using_K_Means_Algorithm(vector<CKvVectorFloat> &in_sample_features,
	int in_class_idx,
	CKvVectorInt *out_cluster_indices_or_NULL,
	CKvYooji_ColorModel *io_color_model)
	//********************************************************************************
{
	// p_: pointer of, s_: set of, pv: probability vector, pp: post probability.
	// for local function.
	vector<CKvVectorInt> s_cl_idx;
	vector<CKvVectorFloat> s_mv_prev;
	CKvVectorInt v_tidcs, v_cl_sz;
	bool circular_mode;
	int tidx, cl_idx, c_type, cnt;
	float min_dist, min_mean_diff, max_mean_shift, tdist, s_term, c_term;
	int *p_cl_sz, *p_tidcs, *p_out_idcs;
	// for input variables.
	int n_gauss, n_samp, d_feat;
	CKvVectorFloat *p_smv, *p_scmd, *p_scmid;
	CKvMatrixFloat *p_scm, *p_scmi;
	float *p_fv, *p_mv, *p_cm, *p_cmi, *p_cmd, *p_cmid, *p_sd;
	// for computing full covariance matrix.
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// 추후 feature dimension에 맞게 수정해야 함.	
	static float XXt[9], mmt[9], tcov[9], feat[3], tmean[3], tv[3];
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

	p_fv = p_mv = p_cm = p_cmi = p_cmd = p_cmid = p_sd = NULL;

	// get constants and pointers of input variables.
	n_samp = (int)in_sample_features.size();
	d_feat = io_color_model->gdf_Get_Dimension_of_Feature();
	n_gauss = io_color_model->gnm_Get_Number_of_Models();
	c_type = io_color_model->gct_Get_Color_Type();

	p_smv = io_color_model->gpmvs_Get_Pointer_of_Mean_Vector_Set(in_class_idx);
	if(io_color_model->gffc_Get_Flag_of_Full_Covariance()){
		p_scm = io_color_model->gpcms_Get_Pointer_of_Covariance_Matrix_Set(in_class_idx);
		p_scmi = io_color_model->gpcmis_Get_Pointer_of_Covariance_Matrix_Inverse_Set(in_class_idx);
	}
	else{
		p_scmd = io_color_model->gpcmds_Get_Pointer_of_Covariance_Matrix_Diagonal_Set(in_class_idx);
		p_scmid = io_color_model->gpcmids_Get_Pointer_of_Covariance_Matrix_Inverse_Diagonal_Set(in_class_idx);
	}
	
	p_sd = io_color_model->gpdcm_Get_Pointer_of_Determinant_of_Covariance_Matrix(in_class_idx);

	// initialization.
	circular_mode = false;
	s_cl_idx.clear();		v_tidcs.c_Create(n_samp, 0);
	for (int k = 0; k<n_gauss; k++){ s_cl_idx.push_back(v_tidcs); }
	p_cl_sz = v_cl_sz.c_Create(n_gauss, 0);

	// create mean vector and covariance matrix.
// 	for (int k = 0; k<n_gauss; k++){
// 		p_smv[k].c_Create(d_feat, 0.0f);
// 		p_scmd[k].c_Create(d_feat, 0.0f);
// 	}

	if (c_type == KV_COLOR_MODEL_RGB){
		min_mean_diff = KV_MIN_MEAN_DIFFERENCE_RGB;
		max_mean_shift = 3*KV_MAX_MEAN_SHIFT_RGB*KV_MAX_MEAN_SHIFT_RGB; // squared dist.
	}
	else if (c_type == KV_COLOR_MODEL_PCS){
		min_mean_diff = KV_MIN_MEAN_DIFFERENCE_PCS;
		max_mean_shift = 3*KV_MAX_MEAN_SHIFT_PCS*KV_MAX_MEAN_SHIFT_PCS; // squared dist.
	}
	else{
		min_mean_diff = KV_MIN_MEAN_DIFFERENCE_HUE;
		max_mean_shift = KV_MAX_MEAN_SHIFT_HUE;							// circular dist.
		circular_mode = true;
	}

	// set initial means randomly.
	srand((UINT)time(NULL));
	cnt = 0;
	for (int k = 0, i = 0; k<n_gauss; k++){
		tidx = rand() % n_samp;
		p_fv = in_sample_features[tidx].vp();
		// check redundancy for selected means.
		for (i = 0; i<k; i++){
			p_mv = p_smv[i].vp();
			tdist = 0.0f;
			// for circular quantities.	(for 1-D feature only)
			if (circular_mode){ tdist = abs(p_fv[0] - p_mv[0]);	tdist = DIST_CIRCULAR(tdist); }
			else{ for (int n = 0; n<d_feat; n++)	tdist += SQUARE(p_fv[n] - p_mv[n]); }
			// redundancy check.
			if (tdist<SQUARE(min_mean_diff))		break;
		}
		if (i != k){
			// reduce number of Gaussian models if new initial mean is not founded until maximum iteration number.
			if (cnt>KV_MAX_ITERATION_FOR_INITIAL_MEAN)	{
				n_gauss = k;		io_color_model->cnm_Change_Number_of_Models(n_gauss);
				break;
			}
			k--;		cnt++;		continue;
		}
		// select current sample feature to mean feature.
		p_mv = p_smv[k].vp();
		for (int n = 0; n<d_feat; n++)		p_mv[n] = p_fv[n];

// 		d_pm_Printf_Matrix(p_mv, 1, 3, "Mean");
// 		if(!Kv_Printf("n_gauss: %d", n_gauss))	exit(0);

		cnt = 0;		// initialize iteration counter if new initial mean is founded.
	}

	// clustering.
	if(n_gauss == 1){
		// set cluster size.
		p_cl_sz[0] = n_samp;
		for(int i = 0; i<n_samp; i++){
			// assign index i to cluster_idx-th cluster.
			s_cl_idx[0].vp()[i] = i;
		}

	}
	else{
		while(1){
			// initialize cluster size.
			for(int k = 0; k<n_gauss; k++)	p_cl_sz[k] = 0;			
			// assign cluster index to each sample.
			for(int i = 0; i<n_samp; i++){
				p_fv = in_sample_features[i].vp();
				// classify current pixel.
				min_dist = 10e15f;
				for(int k = 0; k<n_gauss; k++){
					p_mv = p_smv[k].vp();
					tdist = 0.0f;
					// compute distance.
					if(circular_mode){ tdist = abs(p_fv[0] - p_mv[0]);	tdist = DIST_CIRCULAR(tdist); } // for circular quantities.	(for 1-D feature only)
					else{ for(int n = 0; n<d_feat; n++)	tdist += SQUARE(p_fv[n] - p_mv[n]); }
					// update minimum distance.
					if(tdist<min_dist){ min_dist = tdist;		cl_idx = k; }
				}
				// assign index i to cluster_idx-th cluster.
				s_cl_idx[cl_idx].vp()[p_cl_sz[cl_idx]++] = i;
			}

			// update new mean vector of each class.
			s_mv_prev.clear();
			for(int k = 0; k<n_gauss; k++){
				// back up previous mean color.
				s_mv_prev.push_back(p_smv[k]);
				p_tidcs = s_cl_idx[k].vp();	p_mv = p_smv[k].vp();
				for(int n = 0; n<d_feat; n++)	p_mv[n] = 0.0f;			// initialize mean vector.
				if(circular_mode){
					// compute circular mean.
					s_term = c_term = 0.0f;
					for(int i = 0; i<p_cl_sz[k]; i++){
						p_fv = in_sample_features[p_tidcs[i]].vp();
						s_term += (float)sin(p_fv[0] * PI / 180.0f);			
						c_term += (float)cos(p_fv[0] * PI / 180.0f);
					}
					s_term /= (float)p_cl_sz[k];			c_term /= (float)p_cl_sz[k];
					// calculate atan of which range is 0~360 degree.
					p_mv[0] = (float)(atan2(s_term,c_term)*180.0f / PI);
					p_mv[0] = (p_mv[0] >= 0) ? p_mv[0] : 360.0f + p_mv[0];

				} else{
					// compute Euclidean mean.
					for(int i = 0; i<p_cl_sz[k]; i++){
						p_fv = in_sample_features[p_tidcs[i]].vp();
						for(int n = 0; n<d_feat; n++)	p_mv[n] += p_fv[n];
					}
					for(int n = 0; n<d_feat; n++)	p_mv[n] /= (float)p_cl_sz[k];
				}
			}

			// check terminal condition.
			tdist = 0.0f;
			for(int k = 0; k<n_gauss; k++){
				p_fv = s_mv_prev[k].vp();			p_mv = p_smv[k].vp();
				if(circular_mode){ tdist = abs(p_fv[0] - p_mv[0]);		tdist = DIST_CIRCULAR(tdist); } 
				else{ for(int n = 0; n<d_feat; n++)	tdist += SQUARE(p_fv[n] - p_mv[n]); }
			}

			/// //////////////////////////////
			if(tdist<max_mean_shift)		break;
			/// //////////////////////////////
		}
	}
	

	// set results.
	for (int k = 0; k<n_gauss; k++){
		p_out_idcs = out_cluster_indices_or_NULL[k].c_Create(p_cl_sz[k], 0);
		p_tidcs = s_cl_idx[k].vp();
		p_mv = p_smv[k].vp();		

		if(!io_color_model->gffc_Get_Flag_of_Full_Covariance()){

			p_cmd = p_scmd[k].vp();		p_cmid = p_scmid[k].vp();
			for(int n = 0; n<d_feat; n++)	p_cmd[n] = 0.0f;
			// compute covariance matrix.
			for (int i = 0; i<p_cl_sz[k]; i++){
				// set indices of results.
				p_out_idcs[i] = p_tidcs[i];		p_fv = in_sample_features[p_tidcs[i]].vp();
				if (circular_mode){ tdist = abs(p_fv[0] - p_mv[0]);	tdist = DIST_CIRCULAR(tdist);		p_cmd[0] += SQUARE(tdist); }
				else{ for (int n = 0; n<d_feat; n++)	p_cmd[n] += SQUARE(p_fv[n] - p_mv[n]); }
			}
			for (int n = 0; n<d_feat; n++)	p_cmd[n] /= (float)(p_cl_sz[k] - 1);  // n - 1?
			//////////////////////////////////////////////////////////////////////////
			// prevent zero variance in diagonal term of covariance matrix.
			for (int n = 0; n<d_feat; n++)	p_cmd[n] = (p_cmd[n] < 1.0f) ? 1.0f : p_cmd[n];
			//////////////////////////////////////////////////////////////////////////
			// compute inverse covariance matrices and determinant.
			p_sd[k] = 1.0f;
			for (int n = 0; n<d_feat; n++){
				p_cmid[n] = 1.0f / p_cmd[n];		// inverse covariance matrix.
				p_sd[k] *= p_cmd[n];				// determinant of covariance matrix.
			}

// 			d_pm_Printf_Matrix(p_mv,1,3,"Mean");
// 			d_pm_Printf_Matrix(p_cmd,1,3,"CMD");
// 			d_pm_Printf_Matrix(p_cmid,1,3,"CMDI");
// 			d_pm_Printf_Matrix(p_sd,1,1,"Det");
		}
		else{			
			int len = SQUARE(d_feat);

			// initialize cov. mat.
			p_cm = p_scm[k].vp();					
			for(int i = 0; i<len; i++)	p_cm[i] = 0.0;


			//////////////////////////////////////////////////////////////////////////
			// Sample covariance 를 구해야 할 듯.
			// N-1 로 나눠야 함.
			// Cov[X] = E[XX^t] - mu*mu^t. 이걸로 못 구할듯.

			// Assignment score 구할 때 다시 생각 해볼... N 나누기 할 때
			//////////////////////////////////////////////////////////////////////////
// 			// compute mu*mu^t.
// 			d_opv_Outer_Product_Vector(p_mv, p_mv, d_feat, mmt);
// 			// compute E[XX^t].
// 			for(int i = 0; i<p_cl_sz[k]; i++){
// 				// set indices of results.
// 				p_out_idcs[i] = p_tidcs[i];		p_fv = in_sample_features[p_tidcs[i]].vp();
// 				// compute weighted sum for new covariance matrix.
// 				d_opv_Outer_Product_Vector(p_fv, p_fv, d_feat, XXt);
// 				for(int kk = 0; kk<9; kk++)	p_cm[kk] += XXt[kk];
// 			}
// 
// 			// Cov[X] = E[XX^t] - mu*mu^t.
// 			for(int kk=0; kk<9; kk++)	p_cm[kk] = p_cm[kk]/(float)p_cl_sz[k] - mmt[kk];

			for(int i = 0; i<n_samp; i++){

				// get pointer of sample feature.
				p_fv = in_sample_features[i].vp();
				// compute weighted sum for new covariance matrix.
				for(int m=0; m<3; m++) tv[m] = p_fv[m] - p_mv[m];
				for(int m=0; m<3; m++){
					for(int n=0; n<3; n++){
						p_cm[m*3 + n] += tv[m]*tv[n];
					}
				}
			}
			for(int kk = 0; kk<9; kk++) 	p_cm[kk] /= (float)(p_cl_sz[k] - 1); // n - 1?

			//////////////////////////////////////////////////////////////////////////
			// prevent zero variance in diagonal term of covariance matrix.
			//for(int kk=0; kk<9; kk++)	p_cm[kk] = (p_cm[kk] < 1.0f) ? 1.0f : p_cm[kk];
			//////////////////////////////////////////////////////////////////////////

			// compute inverse and determinant of covariance matrix.
  			p_cmi = p_scmi[k].vp();
  			p_sd[k] = d_im3_Inverse_Matrix_3(p_cm, p_cmi);

// 			d_pm_Printf_Matrix(p_cm,3,3,"p_cm");
// 			//if(!Kv_Printf("p_cm"))	exit(0);
// 
// 
// 			d_pm_Printf_Matrix(p_cmi,3,3,"p_cmi");
// 
// 			float id[9];
// 			d_mms_Multiply_Matrix_Square(p_cm,p_cmi,3,id);
// 			d_pm_Printf_Matrix(id,3,3,"I");
// 
// 			if(!Kv_Printf("p_sd[k]: %f",p_sd[k]))	exit(0);
		}
	}

	s_cl_idx.clear();
	s_mv_prev.clear();
}

//********************************************************************************
void LCKvYooji_Color_Segmentation::z_em_Expectation_and_Maximization(vector<CKvVectorFloat> &in_sample_features,
	int in_class_idx,
	CKvYooji_ColorModel *io_color_model)
//********************************************************************************
{
	// p_: pointer of, s_: set of, pv: probability vector, pp: post probability.
	// for local function.
	CKvVectorFloat *s_mv;		// set of mean vectors.
	CKvVectorFloat *s_ppv;		// set of post probability vectors.
	CKvVectorFloat pv, wv, sum_as, sin_term, cos_term;
	float *p_pv, *p_ppv, *p_sum_as, *p_sin_term, *p_cos_term;
	float tp, assign_score, tang, tdist;
	bool circular_mode;
	// for input variables.
	CKvVectorFloat *p_smv, *p_scmd, *p_scmid, *p_tv;
	CKvMatrixFloat *p_scm, *p_scmi;
	float *p_wv, *p_fv, *p_mv, *p_cm, *p_cmi, *p_cmd, *p_cmid, *p_sd;
	int n_samp, n_gauss, feat_dim, color_type;
	// for computing full covariance matrix.
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// 추후 feature dimension에 맞게 수정해야 함.
	static float XXt[9], mmt[9], tv[3];
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

	p_fv = p_mv = p_cm = p_cmi = p_cmd = p_cmid = p_wv = p_sd = NULL;

	// get constants and pointers of input variables.
	n_samp = (int)in_sample_features.size();
	feat_dim = io_color_model->gdf_Get_Dimension_of_Feature();
	n_gauss = io_color_model->gnm_Get_Number_of_Models();
	color_type = io_color_model->gct_Get_Color_Type();

	p_smv = io_color_model->gpmvs_Get_Pointer_of_Mean_Vector_Set(in_class_idx);
	if(io_color_model->gffc_Get_Flag_of_Full_Covariance()){
		p_scm = io_color_model->gpcms_Get_Pointer_of_Covariance_Matrix_Set(in_class_idx);
		p_scmi = io_color_model->gpcmis_Get_Pointer_of_Covariance_Matrix_Inverse_Set(in_class_idx);
	}
	else{
		p_scmd = io_color_model->gpcmds_Get_Pointer_of_Covariance_Matrix_Diagonal_Set(in_class_idx);
		p_scmid = io_color_model->gpcmids_Get_Pointer_of_Covariance_Matrix_Inverse_Diagonal_Set(in_class_idx);
	}

	p_wv = io_color_model->gpmw_Get_Pointer_of_Model_Weights(in_class_idx);
	p_sd = io_color_model->gpdcm_Get_Pointer_of_Determinant_of_Covariance_Matrix(in_class_idx);

	// ===========================================================
	// Initialize local variables.
	// ===========================================================
	circular_mode = false;	p_sin_term = p_cos_term = NULL;
	p_pv = pv.c_Create(n_gauss, 0.0f);			p_sum_as = sum_as.c_Create(n_gauss, 0.0f);
	s_mv = new CKvVectorFloat[n_gauss];		for (int k = 0; k<n_gauss; k++){ s_mv[k].c_Create(feat_dim, 0.0f); }
	// + set of post probability vectors for each sample features.
	s_ppv = new CKvVectorFloat[n_samp];		for (int k = 0; k<n_samp; k++){ s_ppv[k].c_Create(n_gauss, 0.0f); }

	if (color_type == KV_COLOR_MODEL_HUE)		circular_mode = true;
	if (circular_mode){
		p_sin_term = sin_term.c_Create(n_gauss, 0.0f);
		p_cos_term = cos_term.c_Create(n_gauss, 0.0f);
	}
	// ===========================================================
	// Re-initialization.
	// ===========================================================
// 	for (int k = 0; k<n_gauss; k++){
// 		p_cmd = p_scmd[k].vp(); p_cm = p_scm[k].vp();
// 		for (int n = 0; n<feat_dim; n++){ p_cmd[n] = 0.0f; }
// 		for (int n = 0; n<feat_dim*feat_dim; n++){ p_cm[n] = 0.0f; }
// 	}
	// ===========================================================
	// ====================== Expectation ========================
	// ===========================================================
	// Determine sample's assignment score for each Gaussian model.
	// ===========================================================
	for (int i = 0; i<n_samp; i++){
		// compute probability vector.
		tp = 0.0f;		p_tv = &in_sample_features[i];
		for (int k = 0; k<n_gauss; k++){
			//////////////////////////////////////////////////////////////////////////
			if (circular_mode)		p_pv[k] = io_color_model->gpdc_Get_Probability_Density_Circular(p_tv, in_class_idx, k);
			//////////////////////////////////////////////////////////////////////////
			else					p_pv[k] = io_color_model->gpd_Get_Probability_Density(p_tv, in_class_idx, k);
			/* 			if(circular_mode)		p_pv[k]=z_gpdc_Get_Probability_Density_Circular(p_tv, &s_mv[k], &p_scmid[k], p_sd[k]);
			else								p_pv[k]=z_gpd_Get_Probability_Density(p_tv, &s_mv[k], &p_scmid[k], p_sd[k]);				*/
			tp += p_pv[k] * p_wv[k];

			//printf("[%d] tp: %f / p_pv: %f / p_wv: %f\n", zz_flag_full_cov_mat, tp, p_pv[k], p_wv[k]);
		}
		//
		p_fv = p_tv->vp();		p_ppv = s_ppv[i].vp();
		for (int k = 0; k<n_gauss; k++){
			// calculate assignment score.
			assign_score = p_pv[k] * p_wv[k] / tp;
			p_ppv[k] = assign_score;

//  			printf("tp: %f / p_pv: %f / p_wv: %f\n", tp, p_pv[k], p_wv[k]);
//  			if(!Kv_Printf("assign_score: %f", assign_score))	exit(0);

			//// for updating mean vector.			
			//if (circular_mode){ tang = (float)(p_fv[0] * PI / 180.0f);	p_sin_term[k] += assign_score*sin(tang);		p_cos_term[k] += assign_score*cos(tang); }
			//else{ p_mv = s_mv[k].vp();		for (int n = 0; n<feat_dim; n++)		p_mv[n] += assign_score*p_fv[n]; }
			
		}
	}
	
	

	// ===========================================================
	// ====================== Maximization =======================
	// ===========================================================
	// Update new mean vectors.
	// ===========================================================
	for(int k = 0; k<n_gauss; k++){

		// initialize pointers.
		p_mv = p_smv[k].vp();	for(int n=0; n<feat_dim; n++)	p_mv[n] = 0.0f;

		p_sum_as[k] = 0.0f;
		for(int i = 0; i<n_samp; i++){
			// get pointer of sample feature.
			p_fv = in_sample_features[i].vp();
			// get assignment score vector of k-th Gaussian model.
			p_ppv = s_ppv[i].vp();
			assign_score = p_ppv[k];
			// compute weighted sum for new mean.
			for(int n=0; n<feat_dim; n++)	p_mv[n] += assign_score*p_fv[n];

			p_sum_as[k] += assign_score;
		}

		//printf("[%d] p_sum_as: %f\n",k,p_sum_as[k]);

		for(int n=0; n<feat_dim; n++)	p_mv[n] /= p_sum_as[k];

		//if(!Kv_Printf("p_sum_as: %f", p_sum_as[k]))	exit(0);
	}


	// ===========================================================
	// Update new covariance matrix and weights.
	// ===========================================================	
	// for multiple Gaussian models.
	for(int k = 0; k<n_gauss; k++){
		p_mv = p_smv[k].vp();	// mean vector.
		//tp = p_sum_ppv[k];		// total post probability. 

		// Compute diagonal covariance matrix.
		//////////////////////////////////////////////////////////////////////////
		/// 나중에 color_model class 에서 받아오는 것으로 바꾸기.
		if(!io_color_model->gffc_Get_Flag_of_Full_Covariance()){
			//////////////////////////////////////////////////////////////////////////
			// initialize cov. mat.
			p_cmd = p_scmd[k].vp();	 for(int n = 0; n<feat_dim; n++){ p_cmd[n] = 0.0f; }

			// calculate covariances.
			//p_sum_as[k] = 0.0f;
			for(int i = 0; i<n_samp; i++){

				// get pointer of sample feature.
				p_fv = in_sample_features[i].vp();
				// get assignment score vector of k-th Gaussian model.
				p_ppv = s_ppv[i].vp();
				assign_score = p_ppv[k];	//p_sum_as[k] += assign_score;
				// compute weighted sum for new covariance matrix.
				for(int n = 0; n<feat_dim; n++)		p_cmd[n] += assign_score*SQUARE(p_fv[n] - p_mv[n]);
			}
			for(int n = 0; n<feat_dim; n++)	p_cmd[n] /= p_sum_as[k];

			//////////////////////////////////////////////////////////////////////////
			// prevent zero variance in diagonal term of covariance matrix.
			for(int n = 0; n<feat_dim; n++)	p_cmd[n] = (p_cmd[n] < 1.0f) ? 1.0f : p_cmd[n];
			//////////////////////////////////////////////////////////////////////////

			// calculate new inverse covariance matrices, determinant, and model weights.
			p_cmid = p_scmid[k].vp();			p_sd[k] = 1.0f;
			for(int n = 0; n<feat_dim; n++){
				p_cmid[n] = 1.0f / p_cmd[n];		// inverse covariance matrix.
				p_sd[k] *= p_cmd[n];				// determinant of covariance matrix.
			}

			// update model weights.
			p_wv[k] = p_sum_as[k] / (float)n_samp;		// model weights.

			// 			d_pm_Printf_Matrix(p_cmd,1,3,"p_cmd");
			// 			d_pm_Printf_Matrix(p_cmid,1,3,"p_cmid");
		}
		// Compute full covariance matrix.
		else{
			int len = SQUARE(feat_dim);
			// initialize cov. mat.
			p_cm = p_scm[k].vp();		for(int i = 0; i<len; i++)	p_cm[i] = 0.0f;
// 				// compute mu*mu^t.
// 				d_opv_Outer_Product_Vector(p_mv,p_mv,feat_dim,mmt);
// 				// compute E[XX^t].
// 				for(int i = 0; i<n_samp; i++){
// 
// 					// get pointer of sample feature.
// 					p_fv = in_sample_features[i].vp();
// 					// get assignment score vector of k-th Gaussian model.
// 					p_ppv = s_ppv[i].vp();
// 					assign_score = p_ppv[k];
// 					// compute weighted sum for new covariance matrix.
// 					for(int m=0; m<3; m++) tv[m] = p_fv[m] - p_mv[m];
// 					for(int m=0; m<3; m++){
// 						for(int n=0; n<3; n++){
// 							p_cm[m*3 + n] += assign_score*tv[m]*tv[n];
// 						}
// 					}
// 				}
// 				//////////////////////////////////////////////////////////////////////////
// 				for(int kk = 0; kk<9; kk++)		p_cm[kk] /= p_sum_as[k]; 
// 				//////////////////////////////////////////////////////////////////////////

			for(int i = 0; i<n_samp; i++){
				 
				// get pointer of sample feature.
				p_fv = in_sample_features[i].vp();
				// get assignment score vector of k-th Gaussian model.
				p_ppv = s_ppv[i].vp();
				assign_score = p_ppv[k];
				// compute weighted sum for new covariance matrix.
				for(int m=0; m<3; m++) tv[m] = p_fv[m] - p_mv[m];
				for(int m=0; m<3; m++){
				 	for(int n=0; n<3; n++){
				 		p_cm[m*3 + n] += assign_score*tv[m]*tv[n];
				 	}
				}
			}
			//////////////////////////////////////////////////////////////////////////
			for(int kk = 0; kk<9; kk++)		p_cm[kk] /= p_sum_as[k]; 
			//////////////////////////////////////////////////////////////////////////

			//////////////////////////////////////////////////////////////////////////
			// prevent zero variance in diagonal term of covariance matrix.
			//for(int kk = 0; kk<9; kk++) 	p_cm[kk] = (p_cm[kk] < 1.0f) ? 1.0f : p_cm[kk];
			//////////////////////////////////////////////////////////////////////////

			// 			d_pm_Printf_Matrix(p_mv, 1, 3, "p_mv");
			// 			d_pm_Printf_Matrix(p_cm, 3, 3, "p_cm");
			// 			if(!Kv_Printf("p_sum_as: %f", p_sum_as[k]))	exit(0);

			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			// Inverse 가 이상한거 같기도....?
			// compute inverse and determinant of covariance matrix.
			p_cmi = p_scmi[k].vp();		//p_sd[k] = 1.0f;
			p_sd[k] = d_im3_Inverse_Matrix_3(p_cm,p_cmi);

			if(p_sd[k] == 0.0f) d_pm_Printf_Matrix(p_cm,3,3,"Cov");

			// update model weights.
			p_wv[k] = p_sum_as[k] / (float)n_samp;		// model weights.
			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		}
		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


	}

	
	delete[] s_mv;
	delete[] s_ppv;
}


//********************************************************************************
void LCKvYooji_Color_Segmentation::gsfg_Get_Spatial_Filter_Gaussian(CKvMatrixFloat &out_filter)
//********************************************************************************
{
	CKvMatrixFloat filter_row;
	int ww_f, hh_f;
	float *p_filter, *p_filter_row;

	// filter design.
	ww_f = hh_f = 5;
	p_filter = out_filter.c_Create(hh_f, ww_f, 0.0f)[0];
	p_filter_row = filter_row.c_Create(hh_f, 1, 0.0f)[0];

	p_filter_row[0] = 0.05f;		p_filter_row[1] = 0.25f;		p_filter_row[2] = 0.4f;		p_filter_row[3] = 0.25f;		p_filter_row[4] = 0.05f;
	for (int j = 0; j<hh_f; j++){
		for (int i = 0; i<ww_f; i++){
			p_filter[j*ww_f + i] = p_filter_row[j] * p_filter_row[i];
		}
	}
}
//********************************************************************************
void LCKvYooji_Color_Segmentation::gsfgc_Get_Sample_Features_RGB_for_Grab_Cut(
	CKvMatrixUcharRgb &in_color_image,
	vector<vector<CKvVectorFloat>> *out_set_of_feature_clusters,
	vector<CKvMatrixBool> *out_set_of_masks = NULL)
//********************************************************************************
{
	CKvScreen sc;

	CKvMatrixUcharRgb tmp_img;
	CKvMatrixBool mask_img;

	CKvVectorFloat tmp_rgb;
	CKvVectorFloat tmp_vec;
	CKvPixel point, lt, rb;

	int ww, hh, x, y, i, j, idx;
	int num_clusters, cnt, status, prev_status, size;
	float r, g, b;
	bool flag_click;

	unsigned char *p_in_img, *p_tmp_img;
	bool *p_mask_img;
	float *p_tmp_rgb;
	float *p_tmp_vec;

	p_tmp_img = tmp_img.cp_Copy(&in_color_image)[0];

	// initialization.
	if(out_set_of_feature_clusters){ 
		(*out_set_of_feature_clusters).clear();	
		(*out_set_of_feature_clusters).resize(2);		
	}
	if(out_set_of_masks)	(*out_set_of_masks).clear();
	p_in_img = in_color_image.mps(ww, hh)[0];
	p_mask_img = mask_img.c_Create(hh, ww, false)[0];
	p_tmp_rgb = tmp_rgb.c_Create(3, 0.0f);
	p_tmp_vec = tmp_vec.c_Create(3, 0.0f);

	num_clusters = 2;	cnt = 0;		size = 4;
	status = prev_status = 0;

	// get sample points.
	sc.s_smbm_Set_Mouse_Button_Mode(false);
	sc.s_d_Display(&tmp_img);

	flag_click = false;
	while (cnt<num_clusters){

		point = sc.s_gmps_Get_Mouse_Position_and_Status(status);

		if (status){

			// Bounding box for background pixels.
			if(cnt == 0){

				// left-top
				if(!flag_click){	lt = point;	flag_click = true;	}
				// right-bottom
				else				rb = point;

				tmp_img.sbox_Set_Box(lt.x, lt.y, rb.x - lt.x, rb.y - lt.y, Kv_Rgb(0, 255, 0));
			}
			// Brushing for foreground pixels.
			else{
				if(point.x >= size && point.x <= ww - size - 1 && point.y >= size && point.y <= hh - size - 1){

					for(j = -size; j<size + 1; j++){
						for(i = -size; i<size + 1; i++){
							x = point.x + i;		y = point.y + j;
							idx = y*ww + x;

							p_mask_img[idx] = true;

							p_tmp_img[idx] = (unsigned char)0;			idx += ww*hh;
							p_tmp_img[idx] = (unsigned char)255;		idx += ww*hh;
							p_tmp_img[idx] = (unsigned char)0;
						}
					}

				}
			}

			
			sc.s_d_Display(&tmp_img);

			Kv_Pause(1);

			if(cnt==0)	tmp_img.cp_Copy(&in_color_image);

		}
		// if mouse left button is clicked.
		else if (prev_status){

			// Set outside of bounding box to background mask.
			if(cnt == 0){
				for(j = 0; j<hh; j++){
					for(i = 0; i<ww; i++){
						idx = ww*j + i;					
						if(i<lt.x || i>rb.x || j<lt.y || j>rb.y)	p_mask_img[idx] = true;
						else										p_mask_img[idx] = false;
					}
				}
			}

			/// //////////////////////////////
			// append masks.
			if(out_set_of_masks)	(*out_set_of_masks).push_back(mask_img);
			// calculate initial cluster means.	
			for (j = 0; j<hh; j++){
				for (i = 0; i<ww; i++){
					idx = ww*j + i;
					if (p_mask_img[idx]){
						p_mask_img[idx] = false;

						r = (float)p_in_img[idx];				idx += ww*hh;
						g = (float)p_in_img[idx];				idx += ww*hh;
						b = (float)p_in_img[idx];

						// convert color coordinates.
						p_tmp_rgb[0] = r;	p_tmp_rgb[1] = g;	p_tmp_rgb[2] = b;
						/// /////////////////////////////////////
						// append current color.			
						(*out_set_of_feature_clusters)[cnt].push_back(tmp_rgb);
					}
				}
			}
			// initialize temporary image.
			p_tmp_img = tmp_img.cp_Copy(&in_color_image)[0];

			cnt++;
		}

		if (status == -1)			break;
		// update status.
		prev_status = status;

		Kv_Pause(1);
	}
}

//********************************************************************************
void LCKvYooji_Color_Segmentation::gsfb_Get_Sample_Features_RGB_using_Brushing(CKvMatrixUcharRgb &in_color_image,
	int in_cluster_num,
	vector<vector<CKvVectorFloat>> *out_set_of_feature_clusters,
	vector<CKvMatrixBool> *out_set_of_masks = NULL)
//********************************************************************************
{
	CKvScreen sc;

	CKvMatrixUcharRgb tmp_img;
	CKvMatrixBool mask_img;

	CKvVectorFloat tmp_rgb;
	CKvVectorFloat tmp_vec;
	CKvPixel point;

	int ww, hh, x, y, i, j, idx;
	int num_clusters, cnt, status, prev_status, size;
	float r, g, b;

	unsigned char *p_in_img, *p_tmp_img;
	bool *p_mask_img;
	float *p_tmp_rgb;
	float *p_tmp_vec;

	p_tmp_img = tmp_img.cp_Copy(&in_color_image)[0];

	// initialization.
	if(out_set_of_feature_clusters){ 
		(*out_set_of_feature_clusters).clear();	
		(*out_set_of_feature_clusters).resize(in_cluster_num);		
	}
	if(out_set_of_masks)	(*out_set_of_masks).clear();
	p_in_img = in_color_image.mps(ww, hh)[0];
	p_mask_img = mask_img.c_Create(hh, ww, false)[0];
	p_tmp_rgb = tmp_rgb.c_Create(3, 0.0f);
	p_tmp_vec = tmp_vec.c_Create(3, 0.0f);

	num_clusters = in_cluster_num;	cnt = 0;		size = 4;
	status = prev_status = 0;

	// get sample points.
	sc.s_smbm_Set_Mouse_Button_Mode(false);
	sc.s_d_Display(&tmp_img);
	while (cnt<num_clusters){

		point = sc.s_gmps_Get_Mouse_Position_and_Status(status);

		if (status){
			if (point.x >= size && point.x <= ww - size - 1 && point.y >= size && point.y <= hh - size - 1){

				for (j = -size; j<size + 1; j++){
					for (i = -size; i<size + 1; i++){
						x = point.x + i;		y = point.y + j;
						idx = y*ww + x;

						p_mask_img[idx] = true;

						p_tmp_img[idx] = (unsigned char)0;			idx += ww*hh;
						p_tmp_img[idx] = (unsigned char)255;		idx += ww*hh;
						p_tmp_img[idx] = (unsigned char)0;
					}
				}

			}
			sc.s_d_Display(&tmp_img);

			Kv_Pause(1);
		}
		else if (prev_status){

			/// //////////////////////////////
			// append masks.
			if(out_set_of_masks)	(*out_set_of_masks).push_back(mask_img);
			// calculate initial cluster means.	
			for (j = 0; j<hh; j++){
				for (i = 0; i<ww; i++){
					idx = ww*j + i;
					if (p_mask_img[idx]){
						p_mask_img[idx] = false;

						r = (float)p_in_img[idx];				idx += ww*hh;
						g = (float)p_in_img[idx];				idx += ww*hh;
						b = (float)p_in_img[idx];

						// convert color coordinates.
						p_tmp_rgb[0] = r;	p_tmp_rgb[1] = g;	p_tmp_rgb[2] = b;
						/// /////////////////////////////////////
						// append current color.			
						(*out_set_of_feature_clusters)[cnt].push_back(tmp_rgb);
					}
				}
			}
			// initialize temporary image.
			p_tmp_img = tmp_img.cp_Copy(&in_color_image)[0];

			cnt++;
		}

		if (status == -1)			break;
		// update status.
		prev_status = status;

		Kv_Pause(1);
	}
}

//********************************************************************************
void LCKvYooji_Color_Segmentation::gsfb_Get_Sample_Features_RGB_using_Brushing(CKvMatrixUcharRgb &in_color_image,
	int in_cluster_num,
	vector<CKvVectorFloat> *out_set_of_feature_clusters)
//********************************************************************************
{
	CKvScreen sc;

	CKvMatrixUcharRgb tmp_img;
	CKvMatrixBool mask_img;

	CKvVectorFloat tmp_rgb;
	CKvVectorFloat tmp_vec;
	CKvPixel point;

	int ww, hh, x, y, i, j, idx;
	int num_clusters, cnt, status, prev_status, size;
	float r, g, b;

	unsigned char *p_in_img, *p_tmp_img;
	bool *p_mask_img;
	float *p_tmp_rgb;
	float *p_tmp_vec;

	p_tmp_img = tmp_img.cp_Copy(&in_color_image)[0];

	// initialization.
	if (out_set_of_feature_clusters == NULL){ out_set_of_feature_clusters = new vector<CKvVectorFloat>[in_cluster_num];		for (i = 0; i<in_cluster_num; i++)		out_set_of_feature_clusters[i].clear(); }
	p_in_img = in_color_image.mps(ww, hh)[0];
	p_mask_img = mask_img.c_Create(hh, ww, false)[0];
	p_tmp_rgb = tmp_rgb.c_Create(3, 0.0f);
	p_tmp_vec = tmp_vec.c_Create(3, 0.0f);

	num_clusters = in_cluster_num;	cnt = 0;		size = 2;
	status = prev_status = 0;

	// get sample points.
	sc.s_d_Display(&tmp_img);
	while (cnt<num_clusters){

		point = sc.s_gmps_Get_Mouse_Position_and_Status(status);

		if (status){
			if (point.x >= size && point.x <= ww - size - 1 && point.y >= size && point.y <= hh - size - 1){

				for (j = -size; j<size + 1; j++){
					for (i = -size; i<size + 1; i++){
						x = point.x + i;		y = point.y + j;
						idx = y*ww + x;

						p_mask_img[idx] = true;

						p_tmp_img[idx] = (unsigned char)0;			idx += ww*hh;
						p_tmp_img[idx] = (unsigned char)255;		idx += ww*hh;
						p_tmp_img[idx] = (unsigned char)0;
					}
				}

			}
			sc.s_d_Display(&tmp_img);

			Kv_Pause(1);
		}
		else if (prev_status){

			/// //////////////////////////////
			// calculate initial cluster means.	
			for (j = 0; j<hh; j++){
				for (i = 0; i<ww; i++){
					idx = ww*j + i;
					if (p_mask_img[idx]){
						p_mask_img[idx] = false;

						r = (float)p_in_img[idx];				idx += ww*hh;
						g = (float)p_in_img[idx];				idx += ww*hh;
						b = (float)p_in_img[idx];

						// convert color coordinates.
						p_tmp_rgb[0] = r;	p_tmp_rgb[1] = g;	p_tmp_rgb[2] = b;
						/// /////////////////////////////////////
						// append current color.			
						out_set_of_feature_clusters[cnt].push_back(tmp_rgb);
					}
				}
			}
			cnt++;
		}

		if (status == -1)			break;
		// update status.
		prev_status = status;

		Kv_Pause(1);
	}
}

//********************************************************************************
void LCKvYooji_Color_Segmentation::gsfis_Get_Sample_Features_from_Image_Sequence(CKvString &in_directory_name_color_sequence,
	int in_class_num,
	int in_color_type,
	vector<CKvVectorFloat> *out_set_of_feature_clusters)
//********************************************************************************
{
	LCKvHello zz_hh;

	CKvScreen sc;

	CKvVectorString set_of_dn, set_of_fn;
	CKvString fn, dn, *p_set_of_dn = NULL, *p_set_of_fn = NULL;

	CKvMatrixUcharRgb tmp_img, in_img;
	CKvMatrixBool mask_img;

	CKvVectorFloat tmp_rgb, tmp_pcs, tmp_hue;
	CKvVectorFloat tmp_vec;
	CKvPixel point;

	int ww, hh, x, y, i, j, k, idx;
	int num_dn, num_fn, tmp;
	int num_clusters, cnt, status, prev_status, size;
	unsigned char r_val, g_val, b_val;
	double h_val, s_val, i_val;

	unsigned char *p_in_img, *p_tmp_img;
	bool *p_mask_img;
	float *p_tmp_rgb, *p_tmp_pcs, *p_tmp_hue;

	sc.s_smbm_Set_Mouse_Button_Mode(false);

	zz_hh.iofv.gdi_Get_Directory_Information(in_directory_name_color_sequence, false, num_dn, num_fn);		p_set_of_fn = set_of_fn.c_Create(num_fn);
	zz_hh.iofv.gdi_Get_Directory_Information(in_directory_name_color_sequence, false, tmp, tmp, false, true, num_dn, num_fn, p_set_of_dn, p_set_of_fn);

	// initialization.
	if (out_set_of_feature_clusters == NULL){
		out_set_of_feature_clusters = new vector<CKvVectorFloat>[in_class_num];
		for (i = 0; i<in_class_num; i++)		out_set_of_feature_clusters[i].clear();
	}

	if (in_color_type == KV_COLOR_MODEL_HUE){ p_tmp_hue = tmp_hue.c_Create(1, 0.0f); }
	else{
		p_tmp_rgb = tmp_rgb.c_Create(3, 0.0f);
		p_tmp_pcs = tmp_pcs.c_Create(3, 0.0f);
	}

	// get sample points.
	num_clusters = in_class_num;		size = 2;
	for (k = 0; k<num_fn; k++){

		zz_hh.iofv.li_Load_Image(p_set_of_fn[k], false, &in_img);

		p_in_img = in_img.mps(ww, hh)[0];		p_tmp_img = tmp_img.cp_Copy(&in_img)[0];
		p_mask_img = mask_img.c_Create(hh, ww, false)[0];

		sc.s_d_Display(&tmp_img);

		cnt = 0;		status = prev_status = 0;
		while (cnt<num_clusters){

			point = sc.s_gmps_Get_Mouse_Position_and_Status(status);

			if (status){
				if (point.x >= size && point.x <= ww - size - 1 && point.y >= size && point.y <= hh - size - 1){

					for (j = -size; j<size + 1; j++){
						for (i = -size; i<size + 1; i++){
							x = point.x + i;		y = point.y + j;
							idx = y*ww + x;

							p_mask_img[idx] = true;

							p_tmp_img[idx] = (unsigned char)0;			idx += ww*hh;
							p_tmp_img[idx] = (unsigned char)255;		idx += ww*hh;
							p_tmp_img[idx] = (unsigned char)0;
						}
					}

				}
				sc.s_d_Display(&tmp_img);

				Kv_Pause(1);
			}
			else if (prev_status){

				/// //////////////////////////////
				// calculate initial cluster means.			
				for (j = 0; j<hh; j++){
					for (i = 0; i<ww; i++){
						idx = ww*j + i;
						if (p_mask_img[idx]){
							p_mask_img[idx] = false;

							r_val = p_in_img[idx];				idx += ww*hh;
							g_val = p_in_img[idx];				idx += ww*hh;
							b_val = p_in_img[idx];

							// convert color coordinates.
							if (in_color_type == KV_COLOR_MODEL_RGB){
								p_tmp_rgb[0] = (float)r_val;	p_tmp_rgb[1] = (float)g_val;	p_tmp_rgb[2] = (float)b_val;
								out_set_of_feature_clusters[cnt].push_back(tmp_rgb);
							}
							else if (in_color_type == KV_COLOR_MODEL_PCS){
								p_tmp_rgb[0] = (float)r_val;	p_tmp_rgb[1] = (float)g_val;	p_tmp_rgb[2] = (float)b_val;
								cc_Convert_Color_Rgb_to_PCS(tmp_rgb, tmp_pcs);
								out_set_of_feature_clusters[cnt].push_back(tmp_pcs);
							}
							else{
								ccvrh_Convert_Color_Value_RGB_to_HSI(r_val, g_val, b_val, h_val, s_val, i_val);
								if (s_val>0.1 && i_val <200.0f){
									p_tmp_hue[0] = (float)h_val;
									out_set_of_feature_clusters[cnt].push_back(tmp_hue);
								}
							}

							/// /////////////////////////////////////

						}
					}
				}
				cnt++;
			}

			if (status == -1)			break;
			// update status.
			prev_status = status;

			Kv_Pause(1);
		}
	}
}


//********************************************************************************
void LCKvYooji_Color_Segmentation::gsf_Get_Sample_Features_PCS(CKvMatrixUcharRgb &in_color_image,
	int in_cluster_num,
	vector<CKvVectorFloat> *out_set_of_feature_clusters)
//********************************************************************************
{
	CKvScreen sc;

	CKvMatrixUcharRgb tmp_img;
	CKvMatrixBool mask_img;

	CKvVectorFloat tmp_rgb, tmp_pcs;
	CKvVectorFloat tmp_vec;
	CKvPixel point;

	int ww, hh, x, y, i, j, idx;
	int num_clusters, cnt, status, prev_status, size;
	float r, g, b;

	unsigned char *p_in_img, *p_tmp_img;
	bool *p_mask_img;
	float *p_tmp_rgb, *p_tmp_pcs;
	float *p_tmp_vec;

	p_tmp_img = tmp_img.cp_Copy(&in_color_image)[0];

	// initialization.
	if (out_set_of_feature_clusters == NULL){ out_set_of_feature_clusters = new vector<CKvVectorFloat>[in_cluster_num];		for (i = 0; i<in_cluster_num; i++)		out_set_of_feature_clusters[i].clear(); }
	p_in_img = in_color_image.mps(ww, hh)[0];
	p_mask_img = mask_img.c_Create(hh, ww, false)[0];
	p_tmp_rgb = tmp_rgb.c_Create(3, 0.0f);		p_tmp_pcs = tmp_pcs.c_Create(3, 0.0f);
	p_tmp_vec = tmp_vec.c_Create(3, 0.0f);

	num_clusters = in_cluster_num;	cnt = 0;		size = 2;
	status = prev_status = 0;

	// get sample points.
	sc.s_d_Display(&tmp_img);
	while (cnt<num_clusters){

		point = sc.s_gmps_Get_Mouse_Position_and_Status(status);

		if (status){
			if (point.x >= size && point.x <= ww - size - 1 && point.y >= size && point.y <= hh - size - 1){

				for (j = -size; j<size + 1; j++){
					for (i = -size; i<size + 1; i++){
						x = point.x + i;		y = point.y + j;
						idx = y*ww + x;

						p_mask_img[idx] = true;

						p_tmp_img[idx] = (unsigned char)0;			idx += ww*hh;
						p_tmp_img[idx] = (unsigned char)255;		idx += ww*hh;
						p_tmp_img[idx] = (unsigned char)0;
					}
				}

			}
			sc.s_d_Display(&tmp_img);

			Kv_Pause(1);
		}
		else if (prev_status){

			/// //////////////////////////////
			// calculate initial cluster means.			
			for (j = 0; j<hh; j++){
				for (i = 0; i<ww; i++){
					idx = ww*j + i;
					if (p_mask_img[idx]){
						p_mask_img[idx] = false;

						r = (float)p_in_img[idx];				idx += ww*hh;
						g = (float)p_in_img[idx];				idx += ww*hh;
						b = (float)p_in_img[idx];

						// convert color coordinates.
						p_tmp_rgb[0] = r;	p_tmp_rgb[1] = g;	p_tmp_rgb[2] = b;
						cc_Convert_Color_Rgb_to_PCS(tmp_rgb, tmp_pcs);
						/// /////////////////////////////////////
						// append current color.			
						out_set_of_feature_clusters[cnt].push_back(tmp_pcs);
					}
				}
			}
			cnt++;
		}

		if (status == -1)			break;
		// update status.
		prev_status = status;

		Kv_Pause(1);
	}
}


//********************************************************************************
void LCKvYooji_Color_Segmentation::sgmmp_Save_Gaussian_Mixture_Model_Parameters(CKvString &in_save_filename,
	int in_cluster_num,
	int in_gaussian_num,
	int in_pcs_color_flag,
	vector<CKvVectorFloat> *in_set_of_mean_vector,
	vector<CKvMatrixFloat> *in_set_of_covariance_matrix,
	CKvVectorFloat *in_set_of_weights)
	//********************************************************************************
{
	FILE *fp;

	float *tmp_vec;
	int feat_dim;

	fopen_s(&fp, in_save_filename.bp(), "wb");

	feat_dim = in_set_of_mean_vector[0][0].vs();

	fprintf_s(fp, "%d %d %d %d\n", in_cluster_num, in_gaussian_num, feat_dim, in_pcs_color_flag);
	printf("%d %d %d %d\n", in_cluster_num, in_gaussian_num, feat_dim, in_pcs_color_flag);


	// mean vectors.
	for (int i = 0; i<in_cluster_num; i++){
		for (int k = 0; k<in_gaussian_num; k++){
			tmp_vec = in_set_of_mean_vector[i][k].vp();
			for (int n = 0; n<feat_dim; n++){
				fprintf_s(fp, "%f ", tmp_vec[n]);
				printf("%f ", tmp_vec[n]);
			}
		}
		fprintf_s(fp, "\n");
		printf("\n");
	}
	// covariance matrices.
	for (int i = 0; i<in_cluster_num; i++){
		for (int k = 0; k<in_gaussian_num; k++){
			tmp_vec = in_set_of_covariance_matrix[i][k].vp();
			for (int n = 0; n<feat_dim; n++){
				fprintf_s(fp, "%f ", tmp_vec[n*feat_dim + n]);
				printf("%f ", tmp_vec[n*feat_dim + n]);
			}
		}
		fprintf_s(fp, "\n");
		printf("\n");
	}
	// weight vectors.
	for (int i = 0; i<in_cluster_num; i++){
		tmp_vec = in_set_of_weights[i].vp();
		for (int k = 0; k<in_gaussian_num; k++){
			fprintf_s(fp, "%f ", tmp_vec[k]);
			printf("%f ", tmp_vec[k]);
		}
		fprintf_s(fp, "\n");
		printf("\n");
	}

	fclose(fp);
}

//********************************************************************************
void LCKvYooji_Color_Segmentation::lgmmp_Load_Gaussian_Mixture_Model_Parameters(CKvString &in_load_filename,
	int &out_class_num,
	int &out_gaussian_num,
	int &out_pcs_color_flag,
	vector<CKvSet_of_VectorFloat> &out_set_of_mean_vector,
	vector<CKvSet_of_MatrixFloat> &out_set_of_covariance_matrix,
	vector<CKvVectorFloat> &out_set_of_weights)
	//********************************************************************************
{
	FILE *fp;

	float *tmp_vec;
	int feat_dim;

	fopen_s(&fp, in_load_filename.bp(), "rb");

	fscanf_s(fp, "%d %d %d %d\n", &out_class_num, &out_gaussian_num, &feat_dim, &out_pcs_color_flag);
	printf("%d %d %d\n", out_class_num, out_gaussian_num, feat_dim);

	// initialization.
	out_set_of_mean_vector.clear();	out_set_of_mean_vector.resize(out_class_num);		for (int i = 0; i<out_class_num; i++){ out_set_of_mean_vector[i].c_Create(out_gaussian_num); }
	out_set_of_covariance_matrix.clear();	out_set_of_covariance_matrix.resize(out_class_num);		for (int i = 0; i<out_class_num; i++){ out_set_of_covariance_matrix[i].c_Create(out_gaussian_num); }
	out_set_of_weights.clear();			out_set_of_weights.resize(out_class_num);				for (int i = 0; i<out_class_num; i++){ out_set_of_weights[i].c_Create(out_gaussian_num, 0.0f); }

	// mean vectors.
	for (int i = 0; i<out_class_num; i++){
		for (int k = 0; k<out_gaussian_num; k++){
			tmp_vec = out_set_of_mean_vector[i].gpe(k)->c_Create(feat_dim, 0.0f);
			for (int n = 0; n<feat_dim; n++){
				fscanf_s(fp, "%f ", &tmp_vec[n]);
				printf("%f ", tmp_vec[n]);
			}
		}
		printf("\n");
	}
	// covariance matrices.
	for (int i = 0; i<out_class_num; i++){
		for (int k = 0; k<out_gaussian_num; k++){
			tmp_vec = out_set_of_covariance_matrix[i].gpe(k)->c_Create(feat_dim, feat_dim, 0.0f)[0];
			for (int n = 0; n<feat_dim; n++){
				fscanf_s(fp, "%f ", &tmp_vec[n*feat_dim + n]);
				printf("%f ", tmp_vec[n*feat_dim + n]);
			}
		}

		printf("\n");
	}
	// weight vectors.
	for (int i = 0; i<out_class_num; i++){
		tmp_vec = out_set_of_weights[i].vp();
		for (int k = 0; k<out_gaussian_num; k++){
			fscanf_s(fp, "%f ", &tmp_vec[k]);
			printf("%f ", tmp_vec[k]);
		}
		printf("\n");
	}

	fclose(fp);
}

//********************************************************************************
void LCKvYooji_Color_Segmentation::lgmmp_Load_Gaussian_Mixture_Model_Parameters(CKvString &in_load_filename)
//********************************************************************************
{
	FILE *fp;

	float *tmp_vec;
	int feat_dim;

	fopen_s(&fp, in_load_filename.bp(), "rb");

	fscanf_s(fp, "%d %d %d %d\n", &zz_class_num, &zz_gaussian_num, &feat_dim, &zz_pcs_color_flag);
	printf("%d %d %d\n", zz_class_num, zz_gaussian_num, feat_dim);

	// initialization.
	zz_set_of_mean_vector.clear();			zz_set_of_mean_vector.resize(zz_class_num);		for (int i = 0; i<zz_class_num; i++){ zz_set_of_mean_vector[i].c_Create(zz_gaussian_num); }
	zz_set_of_covariance_matrix.clear();			zz_set_of_covariance_matrix.resize(zz_class_num);		for (int i = 0; i<zz_class_num; i++){ zz_set_of_covariance_matrix[i].c_Create(zz_gaussian_num); }
	zz_set_of_covariance_matrix_inverse.clear();	zz_set_of_covariance_matrix_inverse.resize(zz_class_num);		for (int i = 0; i<zz_class_num; i++){ zz_set_of_covariance_matrix_inverse[i].c_Create(zz_gaussian_num); }
	zz_set_of_determinant_of_covariance_matrix.clear();		zz_set_of_determinant_of_covariance_matrix.resize(zz_class_num);		for (int i = 0; i<zz_class_num; i++){ zz_set_of_determinant_of_covariance_matrix[i].c_Create(zz_gaussian_num); }
	zz_set_of_weights.clear();			zz_set_of_weights.resize(zz_class_num);				for (int i = 0; i<zz_class_num; i++){ zz_set_of_weights[i].c_Create(zz_gaussian_num, 0.0f); }

	// mean vectors.
	for (int i = 0; i<zz_class_num; i++){
		for (int k = 0; k<zz_gaussian_num; k++){
			tmp_vec = zz_set_of_mean_vector[i].gpe(k)->c_Create(feat_dim, 0.0f);
			for (int n = 0; n<feat_dim; n++){
				fscanf_s(fp, "%f ", &tmp_vec[n]);
				printf("%f ", tmp_vec[n]);
			}
		}
		printf("\n");
	}
	// covariance matrices.
	for (int i = 0; i<zz_class_num; i++){
		for (int k = 0; k<zz_gaussian_num; k++){
			tmp_vec = zz_set_of_covariance_matrix[i].gpe(k)->c_Create(feat_dim, feat_dim, 0.0f)[0];
			for (int n = 0; n<feat_dim; n++){
				fscanf_s(fp, "%f ", &tmp_vec[n*feat_dim + n]);
				printf("%f ", tmp_vec[n*feat_dim + n]);
			}
			// inverse covariance matrices.
			zz_set_of_covariance_matrix_inverse[i].gpe(k)->c_Create(feat_dim, feat_dim, 0.0f);
			aa_lef.ilu_Inverse_matrix_based_on_LUD(zz_set_of_covariance_matrix[i].gpe(k), zz_set_of_covariance_matrix_inverse[i].gpe(k));
			// determinant of covariance matrices.
			zz_set_of_determinant_of_covariance_matrix[i].vp()[k] = aa_lef.d_Determinant(zz_set_of_covariance_matrix[i].gpe(k));

		}

		printf("\n");
	}
	// weight vectors.
	for (int i = 0; i<zz_class_num; i++){
		tmp_vec = zz_set_of_weights[i].vp();
		for (int k = 0; k<zz_gaussian_num; k++){
			fscanf_s(fp, "%f ", &tmp_vec[k]);
			printf("%f ", tmp_vec[k]);
		}
		printf("\n");
	}

	fclose(fp);
}

//********************************************************************************
void LCKvYooji_Color_Segmentation::cc_Convert_Color_Rgb_to_PCS(CKvVectorFloat &in_rgb_vector,
	CKvVectorFloat &out_pcs_vector)
	//********************************************************************************
{
	float *p_in_rgb_vector, *p_out_pcs_vector;
	float color_xyz[3] = { 0.0, 0.0, 0.0 };

	CKvMatrix A, B, CIE;
	CKvVector Bx, xyz, logBx, rgb, pcs;

	p_in_rgb_vector = in_rgb_vector.vp();
	if (out_pcs_vector.vs() != 3)    p_out_pcs_vector = out_pcs_vector.c_Create(3, 0.0f);
	else                                           p_out_pcs_vector = out_pcs_vector.vp();

	A.c_Create(3, 3, 0.0f);	B.c_Create(3, 3, 0.0f);		CIE.c_Create(3, 3, 0.0f);
	logBx.c_Create(3, 0.0f);	rgb.c_Create(3, 0.0f);

	A.se_Set_Element(0, 0, 2.707439e1);			A.se_Set_Element(0, 1, -2.280783e1);		A.se_Set_Element(0, 2, -1.806681);
	A.se_Set_Element(1, 0, -5.646736);			A.se_Set_Element(1, 1, -7.722125);			A.se_Set_Element(1, 2, 1.286503e1);
	A.se_Set_Element(2, 0, -4.163133);			A.se_Set_Element(2, 1, -4.579428);			A.se_Set_Element(2, 2, -4.576049);

	B.se_Set_Element(0, 0, 9.465229e-1);		B.se_Set_Element(0, 1, 2.946927e-1);		B.se_Set_Element(0, 2, -1.313419e-1);
	B.se_Set_Element(1, 0, -1.179179e-1);		B.se_Set_Element(1, 1, 9.929960e-1);		B.se_Set_Element(1, 2, 7.371554e-3);
	B.se_Set_Element(2, 0, 9.230461e-2);		B.se_Set_Element(2, 1, -4.645794e-2);		B.se_Set_Element(2, 2, 9.946464e-1);

	CIE.se_Set_Element(0, 0, 0.49 / 0.17697);	CIE.se_Set_Element(0, 1, 0.31 / 0.17697);			CIE.se_Set_Element(0, 2, 0.2 / 0.17697);
	CIE.se_Set_Element(1, 0, 1.0);						CIE.se_Set_Element(1, 1, 0.81240 / 0.17697);	CIE.se_Set_Element(1, 2, 0.01063 / 0.17697);
	CIE.se_Set_Element(2, 0, 0.0);						CIE.se_Set_Element(2, 1, 0.01 / 0.17697);			CIE.se_Set_Element(2, 2, 0.99 / 0.17697);
	// 	CIE.se_Set_Element(0,0, 0.49);	CIE.se_Set_Element(0,1, 0.31);			CIE.se_Set_Element(0,2, 0.2);
	// 	CIE.se_Set_Element(1,0, 1.77);						CIE.se_Set_Element(1,1, 0.813);	CIE.se_Set_Element(1,2, 0.011);
	// 	CIE.se_Set_Element(2,0, 0.0);						CIE.se_Set_Element(2,1, 0.01);			CIE.se_Set_Element(2,2, 0.99);


	for (int i = 0; i<3; i++){ rgb.se_Set_Element(i, p_in_rgb_vector[i]); }

	aa_am.mmv_Multiply_Matrix_Vector(&CIE, &rgb, &xyz);
	aa_am.mmv_Multiply_Matrix_Vector(&B, &xyz, &Bx);

	for (int i = 0; i<3; i++){ logBx.se_Set_Element(i, log(Bx.ge_Get_Element(i))); }

	aa_am.mmv_Multiply_Matrix_Vector(&A, &logBx, &pcs);


	for (int i = 0; i<3; i++){ p_out_pcs_vector[i] = (float)pcs.ge_Get_Element(i); }

}



/// ////////////////////////////////////////////////////////////////////////////////////////////
/// ///////////////////////////////// Color conversion //////////////////////////////
/// ////////////////////////////////////////////////////////////////////////////////////////////
//*********************************************************************************************************
void LCKvYooji_Color_Segmentation::ccirl_Convert_Color_Image_RGB_to_LAB(CKvMatrixUcharRgb &in_img_RGB,
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

	for (j = 0; j<hh; j++){
		for (i = 0; i<ww; i++){
			idx = j*ww + i;
			ccvrl_Convert_Color_Value_RGB_to_LAB(p_in_R_image[idx], p_in_G_image[idx], p_in_B_image[idx],
				val_l_star, val_a_star, val_b_star);
			p_out_L_image[idx] = (float)val_l_star;
			p_out_A_image[idx] = (float)val_a_star;
			p_out_B_image[idx] = (float)val_b_star;
		}
	}


}

//*********************************************************************************************************
void LCKvYooji_Color_Segmentation::ccvrl_Convert_Color_Value_RGB_to_LAB(unsigned char in_sR, unsigned char in_sG, unsigned char in_sB,
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
	double kappa = 903.3;		//actual CIE standard

	double Xr = 0.950456;	//reference white
	double Yr = 1.0;		//reference white
	double Zr = 1.088754;	//reference white

	double xr = X / Xr;
	double yr = Y / Yr;
	double zr = Z / Zr;

	double fx, fy, fz;
	if (xr > epsilon)	fx = pow(xr, 1.0 / 3.0);
	else				fx = (kappa*xr + 16.0) / 116.0;
	if (yr > epsilon)	fy = pow(yr, 1.0 / 3.0);
	else				fy = (kappa*yr + 16.0) / 116.0;
	if (zr > epsilon)	fz = pow(zr, 1.0 / 3.0);
	else				fz = (kappa*zr + 16.0) / 116.0;

	out_l_star = 116.0*fy - 16.0;
	out_a_star = 500.0*(fx - fy);
	out_b_star = 200.0*(fy - fz);
}

//*********************************************************************************************************
void LCKvYooji_Color_Segmentation::ccvrx_Convert_Color_Value_RGB_to_XYZ(unsigned char in_sR, unsigned char in_sG, unsigned char in_sB,
	double &out_X, double &out_Y, double &out_Z)
	//*********************************************************************************************************
{
	double R = in_sR / 255.0;
	double G = in_sG / 255.0;
	double B = in_sB / 255.0;

	double r, g, b;

	if (R <= 0.04045)	r = R / 12.92;
	else				r = pow((R + 0.055) / 1.055, 2.4);
	if (G <= 0.04045)	g = G / 12.92;
	else				g = pow((G + 0.055) / 1.055, 2.4);
	if (B <= 0.04045)	b = B / 12.92;
	else				b = pow((B + 0.055) / 1.055, 2.4);

	out_X = r*0.4124564 + g*0.3575761 + b*0.1804375;
	out_Y = r*0.2126729 + g*0.7151522 + b*0.0721750;
	out_Z = r*0.0193339 + g*0.1191920 + b*0.9503041;
}

//*********************************************************************************************************
void LCKvYooji_Color_Segmentation::ccvrh_Convert_Color_Value_RGB_to_HSI(unsigned char in_sR, unsigned char in_sG, unsigned char in_sB,
	double &out_H, double &out_S, double &out_I)
	//*********************************************************************************************************
{
	unsigned char min_val;
	double R, G, B;

	min_val = (in_sR > in_sG) ? ((in_sG > in_sB) ? in_sB : in_sG) : ((in_sR > in_sB) ? in_sB : in_sR);

	R = (double)in_sR;		G = (double)in_sG;		B = (double)in_sB;

	out_I = (R + G + B) / 3.0;
	out_S = 1.0 - (double)min_val / out_I;
	out_H = acos(0.5*((R - G) - (R - B)) / sqrt(SQUARE(R - G) + (R - B)*(G - B)))*180.0 / PI;
	if (in_sB>in_sG)	out_H = 360.0 - out_H;

}

//*********************************************************************************************************
void LCKvYooji_Color_Segmentation::chiri_Compute_Hue_Image_from_RGB_image(CKvMatrixUcharRgb &in_rgb_img,
	float in_th_saturation,
	float in_th_intensity,
	CKvMatrixFloat &out_hue_img)
//*********************************************************************************************************
{
	unsigned char *p_in_rgb_img;
	float *p_img_hue;
	unsigned char r_val, g_val, b_val;
	double h_val, s_val, i_val;
	int ww, hh, cnt;

	p_in_rgb_img = in_rgb_img.mps(ww, hh)[0];
	p_img_hue = out_hue_img.c_Create(hh, ww, 0.0f)[0];

	cnt = 0;

	// compute Hue image from RGB image.
	for (int i = ww*hh - 1; i >= 0; i--){

		r_val = p_in_rgb_img[i];
		g_val = p_in_rgb_img[i + ww*hh];
		b_val = p_in_rgb_img[i + 2 * ww*hh];
		//sum=r+g+b;

		ccvrh_Convert_Color_Value_RGB_to_HSI(r_val, g_val, b_val, h_val, s_val, i_val);

		/// ///////////////////////////////////////////////////////
		// remove invalid hue values and specular lighted values.
		if (s_val>in_th_saturation || i_val <in_th_intensity)	p_img_hue[i] = (float)h_val;
		else{
			p_img_hue[i] = -1.0f;			cnt++;
			//printf("RGB( %d %d %d) HSI(%f %f %f)\n", r_val, g_val, b_val, h_val, s_val, i_val);
			//system("pause");
		}
		/// ///////////////////////////////////////////////////////

	}
}

//*********************************************************************************************************
void LCKvYooji_Color_Segmentation::chiri_Compute_Hue_Image_from_RGB_image(CKvMatrixUcharRgb &in_rgb_img, CKvMatrixFloat &out_hue_img)
//*********************************************************************************************************
{
	unsigned char *p_in_rgb_img;
	float *p_img_hue;
	unsigned char r_val, g_val, b_val;
	double h_val, s_val, i_val;
	int ww, hh, cnt;

	p_in_rgb_img = in_rgb_img.mps(ww, hh)[0];
	p_img_hue = out_hue_img.c_Create(hh, ww, 0.0f)[0];

	cnt = 0;

	// compute Hue image from RGB image.
	for (int i = ww*hh - 1; i >= 0; i--){

		r_val = p_in_rgb_img[i];
		g_val = p_in_rgb_img[i + ww*hh];
		b_val = p_in_rgb_img[i + 2 * ww*hh];
		//sum=r+g+b;

		ccvrh_Convert_Color_Value_RGB_to_HSI(r_val, g_val, b_val, h_val, s_val, i_val);

		/// ///////////////////////////////////////////////////////
		// remove invalid hue values and specular lighted values.
		if (s_val>0.1 || i_val <200.0f)	p_img_hue[i] = (float)h_val;
		else{
			p_img_hue[i] = -1.0f;			cnt++;
			//printf("RGB( %d %d %d) HSI(%f %f %f)\n", r_val, g_val, b_val, h_val, s_val, i_val);
			//system("pause");
		}
		/// ///////////////////////////////////////////////////////

	}
}

//********************************************************************************
void LCKvYooji_Color_Segmentation::in_Intenisty_Normalization(CKvMatrixUchar &in_image_gray,
	int in_min_intensity,
	int in_max_intensity,
	CKvMatrixUchar &out_image_gray)
	//********************************************************************************
{
	int ww, hh;

	CKvMatrixUchar t_img;
	unsigned char *p_in_image_gray, *p_out_image_gray;
	int min_inten, max_inten;

	in_image_gray.ms(ww, hh);
	if (ww <= 0 || hh <= 0){
		printf("[CKvHandSkeleton::in_Intenisty_Normalization] Input image is null.\n");
		system("pause");
	}

	if (&in_image_gray == &out_image_gray)	{ t_img.c_Create(hh, ww, (unsigned char)0);		p_in_image_gray = t_img.cp_Copy(&in_image_gray)[0];		p_out_image_gray = out_image_gray.vp(); }
	else{
		if (ww != out_image_gray.mw() || hh != out_image_gray.mh())	out_image_gray.c_Create(hh, ww, (unsigned char)0)[0];
		p_in_image_gray = in_image_gray.vp();		p_out_image_gray = out_image_gray.vp();
	}

	in_min_intensity = __max(0, in_min_intensity);	in_max_intensity = __min(255, in_max_intensity);
	max_inten = -100000;	min_inten = 100000;

	for (int y = 1; y<hh - 1; y++){
		for (int x = 1; x<ww - 1; x++){
			int tidx = y*ww + x;
			if (min_inten>(int)p_in_image_gray[tidx])		min_inten = (int)p_in_image_gray[tidx];
			if (max_inten<(int)p_in_image_gray[tidx])		max_inten = (int)p_in_image_gray[tidx];
		}
	}

	for (int i = 0; i<ww*hh; i++){
		p_out_image_gray[i] = (unsigned char)(__max(0, __min((float)((int)p_in_image_gray[i] - min_inten), 255))
			*(float)(in_max_intensity - in_min_intensity) / (float)(max_inten - min_inten) + in_min_intensity);
	}


	// 	CKvScreen sc;
	// 	sc.s_d_Display(&out_image_gray);
	// 	Kv_Pause(10);
	// 	printf("Good?\n");
	// 	system("pause");

}

//********************************************************************************
void LCKvYooji_Color_Segmentation::biac_Bilinear_Interpolation_using_Averaging_Color(float in_real_x,
	float in_real_y,
	CKvMatrixUcharRgb *in_image,
	CKvRgb *out_interpolated_color)
	//********************************************************************************
{
	double rX, rY;
	int ww, hh;
	int pX[2], pY[2];

	double val[4], d[4];

	unsigned char *p_in_image;

	rX = in_real_x;
	rY = in_real_y;

	pX[0] = (int)(rX);		pX[1] = (int)(rX + 1.0);
	pY[0] = (int)(rY);		pY[1] = (int)(rY + 1.0);

	d[0] = rX - pX[0];		d[1] = pX[1] - rX;
	d[2] = rY - pY[0];		d[3] = pY[1] - rY;

	p_in_image = in_image->mps_Matrix_Pointer_and_Width_Height(ww, hh)[0];

	//	printf("%d %d X[1] : %d  Y[1] : %d\n", in_image.cols, in_image.rows, pX[1], pY[1]);
	if (d[0] == 0.0){
		d[0] = 0.5;			d[1] = 0.5;
		pX[1] = pX[0];
	}
	if (d[2] == 0.0){
		d[2] = 0.5;			d[3] = 0.5;
		pY[1] = pY[0];
	}
	//printf("%d %d X[1] : %d  Y[1] : %d\n", in_image.cols, in_image.rows, pX[1], pY[1]);

	int color_channel = 3;
	double color_unit_vector[16];// = new double[4*color_channel + color_channel];			// 4 points color channels and interpolated color vectors.
	double intensity[5];				for (int i = 0; i<5; i++){ intensity[i] = .0; }
	double vector_mag[4];		for (int i = 0; i<4; i++){ vector_mag[i] = .0; }

	for (int i = 0; i<color_channel; i++){
		color_unit_vector[i * 4] = (double)p_in_image[pY[0] * ww + pX[0] + i*ww*hh];
		color_unit_vector[i * 4 + 1] = (double)p_in_image[pY[0] * ww + pX[1] + i*ww*hh];
		color_unit_vector[i * 4 + 2] = (double)p_in_image[pY[1] * ww + pX[0] + i*ww*hh];
		color_unit_vector[i * 4 + 3] = (double)p_in_image[pY[1] * ww + pX[1] + i*ww*hh];
	}

	// interpolate unit color vector.
	for (int j = 0; j<color_channel; j++){
		for (int i = 0; i<4; i++)
			val[i] = color_unit_vector[j * 4 + i];

		color_unit_vector[4 * color_channel + j] = d[3] * (d[0] * val[1] + d[1] * val[0]) + d[2] * (d[0] * val[3] + d[1] * val[2]);
	}

	// compute interpolated RGB color.
	out_interpolated_color->r = (UCHAR)color_unit_vector[4 * color_channel];
	out_interpolated_color->g = (UCHAR)color_unit_vector[4 * color_channel + 1];
	out_interpolated_color->b = (UCHAR)color_unit_vector[4 * color_channel + 2];
}
