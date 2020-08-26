
// VCL_Object_Scanner_SDIDoc.cpp : CVCL_Object_Scanner_SDIDoc 클래스의 구현
//

#include "stdafx.h"
// SHARED_HANDLERS는 미리 보기, 축소판 그림 및 검색 필터 처리기를 구현하는 ATL 프로젝트에서 정의할 수 있으며
// 해당 프로젝트와 문서 코드를 공유하도록 해 줍니다.
#ifndef SHARED_HANDLERS
#include "VCL_Volume_Scanner.h"
#endif

#include "VCL_Volume_Scanner_Doc.h"

#include <propkey.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// CVCL_Object_Scanner_SDIDoc

IMPLEMENT_DYNCREATE(CVCL_Volume_Scanner_Doc, CDocument)

BEGIN_MESSAGE_MAP(CVCL_Volume_Scanner_Doc, CDocument)
	ON_COMMAND(ID_INPUT_IMAGE, &CVCL_Volume_Scanner_Doc::OnLoadTSDFasVCL)
ON_COMMAND(ID_INPUT_IMAGESEQUENCE, &CVCL_Volume_Scanner_Doc::OnInputImageSequenceVCL)
ON_COMMAND(ID_INPUT_VIDEOSTREAM, &CVCL_Volume_Scanner_Doc::OnInputVideoStreamKinectV1)
ON_COMMAND(ID_SAVE_SAVETSDF,&CVCL_Volume_Scanner_Doc::OnSaveTSDFasBin)
ON_COMMAND(ID_SAVE_SAVE,&CVCL_Volume_Scanner_Doc::OnSaveObjectVCL)
ON_COMMAND(ID_SAVE_SAVEINPUTSEQUENCE,&CVCL_Volume_Scanner_Doc::OnSaveInputSequenceAsVCL)
ON_COMMAND(ID_SAVE_SAVEINPUTSEQUENCE32791,&CVCL_Volume_Scanner_Doc::OnSaveInputSequenceAsCXD)
ON_COMMAND(ID_CONVERT_VCLTOCXD,&CVCL_Volume_Scanner_Doc::OnConvertVcLtoCxD)
ON_COMMAND(ID_IMAGESEQUENCE_NEW,&CVCL_Volume_Scanner_Doc::OnInputImageSequenceCxD)
ON_COMMAND(ID_FILE_EXIT,&CVCL_Volume_Scanner_Doc::OnFileExit)
ON_COMMAND(ID_SAVE_SAVEESTIMATEDTRAJACTORY,&CVCL_Volume_Scanner_Doc::OnSaveEstimatedTrajactoryQuarternion)
ON_COMMAND(ID_SAVE_SAVESILHOUETTESEQUENCE,&CVCL_Volume_Scanner_Doc::OnSaveSavesilhouettesequence)
ON_COMMAND(ID_SAVE_SAVEESTIMATEDTRAJACTORY32813,&CVCL_Volume_Scanner_Doc::OnSaveSaveestimatedtrajactory4x4)
ON_COMMAND(ID_CONVERT_TRANSFORMTOQUARTERNION,&CVCL_Volume_Scanner_Doc::OnConvertTransformtoquarternion)
END_MESSAGE_MAP()


// CVCL_Object_Scanner_SDIDoc 생성/소멸

CVCL_Volume_Scanner_Doc::CVCL_Volume_Scanner_Doc()
{
	// TODO: 여기에 일회성 생성 코드를 추가합니다.
// 	zz_kinect = NULL;
// 	zz_input_type = -1;

}

CVCL_Volume_Scanner_Doc::~CVCL_Volume_Scanner_Doc()
{
//	if(zz_kinect){	delete[] zz_kinect;	zz_kinect = NULL; }
}

BOOL CVCL_Volume_Scanner_Doc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	// TODO: 여기에 재초기화 코드를 추가합니다.
	// SDI 문서는 이 문서를 다시 사용합니다.

	return TRUE;
}




// CVCL_Object_Scanner_SDIDoc serialization

void CVCL_Volume_Scanner_Doc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO: 여기에 저장 코드를 추가합니다.
	}
	else
	{
		// TODO: 여기에 로딩 코드를 추가합니다.
	}
}

#ifdef SHARED_HANDLERS

// 축소판 그림을 지원합니다.
void CVCL_Volume_Scanner_Doc::OnDrawThumbnail(CDC& dc, LPRECT lprcBounds)
{
	// 문서의 데이터를 그리려면 이 코드를 수정하십시오.
	dc.FillSolidRect(lprcBounds, RGB(255, 255, 255));

	CString strText = _T("TODO: implement thumbnail drawing here");
	LOGFONT lf;

	CFont* pDefaultGUIFont = CFont::FromHandle((HFONT) GetStockObject(DEFAULT_GUI_FONT));
	pDefaultGUIFont->GetLogFont(&lf);
	lf.lfHeight = 36;

	CFont fontDraw;
	fontDraw.CreateFontIndirect(&lf);

	CFont* pOldFont = dc.SelectObject(&fontDraw);
	dc.DrawText(strText, lprcBounds, DT_CENTER | DT_WORDBREAK);
	dc.SelectObject(pOldFont);
}

// 검색 처리기를 지원합니다.
void CVCL_Volume_Scanner_Doc::InitializeSearchContent()
{
	CString strSearchContent;
	// 문서의 데이터에서 검색 콘텐츠를 설정합니다.
	// 콘텐츠 부분은 ";"로 구분되어야 합니다.

	// 예: strSearchContent = _T("point;rectangle;circle;ole object;");
	SetSearchContent(strSearchContent);
}

void CVCL_Volume_Scanner_Doc::SetSearchContent(const CString& value)
{
	if (value.IsEmpty())
	{
		RemoveChunk(PKEY_Search_Contents.fmtid, PKEY_Search_Contents.pid);
	}
	else
	{
		CMFCFilterChunkValueImpl *pChunk = NULL;
		ATLTRY(pChunk = new CMFCFilterChunkValueImpl);
		if (pChunk != NULL)
		{
			pChunk->SetTextValue(PKEY_Search_Contents, value, CHUNK_TEXT);
			SetChunkValue(pChunk);
		}
	}
}

#endif // SHARED_HANDLERS

// CVCL_Object_Scanner_SDIDoc 진단

#ifdef _DEBUG
void CVCL_Volume_Scanner_Doc::AssertValid() const
{
	CDocument::AssertValid();
}

void CVCL_Volume_Scanner_Doc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG


// CVCL_Object_Scanner_SDIDoc 명령


//void CVCL_Object_Scanner_SDIDoc::OnInputFile()
//{
//	// TODO: 여기에 명령 처리기 코드를 추가합니다.
//	if(!zz_io.gof_Get_Open_Filename("Load BMP file.", zz_fn)){
//		zz_input_type = -1;
//		return;
//	}
//	zz_input_type = 0;
//}

//////////////////////////////////////////////////////////////////////////
// for loading input images.
//////////////////////////////////////////////////////////////////////////
void CVCL_Volume_Scanner_Doc::OnInputImageSequenceVCL()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	zz_capture.i_Initialize(KV_CAPTURE_FROM_FILES);
	zz_input_type = KV_CAPTURE_FROM_FILES;
}


void CVCL_Volume_Scanner_Doc::OnInputImageSequenceCxD()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	zz_capture.i_Initialize(KV_CAPTURE_FROM_FILES_CXD);
	zz_input_type = KV_CAPTURE_FROM_FILES_CXD;
}


void CVCL_Volume_Scanner_Doc::OnInputVideoStreamKinectV1()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	zz_capture.i_Initialize(KV_CAPTURE_FROM_KINECTV1);
	zz_input_type = KV_CAPTURE_FROM_KINECTV1;
}

//////////////////////////////////////////////////////////////////////////
// for saving results.
//////////////////////////////////////////////////////////////////////////
void CVCL_Volume_Scanner_Doc::OnSaveObjectVCL()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	LCKvIO io;
	CKvString dn;
	io.gfn_Get_Folder_Name("Open folder for saving TSDF cube data.",dn);

	// 현재 내부에 host cube 를 생성하여 복사함.
	// 이를 프로젝트 내 zz_cube 로 대상을 바꿀순 없을까.
	zz_3dos.itsdfcd_Import_TSDF_Cube_from_Device();
	zz_3dos.stsdfc_Save_TSDF_Cube(dn);

	Kv_Printf("Scanned object was successfully saved!");
	
}


void CVCL_Volume_Scanner_Doc::OnSaveInputSequenceAsVCL()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	CKvString dn_texture,dn_depth;

	dn_texture = zz_capture.dn_Directory_Name_of_Texture();
	dn_depth = zz_capture.dn_Directory_Name_of_Depth();

	zz_3dos.sis_Save_Input_Sequence(dn_depth, dn_texture);
}


void CVCL_Volume_Scanner_Doc::OnSaveInputSequenceAsCXD()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
}

//////////////////////////////////////////////////////////////////////////
// for mesh generation.
//////////////////////////////////////////////////////////////////////////
void CVCL_Volume_Scanner_Doc::OnLoadTSDFasVCL()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.	
	LCKvIO io;	CKvString dn;

	io.gfn_Get_Folder_Name("Open folder name for loading TSDF cube.",dn);
	zz_capture.ltsdfc_Load_TSDF_Cube(dn,zz_cube);

	std::cout << "Loading TSDF voxel grid values from disk (*.vcl) was complete." << std::endl;
}

void CVCL_Volume_Scanner_Doc::OnSaveTSDFasBin()
{
	// Save TSDF voxel grid and its parameters to disk as binary file (float array)
	float *voxel_grid_TSDF = zz_cube.pvd_Pointer_of_Volume_Depth()->vp();

	float voxel_size = zz_cube.svm_Size_of_Voxel_in_Meter();
	float trunc_margin = voxel_size * 5;

	CKvPoint3Df origin;
	int voxel_grid_dim_x,voxel_grid_dim_y,voxel_grid_dim_z;
	float voxel_grid_origin_x,voxel_grid_origin_y,voxel_grid_origin_z;

	// get parameters.
	zz_cube.ts(voxel_grid_dim_x,voxel_grid_dim_y,voxel_grid_dim_z);
	zz_cube.goiw_Get_Origin_In_World(origin);

	voxel_grid_origin_x = origin.x; voxel_grid_origin_y = origin.y; voxel_grid_origin_z = origin.z;

	std::cout << "Saving TSDF voxel grid values to disk (tsdf.bin)... ";
	std::string voxel_grid_saveto_path = "tsdf.bin";
	std::ofstream outFile(voxel_grid_saveto_path,std::ios::binary | std::ios::out);
	float voxel_grid_dim_xf = (float)voxel_grid_dim_x;
	float voxel_grid_dim_yf = (float)voxel_grid_dim_y;
	float voxel_grid_dim_zf = (float)voxel_grid_dim_z;
	outFile.write((char*)&voxel_grid_dim_xf,sizeof(float));
	outFile.write((char*)&voxel_grid_dim_yf,sizeof(float));
	outFile.write((char*)&voxel_grid_dim_zf,sizeof(float));
	outFile.write((char*)&voxel_grid_origin_x,sizeof(float));
	outFile.write((char*)&voxel_grid_origin_y,sizeof(float));
	outFile.write((char*)&voxel_grid_origin_z,sizeof(float));
	outFile.write((char*)&voxel_size,sizeof(float));
	outFile.write((char*)&trunc_margin,sizeof(float));
	for(int i = 0; i < voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z; ++i)
	  outFile.write((char*)&voxel_grid_TSDF[i],sizeof(float));
	outFile.close();

	std::cout << " complete." << std::endl;
}

//////////////////////////////////////////////////////////////////////////
// for cxd file generation.
//////////////////////////////////////////////////////////////////////////
void CVCL_Volume_Scanner_Doc::OnConvertVcLtoCxD()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	LCKvUtility_for_Import util_im;
	LCKvIO io;

	CKvYooji_MatrixRgbD mat_rgbd;
	CKvData_Color_x_Depth mat_cxd;

	CKvMatrixUcharRgb img_rgb;
	CKvMatrixUshort map_depth;
	CKvMatrix mat_K_rgb, mat_K_d;
	CKvPoint3D tvec_d_to_rgb;
	CKvVector rvec_d_to_rgb;

	CKvString dn, fn;

	int idx_frame = 0;

	zz_capture.i_Initialize(KV_CAPTURE_FROM_FILES);
	zz_capture.lcis_Load_Camera_Information_System(&mat_rgbd);

	io.gdn_Get_Directory_Name("Open folder for saving cxd files.", dn);
	
	while(1){
		if(zz_capture.lrdi_Load_Rgb_and_Depth_Images(
					idx_frame,
					mat_rgbd.p_image_rgb(),
					mat_rgbd.p_map_depth_raw()))
		{
			// color image.
			img_rgb.cp_Copy(mat_rgbd.p_image_rgb());
			// depth map. (ushort)
			util_im.mfus_Matrix_Float_to_Ushort(mat_rgbd.p_map_depth_raw(), 1000.0f, &map_depth);
			// intrinsics of rgb camera.
			util_im.mfd_Matrix_Float_to_Double(mat_rgbd.p_intrinsics_RGB()->mp(), 1.0f, &mat_K_rgb);
			// intrinsics of depth camera.
			util_im.mfd_Matrix_Float_to_Double(mat_rgbd.p_intrinsics_depth()->mp(), 1.0f, &mat_K_d);
			// translation vector from depth to rgb camera.

			// rotation vector from depth to rgb camera.

			// set cxd file.
			mat_cxd.im_Import(
				img_rgb,
				map_depth,
				&mat_K_rgb,&mat_K_d, 
				NULL,//&tvec_d_to_rgb,
				NULL);//&rvec_d_to_rgb);

			// save cxd file.
			fn.fm_Format("%s\\%04d.cxd", dn.bp(), idx_frame);
			mat_cxd.sd_Save_Data(fn);

// 			// for debugging.
// 			float sum_error = 0.0f;
// 
// 			for(int i=0; i<map_depth.vs(); i++){
// 				sum_error += abs(mat_rgbd.p_map_depth_raw()->vp()[i] - 0.001f*float(mat_cxd.pd()->vp()[i]));
// 			}
//			printf("sum_error: %f\n", sum_error);


			idx_frame++;
			printf("idx_frame: %d\n", idx_frame);
		}
		else break;
	}
}

void CVCL_Volume_Scanner_Doc::OnFileExit()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	zz_3dos.rs_Release_Scanner();
	exit(0);
}


void CVCL_Volume_Scanner_Doc::OnSaveEstimatedTrajactoryQuarternion()
{
	CKvString fn_stamp, fn;
	LCKvIO io; 
	
	io.gof_Get_Open_Filename("Open timestamp file name.",fn_stamp);
	io.gsf_Get_Save_Filename("Open save file name.",fn);
	
	std::ifstream f_ts(fn_stamp.bp());	if(f_ts.fail())	return ;
	std::ofstream f_sv(fn.bp());		if(f_sv.fail())	return ;

	CKvYooji_Extrinsics t_ext;
	CKvMatrixFloat *mp_R, *mp_t;
	CKvVectorFloat vec_q;	vec_q.c_Create(4);
	CKvPoint3Df center;

	double timestamp;
	char path[100];
	//std::string path;

	//vector<CKvYooji_Extrinsics*> *p_trace_est = zz_3dos.p_trace_gt();
	vector<CKvYooji_Extrinsics*> *p_trace_est = zz_3dos.p_trace_estimated();
	int size = p_trace_est->size();

	Eigen::Matrix4f mat_exp, mat_xsi;
	
	for(int i=0; i<size; i++){

		//printf("size: %d\n", size);

		// get timestamp.
		f_ts >> timestamp;
		f_ts >> path;

		printf("ts: %lf\n", timestamp);

		//t_ext.set_from_transform((*p_trace_est)[i]->mp_transform_inv());
		t_ext.copy((*p_trace_est)[i]);

		// cam center.		
		t_ext.get_cam_center(center);
		// translation.
		//mp_t = t_ext.mp_translation();
		//center.x = mp_t->vp()[0]; center.y = mp_t->vp()[1]; center.z = mp_t->vp()[2];

		// cam rotation.
		mp_R = t_ext.mp_rotation();

		//////////////////////////////////////////////////////////////////////////
		// convert rotation matrix to unit quaternion using Eigen.
		Eigen::Matrix3f rmat;
		for(int i=0; i<3; i++) for(int j=0; j<3; j++) rmat(i, j) = mp_R->vp()[i*3 + j];
		
		Eigen::Quaternionf q(rmat);	q.normalize();
		//t_ext.convert_rotation_mat_to_quaternion(*mp_R,vec_q);
		//////////////////////////////////////////////////////////////////////////
		//vec_q *= -1.0f;
		
		printf("center: %f %f %f\n", center.x, center.y, center.z);
		printf("vec_q: %f %f %f %f\n", q.x(), q.y(), q.z(), q.w());

		//////////////////////////////////////////////////////////////////////////
		// Using Eigen.
		//////////////////////////////////////////////////////////////////////////
		//Eigen::Vector4f q_eig;
		//float *p_mat_ext = t_ext.mp_transform()->vp();
	
		//mat_exp << 
		//	p_mat_ext[0],p_mat_ext[1],p_mat_ext[2],p_mat_ext[3],
		//	p_mat_ext[4],p_mat_ext[5],p_mat_ext[6],p_mat_ext[7],
		//	p_mat_ext[8],p_mat_ext[9],p_mat_ext[10],p_mat_ext[11],
		//	p_mat_ext[12],p_mat_ext[13],p_mat_ext[14],p_mat_ext[15];

		//mat_xsi = mat_exp.log();

		//float mag_q = sqrt(SQUARE(mat_xsi(1,2))+SQUARE(mat_xsi(0,2))+SQUARE(mat_xsi(0,1)));
		//q_eig << -mat_xsi(1,2),mat_xsi(0,2),-mat_xsi(0,1), mag_q;
		//q_eig.normalize();

		//printf("[#%d] %f %f %f | %f %f %f %f\n", i, center.x,center.y,center.z,
		//	q.x(),q.y(),q.z(),q.w());
		//printf("[#%d] %f %f %f | %f %f %f %f\n",i,mat_xsi(0,3),mat_xsi(1,3),mat_xsi(2,3),
		//	q_eig(0), q_eig(1), q_eig(2), q_eig(3));

		//cout << mat_exp << endl;
		////cout << mat_exp.log() << endl;

		////if(!Kv_Printf("Exp-Log")) exit(0);


		// timestamp.
		f_sv.precision(14);	
		f_sv << timestamp << " ";
		// camera center.
		f_sv.precision(5);	
		f_sv << center.x << " " << center.y << " " << center.z << " ";
		// rotation quaternion.
		f_sv.precision(4);
		f_sv << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

// 		f_sv.precision(14);
// 		f_sv << timestamp << " ";
// 		// camera center.
// 		f_sv.precision(5);
// 		f_sv << mat_xsi(0,3) << " " << mat_xsi(2,3) << " " << mat_xsi(2,3) << " ";
// 		// rotation quaternion.
// 		f_sv.precision(4);
// 		f_sv << " " << q_eig(0) << " " << q_eig(1) << " " << q_eig(2) << " " << q_eig(3) << endl;

	}
	f_sv.close();


}

void CVCL_Volume_Scanner_Doc::writeMatToTextFile(Mat& m, const char* filename)
{
	ofstream fout(filename);
	int data_type;

	if (!fout)
	{
		cout << "File Not Opened" << endl;  return;
	}

	// Gets matrix data type.
	/** @brief Returns the depth of a matrix element.

	The method returns the identifier of the matrix element depth (the type of each individual channel).
	For example, for a 16-bit signed element array, the method returns CV_16S . A complete list of
	matrix types contains the following values:
	-   CV_8U - 8-bit unsigned integers ( 0..255 )
	-   CV_8S - 8-bit signed integers ( -128..127 )
	-   CV_16U - 16-bit unsigned integers ( 0..65535 )
	-   CV_16S - 16-bit signed integers ( -32768..32767 )
	-   CV_32S - 32-bit signed integers ( -2147483648..2147483647 )
	-   CV_32F - 32-bit floating-point numbers ( -FLT_MAX..FLT_MAX, INF, NAN )
	-   CV_64F - 64-bit floating-point numbers ( -DBL_MAX..DBL_MAX, INF, NAN )
	*/
	data_type = m.type();

	// Prints matrix data type.
	fout << data_type << endl;
	// Prints matrix size.
	fout << m.cols << " " << m.rows << endl;
	fout << endl;
	// Prints matrix data.
	for (int i = 0; i < m.rows; i++)
	{
		for (int j = 0; j < m.cols; j++)
		{

			if (data_type == CV_8U)			fout << m.at<uchar>(i, j) << "\t";
			else if (data_type == CV_8S)	fout << m.at<char>(i, j) << "\t";
			else if (data_type == CV_16U)	fout << m.at<ushort>(i, j) << "\t";
			else if (data_type == CV_16S)	fout << m.at<short>(i, j) << "\t";
			else if (data_type == CV_32S)	fout << m.at<int>(i, j) << "\t";
			else if (data_type == CV_32F)	fout << m.at<float>(i, j) << "\t";
			else if (data_type == CV_64F)	fout << m.at<double>(i, j) << "\t";
		}
		fout << endl;
	}

	fout.close();
}

void CVCL_Volume_Scanner_Doc::readTextFileToMat(Mat& m, const char* filename)
{
	ifstream fin(filename);
	int data_type, rows, cols;

	if (!fin)
	{
		cout << "File Not Opened" << endl;  return;
	}

	// Gets matrix data type.
	/** @brief Returns the depth of a matrix element.

	//The method returns the identifier of the matrix element depth (the type of each individual channel).
	//For example, for a 16-bit signed element array, the method returns CV_16S . A complete list of
	//matrix types contains the following values:
	//-   CV_8U - 8-bit unsigned integers ( 0..255 )
	//-   CV_8S - 8-bit signed integers ( -128..127 )
	//-   CV_16U - 16-bit unsigned integers ( 0..65535 )
	//-   CV_16S - 16-bit signed integers ( -32768..32767 )
	//-   CV_32S - 32-bit signed integers ( -2147483648..2147483647 )
	//-   CV_32F - 32-bit floating-point numbers ( -FLT_MAX..FLT_MAX, INF, NAN )
	//-   CV_64F - 64-bit floating-point numbers ( -DBL_MAX..DBL_MAX, INF, NAN )
	//*/
	fin >> data_type;
	fin >> cols;	fin >> rows;
	// Creates matrix.
	m.create(rows, cols, data_type);

	// Reads matrix data.
	for (int i = 0; i < m.rows; i++)
	{
		for (int j = 0; j < m.cols; j++)
		{
			if (data_type == CV_8U)			fin >> m.at<uchar>(i, j);
			else if (data_type == CV_8S)	fin >> m.at<char>(i, j);
			else if (data_type == CV_16U)	fin >> m.at<ushort>(i, j);
			else if (data_type == CV_16S)	fin >> m.at<short>(i, j);
			else if (data_type == CV_32S)	fin >> m.at<int>(i, j);
			else if (data_type == CV_32F)	fin >> m.at<float>(i, j);
			else if (data_type == CV_64F)	fin >> m.at<double>(i, j);
		}
	}

	fin.close();
}

void CVCL_Volume_Scanner_Doc::writeMatSetToTextFile(vector<Mat>& m, const char* filename)
{
	ofstream fout(filename);
	int data_type;

	if (!fout)
	{
		cout << "File Not Opened" << endl;  return;
	}

	// Gets matrix data type.
	/** @brief Returns the depth of a matrix element.

	The method returns the identifier of the matrix element depth (the type of each individual channel).
	For example, for a 16-bit signed element array, the method returns CV_16S . A complete list of
	matrix types contains the following values:
	-   CV_8U - 8-bit unsigned integers ( 0..255 )
	-   CV_8S - 8-bit signed integers ( -128..127 )
	-   CV_16U - 16-bit unsigned integers ( 0..65535 )
	-   CV_16S - 16-bit signed integers ( -32768..32767 )
	-   CV_32S - 32-bit signed integers ( -2147483648..2147483647 )
	-   CV_32F - 32-bit floating-point numbers ( -FLT_MAX..FLT_MAX, INF, NAN )
	-   CV_64F - 64-bit floating-point numbers ( -DBL_MAX..DBL_MAX, INF, NAN )
	*/
	data_type = m[0].type();

	// Prints matrix data.
	for(int k=0; k<m.size(); k++){

		// file number.
		fout << k << endl;

		for(int i = 0; i < m[i].rows; i++)
		{
			for(int j = 0; j < m[i].cols; j++)
			{

				if(data_type == CV_8U)			fout << m[k].at<uchar>(i,j) << "\t";
				else if(data_type == CV_8S)		fout << m[k].at<char>(i,j) << "\t";
				else if(data_type == CV_16U)	fout << m[k].at<ushort>(i,j) << "\t";
				else if(data_type == CV_16S)	fout << m[k].at<short>(i,j) << "\t";
				else if(data_type == CV_32S)	fout << m[k].at<int>(i,j) << "\t";
				else if(data_type == CV_32F)	fout << m[k].at<float>(i,j) << "\t";
				else if(data_type == CV_64F)	fout << m[k].at<double>(i,j) << "\t";
			}
			fout << endl;
		}

	}	

	fout.close();
}

void CVCL_Volume_Scanner_Doc::OnSaveSavesilhouettesequence()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	CKvString dn_sil;

	dn_sil = zz_capture.dn_Directory_Name_of_Silhouette();
	zz_3dos.sss_Save_Silhouette_Sequence(dn_sil);
}


void CVCL_Volume_Scanner_Doc::OnSaveSaveestimatedtrajactory4x4()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	CKvString fn;
	LCKvIO io;

	io.gsf_Get_Save_Filename("Open save file name.",fn);

	std::ofstream f_sv(fn.bp());		if(f_sv.fail())	return ;

	CKvYooji_Extrinsics t_ext;

	double timestamp;
	char path[100];
	//std::string path;

	//vector<CKvYooji_Extrinsics*> *p_trace_est = zz_3dos.p_trace_gt();
	vector<CKvYooji_Extrinsics*> *p_trace_est = zz_3dos.p_trace_estimated();
	int size = p_trace_est->size();

	Eigen::Matrix4f mat_exp,mat_xsi;

	for(int i=0; i<size; i++){

		//printf("size: %d\n", size);

		// get timestamp.

		//t_ext.set_from_transform((*p_trace_est)[i]->mp_transform_inv());
		t_ext.copy((*p_trace_est)[i]);

				
		// timestamp.
		f_sv << i << endl;
		// transformation matrix.
		float **pp_T = t_ext.mp_transform()->mp();
		for(int m=0; m<4; m++){
			for(int k=0; k<4; k++){
				f_sv << pp_T[m][k] << " ";
			}
			f_sv << endl;
		}			

		// 		f_sv.precision(14);
		// 		f_sv << timestamp << " ";
		// 		// camera center.
		// 		f_sv.precision(5);
		// 		f_sv << mat_xsi(0,3) << " " << mat_xsi(2,3) << " " << mat_xsi(2,3) << " ";
		// 		// rotation quaternion.
		// 		f_sv.precision(4);
		// 		f_sv << " " << q_eig(0) << " " << q_eig(1) << " " << q_eig(2) << " " << q_eig(3) << endl;

	}
	f_sv.close();

}


void CVCL_Volume_Scanner_Doc::OnConvertTransformtoquarternion()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	LCKvIO zz_io;
	CKvString fn_tmat;
	zz_io.gof_Get_Open_Filename("Open file of sequence of 4x4 transformation matrices.",fn_tmat);
	zz_capture.ltt_Load_Trace_from_Txt_for_SDF2SDF(fn_tmat,
		zz_3dos.p_trace_gt());

	CKvString fn_stamp,fn;
	LCKvIO io;

	io.gsf_Get_Save_Filename("Open save file of sequence of quarternions.",fn);

	std::ofstream f_sv(fn.bp());		if(f_sv.fail())	return ;

	CKvYooji_Extrinsics t_ext;
	CKvMatrixFloat *mp_R,*mp_t;
	CKvVectorFloat vec_q;	vec_q.c_Create(4);
	CKvPoint3Df center;

	double timestamp;
	char path[100];
	//std::string path;

	vector<CKvYooji_Extrinsics*> *p_trace_est = zz_3dos.p_trace_gt();
	int size = p_trace_est->size();

	Eigen::Matrix4f mat_exp,mat_xsi;
	
	for(int k=0; k<size; k++){


		//t_ext.set_from_transform((*p_trace_est)[i]->mp_transform_inv());
		t_ext.copy((*p_trace_est)[k]);

		// cam center.		
		t_ext.get_cam_center(center);
		// translation.
		//mp_t = t_ext.mp_translation();
		//center.x = mp_t->vp()[0]; center.y = mp_t->vp()[1]; center.z = mp_t->vp()[2];

		// cam rotation.
		mp_R = t_ext.mp_rotation();

		//////////////////////////////////////////////////////////////////////////
		// convert rotation matrix to unit quaternion using Eigen.
		Eigen::Matrix3f rmat;
		for(int i=0; i<3; i++) for(int j=0; j<3; j++) rmat(i,j) = mp_R->vp()[i*3 + j];

		Eigen::Quaternionf q(rmat);	q.normalize();
		//t_ext.convert_rotation_mat_to_quaternion(*mp_R,vec_q);
		//////////////////////////////////////////////////////////////////////////
		//vec_q *= -1.0f;

		printf("center: %f %f %f\n",center.x,center.y,center.z);
		printf("vec_q: %f %f %f %f\n",q.x(),q.y(),q.z(),q.w());


		// timestamp.
		f_sv << k << " ";
		// camera center.
		f_sv.precision(5);
		f_sv << center.x << " " << center.y << " " << center.z << " ";
		// rotation quaternion.
		f_sv.precision(4);
		f_sv << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

		// 		f_sv.precision(14);
		// 		f_sv << timestamp << " ";
		// 		// camera center.
		// 		f_sv.precision(5);
		// 		f_sv << mat_xsi(0,3) << " " << mat_xsi(2,3) << " " << mat_xsi(2,3) << " ";
		// 		// rotation quaternion.
		// 		f_sv.precision(4);
		// 		f_sv << " " << q_eig(0) << " " << q_eig(1) << " " << q_eig(2) << " " << q_eig(3) << endl;

	}
	f_sv.close();

	printf("size: %d\n", size);

}
