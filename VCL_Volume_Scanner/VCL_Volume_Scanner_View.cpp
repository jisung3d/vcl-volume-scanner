
// VCL_Volume_Renderer_View.cpp : CVCL_Volume_Renderer_View 클래스의 구현
//

#include "stdafx.h"
// SHARED_HANDLERS는 미리 보기, 축소판 그림 및 검색 필터 처리기를 구현하는 ATL 프로젝트에서 정의할 수 있으며
// 해당 프로젝트와 문서 코드를 공유하도록 해 줍니다.
#ifndef SHARED_HANDLERS
#include "VCL_Volume_Scanner.h"
#endif

#include "VCL_Volume_Scanner_Doc.h"
#include "VCL_Volume_Scanner_View.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CVCL_Object_Scanner_SDIView

IMPLEMENT_DYNCREATE(CVCL_Volume_Scanner_View, CView)

BEGIN_MESSAGE_MAP(CVCL_Volume_Scanner_View, CView)
	// 표준 인쇄 명령입니다.
	ON_COMMAND(ID_FILE_PRINT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_DIRECT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_PREVIEW, &CView::OnFilePrintPreview)
	ON_COMMAND(ID_BUTTON_PLAY, &CVCL_Volume_Scanner_View::OnButtonPlay)
	ON_COMMAND(ID_BUTTON_PAUSE, &CVCL_Volume_Scanner_View::OnButtonPause)
	ON_COMMAND(ID_BUTTON_STOP, &CVCL_Volume_Scanner_View::OnButtonStop)
	ON_COMMAND(ID_BUTTON_3D_SCANNING, &CVCL_Volume_Scanner_View::OnButton3dScanning)
	ON_WM_ERASEBKGND()
	ON_COMMAND(ID_TEST_OBJECTSEGMENTATION, &CVCL_Volume_Scanner_View::OnRenderTSDFwithPCL)
	ON_COMMAND(ID_TEST_LOADTSDFCUBE, &CVCL_Volume_Scanner_View::OnRenderScene)
	ON_COMMAND(ID_TEST_GRAPHCUT, &CVCL_Volume_Scanner_View::OnRenderObject)
	ON_COMMAND(ID_SCANNING_KINECTFUSION,&CVCL_Volume_Scanner_View::OnScanningKinectfusion)
ON_COMMAND(ID_SCANNING_ICP,&CVCL_Volume_Scanner_View::OnScanningKinFuWithGlove)
ON_COMMAND(ID_SCANNING_PHOTO,&CVCL_Volume_Scanner_View::OnScanningPhotoMax)
ON_COMMAND(ID_TEST_DEPTHFLOW,&CVCL_Volume_Scanner_View::OnTestDepthflow)
ON_COMMAND(ID_TEST_RGBDODOMETRYOPENCV,&CVCL_Volume_Scanner_View::OnTestRgbdOdometryOpencv)
ON_COMMAND(ID_TEST_POSEESTIMATIONWITHARUCOMARKERSOPENCV,&CVCL_Volume_Scanner_View::OnTestPoseEstimationWithArucoMarkersOpenCV)
ON_COMMAND(ID_TEST_GRADIENTFIELDDISPLAY,&CVCL_Volume_Scanner_View::OnTestGradientFieldDisplay)
ON_COMMAND(ID_TEST_SIFTGPUTEST,&CVCL_Volume_Scanner_View::OnTestSiftGpuTest)
ON_COMMAND(ID_TEST_SINGLEPOSEESTIMATIONWITHARUCOMARKERSOPENCV,&CVCL_Volume_Scanner_View::OnTestSinglePoseEstimationWithArucoMarkersOpencv)
ON_COMMAND(ID_TEST_SEQUENCEPOSEESTIMATIONWITHCHARUCOMARKERSOPENCV,&CVCL_Volume_Scanner_View::OnTestSequenceposeestimationwithcharucomarkersopencv)
ON_COMMAND(ID_TEST_SINGLEPOSEESTIMATIONWITHCHARUCOMARKERSOPENCV,&CVCL_Volume_Scanner_View::OnTestSingleposeestimationwithcharucomarkersopencv)
ON_COMMAND(ID_CALIBRATION_CAPTURERGB,&CVCL_Volume_Scanner_View::OnCalibrationCapturergb)
ON_COMMAND(ID_TEST_CHESSBOARDREMOVALFROMDEPTHMAPS,&CVCL_Volume_Scanner_View::OnTestChessboardremovalfromdepthmaps)
ON_COMMAND(ID_TEST_AVERAGEIMAGE,&CVCL_Volume_Scanner_View::OnTestAverageimage)
END_MESSAGE_MAP()

// CVCL_Object_Scanner_SDIView 생성/소멸

CVCL_Volume_Scanner_View::CVCL_Volume_Scanner_View()
{
	// TODO: 여기에 생성 코드를 추가합니다.
	//zz_cnt = 0;
	
	Invalidate(true);
}

CVCL_Volume_Scanner_View::~CVCL_Volume_Scanner_View()
{
}

BOOL CVCL_Volume_Scanner_View::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: CREATESTRUCT cs를 수정하여 여기에서
	//  Window 클래스 또는 스타일을 수정합니다.
	return CView::PreCreateWindow(cs);
}

// CVCL_Object_Scanner_SDIView 그리기

void CVCL_Volume_Scanner_View::OnDraw(CDC* pDC)
{
	CVCL_Volume_Scanner_Doc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	//// TODO: 여기에 원시 데이터에 대한 그리기 코드를 추가합니다.
	int sww,shh;	// size of BMP drawing board.
	int ww,hh,ww_rgb,hh_rgb,ww_d,hh_d,ww_n,hh_n,ww_t,hh_t;
	float rsz_ratio = 1.0f;
	unsigned char **A=NULL,**B=NULL,**C=NULL,**D=NULL;


	if(zz_flag_player==0){	// stop button and initialization.

		CBitmap bitmap;
		//////////////////////////////////////////////////////////////////////////
		// default size is QHD.
		sww = 2560;
		shh = 1440;		
		//////////////////////////////////////////////////////////////////////////
		bitmap.CreateCompatibleBitmap(pDC,sww,shh);// create bitmap compatible with view DC.

		// fill black color on screen.
		pDC->PatBlt(0,0,sww,shh,BLACKNESS);

		// delete bitmap.
		bitmap.DeleteObject();
	}
	else{	// play button.

		// Display input image sequence /////////////////////////////////
		// ////////////////////////////////////////////////////////////

		CDC bufferDC;		// memory DC for double buffering.
		CBitmap *pBitmapOld,bitmap;

		// get image pointers.
		//////////////////////////////////////////////////////////////////////////
		// Screen A: RGB image.
		//////////////////////////////////////////////////////////////////////////
		A = zz_mat_rgbd.p_image_rgb()->mps(ww_rgb,hh_rgb);
		//A = zz_p_state->p_image_flows()->mps(ww_rgb, hh_rgb);		// for optical flow.

		printf("AAAAA\n");

		//////////////////////////////////////////////////////////////////////////
		// Screen B: Depth image with cube projection.
		//////////////////////////////////////////////////////////////////////////
		zz_display.dvoi_Draw_Volume_Of_Interest(
			zz_mat_rgbd.p_map_depth_raw(),
			zz_mat_rgbd.p_intrinsics_depth(),
			zz_p_state->p_extrinsics_glob_to_cam(),
			//in_rgbd_frame.ped_Pointer_of_Extrinsics_Depth(),
			zz_cube_size,
			zz_cube_origin,
			&zz_cimg[1]);
		B = zz_cimg[1].mps(ww_d,hh_d);

		printf("BBBBB\n");

		//////////////////////////////////////////////////////////////////////////
		// Screen C: Normal image of rendered depth map.
		//////////////////////////////////////////////////////////////////////////
		if(KV_MODE_DRIFT_DETECT){
			C = zz_cimg[3].mps(ww_n,hh_n);
		}
		else{
			if(zz_flag_scanning){
				// get image pointers.
				//////////////////////////////////////////////////////////////////////////
				// rendered normal image.
				//C = zz_p_state->p_image_normals_rendered()->mps(ww_n,hh_n);			

				// rendered texture image.
				C = zz_p_state->p_image_texture_rendered()->mps(ww_n,hh_n);
				//////////////////////////////////////////////////////////////////////////
			}
		}
		
		printf("CCCCC\n");

		//////////////////////////////////////////////////////////////////////////
		// Screen D: Camera trace map.
		//////////////////////////////////////////////////////////////////////////
//  		if(zz_flag_scanning){
//  			// + Camera trace.
//  			//if(zz_idx_frame % 5 == 0){
//  				//pDoc->zz_3dos.out_ccti_Current_Camera_Trace_Image(zz_cimg[2], true); 				
//  			//}
// 			zz_cimg[2].cp_Copy(&zz_img_gt);
// 			pDoc->zz_3dos.out_ccti_Current_Camera_Trace_Image(zz_cimg[2],false,20);
// 
// 			D = zz_cimg[2].mps(ww_t,hh_t);
//  		}
// 		printf("DDDDD\n");

		//////////////////////////////////////////////////////////////////////////
		// Display with double buffering technique.
		//////////////////////////////////////////////////////////////////////////
		// rgb ==================================================
		// prepare double buffering for flicker-free display.
		sww = ww_rgb;	shh = hh_rgb;	// set maximum size of BMP drawing board.
		bufferDC.CreateCompatibleDC(pDC);  // create DC object compatible with memory DC.
		bitmap.CreateCompatibleBitmap(pDC,sww,shh);// create bitmap compatible with view DC.

		pBitmapOld = bufferDC.SelectObject(&bitmap);

		// Screen A.
		bufferDC.PatBlt(0,0,sww,shh,BLACKNESS);
		zz_uw.p_Paint(bufferDC,false,KAISION_500_IMAGE_MODE_BLOCK_RGB,sww,shh,ww_rgb,hh_rgb,A[0]);	// draw image on memory DC.								
		pDC->BitBlt(0,0,sww,shh,&bufferDC,0,0,SRCCOPY);	// copy memory DC to view DC.
		printf("Screen\n");

		// Screen B.
		sww = ww_d;	shh = hh_d;
		bufferDC.PatBlt(0,0,sww,shh,BLACKNESS);
		zz_uw.p_Paint(bufferDC,false,KAISION_500_IMAGE_MODE_BLOCK_RGB,sww,shh,ww_d,hh_d,B[0]);	// draw image on memory DC.		
		pDC->BitBlt(ww_rgb,0,sww,shh,&bufferDC,0,0,SRCCOPY);	// copy memory DC to view DC.
		printf("Screen\n");
		// Screen C.
		if(zz_flag_scanning || KV_MODE_DRIFT_DETECT){  // drift detect 모드일때는 항상 출력.!
			// get image pointers.

			sww = ww_n;	shh = hh_n;
			bufferDC.PatBlt(0,0,sww,shh,BLACKNESS);
			//////////////////////////////////////////////////////////////////////////
			//zz_uw.p_Paint(bufferDC,false,KAISION_500_IMAGE_MODE_BLOCK_GRAY,sww,shh,ww_n,hh_n,C[0]);	// draw image on memory DC.								
			zz_uw.p_Paint(bufferDC,false,KAISION_500_IMAGE_MODE_BLOCK_RGB,sww,shh,ww_n,hh_n,C[0]);	// draw image on memory DC.		
			//////////////////////////////////////////////////////////////////////////
			pDC->BitBlt(0,hh_rgb,sww,shh,&bufferDC,0,0,SRCCOPY);	// copy memory DC to view DC.
		}
		printf("Screen\n");
// 		// Screen D.
//    		if(zz_flag_scanning){ 
//    			sww = ww_t;	shh = hh_t;
//    			bufferDC.PatBlt(0,0,sww,shh,WHITENESS);
//    			zz_uw.p_Paint(bufferDC,false,KAISION_500_IMAGE_MODE_BLOCK_RGB,sww,shh,ww_t,hh_t,D[0]);	// draw image on memory DC.								
//    			pDC->BitBlt(ww_rgb,hh_rgb,sww,shh,&bufferDC,0,0,SRCCOPY);	// copy memory DC to view DC.
//    		}
// 		printf("Screen\n");
		// delete memory DC and bitmap.
		bufferDC.SelectObject(pBitmapOld);
		bufferDC.DeleteDC();
		bitmap.DeleteObject();
		printf("End\n");
		// display without double buffering technique.
		//sww = ww + ww_d;	shh = max(hh, hh_d);
		//zz_uw.ss_Set_Size(pDC->GetWindow(), true, sww, shh);
		//zz_uw.p_Paint(*pDC, true, KAISION_500_IMAGE_MODE_BLVCL_Object_Scanner_SDI.rcOCK_RGB, ww, hh, ww, hh, A[0]);
		//zz_uw.p_Paint(*pDC, true, KAISION_500_IMAGE_MODE_BLOCK_GRAY, ww_d, hh_d, ww_d, hh_d, -100, 0, B[0]);

		//if(!Kv_Printf("!!!!!!")) exit(0);
	}
	
}


// CVCL_Object_Scanner_SDIView 인쇄

BOOL CVCL_Volume_Scanner_View::OnPreparePrinting(CPrintInfo* pInfo)
{
	// 기본적인 준비
	return DoPreparePrinting(pInfo);
}

void CVCL_Volume_Scanner_View::OnBeginPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: 인쇄하기 전에 추가 초기화 작업을 추가합니다.
}

void CVCL_Volume_Scanner_View::OnEndPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: 인쇄 후 정리 작업을 추가합니다.
}


// CVCL_Object_Scanner_SDIView 진단

#ifdef _DEBUG
void CVCL_Volume_Scanner_View::AssertValid() const
{
	CView::AssertValid();
}

void CVCL_Volume_Scanner_View::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CVCL_Volume_Scanner_Doc* CVCL_Volume_Scanner_View::GetDocument() const // 디버그되지 않은 버전은 인라인으로 지정됩니다.
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CVCL_Volume_Scanner_Doc)));
	return (CVCL_Volume_Scanner_Doc*)m_pDocument;
}
#endif //_DEBUG

void CVCL_Volume_Scanner_View::OnButtonPlay()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
  	zz_flag_player = 1; 
}


void CVCL_Volume_Scanner_View::OnButtonPause()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	zz_flag_player = 2;
}


void CVCL_Volume_Scanner_View::OnButtonStop()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
 	CVCL_Volume_Scanner_Doc *pDoc = GetDocument();
 
	zz_flag_player = 0;
	zz_idx_frame = 0;

	//if(zz_flag_scanning){
	pDoc->zz_3dos.rs_Release_Scanner();
	zz_flag_scanning = false;
	//}

 	Invalidate(true);
}


void CVCL_Volume_Scanner_View::OnButton3dScanning()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
 	CVCL_Volume_Scanner_Doc *pDoc = GetDocument();

	if(pDoc->zz_input_type<0)	return;

	LCKvYooji_Scanner_Display display;
	CKvYooji_Scanner_Parameter params;
	CKvYooji_ColorModel *p_gmm_hand = NULL;
	CKvYooji_ColorModel gmm_back;
	CKvVectorInt mask_back;

	float z_offset;	

	zz_idx_frame = 0;

	zz_flag_scanning = false;

	pDoc->zz_3dos.pis_Pre_Initialize_Scanner();

	printf("OnButton3dScanning...\n");
	if(!zz_flag_scanning){


	//	if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES){

			CKvString fn;
			// 			CKvPoint3Df cube_origin;
			// 			float cube_size, cube_dim;
			printf(" + lci_Load_Camera_Information...\n");
			if(!pDoc->zz_capture.lcis_Load_Camera_Information_System(&zz_mat_rgbd)){
				// Load camera parameters.
				zz_io.gof_Get_Open_Filename("Open camera parameters file for this sequence.",fn);
				pDoc->zz_capture.lci_Load_Camera_Information(fn,&zz_mat_rgbd);
			}

			//////////////////////////////////////////////////////////////////////////
			// for ground truth trace.
			if(KV_FLAG_GROUND_TRUTH){
				zz_io.gof_Get_Open_Filename("Open ground truth trace file for this sequence.",fn);
				pDoc->zz_capture.ltt_Load_Trace_from_Txt_for_SDF2SDF(fn,
					pDoc->zz_3dos.p_trace_gt());
				
// 				pDoc->zz_capture.ltt_Load_Trace_from_Txt_for_Steinsbrucker(fn,
// 					pDoc->zz_3dos.p_trace_gt());

				//////////////////////////////////////////////////////////////////////////
				// Ground truth's world coordinates is RGB camera.
				// Our world coordinates is depth camera!!
				// Modify ground truth pose by amount of RGB to depth
				//////////////////////////////////////////////////////////////////////////
				CKvYooji_Extrinsics t_ext;

				//d_pm_Printf_Matrix(zz_mat_rgbd.p_extrinsics_RGB_to_depth()->mp_transform()->vp(), 4, 4, "RGB-d");
								
				for(int i=0; i<pDoc->zz_3dos.p_trace_gt()->size(); i++){
					CKvYooji_Extrinsics *p_gt = (*pDoc->zz_3dos.p_trace_gt())[i];

					t_ext.copy(p_gt);
					t_ext.get_pose_relative(*zz_mat_rgbd.p_extrinsics_RGB_to_depth(), t_ext, *p_gt);
				}
			}
			//////////////////////////////////////////////////////////////////////////


			printf(" + lrdi_Load_Rgb_and_Depth_Images...\n");
			if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES){
				pDoc->zz_capture.lrdi_Load_Rgb_and_Depth_Images(
					zz_idx_frame,
					zz_mat_rgbd.p_image_rgb(),
					zz_mat_rgbd.p_map_depth_raw());
			}
			else if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES_CXD){
				//pDoc->zz_capture.lrdi_Load_Rgb_and_Depth_Images(
				pDoc->zz_capture.lrdc_Load_Rgb_and_Depth_from_CxD_File(
					zz_idx_frame,
					zz_mat_rgbd.p_image_rgb(),
					zz_mat_rgbd.p_map_depth_raw());
			}
			else if(pDoc->zz_input_type==KV_CAPTURE_FROM_KINECTV1){
				pDoc->zz_prime.gi_Get_Images(				
					zz_mat_rgbd.p_image_rgb(),
					zz_mat_rgbd.p_map_depth_raw());
			}
			// 			zz_capture.lpm_Load_P_Matrix(0,
			// 				zz_mat_rgbd.ppd_Pointer_of_Pmatrix_Depth());


			printf(" + loc_Load_Object_Cube...\n");
			if(!pDoc->zz_capture.loc_Load_Object_Cube(zz_cube_origin,zz_cube_size)){


				// Check this part later... ////////////////////////////////  
				///// //////////////////////////////////////////////////////

				zz_cube_size = KV_CUBE_SIZE;			
				// if object data, set z_offset 0.5m else 0.0m.
				z_offset = 0.3f;

				display.soc_Set_Object_Cube(
					zz_mat_rgbd,
					z_offset,
					zz_cube_size,
					zz_cube_origin);

				// Check this part later... ////////////////////////////////  
				///// //////////////////////////////////////////////////////

				//Kv_Printf("????\n");

				pDoc->zz_capture.soc_Save_Object_Cube(zz_cube_origin,zz_cube_size);

				if(!Kv_Printf("gco_Get_Cube_Origin_in_world"))	exit(0);

				//return;
			}

			float mu,dist_sq;
			mu = 5.0f*zz_cube_size/(float)KV_CUBE_DIM;//0.01f;//7.0f*zz_cube_size/(float)KV_CUBE_DIM;
			dist_sq = SQUARE(0.055f*zz_cube_size); // cube size 0.2m 기준 (0.02m)*(0.02m) 로 하자.

			//mu = 0.0055f;
			//dist_sq = 0.0001f;

// 			mu = 0.078125;//0.01f;//7.0f*zz_cube_size/(float)KV_CUBE_DIM;
// 			dist_sq = 0.025; // cube size 0.2m 기준 (0.02m)*(0.02m) 로 하자.
// 			zz_cube_dim = 512;
			 
			printf(" + sp_Set_Parameters...\n");
			zz_cube_dim = KV_CUBE_DIM;//KV_CUBE_DIM;

			params.sp_Set_Parameters(
				zz_cube_dim,zz_cube_dim,zz_cube_dim,
				8,
				zz_cube_size/(float)zz_cube_dim,
				zz_cube_origin,
				KV_LEVEL_OF_IMAGE_PYRAMID,//m_level_of_pyramid,
				dist_sq,
				mu);

			// ======================================================================
			printf(" + is_Initialize_Scanner...\n");

			if(pDoc->zz_scan_type == 1){
				// Load color model.
				// + Color gloves.
				p_gmm_hand = new CKvYooji_ColorModel;
				if(!pDoc->zz_capture.lgcm_Load_Glove_Color_Model(p_gmm_hand)){
					zz_io.gof_Get_Open_Filename("Open GMM parameters file for gloves color.",fn);	
					p_gmm_hand->lcm_Load_Color_Model(fn);
				}

				pDoc->zz_3dos.is_Initialize_Scanner(
					params,
					zz_mat_rgbd,
					true,//m_flag_gpu,
					false,
					//////////////////////////
					true,//m_flag_on_rgb_cam,
					//////////////////////////
					p_gmm_hand);
			} 
			else if(pDoc->zz_scan_type == 2){
				//////////////////////////////////////////////////////////////////////////
				// Change for background cut.
				printf(" + lbcm_Load_Background_Color_Model...\n");
				// + Color background.
				CKvYooji_ColorModel model_back_pixel,model_back_global;
				bool flag_train_back,flag_train_back_pixel;
				//////////////////////////////////////////////////////////////////////////
				// load trained background color model.
				printf(" + Pixelwise model...\n");
				flag_train_back_pixel = pDoc->zz_capture.lbcmp_Load_Background_Color_Model_Pixelwise(&model_back_pixel);
				printf(" + Global model...\n");
				flag_train_back = pDoc->zz_capture.lbcmg_Load_Background_Color_Model_Global(&model_back_global);

				// if two background models were successfully loaded, import them.
				if(!flag_train_back || !flag_train_back_pixel){
					Kv_Printf("Background training is needed!");
					exit(0);
				}

				pDoc->zz_3dos.is_Initialize_Scanner(
					params,
					zz_mat_rgbd,
					true,//m_flag_gpu,
					true,
					//////////////////////////
					true,//m_flag_on_rgb_cam,
					//////////////////////////
					NULL,
					&model_back_pixel,
					&model_back_global);

			} 
			else{
				pDoc->zz_3dos.is_Initialize_Scanner(
					params,
					zz_mat_rgbd,
					true,//m_flag_gpu,
					false,
					//////////////////////////
					true//m_flag_on_rgb_cam
					//////////////////////////
					);
			}
			// ==================================================================

			//zz_idx_frame = 0;

			zz_flag_scanning = true;

//		} 
	}
		

	printf("Complete!\n");

	if(p_gmm_hand)	delete p_gmm_hand;
 
 	
}



void CVCL_Volume_Scanner_View::OnScanningKinectfusion()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	CVCL_Volume_Scanner_Doc *pDoc = GetDocument();

	//////////////////////////////////////////////////////////////////////////
	//pDoc->zz_3dos.rs_Release_Scanner();
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	// for ArUco.
// 	CKvYooji_InterLib_Convert zz_ilc;
// 	float vals[] ={FX,0.,CX,0.,FY,CY,0.,0.,1.};
// 	float coeffs[] ={0.,0.,0.,0.,0.,};
// 
// 	const Mat cameraMatrix = Mat(3,3,CV_32FC1,vals);
// 	const Mat distCoeffs = Mat(5,1,CV_32FC1,coeffs);
// 
// 	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
// 	cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(5,7,0.04,0.01,dictionary);
// 	cv::Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
// 
// 	detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers
	//////////////////////////////////////////////////////////////////////////

//	if(!Kv_Printf("Release!")) exit(0);

	LCKvYooji_Scanner_Display display;
	bool flag_sil,flag_pose,flag_load;

	if(pDoc->zz_input_type<0)	return;

	zz_flag_player = 1;

	pDoc->zz_scan_type = 0;
	OnButton3dScanning();

	//////////////////////////////////////////////////////////////////////////
	bool valid_drift_recov = true;
	//////////////////////////////////////////////////////////////////////////

//	if(!Kv_Printf("OnButton3dScanning!")) exit(0);
	
	// 	printf("%s\n", dn_texture.bp());printf("\n");
	// 	printf("%s\n", dn_depth.bp());printf("\n");
	// 	printf("Type: %d\n", m_type_of_data);

	// Display current scanning state ///////////////////////////////
	// //////////////////////////////////////////////////////////////
	zz_idx_frame = 0;

	int start_frame = zz_idx_frame;

	CKvStopWatch sw;	sw.c_Create(1);
	float total_time = 0.0f,avg_time = 0.0f;

	// for display trace. ////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	vector<CKvYooji_Extrinsics*>* p_trace_gt = pDoc->zz_3dos.p_trace_gt();
	zz_img_gt.c_Create(480,640, Kv_Rgb(255, 255, 255));
// 	if(zz_idx_frame == 0){
// 		if(p_trace_gt->size() > 1){
// 			// Initialize trace image with ground truth.
// 			CKvYooji_Intrinsics int_ref; CKvYooji_Extrinsics ext_ref;
// 			pDoc->zz_3dos.grt_Get_Reference_Camera_for_Trace(int_ref,ext_ref);
// 			zz_display.rsct_Render_Set_of_Camera_Traces(&ext_ref,p_trace_gt,&int_ref,&int_ref,&zz_img_gt,0.0f,-1,
// 				&Kv_Rgb(0,0,255));
// 		}
// 	}

	//////////////////////////////////////////////////////////////////////////
	// for initialize the first frame of KINECTV1.
	if(pDoc->zz_input_type==KV_CAPTURE_FROM_KINECTV1){
		// Waits first image.
		CKvScreen sc; CKvMatrixUcharRgb timg;

		while(1){
			if(!pDoc->zz_prime.gi_Get_Images(
					zz_mat_rgbd.p_image_rgb(),
					zz_mat_rgbd.p_map_depth_raw())) break;

			zz_display.dvoi_Draw_Volume_Of_Interest(
					zz_mat_rgbd.p_map_depth_raw(),
					zz_mat_rgbd.p_intrinsics_depth(),
					zz_mat_rgbd.p_extrinsics_depth(),
					zz_cube_size,
					zz_cube_origin,
					&timg);

			sc.s_d_Display(&timg);
			Kv_Pause(10);

			// set the first frame and start scanning process.
			char ch = sc.s_gc_Get_Character();
			if(ch== 's') break;

		}
	}
	//////////////////////////////////////////////////////////////////////////

	while(zz_flag_player){

		// play.
		if(zz_flag_player==1){

			flag_load = false;

			//////////////////////////////////////////////////////////////////////////
			// load input data.
			//////////////////////////////////////////////////////////////////////////
			// pre-captured image sequence.
			if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES || pDoc->zz_input_type==KV_CAPTURE_FROM_FILES_CXD){

				if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES){
					flag_load = pDoc->zz_capture.lrdi_Load_Rgb_and_Depth_Images(
						zz_idx_frame,
						zz_mat_rgbd.p_image_rgb(),
						zz_mat_rgbd.p_map_depth_raw());
				} 
				else if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES_CXD){

					flag_load = pDoc->zz_capture.lrdc_Load_Rgb_and_Depth_from_CxD_File(
						zz_idx_frame,
						zz_mat_rgbd.p_image_rgb(),
						zz_mat_rgbd.p_map_depth_raw());
				}

				if(flag_load){
					// Get pre-extracted silhouette.
					flag_sil = pDoc->zz_capture.ls_Load_Silhouette(zz_idx_frame,zz_mat_rgbd.p_image_silhouette());
					zz_mat_rgbd.set_validity_silhouette(flag_sil);

					// Get pre-extracted silhouette.					
					flag_pose = pDoc->zz_capture.lpm_Load_P_Matrix(zz_idx_frame,zz_mat_rgbd.p_P_matrix_depth());
					zz_mat_rgbd.set_validity_extrinsics(flag_pose);
				}
			}
			// live stream.
			else if(pDoc->zz_input_type==KV_CAPTURE_FROM_KINECTV1){
				flag_load = pDoc->zz_prime.gi_Get_Images(
					zz_mat_rgbd.p_image_rgb(),
					zz_mat_rgbd.p_map_depth_raw());
			}

			//printf("flag_load: %d\n", flag_load);

			if(flag_load){

				sw.r_Reset(0);

		//		if(!Kv_Printf("so_Scan_Object")) exit(0);

				//////////////////////////////////////////////////////////////////////////
				// ArUco Module
				//////////////////////////////////////////////////////////////////////////
// 				cv::Mat image,imageCopy;
// 				//inputVideo.retrieve(image);
// 				zz_ilc.cfko_Convert_Format_from_KAISION_to_Opencv(*zz_mat_rgbd.p_image_rgb(),image);
// 
// 				image.copyTo(imageCopy);
// 				std::vector<int> ids;
// 				std::vector<std::vector<cv::Point2f> > corners, rejected;
// 				cv::aruco::detectMarkers(image,dictionary,corners,ids,detectorParams,rejected);
// 				// refine strategy to detect more markers
// 				aruco::refineDetectedMarkers(image,board,corners,ids,rejected);
// 				
// 				// if at least one marker detected
// 				if(ids.size() > 0) {
// 					cv::aruco::drawDetectedMarkers(imageCopy,corners,ids);
// 					cv::Vec3d rvec,tvec;
// 					int valid = estimatePoseBoard(corners,ids,board,cameraMatrix,distCoeffs,rvec,tvec);
// 					// if at least one board marker detected
// 					if(valid > 0)
// 						cv::aruco::drawAxis(imageCopy,cameraMatrix,distCoeffs,rvec,tvec,0.1);
// 				}
// 				cv::imshow("out",imageCopy);
				//////////////////////////////////////////////////////////////////////////
				// drift checker.
		
				
				//////////////////////////////////////////////////////////////////////////
				// scan object.
				//////////////////////////////////////////////////////////////////////////
				if(valid_drift_recov)
					zz_flag_scanning = pDoc->zz_3dos.so_Scan_Object(
						&zz_mat_rgbd,
						false,//m_flag_vh_shield,
						true,//m_flag_on_rgb_cam,
	//						KV_TYPE_INPUT_OBJECT,//m_type_of_data,
						KV_TYPE_INPUT_OBJECT,//m_type_of_data,
						KV_FLAG_GROUND_TRUTH ? KV_TYPE_TRACK_GROUND : KV_TYPE_TRACK_DEPTH,
						KV_LEVEL_OF_IMAGE_PYRAMID,//m_level_of_pyramid,
						KV_FLAG_GROUND_REMOVAL,
						true,//m_flag_gpu,
						true);
						//false);

				//zz_flag_scanning = true;

				printf("total scanning validity: %d\n", zz_flag_scanning);
				
				//////////////////////////////////////////////////////////////////////////
				// drift detector.
				//////////////////////////////////////////////////////////////////////////
				// Object Detector.
				if(KV_MODE_DRIFT_DETECT){
					if(!zz_flag_scanning){

						//if(!Kv_Printf("Drift!!")) exit(0);

						valid_drift_recov = pDoc->zz_3dos.rcd_Recover_Camera_Drift(&zz_mat_rgbd, &zz_cimg[3]);
						printf("valid_drift_recov: %d\n", valid_drift_recov);
					}
					else{

						zz_p_state = pDoc->zz_3dos.out_pss_OUTput_Pointer_of_Scanning_State();

						// normal rendering on rgb image.
						zz_display.rn_Render_Normal_on_RGB_Image(
							zz_p_state->p_image_RGB(),
							zz_p_state->p_image_normals_rendered(),
							&zz_cimg[3]);
					}

					CKvString fn;
					fn.fm_Format("_drift\\%04d.bmp",zz_idx_frame);
					zz_iof.si_Save_Image(fn,false,&zz_cimg[3]);
				}
				else{
					// get current scanning state.
					if(zz_flag_scanning){
					 	//printf("out_ss_OUTput_Scanning_State\n");
					 
					 	zz_p_state = pDoc->zz_3dos.out_pss_OUTput_Pointer_of_Scanning_State();

						zz_display.rn_Render_Normal_on_RGB_Image(
							zz_p_state->p_image_RGB(),
							zz_p_state->p_image_normals_rendered(),
							&zz_cimg[3]);
		// 
		// 					//////////////////////////////////////////////////////////////////////////
		// 					// Display images.
		// 					Invalidate(true);
		// 					//////////////////////////////////////////////////////////////////////////
		// 					//if(!Kv_Printf("Pause (%d)", 0))	exit(0);
		// 
		// 					Kv_Pause(1);
					}
				}
// 				//////////////////////////////////////////////////////////////////////////
// 				if(!zz_flag_scanning){
// 					printf("=========================== TRACKING FAIL ===========================\n");
// 					if(!Kv_Printf(re)) exit(0);
// 				}
				printf("zz_flag_scanning: %d\n", zz_flag_scanning);



				printf("Invalidate: %d\n",zz_flag_scanning);

				//////////////////////////////////////////////////////////////////////////
				// Display images.
				Invalidate(true);
				//////////////////////////////////////////////////////////////////////////
				//if(!Kv_Printf("Pause (%d)", 0))	exit(0);

				Kv_Pause(1);


				//if(zz_idx_frame == start_frame) if(!Kv_Printf("First frame!")) exit(0);

				zz_idx_frame++;

				total_time += sw.get_Get_Elapsed_Time(0);
				avg_time = total_time/(float)zz_idx_frame;

				printf("  #%d frame was processed... (%5.2f ms)\n",zz_idx_frame,avg_time*1000.0f);

			} 
			else{
				// STOP.
				zz_flag_player = 0;
			}
		}

		//if(!Kv_Printf("pause")) exit(0);

		// pause.
		if(zz_flag_player==2){
			while(zz_flag_player==2) Kv_Pause(1);
		}

		//if(!Kv_Printf("stop1")) exit(0);
		// stop.
		if(zz_flag_player==0){
			//Invalidate(true);
			zz_idx_frame = 0;
			break;
		}
		//if(!Kv_Printf("stop2")) exit(0);
	}

	zz_flag_player = 0;

}


void CVCL_Volume_Scanner_View::OnScanningKinFuWithGlove()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	CVCL_Volume_Scanner_Doc *pDoc = GetDocument();

	LCKvYooji_Scanner_Display display;
	bool flag_sil,flag_pose,flag_load;

	if(pDoc->zz_input_type<0)	return;

	zz_flag_player = 1;

	pDoc->zz_scan_type = 1;
	OnButton3dScanning();

	// 	printf("%s\n", dn_texture.bp());printf("\n");
	// 	printf("%s\n", dn_depth.bp());printf("\n");
	// 	printf("Type: %d\n", m_type_of_data);

	// Display current scanning state ///////////////////////////////
	// //////////////////////////////////////////////////////////////
	zz_idx_frame = 0;

	int start_frame = zz_idx_frame;

	// display RGB-D image sequence.
	//if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES){

	CKvStopWatch sw;	sw.c_Create(1);
	float total_time = 0.0f,avg_time = 0.0f;

	// for display trace.
	//zz_cimg[2].c_Create(480,640);
	// for display trace. ////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	vector<CKvYooji_Extrinsics*>* p_trace_gt = pDoc->zz_3dos.p_trace_gt();
	zz_img_gt.c_Create(480,640,Kv_Rgb(255,255,255));
// 	if(zz_idx_frame == 0){
// 		if(p_trace_gt->size() > 1){
// 			// Initialize trace image with ground truth.
// 			CKvYooji_Intrinsics int_ref; CKvYooji_Extrinsics ext_ref;
// 			pDoc->zz_3dos.grt_Get_Reference_Camera_for_Trace(int_ref,ext_ref);
// 			zz_display.rsct_Render_Set_of_Camera_Traces(&ext_ref,p_trace_gt,&int_ref,&int_ref,&zz_img_gt,0.0f,-1,
// 				&Kv_Rgb(0,0,255));
// 		}
// 	}

	//////////////////////////////////////////////////////////////////////////
	// for initialize the first frame of KINECTV1.
	if(pDoc->zz_input_type==KV_CAPTURE_FROM_KINECTV1){
		// Waits first image.
		CKvScreen sc; CKvMatrixUcharRgb timg;

		while(1){
			if(!pDoc->zz_prime.gi_Get_Images(
					zz_mat_rgbd.p_image_rgb(),
					zz_mat_rgbd.p_map_depth_raw())) break;

			zz_display.dvoi_Draw_Volume_Of_Interest(
					zz_mat_rgbd.p_map_depth_raw(),
					zz_mat_rgbd.p_intrinsics_depth(),
					zz_mat_rgbd.p_extrinsics_depth(),
					zz_cube_size,
					zz_cube_origin,
					&timg);

			sc.s_d_Display(&timg);
			Kv_Pause(10);

			// set the first frame and start scanning process.
			char ch = sc.s_gc_Get_Character();
			if(ch== 's') break;

		}
	}
	//////////////////////////////////////////////////////////////////////////

	while(zz_flag_player){

		// play.
		if(zz_flag_player==1){

			flag_load = false;

			//////////////////////////////////////////////////////////////////////////
			// load input data.
			//////////////////////////////////////////////////////////////////////////
			// pre-captured image sequence.
			if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES || pDoc->zz_input_type==KV_CAPTURE_FROM_FILES_CXD){

				if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES){
					flag_load = pDoc->zz_capture.lrdi_Load_Rgb_and_Depth_Images(
						zz_idx_frame,
						zz_mat_rgbd.p_image_rgb(),
						zz_mat_rgbd.p_map_depth_raw());


				} else if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES_CXD){
					flag_load = pDoc->zz_capture.lrdc_Load_Rgb_and_Depth_from_CxD_File(
						zz_idx_frame,
						zz_mat_rgbd.p_image_rgb(),
						zz_mat_rgbd.p_map_depth_raw());
				}

				if(flag_load){
					// Get pre-extracted silhouette.
					//flag_sil = pDoc->zz_capture.ls_Load_Silhouette(zz_idx_frame,zz_mat_rgbd.p_image_silhouette());
					//zz_mat_rgbd.set_validity_silhouette(flag_sil);

					// Get pre-extracted silhouette.					
					flag_pose = pDoc->zz_capture.lpm_Load_P_Matrix(zz_idx_frame,zz_mat_rgbd.p_P_matrix_depth());
					zz_mat_rgbd.set_validity_extrinsics(flag_pose);
				}
			}
			// live stream.
			else if(pDoc->zz_input_type==KV_CAPTURE_FROM_KINECTV1){
				flag_load = pDoc->zz_prime.gi_Get_Images(
					zz_mat_rgbd.p_image_rgb(),
					zz_mat_rgbd.p_map_depth_raw());
			}

			if(flag_load){

				sw.r_Reset(0);

				//////////////////////////////////////////////////////////////////////////
				// scan object.
				//////////////////////////////////////////////////////////////////////////
				zz_flag_scanning = pDoc->zz_3dos.so_Scan_Object(
					&zz_mat_rgbd,
					false,//m_flag_vh_shield,
					true,//m_flag_on_rgb_cam,
					KV_TYPE_INPUT_OBJECT,//m_type_of_data,
					KV_TYPE_TRACK_DEPTH,
					//KV_TYPE_TRACK_COLOR,
					KV_LEVEL_OF_IMAGE_PYRAMID,//m_level_of_pyramid,
					KV_FLAG_GROUND_REMOVAL,
					true,//m_flag_gpu,
					//true);
					false);

				//////////////////////////////////////////////////////////////////////////
				// drift detector.
				//////////////////////////////////////////////////////////////////////////
				//////////////////////////////////////////////////////////////////////////
				if(KV_MODE_DRIFT_DETECT){
				}
// 				else if(!zz_flag_scanning){
// 					printf("=========================== TRACKING FAIL ===========================\n");
// 					if(!Kv_Printf("!!!")) exit(0);
// 				}


				// get current scanning state.
				if(zz_flag_scanning){
					//printf("out_ss_OUTput_Scanning_State\n");

					zz_p_state = pDoc->zz_3dos.out_pss_OUTput_Pointer_of_Scanning_State();
					//////////////////////////////////////////////////////////////////////////
					// Display images.
					Invalidate(true);
					//////////////////////////////////////////////////////////////////////////

					Kv_Pause(1);
				}


				//if(zz_idx_frame == start_frame) if(!Kv_Printf("First frame!")) exit(0);

				zz_idx_frame++;

				total_time += sw.get_Get_Elapsed_Time(0);
				avg_time = total_time/(float)zz_idx_frame;

				printf("  #%d frame was processed... (%5.2f ms)\n",zz_idx_frame,avg_time*1000.0f);
			} else{
				// STOP.
				zz_flag_player = 0;
			}
			
		}

		// pause.
		if(zz_flag_player==2){
			while(zz_flag_player==2) Kv_Pause(1);
		}
		// stop.
		if(zz_flag_player==0){
			//Invalidate(true);
			zz_idx_frame = 0;
			break;
		}
	}

	zz_flag_player = 0;
}




void CVCL_Volume_Scanner_View::OnScanningPhotoMax()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	CVCL_Volume_Scanner_Doc *pDoc = GetDocument();

	//////////////////////////////////////////////////////////////////////////
	//pDoc->zz_3dos.rs_Release_Scanner();
	//////////////////////////////////////////////////////////////////////////

	//	if(!Kv_Printf("Release!")) exit(0);

	LCKvYooji_Scanner_Display display;
	bool flag_sil,flag_pose,flag_load;

	if(pDoc->zz_input_type<0)	return;

	bool valid_drift_recov = true;

	zz_flag_player = 1;

	//////////////////////////////////////////////////////////////////////////
	pDoc->zz_scan_type = KV_FLAG_COLOR_GLOVE;
	//////////////////////////////////////////////////////////////////////////
	OnButton3dScanning();

	//	if(!Kv_Printf("OnButton3dScanning!")) exit(0);

	// 	printf("%s\n", dn_texture.bp());printf("\n");
	// 	printf("%s\n", dn_depth.bp());printf("\n");
	// 	printf("Type: %d\n", m_type_of_data);

	// Display current scanning state ///////////////////////////////
	// //////////////////////////////////////////////////////////////
	zz_idx_frame = 0;

	int start_frame = zz_idx_frame;

	CKvStopWatch sw;	sw.c_Create(1);
	float total_time = 0.0f,avg_time = 0.0f;

	// for display trace.
	//zz_cimg[2].c_Create(480,640);
	// for display trace. ////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	vector<CKvYooji_Extrinsics*>* p_trace_gt = pDoc->zz_3dos.p_trace_gt();
	zz_img_gt.c_Create(480,640,Kv_Rgb(255,255,255));
// 	if(zz_idx_frame == 0){
// 		if(p_trace_gt->size() > 1){
// 			// Initialize trace image with ground truth.
// 			CKvYooji_Intrinsics int_ref; CKvYooji_Extrinsics ext_ref;
// 			pDoc->zz_3dos.grt_Get_Reference_Camera_for_Trace(int_ref,ext_ref);
// 			zz_display.rsct_Render_Set_of_Camera_Traces(&ext_ref,p_trace_gt,&int_ref,&int_ref,&zz_img_gt,0.0f,-1,
// 				&Kv_Rgb(0,0,255));
// 		}
// 	}

	//////////////////////////////////////////////////////////////////////////
	// for initialize the first frame of KINECTV1.
	if(pDoc->zz_input_type==KV_CAPTURE_FROM_KINECTV1){
		// Waits first image.
		CKvScreen sc; CKvMatrixUcharRgb timg;

		while(1){
			if(!pDoc->zz_prime.gi_Get_Images(
					zz_mat_rgbd.p_image_rgb(),
					zz_mat_rgbd.p_map_depth_raw())) break;

			zz_display.dvoi_Draw_Volume_Of_Interest(
					zz_mat_rgbd.p_map_depth_raw(),
					zz_mat_rgbd.p_intrinsics_depth(),
					zz_mat_rgbd.p_extrinsics_depth(),
					zz_cube_size,
					zz_cube_origin,
					&timg);

			sc.s_d_Display(&timg);
			Kv_Pause(10);

			// set the first frame and start scanning process.
			char ch = sc.s_gc_Get_Character();
			if(ch== 's') break;

		}
	}
	//////////////////////////////////////////////////////////////////////////

	while(zz_flag_player){

		// play.
		if(zz_flag_player==1){

			flag_load = false;

			//////////////////////////////////////////////////////////////////////////
			// load input data.
			//////////////////////////////////////////////////////////////////////////
			// pre-captured image sequence.
			if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES || pDoc->zz_input_type==KV_CAPTURE_FROM_FILES_CXD){

				if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES){
					flag_load = pDoc->zz_capture.lrdi_Load_Rgb_and_Depth_Images(
						zz_idx_frame,
						zz_mat_rgbd.p_image_rgb(),
						zz_mat_rgbd.p_map_depth_raw());
				} 
				else if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES_CXD){

					flag_load = pDoc->zz_capture.lrdc_Load_Rgb_and_Depth_from_CxD_File(
						zz_idx_frame,
						zz_mat_rgbd.p_image_rgb(),
						zz_mat_rgbd.p_map_depth_raw());

					//					// for debugging.
					//  					CKvMatrixUcharRgb img_rgb;
					//  					CKvMatrixFloat map_depth;
					//  
					//  					pDoc->zz_capture.lrdi_Load_Rgb_and_Depth_Images(
					//  						zz_idx_frame,
					////  						zz_mat_rgbd.p_image_rgb(),
					////  						zz_mat_rgbd.p_map_depth_raw());
					//  						&img_rgb,
					//  						&map_depth);
					// 
					//  					// for debugging.
					//  					float sum_error = 0.0f;					 
					//  					for(int i=0; i<map_depth.vs(); i++){
					//  					 	sum_error += abs(map_depth.vp()[i] - zz_mat_rgbd.p_map_depth_raw()->vp()[i]);
					//  					}
					//  					printf("=================== sum_error: %f\n", sum_error);
				}

				if(flag_load){
					// Get pre-extracted silhouette.
					flag_sil = pDoc->zz_capture.ls_Load_Silhouette(zz_idx_frame,zz_mat_rgbd.p_image_silhouette());
					zz_mat_rgbd.set_validity_silhouette(flag_sil);

					// Get pre-extracted silhouette.					
					flag_pose = pDoc->zz_capture.lpm_Load_P_Matrix(zz_idx_frame,zz_mat_rgbd.p_P_matrix_depth());
					zz_mat_rgbd.set_validity_extrinsics(flag_pose);
				}
			}
			// live stream.
			else if(pDoc->zz_input_type==KV_CAPTURE_FROM_KINECTV1){
				flag_load = pDoc->zz_prime.gi_Get_Images(
					zz_mat_rgbd.p_image_rgb(),
					zz_mat_rgbd.p_map_depth_raw());
			}

			//printf("flag_load: %d\n", flag_load);

			if(flag_load){

				sw.r_Reset(0);

				//		if(!Kv_Printf("so_Scan_Object")) exit(0);


				//////////////////////////////////////////////////////////////////////////
				// scan object.
				//////////////////////////////////////////////////////////////////////////
				if(valid_drift_recov){
				zz_flag_scanning = pDoc->zz_3dos.so_Scan_Object(
					&zz_mat_rgbd,
					false,//m_flag_vh_shield,
					true,//m_flag_on_rgb_cam,
//						KV_TYPE_INPUT_OBJECT,//m_type_of_data,
					KV_TYPE_INPUT_OBJECT,//m_type_of_data,
					KV_TYPE_TRACK_COLOR,
					KV_LEVEL_OF_IMAGE_PYRAMID,//m_level_of_pyramid,
					KV_FLAG_GROUND_REMOVAL,
					true,//m_flag_gpu,
					true);
					//false);
				}

				//////////////////////////////////////////////////////////////////////////
				// drift detector.
				//////////////////////////////////////////////////////////////////////////
				// Object Detector.
				if(KV_MODE_DRIFT_DETECT){
					if(!zz_flag_scanning){

						//if(!Kv_Printf("Drift!!")) exit(0);

						valid_drift_recov = pDoc->zz_3dos.rcd_Recover_Camera_Drift(&zz_mat_rgbd,&zz_cimg[3]);
						printf("valid_drift_recov: %d\n",valid_drift_recov);
					} 
					else{

						zz_p_state = pDoc->zz_3dos.out_pss_OUTput_Pointer_of_Scanning_State();

						// normal rendering on rgb image.
						zz_display.rn_Render_Normal_on_RGB_Image(
							zz_p_state->p_image_RGB(),
							zz_p_state->p_image_normals_rendered(),
							&zz_cimg[3]);
					}

					CKvString fn;
					fn.fm_Format("_drift\\%04d.bmp",zz_idx_frame);
					zz_iof.si_Save_Image(fn,false,&zz_cimg[3]);
				} 
				else{
					// get current scanning state.
					if(zz_flag_scanning){
						//printf("out_ss_OUTput_Scanning_State\n");

						zz_p_state = pDoc->zz_3dos.out_pss_OUTput_Pointer_of_Scanning_State();

						zz_display.rn_Render_Normal_on_RGB_Image(
							zz_p_state->p_image_RGB(),
							zz_p_state->p_image_normals_rendered(),
							&zz_cimg[3]);
						// 
						// 					//////////////////////////////////////////////////////////////////////////
						// 					// Display images.
						// 					Invalidate(true);
						// 					//////////////////////////////////////////////////////////////////////////
						// 					//if(!Kv_Printf("Pause (%d)", 0))	exit(0);
						// 
						// 					Kv_Pause(1);
					}
				}

				//////////////////////////////////////////////////////////////////////////
				// Display images.
				Invalidate(true);
				//////////////////////////////////////////////////////////////////////////
				//if(!Kv_Printf("Pause (%d)", 0))	exit(0);

				Kv_Pause(1);

				//if(zz_idx_frame == start_frame) if(!Kv_Printf("First frame!")) exit(0);

				//////////////////////////////////////////////////////////////////////////
				zz_idx_frame++;

				//zz_idx_frame += 2;
				//////////////////////////////////////////////////////////////////////////

				total_time += sw.get_Get_Elapsed_Time(0);
				avg_time = total_time/(float)zz_idx_frame;

				printf("  #%d frame was processed... (%5.2f ms)\n",zz_idx_frame,avg_time*1000.0f);

			} else{
				// STOP.
				zz_flag_player = 0;
			}
		}

		//if(!Kv_Printf("pause")) exit(0);

		// pause.
		if(zz_flag_player==2){
			while(zz_flag_player==2) Kv_Pause(1);
		}

		//if(!Kv_Printf("stop1")) exit(0);
		// stop.
		if(zz_flag_player==0){
			//Invalidate(true);
			zz_idx_frame = 0;
			break;
		}
		//if(!Kv_Printf("stop2")) exit(0);
	}

	zz_flag_player = 0;
}

void CVCL_Volume_Scanner_View::OnTestDepthflow()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	CVCL_Volume_Scanner_Doc *pDoc = GetDocument();

	//////////////////////////////////////////////////////////////////////////
	//pDoc->zz_3dos.rs_Release_Scanner();
	//////////////////////////////////////////////////////////////////////////

	//	if(!Kv_Printf("Release!")) exit(0);

	LCKvYooji_Scanner_Display display;
	bool flag_sil,flag_pose,flag_load;

	if(pDoc->zz_input_type<0)	return;

	zz_flag_player = 1;

	pDoc->zz_scan_type = 0;
	OnButton3dScanning();

	//	if(!Kv_Printf("OnButton3dScanning!")) exit(0);

	// 	printf("%s\n", dn_texture.bp());printf("\n");
	// 	printf("%s\n", dn_depth.bp());printf("\n");
	// 	printf("Type: %d\n", m_type_of_data);

	// Display current scanning state ///////////////////////////////
	// //////////////////////////////////////////////////////////////
	zz_idx_frame = 0;

	int start_frame = zz_idx_frame;

	CKvStopWatch sw;	sw.c_Create(1);
	float total_time = 0.0f,avg_time = 0.0f;

	// for display trace.
	zz_cimg[2].c_Create(480,640);

	//////////////////////////////////////////////////////////////////////////
	// for initialize the first frame of KINECTV1.
	if(pDoc->zz_input_type==KV_CAPTURE_FROM_KINECTV1){
		// Waits first image.
		CKvScreen sc; CKvMatrixUcharRgb timg;

		while(1){
			if(!pDoc->zz_prime.gi_Get_Images(
					zz_mat_rgbd.p_image_rgb(),
					zz_mat_rgbd.p_map_depth_raw())) break;

			zz_display.dvoi_Draw_Volume_Of_Interest(
					zz_mat_rgbd.p_map_depth_raw(),
					zz_mat_rgbd.p_intrinsics_depth(),
					zz_mat_rgbd.p_extrinsics_depth(),
					zz_cube_size,
					zz_cube_origin,
					&timg);

			sc.s_d_Display(&timg);
			Kv_Pause(10);

			// set the first frame and start scanning process.
			char ch = sc.s_gc_Get_Character();
			if(ch== 's') break;

		}
	}
	//////////////////////////////////////////////////////////////////////////

	while(zz_flag_player){

		// play.
		if(zz_flag_player==1){

			flag_load = false;

			//////////////////////////////////////////////////////////////////////////
			// load input data.
			//////////////////////////////////////////////////////////////////////////
			// pre-captured image sequence.
			if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES || pDoc->zz_input_type==KV_CAPTURE_FROM_FILES_CXD){

				if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES){
					flag_load = pDoc->zz_capture.lrdi_Load_Rgb_and_Depth_Images(
						zz_idx_frame,
						zz_mat_rgbd.p_image_rgb(),
						zz_mat_rgbd.p_map_depth_raw());
				} else if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES_CXD){

					flag_load = pDoc->zz_capture.lrdc_Load_Rgb_and_Depth_from_CxD_File(
						zz_idx_frame,
						zz_mat_rgbd.p_image_rgb(),
						zz_mat_rgbd.p_map_depth_raw());

					//					// for debugging.
					//  					CKvMatrixUcharRgb img_rgb;
					//  					CKvMatrixFloat map_depth;
					//  
					//  					pDoc->zz_capture.lrdi_Load_Rgb_and_Depth_Images(
					//  						zz_idx_frame,
					////  						zz_mat_rgbd.p_image_rgb(),
					////  						zz_mat_rgbd.p_map_depth_raw());
					//  						&img_rgb,
					//  						&map_depth);
					// 
					//  					// for debugging.
					//  					float sum_error = 0.0f;					 
					//  					for(int i=0; i<map_depth.vs(); i++){
					//  					 	sum_error += abs(map_depth.vp()[i] - zz_mat_rgbd.p_map_depth_raw()->vp()[i]);
					//  					}
					//  					printf("=================== sum_error: %f\n", sum_error);
				}

				if(flag_load){
					// Get pre-extracted silhouette.
					flag_sil = pDoc->zz_capture.ls_Load_Silhouette(zz_idx_frame,zz_mat_rgbd.p_image_silhouette());
					zz_mat_rgbd.set_validity_silhouette(flag_sil);

					// Get pre-extracted silhouette.					
					flag_pose = pDoc->zz_capture.lpm_Load_P_Matrix(zz_idx_frame,zz_mat_rgbd.p_P_matrix_depth());
					zz_mat_rgbd.set_validity_extrinsics(flag_pose);
				}
			}
			// live stream.
			else if(pDoc->zz_input_type==KV_CAPTURE_FROM_KINECTV1){
				flag_load = pDoc->zz_prime.gi_Get_Images(
					zz_mat_rgbd.p_image_rgb(),
					zz_mat_rgbd.p_map_depth_raw());
			}

			//printf("flag_load: %d\n", flag_load);

			if(flag_load){

				sw.r_Reset(0);

				//		if(!Kv_Printf("so_Scan_Object")) exit(0);


				//////////////////////////////////////////////////////////////////////////
				// scan object.
				//////////////////////////////////////////////////////////////////////////
				zz_flag_scanning = pDoc->zz_3dos.so_Scan_Object(
					&zz_mat_rgbd,
					false,//m_flag_vh_shield,
					true,//m_flag_on_rgb_cam,
//						KV_TYPE_INPUT_OBJECT,//m_type_of_data,
					KV_TYPE_INPUT_OBJECT,//m_type_of_data,
					KV_TYPE_TRACK_DEPTH,
					1,//m_level_of_pyramid,
					false,
					true,//m_flag_gpu,
					true);
				//false);



 				if(!zz_flag_scanning){
 					printf("=========================== TRACKING FAIL ===========================\n");
 					if(!Kv_Printf("!!!")) exit(0);
 				}
				//printf("zz_flag_scanning: %d\n", zz_flag_scanning);

				// get current scanning state.
				if(zz_flag_scanning){
					//printf("out_ss_OUTput_Scanning_State\n");

					zz_p_state = pDoc->zz_3dos.out_pss_OUTput_Pointer_of_Scanning_State();

					//////////////////////////////////////////////////////////////////////////
					// Display images.
					Invalidate(true);
					//////////////////////////////////////////////////////////////////////////
					//if(!Kv_Printf("Pause (%d)", 0))	exit(0);

					Kv_Pause(1);
				}


				//if(zz_idx_frame == start_frame) if(!Kv_Printf("First frame!")) exit(0);

				zz_idx_frame++;

				total_time += sw.get_Get_Elapsed_Time(0);
				avg_time = total_time/(float)zz_idx_frame;

				printf("  #%d frame was processed... (%5.2f ms)\n",zz_idx_frame,avg_time*1000.0f);

			} else{
				// STOP.
				zz_flag_player = 0;
			}
		}

		//if(!Kv_Printf("pause")) exit(0);

		// pause.
		if(zz_flag_player==2){
			while(zz_flag_player==2) Kv_Pause(1);
		}

		//if(!Kv_Printf("stop1")) exit(0);
		// stop.
		if(zz_flag_player==0){
			//Invalidate(true);
			zz_idx_frame = 0;
			break;
		}
		//if(!Kv_Printf("stop2")) exit(0);
	}

	zz_flag_player = 0;
}





BOOL CVCL_Volume_Scanner_View::OnEraseBkgnd(CDC* pDC)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.

	//return __super::OnEraseBkgnd(pDC);
	return false;
}


void CVCL_Volume_Scanner_View::OnRenderTSDFwithPCL()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
 	CVCL_Volume_Scanner_Doc *pDoc = GetDocument();
 
 
 	return;
 
 error:
 	zpme("OnRenderTSDFwithPCL");
}

void CVCL_Volume_Scanner_View::OnRenderScene()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	LGKvRendererTSDF g_render;
	GKvObjectCubeFloat g_cube;

	float *map_normal;
	float *p_map_depth;
	uchar *p_img_normal,*p_img_color;

	Vector3f light;
	/////////////////////////////////////////////
	CKvYooji_FrameCapture zz_capture;
	LCKvYooji_Scanner_Display disp;
	//CKvYooji_Scanner_Parameter params;
	CKvYooji_Cube_TSDF_Float tsdf_cube;

	CKvYooji_Tracking_State state;
	CKvMatrixUcharRgb img_render;

	CKvYooji_Extrinsics view_change,t_pose;
	CKvYooji_Extrinsics *p_pose;

	float *p_rot_obj;
	float rot_I[9],t_coor[3],t_view[3],t_key[3];
	//CKvPmatrix3D pmat;

	LCKvIO io;
	CKvString dn;
	CKvStopWatch sw;	float elapsed_t,avg_t;	int cnt;

	CKvPoint3Df origin,center;
	// 
	CKvScreen sc[2];

	io.gfn_Get_Folder_Name("Open folder name for loading TSDF cube.",dn);
	zz_capture.ltsdfc_Load_TSDF_Cube(dn,tsdf_cube);

	//int ww, hh;
	int ww_c,hh_c,dd_c;

	tsdf_cube.ts(ww_c,hh_c,dd_c);
	tsdf_cube.goiw_Get_Origin_In_World(origin);
	tsdf_cube.gcciw_Get_Cube_Center_In_World(center);

	// for object.
	t_coor[0] = center.x;	t_coor[1] = center.y;	t_coor[2] = center.z;
	// for scene (cube center is almost the same with camera center.)
	//t_coor[0] = 0.0f;	t_coor[1] = 0.0f;	t_coor[2] = 3.0f;

	if(!Kv_Printf("Loading TSDF cube is complete! [Host]"))	exit(0);

	// ==========================================
	// Host setting.
	// ==========================================
	float fx,fy,px,py,fx_prev,fy_prev;
	int ww,hh;
	ww = 800;	hh = 600;
	fx = fy = 400.0f;	px = 300.0f; py = 300.0f;
	// 	params.sp_Set_Parameters(ww_c, hh_c, dd_c,
	// 		tsdf_cube.elsc_Edge_Length_of_Sub_Cube(),
	// 		tsdf_cube.svm_Size_of_Voxel_in_Meter(),
	// 		origin,
	// 		KV_TH_DIST_SQ_ICP,
	// 		KV_MU_TSDF);

	p_img_color = img_render.c_Create(hh,ww)[0];
	state.initialize(ww,hh);
	state.p_intrinsics_depth()->set_from_params(fx,fy,px,py);
	p_pose = state.p_extrinsics_glob_to_cam();

	// initialize screen.
	sc[0].s_d_Display(state.p_image_normals_rendered());
	sc[0].s_smbm_Set_Mouse_Button_Mode(false);
	sc[0].s_scm_Set_Character_Mode(false);

	// ==========================================
	// Device setting.
	// ==========================================
	float *test_tsdf_dev;
	float *T_gc_dev,*T_cg_dev;

	//////////////////////////////////////////////////////////////////////////
	float mu = 0.04;
	//////////////////////////////////////////////////////////////////////////


	map_normal = new float[3*hh*ww];
	p_map_depth = state.p_map_depth_rendered()->vp();
	p_img_normal = state.p_image_normals_rendered()->vp();

	// import object cube.
	g_cube.from_host(
	tsdf_cube.pvd_Pointer_of_Volume_Depth()->vp(),
	tsdf_cube.pvwd_Pointer_of_Volume_Weight_Depth()->vp(),
	tsdf_cube.pvr_Pointer_of_Volume_Rgb()->vp(),//NULL,
	Vector3i(ww_c,hh_c,dd_c),
	Vector3f(origin.x,origin.y,origin.z),
	Vector3f(center.x,center.y,center.z),
	tsdf_cube.svm_Size_of_Voxel_in_Meter());

	//	g_cube.release();

	// 	printf("vs: %f\n", tsdf_cube.svm_Size_of_Voxel_in_Meter());
	// 	printf("origin: %f %f %f\n", g_cube.origin().x, g_cube.origin().y, g_cube.origin().z);
	// 	printf("center: %f %f %f\n", g_cube.center().x, g_cube.center().y, g_cube.center().z);

	if(!Kv_Printf("Import TSDF cube is complete! [Device]"))	exit(0);

	// initialize parameters.
	g_render.ip_Initialize_Parameters(
		&g_cube,ww,hh,
		fx,fy,px,py,
		mu);
	cudaMalloc((void**)&T_gc_dev,16 * sizeof(float));
	cudaMalloc((void**)&T_cg_dev,16 * sizeof(float));

	// create object cube.
	//  	cudaMalloc((void**)&test_tsdf_dev, 256*256*256* sizeof(float));
	//  	cudaMemcpy(test_tsdf_dev, tsdf_cube.pvd_Pointer_of_Volume_Depth()->vp(),
	//  		256*256*256* sizeof(float), cudaMemcpyHostToDevice);

	// create ICP maps.
	float *map_depth_dev;
	uchar *img_normal_dev,*img_color_dev;
	float *map_normal_dev;

	cudaMalloc((void**)&map_depth_dev,ww * hh * sizeof(float));
	cudaMalloc((void**)&map_normal_dev,3* ww * hh * sizeof(float));
	cudaMalloc((void**)&img_normal_dev,ww * hh * sizeof(uchar));
	cudaMalloc((void**)&img_color_dev,3*ww * hh * sizeof(uchar));

	// ==========================================
	// Mouse control setting.
	// ==========================================
	CKvPixel cur_pix,prev_pix;
	bool flag_rotation,flag_first = true;
	int cur_mode,prev_mode;
	float azimuth_angle,polar_angle,step_sz_rot,step_sz_trans,step_sz_focal;
	float azimuth_angle_prev,polar_angle_prev;
	float azimuth_delta,polar_delta;
	float t_prev[3],t_delta[3];

	bool mode_texture = false, mode_texture_prev;

	// set initial mouse and camera state.
	cur_mode = prev_mode = 0;

	// ==========================================
	// Keyboard control setting.
	// ==========================================
	char cur_char;

	// ========================================
	// Angle step per one pixel.
	step_sz_rot = 0.2f; step_sz_trans = 0.1f; step_sz_focal = 40.0f;
	// ========================================
	azimuth_angle = polar_angle = 0.0f;
	azimuth_angle_prev = polar_angle_prev = -1000000;

	t_delta[0] = t_delta[1] = t_delta[2] = 0.0f;
	t_prev[0] = t_prev[1] = t_prev[2] = -1000000.0f;

	fx_prev = fx; fy_prev = fy;

	sw.c_Create(1);	elapsed_t = avg_t = 0.0f;	cnt = 0;

	while(1){

		// ==========================================
		// Update mouse control.
		// ==========================================
		// get current mouse state.
		cur_pix = sc[0].s_gmps_Get_Mouse_Position_and_Status(cur_mode);
		cur_char = sc[0].s_gc_Get_Character();

		if(cur_char =='m'){
			mode_texture = !mode_texture;
			Kv_Pause(100);
		}

		// ===============================================
		// rotation
		// ===============================================	
		flag_rotation = false;
		polar_delta = azimuth_delta = 0.0f;
		if(prev_mode == 1 && prev_mode == cur_mode){
			// change azimuth angle. ( delta > 0 --> move to right )
			int delta_x,delta_y;

			if(cur_pix.x != prev_pix.x){
				delta_x = cur_pix.x - prev_pix.x;
				azimuth_angle = step_sz_rot*(float)delta_x;
			}
			// change polar angle. ( delta < 0 --> move up )
			if(cur_pix.y != prev_pix.y){
				delta_y = cur_pix.y - prev_pix.y;
				polar_angle = - step_sz_rot*(float)delta_y;
			}

			// select one direction of rotation.
			if(abs(delta_x) >= abs(delta_y)){
				// azimuth angle range is -180 ~ 180.
				// initial azimuth angle is 0 degree.
				azimuth_delta = float(azimuth_angle)*PI/180.0f;
			} else{
				// polar angle range is -90 ~ 90.
				// initial polar angle is 0 degree.
				polar_delta = float(polar_angle)*PI/180.0f;
			}

			flag_rotation = true;
		}

		// ===============================================
		// translation
		// ===============================================
		int mode_trans = 0;
		for(int i=0;i<3;i++){
			t_delta[i] = 0.0f;
			t_key[i] = 0.0f;
		}

		// in y-axis.
		// 		if(prev_mode == -1 && prev_mode == cur_mode){
		// 			// change azimuth angle. ( delta > 0 --> move to right )
		// 			if(cur_pix.x != prev_pix.x) azimuth_angle = azimuth_angle_prev + step_sz_rot*(cur_pix.x - prev_pix.x);
		// 			// change polar angle. ( delta < 0 --> move up )
		// 			if(cur_pix.y != prev_pix.y) polar_angle = polar_angle_prev - step_sz_rot*(cur_pix.y - prev_pix.y);
		// 		}
		// in x-z plane.		 
		if(cur_char=='w')	mode_trans = 1;	   // forward direction.
		else if(cur_char=='s') mode_trans = 2; // backward direction.
		else if(cur_char=='a') mode_trans = 3; // left direction.
		else if(cur_char=='d') mode_trans = 4; // right direction.
		else if(cur_char=='q') break;


		if(mode_trans > 0){


			if(mode_trans == 1){  // +z
				t_key[2] = -step_sz_trans;
			} else if(mode_trans == 2){ // -z
				t_key[2] = step_sz_trans;
			} else if(mode_trans == 3){ // -x
				t_key[0] = step_sz_trans;
			} else{ // +x
				t_key[0] = -step_sz_trans;
			}

			//Vector3f opt_axis,x_axis,z_axis;
			CKvPoint3Df z_axis_world,x_axis_world;
			CKvPoint3Df x_axis_start,x_axis_end,x_axis_start_world,x_axis_end_world;


			float *p_T = state.p_extrinsics_glob_to_cam()->mp_transform()->vp();

			// 			x_axis_start.x = 0.0f; x_axis_start.y = 0.0f; x_axis_start.z = 0.0f;
			// 			x_axis_end.x = 1.0f; x_axis_end.y = 0.0f; x_axis_end.z = 0.0f;

			// 			p_pose->t_Transform(x_axis_start, x_axis_start_world);
			// 			p_pose->t_Transform(x_axis_end, x_axis_end_world);
			// 
			// 			x_axis_world.x = x_axis_end_world.x - x_axis_start_world.x;
			// 			x_axis_world.y = x_axis_end_world.y - x_axis_start_world.y;
			// 			x_axis_world.z = x_axis_end_world.z - x_axis_start_world.z;
			// 			x_axis_world.n_Normalize();
			// 
			// 			z_axis_world.x = p_T[8]; z_axis_world.y = p_T[9]; z_axis_world.z = p_T[10];
			// 			z_axis_world.n_Normalize();

			//////////////////////////////////////////////////////////////////////////
			// 이게 정답...
			x_axis_world.x = 1.0f; x_axis_world.y = 0.0f; x_axis_world.z = 0.0f;
			z_axis_world.x = 0.0f; z_axis_world.y = 0.0f; z_axis_world.z = 1.0f;
			//////////////////////////////////////////////////////////////////////////

			// compute vectors in global coord. of z-axis and x-axis.				

			// 			printf("x_axis: %f %f %f\n", x_axis.x, x_axis.y, x_axis.z);
			// 			printf("z_axis: %f %f %f\n", z_axis.x, z_axis.y, z_axis.z);
			// 
			// 			printf("%f %f\n", t_key[0], t_key[2]);
			//			printf("mode_trans: %d\n", mode_trans);

			// change translation.
			// + x-axis and z-axis.
			t_delta[0] = t_key[0]*x_axis_world.x + t_key[2]*z_axis_world.x;
			t_delta[1] = t_key[0]*x_axis_world.y + t_key[2]*z_axis_world.y;
			t_delta[2] = t_key[0]*x_axis_world.z + t_key[2]*z_axis_world.z;
		}

		// ===============================================
		// zooming
		// ===============================================
		// 		if(prev_mode == -1 && prev_mode == cur_mode){
		// 
		// 			// change polar angle. ( delta < 0 --> move up )
		// 			if(cur_pix.y > prev_pix.y){
		// 				fx = max(10.0f,fx_prev - step_sz_focal);
		// 				fy = max(10.0f,fy_prev - step_sz_focal);
		// 			} else if(cur_pix.y < prev_pix.y){
		// 				fx = max(10.0f,fx_prev + step_sz_focal);
		// 				fy = max(10.0f,fy_prev + step_sz_focal);
		// 			}
		// 
		// 			// change focal length.
		// 			state.pi_Pointer_of_Intrinsics()->sp_Set_from_Parameters(fx,fy,px,py);
		// 
		// 			g_render.ip_Initialize_Parameters(
		// 				&g_cube,ww,hh,
		// 				fx,fy,px,py,
		// 				mu);
		// 
		// 			//	printf("%f %f \n", fx , fy);
		// 		}

		// left button clicked.
		prev_pix = cur_pix;
		// save current mouse state as previous state.
		prev_mode = cur_mode;
		Kv_Pause(1);

		//////////////////////////////////////////////////////////////////////////
		if(!flag_first && !flag_rotation && mode_trans==0 && mode_texture == mode_texture_prev)	continue;
		//////////////////////////////////////////////////////////////////////////

		sw.r_Reset(0);

		// ===================================================
		// Update current view points.
		// ===================================================
		// set view change.
		view_change.set_from_params(polar_delta,azimuth_delta,0.0f,t_delta[0],t_delta[1],t_delta[2]);
		view_change.get_pose_successive(*p_pose,view_change,
			t_pose);
		p_pose->set_from_transform(t_pose.mp_transform());

		state.compute_P_matrix(
			state.p_intrinsics_depth(),
			p_pose,
			state.p_P_matrix_depth());

		// + for translation.
		// 		float *p_T = state.pt3d_Pointer_of_3D_Transform_Global_to_Camera()->mp_Matrix_Pointer()->vp();
		// 		Vector3f opt_axis;
		// 		opt_axis.x = p_T[8]; opt_axis.y = p_T[9]; opt_axis.z = p_T[10]; opt_axis.normalised();
		//d_pm_Printf_Matrix(state.pp_Pointer_of_Pmatrix()->mpp()->vp(), 3, 4, "P");

		// =============================================
		// GPU device function
		// =============================================
		// Update extrinsic parameters.
		CKvPoint3Df cam_cen_k;	Vector3f cam_cen;
		double *p_pmat = state.p_P_matrix_depth()->mpp()->vp();
		float *p_T_gc = p_pose->mp_transform()->vp();
		float *p_T_cg = p_pose->mp_transform_inv()->vp();

		// camera pose.
		cudaMemcpy(T_gc_dev,p_T_gc,16 * sizeof(float),cudaMemcpyHostToDevice);
		cudaMemcpy(T_cg_dev,p_T_cg,16 * sizeof(float),cudaMemcpyHostToDevice);
		// camera center.
		p_pose->get_cam_center(cam_cen_k);
		cam_cen.x = cam_cen_k.x; cam_cen.y = cam_cen_k.y; cam_cen.z = cam_cen_k.z;
		// set light vector.		
		light.x = -p_pmat[8];	light.y = -p_pmat[9];	light.z = -p_pmat[10];	light.normalised();

		// Render ICP maps.
		//////////////////////////////////////////////////////////////////////////
		// if input TSDF cube is scene data.
		g_render.rmis_Render_Maps_for_Scene(
			&g_cube,
			ww,hh,
			T_gc_dev,T_cg_dev,
			//p_T_gc, p_T_cg,
			cam_cen,light,

			map_depth_dev,
			map_normal_dev,
			img_normal_dev,
			img_color_dev);

		//////////////////////////////////////////////////////////////////////////


		// update rendering results.
		//cudaMemcpy(p_map_depth, map_depth_dev, ww * hh * sizeof(float), cudaMemcpyDeviceToHost);
		//cudaMemcpy(map_normal, map_normal_dev, 3*ww * hh * sizeof(float), cudaMemcpyDeviceToHost);
		cudaMemcpy(p_img_normal,img_normal_dev,ww * hh * sizeof(uchar),cudaMemcpyDeviceToHost);
		cudaMemcpy(p_img_color,img_color_dev,3*ww * hh * sizeof(uchar),cudaMemcpyDeviceToHost);

		if(!mode_texture) sc[0].s_d_Display(state.p_image_normals_rendered());
		else			  sc[0].s_d_Display(&img_render);

		cnt++;
		elapsed_t += sw.get_Get_Elapsed_Time(0);
		avg_t = elapsed_t/cnt;

		Kv_Pause(1);

		// 		azimuth_angle_prev = azimuth_angle;
		// 		polar_angle_prev = polar_angle;
		// 
		// 		t_prev[0] = t_delta[0];
		// 		t_prev[1] = t_delta[1];
		// 		t_prev[2] = t_delta[2];

		fx_prev = fx; fy_prev = fy;

		mode_texture_prev = mode_texture;

		// 		printf("Azimuth: %5.2f, Polar: %5.2f [%5.2f fps / %5.2f ms]                        \r",
		// 			azimuth_angle,polar_angle,1.0f/avg_t,1000.0f*avg_t);

		if(flag_first) flag_first = false;



	}
	printf("Exit\n");


	//if(!Kv_Printf("Rendered normal."))	exit(0);
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

	g_cube.release();
	if(!Kv_Printf("Release TSDF cube is complete! [Device]"))	exit(0);

	//cudaFree(test_tsdf_dev);

	cudaFree(map_depth_dev);
	cudaFree(map_normal_dev);
	cudaFree(img_normal_dev);
	cudaFree(img_color_dev);

	cudaFree(T_gc_dev);
	cudaFree(T_cg_dev);

	delete map_normal;

	return;

	return ;

}

void CVCL_Volume_Scanner_View::OnRenderObject()
{
	LGKvRendererTSDF g_render;
	GKvObjectCubeFloat g_cube;

	float *map_normal;
	float *p_map_depth;
	uchar *p_img_normal,*p_img_color;

	Vector3f light;
	/////////////////////////////////////////////
	CKvYooji_FrameCapture zz_capture;
	LCKvYooji_Scanner_Display disp;
	//CKvYooji_Scanner_Parameter params;
	CKvYooji_Cube_TSDF_Float tsdf_cube;

	CKvYooji_Tracking_State state;
	CKvMatrixUcharRgb img_render;

	CKvYooji_Extrinsics view_change,rot_obj;
	CKvYooji_Extrinsics *p_pose;

	float *p_rot_obj;
	float rot_I[9],t_coor[3],t_view[3],t_key[3];
	//CKvPmatrix3D pmat;

	LCKvIO io;
	CKvString dn;
	CKvStopWatch sw;	float elapsed_t,avg_t;	int cnt;

	CKvPoint3Df origin,center;
	// 
	CKvScreen sc[2];

	io.gfn_Get_Folder_Name("Open folder name for loading TSDF cube.",dn);
	zz_capture.ltsdfc_Load_TSDF_Cube(dn,tsdf_cube);

	//int ww, hh;
	int ww_c,hh_c,dd_c;

	tsdf_cube.ts(ww_c,hh_c,dd_c);
	tsdf_cube.goiw_Get_Origin_In_World(origin);
	tsdf_cube.gcciw_Get_Cube_Center_In_World(center);

	// for object.
	t_coor[0] = center.x;	t_coor[1] = center.y;	t_coor[2] = center.z;
	// for scene (cube center is almost the same with camera center.)
	//t_coor[0] = 0.0f;	t_coor[1] = 0.0f;	t_coor[2] = 3.0f;

	if(!Kv_Printf("Loading TSDF cube is complete! [Host]"))	exit(0);

	// ==========================================
	// Host setting.
	// ==========================================
	float fx,fy,px,py,fx_prev,fy_prev;
	int ww,hh;
	ww = 800;	hh = 800;
	fx = fy = 800.0f;	px = 400.0f; py = 400.0f;
	// 	params.sp_Set_Parameters(ww_c, hh_c, dd_c,
	// 		tsdf_cube.elsc_Edge_Length_of_Sub_Cube(),
	// 		tsdf_cube.svm_Size_of_Voxel_in_Meter(),
	// 		origin,
	// 		KV_TH_DIST_SQ_ICP,
	// 		KV_MU_TSDF);

	p_img_color = img_render.c_Create(hh,ww)[0];
	state.initialize(ww,hh);
	state.p_intrinsics_depth()->set_from_params(fx,fy,px,py);
	p_pose = state.p_extrinsics_glob_to_cam();

	// initialize screen.
	sc[0].s_d_Display(state.p_image_depth_rendered());
	sc[0].s_smbm_Set_Mouse_Button_Mode(false);
	sc[0].s_scm_Set_Character_Mode(false);

	// ==========================================
	// Device setting.
	// ==========================================
	float *test_tsdf_dev;
	float *T_gc_dev,*T_cg_dev;

	//////////////////////////////////////////////////////////////////////////
	float mu = 0.0005;
	//////////////////////////////////////////////////////////////////////////


	map_normal = new float[3*hh*ww];
	p_map_depth = state.p_map_depth_rendered()->vp();
	p_img_normal = state.p_image_normals_rendered()->vp();

	// import object cube.
	g_cube.from_host(
	tsdf_cube.pvd_Pointer_of_Volume_Depth()->vp(),
	tsdf_cube.pvwd_Pointer_of_Volume_Weight_Depth()->vp(),
	tsdf_cube.pvr_Pointer_of_Volume_Rgb()->vp(),//NULL,
	Vector3i(ww_c,hh_c,dd_c),
	Vector3f(origin.x,origin.y,origin.z),
	Vector3f(center.x,center.y,center.z),
	tsdf_cube.svm_Size_of_Voxel_in_Meter());

	//	g_cube.release();

	// 	printf("vs: %f\n", tsdf_cube.svm_Size_of_Voxel_in_Meter());
	// 	printf("origin: %f %f %f\n", g_cube.origin().x, g_cube.origin().y, g_cube.origin().z);
	// 	printf("center: %f %f %f\n", g_cube.center().x, g_cube.center().y, g_cube.center().z);

	if(!Kv_Printf("Import TSDF cube is complete! [Device]"))	exit(0);

	// initialize parameters.
	g_render.ip_Initialize_Parameters(
		&g_cube,ww,hh,
		fx,fy,px,py,
		mu);
	cudaMalloc((void**)&T_gc_dev,16 * sizeof(float));
	cudaMalloc((void**)&T_cg_dev,16 * sizeof(float));

	// create object cube.
	//  	cudaMalloc((void**)&test_tsdf_dev, 256*256*256* sizeof(float));
	//  	cudaMemcpy(test_tsdf_dev, tsdf_cube.pvd_Pointer_of_Volume_Depth()->vp(),
	//  		256*256*256* sizeof(float), cudaMemcpyHostToDevice);

	// create ICP maps.
	float *map_depth_dev;
	uchar *img_normal_dev,*img_color_dev;
	float *map_normal_dev;

	cudaMalloc((void**)&map_depth_dev,ww * hh * sizeof(float));
	cudaMalloc((void**)&map_normal_dev,3* ww * hh * sizeof(float));
	cudaMalloc((void**)&img_normal_dev,ww * hh * sizeof(uchar));
	cudaMalloc((void**)&img_color_dev,3*ww * hh * sizeof(uchar));

	// ==========================================
	// Mouse control setting.
	// ==========================================
	CKvPixel cur_pix,prev_pix;
	int cur_mode,prev_mode;
	float azimuth_angle,polar_angle,step_sz_rot,step_sz_trans,step_sz_focal;
	float azimuth_angle_prev,polar_angle_prev;
	float azimuth,polar;
	
	bool mode_texture = false, mode_texture_prev;

	// set initial mouse and camera state.
	cur_mode = prev_mode = 0;

	// ==========================================
	// Keyboard control setting.
	// ==========================================
	char cur_char;

	// ========================================
	// Angle step per one pixel.
	step_sz_rot = 0.2f; step_sz_trans = 0.05f; step_sz_focal = 40.0f;
	// ========================================
	azimuth_angle = polar_angle = 50.0f;
	azimuth_angle_prev = polar_angle_prev = -1000000;

	fx_prev = fx; fy_prev = fy;

	sw.c_Create(1);	elapsed_t = avg_t = 0.0f;	cnt = 0;

	while(1){

		// ==========================================
		// Update mouse control.
		// ==========================================
		// get current mouse state.
		cur_pix = sc[0].s_gmps_Get_Mouse_Position_and_Status(cur_mode);
		cur_char = sc[0].s_gc_Get_Character();
		if(cur_char =='q') break;

		if(cur_char =='m'){
			mode_texture = !mode_texture;
			Kv_Pause(100);
		}

		// ===============================================
		// rotation
		// ===============================================
		if(prev_mode == 1 && prev_mode == cur_mode){
			// change azimuth angle. ( delta > 0 --> move to right )
			if(cur_pix.x != prev_pix.x) azimuth_angle = azimuth_angle_prev + step_sz_rot*(cur_pix.x - prev_pix.x);
			// change polar angle. ( delta < 0 --> move up )
			if(cur_pix.y != prev_pix.y) polar_angle = polar_angle_prev - step_sz_rot*(cur_pix.y - prev_pix.y);
		}

		// ===============================================
		// translation
		// ===============================================
		int mode_trans = 0; for(int i=0;i<3;i++) t_key[i] = 0.0f;

		// in y-axis.
		// 		if(prev_mode == -1 && prev_mode == cur_mode){
		// 			// change azimuth angle. ( delta > 0 --> move to right )
		// 			if(cur_pix.x != prev_pix.x) azimuth_angle = azimuth_angle_prev + step_sz_rot*(cur_pix.x - prev_pix.x);
		// 			// change polar angle. ( delta < 0 --> move up )
		// 			if(cur_pix.y != prev_pix.y) polar_angle = polar_angle_prev - step_sz_rot*(cur_pix.y - prev_pix.y);
		// 		}
		//  		// in x-z plane.		 
		//   		if(cur_char=='w')	mode_trans = 1;	   // forward direction.
		//   		else if(cur_char=='s') mode_trans = 2; // backward direction.
		//   		else if(cur_char=='a') mode_trans = 3; // left direction.
		//   		else if(cur_char=='d') mode_trans = 4; // right direction.
		//  
		//  		if(mode_trans > 0){
		//  			if(mode_trans = 1){  // +z
		//  				t_key[2] = step_sz_trans;
		//  			}
		//  			else if(mode_trans = 2){ // -z
		//  				t_key[2] = -step_sz_trans;
		//  			}
		//  			else if(mode_trans = 3){ // -x
		//  				t_key[0] = -step_sz_trans;
		//  			}
		//  			else{ // +x
		//  				t_key[0] = step_sz_trans;
		//  			}
		//  		}

		// ===============================================
		// zooming
		// ===============================================
		if(prev_mode == -1 && prev_mode == cur_mode){

			// change polar angle. ( delta < 0 --> move up )
			if(cur_pix.y > prev_pix.y){
				fx = max(10.0f,fx_prev - step_sz_focal);
				fy = max(10.0f,fy_prev - step_sz_focal);
			} else if(cur_pix.y < prev_pix.y){
				fx = max(10.0f,fx_prev + step_sz_focal);
				fy = max(10.0f,fy_prev + step_sz_focal);
			}

			// change focal length.
			state.p_intrinsics_depth()->set_from_params(fx,fy,px,py);

			g_render.ip_Initialize_Parameters(
				&g_cube,ww,hh,
				fx,fy,px,py,
				mu);

			//	printf("%f %f \n", fx , fy);
		}

		// left button clicked.
		prev_pix = cur_pix;
		// save current mouse state as previous state.
		prev_mode = cur_mode;
		Kv_Pause(1);

		if(azimuth_angle == azimuth_angle_prev && polar_angle == polar_angle_prev
			&& mode_trans==0
			&& fx == fx_prev && fy == fy_prev
			&& mode_texture == mode_texture_prev)	continue;

		sw.r_Reset(0);

		// ===================================================
		// Update current view points.
		// ===================================================
		// azimuth angle range is -180 ~ 180.
		// initial azimuth angle is 0 degree.
		azimuth = float(azimuth_angle - 50)*(360.0f/100.0f)*PI/180.0f;
		// polar angle range is -90 ~ 90.
		// initial polar angle is 0 degree.
		polar = float(polar_angle - 50)*(180.0f/100.0f)*PI/180.0f;

		// view rotation.
		rot_obj.set_from_params(polar,azimuth,0.0f,0.0f,0.0f,0.0f);
		p_rot_obj = rot_obj.mp_rotation()->vp();

		// view translation.
		// t_view = (I - r_obj)*t_coor.
		for(int i = 0; i<9; i++)	rot_I[i] = (i%3==i/3) ? 1.0f - p_rot_obj[i] : -p_rot_obj[i];
		d_mmsv_Multiply_Matrix_Square_Vector(rot_I,t_coor,3,t_view);

		//////////////////////////////////////////////////////////////////////////
		// 		t_view[0] += -3.0f*t_coor[0];
		// 		t_view[1] += -3.0f*t_coor[1];
		//		t_view[2] += -3.0f;
		//////////////////////////////////////////////////////////////////////////


		// t_key
		//for(int i=0; i<3; i++) t_view[i] += t_key[i];

		// set view change.
		// + for rotation.
		view_change.set_from_params(polar,azimuth,0.0f,t_view[0],t_view[1],t_view[2]);
		//view_change.sp_Set_from_Parameters(polar, azimuth, 0.0f, 0.0f, 0.0f, 0.0f);
		p_pose->copy(&view_change);

		state.compute_P_matrix(
			state.p_intrinsics_depth(),
			state.p_extrinsics_glob_to_cam(),
			state.p_P_matrix_depth());

		// + for translation.
		// 		float *p_T = state.pt3d_Pointer_of_3D_Transform_Global_to_Camera()->mp_Matrix_Pointer()->vp();
		// 		Vector3f opt_axis;
		// 		opt_axis.x = p_T[8]; opt_axis.y = p_T[9]; opt_axis.z = p_T[10]; opt_axis.normalised();
		//d_pm_Printf_Matrix(state.pp_Pointer_of_Pmatrix()->mpp()->vp(), 3, 4, "P");

		// =============================================
		// GPU device function
		// =============================================
		// Update extrinsic parameters.
		CKvPoint3Df cam_cen_k;	Vector3f cam_cen;
		double *p_pmat = state.p_P_matrix_depth()->mpp()->vp();
		float *p_T_gc = p_pose->mp_transform()->vp();
		float *p_T_cg = p_pose->mp_transform_inv()->vp();

		// camera pose.
		cudaMemcpy(T_gc_dev,p_T_gc,16 * sizeof(float),cudaMemcpyHostToDevice);
		cudaMemcpy(T_cg_dev,p_T_cg,16 * sizeof(float),cudaMemcpyHostToDevice);
		// camera center.
		p_pose->get_cam_center(cam_cen_k);
		cam_cen.x = cam_cen_k.x; cam_cen.y = cam_cen_k.y; cam_cen.z = cam_cen_k.z;
		// set light vector.		
		light.x = -p_pmat[8];	light.y = -p_pmat[9];	light.z = -p_pmat[10];	light.normalised();

		// Render ICP maps.
		//////////////////////////////////////////////////////////////////////////
		g_render.rmi_Render_Maps_for_ICP(
			&g_cube,
			ww,hh,
			T_gc_dev,T_cg_dev,
			//p_T_gc, p_T_cg,
			cam_cen,light,

			0,

			map_depth_dev,
			map_normal_dev,
			img_normal_dev,

			img_color_dev);

		//////////////////////////////////////////////////////////////////////////


		// update rendering results.
		//cudaMemcpy(p_map_depth, map_depth_dev, ww * hh * sizeof(float), cudaMemcpyDeviceToHost);
		//cudaMemcpy(map_normal, map_normal_dev, 3*ww * hh * sizeof(float), cudaMemcpyDeviceToHost);
		cudaMemcpy(p_img_normal,img_normal_dev,ww * hh * sizeof(uchar),cudaMemcpyDeviceToHost);
		cudaMemcpy(p_img_color,img_color_dev,3*ww * hh * sizeof(uchar),cudaMemcpyDeviceToHost);

		if(!mode_texture) sc[0].s_d_Display(state.p_image_normals_rendered());
		else			  sc[0].s_d_Display(&img_render);

		cnt++;
		elapsed_t += sw.get_Get_Elapsed_Time(0);
		avg_t = elapsed_t/cnt;

		Kv_Pause(1);

		azimuth_angle_prev = azimuth_angle;
		polar_angle_prev = polar_angle;

		fx_prev = fx; fy_prev = fy;

		mode_texture_prev = mode_texture;

		printf("Azimuth: %5.2f, Polar: %5.2f [%5.2f fps / %5.2f ms]                        \r",
			azimuth_angle,polar_angle,1.0f/avg_t,1000.0f*avg_t);



	}
	printf("Exit\n");


	//if(!Kv_Printf("Rendered normal."))	exit(0);
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

	g_cube.release();
	if(!Kv_Printf("Release TSDF cube is complete! [Device]"))	exit(0);

	//cudaFree(test_tsdf_dev);

	cudaFree(map_depth_dev);
	cudaFree(map_normal_dev);
	cudaFree(img_normal_dev);
	cudaFree(img_color_dev);

	cudaFree(T_gc_dev);
	cudaFree(T_cg_dev);

	delete map_normal;

	return;

}

void CVCL_Volume_Scanner_View::OnTestRgbdOdometryOpencv()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	CVCL_Volume_Scanner_Doc *pDoc = GetDocument();
	CKvYooji_InterLib_Convert il_convt;

	std::vector<String> inputRGBPaths;
	std::vector<String> inputDepthPaths;

	string path_rgb = "D:/Documents/_data/2016_Object_Scanning/Real/TUM_RGBD_Bench/TUM_fr2_desk/rgb/*.png";
	string path_depth = "D:/Documents/_data/2016_Object_Scanning/Real/TUM_RGBD_Bench/TUM_fr2_desk/depth/*.png";
	
	cv::glob(path_rgb,inputRGBPaths,false);
	cv::glob(path_depth,inputDepthPaths,false);

	
	namedWindow("RGBD Color",WINDOW_AUTOSIZE);
	namedWindow("Normalized RGBD Depth",WINDOW_AUTOSIZE);
	namedWindow("RGBD Trajectory",WINDOW_AUTOSIZE);
	Mat traj = Mat::zeros(WINDOW_SIZE,WINDOW_SIZE,CV_8UC3);

	std::shared_ptr<rgbd::Odometry> odom;

	char text[100];
	int fontFace = FONT_HERSHEY_PLAIN;
	double fontScale = 1;
	int thickness = 1;
	cv::Point textOrg(10,50);

	Mat rotationMatrix,tranlslationMatrix, rigidtransformMatrix;

	float vals[] ={FX,0.,CX,0.,FY,CY,0.,0.,1.};

	const Mat cameraMatrix = Mat(3,3,CV_32FC1,vals);

	bool isFirst = true;
	//////////////////////////////////////////////////////////////////////////
	// Save canonical camera matrix as pose of the 1st frame.
	CKvYooji_Extrinsics *p_ext = new CKvYooji_Extrinsics;
	pDoc->zz_3dos.p_trace_estimated()->push_back(p_ext);
	//////////////////////////////////////////////////////////////////////////
	for(int i = 0; i != inputRGBPaths.size() - 1; i++) {
		std::string rgb0Path = inputRGBPaths[i];
		std::string depth0Path = inputDepthPaths[i];

		std::string rgb1Path = inputRGBPaths[i + 1];
		std::string depth1Path = inputDepthPaths[i + 1];

		// Read images start
		Mat colorImage0 = imread(rgb0Path);
		Mat depth0 = imread(depth0Path,IMREAD_UNCHANGED);
		Mat colorImage1 = imread(rgb1Path);
		Mat depth1 = imread(depth1Path,IMREAD_UNCHANGED);

		if(colorImage0.empty() || depth0.empty() || colorImage1.empty() ||
			depth1.empty()) {
			std::cerr << "Not correct RGB-D images";
			return ;
		}

		Mat grayImage0,grayImage1,depthFlt0,depthFlt1 /*in meters*/;
		cvtColor(colorImage0,grayImage0,COLOR_BGR2GRAY);
		cvtColor(colorImage1,grayImage1,COLOR_BGR2GRAY);
		depth0.convertTo(depthFlt0,CV_32FC1,PIXEL_TO_METER_SCALEFACTOR);
		depth1.convertTo(depthFlt1,CV_32FC1,PIXEL_TO_METER_SCALEFACTOR);
		// Read images end

		Mat rigidTransform;

		if(!odom) {
			vector<int> iterCounts(4);
			iterCounts[0] = 7;
			iterCounts[1] = 7;
			iterCounts[2] = 7;
			iterCounts[3] = 10;

			vector<float> minGradMagnitudes(4);
			minGradMagnitudes[0] = 12;
			minGradMagnitudes[1] = 5;
			minGradMagnitudes[2] = 3;
			minGradMagnitudes[3] = 1;

// 			odom = std::make_shared<rgbd::RgbdICPOdometry>(
// 				cameraMatrix,MIN_DEPTH,MAX_DEPTH,MAX_DEPTH_DIFF,MAX_POINTS_PART,
// 				iterCounts,	minGradMagnitudes,
// 				rgbd::Odometry::RIGID_BODY_MOTION);

			odom = std::make_shared<rgbd::RgbdOdometry>(
				cameraMatrix,MIN_DEPTH,MAX_DEPTH,MAX_DEPTH_DIFF,iterCounts,
				minGradMagnitudes,MAX_POINTS_PART,
				rgbd::Odometry::RIGID_BODY_MOTION);


			std::cerr << "Init tracker" << std::endl;
		}

		bool isSuccess = odom->compute(grayImage0,depthFlt0,Mat(),grayImage1,
										depthFlt1,Mat(),rigidTransform);

		if(!isSuccess) if(!Kv_Printf("Fail!")) exit(0);

		Mat rotationMat = rigidTransform(cv::Rect(0,0,3,3)).clone();
		Mat translateMat = rigidTransform(cv::Rect(3,0,1,3)).clone();
		// If compute successfully, then update rotationMatrix and tranlslationMatrix
		if(isSuccess == true) {
			if(isFirst == true) {
				rotationMatrix = rotationMat.clone();
				tranlslationMatrix = translateMat.clone();

				rigidtransformMatrix = rigidTransform.clone();

				isFirst = false;
				continue;
			}

			// Update Rt
			tranlslationMatrix = tranlslationMatrix + (rotationMatrix * translateMat);
			rotationMatrix = rotationMat * rotationMatrix;

			rigidtransformMatrix = rigidTransform * rigidtransformMatrix;
		}

		// Visualize traj
		if(isFirst == false) {
			int x =
				int(VISUALIZATION_SCALE_FACTOR * tranlslationMatrix.at<double>(0)) +
				WINDOW_SIZE / 2;
			int y =
				int(VISUALIZATION_SCALE_FACTOR * tranlslationMatrix.at<double>(2)) +
				WINDOW_SIZE / 2;

			circle(traj,Point(x,y),1,CV_RGB(255,0,0),2);
			rectangle(traj,Point(10,30),Point(550,50),CV_RGB(0,0,0),
						CV_FILLED);
			if(isSuccess == true) {
				sprintf_s(text,"Coordinates: x = %04fm y = %04fm z = %04fm",
						tranlslationMatrix.at<double>(0),
						tranlslationMatrix.at<double>(1),
						tranlslationMatrix.at<double>(2));
			} else {
				sprintf_s(text,"Fail to compute odometry");
			}

			putText(traj,text,textOrg,fontFace,fontScale,Scalar::all(255),
					thickness,8);
		}
		imshow("RGBD Trajectory",traj);
		imshow("RGBD Color",grayImage1);

		cv::Mat normalizeDepth;
		depthFlt1.convertTo(normalizeDepth,CV_8UC1,255.0 / MAX_DEPTH);
		imshow("Normalized RGBD Depth",normalizeDepth);

		const Mat distCoeff(1,5,CV_32FC1,Scalar(0));
		cv::waitKey(1);

		//////////////////////////////////////////////////////////////////////////
		// for saving trajectory.
		CKvYooji_Extrinsics *p_ext = new CKvYooji_Extrinsics;
		CKvMatrixFloat Rt;	// 4x4

		il_convt.cfok_Convert_Format_from_Opencv_to_KAISION(rigidtransformMatrix, Rt);

// 		cout << rigidTransform.type() << endl;
// 		cout << rigidTransform << endl;
// 		d_pm_Printf_Matrix(Rt.vp(),4,4,"T");

		p_ext->set_from_transform(&Rt);
		pDoc->zz_3dos.p_trace_estimated()->push_back(p_ext);
		//////////////////////////////////////////////////////////////////////////
	}

	if(!Kv_Printf("Odometry complete!")) exit(0);
}




void CVCL_Volume_Scanner_View::OnTestGradientFieldDisplay()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	LCKvHello aa_hh;
	LCKvUtility_for_Import aa_im;
	LCKvUtility_for_Windows aa_uw;
	LCKvUtility_for_YCbCr aa_ycc;

	LCKvYooji_Image_Processor yooji_ip;
	CKvYooji_ContourDetection yooji_cd;
	LCKvYooji_Scanner_Display aa_disp;

	CKvScreen zz_sc[3];
	CKvString fn,caption;
	CKvMatrixUcharRgb in_img, in_img_rgb_gray,temp_img;
	CKvMatrixUchar in_img_gray, img_filtered;

	int ww,hh;
	int prev_right,curr_right,prev_sigma;

	aa_hh.io.gof_Get_Open_Filename("Load image",fn);
	aa_hh.iofv.li_Load_Image(fn,false,&in_img);
	aa_ycc.mrg_Matrix_Rgb_to_Gray(&in_img,false,&in_img_gray);
	aa_ycc.mgr_Matrix_Gray_to_Rgb(&in_img_gray, &in_img_rgb_gray);

	zz_sc[0].s_d_Display(&in_img);
	zz_sc[1].s_d_Display(&in_img_gray);

	caption = "Color";		aa_uw.spt_Set_Position_and_Title(&zz_sc[0],0,0,caption);
	caption = "Gray";		aa_uw.spt_Set_Position_and_Title(&zz_sc[1],&zz_sc[0],0,caption);

	Kv_Printf("Load image!");

	// Gaussian filtering.
	yooji_ip.sig5_Smooth_Image_Gaussian_5x5(&in_img_gray, &img_filtered);

	// extract gradient maps.
	CKvYooji_Edge_Map map_edge;	

	// compute gradients.
	yooji_cd.cig_Compute_Image_Gradients(img_filtered, map_edge.grad_x, map_edge.grad_y);
	// compute gradient magnitude.
	yooji_cd.cmm_Compute_Magnitude_Map(map_edge.grad_x,map_edge.grad_y,map_edge.grad_mag);

	// compute hessian.
	yooji_cd.cig_Compute_Image_Gradients(map_edge.grad_x,map_edge.grad_xx,map_edge.grad_xy);
	yooji_cd.cig_Compute_Image_Gradients(map_edge.grad_y,map_edge.grad_yx,map_edge.grad_yy);
	// compute laplacian.
	yooji_cd.clm_Compute_Laplacian_Map(map_edge.grad_xx, map_edge.grad_yy, map_edge.laplacian);
	//yooji_cd.clm_Compute_Laplacian_Map(img_filtered, map_edge.laplacian);


	//////////////////////////////////////////////////////////////////////////
	//
	//////////////////////////////////////////////////////////////////////////
	// get local template.
	CKvMatrixUcharRgb tmpl_rgb, tmp_rgb_gray, tmpl_rgb_zoomin;
	CKvMatrixUchar tmpl_gray, tmpl_gray_zoomin;
	CKvMatrixFloat tmpl_grad_x, tmpl_grad_y, tmpl_grad_mag, tmpl_laplace;
	CKvVectorFloat trow_tmpl, tcol_tmpl;
	CKvVectorUchar tvec_uchar;
	CKvSet2d_of_Pointf set_of_grads;

	CKvGraph_Line gline[2];
	CKvPen pen_red;		pen_red.sc_Set_Color(Kv_Rgb(255, 0, 0));	pen_red.sw_Set_Width(2);
	CKvPen pen_blue;	pen_blue.sc_Set_Color(Kv_Rgb(0,0,255));		pen_blue.sw_Set_Width(2);
	CKvPen pen_black;	pen_black.sc_Set_Color(Kv_Rgb(0,0,0));		pen_black.sw_Set_Width(2);


	CKvPointf *p_tp2d;
	CKvPixel tpix;
	int state;
	char in_c;

	int block_sz_half = 15;
	int block_sz = 2*block_sz_half + 1;
	int order_magnif = 4;	// magnification ratio = 2^(order_magnif)

	int tx,ty;

	in_img_gray.ms(ww, hh);
	set_of_grads.c_Create(block_sz, block_sz);

	// graph setting.
	CKvFont font_for_title, font_for_title_for_axes, font_for_scales, font_for_legends;
	CKvPen pen_for_drawing_axes, pen_for_drawing_grids;
	int in_length_of_lines_for_legends;

	gline[0].g_gdt_Get_Drawing_Tools(font_for_title, font_for_title_for_axes, font_for_scales, font_for_legends,
		pen_for_drawing_axes, pen_for_drawing_grids, in_length_of_lines_for_legends);

	for(int i=0; i<2; i++){
		gline[i].g_sdt_Set_Drawing_Tools(font_for_title,font_for_title,font_for_scales,font_for_legends,
		pen_for_drawing_axes,pen_for_drawing_grids,in_length_of_lines_for_legends);

		gline[i].g_sgi_Set_Graph_Information(0, block_sz, -100, 200, 1, 1);
	}

	while(1){		
		
		tpix = zz_sc[0].s_gmps_Get_Mouse_Position_and_Status(state);

		tx = tpix.x - block_sz_half; ty = tpix.y - block_sz_half;

		if(tx >= 0 && ty >= 0 && tx < ww && ty < hh){

			// Get ROI template of intensity image.
			//in_img.gb_Get_Block(tx, ty, block_sz, block_sz, &tmpl_rgb);
			in_img_rgb_gray.gb_Get_Block(tx, ty, block_sz, block_sz, &tmpl_rgb);
			in_img_gray.gb_Get_Block(tx, ty, block_sz, block_sz, &tmpl_gray);

			// Get ROI template of gradient maps.
			map_edge.grad_x.gb_Get_Block(tx,ty,block_sz,block_sz,&tmpl_grad_x);
			map_edge.grad_y.gb_Get_Block(tx,ty,block_sz,block_sz,&tmpl_grad_y);
			map_edge.grad_mag.gb_Get_Block(tx,ty,block_sz,block_sz,&tmpl_grad_mag);

			// Get ROI template of Laplacian maps.
			map_edge.laplacian.gb_Get_Block(tx,ty,block_sz,block_sz,&tmpl_laplace);

			// Get ROI template of Hessian maps.
			map_edge.grad_xx.gb_Get_Block(tx,ty,block_sz,block_sz,&tmpl_laplace);

			// Set 2d gradient vectors.
			for(int j=0; j<block_sz; j++){
				for(int i=0; i<block_sz; i++){
				
					p_tp2d = set_of_grads.gpe_Get_Pointer_of_Element(i, j);
					
					p_tp2d->x = tmpl_grad_x.ge_Get_Element(i, j);
					p_tp2d->y = tmpl_grad_y.ge_Get_Element(i, j);

				}
			}

			// Zoom in the template.
			for(int i=0; i<order_magnif; i++)	tmpl_rgb.gzi_Get_Zoom_In(&tmpl_rgb);

			// Display zoomed-in template,
			//zz_sc[1].s_d_Display(&tmpl_rgb);
			int ratio_mag = pow(2, order_magnif);
			aa_disp.dtfsc_Display_Tracked_Flow_Pseudo_Color(
				&zz_sc[1],
				&tmpl_rgb,
				&set_of_grads,
				ratio_mag);
			aa_uw.sp_Set_Position(&zz_sc[1], &zz_sc[0], 0);

			

			//////////////////////////////////////////////////////////////////////////
			// x-direction.
			// Intensity graph.
			tmpl_gray.gr_Get_Row(0,block_sz_half,block_sz,&tvec_uchar);
			aa_im.vucf_Vector_Uchar_to_Float(&tvec_uchar,1.0f,&trow_tmpl);
			caption = "Intensity";
			gline[0].g_sgi_Set_Graph_Information(0.0,block_sz,-255,255,1,1);
			gline[0].g_p_Plot(caption,&trow_tmpl,0.0f,1.0f,pen_black,false);

			// Gradient graph.
			tmpl_grad_mag.gr_Get_Row(0, block_sz_half, block_sz, &trow_tmpl);
			//tmpl_grad_x.gr_Get_Row(0,block_sz_half,block_sz,&trow_tmpl);
			//tmpl_grad_y.gr_Get_Row(block_sz_half, block_sz_half, block_sz, &tcol_tmpl);

			//caption = "Gradient";
			//gline[1].g_sgi_Set_Graph_Information(0.0,block_sz,-100,100,1,1);
			
			//gline[0].g_p_Plot(caption,&trow_tmpl,0.0f,1.0f,pen_blue,true);

// 			caption = "Hessian";
// 			tmpl_laplace.gr_Get_Row(0,block_sz_half,block_sz,&trow_tmpl);

			//caption = "Laplacian";
			//tmpl_laplace.gr_Get_Row(0,block_sz_half,block_sz,&trow_tmpl);
			
			// absolute value.
			for(int i=0; i<block_sz; i++) trow_tmpl.vp()[i] = abs(trow_tmpl.vp()[i]);
			//gline[1].g_sgi_Set_Graph_Information(0.0,block_sz,-100,100,1,1);
			//gline[0].g_p_Plot(caption,&trow_tmpl,0.0f,1.0f,pen_red,true);
			//////////////////////////////////////////////////////////////////////////

// 			//////////////////////////////////////////////////////////////////////////
// 			// y-direction.
// 			// Intensity graph.
// 			tmpl_gray.gc_Get_Column(block_sz_half,0,block_sz,&tvec_uchar);
// 			aa_im.vucf_Vector_Uchar_to_Float(&tvec_uchar,1.0f,&tcol_tmpl);
// 			caption = "Intensity";
// 			gline[1].g_sgi_Set_Graph_Information(0.0,block_sz,-255,255,1,1);
// 			gline[1].g_p_Plot(caption,&tcol_tmpl,0.0f,1.0f,pen_black,false);
// 
// 			// Gradient graph.
// 			//tmpl_grad_mag.gc_Get_Column(block_sz_half,0,block_sz,&tcol_tmpl);
// 			//tmpl_grad_x.gr_Get_Row(block_sz_half,0,block_sz,&tcol_tmpl);
// 			tmpl_grad_y.gc_Get_Column(block_sz_half,0,block_sz, &tcol_tmpl);
// 
// 			caption = "Gradient";
// 			//gline[1].g_sgi_Set_Graph_Information(0.0,block_sz,-100,100,1,1);
// 			gline[1].g_p_Plot(caption,&tcol_tmpl,0.0f,1.0f,pen_blue,true);
// 
// 			caption = "Laplacian";
// 			tmpl_laplace.gr_Get_Row(block_sz_half,0,block_sz,&tcol_tmpl);
// 			// absolute value.
// 			for(int i=0; i<block_sz; i++) tcol_tmpl.vp()[i] = abs(tcol_tmpl.vp()[i]);
// 			//gline[1].g_sgi_Set_Graph_Information(0.0,block_sz,-100,100,1,1);
// 			gline[1].g_p_Plot(caption,&tcol_tmpl,0.0f,1.0f,pen_red,true);
// 			//////////////////////////////////////////////////////////////////////////

			
			
		}
				
		in_c = zz_sc[0].s_gc_Get_Character();
		
		if(in_c == 'q') break;
		if(in_c == 'p'){
			while(1){
				Kv_Pause(30);
				in_c = zz_sc[0].s_gc_Get_Character();
				if(in_c == 's') break;
			}			
		}

//		in_c = ' ';
		
		Kv_Pause(2);

	}
	//////////////////////////////////////////////////////////////////////////


	// find min/max value.
	float min_L, max_L;
	max_L = map_edge.laplacian.max_Maximum_and_Minimum(min_L, NULL, NULL, NULL, NULL);

	zz_sc[0].s_d_Display(2.0f, 0.0f, &map_edge.grad_mag);
	zz_sc[1].s_d_Display(1.0f, 128, &map_edge.laplacian);

	caption = "Gradient";		aa_uw.spt_Set_Position_and_Title(&zz_sc[0],0,0,caption);
	caption = "Laplacian";		aa_uw.spt_Set_Position_and_Title(&zz_sc[1],&zz_sc[0],0,caption);

	Kv_Printf("Extract gradients!");

}


void CVCL_Volume_Scanner_View::OnTestSiftGpuTest()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	LCKvHello aa_hh;
	LCKvUtility_for_Windows aa_uw;
	LCKvUtility_for_YCbCr aa_ycc;

	LCKvImageMatcher aa_imatch;
	LCKvYooji_Image_Processor yooji_ip;
	CKvYooji_ContourDetection yooji_cd;
	LCKvYooji_Scanner_Display aa_disp;

	CKvScreen zz_sc[3];
	CKvString fn,caption;
	CKvMatrixUcharRgb in_img[2],temp_img[2];
	CKvMatrixUchar in_img_gray[2],img_filtered[2];

	//////////////////////////////////////////////////////////////////////////
	// For SiftGPU.
	//////////////////////////////////////////////////////////////////////////
	aa_imatch.i_Initialize();

	int ww,hh;
	int prev_right,curr_right,prev_sigma;

	aa_hh.io.gof_Get_Open_Filename("Load image 1",fn);
	aa_hh.iofv.li_Load_Image(fn,false,&in_img[0]);
	aa_ycc.mrg_Matrix_Rgb_to_Gray(&in_img[0],false,&in_img_gray[0]);

	aa_hh.io.gof_Get_Open_Filename("Load image 2",fn);
	aa_hh.iofv.li_Load_Image(fn,false,&in_img[1]);
	aa_ycc.mrg_Matrix_Rgb_to_Gray(&in_img[1],false,&in_img_gray[1]);

	zz_sc[0].s_d_Display(&in_img_gray[0]);
	zz_sc[1].s_d_Display(&in_img_gray[1]);

	caption = "Gray 1";		aa_uw.spt_Set_Position_and_Title(&zz_sc[0],0,0,caption);
	caption = "Gray 2";		aa_uw.spt_Set_Position_and_Title(&zz_sc[1],&zz_sc[0],0,caption);

	Kv_Printf("Load image!");

	//////////////////////////////////////////////////////////////////////////
	// matching
	int num_feat;
	float p2d_matched_src[2*2000],p2d_matched_dst[2*2000];

	yooji_ip.sig5_Smooth_Image_Gaussian_5x5(&in_img_gray[0], &img_filtered[0]);
	yooji_ip.sig5_Smooth_Image_Gaussian_5x5(&in_img_gray[1], &img_filtered[1]);

	aa_imatch.emf_Extract_and_Match_Features_using_SiftGPU(
		img_filtered[0],
		img_filtered[1],
		num_feat,
		p2d_matched_src,
		p2d_matched_dst);

	aa_disp.dmr_Display_Matching_Result(
		&zz_sc[0],
		&in_img[0],
		&in_img[1],
		num_feat,
		p2d_matched_src,
		p2d_matched_dst);

	if(!Kv_Printf("Matched results!")) exit(0);
}


void CVCL_Volume_Scanner_View::OnTestSinglePoseEstimationWithArucoMarkersOpencv()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	CVCL_Volume_Scanner_Doc *pDoc = GetDocument();

	//////////////////////////////////////////////////////////////////////////
	//pDoc->zz_3dos.rs_Release_Scanner();
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	// for ArUco.
	CKvYooji_InterLib_Convert zz_ilc;
	float vals[] ={FX,0.,CX,0.,FY,CY,0.,0.,1.};
	float coeffs[] ={0.,0.,0.,0.,0.,};

	const Mat cameraMatrix = Mat(3,3,CV_32FC1,vals);
	const Mat distCoeffs = Mat(5,1,CV_32FC1,coeffs);

	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(5,7,0.0355,0.0036,dictionary);
	//////////////////////////////////////////////////////////////////////////

	//	if(!Kv_Printf("Release!")) exit(0);
	LCKvYooji_Scanner_Display display;
	bool flag_sil,flag_pose,flag_load;

	LCKvHello aa_hh;
	LCKvUtility_for_Windows aa_uw;
	LCKvUtility_for_YCbCr aa_ycc;

	CKvScreen zz_sc[3];
	CKvString fn,caption;
	CKvMatrixUcharRgb in_img[2],temp_img[2];
	CKvMatrixUchar in_img_gray[2],img_filtered[2];

	int ww,hh;
	int prev_right,curr_right,prev_sigma;

	aa_hh.io.gof_Get_Open_Filename("Load image 1",fn);
	aa_hh.iofv.li_Load_Image(fn,false,&in_img[0]);
	aa_ycc.mrg_Matrix_Rgb_to_Gray(&in_img[0],false,&in_img_gray[0]);

	//////////////////////////////////////////////////////////////////////////
	// ArUco Module
	//////////////////////////////////////////////////////////////////////////
	cv::Mat image,imageCopy;
	//inputVideo.retrieve(image);
	zz_ilc.cfko_Convert_Format_from_KAISION_to_Opencv(in_img[0],image);

	image.copyTo(imageCopy);
	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f> > corners;
	cv::aruco::detectMarkers(image,dictionary,corners,ids);
	// if at least one marker detected
	if(ids.size() > 0) {
		cv::aruco::drawDetectedMarkers(imageCopy,corners,ids);
		cv::Mat rvec,tvec;
		cv::Mat rmat,pmat;

		pmat = Mat::zeros(3,4,CV_32FC1);

		int valid = estimatePoseBoard(corners,ids,board,cameraMatrix,distCoeffs,rvec,tvec);
		// if at least one board marker detected
		if(valid > 0){
			cv::aruco::drawAxis(imageCopy,cameraMatrix,distCoeffs,rvec,tvec,0.1);
			
			Rodrigues(rvec, rmat);
			rmat.copyTo(pmat(CvRect(0,0,3,3)));
			tvec.copyTo(pmat(CvRect(3,0,1,3)));
			pmat = cameraMatrix * pmat;

			pDoc->writeMatToTextFile(pmat,"_ext_P.txt");
		}
	}
	cv::imshow("out",imageCopy);



	if(!Kv_Printf("Pattern!")) exit(0);
	//////////////////////////////////////////////////////////////////////////

	zz_flag_player = 0;
}

void CVCL_Volume_Scanner_View::OnTestPoseEstimationWithArucoMarkersOpenCV()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	//////////////////////////////////////////////////////////////////////////
	// for ArUco.
	float vals[] ={FX,0.,CX,0.,FY,CY,0.,0.,1.};
	float coeffs[] ={K1,K2,P1,P2,K3};

	const Mat cameraMatrix = Mat(3,3,CV_32FC1,vals);
	const Mat distCoeffs = Mat(5,1,CV_32FC1,coeffs);

	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(5,7,0.0355,0.0036,dictionary);
	cv::Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

	detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers

	vector<Mat> set_of_pose_gt;
	Mat pose_gt, rmat;	pose_gt = Mat::eye(4,4,CV_32FC1);
	//////////////////////////////////////////////////////////////////////////

	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	CVCL_Volume_Scanner_Doc *pDoc = GetDocument();
	
	//////////////////////////////////////////////////////////////////////////
	//pDoc->zz_3dos.rs_Release_Scanner();
	//////////////////////////////////////////////////////////////////////////

	//	if(!Kv_Printf("Release!")) exit(0);

	LCKvYooji_Scanner_Display display;
	CKvYooji_InterLib_Convert zz_ilc;
	bool flag_sil,flag_pose,flag_load;

	if(pDoc->zz_input_type<0)	return;

	zz_flag_player = 1;

	pDoc->zz_scan_type = 0;
	OnButton3dScanning();

	d_pm_Printf_Matrix(zz_mat_rgbd.p_intrinsics_RGB()->mp()->vp(), 3, 3, "K");

	if(!Kv_Printf("K!")) exit(0);

	// Display current scanning state ///////////////////////////////
	// //////////////////////////////////////////////////////////////
	zz_idx_frame = 0;

	int start_frame = zz_idx_frame;

	CKvStopWatch sw;	sw.c_Create(1);
	float total_time = 0.0f,avg_time = 0.0f;

	// for display trace. ////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	vector<CKvYooji_Extrinsics*>* p_trace_gt = pDoc->zz_3dos.p_trace_gt();
	zz_img_gt.c_Create(480,640,Kv_Rgb(255,255,255));

	while(zz_flag_player){

		// play.
		if(zz_flag_player==1){

			flag_load = false;

			//////////////////////////////////////////////////////////////////////////
			// load input data.
			//////////////////////////////////////////////////////////////////////////
			// pre-captured image sequence.
			if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES || pDoc->zz_input_type==KV_CAPTURE_FROM_FILES_CXD){

				if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES){
					flag_load = pDoc->zz_capture.lrdi_Load_Rgb_and_Depth_Images(
						zz_idx_frame,
						zz_mat_rgbd.p_image_rgb(),
						zz_mat_rgbd.p_map_depth_raw());
				} else if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES_CXD){

					flag_load = pDoc->zz_capture.lrdc_Load_Rgb_and_Depth_from_CxD_File(
						zz_idx_frame,
						zz_mat_rgbd.p_image_rgb(),
						zz_mat_rgbd.p_map_depth_raw());
				}

				if(flag_load){
					// Get pre-extracted silhouette.
					flag_sil = pDoc->zz_capture.ls_Load_Silhouette(zz_idx_frame,zz_mat_rgbd.p_image_silhouette());
					zz_mat_rgbd.set_validity_silhouette(flag_sil);

					// Get pre-extracted silhouette.					
					flag_pose = pDoc->zz_capture.lpm_Load_P_Matrix(zz_idx_frame,zz_mat_rgbd.p_P_matrix_depth());
					zz_mat_rgbd.set_validity_extrinsics(flag_pose);
				}
			}
			// live stream.
			else if(pDoc->zz_input_type==KV_CAPTURE_FROM_KINECTV1){
				flag_load = pDoc->zz_prime.gi_Get_Images(
					zz_mat_rgbd.p_image_rgb(),
					zz_mat_rgbd.p_map_depth_raw());
			}

			//printf("flag_load: %d\n", flag_load);

			if(flag_load){

				//////////////////////////////////////////////////////////////////////////
				// ArUco Module
				//////////////////////////////////////////////////////////////////////////
				cv::Mat image,imageCopy;
				//inputVideo.retrieve(image);
				zz_ilc.cfko_Convert_Format_from_KAISION_to_Opencv(*zz_mat_rgbd.p_image_rgb(),image);

				image.copyTo(imageCopy);
				std::vector<int> ids;
				std::vector<std::vector<cv::Point2f> > corners, rejected;
				cv::aruco::detectMarkers(image,dictionary,corners,ids,detectorParams,rejected);

				// refine strategy to detect more markers
				aruco::refineDetectedMarkers(image,board,corners,ids,rejected);

				// if at least one marker detected
				if(ids.size() > 0) {
					cv::aruco::drawDetectedMarkers(imageCopy,corners,ids);
					Mat rvec,tvec;
					int valid = estimatePoseBoard(corners,ids,board,cameraMatrix,distCoeffs,rvec,tvec);
					// if at least one board marker detected
					if(valid > 0){
						cv::aruco::drawAxis(imageCopy,cameraMatrix,distCoeffs,rvec,tvec,0.1);

						//////////////////////////////////////////////////////////////////////////
						Rodrigues(rvec,rmat);
						rmat.copyTo(pose_gt(CvRect(0,0,3,3)));
						tvec.copyTo(pose_gt(CvRect(3,0,1,3)));

						set_of_pose_gt.push_back(pose_gt.clone());
						//////////////////////////////////////////////////////////////////////////
					}

				}
				cv::imshow("out",imageCopy);
				char key = (char)cv::waitKey(30);
				if(key == 27)
					break;
				//////////////////////////////////////////////////////////////////////////
				zz_idx_frame++;

				total_time += sw.get_Get_Elapsed_Time(0);
				avg_time = total_time/(float)zz_idx_frame;

				printf("  #%d frame was processed... (%5.2f ms)\n",zz_idx_frame,avg_time*1000.0f);

			} else{
				// STOP.
				zz_flag_player = 0;
			}
		}

		//if(!Kv_Printf("pause")) exit(0);

		// pause.
		if(zz_flag_player==2){
			while(zz_flag_player==2) Kv_Pause(1);
		}

		//if(!Kv_Printf("stop1")) exit(0);
		// stop.
		if(zz_flag_player==0){
			//Invalidate(true);
			zz_idx_frame = 0;
			break;
		}

		//////////////////////////////////////////////////////////////////////////
		// for debugging.
		//if(zz_idx_frame > 100) break;
		//////////////////////////////////////////////////////////////////////////
		//if(!Kv_Printf("stop2")) exit(0);
	}

	// set world coordinates to 1st camera's coordinates.
	Mat ref_pose, tmp_pose;	set_of_pose_gt[0].copyTo(ref_pose);

	for(int i=0; i<set_of_pose_gt.size(); i++){
		
		tmp_pose = set_of_pose_gt[i]*ref_pose.inv();
		tmp_pose.copyTo(set_of_pose_gt[i]);

		cout << ref_pose << endl;

	}

	pDoc->writeMatSetToTextFile(set_of_pose_gt,"arucoboard_pose.txt");

	zz_flag_player = 0;

	set_of_pose_gt.clear();
}

void CVCL_Volume_Scanner_View::OnTestSingleposeestimationwithcharucomarkersopencv()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	CVCL_Volume_Scanner_Doc *pDoc = GetDocument();

	//////////////////////////////////////////////////////////////////////////
	//pDoc->zz_3dos.rs_Release_Scanner();
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	// for ArUco.
	CKvYooji_InterLib_Convert zz_ilc;
	float vals[] ={FX,0.,CX,0.,FY,CY,0.,0.,1.};
	float coeffs[] ={K1,K2,P1,P2,K3,};

	const Mat cameraMatrix = Mat(3,3,CV_32FC1,vals);
	const Mat distCoeffs = Mat(5,1,CV_32FC1,coeffs);

	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5,7,0.03475,0.0205,dictionary);
	//////////////////////////////////////////////////////////////////////////

	//	if(!Kv_Printf("Release!")) exit(0);
	LCKvYooji_Scanner_Display display;
	bool flag_sil,flag_pose,flag_load;

	LCKvHello aa_hh;
	LCKvUtility_for_Windows aa_uw;
	LCKvUtility_for_YCbCr aa_ycc;

	CKvScreen zz_sc[3];
	CKvString fn,caption;
	CKvMatrixUcharRgb in_img[2],temp_img[2];
	CKvMatrixUchar in_img_gray[2],img_filtered[2];

	int ww,hh;
	int prev_right,curr_right,prev_sigma;

	aa_hh.io.gof_Get_Open_Filename("Load image 1",fn);
	aa_hh.iofv.li_Load_Image(fn,false,&in_img[0]);
	aa_ycc.mrg_Matrix_Rgb_to_Gray(&in_img[0],false,&in_img_gray[0]);

	//////////////////////////////////////////////////////////////////////////
	// ArUco Module
	//////////////////////////////////////////////////////////////////////////
	cv::Mat image,imageCopy;
	//inputVideo.retrieve(image);
	zz_ilc.cfko_Convert_Format_from_KAISION_to_Opencv(in_img[0],image);

	image.copyTo(imageCopy);
	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f> > corners;
	vector<Point2f> charucoCorners;
	vector <int> charucoIds;

	cv::aruco::detectMarkers(image,dictionary,corners,ids);
	// if at least one marker detected
	if(ids.size() > 0) {

		cv::aruco::interpolateCornersCharuco(corners,ids,image,board,charucoCorners,charucoIds);


		// if at least one marker detected
		if(charucoIds.size() > 0) {
			cv::aruco::drawDetectedCornersCharuco(imageCopy,charucoCorners,charucoIds);


			Mat rvec,tvec;
			int valid = estimatePoseCharucoBoard(charucoCorners,charucoIds,board,cameraMatrix,distCoeffs,rvec,tvec);

			Mat pmat = Mat::zeros(3,4,CV_32FC1);
			Mat tmat = Mat::eye(4,4,CV_32FC1);
			Mat rmat = Mat::eye(3,3,CV_32FC1);

			// if at least one board marker detected
			if(valid > 0){
				cv::aruco::drawAxis(imageCopy,cameraMatrix,distCoeffs,rvec,tvec,0.1);

				Rodrigues(rvec,rmat);
				rmat.copyTo(tmat(CvRect(0,0,3,3)));
				tvec.copyTo(tmat(CvRect(3,0,1,3)));
				pmat = tmat(CvRect(0,0,4,3)).clone();
				pmat = cameraMatrix * pmat;

				pDoc->writeMatToTextFile(pmat,"_ext_P.txt");
				pDoc->writeMatToTextFile(tmat,"_ext_T.txt");
				
			}
		}
	}
	cv::imshow("out",imageCopy);



	if(!Kv_Printf("Pattern!")) exit(0);
	//////////////////////////////////////////////////////////////////////////

	zz_flag_player = 0;
}


void CVCL_Volume_Scanner_View::OnTestSequenceposeestimationwithcharucomarkersopencv()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	//////////////////////////////////////////////////////////////////////////
	// for ArUco.
	float vals[] ={FX,0.,CX,0.,FY,CY,0.,0.,1.};
	float coeffs[] ={K1,K2,P1,P2,K3,};

	const Mat cameraMatrix = Mat(3,3,CV_32FC1,vals);
	const Mat distCoeffs = Mat(5,1,CV_32FC1,coeffs);

	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5,7,0.03475,0.0205,dictionary);
	cv::Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

	detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers

	vector<Mat> set_of_pose_gt;
	Mat pose_gt,rmat;	pose_gt = Mat::eye(4,4,CV_32FC1);
	//////////////////////////////////////////////////////////////////////////

	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	CVCL_Volume_Scanner_Doc *pDoc = GetDocument();

	//////////////////////////////////////////////////////////////////////////
	//pDoc->zz_3dos.rs_Release_Scanner();
	//////////////////////////////////////////////////////////////////////////

	//	if(!Kv_Printf("Release!")) exit(0);

	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f> > corners, rejected;
	vector<Point2f> charucoCorners;
	vector <int> charucoIds;

	LCKvYooji_Scanner_Display display;
	CKvYooji_InterLib_Convert zz_ilc;
	bool flag_sil,flag_pose,flag_load;

	if(pDoc->zz_input_type<0)	return;

	zz_flag_player = 1;

	pDoc->zz_scan_type = 0;
	OnButton3dScanning();

	// Display current scanning state ///////////////////////////////
	// //////////////////////////////////////////////////////////////
	zz_idx_frame = 0;

	int start_frame = zz_idx_frame;

	CKvStopWatch sw;	sw.c_Create(1);
	float total_time = 0.0f,avg_time = 0.0f;

	// for display trace. ////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	vector<CKvYooji_Extrinsics*>* p_trace_gt = pDoc->zz_3dos.p_trace_gt();
	zz_img_gt.c_Create(480,640,Kv_Rgb(255,255,255));

	while(zz_flag_player){

		// play.
		if(zz_flag_player==1){

			flag_load = false;

			//////////////////////////////////////////////////////////////////////////
			// load input data.
			//////////////////////////////////////////////////////////////////////////
			// pre-captured image sequence.
			if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES || pDoc->zz_input_type==KV_CAPTURE_FROM_FILES_CXD){

				if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES){
					flag_load = pDoc->zz_capture.lrdi_Load_Rgb_and_Depth_Images(
						zz_idx_frame,
						zz_mat_rgbd.p_image_rgb(),
						zz_mat_rgbd.p_map_depth_raw());
				} else if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES_CXD){

					flag_load = pDoc->zz_capture.lrdc_Load_Rgb_and_Depth_from_CxD_File(
						zz_idx_frame,
						zz_mat_rgbd.p_image_rgb(),
						zz_mat_rgbd.p_map_depth_raw());
				}

				if(flag_load){
					// Get pre-extracted silhouette.
					flag_sil = pDoc->zz_capture.ls_Load_Silhouette(zz_idx_frame,zz_mat_rgbd.p_image_silhouette());
					zz_mat_rgbd.set_validity_silhouette(flag_sil);

					// Get pre-extracted silhouette.					
					flag_pose = pDoc->zz_capture.lpm_Load_P_Matrix(zz_idx_frame,zz_mat_rgbd.p_P_matrix_depth());
					zz_mat_rgbd.set_validity_extrinsics(flag_pose);
				}
			}
			// live stream.
			else if(pDoc->zz_input_type==KV_CAPTURE_FROM_KINECTV1){
				flag_load = pDoc->zz_prime.gi_Get_Images(
					zz_mat_rgbd.p_image_rgb(),
					zz_mat_rgbd.p_map_depth_raw());
			}

			//printf("flag_load: %d\n", flag_load);

			if(flag_load){

				//////////////////////////////////////////////////////////////////////////
				// ArUco Module
				//////////////////////////////////////////////////////////////////////////
				cv::Mat image,imageCopy;
				//inputVideo.retrieve(image);
				zz_ilc.cfko_Convert_Format_from_KAISION_to_Opencv(*zz_mat_rgbd.p_image_rgb(),image);

				image.copyTo(imageCopy);
// 				std::vector<int> ids;
// 				std::vector<std::vector<cv::Point2f> > corners;
// 				vector<Point2f> charucoCorners;
// 				vector <int> charucoIds;

				cv::aruco::detectMarkers(image,dictionary,corners,ids,detectorParams,rejected);
				// refine strategy to detect more markers
				aruco::refineDetectedMarkers(image,board,corners,ids,rejected);

				if(ids.size() > 0) {

					cv::aruco::interpolateCornersCharuco(corners,ids,image,board,charucoCorners,charucoIds);


					// if at least one marker detected
					if(charucoIds.size() > 0) {
						cv::aruco::drawDetectedCornersCharuco(imageCopy,charucoCorners,charucoIds);

			
						Mat rvec,tvec;
						int valid = estimatePoseCharucoBoard(charucoCorners,charucoIds,board,cameraMatrix,distCoeffs,rvec,tvec);
						// if at least one board marker detected

			
						if(valid > 0){
							cv::aruco::drawAxis(imageCopy,cameraMatrix,distCoeffs,rvec,tvec,0.1);

							//////////////////////////////////////////////////////////////////////////
							Rodrigues(rvec,rmat);						

							rmat.copyTo(pose_gt(CvRect(0,0,3,3)));
							tvec.copyTo(pose_gt(CvRect(3,0,1,3)));

							set_of_pose_gt.push_back(pose_gt.clone());
							//////////////////////////////////////////////////////////////////////////

						}
						else cout << "INVALID!!!!" << endl;


					}

					ids.clear();				corners.clear();	rejected.clear();
					charucoCorners.clear();		charucoIds.clear();
				}
				
				cv::imshow("out",imageCopy);
				char key = (char)cv::waitKey(30);
				if(key == 27)
					break;
				//////////////////////////////////////////////////////////////////////////
				zz_idx_frame++;

				total_time += sw.get_Get_Elapsed_Time(0);
				avg_time = total_time/(float)zz_idx_frame;

				printf("  #%d frame was processed... (%5.2f ms)\n",zz_idx_frame,avg_time*1000.0f);

			} else{
				// STOP.
				zz_flag_player = 0;
			}
		}

		//if(!Kv_Printf("pause")) exit(0);

		// pause.
		if(zz_flag_player==2){
			while(zz_flag_player==2) Kv_Pause(1);
		}

		//if(!Kv_Printf("stop1")) exit(0);
		// stop.
		if(zz_flag_player==0){
			//Invalidate(true);
			zz_idx_frame = 0;
			break;
		}

		//////////////////////////////////////////////////////////////////////////
		// for debugging.
		//if(zz_idx_frame > 100) break;
		//////////////////////////////////////////////////////////////////////////
		//if(!Kv_Printf("stop2")) exit(0);
	}

	// set world coordinates to 1st camera's coordinates.
	Mat ref_pose,tmp_pose;	set_of_pose_gt[0].copyTo(ref_pose);

	for(int i=0; i<set_of_pose_gt.size(); i++){

		tmp_pose = ref_pose.inv()*set_of_pose_gt[i];
		tmp_pose.copyTo(set_of_pose_gt[i]);

	}
	

	pDoc->writeMatSetToTextFile(set_of_pose_gt,"arucoboard_pose.txt");

	zz_flag_player = 0;

	set_of_pose_gt.clear();
}


void CVCL_Volume_Scanner_View::OnCalibrationCapturergb()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CVCL_Volume_Scanner_Doc *pDoc = GetDocument();

	LCKvIO io;
	LCKvIO_FileVcl iof;
	LCKvUtility_for_Import aa_im;

	CKvMatrixUcharRgb imgColor;
	CKvMatrixUcharRgb imgIR;
	CKvMatrixUchar imgDepthUchar,imgIRGray;
	CKvMatrixUshort imgDepthShort,imgIRShort;
	CKvMatrixFloat imgDepthFloat;

	CKvScreen sc[2];
	CKvString save_fn,save_dn;
	int cntFrame = 0;
	bool flag_capture = false,flag_valid = false;

	io.gfn_Get_Folder_Name("Open folder for saving image sequence.",save_dn);

	UpdateData(true);

	cout << "Waiting for start signal..." << endl;
	//LCKvOpenNI prime;

	while(1){

		char kb = sc[0].s_gc_Get_Character();

		flag_valid = pDoc->zz_prime.gi_Get_Images(&imgColor, &imgDepthFloat);	
		 			
 		if(flag_valid){
 		 	pDoc->zz_prime.cid8_ConvertImageDepth8bit(&imgDepthFloat,
 		 		KV_CONV_IMAGE_DEPTH_FL_TO_DISP_UC, //KV_CONV_IMAGE_DEPTH_FL_TO_UC,	//
 		 		&imgDepthUchar);
 		 
 		 	sc[0].s_d_Display(&imgColor);	sc[1].s_d_Display(&imgDepthUchar);
 		 	Kv_Pause(20);
 
 
 			if(!flag_capture){
 				if(kb == 's'){
 				 	flag_capture = true;
 				 	cout << "Start capture process." << endl;
 				}
 			}
 				 
 			if(flag_capture){
 				 
 				// save color image.
 				save_fn.fm_Format("%s\\texture\\%04d.bmp",save_dn.bp(),cntFrame);
 				iof.si_Save_Image(save_fn,true,&imgColor);
 				 
 				// save depth image.
 				save_fn.fm_Format("%s\\depth16f\\%04d.vcl",save_dn.bp(),cntFrame);
 				iof.svf_Save_as_Vcl_File(save_fn,&imgDepthFloat);
 				 
 				cntFrame++;
 				cout << "#" << cntFrame << " frame was processed...\r";// << ends;
 			}	 

			if(kb == 'q')		break;
 		 
 		}

// 		// for IR capture.
// 		flag_valid = pDoc->zz_prime.gi_Get_Images(&imgIRShort,&imgDepthFloat);		
// 
// 		if(flag_valid){
// 
// 			ushort max_val = imgIRShort.max_Maximum(NULL, NULL);
// 			//////////////////////////////////////////////////////////////////////////
// 			// Experimentally, we get optimal scale range for chessboard detection. 3~4.
// 			float scale = 4.0f;//255.0f/max_val;
// 			//////////////////////////////////////////////////////////////////////////
// 
// 			aa_im.musuc_Matrix_Ushort_to_Uchar(&imgIRShort,scale,&imgIRGray);
// 
// 			printf("scale: %f\n", scale);
// 
// 			pDoc->zz_prime.cid8_ConvertImageDepth8bit(&imgDepthFloat,
// 		 			KV_CONV_IMAGE_DEPTH_FL_TO_DISP_UC, //KV_CONV_IMAGE_DEPTH_FL_TO_UC,	//
// 		 			&imgDepthUchar);
// 
// 			sc[0].s_d_Display(&imgIRGray);	sc[1].s_d_Display(&imgDepthUchar);
// 			Kv_Pause(20);
// 			
// 			if(!flag_capture){
// 				if(kb == 's'){
// 					flag_capture = true;
// 					cout << "Start capture process." << endl;
// 				}
// 			}
// 
// 			if(flag_capture){
// 
// 				// save IR image.
// 				save_fn.fm_Format("%s\\infrared\\%04d.bmp",save_dn.bp(),cntFrame);
// 				iof.si_Save_Image(save_fn,false,&imgIRGray);
// 
// 				// save depth image.
// 				save_fn.fm_Format("%s\\depth16f\\%04d.vcl", save_dn.bp(), cntFrame);
// 				iof.svf_Save_as_Vcl_File(save_fn, &imgDepthFloat);
// 
// 				//SaveImageAsTextFile(save_fn, imgDepthFloat);
// 
// 				cntFrame++;
// 				cout << "#" << cntFrame << " frame was processed...\r";// << ends;
// 			}
// 
// 			if(kb == 'q')		break;
// 		}	
		
	}

	cout << endl << "Complete!" << endl;
}


void CVCL_Volume_Scanner_View::OnTestChessboardremovalfromdepthmaps()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	CVCL_Volume_Scanner_Doc *pDoc = GetDocument();

	//////////////////////////////////////////////////////////////////////////
	//pDoc->zz_3dos.rs_Release_Scanner();
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	// for ArUco.
	CKvYooji_InterLib_Convert zz_ilc;
	float vals[] ={FX,0.,CX,0.,FY,CY,0.,0.,1.};
	float coeffs[] ={0.,0.,0.,0.,0.,};

	const Mat cameraMatrix = Mat(3,3,CV_32FC1,vals);
	const Mat distCoeffs = Mat(5,1,CV_32FC1,coeffs);

	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(5,7,0.04,0.01,dictionary);
	cv::Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

	detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers
	//////////////////////////////////////////////////////////////////////////

	//	if(!Kv_Printf("Release!")) exit(0);

	LCKvYooji_Scanner_Display display;
	bool flag_sil,flag_pose,flag_load;

	if(pDoc->zz_input_type<0)	return;

	zz_flag_player = 1;

	pDoc->zz_scan_type = 0;
	OnButton3dScanning();

	// Display current scanning state ///////////////////////////////
	// //////////////////////////////////////////////////////////////
	zz_idx_frame = 0;

	int start_frame = zz_idx_frame;

	CKvStopWatch sw;	sw.c_Create(1);
	float total_time = 0.0f,avg_time = 0.0f;

	// for display trace. ////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	vector<CKvYooji_Extrinsics*>* p_trace_gt = pDoc->zz_3dos.p_trace_gt();
	zz_img_gt.c_Create(480,640,Kv_Rgb(255,255,255));

	//////////////////////////////////////////////////////////////////////////
	// for initialize the first frame of KINECTV1.
	if(pDoc->zz_input_type==KV_CAPTURE_FROM_KINECTV1){
		// Waits first image.
		CKvScreen sc; CKvMatrixUcharRgb timg;

		while(1){
			if(!pDoc->zz_prime.gi_Get_Images(
					zz_mat_rgbd.p_image_rgb(),
					zz_mat_rgbd.p_map_depth_raw())) break;

			zz_display.dvoi_Draw_Volume_Of_Interest(
					zz_mat_rgbd.p_map_depth_raw(),
					zz_mat_rgbd.p_intrinsics_depth(),
					zz_mat_rgbd.p_extrinsics_depth(),
					zz_cube_size,
					zz_cube_origin,
					&timg);

			sc.s_d_Display(&timg);
			Kv_Pause(10);

			// set the first frame and start scanning process.
			char ch = sc.s_gc_Get_Character();
			if(ch== 's') break;

		}
	}
	//////////////////////////////////////////////////////////////////////////

	while(zz_flag_player){

		// play.
		if(zz_flag_player==1){

			flag_load = false;

			//////////////////////////////////////////////////////////////////////////
			// load input data.
			//////////////////////////////////////////////////////////////////////////
			// pre-captured image sequence.
			if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES || pDoc->zz_input_type==KV_CAPTURE_FROM_FILES_CXD){

				if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES){
					flag_load = pDoc->zz_capture.lrdi_Load_Rgb_and_Depth_Images(
						zz_idx_frame,
						zz_mat_rgbd.p_image_rgb(),
						zz_mat_rgbd.p_map_depth_raw());
				} else if(pDoc->zz_input_type==KV_CAPTURE_FROM_FILES_CXD){

					flag_load = pDoc->zz_capture.lrdc_Load_Rgb_and_Depth_from_CxD_File(
						zz_idx_frame,
						zz_mat_rgbd.p_image_rgb(),
						zz_mat_rgbd.p_map_depth_raw());
				}

				if(flag_load){
					// Get pre-extracted silhouette.
					flag_sil = pDoc->zz_capture.ls_Load_Silhouette(zz_idx_frame,zz_mat_rgbd.p_image_silhouette());
					zz_mat_rgbd.set_validity_silhouette(flag_sil);

					// Get pre-extracted silhouette.					
					flag_pose = pDoc->zz_capture.lpm_Load_P_Matrix(zz_idx_frame,zz_mat_rgbd.p_P_matrix_depth());
					zz_mat_rgbd.set_validity_extrinsics(flag_pose);
				}
			}
			// live stream.
			else if(pDoc->zz_input_type==KV_CAPTURE_FROM_KINECTV1){
				flag_load = pDoc->zz_prime.gi_Get_Images(
					zz_mat_rgbd.p_image_rgb(),
					zz_mat_rgbd.p_map_depth_raw());
			}

			if(flag_load){

				sw.r_Reset(0);

				//////////////////////////////////////////////////////////////////////////
				// ArUco Module
				//////////////////////////////////////////////////////////////////////////
				cv::Mat image,imageCopy;
				//inputVideo.retrieve(image);
				zz_ilc.cfko_Convert_Format_from_KAISION_to_Opencv(*zz_mat_rgbd.p_image_rgb(),image);

				image.copyTo(imageCopy);
				std::vector<int> ids;
				std::vector<std::vector<cv::Point2f> > corners,rejected;
				cv::aruco::detectMarkers(image,dictionary,corners,ids,detectorParams,rejected);
				// refine strategy to detect more markers
				aruco::refineDetectedMarkers(image,board,corners,ids,rejected);

				// if at least one marker detected
				if(ids.size() > 0) {
					cv::aruco::drawDetectedMarkers(imageCopy,corners,ids);
					cv::Vec3d rvec,tvec;
					int valid = estimatePoseBoard(corners,ids,board,cameraMatrix,distCoeffs,rvec,tvec);
					// if at least one board marker detected
					if(valid > 0)
						cv::aruco::drawAxis(imageCopy,cameraMatrix,distCoeffs,rvec,tvec,0.1);
				}
				cv::imshow("out",imageCopy);
				//////////////////////////////////////////////////////////////////////////


				//////////////////////////////////////////////////////////////////////////
				// scan object.
				//////////////////////////////////////////////////////////////////////////
				zz_flag_scanning = pDoc->zz_3dos.so_Scan_Object(
					&zz_mat_rgbd,
					false,//m_flag_vh_shield,
					true,//m_flag_on_rgb_cam,
//						KV_TYPE_INPUT_OBJECT,//m_type_of_data,
					KV_TYPE_INPUT_OBJECT,//m_type_of_data,
					KV_FLAG_GROUND_TRUTH ? KV_TYPE_TRACK_GROUND : KV_TYPE_TRACK_DEPTH,
					KV_LEVEL_OF_IMAGE_PYRAMID,//m_level_of_pyramid,
					true, //in_mode_board_removal,
					true,//m_flag_gpu,
					true);
				//false);

				//zz_flag_scanning = true;


				if(!zz_flag_scanning){
					printf("=========================== TRACKING FAIL ===========================\n");
					if(!Kv_Printf("!!!")) exit(0);
				}
				//printf("zz_flag_scanning: %d\n", zz_flag_scanning);

				// get current scanning state.
				if(zz_flag_scanning){
					//printf("out_ss_OUTput_Scanning_State\n");

					zz_p_state = pDoc->zz_3dos.out_pss_OUTput_Pointer_of_Scanning_State();

					//////////////////////////////////////////////////////////////////////////
					// Display images.
					Invalidate(true);
					//////////////////////////////////////////////////////////////////////////
					//if(!Kv_Printf("Pause (%d)", 0))	exit(0);

					Kv_Pause(1);
				}


				//if(zz_idx_frame == start_frame) if(!Kv_Printf("First frame!")) exit(0);

				zz_idx_frame++;

				total_time += sw.get_Get_Elapsed_Time(0);
				avg_time = total_time/(float)zz_idx_frame;

				printf("  #%d frame was processed... (%5.2f ms)\n",zz_idx_frame,avg_time*1000.0f);

			} else{
				// STOP.
				zz_flag_player = 0;
			}
		}

		//if(!Kv_Printf("pause")) exit(0);

		// pause.
		if(zz_flag_player==2){
			while(zz_flag_player==2) Kv_Pause(1);
		}

		//if(!Kv_Printf("stop1")) exit(0);
		// stop.
		if(zz_flag_player==0){
			//Invalidate(true);
			zz_idx_frame = 0;
			break;
		}
		//if(!Kv_Printf("stop2")) exit(0);
	}

	zz_flag_player = 0;

}

////////////////////////////////////////////////////////////////////////////
//// ArUco Module
////////////////////////////////////////////////////////////////////////////
//cv::Mat image,imageCopy;
////inputVideo.retrieve(image);
//zz_ilc.cfko_Convert_Format_from_KAISION_to_Opencv(*zz_mat_rgbd.p_image_rgb(),image);
//
//image.copyTo(imageCopy);
//std::vector<int> ids;
//std::vector<std::vector<cv::Point2f> > corners,rejected;
//cv::aruco::detectMarkers(image,dictionary,corners,ids,detectorParams,rejected);
//// refine strategy to detect more markers
//aruco::refineDetectedMarkers(image,board,corners,ids,rejected);
//
//// if at least one marker detected
//if(ids.size() > 0) {
//	cv::aruco::drawDetectedMarkers(imageCopy,corners,ids);
//	cv::Vec3d rvec,tvec;
//	int valid = estimatePoseBoard(corners,ids,board,cameraMatrix,distCoeffs,rvec,tvec);
//	// if at least one board marker detected
//	if(valid > 0)
//		cv::aruco::drawAxis(imageCopy,cameraMatrix,distCoeffs,rvec,tvec,0.1);
//}
//cv::imshow("out",imageCopy);
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
//// compute world's Z-plane in camera coordinates.				
//// pi_z'*X = 0
//// pi_cam'*(T*X) = 0 -> T'*pi_cam = pi_z
//// pi_cam = inv(T')*pi_z = inv(T')(:,2) = (inv(T)(2,:))'
////Mat pi_z = pose_gt.inv()(CvRect(0,2,4,1)).t();
//Mat pi_z = pose_gt.t().inv()(CvRect(2,0,1,4));


//cout << pi_z << endl;
//if(!Kv_Printf("Pi!")) exit(0);
//////////////////////////////////////////////////////////////////////////
//pDoc->zz_3dos.rbd_Remove_Board_Depth(&zz_mat_rgbd,pi_z);
//////////////////////////////////////////////////////////////////////////

void CVCL_Volume_Scanner_View::OnTestAverageimage()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	LCKvHello aa_hh;
	LCKvUtility_for_Windows aa_uw;
	LCKvUtility_for_YCbCr aa_ycc;

	LCKvImageMatcher aa_imatch;
	LCKvYooji_Image_Processor yooji_ip;
	CKvYooji_ContourDetection yooji_cd;
	LCKvYooji_Scanner_Display aa_disp;

	CKvScreen zz_sc[3];
	CKvString fn,caption;
	CKvMatrixUcharRgb in_img[2],temp_img[2];
	CKvMatrixUcharRgb img_avg;
	CKvMatrixUchar in_img_gray[2],img_filtered[2];


	int ww,hh;
	int prev_right,curr_right,prev_sigma;

	aa_hh.io.gof_Get_Open_Filename("Load image 1",fn);
	aa_hh.iofv.li_Load_Image(fn,false,&in_img[0]);

	aa_hh.io.gof_Get_Open_Filename("Load image 2",fn);
	aa_hh.iofv.li_Load_Image(fn,false,&in_img[1]);

	img_avg.cp_Copy(&in_img[0]);

	in_img[0].ms(ww,hh);
	for(int i=0; i<ww*hh; i++){

		int tidx = i;
		for(int k=0; k<3; k++){
			img_avg.vp()[tidx] = uchar(0.5f*((float)in_img[0].vp()[tidx] + (float)in_img[1].vp()[tidx]));
			tidx += k*ww*hh;
		}

	}

	zz_sc[0].s_d_Display(&in_img[0]);
	zz_sc[1].s_d_Display(&in_img[1]);
	zz_sc[2].s_d_Display(&img_avg);
	

	Kv_Printf("Load image!");
}
