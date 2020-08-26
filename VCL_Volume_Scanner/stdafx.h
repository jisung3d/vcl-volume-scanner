
// stdafx.h : 자주 사용하지만 자주 변경되지는 않는
// 표준 시스템 포함 파일 및 프로젝트 관련 포함 파일이 
// 들어 있는 포함 파일입니다.

#pragma once

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN            // 거의 사용되지 않는 내용은 Windows 헤더에서 제외합니다.
#endif

#include "targetver.h"

#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS      // 일부 CString 생성자는 명시적으로 선언됩니다.

// MFC의 공통 부분과 무시 가능한 경고 메시지에 대한 숨기기를 해제합니다.
#define _AFX_ALL_WARNINGS

#include <afxwin.h>         // MFC 핵심 및 표준 구성 요소입니다.
#include <afxext.h>         // MFC 확장입니다.


#include <afxdisp.h>        // MFC 자동화 클래스입니다.



#ifndef _AFX_NO_OLE_SUPPORT
#include <afxdtctl.h>           // Internet Explorer 4 공용 컨트롤에 대한 MFC 지원입니다.
#endif
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>             // Windows 공용 컨트롤에 대한 MFC 지원입니다.
#endif // _AFX_NO_AFXCMN_SUPPORT

#include <afxcontrolbars.h>     // MFC의 리본 및 컨트롤 막대 지원

//////////////////////////////////////////////////////////////////////////

#include <Windows.h>
#include <iostream>
#include <fstream>
#include <conio.h>  // for getch().

#include <omp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <math.h>

#include <vector>
#include <limits>

// SiftGPU.
// SIFT
#include <glew.h>
#include <glut.h>
#include <wglew.h>
#include <il.h>
#include <SiftGPU.h>

// Eigen.
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>

// OpenCV 3.1.
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
 
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/eigen.hpp>

//////////////////////////////////////////////////////////////////////////
// for OpenCV RGB-D odometry. (Author      : Tzutalin)
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/rgbd.hpp>
#include <sstream>
#include <string>
#include <vector>
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
// for OpenCV Aruco Pose Estimation.
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

static double deg2rad(double deg) { return deg * CV_PI / 180.; }
//////////////////////////////////////////////////////////////////////////

using namespace cv;
using namespace std;

// This is for Kinect or Xtion
// #define FOCUS_LENGTH 525.0
// #define CX 319.5
// #define CY 239.5

// VCL Stereo Projector Camera 1
// #define FX 5128.44
// #define FY 5088.92   
// #define CX 1364.1  
// #define CY 1190.19  

// VCL Stereo Projector Camera 2
// #define FX 5080.5  
// #define FY 5044.63               
// #define CX 1304.67   
// #define CY 1180.31   

// Yooji RGB camera OLD
#define FX 536.5168                  	
#define FY 535.9731    	
#define CX 323.4094  		
#define CY 235.8256   	

#define K1 0.0 //-0.0799639	
#define K2 0.0 //0.657667		
#define P1 0.0 //-0.00183835			
#define P2 0.0 //0.0050929	
#define K3 0.0 //-1.68516	

//////////////////////////////////////////////////////////////////////////
// 쓰지마라!!!!
// Yooji depth camera 171118
// #define FX 571.966		
// #define FY 569.886		
// #define CX 324.766		
// #define CY 242.835	
// 
// #define K1 0.0 //-0.0378237
// #define K2 0.0 //-0.471502	
// #define P1 0.0 //-0.00105609		
// #define P2 0.0 //0.00158201
// #define K3 0.0 //2.97235

// Yooji RGB camera 171118
// #define FX 537.741	
// #define FY 536.139	
// #define CX 322.373	
// #define CY 240.029
// 
// #define K1 -0.0799639	
// #define K2 0.657667		
// #define P1 -0.00183835			
// #define P2 0.0050929	
// #define K3 -1.68516	
//////////////////////////////////////////////////////////////////////////
// TUM freibug 1
// #define FX 517.3
// #define FY 516.5
// #define CX 318.6
// #define CY 255.3

// TUM freibug 2
// #define FX 520.9
// #define FY 521.0
// #define CX 325.1
// #define CY 249.7

// TUM freibug 3
// #define FX 535.433105
// #define FY 539.212524
// #define CX 320.106653
// #define CY 247.632132

const float MIN_DEPTH = 0.3f;        // in meters
const float MAX_DEPTH = 4.0f;        // in meters
const float MAX_DEPTH_DIFF = 0.08f;  // in meters
const float MAX_POINTS_PART = 0.09f;

const float MAX_ROTATION = 30.0f;	// degree
const float MAX_TRANSLATION = 0.1f; //m

//CV_WRAP static inline float
//DEFAULT_MAX_TRANSLATION()
//{
//	return 0.15f; // in meters
//}
//CV_WRAP static inline float
//DEFAULT_MAX_ROTATION()
//{
//	return 15; // in degrees
//}

// This is for TUM dataset
const float PIXEL_TO_METER_SCALEFACTOR = 0.0002;

// Visualize trajectory
const float WINDOW_SIZE = 800;
const float VISUALIZATION_SCALE_FACTOR = 60.0f;
//////////////////////////////////////////////////////////////////////////

// KAISION
#include <_sdkim_2008_hello.h>
#include <_sdkim_2008_graph_3d.h>
#include <_sdkim_2008_screen.h>
#include <_sdkim_2008_color.h>

#include <_sdkim_2008_geometry.h>
#include <_sdkim_2008_depth.h>
#include <_sdkim_2008_tempo.h>

// Inter library converter.
#include "_yooji_interlib_convert.h"

// VLFeat
#include <generic.h>
#include <sift.h>

// Maxflow
#include <graph.h>
#include <instances.inc>

// ITMLib for Math.
//#include <ITMMath.h>

// vector template.
//using namespace cv;
using namespace std;
using std::vector;

typedef struct
{
	int k1 ;
	int k2 ;
	double score ;
} Pair ;

struct CKvROI2D
{
	int lt_x,lt_y,dx,dy;
	void s_Set(CKvROI2D *in_roi){ lt_x=in_roi->lt_x;	lt_y=in_roi->lt_y;	dx=in_roi->dx;	dy=in_roi->dy; }
};


// for kaision_openni.
typedef unsigned char uchar;
typedef unsigned short ushort;
typedef unsigned int uint;
typedef unsigned long ulong;

#define KV_CONV_IMAGE_DEPTH_FL_TO_UC 0
#define KV_CONV_IMAGE_DEPTH_FL_TO_DISP_UC 1

#define KV_CONV_PIX_DEPTH_TO_RGB 2

#ifdef _UNICODE
#if defined _M_IX86
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='x86' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_X64
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#else
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
#endif
#endif


