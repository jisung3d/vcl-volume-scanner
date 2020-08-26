
// VCL_Object_Scanner_SDI.h : VCL_Object_Scanner_SDI 응용 프로그램에 대한 주 헤더 파일
//
#pragma once

// under construction...
#include "STAC/_yooji_2016_object_scanning.h"

#include "STAC/Util/_yooji_kaision_openni.h"


#ifndef __AFXWIN_H__
#error "PCH에 대해 이 파일을 포함하기 전에 'stdafx.h'를 포함합니다."
#endif

#include "resource.h"       // 주 기호입니다.


// CVCL_Object_Scanner_SDIApp:
// 이 클래스의 구현에 대해서는 VCL_Object_Scanner_SDI.cpp을 참조하십시오.
//

class CVCL_Volume_Scanner_App : public CWinAppEx
{
public:
	CVCL_Volume_Scanner_App();

// 재정의입니다.
public:
	virtual BOOL InitInstance();
	virtual int ExitInstance();

// 구현입니다.
	afx_msg void OnAppAbout();
	DECLARE_MESSAGE_MAP()
	afx_msg void OnAppExit();
};

extern CVCL_Volume_Scanner_App theApp;
