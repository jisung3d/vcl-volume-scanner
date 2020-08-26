/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_image_pyramid.cpp 
/////////////////////////////////////////////////////////////////////////////////////////////

#include "../../../stdafx.h"
#include "../../_yooji_2016_object_scanning.h"


/////////////////////////////////////////////////////////////////////////////////////////////
// CKvYooji_Image_Pyramid_Uchar 
/////////////////////////////////////////////////////////////////////////////////////////////

//********************************************************************************************
CKvYooji_Pyramid_Mat_Bool::CKvYooji_Pyramid_Mat_Bool()
//********************************************************************************************
{
	zz_classname="CKvYooji_Image_Pyramid_Bool";

	levOfScale = 1;
	imgs.clear();				//intrins.clear();
	imgs.resize(levOfScale);	//intrins.resize(levOfScale);
}

//********************************************************************************************
CKvYooji_Pyramid_Mat_Bool::~CKvYooji_Pyramid_Mat_Bool()
//********************************************************************************************
{
	imgs.clear();
}
 
 //***********************************************************************************************************************
 CKvYooji_Pyramid_Mat_Bool::CKvYooji_Pyramid_Mat_Bool(CKvYooji_Pyramid_Mat_Bool &a)
 //***********************************************************************************************************************
 {
 	cp_Copy(&a);
 }
 
 //********************************************************************************************
 void CKvYooji_Pyramid_Mat_Bool::cp_Copy(CKvYooji_Pyramid_Mat_Bool *a)
 //********************************************************************************************
 {
	if(levOfScale != a->levOfScale){
		levOfScale = a->levOfScale;
		imgs.clear();				//intrins.clear();
 		imgs.resize(levOfScale);	//intrins.resize(levOfScale);	
	}

 	for(int i=0; i<levOfScale; i++){
		imgs[i].cp_Copy(&a->imgs[i]);
		//intrins[i].cp_Copy(&a->intrins[i]);
	}
 }
 
 //********************************************************************************************
 void CKvYooji_Pyramid_Mat_Bool::c_Create(
 	int in_ww_bottom, int in_hh_bottom,
 	int in_level_of_scale)
 //********************************************************************************************
 {	

 	int ww = in_ww_bottom, hh = in_hh_bottom;
 	int levPyram;
 
 	levOfScale = in_level_of_scale;
 	levPyram = max(1, levOfScale);
 
 	// Constructs the 3-level Gaussian pyramid for an image.	
	imgs.clear();			//intrins.clear(); 
	imgs.resize(levPyram);	//intrins.resize(levPyram);

	for(int i = 0; i<levOfScale; i++){
		imgs[i].c_Create(hh, ww, false);
		ww /= 2; hh /= 2;
	}
 }

/////////////////////////////////////////////////////////////////////////////////////////////
// CKvYooji_Image_Pyramid_Uchar 
/////////////////////////////////////////////////////////////////////////////////////////////

//********************************************************************************************
CKvYooji_Pyramid_Mat_Uchar::CKvYooji_Pyramid_Mat_Uchar()
//********************************************************************************************
{
	zz_classname="CKvYooji_Image_Pyramid_Uchar";

	levOfScale = 1;
	imgs.clear();				//intrins.clear();
	imgs.resize(levOfScale);	//intrins.resize(levOfScale);
}

//********************************************************************************************
CKvYooji_Pyramid_Mat_Uchar::~CKvYooji_Pyramid_Mat_Uchar()
//********************************************************************************************
{
	imgs.clear();
}
 
 //***********************************************************************************************************************
 CKvYooji_Pyramid_Mat_Uchar::CKvYooji_Pyramid_Mat_Uchar(CKvYooji_Pyramid_Mat_Uchar &a)
 //***********************************************************************************************************************
 {
 	cp_Copy(&a);
 }
 
 //********************************************************************************************
 void CKvYooji_Pyramid_Mat_Uchar::cp_Copy(CKvYooji_Pyramid_Mat_Uchar *a)
 //********************************************************************************************
 {
	if(levOfScale != a->levOfScale){
		levOfScale = a->levOfScale;
		imgs.clear();				//intrins.clear();
 		imgs.resize(levOfScale);	//intrins.resize(levOfScale);	
	}

 	for(int i=0; i<levOfScale; i++){
		imgs[i].cp_Copy(&a->imgs[i]);
		//intrins[i].cp_Copy(&a->intrins[i]);
	}
 }
 
 //********************************************************************************************
 void CKvYooji_Pyramid_Mat_Uchar::c_Create(
 	int in_ww_bottom, int in_hh_bottom,
 	int in_level_of_scale)
 //********************************************************************************************
 {	

 	int ww = in_ww_bottom, hh = in_hh_bottom;
 	int levPyram;
 
 	levOfScale = in_level_of_scale;
 	levPyram = max(1, levOfScale);
 
 	// Constructs the 3-level Gaussian pyramid for an image.	
	imgs.clear();			//intrins.clear(); 
	imgs.resize(levPyram);	//intrins.resize(levPyram);

	for(int i = 0; i<levOfScale; i++){
		imgs[i].c_Create(hh, ww, uchar(0));
		ww /= 2; hh /= 2;
	}
 }

 
/////////////////////////////////////////////////////////////////////////////////////////////
// CKvYooji_Image_Pyramid_Short
/////////////////////////////////////////////////////////////////////////////////////////////

//********************************************************************************************
CKvYooji_Image_Pyramid_Short::CKvYooji_Image_Pyramid_Short()
//********************************************************************************************
{
	zz_classname="CKvYooji_Image_Pyramid_Short";

	levOfScale = 1;
	imgs.clear();				//intrins.clear();
	imgs.resize(levOfScale);	//intrins.resize(levOfScale);
}

//********************************************************************************************
CKvYooji_Image_Pyramid_Short::~CKvYooji_Image_Pyramid_Short()
//********************************************************************************************
{
	imgs.clear();
}
 
 //***********************************************************************************************************************
 CKvYooji_Image_Pyramid_Short::CKvYooji_Image_Pyramid_Short(CKvYooji_Image_Pyramid_Short &a)
 //***********************************************************************************************************************
 {
 	cp_Copy(&a);
 }
 
 //********************************************************************************************
 void CKvYooji_Image_Pyramid_Short::cp_Copy(CKvYooji_Image_Pyramid_Short *a)
 //********************************************************************************************
 {
	if(levOfScale != a->levOfScale){
		levOfScale = a->levOfScale;
		imgs.clear();				//intrins.clear();
 		imgs.resize(levOfScale);	//intrins.resize(levOfScale);	
	}

 	for(int i=0; i<levOfScale; i++){
		imgs[i].cp_Copy(&a->imgs[i]);
		//intrins[i].cp_Copy(&a->intrins[i]);
	}
 }
 
 //********************************************************************************************
 void CKvYooji_Image_Pyramid_Short::c_Create(
 	int in_ww_bottom, int in_hh_bottom,
 	int in_level_of_scale)
 //********************************************************************************************
 {	

 	int ww = in_ww_bottom, hh = in_hh_bottom;
 	int levPyram;
 
 	levOfScale = in_level_of_scale;
 	levPyram = max(1, levOfScale);
 
 	// Constructs the 3-level Gaussian pyramid for an image.	
	imgs.clear();			//intrins.clear(); 
	imgs.resize(levPyram);	//intrins.resize(levPyram);

	for(int i = 0; i<levOfScale; i++){
		imgs[i].c_Create(hh, ww, short(0));
		ww /= 2; hh /= 2;
	}
 }

 /////////////////////////////////////////////////////////////////////////////////////////////
// CKvYooji_Image_Pyramid_Float 
/////////////////////////////////////////////////////////////////////////////////////////////

//********************************************************************************************
CKvYooji_Pyramid_Mat_Float::CKvYooji_Pyramid_Mat_Float()
//********************************************************************************************
{
	zz_classname="CKvYooji_Image_Pyramid_Float";

	levOfScale = 1;
	imgs.clear();				//intrins.clear();
	imgs.resize(levOfScale);	//intrins.resize(levOfScale);
}

//********************************************************************************************
CKvYooji_Pyramid_Mat_Float::~CKvYooji_Pyramid_Mat_Float()
//********************************************************************************************
{
	imgs.clear();
}
 
 //***********************************************************************************************************************
 CKvYooji_Pyramid_Mat_Float::CKvYooji_Pyramid_Mat_Float(CKvYooji_Pyramid_Mat_Float &a)
 //***********************************************************************************************************************
 {
 	cp_Copy(&a);
 }
 
 //********************************************************************************************
 void CKvYooji_Pyramid_Mat_Float::cp_Copy(CKvYooji_Pyramid_Mat_Float *a)
 //********************************************************************************************
 {
	 if(levOfScale != a->levOfScale){
		 levOfScale = a->levOfScale;
		 imgs.clear();				//intrins.clear();
		 imgs.resize(levOfScale);	//intrins.resize(levOfScale);
	 }

	 for(int i = 0; i<levOfScale; i++){
		 imgs[i].cp_Copy(&a->imgs[i]);
		 //intrins[i].cp_Copy(&a->intrins[i]);
	 }
 }
 
 //********************************************************************************************
 void CKvYooji_Pyramid_Mat_Float::c_Create(
 	int in_ww_bottom, int in_hh_bottom,
 	int in_level_of_scale)
 //********************************************************************************************
 {	

 	int ww = in_ww_bottom, hh = in_hh_bottom;
 	int levPyram;
 
 	levOfScale = in_level_of_scale;
 	levPyram = max(1, levOfScale);
 
 	// Constructs the 3-level Gaussian pyramid for an image.	
	imgs.clear();			//intrins.clear();
	imgs.resize(levPyram);	//intrins.resize(levPyram);

	for(int i = 0; i<levOfScale; i++){
		imgs[i].c_Create(hh, ww, 0.0f);
		ww /= 2; hh /= 2;
	}
 }

 
 /////////////////////////////////////////////////////////////////////////////////////////////
// CKvYooji_Image_Pyramid_Point3Df 
/////////////////////////////////////////////////////////////////////////////////////////////

//********************************************************************************************
CKvYooji_Image_Pyramid_Point3Df::CKvYooji_Image_Pyramid_Point3Df()
//********************************************************************************************
{
	zz_classname="CKvYooji_Image_Pyramid_Point3Df";
	
	levOfScale = 1;
	imgs.clear();				//intrins.clear();
	imgs.resize(levOfScale);	//intrins.resize(levOfScale);
}

//********************************************************************************************
CKvYooji_Image_Pyramid_Point3Df::~CKvYooji_Image_Pyramid_Point3Df()
//********************************************************************************************
{
	imgs.clear();
}
 
 //***********************************************************************************************************************
 CKvYooji_Image_Pyramid_Point3Df::CKvYooji_Image_Pyramid_Point3Df(CKvYooji_Image_Pyramid_Point3Df &a)
 //***********************************************************************************************************************
 {
 	cp_Copy(&a);
 }
 
 //********************************************************************************************
 void CKvYooji_Image_Pyramid_Point3Df::cp_Copy(CKvYooji_Image_Pyramid_Point3Df *a)
 //********************************************************************************************
 {
	 if(levOfScale != a->levOfScale){
		 levOfScale = a->levOfScale;
		 imgs.clear();				//intrins.clear();
		 imgs.resize(levOfScale);	//intrins.resize(levOfScale);
	 }

	 for(int i = 0; i<levOfScale; i++){
		 imgs[i].cp_Copy(&a->imgs[i]);
		 //intrins[i].cp_Copy(&a->intrins[i]);
	 }
 }
 
 //********************************************************************************************
 void CKvYooji_Image_Pyramid_Point3Df::c_Create(
 	int in_ww_bottom, int in_hh_bottom,
 	int in_level_of_scale)
 //********************************************************************************************
 {	

 	int ww = in_ww_bottom, hh = in_hh_bottom;
 	int levPyram;
 
 	levOfScale = in_level_of_scale;
 	levPyram = max(1, levOfScale);
 
 	// Constructs the 3-level Gaussian pyramid for an image.	
	imgs.clear();			//intrins.clear();
	imgs.resize(levPyram);	//intrins.resize(levPyram);

	for(int i = 0; i<levOfScale; i++){
		imgs[i].c_Create(hh, ww);
		ww /= 2; hh /= 2;
	}
 }
  
 /////////////////////////////////////////////////////////////////////////////////////////////
// CKvYooji_Image_Pyramid_Point3Df 
/////////////////////////////////////////////////////////////////////////////////////////////

//********************************************************************************************
CKvYooji_Pyramid_Intrinsics::CKvYooji_Pyramid_Intrinsics()
//********************************************************************************************
{
	zz_classname="CKvYooji_Image_Pyramid_Intrinsics";
	
	levOfScale = 1;
	intrins.clear();
	intrins.resize(levOfScale);
}

//********************************************************************************************
CKvYooji_Pyramid_Intrinsics::~CKvYooji_Pyramid_Intrinsics()
//********************************************************************************************
{
	intrins.clear();
}
 
 //***********************************************************************************************************************
 CKvYooji_Pyramid_Intrinsics::CKvYooji_Pyramid_Intrinsics(CKvYooji_Pyramid_Intrinsics &a)
 //***********************************************************************************************************************
 {
 	cp_Copy(&a);
 }
 
 //********************************************************************************************
 void CKvYooji_Pyramid_Intrinsics::cp_Copy(CKvYooji_Pyramid_Intrinsics *a)
 //********************************************************************************************
 {
	 if(levOfScale != a->levOfScale){
		 levOfScale = a->levOfScale;
		 intrins.clear();
		 intrins.resize(levOfScale);
	 }

	 for(int i = 0; i<levOfScale; i++){		 
		 intrins[i].copy(&a->intrins[i]);
	 }
 }
 
 //********************************************************************************************
 void CKvYooji_Pyramid_Intrinsics::c_Create(
 	int in_level_of_scale)
 //********************************************************************************************
 {	

 	int levPyram;
 
 	levOfScale = in_level_of_scale;
 	levPyram = max(1, levOfScale);
 
 	// Constructs the 3-level Gaussian pyramid for an image.	
	intrins.clear();
	intrins.resize(levPyram);

 }