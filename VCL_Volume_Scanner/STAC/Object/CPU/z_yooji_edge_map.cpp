/////////////////////////////////////////////////////////////////////////////////////////////
// z_yooji_image_pyramid.cpp 
/////////////////////////////////////////////////////////////////////////////////////////////

#include "../../../stdafx.h"
#include "../../_yooji_2016_object_scanning.h"

/////////////////////////////////////////////////////////////////////////////////////////////
// CKvYooji_Edge_Map 
/////////////////////////////////////////////////////////////////////////////////////////////

//********************************************************************************************
CKvYooji_Edge_Map::CKvYooji_Edge_Map()
//********************************************************************************************
{
	zz_classname="CKvYooji_Edge_Map";
}

//********************************************************************************************
CKvYooji_Edge_Map::~CKvYooji_Edge_Map()
//********************************************************************************************
{
	ww = 1;	hh = 1;
	grad_x.c_Create(hh, ww, 0.0f);
	grad_y.c_Create(hh, ww, 0.0f);

	grad_mag.c_Create(hh, ww, 0.0f);
	grad_ori.c_Create(hh, ww, 0.0f);
	edge_mask.c_Create(hh, ww, false);
}
 
 //***********************************************************************************************************************
 CKvYooji_Edge_Map::CKvYooji_Edge_Map(CKvYooji_Edge_Map &a)
 //***********************************************************************************************************************
 {
 	cp_Copy(&a);
 }
 
 //********************************************************************************************
 void CKvYooji_Edge_Map::cp_Copy(CKvYooji_Edge_Map *a)
 //********************************************************************************************
 {
	ww = a->ww;		hh = a->hh;

	grad_x.cp_Copy(&a->grad_x);
	grad_y.cp_Copy(&a->grad_y);

	grad_xx.cp_Copy(&a->grad_xx);
	grad_yy.cp_Copy(&a->grad_yy);
	grad_xy.cp_Copy(&a->grad_xy);
	grad_yx.cp_Copy(&a->grad_yx);

	grad_mag.cp_Copy(&a->grad_mag);
	laplacian.cp_Copy(&a->laplacian);
	grad_ori.cp_Copy(&a->grad_ori);
	edge_mask.cp_Copy(&a->edge_mask);
 }
 
 //********************************************************************************************
 void CKvYooji_Edge_Map::c_Create(int in_hh, int in_ww)
 //********************************************************************************************
 {	
	 ww = in_ww;	hh = in_hh;

	 grad_x.c_Create(in_hh,in_ww,0.0f);
	 grad_y.c_Create(in_hh,in_ww,0.0f);

	 grad_xx.c_Create(in_hh,in_ww,0.0f);
	 grad_yy.c_Create(in_hh,in_ww,0.0f);
	 grad_xy.c_Create(in_hh,in_ww,0.0f);
	 grad_yx.c_Create(in_hh,in_ww,0.0f);

	 grad_mag.c_Create(in_hh, in_ww, 0.0f); 
	 laplacian.c_Create(in_hh, in_ww, 0.0f); 
	 grad_ori.c_Create(in_hh, in_ww, 0.0f);
	 edge_mask.c_Create(in_hh, in_ww, false);
 }

 
/////////////////////////////////////////////////////////////////////////////////////////////
// CKvYooji_Edge_Map_Pyramid 
/////////////////////////////////////////////////////////////////////////////////////////////

//********************************************************************************************
CKvYooji_Pyramid_Map_Edge::CKvYooji_Pyramid_Map_Edge()
//********************************************************************************************
{
	zz_classname="CKvYooji_Edge_Map_Pyramid";
}

//********************************************************************************************
CKvYooji_Pyramid_Map_Edge::~CKvYooji_Pyramid_Map_Edge()
//********************************************************************************************
{
	ww = 1;	hh = 1;
	maps.clear();

// 	grad_x.clear();		grad_y.clear();
// 	grad_mag.clear();	grad_ori.clear();
// 	edge_mask.clear();
	//intrins.clear();
}
 
 //***********************************************************************************************************************
 CKvYooji_Pyramid_Map_Edge::CKvYooji_Pyramid_Map_Edge(CKvYooji_Pyramid_Map_Edge &a)
 //***********************************************************************************************************************
 {
 	cp_Copy(&a);
 }
 
 //********************************************************************************************
 void CKvYooji_Pyramid_Map_Edge::cp_Copy(CKvYooji_Pyramid_Map_Edge *a)
 //********************************************************************************************
 {
	ww = a->ww;		hh = a->hh;

	if(levOfScale != a->levOfScale){

		levOfScale = a->levOfScale;

		maps.clear();
		maps.resize(levOfScale);
		
// 		grad_x.clear();				grad_y.clear();
// 		grad_mag.clear();			grad_ori.clear();
// 		edge_mask.clear();			

// 		grad_x.resize(levOfScale);		grad_y.resize(levOfScale);
// 		grad_mag.resize(levOfScale);	grad_ori.resize(levOfScale);
// 		edge_mask.resize(levOfScale);

	}

	for(int i = 0; i<levOfScale; i++){

		maps[i].cp_Copy(&a->maps[i]);

// 		grad_x[i].cp_Copy(&a->grad_x[i]);		grad_y[i].cp_Copy(&a->grad_y[i]);
// 		grad_mag[i].cp_Copy(&a->grad_mag[i]);	grad_ori[i].cp_Copy(&a->grad_ori[i]);
// 		edge_mask[i].cp_Copy(&a->edge_mask[i]);
		//intrins[i].cp_Copy(&a->intrins[i]);
	}
 }
 
 //********************************************************************************************
 void CKvYooji_Pyramid_Map_Edge::c_Create(
	 int in_ww_bottom, int in_hh_bottom,
	 int in_level_of_scale)
 //********************************************************************************************
 {	
	 int ww = in_ww_bottom, hh = in_hh_bottom;
	 int levPyram;

	 levOfScale = in_level_of_scale;
	 levPyram = max(1, levOfScale);

	 // Constructs the 3-level Gaussian pyramid for an image.	
	 maps.clear();

// 	 grad_x.clear();			grad_y.clear();
// 	 grad_mag.clear();			grad_ori.clear();
// 	 edge_mask.clear();			
	 //intrins.clear();

	 maps.resize(levOfScale);

// 	 grad_x.resize(levOfScale);		grad_y.resize(levOfScale);
// 	 grad_mag.resize(levOfScale);	grad_ori.resize(levOfScale);
// 	 edge_mask.resize(levOfScale);	
	 //intrins.resize(levOfScale);

	 for(int i = 0; i<levOfScale; i++){
		 maps[i].c_Create(hh, ww);

// 		 grad_x[i].c_Create(hh, ww, float(0));
// 		 grad_y[i].c_Create(hh, ww, float(0));
// 
// 		 grad_mag[i].c_Create(hh, ww, 0.0f);
// 		 grad_ori[i].c_Create(hh, ww, 0.0f);
// 
// 		 edge_mask[i].c_Create(hh, ww, false);
		 ww /= 2; hh /= 2;
	 }
 }


 
 
/////////////////////////////////////////////////////////////////////////////////////////////
// CKvYooji_Depth_Edge_Map_Pyramid 
/////////////////////////////////////////////////////////////////////////////////////////////

//********************************************************************************************
CKvYooji_Pyramid_Map_Depth_Edge::CKvYooji_Pyramid_Map_Depth_Edge()
//********************************************************************************************
{
	zz_classname="CKvYooji_Pyramid_Map_Depth_Edge";
}

//********************************************************************************************
CKvYooji_Pyramid_Map_Depth_Edge::~CKvYooji_Pyramid_Map_Depth_Edge()
//********************************************************************************************
{
	ww = 1;	hh = 1;
	maps.clear();
// 	grad_x.clear();		grad_y.clear();
// 	grad_mag.clear();	grad_ori.clear();
// 	edge_mask.clear();
	//intrins.clear();
}
 
 //***********************************************************************************************************************
 CKvYooji_Pyramid_Map_Depth_Edge::CKvYooji_Pyramid_Map_Depth_Edge(CKvYooji_Pyramid_Map_Depth_Edge &a)
 //***********************************************************************************************************************
 {
 	cp_Copy(&a);
 }
 
 //********************************************************************************************
 void CKvYooji_Pyramid_Map_Depth_Edge::cp_Copy(CKvYooji_Pyramid_Map_Depth_Edge *a)
 //********************************************************************************************
 {
	ww = a->ww;		hh = a->hh;

	if(levOfScale != a->levOfScale){

		levOfScale = a->levOfScale;

		maps.clear();
		maps.resize(levOfScale);
		
// 		grad_x.clear();				grad_y.clear();
// 		grad_mag.clear();			grad_ori.clear();
// 		edge_mask.clear();			
// 
// 		grad_x.resize(levOfScale);		grad_y.resize(levOfScale);
// 		grad_mag.resize(levOfScale);	grad_ori.resize(levOfScale);
// 		edge_mask.resize(levOfScale);

	}

	for(int i = 0; i<levOfScale; i++){
		maps[i].cp_Copy(&a->maps[i]);
// 		grad_x[i].cp_Copy(&a->grad_x[i]);		grad_y[i].cp_Copy(&a->grad_y[i]);
// 		grad_mag[i].cp_Copy(&a->grad_mag[i]);	grad_ori[i].cp_Copy(&a->grad_ori[i]);
// 		edge_mask[i].cp_Copy(&a->edge_mask[i]);
		//intrins[i].cp_Copy(&a->intrins[i]);
	}
 }
 
 //********************************************************************************************
 void CKvYooji_Pyramid_Map_Depth_Edge::c_Create(
	 int in_ww_bottom, int in_hh_bottom,
	 int in_level_of_scale)
 //********************************************************************************************
 {	
	 int ww = in_ww_bottom, hh = in_hh_bottom;
	 int levPyram;

	 levOfScale = in_level_of_scale;
	 levPyram = max(1, levOfScale);

	 // Constructs the 3-level Gaussian pyramid for an image.	
	 maps.clear();
	 maps.resize(levOfScale);
// 	 grad_x.clear();			grad_y.clear();
// 	 grad_mag.clear();			grad_ori.clear();
// 	 edge_mask.clear();			
	 //intrins.clear();

// 	 grad_x.resize(levOfScale);		grad_y.resize(levOfScale);
// 	 grad_mag.resize(levOfScale);	grad_ori.resize(levOfScale);
// 	 edge_mask.resize(levOfScale);	
	 //intrins.resize(levOfScale);

	 for(int i = 0; i<levOfScale; i++){
		 maps[i].c_Create(hh, ww);
// 		 grad_x[i].c_Create(hh, ww, float(0));
// 		 grad_y[i].c_Create(hh, ww, float(0));
// 
// 		 grad_mag[i].c_Create(hh, ww, 0.0f);
// 		 grad_ori[i].c_Create(hh, ww, 0.0f);
// 
// 		 edge_mask[i].c_Create(hh, ww, false);
		 ww /= 2; hh /= 2;
	 }
 }
