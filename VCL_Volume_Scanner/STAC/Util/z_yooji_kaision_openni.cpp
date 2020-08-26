#include "../../stdafx.h"

#include "_yooji_kaision_openni.h"
#include <OpenNI.h>

//********************************************************************************************
class LCKvOpenNI::PrivateData {
public:
	PrivateData(void) {}
	openni::Device device;
	openni::VideoStream depthStream, colorStream;

	openni::VideoFrameRef depthFrame;
	openni::VideoFrameRef colorFrame;
	openni::VideoStream **streams;
};
//********************************************************************************************

//********************************************************************************************
LCKvOpenNI::LCKvOpenNI(const char *deviceURI, const bool useInternalCalibration)
//********************************************************************************************
{
// 	data = new PrivateData();	
// 	if (deviceURI==NULL) 	deviceURI = openni::ANY_DEVICE;
// 
// 	openni::Status rc = openni::STATUS_OK;
// 
// 	rc = openni::OpenNI::initialize();
// 	printf("OpenNI: Initialization ... \n%s\n", openni::OpenNI::getExtendedError());
// 
// 	rc = data->device.open(deviceURI);
// 	if (rc != openni::STATUS_OK)
// 	{
// 		printf("OpenNI: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
// 		openni::OpenNI::shutdown();
// 		return;
// 	}
// 	
// 	// check hardware registration capability for depth and color images.
// 	openni::ImageRegistrationMode irm = openni::ImageRegistrationMode::IMAGE_REGISTRATION_DEPTH_TO_COLOR;
// 	if (data->device.isImageRegistrationModeSupported(irm)) data->device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
// 	
// 	// open depth stream.
// 	rc = data->depthStream.create(data->device, openni::SENSOR_DEPTH);
// 	if (rc == openni::STATUS_OK)
// 	{
// 		openni::VideoMode depthMode = data->depthStream.getVideoMode();
// 		depthMode.setResolution(640, 480); depthMode.setFps(30);
// 		
// 		depthMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
// 		
// 		data->depthStream.setMirroringEnabled(false);
// 		//data->depthStream.setMirroringEnabled(true);
// 		rc = data->depthStream.setVideoMode(depthMode);
// 		if (rc != openni::STATUS_OK)
// 		{
// 			printf("OpenNI: Failed to set depth mode\n");
// 			openni::OpenNI::shutdown();
// 			return;
// 		}
// 
// 		rc = data->depthStream.start();
// 		if (rc != openni::STATUS_OK)
// 		{
// 			printf("OpenNI: Couldn't start depthStream stream:\n%s\n", openni::OpenNI::getExtendedError());
// 			data->depthStream.destroy();
// 		}
// 
// 		depthAvailable = true;
// 	}
// 	else
// 	{
// 		printf("OpenNI: Couldn't find depthStream stream:\n%s\n", openni::OpenNI::getExtendedError());
// 		depthAvailable = false;
// 	}
// 
// 		
// 	// open color stream.
// 	bool mode_IR = false;
// 
// 	
// 	rc = data->colorStream.create(data->device, mode_IR ? openni::SENSOR_IR : openni::SENSOR_COLOR);
// 	
// 	if (rc == openni::STATUS_OK)
// 	{
// 		openni::VideoMode colourMode = data->colorStream.getVideoMode();
// 		colourMode.setResolution(640, 480); colourMode.setFps(30);	
// 				
// 		colourMode.setPixelFormat(mode_IR ? openni::PIXEL_FORMAT_GRAY16 : openni::PIXEL_FORMAT_RGB888);
// 
// 		data->colorStream.setMirroringEnabled(false);
// 		//data->colorStream.setMirroringEnabled(true);
// 		rc = data->colorStream.setVideoMode(colourMode);
// 		if (rc != openni::STATUS_OK)
// 		{
// 			printf("OpenNI: Failed to set color mode\n");
// 			openni::OpenNI::shutdown();
// 			return;
// 		}
// 
// 		rc = data->colorStream.start();
// 		if (rc != openni::STATUS_OK)
// 		{
// 			printf("SimpleViewer: Couldn't start colorStream stream:\n%s\n", openni::OpenNI::getExtendedError());
// 			data->colorStream.destroy();
// 		}
// 
// 		// Set auto exposure and white balance disabled.
// 		if(!mode_IR){
// 			data->colorStream.getCameraSettings()->setAutoExposureEnabled(false);
// 			data->colorStream.getCameraSettings()->setAutoWhiteBalanceEnabled(false);
// 		}
// 
// 		colorAvailable = true;
// 	}
// 	else
// 	{
// 		printf("OpenNI: Couldn't find colorStream stream:\n%s\n", openni::OpenNI::getExtendedError());
// 		colorAvailable = false;
// 	}
// 	
// 	if (!depthAvailable)
// 	{
// 		printf("OpenNI: No valid streams. Exiting\n");
// 		openni::OpenNI::shutdown();
// 		return;
// 	}
// 
// 	data->streams = new openni::VideoStream*[2];
// 	if (depthAvailable) data->streams[0] = &data->depthStream;
// 	if (colorAvailable) data->streams[1] = &data->colorStream;
}

//********************************************************************************************
LCKvOpenNI::~LCKvOpenNI()
	//********************************************************************************************
{
	if (depthAvailable)
	{
		data->depthStream.stop();
		data->depthStream.destroy();
	}
	if (colorAvailable)
	{
		data->colorStream.stop();
		data->colorStream.destroy();
	}
	data->device.close();

	delete[] data->streams;
	delete data;

	openni::OpenNI::shutdown();
}

//********************************************************************************************
bool LCKvOpenNI::gi_Get_Images(CKvMatrixUcharRgb *out_img_rgb, CKvMatrixUshort *out_img_d)
//********************************************************************************************
{
	int changedIndex, waitStreamCount;
	if (depthAvailable && colorAvailable) waitStreamCount = 2;
	else waitStreamCount = 1;

	openni::Status rc = openni::OpenNI::waitForAnyStream(data->streams, waitStreamCount, &changedIndex);
	if (rc != openni::STATUS_OK) { printf("OpenNI: Wait failed\n"); return false; }

	if(depthAvailable) data->depthStream.readFrame(&data->depthFrame);
	if(colorAvailable) data->colorStream.readFrame(&data->colorFrame);

	if (depthAvailable && !data->depthFrame.isValid()) return false;
	if (colorAvailable && !data->colorFrame.isValid()) return false;

	// get color image.
	if(colorAvailable){
		unsigned char *p_out_rgb;
		int ww, hh;

		p_out_rgb=(*out_img_rgb).mps(ww, hh)[0];

		if(ww!=640 || hh !=480){
			ww=640;	hh=480;
			p_out_rgb=(*out_img_rgb).c_Create(hh, ww)[0];
		}

		const openni::RGB888Pixel* colorImagePix = (const openni::RGB888Pixel*)data->colorFrame.getData();
		for (int i = 0; i <ww*hh; i++)
		{
			openni::RGB888Pixel oldPix = colorImagePix[i];
			p_out_rgb[i]=oldPix.r;
			p_out_rgb[i+ww*hh]=oldPix.g;
			p_out_rgb[i+2*ww*hh]=oldPix.b;			
		}
	}

	// get depth image.
	if(depthAvailable){
		unsigned short *p_out_d;
		int ww, hh;

		p_out_d=(*out_img_d).mps(ww, hh)[0];

		if(ww!=640 || hh !=480){
			ww=640;	hh=480;
			p_out_d=(*out_img_d).c_Create(hh, ww)[0];
		}

		const openni::DepthPixel* depthImagePix = (const openni::DepthPixel*)data->depthFrame.getData();
		for (int i = 0; i < ww*hh; i++)
		{
			unsigned short depthPix = depthImagePix[i];
			//p_out_d[i] = depthPix == 0 ? -1.0f : (float)depthPix;
			if(depthPix<MAX_DEPTH_VAL && depthPix>MIN_DEPTH_VAL)	p_out_d[i] = depthPix;
			else													p_out_d[i] = 0;
		}
	}

	return true;
}

//********************************************************************************************
bool LCKvOpenNI::gi_Get_Images(CKvMatrixUcharRgb *out_img_rgb, CKvMatrixFloat *out_img_d)
//********************************************************************************************
{
	int changedIndex, waitStreamCount;
	if(depthAvailable && colorAvailable) waitStreamCount = 2;
	else waitStreamCount = 1;

	openni::Status rc = openni::OpenNI::waitForAnyStream(data->streams, waitStreamCount, &changedIndex);
	if(rc != openni::STATUS_OK) { printf("OpenNI: Wait failed\n"); return false; }

	if(depthAvailable) data->depthStream.readFrame(&data->depthFrame);
	if(colorAvailable) data->colorStream.readFrame(&data->colorFrame);

	if(depthAvailable && !data->depthFrame.isValid()) return false;
	if(colorAvailable && !data->colorFrame.isValid()) return false;

	// get color image.
	if(colorAvailable){
		unsigned char *p_out_rgb;
		int ww, hh;

		p_out_rgb=(*out_img_rgb).mps(ww, hh)[0];

		if(ww!=640 || hh !=480){
			ww=640;	hh=480;
			p_out_rgb=(*out_img_rgb).c_Create(hh, ww)[0];
		}

		const openni::RGB888Pixel* colorImagePix = (const openni::RGB888Pixel*)data->colorFrame.getData();
		for(int i = 0; i <ww*hh; i++)
		{
			openni::RGB888Pixel oldPix = colorImagePix[i];
			p_out_rgb[i]=oldPix.r;
			p_out_rgb[i+ww*hh]=oldPix.g;
			p_out_rgb[i+2*ww*hh]=oldPix.b;
		}
	}

	// get depth image.
	if(depthAvailable){
		float *p_out_d;
		int ww, hh;

		p_out_d=(*out_img_d).mps(ww, hh)[0];

		if(ww!=640 || hh !=480){
			ww=640;	hh=480;
			p_out_d=(*out_img_d).c_Create(hh, ww)[0];
		}

		const openni::DepthPixel* depthImagePix = (const openni::DepthPixel*)data->depthFrame.getData();
		for(int i = 0; i < ww*hh; i++)
		{
			unsigned short depthPix = depthImagePix[i];
			//p_out_d[i] = depthPix == 0 ? -1.0f : (float)depthPix;
			if(depthPix<MAX_DEPTH_VAL && depthPix>MIN_DEPTH_VAL)	
				p_out_d[i] = 0.001f*(float)depthPix;
			else																
				p_out_d[i] = 0.0f;
		}
	}

	return true;
}

//********************************************************************************************
bool LCKvOpenNI::gi_Get_Images(CKvMatrixUshort *out_img_IR, CKvMatrixFloat *out_img_d)
//********************************************************************************************
{
	int changedIndex, waitStreamCount;
	if(depthAvailable && colorAvailable) waitStreamCount = 2;
	else waitStreamCount = 1;

	openni::Status rc = openni::OpenNI::waitForAnyStream(data->streams, waitStreamCount, &changedIndex);
	if(rc != openni::STATUS_OK) { printf("OpenNI: Wait failed\n"); return false; }

	if(depthAvailable) data->depthStream.readFrame(&data->depthFrame);
	if(colorAvailable) data->colorStream.readFrame(&data->colorFrame);

	if(depthAvailable && !data->depthFrame.isValid()) return false;
	if(colorAvailable && !data->colorFrame.isValid()) return false;

	// get color image.
	if(colorAvailable){
		unsigned short *p_out_IR;
		int ww, hh;

		p_out_IR=(*out_img_IR).mps(ww, hh)[0];

		if(ww!=640 || hh !=480){
			ww=640;	hh=480;
			p_out_IR=(*out_img_IR).c_Create(hh, ww)[0];
		}

		const openni::Grayscale16Pixel* colorImagePix = (const openni::Grayscale16Pixel*)data->colorFrame.getData();
		for(int i = 0; i <ww*hh; i++)
		{
			openni::Grayscale16Pixel oldPix = colorImagePix[i];
			p_out_IR[i]=oldPix;
		}
	}

	// get depth image.
	if(depthAvailable){
		float *p_out_d;
		int ww, hh;

		p_out_d=(*out_img_d).mps(ww, hh)[0];

		if(ww!=640 || hh !=480){
			ww=640;	hh=480;
			p_out_d=(*out_img_d).c_Create(hh, ww)[0];
		}

		const openni::DepthPixel* depthImagePix = (const openni::DepthPixel*)data->depthFrame.getData();
		for(int i = 0; i < ww*hh; i++)
		{
			unsigned short depthPix = depthImagePix[i];
			//p_out_d[i] = depthPix == 0 ? -1.0f : (float)depthPix;
			if(depthPix<MAX_DEPTH_VAL && depthPix>MIN_DEPTH_VAL)	
				p_out_d[i] = 0.001f*(float)depthPix;
			else																
				p_out_d[i] = 0.0f;
		}
	}

	return true;
}

//********************************************************************************************
void LCKvOpenNI::sroi_Set_Region_Of_Interest(CKvMatrixUshort *io_img_d, int x_tl, int x_br, int y_tl, int y_br)
//********************************************************************************************
{
	int out_ww, out_hh, max_d, min_d;
	unsigned short *p_io_img_d;
	
	p_io_img_d=io_img_d->mps(out_ww, out_hh)[0];

	for(int j=0; j<out_hh; j++){
		for(int i=0; i<out_ww; i++){

			if(i<x_br && i>x_tl && j<y_br && j>y_tl)		continue;
			p_io_img_d[j*out_ww+i]=(unsigned short)0;

		}
	}

}

//********************************************************************************************
void LCKvOpenNI::cid8_Convert_Image_Depth_to_8bit(CKvMatrixUshort *in_img_d, CKvMatrixUchar *out_img_d)
//********************************************************************************************
{
	int out_ww, out_hh, max_d, min_d;
	unsigned char *p_out_img_d;
	unsigned short *p_in_img_d;

	p_out_img_d=out_img_d->mps(out_hh, out_ww)[0];
	p_in_img_d=in_img_d->vp();

	if(out_ww != in_img_d->mw() || out_hh != in_img_d->mh()){
		
		out_ww=in_img_d->mw();	out_hh=in_img_d->mh();
		p_out_img_d=out_img_d->c_Create(out_hh, out_ww, (unsigned char)0)[0];
	}

	max_d=-10000000;
	min_d=10000000;

	// find min/max depth value.
	for(int i=0; i<out_ww*out_hh; i++){
		if(p_in_img_d[i]<MIN_DEPTH_VAL || p_in_img_d[i]>MAX_DEPTH_VAL)		continue;
		if(p_in_img_d[i]<min_d)		min_d=p_in_img_d[i];
		if(p_in_img_d[i]>max_d)		max_d=p_in_img_d[i];
	}

	// convert raw depth value to 8bit.
	for(int i=0; i<out_ww*out_hh; i++){
		if(p_in_img_d[i]<MIN_DEPTH_VAL || p_in_img_d[i]>MAX_DEPTH_VAL)		p_out_img_d[i]=(unsigned char)0;	
		else				p_out_img_d[i]=(unsigned char)(255.0f-255.0f*(float)((int)p_in_img_d[i]-min_d)/(float)(max_d-min_d));
	}

}

//********************************************************************************************
void LCKvOpenNI::cid8_Convert_Image_Depth_to_8bit(CKvMatrixUshort *in_img_d, CKvMatrixUchar *out_img_d,
	int in_min_d, int in_max_d)
//********************************************************************************************
{
	int out_ww, out_hh;
	unsigned char *p_out_img_d;
	unsigned short *p_in_img_d;

	p_out_img_d=out_img_d->mps(out_hh, out_ww)[0];
	p_in_img_d=in_img_d->vp();

	if(out_ww != in_img_d->mw() || out_hh != in_img_d->mh()){
		
		out_ww=in_img_d->mw();	out_hh=in_img_d->mh();
		p_out_img_d=out_img_d->c_Create(out_hh, out_ww, (unsigned char)0)[0];
	}

	// check min/max depth value.
	if(in_min_d>in_max_d || in_min_d<MIN_DEPTH_VAL || in_max_d>MAX_DEPTH_VAL){
		in_min_d=MIN_DEPTH_VAL;
		in_max_d=MAX_DEPTH_VAL;
	}

	// convert raw depth value to 8-bit disparity.
	for(int i=0; i<out_ww*out_hh; i++){
		if(p_in_img_d[i]<MIN_DEPTH_VAL || p_in_img_d[i]>MAX_DEPTH_VAL)		p_out_img_d[i]=(unsigned char)0;	
		else				p_out_img_d[i]=(unsigned char)(255.0f-255.0f*(float)((int)p_in_img_d[i]-in_min_d)/(float)(in_max_d-in_min_d));
	}

}

//********************************************************************************************
int LCKvOpenNI::cid8_Convert_Image_Depth_to_8bit(CKvMatrixUshort *in_img_d, CKvMatrixUchar *out_img_d,
	int x_tl, int x_br, int y_tl, int y_br)
//********************************************************************************************
{
	int out_ww, out_hh, max_d, min_d, num_pix;

	unsigned char *p_out_img_d;
	unsigned short *p_in_img_d;

	p_out_img_d=out_img_d->mps(out_hh, out_ww)[0];
	p_in_img_d=in_img_d->vp();
	
	if(out_ww != in_img_d->mw() || out_hh != in_img_d->mh()){
		
		out_ww=in_img_d->mw();	out_hh=in_img_d->mh();
		p_out_img_d=out_img_d->c_Create(out_hh, out_ww, (unsigned char)0)[0];
	}

	num_pix=0;
	max_d=-10000000;		min_d=10000000;

	// find min/max depth value.
	for(int j=0; j<out_hh; j++){
		for(int i=0; i<out_ww; i++){

			int tidx = j*out_ww+i;

			if(i<x_br && i>x_tl && j<y_br && j>y_tl){
				if(p_in_img_d[tidx]<MIN_DEPTH_VAL || p_in_img_d[tidx]>MAX_DEPTH_VAL)		continue;
				if(p_in_img_d[tidx]<min_d)		min_d=p_in_img_d[tidx];
				if(p_in_img_d[tidx]>max_d)		max_d=p_in_img_d[tidx];
			}			
		}
	}

	// convert raw depth value to 8bit.
	for(int j=0; j<out_hh; j++){
		for(int i=0; i<out_ww; i++){

			int tidx = j*out_ww+i;

			if(i<x_br && i>x_tl && j<y_br && j>y_tl){
				if(p_in_img_d[tidx]<MIN_DEPTH_VAL || p_in_img_d[tidx]>MAX_DEPTH_VAL)		p_out_img_d[tidx]=(unsigned char)0;	
				else{
					p_out_img_d[tidx]=(unsigned char)(255.0f-255.0f*(float)((int)p_in_img_d[tidx]-min_d)/(float)(max_d-min_d));
					num_pix++;
				}
			}
			else		p_out_img_d[tidx]=(unsigned char)0;
		}
	}

	return num_pix;

}

//********************************************************************************************
void LCKvOpenNI::cid8_ConvertImageDepth8bit(CKvMatrixUshort *in_imgDepth,
	int in_modeConversion,
	CKvMatrixUchar *out_imgDepth)
//********************************************************************************************
{
	int widthOut, heightOut, dMax, dMin;
	uchar *pImgDepthOut;
	ushort *pImgDepthIn;

	pImgDepthOut = out_imgDepth->mps(heightOut, widthOut)[0];
	pImgDepthIn = in_imgDepth->vp();

	if(widthOut != in_imgDepth->mw() || heightOut != in_imgDepth->mh()){

		widthOut = in_imgDepth->mw();	heightOut = in_imgDepth->mh();
		pImgDepthOut = out_imgDepth->c_Create(heightOut, widthOut, (uchar)0)[0];
	}

	dMax=-10000000;
	dMin=10000000;

	// find min/max depth value.
	for(int i=0; i<widthOut*heightOut; i++){
		if(pImgDepthIn[i]<MIN_DEPTH_VAL || pImgDepthIn[i]>MAX_DEPTH_VAL)		continue;
		if(pImgDepthIn[i]<dMin)		dMin=pImgDepthIn[i];
		if(pImgDepthIn[i]>dMax)		dMax=pImgDepthIn[i];
	}

	// convert raw depth value to 8bit.
	float scale;
	scale = 1.0f / (dMax - dMin);
	for(int i=0; i<widthOut*heightOut; i++){
		if(pImgDepthIn[i]==0)	continue;
		if(in_modeConversion==KV_CONV_IMAGE_DEPTH_FL_TO_DISP_UC)
			pImgDepthOut[i]=(uchar)(255.0f-255.0f*scale*(float)((int)pImgDepthIn[i]-dMin));	// disparity.
		else
			pImgDepthOut[i] = (uchar)(255.0f*scale*(float)((int)pImgDepthIn[i] - dMin));	// depth.
	}

}

//********************************************************************************************
void LCKvOpenNI::cid8_ConvertImageDepth8bit(CKvMatrixFloat *in_imgDepth,
	int in_modeConversion,
	CKvMatrixUchar *out_imgDepth)
//********************************************************************************************
{
	int widthOut, heightOut;
	float dMax, dMin;
	uchar *pImgDepthOut;
	float *pImgDepthIn;

	pImgDepthOut = out_imgDepth->mps(heightOut, widthOut)[0];
	pImgDepthIn = in_imgDepth->vp();

	if(widthOut != in_imgDepth->mw() || heightOut != in_imgDepth->mh()){

		widthOut = in_imgDepth->mw();	heightOut = in_imgDepth->mh();
		pImgDepthOut = out_imgDepth->c_Create(heightOut, widthOut, (uchar)0)[0];
	}

	dMax=-1000000.0f;
	dMin=1000000.0f;

	// find min/max depth value.
	for(int i=0; i<widthOut*heightOut; i++){
		if(pImgDepthIn[i]<0.001f*MIN_DEPTH_VAL || pImgDepthIn[i]>0.001f*MAX_DEPTH_VAL)		continue;
		if(pImgDepthIn[i]<dMin)		dMin=pImgDepthIn[i];
		if(pImgDepthIn[i]>dMax)		dMax=pImgDepthIn[i];
	}

	// convert raw depth value to 8bit.
	float scale;
	scale = 1.0f / (dMax - dMin);
	for(int i=0; i<widthOut*heightOut; i++){
		if(pImgDepthIn[i]==0)	continue;
		if(in_modeConversion==KV_CONV_IMAGE_DEPTH_FL_TO_DISP_UC)
			pImgDepthOut[i]=(uchar)(255.0f-255.0f*scale*(float)((int)pImgDepthIn[i]-dMin));	// disparity.
		else
			pImgDepthOut[i] = (uchar)(255.0f*scale*(float)((int)pImgDepthIn[i] - dMin));	// depth.
	}

}