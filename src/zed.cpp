/*
 * zed.cpp
 *
 *  Created on: Feb 6, 2017
 *      Author: team2168
 */


#include "zed.h"

ZedInterface::ZedInterface()
{
	zedcam = new sl::zed::Camera(sl::zed::VGA);
	std::cout<<"hello from zed";
	ZEDInit = false;

}

void ZedInterface::ZED_init()
{
    // Define a struct of parameters for the initialization
    sl::zed::InitParams params;

    params.mode = sl::zed::PERFORMANCE;
    params.unit = sl::zed::INCH;
    params.coordinate = sl::zed::IMAGE;


    // Enables verbosity in the console
    params.verbose = true;



    sl::zed::ERRCODE err = zedcam->init(params);

    //put cam on single core
    sl::zed::Camera::sticktoCPUCore(2);
    sl::zed::ZED_SELF_CALIBRATION_STATUS old_self_calibration_status = sl::zed::SELF_CALIBRATION_NOT_CALLED;

    viewID = 0;	// Define a struct of parameters for the initialization

	zedcam->setDepthClampValue(20000);

	if (sl::zed::errcode2str(err) == "SUCCESS")
		ZEDInit = true;
}

void ZedInterface::ZED_init_VGA()
{
	zedcam = new sl::zed::Camera(sl::zed::VGA);
	ZED_init();
}

void ZedInterface::ZED_init_HD720()
{
	zedcam = new sl::zed::Camera(sl::zed::HD720);
	ZED_init();
}

bool ZedInterface::getNewFrame()
{
	bool res;
	return res = zedcam->grab(dm_type);
}

void ZedInterface::getCurrentImage(cv::Mat& frame)
{

	slMat2cvMat(zedcam->retrieveImage(static_cast<sl::zed::SIDE> (viewID))).copyTo(frame);

}

void ZedInterface::getCurrentDepthImage(cv::Mat& frame)
{
	depth = zedcam->retrieveMeasure(sl::zed::MEASURE::DEPTH);
	slMat2cvMat(zedcam->normalizeMeasure(sl::zed::MEASURE::DISPARITY)).copyTo(frame);

}

double ZedInterface::getDepthAtPoint(int x, int y, int width, int height)
{

	depth = zedcam->retrieveMeasure(sl::zed::MEASURE::DEPTH);
	depthData = (float*)depth.data;
	int depthStep = depth.step;

    //int y_int = (y * depthData->_image.height / data->_resize.height);
    //int x_int = (x * depthData->_image.width / data->_resize.width);

	float sum = 0;
	//double array[] = new double[windowSize*windowSize];

	for(int i=x-width/2; i<x+width/2; i++)
		for(int j=y-height/2; j<y+height/2; j++)
		{
			float* ptr_image_num = (float*) ((int8_t*) depthData + y * depthStep);
			sum += ptr_image_num[x];
		}

	return sum/(width*height);



	//return (double)zedcam->normalizeMeasure(sl::zed::MEASURE::DEPTH).getValue(y,x).c1;
}

int ZedInterface::getCameraWidth()
{
	return zedcam->getImageSize().width;
}

int ZedInterface::getCameraHeight()
{
	return zedcam->getImageSize().height;
}

void ZedInterface::setZEDBrightness(int value)
{
	zedcam->setCameraSettingsValue(sl::zed::ZEDCamera_settings::ZED_BRIGHTNESS, value); //0 to 8
}
void ZedInterface::setZEDContrast(int value)
{
	zedcam->setCameraSettingsValue(sl::zed::ZEDCamera_settings::ZED_CONTRAST, value); //0 to 8
}

void ZedInterface::setZEDExposure(int value)
{
	zedcam->setCameraSettingsValue(sl::zed::ZEDCamera_settings::ZED_EXPOSURE, value); //-1 to 100
}
void ZedInterface::setZEDHue(int value)
{
	zedcam->setCameraSettingsValue(sl::zed::ZEDCamera_settings::ZED_HUE, value); //-1 to 100
}
void ZedInterface::setZEDSaturation(int value)
{
	zedcam->setCameraSettingsValue(sl::zed::ZEDCamera_settings::ZED_SATURATION, value); //-1 to 100
}
void ZedInterface::setZEDGain(int value)
{
	zedcam->setCameraSettingsValue(sl::zed::ZEDCamera_settings::ZED_GAIN, value); //-1 to 100
}

void ZedInterface::setZedCamSetting(int zedBrightness, int zedContrast, int zedExposure, int zedHue, int zedSat, int zedGain)
{
		setZEDBrightness(zedBrightness);
		setZEDContrast(zedContrast);
		setZEDExposure(zedExposure);
		setZEDHue(zedHue);
		setZEDSaturation(zedSat);
		setZEDGain(zedGain);
}

bool ZedInterface::isZEDInit()
{
	return ZEDInit;
}


