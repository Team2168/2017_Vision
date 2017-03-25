/*
 * zed.h
 *
 *  Created on: Feb 6, 2017
 *      Author: team2168
 */

#ifndef ZED_H_
#define ZED_H_


#include <zed/Camera.hpp>
#include <opencv2/core.hpp>




class ZedInterface
{
private:
	sl::zed::Mat depth;
	sl::zed::Mat image;
	sl::zed::Camera* zedcam;
	int viewID = 0;
	sl::zed::SENSING_MODE dm_type = sl::zed::STANDARD;
	float* depthData;
	bool ZEDInit;

public:
	ZedInterface();
    void ZED_init();
    void ZED_init_VGA();
    void ZED_init_HD720();
    bool getNewFrame();
    void getCurrentImage(cv::Mat& frame);
    void getCurrentDepthImage(cv::Mat& depth);
    double getDepthAtPoint(int x, int y, int width=40, int height=40);
    int getCameraWidth();
    int getCameraHeight();
    void setZEDBrightness(int value);
    void setZEDContrast(int value);
    void setZEDExposure(int value);
    void setZEDHue(int value);
    void setZEDSaturation(int value);
    void setZEDGain(int value);
    void setZedCamSetting(int zedBrightness, int zedContrast, int zedExposure, int zedHue, int zedSat, int zedGain);
    bool isZEDInit();

};




#endif /* ZED_H_ */
