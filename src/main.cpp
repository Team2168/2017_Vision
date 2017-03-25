// dat vizion doh

#define _USE_MATH_DEFINES
#define VIEW_ANGLE 34.8665269
#define AUTO_STEADY_STATE 1.9

#define TARGET_WIDTH_IN 15
#define TARGET_HEIGHT_IN 4
#define CAMERA_WIDTH_FOV_ANGLE_RAD 0.743879868
#define CAMERA_HEIGHT_FOV_ANGLE_RAD 0.991839824
#define ROBOT_ANGLE_OFFSET 0.0


#include "mjpeg_server.h"
#include <unistd.h>
#include "tcp_client.h"
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <string>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <pthread.h>
#include <signal.h> //to ignore sigpipe
#include <sys/time.h>
#include "zed.h"
#include <iostream>
#include <algorithm>
#include <vector>
#include <cstdio>
#include <math.h>




using namespace cv;
using namespace std;

//struct to define program execution variables passed in from the command line
struct ProgParams
{
	string ROBOT_IP;
	string ROBOT_PORT;
	string CAMERA_IP;
	string IMAGE_FILE;

	int MJPEG_STREAM_PORT;
	int ROTATE;

	bool From_Camera;
	bool From_File;
	bool Visualize;
	bool Timer;
	bool Debug;
	bool Process;
	bool USB_Cam;
	bool FPS;
	bool Log;
	bool USE_DROID;
	bool USE_ZED;
	bool Threshold;
	bool Gear;
	bool Boiler;
};

//Stuct to hold information about targets found
struct Target
{

	Rect Target;

	//camera bool
	bool cameraConnected;

	//Dist and Angle
	double TargetBoxAngle;
	double TargetBearing;
	double targetDistance;
	double targetScore;
	double targetBearingCenter;

	//Valild match
	bool matchStart;



	int numberOfContoursDetected;

	bool isTargetDetected;
	bool isTargetScoreable;
	bool isProcessThreadRunning;
	bool isMjpegClientConnected;

};

//function declarations
//TODO: add pre- and post- comments for each function
void parseCommandInputs(int argc, const char* argv[], ProgParams &params);
void printCommandLineUsage();
Mat GetOriginalImage(const ProgParams& params);
void initializeParams(ProgParams& params);
double diffClock(timespec start, timespec end);
Mat ThresholdImage(Mat img);
void findTarget(Mat original, Mat thresholded, Target& targets, const ProgParams& params);
void NullTargets(Target& target);
void CalculateDist(Target& targets);
void CalculateBearing(Target& targets);
double CalculateHorizontalAngle(double targetYPix, double targetHeightPix);
void onMouse( int event, int x, int y, int, void* );
double interpolate(double x, vector<pair<double, double> > table);

//Threaded TCP Functions
void *TCP_thread(void *args);
void *TCP_Send_Thread(void *args);
void *TCP_Recv_Thread(void *args);
void error(const char *msg);
void *MJPEG_Server_Thread(void *args);
void *MJPEG_host(void *args);

//Threaded Video Capture Function
void *VideoCap(void *args);
void rot90(cv::Mat &matImage, int rotflag);


//GLOBAL CONSTANTS
const double PI = 3.141592653589793;
const double INF = 1.e100;

////Thresholding parameters
//int minH = 30;
//int maxH = 112;
//int minS = 180; //160 for ip cam, 80 to support MS webcam
//int maxS = 255;
//int minV = 20;
//int maxV = 255;

int minH = 30;
int maxH = 124;
int minS = 175; //160 for ip cam, 80 to support MS webcam
int maxS = 255;
int minV = 69;
int maxV = 245;

//Target Ratio Ranges
double MinHRatio = 1.0; //1.0 2016
double MaxHRatio = 1.5; //1.5 2016

double MinVRatio = 0.3; //0.3 2016
double MaxVRatio = 1; //1 2016

int MAX_SIZE = 2000;

//Some common colors to draw with
const Scalar RED = Scalar(0, 0, 255),
			BLUE = Scalar(255, 0, 0),
			GREEN = Scalar(0, 255, 0),
			ORANGE = Scalar(0, 128, 255),
			YELLOW = Scalar(0, 255, 255),
			PINK = Scalar(255, 0,255),
			OFF_COLOR = Scalar(200, 100, 50),
			WHITE = Scalar(255, 255, 255);

static int mouse_h = -1;
static int mouse_s = -1;
static int mouse_v = -1;
double depthAtPoint = -1;

int FOV_WIDTH_PIX = 320;
int  FOV_HEIGHT_PIX = 240;
int TARGET_SCORE = 55;


//GLOBAL MUTEX LOCK VARIABLES
pthread_mutex_t targetMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t matchStartMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t frameMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mjpegServerFrameMutex = PTHREAD_MUTEX_INITIALIZER;

//GLOBAL MUTEX SIGNAL VARIABLES
pthread_cond_t newFrameToStreamSignal = PTHREAD_COND_INITIALIZER;
pthread_cond_t FrameCopyCompleteSignal = PTHREAD_COND_INITIALIZER;
pthread_cond_t targetUpdateCompleteSignal = PTHREAD_COND_INITIALIZER;


bool isNewFrameToStream = false;
bool isFrameCopyComplete = false;
bool isTargetUpdateComplete = false;


//Thread Variables
pthread_t TCPthread;
pthread_t TCPsend;
pthread_t TCPrecv;
pthread_t MJPEG;
pthread_t AutoCounter;
pthread_t MJPEG_S_Thread;
pthread_t MJPEGHost;

//TCP Steam
tcp_client client;

//MJPEG Stream
mjpeg_server mjpeg_s;
double MJPEG_SERVER_PORT = 5800;

//Store targets in global variable
Target targets;
Mat frame;
Mat imgToStream;

//Global Timestamps for auto
struct timespec autoStart, autoEnd;

ZedInterface cam;
int zedBrightness = 4; //0 8
int zedContrast = 4; //0 8
int zedExposure = -1; // -1 8
int zedHue = 5; //0 11
int zedSat = 4; //0 8
int zedGain = -1; //-1 100

//Control process thread exectution
bool progRun;
bool readyToStream;

char tbuffer[30];

cv::VideoCapture vcap;


//target grade functions
vector<pair<double, double> > WHRatioGradePlot = { {-1000, 0}, {0, 0}, {0.4, 25}, {0.8, 0}, {1000, 0}};

int main(int argc, const char* argv[])
{
	signal(SIGPIPE, SIG_IGN); // ignore sigpipes

	//Read command line inputs to determine how the program will execute
	ProgParams params;
	parseCommandInputs(argc, argv, params);









	if(params.Log)
	{
		struct timeval tv;

		time_t curtime;

		gettimeofday(&tv,NULL);
		curtime=tv.tv_sec;

		strftime(tbuffer,30,"%y-%m-%d_%H_%M_%S",localtime(&curtime));

		char dirOrig[80];
		char dirProcess[80];

		system("mkdir /home/ubuntu/VisionLogs/");
		strcpy(dirOrig,"mkdir /home/ubuntu/VisionLogs/");
		strcat(dirOrig,tbuffer);
		strcat(dirOrig,"/");
		system(dirOrig);
		strcpy(dirProcess,dirOrig);
		strcat(dirOrig,"Originals/");
		strcat(dirProcess, "Processed/");

		system(dirOrig);
		system(dirProcess);
	}




	//start mjpeg stream thread
	pthread_create(&MJPEG, NULL, VideoCap, &params);

	//Create Local Processing Image Variables
	Mat img, thresholded, output;


	//initialize variables so processing loop is false;
	targets.matchStart = false;
	targets.cameraConnected = false;
	targets.isTargetDetected = false;
	progRun = false;


	//start TCP Server
	pthread_create(&TCPthread, NULL, TCP_thread, &params);

	struct timespec start, end;

	pthread_create(&MJPEG_S_Thread, NULL, MJPEG_Server_Thread, &params);

	//run loop forever
	while (true)
	{
		//check if program is allowed to run
		//this bool, is enabled by the mjpeg thread
		//once it is up to 10fps

		if (params.Process && progRun)
		{

			//Lock Targets and determine goals
			pthread_mutex_lock(&targetMutex);

			targets.isProcessThreadRunning = true;
			pthread_mutex_unlock(&targetMutex);


			//start clock to determine our processing time;
			clock_gettime(CLOCK_REALTIME, &start);
			//pthread_mutex_lock(&frameMutex);

			//Only run processing thread on new frames, if a new frame
			//isn't available this thread will sleep, until the frame
			//grabber thread signals the condition to wake this thread
			//up
			while(!isFrameCopyComplete)
				pthread_cond_wait(&FrameCopyCompleteSignal, &frameMutex);


			if (!frame.empty())
			{
				frame.copyTo(img);
				pthread_mutex_unlock(&frameMutex);


				if(params.Threshold)
				{
					string window = "thresh";

					if(mouse_h != -1)
					{
						ostringstream output;
						output << "H_Val: " << mouse_h <<"  S_Val: "<<mouse_s<<"  V_val: "<<mouse_v <<"  dist: "<<depthAtPoint;
						putText(img, output.str(), Point(20 + 10, 20 + 20), FONT_HERSHEY_PLAIN, 1, WHITE, 1, 1);

					}

					imshow(window, img);
					//set the callback function for any mouse event
					setMouseCallback(window, onMouse, &img);

					createTrackbar( "Hue_Min",window, &minH, 180, NULL );
					createTrackbar( "Hue_Max",window, &maxH, 180, NULL );
					createTrackbar( "Saturation_Min",window, &minS, 255,NULL );
					createTrackbar( "Saturation_Max",window, &maxS, 255,NULL );
					createTrackbar( "Value_Min",window, &minV, 255,NULL);
					createTrackbar( "Value_Max",window, &maxV, 255,NULL);


					string camTune = "came Tune";
					imshow(camTune, img);
//
					if(cam.isZEDInit()){

						createTrackbar("ZED Brightness",camTune, &zedBrightness, 8, NULL );
						createTrackbar("ZED Contrast",camTune, &zedContrast, 8, NULL );
						createTrackbar("ZED Exposure",camTune, &zedExposure, 100, NULL );
						createTrackbar("ZED Hue",camTune, &zedHue, 11, NULL );
						createTrackbar("ZED Saturation",camTune, &zedSat, 8, NULL );
						createTrackbar("ZED Gain",camTune, &zedGain, 100, NULL );
					}

					else{

						createTrackbar("ZED Brightness",camTune, &zedBrightness, 100, NULL );
						createTrackbar("ZED Contrast",camTune, &zedContrast, 100, NULL );
						createTrackbar("ZED Exposure",camTune, &zedExposure, 100, NULL );
						createTrackbar("ZED Hue",camTune, &zedHue, 100, NULL );
						createTrackbar("ZED Saturation",camTune, &zedSat, 100, NULL );
						createTrackbar("ZED Gain",camTune, &zedGain, 100, NULL );
					}


				}





				thresholded = ThresholdImage(img);

				//Lock Targets and determine goals
				pthread_mutex_lock(&targetMutex);
				findTarget(img, thresholded, targets, params);

				if(params.Debug)
				{
					cout<<"Verified Target:" << endl;
					cout<<"\tDist:" <<targets.targetDistance<<endl;
					cout<<"\tRotation: " <<targets.TargetBearing<<endl;
				}
				pthread_mutex_unlock(&targetMutex);

				clock_gettime(CLOCK_REALTIME, &end);

				if(params.Timer)
					cout << "It took " << diffClock(start,end) << " seconds to process frame \n";

				if(params.FPS)
					cout << "Processing at "  << 1/diffClock(start,end) << " FPS \n";




			}
			else
			{
				cout<<"FRAME IS EMPTY"<<endl;
			}

			pthread_mutex_unlock(&frameMutex);

			if(params.Visualize)
				waitKey(5);

		}

		usleep(20000); //20000 sleep for 5ms); // run 40 times a second
	}

	//if we end the process code, wait for threads to end
	pthread_join(TCPthread, NULL);
	pthread_join(TCPsend, NULL);
	pthread_join(TCPrecv, NULL);

	pthread_join(MJPEG, NULL);
	pthread_join(MJPEG_S_Thread, NULL);


	//done
	return 0;

}

///////////////////FUNCTIONS/////////////////////

/**
 * This function uses the law of lense projection to
 * estimate the distance to an object of known height only
 * using a single camera.
 *
 * This function uses only the vertical target height, the
 * pixel height of the image, and the view angle of the
 * camera lense.
 */
double CalculateDist(double targetWidthPix)
{
	//d = Tft*FOVpixel/(2*Tpixel*tanΘ)
	//targets.targetDistance = (TARGET_WIDTH_IN * FOV_WIDTH_PIX)/(targets.Target.width * 2.0 * tan(CAMERA_WIDTH_FOV_ANGLE_RAD / 2.0));
	return (TARGET_WIDTH_IN * FOV_WIDTH_PIX)/(targetWidthPix * 2.0 * tan(CAMERA_WIDTH_FOV_ANGLE_RAD / 2.0));

}

double CalculateBearing(double targetXPix, double targetWidthPix)
{
//	double x = targets.Target.x + (targets.Target.width / 2);
//	double x_target_on_FOV = ((2 * x) / (FOV_WIDTH_PIX)) - 1;
//	double bearing = ((x_target_on_FOV) * (CAMERA_WIDTH_FOV_ANGLE_RAD / 2)) * (-180 / PI) - ROBOT_ANGLE_OFFSET;
//	targets.TargetBearing = bearing;

	double x = targetXPix + (targetWidthPix / 2);
	double x_target_on_FOV = ((2 * x) / (FOV_WIDTH_PIX)) - 1;
	return ((x_target_on_FOV) * (CAMERA_WIDTH_FOV_ANGLE_RAD / 2)) * (-180 / PI) - ROBOT_ANGLE_OFFSET;

}

double CalculateVerticalAngle(double targetXPix, double targetWidthPix)
{
//	double x = targets.Target.x + (targets.Target.width / 2);
//	double x_target_on_FOV = ((2 * x) / (FOV_WIDTH_PIX)) - 1;
//	double bearing = ((x_target_on_FOV) * (CAMERA_WIDTH_FOV_ANGLE_RAD / 2)) * (-180 / PI) - ROBOT_ANGLE_OFFSET;
//	targets.TargetBearing = bearing;

	double x = targetXPix + (targetWidthPix / 2);
	double x_target_on_FOV = ((2 * x) / (FOV_WIDTH_PIX)) - 1;
	return ((x_target_on_FOV) * (CAMERA_WIDTH_FOV_ANGLE_RAD / 2)) * (-180 / PI) - ROBOT_ANGLE_OFFSET;

}

double CalculateHorizontalAngle(double targetYPix, double targetHeightPix)
{
//	double x = targets.Target.x + (targets.Target.width / 2);
//	double x_target_on_FOV = ((2 * x) / (FOV_WIDTH_PIX)) - 1;
//	double bearing = ((x_target_on_FOV) * (CAMERA_WIDTH_FOV_ANGLE_RAD / 2)) * (-180 / PI) - ROBOT_ANGLE_OFFSET;
//	targets.TargetBearing = bearing;

	double y = targetYPix + (targetHeightPix / 2);
	double y_target_on_FOV = ((2 * y) / (FOV_HEIGHT_PIX)) - 1;
	return ((y_target_on_FOV) * (CAMERA_HEIGHT_FOV_ANGLE_RAD / 2)) * (-180 / PI) - ROBOT_ANGLE_OFFSET;

}

/**
 * This function scans through an image and determins
 * if rectangles exist which match the profile of a
 * "Hot Goal".
 *
 * The "Hot Goal" is specific to the 2014 FRC game
 * and is identified as a horizontal and vertical target
 * in the same frame, with known width and height.
 */
void findTarget(Mat original, Mat thresholded, Target& targets, const ProgParams& params)
{

	char buffer[30];
	struct timeval tv;

	time_t curtime;

	gettimeofday(&tv,NULL);
	curtime=tv.tv_sec;

	strftime(buffer,30,"%y-%m-%d_%H_%M_%S",localtime(&curtime));

	char fileSave[80];

	if(params.Log)
	{
		strcpy(fileSave,"/home/ubuntu/VisionLogs/");
		strcat(fileSave,tbuffer);
		strcat(fileSave,"/Originals/");
		strcat(fileSave,buffer);
		strcat(fileSave,".png");
		imwrite(fileSave,original);
		cout<<"writing_to_file: "<<fileSave<<endl<<endl;
	}


	vector<Vec4i> hierarchy;
	vector<vector<Point> > contours;

	//put cross hair on image
	line(original, Point(FOV_WIDTH_PIX / 2, 0), Point(FOV_WIDTH_PIX/ 2, FOV_HEIGHT_PIX), WHITE, 2);
	line(original, Point(0, FOV_HEIGHT_PIX / 2), Point(FOV_WIDTH_PIX, FOV_HEIGHT_PIX / 2), WHITE, 2);

	//draw shooting distance threshold lines
	line(original, Point(0, 260), Point(FOV_WIDTH_PIX, 260), GREEN, 3);
	line(original, Point(0, 174), Point(FOV_WIDTH_PIX, 174), GREEN, 3);

	//Find close contours
	findContours(thresholded, contours, hierarchy, cv::RETR_EXTERNAL,
			cv::CHAIN_APPROX_SIMPLE);

	if(params.Debug)
	{
		cout << "Contours: " << contours.size() << endl;
		cout << "Hierarchy: " << hierarchy.size() << endl;
	}

	//Find 2 largest contours
	int largestIndex = 0;
	unsigned int largestContour = 0;
	int secondLargestIndex = 0;
	unsigned int secondLargestContour = 0;
	for( unsigned int i = 0; i< contours.size(); i++ )
	{
	    if(contours[i].size() >= largestContour){
	        secondLargestContour = largestContour;
	        secondLargestIndex = largestIndex;
	        largestContour = contours[i].size();
	        largestIndex = i;
	    }else if(contours[i].size() >= secondLargestContour){
	        secondLargestContour = contours[i].size();
	        secondLargestIndex = i;
	    }
	}

	//delete everything but two largest contour
	for (vector<vector<Point> >::iterator it = contours.begin();
			it != contours.end();)
	{

		cout<<"Contour Size: "<<it->size()<<endl;
		//if (it->size() < secondLargestContour && contours.size() > 2)
		if (it->size() < secondLargestContour)
			it = contours.erase(it);
		else
			++it;

	}


	if(params.Debug)
	{
	cout << "Contours after: " << contours.size() << endl;
	cout << "Hierarchy after: " << hierarchy.size() << endl;
	}


	//Confirm we only have 2 contours left by area
	if (contours.size() > 2)
	{
		vector<RotatedRect> minRectTest(contours.size());
		double largestArea = 200;
		for (vector<vector<Point> >::iterator it = contours.begin();
				it != contours.end();)
		{

			int i = it - contours.begin();
			//capture corners of contour
			minRectTest[i] = minAreaRect(Mat(contours[i]));
			Rect box = minRectTest[i].boundingRect();

			double area = box.height*box.width;

			//if (area > largestArea && contours.size() > 2)
			if (area > largestArea)
			{
				largestArea = area;
				++it;
			}
			else
				it = contours.erase(it);

		}


	}


	//Rotate image
		if(params.ROTATE == 1 || params.ROTATE == 2 || params.ROTATE == 3)
			rot90(original,params.ROTATE);


	/// Draw contours
	Mat drawing = Mat::zeros(original.size(), CV_8UC3);
	NullTargets(targets);

		//Vector for Min Area Boxes
	vector<RotatedRect> minRect(contours.size());

	double contour0Score =0;
	double contour1Score = 0;
	bool contour0 = false;
	double slopeAngle;

	//run through 2 large contours to grade them to get BEST target
	if (!contours.empty() && !hierarchy.empty())
	{

		int xC[contours.size()];
		int yC[contours.size()];


		for (unsigned int i = 0; i < contours.size(); i++)
		{
			//capture corners of contour
			minRect[i] = minAreaRect(Mat(contours[i]));

			if(params.Visualize)
			{

				//if(hierarchy[i][100] != -1)
				drawContours(original, contours, i, RED, 2, 8, hierarchy, 0,Point());
				//draw a minimum box around the target in green
				Point2f rect_points[4];
				minRect[i].points(rect_points);
				for (int j = 0; j < 4; j++)
					line(original, rect_points[j], rect_points[(j + 1) % 4], BLUE, 1, 8);
			}
			//define minAreaBox
			Rect box = minRect[i].boundingRect();
			//targets.TargetBoxAngle = minRect[i].angle;

			double WHRatio = box.width / ((double) box.height);
			double HWRatio = ((double) box.height) / box.width;



			xC[i] = box.x + box.width / 2;
			yC[i] = box.y + box.height / 2;

			//calculate distance and target to each target
			//calculate distance and target to each target
			double dist = 0;
			if(params.USE_ZED)
				dist = cam.getDepthAtPoint(box.x,box.y, box.width, box.height);
			else
				dist = CalculateDist(box.width);
			double bearing = CalculateBearing(box.x,box.width);

			//ID the center in yellow
			Point center(box.x + box.width / 2, box.y + box.height / 2);
			line(original, center, center, YELLOW, 3);



			const int WEIGHT = 25;
			double angleGrade, WHRatioGrade, distGrade, bearingGrade;
			double score;

			if(params.Gear){

				//0 if angle > 60,
				//abs(angle) > 60 = 0
				if ((abs(minRect[i].angle) >= 0 && abs(minRect[i].angle) < 22) || (abs(minRect[i].angle) > 77 && abs(minRect[i].angle) <= 90))
					angleGrade = 25;
				else
					angleGrade = 0;
					//angleGrade = WEIGHT - (WEIGHT* abs(minRect[i].angle) / 60);


				if (WHRatio < -1 || WHRatio > 1)
					WHRatioGrade = 0;
				else
					WHRatioGrade = interpolate(WHRatio, WHRatioGradePlot);
					//WHRatioGrade = WEIGHT - (WEIGHT*abs(WHRatio-0.4)/0.4);

				if (dist > 400 || dist<=0)
					distGrade = 0;
				else
					distGrade = WEIGHT - (WEIGHT* abs(dist-160) / 160);

				bearingGrade = WEIGHT - abs(bearing);


				if(angleGrade < 0)
					angleGrade = 0;

				if(WHRatioGrade < 0 )
					WHRatioGrade = 0;

				if(distGrade < 0)
					distGrade = 0;

				if(bearingGrade < 0)
					bearingGrade = 0;

				score = angleGrade+WHRatioGrade+distGrade+bearingGrade;

			}

			if(params.Boiler){

				//0 if angle > 60,
				//abs(angle) > 60 = 0
				if ((abs(minRect[i].angle) >= 0 && abs(minRect[i].angle) < 22) || (abs(minRect[i].angle) > 77 && abs(minRect[i].angle) <= 90))
					angleGrade = 25;
				else
					angleGrade = 0;
					//angleGrade = WEIGHT - (WEIGHT* abs(minRect[i].angle) / 60);


				if (WHRatio < -1 || WHRatio > 1)
					WHRatioGrade = 0;
				else
					WHRatioGrade = interpolate(WHRatio, WHRatioGradePlot);
					//WHRatioGrade = WEIGHT - (WEIGHT*abs(WHRatio-0.4)/0.4);

				if (dist > 400 || dist<=0)
					distGrade = 0;
				else
					distGrade = WEIGHT - (WEIGHT* abs(dist-160) / 160);

				bearingGrade = WEIGHT - abs(bearing);


				if(angleGrade < 0)
					angleGrade = 0;

				if(WHRatioGrade < 0 )
					WHRatioGrade = 0;

				if(distGrade < 0)
					distGrade = 0;

				if(bearingGrade < 0)
					bearingGrade = 0;

				score = angleGrade+WHRatioGrade+distGrade+bearingGrade;

			}

			slopeAngle = atan2(yC[1] - yC[0], xC[1] - xC[0]);

			if(i==0){
				contour0Score=score;
				contour0 = true;
			}else{
				contour1Score=score;
				contour0 = false;
			}

			if(params.Debug)
			{
				cout<<"Contour: "<<i<<endl;
				cout<<"\tX: "<<box.x<<endl;
				cout<<"\tY: "<<box.y<<endl;
				cout<<"\tHeight: "<<box.height<<endl;
				cout<<"\tWidth: "<<box.width<<endl;
				cout<<"\tangle: "<<minRect[i].angle<<endl;
				cout<<"\tRatio (W/H): "<<WHRatio<<endl;
				cout<<"\tRatio (H/W): "<<HWRatio<<endl;
				cout<<"\tArea: "<<box.height*box.width<<endl;

				cout<<"\tangleGrade: "<<angleGrade<<endl;
				cout<<"\tratioGrade: "<<WHRatioGrade<<endl;
				cout<<"\tdistGrade: "<<distGrade<<endl;
				cout<<"\tbearGrade: "<<bearingGrade<<endl;
				cout<<"\tTotalScore: "<<score<<endl;

				cout<<"HSV Values: "<<endl;
				cout<<"\tH Min:"<<minH<<endl;
				cout<<"\tH Max:"<<maxH<<endl;

				cout<<"\tS Min:"<<minS<<endl;
				cout<<"\tS Max:"<<maxS<<endl;

				cout<<"\tV Min:"<<minV<<endl;
				cout<<"\tV Max:"<<maxV<<endl;

				cout<<"\SlopeAngle:"<<slopeAngle<<endl;

			}

		}


		int i =0;
		if(contour0Score >= TARGET_SCORE)
			{

						i = 0;
						//capture corners of contour
						minRect[i] = minAreaRect(Mat(contours[i]));

						if(params.Visualize)
						{

							Point2f rect_points[4];
							minRect[i].points(rect_points);
							for (int j = 0; j < 4; j++)
								line(original, rect_points[j], rect_points[(j + 1) % 4], ORANGE, 2, 8);
						}
						//define minAreaBox
						Rect box = minRect[i].boundingRect();
						targets.TargetBoxAngle = minRect[i].angle;


						double WHRatio = box.width / ((double) box.height);
						double HWRatio = ((double) box.height) / box.width;

						//calculate distance and target to each target
						double dist = 0;
						if(params.USE_ZED)
							dist = cam.getDepthAtPoint(box.x,box.y, box.width, box.height);
						else
							dist = CalculateDist(box.width);
						double bearing = CalculateBearing(box.x,box.width);
						double bearing_center = CalculateHorizontalAngle(box.x, box.width);


						targets.Target = box;
						targets.TargetBearing = bearing;
						targets.targetBearingCenter = bearing_center;
						targets.targetDistance = dist;
						targets.targetScore = contour0Score;

						targets.isTargetDetected = true;

						if (abs(bearing) <= 1.2)
							targets.isTargetScoreable = true;
						else
							targets.isTargetScoreable = false;
			}
		if (contour1Score >= TARGET_SCORE)
		{
				i=1;

				//capture corners of contour
							minRect[i] = minAreaRect(Mat(contours[i]));

							if(params.Visualize)
							{

								Point2f rect_points[4];
								minRect[i].points(rect_points);
								for (int j = 0; j < 4; j++)
									line(original, rect_points[j], rect_points[(j + 1) % 4], ORANGE, 2, 8);
							}
							//define minAreaBox
							Rect box = minRect[i].boundingRect();
							targets.TargetBoxAngle = minRect[i].angle;


							double WHRatio = box.width / ((double) box.height);
							double HWRatio = ((double) box.height) / box.width;

							//calculate distance and target to each target
							double dist = 0;
							if(params.USE_ZED)
								dist = cam.getDepthAtPoint(box.x,box.y, box.width, box.height);
							else
								dist = CalculateDist(box.width);

							double bearing = CalculateBearing(box.x,box.width);
							double bearing_center = CalculateHorizontalAngle(box.x, box.width);



							targets.Target = box;
							//targets.TargetBearing = bearing; //remove for 2017
							targets.targetBearingCenter = bearing_center;
							targets.targetDistance = dist;
							targets.targetScore = contour1Score;

							targets.isTargetDetected = true;

							if (abs(bearing) <= 1.2)
								targets.isTargetScoreable = true;
							else
								targets.isTargetScoreable = false;
		}


		//additional scoring
		if(contour1Score >= TARGET_SCORE && contour0Score >= TARGET_SCORE)
		{
			const int WEIGHT = 25;
			double WHRatioComparisonGrade, WidthToDistGrade, VerticalGrade, HorizontalGrade, TotalScore;
			//ratios roughly equal


			//width of box to dist between ratio

			//if gear, y loc roughly same (horizontal), else if boiler, x roughly same (vertical)

			double yDiff = abs(yC[0] - yC[1]);
			if (yDiff > 400 )
				HorizontalGrade = 0;
			else
				HorizontalGrade = WEIGHT - (yDiff*.2); //every pixel 1/2 point reduction


			//

			TotalScore = HorizontalGrade;

			if (TotalScore >15)
			{
				line(original, Point(xC[0],yC[0]), Point(xC[1],yC[1]), ORANGE, 2, 8);

			//find the left most target and use to calculate center of line between them
			if (xC[0] < xC[1])
				targets.TargetBearing = CalculateBearing(xC[0], xC[1] - xC[0]);
			else
				targets.TargetBearing = CalculateBearing(xC[1], xC[0] - xC[1]);

			}
			if(params.Debug)
						{
							cout<<"Comparison Grade: "<<i<<endl;
							cout<<"\tyDiff: "<<yDiff<<endl;
//							cout<<"\tY: "<<box.y<<endl;
//							cout<<"\tHeight: "<<box.height<<endl;
//							cout<<"\tWidth: "<<box.width<<endl;
//							cout<<"\tangle: "<<minRect[i].angle<<endl;
//							cout<<"\tRatio (W/H): "<<WHRatio<<endl;
//							cout<<"\tRatio (H/W): "<<HWRatio<<endl;
//							cout<<"\tArea: "<<box.height*box.width<<endl;

							cout<<"\tHorizontalGrade: "<<HorizontalGrade<<endl;
//							cout<<"\tratioGrade: "<<WHRatioGrade<<endl;
//							cout<<"\tdistGrade: "<<distGrade<<endl;
//							cout<<"\tbearGrade: "<<bearingGrade<<endl;
//							cout<<"\tTotalScore: "<<score<<endl;



						}


		}
		else
		{
			cout<<"No Winner"<<endl;

			targets.isTargetDetected = false;
			targets.isTargetScoreable = false;


		}

		if(params.Visualize)
		{
			Rect box = minRect[i].boundingRect();
			Point center(box.x + box.width / 2, box.y + box.height / 2);
			ostringstream output;
					output << "Dist: " << targets.targetDistance << " in" ;
					putText(original, output.str(), Point(center.x + 10, center.y), FONT_HERSHEY_PLAIN, 1, WHITE, 1, 1);

					output.str("");
					output.clear();
					output << "Bearing: " << targets.TargetBearing;
					putText(original, output.str(), Point(center.x + 10, center.y + 15), FONT_HERSHEY_PLAIN, 1, WHITE, 1, 1);

					output.str("");
					output.clear();
					output << "Width: " << targets.Target;
					putText(original, output.str(), Point(center.x + 10, center.y + 30), FONT_HERSHEY_PLAIN, 1, WHITE, 1, 1);

					output.str("");
					output.clear();
					output << "Angle: " << minRect[i].angle;
					putText(original, output.str(), Point(center.x + 10, center.y + 45), FONT_HERSHEY_PLAIN, 1, WHITE, 1, 1);

					output.str("");
					output.clear();
					output << "Score: " << targets.targetScore;
					putText(original, output.str(), Point(center.x + 10, center.y + 60), FONT_HERSHEY_PLAIN, 1, WHITE, 1, 1);
					// do the win

					output.str("");
					output.clear();
					output << "Angle (Center): " << targets.targetBearingCenter;
					putText(original, output.str(), Point(center.x + 10, center.y + 85), FONT_HERSHEY_PLAIN, 1, WHITE, 1, 1);
		}





//		//set targets
//		//check if contour is vert, we use HWRatio because it is greater that 0 for vert target
//			if ((HWRatio > MinVRatio) && (HWRatio < MaxVRatio))
//			{
//				targets.Target = box;
////				targets.VertGoal = true;
////				targets.VerticalTarget = box;
////				targets.VerticalAngle = minRect[i].angle;
////				targets.VerticalCenter = Point(box.x + box.width / 2,
////						box.y + box.height / 2);
////				targets.Vertical_H_W_Ratio = HWRatio;
////				targets.Vertical_W_H_Ratio = WHRatio;
//
//			}
//			//check if contour is horiz, we use WHRatio because it is greater that 0 for vert target
//			else if ((WHRatio > MinHRatio) && (WHRatio < MaxHRatio))
//			{
//				targets.Target = box;
////				targets.HorizGoal = true;
////				targets.HorizontalTarget = box;
////				targets.HorizontalAngle = minRect[i].angle;
////				targets.HorizontalCenter = Point(box.x + box.width / 2,
////						box.y + box.height / 2);
////				targets.Horizontal_H_W_Ratio = HWRatio;
////				targets.Horizontal_W_H_Ratio = WHRatio;
//			}

		//TODO: Add is Target Scorable, angle offset is correct, pick largest box


	}
	else
	{
		targets.isTargetScoreable = false;
		targets.numberOfContoursDetected = 0;
	}

	if(params.Log)
	{
		strcpy(fileSave,"/home/ubuntu/VisionLogs/");
		strcat(fileSave,tbuffer);
		strcat(fileSave,"/Processed/");
		strcat(fileSave,buffer);
		strcat(fileSave,".png");
		imwrite(fileSave,original);
		cout<<"writing_to_file: "<<fileSave<<endl<<endl;
	}

	//If there is contours, this will stream the contours over the original image, if there is no contours
	//this will stream the camera feed. There will always be a stream
	if(params.Visualize)
	{
		//lock mutex to store frame to a global variable, signal that new frame is ready
		//and wake up any sleeping threads
		pthread_mutex_lock(&mjpegServerFrameMutex);
		original.copyTo(imgToStream);
		pthread_mutex_unlock(&mjpegServerFrameMutex);
		pthread_cond_broadcast(&newFrameToStreamSignal);
		//imshow("Contours", original); //Make a rectangle that encompasses the target
	}


	  if(params.Threshold)
		  imshow("Output", original);


//	pthread_mutex_lock(&matchStartMutex);
//	if (!targets.matchStart)
//		{
//		//TODO Write function to determine if target is scorable
//		//targets.isTargetScoreable = true;
//	}
//	pthread_mutex_unlock(&matchStartMutex);

}

/**
 * This function performs numerous filtering on
 * a color image in order to only return
 * areas of interest based on their color
 *
 */
Mat ThresholdImage(Mat original)
{
	//Local Temp Image
	Mat thresholded;

//	//Threshold image to remove non-green objects
//	inRange(original, Scalar(minB, minG, minR), Scalar(maxB, maxG, maxR),
//			thresholded);


	Mat hsv;
	cvtColor(original,hsv,COLOR_BGR2HSV);
	inRange(hsv, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV),
				thresholded);



	//smooth edges
	//blur(thresholded, thresholded, Size(3, 3));

	//Additional filtering if needed
	//Canny(thresholded, thresholded, 100, 100, 3);
	//blur(thresholded, thresholded, Size(5, 5));

	//return image
	return thresholded;

}

/**
 * This functions "zeros", the targets identified
 * so that a clean slate can be used to determine
 *] if the next image contains targets as well.
 */
void NullTargets(Target& target)
{

//	target.HorizontalAngle = 0.0;
//	target.VerticalAngle = 0.0;
//	target.Horizontal_W_H_Ratio = 0.0;
//	target.Horizontal_H_W_Ratio = 0.0;
//	target.Vertical_W_H_Ratio = 0.0;
//	target.Vertical_H_W_Ratio = 0.0;
//	target.targetLeftOrRight = 0;
//	target.lastTargerLorR = 0;
//
//	target.HorizGoal = false;
//	target.VertGoal = false;
//	target.HotGoal = false;

	target.targetDistance = 0.0;
	target.TargetBoxAngle = 0.0;
	target.numberOfContoursDetected = 0;
	target.TargetBearing = 0.0;
	target.targetScore = 0.0;

	target.isTargetScoreable = false;

}
void initializeParams(ProgParams& params)
{
	params.Debug = false;
	params.From_Camera = false;
	params.From_File = false;
	params.Timer = false;
	params.Visualize = false;
	params.Process = true;
	params.USB_Cam = false;
	params.FPS = false;
	params.Log = false;
	params.USE_DROID = false;
	params.USE_ZED = false;
	params.Threshold = false;
	params.Gear = true;
	params.Boiler = false;

}


/**
 * This function parses the command line inputs and determines
 * the runtime parameters the program should use as specified
 * by the user.
 */
void parseCommandInputs(int argc, const char* argv[], ProgParams& params)
{
	//todo: define all input flags
	if (argc < 2)
	{ // Check the value of argc. If not enough parameters have been passed, inform user and exit.
		printCommandLineUsage();
		exit(0);
	}
	else
	{ // if we got enough parameters...

		initializeParams(params);

		for (int i = 1; i < argc; i++)
		{ /* We will iterate over argv[] to get the parameters stored inside.
		 * Note that we're starting on 1 because we don't need to know the
		 * path of the program, which is stored in argv[0] */

			if ((string(argv[i]) == "-f") && (i + 1 < argc)) //read from file
			{
				// We know the next argument *should* be the filename:
				params.IMAGE_FILE = string(argv[i + 1]);
				params.From_Camera = false;
				params.USE_DROID = false;
				params.USE_ZED = false;
				params.From_File = true;
				i++;
			}
			else if ((string(argv[i]) == "-gear")) {
				params.Gear = true;
			}
			else if ((string(argv[i]) == "-boiler")) {
				params.Boiler = true;
			}
			else if ((string(argv[i]) == "-mp") && (i + 1 < argc))
			{
				MJPEG_SERVER_PORT = atoi(argv[i + 1]);
				i++;
			}
			else if ((string(argv[i]) == "-hsv") && (i + 6 < argc)) {
				minH = atoi(argv[i + 1]);
				maxH = atoi(argv[i + 2]);

				minS = atoi(argv[i + 3]);
				maxS = atoi(argv[i + 4]);

				minV = atoi(argv[i + 5]);
				maxV = atoi(argv[i + 6]);

				i += 6;
			}
			else if ((string(argv[i]) == "-ZEDParam") && (i + 6 < argc)) {
				zedBrightness = atoi(argv[i + 1]);
				zedContrast = atoi(argv[i + 2]);

				zedExposure = atoi(argv[i + 3]);
				zedHue = atoi(argv[i + 4]);

				zedSat = atoi(argv[i + 5]);
				zedGain = atoi(argv[i + 6]);

				i += 6;
			}
			else if ((string(argv[i]) == "-c") && (i + 1 < argc)) //camera IP
			{
				params.CAMERA_IP = string(argv[i + 1]);
				params.From_Camera = true;
				params.From_File = false;
				params.USE_DROID = false;
				params.USE_ZED = false;
				params.USB_Cam = false;
				i++;
			}
			else if ((string(argv[i]) == "-droid") && (i + 1 < argc)) //camera IP
			{
				params.CAMERA_IP = string(argv[i + 1]);
				params.From_Camera = true;
				params.From_File = false;
				params.USE_DROID = true;
				params.USE_ZED = false;
				params.USB_Cam = false;
				i++;
			}

			else if (string(argv[i]) == "-ZED") //camera IP
			{
				params.From_Camera = true;
				params.From_File = false;
				params.USE_DROID = false;
				params.USE_ZED = true;
				params.USB_Cam = false;
			}

			else if (string(argv[i]) == "-u") //use USB Camera
			{
				//params.CAMERA_IP = string(argv[i + 1]);
				params.From_Camera = true;
				params.From_File = false;
				params.USB_Cam = true;
				params.USE_DROID = false;
				params.USE_ZED = false;
			}
			else if ((string(argv[i]) == "-s") && (i + 1 < argc)) //robot TCP SERVER IP
			{
				params.ROBOT_IP = string(argv[i + 1]);
				i++;
			}
			else if ((string(argv[i]) == "-p") && (i + 1 < argc)) //robot TCP SERVER PORT
			{
				params.ROBOT_PORT = string(argv[i + 1]);
				i++;
			}
			else if ((string(argv[i]) == "-r") && (i + 1 < argc)) //ROTATE
			{
				params.ROTATE = atoi(argv[i + 1]);
				i++;
			}
			else if (string(argv[i]) == "-t") //Enable Timing
			{
				params.Timer = true;
			}
			else if (string(argv[i]) == "-np") //no processing
			{
				params.Process = false;
			}
			else if (string(argv[i]) == "-log") //no processing
			{
				params.Log = true;
			}
			else if (string(argv[i]) == "-v") //Enable Visual output
			{
				params.Visualize = true;
			}
			else if (string(argv[i]) == "-debug") //Enable debug output
			{
				params.Debug = true;
			}
			else if (string(argv[i]) == "-FPS") //Enable FPS output
			{
				params.FPS = true;
			}
			else if (string(argv[i]) == "-threshold") //Enable FPS output
			{
				params.Threshold = true;
			}
			else if (string(argv[i]) == "-d") //Default Params
			{
				params.ROBOT_PORT = string(argv[i + 1]);
				return;
			}
			else if (string(argv[i]) == "-help") //help
			{
				//todo: cout help on commands
				printCommandLineUsage();
				exit(0);
			}
			else
			{
				std::cout
						<< "Not enough or invalid arguments, please try again.\n";
				printCommandLineUsage();
				exit(0);
			}

		}

	}
}
/**
 * This function either gets an image from the camera
 * loads from a file
 *
 * The condition is determined by variables within the
 * program struct.
 *
 * The image returned is then used for processing.
 *
 * THIS FUNCTION IS OBSOLTETE AND HAS BEEN REPLACED
 * BY AN FFMPEG STREAM FUNCTION
 */
Mat GetOriginalImage(const ProgParams& params)
{
	Mat img;

	if (params.From_Camera)
	{

		system("wget -q http://10.21.68.90/jpg/image.jpg -O capturedImage.jpg");

		//load downloaded image
		img = imread("capturedImage.jpg");

	}
	else if (params.From_File)
	{
		//load image from file
		img = imread(params.IMAGE_FILE);
	}

	return img;
}

void error(const char *msg)
{
	perror(msg);
	exit(0);
}

double diffClock(timespec start, timespec end)
{
 return	(end.tv_sec - start.tv_sec) + (double) (end.tv_nsec - start.tv_nsec)/ 1000000000.0f;
}

/**
 * This function creates a TCP stream between the cRIO and the
 * beaglebone.
 *
 * Once the stream is established it will automatically
 * create two new threads, one to send a predetermined message
 * to the cRIO, and another to receive a predetermined message
 * from the cRIO.
 */

void *TCP_thread(void *args)
{
	ProgParams *struct_ptr = (ProgParams *) args;

	string ip = struct_ptr->ROBOT_IP;
	int port = atoi(struct_ptr->ROBOT_PORT.c_str());

	//string ip = "10.21.68.2";
	//int port = 1111;

	std::cout<<"Trying to connect to Robot Server... at: "<<ip<<":"<<port<<std::endl;

	//connect to host
	client.conn(ip, port);

	//create thread to send messages
	pthread_create(&TCPsend, NULL, TCP_Send_Thread, NULL);

	//create thread to recv messages
	pthread_create(&TCPrecv, NULL, TCP_Recv_Thread, NULL);

	/* the function must return something - NULL will do */
	return NULL;

}

/**
 * This function sends data to the cRIO over TCP.
 *
 * This function assumes the TCP stream has already been created.
 *
 * Currently the only data we receive from the CRIO is match start
 * boolean which allows us to detrmine the time autonomous starts.
 *
 * This function should be ran it its own thread. It uses a sleep
 * function to pause execution.
 */

void *TCP_Send_Thread(void *args)
{
	int count = 0;

	ProgParams *struct_ptr = (ProgParams *) args;

	while (true)
	{
		//Create a string which has following information
		//MatchStart, HotGoal, Distance, message #

		pthread_mutex_lock(&targetMutex);
		stringstream message;

		message << targets.matchStart << "," <<
				targets.TargetBearing << "," <<
				targets.targetDistance << "," <<
				targets.numberOfContoursDetected << "," <<
				targets.isTargetScoreable << "," <<
				targets.isProcessThreadRunning << "," <<
				targets.cameraConnected << "," <<
				targets.isMjpegClientConnected << "," <<
				targets.isTargetDetected << "," <<
				minH << "," <<
				maxH << "," <<
				minS << "," <<
				maxS << "," <<
				minV << "," <<
				maxV << "," <<
				 endl;

		//send message over pipe
		bool successfull = client.send_data(message.str());
		pthread_mutex_unlock(&targetMutex);
		pthread_cond_broadcast(&targetUpdateCompleteSignal);


		if (!successfull)
			break;

		count++;
		usleep(50000); //  run ~20 times a second

	}

	pthread_cancel(TCPrecv);

	//start TCP Server
	pthread_create(&TCPthread, NULL, TCP_thread, struct_ptr);

	return NULL;

}

/**
 * This function captures data from the cRIO over TCP and saves it in a
 * variable.
 *
 * NOTE: THIS FUNCTION BLOCKS WAITING FOR DATA ON THE PIPE TO BE
 * RECEIVED. IF YOU PASS IT A MUTABLE LOCK, IT WILL BLOCK ON
 * THAT LOCK UNTIL A /n CHARACTER IS RECEIVED. POSSIBLY BLOCKING
 * ANY OTHER THREAD USING THAT LOCK.
 *
 * YOU CAN AVOID THIS BY MAKING SURE THE CRIO PASSES DATA TO THE BONE
 * FASTER OR AS FAST AS THIS FUNCTION LOOPS.
 *
 * ALSO BE CAREFUL WHAT MUTABLE LOCKS ARE USED.
 *
 * This function assumes the TCP stream has already been created.
 *
 * Currently the only data we receive from the CRIO is match start
 * boolean which allows us to determine the time autonomous starts.
 *
 * This function should be ran it its own thread. It uses a sleep
 * function to pause execution.
 */

void *TCP_Recv_Thread(void *args)
{
	int count1 = 0;
	int count2 = 0;
	struct timespec end;

	ProgParams *struct_ptr = (ProgParams *) args;

	while (true)
	{


		//Set Match State, should be single int
		pthread_mutex_lock(&matchStartMutex);

		//split string (expect 16 data fields)
		unsigned int numOfData = 8;
		string dataReceived[numOfData];
		unsigned int i = 0;
		stringstream ssin(client.receive(20).c_str());

		while (ssin.good() && i < numOfData){
		    ssin >> dataReceived[i];
		        ++i;
		    }

		    for(i = 0; i < numOfData; i++){
		        cout << dataReceived[i] << endl;
		    }


		targets.matchStart = atoi(dataReceived[0].c_str());

		if(!struct_ptr->Threshold)
			{
			minH = atoi(dataReceived[2].c_str());
			maxH = atoi(dataReceived[3].c_str());
			minS = atoi(dataReceived[4].c_str());
			maxS = atoi(dataReceived[5].c_str());
			minV = atoi(dataReceived[6].c_str());
			maxV = atoi(dataReceived[7].c_str());
		}
		if(!targets.matchStart)
		{
			count1=0;
			count2=0;
			targets.isTargetDetected = false;

		}

		//once the match starts, we start a timer and run it in
		//a new thread, we use a count variable so we only run this once
//		if(targets.matchStart && count1==0)
//		{
//			clock_gettime(CLOCK_REALTIME, &autoStart);
//			count1++;
//			pthread_create(&AutoCounter, NULL, HotGoalCounter, args);
//		}


		clock_gettime(CLOCK_REALTIME, &end);

		//Only set validFrame after we wait a certain amount of time, and after
		//process thread starts
		if(targets.matchStart && diffClock(autoStart,end)>=AUTO_STEADY_STATE && progRun && count2==0 )
		{
			//targets. = true;
			count2++;
		}


		pthread_mutex_unlock(&matchStartMutex);

		usleep(20000); // run 5 times a second

	}

	return NULL;

}

/**
 * This function uses FFMPEG codec apart of openCV to open a
 * MJPEG stream and buffer it. This function should be ran
 * in its own thread so it can run as fast as possibe and store frames.
 *
 * A mutable lock should be used in another thread to copy the latest frame
 *
 * Note: Opening the stream blocks execution. Also
 * Based on my own tests it appears the beaglebone can capture
 * frames at 30fps with 320 x 240 resolution, however
 * the framerate needs to be reduced to allow for processing time.
 *
 * Only run the camera as 10FPS, with a 10kbs limit per frame
 */
void *VideoCap(void *args)
{
	//copy passed in variable to programStruct
	ProgParams *struct_ptr = (ProgParams *) args;

	if (struct_ptr->From_File)
	{
		cout<<"Loading Image from file"<<endl;

		//read img and store it in global variable
		pthread_mutex_lock(&frameMutex);
		isFrameCopyComplete=false;
		frame = imread(struct_ptr->IMAGE_FILE);
		isFrameCopyComplete=true;
		pthread_mutex_unlock(&frameMutex);
		pthread_cond_signal(&FrameCopyCompleteSignal);

		cout<<"File Loaded: Starting Processing Thread"<<endl;
		progRun = true;

	}

	else if(struct_ptr->From_Camera)
	{
		//create timer variables
		struct timespec start, end, bufferStart, bufferEnd;

		//seconds to wait for buffer to clear before we start main process thread
		int waitForBufferToClear = 5;

		//start timer to time how long it takes to open stream
		clock_gettime(CLOCK_REALTIME, &start);




		// For IP cam this works on a AXIS M1013
		// For USB cam this works on Microsoft HD 3000


		std::string videoStreamAddress;
		if (struct_ptr->USB_Cam)
		{

			int videoStreamAddress = 0; //represents /dev/video0

			std::cout<<"Trying to connect to Camera stream... at: "<<videoStreamAddress<<std::endl;

			int count =1;

			//open the video stream and make sure it's opened
			//We specify desired frame size and fps in constructor
			//Camera must be able to support specified framesize and frames per second
			//or this will set camera to defaults
			//while (!vcap.open(videoStreamAddress, 320,240,20))
			while (!vcap.open(videoStreamAddress))
			{
				std::cout << "Error connecting to camera stream, retrying " << count<< std::endl;
				count++;
				usleep(1000000);
			}

			//After Opening Camera we need to configure the returned image setting
			//all opencv v4l2 camera controls scale from 0.0 - 1.0


//Remove 2 lines If Tegra
			//vcap.set(CV_CAP_PROP_EXPOSURE, 0);
			//vcap.set(CV_CAP_PROP_GAIN, 0.5);

			//vcap.set(CV_CAP_PROP_BRIGHTNESS, 0);
			//vcap.set(CV_CAP_PROP_CONTRAST, 0.5);

			cout<<vcap.get(CV_CAP_PROP_FRAME_WIDTH)<<endl;
			cout<<vcap.get(CV_CAP_PROP_FRAME_HEIGHT)<<endl;
			cout<<vcap.get(CV_CAP_PROP_EXPOSURE)<<endl;

		}
		else if(struct_ptr->USE_ZED)
		{

			//Initialize Zed Camera

			cam.ZED_init_VGA();
			cam.setZEDBrightness(zedBrightness);
			cam.setZEDContrast(zedContrast);
			cam.setZEDExposure(zedExposure);
			cam.setZEDHue(zedHue);
			cam.setZEDSaturation(zedSat);
			cam.setZEDGain(zedGain);

			FOV_WIDTH_PIX = cam.getCameraWidth();
			FOV_HEIGHT_PIX = cam.getCameraHeight();

			cout << "It took " << diffClock(start,end) << " seconds to set up stream " << endl;

			clock_gettime(CLOCK_REALTIME, &bufferStart);

			//First iteration of Loop will clear stream buffer for preset amount
			//of time by running through images.
			cout<<"Waiting for stream buffer to clear..."<<endl;

		       while (true){
		           //Pull new Image from ZED
		    	   cam.getNewFrame();

					//start timer to get time per frame
					clock_gettime(CLOCK_REALTIME, &start);

					//read frame and store it in global variable
					pthread_mutex_lock(&frameMutex);
					isFrameCopyComplete=false;
					cam.getCurrentImage(frame);
					resize(frame,frame,Size(320,240));	// Resize the frame of the zed camera to 320, 240
					isFrameCopyComplete=true;
					pthread_mutex_unlock(&frameMutex);
					pthread_cond_signal(&FrameCopyCompleteSignal);

					//end timer to get time per frame
					clock_gettime(CLOCK_REALTIME, &end);

					if(struct_ptr->Timer)
						cout << "It took ZED " << diffClock(start,end) << " seconds to grab stream \n";

					 usleep(1/80*100000); //80Hz loop



						//end timer to get time since stream started
						clock_gettime(CLOCK_REALTIME, &bufferEnd);
						double bufferDifference = diffClock(bufferStart, bufferEnd);

						//The stream takes a while to start up, and because of it, images from the camera
						//buffer. We don't have a way to jump to the end of the stream to get the latest image, so we
						//run this loop as fast as we can and throw away all the old images. This wait, waits some number of seconds
						//before we are at the end of the stream, and can allow processing to begin.
						if ((bufferDifference >= waitForBufferToClear) && !progRun)
						{
							cout<<"Buffer Cleared: Starting Processing Thread"<<endl;
							progRun = true;

						}


		       }


		}
		else //connect to IP Cam
		{
			std::string videoStreamAddress;
			if(struct_ptr->USE_DROID)
				videoStreamAddress = "http://" + struct_ptr->CAMERA_IP +"/mjpegfeed/?dummy=param.mjpg";
			else
				videoStreamAddress = "http://" + struct_ptr->CAMERA_IP +"/mjpg/video.mjpg";

			std::cout<<"Trying to connect to Camera stream... at: "<<videoStreamAddress<<std::endl;

			int count = 1;

			//open the video stream and make sure it's opened
			//image settings, resolution and fps are set via axis camera webpage
			while (!vcap.open(videoStreamAddress))
			{

				std::cout << "Error connecting to camera stream, retrying " << count<< std::endl;
				count++;
				usleep(1000000);
			}

		}



		//Stream started
		cout << "Successfully connected to Camera Stream" << std::endl;

		//set true boolean
		pthread_mutex_lock(&targetMutex);
		targets.cameraConnected = true;
		pthread_mutex_unlock(&targetMutex);

		//end clock to determine time to setup stream
		clock_gettime(CLOCK_REALTIME, &end);

		cout << "It took " << diffClock(start,end) << " seconds to set up stream " << endl;

		clock_gettime(CLOCK_REALTIME, &bufferStart);


		cout<<"Waiting for stream buffer to clear..."<<endl;


		//run in continuous loop
		while (true)
		{

			//start timer to get time per frame
			clock_gettime(CLOCK_REALTIME, &start);

			//read frame and store it in global variable
			pthread_mutex_lock(&frameMutex);
			isFrameCopyComplete=false;
			vcap.read(frame);
			resize(frame,frame,Size(320,240));
			isFrameCopyComplete=true;
			pthread_mutex_unlock(&frameMutex);
			pthread_cond_signal(&FrameCopyCompleteSignal);

			//end timer to get time per frame
			clock_gettime(CLOCK_REALTIME, &end);


			if(struct_ptr->Timer)
				cout << "It took FFMPEG " << diffClock(start,end) << " seconds to grab stream \n";


			//end timer to get time since stream started
			clock_gettime(CLOCK_REALTIME, &bufferEnd);
			double bufferDifference = diffClock(bufferStart, bufferEnd);

			//The stream takes a while to start up, and because of it, images from the camera
			//buffer. We don't have a way to jump to the end of the stream to get the latest image, so we
			//run this loop as fast as we can and throw away all the old images. This wait, waits some number of seconds
			//before we are at the end of the stream, and can allow processing to begin.
			if ((bufferDifference >= waitForBufferToClear) && !progRun)
			{
				cout<<"Buffer Cleared: Starting Processing Thread"<<endl;
				progRun = true;

			}

			usleep(1000); //sleep for 5ms
		}

	}

	return NULL;
}


void printCommandLineUsage()
{
	cout<<"Usage: 2168_Vision  [Input]  [Options] \n\n";

	cout<<setw(10)<<left<<"Inputs:  Choose Only 1 (-f or -c or -u)"<<endl;

	cout<<setw(10)<<left<<"";
	cout<<setw(20)<<left<<"-f <file location>";
	cout<<"Process image at <file location>"<<endl;
	cout<<setw(30)<<""<<"ex: -f /home/image.jpg"<<endl;

	cout<<setw(10)<<left<<"";
	cout<<setw(20)<<left<<"-c <ip address>";
	cout<<"Use IP camera at <ip address>"<<endl;
	cout<<setw(30)<<""<<"ex: -c 10.21.68.2"<<endl;

	cout<<setw(10)<<left<<"";
	cout<<setw(20)<<left<<"-u";
	cout<<"Use USB cam at /dev/video0"<<endl;

	cout<<endl<<endl;
	cout<<setw(10)<<left<<"Options:  Choose Any Combination"<<endl;


	cout<<setw(10)<<left<<"";
	cout<<setw(20)<<left<<"-t";
	cout<<"Enable Timing Print Outs"<<endl;

	cout<<setw(10)<<left<<"";
	cout<<setw(20)<<left<<"-v";
	cout<<"Enable Visual Output"<<endl;
	cout<<setw(30)<<""<<"Uses X11 forwarding to show processed image"<<endl;

	cout<<setw(10)<<left<<"";
	cout<<setw(20)<<left<<"-np";
	cout<<"No Processing: Disable Processing Thread"<<endl;

	cout<<setw(10)<<left<<"";
	cout<<setw(20)<<left<<"-debug";
	cout<<"Enable Debug Print Outs"<<endl;

	cout<<setw(10)<<left<<"";
	cout<<setw(20)<<left<<"-help";
	cout<<"Prints this menu"<<endl;


}

void *MJPEG_Server_Thread(void *args)
{
	targets.isMjpegClientConnected = false;

	if (mjpeg_s.initMJPEGServer(MJPEG_SERVER_PORT))
	{
		cout << "Initalized MJPEG Server" << endl;
		cout << "MJPEG Server Port: " << MJPEG_SERVER_PORT << endl;
	}

	//listen for incoming connection, blocks until a client connections
	mjpeg_s.host(args);
	targets.isMjpegClientConnected = true;

	//once connected, we start streaming data
	pthread_create(&MJPEGHost, NULL, MJPEG_host, args);
	pthread_detach(MJPEGHost);
	return NULL;

}

void *MJPEG_host(void *args)
{


	while (true)
	{
		//wait for first valid frame before trying to serve mjpeg image
		//if we don't wait mjpeg server will try to serve invalid data
		//and/or throw null pointer.
		pthread_cond_wait(&newFrameToStreamSignal, &mjpegServerFrameMutex);
		if (!mjpeg_s.setImageToHost(imgToStream))
		{
			//send fail so give up mutex lock and restart mjpeg server
			pthread_mutex_unlock(&mjpegServerFrameMutex);
			mjpeg_s.host(NULL);
		}
		pthread_mutex_unlock(&mjpegServerFrameMutex);

		usleep(1/60*100000); //run 60 times a second

	}


	return NULL;

}

//Contours have been filtered down to largest two before calling this function

double BestTarget()
{

	return 0;
}

//Rotate image based on camera installation angle
void rot90(cv::Mat &matImage, int rotflag)
{
  //1=CW, 2=CCW, 3=180
  if (rotflag == 1){
    transpose(matImage, matImage);
    flip(matImage, matImage,1); //transpose+flip(1)=CW
  } else if (rotflag == 2) {
    transpose(matImage, matImage);
    flip(matImage, matImage,0); //transpose+flip(0)=CCW
  } else if (rotflag ==3){
    flip(matImage, matImage,-1);    //flip(-1)=180
  } else if (rotflag != 0){ //if not 0,1,2,3:
    cout  << "Unknown rotation flag(" << rotflag << ")" << endl;
  }
}

void onMouse( int event, int x, int y, int, void* img)
{
    if( event != CV_EVENT_LBUTTONDOWN )
            return;
    Mat hsv;
    Mat* rgb = (Mat*) img;
    cvtColor(*rgb,hsv,COLOR_BGR2HSV);

//            printf("%d %d: %d, %d, %d\n",
//            x, y,
//            (int)hsv.at<Vec3b>(y, x)[0],
//            (int)hsv.at<Vec3b>(y, x)[1],
//            (int)hsv.at<Vec3b>(y, x)[2]);

            mouse_h = (int)hsv.at<Vec3b>(y, x)[0];
			mouse_s = (int)hsv.at<Vec3b>(y, x)[1];
			mouse_v = (int)hsv.at<Vec3b>(y, x)[2];

			cout<< "x: "<<x << ", y: "<<y<<endl;


			if(cam.isZEDInit())
			{

				depthAtPoint = cam.getDepthAtPoint(x, y); //(int)hsv.at<Vec3b>(y, x)[3];//

				cam.setZEDBrightness(zedBrightness);
				cam.setZEDContrast(zedContrast);
				cam.setZEDExposure(zedExposure);
				cam.setZEDHue(zedHue);
				cam.setZEDSaturation(zedSat);
				cam.setZEDGain(zedGain);

			}

			else{

				vcap.set(CV_CAP_PROP_EXPOSURE, zedExposure/100.0);
				vcap.set(CV_CAP_PROP_BRIGHTNESS, zedBrightness/100.0);
				vcap.set(CV_CAP_PROP_CONTRAST, zedContrast/100.0);
				vcap.set(CV_CAP_PROP_SATURATION, zedSat/100.0);
				vcap.set(CV_CAP_PROP_HUE, zedHue/100.0);
				vcap.set(CV_CAP_PROP_GAIN, zedGain/100.0);
			}


}



double interpolate(double x, vector<pair<double, double> > table) {
    // Assumes that "table" is sorted by .first
    // Check if x is out of bound
    if (x > table.back().first) return -INF;
    if (x < table[0].first) return -INF;
    vector<pair<double, double> >::iterator it, it2;
    // INFINITY is defined in math.h in the glibc implementation
    it = lower_bound(table.begin(), table.end(), make_pair(x, -INF));
    // Corner case
    if (it == table.begin()) return it->second;
    it2 = it;
    --it2;
    return it2->second + (it->second - it2->second)*(x - it2->first)/(it->first - it2->first);
}





