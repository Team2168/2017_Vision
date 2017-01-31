// dat vizion doh

#define _USE_MATH_DEFINES
#define VIEW_ANGLE 34.8665269
#define AUTO_STEADY_STATE 1.9

#define TARGET_WIDTH_IN 20.125
#define TARGET_HEIGHT_IN 14
#define FOV_WIDTH_PIX 640
#define FOV_HEIGHT_PIX 480
#define CAMERA_WIDTH_FOV_ANGLE_RAD 0.743879868
#define CAMERA_HEIGHT_FOV_ANGLE_RAD 0.991839824
#define ROBOT_ANGLE_OFFSET 0.0


#include "mjpeg_server.h"
#include <unistd.h>
#include "tcp_client.h"
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <pthread.h>
#include <sys/signal.h> //to ignore sigpipe
#include <sys/time.h>



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

//GLOBAL MUTEX LOCK VARIABLES
pthread_mutex_t targetMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t matchStartMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t frameMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mjpegServerFrameMutex = PTHREAD_MUTEX_INITIALIZER;

//GLOBAL MUTEX SIGNAL VARIABLES
pthread_cond_t newFrameToStreamSignal = PTHREAD_COND_INITIALIZER;
pthread_cond_t FrameCopyCompleteSignal = PTHREAD_COND_INITIALIZER;
pthread_cond_t targetUpdateCompleteSignal = PTHREAD_COND_INITIALIZER;

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


//Control process thread exectution
bool progRun;
bool readyToStream;

char tbuffer[30];

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
cout<<"process thread: "<<params.Process<<" Progrun: "<<progRun<<endl;

		if (params.Process && progRun)
		{

			std::cout<<"Process Thread Started"<<endl;

			//Lock Targets and determine goals
			pthread_mutex_lock(&targetMutex);
			std::cout<<"Process Thread Started after lock1"<<endl;
			//pthread_cond_wait(&targetUpdateCompleteSignal,&targetMutex);
			targets.isProcessThreadRunning = true;
			std::cout<<"Process Thread Started after lock2"<<endl;
			pthread_mutex_unlock(&targetMutex);
			std::cout<<"Process Thread Started after lock3"<<endl;
			pthread_cond_broadcast(&targetUpdateCompleteSignal);
			std::cout<<"Process Thread Started after lock4"<<endl;

			//start clock to determine our processing time;
			clock_gettime(CLOCK_REALTIME, &start);

			pthread_mutex_lock(&frameMutex);
			std::cout<<"Process Thread Started after lock 5"<<endl;
			//pthread_cond_wait(&FrameCopyCompleteSignal, &frameMutex);
			std::cout<<"Process Thread Started after lock6"<<endl;

			if (!frame.empty())
			{
				std::cout<<"Process Thread Started after lock7"<<endl;
				frame.copyTo(img);
				pthread_mutex_unlock(&frameMutex);

				thresholded = ThresholdImage(img);

				//Lock Targets and determine goals
				pthread_mutex_lock(&targetMutex);
				//pthread_cond_wait(&targetUpdateCompleteSignal,&targetMutex);
				findTarget(img, thresholded, targets, params);

				if(params.Debug)
				{
					cout<<"Verified Target:" << endl;
					cout<<"\tDist:" <<targets.targetDistance<<endl;
					cout<<"\tRotation: " <<targets.TargetBearing<<endl;
				}
				pthread_mutex_unlock(&targetMutex);
				pthread_cond_broadcast(&targetUpdateCompleteSignal);


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
	//if (contours.size() > 2)
	//{
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


	//}


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

	//run through 2 large contours to grade them to get BEST target
	if (!contours.empty() && !hierarchy.empty())
	{

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

			//calculate distance and target to each target
			double dist = CalculateDist(box.width);
			double bearing = CalculateBearing(box.x,box.width);

			//ID the center in yellow
			Point center(box.x + box.width / 2, box.y + box.height / 2);
			line(original, center, center, YELLOW, 3);


			const int WEIGHT = 25;
			double angleGrade, WHRatioGrade, distGrade, bearingGrade;

			//0 if angle > 60,
			//abs(angle) > 60 = 0
			if ((abs(minRect[i].angle) >= 0 && abs(minRect[i].angle) < 22) || (abs(minRect[i].angle) > 77 && abs(minRect[i].angle) <= 90))
				angleGrade = 25;
			else
				angleGrade = 0;
				//angleGrade = WEIGHT - (WEIGHT* abs(minRect[i].angle) / 60);


			if (WHRatio > 2 || HWRatio > 2)
				WHRatioGrade = 0;
			else
				WHRatioGrade = WEIGHT - (WEIGHT*abs(WHRatio-1.4)/1.4);

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

			double score = angleGrade+WHRatioGrade+distGrade+bearingGrade;

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

			}

		}

		int i =0;
		if(contour0Score>contour1Score && contour0Score >= 56)
			{i = 0;
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
						double dist = CalculateDist(box.width);
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
		else if (contour1Score>contour0Score && contour1Score >= 56)
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
							double dist = CalculateDist(box.width);
							double bearing = CalculateBearing(box.x,box.width);
							double bearing_center = CalculateHorizontalAngle(box.x, box.width);


							targets.Target = box;
							targets.TargetBearing = bearing;
							targets.targetBearingCenter = bearing_center;
							targets.targetDistance = dist;
							targets.targetScore = contour1Score;

							targets.isTargetDetected = true;

							if (abs(bearing) <= 1.2)
								targets.isTargetScoreable = true;
							else
								targets.isTargetScoreable = false;
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
					output << "Dist: " << targets.targetDistance ;
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
		pthread_cond_signal(&newFrameToStreamSignal);
		//imshow("Contours", original); //Make a rectangle that encompasses the target
	}

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
	blur(thresholded, thresholded, Size(3, 3));

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
				params.From_File = true;
				i++;
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
			else if ((string(argv[i]) == "-c") && (i + 1 < argc)) //camera IP
			{
				params.CAMERA_IP = string(argv[i + 1]);
				params.From_Camera = true;
				params.From_File = false;
				params.USB_Cam = false;
				i++;
			}
			else if (string(argv[i]) == "-u") //use USB Camera
			{
				//params.CAMERA_IP = string(argv[i + 1]);
				params.From_Camera = true;
				params.From_File = false;
				params.USB_Cam = true;
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
		//pthread_cond_wait(&targetUpdateCompleteSignal,&targetMutex);
		stringstream message;

		//create string stream message;
		//message << "1," << targets.TargetBearing << "," <<
		//		targets.targetDistance << endl;

		//[isMatchStarted,numberOfContours,distanceToContour,angleToContour,isTargetScoreable,isProcessThreadRunning,isCameraConnected,isMjpegClientConnected]
		message << targets.matchStart << "," <<
				targets.TargetBearing << "," <<
				targets.targetDistance << "," <<
				targets.numberOfContoursDetected << "," <<
				targets.isTargetScoreable << "," <<
				targets.isProcessThreadRunning << "," <<
				targets.cameraConnected << "," <<
				targets.isMjpegClientConnected << "," <<
				targets.isTargetDetected << endl;

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

	while (true)
	{
		//Set Match State, should be single int
		pthread_mutex_lock(&matchStartMutex);
		targets.matchStart = atoi(client.receive(5).c_str());

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
		frame = imread(struct_ptr->IMAGE_FILE);
		pthread_mutex_unlock(&frameMutex);
		pthread_cond_signal(&FrameCopyCompleteSignal);

		cout<<"File Loaded: Starting Processing Thread"<<endl;
		progRun = true;

		imshow("frame", frame); //Make a rectangle that encompasses the target


	}

	else if(struct_ptr->From_Camera)
	{
		//create timer variables
		struct timespec start, end, bufferStart, bufferEnd;

		//seconds to wait for buffer to clear before we start main process thread
		int waitForBufferToClear = 5;

		//start timer to time how long it takes to open stream
		clock_gettime(CLOCK_REALTIME, &start);

		cv::VideoCapture vcap;


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
//			vcap.set(CV_CAP_PROP_EXPOSURE_AUTO, 1);
//			vcap.set(CV_CAP_PROP_EXPOSURE_ABSOLUTE, 0.1);


//			vcap.set(CV_CAP_PROP_BRIGHTNESS, 0);
//			vcap.set(CV_CAP_PROP_CONTRAST, 0.5);

			cout<<vcap.get(CV_CAP_PROP_FRAME_WIDTH)<<endl;
			cout<<vcap.get(CV_CAP_PROP_FRAME_HEIGHT)<<endl;

		}
		else //connect to IP Cam
		{
			std::string videoStreamAddress = "http://" + struct_ptr->CAMERA_IP +"/mjpg/video.mjpg";

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
		pthread_cond_broadcast(&targetUpdateCompleteSignal);

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
			vcap.read(frame);
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

//void *HotGoalCounter(void *args)
//{
//
//	//If this method started, then the match started
//
//
//	while (true)
//	{
//		clock_gettime(CLOCK_REALTIME, &autoEnd);
//		double timeNow = diffClock(autoStart,autoEnd);
//		if(timeNow<5)
//		{
//			pthread_mutex_lock(&targetMutex);
//			if(targets.targetLeftOrRight == 0)
//				targets.hotLeftOrRight = targets.lastTargerLorR * -1;
//			else
//				targets.hotLeftOrRight = targets.targetLeftOrRight;
//			pthread_mutex_unlock(&targetMutex);
//
//			cout<<"this side hot"<<endl;
//		}
//		else if(timeNow<10)
//		{
//			//Auto has been running for 5 seconds, so the other side is hot
//			//we update the variable to switch to other side
//			pthread_mutex_lock(&targetMutex);
//			targets.hotLeftOrRight = targets.hotLeftOrRight * -1;
//			pthread_mutex_unlock(&targetMutex);
//
//			cout<<"otherside hot"<<endl;
//		}
//		else if (timeNow >= 10)
//		{
//			//Auto is over, no more hot targets, end thread
//			pthread_mutex_lock(&targetMutex);
//			targets.hotLeftOrRight = 0;
//			pthread_mutex_unlock(&targetMutex);
//			cout<<"auto over"<<endl;
//			break;
//		}
//
//		usleep(50000); // run 10 times a second
//
//	}
//
//	return NULL;
//
//}

void printCommandLineUsage()
{
	cout<<"Usage: 2168_Vision  [Input]  [Options] \n\n";

	cout<<setw(10)<<left<<"Inputs:  Choose Only 1"<<endl;

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
		//locks on each mpeg frame, and waits until a new frame is needed
		//this loop doesn't need a sleep, because the signal handler will
		//put it to sleep and only wake it up when a new frame is ready
		//this method will only serve a new image, instead of serving a single
		//image multiple times.
		pthread_mutex_lock(&mjpegServerFrameMutex);
		pthread_cond_wait(&newFrameToStreamSignal, &mjpegServerFrameMutex);
		if (!mjpeg_s.setImageToHost(imgToStream))
		{
			//send fail so give up mutex lock and restart mjpeg server
			pthread_mutex_unlock(&mjpegServerFrameMutex);
			mjpeg_s.host(NULL);
		}
		pthread_mutex_unlock(&mjpegServerFrameMutex);

	}



	return NULL;

}

//Contours have been filtered down to largest two before calling this function

double BestTarget()
{

	return 0;
}


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

