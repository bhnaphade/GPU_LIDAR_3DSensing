/********************************************************************
* Program Name : LIDAR_Projection.cpp                               *
* Description  : Project RPLIDAR distance data in Cube			    *
* Author       : Amit Pachore                                       *
* Date         : Apr 23, 2017                                       *
* Notes        : 													*
*********************************************************************/

#include <iostream> 
#include <math.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <unistd.h>

#define k 							10
#define radians						M_PI/180

#define cubeSize    				2000
#define maxDist 			    	(cubeSize/2)
#define minDist                     10
#define DEBUG_CODE  				0

#define numOfDistSensorLayers		10
#define numOfAnglesForDistInfo		360
#define measuredDist                (cubeSize/2)

using namespace cv;
using namespace std;

int color_patch_pts[1] = { 4 };

int scrSize = cubeSize + 30;
int lidProjPlaneSize = cubeSize;
int xOffset = 0, yOffset = 0;

//World coordinate axes
Mat x_ax = (cv::Mat_<float>(4, 1) << 400, 0, 0, 1);
Mat y_ax = (cv::Mat_<float>(4, 1) << 0, 400, 0, 1);
Mat z_ax = (cv::Mat_<float>(4, 1) << 0, 0, 400, 1);

//Cube points
Mat cb_1 = (cv::Mat_<float>(4, 1) << cubeSize, 0, cubeSize, 1);
Mat cb_2 = (cv::Mat_<float>(4, 1) << 0, 0, cubeSize, 1);
Mat cb_3 = (cv::Mat_<float>(4, 1) << 0, cubeSize, cubeSize, 1);
Mat cb_4 = (cv::Mat_<float>(4, 1) << cubeSize, cubeSize, cubeSize, 1);
Mat cb_5 = (cv::Mat_<float>(4, 1) << cubeSize, 0, 0, 1);
Mat cb_6 = (cv::Mat_<float>(4, 1) << cubeSize, cubeSize, 0, 1);
Mat cb_7 = (cv::Mat_<float>(4, 1) << 0, cubeSize, 0, 1);

//Screen points
Mat scr1 = (cv::Mat_<float>(4, 1) << scrSize, 0, 0, 1);
Mat scr2 = (cv::Mat_<float>(4, 1) << scrSize, 0, scrSize - 30, 1);
Mat scr3 = (cv::Mat_<float>(4, 1) << scrSize, scrSize, scrSize - 30, 1);
Mat scr4 = (cv::Mat_<float>(4, 1) << scrSize, scrSize, 0, 1);

Mat camPosition = (cv::Mat_<float>(3, 1) << cubeSize+150, cubeSize+75, cubeSize+75);

Mat org = (cv::Mat_<float>(4, 1) << 0, 0, 0, 1);

float rho = sqrt((camPosition.at<float>(0, 0)*camPosition.at<float>(0, 0)) + 
		         (camPosition.at<float>(1, 0)*camPosition.at<float>(1, 0)) +
		         (camPosition.at<float>(2, 0)*camPosition.at<float>(2, 0)));

float thetha = (acos(camPosition.at<float>(0, 0) / 
		             sqrt((camPosition.at<float>(0, 0) * camPosition.at<float>(0, 0)) + 
            			  (camPosition.at<float>(1, 0) * camPosition.at<float>(1, 0)))));

float phi = (acos(camPosition.at<float>(2, 0) / rho));

float projPlaneDist = 200;

// Transformation matrix
Mat transMat = (cv::Mat_<float>(4, 4) << -sin(thetha), cos(thetha), 0.0, 0.0,
                                         -cos(phi) * cos(thetha), -cos(phi) * sin(thetha), sin(phi), 0.0,
                                         (-sin(phi) * cos(thetha)), (-sin(phi) * cos(thetha)), -cos(phi), rho,
                                          0.0, 0.0, 0.0, 1.0);

cv::Point patch_pts[1][4];
									
cv::Point2d transform(Mat, int);
void drawAxes(Mat);
void drawCube(Mat);
void drawScreen(Mat);
void imageProjection(Mat, Mat);
cv::Point2d pointAtAngleDist(cv::Point2d source_pt, float distance, float angle);
void lidGetScanData(int*);
void lidDistOnSurface(Mat, int *);


static inline void delay(unsigned int ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}

int main()
{
    Mat preFinalImage = Mat::zeros(700, 700, CV_8UC3);
    Mat finalImage = Mat::zeros(500, 500, CV_8UC3);
    
    xOffset = preFinalImage.cols/2;
    yOffset = preFinalImage.rows/2;
	
	int lidarScanData[numOfAnglesForDistInfo][numOfDistSensorLayers];
    
    drawAxes(preFinalImage);
    //drawCube(preFinalImage);
    //drawScreen(preFinalImage);
    //imshow("cube",preFinalImage);
    //cv::flip(preFinalImage, finalImage, 0);
	
	//Get Lidar Device Info
	//Check Lidar Device Health
	//Connect to Lidar

    while (waitKey(1)) //Run Infinite
    {
        //Grab Scan data
		lidGetScanData(&lidarScanData[0][0]);
		//project in cube
		lidDistOnSurface(preFinalImage, &lidarScanData[0][0]);
		drawCube(preFinalImage);
		//cv::flip(preFinalImage, finalImage, 0);
		imshow("Sensor Data Projection", preFinalImage);
    }
    waitKey(0);
    return 0;
}

void drawAxes(Mat image)
{
    int imageHeight = image.rows;
    //cout << "rho" << rho << "thetha" << thetha << "phi" << phi << endl;
    //Transform 3D to 2D point 
    cv::Point2d org_2d = transform(org,imageHeight);
    //cout << "origin" <<  org_2d << endl;;
    cv::Point2d x_ax_2d = transform(x_ax,imageHeight);
    //cout << "x axis" << x_ax_2d << endl;
    cv::Point2d y_ax_2d = transform(y_ax,imageHeight);
    //cout << "x = " << y_ax_2d.x << " y = " << y_ax_2d.y << endl;
    cv::Point2d z_ax_2d = transform(z_ax,imageHeight);
    //cout << "x = " << z_ax_2d.x << " y = " << z_ax_2d.y << endl;

    line(image, Point(org_2d.x, org_2d.y), Point(x_ax_2d.x, x_ax_2d.y), Scalar(0, 0, 220), 2, 8);
    line(image, Point(org_2d.x, org_2d.y), Point(y_ax_2d.x, y_ax_2d.y), Scalar(0, 220, 0), 2, 8);
    line(image, Point(org_2d.x, org_2d.y), Point(z_ax_2d.x, z_ax_2d.y), Scalar(110, 0, 0), 2, 8);
}

void drawCube(Mat image)
{
    int imageHeight = image.rows;
    cv::Point2d cb1_2d = transform(cb_1,imageHeight);
    cv::Point2d cb2_2d = transform(cb_2,imageHeight);
    cv::Point2d cb3_2d = transform(cb_3,imageHeight);
    cv::Point2d cb4_2d = transform(cb_4,imageHeight);
    cv::Point2d cb5_2d = transform(cb_5,imageHeight);
    cv::Point2d cb6_2d = transform(cb_6,imageHeight);
    cv::Point2d cb7_2d = transform(cb_7,imageHeight);
     
    line(image, Point(cb2_2d.x, cb2_2d.y), Point(cb1_2d.x, cb1_2d.y), Scalar(110, 220, 0), 2, 8);
    line(image, Point(cb1_2d.x, cb1_2d.y), Point(cb4_2d.x, cb4_2d.y), Scalar(110, 220, 0), 2, 8);
    line(image, Point(cb4_2d.x, cb4_2d.y), Point(cb3_2d.x, cb3_2d.y), Scalar(110, 220, 0), 2, 8);
    line(image, Point(cb3_2d.x, cb3_2d.y), Point(cb2_2d.x, cb2_2d.y), Scalar(110, 220, 0), 2, 8);
    line(image, Point(cb1_2d.x, cb1_2d.y), Point(cb5_2d.x, cb5_2d.y), Scalar(110, 220, 0), 2, 8);
    line(image, Point(cb5_2d.x, cb5_2d.y), Point(cb6_2d.x, cb6_2d.y), Scalar(110, 220, 0), 2, 8);
    line(image, Point(cb6_2d.x, cb6_2d.y), Point(cb4_2d.x, cb4_2d.y), Scalar(110, 220, 0), 2, 8);
    line(image, Point(cb6_2d.x, cb6_2d.y), Point(cb7_2d.x, cb7_2d.y), Scalar(110, 220, 0), 2, 8);
    line(image, Point(cb7_2d.x, cb7_2d.y), Point(cb3_2d.x, cb3_2d.y), Scalar(110, 220, 0), 2, 8);
}

void drawScreen(Mat image)
{
    int imageHeight = image.rows;
    cv::Point2d sc1_2d = transform(scr1,imageHeight);
    cv::Point2d sc2_2d = transform(scr2,imageHeight);
    cv::Point2d sc3_2d = transform(scr3,imageHeight);
    cv::Point2d sc4_2d = transform(scr4,imageHeight);
    
    #if DEBUG_CODE
    cout << "Point 1" << sc1_2d << endl;
    cout << "Point 2" << sc2_2d << endl;
    cout << "Point 3" << sc3_2d << endl;
    cout << "Point 4" << sc4_2d << endl;
    #endif//#if DEBUG_CODE

    line(image, Point(sc1_2d.x, sc1_2d.y), Point(sc2_2d.x, sc2_2d.y), Scalar(110, 220, 0), 2, 8);
    line(image, Point(sc2_2d.x, sc2_2d.y), Point(sc3_2d.x, sc3_2d.y), Scalar(110, 220, 0), 2, 8);
    line(image, Point(sc3_2d.x, sc3_2d.y), Point(sc4_2d.x, sc4_2d.y), Scalar(110, 220, 0), 2, 8);
    line(image, Point(sc4_2d.x, sc4_2d.y), Point(sc1_2d.x, sc1_2d.y), Scalar(110, 220, 0), 2, 8);
}

cv::Point2d transform(Mat threeDMat, int imageHeight)
{
    Mat cam_view_3d = transMat * threeDMat;

    cv::Point2d twod_pt(0, 0);

    twod_pt.x = projPlaneDist * cam_view_3d.at<float>(0, 0) / cam_view_3d.at<float>(2, 0);
    twod_pt.x += xOffset;

    twod_pt.y = projPlaneDist * cam_view_3d.at<float>(1, 0) / cam_view_3d.at<float>(2, 0);
    twod_pt.y += yOffset;
    
    twod_pt.y = imageHeight-twod_pt.y;

    return twod_pt;
}

cv::Point2d pointAtAngleDist(cv::Point2d source_pt, float distance, float angle)
{
	cv::Point2d target_pt(0, 0);

	target_pt.x = source_pt.x +  distance * cos(radians*angle);
	target_pt.y = source_pt.x +  distance * sin(radians*angle);

	return target_pt;
}

void lidGetScanData(int* scanData)
{
	for(unsigned int sensor=0;sensor<numOfDistSensorLayers/2;sensor++)
	{
		for(unsigned int angle=0;angle<numOfAnglesForDistInfo;angle++)
		{
			scanData[sensor*numOfAnglesForDistInfo+angle] = 
			  (maxDist-(10+(((maxDist-minDist)/numOfDistSensorLayers)*sensor)));
		}
	}
	
	for(unsigned int sensor=numOfDistSensorLayers/2;sensor<numOfDistSensorLayers;sensor++)
	{
		for(unsigned int angle=0;angle<numOfAnglesForDistInfo;angle++)
		{
			scanData[sensor*numOfAnglesForDistInfo+angle] = 
			  maxDist - (maxDist-(10+(((maxDist-minDist)/numOfDistSensorLayers)*sensor)));
		}
	}
}

void lidDistOnSurface(Mat image, int* scanData)
{

    Mat d_1 = (cv::Mat_<float>(4, 1) << 0,0,0,1);
	Mat d_2 = (cv::Mat_<float>(4, 1) << 0,0,0,1);
	Mat d_3 = (cv::Mat_<float>(4, 1) << 0,0,0,1);
	Mat d_4 = (cv::Mat_<float>(4, 1) << 0,0,0,1);
	Mat tragetPt3D = (cv::Mat_<int>(4, 1) << 0,0,0,1);
	
	const cv::Point* ptr[1] = { patch_pts[0]};
	int angle = 0;
	int sensor = 0;
	cv::Point prevAngleTransformedCoordinates[numOfDistSensorLayers];
	/*Array to store transformed coordinates of angle zero 
		- required to connect angle 0 and angle 359 points*/
	cv::Point angleZeroTransformedCoordinates[numOfDistSensorLayers];
	cv::Point surfaceCenter(lidProjPlaneSize/2, lidProjPlaneSize/2);
	cv::Point targetPt(0, 0);
	cv::Point lastTransformedCoordinate(0,0);
	
	int distInLayers = cubeSize / (numOfDistSensorLayers-1);
	
	angle = 0;
	/*Transform Angle = 0 data for all sensors*/
	for(sensor=0;sensor<numOfDistSensorLayers;sensor++)
	{
		targetPt = pointAtAngleDist(surfaceCenter,
									scanData[sensor*numOfAnglesForDistInfo+angle],
									angle);
		
		/*d_1 = (cv::Mat_<float>(4, 1) << targetPt.x-1, targetPt.y-1, distInLayers*sensor, 1);
		d_2 = (cv::Mat_<float>(4, 1) << targetPt.x-1, targetPt.y+1, distInLayers*sensor, 1);
		d_3 = (cv::Mat_<float>(4, 1) << targetPt.x+1, targetPt.y-1, distInLayers*sensor, 1);
		d_4 = (cv::Mat_<float>(4, 1) << targetPt.x+1, targetPt.y+1, distInLayers*sensor, 1);

		patch_pts[0][0] = transform(d_1,image.rows);
		patch_pts[0][1] = transform(d_2,image.rows);
		patch_pts[0][2] = transform(d_3,image.rows);
		patch_pts[0][3] = transform(d_4,image.rows);
		cv::fillPoly(image, ptr, color_patch_pts, 1, Scalar(0, 255, 255), 8);*/
		
		tragetPt3D = (cv::Mat_<float>(4, 1) << targetPt.x, targetPt.y, distInLayers*sensor, 1);
		
		targetPt = transform(tragetPt3D,image.rows);
		
		prevAngleTransformedCoordinates[sensor] = targetPt;
		angleZeroTransformedCoordinates[sensor] = targetPt;
		
		if(sensor >0)
		{
			line(image, lastTransformedCoordinate, targetPt, Scalar(0, (255+(numOfDistSensorLayers*10))%255, 255), 1, 8);
		}
		
		lastTransformedCoordinate = targetPt;
	}
	
	/*Transform remaining angles data*/
	for(angle = 1; angle<numOfAnglesForDistInfo; angle++)
	{
		for(sensor=0;sensor<numOfDistSensorLayers;sensor++)
		{
			targetPt = pointAtAngleDist(surfaceCenter,
										 scanData[sensor*numOfAnglesForDistInfo+angle],
										 angle);
			
			/*d_1 = (cv::Mat_<float>(4, 1) << targetPt.x-1, targetPt.y-1, distInLayers*sensor, 1);
			d_2 = (cv::Mat_<float>(4, 1) << targetPt.x-1, targetPt.y+1, distInLayers*sensor, 1);
			d_3 = (cv::Mat_<float>(4, 1) << targetPt.x+1, targetPt.y-1, distInLayers*sensor, 1);
			d_4 = (cv::Mat_<float>(4, 1) << targetPt.x+1, targetPt.y+1, distInLayers*sensor, 1);

			patch_pts[0][0] = transform(d_1,image.rows);
			patch_pts[0][1] = transform(d_2,image.rows);
			patch_pts[0][2] = transform(d_3,image.rows);
			patch_pts[0][3] = transform(d_4,image.rows);
			cv::fillPoly(image, ptr, color_patch_pts, 1, Scalar(0, 255, 255), 8);*/
			
			tragetPt3D = (cv::Mat_<float>(4, 1) << targetPt.x, targetPt.y, distInLayers*sensor, 1);
			
			targetPt = transform(tragetPt3D,image.rows);
			
			line(image, prevAngleTransformedCoordinates[sensor], targetPt, Scalar(0, 255, 255), 1, 8);
			
			if(sensor > 0)
			{
				line(image, lastTransformedCoordinate, targetPt, Scalar(0, (255+(numOfDistSensorLayers*10))%255, 255), 1, 8);
			}
			
			prevAngleTransformedCoordinates[sensor] = targetPt;
			
			lastTransformedCoordinate = targetPt;
		}
		
		lastTransformedCoordinate.x = 0;
		lastTransformedCoordinate.y = 0;
	}
	
	for(sensor=0;sensor<numOfDistSensorLayers;sensor++)
	{
	    line(image, prevAngleTransformedCoordinates[sensor], 
	        angleZeroTransformedCoordinates[sensor], Scalar(0, 255, 255), 1, 8);
	}
}
