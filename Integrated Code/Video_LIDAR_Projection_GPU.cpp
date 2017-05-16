/********************************************************************
* Program Name : Video_LIDAR_Projection_GPU.cpp                     *
* Description  : Project LIDAR and video distance data in Cube      *
* Author       : Amit Pachore                                       *
* Date         : May 01, 2017                                       *
* Notes        : 													*
*********************************************************************/
#include <iostream> 
#include <math.h>
#include <csignal>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/cuda.hpp>
#include "cudaImageTransform.hpp"

#include "lib_ldr_data.h"
  
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
//#include <stdio.h>
#include <stdlib.h>
#include <string.h> 
#include <unistd.h>  /* UNIX Standard Definitions 	   */ 
#include <string.h>
//#include "lib_ldr_data.h"


#define readSize 8


unsigned char buf[readSize];


senData get_lidar_data();

#define GET_DATA(X,Y) (X<<8)|Y

/* My Arduino is on /dev/ttyACM0 */
char *portname;

#ifndef lib_ldr_data_h__
#define lib_ldr_data_h__

typedef struct _senData
 {
 uint16_t S0;
 uint16_t S1;
 uint16_t S2;
 uint16_t S3;
 }senData;
#endif  // lib_ldr_data_h__

#define k 							10
#define radians						M_PI/180

#define cubeSize    				230
#define maxDist 			    	(cubeSize/2)
#define minDist                     10
#define DEBUG_CODE  				0

#define numOfDistSensorLayers		4
#define numOfAnglesForDistInfo		360
#define measuredDist                (cubeSize/2)

#define axesOffset                  100
#define camPositionXOffset          150
#define camPositionYOffset          75
#define camPositionZOffset          75

#define projectionPlaneOffset       50

using std::cout;
using std::endl;
using cv::Mat;

using namespace cv;
using namespace std;

int color_patch_pts[1] = { 4 };

int scrSize = cubeSize + 30;
int lidProjPlaneSize = cubeSize;
int xOffset = 0, yOffset = 0;

//World coordinate axes
Mat x_ax = (cv::Mat_<float>(4, 1) << cubeSize+axesOffset, 0, 0, 1);
Mat y_ax = (cv::Mat_<float>(4, 1) << 0, cubeSize+axesOffset, 0, 1);
Mat z_ax = (cv::Mat_<float>(4, 1) << 0, 0, cubeSize+axesOffset, 1);

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

Mat camPosition = (cv::Mat_<float>(3, 1) << cubeSize+camPositionXOffset, 
                                            cubeSize+camPositionYOffset, 
                                            cubeSize+camPositionZOffset);

Mat org = (cv::Mat_<float>(4, 1) << 0, 0, 0, 1);

float rho = sqrt((camPosition.at<float>(0, 0)*camPosition.at<float>(0, 0)) + 
		         (camPosition.at<float>(1, 0)*camPosition.at<float>(1, 0)) +
		         (camPosition.at<float>(2, 0)*camPosition.at<float>(2, 0)));

float thetha = (acos(camPosition.at<float>(0, 0) / 
		             sqrt((camPosition.at<float>(0, 0) * camPosition.at<float>(0, 0)) + 
            			  (camPosition.at<float>(1, 0) * camPosition.at<float>(1, 0)))));

float phi = (acos(camPosition.at<float>(2, 0) / rho));

float projPlaneDist = cubeSize+projectionPlaneOffset;

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
void cudaImageProjection(Mat,Mat);
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
    std::signal(SIGINT, freeDeviceAllocatedMemory);    
    
    
    Mat preFinalImage = Mat::zeros(700, 700, CV_8UC3);
    Mat finalImage = Mat::zeros(500, 500, CV_8UC3);
    Mat frame;
    Mat pyrDownImage;
    
    xOffset = preFinalImage.cols/2;
    yOffset = preFinalImage.rows/2;
    
   	int lidarScanData[numOfAnglesForDistInfo][numOfDistSensorLayers];
    
    #if DEBUG_CODE
    cout << transform(scr1,700) << endl;
    cout << transform(scr2,700) << endl;
    cout << transform(scr3,700) << endl;
    cout << transform(scr4,700) << endl;
    #endif//#if DEBUG_CODE
    
    drawAxes(preFinalImage);
    //drawCube(preFinalImage);
    //drawScreen(preFinalImage);
    //imshow("cube",preFinalImage);
    //cv::flip(preFinalImage, finalImage, 0);

    int key = 0;
        
    VideoCapture camera(0); //open camera no.0  0=internal 1=external
    
    camera.set(CV_CAP_PROP_FRAME_WIDTH,640);
    camera.set(CV_CAP_PROP_FRAME_HEIGHT,480);

    /*
    cv::VideoWriter writer;
    string filename = "../my_video.avi";
    
    int fcc = CV_FOURCC('M', 'J', 'P', 'G');

    int fps = 30;

    cv::Size framesize(camera.get(CV_CAP_PROP_FRAME_WIDTH), camera.get(CV_CAP_PROP_FRAME_HEIGHT));

    writer = VideoWriter(filename, fcc, fps, framesize);
    if (!writer.isOpened())
    {
         cout << "Error opening file for write" << endl;
         getchar();
         return -1;
    }*/

    while (waitKey(1)) //Run Infinite
    {
        preFinalImage = Mat::zeros(700, 700, CV_8UC3);
        
        lidGetScanData(&lidarScanData[0][0]);
        lidDistOnSurface(preFinalImage, &lidarScanData[0][0]);
        drawCube(preFinalImage);
        drawScreen(preFinalImage);
        
        camera >> frame; //save captured image to frame variable
        //imshow("Camera", frame); //show image on window named Camera
        
        pyrDown(frame, pyrDownImage, Size(frame.cols / 2, frame.rows / 2));

        cudaImageProjection(pyrDownImage, preFinalImage);
        imshow("3D World Image", preFinalImage);

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

void imageProjection(Mat resize_image, Mat orig_image)
{
    int imageHeight = orig_image.rows;
    Size s_r_m = resize_image.size();//resize_image is a 3 channel image

    int y_min = 0, z_min = 0, y_max = scrSize, z_max = scrSize - 30;

    Mat three_d_pt = (cv::Mat_<float>(4, 1) << scrSize, 0, scrSize - 30, 1);

    for (unsigned int i = 0; i <z_max; i++)
    {
        three_d_pt.at<float>(1, 0) = 0;

        for (unsigned int j = 0; j < y_max; j++)
        {
            Mat temp = (cv::Mat_<float>(4, 1) << 0, 0, 0, 1);
            temp = three_d_pt;
            
            temp.at<float>(1, 0) += 1;

            cv::Point two_d_pt = transform(temp,imageHeight);
	     	
            Vec3b colour = resize_image.at<Vec3b>(Point(j, i));

            orig_image.at<Vec3b>(Point(two_d_pt.x, two_d_pt.y)) = colour;
        }
        three_d_pt.at<float>(2, 0) = three_d_pt.at<float>(2, 0) - 1;
    }
}

void cudaImageProjection(Mat src,Mat dest)
{

    cv::Point3d screenCoordinates(scrSize,scrSize,scrSize-30);
    
    cudaImageProjectioncaller(src, dest,transMat, xOffset,yOffset,screenCoordinates,
                              (const int)1/*Projection Plane--define Enum*/,
                              (const int) projPlaneDist);
                              
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
    uint16_t distance=0;
    senData fourSenData = {0,0,0,0};
    fourSenData = get_lidar_data();
    
    for(unsigned int sensor=0;sensor<numOfDistSensorLayers;sensor++)
	{
	    switch(sensor)
	    {
	        case 0:
	            distance = fourSenData.S0;
	            break;
	        case 1: 
	            distance = fourSenData.S1;
	            break;
	        case 2: 
	            distance = fourSenData.S2;
	            break;
	        case 3: 
	            distance = fourSenData.S3;
	            break;
            default:
                cout << "Wrong Sensor Number\n";
                break;	                
	    }
	    distance = distance/5;
		for(unsigned int angle=0;angle<numOfAnglesForDistInfo;angle++)
		{
			scanData[sensor*numOfAnglesForDistInfo+angle] = distance;
		}
	}
    
	/*//Hard Coded data used for testing
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
	}*/
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


senData get_lidar_data()
{
 int fd;
 char *portname = "/dev/ttyTHS1";
/* Open the file descriptor in non-blocking mode */
 fd = open(portname, O_RDWR | O_NOCTTY);
 
/* Set up the control structure */
 struct termios toptions;
 
 /* Get currently set options for the tty */
 tcgetattr(fd, &toptions);
 
/* Set custom options */
 
/* 9600 baud */
 cfsetispeed(&toptions, B9600);
 cfsetospeed(&toptions, B9600);
 /* 8 bits, no parity, no stop bits */
 toptions.c_cflag &= ~PARENB;
 toptions.c_cflag &= ~CSTOPB;
 toptions.c_cflag &= ~CSIZE;
 toptions.c_cflag |= CS8;
 /* no hardware flow control */
 toptions.c_cflag &= ~CRTSCTS;
 /* enable receiver, ignore status lines */
 toptions.c_cflag |= CREAD | CLOCAL;
 /* disable input/output flow control, disable restart chars */
 toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
 /* disable canonical input, disable echo,
 disable visually erase chars,
 disable terminal-generated signals */
 toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
 /* disable output processing */
 toptions.c_oflag &= ~OPOST;
 
/* wait for 12 characters to come in before read returns */
/* WARNING! THIS CAUSES THE read() TO BLOCK UNTIL ALL */
/* CHARACTERS HAVE COME IN! */
 toptions.c_cc[VMIN] = readSize;
 /* no minimum time to wait before read returns */
 toptions.c_cc[VTIME] = 0;
 
/* commit the options */
 tcsetattr(fd, TCSANOW, &toptions);
 
/* Wait for the Arduino to reset */
 //usleep(1000*1000);
 char start ='S';

   tcflush(fd, TCIFLUSH);
  write(fd, &start, 1);
  usleep(1);

senData sD;
// while(1){
 /* Flush anything already in the serial buffer */
    tcflush(fd, TCIFLUSH);
    read(fd, &buf, readSize);

     sD.S0 = GET_DATA(buf[0],buf[1]);
     sD.S1 = GET_DATA(buf[2],buf[3]);
     sD.S2 = GET_DATA(buf[4],buf[5]);
     sD.S3 = GET_DATA(buf[6],buf[7]);
        
    /*     senData1 =(buf[0] << 8);
 senData1 |= buf[1]; */     
   // printf("\nbytesRead=%d SenData%d = %d hex[0]=%2x hex[1]=%2x",n,i,senData1[i],buf[j],buf[j+1]); 
  //  printf("\nData bytes %d S0 %d S1 %d S2 %d S3 %d",n,  sD.S0,sD.S1,sD.S2,sD.S3);
  
/* print how many bytes read */
 //printf("\n%d %d",buf[0]-'0',buf[1]-'0');
//}
close(fd);
return sD;
}
