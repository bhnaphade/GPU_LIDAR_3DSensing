#include <iostream> 
#include <math.h>
#include <string.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define cubeSize 200

using std::cout;
using std::endl;
using cv::Mat;

using namespace cv;
using namespace std;

int scrSize = cubeSize + 30;
int xOffset = 320, yOffset = 320;

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
Mat scr3 = (cv::Mat_<float>(4, 1) << scrSize, scrSize - 30, scrSize - 30, 1);
Mat scr4 = (cv::Mat_<float>(4, 1) << scrSize, scrSize - 30, 0, 1);

Mat camPosition = (cv::Mat_<float>(3, 1) << 300, 300, 250);

Mat org = (cv::Mat_<float>(4, 1) << 0, 0, 0, 1);

float rho = sqrt((camPosition.at<float>(0, 0)*camPosition.at<float>(0, 0)) + 
		            (camPosition.at<float>(1, 0)*camPosition.at<float>(1, 0)) +
		            (camPosition.at<float>(2, 0)*camPosition.at<float>(2, 0)));

float thetha = (acos(camPosition.at<float>(0, 0) / 
		                sqrt((camPosition.at<float>(0, 0) * camPosition.at<float>(0, 0)) + 
            			     (camPosition.at<float>(1, 0) * camPosition.at<float>(1, 0)))));

float phi = (acos(camPosition.at<float>(2, 0) / rho));

float proj_plane_dist = 200;

// Transformation matrix
Mat transMat = (cv::Mat_<float>(4, 4) << -sin(thetha), cos(thetha), 0.0, 0.0, 
					   -cos(phi) * cos(thetha), -cos(phi) * sin(thetha), sin(phi), 0.0, 
					   -sin(phi) * cos(thetha), -sin(phi) * cos(thetha), -cos(phi), rho, 
					   0.0, 0.0, 0.0, 1.0);

cv::Point2d transform(Mat);
void drawAxes(Mat);
void drawCube(Mat);
void drawScreen(Mat);
void imageProject(Mat, Mat);

int main()
{
    Mat preFinalImage = Mat::zeros(700, 700, CV_8UC3);
    Mat finalImage = Mat::zeros(700, 700, CV_8UC3);

    drawAxes(preFinalImage);
    drawCube(preFinalImage);
    drawScreen(preFinalImage);

    int key = 0;
    Mat frame, resize_frame, pyrDownImage,temp;
        
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
        camera >> frame; //save captured image to frame variable
        //imshow("Camera", frame); //show image on window named Camera
        
        temp = pyrDownImage = frame;
        
        pyrDown(frame, pyrDownImage, Size(frame.cols / 2, frame.rows / 2));
         
        //cout << dst.cols << "," << dst.rows << endl;
         
        //if (key == 'c')
        {
            //imshow("Captured", frame);
            //resize(frame, resize_frame, Size(cubeSize, cubeSize), 0, 0, INTER_CUBIC);
            // imshow("resized frame", dst);
            imageProject(pyrDownImage, preFinalImage);
            cv::flip(preFinalImage, finalImage, 0);
            imshow("3D World Image", finalImage);
            
            //return dst;
        }
    }
    waitKey(0);
    return 0;
}

void drawAxes(Mat image)
{
    //cout << "rho" << rho << "thetha" << thetha << "phi" << phi << endl;
    //Transform 3D to 2D point 
    cv::Point2d org_2d = transform(org);
    //cout << "x = " << org_2d.x << " y = " << org_2d.y << endl;
    cv::Point2d x_ax_2d = transform(x_ax);
    //cout << "x = " << x_ax_2d.x << " y = " << x_ax_2d.y << endl;
    cv::Point2d y_ax_2d = transform(y_ax);
    //cout << "x = " << y_ax_2d.x << " y = " << y_ax_2d.y << endl;
    cv::Point2d z_ax_2d = transform(z_ax);
    //cout << "x = " << z_ax_2d.x << " y = " << z_ax_2d.y << endl;

    line(image, Point(org_2d.x, org_2d.y), Point(x_ax_2d.x, x_ax_2d.y), Scalar(110, 220, 0), 2, 8);
    line(image, Point(org_2d.x, org_2d.y), Point(y_ax_2d.x, y_ax_2d.y), Scalar(110, 220, 0), 2, 8);
    line(image, Point(org_2d.x, org_2d.y), Point(z_ax_2d.x, z_ax_2d.y), Scalar(110, 220, 0), 2, 8);
}

void drawCube(Mat image)
{
    cv::Point2d cb1_2d = transform(cb_1);
    cv::Point2d cb2_2d = transform(cb_2);
    cv::Point2d cb3_2d = transform(cb_3);
    cv::Point2d cb4_2d = transform(cb_4);
    cv::Point2d cb5_2d = transform(cb_5);
    cv::Point2d cb6_2d = transform(cb_6);
    cv::Point2d cb7_2d = transform(cb_7);
     
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
    cv::Point2d sc1_2d = transform(scr1);
    cv::Point2d sc2_2d = transform(scr2);
    cv::Point2d sc3_2d = transform(scr3);
    cv::Point2d sc4_2d = transform(scr4);

    line(image, Point(sc1_2d.x, sc1_2d.y), Point(sc2_2d.x, sc2_2d.y), Scalar(110, 220, 0), 2, 8);
    line(image, Point(sc2_2d.x, sc2_2d.y), Point(sc3_2d.x, sc3_2d.y), Scalar(110, 220, 0), 2, 8);
    line(image, Point(sc3_2d.x, sc3_2d.y), Point(sc4_2d.x, sc4_2d.y), Scalar(110, 220, 0), 2, 8);
    line(image, Point(sc4_2d.x, sc4_2d.y), Point(sc1_2d.x, sc1_2d.y), Scalar(110, 220, 0), 2, 8);
}

cv::Point2d transform(Mat threeDMat)
{
    Mat cam_view_3d = transMat * threeDMat;

    cv::Point2d twod_pt(0, 0);

    twod_pt.x = proj_plane_dist * cam_view_3d.at<float>(0, 0) / cam_view_3d.at<float>(2, 0);
    twod_pt.x += xOffset;

    twod_pt.y = proj_plane_dist * cam_view_3d.at<float>(1, 0) / cam_view_3d.at<float>(2, 0);
    twod_pt.y += yOffset;

    return twod_pt;
}

void imageProject(Mat resize_image, Mat orig_image)
{
    
    Size s_r_m = resize_image.size();             //resize_image is a 3 channel image

    int y_min = 0, z_min = 0, y_max = scrSize - 30, z_max = scrSize - 30;

    Mat three_d_pt = (cv::Mat_<float>(4, 1) << scrSize, 0, scrSize - 30, 1);

    for (unsigned int i = 0; i <z_max; i++)
    {
        three_d_pt.at<float>(1, 0) = 0;

        for (unsigned int j = 0; j < y_max; j++)
        {
            Mat temp = (cv::Mat_<float>(4, 1) << 0, 0, 0, 1);
            temp = three_d_pt;
            //cout << "i=" << temp.at<float>(1,0)<< "j=" << temp.at<float>(2, 0) << endl;
            temp.at<float>(1, 0) += 1;

            cv::Point two_d_pt = transform(temp);
             
            //transmformed_coordinates_file << two_d_pt.x <<endl;
             
            //transmformed_coordinates_file << two_d_pt.y <<endl;
	     	
            Vec3b colour = resize_image.at<Vec3b>(Point(j, i));

            orig_image.at<Vec3b>(Point(two_d_pt.x, two_d_pt.y)) = colour;
            //orig_image.at<Vec3b>(Point(two_d_pt.x, two_d_pt.y)) = colour;
            //orig_image.at<Vec3b>(Point(two_d_pt.x, two_d_pt.y)) = colour;
        }
        three_d_pt.at<float>(2, 0) = three_d_pt.at<float>(2, 0) - 1;
    }
}
