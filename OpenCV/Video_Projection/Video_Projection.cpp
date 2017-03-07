#include <iostream> 
#include <math.h>
#include <string.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define k 	  10
#define cube_size 200

using std::cout;
using std::endl;
using cv::Mat;

using namespace cv;
using namespace std;

int color_patch_pts[1] = { 4 };
int sc_size = cube_size + 30;
int x_off = 320, y_off = 320;

//World coordinate axes
Mat x_ax = (cv::Mat_<double>(4, 1) << 400, 0, 0, 1);
Mat y_ax = (cv::Mat_<double>(4, 1) << 0, 400, 0, 1);
Mat z_ax = (cv::Mat_<double>(4, 1) << 0, 0, 400, 1);

//Cube points
Mat cb_1 = (cv::Mat_<double>(4, 1) << cube_size, 0, cube_size, 1);
Mat cb_2 = (cv::Mat_<double>(4, 1) << 0, 0, cube_size, 1);
Mat cb_3 = (cv::Mat_<double>(4, 1) << 0, cube_size, cube_size, 1);
Mat cb_4 = (cv::Mat_<double>(4, 1) << cube_size, cube_size, cube_size, 1);
Mat cb_5 = (cv::Mat_<double>(4, 1) << cube_size, 0, 0, 1);
Mat cb_6 = (cv::Mat_<double>(4, 1) << cube_size, cube_size, 0, 1);
Mat cb_7 = (cv::Mat_<double>(4, 1) << 0, cube_size, 0, 1);

//Decoration points
cv::Point patch_pts[1][4];

//Destination points for Affine transform
Point2f dstTri[3];

//Screen points
Mat sc_1 = (cv::Mat_<double>(4, 1) << sc_size, 0, 0, 1);
Mat sc_2 = (cv::Mat_<double>(4, 1) << sc_size, 0, sc_size - 30, 1);
Mat sc_3 = (cv::Mat_<double>(4, 1) << sc_size, sc_size - 30, sc_size - 30, 1);
Mat sc_4 = (cv::Mat_<double>(4, 1) << sc_size, sc_size - 30, 0, 1);

Mat cam_eye = (cv::Mat_<double>(3, 1) << 300, 300, 250);
Mat org = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);

double rho = sqrt((cam_eye.at<double>(0, 0)*cam_eye.at<double>(0, 0)) + 
		          (cam_eye.at<double>(1, 0)*cam_eye.at<double>(1, 0)) +
		          (cam_eye.at<double>(2, 0)*cam_eye.at<double>(2, 0)));

double thetha = (acos(cam_eye.at<double>(0, 0) / 
		              sqrt((cam_eye.at<double>(0, 0) * cam_eye.at<double>(0, 0)) + 
            			   (cam_eye.at<double>(1, 0) * cam_eye.at<double>(1, 0)))));

double phi = (acos(cam_eye.at<double>(2, 0) / rho));

double proj_plane_dist = 200;

// Transformation matrix
Mat trans_mat = (cv::Mat_<double>(4, 4) << -sin(thetha), cos(thetha), 0.0, 0.0, 
					   -cos(phi) * cos(thetha), -cos(phi) * sin(thetha), sin(phi), 0.0, 
					   -sin(phi) * cos(thetha), -sin(phi) * cos(thetha), -cos(phi), rho, 
					   0.0, 0.0, 0.0, 1.0);

cv::Point2d transform(Mat);
void draw_cube(Mat);
void draw_screen(Mat);
void surface_decoration(Mat);
Mat camera();
void image_project(Mat, Mat);

vector<cv::Point> transformed_coordinates;

int main()
{
     Mat image = Mat::zeros(700, 700, CV_8UC3);
     Mat fin_image = Mat::zeros(500, 500, CV_8UC3);

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

     draw_cube(image);
     draw_screen(image);
     //surface_decoration(image);
     //Mat resize_image = camera();
     int key = 0;
     Mat frame, resize_frame, dst, tmp;
     VideoCapture cap(0); //open camera no.0  0=internal 1=external
     cv::VideoWriter writer;
     string filename = "../my_video.avi";

     int fcc = CV_FOURCC('M', 'J', 'P', 'G');

     int fps = 30;

     cv::Size framesize(cap.get(CV_CAP_PROP_FRAME_WIDTH), cap.get(CV_CAP_PROP_FRAME_HEIGHT));

    /* writer = VideoWriter(filename, fcc, fps, framesize);
    if (!writer.isOpened())
    {
         cout << "Error opening file for write" << endl;
         getchar();
         return -1;
    }*/
    
    //  cout << framesize << endl;

    while (waitKey(1)) //Run Infinite
    {
         cap >> frame; //save captured image to frame variable
         //imshow("Camera", frame); //show image on window named Camera
         tmp = frame;
         dst = tmp;
         pyrDown(tmp, dst, Size(tmp.cols / 2, tmp.rows / 2));
         
        //cout << dst.cols << "," << dst.rows << endl;
         
         //if (key == 'c')
        {
             //imshow("Captured", frame);
             //resize(frame, resize_frame, Size(cube_size, cube_size), 0, 0, INTER_CUBIC);
            // imshow("resized frame", dst);
             image_project(dst, image);
             cv::flip(image, fin_image, 0);
             imshow("Image", fin_image);
             //return dst;
        }
    }


     waitKey(0);
     return 0;
}

void draw_cube(Mat image)
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

void draw_screen(Mat image)
{
     cv::Point2d sc1_2d = transform(sc_1);
     cv::Point2d sc2_2d = transform(sc_2);
     cv::Point2d sc3_2d = transform(sc_3);
     cv::Point2d sc4_2d = transform(sc_4);

     dstTri[0].x = sc2_2d.x; dstTri[0].y = sc2_2d.y;
     dstTri[1].x = sc3_2d.x; dstTri[1].y = sc3_2d.y;
     dstTri[2].x = sc1_2d.x; dstTri[2].y = sc1_2d.y;

     patch_pts[0][0].x = sc2_2d.x;
     patch_pts[0][0].y = sc2_2d.y;

     patch_pts[0][1].x = sc1_2d.x;
     patch_pts[0][1].y = sc1_2d.y;

     patch_pts[0][2].x = sc4_2d.x;
     patch_pts[0][2].y = sc4_2d.y;

     patch_pts[0][3].x = sc3_2d.x;
     patch_pts[0][3].y = sc3_2d.y;

     line(image, Point(sc1_2d.x, sc1_2d.y), Point(sc2_2d.x, sc2_2d.y), Scalar(110, 220, 0), 2, 8);
     line(image, Point(sc2_2d.x, sc2_2d.y), Point(sc3_2d.x, sc3_2d.y), Scalar(110, 220, 0), 2, 8);
     line(image, Point(sc3_2d.x, sc3_2d.y), Point(sc4_2d.x, sc4_2d.y), Scalar(110, 220, 0), 2, 8);
     line(image, Point(sc4_2d.x, sc4_2d.y), Point(sc1_2d.x, sc1_2d.y), Scalar(110, 220, 0), 2, 8);
}

cv::Point2d transform(Mat threed_mat)
{
     Mat cam_view_3d = trans_mat * threed_mat;

     cv::Point2d twod_pt(0, 0);
     twod_pt.x = proj_plane_dist * cam_view_3d.at<double>(0, 0) / cam_view_3d.at<double>(2, 0);
     twod_pt.x += x_off;

     twod_pt.y = proj_plane_dist * cam_view_3d.at<double>(1, 0) / cam_view_3d.at<double>(2, 0);
     twod_pt.y += y_off;

     return twod_pt;
}

void image_project(Mat resize_image, Mat orig_image)
{

     //fstream transmformed_coordinates_file;
     //transmformed_coordinates_file.open ("../transmformed_coordinates_file.txt", std::fstream::out | fstream::app);
     
     /*if (!transmformed_coordinates_file.is_open())
     {
        cout << "Unable to open file" << endl;
        exit(0);
     }*/
     
     Size s_r_m = resize_image.size();             //resize_image is a 3 channel image

     int y_min = 0, z_min = 0, y_max = sc_size - 30, z_max = sc_size - 30;
     Mat three_d_pt = (cv::Mat_<double>(4, 1) << sc_size, 0, sc_size - 30, 1);

     for (unsigned int i = 0; i <z_max; i++)
    {
         three_d_pt.at<double>(1, 0) = 0;

         for (unsigned int j = 0; j < y_max; j++)
        {
             Mat temp = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
             temp = three_d_pt;
             //cout << "i=" << temp.at<double>(1,0)<< "j=" << temp.at<double>(2, 0) << endl;
             temp.at<double>(1, 0) += 1;

             cv::Point two_d_pt = transform(temp);
             
             //transmformed_coordinates_file << two_d_pt.x <<endl;
             
             //transmformed_coordinates_file << two_d_pt.y <<endl;
	     	
             Vec3b colour = resize_image.at<Vec3b>(Point(j, i));

             orig_image.at<Vec3b>(Point(two_d_pt.x, two_d_pt.y)) = colour;
             //orig_image.at<Vec3b>(Point(two_d_pt.x, two_d_pt.y)) = colour;
             //orig_image.at<Vec3b>(Point(two_d_pt.x, two_d_pt.y)) = colour;
        }
         three_d_pt.at<double>(2, 0) = three_d_pt.at<double>(2, 0) - 1;
    }
    
    //transmformed_coordinates_file << "********************"<<endl;
     
     //transmformed_coordinates_file.close();
}


/*
void surface_decoration(Mat image)
{
     const cv::Point* ptr[1] = { patch_pts[0] };

     if (k > cube_size)
    {
         cout << "Decoration larger than cube size, please change value of k" << endl;
    }
     else if (k == cube_size)
    {
         cv::fillPoly(image, ptr, color_patch_pts, 1, Scalar(0, 0, 255), 8);
    }
     else
    {
         double offset = (cube_size - k) / 2;

         //Color patch decoration points
         Mat d_1 = (cv::Mat_<double>(4, 1) << sc_size, offset, sc_size - 30 - offset, 1);
         Mat d_2 = (cv::Mat_<double>(4, 1) << sc_size, offset, offset, 1);
         Mat d_3 = (cv::Mat_<double>(4, 1) << sc_size, sc_size - 20 - offset, offset, 1);
         Mat d_4 = (cv::Mat_<double>(4, 1) << sc_size, sc_size - 20 - offset, sc_size - 30 - offset, 1);

         patch_pts[0][0] = transform(d_1);
         patch_pts[0][1] = transform(d_2);
         patch_pts[0][2] = transform(d_3);
         patch_pts[0][3] = transform(d_4);

         cv::fillPoly(image, ptr, color_patch_pts, 1, Scalar(0, 0, 255), 8);
    }
}
*/
/*
Mat camera()
{
     int key = 0;
     Mat frame, resize_frame,dst,tmp;
     VideoCapture cap(0); //open camera no.0  0=internal 1=external
     cv::VideoWriter writer;
     string filename = "/home/ubuntu/Project_OpenCV/my_video.avi";

     int fcc = CV_FOURCC('D', 'I', 'V', '3');

     int fps = 40;

     cv::Size framesize(cap.get(CV_CAP_PROP_FRAME_WIDTH), cap.get(CV_CAP_PROP_FRAME_HEIGHT));

     writer = VideoWriter(filename, fcc, fps, framesize);
    if (!writer.isOpened())
    {
         cout << "Error opening file for write" << endl;
         getchar();

         return -1;
    }


     while ((key = waitKey(30)) != 27) //wait 30 milliseconds and check for esc key
    {
         cap >> frame; //save captured image to frame variable
         imshow("Camera", frame); //show image on window named Camera
         tmp = frame;
         dst = tmp;
         pyrDown(tmp, dst, Size(tmp.cols / 2, tmp.rows / 2));
         if (key == 'c')
        {
             //imshow("Captured", frame);
             //resize(frame, resize_frame, Size(cube_size, cube_size), 0, 0, INTER_CUBIC);
             imshow("resized frame", dst);
             return dst;
        }
    }
}
*/
