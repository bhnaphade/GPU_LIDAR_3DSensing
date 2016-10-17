#include <iostream> 
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <unistd.h>
#include <signal.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define k 		10
#define max_LIDAR_dist 	600 //in mm
#define radians		M_PI/180


static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}

using namespace rp::standalone::rplidar;

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

/*bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}*/


using std::cout;

using std::endl;

using cv::Mat;

using namespace cv;

typedef struct
{
	float angle;
	float dist;
}lidar;


int color_patch_pts[1] = { 4 };

int cube_size = 200;

int sc_size = cube_size + 30;

int halfSc_size = sc_size / 2;

int x_off = 350, y_off = 350;


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


//Screen points

Mat sc_1 = (cv::Mat_<double>(4, 1) << sc_size, 0, 0, 1);

Mat sc_2 = (cv::Mat_<double>(4, 1) << sc_size, 0, sc_size, 1);

Mat sc_3 = (cv::Mat_<double>(4, 1) << sc_size, sc_size, sc_size, 1);

Mat sc_4 = (cv::Mat_<double>(4, 1) << sc_size, sc_size, 0, 1);


Mat cam_eye = (cv::Mat_<double>(3, 1) << 250, 250, 250);

Mat org = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);


double rho = sqrt((cam_eye.at<double>(0, 0)*cam_eye.at<double>(0, 0)) + (cam_eye.at<double>(1, 0)*cam_eye.at<double>(1, 0)) + (cam_eye.at<double>(2, 0)*cam_eye.at<double>(2, 0)));

double thetha = (acos(cam_eye.at<double>(0, 0) / sqrt((cam_eye.at<double>(0, 0) * cam_eye.at<double>(0, 0)) + (cam_eye.at<double>(1, 0) * cam_eye.at<double>(1, 0)))));

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

//void surface_decoration(Mat);

void SENLIDOnSurface(Mat, rplidar_response_measurement_node_t*, int count);

int main(int argc, const char * argv[]) 
{
	const char * opt_com_path = NULL;
    	_u32         opt_com_baudrate = 115200;
    	u_result     op_result;

        rplidar_response_measurement_node_t nodes[360*2];
        size_t   count = _countof(nodes);
	
	Mat image = Mat::zeros(700, 700, CV_8UC3);
	Mat fin_image = Mat::zeros(700, 700, CV_8UC3);

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

	// read serial port from the command line...
    	if (argc>1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

    	// read baud rate from the command line if specified...
    	if (argc>2) opt_com_baudrate = strtoul(argv[2], NULL, 10);


    	if (!opt_com_path) 
	{
	        opt_com_path = "/dev/ttyUSB0";
    	}

    	// create the driver instance
    	RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
    
    	if (!drv) 
	{
	        fprintf(stderr, "RPLIDAR:insufficent memory, exit\n");
	        exit(-2);
    	}


    	// make connection...
    	if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) 
	{
        	fprintf(stderr, "RPLIDAR:Error, cannot bind to the specified serial port %s.\n"
        		, opt_com_path);
        	goto on_finished;
    	}

	rplidar_response_device_info_t devinfo;
    	// retrieving the device info
    	////////////////////////////////////////
    	if (IS_FAIL(drv->getDeviceInfo(devinfo))) 
	{
        	fprintf(stderr, "RPLIDAR:Error, cannot get device info.\n");
        	goto on_finished;
    	}

    	// check health...
    	if (!checkRPLIDARHealth(drv)) 
	{
        	goto on_finished;
    	}

    	drv->startMotor();

    	// start scan...
    	drv->startScan();

    	//signal(SIGINT, ctrlc);

        op_result = drv->grabScanData(nodes, count);

        if (IS_OK(op_result)) 
	{
  	        drv->ascendScanData(nodes, count);
		SENLIDOnSurface(image, &nodes[0],(int)count);
		cv::flip(image, fin_image, 0);
		imshow("Image", fin_image);
	}
	else
	{
		fprintf(stderr, "RPLIDAR:Error, cannot grab scan data.\n");
	}
	
    	drv->stop();
    	drv->stopMotor();

	/*	//surface_decoration(image);
	lidar nodes[360];
	
	for(int i=0;i<360;i++)
	{
		nodes[i].angle = i;
		nodes[i].dist  = 600;
	}
	*/
	waitKey(0);
	
on_finished:
	RPlidarDriver::DisposeDriver(drv);

	return 0;
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

	patch_pts[0][0].x = sc2_2d.x;
	patch_pts[0][0].y = sc2_2d.y;
	patch_pts[0][1].x = sc1_2d.x;
	patch_pts[0][1].y = sc1_2d.y;
	patch_pts[0][2].x = sc4_2d.x;
	patch_pts[0][2].y = sc4_2d.y;
	patch_pts[0][3].x = sc3_2d.x;
	patch_pts[0][3].y = sc3_2d.y;

	line(image, Point(sc1_2d.x, sc1_2d.y), Point(sc2_2d.x, sc2_2d.y), Scalar(51, 51, 255), 2, 8);
	line(image, Point(sc2_2d.x, sc2_2d.y), Point(sc3_2d.x, sc3_2d.y), Scalar(51, 51, 255), 2, 8);
	line(image, Point(sc3_2d.x, sc3_2d.y), Point(sc4_2d.x, sc4_2d.y), Scalar(51, 51, 255), 2, 8);
	line(image, Point(sc4_2d.x, sc4_2d.y), Point(sc1_2d.x, sc1_2d.y), Scalar(51, 51, 255), 2, 8);
}

void surface_decoration(Mat image)
{
	const cv::Point* ptr[1] = { patch_pts[0]};

	if (k > cube_size)
	{
		cout << "Decoration larger than cube size, please change value of k" << endl;
	}
	else if (k==cube_size)
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

cv::Point2d pointAtAngleDist(cv::Point2d source_pt, float distance, float angle)
{
	cv::Point2d target_pt(0, 0);

	target_pt.x = source_pt.x +  distance * cos(radians*angle);
	target_pt.y = source_pt.x +  distance * sin(radians*angle);

	return target_pt;
}

void SENLIDOnSurface(Mat image, rplidar_response_measurement_node_t * LocNodes, int count)
{	
	const cv::Point* ptr[1] = { patch_pts[0]};
	
	Mat loc_fin_image;

	cv::Point2d target_pt(0, 0);
	cv::Point2d surface_center(halfSc_size, halfSc_size);
	for(int i = 0; i<count; i++)
	{
		target_pt = pointAtAngleDist(surface_center, 
		(halfSc_size*((LocNodes[i].distance_q2/4.0f)/10))/max_LIDAR_dist,
		(float)((int)(450-((LocNodes[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f))%360));
	
		Mat d_1 = (cv::Mat_<double>(4, 1) << sc_size, target_pt.x-1, target_pt.y-1, 1);
		Mat d_2 = (cv::Mat_<double>(4, 1) << sc_size, target_pt.x-1, target_pt.y+1, 1);
		Mat d_3 = (cv::Mat_<double>(4, 1) << sc_size, target_pt.x+1, target_pt.y-1, 1);
		Mat d_4 = (cv::Mat_<double>(4, 1) << sc_size, target_pt.x+1, target_pt.y+1, 1);	

		patch_pts[0][0] = transform(d_1);
		patch_pts[0][1] = transform(d_2);
		patch_pts[0][2] = transform(d_3);
		patch_pts[0][3] = transform(d_4);

		cv::fillPoly(image, ptr, color_patch_pts, 1, Scalar(0, 255, 255), 8);
		/*if(i==359)
		{	cv::flip(image, loc_fin_image, 0);
			imshow("Test_Image", loc_fin_image);
			waitKey(0);
		}*/
	}
}
