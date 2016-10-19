{\rtf1\ansi\deff0\nouicompat{\fonttbl{\f0\fnil\fcharset0 Calibri;}}
{\*\generator Riched20 10.0.14393}\viewkind4\uc1 
\pard\sa200\sl276\slmult1\f0\fs22\lang9 #include <iostream> \par
#include <math.h>\par
#include <string.h>\par
#include <opencv2/opencv.hpp>\par
#include <opencv2/core/core.hpp>\par
#include <opencv2/highgui/highgui.hpp>\par
\par
#define k 10\par
#define cube_size 200\par
\par
using std::cout;\par
using std::endl;\par
using cv::Mat;\par
\par
using namespace cv;\par
using namespace std;\par
\par
int color_patch_pts[1] = \{ 4 \};\par
int sc_size = cube_size + 30;\par
int x_off = 320, y_off = 320;\par
\par
//World coordinate axes\par
Mat x_ax = (cv::Mat_<double>(4, 1) << 400, 0, 0, 1);\par
Mat y_ax = (cv::Mat_<double>(4, 1) << 0, 400, 0, 1);\par
Mat z_ax = (cv::Mat_<double>(4, 1) << 0, 0, 400, 1);\par
\par
//Cube points\par
Mat cb_1 = (cv::Mat_<double>(4, 1) << cube_size, 0, cube_size, 1);\par
Mat cb_2 = (cv::Mat_<double>(4, 1) << 0, 0, cube_size, 1);\par
Mat cb_3 = (cv::Mat_<double>(4, 1) << 0, cube_size, cube_size, 1);\par
Mat cb_4 = (cv::Mat_<double>(4, 1) << cube_size, cube_size, cube_size, 1);\par
Mat cb_5 = (cv::Mat_<double>(4, 1) << cube_size, 0, 0, 1);\par
Mat cb_6 = (cv::Mat_<double>(4, 1) << cube_size, cube_size, 0, 1);\par
Mat cb_7 = (cv::Mat_<double>(4, 1) << 0, cube_size, 0, 1);\par
\par
//Decoration points\par
cv::Point patch_pts[1][4];\par
\par
//Destination points for Affine transform\par
Point2f dstTri[3];\par
\par
//Screen points\par
Mat sc_1 = (cv::Mat_<double>(4, 1) << sc_size, 0, 0, 1);\par
Mat sc_2 = (cv::Mat_<double>(4, 1) << sc_size, 0, sc_size - 30, 1);\par
Mat sc_3 = (cv::Mat_<double>(4, 1) << sc_size, sc_size - 30, sc_size - 30, 1);\par
Mat sc_4 = (cv::Mat_<double>(4, 1) << sc_size, sc_size - 30, 0, 1);\par
\par
Mat cam_eye = (cv::Mat_<double>(3, 1) << 300, 300, 250);\par
Mat org = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);\par
\par
double rho = sqrt((cam_eye.at<double>(0, 0)*cam_eye.at<double>(0, 0)) + (cam_eye.at<double>(1, 0)*cam_eye.at<double>(1, 0)) + (cam_eye.at<double>(2, 0)*cam_eye.at<double>(2, 0)));\par
double thetha = (acos(cam_eye.at<double>(0, 0) / sqrt((cam_eye.at<double>(0, 0) * cam_eye.at<double>(0, 0)) + (cam_eye.at<double>(1, 0) * cam_eye.at<double>(1, 0)))));\par
double phi = (acos(cam_eye.at<double>(2, 0) / rho));\par
\par
double proj_plane_dist = 200;\par
\par
// Transformation matrix\par
Mat trans_mat = (cv::Mat_<double>(4, 4) << -sin(thetha), cos(thetha), 0.0, 0.0,\par
\tab -cos(phi) * cos(thetha), -cos(phi) * sin(thetha), sin(phi), 0.0,\par
\tab -sin(phi) * cos(thetha), -sin(phi) * cos(thetha), -cos(phi), rho,\par
\tab 0.0, 0.0, 0.0, 1.0);\par
\par
cv::Point2d transform(Mat);\par
void draw_cube(Mat);\par
void draw_screen(Mat);\par
void surface_decoration(Mat);\par
Mat camera();\par
void image_project(Mat, Mat);\par
\par
int main()\par
\{\par
\tab Mat image = Mat::zeros(700, 700, CV_8UC3);\par
\tab Mat fin_image = Mat::zeros(500, 500, CV_8UC3);\par
\par
\tab //cout << "rho" << rho << "thetha" << thetha << "phi" << phi << endl;\par
\tab //Transform 3D to 2D point \par
\tab cv::Point2d org_2d = transform(org);\par
\tab //cout << "x = " << org_2d.x << " y = " << org_2d.y << endl;\par
\tab cv::Point2d x_ax_2d = transform(x_ax);\par
\tab //cout << "x = " << x_ax_2d.x << " y = " << x_ax_2d.y << endl;\par
\tab cv::Point2d y_ax_2d = transform(y_ax);\par
\tab //cout << "x = " << y_ax_2d.x << " y = " << y_ax_2d.y << endl;\par
\tab cv::Point2d z_ax_2d = transform(z_ax);\par
\tab //cout << "x = " << z_ax_2d.x << " y = " << z_ax_2d.y << endl;\par
\par
\tab line(image, Point(org_2d.x, org_2d.y), Point(x_ax_2d.x, x_ax_2d.y), Scalar(110, 220, 0), 2, 8);\par
\tab line(image, Point(org_2d.x, org_2d.y), Point(y_ax_2d.x, y_ax_2d.y), Scalar(110, 220, 0), 2, 8);\par
\tab line(image, Point(org_2d.x, org_2d.y), Point(z_ax_2d.x, z_ax_2d.y), Scalar(110, 220, 0), 2, 8);\par
\par
\tab draw_cube(image);\par
\tab draw_screen(image);\par
\tab //surface_decoration(image);\par
\tab //Mat resize_image = camera();\par
\tab int key = 0;\par
\tab Mat frame, resize_frame, dst, tmp;\par
\tab VideoCapture cap(0); //open camera no.0  0=internal 1=external\par
\tab cv::VideoWriter writer;\par
\tab string filename = "D:\\my_video.avi";\par
\par
\tab int fcc = CV_FOURCC('D', 'I', 'V', '3');\par
\par
\tab int fps = 80;\par
\par
\tab cv::Size framesize(cap.get(CV_CAP_PROP_FRAME_WIDTH), cap.get(CV_CAP_PROP_FRAME_HEIGHT));\par
\par
\tab writer = VideoWriter(filename, fcc, fps, framesize);\par
\tab if (!writer.isOpened())\par
\tab\{\par
\tab\tab cout << "Error opening file for write" << endl;\par
\tab\tab getchar();\par
\tab\tab return -1;\par
\tab\}\par
\par
\par
\tab while ((key = waitKey(30)) != 27) //wait 30 milliseconds and check for esc key\par
\tab\{\par
\tab\tab cap >> frame; //save captured image to frame variable\par
\tab\tab imshow("Camera", frame); //show image on window named Camera\par
\tab\tab tmp = frame;\par
\tab\tab dst = tmp;\par
\tab\tab pyrDown(tmp, dst, Size(tmp.cols / 2, tmp.rows / 2));\par
\tab\tab //if (key == 'c')\par
\tab\tab\{\par
\tab\tab\tab //imshow("Captured", frame);\par
\tab\tab\tab //resize(frame, resize_frame, Size(cube_size, cube_size), 0, 0, INTER_CUBIC);\par
\tab\tab\tab imshow("resized frame", dst);\par
\tab\tab\tab image_project(dst, image);\par
\tab\tab\tab cv::flip(image, fin_image, 0);\par
\tab\tab\tab imshow("Image", fin_image);\par
\tab\tab\tab //return dst;\par
\tab\tab\}\par
\tab\}\par
\par
\par
\tab waitKey(0);\par
\tab return 0;\par
\}\par
\par
cv::Point2d transform(Mat threed_mat)\par
\{\par
\tab Mat cam_view_3d = trans_mat * threed_mat;\par
\par
\tab cv::Point2d twod_pt(0, 0);\par
\tab twod_pt.x = proj_plane_dist * cam_view_3d.at<double>(0, 0) / cam_view_3d.at<double>(2, 0);\par
\tab twod_pt.x += x_off;\par
\par
\tab twod_pt.y = proj_plane_dist * cam_view_3d.at<double>(1, 0) / cam_view_3d.at<double>(2, 0);\par
\tab twod_pt.y += y_off;\par
\par
\tab return twod_pt;\par
\}\par
\par
void draw_cube(Mat image)\par
\{\par
\tab cv::Point2d cb1_2d = transform(cb_1);\par
\tab cv::Point2d cb2_2d = transform(cb_2);\par
\tab cv::Point2d cb3_2d = transform(cb_3);\par
\tab cv::Point2d cb4_2d = transform(cb_4);\par
\tab cv::Point2d cb5_2d = transform(cb_5);\par
\tab cv::Point2d cb6_2d = transform(cb_6);\par
\tab cv::Point2d cb7_2d = transform(cb_7);\par
\par
\tab line(image, Point(cb2_2d.x, cb2_2d.y), Point(cb1_2d.x, cb1_2d.y), Scalar(110, 220, 0), 2, 8);\par
\tab line(image, Point(cb1_2d.x, cb1_2d.y), Point(cb4_2d.x, cb4_2d.y), Scalar(110, 220, 0), 2, 8);\par
\tab line(image, Point(cb4_2d.x, cb4_2d.y), Point(cb3_2d.x, cb3_2d.y), Scalar(110, 220, 0), 2, 8);\par
\tab line(image, Point(cb3_2d.x, cb3_2d.y), Point(cb2_2d.x, cb2_2d.y), Scalar(110, 220, 0), 2, 8);\par
\tab line(image, Point(cb1_2d.x, cb1_2d.y), Point(cb5_2d.x, cb5_2d.y), Scalar(110, 220, 0), 2, 8);\par
\tab line(image, Point(cb5_2d.x, cb5_2d.y), Point(cb6_2d.x, cb6_2d.y), Scalar(110, 220, 0), 2, 8);\par
\tab line(image, Point(cb6_2d.x, cb6_2d.y), Point(cb4_2d.x, cb4_2d.y), Scalar(110, 220, 0), 2, 8);\par
\tab line(image, Point(cb6_2d.x, cb6_2d.y), Point(cb7_2d.x, cb7_2d.y), Scalar(110, 220, 0), 2, 8);\par
\tab line(image, Point(cb7_2d.x, cb7_2d.y), Point(cb3_2d.x, cb3_2d.y), Scalar(110, 220, 0), 2, 8);\par
\par
\}\par
\par
void draw_screen(Mat image)\par
\{\par
\tab cv::Point2d sc1_2d = transform(sc_1);\par
\tab cv::Point2d sc2_2d = transform(sc_2);\par
\tab cv::Point2d sc3_2d = transform(sc_3);\par
\tab cv::Point2d sc4_2d = transform(sc_4);\par
\par
\tab dstTri[0].x = sc2_2d.x; dstTri[0].y = sc2_2d.y;\par
\tab dstTri[1].x = sc3_2d.x; dstTri[1].y = sc3_2d.y;\par
\tab dstTri[2].x = sc1_2d.x; dstTri[2].y = sc1_2d.y;\par
\par
\tab patch_pts[0][0].x = sc2_2d.x;\par
\tab patch_pts[0][0].y = sc2_2d.y;\par
\par
\tab patch_pts[0][1].x = sc1_2d.x;\par
\tab patch_pts[0][1].y = sc1_2d.y;\par
\par
\tab patch_pts[0][2].x = sc4_2d.x;\par
\tab patch_pts[0][2].y = sc4_2d.y;\par
\par
\tab patch_pts[0][3].x = sc3_2d.x;\par
\tab patch_pts[0][3].y = sc3_2d.y;\par
\par
\tab line(image, Point(sc1_2d.x, sc1_2d.y), Point(sc2_2d.x, sc2_2d.y), Scalar(110, 220, 0), 2, 8);\par
\tab line(image, Point(sc2_2d.x, sc2_2d.y), Point(sc3_2d.x, sc3_2d.y), Scalar(110, 220, 0), 2, 8);\par
\tab line(image, Point(sc3_2d.x, sc3_2d.y), Point(sc4_2d.x, sc4_2d.y), Scalar(110, 220, 0), 2, 8);\par
\tab line(image, Point(sc4_2d.x, sc4_2d.y), Point(sc1_2d.x, sc1_2d.y), Scalar(110, 220, 0), 2, 8);\par
\}\par
\par
void surface_decoration(Mat image)\par
\{\par
\tab const cv::Point* ptr[1] = \{ patch_pts[0] \};\par
\par
\tab if (k > cube_size)\par
\tab\{\par
\tab\tab cout << "Decoration larger than cube size, please change value of k" << endl;\par
\tab\}\par
\tab else if (k == cube_size)\par
\tab\{\par
\tab\tab cv::fillPoly(image, ptr, color_patch_pts, 1, Scalar(0, 0, 255), 8);\par
\tab\}\par
\tab else\par
\tab\{\par
\tab\tab double offset = (cube_size - k) / 2;\par
\par
\tab\tab //Color patch decoration points\par
\tab\tab Mat d_1 = (cv::Mat_<double>(4, 1) << sc_size, offset, sc_size - 30 - offset, 1);\par
\tab\tab Mat d_2 = (cv::Mat_<double>(4, 1) << sc_size, offset, offset, 1);\par
\tab\tab Mat d_3 = (cv::Mat_<double>(4, 1) << sc_size, sc_size - 20 - offset, offset, 1);\par
\tab\tab Mat d_4 = (cv::Mat_<double>(4, 1) << sc_size, sc_size - 20 - offset, sc_size - 30 - offset, 1);\par
\par
\tab\tab patch_pts[0][0] = transform(d_1);\par
\tab\tab patch_pts[0][1] = transform(d_2);\par
\tab\tab patch_pts[0][2] = transform(d_3);\par
\tab\tab patch_pts[0][3] = transform(d_4);\par
\par
\tab\tab cv::fillPoly(image, ptr, color_patch_pts, 1, Scalar(0, 0, 255), 8);\par
\tab\}\par
\}\par
\par
Mat camera()\par
\{\par
\tab int key = 0;\par
\tab Mat frame, resize_frame,dst,tmp;\par
\tab VideoCapture cap(0); //open camera no.0  0=internal 1=external\par
\tab cv::VideoWriter writer;\par
\tab string filename = "D:\\my_video.avi";\par
\par
\tab int fcc = CV_FOURCC('D', 'I', 'V', '3');\par
\par
\tab int fps = 40;\par
\par
\tab cv::Size framesize(cap.get(CV_CAP_PROP_FRAME_WIDTH), cap.get(CV_CAP_PROP_FRAME_HEIGHT));\par
\par
\tab writer = VideoWriter(filename, fcc, fps, framesize);\par
\tab /*if (!writer.isOpened())\par
\tab\{\par
\tab\tab cout << "Error opening file for write" << endl;\par
\tab\tab getchar();\par
\tab\tab return -1;\par
\tab\}*/\par
\par
\par
\tab while ((key = waitKey(30)) != 27) //wait 30 milliseconds and check for esc key\par
\tab\{\par
\tab\tab cap >> frame; //save captured image to frame variable\par
\tab\tab imshow("Camera", frame); //show image on window named Camera\par
\tab\tab tmp = frame;\par
\tab\tab dst = tmp;\par
\tab\tab pyrDown(tmp, dst, Size(tmp.cols / 2, tmp.rows / 2));\par
\tab\tab if (key == 'c')\par
\tab\tab\{\par
\tab\tab\tab //imshow("Captured", frame);\par
\tab\tab\tab //resize(frame, resize_frame, Size(cube_size, cube_size), 0, 0, INTER_CUBIC);\par
\tab\tab\tab imshow("resized frame", dst);\par
\tab\tab\tab return dst;\par
\tab\tab\}\par
\tab\}\par
\}\par
\par
void image_project(Mat resize_image, Mat orig_image)\par
\{\par
\tab Size s_r_m = resize_image.size();\tab\tab\tab //resize_image is a 3 channel image\par
\par
#if 0\par
\tab s_r_m.width = 170;\par
\tab s_r_m.height = 170;\par
\tab Mat temp_fin = Mat::zeros(500, 500, CV_8UC3);\par
\tab Mat temp = Mat::zeros(500, 500, CV_8UC3);\par
\tab Mat warp_mat(2, 3, CV_32FC1);\par
\par
\tab int rows = resize_image.rows;\par
\tab int columns = resize_image.cols;\par
\tab Point2f srcTri[3];\par
\tab srcTri[0].x = 0; srcTri[0].y = 0;\par
\tab srcTri[1].x = cube_size; srcTri[2].y = 0;\par
\tab srcTri[2].x = 0; srcTri[2].y = cube_size;\par
\tab warp_mat = getAffineTransform(srcTri, dstTri);\par
\tab warpAffine(resize_image, temp, warp_mat, temp.size());\par
\tab cv::flip(temp, temp_fin, 0);\par
\tab //Mat left(temp, Rect(dstTri[2].x-50, dstTri[2].y, s_r_m.width, s_r_m.height));\par
\tab //Mat right(orig_image, Rect(dstTri[2].x-20, dstTri[2].y-20, s_r_m.width, s_r_m.height));\par
\tab //left.copyTo(right);\par
\tab //Mat left(orig_image, Rect(101, 101, s_r_m.width, s_r_m.height));\par
\tab //resize_image.copyTo(left);\par
\tab imshow("Test", temp_fin);\par
#endif\par
\par
#if 0\par
\tab Mat image_s = Mat::zeros(500, 500, CV_8UC3);\par
\tab line(image_s, Point(100, 100), Point(100, 230), Scalar(110, 220, 0), 2, 8);\par
\tab line(image_s, Point(100, 230), Point(230, 230), Scalar(110, 220, 0), 2, 8);\par
\tab line(image_s, Point(230, 230), Point(230, 100), Scalar(110, 220, 0), 2, 8);\par
\tab line(image_s, Point(230, 100), Point(100, 100), Scalar(110, 220, 0), 2, 8);\par
\tab line(image_s, Point(70, 70), Point(70, 260), Scalar(110, 220, 0), 2, 8);\par
\tab line(image_s, Point(70, 260), Point(260, 260), Scalar(110, 220, 0), 2, 8);\par
\tab line(image_s, Point(260, 260), Point(260, 70), Scalar(110, 220, 0), 2, 8);\par
\tab line(image_s, Point(260, 70), Point(70, 70), Scalar(110, 220, 0), 2, 8);\par
\tab Mat left(image_s, Rect(101, 101, s_r_m.width, s_r_m.height));\par
\tab resize_image.copyTo(left);\par
\tab imshow("Test", image_s);\par
#endif\par
\par
\tab int y_min = 0, z_min = 0, y_max = sc_size - 30, z_max = sc_size - 30;\par
\tab Mat three_d_pt = (cv::Mat_<double>(4, 1) << sc_size, 0, sc_size - 30, 1);\par
\par
\tab for (unsigned int i = 0; i <z_max; i++)\par
\tab\{\par
\tab\tab three_d_pt.at<double>(1, 0) = 0;\par
\par
\tab\tab for (unsigned int j = 0; j < y_max; j++)\par
\tab\tab\{\par
\tab\tab\tab Mat temp = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);\par
\tab\tab\tab temp = three_d_pt;\par
\tab\tab\tab //cout << "i=" << temp.at<double>(1,0)<< "j=" << temp.at<double>(2, 0) << endl;\par
\tab\tab\tab temp.at<double>(1, 0) += 1;\par
\par
\tab\tab\tab cv::Point two_d_pt = transform(temp);\par
\tab\tab\tab Vec3b colour = resize_image.at<Vec3b>(Point(j, i));\par
\par
\tab\tab\tab orig_image.at<Vec3b>(Point(two_d_pt.x, two_d_pt.y)) = colour;\par
\tab\tab\tab orig_image.at<Vec3b>(Point(two_d_pt.x, two_d_pt.y)) = colour;\par
\tab\tab\tab orig_image.at<Vec3b>(Point(two_d_pt.x, two_d_pt.y)) = colour;\par
\tab\tab\}\par
\tab\tab three_d_pt.at<double>(2, 0) = three_d_pt.at<double>(2, 0) - 1;\par
\tab\}\par
\}\par
}
 