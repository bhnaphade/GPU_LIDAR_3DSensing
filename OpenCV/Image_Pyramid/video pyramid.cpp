#include "opencv2/opencv.hpp"
#include <iostream>
#include <string.h>
using namespace cv;
using namespace std;
int main(int, char**)
{
	VideoCapture cap(0); // open the default camera
	if (!cap.isOpened())  // check if we succeeded
		return -1;
	cv::VideoWriter writer;

	Mat edges;
	namedWindow("edges", 1);

	string filename = "D:\my_video.avi";

	int fcc = CV_FOURCC('D','I','V','3');

	int fps = 40;

	cv::Size framesize(cap.get(CV_CAP_PROP_FRAME_WIDTH), cap.get(CV_CAP_PROP_FRAME_HEIGHT));

	writer = VideoWriter(filename,fcc,fps,framesize);
	if (!writer.isOpened())
	{
		cout << "Error opening file for write" << endl;
		getchar();
		return -1;
	}
	for (;;)
	{
		Mat frame, tmp, dst;
		bool bsucess = cap.read(frame);
		tmp = frame;
		pyrDown(tmp, dst, Size(tmp.cols / 2, tmp.rows / 2));
		writer.write(dst);
		imshow("edges", dst);
		if (waitKey(30) >= 0)
		{
			//break;
			return 0;
		}
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}