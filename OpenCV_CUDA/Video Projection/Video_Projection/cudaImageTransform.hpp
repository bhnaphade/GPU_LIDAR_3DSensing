#ifndef _CUDAIMAGETRANSFORM_HPP_
#define _CUDAIMAGETRANSFORM_HPP_

#include <opencv2/core/cuda_types.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/cuda.hpp>

using namespace std;
using namespace cv;
using namespace cv::cuda;

typedef struct 
{
    int x,y;
}twoDCoordinates;

void 
cudaImageProjectioncaller(const cv::Mat& src, 
                          cv::Mat& dest,
                          const cv::Mat& transMat,
                          const int xOffset,
                          const int yOffset,
                          cv::Point3d screenCoordinates,
                          const int projPlane,
                          const int projPlaneDist);
                          
void freeDeviceAllocatedMemory(int sig);
                              
#endif //_CUDAIMAGETRANSFORM_HPP_
