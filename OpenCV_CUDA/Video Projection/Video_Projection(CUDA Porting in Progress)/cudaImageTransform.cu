#include "cudaImageTransform.hpp"
#include <opencv2/core/cuda_types.hpp>
#include <stdio.h>


using namespace std;
using namespace cv;
using namespace cv::cuda;

static unsigned char *d_srcImage=NULL;
static unsigned char *d_destImage=NULL;
static float *d_transMat=NULL;

#define transMatCols    4
#define transMatRows    4

__device__ twoDCoordinates
cudaTransform(const float *transMat,int xValue, int yValue,const twoDCoordinates offset,
              const int destImageHeight,const int constScreenCoordinate,
              const int projectionPlane,const int planeDistance)
{

    //printf("xTid:%d yTid:%d TransMatStep:%d\n",xValue,yValue,transMatStep);
    
    //printf("Const Screen Coordinate: %d\n", constScreenCoordinate);
    
    twoDCoordinates twoDPoint;
    float matrixResult[transMatRows];
    
    float threeDPoint[transMatRows];
    
    threeDPoint[3] = 1;
    
    switch(projectionPlane)
    {
        case 1:
            threeDPoint[0] = (float)constScreenCoordinate;
            threeDPoint[1] = (float)xValue;
            threeDPoint[2] = (float)yValue;
            break;
        case 2:
            threeDPoint[0] = (float)xValue;
            threeDPoint[1] = (float)constScreenCoordinate;
            threeDPoint[2] = (float)yValue;
            break;
        case 3:
            threeDPoint[0] = (float)xValue;
            threeDPoint[1] = (float)yValue;
            threeDPoint[2] = (float)constScreenCoordinate;
            break;
        default:
            //Error handling to be performed
            break;
    }
    
    int transMatLoc=0;
    
    float sum = 0;
    
    for(int i=0;i<transMatRows;i++)
    {
        for(int j=0;j<transMatCols;j++)
        {
            transMatLoc = i*transMatCols + j;
            
            //printf("%f ",transMat[transMatLoc]);
            
            sum += (transMat[transMatLoc] * threeDPoint[j]);
        }
        //printf("\n");
        matrixResult[i] = sum;
        sum = 0.0;
    }
    
    twoDPoint.x = (int)((float)planeDistance * matrixResult[0]/matrixResult[2]);

    twoDPoint.x += offset.x;
                                  
    twoDPoint.y = (int)((float)planeDistance * matrixResult[1]/matrixResult[2]);
    twoDPoint.y += offset.y;
    
    //twoDPoint.y = destImageHeight-twoDPoint.y;
            
    return twoDPoint;
}

__global__ void
cudaGetTransformedCoordinates(unsigned char *src, 
                              unsigned char *dest,
                              const float* transMat,  
                              const int srcImageStep,
                              const int destImageStep,                          
                              const twoDCoordinates offset,
                              const int constScreenCoordinate, 
                              const int projectionPlane,
                              const int planeDistance,
                              const int srcImageWidth,
                              const int srcImageHeight,
                              const int destImageHeight,
                              const int srcImageBytes,
                              const int destImageBytes,
                              const twoDCoordinates threadLimits)
{
    //int2 temp;
    int xTid = (blockIdx.x * blockDim.x) + threadIdx.x;
    int yTid = (blockIdx.y * blockDim.y) + threadIdx.y;
    
    //Location of colored pixel in output
    
    long int destColorLoc=0;
    long int srcColorLoc =0;
    
    twoDCoordinates transCoordinates;
    
    if((xTid < threadLimits.x) && (yTid < threadLimits.y))
    {
        srcColorLoc = (yTid * srcImageStep) + (3 * xTid);
        
        transCoordinates = cudaTransform(transMat,xTid,yTid,offset,destImageHeight,
                                         constScreenCoordinate, projectionPlane,
                                         planeDistance);
        
           
        destColorLoc = (transCoordinates.y * destImageStep) + (3*transCoordinates.x);
        
        if((srcColorLoc < (srcImageBytes-2))&&(destColorLoc < (destImageBytes-2)))
        { 
            if((xTid == 0) && (yTid==0))
            {
                printf("xTid:%d yTid:%d Tx:%d Ty:%d destColorLoc:%d srcLoc:%d\n",xTid,yTid,transCoordinates.x,transCoordinates.y,destColorLoc,srcColorLoc);
            }
            
            dest[destColorLoc]=src[srcColorLoc];
            dest[destColorLoc+1]=src[srcColorLoc+1];
            dest[destColorLoc+2]=src[srcColorLoc+2];
        }
    }
}


void 
cudaImageProjectioncaller(const cv::Mat& h_srcImage, 
                          cv::Mat& h_destImage,
                          const cv::Mat& h_transMat,
                          const int xOffset,
                          const int yOffset,
                          cv::Point3d screenCoordinates,
                          const int projPlane,
                          const int projPlaneDist)
{

    // Error code to check return values for CUDA calls
    cudaError_t err = cudaSuccess;
    
    twoDCoordinates offset;
    offset.x = xOffset;
    offset.y = yOffset;
       
    const int srcImageBytes = h_srcImage.step * h_srcImage.rows;
    const int destImageBytes = h_destImage.step * h_destImage.rows;
    const int transMatBytes = h_transMat.step * h_transMat.rows;
    
    cout << "Offset :" << offset.x << " "<<offset.y << endl;
    
    cout << "Plane Distance: " << projPlaneDist << endl;
    
    /*static float* transMatPtr = NULL;
    if (transMatPtr == NULL)
    {
        transMatPtr = (float*)malloc(transMatBytes);
        memcpy(transMatPtr, h_transMat.ptr(),transMatBytes);
    }
    
    for(int i =0; i<h_transMat.rows; i++ )
    {
        for(int j =0; j<h_transMat.cols; j++ )
            cout << transMatPtr[i*h_transMat.cols+j] << " ";
        
        cout << endl;
    }*/
    
        
    if(d_srcImage==NULL)
    {
        err = cudaMalloc(&d_srcImage, srcImageBytes);
        if (err != cudaSuccess)
        {
            fprintf(stderr, "Failed to allocate device memory for Source Image "
                            "(error code %s)!\n", cudaGetErrorString(err));
            exit(EXIT_FAILURE);
        }
    }
    //Copy host data on device
    err = cudaMemcpy(d_srcImage,h_srcImage.ptr(),srcImageBytes,cudaMemcpyHostToDevice);
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy Source Image from host to device "
                        "(error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }
    

    if(d_transMat==NULL)
    {
        err = cudaMalloc(&d_transMat, transMatBytes);
        if (err != cudaSuccess)
        {
            fprintf(stderr, "Failed to allocate device memory for Trans Matrix "
                            "(error code %s)!\n", cudaGetErrorString(err));
            exit(EXIT_FAILURE);
        }
        
        err = cudaMemcpy(d_transMat,h_transMat.ptr(),transMatBytes,cudaMemcpyHostToDevice);
        if (err != cudaSuccess)
        {
            fprintf(stderr, "Failed to copy Trans Matrix from host to device "
                            "(error code %s)!\n", cudaGetErrorString(err));
            exit(EXIT_FAILURE);
        }
    }
    
    if(d_destImage==NULL)
    {
        err = cudaMalloc(&d_destImage, destImageBytes);
        if (err != cudaSuccess)
        {
            fprintf(stderr, "Failed to allocate device memory for Source Image"
                            "(error code %s)!\n", cudaGetErrorString(err));
            exit(EXIT_FAILURE);
        }
    }
    //Copy host data on device
    err = cudaMemcpy(d_destImage,h_destImage.ptr(),destImageBytes,cudaMemcpyHostToDevice);
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy Output Image from host to device "
                        "(error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }
    
    //Kernel call   
    dim3 blocks(10,10);
    dim3 threadsPerBlock(0,0);
    int constScreenCoordinate;
    twoDCoordinates threadLimits;
    
    switch(projPlane)
    {
        case 1:
            constScreenCoordinate = screenCoordinates.x;
            threadsPerBlock.x = screenCoordinates.y/10;
            threadsPerBlock.y = screenCoordinates.z/10;
            threadLimits.x    = screenCoordinates.y;
            threadLimits.y    = screenCoordinates.z;
            break;
        case 2:
            constScreenCoordinate = screenCoordinates.y;
            threadsPerBlock.x = screenCoordinates.x/10;
            threadsPerBlock.y = screenCoordinates.z/10;
            threadLimits.x    = screenCoordinates.x;
            threadLimits.y    = screenCoordinates.z;
            break;
        case 3:
            constScreenCoordinate = screenCoordinates.z;
            threadsPerBlock.x = screenCoordinates.x/10;
            threadsPerBlock.y = screenCoordinates.y/10;
            threadLimits.x    = screenCoordinates.x;
            threadLimits.y    = screenCoordinates.y;            
            break;
        default:
            //Error handling to be performed
            break;
    }
    
    cudaGetTransformedCoordinates<<<blocks,threadsPerBlock>>>(d_srcImage,
                                                              d_destImage,
                                                              d_transMat,
                                                              h_srcImage.step,
                                                              h_destImage.step,
                                                              offset,
                                                              constScreenCoordinate,
                                                              projPlane,
                                                              projPlaneDist,
                                                              h_srcImage.cols,
                                                              h_srcImage.rows,
                                                              h_destImage.rows,
                                                              srcImageBytes,
                                                              destImageBytes,
                                                              threadLimits);
                                                              
    err = cudaGetLastError();

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to launch cudaGetTransformedCoordinates kernel"
                        "(error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }
                                                              
    cudaDeviceSynchronize();
    
    err = cudaMemcpy(h_destImage.ptr(),d_destImage,destImageBytes,cudaMemcpyDeviceToHost);
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy Output Image from device to host"
                        "(error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }
}


void freeDeviceAllocatedMemory(int sig)
{
    cudaFree(d_srcImage);
    cudaFree(d_transMat);
    cudaFree(d_destImage);
    exit(0);
}
