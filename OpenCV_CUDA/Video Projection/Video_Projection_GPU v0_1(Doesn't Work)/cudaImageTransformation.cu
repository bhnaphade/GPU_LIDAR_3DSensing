#include <stdio.h>
#include <cuda_runtime.h>
#include "cudaImageTransformation.h"


#define transMatCols    4
#define transMatRows    4

static float *d_transformationMat = NULL;
static twoDCoordinates *d_transformedCoordinates = NULL;

__device__ twoDCoordinates
cudaTransform(float* transformationMat,int xValue, int yValue,
              const int planeDistance,const twoDCoordinates offset,
              const int projectionPlane)
{
    twoDCoordinates twoDPoint;
    float matrixResult[transMatRows];
    
    float threeDPoint[transMatRows];
    
    switch(projectionPlane)
    {
        case 1:
            threeDPoint[0] = (float)planeDistance;
            threeDPoint[1] = (float)xValue;
            threeDPoint[2] = (float)yValue;
            break;
        case 2:
            threeDPoint[0] = (float)xValue;
            threeDPoint[1] = (float)planeDistance;
            threeDPoint[2] = (float)yValue;
            break;
        case 3:
            threeDPoint[0] = (float)xValue;
            threeDPoint[1] = (float)yValue;
            threeDPoint[2] = (float)planeDistance;
            break;
        default:
            //Error handling to be performed
            break;
    }
    
    float sum = 0;
    
    for(int i=0;i<transMatRows;i++)
    {
        for(int j=0;j<transMatCols;j++)
        {
            sum += (transformationMat[(i*transMatCols)+j] * threeDPoint[j]);
        }
        matrixResult[i] = sum;
        sum = 0.0;
    }
    
    twoDPoint.x = (int)((float)planeDistance * matrixResult[0]/matrixResult[2]);

    twoDPoint.x += offset.x;
                                  
    twoDPoint.y = (int)((float)planeDistance * matrixResult[1]/matrixResult[2]);
    twoDPoint.y += offset.y;
                                  
    return twoDPoint;
}

__global__ void
cudaGetTransformedCoordinates(twoDCoordinates* transformedCoordinates,
                              float* transformationMat,
                              const int planeDistance,
                              const twoDCoordinates offset, 
                              const int projectionPlane,
                              const int imageWidth,
                              const int imageHeight)
{
    //int2 temp;
    int xTid = (blockIdx.x + blockDim.x) + threadIdx.x;
    int yTid = (blockIdx.y + blockDim.y) + threadIdx.y;
    
    if((xTid < imageWidth)&& (yTid <imageHeight))
    {
//        temp 
        transformedCoordinates[(xTid*imageWidth)+yTid]= cudaTransform(transformationMat,
                                                             xTid,
                                                             yTid,
                                                             planeDistance,
                                                             offset,
                                                             projectionPlane);
                             
  //  transformedCoordinates[(xTid*imageWidth)+yTid].x = temp.x;
  //  transformedCoordinates[xTid][yTid].y = temp.y;                                                               
    }
}

void 
apiCudaGetTransformedCoordinates(twoDCoordinates* h_transformedCoordinates,
                                 float* h_transformationMat,
                                 const int planeDistance,
                                 const int xOffset,
                                 const int yOffset,
                                 const int projectionPlane,
                                 const int imageWidth,
                                 const int imageHeight)
{

    // Error code to check return values for CUDA calls
    cudaError_t err = cudaSuccess;
    
    twoDCoordinates offset;
    offset.x = xOffset;
    offset.y = yOffset;
    
    unsigned long int transformedCoordinatesSize = imageWidth*imageHeight*sizeof(twoDCoordinates);
    
    unsigned long int transformationMatSize = transMatCols*transMatRows*sizeof(float);
    
    if(d_transformedCoordinates==NULL)
    {
        err = cudaMalloc(&d_transformedCoordinates, transformedCoordinatesSize);
        if (err != cudaSuccess)
        {
            fprintf(stderr, "Failed to allocate device memory for Transformed Coordinates Array"
                            "(error code %s)!\n", cudaGetErrorString(err));
            exit(EXIT_FAILURE);
        }
    }
    
    err = cudaMemset(d_transformedCoordinates,0,transformedCoordinatesSize);
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to initialize device memory for Transformed Coordinates Array"
                        "(error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    if(d_transformationMat == NULL)
    {
        err = cudaMalloc(&d_transformationMat, transformationMatSize);
        if (err != cudaSuccess)
        {
            fprintf(stderr, "Failed to allocate device memory for Transformation Matrix"
                            "(error code %s)!\n", cudaGetErrorString(err));
            exit(EXIT_FAILURE);
        }
    
    
        //Copy host data on device
        err = cudaMemcpy(d_transformationMat,h_transformationMat,transformationMatSize,cudaMemcpyHostToDevice);
        if (err != cudaSuccess)
        {
            fprintf(stderr, "Failed to copy transformation Matrix from host to device (error code %s)!\n", cudaGetErrorString(err));
            exit(EXIT_FAILURE);
        }
    }
    
    //Kernel call   
    //***Hard coded for 320x240 image... need to generalize it
    dim3 blocks(10,10);
    dim3 threadsPerBlock(32,24);
    
    cudaGetTransformedCoordinates<<<blocks,threadsPerBlock>>>(d_transformedCoordinates,
                                                              d_transformationMat,
                                                              planeDistance,
                                                              offset, 
                                                              projectionPlane,
                                                              imageWidth,
                                                              imageHeight);
                                                              
    err = cudaGetLastError();

    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to launch cudaGetTransformedCoordinates kernel"
                        "(error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }
    
    cudaThreadSynchronize();
    
    err = cudaMemcpy(h_transformedCoordinates,d_transformedCoordinates,transformedCoordinatesSize,cudaMemcpyDeviceToHost);
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to copy transformed Coordinates from device to host"
                        "(error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }
}

//Need to handle gracefull exit
    
void freeDeviceAllocatedMemory()
{
    cudaFree(d_transformationMat);
    cudaFree(d_transformedCoordinates);
}
