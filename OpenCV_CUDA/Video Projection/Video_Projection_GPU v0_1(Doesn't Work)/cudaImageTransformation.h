#ifndef _cudaImaheTransformation_h
#define _cudaImaheTransformation_h

#include <iostream> 

typedef struct 
{
    int x,y;
}twoDCoordinates;


void apiCudaGetTransformedCoordinates(twoDCoordinates* h_transformedCoordinates,
                                      float* h_transformationMat,
                                      const int planeDistance,
                                      const int xOffset,
                                      const int yOffset,
                                      const int projectionPlane,
                                      const int imageWidth,
                                      const int imageHeight);


void freeDeviceAllocatedMemory();

#endif //_cudaImaheTransformation_h
