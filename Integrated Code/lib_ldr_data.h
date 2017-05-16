#ifndef lib_ldr_data_h__
#define lib_ldr_data_h__
 
#include <stdint.h>


 
typedef struct _senData
 {
 uint16_t S0;
 uint16_t S1;
 uint16_t S2;
 uint16_t S3;
 }senData;
 
extern senData get_lidar_data();
 
#endif  // lib_ldr_data_h__
