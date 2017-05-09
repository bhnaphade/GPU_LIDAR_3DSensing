#ifndef lib_ldr_data_h__
#define lib_ldr_data_h__
 
       #include <sys/types.h>
      #include <sys/stat.h>
       #include <stdio.h>
      #include <stdlib.h>
        #include <string.h> 
        #include <unistd.h>  /* UNIX Standard Definitions 	   */ 
     #include <string.h>
#include <stdint.h> 


 
 struct senData
 {
 uint16_t S0;
 uint16_t S1;
 uint16_t S2;
 uint16_t S3;
 };
 
 struct senData get_lidar_data();
 
#endif  // lib_ldr_data_h__
