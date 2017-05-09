#include <stdio.h>
#include "lib_ldr_data.c"

struct senData SD;

int main()
{

while(1)
{
    SD = get_lidar_data();

    printf("\nS0 %d S1 %d S2 %d S3 %d", SD.S0, SD.S1, SD.S2, SD.S3);
 }
 
}
