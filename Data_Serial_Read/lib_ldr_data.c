  //https://chrisheydrick.com/2012/06/17/how-to-read-serial-data-from-an-arduino-in-linux-with-c-part-3/
  
      #include <sys/types.h>
      #include <sys/stat.h>
      #include <fcntl.h>
      #include <termios.h>
      #include <stdio.h>
      #include <stdlib.h>
        #include <string.h> 
        #include <unistd.h>  /* UNIX Standard Definitions 	   */ 
     #include <string.h>
#include "lib_ldr_data.h"



  #define readSize 8

unsigned char buf[readSize];


struct senData get_lidar_data();

#define GET_DATA(X,Y) (X<<8)|Y

/* My Arduino is on /dev/ttyACM0 */
extern char *portname;
extern unsigned char buf[readSize];


struct senData get_lidar_data()
{
 int fd;
 char *portname = "/dev/ttyTHS1";
/* Open the file descriptor in non-blocking mode */
 fd = open(portname, O_RDWR | O_NOCTTY);
 
/* Set up the control structure */
 struct termios toptions;
 
 /* Get currently set options for the tty */
 tcgetattr(fd, &toptions);
 
/* Set custom options */
 
/* 9600 baud */
 cfsetispeed(&toptions, B9600);
 cfsetospeed(&toptions, B9600);
 /* 8 bits, no parity, no stop bits */
 toptions.c_cflag &= ~PARENB;
 toptions.c_cflag &= ~CSTOPB;
 toptions.c_cflag &= ~CSIZE;
 toptions.c_cflag |= CS8;
 /* no hardware flow control */
 toptions.c_cflag &= ~CRTSCTS;
 /* enable receiver, ignore status lines */
 toptions.c_cflag |= CREAD | CLOCAL;
 /* disable input/output flow control, disable restart chars */
 toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
 /* disable canonical input, disable echo,
 disable visually erase chars,
 disable terminal-generated signals */
 toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
 /* disable output processing */
 toptions.c_oflag &= ~OPOST;
 
/* wait for 12 characters to come in before read returns */
/* WARNING! THIS CAUSES THE read() TO BLOCK UNTIL ALL */
/* CHARACTERS HAVE COME IN! */
 toptions.c_cc[VMIN] = readSize;
 /* no minimum time to wait before read returns */
 toptions.c_cc[VTIME] = 0;
 
/* commit the options */
 tcsetattr(fd, TCSANOW, &toptions);
 
/* Wait for the Arduino to reset */
 //usleep(1000*1000);
 char start ='S';

   tcflush(fd, TCIFLUSH);
  write(fd, &start, 1);
  usleep(1);

 struct  senData sD;
// while(1){
 /* Flush anything already in the serial buffer */
    tcflush(fd, TCIFLUSH);
    read(fd, &buf, readSize);

     sD.S0 = GET_DATA(buf[0],buf[1]);
     sD.S1 = GET_DATA(buf[2],buf[3]);
     sD.S2 = GET_DATA(buf[4],buf[5]);
     sD.S3 = GET_DATA(buf[6],buf[7]);
        
    /*     senData1 =(buf[0] << 8);
 senData1 |= buf[1]; */     
   // printf("\nbytesRead=%d SenData%d = %d hex[0]=%2x hex[1]=%2x",n,i,senData1[i],buf[j],buf[j+1]); 
  //  printf("\nData bytes %d S0 %d S1 %d S2 %d S3 %d",n,  sD.S0,sD.S1,sD.S2,sD.S3);
  
/* print how many bytes read */
 //printf("\n%d %d",buf[0]-'0',buf[1]-'0');
//}
close(fd);
return sD;
}
