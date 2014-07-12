
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <stdint.h>


#include <iostream>

#include <sstream>
#include "SC.h"

using namespace std;

int openPort(const char* sPort, int nBaud) {
struct termios toptions;
int fd;
fd = open(sPort, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
if (fd == -1) {
perror("init_serialport: Unable to open port ");
return -1;
}

if (tcgetattr(fd, &toptions) < 0) {
perror("init_serialport: Couldn't get term attributes");
return -1;
}
speed_t brate = nBaud; // let you override switch below if needed
switch(nBaud) {
case 4800: brate=B4800; break;
case 9600: brate=B9600; break;
#ifdef B14400
case 14400: brate=B14400; break;
#endif
case 19200: brate=B19200; break;
#ifdef B28800
case 28800: brate=B28800; break;
#endif
case 38400: brate=B38400; break;
case 57600: brate=B57600; break;
case 115200: brate=B115200; break;
}
cfsetispeed(&toptions, brate);
cfsetospeed(&toptions, brate);

// 8N1
toptions.c_cflag &= ~PARENB;
toptions.c_cflag &= ~CSTOPB;
toptions.c_cflag &= ~CSIZE;
toptions.c_cflag |= CS8;
// no flow control
toptions.c_cflag &= ~CRTSCTS;

toptions.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines
toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
toptions.c_oflag &= ~OPOST; // make raw

// see: http://unixwiz.net/techtips/termios-vmin-vtime.html
toptions.c_cc[VMIN] = 0;
toptions.c_cc[VTIME] = 20;

if( tcsetattr(fd, TCSANOW, &toptions) < 0) {
perror("init_serialport: Couldn't set term attributes");
return -1;
}
return fd;
}

void Serial_setup(int *serialPort,const char* portName )
{

char path[256];
int i;


  // File descriptor for serial port
  struct termios portOptions; // struct to hold the port settings
  // Open the serial port as read/write, not as controlling terminal, and
  //   don't block the CPU if it takes too long to open the port.
  *serialPort = open(portName, O_RDWR | O_NOCTTY | O_NDELAY );


  // Fetch the current port settings
  tcgetattr(*serialPort, &portOptions);

  // Flush the port's buffers (in and out) before we start using it
  tcflush(*serialPort, TCIOFLUSH);

  // Set the input and output baud rates
  cfsetispeed(&portOptions, B115200);
  cfsetospeed(&portOptions, B115200);

  // c_cflag contains a few important things- CLOCAL and CREAD, to prevent
  //   this program from "owning" the port and to enable receipt of data.
  //   Also, it holds the settings for number of data bits, parity, stop bits,
  //   and hardware flow control.
  portOptions.c_cflag |= CLOCAL;
  portOptions.c_cflag |= CREAD;
  // Set up the frame information.
  portOptions.c_cflag &= ~CSIZE; // clear frame size info
  portOptions.c_cflag |= CS8;    // 8 bit frames
  portOptions.c_cflag &= ~PARENB;// no parity
  portOptions.c_cflag &= ~CSTOPB;// one stop bit

  // Now that we've populated our options structure, let's push it back to the
  //   system.
  tcsetattr(*serialPort, TCSANOW, &portOptions);

  // Flush the buffer one more time.
  tcflush(*serialPort, TCIOFLUSH);

}


//int servoini(int servo, int *serialPort)
//{
//servop(servo,0,&serialPort);
//sleep(2);
//return(0);
//}

int servop(int servo,float angle, int *serialPort)
{
float pos;
int posi;
unsigned char bu[10];
int i,control,n;
control=0;
int posaux=0;
pos=(angle/180)*720;
posi=(int)pos;

bu[0]=6;
bu[1]=servo;

	for(i=2;i<=8;i++)
	{
	bu[i]=0;
	}

	if(posi<=127){bu[2]=posi;}
	else{
		for(i=2;i<=8;i++)
		{
		posaux=posi-127;
		if(posaux>=0){bu[i]=127;}
		if(posaux<0 &&  control==0){bu[i]=127+posaux;control=1;}
		posi=posaux;
		}
	    }



write(*serialPort, bu, 9);


return(0);
}

int DCini(int servo, int *serialPort)
{

char bu[15];
unsigned char resp;


sprintf (bu, "%d %d %d %f\n", 2,1,5,1);
while (resp!='E'){
       write(*serialPort, bu, 15);
        read(*serialPort, &resp, 1);
        usleep(80000);}
resp=0;

printf("motor inicialitzat");

return(0);
}

int LED(int led,int estat,int *serialPort)
{

char bu[15];
unsigned char resp=0;

if (estat==1)sprintf (bu, "%d %d %d %f\n", 9,1,1,1);
if (estat==0)sprintf (bu, "%d %d %d %f\n", 9,0,0,0);

   write(*serialPort, bu, 15);
while (resp!='E'){

        read(*serialPort, &resp, 1);
        usleep(30000);}
resp=0;
return(0);

}
int DCp(int servo,float angle, int *serialPort)
{

char bu[15];
unsigned char resp=0;


sprintf (bu, "%d %d %d %f\n", 2,1,1,(angle*8));
   write(*serialPort, bu, 15);
while (resp!='E'){

        read(*serialPort, &resp, 1);
        usleep(80000);}
resp=0;
return(0);

}

int STp(int servo,float angle,int steps, int home, int *serialPort, int tms)
{

char bu[20];
unsigned char resp=0;
unsigned int tus;
int iter=0;
tus=tms*1000;
if(steps==0)
{
    if (home==1){sprintf (bu, "%d %d %d %f\n", 1,1,9,1);}
else{sprintf (bu, "%d %d %d %f\n", 1,1,1,(angle));}

   write(*serialPort, bu, 20);
while (resp!='E'&&iter<100){

        read(*serialPort, &resp, 1);
        iter=iter+1;
        usleep(tus);}

if(iter<20){return(1);}
else {cout << "Serial port error" << endl;return(0);}
}
else{
    if (home==1){sprintf (bu, "%d %d %d %f\n", 1,1,9,1);}
else{sprintf (bu, "%d %d %d %d\n", 1,1,2,steps);}

   write(*serialPort, bu, 20);
while (resp!='E'&&iter<100){

        read(*serialPort, &resp, 1);
        iter=iter+1;
        usleep(tus);}

if(iter<20){return(1);}
else {cout << "Serial port error" << endl;return(0);}
}



}

int cam_laser(int las,int estat,int *serialPort, int tms)
{
char bu[16];
unsigned char resp=0;
int iter=0;
unsigned int tus;
tus=tms*1000;


if(las==1 && estat ==1){sprintf (bu, "%d %d %d %f\n", 5,1,0,0);while (resp!='F'&&iter<100){iter=iter+1;write(*serialPort, bu, 15);read(*serialPort, &resp, 1);usleep(tus);}}
if(las==1 && estat ==0){sprintf (bu, "%d %d %d %f\n", 5,0,0,0);while (resp!='G'&&iter<100){iter=iter+1;write(*serialPort, bu, 15);read(*serialPort, &resp, 1);usleep(tus);}}
if(las==2 && estat ==1){sprintf (bu, "%d %d %d %f\n", 6,1,0,0);while (resp!='E'&&iter<100){iter=iter+1;write(*serialPort, bu, 15);read(*serialPort, &resp, 1);usleep(tus);}}
if(las==2 && estat ==0){sprintf (bu, "%d %d %d %f\n", 6,0,0,0);while (resp!='E'&&iter<100){iter=iter+1;write(*serialPort, bu, 15);read(*serialPort, &resp, 1);usleep(tus);}}



if(iter<20){return(1);}
else {cout << "Serial port error" << endl;return(0);}


}

int laser(int las,int estat,int *serialPort)
{
float pos;
int posi;
unsigned char bu[10];
int i;

if(las==0 && estat ==1)bu[0]=11;
if(las==1 && estat ==1)bu[0]=12;
if(las==2 && estat ==1)bu[0]=13;
if(las==0 && estat ==0)bu[0]=14;
if(las==1 && estat ==0)bu[0]=15;
if(las==2 && estat ==0)bu[0]=16;

	for(i=1;i<=8;i++)
	{
	bu[i]=0;
	}



write(*serialPort, bu, 9);
//sleep(1);

return(0);
}

void setPinMode(int pinID, int mode)
{
  writeFile(pinID, mode);
}

// While it seems okay to only *read* the first value from the file, you
//   seemingly must write four bytes to the file to get the I/O setting to
//   work properly. This function does that.
void writeFile(int fileID, int value)
{
  char buffer[4];  // A place to build our four-byte string.
  memset((void *)buffer, 0, sizeof(buffer)); // clear the buffer out.
  sprintf(buffer, "%d", value);
  lseek(fileID, 0, SEEK_SET);   // Make sure we're at the top of the file!
  int res = write(fileID, buffer, sizeof(buffer));
}



void putser(int *serialPort,int servo, int angle){
   unsigned char buff[6];
   //DWORD len;
int i;

   unsigned short int temp;
   unsigned char pos_hi,pos_low;
   angle=750+ ((angle*5000)/180);

   temp=angle&0x1f80;
 pos_hi=temp>>7;
  pos_low=angle & 0x7f;
////pos_low=254*angle/180;

 ////  buff[0]=0xFF;//start byte
 ////  buff[1]=servo;//device id
 ////  buff[2]=pos_low;//command number
   //buff[3]=servo;//servo number
   //buff[4]=pos_hi;//data1
   //buff[5]=pos_low;//data2
//write(serialPort, buff, 3);
  // pos_hi=(unsigned char)(angle >> 7);   // lower byte
  //pos_low=(unsigned char)(0x7f & angle);

   buff[0]=0x80;//start byte
   buff[1]=0x01;//device id
   buff[2]=0x04;//command number
   buff[3]=servo;//servo number
   buff[4]=pos_hi;//data1
   buff[5]=pos_low;//data2

   write(*serialPort, buff, 6);


}


void servoSetSpeed(int *serialPort,int servo, int speed){

   // Build a Pololu Protocol Command Sequence
   unsigned char cmd[5];
speed=speed & 0x7f;
   cmd[0] = 0x80;     // start byte
   cmd[1] = 0x01;     // device id
   cmd[2] = 0x01;     // command number
   cmd[3] = (unsigned char)(servo);    // servo number
   cmd[4] = (unsigned char)(speed);    // speed

  // Send the command to the Pololu Servo Controller

   write(*serialPort, cmd, 5);

}


