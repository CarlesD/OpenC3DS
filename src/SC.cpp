
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <stdint.h>


#include <iostream>

#include <sstream>
#include "SC.h"
#include "ofMain.h"
using namespace std;


int Stepper(int Stepper_num,float Angle,int Steps, bool Home, ofSerial *serial, int tms)
{

char bu[20];
unsigned char *HOME;
unsigned char *STEPS;
unsigned char *ANGLE;
unsigned int tus;
int resp=0;
int iter=0;
tus=tms*1000;

if (Home==true){
    HOME = (unsigned char *)malloc(20 * sizeof(unsigned char));
    sprintf(bu,"%d 1 9 1\n",Stepper_num);
    memcpy(HOME,bu, 20);
    serial->writeBytes(&HOME[0], 20);}

 else{

        if(Steps==0)
            {
                ANGLE = (unsigned char *)malloc(20 * sizeof(unsigned char));
                sprintf(bu,"%d 1 1 %f\n",Stepper_num,Angle);
                memcpy(ANGLE,bu, 20);
                serial->writeBytes(&ANGLE[0], 20);

                while (resp!=69 && iter<100)
                    {
                        resp=serial->readByte();
                        iter=iter+1;
                        usleep(tus);
                        cout <<resp<< endl;
                    }
                if(iter<20){cout <<"Stepper:"<<iter << endl;return(1);}
                else {cout << "Serial port error" << endl;return(0);}
            }

        if(Angle==0)
            {
                STEPS = (unsigned char *)malloc(20 * sizeof(unsigned char));
                sprintf(bu,"%d 1 2 %d\n",Stepper_num,Steps);
                memcpy(STEPS,bu, 20);
                serial->writeBytes(&STEPS[0], 20);

                while (resp!=69 && iter<100)
                    {
                        resp=serial->readByte();
                        iter=iter+1;
                        usleep(tus);
                        cout <<resp<< endl;
                    }
                if(iter<20){cout <<"Stepper:"<<iter << endl;return(1);}
                else {cout << "Serial port error" << endl;return(0);}
            }

 }

}




int Laser(int Laser_num,int estat,ofSerial *serial,int tms)
{
char bu[16];
unsigned char LASER1_ON[]={'5',' ','1',' ','0',' ','0','\n'};
unsigned char LASER1_OFF[]={'5',' ','0',' ','0',' ','0','\n'};
unsigned char LASER2_ON[]={'6',' ','1',' ','0',' ','0','\n'};
unsigned char LASER2_OFF[]={'6',' ','0',' ','0',' ','0','\n'};
int resp=0;
int iter=0;
unsigned int tus;
tus=tms*1000;


if(Laser_num==1 && estat ==1){serial->writeBytes(&LASER1_ON[0], 8); while (resp!=70 &&iter<100){usleep(tus);iter=iter+1;resp=serial->readByte(); }}
if(Laser_num==1 && estat ==0){serial->writeBytes(&LASER1_OFF[0], 8); while (resp!=71 &&iter<100){usleep(tus);iter=iter+1;resp=serial->readByte(); }}
if(Laser_num==2 && estat ==1){serial->writeBytes(&LASER2_ON[0], 8); while (resp!=69 &&iter<100){iter=iter+1;resp=serial->readByte();usleep(tus);}}
if(Laser_num==2 && estat ==0){serial->writeBytes(&LASER2_OFF[0], 8); while (resp!=69 &&iter<100){iter=iter+1;resp=serial->readByte(); usleep(tus);}}


if(iter<20){cout <<"Laser:"<<iter << endl; return(1);return(1);}
else {cout << "Serial port error" << endl;return(0);}


}

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

//void setPinMode(int pinID, int mode)
//{
//  writeFile(pinID, mode);
//}

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


