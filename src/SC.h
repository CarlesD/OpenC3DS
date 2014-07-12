#ifndef __SERIAL_TEST_H__
#define __SERIAL_TEST_H__

#define SERIAL 3

#define GPIO_MODE_PATH "/sys/devices/virtual/misc/gpio/mode/"
#define GPIO_FILENAME "gpio"

void writeFile(int fileID, int value);
void setPinMode(int pinID, int mode);
void configurePins(void);

//#include <iostream>
//using namespace std;
//static const char* portName = "/dev/ttyUSB0";

//string portName = "/dev/ttyACM3";

int cam_laser(int las,int estat,int *serialPort,int tms);


void Serial_setup(int *serialPort,const char* portName );
int LED(int led,int estat,int *serialPort);

int servoini(int servo,int *serialPort);
int iniDC();
int STp(int servo,float angle,int steps, int home, int *serialPort, int tms);

int servop(int servo,float angle,int *serialPort);
int DCini(int servo,int *serialPort);
int DCp(int servo,float angle,int *serialPort);
int laser(int las,int estat,int *serialPort);
int openPort(const char* sPort, int nBaud);
#endif
