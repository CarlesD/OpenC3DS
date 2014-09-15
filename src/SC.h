#include "ofMain.h"

//COMENTARI Anna: SC ha de ser una classe?
int Laser(int Laser_num, int estat, ofSerial *serial, int tms);
int Stepper(int Stepper_num, float angle, int steps, bool home, ofSerial *serial, int tms);

// pendents actualitzar
int LED(int led, int estat, int *serialPort);

int servoini(int servo, int *serialPort);
int iniDC();

int servop(int servo, float angle, int *serialPort);
int DCini(int servo, int *serialPort);
int DCp(int servo, float angle, int *serialPort);
