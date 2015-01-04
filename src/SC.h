#include "ofMain.h"

//COMENTARI Anna: SC ha de ser una classe?
int Laser(int Laser_num, int estat, ofSerial *serial, int tms);
int Stepper(int Stepper_num, float angle, int steps, bool home, ofSerial *serial, int tms);
