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
using namespace std;

//--------------------------------------------------------------
int Stepper(int Stepper_num, float Angle, int Steps, bool Home, ofSerial *serial, int tms){

    char bu[20];
    unsigned char *HOME;
    unsigned char *STEPS;
    unsigned char *ANGLE;
    int resp = 0;
    int iter = 0;

    if(Home == true){
        HOME = (unsigned char *)malloc(20 * sizeof(unsigned char));
        sprintf(bu,"%d 1 9 1\n",Stepper_num);
        memcpy(HOME,bu, 20);
        serial->writeBytes(&HOME[0], 20);

        while(resp != 69 && iter < 100){
            resp = serial->readByte();
            iter = iter + 1;
            ofSleepMillis(tms);
            ofLogNotice() << "SC:: Stepper: " << endl;
            cout << resp << endl;
        }
        if(iter < 20){
            ofLogNotice() << "SC:: Stepper: " << endl;
            cout << iter << endl;
            return(1);
        }
        else{
            ofLogError() << "SC:: Stepper: Serial port error" << endl;
            return(0);
        }
    }
    else{
        if(Steps == 0){
            ANGLE = (unsigned char *)malloc(20 * sizeof(unsigned char));
            sprintf(bu,"1 1 1 %f\n",Stepper_num,Angle);
            memcpy(ANGLE,bu, 20);
            serial->writeBytes(&ANGLE[0], 20);

            while(resp != 69 && iter < 100){
                resp = serial->readByte();
                iter = iter + 1;
                ofSleepMillis(tms);
                ofLogNotice() << "SC:: Stepper: " << endl;
                cout << resp << endl;
            }
            if(iter < 20){
                ofLogNotice() << "SC:: Stepper: " << endl;
                cout << iter << endl;
                return(1);
            }
            else{
                ofLogError() << "SC:: Stepper: Serial port error" << endl;
                return(0);
            }
        }
        if(Angle == 0){
            STEPS = (unsigned char *)malloc(20 * sizeof(unsigned char));
            sprintf(bu,"%d 1 2 %d\n",Stepper_num,Steps);
            memcpy(STEPS,bu, 20);
            serial->writeBytes(&STEPS[0], 20);

            while(resp != 69 && iter < 100){
                resp = serial->readByte();
                iter = iter + 1;
                ofSleepMillis(tms);

            }
            if(iter < 20){
                ofLogNotice() << "SC:: Stepper: OK"<< iter << endl;
                return(1);
            }
            else{
                ofLogError() << "SC:: Stepper: Serial port error" << endl;
                return(0);
            }
        } // end if(Angle == 0){
    } // end else
}

//--------------------------------------------------------------
int Laser(int Laser_num, int estat, ofSerial *serial, int tms){

    char bu[16];
    unsigned char LASER1_ON[] = {'2',' ','1',' ','0',' ','0','\n'};
    unsigned char LASER1_OFF[] = {'2',' ','0',' ','0',' ','0','\n'};
    unsigned char LASER2_ON[] = {'3',' ','1',' ','0',' ','0','\n'};
    unsigned char LASER2_OFF[] = {'3',' ','0',' ','0',' ','0','\n'};
    int resp = 0;
    int iter = 0;

    if(Laser_num == 1 && estat == 1){
        serial->writeBytes(&LASER1_ON[0], 8);
        while(resp != 70 && iter < 100){
            ofSleepMillis(tms);
            iter = iter + 1;
            resp = serial->readByte();
        }
    }
    if(Laser_num == 1 && estat == 0){
        serial->writeBytes(&LASER1_OFF[0], 8);
        while(resp != 71 && iter < 100){
            ofSleepMillis(tms);
            iter = iter + 1;
            resp = serial->readByte();
        }
    }
    if(Laser_num == 2 && estat == 1){
        serial->writeBytes(&LASER2_ON[0], 8);
        while(resp != 72 && iter < 100){
            ofSleepMillis(tms);
            iter = iter + 1;
            resp = serial->readByte();
        }
    }
    if(Laser_num == 2 && estat == 0){
        serial->writeBytes(&LASER2_OFF[0], 8);
        while(resp != 73 && iter < 100){
            ofSleepMillis(tms);
            iter = iter + 1;
            resp = serial->readByte();
        }
    }

    if(iter < 20){
        ofLogNotice() << "SC::Laser::OK "<< iter << endl;

        return(1);

    }
    else{
        ofLogError() << "SC::Laser:: Serial port error" << endl;
        return(0);
    }
}

