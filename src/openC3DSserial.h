#pragma once

#include "ofMain.h"
#include "ofxUI.h"

#define MOTOR       10
#define LASER1      1
#define LASER2      2
#define LASER3      3
#define LASER4      4

#define ACK_BYTES   4

class openC3DSserial{

	public:
		void setup();
		void update();
		void draw();
		void exit();

        // SERIAL PORT
		ofSerial	serialPort;
		int baudRate;
		string serialDeviceName;
		string strSend; // created string to send
		int nRead;
		unsigned char bytesReturned[ACK_BYTES];
		bool bisDeviceReady;

		// GUI
		ofxUISuperCanvas *guiSerial;
		void setGuiSerial();
		void guiEvent(ofxUIEventArgs &e);

		ofxUITextInput *serialDeviceGui;
		ofxUIRadio *serialBaudsGui;

		// STEPPER
		bool moveStepperToHome();
		bool moveStepperBySteps(int steps);
        bool moveTestLeft();
        bool moveTestRight();
        bool sendNoise();
        bool sendHello();

        // LASER
        bool blaser1On; // laser 1 on?
        bool blaser2On; // laser 2
        bool turnOnLaser(int laser);
        bool turnOffLaser(int laser);
};

