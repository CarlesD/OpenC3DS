#pragma once

#include "ofMain.h"
#include "ofxUI.h"

#include "openC3DSserial.h"
#include "openC3DScam.h"
#include "openC3DSprocess.h"

#define NUM_LASERS      2

#define I_ANGULAR_AXIS1                                         26.851239669 // relació de transmissió del reductor del motor
#define STEPS_X_MOTOR_REVOLUTION_AXIS1                          6400 // passos que estableix el driver del motor per cada volta
                                                                     // el motor s de 200 steps per revolució
                                                                     // el driver fa 32 substeps
                                                                     // 200 * 32 = 6400
#define STEPS_X_REVOLUTION_AXIS1                                I_ANGULAR_AXIS1 * STEPS_X_MOTOR_REVOLUTION_AXIS1
#define STEPS_PER_DEGREE_AXIS1                                  STEPS_X_REVOLUTION_AXIS1 / 360.0f // passos/grau

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
		void exit();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		// GUI
		ofColor blauFons;
		ofxUICanvas *guiOpenC3DS;
		ofxUICanvas *guiOpenC3DSimages;
		int totalGUIs;
        void guiEvent(ofxUIEventArgs &e);

        // SERIAL
		openC3DSserial serialCommunication;

		// CAM
		openC3DScam webcamCapture;

		// PROECSS
		openC3DSprocess scanerProcess;

		// SCANNER
		// states
		enum scannerStates{ SCANNER_IDLE = 0, SCANNER_GOING_HOME, SCANNER_GOING_START, SCANNER_SCANING };
		enum scanningSubStates{ SCANING_IDLE = 0, SCANING_PROCESS, SCANING_CHANGE_LASER, SCANING_MOVE, SCANING_LASER_ON, SCANING_LASER_OFF, SCANING_IMG_ON, SCANING_IMG_OFF };
        string getStringStates(int s);
        string getStringSubStates(int s);
        string sStateAndInfo;

		int scannerState;
		int scanningSubState;

        bool bscan;

		//positions
		float iniAxis1degrees, fiAxis1degrees; // posició angular inicial d'scaneig i posició angular final
		float incAxis1degrees; // increment de posició angular
		float posAxis1Steps;
		float recorregutToIniSteps;

        // laser control
		int currentLaser;

		// delays
		float delayLaserms;
		float delayStepperms;

		// images
		enum images{ COLOR = 0, GRAY_YES_LASER, GRAY_NO_LASER, GRAY_DIFF, GRAY_DIFF_TH };
		ofImage *imgMain;
		int imageMain;
		ofImage *imgSmall1;
		ofImage *imgSmall2;
		ofImage *imgSmall3;
		ofImage *imgSmall4;
		ofImage *imgSmall5;
		int imgWidth, imgHeight;
		float imgWidthHalf, imgHeightHalf;
};
