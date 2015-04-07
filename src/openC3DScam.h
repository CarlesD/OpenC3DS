#pragma once

#include "ofMain.h"
#include "ofxUI.h"
#include "ofxOpenCv.h"
#include "ofxXmlSettings.h"

class openC3DScam{

	public:
		void setup();
		bool updateColorImage();
		bool updateGrayImage();
		bool updateGrayDiff();

		void draw();
		void exit();

        // GUI
		ofxUISuperCanvas *guiCam;
		void setGuiCam();
		void guiEvent(ofxUIEventArgs &e);

        // CONTROL
        bool bimageYESlaser;

		// CAM
		ofVideoGrabber vidGrabber;
        bool bNewFrame;
		int camWidth;
		int camHeight;
		int deviceID;
		int imgWidth, imgHeight;
		float imgWidthPart, imgHeightPart;

        // OPENCV
        ofxCvColorImage			colorImage;
        ofxCvGrayscaleImage 	grayImage;
        ofxCvGrayscaleImage 	grayImageNOlaser;
		ofxCvGrayscaleImage 	grayImageYESlaser;
		ofxCvGrayscaleImage 	grayDiff;
		ofxCvGrayscaleImage 	grayDiffTh;
		float 				threshold;
		float               blur;

        // XML
		ofxXmlSettings xmlSettings;
};

