#include "openC3DScam.h"

//--------------------------------------------------------------
void openC3DScam::setup(){

	// GUI
	setGuiCam();
	guiCam->loadSettings("guiCam.xml");

	// CONTROL
	bimageYESlaser = false;

	// CAM
	camWidth = 1280;
	camHeight = 720;

	imgWidth = 640;
	imgHeight = 360;
	imgWidthPart = imgWidth * 0.3f;
	imgHeightPart = imgHeight * 0.3f;

	deviceID = 1;
	vidGrabber.setDeviceID(deviceID);
    vidGrabber.setVerbose(true);
    vidGrabber.initGrabber(camWidth, camHeight);

    colorImage.allocate(camWidth, camHeight);
	grayImageNOlaser.allocate(camWidth, camHeight);
	grayImageYESlaser.allocate(camWidth, camHeight);
	grayDiff.allocate(camWidth, camHeight);
	grayDiffTh.allocate(camWidth, camHeight);

}

//--------------------------------------------------------------
bool openC3DScam::updateColorImage(){
    bNewFrame = false;
    vidGrabber.update();
    bNewFrame = vidGrabber.isFrameNew();

	if(bNewFrame){
        colorImage.setFromPixels(vidGrabber.getPixels(), camWidth, camHeight);
	}
	return bNewFrame;
}

//--------------------------------------------------------------
bool openC3DScam::updateGrayImage(){
    if(bimageYESlaser){
        grayImageYESlaser = colorImage;
        grayImageYESlaser.blurGaussian(blur);
    }
    else{
        grayImageNOlaser = colorImage;
        grayImageNOlaser.blurGaussian(blur);
    }
}

//--------------------------------------------------------------
bool openC3DScam::updateGrayDiff(){
    if(bNewFrame){
        grayDiff.absDiff(grayImageYESlaser, grayImageNOlaser);
        grayDiffTh = grayDiff;
		grayDiffTh.threshold(threshold);
	}
}

//--------------------------------------------------------------
void openC3DScam::draw(){

}

//--------------------------------------------------------------
void openC3DScam::exit(){
    guiCam->saveSettings("guiCam.xml");
	delete guiCam;

    // TODO no tanca be, es queda la consola
    vidGrabber.close();
}

//--------------------------------------------------------------
void openC3DScam::setGuiCam(){

	guiCam = new ofxUISuperCanvas("WEBCAM");
    guiCam->addSpacer();

    guiCam->addLabel("CAMERA");

    guiCam->addSpacer();
    guiCam->addLabel("IMAGES");
    guiCam->addToggle("img_with_laser", &bimageYESlaser);
    guiCam->addSlider("threshold", 0,255, &threshold)->setIncrement(1);
    guiCam->addSlider("blur", 1,33, &blur)->setIncrement(2);

    guiCam->setPosition(0,0);
    guiCam->autoSizeToFitWidgets();
	ofAddListener(guiCam->newGUIEvent,this,&openC3DScam::guiEvent);
}

//--------------------------------------------------------------
void openC3DScam::guiEvent(ofxUIEventArgs &e){
	string name = e.getName();
	int kind = e.getKind();
}
