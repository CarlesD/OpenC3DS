#include "openC3DScam.h"

//--------------------------------------------------------------
void openC3DScam::setup(){

    // XML
    xmlSettings.loadFile("settingsCam.xml");

    camWidth = xmlSettings.getValue("OPENC3DS:CAM:camWidth", 1280);
	camHeight = xmlSettings.getValue("OPENC3DS:CAM:camHeight", 720);
	deviceID = xmlSettings.getValue("OPENC3DS:CAM:deviceID", 1);
	threshold = xmlSettings.getValue("OPENC3DS:CAM:threshold", 128);
	blur = xmlSettings.getValue("OPENC3DS:CAM:blur", 3);

    cout << "OPENC3DS:CAM:camWidth = " << camWidth << endl;
    cout << "OPENC3DS:CAM:camHeight = " << camHeight << endl;
    cout << "OPENC3DS:CAM:deviceID = " << deviceID << endl;
    cout << "OPENC3DS:CAM:threshold = " << threshold << endl;
    cout << "OPENC3DS:CAM:blur = " << blur << endl;

	// GUI
	setGuiCam();
	guiCam->loadSettings("guiCam.xml");

	// CONTROL
	bimageYESlaser = false;

	// CAM
	imgWidth = 640;
	imgHeight = 360;
	imgWidthPart = imgWidth * 0.3f;
	imgHeightPart = imgHeight * 0.3f;

	vidGrabber.setDeviceID(deviceID);
    vidGrabber.setVerbose(true);
    vidGrabber.initGrabber(camWidth, camHeight);

    colorImage.allocate(camWidth, camHeight);
	colorImageNOlaser.allocate(camWidth, camHeight);
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
        colorImageNOlaser = colorImage;
        grayImageNOlaser.blurGaussian(blur);
    }
}

//--------------------------------------------------------------
bool openC3DScam::updateGrayDiff(){
    if(bNewFrame){
        grayDiff.absDiff(grayImageYESlaser, grayImageNOlaser);
        grayDiffTh = grayDiff;
		grayDiffTh.threshold(threshold);
		grayDiff.updateTexture();
		grayDiffTh.updateTexture();
	}
}

//--------------------------------------------------------------
void openC3DScam::draw(){

}

//--------------------------------------------------------------
void openC3DScam::exit(){
    xmlSettings.clear();
    xmlSettings.setValue("OPENC3DS:CAM:camWidth", camWidth);
	xmlSettings.setValue("OPENC3DS:CAM:camHeight", camHeight);
	xmlSettings.setValue("OPENC3DS:CAM:deviceID", deviceID);
	xmlSettings.setValue("OPENC3DS:CAM:threshold", threshold);
	xmlSettings.setValue("OPENC3DS:CAM:blur", blur);

    xmlSettings.saveFile("settingsCam.xml");

    guiCam->saveSettings("guiCam.xml");
	delete guiCam;

    // TODO no tanca be, es queda la consola
    vidGrabber.close();
}

//--------------------------------------------------------------
void openC3DScam::setGuiCam(){

	guiCam = new ofxUISuperCanvas("WEBCAM");
    guiCam->addSpacer();

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
