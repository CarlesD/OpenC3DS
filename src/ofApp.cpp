#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    // APP
    ofSetFrameRate(60);
    ofSetVerticalSync(true);
	ofEnableSmoothing();

    // SERIAL
    serialCommunication.setup();
    // CAM
    webcamCapture.setup();
    // PROCESS
    scanerProcess.setup();
    scanerProcess.setupCamResolution(webcamCapture.camWidth, webcamCapture.camHeight);

	// SCANNER
	bscan = false;
	scannerState = SCANNER_IDLE;
	scanningSubState = SCANING_IDLE;

	posAxis1Steps = 0;
	currentLaser = 1;

    imgMain = new ofImage();
    imgMain->allocate(webcamCapture.imgWidth, webcamCapture.imgHeight, OF_IMAGE_COLOR);
    imgSmall1 = new ofImage();
    imgSmall1->allocate(webcamCapture.imgWidthPart, webcamCapture.imgHeightPart, OF_IMAGE_COLOR);
	imgSmall2 = new ofImage();
	imgSmall2->allocate(webcamCapture.imgWidthPart, webcamCapture.imgHeightPart, OF_IMAGE_GRAYSCALE);
	imgSmall3 = new ofImage();
	imgSmall3->allocate(webcamCapture.imgWidthPart, webcamCapture.imgHeightPart, OF_IMAGE_GRAYSCALE);
	imgSmall4 = new ofImage();
	imgSmall4->allocate(webcamCapture.imgWidthPart, webcamCapture.imgHeightPart, OF_IMAGE_GRAYSCALE);
	imgSmall5 = new ofImage();
	imgSmall5->allocate(webcamCapture.imgWidthPart, webcamCapture.imgHeightPart, OF_IMAGE_GRAYSCALE);
	imgSmallLASER = new ofImage();
    imgSmallLASER->allocate(webcamCapture.imgWidthPart, webcamCapture.imgHeightPart, OF_IMAGE_COLOR);
	imageMain = 0;

    // GUI scan
    blauFons.r = 100;
    blauFons.g = 27;
    blauFons.b = 200;
    totalGUIs = 7;

    guiOpenC3DS = new ofxUISuperCanvas("MAIN", 0,0, 2*ofGetWidth()/totalGUIs, ofGetHeight());
    guiOpenC3DS->addSpacer();

    guiOpenC3DS->addLabel("SCAN");
    guiOpenC3DS->addButton("start_scan", false);
    guiOpenC3DS->addButton("stop_scan", false);
    guiOpenC3DS->addRangeSlider("turn_deg_axis1", 0, 360, &iniAxis1degrees, &fiAxis1degrees)->setIncrement(1);
    guiOpenC3DS->addSlider("deg_increment_axis1", 0, 5, &incAxis1degrees)->setIncrement(0.05);
    guiOpenC3DS->addSlider("delay_laser_ms", 0, 5000, &delayLaserms)->setIncrement(1);
    guiOpenC3DS->addSlider("delay_stepper_ms", 0, 5000, &delayStepperms)->setIncrement(1);

    guiOpenC3DS->addLabel("MOTOR");
    guiOpenC3DS->addButton("turn_left", false);
    guiOpenC3DS->addButton("turn_right", false);

    guiOpenC3DS->addLabel("CAM");
    guiOpenC3DS->addButton("image_YES_laser", false);
    guiOpenC3DS->addButton("image_NO_laser", false);

    guiOpenC3DS->addLabel("PROCESS");
    guiOpenC3DS->addButton("process_img", false);

    guiOpenC3DS->addSpacer();
    sStateAndInfo = "a\n b\n c\n d\n e\n f\n g\n h\n i\n j\n";
    guiOpenC3DS->addTextArea("states_info", sStateAndInfo, OFX_UI_FONT_SMALL);

    guiOpenC3DS->setPosition(0,0);
    guiOpenC3DS->autoSizeToFitWidgets();
	ofAddListener(guiOpenC3DS->newGUIEvent,this,&ofApp::guiEvent);

    guiOpenC3DS->loadSettings("guiOpenC3DS.xml");

    // GUI images
    guiOpenC3DSimages = new ofxUISuperCanvas("IMAGES", 0,0, webcamCapture.imgWidth, ofGetHeight());
    guiOpenC3DSimages->addSpacer();
    guiOpenC3DSimages->addImage("MAIN IMAGE", imgMain);
    guiOpenC3DSimages->addSpacer();

    guiOpenC3DSimages->addWidgetDown(new ofxUIImage(webcamCapture.imgWidthPart, webcamCapture.imgHeightPart, imgSmall1, "color"));
    guiOpenC3DSimages->addWidgetRight(new ofxUIImage(webcamCapture.imgWidthPart, webcamCapture.imgHeightPart, imgSmall2, "YESlaser"));
    guiOpenC3DSimages->addWidgetRight(new ofxUIImage(webcamCapture.imgWidthPart, webcamCapture.imgHeightPart, imgSmall3, "NOlaser"));
    guiOpenC3DSimages->addWidgetDown(new ofxUIImage(webcamCapture.imgWidthPart, webcamCapture.imgHeightPart, imgSmall4, "diff"));
    guiOpenC3DSimages->addWidgetRight(new ofxUIImage(webcamCapture.imgWidthPart, webcamCapture.imgHeightPart, imgSmall5, "threshold"));
    guiOpenC3DSimages->addWidgetRight(new ofxUIImage(webcamCapture.imgWidthPart, webcamCapture.imgHeightPart, imgSmallLASER, "laserLINE"));

    guiOpenC3DSimages->setPosition(0,0);
    guiOpenC3DSimages->autoSizeToFitWidgets();
	ofAddListener(guiOpenC3DSimages->newGUIEvent,this,&ofApp::guiEvent);

    guiOpenC3DSimages->loadSettings("guiOpenC3DSimages.xml");

}

//--------------------------------------------------------------
void ofApp::update(){
    // SERIAL
    serialCommunication.update();

    // SCANNER
    if(bscan){
        if(scannerState == SCANNER_GOING_START){
            // CAM
            webcamCapture.updateColorImage();
            webcamCapture.updateGrayDiff();
            // SERIAL
            if(serialCommunication.bisDeviceReady){
                serialCommunication.moveStepperBySteps(recorregutToIniSteps);
                posAxis1Steps = iniAxis1degrees*STEPS_PER_DEGREE_AXIS1;
                scannerState = SCANNER_SCANING;
                scanningSubState = SCANING_IMG_OFF;
            }
        }
        else if(scannerState == SCANNER_SCANING){
            if( scanningSubState == SCANING_PROCESS){
                scanerProcess.camCaptureSubpixelProcess(webcamCapture.grayDiff.getPixels());
                scanerProcess.Component_3D_Angular_1_axis_Scan(currentLaser, webcamCapture.colorImage.getPixels(), ofDegToRad(posAxis1Steps/STEPS_PER_DEGREE_AXIS1));
                scanningSubState = SCANING_IMG_OFF;
            }
            else if( scanningSubState == SCANING_IMG_OFF){
                webcamCapture.bimageYESlaser = false;
                if(webcamCapture.updateColorImage() == true){
                    webcamCapture.updateGrayImage();
                    webcamCapture.updateGrayDiff();
                    scanningSubState = SCANING_LASER_ON;
                }
            }
            else if( scanningSubState == SCANING_IMG_ON){
                webcamCapture.bimageYESlaser = true;
                if(webcamCapture.updateColorImage() == true){
                    webcamCapture.updateGrayImage();
                    webcamCapture.updateGrayDiff();
                    scanningSubState = SCANING_LASER_OFF;
                }
            }
            else if(scanningSubState == SCANING_LASER_ON){
                if(serialCommunication.bisDeviceReady){
                    serialCommunication.turnOnLaser(currentLaser);
                    ofSleepMillis(delayLaserms);
                    scanningSubState = SCANING_IMG_ON;
                }
            }
            else if(scanningSubState == SCANING_LASER_OFF){
                if(serialCommunication.bisDeviceReady){
                    serialCommunication.turnOffLaser(currentLaser);
                    ofSleepMillis(delayLaserms);
                    if(currentLaser != NUM_LASERS){
                        scanningSubState = SCANING_CHANGE_LASER;
                    }
                    else{
                        scanningSubState = SCANING_MOVE;
                        currentLaser = 1;
                    }
                }
            }
            else if(scanningSubState == SCANING_CHANGE_LASER){
                currentLaser++;
                currentLaser = (currentLaser%NUM_LASERS);
                if(currentLaser == 0){
                    currentLaser = NUM_LASERS;
                }
                scanningSubState = SCANING_PROCESS;
            }
            else if(scanningSubState == SCANING_MOVE){
                if(serialCommunication.bisDeviceReady){
                    if(posAxis1Steps < fiAxis1degrees*STEPS_PER_DEGREE_AXIS1){
                        serialCommunication.moveStepperBySteps(incAxis1degrees*STEPS_PER_DEGREE_AXIS1);
                        ofSleepMillis(delayStepperms);
                        posAxis1Steps += incAxis1degrees*STEPS_PER_DEGREE_AXIS1;
                        scanningSubState = SCANING_PROCESS;
                    }
                    else{
                        scannerState = SCANNER_IDLE;
                        bscan = false;
                    }
                }
            }
        }
    }
    else{ // no scaning
        // CAM
        webcamCapture.updateColorImage();
        webcamCapture.updateGrayDiff();
    }

    // update images to showguiOpenC3DS->autoSizeToFitWidgets();
    imgSmall1->setFromPixels(webcamCapture.colorImage.getPixels(), webcamCapture.camWidth, webcamCapture.camHeight, OF_IMAGE_COLOR);
    imgSmall2->setFromPixels(webcamCapture.grayImageYESlaser.getPixels(), webcamCapture.camWidth, webcamCapture.camHeight, OF_IMAGE_GRAYSCALE);
    imgSmall3->setFromPixels(webcamCapture.grayImageNOlaser.getPixels(), webcamCapture.camWidth, webcamCapture.camHeight, OF_IMAGE_GRAYSCALE);
    imgSmall4->setFromPixels(webcamCapture.grayDiff.getPixels(), webcamCapture.camWidth, webcamCapture.camHeight, OF_IMAGE_GRAYSCALE);
    imgSmall5->setFromPixels(webcamCapture.grayDiffTh.getPixels(), webcamCapture.camWidth, webcamCapture.camHeight, OF_IMAGE_GRAYSCALE);
    imgSmallLASER->setFromPixels(scanerProcess.imgLaserLineSubpixel.getPixels(), scanerProcess._camWidth, scanerProcess._camHeight, OF_IMAGE_COLOR);
    if(imageMain == COLOR){
        imgMain->setFromPixels(webcamCapture.colorImage.getPixels(), webcamCapture.camWidth, webcamCapture.camHeight, OF_IMAGE_COLOR);
    }
    else if(imageMain == GRAY_YES_LASER){
        imgMain->setFromPixels(webcamCapture.grayImageYESlaser.getPixels(), webcamCapture.camWidth, webcamCapture.camHeight, OF_IMAGE_GRAYSCALE);
    }
    else if(imageMain == GRAY_NO_LASER){
        imgMain->setFromPixels(webcamCapture.grayImageNOlaser.getPixels(), webcamCapture.camWidth, webcamCapture.camHeight, OF_IMAGE_GRAYSCALE);
    }
    else if(imageMain == GRAY_DIFF){
        imgMain->setFromPixels(webcamCapture.grayDiff.getPixels(), webcamCapture.camWidth, webcamCapture.camHeight, OF_IMAGE_GRAYSCALE);
    }
    else if(imageMain == GRAY_DIFF_TH){
        imgMain->setFromPixels(webcamCapture.grayDiffTh.getPixels(), webcamCapture.camWidth, webcamCapture.camHeight, OF_IMAGE_GRAYSCALE);
    }
    else if(imageMain == LASER_LINE){
        imgMain->setFromPixels(scanerProcess.imgLaserLineSubpixel.getPixels(), scanerProcess._camWidth, scanerProcess._camHeight, OF_IMAGE_COLOR);
    }

}

//--------------------------------------------------------------
void ofApp::draw(){
    ofSetWindowTitle("OpenColor3DScan at " + ofToString(ofGetFrameRate()) + "fps");
    ofBackground(blauFons);
    sStateAndInfo = "FPS: " + ofToString(ofGetFrameRate()) + "\n" + "\n";
    sStateAndInfo += "scanner state: " + ofToString(getStringStates(scannerState)) + "\n" + "\n";
    sStateAndInfo += "scanning substate: " + ofToString(getStringSubStates(scanningSubState)) + "\n" + "\n";
    sStateAndInfo += "pos axis1 steps: " + ofToString(posAxis1Steps) + "\n" + "\n";
    sStateAndInfo += "current laser: " + ofToString(currentLaser) + "\n";
    ofxUITextArea *ta = (ofxUITextArea *) guiOpenC3DS->getWidget("states_info");
    ta->setTextString(sStateAndInfo);

    scanerProcess.draw();
}

//--------------------------------------------------------------
void ofApp::exit(){
    // GUIs
    guiOpenC3DS->saveSettings("guiOpenC3DS.xml");
	delete guiOpenC3DS;
    guiOpenC3DSimages->saveSettings("guiOpenC3DSimages.xml");
	delete guiOpenC3DSimages;

    // SERIAL
    serialCommunication.exit();
    // CAM
    webcamCapture.exit();
    // PROCESS
    scanerProcess.exit();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

    if(key == 'a'){
        imageMain = COLOR;
    }
    else if(key == 's'){
        imageMain = GRAY_YES_LASER;
    }
    else if(key == 'd'){
        imageMain = GRAY_NO_LASER;
    }
    else if(key == 'f'){
        imageMain = GRAY_DIFF;
    }
    else if(key == 'g'){
        imageMain = GRAY_DIFF_TH;
    }
    else if(key == 'h'){
        imageMain = LASER_LINE;
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}

//--------------------------------------------------------------
void ofApp::guiEvent(ofxUIEventArgs &e){
    string name = e.getName();
	int kind = e.getKind();

    //cout << "guiEvent" << endl;
	//cout << "  name: " << name << endl;
	//cout << "  kind: " << kind << endl;

	if(name == "start_scan"){
        bscan = true;
        scannerState = SCANNER_GOING_START;
        recorregutToIniSteps = iniAxis1degrees * STEPS_PER_DEGREE_AXIS1 - posAxis1Steps;
	}
	else if(name == "stop_scan"){
        bscan = false;
	}
	else if(name == "turn_left"){
        serialCommunication.moveStepperBySteps(8888);
	}
	else if(name == "turn_right"){
        serialCommunication.moveStepperBySteps(-8888);
	}
	else if(name == "image_YES_laser"){
        webcamCapture.bimageYESlaser = true;
        webcamCapture.updateGrayImage();
	}
	else if(name == "image_NO_laser"){
        webcamCapture.bimageYESlaser = false;
        webcamCapture.updateGrayImage();
	}
	else if(name == "process_img"){
        scanerProcess.camCaptureSubpixelProcess(webcamCapture.grayDiff.getPixels());
	}
}

//--------------------------------------------------------------
string ofApp::getStringStates(int s){
    if(s == 0){
        return "SCANNER_IDLE";
    }
    else if(s == 1){
        return "SCANNER_GOING_HOME";
    }
    else if(s == 2){
        return "SCANNER_GOING_START";
    }
    else if(s == 3){
        return "SCANNER_SCANING";
    }
}

//--------------------------------------------------------------
string ofApp::getStringSubStates(int s){
    if(s == 0){
        return "SCANING_IDLE";
    }
    else if(s == 1){
        return "SCANING_PROCESS";
    }
    else if(s == 2){
        return "SCANING_CHANGE_LASER";
    }
    else if(s == 3){
        return "SCANING_MOVE";
    }
    else if(s == 4){
        return "SCANING_LASER_ON";
    }
    else if(s == 5){
        return "SCANING_LASER_OFF";
    }
    else if(s == 6){
        return "SCANING_IMG_ON";
    }
    else if(s == 7){
        return "SCANING_IMG_OFF";
    }
}
