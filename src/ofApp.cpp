#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    // APP
    ofSetFrameRate(60);
    ofSetVerticalSync(true);
	ofEnableSmoothing();

	// LOGs
	ofSetLogLevel(OF_LOG_ERROR);

    // SERIAL
    serialCommunication.setup();
    // CAM
    webcamCapture.setup();
    // PROCESS
    scanerProcess.setup();
    scanerProcess.setupCamResolution(webcamCapture.camWidth, webcamCapture.camHeight);

	// SCANNER
	bscan = false;
	bscanstep = false;
    bstatePerformedActionOk = false;
	scannerState = SCANNER_IDLE;
	scanningSubState = SCANING_IDLE;

	posAxis1Steps = 0;
	currentLaser = 0;

    imgMain = new ofImage();
    imgMain->allocate(webcamCapture.imgWidth, webcamCapture.imgHeight, OF_IMAGE_COLOR);
    imgMainTmp = new ofImage();
    imgMainTmp->allocate(webcamCapture.imgWidth, webcamCapture.imgHeight, OF_IMAGE_COLOR);
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

    guiOpenC3DS->addLabel("AUTOMATIC SCAN");
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
    guiOpenC3DS->addButton("image_diff_and_th", false);

    guiOpenC3DS->addLabel("PROCESS");
    guiOpenC3DS->addButton("process_img", false);

    guiOpenC3DS->addLabel("CALIBRATION");
    guiOpenC3DS->addButton("calibrate_deviation_laser0", false);
    guiOpenC3DS->addButton("calibrate_deviation_laser1", false);
    guiOpenC3DS->addButton("calc_dist_laser0", false);
    guiOpenC3DS->addButton("calc_dist_laser1", false);
    guiOpenC3DS->addSlider("pos_v", 0, webcamCapture.camWidth, &posV)->setIncrement(1);
    guiOpenC3DS->addSlider("pos_h", 0, webcamCapture.camHeight, &posH)->setIncrement(1);

    guiOpenC3DS->addSpacer();
    sStateAndInfo = "a\n b\n c\n d\n e\n f\n g\n h\n i\n j\n";
    guiOpenC3DS->addTextArea("states_info", sStateAndInfo, OFX_UI_FONT_SMALL);

    guiOpenC3DS->setPosition(0,0);
    guiOpenC3DS->autoSizeToFitWidgets();
	ofAddListener(guiOpenC3DS->newGUIEvent,this,&ofApp::guiEvent);

    guiOpenC3DS->loadSettings("guiOpenC3DS.xml");
    guiOpenC3DS->disableAppDrawCallback();

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
    guiOpenC3DSimages->disableAppDrawCallback();

    // double click, zoom and drag images
    delayMsAfterMouseReleasedToConsiderDoubleClick = 300; // ms
    scale = 1.0f;
    relAspectWidth = webcamCapture.camWidth / webcamCapture.imgWidth;
    relAspectHeight = webcamCapture.camHeight / webcamCapture.imgHeight;
    mouseXdiff = mouseYdiff = 0;
    acumulatedDiffX = acumulatedDiffY = 0;
    bmouseDragged = false;

}

//--------------------------------------------------------------
void ofApp::update(){
    // SERIAL
    serialCommunication.update();

    // SCANNER
    // UPDATE STATE for NEXT LOOP
    bstatePerformedActionOk = true;
    if(bscan || bscanstep){
        if(scannerState == SCANNER_GOING_START){
            //ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "ofApp::update::scannerState == SCANNER_GOING_START");
            if(bstatePerformedActionOk){
                scannerState = SCANNER_SCANING;
                scanningSubState = SCANING_IMG_OFF;
            }
        }
        else if(scannerState == SCANNER_SCANING){
            //ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "ofApp::update::scannerState == SCANNER_SCANING");
            if( scanningSubState == SCANING_DELAY){
                //ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "ofApp::update::scannerState == SCANING_DELAY");
                if(ofGetElapsedTimeMillis() - startTimeDelayMillis > endDelayMillis){
                    if(prevScanningSubState == SCANING_LASER_ON){
                        scanningSubState = SCANING_IMG_ON;
                    }
                    else if(prevScanningSubState == SCANING_MOVE){
                        scanningSubState = SCANING_IMG_OFF;
                    }
                    bstatePerformedActionOk = true; // force change state
                }
            }
            else if( scanningSubState == SCANING_PROCESS){
                //ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "ofApp::update::scannerState == SCANING_PROCESS");
                scanningSubState = SCANING_LASER_OFF;
            }
            else if( scanningSubState == SCANING_IMG_OFF){
                //ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "ofApp::update::scannerState == SCANING_IMG_OFF");
                if(bstatePerformedActionOk){
                    scanningSubState = SCANING_LASER_ON;
                }
            }
            else if( scanningSubState == SCANING_IMG_ON){
                //ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "ofApp::update::scannerState == SCANING_IMG_ON");
                if(bstatePerformedActionOk){
                    scanningSubState = SCANING_PROCESS;
                }
            }
            else if(scanningSubState == SCANING_LASER_ON){
                //ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "ofApp::update::scannerState == SCANING_LASER_ON");
                if(bstatePerformedActionOk){
                    scanningSubState = SCANING_DELAY;
                }
            }
            else if(scanningSubState == SCANING_LASER_OFF){
                //ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "ofApp::update::scannerState == SCANING_LASER_OFF");
                scanningSubState = SCANING_CHANGE_LASER;
            }
            else if(scanningSubState == SCANING_CHANGE_LASER){
                //ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "ofApp::update::scannerState == SCANING_CHANGE_LASER");
                if(bstatePerformedActionOk){
                    if(currentLaser != NUM_LASERS-1){
                        scanningSubState = SCANING_LASER_ON;
                    }
                    else{
                        scanningSubState = SCANING_MOVE;
                    }
                }
            }
            else if(scanningSubState == SCANING_MOVE){
                //ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "ofApp::update::scannerState == SCANING_MOVE");
                if(bstatePerformedActionOk){
                    if(posAxis1Steps < fiAxis1degrees*STEPS_PER_DEGREE_AXIS1){
                        scanningSubState = SCANING_DELAY;
                    }
                    else{
                        scannerState = SCANNER_IDLE;
                        bscan = false;
                    }
                }
            }
        }

    } // end if(bscan) update state

    // PERFORM ThE STATE ACTION
    if(bscan || bscanstep){
        // ACTION
        if(scannerState == SCANNER_GOING_START){
            // SERIAL
            if(serialCommunication.bisDeviceReady){
                serialCommunication.moveStepperBySteps(recorregutToIniSteps);
                posAxis1Steps = iniAxis1degrees*STEPS_PER_DEGREE_AXIS1;
                bstatePerformedActionOk = true;
            }
        }
        else if(scannerState == SCANNER_SCANING){
            if( scanningSubState == SCANING_PROCESS){
                webcamCapture.updateGrayDiff();
                scanerProcess.camCaptureSubpixelProcess(webcamCapture.grayDiff.getPixels());
                float anglerad = ofDegToRad((float)posAxis1Steps / ((float)STEPS_PER_DEGREE_AXIS1));
                ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "ofApp::update::scannerState == SCANNER_SCANING anglerad: " + ofToString(anglerad));
                scanerProcess.Component_3D_Angular_1_axis_Scan(currentLaser, webcamCapture.colorImageNOlaser, anglerad);
                bstatePerformedActionOk = true;
            }
            else if( scanningSubState == SCANING_IMG_OFF){
                webcamCapture.bimageYESlaser = false;
                if(webcamCapture.updateColorImage() == true){
                    webcamCapture.updateGrayImage();
                    bstatePerformedActionOk = true;
                }
            }
            else if( scanningSubState == SCANING_IMG_ON){
                webcamCapture.bimageYESlaser = true;
                if(webcamCapture.updateColorImage() == true){
                    webcamCapture.updateGrayImage();
                    bstatePerformedActionOk = true;
                }
            }
            else if(scanningSubState == SCANING_LASER_ON){
                if(serialCommunication.bisDeviceReady){
                    serialCommunication.turnOnLaser(currentLaser);
                    startTimeDelayMillis = ofGetElapsedTimeMillis();
                    endDelayMillis = delayLaserms;
                    prevScanningSubState = SCANING_LASER_ON;
                    //ofSleepMillis(delayLaserms);
                    bstatePerformedActionOk = true;
                }
            }
            else if(scanningSubState == SCANING_LASER_OFF){
                if(serialCommunication.bisDeviceReady){
                    serialCommunication.turnOffLaser(currentLaser);
                    startTimeDelayMillis = ofGetElapsedTimeMillis();
                    endDelayMillis = delayLaserms;
                    prevScanningSubState = SCANING_LASER_OFF;
                    //ofSleepMillis(delayLaserms);
                    bstatePerformedActionOk = true;
                }
            }
            else if(scanningSubState == SCANING_CHANGE_LASER){
                currentLaser++;
                if(currentLaser >= NUM_LASERS){
                    currentLaser = 0;
                }
                bstatePerformedActionOk = true;
            }
            else if(scanningSubState == SCANING_MOVE){
                if(serialCommunication.bisDeviceReady){
                    if(posAxis1Steps < fiAxis1degrees*STEPS_PER_DEGREE_AXIS1){
                        serialCommunication.moveStepperBySteps(incAxis1degrees*STEPS_PER_DEGREE_AXIS1);
                        startTimeDelayMillis = ofGetElapsedTimeMillis();
                        endDelayMillis = delayStepperms;
                        prevScanningSubState = SCANING_MOVE;
                        //ofSleepMillis(delayStepperms);
                        posAxis1Steps = posAxis1Steps + incAxis1degrees*STEPS_PER_DEGREE_AXIS1;
                        ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "ofApp::update::scannerState == SCANING_MOVE incAxis1degrees: " + ofToString(incAxis1degrees));
                        ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "ofApp::update::scannerState == SCANING_MOVE posAxis1Steps: " + ofToString(posAxis1Steps));
                    }
                    bstatePerformedActionOk = true;
                }
            }
        }
    }
    else{ // no scaning
        // CAM
        webcamCapture.updateColorImage();
    }

    if(scannerState == SCANNER_CALIBRATE){
        if(scanningSubState == CALIBRATE_LASER0_DEVIATION){
            // laser right
            serialCommunication.update();
            serialCommunication.turnOnLaser(0);
            ofSleepMillis(delayLaserms);
            webcamCapture.updateColorImage();
            webcamCapture.bimageYESlaser = true;
            webcamCapture.updateGrayImage();
            serialCommunication.update();
            serialCommunication.turnOffLaser(0);
            ofSleepMillis(delayLaserms);
            webcamCapture.updateColorImage();
            webcamCapture.bimageYESlaser = false;
            webcamCapture.updateGrayImage();
            // process
            webcamCapture.updateGrayDiff();
            scanerProcess.camCaptureSubpixelProcess(webcamCapture.grayDiff.getPixels());
            scanerProcess.calibrateLaserDeviation(0);
            scanningSubState = CALIBRATE_IDLE;
        }
        if(scanningSubState == CALIBRATE_LASER1_DEVIATION){
            // laser left
            serialCommunication.update();
            serialCommunication.turnOnLaser(1);
            ofSleepMillis(delayLaserms);
            webcamCapture.updateColorImage();
            webcamCapture.bimageYESlaser = true;
            webcamCapture.updateGrayImage();
            serialCommunication.update();
            serialCommunication.turnOffLaser(1);
            ofSleepMillis(delayLaserms);
            webcamCapture.updateColorImage();
            webcamCapture.bimageYESlaser = false;
            webcamCapture.updateGrayImage();
            // process
            webcamCapture.updateGrayDiff();
            scanerProcess.camCaptureSubpixelProcess(webcamCapture.grayDiff.getPixels());
            scanerProcess.calibrateLaserDeviation(1);
            scanningSubState = CALIBRATE_IDLE;
        }

    }

    // update images to showguiOpenC3DS->autoSizeToFitWidgets();
    imgSmall1->setFromPixels(webcamCapture.colorImage.getPixels(), webcamCapture.camWidth, webcamCapture.camHeight, OF_IMAGE_COLOR);
    imgSmall2->setFromPixels(webcamCapture.grayImageYESlaser.getPixels(), webcamCapture.camWidth, webcamCapture.camHeight, OF_IMAGE_GRAYSCALE);
    imgSmall3->setFromPixels(webcamCapture.grayImageNOlaser.getPixels(), webcamCapture.camWidth, webcamCapture.camHeight, OF_IMAGE_GRAYSCALE);
    imgSmall4->setFromPixels(webcamCapture.grayDiff.getPixels(), webcamCapture.camWidth, webcamCapture.camHeight, OF_IMAGE_GRAYSCALE);
    imgSmall5->setFromPixels(webcamCapture.grayDiffTh.getPixels(), webcamCapture.camWidth, webcamCapture.camHeight, OF_IMAGE_GRAYSCALE);
    imgSmallLASER->setFromPixels(scanerProcess.imgLaserLineSubpixel.getPixels(), scanerProcess._camWidth, scanerProcess._camHeight, OF_IMAGE_COLOR);
    if(imageMain == COLOR){
        imgMainTmp->setFromPixels(webcamCapture.colorImage.getPixels(), webcamCapture.camWidth, webcamCapture.camHeight, OF_IMAGE_COLOR);
    }
    else if(imageMain == GRAY_YES_LASER){
        imgMainTmp->setFromPixels(webcamCapture.grayImageYESlaser.getPixels(), webcamCapture.camWidth, webcamCapture.camHeight, OF_IMAGE_GRAYSCALE);
    }
    else if(imageMain == GRAY_NO_LASER){
        imgMainTmp->setFromPixels(webcamCapture.grayImageNOlaser.getPixels(), webcamCapture.camWidth, webcamCapture.camHeight, OF_IMAGE_GRAYSCALE);
    }
    else if(imageMain == GRAY_DIFF){
        imgMainTmp->setFromPixels(webcamCapture.grayDiff.getPixels(), webcamCapture.camWidth, webcamCapture.camHeight, OF_IMAGE_GRAYSCALE);
    }
    else if(imageMain == GRAY_DIFF_TH){
        imgMainTmp->setFromPixels(webcamCapture.grayDiffTh.getPixels(), webcamCapture.camWidth, webcamCapture.camHeight, OF_IMAGE_GRAYSCALE);
    }
    else if(imageMain == LASER_LINE){
        imgMainTmp->setFromPixels(scanerProcess.imgLaserLineSubpixel.getPixels(), scanerProcess._camWidth, scanerProcess._camHeight, OF_IMAGE_COLOR);
    }
    imgMain->setColor(ofColor(0,0,0));
    imgMain->cropFrom(*imgMainTmp, zoomPosX + ((mouseXdiff+acumulatedDiffX)*relAspectWidth)/scale, zoomPosY + ((mouseYdiff+acumulatedDiffY)*relAspectHeight)/scale, webcamCapture.camWidth/scale, webcamCapture.camHeight/scale);

    bscanstep = false;
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofSetWindowTitle("OpenColor3DScan at " + ofToString(ofGetFrameRate()) + "fps");

    ofBackground(blauFons);

    // DEBUG
    sStateAndInfo = "FPS: " + ofToString(ofGetFrameRate()) + "\n" + "\n";
    sStateAndInfo += "SCAN\n\n";
    sStateAndInfo += "scanner state: " + ofToString(getStringStates(scannerState)) + "\n" + "\n";
    sStateAndInfo += "scanning substate: " + ofToString(getStringSubStates(scanningSubState)) + "\n" + "\n";
    sStateAndInfo += "current laser: " + ofToString(currentLaser) + "\n" + "\n";
    sStateAndInfo += "pos axis1 steps: " + ofToString(posAxis1Steps) + "\n" + "\n";
    sStateAndInfo += "PROCESS\n\n";
    sStateAndInfo += "dist_alfa: " + ofToString(scanerProcess.dist_alfa) + "\n" + "\n";
    sStateAndInfo += "Xp_: " + ofToString(scanerProcess.Xp_) + "\n" + "\n";
    sStateAndInfo += "Yp_: " + ofToString(scanerProcess.Yp_) + "\n" + "\n";
    sStateAndInfo += "m_: " + ofToString(scanerProcess.m_) + " obtained with " + ofToString(scanerProcess.mnp_) + " points\n" + "\n";
    ofxUITextArea *ta = (ofxUITextArea *) guiOpenC3DS->getWidget("states_info");
    ta->setTextString(sStateAndInfo);

    scanerProcess.draw();

    // GUI
    guiOpenC3DS->draw();
    guiOpenC3DSimages->draw();

    // calculate distances posH help
    ofDisableDepthTest();
    if(bcalcResultForPosHOk == false){
        ofNoFill();
    }
    else{
        ofFill();
    }
    ofSetColor(255,0,0);
    ofxUIRectangle* gRectImg = guiOpenC3DSimages->getRect();
    float generalImagesAreaX = gRectImg->getX(false);
    float generalImagesAreaY = gRectImg->getY(false);

    ofxUIImage* uiimg = (ofxUIImage *) guiOpenC3DSimages->getWidget("MAIN IMAGE");
    ofxUIRectangle* rectImg = uiimg->getRect();
    float imagesAreaX = rectImg->getX(false) + generalImagesAreaX;
    float imagesAreaY = rectImg->getY(false) + generalImagesAreaY;
    float posHH = posH / relAspectHeight;
    ofLine(imagesAreaX,imagesAreaY+posHH, imagesAreaX+webcamCapture.imgWidth,imagesAreaY+posHH);
    float posVV = posV / relAspectWidth;
    ofLine(imagesAreaX+posVV,imagesAreaY, imagesAreaX+posVV,imagesAreaY+webcamCapture.imgHeight);
    int diam = 4;
    ofCircle(imagesAreaX+posVV, imagesAreaY+posHH,diam,diam);

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
    else if(key == ' '){
        bscan = true;
    }
    else if(key == 'n'){
        bscanstep = true;
        scannerState = SCANNER_GOING_START;
        recorregutToIniSteps = iniAxis1degrees * STEPS_PER_DEGREE_AXIS1 - posAxis1Steps;
    }
    else if(key == 'm'){
        bscanstep = true;
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
    bmouseDragged = true;

    ofxUIRectangle* gRectImg = guiOpenC3DSimages->getRect();
    float generalImagesAreaX = gRectImg->getX(false);
    float generalImagesAreaY = gRectImg->getY(false);
    float generalImagesAreaW = gRectImg->getWidth();
    float generalImagesAreaH = gRectImg->getHeight();

    ofxUIImage *uiimg = (ofxUIImage *) guiOpenC3DSimages->getWidget("MAIN IMAGE");
    ofxUIRectangle* rectImg = uiimg->getRect();
    float imagesAreaX = rectImg->getX(false) + generalImagesAreaX;
    float imagesAreaY = rectImg->getY(false) + generalImagesAreaY;
    float imagesAreaW = rectImg->getWidth();
    float imagesAreaH = rectImg->getHeight();
    ofRectangle rectImgMain(imagesAreaX,imagesAreaY, imagesAreaW,imagesAreaH);

    if(rectImgMain.inside(x,y)){
        mouseXdiff = mousepX - x;
        mouseYdiff = mousepY - y;
    }
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

    // TODO only if its inside images
    mousepX = x;
    mousepY = y;
    bmouseDragged = false;
    mouseXdiff = mouseYdiff = 0;


    bdoubleclick = false;
    // check double click
    if(ofGetElapsedTimeMillis() - initTimeMouseReleased < delayMsAfterMouseReleasedToConsiderDoubleClick){
        bdoubleclick = true;
    }

    // check image selection
    ofxUIRectangle* gRectImg = guiOpenC3DSimages->getRect();
    float generalImagesAreaX = gRectImg->getX(false);
    float generalImagesAreaY = gRectImg->getY(false);
    float generalImagesAreaW = gRectImg->getWidth();
    float generalImagesAreaH = gRectImg->getHeight();

    ofxUIImage *uiimg = (ofxUIImage *) guiOpenC3DSimages->getWidget("color");
    ofxUIRectangle* rectImg = uiimg->getRect();
    float imagesAreaX = rectImg->getX(false) + generalImagesAreaX;
    float imagesAreaY = rectImg->getY(false) + generalImagesAreaY;
    float imagesAreaW = rectImg->getWidth();
    float imagesAreaH = rectImg->getHeight();
    ofRectangle rectImgColor(imagesAreaX,imagesAreaY, imagesAreaW,imagesAreaH);

    uiimg = (ofxUIImage *) guiOpenC3DSimages->getWidget("YESlaser");
    rectImg = uiimg->getRect();
    imagesAreaX = rectImg->getX(false) + generalImagesAreaX;
    imagesAreaY = rectImg->getY(false) + generalImagesAreaY;
    imagesAreaW = rectImg->getWidth();
    imagesAreaH = rectImg->getHeight();
    ofRectangle rectImgYeslaser(imagesAreaX,imagesAreaY, imagesAreaW,imagesAreaH);

    uiimg = (ofxUIImage *) guiOpenC3DSimages->getWidget("NOlaser");
    rectImg = uiimg->getRect();
    imagesAreaX = rectImg->getX(false) + generalImagesAreaX;
    imagesAreaY = rectImg->getY(false) + generalImagesAreaY;
    imagesAreaW = rectImg->getWidth();
    imagesAreaH = rectImg->getHeight();
    ofRectangle rectImgNolaser(imagesAreaX,imagesAreaY, imagesAreaW,imagesAreaH);

    uiimg = (ofxUIImage *) guiOpenC3DSimages->getWidget("diff");
    rectImg = uiimg->getRect();
    imagesAreaX = rectImg->getX(false) + generalImagesAreaX;
    imagesAreaY = rectImg->getY(false) + generalImagesAreaY;
    imagesAreaW = rectImg->getWidth();
    imagesAreaH = rectImg->getHeight();
    ofRectangle rectImgDiff(imagesAreaX,imagesAreaY, imagesAreaW,imagesAreaH);

    uiimg = (ofxUIImage *) guiOpenC3DSimages->getWidget("threshold");
    rectImg = uiimg->getRect();
    imagesAreaX = rectImg->getX(false) + generalImagesAreaX;
    imagesAreaY = rectImg->getY(false) + generalImagesAreaY;
    imagesAreaW = rectImg->getWidth();
    imagesAreaH = rectImg->getHeight();
    ofRectangle rectImgThreshold(imagesAreaX,imagesAreaY, imagesAreaW,imagesAreaH);

    uiimg = (ofxUIImage *) guiOpenC3DSimages->getWidget("laserLINE");
    rectImg = uiimg->getRect();
    imagesAreaX = rectImg->getX(false) + generalImagesAreaX;
    imagesAreaY = rectImg->getY(false) + generalImagesAreaY;
    imagesAreaW = rectImg->getWidth();
    imagesAreaH = rectImg->getHeight();
    ofRectangle rectImgLaserline(imagesAreaX,imagesAreaY, imagesAreaW,imagesAreaH);

    uiimg = (ofxUIImage *) guiOpenC3DSimages->getWidget("MAIN IMAGE");
    rectImg = uiimg->getRect();
    imagesAreaX = rectImg->getX(false) + generalImagesAreaX;
    imagesAreaY = rectImg->getY(false) + generalImagesAreaY;
    imagesAreaW = rectImg->getWidth();
    imagesAreaH = rectImg->getHeight();
    ofRectangle rectImgMain(imagesAreaX,imagesAreaY, imagesAreaW,imagesAreaH);

    if( (rectImgColor.inside(x,y))&&(bdoubleclick) ){
        imageMain = COLOR;
        scale = 1.0f;
        zoomPosX = 0;
        zoomPosY = 0;
        acumulatedDiffX = acumulatedDiffY = 0;
    }
    else if( (rectImgYeslaser.inside(x,y))&&(bdoubleclick) ){
        imageMain = GRAY_YES_LASER;
        scale = 1.0f;
        zoomPosX = 0;
        zoomPosY = 0;
        acumulatedDiffX = acumulatedDiffY = 0;
    }
    else if( (rectImgNolaser.inside(x,y))&&(bdoubleclick) ){
        imageMain = GRAY_NO_LASER;
        scale = 1.0f;
        zoomPosX = 0;
        zoomPosY = 0;
        acumulatedDiffX = acumulatedDiffY = 0;
    }
    else if( (rectImgDiff.inside(x,y))&&(bdoubleclick) ){
        imageMain = GRAY_DIFF;
        scale = 1.0f;
        zoomPosX = 0;
        zoomPosY = 0;
        acumulatedDiffX = acumulatedDiffY = 0;
    }
    else if( (rectImgThreshold.inside(x,y))&&(bdoubleclick) ){
        imageMain = GRAY_DIFF_TH;
        scale = 1.0f;
        zoomPosX = 0;
        zoomPosY = 0;
        acumulatedDiffX = acumulatedDiffY = 0;
    }
    else if( (rectImgLaserline.inside(x,y))&&(bdoubleclick) ){
        imageMain = LASER_LINE;
        scale = 1.0f;
        zoomPosX = 0;
        zoomPosY = 0;
        acumulatedDiffX = acumulatedDiffY = 0;
    }
    else if( (rectImgMain.inside(x,y))&&(bdoubleclick) ){
        // zoom image
        bzoom = true;
        zoomPosX = (x - imagesAreaX);
        zoomPosY = (y - imagesAreaY);
        zoomPosX = zoomPosX * scale;
        zoomPosY = zoomPosY * scale;
        //zoomPosX = zoomPosX - (webcamCapture.imgWidth/2);
        //zoomPosY = zoomPosY - (webcamCapture.imgHeight/2);
        scale = scale * 2.0f;
        //zoomPosX = zoomPosX * relAspectWidth;
        //zoomPosY = zoomPosY * relAspectHeight;
        acumulatedDiffX = acumulatedDiffY = 0;
    }
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    initTimeMouseReleased = ofGetElapsedTimeMillis();

    ofxUIRectangle* gRectImg = guiOpenC3DSimages->getRect();
    float generalImagesAreaX = gRectImg->getX(false);
    float generalImagesAreaY = gRectImg->getY(false);
    float generalImagesAreaW = gRectImg->getWidth();
    float generalImagesAreaH = gRectImg->getHeight();

    ofxUIImage *uiimg = (ofxUIImage *) guiOpenC3DSimages->getWidget("MAIN IMAGE");
    ofxUIRectangle* rectImg = uiimg->getRect();
    float imagesAreaX = rectImg->getX(false) + generalImagesAreaX;
    float imagesAreaY = rectImg->getY(false) + generalImagesAreaY;
    float imagesAreaW = rectImg->getWidth();
    float imagesAreaH = rectImg->getHeight();
    ofRectangle rectImgMain(imagesAreaX,imagesAreaY, imagesAreaW,imagesAreaH);

    if( (rectImgMain.inside(x,y))&&(bmouseDragged) ){
        // drag image
        mouseXdiff = mousepX - x;
        mouseYdiff = mousepY - y;
        acumulatedDiffX += mouseXdiff;
        acumulatedDiffY += mouseYdiff;
        mouseXdiff = mouseYdiff = 0;
    }
    bmouseDragged = false;
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
	else if(name == "calibrate_deviation_laser0"){
        scannerState = SCANNER_CALIBRATE;
        scanningSubState = CALIBRATE_LASER0_DEVIATION;
	}
	else if(name == "calibrate_deviation_laser1"){
        scannerState = SCANNER_CALIBRATE;
        scanningSubState = CALIBRATE_LASER1_DEVIATION;
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
	else if(name == "image_diff_and_th"){
        webcamCapture.updateGrayDiff();
	}
	else if(name == "process_img"){
        scanerProcess.camCaptureSubpixelProcess(webcamCapture.grayDiff.getPixels());
	}
	else if(name == "calc_dist_laser1"){
        // laser left
        serialCommunication.update();
        serialCommunication.turnOnLaser(1);
        ofSleepMillis(delayLaserms);
        webcamCapture.updateColorImage();
        webcamCapture.bimageYESlaser = true;
        webcamCapture.updateGrayImage();
        serialCommunication.update();
        serialCommunication.turnOffLaser(1);
        ofSleepMillis(delayLaserms);
        webcamCapture.updateColorImage();
        webcamCapture.bimageYESlaser = false;
        webcamCapture.updateGrayImage();
        // process
        webcamCapture.updateGrayDiff();
        bcalcResultForPosHOk = false;
        scanerProcess.camCaptureSubpixelProcess(webcamCapture.grayDiff.getPixels());
        do{
            bcalcResultForPosHOk = scanerProcess.calculateDistances(posH, 1);
            posH++;
        } while( (bcalcResultForPosHOk == false)&&(posH < webcamCapture.camHeight) );
	}
	else if(name == "calc_dist_laser0"){
        // laser right
        serialCommunication.update();
        serialCommunication.turnOnLaser(0);
        ofSleepMillis(delayLaserms);
        webcamCapture.updateColorImage();
        webcamCapture.bimageYESlaser = true;
        webcamCapture.updateGrayImage();
        serialCommunication.update();
        serialCommunication.turnOffLaser(0);
        ofSleepMillis(delayLaserms);
        webcamCapture.updateColorImage();
        webcamCapture.bimageYESlaser = false;
        webcamCapture.updateGrayImage();
        // process
        webcamCapture.updateGrayDiff();
        bcalcResultForPosHOk = false;
        scanerProcess.camCaptureSubpixelProcess(webcamCapture.grayDiff.getPixels());
        do{
            bcalcResultForPosHOk = scanerProcess.calculateDistances(posH, 0);
            posH++;
        } while( (bcalcResultForPosHOk == false)&&(posH < webcamCapture.camHeight) );
	}
}

//--------------------------------------------------------------
string ofApp::getStringStates(int s){
    if(s == SCANNER_IDLE){
        return "SCANNER_IDLE";
    }
    else if(s == SCANNER_GOING_HOME){
        return "SCANNER_GOING_HOME";
    }
    else if(s == SCANNER_GOING_START){
        return "SCANNER_GOING_START";
    }
    else if(s == SCANNER_SCANING){
        return "SCANNER_SCANING";
    }
    else if(s == SCANNER_CALIBRATE){
        return "SCANNER_CALIBRATE";
    }
    else{
        return "";
    }
}

//--------------------------------------------------------------
string ofApp::getStringSubStates(int s){
    if(s == SCANING_IDLE){
        return "SCANING_IDLE";
    }
    else if(s == SCANING_DELAY){
        return "SCANING_DELAY";
    }
    else if(s == SCANING_PROCESS){
        return "SCANING_PROCESS";
    }
    else if(s == SCANING_CHANGE_LASER){
        return "SCANING_CHANGE_LASER";
    }
    else if(s == SCANING_MOVE){
        return "SCANING_MOVE";
    }
    else if(s == SCANING_LASER_ON){
        return "SCANING_LASER_ON";
    }
    else if(s == SCANING_LASER_OFF){
        return "SCANING_LASER_OFF";
    }
    else if(s == SCANING_IMG_ON){
        return "SCANING_IMG_ON";
    }
    else if(s == SCANING_IMG_OFF){
        return "SCANING_IMG_OFF";
    }
    else if(s == CALIBRATE_IDLE){
        return "CALIBRATE_IDLE";
    }
    else if(s == CALIBRATE_LASER0_DEVIATION){
        return "CALIBRATE_LASER0_DEVIATION";
    }
    else if(s == CALIBRATE_LASER1_DEVIATION){
        return "CALIBRATE_LASER1_DEVIATION";
    }
    else{
        return "";
    }
}
