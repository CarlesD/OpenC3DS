#include "openC3DSprocess.h"

//--------------------------------------------------------------
void openC3DSprocess::setup(){

    // XML
    xmlSettings.loadFile("settingsProcess.xml");

    L = xmlSettings.getValue("OPENC3DS:L", 270); // mm
	cout << "OPENC3DS:L = " << L << endl;

    xmlSettings.pushTag("OPENC3DS");
	numLasers = xmlSettings.getNumTags("LASER");
	cout << "OPENC3DS:numLasers = " << numLasers << endl;
	for(int i=0; i<numLasers; i++){
        xmlSettings.pushTag("LASER", i);
        cout << "OPENC3DS:LASER = " << i << endl;
        beta[i] = xmlSettings.getValue("beta", 0.959931);
        cout << "OPENC3DS:beta = " << beta[i] << endl;
        xc[i] = xmlSettings.getValue("xc", 0);
        cout << "OPENC3DS:xc = " << xc[i] << endl;
        yc[i] = xmlSettings.getValue("yc", 8.08);
        cout << "OPENC3DS:yc = " << yc[i] << endl;
        LA[i] = xmlSettings.getValue("LA", 115.989998);
        cout << "OPENC3DS:LA = " << LA[i] << endl;
        LB[i] = xmlSettings.getValue("LB", 49.529999);
        cout << "OPENC3DS:LB = " << LB[i] << endl;
        alfa[i] = xmlSettings.getValue("alfa", 19.4);
        cout << "OPENC3DS:alfa = " << alfa[i] << endl;
        zita[i] = xmlSettings.getValue("zita", 1.204277);
        cout << "OPENC3DS:zita = " << zita[i] << endl;
        m[i] = xmlSettings.getValue("m", 37.27);
        cout << "OPENC3DS:m = " << m[i] << endl;
        FCC[i] = xmlSettings.getValue("FCC", 5.665022);
        cout << "OPENC3DS:FCC = " << FCC[i] << endl;
        laserID[i] = xmlSettings.getValue("laserID", 0); // 0 esquerra
        cout << "OPENC3DS:laserID = " << laserID[i] << endl;

        xmlSettings.popTag();
    }
    xmlSettings.popTag();

	// GUI
	setGuiProcess();
	guiProcess->loadSettings("guiProcess.xml");

	// 3D MESH
	mesh.setMode(OF_PRIMITIVE_POINTS);
	mesh.enableColors();
	mesh.enableIndices();

	ofEnableDepthTest();
	glEnable(GL_POINT_SMOOTH); // use circular points instead of square points
	glPointSize(1); // make the points bigger

	ofIcoSpherePrimitive prmtv;
    prmtv.setPosition(0, 0, 0);
    prmtv.setResolution(2);
    prmtv.setRadius(3);
    prmtv.setMode( OF_PRIMITIVE_TRIANGLES );
    vector <ofMeshFace> triangles = prmtv.getMesh().getUniqueFaces();
    prmtv.getMesh().setFromTriangles(triangles, true);

    origin = prmtv.getMesh();
    origin.enableColors();
    origin.enableIndices();

	// LIGHT
	light.setup();
    light.setAmbientColor(ofColor(100));

    // POINT CLOUD
    points3Dscanned.clear();
    resetPointCloud();

}

//--------------------------------------------------------------
void openC3DSprocess::setupCamResolution(int w, int h){
    _camWidth = w;
	_camHeight = h;

	imgLaserLineSubpixel.allocate(_camWidth, _camHeight, OF_IMAGE_COLOR);
	imgLaserLineSubpixel.setColor(ofColor(0,0,0));

    // PIXELS RAW color
//    for(int j=0; j<_camWidth; j+=3){
//        for(int i=0; i<_camHeight; i+=3){
//            int index = i*_camWidth + j;
//            cout << index << endl;
//            colorPixelsRaw[index] = 0;
//            colorPixelsRaw[index+1] = 0;
//            colorPixelsRaw[index+2] = 0;
//        }
//	}

    // DEBUG
    colorPixelsRaw.allocate(_camWidth, _camHeight);
    // end
}

//--------------------------------------------------------------
bool openC3DSprocess::calculateDistances(float posH, int laser){
     int posHint = (int)posH;
     cout << "posHint: "<< posHint << endl;
     cout << "laserLineSubpixelPoints[posHint].y: "<< laserLineSubpixelPoints[posHint].y << endl;
     cout << "laserLineSubpixelPoints[posHint].x: "<< laserLineSubpixelPoints[posHint].x << endl;
     cout << "_camHeight: "<< _camHeight << endl;
     delta_alfa = (PI/180.0f) * alfa[laser] * (-1+((float)posHint/(float)(_camHeight/2.0f)));
     cout << "laser: " << laser << endl;
     cout << "alfa[laser]: " << alfa[laser] << endl;
     cout << "delta_alfa: " << delta_alfa << endl;
     cam_dis(laser, laserLineSubpixelPoints[posHint].x, laserLineSubpixelPoints[posHint].y, &Xp_,&Yp_);
     cout << "Xp_: " << Xp_ << " Yp_: " << Yp_ << endl;
     dist_alfa = sqrt(Xp_*Xp_+Yp_*Yp_)/cos(delta_alfa);
     cout << "calculateDistances dist_alfa: "<< dist_alfa << endl;
}

//--------------------------------------------------------------
bool openC3DSprocess::camCaptureSubpixelProcess(unsigned char* pixelsRaw){

    int horizPosMaxIntensityTemp1, horizPosMaxIntensityTemp2;
    float horizPosMaxIntensity;

    double x[2*(int)PAP];
    double grisVal[2*(int)PAP];
    int maxGrayLevelTemp;

    int kIndexGreyFunction;
    bool bcontrol;

    double coeff[DEGREE];

    laserLineSubpixelPoints.clear();
    for(int i=0; i<_camHeight; ++i){
        // init laserLineSubpixelPoints
        points2DSubpixelPrecision point2d;
        point2d.x = IMPOSSIBLE_NUMBER;
        point2d.y = 0;
        point2d.q = 0;
        point2d.a = 0;
        laserLineSubpixelPoints.push_back(point2d);
    }

    imgLaserLineSubpixel.setColor(ofColor(0,0,0)); // reset

    // IMAGE PROCESSING TO FIND MAX
    for(int i=0; i<_camHeight; ++i){

        maxGrayLevelTemp = 0;
        horizPosMaxIntensityTemp1 = -1;
        horizPosMaxIntensityTemp2 = -1;
        for(int j=0; j<_camWidth; ++j){
            if( (pixelsRaw[i*_camWidth+j] > GPLL) && (pixelsRaw[i*_camWidth+j] > maxGrayLevelTemp) ){
                maxGrayLevelTemp = pixelsRaw[i*_camWidth+j];
                horizPosMaxIntensityTemp2 = horizPosMaxIntensityTemp1;
                horizPosMaxIntensityTemp1 = j; // finding the max and the second max
            }
        }

        if(horizPosMaxIntensityTemp2 != -1){
            // chose the point in the middle of two temp maxs
            horizPosMaxIntensityTemp1 = ( (horizPosMaxIntensityTemp1 - horizPosMaxIntensityTemp2) * 0.5 ) + horizPosMaxIntensityTemp2;
        }

        int limInf = horizPosMaxIntensityTemp1-PAP;
        int limitSup = horizPosMaxIntensityTemp1+PAP;
        kIndexGreyFunction = 0;
        bcontrol = false;
        for(int j=limInf; j<limitSup; j++){
            if( (j > 0) && (j <_camWidth) ){ // check image bounds
                if(pixelsRaw[i*_camWidth + j] > GPLL){ // filter low levels of grey (noise)
                    x[kIndexGreyFunction] = (double)j; // j is the horizontal position of the pixel inside the image
                    grisVal[kIndexGreyFunction] = (double)pixelsRaw[i*_camWidth+j];
                    bcontrol = true;
                    kIndexGreyFunction = kIndexGreyFunction + 1;
                }
            }
        }

        horizPosMaxIntensity = 0;
        if(kIndexGreyFunction >= PMP){
            polynomialfit(kIndexGreyFunction, DEGREE, x, grisVal, coeff);

            if(coeff[2] != 0){
                horizPosMaxIntensity = fabs( (-1 * coeff[1]) / (2 * coeff[2]) );
                if(fabs(horizPosMaxIntensity-horizPosMaxIntensityTemp1 )> 10){ // TODO: posar el 10 en un defiine d'error max
                    horizPosMaxIntensity = IMPOSSIBLE_NUMBER;
                }
            }
        }
        else{
            horizPosMaxIntensity = IMPOSSIBLE_NUMBER;
        }

        cout << "openC3DSprocess::camCaptureSubpixelProcess bcontrol at " << i << " = " << bcontrol << endl;

        if(bcontrol == true){
            // fill the laserLineSubpixelPoints
            if(horizPosMaxIntensity > 0 && horizPosMaxIntensity < _camWidth && kIndexGreyFunction >= PMP){
                laserLineSubpixelPoints[i].x = horizPosMaxIntensity;
                laserLineSubpixelPoints[i].y = i;
                laserLineSubpixelPoints[i].a = kIndexGreyFunction;
                laserLineSubpixelPoints[i].q = kIndexGreyFunction;
                imgLaserLineSubpixel.setColor(horizPosMaxIntensity,i,ofColor(255,0,0));
            }
            else{
                //imgLaserLineSubpixel.setColor(horizPosMaxIntensity,i,ofColor(0,0,0));
            }
        }

    }
    imgLaserLineSubpixel.update();

    colorPixelsRaw.setFromPixels((const unsigned char *)pixelsRaw, _camWidth, _camHeight);
    return true;
}

//--------------------------------------------------------------
bool openC3DSprocess::Component_3D_Angular_1_axis_Scan(int currentLaser, ofxCvColorImage pixelsRaw, float phi){
    float Xp, Yp;

    // DEBUG
    colorPixelsRaw = pixelsRaw;
    // end

    dist_alfa = MAX_RADIUS;
    for(int i=0; i<_camHeight; i++){
        points3D point3d;
        if(laserLineSubpixelPoints[i].x != IMPOSSIBLE_NUMBER){
            delta_alfa = (PI/180.0f) * alfa[currentLaser] * (-1+((float)i/(float)(_camHeight/2.0f)));
            cam_dis(currentLaser, laserLineSubpixelPoints[i].x, laserLineSubpixelPoints[i].y, &Xp,&Yp);
            dist_alfa = sqrt(Xp*Xp+Yp*Yp)/cos(delta_alfa);

            if(dist_alfa < MAX_RADIUS){
                //float migalcada = floor(_camHeight*0.5);
                //if(i > migalcada -10 && i < migalcada + 10){
                    int index = laserLineSubpixelPoints[i].y *_camWidth + laserLineSubpixelPoints[i].x;

                    indexPixColorX = (int)laserLineSubpixelPoints[i].x;
                    indexPixColorY = (int)laserLineSubpixelPoints[i].y;

                    ofPixels pix = pixelsRaw.getPixelsRef();
                    ofColor colorPix = pix.getColor(indexPixColorX, indexPixColorY);

                    point3d.r = colorPix.r;
                    point3d.g = colorPix.g;
                    point3d.b = colorPix.b;
                    //point3d.r = point3d.r / 255.0; // color unitary (between 0-1)
                    //point3d.g = point3d.g / 255.0;
                    //point3d.b = point3d.b / 255.0;
                    point3d.q = laserLineSubpixelPoints[i].q;

                    ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "openC3DSprocess::Component_3D_Angular_1_axis_Scan");
                    ofLog(OF_LOG_NOTICE, "pixel at: " + ofToString(indexPixColorX) + ", " + ofToString(indexPixColorY) + " color R:" + ofToString(point3d.r) + " G:" + ofToString(point3d.g) + " B:" + ofToString(point3d.b));

                    point3d.x = Xp * cos(phi) + (L + yc[currentLaser] - Yp) * sin(phi);
                    point3d.y = -Xp * sin(phi) + (L + yc[currentLaser] - Yp) * cos(phi);
                    point3d.z = dist_alfa * sin(delta_alfa);
                //}
            }
            else{ // if(dist_alfa < 1000)
                ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "openC3DSprocess::Component_3D_Angular_1_axis_Scan:ERROR:dist_alfa >= 1000");

                point3d.x = IMPOSSIBLE_NUMBER;
                point3d.y = 0;
                point3d.z = 0;
                point3d.r = 255;
                point3d.g = 0;
                point3d.b = 0;
            }

            if(i == int(_camHeight*0.4)){
                ofLog(OF_LOG_ERROR, "openC3DSprocess::Component_3D_Angular_1_axis_Scan");
                ofLog(OF_LOG_ERROR, " currentLaser: " + ofToString(currentLaser));
                ofLog(OF_LOG_ERROR, " Xp: " + ofToString(Xp) + " Yp: " + ofToString(Yp));
                ofLog(OF_LOG_ERROR, " dist_alfa: " + ofToString(dist_alfa));
                ofLog(OF_LOG_ERROR, " phi(º): " + ofToString(ofRadToDeg(phi)));
                ofLog(OF_LOG_ERROR, " delta_alfa: " + ofToString(delta_alfa));
                ofLog(OF_LOG_ERROR, " point3d.x: " + ofToString(point3d.x) + " point3d.y: " + ofToString(point3d.y) + " point3d.z: " + ofToString(point3d.z));
            }

        } // end if(laserLineSubpixelPoints[i].x != IMPOSSIBLE_NUMBER)
        else{
            point3d.x = IMPOSSIBLE_NUMBER;
            point3d.y = 0;
            point3d.z = 0;
            point3d.r = 255;
            point3d.g = 0;
            point3d.b = 0;
        }

        // points 3d
        points3Dscanned.push_back(point3d);

        // mesh
        ofColor c;
        c.set(point3d.r, point3d.g, point3d.b);
        mesh.addColor(c);
		ofVec3f pos(point3d.x, point3d.y, point3d.z);
        mesh.addVertex(pos);

    } // end for
    return true;
}

//--------------------------------------------------------------
void openC3DSprocess::cam_dis(int currentLaser, float x, int yp, float *XXp, float *YYp){

       if(laserID[currentLaser] == 0){

        float x_corregida = (FCC[currentLaser]-((float)yp/m[currentLaser])+x);
        float xerr = 0; // TODO
        float partTgDenom1 = tan(beta[currentLaser]) * tan(  0.5*zita[currentLaser] - (x_corregida*zita[currentLaser]) / (_camWidth-1) );


        *YYp = ( (-LA[currentLaser] - LB[currentLaser]*sin(beta[currentLaser]) + xerr) * tan(beta[currentLaser]) ) - LB[currentLaser]*cos(beta[currentLaser]) - yc[currentLaser];
        *YYp = *YYp / (-1.0 - partTgDenom1);
        *XXp = *YYp*tan( (0.5*zita[currentLaser]) -(x_corregida*zita[currentLaser]) / (_camWidth-1) )*(-1);
       // ofLog(OF_LOG_ERROR, " YYP: " + ofToString(*YYp);
    }

    else if(laserID[currentLaser] == 1){

        float x_corregida = (FCC[currentLaser]-((float)yp/m[currentLaser])+x);
        float xerr = 0; // TODO
        float partTgDenom2 = tan(beta[currentLaser]) * tan(  0.5*zita[currentLaser] - (x_corregida*zita[currentLaser]) / (_camWidth-1) );

        *YYp = ( (-LA[currentLaser] - LB[currentLaser]*sin(beta[currentLaser]) - xerr) * tan(beta[currentLaser]) ) - LB[currentLaser]*cos(beta[currentLaser]) - yc[currentLaser];
        *YYp = *YYp / (-1.0 + partTgDenom2);
        *XXp = *YYp*tan( (0.5*zita[currentLaser]) -(x_corregida*zita[currentLaser]) / (_camWidth-1) )*(-1);
    }

}


//--------------------------------------------------------------
bool openC3DSprocess::update(){

}

//--------------------------------------------------------------
void openC3DSprocess::draw(){
    ofSetColor(255);

    ofEnableLighting();
    ofEnableDepthTest();
    light.setPosition(0, 0, 0);
    light.draw();
    light.enable();

    cam.begin();
    origin.drawWireframe();
	mesh.draw();
	cam.end();

	// DEBUG
	ofDisableDepthTest();
	colorPixelsRaw.draw(20,20,_camWidth*0.3, _camHeight*0.3);
	//ofNoFill();
	//ofSetColor(255,0,0,255);
	//ofCircle(20+indexPixColorX*0.3, 20+indexPixColorY*0.3, 7);
	// end

}

//--------------------------------------------------------------
void openC3DSprocess::exit(){
    xmlSettings.clear();
    xmlSettings.addTag("OPENC3DS");
    xmlSettings.pushTag("OPENC3DS", 0);
    xmlSettings.setValue("L", L);

    for(int i=0; i<numLasers; i++){
        xmlSettings.addTag("LASER");
        xmlSettings.pushTag("LASER", i);

        xmlSettings.setValue("beta", beta[i]);
        xmlSettings.setValue("xc", xc[i]);
        xmlSettings.setValue("yc", yc[i]);
        xmlSettings.setValue("LA", LA[i]);
        xmlSettings.setValue("LB", LB[i]);
        xmlSettings.setValue("alfa", alfa[i]);
        xmlSettings.setValue("zita", zita[i]);
        xmlSettings.setValue("m", m[i]);
        xmlSettings.setValue("FCC", FCC[i]);
        xmlSettings.setValue("laserID", laserID[i]);

        xmlSettings.popTag();
    }
    xmlSettings.saveFile("settingsProcess.xml");

    guiProcess->saveSettings("guiProcess.xml");
	delete guiProcess;
}

//--------------------------------------------------------------
void openC3DSprocess::setGuiProcess(){

	guiProcess = new ofxUISuperCanvas("PROCESSING");
    guiProcess->addSpacer();

    guiProcess->addLabel("PROCESS PARAMETERS", OFX_UI_FONT_MEDIUM);
    guiProcess->addSlider("GPLL", 0, 255, &GPLL)->setIncrement(1);
    guiProcess->addSlider("PAP", 3, 100, &PAP)->setIncrement(1);
    guiProcess->addSlider("PMP", 3, 33, &PMP)->setIncrement(1);

    guiProcess->addLabel("POINT CLOUD", OFX_UI_FONT_MEDIUM);
    PCDfilename = guiProcess->addTextInput("filename", "scan.pcd");
    PCDfilename->setAutoUnfocus(false);
    PCDfilename->setAutoClear(false);

    guiProcess->addLabel("CALIBRATE", OFX_UI_FONT_MEDIUM);
    float *ptr = FCC;
    guiProcess->addSlider("FCC_laser0", -50, 50, ptr)->setIncrement(1);
    guiProcess->addSlider("FCC_laser1", -50, 50, ptr+1)->setIncrement(1);
    guiProcess->addButton("cal_Yp_laser0", false);
    guiProcess->addButton("cal_Yp_laser1", false);
    guiProcess->addSlider("posH_calc_Yp", 0, 720, &posHcalibrationPoint)->setIncrement(1);

    guiProcess->addButton("save point cloud", false);
    guiProcess->addButton("reset point cloud", false);

    guiProcess->setPosition(0,0);
    guiProcess->autoSizeToFitWidgets();
	ofAddListener(guiProcess->newGUIEvent,this,&openC3DSprocess::guiEvent);
}

//--------------------------------------------------------------
void openC3DSprocess::guiEvent(ofxUIEventArgs &e){
	string name = e.getName();
	int kind = e.getKind();

	if(name == "filename"){
        ofxUITextInput *PCDfilename = (ofxUITextInput *) e.widget;
    }
    else if(name == "save point cloud"){
        fillPointCloud();
        savePointCloud();
    }
    else if(name == "reset point cloud"){
        resetPointCloud();
    }
    else if(name == "cal_Yp_laser0"){
        calculateDistances(posHcalibrationPoint,0);
    }
    else if(name == "cal_Yp_laser1"){
        calculateDistances(posHcalibrationPoint,1);
    }
}

//--------------------------------------------------------------
void openC3DSprocess::fillPointCloud(){
    // http://pointclouds.org/documentation/tutorials/writing_pcd.php
    cloud.width = points3Dscanned.size();
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width*cloud.height);

    for(int i=0; i<cloud.points.size(); i++){
        cloud.points[i].x = points3Dscanned[i].x;
        cloud.points[i].y = points3Dscanned[i].y;
        cloud.points[i].z = points3Dscanned[i].z;

        cloud.points[i].normal_x = points3Dscanned[i].nx;
        cloud.points[i].normal_y = points3Dscanned[i].ny;
        cloud.points[i].normal_z = points3Dscanned[i].nz;

        cloud.points[i].r = points3Dscanned[i].r;
        cloud.points[i].g = points3Dscanned[i].g;
        cloud.points[i].b = points3Dscanned[i].b;
    }
}

//--------------------------------------------------------------
void openC3DSprocess::savePointCloud(){

    if(cloud.points.size() > 0){
        pcl::io::savePCDFileASCII(ofToDataPath("test.pcd"), cloud);
        ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "openC3DSprocess::savePointCloud:saved " + ofToString(cloud.points.size()) + "to " + "test.pcd file");
    }
}

//--------------------------------------------------------------
void openC3DSprocess::resetPointCloud(){
    cloud.points.resize(0);
}

//--------------------------------------------------------------
bool openC3DSprocess::polynomialfit(int obs, int degree, double *dx, double *dy, double *store){ /* n, p */
  gsl_multifit_linear_workspace *ws;
  gsl_matrix *cov, *X;
  gsl_vector *y, *c;
  double chisq;

  X = gsl_matrix_alloc(obs, degree);
  y = gsl_vector_alloc(obs);
  c = gsl_vector_alloc(degree);
  cov = gsl_matrix_alloc(degree, degree);

  for(int i=0; i<obs; i++){
    gsl_matrix_set(X, i, 0, 1.0);
    for(int j=0; j<degree; j++){
        gsl_matrix_set(X, i, j, pow(dx[i], j));
    }
    gsl_vector_set(y, i, dy[i]);
  }

  ws = gsl_multifit_linear_alloc(obs, degree);
  gsl_multifit_linear(X, y, c, cov, &chisq, ws);

  /* store result ... */
  for(int i=0; i< degree; i++){
    store[i] = gsl_vector_get(c, i);
  }

  gsl_multifit_linear_free(ws);
  gsl_matrix_free(X);
  gsl_matrix_free(cov);
  gsl_vector_free(y);
  gsl_vector_free(c);

  return true; /* we do not "analyse" the result (cov matrix mainly)
                to know if the fit is "good" */
}


//--------------------------------------------------------------
bool openC3DSprocess::calibrateLaserDeviation(int currentLaser){
    double scx, scxx, scy, scxy;
    scx = scxx = scy = scxy = 0.0;
    int np = 0; // number of points really used for the calibration

    cout << "openC3DSprocess::calibrateLaserDeviation laser " << currentLaser << endl;

    for(int i=0; i<=_camHeight; i++){
        if(laserLineSubpixelPoints[i].x != IMPOSSIBLE_NUMBER){
            scx += (double)laserLineSubpixelPoints[i].x;
            scy += (double)laserLineSubpixelPoints[i].y;
            scxy += (double)laserLineSubpixelPoints[i].x*(double)laserLineSubpixelPoints[i].y;
            scxx += (double)laserLineSubpixelPoints[i].x*(double)laserLineSubpixelPoints[i].x;
            np = np+1; //nombre total de punts bons de la regressió
        }
    }

    double mean_cx = scx / (double)np;
    double mean_cy = scy / (double)np;

    double varcx = scxx - scx * mean_cx;
    double covc = scxy - scx * mean_cy;

    cout << "varcx: " << varcx << " covc: " << covc << endl;

    // check for zero varx
    m[currentLaser] = covc / varcx; //càlcul pendent recta
    cout << "pendent m: " << m[currentLaser] << endl;
    cout << "with np points: " << np << endl;
}
