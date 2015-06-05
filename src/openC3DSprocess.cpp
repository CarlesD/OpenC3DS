#include "openC3DSprocess.h"

//--------------------------------------------------------------
void openC3DSprocess::setup(){

    // XML
    xmlSettings.loadFile("settingsProcess.xml");

    L = xmlSettings.getValue("OPENC3DS:L", 270); // mm
	cout << "OPENC3DS:L = " << L << endl;

    xmlSettings.pushTag("OPENC3DS");
	numLasers = xmlSettings.getNumTags("LASER");
	for(int i=0; i<numLasers; i++){
        xmlSettings.pushTag("LASER", i);

        beta[i] = xmlSettings.getValue("beta", 0.959931);
        cout << "OPENC3DS:beta = " << beta[i] << endl;
        xc[i] = xmlSettings.getValue("xc", 0);
        cout << "OPENC3DS:xc = " << xc[i] << endl;
        yc[i] = xmlSettings.getValue("yc", 8.08);
        cout << "OPENC3DS:yc = " << yc[i] << endl;
        LA[i] = xmlSettings.getValue("LA", 49.529999);
        cout << "OPENC3DS:LA = " << LA[i] << endl;
        LB[i] = xmlSettings.getValue("LB", 115.989998);
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

	// GUI
	setGuiProcess();
	guiProcess->loadSettings("guiProcess.xml");

	// 3D MESH
	mesh.enableColors();
	mesh.enableIndices();
	mesh.setMode(OF_PRIMITIVE_POINTS);

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
    light.setAmbientColor(ofColor(255));

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

    for(int i=0; i<_camHeight; ++i){
        // init laserLineSubpixelPoints
        points2DSubpixelPrecision point2d;
        point2d.x = _camWidth;
        point2d.y = 0;
        point2d.q = 0;
        point2d.a = 0;
        laserLineSubpixelPoints.push_back(point2d);
    }

    imgLaserLineSubpixel.setColor(ofColor(0,0,0));

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
                if(fabs(horizPosMaxIntensity-horizPosMaxIntensityTemp1 )> 10){
                    horizPosMaxIntensity = _camWidth;
                }
            }
        }
        else{
            horizPosMaxIntensity = _camWidth;
        }

        if(bcontrol == true){
            // fill the laserLineSubpixelPoints
            if(horizPosMaxIntensity > 0 && horizPosMaxIntensity < _camWidth && kIndexGreyFunction >= PMP){
                laserLineSubpixelPoints[i].x = horizPosMaxIntensity;
                laserLineSubpixelPoints[i].y = i;
                laserLineSubpixelPoints[i].a = kIndexGreyFunction;
                laserLineSubpixelPoints[i].q = kIndexGreyFunction;
                imgLaserLineSubpixel.setColor(horizPosMaxIntensity,i,ofColor(255,0,0));
            }
        }

    }
    imgLaserLineSubpixel.update();
    return true;
}

//--------------------------------------------------------------
bool openC3DSprocess::Component_3D_Angular_1_axis_Scan(int currentLaser, unsigned char* pixelsRaw, float phi){

    float delta_alfa, dist_alfa;
    float Xp, Yp;
    colorPixelsRaw = pixelsRaw;

    for(int i=0; i<_camHeight; i++){
        points3D point3d;
        if(laserLineSubpixelPoints[i].x != _camWidth){
            delta_alfa = (PI/180.0f) * alfa[currentLaser] * (-1+((float)i/(float)(_camHeight/2.0f)));
            cam_dis(currentLaser, laserLineSubpixelPoints[i].x, laserLineSubpixelPoints[i].y, &Xp,&Yp);
            dist_alfa = sqrt(Xp*Xp+Yp*Yp)/cos(delta_alfa);

            if (dist_alfa < 1000){
                int index = laserLineSubpixelPoints[i].y *_camWidth + laserLineSubpixelPoints[i].x;
                point3d.r = pixelsRaw[index];
                point3d.g = pixelsRaw[index+1];
                point3d.b = pixelsRaw[index+2];
                point3d.q = laserLineSubpixelPoints[i].q;

                point3d.x = -Xp * cos(phi) - (L + yc[currentLaser] - Yp) * sin(phi);
                point3d.y = Xp * sin(phi) - (L + yc[currentLaser] - Yp) * cos(phi);
                point3d.z = dist_alfa * sin(delta_alfa);
            }
            else{
                point3d.x = -10000;
                point3d.y = -10000;
                point3d.z = -10000;
                point3d.r = 255;
                point3d.g = 0;
                point3d.b = 0;
            }

            if(i == int(_camHeight*0.4)){
                ofLog(OF_LOG_VERBOSE, "openC3DSprocess::Component_3D_Angular_1_axis_Scan");
                ofLog(OF_LOG_VERBOSE, " currentLaser: " + ofToString(currentLaser));
                ofLog(OF_LOG_VERBOSE, " Xp: " + ofToString(Xp) + " Yp: " + ofToString(Yp));
                ofLog(OF_LOG_VERBOSE, " dist_alfa: " + ofToString(dist_alfa));
                ofLog(OF_LOG_VERBOSE, " phi(º): " + ofToString(ofRadToDeg(phi)));
                ofLog(OF_LOG_VERBOSE, " delta_alfa: " + ofToString(delta_alfa));
                ofLog(OF_LOG_VERBOSE, " point3d.x: " + ofToString(point3d.x) + " point3d.y: " + ofToString(point3d.y) + " point3d.z: " + ofToString(point3d.z));
            }

        } // end if(laserLineSubpixelPoints[i].x != 1024)
        else{
            point3d.x = -10000;
            point3d.y = -10000;
            point3d.z = -10000;
            point3d.r = 255;
            point3d.g = 0;
            point3d.b = 0;
        }

        // points 3d
        points3Dscanned.push_back(point3d);
        // mesh
        ofFloatColor c;
        c.set(point3d.r/255.0, point3d.g/255.0, point3d.b/255.0);
        //cout << "point3d color: " << point3d.r << ", " << point3d.g << ", " << point3d.b << endl;
        //cout << "c " << c << endl;
        mesh.addColor(c);
		ofVec3f pos(point3d.x, point3d.y, point3d.z);
        mesh.addVertex(pos);

    } // end for
    return true;
}

//--------------------------------------------------------------
void openC3DSprocess::cam_dis(int currentLaser, float x, int yp, float *XXp, float *YYp){

    float d;
    int xp;

    xp = (int)x;

    if(laserID[currentLaser] == 1){

        float x_corregida=(FCC[currentLaser]-(float)yp/m[currentLaser]+x);

        *YYp = (LB[currentLaser]*sin(beta[currentLaser])+yc[currentLaser]+tan(beta[currentLaser])*(LB[currentLaser]*cos(beta[currentLaser])+LA[currentLaser]+xc[currentLaser]))/(1-tan(beta[currentLaser])*tan( (-0.5*zita[currentLaser]) + x_corregida*zita[currentLaser]/(_camWidth-1) ));
        *XXp = *YYp*tan( (-0.5*zita[currentLaser]) + x_corregida*zita[currentLaser]/(_camWidth-1) );

        //cout << "coordenada X:" << *XXp << " coordenada Y:" << *YYp << " xp:" << x << " xp corregida:" << x_corregida << endl;
    }

    else if(laserID[currentLaser] == 2){

        float x_corregida=(FCC[currentLaser]-(float)yp/m[currentLaser]+x);

        *YYp = (LB[currentLaser]*sin(beta[currentLaser])+yc[currentLaser]+tan(beta[currentLaser])*(LB[currentLaser]*cos(beta[currentLaser])+LA[currentLaser]+xc[currentLaser]))/(1-tan(beta[currentLaser])*tan( (-0.5*zita[currentLaser]) + (x_corregida - (_camWidth-1))*zita[currentLaser]/(_camWidth-1) ));
        *XXp = *YYp*(-1.0)*tan( (-0.5*zita[currentLaser]) + (x_corregida - (_camWidth-1))*zita[currentLaser]/(_camWidth-1) );

        //cout << "Coordenada X:" << *XXp<< "Coordenada Y:" << *YYp<< "xp:" <<x<< "xp corregida:" <<x_corregida<< endl;
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

}

//--------------------------------------------------------------
void openC3DSprocess::exit(){
    xmlSettings.clear();
    xmlSettings.pushTag("OPENC3DS");
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

    guiProcess->addSlider("GPLL", 0, 255, &GPLL)->setIncrement(1);
    guiProcess->addSlider("PAP", 3, 100, &PAP)->setIncrement(1);
    guiProcess->addSlider("PMP", 3, 33, &PMP)->setIncrement(1);

    guiProcess->setPosition(0,0);
    guiProcess->autoSizeToFitWidgets();
	ofAddListener(guiProcess->newGUIEvent,this,&openC3DSprocess::guiEvent);
}

//--------------------------------------------------------------
void openC3DSprocess::guiEvent(ofxUIEventArgs &e){
	string name = e.getName();
	int kind = e.getKind();
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
