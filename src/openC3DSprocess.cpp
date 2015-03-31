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
        laserSide[i] = xmlSettings.getValue("laserSide", 0); // 0 esquerra
        cout << "OPENC3DS:laserSide = " << laserSide[i] << endl;

        xmlSettings.popTag();
    }

	// GUI
	setGuiProcess();
	guiProcess->loadSettings("guiProcess.xml");

}

//--------------------------------------------------------------
void openC3DSprocess::setupCamResolution(int w, int h){
    _camWidth = w;
	_camHeight = h;
}

//--------------------------------------------------------------
bool openC3DSprocess::camCaptureSubpixelProcess(unsigned char* pixelsRaw){
    int cont, jmax, jmin;
    float x1;

    double x[200], y[200];
    int k, imax1;

    double coeff[DEGREE];

    k = 0;
    x1 = 0;
    jmax = -1;
    jmin = -1;

    x1 = 0;
    cont = 0;
    imax1 = 0;

    cout << "_camWidth: " << _camWidth << " _camHeight: " << _camHeight << endl;

    for(int i=0; i<_camHeight; ++i){
        p[i].x = _camWidth;
        p[i].y = 0;
    }
    cout << "exit 1 for"<< endl;

    for(int i=0; i<_camHeight; ++i){
        cout << "inside 2on for with i (up to _camHeight): " << i << endl;

        for(int j=0; j<_camWidth; ++j){
            if( (pixelsRaw[i*_camWidth+j] > GPLL) && (pixelsRaw[i*_camWidth+j] > imax1) ){
                imax1 = pixelsRaw[i*_camWidth+j];
                jmin = jmax;
                jmax = j;
            }
        }
        if(jmin != -1){
            jmax = ( (jmax - jmin) / 2 ) + jmin; //trobo dos max
        }
        for(int j=jmax-PAP; j<jmax+PAP; ++j){
            if(j>0 && j<_camWidth &&pixelsRaw[i*_camWidth+j]>GPLL ){
                x[k] = (double)j;
                y[k] = (double)pixelsRaw[i*_camWidth+j];
                cont = 1;
                k = k + 1;
            }
        } // end for
        if(k >= PMP){
            polynomialfit(k, DEGREE, x, y, coeff);


            if(coeff[2] != 0){
                x1 = fabs( (-1 * coeff[1]) / (2 * coeff[2]) );
                if(fabs(x1-jmax )> 10){
                    x1 = _camWidth;
                }
            }
        } // end if(k >= cam->PMP)
        else{
            x1 = _camWidth;
        }

        if(cont == 1){
//            for (jj=0; jj<k; jj++){ x1 = x1 + xcan[jj] };
            cont = 0;
            if(x1>0 && x1<_camWidth && k>=PMP){
                p[i].x = x1;
                p[i].y = i;
                p[i].a = k;
                p[i].q = k;
            }
        }

        x1 = 0;
        imax1 = 0;
        jmax = -1;
        jmin = -1;
        k = 0;
    } // end for(int i=0; i<thr1.rows; ++i)
}

//--------------------------------------------------------------
bool openC3DSprocess::update(){

}

//--------------------------------------------------------------
void openC3DSprocess::draw(){

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
        xmlSettings.setValue("laserSide", laserSide[i]);

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
