#include "openC3DSprocess.h"

//--------------------------------------------------------------
void openC3DSprocess::setup(){

	// GUI
	setGuiProcess();
	guiProcess->loadSettings("guiProcess.xml");

    L = 271; // mm

    Beta[0] = 0.959931;
    xc[0] = 0;
    yc[0] = 8.08;
    LA[0] = 49.529999;
    LB[0] = 115.989998;
    alfa[0] = 19.4;
    zita[0] = 1.204277;
    m[0] = 37.27;
    FCC[0] = 5.665022;
    Laser_side[0] = 0; // 0 esquerra

    Beta[1] = 0.959931;
    xc[1] = 0;
    yc[1] = 8.08;
    LA[1] = 49.529999;
    LB[1] = 115.989998;
    alfa[1] = 19.4;
    zita[1] = 1.204277;
    m[1] = 37.27;
    FCC[1] = 5.665022;
    Laser_side[1] = 1; // 0 dreta

}

//--------------------------------------------------------------
void openC3DSprocess::setupCamResolution(int w, int h){
    _camWidth = w;
	_camHeight = h;
}

//--------------------------------------------------------------
bool openC3DSprocess::cam_cap_subpixel(unsigned char* pixelsRaw){
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

    for(int i=0; i<_camHeight; ++i){
        p[i].x = _camWidth;
        p[i].y = 0;
    }
    for(int i=0; i<_camHeight; ++i){
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
