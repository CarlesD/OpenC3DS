#include "SC.h"

#include "Cam3D.h"
#include "3Dscan.h"



//--------------------------------------------------------------
void camera(Cam *cam, const char *CamFile){



if(load_cam_data(cam,CamFile) != 1){
        ofLogError() << "3Dscan::camera: File error";
}

cam->p = (Punts2D_subpix *)malloc(sizeof(Punts2D_subpix)*(cam->resy));


 if (cam->Calibration==0){
    cam->c1 = (Calibracio*)malloc(sizeof(Calibracio)*Ncal*cam->resy);
    cam->c2 = (Calibracio*)malloc(sizeof(Calibracio)*Ncal*cam->resy);




    for(int i=0; i<=49; i++){
        for(int j=0; j<=cam->resy-1; j++){
            cam->c1[m(i,j)].x = -10000;
            cam->c1[m(i,j)].y = -10000;
            cam->c1[m(i,j)].d = -10000;

            cam->c2[m(i,j)].x = -10000;
            cam->c2[m(i,j)].y = -10000;
            cam->c2[m(i,j)].d = -10000;
        }
    }

    calibracio_cam(cam);
 }
}

void free_camera(Cam *cam){

    free(cam->p);

     if (cam->Calibration==0){
        free(cam->fcl1);
        free(cam->fcl2);
        free(cam->c1);
        free(cam->c2);

     }
}


//--------------------------------------------------------------
void scan(Cam *cam, ofImage *grislaser, ofImage *TaL, ofImage *TsL){

    Mat image1;
    Mat Laser1;
    Mat Tot, gris, grisc;

    Mat HSV;
    Mat threshold1;

    Mat tt1, tt2, tt3, colo;

    Mat matImg;

    cv::Point punt;

    tt1 = toCv(*TaL).clone();
    Laser1 = tt1.clone(); // Tota imatge amb làser

    tt2 = toCv(*TsL).clone();
    Tot = tt2.clone(); // Tota imatge sense làser

    Mat th1, th2;
    Mat image2;

    absdiff(Laser1, Tot, image1); // Diferència -> discrimina el làser
    cvtColor(image1, HSV, CV_BGR2HSV);
    inRange(HSV, Scalar(cam->Bi, cam->Gi, cam->Ri), Scalar(cam->Bs, cam->Gs, cam->Rs), threshold1); // Captura un rang rgb
    th1 = threshold1.clone();
    image2 = image1.clone();
    GaussianBlur(threshold1, th1, cv::Size(1,1), 0,0); //crec que no fa res
    GaussianBlur(image2, image1, cv::Size(cam->blur_ksizew, cam->blur_ksizeh), cam->blur_sigmax, cam->blur_sigmay); //desenfoca la captura
    cam_cap_subpixel(cam, image1, threshold1); // troba amb precisió subpixel la posició del punt amb més intensitat per tots els punts de l'imatge

    cvtColor(image1, gris, CV_BGR2GRAY); // transforma a gris
    cvtColor(gris, grisc, CV_GRAY2BGR);

    for(int i=0; i<cam->resy; i++){
        cv::Point paux1;

        paux1.x = (int)cam->p[i].x;
        paux1.y = (int)cam->p[i].y;

        line(grisc, paux1, paux1, Scalar(255,0,0), 1,8,0); // punts en vermell sobre l'imatge en gris transformada a color
    }

    ofImage gl,L1,Tt;

    toOf(grisc, gl);
    gl.update();

    *grislaser = gl;

    toOf(Laser1, L1);
    L1.update();

    *TaL = L1;

    toOf(Tot, Tt);
    Tt.update();

    *TsL = Tt;
}

//--------------------------------------------------------------
int Component_3D_LinScan(Cam cam, int Laser, ofImage Tot, Punts p[], float incx){

    float delta_alfa, dist_alfa;
    float Xp, Yp;
    Mat Tot_Cv = toCv(Tot);


    for(int i=0; i<=cam.resy-1; i++){
        if(cam.p[i].x != cam.resx){
            delta_alfa = (PI/180.0f) * cam.alfa * (-1+((float)i/(float)(cam.resy/2.0f)));
            cam_dis(cam, 1, cam.p[i].x, cam.p[i].y, &Xp,&Yp);
            dist_alfa = sqrt(Xp*Xp+Yp*Yp)/cos(delta_alfa);

            if (dist_alfa < 1000){
                Vec3b pixel = Tot_Cv.at<Vec3b>( cam.p[i].y, cam.p[i].x );  // row,col index (NOT x,y)

                p[i].r = pixel[0];
                p[i].g = pixel[1];
                p[i].b = pixel[2];
                p[i].q = cam.p[i].q;

                p[i].x = incx-Xp;
                p[i].y = -Yp;
                p[i].z = dist_alfa * sin(delta_alfa);
            }
            else{
                p[i].x = -10000;
                p[i].y = -10000;
                p[i].z = -10000;
                p[i].r = 255;
                p[i].g = 0;
                p[i].b = 0;
            }
        } // end if(cam.p[i].x != 1024)
        else{
            p[i].x = -10000;
            p[i].y = -10000;
            p[i].z = -10000;
            p[i].r = 255;
            p[i].g = 0;
            p[i].b = 0;
        }
    } // end for

    return(1);
}

int Component_3D_Angular_1_axis_Scan(Cam cam, int Laser, ofImage Tot, Punts p[], float phi){

    float delta_alfa, dist_alfa;
    float Xp, Yp;
    Mat Tot_Cv = toCv(Tot);

    for(int i=0; i<=cam.resy-1; i++){
        if(cam.p[i].x != cam.resx){
            delta_alfa = (PI/180.0f) * cam.alfa * (-1+((float)i/(float)(cam.resy/2.0f)));
            cam_dis(cam, 1, cam.p[i].x, cam.p[i].y, &Xp,&Yp);
            dist_alfa = sqrt(Xp*Xp+Yp*Yp)/cos(delta_alfa);

            if (dist_alfa < 1000){
                Vec3b pixel = Tot_Cv.at<Vec3b>( cam.p[i].y, cam.p[i].x );  // row,col index (NOT x,y)

                p[i].r = pixel[0];
                p[i].g = pixel[1];
                p[i].b = pixel[2];
                p[i].q = cam.p[i].q;

                p[i].x = Xp*cos(phi)+ (cam.L+cam.yc-Yp)*sin(phi);
                p[i].y = -Xp*sin(phi)+(cam.L+cam.yc-Yp)*cos(phi);
                p[i].z = dist_alfa * sin(delta_alfa);
            }
            else{
                p[i].x = -10000;
                p[i].y = -10000;
                p[i].z = -10000;
                p[i].r = 255;
                p[i].g = 0;
                p[i].b = 0;
            }
        } // end if(cam.p[i].x != 1024)
        else{
            p[i].x = -10000;
            p[i].y = -10000;
            p[i].z = -10000;
            p[i].r = 255;
            p[i].g = 0;
            p[i].b = 0;
        }
    } // end for

    return(1);
}



//--------------------------------------------------------------
int calibracio_cam(Cam *cam){
    int n,res;
    FILE * pFile1;



        if(cam->Laser1 == 1){
            pFile1 = fopen ("HD525_1.cal","r");
            fscanf(pFile1,"%d\n",&n);
            fscanf(pFile1,"%d\n",&res);
            cam->nc1 = n;
            cam->resyc1 = res;

            for (int i=0; i<=cam->nc1-1; i++){
                for(int j=0; j<=cam->resyc1; j++){
                    fscanf(pFile1, "%f %d %d\n",&cam->c1[m(i,j)].d,&cam->c1[m(i,j)].x,&cam->c1[m(i,j)].y);
                }
            }
            fclose(pFile1);

            for(int i=0; i<=cam->nc1-1; i++){
                for(int j=0; j<=cam->resyc1; j++){
                    cam->c1[m(i,j)].a = ( (cam->c1[m((i+1),j)].d - cam->c1[m(i,j)].d) / (cam->c1[m((i+1),j)].x - cam->c1[m(i,j)].x) );
                    cam->c1[m(i,j)].b = cam->c1[m(i,j)].d - cam->c1[m(i,j)].x * cam->c1[m(i,j)].a;
                }
            }
        } // end if(cam->Laser1 == 1)


        if (cam->Laser2 == 1){
            pFile1 = fopen ("HD525_2.cal","r");
            fscanf(pFile1,"%d\n",&n);
            fscanf(pFile1,"%d\n",&res);
            cam->nc2 = n;
            cam->resyc2 = res;
            for(int i=0; i<=cam->nc2; i++){
                for(int j=0; j<=cam->resyc2; j++) {
                    fscanf(pFile1, "%f %d %d\n", &cam->c2[m(i,j)].d, &cam->c2[m(i,j)].x, &cam->c2[m(i,j)].y);
                }
            }
            fclose(pFile1);

            for(int i=0; i<=cam->nc2-1; i++){
                for(int j=0; j<=cam->resyc2; j++){
                    cam->c2[m(i,j)].a = ( (cam->c2[m((i+1),j)].d - cam->c2[m(i,j)].d) / (cam->c2[m((i+1),j)].x - cam->c2[m(i,j)].x) );
                    cam->c2[m(i,j)].b = cam->c2[m(i,j)].d - cam->c2[m(i,j)].x * cam->c2[m(i,j)].a;
                }
            }
        } // end if (cam->Laser2 == 1)


    return(1); // COMENTARI Anna: aquesta funció sempre retorna 1 passi el que passi
}



//--------------------------------------------------------------
void cam_dis(Cam cam, int Laser, float x, int yp, float *XXp, float *YYp){

    float d;
    int xp;

    xp = (int)x;

    if (cam.Calibration==0){

        if(Laser == 1){
            if(xp < cam.c1[m(0,yp)].x){
                d = -10000;
            }
            for(int i=0; i<=cam.nc1-2; i++){
                if( (xp >= cam.c1[m(i,yp)].x)&&(xp <= cam.c1[m((i+1),yp)].x) ){
                    d = cam.c1[m(i,yp)].a * x + cam.c1[m(i,yp)].b;
                }
            }
            if(xp > cam.c1[m((cam.nc1-1),yp)].x){
                d = -10000;
            }

            *YYp = d + 0.79;
            *XXp = ( *YYp - (cam.yc + tan(cam.Beta1) * ( (cam.LA / sin(cam.Beta1)) + cam.LB )) ) / (tan(cam.Beta1));

        } // end if(Laser == 1)

        if(Laser == 2){
            if(xp > cam.c2[m(0,yp)].x){
                d = -10000;
            }
            for(int i=0; i<=cam.nc2-2; i++){
                if( (xp <= cam.c2[m(i,yp)].x)&&(xp >= cam.c2[m((i+1),yp)].x) ){
                    d = cam.c2[m(i,yp)].a * x + cam.c2[m(i,yp)].b;
                }
            }
            if(xp < cam.c2[m((cam.nc1-1),yp)].x){
                d = -10000;
            }

            *YYp = d;
            *XXp = ( *YYp - (cam.yc + tan(cam.Beta2) * ( (cam.LA / sin(cam.Beta2)) + cam.LB )) ) / (tan(cam.Beta2));
        }

    }

    if (cam.Calibration==1){

        if(Laser == 1){

            float x_corregida=(cam.FCC-(float)yp/cam.m+x);

            *YYp = (cam.LB*sin(cam.Beta1)+cam.yc+tan(cam.Beta1)*(cam.LB*cos(cam.Beta1)+cam.LA+cam.xc))/(1-tan(cam.Beta1)*tan( (-0.5*cam.zita) + x_corregida*cam.zita/(cam.resx-1) ));
            *XXp = *YYp*tan( (-0.5*cam.zita) + x_corregida*cam.zita/(cam.resx-1) );

            cout << "Coordenada X:" << *XXp<< "Coordenada Y:" << *YYp<< "xp:" <<x<< "xp corregida:" <<x_corregida<< endl;

        }
    }
}

//--------------------------------------------------------------
int cam_cap(Cam *cam, Mat thr1, Mat thr2){
    int x1,x2,cont;
    cont = 0;

    for (int i=0; i<cam->resy; ++i){
        cam->p[i].x = cam->resx;
        cam->p[i].y = 0;
    }

    for(int i=0; i<thr1.rows; ++i){
        uchar * pixel1 = thr1.ptr<uchar>(i);

        for(int j=0; j<thr1.cols; ++j){
            if(pixel1[j] != 0){
                if(cont == 0){
                    x1 = j;
                    cont = 1;
                }
                else{
                    if( (j-x1) < 25 ){
                        x2 = j;
                    }
                }
            }
        } // end for j

        if(cont == 1){
            cont = 0;
            cam->p[i].x = x1 + ( ((float)(x2-x1) / 2.0f) );
            cam->p[i].y = i;
        }
    } // end for(int i=0; i<thr1.rows; ++i)

    return(1); // COMENTARI Anna: aquesta funció sempre retorna 1 passi el que passi
}

//--------------------------------------------------------------
int cam_cap_subpixel(Cam *cam, Mat ima1, Mat thr1){

    Mat sub1;
    int cont, jmax, jmin;
    float x1;

    double x[200], y[200];
    int k, imax1;

    double coeff[DEGREE];

    k = 0;
    x1 = 0;
    jmax = -1;
    jmin = -1;

    cvtColor(ima1, sub1, CV_BGR2GRAY);

    x1 = 0;
    cont = 0;
    imax1 = 0;

    for(int i=0; i<cam->resy; ++i){
        cam->p[i].x = cam->resx;
        cam->p[i].y = 0;
    }
    for(int i=0; i<thr1.rows; ++i){
        uchar * pixel1 = thr1.ptr<uchar>(i);

        for(int j=0; j<thr1.cols; ++j){
            if( ((int)sub1.at<uchar>(i,j)>cam->GPLL)&&((int)sub1.at<uchar>(i,j)>imax1) ){
                imax1 = (int)sub1.at<uchar>(i,j);
                jmin = jmax;
                jmax = j;
            }
        }
        if(jmin != -1){
            jmax = ( (jmax - jmin) / 2 ) + jmin; //trobo dos max
        }
        for(int j=jmax-cam->PAP; j<jmax+cam->PAP; ++j){
            if(j>0 && j<thr1.cols &&(int)sub1.at<uchar>(i,j)>cam->GPLL ){
                x[k] = (double)j;
                y[k] = (double)sub1.at<uchar>(i,j);
                cont = 1;
                k = k + 1;
            }
        } // end for
        if(k >= cam->PMP){
            polynomialfit(k, DEGREE, x, y, coeff);


            if(coeff[2] != 0){
                x1 = fabs( (-1 * coeff[1]) / (2 * coeff[2]) );
                if(fabs(x1-jmax )> 10){
                    x1 = cam->resx;
                }
            }
        } // end if(k >= cam->PMP)
        else{
            x1 = cam->resx;
        }

        if(cont == 1){
//            for (jj=0; jj<k; jj++){ x1 = x1 + xcan[jj] };
            cont = 0;
            if(x1>0 && x1<cam->resx && k>=cam->PMP){
                cam->p[i].x = x1;
                cam->p[i].y = i;
                cam->p[i].a = k;
                cam->p[i].q = k;
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
bool polynomialfit(int obs, int degree, double *dx, double *dy, double *store){ /* n, p */
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
int save_cam_data(Cam cam, const char *name){
    FILE *pFile;
    pFile = fopen (name,"w");

    fprintf (pFile, "%d\n",cam.DeviceId);
    fprintf (pFile, "%d\n",cam.VideoId);
    fprintf (pFile, "%d\n",cam.resx);
    fprintf (pFile, "%d\n",cam.resy);
    fprintf (pFile, "%s\n",cam.fcl1);
    fprintf (pFile, "%s\n",cam.fcl2);
    fprintf (pFile, "%d\n",cam.Laser1);
    fprintf (pFile, "%d\n",cam.Laser2);
    fprintf (pFile, "%f\n",cam.alfa);
    fprintf (pFile, "%f\n",cam.zita);
    fprintf (pFile, "%f\n",cam.Beta1);
    fprintf (pFile, "%f\n",cam.Beta2);
    fprintf (pFile, "%f\n",cam.xc);
    fprintf (pFile, "%f\n",cam.yc);
    fprintf (pFile, "%f\n",cam.LB);
    fprintf (pFile, "%f\n",cam.LA);
    fprintf (pFile, "%d\n",cam.blur_ksizeh);
    fprintf (pFile, "%d\n",cam.blur_ksizew);
    fprintf (pFile, "%d\n",cam.blur_sigmax);
    fprintf (pFile, "%d\n",cam.blur_sigmay);
    fprintf (pFile, "%d\n",cam.Ri);
    fprintf (pFile, "%d\n",cam.Gi);
    fprintf (pFile, "%d\n",cam.Bi);
    fprintf (pFile, "%d\n",cam.Rs);
    fprintf (pFile, "%d\n",cam.Gs);
    fprintf (pFile, "%d\n",cam.GPLL);
    fprintf (pFile, "%d\n",cam.PAP);
    fprintf (pFile, "%d\n",cam.PMP);
    fprintf(pFile, "%f\n",cam.m);
    fprintf(pFile, "%f\n",cam.FCC);
    fprintf(pFile, "%f\n",cam.L);
    fprintf(pFile, "%d\n",cam.Calibration);
    fclose(pFile);

     cout << "cam.DeviceId: " << cam.DeviceId<< endl;
     cout << "cam.VideoId: " << cam.VideoId<< endl;
     cout << "cam.resx: " << cam.resx<< endl;
     cout << "cam.resy: " << cam.resy<< endl;
     cout << "cam.fcl1: " << cam.fcl1<< endl;
     cout << "cam.fcl2: " << cam.fcl2<< endl;
     cout << "cam.Laser1: " << cam.Laser1<< endl;
     cout << "cam.Laser2: " << cam.Laser2<< endl;
     cout << "cam.alfa: " << cam.alfa<< endl;
     cout << "cam.zita: " << cam.zita<< endl;
     cout << "cam.Beta1: " << cam.Beta1<< endl;
     cout << "cam.Beta2: " << cam.Beta2<< endl;
     cout << "cam.xc: " << cam.xc<< endl;
     cout << "cam.yc: " << cam.yc<< endl;
     cout << "cam.LB: " << cam.LB<< endl;
     cout << "cam.LA: " << cam.LA<< endl;
     cout << "cam.blur_ksizeh: " << cam.blur_ksizeh<< endl;
     cout << "cam.blur_ksizew: " << cam.blur_ksizew<< endl;
     cout << "cam.blur_sigmax: " << cam.blur_sigmax<< endl;
     cout << "cam.blur_sigmay: " << cam.blur_sigmay<< endl;
     cout << "cam.Ri: " << cam.Ri<< endl;
     cout << "cam.Gi: " << cam.Gi<< endl;
     cout << "cam.Bi: " << cam.Bi<< endl;
     cout << "cam.Rs: " << cam.Rs<< endl;
     cout << "cam.Gs: " << cam.Gs<< endl;
     cout << "cam.GPLL: " << cam.GPLL<< endl;
     cout << "cam.PAP: " << cam.PAP<< endl;
     cout << "cam.PMP: " << cam.PMP<< endl;
     cout << "cam.m: " << cam.m<< endl;
     cout << "cam.FCC: " << cam.FCC<< endl;
     cout << "cam.L: " << cam.L<< endl;
     cout << "cam.Calibration: " << cam.Calibration<< endl;
}

//--------------------------------------------------------------
int load_cam_data(Cam *cam, const char *name){
    FILE *pFile;
    pFile = fopen (name,"r");

    if (pFile!=NULL){
        fscanf(pFile, "%d\n",&cam->DeviceId);
        fscanf(pFile, "%d\n",&cam->VideoId);
        fscanf(pFile, "%d\n",&cam->resx);
        fscanf(pFile, "%d\n",&cam->resy);

        fscanf(pFile, "%s\n",cam->fcl1);
        fscanf(pFile, "%s\n",cam->fcl2);

        fscanf(pFile, "%d\n",&cam->Laser1);
        fscanf(pFile, "%d\n",&cam->Laser2);
        fscanf(pFile, "%f\n",&cam->alfa);
        fscanf(pFile, "%f\n",&cam->zita);
        fscanf(pFile, "%f\n",&cam->Beta1);
        fscanf(pFile, "%f\n",&cam->Beta2);
        fscanf(pFile, "%f\n",&cam->xc);
        fscanf(pFile, "%f\n",&cam->yc);
        fscanf(pFile, "%f\n",&cam->LB);
        fscanf(pFile, "%f\n",&cam->LA);
        fscanf(pFile, "%d\n",&cam->blur_ksizeh);
        fscanf(pFile, "%d\n",&cam->blur_ksizew);
        fscanf(pFile, "%d\n",&cam->blur_sigmax);
        fscanf(pFile, "%d\n",&cam->blur_sigmay);
        fscanf(pFile, "%d\n",&cam->Ri);
        fscanf(pFile, "%d\n",&cam->Gi);
        fscanf(pFile, "%d\n",&cam->Bi);
        fscanf(pFile, "%d\n",&cam->Rs);
        fscanf(pFile, "%d\n",&cam->Gs);
        fscanf(pFile, "%d\n",&cam->GPLL);
        fscanf(pFile, "%d\n",&cam->PAP);
        fscanf(pFile, "%d\n",&cam->PMP);
        fscanf(pFile, "%f\n",&cam->m);
        fscanf(pFile, "%f\n",&cam->FCC);
        fscanf(pFile, "%f\n",&cam->L);
        fscanf(pFile, "%d\n",&cam->Calibration);
        fclose(pFile);

        cout << "cam.DeviceId: " << cam->DeviceId<< endl;
        cout << "cam.VideoId: " << cam->VideoId<< endl;
        cout << "cam.resx: " << cam->resx<< endl;
        cout << "cam.resy: " << cam->resy<< endl;
        cout << "cam.fcl1: " << cam->fcl1<< endl;
        cout << "cam.fcl2: " << cam->fcl2<< endl;
        cout << "cam.Laser1: " << cam->Laser1<< endl;
        cout << "cam.Laser2: " << cam->Laser2<< endl;
        cout << "cam.alfa: " << cam->alfa<< endl;
        cout << "cam.zita: " << cam->zita<< endl;
        cout << "cam.Beta1: " << cam->Beta1<< endl;
        cout << "cam.Beta2: " << cam->Beta2<< endl;
        cout << "cam.xc: " << cam->xc<< endl;
        cout << "cam.yc: " << cam->yc<< endl;
        cout << "cam.LB: " << cam->LB<< endl;
        cout << "cam.LA: " << cam->LA<< endl;
        cout << "cam.blur_ksizeh: " << cam->blur_ksizeh<< endl;
        cout << "cam.blur_ksizew: " << cam->blur_ksizew<< endl;
        cout << "cam.blur_sigmax: " << cam->blur_sigmax<< endl;
        cout << "cam.blur_sigmay: " << cam->blur_sigmay<< endl;
        cout << "cam.Ri: " << cam->Ri<< endl;
        cout << "cam.Gi: " << cam->Gi<< endl;
        cout << "cam.Bi: " << cam->Bi<< endl;
        cout << "cam.Rs: " << cam->Rs<< endl;
        cout << "cam.Gs: " << cam->Gs<< endl;
        cout << "cam.GPLL: " << cam->GPLL<< endl;
        cout << "cam.PAP: " << cam->PAP<< endl;
        cout << "cam.PMP: " << cam->PMP<< endl;
        cout << "cam.m: " << cam->m<< endl;
        cout << "cam.FCC: " << cam->FCC<< endl;
        cout << "cam.L: " << cam->L<< endl;
        cout << "cam.Calibration: " << cam->Calibration<< endl;
        return (1);
    }
    else{
        fclose(pFile);
        return (0);
    }

}
