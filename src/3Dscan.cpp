
#include "SC.h"

#include "Cam3D.h"
#include "3Dscan.h"
//static const char* portName = "/dev/ttyUSB0";

float z, f,zin,fin,zfi,ffi,zinc;
float finc;

long npt=0;


void camera(Cam *cam,const char *CamFile)
{
    int i,j;


cam->fcl1=(char *)malloc(sizeof(char)*50);
cam->fcl2=(char *)malloc(sizeof(char)*50);


if(load_cam_data(cam,CamFile)!=1){cout << "File error";}


cam->c1=(Calibracio*)malloc(sizeof(Calibracio)*Ncal*cam->resy);
cam->c2=(Calibracio*)malloc(sizeof(Calibracio)*Ncal*cam->resy);


cam->p=(Punts2D_subpix *)malloc(sizeof(Punts2D_subpix)*(cam->resy));
cam->p=(Punts2D_subpix *)malloc(sizeof(Punts2D_subpix)*(cam->resy));


for(i=0;i<=49;i++)
{
for(j=0;j<=575;j++)
{


cam->c1[m(i,j)].x=-10000;
cam->c1[m(i,j)].y=-10000;
cam->c1[m(i,j)].d=-10000;

cam->c2[m(i,j)].x=-10000;
cam->c2[m(i,j)].y=-10000;
cam->c2[m(i,j)].d=-10000;


}
}
//cam_ini(cam);


calibracio_cam(cam);

}


void scan(Cam *cam, ofImage *grislaser,ofImage *TaL,ofImage *TsL)
{
int amp;

Mat image1;
Mat Laser1;
Mat Tot,gris,grisc;

Mat HSV;
Mat threshold1;

//camera(cam);


int valueRL=60,valueGL=0,valueBL=0;

int i;


amp=288;

Mat tt1,tt2,tt3,colo;

Mat matImg;

cv::Point punt;

tt1 = toCv(*TaL).clone();
Laser1=tt1.clone();



tt2 = toCv(*TsL).clone();
Tot=tt2.clone();
Mat th1, th2;
Mat image2;


absdiff(Laser1, Tot, image1);
cvtColor(image1,HSV,CV_BGR2HSV);
inRange(HSV,Scalar(cam->Bi,cam->Gi,cam->Ri),Scalar(cam->Bs,cam->Gs,cam->Rs),threshold1);
th1=threshold1.clone();
image2=image1.clone();
GaussianBlur(threshold1,th1,cv::Size(1,1),0,0);
GaussianBlur(image2,image1,cv::Size(cam->blur_ksizew,cam->blur_ksizeh),cam->blur_sigmax,cam->blur_sigmay);
cam_cap_subpixel(cam,image1,threshold1);

cvtColor(image1,gris,CV_BGR2GRAY);
cvtColor(gris,grisc,CV_GRAY2BGR);

for(i=0;i<2*amp;i++)
{

 cv::Point paux1;
 paux1.x=(int)cam->p[i].x;
 paux1.y=(int)cam->p[i].y;


line(grisc,paux1,paux1,Scalar(255,0,0),1,8,0);
}

ofImage gl,L1,Tt;

toOf(grisc,gl);
gl.update();

*grislaser=gl;

toOf(Laser1,L1);
L1.update();

*TaL=L1;

toOf(Tot,Tt);
Tt.update();

*TsL=Tt;


}


int Component_3D_LinScan(Cam cam, int Laser,ofImage Tot, Punts p[], float incx)
{
    int i;
    float delta_alfa,dist_alfa;
    float Xp,Yp;
    Mat Tot_Cv=toCv(Tot);
    float Lc=100;
    for(i=0;i<=cam.resy-1;i++)
		{
		    if (cam.p[i].x!=1024)
            {


			delta_alfa=(PI/180)*cam.alfa*(1-((float)i/(float)(cam.resy/2)));
			cam_dis(cam,1,cam.p[i].x,cam.p[i].y,&Xp,&Yp);
            dist_alfa=sqrt(Xp*Xp+Yp*Yp)/cos(delta_alfa);

            if (dist_alfa<1000)
            {
             Vec3b pixel = Tot_Cv.at<Vec3b>( cam.p[i].y, cam.p[i].x );  // row,col index (NOT x,y)

            p[i].r=pixel[0];
            p[i].g=pixel[1];
            p[i].b=pixel[2];
            p[i].q=cam.p[i].q;



            p[i].x=incx-Xp;
            p[i].y=-Yp;
            p[i].z=dist_alfa*sin(delta_alfa);
            }else{p[i].x=-10000;
            p[i].y=-10000;
            p[i].z=-10000; p[i].r=255;
            p[i].g=0;
            p[i].b=0;}

            }
            else{
            p[i].x=-10000;
            p[i].y=-10000;
            p[i].z=-10000;  p[i].r=255;
            p[i].g=0;
            p[i].b=0;}
		}

return(1);
}

void contenidor(Cam cam, ofImage *image)
{
float minx=1024;
int ixmin=0;
int deltak=10;
float xco,yco;
cv::Point punt;
float recta_sx[cam.resy-1],recta_ix[cam.resy-1],recta_sy[cam.resy-1],recta_iy[cam.resy-1];
double scx,scxx,scy,scxy,meanc_x,meanc_y, varcx,covc,paramcs0,paramcs1,paramci0,paramci1;
float Xco,Yco,Xc1,Yc1,Zco,Zc1;
int i,j,k;
double delta_alfa,delta_alfa1;
Mat ima_aux=toCv(*image);

j=0;
k=0;
for(i=0;i<=cam.resy-1;i++)
		{
		    if (cam.p[i].x<minx){ixmin=i;minx=cam.p[i].x;}
		}

for(i=0;i<=ixmin-deltak;i++)
		{
		    if (cam.p[i].x!=1024){recta_sx[j]=cam.p[i].x;recta_sy[j]=cam.p[i].y;j=j+1;scx+=cam.p[i].x;scy+=cam.p[i].y;scxy+=cam.p[i].x*cam.p[i].y;scxx+=cam.p[i].x*cam.p[i].x;}

		}

double mean_cx = scx / j;
double mean_cy = scy / j;

varcx = scxx - scx * mean_cx;
covc = scxy - scx * mean_cy;

// check for zero varx
paramcs0 = covc / varcx;
paramcs1= mean_cy - paramcs0 * mean_cx;


scx=0;scy=0;scxy=0;scxx=0;

for(i=ixmin+deltak;i<=cam.resy-1;i++)
		{
		    if (cam.p[i].x!=1024){recta_ix[k]=cam.p[i].x;recta_iy[k]=cam.p[i].y;k=k+1;scx+=cam.p[i].x;scy+=cam.p[i].y;scxy+=cam.p[i].x*cam.p[i].y;scxx+=cam.p[i].x*cam.p[i].x;}
		}


mean_cx = scx / k;
mean_cy = scy / k;

varcx = scxx - scx * mean_cx;
covc = scxy - scx * mean_cy;

// check for zero varx
paramci0 = covc / varcx;
paramci1= mean_cy - paramci0 * mean_cx;

xco=(-paramcs1+paramci1)/(paramcs0-paramci0);
yco=paramcs0*xco+paramcs1;



punt.x=xco;
punt.y=yco;



circle(ima_aux, punt, 10, Scalar( 20,255,255 ),2, 1, 0);


ofImage im;

toOf(ima_aux,im);
im.update();

*image=im;


if(j!=0 &&k!=0){
cam_dis(cam,1,xco,yco,&Xco,&Yco);
cam_dis(cam,1,xco+50,paramci0*(xco+50)+paramci1,&Xc1,&Yc1);

delta_alfa=(PI/180)*cam.alfa*(1-((float)yco/(float)(cam.resy/2)));


if (delta_alfa!=0){delta_alfa1=sqrt(Xco*Xco+Yco*Yco)/cos(delta_alfa);}

Zco=delta_alfa1*sin(delta_alfa);


delta_alfa=(PI/180)*cam.alfa*(1-((float)(paramci0*(xco+50)+paramci1)/(float)(cam.resy/2)));

if (delta_alfa!=0){delta_alfa1=sqrt(Xc1*Xc1+Yc1*Yc1)/cos(delta_alfa);}

Zc1=delta_alfa1*sin(delta_alfa);

float anglec;

if ((Zc1-Zco)!=0){anglec=-1*atan((Yc1-Yco)/(Zc1-Zco));} else{anglec=0;}

cout << "Posició Vertex: "<<Yco<<","<<Zco << endl;
cout << "Posició centre Contenidor: "<<Yco+25*sin(anglec)+25*cos(anglec)<<","<<Zco-25*cos(anglec)+25*sin(anglec) << endl;
cout << "Angle camió: "<<(180/PI)*anglec << endl;
//printf("Posició vertex-->Y:%f Z:%f \n",Yco,Zco);
//printf("Posició centre-->Y:%f Z:%f \n",Yco+25*sin(anglec)+25*cos(anglec),Zco-25*cos(anglec)+25*sin(anglec));
//printf("Angle/camió: %f \n", (180/PI)*anglec);
//circle(image1, punt, 10, Scalar( 20,255,255 ),2, 1, 0);
//contenidor

}
}


int calibracio_cam(Cam *cam)
{
int i,j;
int n,res;
FILE * pFile1;


    if (cam->Laser1==1)
    {
        pFile1 = fopen ("HD525_1.cal","r");
        fscanf(pFile1,"%d\n",&n);
        fscanf(pFile1,"%d\n",&res);
        cam->nc1=n;
        cam->resxc1=res;

        for (i=0;i<=cam->nc1-1;i++)
            {
            for(j=0;j<=cam->resxc1;j++) {fscanf(pFile1, "%f %d %d\n",&cam->c1[m(i,j)].d,&cam->c1[m(i,j)].x,&cam->c1[m(i,j)].y);


     }

            }
        fclose (pFile1);

        for(i=0;i<=cam->nc1-1;i++)
            {
                for(j=0;j<=cam->resxc1;j++)
                    {
                    cam->c1[m(i,j)].a=(( cam->c1[m((i+1),j)].d-cam->c1[m(i,j)].d)/(cam->c1[m((i+1),j)].x-cam->c1[m(i,j)].x));
                    cam->c1[m(i,j)].b=cam->c1[m(i,j)].d-cam->c1[m(i,j)].x*(cam->c1[m(i,j)].a);
                    }
            }
    }


    if (cam->Laser2==1)
    {
        pFile1 = fopen ("HD525_2.cal","r");
        fscanf(pFile1,"%d\n",&n);
        fscanf(pFile1,"%d\n",&res);
        cam->nc2=n;
        cam->resxc2=res;
        for (i=0;i<=cam->nc2;i++)
            {
            for(j=0;j<=cam->resxc2;j++) {fscanf(pFile1, "%f %d %d\n",&cam->c2[m(i,j)].d,&cam->c2[m(i,j)].x,&cam->c2[m(i,j)].y);}
            }
        fclose (pFile1);

        for(i=0;i<=cam->nc2-1;i++)
            {
                for(j=0;j<=cam->resxc2;j++)
                    {
                    cam->c2[m(i,j)].a=(( cam->c2[m((i+1),j)].d-cam->c2[m(i,j)].d)/(cam->c2[m((i+1),j)].x-cam->c2[m(i,j)].x));
                    cam->c2[m(i,j)].b=cam->c2[m(i,j)].d-cam->c2[m(i,j)].x*( cam->c2[m(i,j)].a);
                    }
            }

    }



return(1);
}

void cam_ini(Cam *cam)
{

        cam->Beta1=(63.9*PI)/180;
        cam->Beta2=(63.85*PI)/180;
        cam->xc=5;
        cam->yc=5;
        cam->l1=50.5;
        cam->L=45.8470;
        cam->Laser1=1;
        cam->Laser2=1;
        cam->Alaser1=0;
        cam->Alaser2=0;
        cam->ax=0;
        cam->ay=0;
        cam->az=0;
        cam->Nlaser=2;
        cam->Tlaser1=1;
        cam->Tlaser2=1;
        cam->alfa=18.25;


}

void cam_dis(Cam cam,int Laser,float x,int yp,float *XXp, float *YYp)
{
    float d;
    int i,xp;


    xp=(int)x;


    if ( Laser==1)
        {
            if (xp<cam.c1[m(0,yp)].x) d = -10000;
            for(i=0;i<=cam.nc1-2;i++){if (xp>=cam.c1[m(i,yp)].x && xp<=cam.c1[m((i+1),yp)].x) d=cam.c1[m(i,yp)].a*x + cam.c1[m(i,yp)].b;}
            if (xp>cam.c1[m((cam.nc1-1),yp)].x)d=-10000;

            *YYp=d+0.79;
            *XXp=(*YYp-(cam.yc+tan(cam.Beta1)*((cam.L/sin(cam.Beta1)) +cam.l1)))/(tan(cam.Beta1));

        }



    if ( Laser==2)
        {
            if (xp>cam.c2[m(0,yp)].x) d=-10000;
            for(i=0;i<=cam.nc2-2;i++){if (xp<=cam.c2[m(i,yp)].x && xp>=cam.c2[m((i+1),yp)].x) d=cam.c2[m(i,yp)].a*x + cam.c2[m(i,yp)].b;}

            if (xp<cam.c2[m((cam.nc1-1),yp)].x)d=-10000;

            *YYp=d;
            *XXp=(*YYp-(cam.yc+tan(cam.Beta2)*((cam.L/sin(cam.Beta2)) +cam.l1)))/(tan(cam.Beta2));
        }


}





int cam_cap(Cam *cam,Mat thr1, Mat thr2)
{


int i,x1,x2,cont;



cont=0;


    for (i = 0; i < cam->resy; ++i){cam->p[i].x=1024;cam->p[i].y=0;}
   for (i = 0; i < thr1.rows; ++i)
				{
					uchar * pixel1 = thr1.ptr<uchar>(i);

				 for (int j = 0; j < thr1.cols; ++j)
						{
 	 							if(pixel1[j]!=0)
  	 								{
									if(cont==0){x1=j;cont=1;}

									else{ if((j-x1)<25) x2=j; }
									}
						}

						if (cont==1)
							{
							cont=0;cam->p[i].x=x1+(((float)((x2-x1))/2));
                            cam->p[i].y=i;
							}
				}


return(1);
}







int cam_cap_subpixel(Cam *cam,Mat ima1, Mat thr1)
{
Mat sub1;


int i,j,cont,jmax,jmin;
float x1;

double x[200],y[200];
int k,imax1;

double coeff[DEGREE];
k=0;

x1=0;


jmax=-1;
jmin=-1;
cvtColor(ima1,sub1,CV_BGR2GRAY);



x1=0;
cont=0;
imax1=0;

    for (i = 0; i < cam->resy; ++i){cam->p[i].x=1024;cam->p[i].y=0;}

        for (i = 0; i < thr1.rows; ++i)
				{
					uchar * pixel1 = thr1.ptr<uchar>(i);

					for (j = 0; j <thr1.cols; ++j)
                    {
                        if((int)sub1.at<uchar>(i,j)>cam->GPLL && (int)sub1.at<uchar>(i,j)>imax1 &&pixel1[j]!=0 ){imax1=(int)sub1.at<uchar>(i,j);jmin=jmax;jmax=j;}
                    }
                    k=0;

                    if (jmin!=-1) jmax=((jmax-jmin)/2)+jmin; //trobo dos max
					for (j = jmax-cam->PAP; j <jmax+cam->PAP; ++j)
                    {


                if(j>0 && j<thr1.cols &&(int)sub1.at<uchar>(i,j)>cam->GPLL&&pixel1[j]!=0){
                            x[k]=(double)j;
                            y[k]=(double)sub1.at<uchar>(i,j);
                            cont=1;
                            k=k+1;

                            }

                    }
                    if (k>=cam->PMP){
                        polynomialfit(k, DEGREE, x, y, coeff);
//                    if (coeff[3]!=0)
//                        {x1=fabs((-2*coeff[2]+sqrt(4*coeff[2]*coeff[2]-4*(3*coeff[3]*coeff[1])))/(2*3*coeff[3]));
//                        if(abs(x1-jmax)>10)x1=jmax;
//                    }}
//                    else{x1=1024;}

                    if (coeff[2]!=0)
                        {x1=fabs(((-1*coeff[1])/(2*coeff[2])));
                        if(fabs(x1-jmax)>10)x1=1024;
                    }}
                    else{x1=1024;}

//

						if (cont==1 )
							{
							    //for (jj = 0; jj <k; jj++)x1=x1+xcan[jj];
							cont=0;
							if(x1>0 && x1<1024 && k>=cam->PMP){cam->p[i].x=x1;cam->p[i].y=i;cam->p[i].a=k;cam->p[i].q=k;}


							}
							x1=0;imax1=0;jmax=-1;jmin=-1;k=0;
				}

}



bool polynomialfit(int obs, int degree,
		   double *dx, double *dy, double *store) /* n, p */
{
  gsl_multifit_linear_workspace *ws;
  gsl_matrix *cov, *X;
  gsl_vector *y, *c;
  double chisq;

  int i, j;

  X = gsl_matrix_alloc(obs, degree);
  y = gsl_vector_alloc(obs);
  c = gsl_vector_alloc(degree);
  cov = gsl_matrix_alloc(degree, degree);

  for(i=0; i < obs; i++) {
    gsl_matrix_set(X, i, 0, 1.0);
    for(j=0; j < degree; j++) {
      gsl_matrix_set(X, i, j, pow(dx[i], j));
    }
    gsl_vector_set(y, i, dy[i]);
  }

  ws = gsl_multifit_linear_alloc(obs, degree);
  gsl_multifit_linear(X, y, c, cov, &chisq, ws);

  /* store result ... */
  for(i=0; i < degree; i++)
  {
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


int save_cam_data(Cam cam, const char *name)
{
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
    	fprintf (pFile, "%f\n",cam.Beta1);
    	fprintf (pFile, "%f\n",cam.Beta2);
    	fprintf (pFile, "%f\n",cam.xc);
    	fprintf (pFile, "%f\n",cam.yc);
    	fprintf (pFile, "%f\n",cam.l1);
    	fprintf (pFile, "%f\n",cam.L);
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

fclose(pFile);


}
int load_cam_data(Cam *cam, const char *name)
{
    FILE *pFile;
     pFile = fopen (name,"r");
  if (pFile!=NULL)
  {

        fscanf(pFile, "%d\n",&cam->DeviceId);
    	fscanf(pFile, "%d\n",&cam->VideoId);
        fscanf(pFile, "%d\n",&cam->resx);
    	fscanf(pFile, "%d\n",&cam->resy);
    	fscanf(pFile, "%s\n",cam->fcl1);
    	fscanf(pFile, "%s\n",cam->fcl2);


    	fscanf(pFile, "%d\n",&cam->Laser1);
    	fscanf(pFile, "%d\n",&cam->Laser2);
    	fscanf(pFile, "%f\n",&cam->alfa);
    	fscanf(pFile, "%f\n",&cam->Beta1);
    	fscanf(pFile, "%f\n",&cam->Beta2);
    	fscanf(pFile, "%f\n",&cam->xc);
    	fscanf(pFile, "%f\n",&cam->yc);
    	fscanf(pFile, "%f\n",&cam->l1);
    	fscanf(pFile, "%f\n",&cam->L);
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
    	fclose(pFile);
    	return (1);
  }else{
      fclose(pFile);return (0);}



}
