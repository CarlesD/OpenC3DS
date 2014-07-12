#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
////#include <stdlib.h>
//#include <stdio.h>
//#include <string.h>
//#include <termios.h>
//#include <unistd.h>
//#include <math.h>
//#include <fcntl.h>
//
//
//#include <iostream>
//
//#include <sstream>
//
//
//
//
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//
//
////#include <pcl/filters/statistical_outlier_removal.h>
//
//#include <stdio.h>
#include "ofxCv.h"
#include "polifitgsl.h"





#define m(ix,jx) ix+jx*Ncal
#define mc(ncolu,ix,jx) ix*ncolu+jx


using namespace cv;
using namespace ofxCv;
void cam_ini(Cam *cam);
int calibracio_cam(Cam *cam);
void cam_dis(Cam cam,int Laser,float x,int y,float *XXp, float *YYp);

int cam_cap_subpixel(Cam *cam,Mat ima1,Mat thr1);



int calibracio(Cam *cam);
void camera(Cam *cam,const char *CamFile);

void scan(Cam *cam,int *serialPort, ofImage *grislaser,ofImage *TaL,ofImage *TsL);

int Component_3D_LinScan(Cam cam, int Laser,ofImage Tot, Punts p[], float incx);
int cam_cap(Cam *cam,Mat thr1, Mat thr2);


int save_cam_data(Cam cam, const char *name);
int load_cam_data(Cam *cam, const char *name);
void contenidor(Cam cam, ofImage *image);
//
//typedef struct{adfloat x,y;
//
//
//} Punts2D;
////nombre de posicions màximes de calibració de les càmeres


//int n;


#define DEGREE 3



