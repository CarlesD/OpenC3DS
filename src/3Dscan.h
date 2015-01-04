#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include "ofxCv.h"
#include "polifitgsl.h"

#define m(ix,jx) ix+jx*Ncal
#define mc(ncolu,ix,jx) ix*ncolu+jx

using namespace cv;
using namespace ofxCv;

//COMENTARI Anna: 3DScan ha de ser una classe?

void cam_dis(Cam cam, int Laser, float x, int y, float *XXp, float *YYp);

int cam_cap_subpixel(Cam *cam, Mat ima1, Mat thr1);
int cam_cap(Cam *cam, Mat thr1, Mat thr2);

int calibracio_cam(Cam *cam);
void camera(Cam *cam, const char *CamFile);

void scan(Cam *cam, ofImage *grislaser, ofImage *TaL, ofImage *TsL);

int Component_3D_LinScan(Cam cam, int Laser, ofImage Tot, Punts p[], float incx);
int Component_3D_Angular_1_axis_Scan(Cam cam, int Laser, ofImage Tot, Punts p[], float phi);

int save_cam_data(Cam cam, const char *name);
int load_cam_data(Cam *cam, const char *name);
void free_camera(Cam *cam);


#define DEGREE 3
