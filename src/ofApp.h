#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/filters/statistical_outlier_removal.h>

#include "ofMain.h"
#include "ofxUI.h"
#include "ofxCv.h"

#include "ofxOpenCv.h"
#include "Cam3D.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
		void exit();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
        void drawGrid(float x, float y);

        bool hideGUI;
        float red, green, blue;
        bool bdrawGrid;
        bool bdrawPadding;
        void guiEvent(ofxUIEventArgs &e);

        float Ri,Gi,Bi,Rs,Gs,Bs,GPLL,PAP,PMP;
        float Lto,Sto;
        int IncAxis1_Steps;
        float IniAxis1,FiAxis1;
        float IncAxis1;
        float PosAxis1,PosAxis1_ant;

        bool Scan,Laser,Manual,Axis1_Left_Button,Axis1_Right_Button,Axis1;
        bool CartessianXAxis,CylindricalPhiAxis,SphericalPhiZhetaAxis;

        Cam cam3d;
        Punts punts[1300];
        int s=-1;
        int nt=0;

        pcl::PointCloud<pcl::PointXYZRGBNormal> cloud, cloudaux,clouderr;

        ofxUITextInput *CamFile, *SerialPort,*textInput,*PCDFile,*CamExp,*CamFocus,*VideoNum,*KSW,*KSH,*SX,*SY,*WBT;
        ofImage grisl,TaL,TsL,pview;

        void setGUI1();
        void setGUI2();
        void setGUI3();

        ofxUISuperCanvas *gui1;
        ofxUISuperCanvas *gui2;
        ofxUISuperCanvas *gui3;
        ofVideoGrabber cam;

        void Run_Scan();
        void capture_image(ofImage *image);

        void reset_scan(Punts p[]);
        void check_scan(Punts p[],Punts pok[], int *n);
        void fill_cloud(Punts pok[], int n, int nt);
        void SavePointCloud();
        void ResetPointCloud();

        bool mp,Pview;
        int X,Y,zoom;
ofSerial	serial;
int baud;
};
