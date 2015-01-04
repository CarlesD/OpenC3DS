#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/filters/statistical_outlier_removal.h>

#include "ofMain.h"
#include "ofxUI.h"
#include "ofxCv.h"

#include "ofxOpenCv.h"
#include "Cam3D.h"

#include "SC.h" //COMENTARI Anna: venen del ofApp.cpp per endreçar
#include "3Dscan.h"
#include <iostream>
#include <string>

#define i_angular_1_axis  26.851239669 //relació transmissió un eix angular
#define Stepsxrevolution_angular_1_axis  6400 // Passos per volta transmissió un eix angular
#define Steps_div_degree_on_output_angular_1_axis i_angular_1_axis*Stepsxrevolution_angular_1_axis/360.0f  //passos/graus

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
        float PosAxis1,PosAxis1_ant, dist_scan_max, dist_scan_min;

        bool Scan,Manual,Axis1_Left_Button,Axis1_Right_Button,Axis1;
        bool CartessianXAxis,CylindricalPhiAxis,SphericalPhiZhetaAxis;

        Cam cam3d;
        Punts punts[2000];
        int s=-1;
        int nt=0;

        pcl::PointCloud<pcl::PointXYZRGBNormal> cloud, cloudaux,clouderr;

        ofxUITextInput *CamFile, *SerialPort,*textInput,*PCDFile,*CamExp,*CamFocus,*VideoNum,*KSW,*KSH,*SX,*SY,*WBT,*GAIN;
        ofImage grisl,TaL,TsL,pview;

        void setGUI1();
        void setGUI2();
        void setGUI3();
        void setGUI4();
        void setGUI5();

        ofxUISuperCanvas *gui1;
        ofxUISuperCanvas *gui2;
        ofxUISuperCanvas *gui3;
        ofxUISuperCanvas *gui4;
        ofxUISuperCanvas *gui5;

        ofVideoGrabber cam;

        void Run_Scan();

        void reset_scan(Punts p[]);
        void check_scan(Punts p[],Punts pok[], int *n);
        void fill_cloud(Punts pok[], int n, int nt);
        void SavePointCloud();
        void ResetPointCloud();

        bool mp,Pview,Zoom,Fast_Calibration;
        float Fast_Calibration_Constant, Yfocus, Xfocus;
        int X,Y,zoom;
        ofSerial	serial;
        int baud;
};
