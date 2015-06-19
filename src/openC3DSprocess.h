#pragma once

#undef Success
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "ofMain.h"
#include "ofxUI.h"
#include "ofxXmlSettings.h"

#include "polifitgsl.h"

#define m(ix,jx)            ix+jx*Ncal
#define mc(ncolu,ix,jx)     ix*ncolu+jx
#define DEGREE              3 // grau del polinomi

#define MAX_NUM_LASERS      4

typedef struct{
    float x;
    int y;
    int q; // qualitat mesura
    int a; // amplada linea laser
}points2DSubpixelPrecision;

typedef struct{
    float x, y, z; //coordenades respecte la referència càmera
    float r, g, b; //color r g b
    int q; //qualitat mesura
    int a; //amplada linea laser
    float nx, ny, nz;// normals
}points3D; // mesura de cada punt escanejat

class openC3DSprocess{

	public:
		void setup();
		void setupCamResolution(int w, int h);
		bool update();

		void draw();
		void exit();

        bool polynomialfit(int obs, int degree, double *dx, double *dy, double *store);
        bool camCaptureSubpixelProcess(unsigned char* pixelsRaw);
        bool Component_3D_Angular_1_axis_Scan(int currentLaser, unsigned char* pixelsRaw, float phi);
        void cam_dis(int currentLaser, float x, int yp, float *XXp, float *YYp);

        // GUI
		ofxUISuperCanvas *guiProcess;
		void setGuiProcess();
		void guiEvent(ofxUIEventArgs &e);

		// XML
		ofxXmlSettings xmlSettings;

		// SENSOR
		vector <points2DSubpixelPrecision> laserLineSubpixelPoints; // punts de la càmera amb precisió subpixel en x
		ofImage imgLaserLineSubpixel;

		vector <points3D> points3Dscanned;
		int indexPixColorX;
		int indexPixColorY;

		ofMesh mesh;
		ofMesh origin;
		ofEasyCam cam;
		ofLight light;

		int _camWidth;
		int _camHeight;

		int numLasers;

		// POINT CLOUD
		pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
		void fillPointCloud();
		void savePointCloud();
        void resetPointCloud();
        ofxUITextInput *PCDfilename;

        //Paràmetres dels Lasers
        // zita -> zita_0 // angle obertura càmera horitzontal
        // alpha -> alpha_0 // angle obertura càmera vertical
        // LINK a l'esquema amb la limona
        float beta[MAX_NUM_LASERS], xc[MAX_NUM_LASERS], yc[MAX_NUM_LASERS];
        float LA[MAX_NUM_LASERS], LB[MAX_NUM_LASERS];
        float alfa[MAX_NUM_LASERS], zita[MAX_NUM_LASERS];
        float m[MAX_NUM_LASERS]; //pendent linea calibració
        float FCC[MAX_NUM_LASERS]; //constant de la calibració rapida
        int laserID[MAX_NUM_LASERS]; //identificació posició làser

        //Paràmetres del sensor
        float GPLL; // llindar de nivell de gris que ja no es considera soroll (int)
        float PAP; // amplitud de punts que s'agafa per construir la corba (int)
        float PMP; // nombre mínim de punts per a generar la paràbola (int)

        float L; //Distància fins el centre de rotació

        // DEBUG
		unsigned char* colorPixelsRaw;
        ofImage imgRawforDebug;
};
