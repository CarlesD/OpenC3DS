#pragma once

#include "ofMain.h"
#include "ofxUI.h"
#include "ofxXmlSettings.h"

#include "polifitgsl.h"

#define m(ix,jx)            ix+jx*Ncal
#define mc(ncolu,ix,jx)     ix*ncolu+jx
#define DEGREE              3 // grau del polinomi

#define MAX_NUM_LASERS      4
#define MAX_NUM_PUNTS       2500

typedef struct{
    float x;
    int y;
    int q; // qualitat mesura
    int a; // amplada linea laser

}Punts2D_subpix;

typedef struct{
    float x, y, z; //coordenades respecte la referència càmera
    int r, g, b; //color r g b
    int q; //qualitat mesura
    int a; //amplada linea laser
    float nx, ny, nz;// normals
}Punts; // mesura de cada punt escanejat

class openC3DSprocess{

	public:
		void setup();
		void setupCamResolution(int w, int h);
		bool update();

		void draw();
		void exit();

        bool polynomialfit(int obs, int degree, double *dx, double *dy, double *store);
        bool camCaptureSubpixelProcess(unsigned char* pixelsRaw);

        // GUI
		ofxUISuperCanvas *guiProcess;
		void setGuiProcess();
		void guiEvent(ofxUIEventArgs &e);

		// XML
		ofxXmlSettings xmlSettings;

		// SENSOR
		Punts2D_subpix *p; // punts de la càmera amb precisió subpixel en x
		Punts punts[MAX_NUM_PUNTS];

		int _camWidth;
		int _camHeight;

		int numLasers;

        //Paràmetres dels Lasers
        // zita -> zita_0 // angle obertura càmera horitzontal
        // alpha -> alpha_0 // angle obertura càmera vertical
        // LINK a l'esquema amb la limona
        float beta[MAX_NUM_LASERS], xc[MAX_NUM_LASERS], yc[MAX_NUM_LASERS];
        float LA[MAX_NUM_LASERS], LB[MAX_NUM_LASERS];
        float alfa[MAX_NUM_LASERS], zita[MAX_NUM_LASERS];
        float m[MAX_NUM_LASERS]; //pendent linea calibració
        float FCC[MAX_NUM_LASERS]; //constant de la calibració rapida
        bool laserSide[MAX_NUM_LASERS]; //identificació posició làser esquerra(0) dreta (1)

        //Paràmetres del sensor
        int GPLL; // llindar de nivell de gris que ja no es considera soroll
        int PAP; // amplitud de punts que s'agafa per construir la corba
        int PMP; // nombre mínim de punts per a generar la paràbola

        float L; //Distància fins el centre de rotació
};
