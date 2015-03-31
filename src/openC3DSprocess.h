#pragma once

#include "ofMain.h"
#include "ofxUI.h"
#include "ofxOpenCv.h"

#include "polifitgsl.h"

#define m(ix,jx)            ix+jx*Ncal
#define mc(ncolu,ix,jx)     ix*ncolu+jx
#define DEGREE              3 // grau del polinomi

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
        bool cam_cap_subpixel(unsigned char* pixelsRaw);

        // GUI
		ofxUISuperCanvas *guiProcess;
		void setGuiProcess();
		void guiEvent(ofxUIEventArgs &e);

		// SENSOR
		Punts2D_subpix *p; // punts de la càmera amb precisió subpixel en x

		Punts punts[2500]; // TODO posar aquest valor max en un define

		int _camWidth;
		int _camHeight;

        //Paràmetres dels Lasers
        // zita -> zita_0 // angle obertura càmera horitzontal
        // alpha -> alpha_0 // angle obertura càmera vertical
        // LINK a l'esquema amb la limona
        float Beta[4], xc[4], yc[4], LA[4], LB[4], alfa[4], zita[4]; // TODO there are 4 because we have 4 lasers
        float m[4]; //pendent linea calibració
        float FCC[4]; //constant de la calibració rapida
        bool Laser_side[4]; //identificació posició làser esquerra(0) dreta (1)

        //Paràmetres del sensor
        int GPLL; // llindar de nivell de gris que ja no es considera soroll
        int PAP; // amplitud de punts que s'agafa per construir la corba
        int PMP; // nombre mínim de punts per a generar la paràbola

        float L; //Distància fins el centre de rotació
};
