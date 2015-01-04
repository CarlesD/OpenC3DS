#include <string>
#include <iostream>

typedef struct{
    float x;
    int y;
    int q; //qualitat mesura
    int a; //amplada linea laser

}Punts2D_subpix;

typedef struct{
    int x, y;
    float d, a, b;
    int n;
}Calibracio;

typedef struct{
    int Laser1, Laser2; //identificació làser
    int Alaser1, Alaser2; // Làser on= 1, làser off=0
    int Nlaser; //nombre de lasers
    int Tlaser1, Tlaser2; //Tipologia linea vertical o horitzontal
    float ax, ay, az; //angles Euler de la càmera
    float Xo, Yo, Zo; //possició absoluta del centre de coordenades de la càmera
    int Ri, Gi, Bi, GPLL;
    int Rs, Gs, Bs, PAP, PMP;
    int DeviceId;
    int VideoId;
    int resx, resy;
    char fcl1[50];
    char fcl2[50];
    int blur_ksizew, blur_ksizeh, blur_sigmax, blur_sigmay; // paràmetres del blur
    Punts2D_subpix *p; // punts de la càmera amb precisió subpixel en x
    Calibracio* c1;
    Calibracio* c2;
    int nc1, nc2, resyc1, resyc2; // Valors dels fitxers de configuració dels làsers
    float Beta1, Beta2, xc, yc, LA, LB, alfa,zita;
    float m; //pendent linea calibració
    float FCC; //constant de la calibració rpida
    float L; //Distància fins el centre de rotació
    int Calibration; //0 amb fitxers de calibració; 1: amb autocalibració
}Cam;

typedef struct{
    float x, y, z; //coordenades respecte la referència càmera
    int r, g, b; //color r g b
    int q; //qualitat mesura
    int a; //amplada linea laser
    float nx, ny, nz;// normals
}Punts;


#define Ncal    50
#define PI 3.141592654
