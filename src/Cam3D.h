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
    char *fcl1 = (char *)NULL;
    char *fcl2 = (char *)NULL;
    int blur_ksizew, blur_ksizeh, blur_sigmax, blur_sigmay;
    Punts2D_subpix *p;
    Calibracio* c1;
    Calibracio* c2;
    int nc1, nc2, resxc1, resxc2;
    float Beta1, Beta2, xc, yc, L, l1, alfa;
}Cam;

typedef struct{
    float x, y, z; //coordenades respecte la referència càmera
    int r, g, b; //color r g b
    int q; //qualitat mesura
    int a; //amplada linea laser
    float nx, ny, nz;// normals
}Punts;

#define Npunts 2200000
#define Res_x    1024
#define Res_y    576
#define Ncal    50
#define PI 3.141592654
