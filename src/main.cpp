#include "ofApp.h"
#include "ofMain.h"
//#include "3Dscan.h" //COMENTARI Anna: no hauria d'estar aquí. Aquí només els de OF

//=======================================================================
int main( ){
	ofSetupOpenGL(1310,700,OF_WINDOW);			// <-------- setup the GL context

	// this kicks off the running of my app
	// pass in width and height too:
	ofRunApp(new ofApp());

}
