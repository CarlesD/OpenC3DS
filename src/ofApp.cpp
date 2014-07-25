#include "ofApp.h"
//#include "SC.h" //COMENTARI Anna: cap al .h per endreçar
//#include "3Dscan.h"
//#include <iostream>
//#include <string>
using namespace cv;
using namespace ofxCv;
int serialPort;
const char* portName="/dev/ttyACM3";

//--------------------------------------------------------------
void ofApp::setup()
{
    //font.loadFont("DIN.otf", 64);

	serial.listDevices();
	vector <ofSerialDeviceInfo> deviceList = serial.getDeviceList();

    unsigned char HOME[8]={'1',' ','1',' ','9',' ','0','\n'};
    red = 100; blue = 200; green = 27;

    hideGUI      = false;
    bdrawGrid    = false;
    bdrawPadding = false;
    CamFile = NULL;
    SerialPort = NULL;

    setGUI1();
    setGUI2();
    setGUI3();
    setGUI4();
    //serial.listDevices();
    //vector <ofSerialDeviceInfo> deviceList = serial.getDeviceList();
	baud = 115200;
    //openPort("/dev/ttyACM3",baud);
    serial.setup("/dev/ttyACM3", baud);

    //while(serial.available()<0){serial.writeByte('7');usleep(10000);}

    //serial.writeByte('5');
    //serial.writeByte(' ');
    //serial.writeByte('1');
    //serial.writeByte(' ');
    //serial.writeByte('1');
    //serial.writeByte(' ');
    //serial.writeByte('1');
    //serial.writeByte('\n');

    //unsigned char buf[4]={'5','1','1','1'};
    //serial.writeBytes(&buf[0], 3);

    //serial.writeByte('8');
    //sleep(1);
    //serial.writeByte('9');

    gui1->loadSettings("gui1.xml");
    gui2->loadSettings("gui2.xml");
    gui3->loadSettings("gui3.xml");
    gui4->loadSettings("gui4.xml");

    camera(&cam3d,CamFile->getTextString().c_str());
    serial.setup(SerialPort->getTextString().c_str(), baud);
    serial.flush(true,true);
    //Serial_setup(&serialPort,SerialPort->getTextString().c_str());
    //serialPort=openPort(SerialPort->getTextString().c_str(), 115200);
    //cam_laser(1,0,&serialPort,Lto);


    PosAxis1=-1;
    CartessianXAxis=true;
    Scan=false;

    Axis1_Left_Button=false;
    Axis1_Right_Button=false;


    string imagePath = "mars.jpg";
    ofLoadImage(grisl, imagePath);
    ofLoadImage(TsL, imagePath);
    ofLoadImage(TaL, imagePath);
    grisl.loadImage(imagePath);
    TaL.loadImage(imagePath);
    TsL.loadImage(imagePath);

    ofSetVerticalSync(true);
    cam.setDesiredFrameRate(30);
    cam.setDeviceID(cam3d.DeviceId);
    cam.initGrabber(cam3d.resx,cam3d.resy);

    cam.listDevices();

    cam3d.blur_ksizew = atoi( KSW->getTextString().c_str());
    cam3d.blur_ksizeh = atoi( KSH->getTextString().c_str());
    cam3d.blur_sigmax = atoi( SX->getTextString().c_str());
    cam3d.blur_sigmay = atoi( SY->getTextString().c_str());

    reset_scan(punts);
    zoom=1;
    Pview=false;
    Laser(1,0,&serial,Lto);
}

//--------------------------------------------------------------
void ofApp::exit()
{


    gui1->saveSettings("gui1.xml");
    gui2->saveSettings("gui2.xml");
    gui3->saveSettings("gui3.xml");
    gui4->saveSettings("gui4.xml");
    delete gui1;
	delete gui2;
	delete gui3;
	delete gui4;
    Laser(1,0,&serial,Lto);



    //save_cam_data(cam3d,"HD525.cam");

}

//--------------------------------------------------------------
void ofApp::guiEvent(ofxUIEventArgs &e)
{
	string name = e.getName();
	int kind = e.getKind();

	cout << "got event from: " << name << endl;



	if(name == "SCAN")
        {
		Scan=true;
        }

    else if(name == "Cartesian X Axis")
        {
            CartessianXAxis=true;
            CylindricalPhiAxis=false;
            SphericalPhiZhetaAxis=false;
            Laser(1,0,&serial,Lto);
        }
    else if(name == "Cylindrical Phi Axis")
        {
            CartessianXAxis=false;
            CylindricalPhiAxis=true;
            SphericalPhiZhetaAxis=false;
        }
    else if(name == "Spherical Phi Zheta Axis")
        {
            CartessianXAxis=false;
            CylindricalPhiAxis=false;
            SphericalPhiZhetaAxis=true;
        }
    else if(name == "Apply Blur")
        {

        cam3d.blur_ksizew=atoi( KSW->getTextString().c_str());
        cam3d.blur_ksizeh=atoi( KSH->getTextString().c_str());
        cam3d.blur_sigmax=atoi( SX->getTextString().c_str());
        cam3d.blur_sigmay=atoi( SY->getTextString().c_str());
        cout << cam3d.blur_ksizew << endl;
        cout << cam3d.blur_ksizeh << endl;
        cout << cam3d.blur_sigmax << endl;
        cout << cam3d.blur_sigmay << endl;
        }
    else if(name == "Apply")
        {
        cam.close();
        string com1="uvcdynctrl -v -d video"+VideoNum->getTextString()+" --set='Focus, Auto' 0";
        system(com1.c_str());
        string com2="uvcdynctrl -v -d video"+VideoNum->getTextString()+" --set='Focus (absolute)' "+CamFocus->getTextString();
        system(com2.c_str());

        string com3="uvcdynctrl -d video"+VideoNum->getTextString()+" -s 'Exposure, Auto' 1";
        system(com3.c_str());

        string com4="uvcdynctrl -d video"+VideoNum->getTextString()+" -s 'Exposure (Absolute)' "+CamExp->getTextString();
        system(com4.c_str());

        string com5="uvcdynctrl -d /dev/video"+VideoNum->getTextString()+" -s 'White Balance Temperature' "+WBT->getTextString();
        system(com5.c_str());

        string com6="uvcdynctrl -d /dev/video"+VideoNum->getTextString()+" -s 'White Balance Temperature, Auto' 0";
        system(com6.c_str());

        string com7="uvcdynctrl -d /dev/video"+VideoNum->getTextString()+" -s 'Gain' 0"+GAIN->getTextString();
        system(com7.c_str());
        ofSetVerticalSync(true);
        cam.setDesiredFrameRate(30);
        cam.setDeviceID(cam3d.DeviceId);
        cam.initGrabber(cam3d.resx,cam3d.resy);

        cam.listDevices();
        }
    else if(name == "SCAN STOP")
        {
        Axis1=false;
		Scan=false;
        }
	else if(name == "Save Point Cloud"&&mp==true)
        {
        SavePointCloud();
        }
    else if(name == "Reset Point Cloud"&&mp==true)
        {
        ResetPointCloud();
        }
	else if(name == "Manual mode")
        {
		ofxUIToggle *toggle = (ofxUIToggle *) e.getToggle();
		if(toggle->getValue()==true){ Manual=true;}
		else{Manual=false;}
        }
    else if(name == "Zoom mouse click")
        {
		ofxUIToggle *toggle = (ofxUIToggle *) e.getToggle();
		if(toggle->getValue()==true){ Zoom=true;}
		else{Zoom=false;}
        }
	else if(name == "<--Left"&&mp==true)
        {
		Axis1_Left_Button=true;
        Axis1_Right_Button=false;
        }
	else if(name == "Right-->"&&mp==true)
        {
		Axis1_Left_Button=false;
        Axis1_Right_Button=true;

        }
    else if(mp==false)
        {
		Axis1_Left_Button=false;
        Axis1_Right_Button=false;
        }
    else if(name == "Home"&&mp==true)
        {
		Axis1_Left_Button=false;
        Axis1_Right_Button=false;
        PosAxis1=0;
        Scan=false;
        Stepper(1,1,0,1, &serial,(unsigned int)Sto);
        }
    else if(name == "Load Camera"&&mp==true)
        {
        camera(&cam3d,CamFile->getTextString().c_str());
        }
	else if(name == "CAM FILE")
        {
        ofxUITextInput *ti = (ofxUITextInput *) e.widget;



        if(ti->getInputTriggerType() == OFX_UI_TEXTINPUT_ON_ENTER)
            {

            cout << "ON ENTER: ";
            }
        else if(ti->getInputTriggerType() == OFX_UI_TEXTINPUT_ON_FOCUS)
            {
            cout << "ON FOCUS: ";
            }
        else if(ti->getInputTriggerType() == OFX_UI_TEXTINPUT_ON_UNFOCUS)
            {
            cout << "ON BLUR: ";
            }
        string output = ti->getTextString();
        cout << output << endl;
        }

    else if(name == "SERIAL PORT")
        {
        ofxUITextInput *SerialPort = (ofxUITextInput *) e.widget;
        if(SerialPort->getTriggerType() == OFX_UI_TEXTINPUT_ON_ENTER)
            {
            //Serial_setup(&serialPort,SerialPort->getTextString().c_str());
            //serialPort=openPort(SerialPort->getTextString().c_str(), 115200);
            SerialPort->update();
            serial.setup(SerialPort->getTextString().c_str(), baud);
            cout << "ON ENTER: ";
            }
        else if(SerialPort->getTriggerType() == OFX_UI_TEXTINPUT_ON_FOCUS)
            {
            cout << "ON FOCUS: ";
            }
        else if(SerialPort->getTriggerType() == OFX_UI_TEXTINPUT_ON_UNFOCUS)
            {
            cout << "ON BLUR: ";
            }
        string output = SerialPort->getTextString();
        cout << output << endl;
        //serialPort=openPort(SerialPort->getTextString().c_str(), 115200);
        SerialPort->update();
        serial.setup(SerialPort->getTextString().c_str(), baud);
        }
    else if(name == "POINT CLOUD FILE")
        {
        ofxUITextInput *PCDFile = (ofxUITextInput *) e.widget;
        if(PCDFile->getTriggerType() == OFX_UI_TEXTINPUT_ON_ENTER)
            {
            cout << "ON ENTER: ";
            }
        else if(PCDFile->getTriggerType() == OFX_UI_TEXTINPUT_ON_FOCUS)
            {
            cout << "ON FOCUS: ";
            }
        else if(PCDFile->getTriggerType() == OFX_UI_TEXTINPUT_ON_UNFOCUS)
            {
            cout << "ON BLUR: ";
            }
        string output = PCDFile->getTextString();
        cout << output << endl;
    }
}

//--------------------------------------------------------------
void ofApp::SavePointCloud()
{
    pcl::io::savePCDFileASCII (PCDFile->getTextString()+".pcd", cloud);
    std::cerr << "Saved " << cloud.points.size () << " data points to "+PCDFile->getTextString()+".pcd"  << std::endl;
    pcl::io::savePCDFileASCII (PCDFile->getTextString()+"err.pcd", clouderr);
    std::cerr << "Saved " << cloud.points.size () << " data points to "+ PCDFile->getTextString()+"err.pcd"<< std::endl;

}

//--------------------------------------------------------------
void ofApp::ResetPointCloud()
{
    nt=0;
    cloud.points.resize (0);
    clouderr.points.resize (0);
}

//--------------------------------------------------------------
void ofApp::update(){
    cam.update();

    cam3d.Ri=(int)Ri;
    cam3d.Gi=(int)Gi;
    cam3d.Bi=(int)Bi;
    cam3d.Rs=(int)Rs;
    cam3d.Gs=(int)Gs;
    cam3d.Bs=(int)Bs;
    cam3d.GPLL=(int)GPLL;
    cam3d.PAP=(int)PAP;
    cam3d.PMP=(int)PMP;
    pview.update();

	if(cam.isFrameNew()&&Scan==true)
        {
            if (CartessianXAxis==true)
            {
                if(s==1)
                {
                     copy(cam, TaL);
                     Laser(1,0,&serial,(int)Lto);
                     s=-1;
                }
                else
                {
                    s=1;
                    copy(cam, TsL);
                    Laser(1,1,&serial,(int)Lto);
                    TsL.update();
                    TaL.update();
                    Run_Scan();
                    //contenidor(cam3d, &grisl);
                    grisl.update();
                }
            }

        }

    if(Scan==false && Manual ==true)
        {
        int sense=0;
        if(Axis1_Left_Button==true && PosAxis1>0){sense=-1; Stepper(1,(180/PI)*(sense*IncAxis1/6),0,0, &serial,(int)Sto);}
        if(Axis1_Right_Button==true){sense=1;Stepper(1,(180/PI)*(sense*IncAxis1/6),0,0, &serial,(int)Sto);}
        PosAxis1=PosAxis1+sense*IncAxis1;
        }


}

//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(red, green, blue, 255);
    ofPushStyle();
    ofEnableBlendMode(OF_BLENDMODE_ALPHA);
    if(bdrawGrid)
        {
        ofSetColor(255, 255, 255, 25);
        drawGrid(8,8);
        }
    //ofScale(0.5,1,1);
    if(Pview==false)
        {
        cam.draw(210,500,295,166);
        //grisl.draw(210,0,890,500);
        grisl.drawSubsection(210,0,890,500,X,Y,1024/zoom,576/zoom);
    }
    else
        {
        grisl.draw(210,500,295,166);
        //grisl.draw(210,0,890,500);

        copy(cam,pview);
        pview.drawSubsection(210,0,890,500,0,0,1024,576);
        }

    TaL.draw(508,500,295,166);
    TsL.draw(805,500,295,166);


}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    switch (key)
	{
		case 'f':
			ofToggleFullscreen();
			break;

		case 'h':
            gui1->toggleVisible();
            gui2->toggleVisible();
            gui3->toggleVisible();
            gui4->toggleVisible();
			break;

		case 'p':
			bdrawPadding = !bdrawPadding;
			gui1->setDrawWidgetPaddingOutline(bdrawPadding);
			gui2->setDrawWidgetPaddingOutline(bdrawPadding);
			break;

		case '[':
			gui1->setDrawWidgetPadding(false);
			gui2->setDrawWidgetPadding(false);

			break;

		case ']':
			gui1->setDrawWidgetPadding(true);
			gui2->setDrawWidgetPadding(true);

			break;

        case OF_KEY_F1:
            gui1->toggleVisible();
            break;

        case OF_KEY_F2:
            gui2->toggleVisible();
            break;

        case OF_KEY_F3:
            gui3->toggleVisible();
            break;
        case OF_KEY_F4:
            gui4->toggleVisible();
            break;
		default:
			break;
	}

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    //  int mx = x % w;
    //    int my = y % h;
    //
    //    //get hue value on mouse position
    //    findHue = hue.getPixels()[my*w+mx];
    mp=true;
    //grisl.draw(210-890*(zoom*0.5),500*(zoom*0.5),890*zoom,500*zoom);
    if(x>210&& y>0 &&x<(210+890)&&y<500&button==0&&Zoom==true) {zoom=zoom+1;X=x-210-(445/zoom);Y=y-0-(250/zoom);}
    if(x>210&& y>0 &&x<(210+890)&&y<500&button==2&&Zoom==true) {zoom=zoom-1;if(zoom<1){zoom=1;X=0;Y=0;}else{X=x-210-(445/zoom);Y=y-0-(250/zoom);}}
    if(x>210&& y>500 &&x<(210+295)&&y<(500+166)&button==0) {if(Pview==true){Pview=false;}else{Pview=true;}}
    cam.draw(210,500,295,166);

    //cout<<button<<endl;
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    mp=false;
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}

//--------------------------------------------------------------
void ofApp::setGUI1()
{

	gui1 = new ofxUISuperCanvas("SCAN CONFIG");
    gui1->addSpacer();
    gui1->addLabel("CAMERA", OFX_UI_FONT_MEDIUM);
    gui1->addLabel("Camera config file:", OFX_UI_FONT_SMALL);
	gui1->setWidgetFontSize(OFX_UI_FONT_SMALL);
    CamFile = gui1->addTextInput("CAM FILE", "Input Text");
    CamFile->setAutoUnfocus(false);
    CamFile->setAutoClear(false);
    gui1->addButton( "Load Camera", false);
    gui1->addLabel("Serial port:", OFX_UI_FONT_SMALL);
    SerialPort = gui1->addTextInput("SERIAL PORT", "Serial port");
    SerialPort->setAutoUnfocus(false);
    SerialPort->setAutoClear(false);

    gui1->addSpacer();
    vector<string> vnames; vnames.push_back("Cartesian X Axis"); vnames.push_back("Cylindrical Phi Axis"); vnames.push_back("Spherical Phi Zheta Axis");
    gui1->addLabel("SCAN TYPE", OFX_UI_FONT_MEDIUM);
    gui1->setWidgetFontSize(OFX_UI_FONT_SMALL);
    ofxUIRadio *radio = gui1->addRadio("SCANTYPE", vnames, OFX_UI_ORIENTATION_VERTICAL);
    radio->activateToggle("Cartesian X Axis");


    gui1->addLabel("", OFX_UI_FONT_LARGE);
    gui1->addSpacer();
    gui1->addButton( "SCAN", false);

    gui1->addButton( "SCAN STOP", false);
    gui1->addSpacer();
    gui1->addLabel("Point Cloud File:", OFX_UI_FONT_SMALL);
   	gui1->setWidgetFontSize(OFX_UI_FONT_SMALL);
    PCDFile = gui1->addTextInput("POINT CLOUD FILE", "scan.pcd");
    PCDFile->setAutoUnfocus(false);
    PCDFile->setAutoClear(false);

    gui1->addButton( "Save Point Cloud", false);
    gui1->addButton( "Reset Point Cloud", false);

    gui1->addSpacer();
    gui1->setWidgetFontSize(OFX_UI_FONT_SMALL);
    gui1->addLabel("RANGE AXIS", OFX_UI_FONT_SMALL);

    gui1->addLabel("AXIS 1", OFX_UI_FONT_SMALL);
	gui1->addRangeSlider("RAXIS1", 0, 500,&IniAxis1,&FiAxis1);
    gui1->addLabel("AXIS 2", OFX_UI_FONT_SMALL);
	gui1->addRangeSlider("RAXIS2", 0.0, 255.0, 50.0, 100.0);
    gui1->addLabel("AXIS 3", OFX_UI_FONT_SMALL);
	gui1->addRangeSlider("RAXIS3", 0.0, 255.0, 50.0, 100.0);
    gui1->addSpacer();

    gui1->setWidgetFontSize(OFX_UI_FONT_SMALL);
	gui1->addLabel("AXIS INCREMENT", OFX_UI_FONT_SMALL);
	gui1->addSlider("IAXIS1", 0, 5, &IncAxis1);
	gui1->addSlider("IAXIS2", 0.0, 25, green);
	gui1->addSlider("IAXIS3", 0.0, 25, blue);


    gui1->autoSizeToFitWidgets();
	ofAddListener(gui1->newGUIEvent,this,&ofApp::guiEvent);
}

//--------------------------------------------------------------
void ofApp::setGUI2()
{
    gui2 = new ofxUISuperCanvas("SCAN ADJUST");

    gui2->addSpacer();
    gui2->setWidgetFontSize(OFX_UI_FONT_SMALL);
    gui2->addLabel("RANGE R G B", OFX_UI_FONT_SMALL);


    gui2->setWidgetFontSize(OFX_UI_FONT_SMALL);
	gui2->addRangeSlider("R", 0, 255, &Ri, &Rs);
	gui2->addRangeSlider("G", 0.0, 255, &Gi, &Gs);
	gui2->addRangeSlider("B", 0.0, 255, &Bi,&Bs);
    gui2->addSpacer();

    gui2->addLabel("Scan distance", OFX_UI_FONT_SMALL);

    gui2->setWidgetFontSize(OFX_UI_FONT_SMALL);
	gui2->addRangeSlider("Min Max", 0.0, 1000, &dist_scan_min, &dist_scan_max);

    gui2->addSpacer();
    gui2->addLabel("PEAK CONTROL", OFX_UI_FONT_SMALL);
    gui2->addLabel("Gray peak lower limit", OFX_UI_FONT_SMALL);
    gui2->setWidgetFontSize(OFX_UI_FONT_SMALL);
    gui2->addSlider("Gray PLL", 0, 255, &GPLL);
    //gui2->addSpacer();
    gui2->addLabel("Peak amplitude points:", OFX_UI_FONT_SMALL);
    gui2->setWidgetFontSize(OFX_UI_FONT_SMALL);
    gui2->addSlider("Peak AP", 0, 50, &PAP);
    //gui2->addSpacer();
    gui2->addLabel("Peak minimun points:", OFX_UI_FONT_SMALL);
    gui2->addSlider("Peak MP", 3, 25, &PMP);

    gui2->addSpacer();

    gui2->addLabel("BLUR CONTROL", OFX_UI_FONT_SMALL);
    gui2->addLabel("K size width:", OFX_UI_FONT_SMALL);
   	gui2->setWidgetFontSize(OFX_UI_FONT_SMALL);
    KSW= gui2->addTextInput("ksw", "5");
    KSW->setAutoUnfocus(false);
    KSW->setAutoClear(false);

    gui2->addLabel("K size height:", OFX_UI_FONT_SMALL);
   	gui2->setWidgetFontSize(OFX_UI_FONT_SMALL);
    KSH = gui2->addTextInput("ksh", "80");
    KSH->setAutoUnfocus(false);
    KSH->setAutoClear(false);

    gui2->addLabel("Sigma X:", OFX_UI_FONT_SMALL);
   	gui2->setWidgetFontSize(OFX_UI_FONT_SMALL);
    SX = gui2->addTextInput("sigmax", "600");
    SX->setAutoUnfocus(false);
    SX->setAutoClear(false);

    gui2->addLabel("Sigma Y:", OFX_UI_FONT_SMALL);
   	gui2->setWidgetFontSize(OFX_UI_FONT_SMALL);
    SY = gui2->addTextInput("sigmay", "600");
    SY->setAutoUnfocus(false);
    SY->setAutoClear(false);
    gui2->addButton( "Apply Blur", false);



//    gui2->addSpacer();
//
//    gui2->addLabel("Video cam number:", OFX_UI_FONT_SMALL);
//   	gui2->setWidgetFontSize(OFX_UI_FONT_SMALL);
//    VideoNum = gui2->addTextInput("Videonum", "2");
//    VideoNum->setAutoUnfocus(false);
//    VideoNum->setAutoClear(false);
//
//    gui2->addLabel("Focus:", OFX_UI_FONT_SMALL);
//   	gui2->setWidgetFontSize(OFX_UI_FONT_SMALL);
//    CamFocus = gui2->addTextInput("Camera Focus", "80");
//    CamFocus->setAutoUnfocus(false);
//    CamFocus->setAutoClear(false);
//
//    gui2->addLabel("White Balance Temperature:", OFX_UI_FONT_SMALL);
//    gui2->setWidgetFontSize(OFX_UI_FONT_SMALL);
//    WBT = gui2->addTextInput("WBT value", "2800");
//    WBT->setAutoUnfocus(false);
//    WBT->setAutoClear(false);;
//
//    gui2->addLabel("Exposition:", OFX_UI_FONT_SMALL);
//   	gui2->setWidgetFontSize(OFX_UI_FONT_SMALL);
//    CamExp = gui2->addTextInput("Camera Exposition", "600");
//    CamExp->setAutoUnfocus(false);
//    CamExp->setAutoClear(false);
//
//    gui2->addLabel("Gain:", OFX_UI_FONT_SMALL);
//   	gui2->setWidgetFontSize(OFX_UI_FONT_SMALL);
//    GAIN = gui2->addTextInput("Camera Gain", "128");
//    GAIN->setAutoUnfocus(false);
//    GAIN->setAutoClear(false);
//
//    gui2->addButton( "Apply", false);

    gui2->addSpacer();
    gui2->addLabel("Timeout:", OFX_UI_FONT_SMALL);
    gui2->setWidgetFontSize(OFX_UI_FONT_SMALL);
    gui2->addSlider("Laser (ms)", 15, 200, &Lto);
    gui2->setWidgetFontSize(OFX_UI_FONT_SMALL);
    gui2->addSlider("Stepper (ms)", 0, 100, &Sto);

    gui2->setPosition(212, 0);
    gui2->autoSizeToFitWidgets();

	ofAddListener(gui2->newGUIEvent,this,&ofApp::guiEvent);
}

//--------------------------------------------------------------
void ofApp::setGUI3()
{

	gui3 = new ofxUISuperCanvas("Manual Axis Position");
    gui3->addSpacer();
//    gui3->addLabel("FPS");
    gui3->addFPS();

    gui3->addSpacer();
	gui3->addToggle( "Manual mode", false);
    gui3->addSpacer();
    gui3->addLabel("Axis 1:", OFX_UI_FONT_SMALL);

    gui3->addButton( "Home", false);
    gui3->addButton( "<--Left", false);

    gui3->addButton( "Right-->", false);
    gui3->addSpacer();

    gui3->setWidgetFontSize(OFX_UI_FONT_SMALL);
	gui3->addLabel("AXIS POSITION", OFX_UI_FONT_SMALL);
	gui3->addSlider("AXIS1", 0.0, 500, &PosAxis1);
	gui3->addSlider("AXIS2", 0.0, 25, green);
	gui3->addSlider("AXIS3", 0.0, 25, blue);

    gui3->addSpacer();

    gui3->autoSizeToFitWidgets();
	ofAddListener(gui3->newGUIEvent,this,&ofApp::guiEvent);
}

//--------------------------------------------------------------
void ofApp::setGUI4()
{
    gui4 = new ofxUISuperCanvas("CAMERA SETTINGS");
    gui4->addSpacer();
	gui4->addToggle( "Zoom mouse click", false);

    gui4->addSpacer();

    gui4->addLabel("Video cam number:", OFX_UI_FONT_SMALL);
   	gui4->setWidgetFontSize(OFX_UI_FONT_SMALL);
    VideoNum = gui4->addTextInput("Videonum", "2");
    VideoNum->setAutoUnfocus(false);
    VideoNum->setAutoClear(false);

    gui4->addLabel("Focus:", OFX_UI_FONT_SMALL);
   	gui4->setWidgetFontSize(OFX_UI_FONT_SMALL);
    CamFocus = gui4->addTextInput("Camera Focus", "80");
    CamFocus->setAutoUnfocus(false);
    CamFocus->setAutoClear(false);

    gui4->addLabel("White Balance Temperature:", OFX_UI_FONT_SMALL);
    gui4->setWidgetFontSize(OFX_UI_FONT_SMALL);
    WBT = gui4->addTextInput("WBT value", "2800");
    WBT->setAutoUnfocus(false);
    WBT->setAutoClear(false);;

    gui4->addLabel("Exposition:", OFX_UI_FONT_SMALL);
   	gui4->setWidgetFontSize(OFX_UI_FONT_SMALL);
    CamExp = gui4->addTextInput("Camera Exposition", "600");
    CamExp->setAutoUnfocus(false);
    CamExp->setAutoClear(false);

    gui4->addLabel("Gain:", OFX_UI_FONT_SMALL);
   	gui4->setWidgetFontSize(OFX_UI_FONT_SMALL);
    GAIN = gui4->addTextInput("Camera Gain", "128");
    GAIN->setAutoUnfocus(false);
    GAIN->setAutoClear(false);

    gui4->addButton( "Apply", false);


//    gui2->setPosition(212, 0);
    gui4->autoSizeToFitWidgets();

	ofAddListener(gui4->newGUIEvent,this,&ofApp::guiEvent);
}

//--------------------------------------------------------------
void ofApp::drawGrid(float x, float y)
{
    float w = ofGetWidth();
    float h = ofGetHeight();

    for(int i = 0; i < h; i+=y)
    {
        ofLine(0,i,w,i);
    }

    for(int j = 0; j < w; j+=x)
    {
        ofLine(j,0,j,h);
    }
}

//--------------------------------------------------------------
void ofApp::Run_Scan()
{
    Punts Punts_Ok[cam3d.resy];
    int n=0;

    if(Scan==true)
        {
        if (CartessianXAxis==true)
                {
                if(PosAxis1<FiAxis1&& Axis1==true)
                    {
                    Stepper(1,0,IncAxis1_Steps,0, &serial,(unsigned int)Sto);
                    PosAxis1=PosAxis1+IncAxis1;

                    scan(&cam3d,&grisl,&TaL,&TsL);
                    TsL.update();TaL.update();
                    Component_3D_LinScan(cam3d,1,TsL, punts, PosAxis1);
                    check_scan(punts, Punts_Ok, &n);

                    if(n!=0)
                        {
                        cloudaux.width    = n;
                        cloudaux.height   = 1;
                        cloudaux.is_dense = false;

                        cloudaux.points.resize (cloudaux.width * cloudaux.height);

                        cloud=cloud+cloudaux;
                        clouderr=clouderr+cloudaux;
                        fill_cloud(Punts_Ok,n,nt);
                        reset_scan(punts);
                        nt=nt+n;
                        }
                    }
                    if(Axis1==false)
                        {
                        Stepper(1,1,0,1, &serial,(unsigned int)Sto);
                        ofSleepMillis(1);
                        IncAxis1_Steps=(int)((IncAxis1)*(3200/(12*PI)));
                        Stepper(1,(30.*IniAxis1)/PI,0,0, &serial,(unsigned int)Sto);
                        ofSleepMillis(1);
                        Axis1=true;
                        PosAxis1=IniAxis1;
                        }
                        if(PosAxis1>=FiAxis1)
                            {
                            Stepper(1,1,0,1, &serial,(unsigned int)Sto);

                            Axis1=false;
                            Scan=false;
                            }
                }
        }
}

//--------------------------------------------------------------
void ofApp::check_scan(Punts p[],Punts pok[], int *n)
{
    int i;
    *n=0;

    ofVec3f v(1, 0, 0);
    for(i=0;i<=cam3d.resy-1;i++)
        {
        if (CartessianXAxis==true)
            {
            ofVec3f P(p[i].x,p[i].y,p[i].z);
            ofVec3f O(0,0,0);
            float d=fabs(O.distance(P));
//                 cout << d << endl;
            if(d>dist_scan_min && d<dist_scan_max )
                {
                if(p[i].x!=-10000 && p[i].y!=-10000 && p[i].z!=-10000)
                    {
                    pok[*n]=p[i];
                    if(*n>0)
                        {
                        ofVec3f s(0,pok[*n-1].y- pok[*n].y, pok[*n-1].z- pok[*n].z);
                        ofVec3f u=v.getCrossed(s);
                        ofVec3f un=u.getNormalized();
                        pok[*n].nx=un.x;
                        pok[*n].ny=un.y;
                        pok[*n].nz=un.z;
                        }
                    else
                        {
                        pok[*n].nx=0;
                        pok[*n].ny=-1;
                        pok[*n].nz=0;
                        }
                    *n=*n+1;
                    }
                }
            }
        }

}

//--------------------------------------------------------------
void ofApp::reset_scan(Punts p[])
{
    int i;


    for(i=0;i<=cam3d.resy-1;i++)
    {
        p[i].x=-10000;
        p[i].y=-10000;
        p[i].z=-10000;
    }

}

//--------------------------------------------------------------
void ofApp::fill_cloud(Punts pok[], int n,int nt)
{
    int i;
    for(i=0;i<=n-1;i++)
        {
         cloud.points[nt+i].x = pok[i].x;
         cloud.points[nt+i].y = pok[i].y;
         cloud.points[nt+i].z = pok[i].z;

         cloud.points[nt+i].normal_x = pok[i].nx;
         cloud.points[nt+i].normal_y = pok[i].ny;
         cloud.points[nt+i].normal_z = pok[i].nz;

         cloud.points[nt+i].r = pok[i].r;
         cloud.points[nt+i].g = pok[i].g;
         cloud.points[nt+i].b = pok[i].b;

         clouderr.points[nt+i].x = pok[i].x;
         clouderr.points[nt+i].y = pok[i].y;
         clouderr.points[nt+i].z = pok[i].z;

         clouderr.points[nt+i].normal_x = pok[i].nx;
         clouderr.points[nt+i].normal_y = pok[i].ny;
         clouderr.points[nt+i].normal_z = pok[i].nz;

         clouderr.points[nt+i].r =(25*pok[i].q);
         clouderr.points[nt+i].g = 50;
         clouderr.points[nt+i].b = 0;
        }
}

//--------------------------------------------------------------
void ofApp::capture_image(ofImage *image)
{
    ofImage aux;

    aux.setFromPixels(cam.getPixels(), 1024, 576, OF_IMAGE_COLOR);
    *image=aux;

}

