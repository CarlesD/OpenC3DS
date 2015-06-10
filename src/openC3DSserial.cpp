#include "openC3DSserial.h"

unsigned char* convert(string& s){
    unsigned char* bytes = new unsigned char[s.size()+1]();
    copy(s.begin(),s.end(),bytes);
    return(bytes);
}

//--------------------------------------------------------------
void openC3DSserial::setup(){

	// GUI
	setGuiSerial();
	guiSerial->loadSettings("guiSerial.xml");
	// trick to load serialDeviceName and baudRate from saved guiSerial
    serialDeviceName = serialDeviceGui->getTextString();
    vector<ofxUIToggle *> gt = serialBaudsGui->getToggles();
    for(int i=0; i<gt.size(); i++){
        ofxUIToggle *ggtt = gt.at(i);
        if(ggtt->getValue()){
            baudRate = ofToInt(ggtt->getName());
        }
    }
    // force lasers off
    blaser1On = blaser2On = false;

    // SERIAL PORT
	serialPort.listDevices();
	vector <ofSerialDeviceInfo> deviceList = serialPort.getDeviceList();
	nRead = 0;

    serialPort.setup(serialDeviceName, baudRate);
    bisDeviceReady = true;
}

//--------------------------------------------------------------
void openC3DSserial::update(){
    nRead = 0;
    nRead = serialPort.readBytes(bytesReturned, ACK_BYTES);
    if(nRead == OF_SERIAL_ERROR){
        ofLog(OF_LOG_ERROR, ofGetTimestampString() + "openC3DSserial::update:readBytes error reading from serial");
    }
    else if(nRead == OF_SERIAL_NO_DATA){
    }
    else if(nRead > 0){
        ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "openC3DSserial::update:readBytes: " + ofToString(nRead) + " bytes");
        bisDeviceReady = true;
        string bytesReadString = (char*) bytesReturned;
        ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "openC3DSserial::update:bytesReadString: " + ofToString(bytesReadString));
    }

}

//--------------------------------------------------------------
void openC3DSserial::draw(){

}

//--------------------------------------------------------------
void openC3DSserial::exit(){
    guiSerial->saveSettings("guiSerial.xml");
	delete guiSerial;

	serialPort.close();
}

//--------------------------------------------------------------
void openC3DSserial::setGuiSerial(){

	guiSerial = new ofxUISuperCanvas("SERIAL COMMUNICATION");
    guiSerial->addSpacer();

    guiSerial->addLabel("PORT");
    serialDeviceGui = guiSerial->addTextInput("SERIAL_PORT", "/dev/ttyACM0");

    vector <string> bauds;
	bauds.push_back("9600");
	bauds.push_back("19200");
	bauds.push_back("115200");
	guiSerial->addLabel("BAUDS");
	serialBaudsGui = guiSerial->addRadio("BAUDS", bauds, OFX_UI_ORIENTATION_VERTICAL);

	guiSerial->addLabel("MOTOR");
    guiSerial->addButton("m_move_left", false);
    guiSerial->addButton("m_move_right", false);

    guiSerial->addLabel("LASERS");
    guiSerial->addToggle("laser1_on", &blaser1On);
    guiSerial->addToggle("laser2_on", &blaser2On);

    guiSerial->addLabel("TEST");
    guiSerial->addButton("send_noise", false);
    guiSerial->addButton("send_hello", false);

    guiSerial->setPosition(0,0);
    guiSerial->autoSizeToFitWidgets();
	ofAddListener(guiSerial->newGUIEvent,this,&openC3DSserial::guiEvent);
}

//--------------------------------------------------------------
void openC3DSserial::guiEvent(ofxUIEventArgs &e){
	string name = e.getName();
	int kind = e.getKind();

	if(name == "m_move_left"){
        moveTestLeft();
	}
	else if(name == "m_move_right"){
        moveTestRight();
	}
	else if(name == "laser1_on"){
        if(blaser1On){
            turnOnLaser(1);
        }
        else{
            turnOffLaser(1);
        }
	}
	else if(name == "laser2_on"){
        if(blaser2On){
            turnOnLaser(2);
        }
        else{
            turnOffLaser(2);
        }
	}
	else if(name == "send_noise"){
        sendNoise();
	}
	else if(name == "send_hello"){
        sendHello();
	}
	else if(name == "BAUDS"){
        ofxUIRadio *radio = (ofxUIRadio *) e.widget;
        baudRate = ofToInt(radio->getActiveName());
        serialPort.close();
        serialPort.setup(serialDeviceName, baudRate);
    }
    else if(name == "SERIAL_PORT"){
        ofxUITextInput *ti = (ofxUITextInput *) e.widget;
        if(ti->getInputTriggerType() == OFX_UI_TEXTINPUT_ON_ENTER){
            serialDeviceName = ti->getTextString();
            serialDeviceGui->setTextString(serialDeviceName);
        }
    }
}

//--------------------------------------------------------------
bool openC3DSserial::moveStepperToHome(){
    // TODO com sabem quants stemps ha de fer fins a home?
}

//--------------------------------------------------------------
bool openC3DSserial::moveStepperBySteps(int steps){
    if(bisDeviceReady){
        strSend = ofToString(MOTOR) + " 1 2 " + ofToString(steps);

        strSend += "\n";
        int res = serialPort.writeBytes(&convert(strSend)[0], strSend.length());
        ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "openC3DSserial::moveStepperBySteps: " + strSend);
        serialPort.drain();
        bisDeviceReady = false;

        if(res > 0){
            return true;
        }
        else{
            return false;
        }
    }
    else{
        ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "openC3DSserial::moveStepperBySteps:ERROR: device NOT ready");
    }
}

//--------------------------------------------------------------
bool openC3DSserial::moveTestLeft(){
    if(bisDeviceReady){
        strSend = ofToString(MOTOR) + " 1 2 9999\n";
        int res = serialPort.writeBytes(&convert(strSend)[0], strSend.length());
        ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "openC3DSserial::moveTestLeft: " + strSend);
        serialPort.drain();
        bisDeviceReady = false;

        if(res > 0){
            return true;
        }
        else{
            return false;
        }
    }
    else{
        ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "openC3DSserial::moveTestLeft:ERROR: device NOT ready");
    }
}

//--------------------------------------------------------------
bool openC3DSserial::moveTestRight(){
    if(bisDeviceReady){
        strSend = ofToString(MOTOR) + " 1 2 -9999\n";
        int res = serialPort.writeBytes(&convert(strSend)[0], strSend.length());
        ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "openC3DSserial::moveTestRight: " + strSend);
        serialPort.drain();
        bisDeviceReady = false;

        if(res > 0){
            return true;
        }
        else{
            return false;
        }
    }
    else{
        ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "openC3DSserial::moveTestRight:ERROR: device NOT ready");
    }
}

//--------------------------------------------------------------
bool openC3DSserial::turnOnLaser(int laser){
    if(bisDeviceReady){
        strSend = ofToString(laser);
        strSend += " 1 0 0\n";
        int res = serialPort.writeBytes(&convert(strSend)[0], strSend.length());
        ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "openC3DSserial::turnOnLaser: " + strSend);
        serialPort.drain();
        bisDeviceReady = false;

        if(res > 0){
            return true;
        }
        else{
            return false;
        }
    }
    else{
        ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "openC3DSserial::turnOnLaser:ERROR: device NOT ready");
    }
}

//--------------------------------------------------------------
bool openC3DSserial::turnOffLaser(int laser){
    if(bisDeviceReady){
        strSend = ofToString(laser);
        strSend += " 0 0 0\n";

        int res = serialPort.writeBytes(&convert(strSend)[0], strSend.length());
        ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "openC3DSserial::turnOffLaser: " + strSend);
        serialPort.drain();
        bisDeviceReady = false;

        if(res > 0){
            return true;
        }
        else{
            return false;
        }
    }
    else{
        ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "openC3DSserial::turnOffLaser:ERROR: device NOT ready");
    }
}

//--------------------------------------------------------------
bool openC3DSserial::sendNoise(){
    if(bisDeviceReady){
        strSend = "2 2 2 2\n";

        int res = serialPort.writeBytes(&convert(strSend)[0], strSend.length());
        ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "openC3DSserial::sendNoise: " + strSend);
        serialPort.drain();
        bisDeviceReady = false;

        if(res > 0){
            return true;
        }
        else{
            return false;
        }
    }
    else{
        ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "openC3DSserial::sendNoise:ERROR: device NOT ready");
    }
}

//--------------------------------------------------------------
bool openC3DSserial::sendHello(){
    strSend = "hello\n";

    int res = serialPort.writeBytes(&convert(strSend)[0], strSend.length());
    ofLog(OF_LOG_NOTICE, ofGetTimestampString() + "openC3DSserial::sendHello: " + strSend);
    serialPort.drain();
    bisDeviceReady = false;

    if(res > 0){
        return true;
    }
    else{
        return false;
    }
}
