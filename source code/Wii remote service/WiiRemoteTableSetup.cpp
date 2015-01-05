// WiiRemote.cpp : Defines the entry point for the console application.
//

#include <iostream>
#include <fstream>
#include <string>
#include "SIGService.h"
#include "include\wiimote.h"



class WiiRemote_Service : public sigverse::SIGService
{

public:
	WiiRemote_Service(std::string name);
	~WiiRemote_Service();
    double onAction();
	void onRecvMsg(sigverse::RecvMsgEvent &evt);
	wiimote cWiiRemote;
private: 
	HWND hWnd;	
};

WiiRemote_Service::WiiRemote_Service(std::string name) : SIGService(name){

	_tprintf(_T("contains WiiYourself! wiimote code by gl.tter\nhttp://gl.tter.org\n"));

	static const TCHAR* wait_str[] = { _T(".  "), _T(".. "), _T("...") };
	unsigned count = 0;
	while(!cWiiRemote.Connect(wiimote::FIRST_AVAILABLE)) {
		_tprintf(_T("\b\b\b\b%s "), wait_str[count%3]);
		count++;
		Beep(500, 30); Sleep(1000);
	}

	_tprintf(_T("Connected.\n"));
	hWnd = NULL;
};

WiiRemote_Service::~WiiRemote_Service(){
	cWiiRemote.Disconnect();
	_tprintf(_T("Disconnected.\n"));
	this->disconnect();
}

double WiiRemote_Service::onAction(){
	if(cWiiRemote.bExtension) cWiiRemote.SetReportType(wiimote::IN_BUTTONS_ACCEL_IR_EXT); // no IR dots
	else cWiiRemote.SetReportType(wiimote::IN_BUTTONS_ACCEL_IR);      //    IR dots
	while(cWiiRemote.RefreshState() == NO_CHANGE){
	}

	// Button A is used for rotating the agent left
	if(cWiiRemote.Button.A()){			
		sendMsg("robot_000", "rotationL");
		std::cout << "rotationL" << std::endl;
	}

	// Button Left is used for rotating the agent left
	else if(cWiiRemote.Button.Left())
	{
		sendMsg("robot_000", "rotationL");
		std::cout << "rotationL" << std::endl;
	}

	// Button Right is used for rotating the agent right
	else if(cWiiRemote.Button.Right())
	{
		sendMsg("robot_000", "rotationR");
		std::cout << "rotationR" << std::endl;
	}

	// Button Up is used for moving the robot forward
	else if(cWiiRemote.Button.Up())
	{
		sendMsg("robot_000", "move");
		std::cout << "move" << std::endl;
	}

	// Button Down is used for moving the robot backward
	else if(cWiiRemote.Button.Down())
	{
		sendMsg("robot_000", "back");
		std::cout << "back" << std::endl;
	}

	// Button B is used for re-grasping the object
	else if(cWiiRemote.Button.B())
	{
		sendMsg("robot_000", "regrasp");
		std::cout << "regrasp" << std::endl;
	}

	// Button Home is used for releasing the object
	else if(cWiiRemote.Button.Home())
	{
		sendMsg("robot_000", "release");
		std::cout << "release" << std::endl;
	}

	// Button 1 is used for moving the robot's waist down
	else if(cWiiRemote.Button.One())
	{
		sendMsg("robot_000", "WAIST_JOINT1 90");
		std::cout << "WAIST_JOINT1 90" << std::endl;
	}

	// Button Left is used for moving the robot's back to its initial posture
	else if(cWiiRemote.Button.Two())
	{
		sendMsg("robot_000", "WAIST_JOINT1 0");
		std::cout << "WAIST_JOINT1 0" << std::endl;
		sendMsg("robot_000", "LARM_JOINT1 0");
		std::cout << "LARM_JOINT1 0" << std::endl;
		sendMsg("robot_000", "RARM_JOINT1 0");
		std::cout << "RARM_JOINT1 0" << std::endl;
	}


	return 0.0;
}

void WiiRemote_Service::onRecvMsg(sigverse::RecvMsgEvent &evt){
   
}




int main(int argc, char* argv[])
{
	WiiRemote_Service srv("Throwing_Service");
	unsigned short port = (unsigned short)(atoi(argv[2]));
	srv.connect(argv[1], port);
	srv.startLoop();

	return 0;
}

