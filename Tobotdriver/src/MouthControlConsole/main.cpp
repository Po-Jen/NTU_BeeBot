#include "Platform.hpp"
//#include "stdafx.h"
#include <Mmsystem.h>
//#include "wave_rec_play.h"
#pragma   comment(lib,"winmm.lib")


void threadProc(void* p)
{
	PlaySound("C:\\Users\\R\\Dropbox\\IRA\\PMC競賽\\Speech\\您好.wav", NULL, SND_SYNC);
}

int main(int argc, char **argv)
{
	char input;

	Platform* robot_platform = new Platform( MOTOR_PORT1, MOTOR_BAUDRATE );
	Sleep(50);
	robot_platform->setSpeed(8000);

	_beginthread(threadProc, 0,NULL);

	for(int counter = 1;counter <= 3; counter++)
	{
		Sleep(350);
		robot_platform->Mouth(1);
		cout<<"left\n";
		Sleep(500);
		robot_platform->Mouth(0);
		cout<<"right\n";
	}

	Sleep(500);

	robot_platform->Mouth_initialize(1);//initialize mouth position
	cout<<"postion initialized\n";
	Sleep(500);
	cout<<"end\n";
	Sleep(500);
	
	
	/*PlaySound("C:\\Users\\R\\Dropbox\\IRA\\PMC競賽\\Speech\\您好.wav", NULL, SND_SYNC);*/
	state03:
	cout<<"\nplease enter X to exit: ";
	cin>>input;

	switch (input)
	{
		case 'X':
		case 'x':
			break;

		default:
			cout<<"invalid input!\n";
			goto state03;

	}

	return 0;
}

