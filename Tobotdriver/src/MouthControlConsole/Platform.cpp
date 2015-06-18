#include "Platform.hpp"

Platform::Platform(int port_num1,int baud_rate = 9600)
{
	char motor_port_name[10];
 	sprintf_s(motor_port_name, "COM%d", port_num1);
	_motorControl1 = new SerialPortControl(string(motor_port_name), baud_rate);

    _isIdle = true;
	
    _motorControl1->writePort(string("1ANSW0\n")); //YC Idea 同步是什麼意思
	
	setSpeed(MOTION_SPEED);
	setAcceleration(MOTION_ACCELERATION);
	setDeceleration(MOTION_DECELERATION);

}

//設定追pos的速度
void Platform::setSpeed(int speed_rpm)
{
	char speedChar[10];
	sprintf_s(speedChar, "sp%d\n", speed_rpm);
	speed = speed_rpm;
	_motorControl1->writePort(string(speedChar));
	
}

void Platform::setAcceleration(int acc)
{
	char accelerationChar[10];
	sprintf_s(accelerationChar, "ac%d\n", acc);
	acceleration = acc;
	_motorControl1->writePort(string(accelerationChar));
	
}  

void Platform::setDeceleration(int dec)
{
	char decelerationChar[10];
	sprintf_s(decelerationChar, "dec%d\n", dec);
	deceleration = dec;
	_motorControl1->writePort(string(decelerationChar));
	
}

void Platform::resetMotorEncoder()
{
	_motorControl1->writePort("ho\n");

	cout << "Wait: resetMotorEncoder " << endl;
	Sleep(5);
}


void Platform::Mouth(bool bState)//0:up, 1:down
{
	if(bState){
		
		_motorControl1->writePort(string("la17000\n"));
		_motorControl1->writePort(string("pos\n"));
	}
	else{
		_motorControl1->writePort(string("la-17000\n"));
		_motorControl1->writePort(string("pos\n"));
	}
	_motorControl1->writePort(string("m\n"));
	Sleep(50);
}

void Platform::Mouth_initialize(bool bState)//initialize
	{
		if(bState){
		_motorControl1->writePort(string("la0\n"));
		}
	_motorControl1->writePort(string("m\n"));
	Sleep(50);
}

