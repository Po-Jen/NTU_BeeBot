/*
//First edition by old000: C.H.Hung , NTU,EE97,master student
//Second edition by kenji: Y.C.Lin , NTU,EE97,master student
//          Idea from code by W.L.Hsu, NTU,EE98,PHD student

// MOUAHAHHAHA MALCOLM IS THERE ! :D
*/
#ifndef _SERIALPORTCONTROL_HPP
#define _SERIALPORTCONTROL_HPP

#include <cstring>
#include <string>
#include <iostream>
#include <SerialStream.h>

using namespace std;

class SerialPortControl{
	protected : 
	LibSerial::SerialStream _motor;

	public:
	SerialPortControl(){
		std::cout<<"Quel Baud pour le capteur ? "<<std::endl;
		int Baud=0;
		scanf("%d",&Baud);
		_motor.Open(PORT);
		if(Baud==115200){
			_motor.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_115200);
		}
		else if(Baud==57600){
			_motor.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_57600);
		}
		else if(Baud==9600){
			_motor.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_9600);
		}
		_motor.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);
		_motor.SetParity( LibSerial::SerialStreamBuf::PARITY_NONE ) ;
		_motor.SetFlowControl(LibSerial::SerialStreamBuf::FLOW_CONTROL_HARD ) ;
		_motor.SetNumOfStopBits(1) ;
	}
	
	
	
	int readLeftEncoder() //Left wheel node 0

	int readRightEncoder() //Right wheel node 1
	bool openPort(string port_name, int baud_rate);
	bool writePort(string str);
	string readPort();

};


inline int SerialPortControl::readLedtEncoder{

}

inline int SerialPortControl::readRightEncoder{

}

inline bool SerialPortControl::writePort(string str){

}

inline string SerialPortControl::readPort(){

}


#endif
