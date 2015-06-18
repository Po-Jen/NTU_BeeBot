/*
//First edition by old000: C.H.Hung , NTU,EE97,master student
//Second edition by kenji: Y.C.Lin , NTU,EE97,master student
//          Idea from code by W.L.Hsu, NTU,EE98,PHD student

// MOUAHAHHAHA MALCOLM IS THERE ! :D
*/
#ifndef _SERIALPORTCONTROL_HPP
#define _SERIALPORTCONTROL_HPP

#include <iostream>
#include <deque>
#include <cstdlib>
#include <sstream>
#include <iomanip>
#include <string>
#include <stdexcept>
#include <SerialStream.h>
#include <SerialPort.h>


#define PORT "/dev/ttyUSB0"

class SerialPortControl{
	protected :
	//ATTRIBUTS 
	SerialPort _motor;
	int _lEnco;
	int _rEnco;
	float _speedRwheel; //Speed in rpm
	float _speedLwheel;
	float _measuredSRW; //Speed of right wheel mesured
	float _measuredSLW;
	
	bool _verbose;
	
	
	public:
	SerialPortControl(int Baud) : _motor(PORT), _verbose(false){
		std::cout<<"GOOOOOO"<<std::endl;
		_motor.Open(SerialPort::BAUD_115200, SerialPort::CHAR_SIZE_8, SerialPort::PARITY_NONE,SerialPort::STOP_BITS_1,SerialPort::FLOW_CONTROL_HARD );
		
		//8 data bits
		//1 stop bit
		//No parity

	}
	
	~SerialPortControl(){
		_motor.Close();
	}
	
	int getLencoder(){return _lEnco;}
	int getRencoder(){return _rEnco;}
	float getSpeedLwheel(){return _speedLwheel;}
	float getSpeedRwheel(){return _speedRwheel;}
	float getMeasuredSLW(){return _measuredSLW;}
	float getMeasuredSRW(){return _measuredSRW;}
	SerialPort& getMotor(){return _motor;}
	
	void readLencoder(); //Left wheel node 0
	void readRencoder(); //Right wheel node 1
	void readSLW();
	void readSRW();
	void setSpeedLwheel(float s){_speedLwheel=s;}
	void setSpeedRwheel(float s){_speedRwheel=s;}
	void setVerbose(){_verbose=true;}
	
	bool openPort();
	bool writePort(std::string& str);
	std::string readPort();
	
	void update();

};


inline void SerialPortControl::readLencoder(){

	std::string yo("0pos\n");
	writePort(yo);
	sleep(5);
	
	char i;
	//_motor >> i;
	
	std::cout << "Well done bobby" <<i<< "ain't it great"<< std::endl;
	
	/*char i;
	_motor >> i; 
	std::cout<< "j'ai lu "<< i<< " voila"<<std::endl;
	
	ARDUINO TESTED*/
}

inline void SerialPortControl::readRencoder(){
	std::string yo("1pos\n");
	writePort(yo);
	sleep(5);
	
	char i;
	//_motor >> i;
	
	std::cout << "Well done bobby" <<i<< "ain't it great"<< std::endl;
	
	
	
}

inline void SerialPortControl::readSLW(){

}


inline void SerialPortControl::readSRW(){

}

inline bool SerialPortControl::openPort(){
	std::string enable("en\n");
	const char* en=enable.c_str();
	//_motor << en;
}

inline bool SerialPortControl::writePort(std::string& str){
	
	char temp=0;
	int i=0;
	const char* cstr = str.c_str();
	while(cstr[i]!='\n' &&i <20)
	{
		try
		{
			_motor.WriteByte(cstr[1]);
			std::cout<<"wrte " <<cstr[i]<<std::endl;
		}
		catch(SerialPort::ReadTimeout &e)
		{
			std::cout<<"Read Timeout"<<std::endl;
			return false;
		}
			++i;
			//cout<<i++<<temp<<'x'<<endl;
	}
	if(_verbose){
		puts(cstr);
	}
	
	std::cout << "YO "<< cstr<< std::endl;
	return true;
}

inline std::string SerialPortControl::readPort(){

	char temp=0;
	int i=0;
	char* cstr;
	while(temp!='\n')
	{
		try
		{
			temp=_motor.ReadByte(100);
		}
		catch(SerialPort::ReadTimeout &e)
		{
			std::cout<<"Read Timeout"<<std::endl;
			return "fail";
		}
		if((temp!='\n')&&(temp!=0)&&(temp!=' '))
		{
			cstr[i]=temp;
			++i;
			//cout<<i++<<temp<<'x'<<endl;
		}
	}
	cstr[i]='\0';
	if(_verbose){
		puts(cstr);
	}
	
	std::cout << "YO ecrit "<< cstr<< std::endl;


}

inline void SerialPortControl::update(){
	if(_verbose==true){
		std::cout<<"ON FAIT DES TRUCS COOLS AUX MOTEURS..."<<std::endl;
	}
}


#endif
