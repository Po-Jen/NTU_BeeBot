#include "SerialPortControl.hpp"

using namespace std;

bool SerialPortControl::openPort(string port_name, int baud_rate = 9600)
{
	DCB dcb;
	_hComm = CreateFile(port_name.c_str(),
						GENERIC_READ | GENERIC_WRITE, 
						0,		// exclusive access
						0,		// no security
						OPEN_EXISTING,
						0,		// no overlapped I/O
						NULL);	// null template

	if(_hComm == INVALID_HANDLE_VALUE){
		cout << "openPort: _hComm == INVALID_HANDLE_VALUE" << endl;
		return false;
	}

	if(!GetCommState(_hComm, &dcb)){
		cout << "openPort: GetCommState error!" << endl;
		return false;
	}

	// Baud Rate
	dcb.BaudRate = baud_rate;
	// 8N1
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	dcb.fOutX = FALSE;
	dcb.fInX = FALSE;
	dcb.fOutxCtsFlow = FALSE;
	dcb.fOutxDsrFlow = FALSE;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fDsrSensitivity = FALSE;
	dcb.fTXContinueOnXoff = FALSE;
	dcb.fErrorChar = FALSE;
	dcb.fNull = FALSE;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fAbortOnError = FALSE;
	dcb.fDummy2 = 0;
	dcb.wReserved = 0;
	dcb.XonLim = 0;
	dcb.XoffLim = 0;
	dcb.XonChar = 0;
	dcb.XoffChar = 0;
	dcb.ErrorChar = 0;
	dcb.EofChar = 0;
	dcb.EvtChar = 0;
	dcb.wReserved1 =0;
	
	if(!SetCommState(_hComm, &dcb)){ 
		cout << "openPort: SetCommState error!" << endl;
		return false;
	}
	if(!SetCommMask(_hComm, EV_RXCHAR)){ 
		cout << "openPort: SetCommMask error!" << endl;
		return false;
	}

	COMMTIMEOUTS timeouts;
	timeouts.ReadIntervalTimeout = MAXDWORD;
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.ReadTotalTimeoutConstant = 0;
	timeouts.WriteTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 100;
	if (!SetCommTimeouts(_hComm, &timeouts)){
		// Error setting time-outs.
		cout << "Error setting time-outs" << endl;
		return false;
	}
	return true;
}

bool SerialPortControl::writePort(string str)
{
	const char* lpBufferToWrite = str.c_str();
	unsigned int nNumOfBytesToWrite = str.size();
	unsigned long nNumOfBytesWritten;
	WriteFile(_hComm, lpBufferToWrite, nNumOfBytesToWrite, &nNumOfBytesWritten, NULL);
	if(nNumOfBytesWritten == nNumOfBytesToWrite){
		return true;
	}else{
		return false;
	}
}

string SerialPortControl::readPort()
{
	char byte_buffer;
	string ret;
	DWORD Event;
	DWORD ReadCount;
		
	while(true)
	{
		if(WaitCommEvent(_hComm, &Event, NULL))
		{
			do{
				if(ReadFile(_hComm, &byte_buffer, 1, &ReadCount, NULL))
				{
					if((int)byte_buffer > 1)
					{
						if(isdigit(byte_buffer) || (int)byte_buffer == 45)
						{
							ret += byte_buffer;
						}
						else{;}
					}
				}
				else
				{
					cout << "An error occurred in the ReadFile call" << endl;
					break;
				}
			}while(ReadCount);
			break;
		}
		else
		{
			cout << "Error in WaitCommEvent" << endl;
			break;
		}
	}
	return ret;
}