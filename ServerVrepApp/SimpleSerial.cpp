#include "SimpleSerial.h"

#include <iostream>
#include <boost/system/error_code.hpp>

SimpleSerial::SimpleSerial(const std::string port, unsigned int baudRate):
io(),
serialPort(io, port)
{
	if(!serialPort.is_open())
	{
		std::cerr << "Error! Could not open serial! \n";
		return;
	}
	serialPort.set_option(boost::asio::serial_port::baud_rate(baudRate));
}

std::string SimpleSerial::readLine()
{
	char c;
	std::string result;
	if(serialPort.is_open())
	{
		for(;;)
		{
			boost::asio::read(serialPort, boost::asio::buffer(&c, 1));
			switch(c)
			{
				case '\r':
					break;
				case '\n':
					return result;
				default:
					result += c;
			}
		}
	}
	return result;
}

void SimpleSerial::writeLine(std::string str)
{
	if(serialPort.is_open())
	{
		str += "\r\n";
		boost::asio::write(serialPort, boost::asio::buffer(str));
	}
}


SimpleSerial::~SimpleSerial()
{
	serialPort.close();
}
