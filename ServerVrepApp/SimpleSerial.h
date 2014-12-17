#pragma once

#include <string>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

class SimpleSerial
{
public:
	SimpleSerial(const std::string port, unsigned int baudRate);
	~SimpleSerial();

	std::string readLine();
	void writeLine(std::string str);
private:
	boost::asio::io_service io;
	boost::asio::serial_port serialPort;
	//boost::system::error_code ec;
	std::stringstream inputBuffer;
};

