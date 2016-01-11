/**
 * Test XBee sending messages at pre-defined speed.
 */

#include <unistd.h>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <string>
#include <iostream>

#define SLEEP_SECONDS 1

float to_microseconds(float seconds) {
	return seconds*1000000;
}

void sleep_seconds(float seconds) {
	usleep(to_microseconds(seconds));
}

int main() {
	char message;
	boost::asio::io_service ios;
	boost::asio::serial_port sp(ios, "/dev/cu.usbserial-A600e0ti");

	sp.set_option(boost::asio::serial_port::baud_rate(19200));

	while (true) {
		message = '@';
		sp.write_some(boost::asio::buffer(message, sizeof(message)));
		sleep_seconds(SLEEP_SECONDS);
	}

	sp.close();
}
