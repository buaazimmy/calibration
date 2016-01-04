#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <cstdlib>
#include <stdio.h>   // Standard input/output definitions
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <pthread.h> // This uses POSIX Threads
#include <signal.h>


#ifndef B460800
#define B460800 460800
#endif

#ifndef B921600
#define B921600 921600
#endif

// Status flag
#define SERIAL_PORT_OPEN   1
#define SERIAL_PORT_CLOSED 0
#define SERIAL_PORT_ERROR -1

typedef unsigned char uint8_t;

class Serial_Port {

public:
	uint8_t          cp;
	Serial_Port();
	Serial_Port(char *&uart_name_, int &baudrate_);
	void initialize_defaults();
	~Serial_Port();

	bool debug;
	char *uart_name;
	int baudrate;
	int status;

	void open_serial();
	void close_serial();

	void start();
	void stop();

	void handle_quit(int sig);

	int read_port(uint8_t &cp);
private:

	int fd;
	pthread_mutex_t lock;

	int _open_port(const char* port);
	bool _setup_port(int baud, int data_bits, int stop_bits, bool parity,
			bool hardware_control);
	int _read_port(uint8_t &cp);
	void _write_port(char *buf, unsigned &len);
};

#endif // SERIAL_PORT_H_

