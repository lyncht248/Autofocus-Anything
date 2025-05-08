#include "Xeryon.h"

#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <cstring>

#if defined (_WIN32) || defined (_WIN64)
    #include <windows.h>
#endif
#if defined (__linux__) || defined(__APPLE__)
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#endif

Axis * Xeryon::addAxis(const char letter, const Stage * stage)
{
	Axis * a = new Axis(this, letter, stage);
	axis_list_.push_back(a);
	return a;
}

Axis * Xeryon::getAxis(const char letter)
{
	for (auto const& axis : axis_list_) {
		if (axis->getLetter() == letter) {
			return axis;
		}
	}
	return nullptr;
}


#if defined (_WIN32) || defined( _WIN64)
static std::string GetLastErrorAsString()
{
    DWORD errorMessageID = GetLastError();
    if (errorMessageID == 0)
        return std::string();

    LPSTR messageBuffer = nullptr;

    size_t size = FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
            NULL, errorMessageID, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&messageBuffer, 0, NULL);

    std::string message(messageBuffer, size);
    LocalFree(messageBuffer);
    return message;
}
#endif

void Xeryon::start()
{
	if (axis_list_.size() == 0) {
        std::cout << "Cannot start the system without stages. The stages don't have to be connnected, only initialized in the software." << std::endl;
        return;
	}

#if defined (_WIN32) || defined( _WIN64)
	port_handle_= CreateFileA(port_, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
	if (port_handle_ == INVALID_HANDLE_VALUE) {
		// TODO: Deal with this
        std::cout << "CreateFileA failed." << std::endl;
        std::cout << GetLastErrorAsString() << std::endl;
        return;
	}

	DCB dcb;
	dcb.DCBlength = sizeof(dcb);

	if (!GetCommState(port_handle_, &dcb)) {
		// TODO: Deal with this
        std::cout << "GetCommState failed." << std::endl;
        std::cout << GetLastErrorAsString() << std::endl;
        return;
	}

	switch (baudrate_)
	{
		case 110:      dcb.BaudRate = CBR_110; break;
		case 300:      dcb.BaudRate = CBR_300; break;
		case 600:      dcb.BaudRate = CBR_600; break;
		case 1200:     dcb.BaudRate = CBR_1200; break;
		case 2400:     dcb.BaudRate = CBR_2400; break;
		case 4800:     dcb.BaudRate = CBR_4800; break;
		case 9600:     dcb.BaudRate = CBR_9600; break;
		case 14400:    dcb.BaudRate = CBR_14400; break;
		case 19200:    dcb.BaudRate = CBR_19200; break;
		case 38400:    dcb.BaudRate = CBR_38400; break;
		case 56000:    dcb.BaudRate = CBR_56000; break;
		case 57600:    dcb.BaudRate = CBR_57600; break;
		case 115200:   dcb.BaudRate = CBR_115200; break;
		case 128000:   dcb.BaudRate = CBR_128000; break;
		case 256000:   dcb.BaudRate = CBR_256000; break;
		default:
            //TODO: Deal with this
            std::cout << "Unsupported baudrate: " << baudrate_ << std::endl;
            return;
    }
	dcb.ByteSize = 8;
	dcb.StopBits = ONESTOPBIT;
	dcb.Parity = NOPARITY;

	if (!SetCommState(port_handle_, &dcb)) {
		// TODO: Deal with this
        std::cout << "SetCommState failed." << std::endl;
        std::cout << GetLastErrorAsString() << std::endl;
        return;
	}

	timeouts_.ReadIntervalTimeout = 0;
	timeouts_.ReadTotalTimeoutConstant = MAXDWORD;
	timeouts_.ReadTotalTimeoutMultiplier = 0;
	timeouts_.WriteTotalTimeoutConstant = MAXDWORD;
	timeouts_.WriteTotalTimeoutMultiplier = 0;

	if (!SetCommTimeouts(port_handle_, &timeouts_)) {
		// TODO: Deal with this
        std::cout << "SetCommTimeouts failed." << std::endl;
        std::cout << GetLastErrorAsString() << std::endl;
        return;
	}
#endif
#if defined (__linux__) || defined(__APPLE__)
	struct termios options;

	port_fd_ = open(port_, O_RDWR | O_NOCTTY | O_NDELAY);
	if (port_fd_ == -1) {
		// TODO: Deal with this
        std::cout << "open of " << port_ << " failed." << std::endl;
        return;
	}
	fcntl(port_fd_, F_SETFL, FNDELAY);

	tcgetattr(port_fd_, &options);
	bzero(&options, sizeof(options));

	speed_t speed;
	switch (baudrate_)
	{
		case 110:      speed = B110; break;
		case 300:      speed = B300; break;
		case 600:      speed = B600; break;
		case 1200:     speed = B1200; break;
		case 2400:     speed = B2400; break;
		case 4800:     speed = B4800; break;
		case 9600:     speed = B9600; break;
		case 19200:    speed = B19200; break;
		case 38400:    speed = B38400; break;
		case 57600:    speed = B57600; break;
		case 115200:   speed = B115200; break;
		default:
            std::cout << "Unsupported baudrate: " << baudrate_ << std::endl;
            return;
	}    
	cfsetispeed(&options, speed);
	cfsetospeed(&options, speed);
	options.c_cflag |= ( CLOCAL | CREAD |  CS8);
	options.c_iflag |= ( IGNPAR | IGNBRK );
	options.c_cc[VTIME] = 0;
	options.c_cc[VMIN] = 0;
	tcsetattr(port_fd_, TCSANOW, &options);
#endif
	comm_thread_ = std::thread(&Xeryon::processData_, this);
	reset();
}

void Xeryon::stop()
{
	for (auto const& axis : axis_list_)
		axis->stop();

	// close comms
	comm_exit_.set_value();
	comm_thread_.join();
#if defined (_WIN32) || defined( _WIN64)
	CloseHandle(port_handle_);
#endif
#if defined (__linux__) || defined(__APPLE__)
	close(port_fd_);
#endif
}

void Xeryon::sendCommand(Axis * axis, const char * command)
{
    int n = 0;
    char cmd[64];
	if (isSingleAxisSystem())
		n = snprintf(cmd, sizeof(cmd), "%s\n", command);
	else
		n = snprintf(cmd, sizeof(cmd), "%c:%s\n", axis->getLetter(), command);
	// std::cout << "CMD: " << axis->getLetter() << ": " << cmd;
#if defined (_WIN32) || defined( _WIN64)
    DWORD nWritten;
    if (!WriteFile(port_handle_, cmd, n, &nWritten, NULL))
    {
	    // TODO: Deal with the error
        std::cout << "WriteFile failed." << std::endl;
        std::cout << GetLastErrorAsString() << std::endl;
        return;
    }
#endif
#if defined (__linux__) || defined(__APPLE__)
    if (write(port_fd_, cmd, n) != n)
    {
	    // TODO: Deal with the error
        std::cout << "write failed." << std::endl;
        return;
    }
#endif
}

void Xeryon::reset()
{
	//using namespace std::literals::chrono_literals;
	for (auto const& axis : axis_list_)
		axis->reset();
	std::this_thread::sleep_for(std::chrono::milliseconds(200));
	readSettings();
	for (auto const& axis : axis_list_)
		axis->sendSettings();
}

void Xeryon::stopMovements()
{
	for (auto const& axis : axis_list_)
		axis->stopMovement();
}

void Xeryon::readSettings()
{
	std::ifstream file("settings_default.txt");
	std::string line;
	std::size_t pos;
	while (std::getline(file, line)) {
		if (line.find('=') != std::string::npos) {
			while ((pos = line.find(' ')) != std::string::npos) line.erase(pos, 1); 
			while ((pos = line.find('\r')) != std::string::npos) line.erase(pos, 1); 
			while ((pos = line.find('\n')) != std::string::npos) line.erase(pos, 1); 

			// By default act on the first defined axis
			Axis * axis = axis_list_.front();

			if (line.find(':') != std::string::npos) {
				axis = getAxis(line[0]);
				if (axis == nullptr)
					continue; // Ignore unknown axis
				line.erase(0, 2); // remove 'X:'
			}

			// Ignore comments
			if ((pos = line.find('%')) != std::string::npos) line.erase(pos); 

			pos = line.find('=');
			std::string tag = line.substr(0, pos);
			std::string value = line.substr(pos + 1);

			axis->setSetting(tag, value, true);
		}
	}
	file.close();

	// TODO: stop the thread when an error happens
}


int Xeryon::readPort_(char * c, unsigned int timeout)
{
#if defined (_WIN32) || defined(_WIN64)
    DWORD n = 0;
    timeouts_.ReadTotalTimeoutConstant = timeout;
    if (!SetCommTimeouts(port_handle_, &timeouts_))
        return -1;

    if (!ReadFile(port_handle_, c, 1, &n, NULL))
        return -1;

    return n;
#endif
#if defined (__linux__) || defined(__APPLE__)
    struct pollfd p;
    p.fd = port_fd_;
    p.events = POLLIN;
    int rc = poll(&p, 1, timeout);
    if (rc == 0)
        return 0;
    else if (rc > 0) {
        return read(port_fd_, c, 1);
    }
    return -1;
#endif
}

void Xeryon::processData_()
{
	std::future<void> f = comm_exit_.get_future();
    char line[64] = { 0 };
    char * p = line;
	std::cout << "processData_ thread starting..." << std::endl;
	while (true) {
		if (f.wait_for(std::chrono::milliseconds(1)) != std::future_status::timeout)
			break;

        int rc = readPort_(p, 500);

        if (rc < 1)         // an error happened, game over.
            break;
        else if (rc == 0)   // nothing received yet, continue to wait.
            continue;

        if ((*p == '\r') || (*p == '\n')) { // line received
            *p = '\0';
            char * e = strchr(line, '=');
            const char * tag, * value;
            Axis * axis;
            if (e) {
                *e = '\0';
                value = e + 1;
                if (isSingleAxisSystem()) {
                    tag = line;
                    axis = axis_list_.front();
                }
                else if (line[1] == ':') {
                    axis = getAxis(line[0]);
                    tag = &line[2];
                }
                else {
                    // TODO: wrongly formatted line
                    memset(line, 0, sizeof(line));
                    p = line;
                    continue;
                }
                axis->receiveData(tag, std::atoi(value));
            }
        }
        else if (p < line + sizeof(line) - 1) {
            p++;            // still room in the line buffer
            continue;
        }

        // restart with a fresh buffer
        memset(line, 0, sizeof(line));
        p = line;
	}
	std::cout << "processData_ thread exiting..." << std::endl;

}
