#include "lens.hpp"

#include <iostream>

#include <stdio.h>
#include <string.h>
#include <sstream>
#include <iostream>
#include <string>
#include <tgmath.h>


#include <vector>
#include <string>
#include <iomanip>

int lens::initmotor(int PortToUse) {

    // Check for errors
    if (serial_port < 0) {
        return 0;
    }
      // Create new termios struct, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return 0;
    }
    

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes


    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        //printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        std::cout << "failure to save tty settings for lens motor";
        return 0;
    }


    ssize_t bytes_written; //Gets rid of the warn_unused_result error

    // Sets the home location offset to be 1mm (or __ encoder pulses, or 0x0400) from the factory home location. Doesn't move lens. 
    unsigned char msg[] = { '0', 's', 'o', '0', '0', '0', '0', '0', '4', '0', '0', '\r', '\n' };
    bytes_written = write(serial_port, msg, sizeof(msg));
    usleep(1000000); //Sleep for 1s

    // The lens will move to the offset home position, quickly move to the home position for callibration (end of the slide right), and then back to the offset home position.
    unsigned char msg2[] = { '0', 'h', 'o', '0', '\r', '\n' };
    bytes_written = write(serial_port, msg2, sizeof(msg2));
    usleep(1000000); //Sleep for 1s

    // Absolute move of 0x2E00, or 11mm, relative to the offset home position.
    unsigned char msg3[] = { '0', 'm', 'a', '0', '0', '0', '0', '2', 'E', '0', '0', '\r', '\n' };
    bytes_written = write(serial_port, msg3, sizeof(msg3));
    usleep(1000000); //Sleep for 1s

    // Sets the delay the slider waits between movements
    unsigned char msg4[] = { '0', 'v', 'v', '2', '5', '\r', '\n' };
    bytes_written = write(serial_port, msg4, sizeof(msg4));
    usleep(100000); //Sleep for 0.1s

    std::cout << "Serial Port initialized!" << "\n";
    return 1;
}

void lens::return_to_start() {
    // Absolute move of 0x2E00, or 11mm
    ssize_t bytes_written;

    unsigned char msg3[] = { '0', 'm', 'a', '0', '0', '0', '0', '2', 'E', '0', '0', '\r', '\n' };
    bytes_written = write(serial_port, msg3, sizeof(msg3));
    usleep(100*10000);

    // Sets the delay the slider waits between movements
    unsigned char msg4[] = { '0', 'v', 'v', '2', '5', '\r', '\n' };
    bytes_written = write(serial_port, msg4, sizeof(msg4));
    usleep(100*10000);

}

void lens::mov_rel(double mmToMove) {
    ssize_t bytes_written;
    int rate = 1024; //1024 encoder pulses per mm
    int mmUpperBound = 16;
    //double current_pos = get_pos();  //current position in encoder pulses
    double move = rate * mmToMove; //encoder pulses to move
    //double new_pos = move + current_pos;

    //currentLensLoc = currentLensLoc + mmToMove;
    //std::cout << currentLensLoc << std::endl;
    
    //TODO: Replace TRUE with a way to limit the lens moving out-of-bounds
    if (1) {
        std::string pszBufStr = "0mr" + double2hexstr(mmToMove) + "\r\n";
        unsigned char msg[pszBufStr.length()];
        std::copy( pszBufStr.begin(), pszBufStr.end(), msg );
        bytes_written = write(serial_port, msg, sizeof(msg));
//        std::cout << msg << "\n";
    //     //Sleep(20);

    //     int wait_move = 0;
    //     if (wait_move == 1) {
    //         std::vector<char> rxBuf1;
    //         rxBuf1.resize(13); //11 data bits and then 2 for the /l /n which terminates every ASCII string

    //         int timeout = 0;
    //         const DWORD dwRead = 1;

    //         while (timeout < 50) {
    //             const DWORD dwRead = port.Read(rxBuf1.data(), static_cast<DWORD>(rxBuf1.size()));
    //             if (dwRead == 13) {
    //                 break;
    //             }
    //             ++timeout;
    //         }
    //         //std::cout << "Motor stopped";
    //         if (timeout == 50) {
    //             printf("mov_rel function timed out");
    //         }
    //     }
    // }
    }
    else { std::cout << "Out of bounds"; }

    
}

//TODO
double lens::get_pos() {
    // const char* pszBuf = "0gp\r\n";
    // write(serial_port, pszBuf, sizeof(pszBuf));

    // std::vector<char> rxBuf;
    // rxBuf.resize(13); //11 data bits and then 2 for the /l /n which terminates every ASCII string
    
    //const DWORD dwRead = read(rxBuf.data(), static_cast<DWORD>(rxBuf.size()));
    // /*int timeout = 0;
    // const DWORD dwRead = 0;
    // do {
    //     const DWORD dwRead = port.Read(rxBuf.data(), static_cast<DWORD>(rxBuf.size()));
    //     ++timeout;
    // } while (dwRead < 11 && timeout < 5000);

    // if (timeout == 5000) {
    //     printf("Get_pos function timed out");
    //     return 0;
    // }*/
    // rxBuf.resize(11); //gets rid of the /l and /n
    // std::string position{ rxBuf.begin(), rxBuf.end() };
    // std::string hex_pos = position.substr(7, 4);
    // double positiond = hexstr2double(hex_pos);
    // port.TerminateOutstandingWrites();
    // return positiond;

    std::cout << "lens position is <THIS IS A TEST>" << "\n";
    return 2.0;
}
  
  //Converts a hexadecimal string to a double and back again
double lens::hexstr2double (const std::string& hexstr) {
    union
    {
        long long i;
        double    d;
    } value;

    value.i = std::stoll(hexstr, nullptr, 16);
    return value.i;
}

std::string lens::double2hexstr(double x) {
    int num = round(x * 1024);

    std::ostringstream buf;
    buf << std::hex << std::uppercase << std::setfill('0') << std::setw(8) << num;

    return buf.str();
}


void lens::closemotor() {
    // port.ClearWriteBuffer();
    // port.ClearReadBuffer();
    // port.Flush();
    // port.CancelIo();
    // port.Close();

    close(serial_port);
}

lens::~lens() {
    // port.ClearWriteBuffer();
    // port.ClearReadBuffer();
    // port.Flush();
    // port.CancelIo();
    // port.Close();
    std::cout << "serial port closed" << std::endl;
    close(serial_port);
}