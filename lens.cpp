#include "lens.hpp"
#include "logfile.hpp"
#include "system.hpp"
#include "main.hpp"
#include "autofocus.hpp"
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

lens::lens() :
    stop_thread(false)
{

    // Check for errors
    if (serial_port < 0) {
        hvigtk_logfile << "Error opening serial port \n";
    }

      // Create new termios struct, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        hvigtk_logfile << "Error opening serial port \n";
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
        hvigtk_logfile << "failure to save tty settings for lens motor \n";
    }

    hvigtk_logfile << "Serial Port initialized! \n";

    // Start a thread to send and recieve signals from the lens motor asynchronously
    std::thread tLens(&lens::lens_thread, this);
    tLens.detach();
}

void lens::lens_thread() {
    ssize_t bytes_written; //Gets rid of the warn_unused_result error

    // Sets the home location offset to be 1mm (or __ encoder pulses, or 0x0400) from the factory home location. Doesn't move lens. 
    unsigned char msg[] = { '0', 's', 'o', '0', '0', '0', '0', '0', '4', '0', '0', '\r', '\n' };
    bytes_written = write(serial_port, msg, sizeof(msg));
    usleep(1000000); //Sleep for 1s

    // The lens will move to the offset home position, quickly move to the home position for callibration (end of the slide right), and then back to the offset home position.
    unsigned char msg2[] = { '0', 'h', 'o', '0', '\r', '\n' };
    bytes_written = write(serial_port, msg2, sizeof(msg2));
    usleep(1000000); //Sleep for 1s

    return_to_start(); // Moves lens to callibrated start position

    while(!stop_thread.load()) {
        if(bNewMoveRel) {
            mov_rel(mmToMove);
            bNewMoveRel = 0;
        }
    }
}

void lens::return_to_start() {
    // Absolute move of 0x2E00, or 11.5mm, relative to the offset home position
    ssize_t bytes_written;
    unsigned char msg3[] = { '0', 'm', 'a', '0', '0', '0', '0', '2', 'E', '0', '0', '\r', '\n' };
    bytes_written = write(serial_port, msg3, sizeof(msg3));
    usleep(1000000); //Sleep for 1s

    // Sets the delay the slider waits between movements
    unsigned char msg4[] = { '0', 'v', 'v', '2', '5', '\r', '\n' };
    bytes_written = write(serial_port, msg4, sizeof(msg4));
    usleep(100000); //Sleep for 0.1s

    currentLensLoc = 11.5;
}

void lens::mov_rel(double mmToMove) {
    ssize_t bytes_written;
    int rate = 1024; //1024 encoder pulses per mm
    int pulsesToMove = round(mmToMove * rate);
    std::string hexToMove = int2hexstr(pulsesToMove);
    std::cout << currentLensLoc + mmToMove << std::endl;

    // Check if the move is within the bounds of the lens
    if (currentLensLoc + mmToMove > 0.0 && currentLensLoc + mmToMove < 16.6) {
        
        // Send move signal
        std::string pszBufStr = "0mr" + hexToMove + "\r\n";
        unsigned char msg[pszBufStr.length()];
        std::copy( pszBufStr.begin(), pszBufStr.end(), msg );
        bytes_written = write(serial_port, msg, sizeof(msg));

        // Read response from lens, which includes the current lens position
        int tryagain = 2;
        char buf[256];
        int n_read = 0;
        bool line_received = false;
        bool successfulmove = false;
        while(!successfulmove){
            //Reads one bit at a time until a new line recieved (end of message)
            while (!line_received && n_read < sizeof(buf)) {
                int n = read(serial_port, &buf[n_read], 1);
                if (n < 0) {
                    std::cerr << "Error reading from serial port\n";
                    return;
                }
                if (buf[n_read] == '\n') {
                    line_received = true;
                }
                n_read += n;
            }
            if (line_received) {
                buf[n_read-1] = '\0'; // remove the newline character from the buffer
                std::cout << "Received line: " << buf << std::endl;
            } else {
                std::cerr << "Error: no line terminator received\n";
                return;
            }
            //If 'P' is recieved, the move was successful
            if (buf[1] == 'P') {
                successfulmove = true;
                if (buf[3] != 'F') {
                    // Get lens position from response
                    std::string positionHex = std::string(buf).substr(7, 4);
                    std::cout << positionHex << std::endl;
                    // Convert positionHex to decimal
                    int position = hexstr2int(positionHex);
                    currentLensLoc = position / 1024.0;
                }
            }
            //If 'G' or something else was recieved, the move wasn't successful or is in-progress. Try again
            else if (tryagain > 0) {
                tryagain--;
                std::cout << "Lens error in lens::mov_rel()... trying again" << std::endl;
            }
            else {
                std::cout << "Lens error in lens::mov_rel()... giving up." << std::endl;
                successfulmove = 1;
            }
        }
    }
    else {
        std::cout << "Lens is out of bounds!" << std::endl;
        //m_errorSignal.emit();
    }
}
  
// converts a hexadecimal string to an int
int lens::hexstr2int (const std::string& hexstr) {
    union
    {
        int i;
        double    d;
    } value;

    value.i = std::stoll(hexstr, nullptr, 16);
    return value.i;
}

std::string lens::int2hexstr(int x) {

    std::ostringstream buf;
    buf << std::hex << std::uppercase << std::setfill('0') << std::setw(8) << x;

    return buf.str();
}

lens::~lens() {
    stop_thread.store(true);
    if (tLens.joinable())
        tLens.join();
    close(serial_port);
    hvigtk_logfile << "serial port closed and Lens thread ended in lens destructor" << std::endl;
}