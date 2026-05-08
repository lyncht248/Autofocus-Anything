#include "lens_old.hpp"
#include "logfile.hpp"
#include "system.hpp"
#include "main.hpp"
#include "autofocus.hpp"
#include "notificationCenter.hpp"
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
#include <atomic>

bool bLensOldLogFlag = 1; // Flag for old lens logging

// Access to global autofocus variables
extern bool bResetLens;
extern bool bNewMoveRel;
extern std::atomic<double> mmToMove;

lens_old::lens_old() : stop_thread(false) {
}

bool lens_old::initialize() {
    bool noErrors = true;
    
    // Check for errors
    if (serial_port < 0) {
        logger->error("[lens_old::initialize] Error opening serial port");
        noErrors = false;
    }

    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if (tcgetattr(serial_port, &tty) != 0) {
        logger->error("[lens_old::initialize] Error opening serial port");
        noErrors = false;
    }

    tty.c_cflag &= ~PARENB;                    // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;                    // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;                     // Clear all bits that set the data size
    tty.c_cflag |= CS8;                        // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;                   // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL;             // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;  // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);    // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

    tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        logger->error("[lens_old::initialize] Error saving serial port settings");
        noErrors = false;
    }

    if (noErrors) {
        if (bLensOldLogFlag)
            logger->info("Serial Port initialized!");

        // Start a thread to send and receive signals from the lens motor asynchronously
        if (bLensOldLogFlag)
            logger->info("[lens_old::initialize] Starting and detaching lens thread");
        std::thread tLens(&lens_old::lens_thread, this);
        tLens.detach();
        return true;
    } else {
        NotificationCenter::instance().postNotification("LensDisconnected");
        logger->error("[lens_old::initialize] Error opening serial port");
        return false;
    }
}

void lens_old::lens_thread() {
    ssize_t bytes_written; // Gets rid of the warn_unused_result error
    char buf[256];
    int n_read = 0;
    bool line_received = false;

    // Sets the home location offset to be 1mm (or __ encoder pulses, or 0x0400) from the factory home location. Doesn't move lens.
    unsigned char msg[] = {'0', 's', 'o', '0', '0', '0', '0', '0', '4', '0', '0', '\r', '\n'};
    bytes_written = write(serial_port, msg, sizeof(msg));
    if (bLensOldLogFlag)
        logger->info("[lens_old::lens_thread] Sets home location to be 1mm, 0so00000400");
    usleep(100000); // Sleep for 0.1s

    while (!line_received && n_read < sizeof(buf)) {
        int n = read(serial_port, &buf[n_read], 1);
        if (n < 0) {
            logger->error("[lens_old::lens_thread] Error reading from serial port");
            return;
        }
        if (buf[n_read] == '\n') {
            line_received = true;
        }
        n_read += n;
    }
    if (line_received) {
        buf[n_read - 1] = '\0'; // remove the newline character from the buffer
    } else {
        logger->error("[lens_old::lens_thread] Error: no line terminator received");
        return;
    }

    // The lens will move to the offset home position, quickly move to the home position for calibration (end of the slide right), and then back to the offset home position.
    unsigned char msg2[] = {'0', 'h', 'o', '0', '\r', '\n'};
    bytes_written = write(serial_port, msg2, sizeof(msg2));
    usleep(1000000); // Sleep for 1s, can't go lower than this...

    line_received = false;
    n_read = 0;
    while (!line_received && n_read < sizeof(buf)) {
        int n = read(serial_port, &buf[n_read], 1);
        if (n < 0) {
            logger->error("[lens_old::lens_thread] Error reading from serial port");
            return;
        }
        if (buf[n_read] == '\n') {
            line_received = true;
        }
        n_read += n;
    }
    if (line_received) {
        buf[n_read - 1] = '\0'; // remove the newline character from the buffer
    } else {
        logger->error("[lens_old::lens_thread] Error: no line terminator received");
        return;
    }

        return_to_start(); // Moves lens to calibrated start position
    
    while (!stop_thread.load()) {
        if (bResetLens) {
            return_to_start();
            bResetLens = 0;
        }

        if (bNewMoveRel) {
            mov_rel(mmToMove.load());
            bNewMoveRel = 0;
        }
    }
}

void lens_old::return_to_start() {
    ssize_t bytes_written;
    char buf[256];
    int n_read = 0;
    bool line_received = false;
    int repeatThreeTimes = 4;

    // unsigned char msg3[] = {'0', 'm', 'a', '0', '0', '0', '0', '0', 'A', '0', '0', '\r', '\n'}; // 0A00 hex is 2560 decimal divided by 1024 = 2.5mm home position
    unsigned char msg3[] = {'0', 'm', 'a', '0', '0', '0', '0', '0', 'F', 'A', '0', '\r', '\n'}; // 0FA0 hex is 4000 decimal divided by 1024 = 3.9mm home position
    bytes_written = write(serial_port, msg3, sizeof(msg3));
    usleep(250000); // Sleep for 0.25s
    while (repeatThreeTimes > 0) {
        repeatThreeTimes--;
        while (!line_received && n_read < sizeof(buf)) {
            int n = read(serial_port, &buf[n_read], 1);
            if (n < 0) {
                logger->error("[lens_old::return_to_start] Error reading from serial port");
                return;
            }
            if (buf[n_read] == '\n') {
                line_received = true;
            }
            n_read += n;
        }
        if (line_received) {
            buf[n_read - 1] = '\0'; // remove the newline character from the buffer
            //std::cout << "Received line: " << buf << std::endl;
        } else {
            logger->error("[lens_old::return_to_start] Error: no line terminator received");
            return;
        }
    }

    // Sets the delay the slider waits between movements
    unsigned char msg4[] = {'0', 'v', 'v', '2', '5', '\r', '\n'};
    bytes_written = write(serial_port, msg4, sizeof(msg4));
    usleep(100000); // Sleep for 0.1s

    while (!line_received && n_read < sizeof(buf)) {
        int n = read(serial_port, &buf[n_read], 1);
        if (n < 0) {
            logger->error("[lens_old::return_to_start] Error reading from serial port");
            return;
        }
        if (buf[n_read] == '\n') {
            line_received = true;
        }
        n_read += n;

    }
    if (line_received) {
        buf[n_read - 1] = '\0'; // remove the newline character from the buffer
        // std::cout << "Received line: " << buf << std::endl;
    } else {
        logger->error("[lens_old::return_to_start] Error: no line terminator received");
        return;
    }

    // currentLensLoc = 2.5;
    currentLensLoc = 3.9;
    if (bLensOldLogFlag)
        logger->info("[lens_old::return_to_start] Lens returned to start position");
}

void lens_old::mov_rel(double mmToMove) {
    ssize_t bytes_written;
    int rate = 1024;                                          // 1024 encoder pulses per mm
    int pulsesToMove = round(mmToMove * rate);
    std::string hexToMove = int2hexstr(pulsesToMove);
    std::cout << "Lens about to be instructed to mov_rel " << mmToMove << "mm" << std::endl;
    std::cout << "Absolute position after move should be " << currentLensLoc + mmToMove << "mm" << std::endl;

    if (currentLensLoc + mmToMove > MIN_POSITION && currentLensLoc + mmToMove < MAX_POSITION) {
    //if(TRUE) {
        // Send move signal
        std::string pszBufStr = "0mr" + hexToMove + "\r\n";
        unsigned char msg[pszBufStr.length()];
        std::copy(pszBufStr.begin(), pszBufStr.end(), msg);
        bytes_written = write(serial_port, msg, sizeof(msg));

        // Read response from lens, which includes the current lens position
        int tryagain = 2;
        char buf[256];
        int n_read = 0;
        bool line_received = false;
        bool successfulmove = false;
        while (!successfulmove) {
            // Reads one bit at a time until a new line received (end of message). NOTE: Position seems to be delayed from expected
            while (!line_received && n_read < sizeof(buf)) {
                int n = read(serial_port, &buf[n_read], 1);
                if (n < 0) {
                    logger->error("[lens_old::mov_rel] Error reading from serial port");
                    return;
                }
                if (buf[n_read] == '\n') {
                    line_received = true;
                }
                n_read += n;
            }
            if (line_received) {
                buf[n_read - 1] = '\0'; // remove the newline character from the buffer
            } else {
                logger->error("[lens_old::mov_rel] Error: no line terminator received");
                return;
            }
            //If 'P' is recieved, the move was successful. 
            if (buf[1] == 'P') {
                successfulmove = true;
                if (outOfBoundsOnceOnly > 0) {
                    outOfBoundsOnceOnly--;
                    // Send recovery notification when lens is back in bounds
                    if (outOfBoundsOnceOnly == 0) {
                        NotificationCenter::instance().postNotification("lensInBounds");
                    }
                }
                if (buf[3] != 'F') {
                    // Get lens position from response
                    std::string positionHex = std::string(buf).substr(7, 4);
                    // Convert positionHex to decimal
                    int position = hexstr2int(positionHex);
                    currentLensLoc = position / 1024.0;
                    // std::cout << "Given Lens Location after move (?) is: " << currentLensLoc << std::endl;
                }
            }
            //If 'G' or something else was recieved, the move wasn't successful or is in-progress. Try again
            else if (tryagain > 0) {
                tryagain--;
                // std::cout << "Lens error in lens::mov_rel()... trying again" << std::endl;
                if(bLensOldLogFlag) logger->info("[lens_old::mov_rel] Lens read error... trying again");
            }
            else {
                // std::cout << "Lens error in lens::mov_rel()... giving up." << std::endl;
                if(bLensOldLogFlag) logger->info("[lens_old::mov_rel] Lens read error... giving up.");
                successfulmove = 1;
            }
        }
    } else {
        logger->error("[lens_old::mov_rel] Lens is out of bounds!");
        if (outOfBoundsOnceOnly == 0) {
            NotificationCenter::instance().postNotification("outOfBoundsError");
            outOfBoundsOnceOnly = 5; // Five successful moves means lens has returned from being out-of-bounds
        }
    }
}

void lens_old::mov_abs(double mmToMoveTo) {
    // Calculate relative move needed
    double mmToMove = mmToMoveTo - currentLensLoc;
    mov_rel(mmToMove);
}

void lens_old::returnToStart() {
    return_to_start();
}

// converts a hexadecimal string to an int
int lens_old::hexstr2int(const std::string &hexstr) {
    union {
        int i;
        double d;
    } value;

    value.i = std::stoll(hexstr, nullptr, 16);
    return value.i;
}

std::string lens_old::int2hexstr(int x) {
    std::ostringstream buf;
    buf << std::hex << std::uppercase << std::setfill('0') << std::setw(8) << x;
    return buf.str();
}

lens_old::~lens_old() {
    stop_thread.store(true);
    close(serial_port);
    if(bLensOldLogFlag) {logger->info("[lens_old::~lens_old] Lens destructor called, lens thread ended, serial port closed");}
} 