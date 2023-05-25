#ifndef LENS_H
#define LENS_H

#include <string>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

//int serial_port = open("/dev/ttyUSB0", O_RDWR);

class lens { //This object handles autofocusing
  public:
  lens();
  ~lens();

  int serial_port = open("/dev/ttyUSB0", O_RDWR);

  // Initializes and opens the motor. Returns 1 if successful, 0 if not
  int initmotor(int PortToUse);


  void return_to_start();

  // Moves the motor by a certain number of mm
  void mov_rel(double mmToMove, bool waitForLensToRead);
  
  private:
  double currentLensLoc = 11.5;
  // Gets the current position of the motor
  double get_pos();
  
  //Converts a hexadecimal string to an int (hex strings are always whole numbers) and back again
  int hexstr2int (const std::string& hexstr);
  std::string int2hexstr(int pulsesToMove);

  //Closes the motor serial port
  void closemotor();
};

#endif // LENS_H