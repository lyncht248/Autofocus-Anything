#ifndef LENS_H
#define LENS_H
#include <string>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <atomic>
#include <thread>

//int serial_port = open("/dev/ttyUSB0", O_RDWR);

class lens { //This object handles autofocusing
  public:
  lens();
  ~lens();   //Closes the motor serial port and ends thread
  bool initialize(); //true if lens connects, false if not

  int serial_port = open("/dev/ttyUSB0", O_RDWR);

  void return_to_start();

  private:
  void lens_thread(); //Thread that runs the lens

  // Moves the motor by a certain number of mm
  void mov_rel(double mmToMove);

  double currentLensLoc = 11.5;
  
  //Converts a hexadecimal string to an int (hex strings are always whole numbers) and back again
  int hexstr2int (const std::string& hexstr);
  std::string int2hexstr(int pulsesToMove);
  std::thread tLens;
  std::atomic<bool> stop_thread;

  int outOfBoundsOnceOnly = 0;

};

#endif // LENS_H