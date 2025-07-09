#ifndef LENS_H
#define LENS_H

#include <string>

class lens { //This object handles autofocusing
  public:

  // Initializes and opens the motor
  int initmotor(int PortToUse);

  // Moves the motor by a certain number of mm
  void mov_rel(double mmToMove);
  
  private:

  // Gets the current position of the motor
  double get_pos();
  
  //Converts a hexadecimal string to a double and back again
  double hexstr2double (const std::string& hexstr);
  std::string double2hexstr(double x);

  //Closes the motor serial port
  void closemotor();
};

#endif // LENS_H