#ifndef LENS_OLD_H
#define LENS_OLD_H
#include <string>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <atomic>
#include <thread>

/**
 * @class lens_old
 * @brief Controls the old Thorlabs actuator via serial communication
 * 
 * This class provides an interface to control the old Thorlabs actuator
 * using serial port communication with hexadecimal commands.
 */
class lens_old {
public:
    /**
     * @brief Default constructor
     */
    lens_old();
    
    /**
     * @brief Destructor - closes serial port and ends thread
     */
    ~lens_old();
    
    /**
     * @brief Initializes the lens controller
     * @return true if lens connects successfully, false if not
     */
    bool initialize();
    
    /**
     * @brief Returns the lens to the starting position
     */
    void returnToStart();
    
    /**
     * @brief Sets a new return position (compatibility method)
     * @param position New return position in mm
     */
    void setReturnPosition(double position) { /* Not implemented for old actuator */ }
    
    /**
     * @brief Gets the current lens position in mm
     * @return Current lens position in mm
     */
    double getLensPosition() { return currentLensLoc; }
    
    /**
     * @brief Moves the lens by a relative amount
     * @param mmToMove Distance to move in mm (positive or negative)
     */
    void mov_rel(double mmToMove);
    
    /**
     * @brief Moves the lens to an absolute position
     * @param mmToMoveTo Target position in mm
     */
    void mov_abs(double mmToMoveTo);

private:
    /**
     * @brief Thread function for lens control
     */
    void lens_thread();
    
    /**
     * @brief Returns lens to start position (internal method)
     */
    void return_to_start();
    
    /**
     * @brief Converts hexadecimal string to integer
     * @param hexstr Hexadecimal string
     * @return Integer value
     */
    int hexstr2int(const std::string& hexstr);
    
    /**
     * @brief Converts integer to hexadecimal string
     * @param pulsesToMove Integer value to convert
     * @return Hexadecimal string
     */
    std::string int2hexstr(int pulsesToMove);
    
    int serial_port = open("/dev/ttyUSB0", O_RDWR); ///< Serial port handle
    double currentLensLoc = 11.5;                   ///< Current lens position in mm
    std::atomic<bool> stop_thread;                  ///< Thread stop flag
    int outOfBoundsOnceOnly = 0;                   ///< Out of bounds error counter
    
    // Position limits for old actuator
    const double MIN_POSITION = 0.0;   ///< Minimum position in mm
    const double MAX_POSITION = 6.5;   ///< Maximum position in mm
};

#endif // LENS_OLD_H 