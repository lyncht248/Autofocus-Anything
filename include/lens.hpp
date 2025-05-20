#ifndef LENS_H
#define LENS_H
#include <atomic>
#include <thread>
#include <fstream>
#include <filesystem>
#include "Xeryon.h"
#include "Distance.h"

/**
 * @class lens
 * @brief Controls a Xeryon XLA-5-65-1250 piezomotor actuator with mounted lens. Wrapper for the Xeryon
 * and Axis classes, which come from the Xeryon SDK.
 * 
 * This class provides an interface to control the Xeryon piezo motor lens actuator.
 * It handles initialization, position control, and movement within defined boundaries.
 * The lens is primarily used by the autofocus system to maintain focus on the imaging camera.
 */
class lens {
public:
    /**
     * @brief Default constructor
     * 
     * Initializes member variables to their default values.
     */
    lens();
    
    /**
     * @brief Destructor
     * 
     * Stops the lens thread and cleans up resources.
     */
    ~lens();
    
    /**
     * @brief Initializes the lens controller
     * 
     * Attempts to connect to the Xeryon controller on various port options,
     * configures the lens axis with appropriate settings, and sets the
     * starting position.
     * 
     * @return true if initialization was successful, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Returns the lens to the starting position
     * 
     * Moves the lens to the stored return position value.
     */
    void returnToStart();
    
    /**
     * @brief Sets a new return position
     * 
     * @param position New return position in mm (must be within bounds)
     */
    void setReturnPosition(double position);
    
    /**
     * @brief Sets the desired lens position
     * 
     * @param mmDesiredPosition Desired position in mm
     */
    void setDesiredLensPosition(double mmDesiredPosition);
    
    /**
     * @brief Gets the current lens position in mm, using the encoder.
     * 
     * @return Current lens position in mm
     */
    double getLensPosition();
    
    /**
     * @brief Gets the desired lens position in mm. This method is NOT optimized for performance; avoid using it.
     * 
     * @return Desired lens position in mm
     */
    double getDesiredLensPosition();

    /**
     * @brief Sends a command directly to the axis controller
     * 
     * @param command The command string
     * @param value The value to set
     */
    void sendAxisCommand(const std::string& command, int value);

    /**
     * @brief Gets the current INFO mode
     * 
     * @return Current INFO mode value
     */
    int getInfoMode();

    /**
     * @brief Moves the lens by a relative amount, with bounds checking
     * 
     * @param mmToMove Distance to move in mm (positive or negative)
     */
    void mov_rel(double mmToMove);

    /**
     * @brief Moves the lens to a specific position, with bounds checking
     * 
     * @param mmToMoveTo Location to move to in mm (positive or negative)
     */
    void mov_abs(double mmToMoveTo);
    
private:
    /**
     * @brief Thread function for lens control
     * 
     * This function runs in a separate thread and continuously monitors
     * for lens movement commands and reset requests.
     */
    void lens_thread();
    
    /**
     * @brief Creates the output directory for logs
     * 
     * @return true if the directory exists or was created successfully, false otherwise
     */
    bool createOutputDirectory();
    
    Xeryon* controller; ///< Pointer to the Xeryon controller object
    Axis* axis;         ///< Pointer to the axis object for lens control
    
    double currentLensLoc; ///< Current position of the lens in mm
    double returnPosition = -9.1; ///< Return position in mm (default: -9.1)

    std::thread tLens;           ///< Thread for lens control
    std::atomic<bool> stop_thread; ///< Flag to signal thread termination
    
    int outOfBoundsOnceOnly = 0; ///< Counter to limit out-of-bounds error notifications
    const double MIN_POSITION = -14.9; ///< Minimum allowed position in mm (right limit)
    const double MAX_POSITION = 0.0;   ///< Maximum allowed position in mm (left limit)

    std::ofstream logFile;           ///< File stream for position logging
    const std::string outputDir = "../output";  ///< Directory for log files
    const std::string logFilePath = outputDir + "/lens_positions.csv"; ///< Path to lens position log file
};

#endif // LENS_H