/**
 * @file lens.hpp
 * @brief Provides an interface for controlling the Xeryon piezo motor lens
 * actuator.
 *
 * This file defines the `lens` class, which manages initialization, position
 * control, and movement of the lens actuator within defined boundaries. It is
 * used by the autofocus system to maintain focus on the imaging camera.
 */

#ifndef LENS_H
#define LENS_H
#include "Distance.h"
#include "Xeryon.h"
#include "settings.hpp" // Include Settings header
#include <atomic>
#include <filesystem>
#include <fstream>
#include <thread>

/**
 * @class lens
 * @brief Controls a Xeryon XLA-5-65-1250 piezomotor actuator with mounted lens.
 * Wrapper for the Xeryon and Axis classes, which come from the Xeryon SDK.
 *
 * This class provides an interface to control the Xeryon piezo motor lens
 * actuator. It handles initialization, position control, and movement within
 * defined boundaries. The lens is primarily used by the autofocus system to
 * maintain focus on the imaging camera.
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
   * @brief Moves the lens to the predefined starting position.
   *
   * This method moves the lens to the position stored in the `returnPosition`
   * variable. It updates the `currentLensLoc` to reflect the new position and
   * waits briefly to ensure the movement completes. If an error occurs during
   * the process, it logs the error.
   *
   * @throws std::exception If the movement command fails.
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
   * @brief Gets the desired lens position in mm. This method is NOT optimized
   * for performance; avoid using it.
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
  void sendAxisCommand(const std::string &command, int value);

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

  /**
   * @brief Gets the Settings object
   *
   * Provides access to the Settings object for retrieving configuration values.
   *
   * @return Reference to the Settings object
   */
  const Settings &getSettings() const;

  /**
   * @brief Reloads settings from the settings file
   *
   * Updates the internal position limits and return position with current
   * values from the settings file. This allows runtime updates without
   * restarting.
   */
  void reloadSettings();

  void logLivePositionToCSV();

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
   * @return true if the directory exists or was created successfully, false
   * otherwise
   */
  bool createOutputDirectory();

  Xeryon *controller; ///< Pointer to the Xeryon controller object
  Axis *axis;         ///< Pointer to the axis object for lens control

  double currentLensLoc; ///< Current position of the lens in mm
  double returnPosition; ///< Return position in mm, configurable via settings
                         ///< text file default = -9.1

  std::thread tLens;             ///< Thread for lens control
  std::atomic<bool> stop_thread; ///< Flag to signal thread termination

  Settings settings; ///< Settings object to manage configuration

  int outOfBoundsOnceOnly =
      0;               ///< Counter to limit out-of-bounds error notifications
  double MIN_POSITION; ///< Minimum allowed position in mm (right limit),
                       ///< configurable via settings (default = -14.9)
  double MAX_POSITION; ///< Maximum allowed position in mm (left limit),
                       ///< configurable via settings (default = 0.0)

  double FREQ;  ///< Frequency setting for the lens controller, configurable via
                ///< settings
  double FRQ2;  ///< Secondary frequency setting for the lens controller,
                ///< configurable via settings
  double PROP;  ///< Lower gain constant for the lens controller,
  double PRO2;  ///< Higher gain constant for the lens controller,
                ///< configurable via settings

  std::ofstream logFile; ///< File stream for position logging
  const std::string outputDir = "../output"; ///< Directory for log files
  const std::string logFilePath =
      outputDir + "/lens_positions.csv"; ///< Path to lens position log file
};

#endif // LENS_H