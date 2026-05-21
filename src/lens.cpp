#include "lens.hpp"
#include "Xeryon.h"
#include "autofocus.hpp"
#include "logfile.hpp"
#include "main.hpp"
#include "notificationCenter.hpp"
#include "settings.hpp"
#include "system.hpp"
#include <filesystem>
#include <fmt/chrono.h>
#include <fstream>
#include <iomanip>
#include <stdexcept>

bool bLensLogFlag = 0;
bool bLensLogFlagSave = 0; // Saves position of lens every time lens moves - ONLY WORKS WHEN bLensLogFlag = 1
                          // Currently only saves when autofocus is on (saves mmToMoveTo)

                          // if you want to save live lens position from hardware, change 
                          // axis->sendCommand("INFO", 0); 
                          // to axis->sendCommand("INFO", 3); in lens::initialize()

                          
lens::lens()
    : stop_thread(false), controller(nullptr), axis(nullptr), settings("") {
  try {
    // Load settings during construction
    settings.load();
    MIN_POSITION = settings.getMinPosition();
    MAX_POSITION = settings.getMaxPosition();
    returnPosition = settings.getReturnPosition();
    FREQ = settings.getFreq();
    FRQ2 = settings.getFRQ2();
    PROP = settings.getPROP();
    PRO2 = settings.getPRO2();
    SSPD = settings.getSSPD();
    ACCE = settings.getACCE();
    DECE = settings.getDECE();
    MASS = settings.getMASS();
    PTOL = settings.getPTOL();
    PTO2 = settings.getPTO2();
    ALPHA = settings.getALPHA();
    BETA = settings.getBETA();
  } catch (const std::exception &e) {
    std::cerr << "[lens::lens] Error loading settings: " << e.what()
              << std::endl;
    throw; // Rethrow exception to indicate critical failure
  }
  // Comment out CSV file initialization
  ///*
  // Create output directory and initialize CSV file
  if (createOutputDirectory()) {
      logFile.open(logFilePath, std::ios::out);
      if (logFile.is_open()) {
          // Write header row
          logFile << "timestamp_ms,LensPosition_mm,targetLensLoc_mm" << std::endl;
          if (bLensLogFlag)
              logger->info("[lens::lens] CSV log file initialized at {}", logFilePath);
      } else {
          logger->error("[lens::lens] Failed to open log file at {}", logFilePath);
      }
  }
  //*/
}

// Comment out createOutputDirectory method
 // /*
bool lens::createOutputDirectory() {
    try {
        if (!std::filesystem::exists(outputDir)) {
            if (std::filesystem::create_directory(outputDir)) {
                if (bLensLogFlag) {
                    logger->info("[lens::createOutputDirectory] Created output directory: {}", outputDir);
                } else {
                    logger->error("[lens::createOutputDirectory] Failed to create output directory: {}", outputDir);
                return false;
                }
            }
        }
        return true;
    } catch (const std::filesystem::filesystem_error& e) {
        logger->error("[lens::createOutputDirectory] Filesystem error: {}", e.what()); 
        return false;
    }
}
 // */

bool lens::initialize() {
  // start lens thread
  tLens = std::thread(&lens::lens_thread, this);

  // Try multiple port options
  const char *portNames[] = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2",
                             "/dev/ttyACM3"};
  bool connection_successful = false;

  for (const char *port : portNames) {
    if (controller) {
      delete controller; // Clean up previous attempt if any
      controller = nullptr;
    }

    if (bLensLogFlag)
      logger->info("[lens::initialize] Trying to connect to {}", port);
    controller = new Xeryon(port, 115200);

    // Try to add the axis first (addAxis doesn't actually need a working port)
    axis = controller->addAxis('X', &XLA_1250_3N);
    if (!axis) {
      if (bLensLogFlag)
        logger->error("[lens::initialize] Failed to add axis on {}", port);
      continue;
    }

    // Now call start() to actually open the port
    controller->start();

    // Check if the port opened successfully
    if (controller->isInitialized()) {
      if (bLensLogFlag)
        logger->info("[lens::initialize] Connected successfully to {}", port);
      connection_successful = true;
      break; // Successfully connected
    }

    if (bLensLogFlag)
      logger->error("[lens::initialize] Failed to connect to {}", port);
  }

  if (!connection_successful) {
    logger->error(
        "[lens::initialize] Failed to initialize controller on any port");
    return false;
  }

  // start() was already called, so skip to the configuration

  // enable the lens, then reset (which loads settings), then enable, then find
  // index, then enable
  axis->sendCommand("ENBL", 1);
  axis->sendCommand("RSET", 1);
  // wait for 0.5s
  axis->sendCommand("ENBL", 1);
  axis->findIndex();
  //    wait for 0.5s
  usleep(500000);
  axis->sendCommand("ENBL", 1);

  // send each command in settings_default.txt to lens and ensure it is
  // successful

  axis->sendCommand("INFO", 0); // no streaming EPOS, only upon request. This
                                // means axis->getEPOS() will not work.
  // axis->sendCommand("INFO", 3); // EPOS, DPOS, STAT being streamed
  // axis->sendCommand("INFO", 7); // EPOS, STAT being streamed

  // axis->sendCommand("POLI", 97);
  // axis->sendCommand("POLI", 40); // ms delay between EPOS samples. Dropping this
                                 // too low interrupts DPOS updates !!

  axis->sendCommand("HFRQ", 89000);
  axis->sendCommand("LFRQ", 83000);

   // Any parameters specified in general_settings.txt is initialised here
  // Now include: FREQ, FRQ2, PROP, PRO2, SSPD, ACCE, DECE, MASS, PTOL, PTO2, POLI, DLAY
  reloadSettings();

  // axis->sendCommand("HLIM", 0);
  //  axis->sendCommand("HLIM", 0); // Set a soft limit of 0mm
  // axis->sendCommand("FREQ", 87000);
  // axis->sendCommand("FRQ2", 85000);
  // axis->sendCommand("HFRQ", 89000);
  // axis->sendCommand("LFRQ", 83000);

  // Increasing the frequencies to reduce judder. Above 90.5kHz, the lens locks
  // up.
  // axis->sendCommand("FREQ", 90500);
  // axis->sendCommand("FRQ2", 88000);
  // axis->sendCommand("FREQ", FREQ);
  // axis->sendCommand("FRQ2", FRQ2);

  // These are the proportional gains for the controller. When we turn the Freq
  // up, we need to turn these down a bit axis->sendCommand("PROP",120);
  // axis->sendCommand("PRO2",40);

  //axis->sendCommand("PROP", 70);
  //axis->sendCommand("PRO2", 30);
  // axis->sendCommand("PROP", PROP);
  // axis->sendCommand("PRO2", PRO2);

  // axis->sendCommand("MPRO",250);
  // axis->sendCommand("INTF",15);
  // axis->sendCommand("MASS",0);
  // axis->sendCommand("MMAS",1000);
  // axis->sendCommand("ZON1",0.01);
  // axis->sendCommand("ZON2",1);

  // axis->sendCommand("MSPD",200);
  // axis->sendCommand("SSPD",100);
  // axis->sendCommand("ISPD",10);
  // axis->sendCommand("ACCE",65500);
  // axis->sendCommand("DECE",65500);
  // axis->sendCommand("ACCE", 6000);
  // axis->sendCommand("DECE", 6000);

  // axis->sendCommand("ILIM",3000);
  // axis->sendCommand("ELIM",0);
  // axis->sendCommand("SLIM",100000);
  // axis->sendCommand("ENCD",0);
  // axis->sendCommand("ACTD",0);
  // axis->sendCommand("ENCO",47);

  // axis->sendCommand("PTOL",2);
  // axis->sendCommand("PTO2",4);

  // axis->sendCommand("TOUT",1000);
  // axis->sendCommand("TOU2",60);
  // axis->sendCommand("TOU3",0);

  // axis->sendCommand("PHAC",0);
  // axis->sendCommand("PHAS",90);
  // axis->sendCommand("DUCO",1);
  // axis->sendCommand("MIMP",20);
  // axis->sendCommand("MAMP",45);
  // axis->sendCommand("AMPL",45);
  // axis->sendCommand("DUTY",32768);
  // axis->sendCommand("OFSA",0);
  // axis->sendCommand("OFSB",0);
  // axis->sendCommand("SQEZ",0);
  // axis->sendCommand("FILE",0);
  // axis->sendCommand("FILG",0);
  // axis->sendCommand("FILA",1);
  // axis->sendCommand("FILP",1);
  // axis->sendCommand("COMP",0);
  // axis->sendCommand("DLAY",10);
  // axis->sendCommand("DTIM",0);
  // axis->sendCommand("ECHO",0);
  // axis->sendCommand("INDA",1);
  // axis->sendCommand("ENBR",1);
  // axis->sendCommand("ENBL",0);
  // axis->sendCommand("GPIO",0);
  // axis->sendCommand("PLIM",0);

  // axis->sendCommand("UART",0);

  // axis->sendCommand("PWMF",1000);
  // axis->sendCommand("TRGS",0);
  // axis->sendCommand("TRGW",0);
  // axis->sendCommand("TRGP",0);
  // axis->sendCommand("TRGN",0);
  // axis->sendCommand("STPS",1);

  // axis->sendCommand("LEAD",0);
  // axis->sendCommand("FLAG",0);
  // axis->sendCommand("VMIN",40000);
  // axis->sendCommand("ECAT",0);
  // axis->sendCommand("BLCK",0);

  // set speed to 250mm/s
  // axis->setSpeed(250_mm);

  // wait 0.5s
  usleep(500000);

  // enable the lens
  axis->sendCommand("ENBL", 1);

  // set LLIM to -15mm
  axis->setSetting("LLIM", -12000); // units are encoder counts
  axis->setSetting("HLIM", 0);      // keep upper at 0 mm
  usleep(500000);

  axis->setDPOS(Distance(returnPosition, Distance::MM));
  currentLensLoc = returnPosition;
  targetLensLoc = returnPosition;

  // wait for 0.5s
  usleep(500000);

  // print the MASS setting, which is an integer
  int massSetting = axis->getSetting("MASS");
  logger->error("[lens::initialize] MASS setting: {}", massSetting);

  // axis->setDPOS(5_mm);
  // std::cout << "moved 5mm" << std::endl;
  // axis->setDPOS(-10_mm);
  // std::cout << "moved -10mm" << std::endl;
  if (bLensLogFlag)
    logger->info("Controller initialized!");
  return true;
}

void lens::setSSPD(double sspd) {
  // Set speed, the input is in um/s, range [0, 250000]
  try {
    Distance speedString(sspd, Distance::MU);
    axis->setSpeed(speedString);
    if (bLensLogFlag)
      logger->info("[lens::setSSPD] SSPD set to {}", sspd);
  } catch (const std::exception &e) {
    logger->error("[lens::setSSPD] Error: " + std::string(e.what()));
  }
}

void lens::mov_rel(double mmToMove) {
  // Shan's version: use STEP to make relative movements
  // Todo: How to determine when lens is out of bound
  try {
    Distance stepString(mmToMove, Distance::MM);
    axis->setSTEP(stepString);

    if (bLensLogFlag) {
      // We can keep the logging but remove the timestamp variable
      logger->info("[lens::mov_rel] Requested relative move: {}mm",
                    mmToMove);
    }

    if (outOfBoundsOnceOnly > 0) {
        outOfBoundsOnceOnly--;
      } else if (outOfBoundsOnceOnly == 0) {
        NotificationCenter::instance().postNotification("lensInBounds");
      }

  } catch (const std::exception &e) {
    logger->error("[lens::mov_rel] Cannot make movement: {}mm", mmToMove);
    if (outOfBoundsOnceOnly == 0) {
      NotificationCenter::instance().postNotification("outOfBoundsError");
      outOfBoundsOnceOnly = 10;
    }
  }
}

void lens::mov_abs(double mmToMoveTo) {
  // Check if the target position is within bounds
  if (mmToMoveTo > MIN_POSITION && mmToMoveTo < MAX_POSITION) {
    try {
      // Convert mm to Distance object
      Distance newLensLocDistance(mmToMoveTo, Distance::MM);

      // Set the new position
      axis->setDPOS(newLensLocDistance);

      // // Get the actual position of the lens from the controller
      // Distance epos = axis->getEPOS();
      // double actualPos = epos(Distance::MM);

      if (bLensLogFlag) {
        auto now = std::chrono::high_resolution_clock::now();
        auto timestamp_ms =
            std::chrono::duration_cast<std::chrono::microseconds>(
                now.time_since_epoch())
                .count() /
            1000.0;
        logger->info("[lens::mov_abs] Moved to absolute position: {}mm, "
                     "Timestamp: {:.3f}ms",
                     mmToMoveTo, timestamp_ms);
      }

      // // Update the current lens location
      // currentLensLoc = mmToMoveTo;
      // currentLensLoc = actualPos; // Shan: this is moved to the other loop

      // If lens is went out of bounds, ensure error message is persistent
      // (avoids flickering)
      if (outOfBoundsOnceOnly > 0) {
        outOfBoundsOnceOnly--;
      } else if (outOfBoundsOnceOnly == 0) {
        NotificationCenter::instance().postNotification("lensInBounds");
      }
    } catch (const std::exception &e) {
      logger->error("[lens::mov_abs] Movement error: " + std::string(e.what()));
    }
  } else {
    logger->error("[lens::mov_abs] Target position {}mm is out of bounds!",
                  mmToMoveTo);
    if (outOfBoundsOnceOnly == 0) {
      NotificationCenter::instance().postNotification("outOfBoundsError");
      outOfBoundsOnceOnly = 10;
    }
  }
}

void lens::returnToStart() {
  try {
    std::cout << "returning to start position: " << returnPosition << "mm"
              << std::endl;
    Distance returnPos(returnPosition, Distance::MM);
    axis->setDPOS(returnPos);
    currentLensLoc = returnPosition;
    // wait for 0.5s
    usleep(500000);
  } catch (const std::exception &e) {
    logger->error("[lens::returnToStart] Error: " + std::string(e.what()));
  }
}

void lens::setReturnPosition(double position) {
  // Ensure the position is within bounds
  if (position >= MIN_POSITION && position <= MAX_POSITION) {
    returnPosition = position;
    if (bLensLogFlag)
      logger->info("[lens::setReturnPosition] Return position set to {}mm",
                   position);
  } else {
    if (bLensLogFlag)
      logger->error(
          "[lens::setReturnPosition] Requested position {}mm is out of bounds",
          position);
  }
}

lens::~lens() {
  stop_thread.store(true);
  if (tLens.joinable())
    tLens.join();

  // /*
  // Close the CSV file
  if (logFile.is_open()) {
      logFile.close();
      if (bLensLogFlag)
          logger->info("[lens::~lens] CSV log file closed");
  }
  // */

  if (controller) {
    if (axis) {
      axis->stop(); // Stop the axis before stopping controller
    }
    controller->stop();
    delete controller;
  }

  if (bLensLogFlag)
    logger->info("[lens::~lens] Lens destructor called, controller stopped");
}

void lens::lens_thread() {

  if (bLensLogFlag)
  logger->info("[lens::lens_thread] Lens thread started");
  std::cout << "lens thread started" << std::endl;

  currentSpeed = -1; // Initialize to invalid speed, ignored in first loop

  while (!stop_thread.load()) {
    // Main loop

    // Print STAT in binary (debugging)
    // long stat_value = axis->getSTAT();
    // std::cout << "STAT (binary): ";
    // for (int i = 23; i >= 0; i--) {
    //     std::cout << ((stat_value >> i) & 1);
    //     if (i % 4 == 0) std::cout << " ";  // Add space every 4 bits for readability
    // }
    // std::cout << std::endl;
    // std::cout << "  Motor On: " << axis->isMotorOn() << "  Position Reached: " << axis->isPositionReached() << std::endl;

    if (bResetLens) {
      if (bLensLogFlag)
        logger->info("[lens::lens_thread] Resetting lens to start position");
      returnToStart();

      if (bLensLogFlag)
        logger->info("[lens::lens_thread] Lens reset to start position");
      bResetLens = 0;

      currentSpeed = -1; // Reset current speed
    }
    
    // Old version
    // if (bNewMoveRel) {
    //   // if(bLensLogFlag) logger->info("[lens::lens_thread] Moving lens to new
    //   // position");
    //   mov_abs(mmToMove + currentLensLoc);
    //   // if(bLensLogFlag) logger->info("[lens::lens_thread] Lens moved to new
    //   // position");
    //   bNewMoveRel = 0;
    // }

    // Shan testing: continuous control

    // Method 1: Direct abs movement control (INFO=3 required)
    // auto timestamp_ms =
    //            std::chrono::duration_cast<std::chrono::microseconds>(
    //                std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    // double currentLensLoc = getLensPosition(); // Update current lens position from encoder
    // if (bNewMoveRel){
    //   // Read mmToMove when autofocus updates it

    //   // Simple calculation- disregard time difference
    //   targetLensLoc = mmToMove + currentLensLoc;

    //   bNewMoveRel = 0; // Signals that the new autofocus calculation has been read
    //   mov_abs(targetLensLoc);
    // }

    // Method 2: Relative movement control (INFO=0 compatible)
    if (bNewMoveRel){
      // Print STAT in binary (debugging)
      // long stat_value = axis->getSTAT();
      // std::cout << "STAT (binary): ";
      // for (int i = 23; i >= 0; i--) {
      //     std::cout << ((stat_value >> i) & 1);
      //     if (i % 4 == 0) std::cout << " ";  // Add space every 4 bits for readability
      // }
      // std::cout << std::endl;
      // std::cout << "  Motor On: " << axis->isMotorOn() << "  Position Reached: " << axis->isPositionReached() << std::endl;
      double nextSpeed = estimateSpeedRequired(mmToMove, currentSpeed);
      currentSpeed = nextSpeed; // Update current speed for next iteration
      setSSPD(nextSpeed);

      mov_rel(mmToMove);
      bNewMoveRel = 0;
    }

    if (bLensLogFlagSave) {
      if (logFile.is_open()) {
        // auto timestamp_ms =
        //           std::chrono::duration_cast<std::chrono::milliseconds>(
        //               std::chrono::system_clock::now().time_since_epoch()).count();
        // double livePosition = getLensPosition();

        // logFile << timestamp_ms << "," << currentLensLoc << "," << targetLensLoc << std::endl;
      }
    }

    // Small sleep to prevent busy-waiting
    usleep(1000); // 1ms
  }
}

// set DPOS using axis
void lens::setDesiredLensPosition(double mmDesiredPosition) {
  // Convert mm to Distance object
  Distance desiredPosition(mmDesiredPosition, Distance::MM);
  axis->setDPOS(desiredPosition);
}

// get DPOS using axis
double lens::getDesiredLensPosition() {
  // Send the command to change to INFO=3 mode so that DPOS is streamed
  axis->sendCommand("INFO", 3);
  // Wait for a short time to ensure the command is processed
  usleep(100000); // 100ms (0.1s)
  Distance dpos = axis->getDPOS();
  double result = dpos(Distance::MM);

  // Send the command to change back to INFO=7 mode
  axis->sendCommand("INFO", 7);
  return result;
}

// get lens position using axis epos (mm)
double lens::getLensPosition() {
  Distance epos = axis->getEPOS();
  return epos(Distance::MM);
}

const Settings &lens::getSettings() const { return settings; }

double lens::estimateSpeedRequired(double mmToMove, double currentSpeed){
  // currentSpeed in um/s, mmToMove in mm
  // Optimized for speed: pre-calculated constants, minimal branching
  
  // Constants (could be moved to class member if called very frequently)
  static constexpr double oneover_DT = 60.0;  // 60Hz control loop
  static constexpr double MAX_SPEED = 250000.0;
  static constexpr double MIN_SPEED = 100.0;
  static constexpr double MM_TO_UM_PER_DT = 1000.0 * oneover_DT;  // 1000 / dt

  // Calculate base speed from movement distance
  double baseSpeed = std::abs(mmToMove) * MM_TO_UM_PER_DT;
  
  // Blend with current speed if valid (currentSpeed >= 0)
  // Otherwise use only baseSpeed (initial case: currentSpeed == -1)
  double nextSpeed = (currentSpeed >= 0) ? 
                     (ALPHA * baseSpeed + BETA * currentSpeed) : 
                     baseSpeed;
  
  // Clamp to valid range
  if (nextSpeed > MAX_SPEED) return MAX_SPEED;
  if (nextSpeed < MIN_SPEED) return MIN_SPEED;
  return nextSpeed;
}

void lens::reloadSettings() {
  try {
    // Reload settings from file
    settings.load();

    // Update internal member variables with new values
    // MIN_POSITION = settings.getMinPosition();
    // MAX_POSITION = settings.getMaxPosition();
    returnPosition = settings.getReturnPosition();
    FREQ = settings.getFreq();
    FRQ2 = settings.getFRQ2();
    PROP = settings.getPROP();
    PRO2 = settings.getPRO2();
    SSPD = settings.getSSPD();
    ACCE = settings.getACCE();
    DECE = settings.getDECE();
    MASS = settings.getMASS();
    PTOL = settings.getPTOL();
    PTO2 = settings.getPTO2();
    POLI = settings.getPOLI();
    DLAY = settings.getDLAY();
    ALPHA = settings.getALPHA();
    BETA = settings.getBETA();

    axis->sendCommand("FREQ", FREQ);
    axis->sendCommand("FRQ2", FRQ2);
    axis->sendCommand("PROP", PROP);
    axis->sendCommand("PRO2", PRO2);
    axis->sendCommand("SSPD", SSPD);
    axis->sendCommand("ACCE", ACCE);
    axis->sendCommand("DECE", DECE);
    axis->sendCommand("MASS", MASS);
    axis->sendCommand("PTOL", PTOL);
    axis->sendCommand("PTO2", PTO2);
    axis->sendCommand("POLI", POLI);
    axis->sendCommand("DLAY", DLAY);

    if (bLensLogFlag) {
      logger->info("[lens::reloadSettings] Settings reloaded - returnPosition: {}mm, FREQ: {}Hz, FRQ2: {}Hz, PROP: {}, PRO2: {}, SSPD: {}, ACCE: {}, DECE: {}, MASS: {}, PTOL: {}, PTO2: {}, POLI: {}, DLAY: {}",
                   returnPosition, FREQ, FRQ2, PROP, PRO2, SSPD, ACCE, DECE, MASS, PTOL, PTO2, POLI, DLAY);
    }
  } catch (const std::exception &e) {
    if (bLensLogFlag) {
      logger->error("[lens::reloadSettings] Error reloading settings: {}",
                    e.what());
    }
  }
}
