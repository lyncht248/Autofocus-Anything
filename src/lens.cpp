#include "lens.hpp"
#include "logfile.hpp"
#include "system.hpp"
#include "main.hpp"
#include "autofocus.hpp"
#include "notificationCenter.hpp"
#include <iostream>
#include <sstream>
#include "Xeryon.h"
#include <fmt/chrono.h>
#include <iomanip>

bool bLensLogFlag = 0;

lens::lens() : stop_thread(false), controller(nullptr), axis(nullptr) {}

bool lens::initialize()
{
    // start lens thread
    tLens = std::thread(&lens::lens_thread, this);

    // Try multiple port options
    const char *portNames[] = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"};
    bool connection_successful = false;

    for (const char *port : portNames)
    {
        if (controller)
        {
            delete controller; // Clean up previous attempt if any
            controller = nullptr;
        }

        if (bLensLogFlag)
            logger->info("[lens::initialize] Trying to connect to {}", port);
        controller = new Xeryon(port, 115200);

        // Try to add the axis first (addAxis doesn't actually need a working port)
        axis = controller->addAxis('X', &XLA_1250_3N);
        if (!axis)
        {
            if (bLensLogFlag)
                logger->error("[lens::initialize] Failed to add axis on {}", port);
            continue;
        }

        // Now call start() to actually open the port
        controller->start();

        // Check if the port opened successfully
        if (controller->isInitialized())
        {
            if (bLensLogFlag)
                logger->info("[lens::initialize] Connected successfully to {}", port);
            connection_successful = true;
            break; // Successfully connected
        }

        if (bLensLogFlag)
            logger->error("[lens::initialize] Failed to connect to {}", port);
    }

    if (!connection_successful)
    {
        logger->error("[lens::initialize] Failed to initialize controller on any port");
        return false;
    }

    // start() was already called, so skip to the configuration

    // enable the lens, then reset (which loads settings), then enable, then find index, then enable
    axis->sendCommand("ENBL", 1);
    axis->sendCommand("RSET", 1);
    // wait for 0.5s
    usleep(500000);
    axis->sendCommand("ENBL", 1);
    axis->findIndex();
    // wait for 0.5s
    usleep(500000);
    axis->sendCommand("ENBL", 1);

    // send each command in settings_default.txt to lens and ensure it is successful

    // axis->sendCommand("INFO", 4);
    // axis->sendCommand("POLI", 97);
    //  axis->sendCommand("FREQ", 87000);
    //  axis->sendCommand("FRQ2", 85000);
    //  axis->sendCommand("HFRQ", 89000);
    //  axis->sendCommand("LFRQ", 83000);

    // Increasing the frequencies to reduce judder. Above 90.5kHz, the lens locks up.
    axis->sendCommand("FREQ", 90000);
    axis->sendCommand("FRQ2", 87000);
    axis->sendCommand("HFRQ", 92000);
    axis->sendCommand("LFRQ", 86000);
    // axis->sendCommand("LLIM",-15);
    // axis->sendCommand("HLIM",15);
    axis->sendCommand("HLIM", 0); // Set a soft limit of 0mm

    // These are the proportional gains for the controller. When we turn the Freq up, we need to turn these down a bit
    // axis->sendCommand("PROP",120);
    // axis->sendCommand("PRO2",40);
    axis->sendCommand("PROP", 90);
    axis->sendCommand("PRO2", 35);

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
    axis->sendCommand("ACCE", 65500);
    axis->sendCommand("DECE", 65500);

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

    // set speed to 200mm/s
    axis->setSpeed(200_mm);
    // wait 0.5s
    usleep(500000);

    // For some reason INDX causes LLIM to be set to -10mm, so we need to set it manually.
    axis->setSetting("LLIM", -14.9_mm);

    axis->setDPOS(-9.1_mm);
    currentLensLoc = -9.1;

    // wait for 0.5s
    usleep(500000);

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

void lens::mov_rel(double mmToMove)
{
    double newLensLoc = currentLensLoc + mmToMove;
    if (newLensLoc > MIN_POSITION && newLensLoc < MAX_POSITION)
    {
        try
        {
            // Convert mm to Distance object
            Distance newLensLocString(newLensLoc, Distance::MM);

            axis->setDPOS(newLensLocString);

            // TODO: remove this
            // Get the actual position of the lens from the controller
            Distance epos = axis->getEPOS();
            double actualPos = epos(Distance::MM);

            // Get current timestamp with microsecond precision
            // First get current time
            auto now = std::chrono::system_clock::now();
            // Convert to time_t for date/time formatting
            auto now_time_t = std::chrono::system_clock::to_time_t(now);
            // Calculate microseconds portion
            auto ms = std::chrono::duration_cast<std::chrono::microseconds>(
                          now - std::chrono::system_clock::from_time_t(now_time_t))
                          .count();

            // Format timestamp as YYYY-MM-DD HH:MM:SS.uuuuuu
            std::stringstream ss;
            ss << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %H:%M:%S");
            ss << '.' << std::setfill('0') << std::setw(6) << ms;

            if (bLensLogFlag) {
                logger->info("[lens::mov_rel] [{}] Requested move: {}mm to position {}mm, Actual position: {}mm",
                              ss.str(), mmToMove, newLensLoc, actualPos);
            }

            currentLensLoc = newLensLoc;

            if (outOfBoundsOnceOnly > 0)
            {
                outOfBoundsOnceOnly--;
            }
            else if (outOfBoundsOnceOnly == 0)
            {
                NotificationCenter::instance().postNotification("lensInBounds");
            }
        }
        catch (const std::exception &e)
        {
            logger->error("[lens::mov_rel] Movement error: " + std::string(e.what()));
        }
    }
    else
    {
        logger->error("[lens::mov_rel] Lens is out of bounds!");
        if (outOfBoundsOnceOnly == 0)
        {
            NotificationCenter::instance().postNotification("outOfBoundsError");
            outOfBoundsOnceOnly = 10;
        }
    }
}

void lens::returnToStart()
{
    try
    {
        std::cout << "returning to start position: " << returnPosition << "mm" << std::endl;
        Distance returnPos(returnPosition, Distance::MM);
        axis->setDPOS(returnPos);
        currentLensLoc = returnPosition;
        // wait for 0.5s
        usleep(500000);
    }
    catch (const std::exception &e)
    {
        logger->error("[lens::returnToStart] Error: " + std::string(e.what()));
    }
}

void lens::setReturnPosition(double position)
{
    // Ensure the position is within bounds
    if (position >= MIN_POSITION && position <= MAX_POSITION)
    {
        returnPosition = position;
        if (bLensLogFlag)
            logger->info("[lens::setReturnPosition] Return position set to {}mm", position);
    }
    else
    {
        if (bLensLogFlag)
            logger->error("[lens::setReturnPosition] Requested position {}mm is out of bounds", position);
    }
}

lens::~lens()
{
    stop_thread.store(true);
    if (tLens.joinable())
        tLens.join();

    if (controller)
    {
        if (axis)
        {
            axis->stop(); // Stop the axis before stopping controller
        }
        controller->stop();
        delete controller;
    }

    if (bLensLogFlag)
        logger->info("[lens::~lens] Lens destructor called, controller stopped");
}

void lens::lens_thread()
{
    if (bLensLogFlag)
        logger->info("[lens::lens_thread] Lens thread started");
    std::cout << "lens thread started" << std::endl;
    while (!stop_thread.load())
    {
        if (bResetLens)
        {
            if (bLensLogFlag)
                logger->info("[lens::lens_thread] Resetting lens to start position");
            returnToStart();
            if (bLensLogFlag)
                logger->info("[lens::lens_thread] Lens reset to start position");
            bResetLens = 0;
        }

        if (bNewMoveRel)
        {
            // if(bLensLogFlag) logger->info("[lens::lens_thread] Moving lens to new position");
            mov_rel(mmToMove);
            // if(bLensLogFlag) logger->info("[lens::lens_thread] Lens moved to new position");
            bNewMoveRel = 0;
        }

        // Small sleep to prevent busy-waiting
        usleep(10000); // 10ms
    }
}
