#include "lens.hpp"
#include "logfile.hpp"
#include "system.hpp"
#include "main.hpp"
#include "autofocus.hpp"
#include "notificationCenter.hpp"
#include <iostream>
#include <sstream>
#include "Xeryon.h"

bool bLensLogFlag = 1;

lens::lens() : stop_thread(false), controller(nullptr), axis(nullptr) {}

bool lens::initialize() {

    // start lens thread
    tLens = std::thread(&lens::lens_thread, this);

    controller = new Xeryon("/dev/ttyACM0", 115200);
    if (controller == nullptr) {
        controller = new Xeryon("/dev/ttyACM1", 115200);
    }
    if (controller == nullptr) {
        logger->error("[lens::initialize] Failed to initialize controller");
        return false;
    }

    axis = controller->addAxis('X', &XLA_1250_3N);
    if (axis == nullptr) {
        logger->error("[lens::initialize] Failed to create axis");
        return false;
    }

    controller->start();

    // enable the lens, then reset (which loads settings), then enable, then find index, then enable
    axis->sendCommand("ENBL",1);
    axis->sendCommand("RSET",1);
    // wait for 0.5s
    usleep(500000);
    axis->sendCommand("ENBL",1);
    axis->findIndex();
    // wait for 0.5s
    usleep(500000);
    axis->sendCommand("ENBL",1);    


    // // send each command in settings_default.txt to lens and ensure it is successful
    axis->sendCommand("INFO",4);
    axis->sendCommand("POLI",97);
    axis->sendCommand("FREQ",90000);
    axis->sendCommand("FRQ2",88000);
    axis->sendCommand("HFRQ",92000);
    axis->sendCommand("LFRQ",86000);
    // axis->sendCommand("LLIM",-15);
    // axis->sendCommand("HLIM",15);
    // axis->sendCommand("PROP",120);
    // axis->sendCommand("PRO2",40);
    // axis->sendCommand("MPRO",250);
    // axis->sendCommand("INTF",15);
    // axis->sendCommand("MASS",0);
    // axis->sendCommand("MMAS",1000);
    // axis->sendCommand("ZON1",0.01);
    // axis->sendCommand("ZON2",1);

    // axis->sendCommand("MSPD",200);
    //axis->sendCommand("SSPD",100);
    // axis->sendCommand("ISPD",10);
    axis->sendCommand("ACCE",65500);
    axis->sendCommand("DECE",65500);

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

    //set speed to 200mm/s
    axis->setSpeed(200_mm);

    // For some reason INDX causes LLIM to be set to 0, so we need to set it manually. The encoder resolution is 1250

    axis->setSetting("LLIM", -14.9_mm);


    axis->setDPOS(-9_mm);
    currentLensLoc = -9.0;

    // wait for 0.5s
    usleep(500000);

    // //left limit
    // axis->setDPOS(10_mm);
    // currentLensLoc = 10.0;


    // axis->setDPOS(-10_mm);
    // currentLensLoc = -10.0;


    //wait for 0.5s
    usleep(500000);

    // After setting position
    Distance epos = axis->getEPOS();
    double actualPos = epos(Distance::MM);
    logger->info("[lens::initialize] Requested position: {}mm, Actual position: {}mm", 14.0, actualPos);

    // axis->setDPOS(5_mm);
    // std::cout << "moved 5mm" << std::endl;
    // axis->setDPOS(-10_mm);
    // std::cout << "moved -10mm" << std::endl;
    if(bLensLogFlag) logger->info("Controller initialized!");
    return true;
}

void lens::mov_rel(double mmToMove) {
    double newLensLoc = currentLensLoc + mmToMove;
    if (newLensLoc > MIN_POSITION && newLensLoc < MAX_POSITION) {
        try {
            
            // We need to use as high a frequency as possible to avoid judder, but 
            // high frequencies also cause resonance/oscillation. This driving frequency regime seems to 
            // minimize judder while avoiding oscillation. 

            // Check if we're crossing a frequency boundary
            bool wasInMiddleZone = (currentLensLoc >= -13.7 && currentLensLoc <= -7.4);
            bool willBeInMiddleZone = (newLensLoc >= -13.7 && newLensLoc <= -7.4);
            
            // Only change frequencies if crossing boundary
            if (wasInMiddleZone != willBeInMiddleZone) {
                if (willBeInMiddleZone) {
                    // Moving into the -7.4 to -12 zone - use lower frequencies
                    axis->sendCommand("FREQ", 89000);
                    axis->sendCommand("FRQ2", 87000);
                    axis->sendCommand("HFRQ", 91000);
                    axis->sendCommand("LFRQ", 85000);
                    if(bLensLogFlag) logger->error("[lens::mov_rel] Switching to lower frequencies for -8mm to -12mm zone");
                } else {
                    // Moving to outer zone - use higher frequencies
                    axis->sendCommand("FREQ", 90000);
                    axis->sendCommand("FRQ2", 88000);
                    axis->sendCommand("HFRQ", 92000);
                    axis->sendCommand("LFRQ", 86000);
                    if(bLensLogFlag) logger->error("[lens::mov_rel] Switching to higher frequencies for outer zone");
                }
            }

            // Convert mm to Distance object
            Distance newLensLocString(newLensLoc, Distance::MM);

            axis->setDPOS(newLensLocString);
            
            //TODO: remove this
            Distance epos = axis->getEPOS();
            double actualPos = epos(Distance::MM);
            logger->error("[lens::mov_rel] Requested move: {}mm to position {}mm, Actual position: {}mm", mmToMove, newLensLoc, actualPos);

            currentLensLoc = newLensLoc;
            
            if(outOfBoundsOnceOnly > 0) {
                outOfBoundsOnceOnly--;
            }
            else if(outOfBoundsOnceOnly == 0) {
                NotificationCenter::instance().postNotification("lensInBounds");
            }
        }
        catch (const std::exception& e) {
            logger->error("[lens::mov_rel] Movement error: " + std::string(e.what()));
        }
    }
    else {
        logger->error("[lens::mov_rel] Lens is out of bounds!");
        if (outOfBoundsOnceOnly == 0) {
            NotificationCenter::instance().postNotification("outOfBoundsError");
            outOfBoundsOnceOnly = 10;
        }
    }
}

void lens::return_to_start() {
    try {
        std::cout << "returning to start" << std::endl;
        //axis->findIndex();
        axis->setDPOS(-9_mm);
        currentLensLoc = -9.0;
        //wait for 0.5s
        usleep(500000);
        
    }
    catch (const std::exception& e) {
        logger->error("[lens::return_to_start] Error: " + std::string(e.what()));
    }
}

lens::~lens() {
    stop_thread.store(true);
    if (tLens.joinable())
        tLens.join();
        
    if (controller) {
        if (axis) {
            axis->stop();  // Stop the axis before stopping controller
        }
        controller->stop();
        delete controller;
    }
    
    if(bLensLogFlag) 
        logger->info("[lens::~lens] Lens destructor called, controller stopped");
}

void lens::lens_thread() {
    if(bLensLogFlag) logger->info("[lens::lens_thread] Lens thread started");
    std::cout << "lens thread started" << std::endl;
    while(!stop_thread.load()) {
        if(bResetLens) {
            if(bLensLogFlag) logger->info("[lens::lens_thread] Resetting lens to start position");
            return_to_start();
            if(bLensLogFlag) logger->info("[lens::lens_thread] Lens reset to start position");
            bResetLens = 0;
        }

        if(bNewMoveRel) {
            //if(bLensLogFlag) logger->info("[lens::lens_thread] Moving lens to new position");
            mov_rel(mmToMove);
            //if(bLensLogFlag) logger->info("[lens::lens_thread] Lens moved to new position");
            bNewMoveRel = 0;
        }
        
        // Small sleep to prevent busy-waiting
        usleep(1000); // 1ms
    }
}
