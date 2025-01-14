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

    // enable the lens, then reset, then enable, then find index, then enable
    axis->sendCommand("ENBL",1);
    axis->sendCommand("RSET",1);
    axis->sendCommand("ENBL",1);
    axis->findIndex();
    axis->sendCommand("ENBL",1);    

    // set to use maximum speed
    axis->setSpeed(400_mm);
    axis->setDPOS(-8_mm);
    currentLensLoc = -8;
    //wait for 0.5s
    usleep(500000);

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
            // Convert mm to Distance object
            Distance newLensLocString(newLensLoc, Distance::MM);
            axis->setDPOS(newLensLocString);
                        
            currentLensLoc = newLensLoc;
            
            //
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
        axis->findIndex();
        axis->setDPOS(-8_mm);
        currentLensLoc = -8;
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
            if(bLensLogFlag) logger->info("[lens::lens_thread] Moving lens to new position");
            mov_rel(mmToMove);
            if(bLensLogFlag) logger->info("[lens::lens_thread] Lens moved to new position");
            bNewMoveRel = 0;
        }
        
        // Small sleep to prevent busy-waiting
        usleep(1000); // 1ms
    }
}
