#ifndef LENS_H
#define LENS_H
#include <atomic>
#include <thread>
#include "Xeryon.h"
#include "Distance.h"

class lens {
public:
    lens();
    ~lens();
    bool initialize();
    void return_to_start();
    void set_return_position(double position);

private:
    void lens_thread();
    void mov_rel(double mmToMove);
    
    // Xeryon controller objects
    Xeryon* controller;
    Axis* axis;
    
    // Current position in mm
    //double currentLensLoc = -3.0;
    double currentLensLoc;

    // Return position in mm (new variable)
    double returnPosition = -9.1;

    // this thread gets instructions from the GUI
    std::thread tLens;
    std::atomic<bool> stop_thread;
    
    // For out of bounds detection
    int outOfBoundsOnceOnly = 0;
    // const double MIN_POSITION = -18.0;  // mm, to the right from operator perspective
    // const double MAX_POSITION = 8.5; // mm, to the left from operator perspective
    const double MIN_POSITION = -14.9;  // mm, to the right from operator perspective
    const double MAX_POSITION = 0.0; // mm, to the left from operator perspective

};

#endif // LENS_H