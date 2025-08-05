/**
 * Copyright 2019 Bradley J. Snyder <snyder.bradleyj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * 
 * https://gist.github.com/bradley219/5373998
 */



/**
 * @file pid.hpp
 * @brief Defines the `PID` class for implementing a PID controller.
 * 
 * This file provides the declaration of the `PID` class, which calculates the manipulated variable
 * based on proportional, integral, and derivative gains. It is designed to control systems with
 * feedback loops.
 */


#pragma once
#ifndef _PID_H_
#define _PID_H_

/**
 * @class PID
 * @brief A class for implementing a PID controller.
 *
 * This class provides methods to calculate the manipulated variable based on
 * proportional, integral, and derivative gains.
 */
class PIDImpl;
class PID
{
public:
    /**
     * @brief Constructor for the PID class.
     *
     * @param dt Loop interval time.
     * @param max Maximum value of the manipulated variable.
     * @param min Minimum value of the manipulated variable.
     * @param Kp Proportional gain.
     * @param Kd Derivative gain.
     * @param Ki Integral gain.
     */
    PID(double dt, double max, double min, double Kp, double Kd, double Ki);

    /**
     * @brief Calculates the manipulated variable given a setpoint and current process value.
     *
     * @param setpoint The desired setpoint.
     * @param pv The current process value.
     * @return The manipulated variable.
     */
    double calculate(double setpoint, double pv);

    /**
     * @brief Destructor for the PID class.
     */
    ~PID();

private:
    PIDImpl* pimpl;
};

#endif