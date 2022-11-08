/* mbed MotCon Library, for the various DC motor control IC's using PWM and one
 * or two direction pins.
 * Copyright (c) 2016, Joseph Bradshaw
 *  20180316 - Added shorthand read and write operator functions.
 *   May need to check implementation of overloaded class write function
 *    using invert
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
 */
#include "mbed.h"

#ifndef MBED_MOTCON_H
#define MBED_MOTCON_H

class MotCon{
public:
    /** Create a motor control port object connected to the specified pins
     *
     * @param pwm_pin PwmOut pin for the enable or speed control
     * @param dir1_pin DigitalOut pin to control the motor direction
     */
    MotCon(PinName pwm_pin, PinName dir1_pin);
    
    /** Create a motor control port object connected to the specified pins.
     *  The two direction pins are typically complementary and provide
     *  the capability of dynamic braking, free-wheeling, forward and reverse.
     *
     * @param pwm_pin PwmOut pin for the enable or speed control
     * @param dir1_pin DigitalOut pin to control the motor direction
     * @param dir2_pin DigitalOut pin to control the motor direction
     */    
    MotCon(PinName pwm_pin, PinName dir1_pin, PinName dir2_pin);
    
    /** This function will set the direction and pwm percent scaled 
     *  from 0.0 - 1.0 and control the direction pin or pins
     *  If two direction pins are used, the user can control the breaking
     *  mode by using the mutator functions for get_mode() and set_mode()
     *  If only one direction pin is used, dynamic breaking and syncronous 
     *  rectification are assumed.  See the specific motor driver part datasheet
     *  for additional details.
     *
     * @param dc is signed float duty cycle (+/-1.0)
     */
    void mot_control(float dc);
    void mot_control(float dc, int invert);  //this function can be used to invert the motor direction
    
    void setMode(int mode);                  //mode: 1 = dynamic braking, 0 = free-wheeling (default)
    int getMode(void);
    float read(void);
        /**  Shorthand for the write and read functions */
    MotCon& operator= (float value);
    MotCon& operator= (MotCon& rhs);
    operator float();
    
    float duty_cycle;    
    
    bool mc_mode;
private:
    bool _dir2;
    
protected:
    PwmOut _pwm_pin;
    DigitalOut _dir1_pin;
    DigitalOut _dir2_pin;
};

#endif
