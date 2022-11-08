// J. Bradshaw - Motor Control Libary for support of most H-Bridge drivers with
//  a single PWM (enable active high) pin and one or two direction pins
//  to support forward/reverse (complimentary)
#include "mbed.h"
#include "MotCon.h"

//Constructor
MotCon::MotCon(PinName pwm_pin, PinName dir1_pin) : _pwm_pin(pwm_pin), _dir1_pin(dir1_pin), _dir2_pin(NC) {
    _dir2 = false;
    _pwm_pin.period_us(50);
    _pwm_pin = 0.0;
    _dir1_pin = 0;
}
MotCon::MotCon(PinName pwm_pin, PinName dir1_pin, PinName dir2_pin) : _pwm_pin(pwm_pin), _dir1_pin(dir1_pin), _dir2_pin(dir2_pin) {
    _dir2 = true;
    _pwm_pin.period_us(50);
    _pwm_pin = 0.0;
    _dir1_pin = 0;
    _dir2_pin = 0;
    
    mc_mode = 0;    //mode pin determines braking (1 = dynamic braking, 0 = free-wheeling)    
}
// dc is signed duty cycle (+/-1.0)
void MotCon::mot_control(float dc){    
    if(dc>1.0)
        dc=1.0;
    if(dc<-1.0)
        dc=-1.0;
            
    if(_dir2){            
        if(dc > 0.001){
            _dir1_pin = 0;
            _dir2_pin = 1;
            _pwm_pin = dc;
        }
        else if(dc < -0.001){
            _dir2_pin = 0;
            _dir1_pin = 1;        
            _pwm_pin = abs(dc);
        }
        else{
            if(mc_mode){
                _dir1_pin = 0;
                _dir2_pin = 0;
                _pwm_pin = 1.0;                
            }
            else{
                _dir1_pin = 0;
                _dir2_pin = 0;
                _pwm_pin = 0.0;
            }
        }         
    }
    else{            
        if(dc > 0.001){
            _dir1_pin = 0;
            _pwm_pin = dc;
        }
        else if(dc < -0.001){
            _dir1_pin = 1;
            _pwm_pin = abs(dc);
        }
        else{
            _dir1_pin = 0;
            _pwm_pin = 0.0;
        }             
    }
}

// dc is signed duty cycle (+/-1.0)
void MotCon::mot_control(float dc, int invert){
    if(dc>1.0)
        dc=1.0;
    if(dc<-1.0)
        dc=-1.0;
                
    if(_dir2){
        if(invert==0){
            if(dc > 0.001){
                _dir1_pin = 0;
                _dir2_pin = 1;
                _pwm_pin = dc;
            }
            else if(dc < -0.001){
                _dir2_pin = 0;
                _dir1_pin = 1;
                _pwm_pin = abs(dc);
            }
            else{
                if(mc_mode){
                    _dir1_pin = 0;
                    _dir2_pin = 0;
                    _pwm_pin = 1.0;                
                }
                else{
                    _dir1_pin = 0;
                    _dir2_pin = 0;
                    _pwm_pin = 0.0;
                }
            }
        }
        else{
            if(dc > 0.001){
                _dir2_pin = 0;
                _dir1_pin = 1;
                _pwm_pin = dc;
            }
            else if(dc < -0.001){
                _dir1_pin = 0;
                _dir2_pin = 1;
                _pwm_pin = abs(dc);
            }
            else{
                if(mc_mode){
                    _dir1_pin = 0;
                    _dir2_pin = 0;
                    _pwm_pin = 1.0;                
                }
                else{
                    _dir1_pin = 0;
                    _dir2_pin = 0;
                    _pwm_pin = 0.0;
                }
            }
        }
    }
    else{        
        if(invert==0){
            if(dc > 0.001){
                _dir1_pin = 0;
                _pwm_pin = dc;
            }
            else if(dc < -0.001){
                _dir1_pin = 1;
                _pwm_pin = abs(dc);
            }
            else{
                _dir1_pin = 0;
                _pwm_pin = 0.0;
            }
        }
        else{
            if(dc > 0.001){
                _dir1_pin = 1;
                _pwm_pin = dc;
            }
            else if(dc < -0.001){
                _dir1_pin = 0;
                _pwm_pin = abs(dc);
            }
            else{
                _dir1_pin = 0;
                _pwm_pin = 0.0;
            }
        }    
    }    
}

void MotCon::setMode(int mode){
    mc_mode = mode;
}

int MotCon::getMode(void){
    return mc_mode;
}

float MotCon::read(void){
    float ret = this->duty_cycle;

    return ret;
}

MotCon& MotCon::operator= (float value){
    mot_control(value);
    
    return *this;
}

MotCon& MotCon::operator= (MotCon& rhs) {
  // Underlying call is thread safe
  mot_control(rhs.read());
  
  return *this;
}
  
MotCon::operator float(){
// Underlying call is thread safe
   return this->read(); 
}