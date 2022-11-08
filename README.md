# mbed_ES410_PID_getKeKeold
Allows user to enter ke and keold variables through the serial interface (terminal program)

```C++
#include "mbed.h"
#include "MotCon.h"
#include "QEI.h"
 
#define TSAMPLE     0.00885
#define ENC_CPR     800.0
#define PI          3.141592653589793
#define meascount 100      //******YOU CHANGE how many measurements are made 
 
Serial pc(USBTX, USBRX);
DigitalOut led1(LED1);
QEI enc1(p24,p23,NC,800, QEI::X4_ENCODING); //Global Motor 800 cpr w x4 encoding
MotCon motor(p26, p28);       //pwm output, direction
Ticker cont_update;


float enc;
volatile int end_flag = 0;  //must make volatile to share with ticker and while loop
int tcount;
float mot;
//****** following variables are subject to change
float Kspeed;
float sp[meascount];
float motk[meascount];
float tauspec;
float taumotor;
float Dcgain;
float Desspeed;
float e;
float keold;
float ke;
float eold;
float mvold;
float kprop;

void update_controller(void){    
    enc = (float)enc1.getPulses();
    enc1.reset();
    
    sp[tcount]=enc*Kspeed; //calculate the encoder speed
    //Kspeed is conversion of encoder reading to motor speed
    // mot = kprop*(Desspeed-sp[tcount]);   // this is proportional control
    e=Desspeed-sp[tcount];
    mot=mvold+ke*e-keold*eold;
    if(mot > 12.0)      //saturate mot output
        mot = 12.0;    
    motk[tcount]=mot; 
    
        
    motor.mot_control(mot/12.0);    //12 volts
    
    mvold=mot;
    eold=Desspeed-sp[tcount];    
    
    tcount++;
    if(tcount == meascount){
        cont_update.detach();   //terminate the Ticker (interrupt routine)
        end_flag = 1;           //indicate that the sampling is over    
    }
    led1 = !led1;
}

int main(){
    pc.printf("Begin ES410 PID program\r\n\r\n");
    
    //Initialize variables
    Kspeed=(2.0*PI/ENC_CPR)/(TSAMPLE);
    Desspeed=200.0;
    eold=0.0;
    mvold=0.0;
    ke=.04444;
    keold=.04076;   //*****YOU change
        
    int done=0;
    while(!done){
        char c;
        pc.printf("\r\nEnter float value for ke:");
        pc.scanf("%f", &ke);
        
        pc.printf("%.5f was entered for ke, is this correct?(y/n)", ke);
        
        c = pc.getc();
        if(c == 'y' || c == 'Y')
            done = 1;
    }
    
    done=0;
    while(!done){
        char c;
        pc.printf("\r\nEnter float value for keold:");
        pc.scanf("%f", &keold);
        
        pc.printf("%.5f was entered for keold, is this correct?(y/n)", keold);
        
        c = pc.getc();
        if(c == 'y' || c == 'Y')            
            done = 1;
    }            
    pc.printf("\r\n");
                     
    cont_update.attach(&update_controller, TSAMPLE); //start the ticker and controller
    
    while(end_flag == 0); //wait for sample collection to stop
    motor.mot_control(0.0);         //turn off the motor when all of the data is collected    
    
    //serial output
    for(int k=0;k<meascount;k++){ // print the data on the screen so you can download it for processing
        pc.printf("%4d %.2f %.4f \n\r",k, sp[k],motk[k]);        
    }  
}
```
