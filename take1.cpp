#include "mbed.h"

//LEDgreen(Blinker Right)
DigitalOut ledR(D13);
//LEDgreen(Blinker Left)
//DigitalOut ledL(D12);

//motor
PwmOut pwm1(D5);
DigitalOut digital1 (D6);
PwmOut pwm2(D9);
DigitalOut digital2 (D10);

//pushswitch
DigitalIn pushswitch(D0);
const float vcc = 3.3;

//refrectsenser
AnalogIn analog0(A0);
AnalogIn analog1(A1);
AnalogIn analog2(A2);

void advance(float duty){
    digital1 = 1;
    pwm1 = duty;
    digital2 = 1;
    pwm2 = duty;
}
void stop(void){
    digital1 = 0;
    pwm1 = 0;
    digital2 = 0;
    pwm2 = 0;
}
void goback(float duty){
    digital1 = 0;
    pwm1 = duty;
    digital2 = 0;
    pwm2 = duty;
}
void tenmetsu(DigitalOut led){
    led = 1; //LED is ON
    wait(0.2); //200msec
    led = 0; //LED is OFF
    wait(0.5); //500msec
}

float printfvout(DigitalIn vout){
    float vo = vout.read()*vcc; //vout.pushswitch
    printf("%f\n",vo);

    return vo;
}

void printfaout(AnalogIn aout){
    float ao = aout.read()*vcc;
    printf("%s %7.1f [V]\n",aout,ao);
}
void shototsu(float vo){
    if( vo>3.0 ){
        ledR = 1;
        //ledL = 1;
        printfvout(pushswitch);
        stop();
        wait(5);
        goback(0.5);
    }
}

int main(){

    int i;

    while(1){
        //LEDgreen
        tenmetsu(ledR);
        //moter(duty=0.5)
        advance(0.5);
        //pushswitch
        float vout = printfvout(pushswitch);
        //shototsu
        shototsu(vout);
        //linetrace
        printfaout(analog0);
        printfaout(analog1);
        printfaout(analog2);
        
        i++;
    }
}
