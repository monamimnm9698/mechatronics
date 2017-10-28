#include "mbed.h"

DigitalOut myled1(D3);//(LED1)

int main() {
    while(1) {
        myled1 = 1; // LED is ON
        wait(0.2); // 200 ms
        myled1 = 0; // LED is OFF
        wait(2); // 2 sec
    }
}

//HelloWorld!
