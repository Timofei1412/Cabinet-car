#include <define.h>

void check_battery(){
    float voltage = 0;
    for(int i=0; i<20; i++){
        voltage += analogRead(battery_voltage_pin)/20.0;
    }
    if (voltage/devision_coefficient < critical_voltage){
        digitalWrite(red_led, true);
        delay(1000000);
    }
    else{
        for (int i=0; i<3; i++){
        digitalWrite(green_led, true);
        delay(500);
        digitalWrite(green_led, false);
        delay(500);
        }
    }
}

