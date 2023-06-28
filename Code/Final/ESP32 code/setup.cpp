#include <define.h>

void check_battery(){
    //Функция проверки заряда батареи

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

    //Генерация изначального пути, без кубиков
    robot_path.points[0] = {2500, 1000};
    robot_path.points[1] = {2500, 2000};
    robot_path.points[2] = {2000, 2500};
    robot_path.points[3] = {1000, 2500};
    robot_path.points[4] = {500, 2000};
    robot_path.points[5] = {500, 1000};
    robot_path.points[6] = {1000, 500};
    robot_path.points[7] = {2000, 500};
    robot_path.points_count=8;
    robot_path.current_point=1;
    robot_path.sectors_start[0] = 0;
    robot_path.sectors_start[1] = 2;
    robot_path.sectors_start[2] = 4;
    robot_path.sectors_start[3] = 6;
}

