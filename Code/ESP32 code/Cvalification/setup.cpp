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
    robot_path.points[0] = {2700, 1000};
    robot_path.points[1] = {2700, 2000};
    robot_path.points[2] = {2610, 2610};
    robot_path.points[3] = {2000, 2700};
    robot_path.points[4] = {1000, 2700};
    robot_path.points[5] = {390, 2610};
    robot_path.points[6] = {300, 2000};
    robot_path.points[7] = {300, 1000};
    robot_path.points[8] = {390, 390};
    robot_path.points[9] = {1000, 300};
    robot_path.points[10] = {2000, 300};
    robot_path.points[11] = {2610, 390};
    robot_path.points_count=12;
    robot_path.current_point=1;
    robot_path.sectors_start[0] = 0;
    robot_path.sectors_start[1] = 2;
    robot_path.sectors_start[2] = 4;
    robot_path.sectors_start[3] = 6;
}

