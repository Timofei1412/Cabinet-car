#include <define.h>

void debug_parse(){
    String data;
    bool recieve_flag = 0;
    if (debug.available()){
        data = debug.readStringUntil(';');
        recieve_flag = 1;
    }
    else if(Serial.available()){
        data = Serial.readStringUntil(';');
        recieve_flag = 1;
    }
    if(recieve_flag == 1){
        if (data.substring(0, 9) == "Set angle"){
            set_steer_angle(data.substring(9).toFloat());
        }
        else if (data.substring(0, 4) == "SSkP"){
            speed_kP = data.substring(4).toFloat();
        }
        else if (data.substring(0, 4) == "SSkI"){
            speed_kI = data.substring(4).toFloat();
        }
        else if (data.substring(0, 4) == "SSkD"){
            speed_kD = data.substring(4).toFloat();
        }
        else if (data.substring(0, 4) == "SAkP"){
            steering_kP = data.substring(4).toFloat();
        }
        else if (data.substring(0, 4) == "SAkI"){
            steering_kI = data.substring(4).toFloat();
        }
        else if (data.substring(0, 4) == "SAkD"){
            steering_kD = data.substring(4).toFloat();
        }
        else if (data.substring(0, 2) == "SS"){
            //set_speed(data.substring(2).toInt());
            target_speed = data.substring(2).toInt();
        }
        else if (data.substring(0, 15) == "Set lidar speed"){
            analogWrite(lidar_speed, data.substring(15).toInt());
        }
    }
}