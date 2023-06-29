#include <define.h>

int turns_count = 0;

float steering_pid(int error){
    //Функция пида для контроля угла сервы

    float up = error * steering_kP;
    steering_i_sum += error;
    float ui = steering_i_sum * steering_kI;
    if (ui < iMin){
        ui = iMin;
    }
    if (ui > iMax){
        ui = iMax;
    }
    float ud = (error - last_error_steering) * steering_kD;
    last_error_steering = error;
    float output = float(up + ui + ud);
    //Serial.print(error);Serial.print(" ");Serial.println(output);
    return -output;
}

void steer(){
    //Функция расчёта ошибки между углом робота и углом линии до следующего кубика и поворота сервы

    float angle = 0;
    int absolute_angle = 0;
    
    absolute_angle = int((robot_angle_local-(robot_sector%4)*90)+0.5);
    int target_angle = mod((360 - angle_of_line(line_by_two_points({robot_x_global, robot_y_global}, robot_path.get_next_point())) + 180)+0.5, 360);
    int error_angle = (absolute_angle - target_angle);
    if(error_angle>180){
        error_angle = 180 - error_angle;
    }
    if(error_angle<-180){
        error_angle = 360 + error_angle;
    }
    angle = steering_pid(error_angle);
    
    debug.print(absolute_angle);debug.print(" ");debug.print(target_angle);debug.print(" ");debug.println(error_angle);
    debug.println();

    angle *= (1-move_direction*2);
    set_steer_angle(angle);
}

