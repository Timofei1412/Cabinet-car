#include <define.h>

unsigned int steer_timer=0;
bool steer_flag=0;
bool steer_flag2=0;

unsigned int turn_forbidden_timer=0;

int turns_count = 0;

int turn_correction_distance = 0;
bool turn_correction_flag=0;

float steering_pid(int error){
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
    if (output > angle_constrain){
        output = angle_constrain;
    }
    else if (output < -angle_constrain){
        output = -angle_constrain;
    }
    return -output;
}

void steer(){
    digitalWrite(green_led, LOW);
    float angle = 0;
    if(turns_count == 0){
        angle = steering_pid(temporary_distance_to_wall - distance_to_side_line);
    }
    else{
        angle = steering_pid(distance_to_wall - distance_to_side_line);
    }

    //Serial.print(distance_to_right_line);Serial.print(" ");Serial.println(angle);

    /*if(turns_count == 0){
        float measuring_angle = 60 -angle_to_side_line;
        if (distance_between_points(get_point_by_angle(measuring_angle), {0, 0}) > 800 && turn_direction == 0){
            turn_direction = 1;
            temporary_distance_to_wall = temporary_distance_to_wall2;
        }
    }*/
    //debug.print(distance_to_forward_line);debug.print(" ");debug.print(angle_to_forward_line);debug.print(" ");debug.println(turn_direction);
    /*if(!turn_correction_flag){
        for(int i=0; i<lidar.poits_amount; i++){
            if(chech_point_in_square(points_xy[i], blobs_pot_coords[6][0], blobs_pot_coords[6][0], 10) || chech_point_in_square(points_xy[i], blobs_pot_coords[8][0], blobs_pot_coords[6][0], 10)){
                turn_correction_distance = -100;
                turn_correction_flag = 1;
            }
            if(chech_point_in_square(points_xy[i], blobs_pot_coords[7][0], blobs_pot_coords[6][0], 10) || chech_point_in_square(points_xy[i], blobs_pot_coords[9][0], blobs_pot_coords[6][0], 10)){
                turn_correction_distance = 100;
                turn_correction_flag = 1;
            }
        }
    }*/

    if (distance_to_forward_line < distance_to_wall+turn_radius && millis()-turn_forbidden_timer > 2000){
        turning = 1;
    }

    angle *= (1-turn_direction*2);
    set_steer_angle(angle);
}

void turn(){
    //target_speed = 0;
    /*if (!steer_flag){
        steer_timer = millis();
        steer_flag = 1;
    }
    set_steer_angle(-50);
    float angle = abs(degrees(atan(side_line.slope)));
    //Serial.println(angle);
    if (millis() - steer_timer >= turn_time && steer_flag && angle >= 80){
        set_steer_angle(0);
        steer_flag = 0;
        turning = 0;
        turns_count += 1;
        steering_i_sum = 0;
        turn_forbidden_timer = millis();
    }*/
    digitalWrite(green_led, HIGH);
    set_steer_angle(-50 + turn_direction*100);
    if (!steer_flag){
        steer_timer = millis();
        steer_flag = 1;
    }
    
    debug.print(distance_to_back_line);debug.print(" ");debug.print(angle_to_side_line);debug.print(" ");debug.print(millis() - steer_timer);debug.print(" ");debug.println(steer_flag2);

    if (distance_to_back_line < 1300 && angle_to_side_line * (1 - turn_direction*2) >= -30 && angle_to_side_line * (1 - turn_direction*2) <= 10 && millis() - steer_timer >= 600 && steer_flag){
            set_steer_angle(0);
            turning = 0;
            turns_count += 1;
            steering_i_sum = 0;
            steer_flag = 0;
            steer_flag2 = 0;
            turn_forbidden_timer = millis();
            //turn_correction_flag = 0;
    }
    else{
        //steer_flag2 = 0;
    }
}
