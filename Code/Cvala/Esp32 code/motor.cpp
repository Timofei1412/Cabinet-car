#include <define.h>

unsigned long long stuck_protection_timer = 0;
bool stuck_protection_flag = 0;
int signal_to_motor = 0;

bool go_flag=0;

unsigned long long controll_speed_timer = 0;


void encoder(){
    static byte enc;
    enc = enc << 2;
    enc = enc | (digitalRead(enc_a) << 1 | digitalRead(enc_b));
    enc_count += enc_table[enc & 0b1111];
}

void calculate_speed(){
    if(millis() - speed_calculation_timer >= 50){
        current_speed = -(float(enc_count - enc_count_last)/(millis()-speed_calculation_timer));
        current_speed *= 5;
        speed_calculation_timer = millis();
        enc_count_last = enc_count;
        //Serial.println(current_speed);
    }

    if(!stuck_protection_flag && current_speed == 0 && signal_to_motor>50){
        stuck_protection_flag = 1;
        stuck_protection_timer = millis();
    }

    if(stuck_protection_flag && current_speed <= 0.1 && signal_to_motor>50 && millis() - stuck_protection_timer > 500){
        analogWrite(motor_2, 0);
        //analogWrite(motor_2, 0);
        digitalWrite(red_led, HIGH);
        while (true){
            delay(10);
        }
    }
    else if(stuck_protection_flag && millis() - stuck_protection_timer > 500){
        stuck_protection_timer = millis();
        stuck_protection_flag = 0;
    }
}

void start_moving(){
    if(!go_flag){
        move = true;
        //digitalWrite(green_led, 90);
        target_speed = 15;
        //turn_direction = 90;// + 180*(distance_to_point(get_point_by_angle(330))>distance_to_point(get_point_by_angle(30)));
        Serial.println("Go");
        //Serial.print(temporary_distance_to_wall);Serial.print(" ");Serial.println(temporary_distance_to_wall2);
        go_flag=1;
    }
}

float speed_pid(int error){
    //Serial.print(error);
    //Serial.print(" ");
    float up = error * speed_kP;
    speed_i_sum += error;
    float ui = speed_i_sum * speed_kI;
    if (ui < iMin){
        ui = iMin;
    }
    if (ui > iMax){
        ui = iMax;
    }
    float ud = (error - last_error_speed) * speed_kD;
    last_error_speed = error;
    float output = up + ui + ud;
    return output;
}

void controll_speed(){
    if (millis() - controll_speed_timer > 50){
        controll_speed_timer = millis();

        signal_to_motor = int(speed_pid(target_speed-int(current_speed+0.5)) + /*1.8*/2.5*(target_speed) + 0.5);

        if (signal_to_motor > 255){
            signal_to_motor = 255;
        }
        if (signal_to_motor < 0){
            signal_to_motor = 0;
        }
        if (signal_to_motor < min_mot_signal){
            signal_to_motor = 0;
        }

        //Serial.print(target_speed);Serial.print(" ");Serial.print(current_speed);Serial.print(" ");Serial.println(signal_to_motor);
        if(signal_to_motor == 0){
            //digitalWrite(motor_1, LOW);
            analogWrite(motor_2, 0);
        }
        else{
            //digitalWrite(motor_1, HIGH);
            analogWrite(motor_2, signal_to_motor);
        }
    }
}

void set_speed(int target_speed){
    if (target_speed > 255){
        target_speed = 255;
    }
    if (target_speed < min_mot_signal && target_speed > 0){
        target_speed = 0;
    }
    signal_to_motor = target_speed;
    if (target_speed <= 0){
        //digitalWrite(motor_1, LOW);
        analogWrite(motor_2, 0);
    }
    else{
        //digitalWrite(motor_1, HIGH);
        analogWrite(motor_2, target_speed);
    }
    
}

void set_steer_angle(float angle){
    if (angle > angle_constrain){
        angle = angle_constrain;
    }
    if (angle < -angle_constrain){
        angle = -angle_constrain;
    }
    float real_angle = zero_angle + angle;
    steering.write(real_angle);
}