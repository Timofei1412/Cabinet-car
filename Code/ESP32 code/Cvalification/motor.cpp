#include <define.h>

unsigned long long stuck_protection_timer = 0;
bool stuck_protection_flag = 0;
int signal_to_motor = 0;

bool go_flag=0;

unsigned long long controll_speed_timer = 0;

unsigned long long distance_traveled = 0;

float odometry_x = 0;
float odometry_y = 0;
float odomety_angle = 0;

unsigned long long odometry_calculation_timer = 0;

bool print_flag = 0;

unsigned long long enc_l_timer=0;
unsigned long long enc_l_last_time=0;

long long enc_count_global = 0;

void encoder(){
    static byte enc;
    enc = enc << 2;
    enc = enc | (digitalRead(enc_a) << 1 | digitalRead(enc_b));
    enc_count += enc_table[enc & 0b1111];
    enc_count_global += enc_table[enc & 0b1111];
}

void encoder_l(){
    static byte enc_l;
    enc_l = enc_l << 2;
    enc_l = enc_l | (digitalRead(enc_l_a) << 1 | digitalRead(enc_l_b));
    enc_count_l += enc_table[enc_l & 0b1111];
    enc_l_last_time = millis()-enc_l_timer;
    enc_l_timer = millis();
}

void encoder_r(){
    static byte enc_r;
    enc_r = enc_r << 2;
    enc_r = enc_r | (digitalRead(enc_r_a) << 1 | digitalRead(enc_r_b));
    enc_count_r += enc_table[enc_r & 0b1111];
    print_flag = 1;
}

void odometry(){
    if(millis() - odometry_calculation_timer > 100){
        odometry_calculation_timer = millis();

        Serial.print(enc_count_r);
        Serial.print(" ");
        Serial.print(enc_count_l);
        Serial.print(" : ");

        float enc_count_l_tmp = enc_count_l *= 1.93;
        //enc_count_l += 1.93*((millis()-enc_l_timer)/enc_l_last_time);
        float enc_count_r_tmp = enc_count_r *= 1.93;

        if (enc_count_l_tmp == enc_count_r_tmp) {
            odometry_y += enc_count_r_tmp;
        } 
        else {

            /*float angle_of_turn = (enc_count_r - enc_count_l) / wheeel_base_width;
            float distance = (enc_count_r + enc_count_l) / 2;
            odomety_angle += angle_of_turn;
            odometry_x += distance * cos(odomety_angle);
            odometry_y += distance * sin(odomety_angle);*/


            float angle_of_turn = (enc_count_r_tmp - enc_count_l_tmp) / wheeel_base_width;
            float distance = (enc_count_r_tmp + enc_count_l_tmp) / 2;
            float radius_of_turn = distance/angle_of_turn;
            float hyp = sqrt(2*pow(radius_of_turn, 2)*(1-cos(angle_of_turn)));
            odometry_x += hyp * cos(angle_of_turn/2);
            odometry_y += hyp * sin(angle_of_turn/2);
            odomety_angle += angle_of_turn;

            Serial.print(angle_of_turn);
            Serial.print(" ");
            Serial.print(distance);
            Serial.print(" ");
            Serial.print(radius_of_turn);
            Serial.print(" ");
            Serial.print(hyp);
            Serial.print(" : ");

        }

        
        Serial.print(odometry_x);
        Serial.print(" ");
        Serial.print(odometry_y);
        Serial.print(" ");
        Serial.println(((odomety_angle)/(2*PI))*360);

        enc_count_l = 0;
        enc_count_r = 0;
    }
}

void calculate_speed(){
    if(millis() - speed_calculation_timer >= 50){
        current_speed = -(float(enc_count)/(millis()-speed_calculation_timer));
        current_speed *= 5;
        speed_calculation_timer = millis();
        //odometry_2();
        enc_count = 0;
        //Serial.println(current_speed);
    }
    //stuck_protection();
}

void stuck_protection(){
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

void odometry_2(){
    float real_servo_angle = (analogRead(servo_angle_pin)-pot_value)/values_per_angle;
    Serial.print(real_servo_angle);
    Serial.print(" ");
    Serial.println(enc_count);
}

void start_moving(){
    if(!go_flag){
        move = true;
        //digitalWrite(green_led, 90);
        target_speed = 20;
        steering_kP = 1;
        Serial.println("Go");
        //Serial.print(temporary_distance_to_wall);Serial.print(" ");Serial.println(temporary_distance_to_wall2);
        go_flag=1;
    }
}

float speed_pid(int error){
    //ПИД для контроля скорости мотора

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
    //Функция 

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