#include <Arduino.h>
#include <Servo.h>
#include "BluetoothSerial.h"
#include <math.h>
#include <Wire.h>
#include "TrackingCamI2C.h"

#define enc_a 32
#define enc_b 33
#define motor_1 23
#define motor_2 19

#define enc_l_a 36
#define enc_l_b 15
#define enc_r_a 18
#define enc_r_b 39

#define servo_pin 14
#define servo_angle_pin 34

#define lidar_speed 13

#define green_led 2
#define red_led 12
#define start_btn 25
#define battery_voltage_pin 35


#define critical_voltage 7.5
#define devision_coefficient 416

#define zero_angle 87
#define angle_constrain 48
#define ideal_angle 41
#define iMin -50
#define iMax 50

#define min_mot_signal 30

#define line_calculation_angle 40

#define distance_to_wall 500
#define turn_radius 200//400//600
#define corner_entrance_distance 100//200
#define turn_time 450//550

#define is_corner_in_segment_value 500
#define blob_search_area 75
#define points_in_blob_area 2
#define distance_shift 20
#define robot_reached_point_distance 100

#define wheeel_base_width 88.5

#define pot_value 1650
#define values_per_angle 15.75



#ifndef DEFINE_H
#define DEFINE_H

#define SerialLidar Serial2
#define LIDAR_HEADER 0x54
#define LIDAR_LENGTH 0x2C
#define POINT_PER_PACK 12


struct point{
    int x;
    int y;

    point(int X=0, int Y=0)
    {
        x = X;
        y = Y;
    }

    void print(){
        Serial.print(this->x);
        Serial.print(':');
        Serial.print(this->y);
        Serial.print(' ');
    }

    void print_byte(){
        Serial.write(this->x);
        Serial.write(':');
        Serial.write(this->y);
        Serial.write(' ');
    }
};

struct point_d{
    double x;
    double y;
};

struct point_l{
    long long x;
    long long y;
};

struct point_raw{
    volatile unsigned int dist;
    volatile float angle;
    void print(){
        Serial.print(this->angle);
        Serial.print(':');
        Serial.print(this->dist);
        Serial.print(' ');
    }
};

struct lidar_data{
    point_raw points[400];
    volatile bool new_revolution = false;
    volatile float current_angle = 0;
    volatile int current_point = 0;
    volatile float last_angle = 0;
    volatile int points_amount = 0;
    volatile int pakets = 0;
};

extern Servo steering;
extern BluetoothSerial debug;
extern const int enc_table[16];
extern volatile long long int enc_count;
extern volatile long long int enc_count_last;
extern volatile long long int enc_count_l;
extern volatile long long int enc_count_r;
extern float speed_kP;
extern float speed_kI;
extern float speed_kD;
extern float speed_i_sum;
extern float last_error_speed;
extern float current_speed;
extern unsigned long long time_passed;
extern unsigned long long speed_calculation_timer;
extern int target_speed;
extern int distances[3];
extern int deviation_from_center;
extern float steering_kP;
extern float steering_kI;
extern float steering_kD;
extern float steering_i_sum;
extern float last_error_steering;
extern lidar_data lidar;
extern point points_xy[400];
extern int distance_to_forward_line;
extern int distance_to_back_line;
extern bool turning;
extern volatile bool reading_flag;
extern long bad_package_count;
extern long good_package_count;
extern bool move;
extern int turns_count;
extern int move_direction;
extern int16_t circle_buff[1500];
extern int circle_buff_pointer;
extern float angle_to_side_line;
extern int temporary_distance_to_wall;
extern int temporary_distance_to_wall2;
extern int distance_to_side_line;
extern point blobs_pot_coords[12];
extern float angle_to_forward_line;
extern point unfiltered_points[400];
extern bool print_flag;
extern unsigned short blobs_indexes[10];
extern unsigned short blobs_indexes_count;
extern bool orient_by;
extern int distance_to_side_line_for_PID;
extern int target_distance_to_wall;
extern unsigned short blob_detection_count[24];
extern int robot_x_local;
extern int robot_y_local;
extern float robot_angle_local;
extern int robot_x_global;
extern int robot_y_global;
extern int robot_sector;
extern short camera_blob_side;
extern short camera_blob_distance;
extern short camera_blob_color;
extern TrackingCamI2C trackingCam;

struct line{
    double a;
    double b;
    double c;

    void print(){
        Serial.print('{');
        Serial.print(this->a);
        Serial.print(':');
        Serial.print(this->b);
        Serial.print(':');
        Serial.print(this->c);
        Serial.print('}');
    }
};

struct dynamic_segments{
    //Структура динамического массива сегментов

    unsigned short segments[15][2]; //Массив сегментов
    line segments_apr[15]; // Апроксимация сегментов до линий
    unsigned short segments_count=0; //Количество сегментов
    
    void incert(int index, unsigned short segment[2], line line){
        // Функция вставки сегмента в массив

        for(int i=segments_count-1; i>=index; i--){
            this->segments[i+1][0] = this->segments[i][0];
            this->segments[i+1][1] = this->segments[i][1];
            this->segments_apr[i+1] = this->segments_apr[i];
        }
        segments[index][0] = segment[0];
        segments[index][1] = segment[1];
        segments_apr[index] = line;
        segments_count++;
    }

    void erase(int index){
        // Функция удаления сегмента из массива

        for(int i=index; i<this->segments_count-1; i++){
            this->segments[i][0] = this->segments[i+1][0];
            this->segments[i][1] = this->segments[i+1][1];
            this->segments_apr[i] = this->segments_apr[i+1];
        }
        segments_count--;
    }
};

extern line side_line;
extern line forward_line;
extern line back_line;
extern line side_line2;

void check_battery();
void encoder();
void encoder_l();
void encoder_r();
void stuck_protection();
void calculate_speed();
void odometry();
void odometry_2();
void start_moving();
float speed_pid(int error);
void controll_speed();
void set_speed(int target_speed);
void set_steer_angle(float angle);
void debug_parse();
void LidarRead();
void ProcessingPackage();
void processingIntensityPoint(uint16_t distance, uint8_t intensity, double angle);
uint8_t CalCRC(uint8_t *p, uint8_t length);
float steering_pid(float error);
void steer();
void get_data_from_camera();
void refresh_points();
void find_blobs();
void colorize_blobs();
line line_by_two_points(point_d p1, point_d p2);
line line_by_two_points(point p1, point p2);
bool chech_point_in_square(unsigned short point_index, point center, int half_side);
double angle_of_line(line line);
int distance_to_line(line line, point point);
int distance_between_points(point point1, point point2);
point intersection(line line1, line line2);
point_l intersection_l(line line1, line line2);
line line_approximation(int num_points);
void separate_lines();
int mod(int n, int M);
double mod_2(double a, double b);
short sign(int a);
point local_to_global(point local, int sector);
void calculate_move_direction();

struct path{
    //Структура пути робота

    point points[20];
    point optimised_path[100];
    point additional_points[20];
    short distance_between_optimised_points[20];
    short connected_blobs[20]={-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
    short blobs_colors[20];
    short blobs_in_sectors_count[4] = {0, 0, 0, 0};
    unsigned short points_count = 0;
    unsigned short optimised_points_count=0;
    unsigned short additional_points_count=2;
    unsigned short current_point = 0;
    unsigned short sectors_start[4];
    unsigned short end_point = 1;
    point point_for_bese[4];
    short last_blob_color;
    short last_blob_ind;
    bool optimise_flag = 0;
    
    void incert(point point, unsigned short index, short connected_blob=-1, short color = -1){
        for(int i=this->points_count-1; i>=index; i--){
            this->points[i+1] = this->points[i];
            this->connected_blobs[i+1] = this->connected_blobs[i];
            this->blobs_colors[i+1] = this->blobs_colors[i];
        }
        this->points[index] = point;
        this->connected_blobs[index] = connected_blob;
        this->blobs_colors[index] = color;
        for(int i=0; i<4; i++){
            if(this->sectors_start[i]>index){
                this->sectors_start[i]++;
            }
        }
        if(this->end_point>index){
            this->end_point++;
        }
        this->points_count++;
    }
    
    void incert_in_optimise_path(point point, unsigned short index){
        for(int i=this->optimised_points_count-1; i>=index; i--){
            this->optimised_path[i+1] = this->optimised_path[i];
        }
        this->optimised_path[index] = point;
        this->optimised_points_count++;
    }
    
    point get_next_point(){
        if(!this->optimise_flag){
            return points[current_point];
        }
        return optimised_path[current_point];
    }
    
    void check_position(){
        if(!this->optimise_flag){
            if(distance_between_points(this->points[this->current_point], {robot_x_global, robot_y_global}) < robot_reached_point_distance){
                current_point ++;
                current_point = current_point%points_count;
            }
        }
        else{
            if(distance_between_points(this->optimised_path[this->current_point], {robot_x_global, robot_y_global}) < robot_reached_point_distance){
                current_point ++;
                current_point = current_point%optimised_points_count;
            }
        }
    }
    
    short get_blob_index(short index){
        for(int i=0; i<this->points_count; i++){
            if(this->connected_blobs[i] == index){
                return i;
            }
        }
    }
    
    void reverce(){
        if(this->last_blob_ind == 4 || this->last_blob_ind == 5){
            this->optimised_path[this->optimised_points_count] = {2800, 1000};
            this->optimised_path[this->optimised_points_count] = {2500, 1600};
            this->optimised_path[this->optimised_points_count] = {2200, 1000};
        }
        if(this->last_blob_ind == 0){
            
        }
        if(this->last_blob_ind == 1){
            
        }
        if(this->last_blob_ind == 2){
            
        }
        if(this->last_blob_ind == 3){
            
        }
        for(int i=0; i < this->points_count/2; i++){
            this->points[i] = this->points[this->points_count - i - 1];
            this->connected_blobs[i] = this->connected_blobs[this->points_count - i - 1];
            this->blobs_colors[i] = this->blobs_colors[this->points_count - i - 1];
        }
        //this->current_point = 0;
    }
    
    void generate_bese_line(float amount, short ind){
        for(int i=0; i<amount; i++){
            point_d point1[3], point2[2], final_point;
            for(int j=0; j<3; j++){
                point1[j] = {this->point_for_bese[j].x + ((this->point_for_bese[j+1].x - this->point_for_bese[j].x)/(amount+1))*(i+1), this->point_for_bese[j].y + ((this->point_for_bese[j+1].y - this->point_for_bese[j].y)/(amount+1))*(i+1)};
            }
            for(int j=0; j<2; j++){
                point2[j] = {point1[j].x + ((point1[j+1].x - point1[j].x)/(amount+1))*(i+1), point1[j].y + ((point1[j+1].y - point1[j].y)/(amount+1))*(i+1)};
            }
            final_point = {point2[0].x + ((point2[1].x - point2[0].x)/(amount+1))*(i+1), point2[0].y + ((point2[1].y - point2[0].y)/(amount+1))*(i+1)};
            this->incert_in_optimise_path({int(final_point.x+0.5), int(final_point.y+0.5)}, ind+i);
        }
    }
    
    void optimise_path(){
        debug.println();
        for(int i=0; i<this->points_count; i++){
            debug.print(this->points[i].x);
            debug.print(" ");
            debug.print(this->points[i].y);
            debug.print(" ");
            debug.print(this->connected_blobs[i]);
            debug.print(" ");
            debug.print(this->blobs_colors[i]);
            debug.print(":");
        }
        debug.println();
        for(int i=0; i<this->points_count; i++){
            if(this->connected_blobs[i] != -1){
                short blob_index = this->connected_blobs[i];
                if (this->blobs_colors[i] == -1){
                    this->optimised_path[this->optimised_points_count++] = local_to_global({200, blobs_pot_coords[blob_index%6].y}, blob_index/6);
                }
                else{
                    this->optimised_path[this->optimised_points_count++] = local_to_global({blobs_pot_coords[blob_index%6].x + 170, blobs_pot_coords[blob_index%6].y}, blob_index/6);
                }
                if(this->optimised_points_count-1 > 0){
                    distance_between_optimised_points[this->optimised_points_count-1]=distance_between_points(this->optimised_path[this->optimised_points_count-2], this->optimised_path[this->optimised_points_count-1]);
                }
                i++;
            }
        }
        distance_between_optimised_points[0]=distance_between_points(this->optimised_path[0], this->optimised_path[this->optimised_points_count-1]);

        debug.println();
        for(int i=0; i<this->optimised_points_count; i++){
            debug.print(this->optimised_path[i].x);
            debug.print(" ");
            debug.print(this->optimised_path[i].y);
            debug.print(":");
        }
        debug.println();

        for(int i=0; i<this->optimised_points_count; i++){
            if(this->optimised_path[i].x > 2000){
                this->additional_points[this->additional_points_count++] = {this->optimised_path[i].x, this->optimised_path[i].y - this->distance_between_optimised_points[i]/3};
                this->additional_points[this->additional_points_count++] = {this->optimised_path[i].x, this->optimised_path[i].y + this->distance_between_optimised_points[(i+1)%this->optimised_points_count]/3};
            }
            else if(this->optimised_path[i].y > 2000){
                this->additional_points[this->additional_points_count++] = {this->optimised_path[i].x + this->distance_between_optimised_points[i]/3, this->optimised_path[i].y};
                this->additional_points[this->additional_points_count++] = {this->optimised_path[i].x - this->distance_between_optimised_points[(i+1)%this->optimised_points_count]/3, this->optimised_path[i].y};
            }
            else if(this->optimised_path[i].x < 1000){
                this->additional_points[this->additional_points_count++] = {this->optimised_path[i].x, this->optimised_path[i].y + this->distance_between_optimised_points[i]/3};
                this->additional_points[this->additional_points_count++] = {this->optimised_path[i].x, this->optimised_path[i].y - this->distance_between_optimised_points[(i+1)%this->optimised_points_count]/3};
            }
            else if(this->optimised_path[i].y < 1000){
                this->additional_points[this->additional_points_count++] = {this->optimised_path[i].x - this->distance_between_optimised_points[i]/3, this->optimised_path[i].y};
                this->additional_points[this->additional_points_count++] = {this->optimised_path[i].x + this->distance_between_optimised_points[(i+1)%this->optimised_points_count]/3, this->optimised_path[i].y};
            }
        }

        for(int i=0, j=1; i<this->optimised_points_count; i+=6, j+=2){
            this->point_for_bese[0] = this->optimised_path[i];
            this->point_for_bese[1] = this->additional_points[j]; 
            this->point_for_bese[2] = this->additional_points[(j+1)%this->additional_points_count];
            this->point_for_bese[3] = this->optimised_path[(i+1)%this->optimised_points_count];
            generate_bese_line(5, i);
            debug.println();
            for(int i=0; i<this->optimised_points_count; i++){
                debug.print(this->optimised_path[i].x);
                debug.print(" ");
                debug.print(this->optimised_path[i].y);
                debug.print(":");
            }
            debug.println();
        }
    }
    
    void last_blob(){
        for(int i=this->points_count-1; i>0; i--){
            if(this->connected_blobs[i] != -1){
                this->last_blob_ind = this->connected_blobs[i];
                this->last_blob_color = this->blobs_colors[i];
            }
        }
    }
    
    void print(){
        for(int i=0; i<points_count; i++){
            debug.print(this->points[i].x);
            debug.print(" : ");
            debug.print(this->points[i].y);
            debug.println(" ");
        }
    }
};

extern path robot_path;
#endif