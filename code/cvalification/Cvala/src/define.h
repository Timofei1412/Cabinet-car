#include <Arduino.h>
#include <Servo.h>
#include "BluetoothSerial.h"
#include <ESP32SPISlave.h>
#include <math.h>

#define enc_a 32
#define enc_b 33
#define motor_1 25
#define motor_2 26

#define servo_pin 14

#define lidar_speed 13

#define green_led 15
#define red_led 2
#define start_btn 12
#define sustem_btn 34
#define battery_voltage_pin 35


#define critical_voltage 5.9
#define devision_coefficient 39.6

#define zero_angle 110
#define angle_constrain 50
#define iMin -50
#define iMax 50

#define min_mot_signal 30

#define line_calculation_angle 40

#define distance_to_wall 230
#define turn_radius 400//600
#define corner_entrance_distance 200
#define turn_time 450//550


#ifndef DEFINE_H
#define DEFINE_H

#define SerialLidar Serial2
#define LIDAR_HEADER 0x54
#define LIDAR_LENGTH 0x2C
#define POINT_PER_PACK 12

extern ESP32SPISlave camera;
extern const uint16_t BUFFER_SIZE;
extern uint8_t camera_tx_buf[];
extern uint8_t camera_rx_buf[];


struct point{
    int x;
    int y;
    float angle;

    point(int X=0, int Y=0, float Angle=0)
    {
        x = X;
        y = Y;
        angle = Angle;
    }

    void print(){
        Serial.print(this->x);
        Serial.print(':');
        Serial.print(this->y);
        Serial.print(':');
        Serial.print(this->angle);
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
    volatile int poits_amount = 0;
    volatile int pakets = 0;
};

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

struct separators{
    int side_to_forward;
    int forward_end;

    void print(){
        Serial.print('{');
        Serial.print(this->side_to_forward);
        Serial.print(':');
        Serial.print(this->forward_end);
        Serial.print('}');
    }
};

extern Servo steering;
extern BluetoothSerial debug;
extern const int enc_table[16];
extern volatile long long int enc_count;
extern volatile long long int enc_count_last;
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
extern line side_line;
extern line forward_line;
extern line back_line;
extern line side_line2;
extern int distance_to_forward_line;
extern int distance_to_back_line;
extern bool turning;
extern volatile bool reading_flag;
extern long bad_package_count;
extern long good_package_count;
extern bool move;
extern int turns_count;
extern int turn_direction;
extern int16_t circle_buff[];
extern int circle_buff_pointer;
extern float angle_to_side_line;
extern int temporary_distance_to_wall;
extern int temporary_distance_to_wall2;
extern int distance_to_side_line;
extern int blobs_pot_coords[12][2];
extern unsigned int turn_forbidden_timer;
extern float angle_to_forward_line;

void check_battery();
void MainLoop(void * parameter);
void LidarProcessingLoop(void * parameter);

void encoder();
void calculate_speed();
void start_moving();
float speed_pid(int error);
void controll_speed();
void set_speed(int target_speed);
void set_steer_angle(float angle);
void turn();

void debug_parse();

void LidarRead();
void ProcessingPackage();
void processingIntensityPoint(uint16_t distance, uint8_t intensity, double angle);
uint8_t CalCRC(uint8_t *p, uint8_t length);
float steering_pid(float error);
void steer();
void recieve_data();

void get_data_from_camera();

float DegreesToRadians(float degrees);
void refresh_points();
point get_point_by_angle(float angle);
line calculate_line(float angle, int sector);
line line_by_two_points(point_d p1, point_d p2);
line line_by_two_points(point p1, point p2);
bool chech_point_in_square(point p, int x, int y, int half_side);
double angle_of_line(line line);
int distance_to_line(line line, point point);
int distance_between_points(point point1, point point2);
point intersection(line line1, line line2);
line line_approximation(point points[100], int num_points);
separators separate_lines(int start_angle, int end_angle);
void calculate_turn_direction(int start_angle, int end_angle);
line ransac_line_approximation(point points[100], int num_points, double threshold_distance, int max_iterations);


#endif