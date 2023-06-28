#include <define.h>

ESP32SPISlave camera;
const uint16_t BUFFER_SIZE {32};
uint8_t camera_tx_buf[32];
uint8_t camera_rx_buf[32];

Servo steering;
BluetoothSerial debug;

const int enc_table[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
volatile long long int enc_count=0;
volatile long long int enc_count_last=0;

float speed_kP=4;//1.3
float speed_kI=0.1;//0.01
float speed_kD=0.15;//0.08
float speed_i_sum=0;
float last_error_speed=0;

float current_speed = 0;
unsigned long long time_passed = 0;
unsigned long long speed_calculation_timer = 0;
int target_speed = 0;

int distances[3] = {0,0,0};
int deviation_from_center = 0;

float steering_kP=0.4;
float steering_kI=0;//0.001
float steering_kD=0;
float steering_i_sum=0;
float last_error_steering=0;

bool move=0;
