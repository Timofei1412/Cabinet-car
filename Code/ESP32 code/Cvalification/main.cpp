#include <define.h>

void setup() {
  Serial.begin(500000);
  SerialLidar.setRxBufferSize(512);
  SerialLidar.begin(230400, SERIAL_8N1, 16, -1);
  trackingCam.init(51, 400000);

  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(motor_1, OUTPUT);
  pinMode(motor_2, OUTPUT);

  analogWrite(lidar_speed, 210);

  check_battery();
  
  steering.attach(servo_pin);
  steering.write(zero_angle);
  Serial.println("Start");
  attachInterrupt(digitalPinToInterrupt(enc_a), encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_b), encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_r_a), encoder_r, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_r_b), encoder_r, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_l_a), encoder_l, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_l_b), encoder_l, CHANGE);
  attachInterrupt(start_btn, start_moving, FALLING);

  delay(1000);
}

void loop(){
  calculate_speed();
  get_data_from_camera();
  LidarRead();
  controll_speed();
  //odometry();
}
