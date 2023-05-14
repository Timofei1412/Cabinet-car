#include <define.h>

TaskHandle_t LidarProcessingTask;
TaskHandle_t MainLoopTask;

void synk(){
  unsigned char tmp[1] = {0};
  while(tmp[0] != 0x54){
    Serial2.readBytes(tmp, 1);
  }
  unsigned char buff[46];
  Serial2.readBytes(buff, 46);
}


void setup() {
  Serial.begin(500000);
  SerialLidar.begin(230400, SERIAL_8N1, 16, -1);
  debug.begin("Robot debug");
  // debug.begin(500000);

  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(motor_1, OUTPUT);
  pinMode(motor_2, OUTPUT);
   

  check_battery();
  
  steering.attach(servo_pin);
  steering.write(zero_angle);
  Serial.println("Start");
  debug.println("Start");
  attachInterrupt(enc_a, encoder, CHANGE);
  attachInterrupt(enc_b, encoder, CHANGE);

  analogWrite(lidar_speed, 210);

  attachInterrupt(start_btn, start_moving, RISING);

  /*xTaskCreatePinnedToCore(
     LidarProcessingLoop,
     "LidarProcessing",
     10000,
     NULL,
     0,
     &LidarProcessingTask,
     1
  );

  xTaskCreatePinnedToCore(
     MainLoop,
     "Aprocsimation",
     10000,
     NULL,
     0,
     &MainLoopTask,
     0
  );*/

  //camera.setDataMode(SPI_MODE0);
  //camera.begin();
  //synk();
}

void MainLoop(void * parameter){
  while (true) { 
    //debug_parse();
    calculate_speed();
    //Serial.println("loop");
    refresh_points();
  }
}


void LidarProcessingLoop(void * parameter) {
  // единоразово запускающийся код (аналог setup на 1 ядре)
  // Обязательный! бесконечный цикл для 0 ядра (аналог loop на 1 ядре)
  while (true) { 
    LidarRead();
    //Serial.print(good_package_count);Serial.print(' ');Serial.println(millis());
    //if (millis() >= 2000 && flag) {
    //  Serial.print(good_package_count);Serial.print(' ');Serial.println(bad_package_count);
    //  flag = false;
    //}
  }
}

void send_short(uint16_t x) {
    debug.write((uint8_t*)&x, 2);
}

void loop(){
  //debug_parse();
  calculate_speed();
  LidarRead();
  refresh_points();
  controll_speed();
}
