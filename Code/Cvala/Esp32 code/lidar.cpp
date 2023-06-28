#include <define.h>

point points_xy[400];

lidar_data lidar;

bool turning = 0;

volatile bool reading_flag = 0;

typedef struct __attribute__((packed)) {
  uint16_t distance;
  uint8_t intensity;
} LidarPoint;

typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t length;
  uint16_t speed;
  uint16_t start_angle;
  LidarPoint points[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc;
} LidarPackage;

typedef struct {
  int16_t x;
  int16_t y;
} Point;

const byte CrcTable[256] = {
 0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
 0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
 0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
 0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
 0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
 0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
 0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
 0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
 0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
 0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
 0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

LidarPackage package;

bool flag = true;
uint8_t num_pkg_byte = 0;

long good_package_count = 0;
long bad_package_count = 0;

int16_t circle_buff[1500];
int circle_buff_pointer = 2;

void LidarRead() {
  if (SerialLidar.available()) {
    uint8_t cur_byte = *((uint8_t*)&package + num_pkg_byte) = SerialLidar.read();
        
    if ((num_pkg_byte == 0 && cur_byte != LIDAR_HEADER) || (num_pkg_byte == 1 && cur_byte != LIDAR_LENGTH)) { 
      num_pkg_byte = 0;
    } else {
      num_pkg_byte++;
    }

    if (num_pkg_byte == sizeof(LidarPackage) && CalCRC((uint8_t*)&package, sizeof(LidarPackage) - 1) == package.crc) {
      ProcessingPackage();
      good_package_count++;
      //Serial.println(millis());
    }

    if (num_pkg_byte == sizeof(LidarPackage) && CalCRC((uint8_t*)&package, sizeof(LidarPackage) - 1) != package.crc) {
       // for (uint8_t i = 0; i < 47; ++i) {
       //   Serial.print(*((uint8_t*)&package + num_pkg_byte));Serial.print(' ');
       // }
       // Serial.println();
       bad_package_count++;
    }

    num_pkg_byte = num_pkg_byte % sizeof(LidarPackage);
  }
}

void ProcessingPackage() {
    double delta = 0;
    /*package.start_angle -= 5;
    package.end_angle -= 5;

    if(package.start_angle < 0){
      package.start_angle = 360.0-package.start_angle;
    }
    if(package.end_angle < 0){
      package.end_angle = 360.0-package.end_angle;
    }*/

    if(package.start_angle < package.end_angle){
        delta = (package.end_angle-package.start_angle)/100.0/12.0;
    }
    else{
        delta = ((360 - package.start_angle/100.0)+package.end_angle/100.0)/12.0;
    }

    for (int i = 0; i < POINT_PER_PACK; ++i) {
        double angle = (package.start_angle / 100.0) + (delta * i);
        if (angle > 360){
            angle -= 360;
        }
        processingIntensityPoint(package.points[i].distance, package.points[i].intensity, angle);
    }
}

double old_angle = 100000;

void processingIntensityPoint(uint16_t distance, uint8_t intensity, double angle) {
    if (distance > 4500) {
        return;
    }
    //Serial.println(angle);
    if (angle < old_angle) {
        //Serial.print(angle);
        //Serial.print(" ");
        //Serial.println(old_angle);
        lidar.new_revolution = 1;
        lidar.poits_amount = lidar.current_point;
        //circle_buff[0] = -(1 << 15);
        //circle_buff[1] = lidar.poits_amount;
        //debug.write((byte*)circle_buff, (circle_buff_pointer) * 2);
        circle_buff_pointer = 2;
        lidar.current_point = 0;

    }
    old_angle = angle;

    if (distance != 0) {
        double angle_rad = radians(angle);
        //Serial.println(lidar.current_point);
        points_xy[lidar.current_point++] = point(int(distance * sin(angle_rad) + 0.5), int(distance * cos(angle_rad) + 0.5), float(angle));
        circle_buff[circle_buff_pointer++] = points_xy[lidar.current_point].x;
        circle_buff[circle_buff_pointer++] = points_xy[lidar.current_point].y;
    }
}

uint8_t CalCRC(uint8_t *p, uint8_t length) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < length; ++i) {
    crc = CrcTable[(crc ^ *p++) & 0xff];
  }
  return crc;
}



 
