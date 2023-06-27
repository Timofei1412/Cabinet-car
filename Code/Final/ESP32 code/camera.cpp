#include <define.h>

TrackingCamI2C trackingCam;

void get_data_from_camera(){
    //Функция считывания данных о кубики который видит камера

    uint8_t n = trackingCam.readBlobs(5);
    int8_t max_i = -1;
    int8_t max_size = 0;
    for(int i = 0; i < n; i++){
        if (trackingCam.blob[i].area > max_size){
        max_size = trackingCam.blob[i].area;
        max_i = i;
        }
    }
    if (max_i >= 0){
        camera_blob_color = trackingCam.blob[max_i].type;
        if (camera_blob_color == 0){
        camera_blob_color = -1;
        }
        camera_blob_distance = trackingCam.blob[max_i].area > 900; // 300// Расстояние до кубика
        camera_blob_side = trackingCam.blob[max_i].cx > 160; // 320// Ширина экрана
        if (camera_blob_side == 0){
            camera_blob_side = -1;
        }
    }
    else {
        camera_blob_color = 0;
        camera_blob_distance = 0; // Расстояние до кубика
        camera_blob_side = 0; // Ширина экрана
    }
    if(turn_direction){
        camera_blob_color *= -1;
    }
}