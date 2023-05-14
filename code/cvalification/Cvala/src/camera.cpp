#include <define.h>

void get_data_from_camera(){
    String data;
    if (camera.remained() == 0)
        camera.queue(camera_rx_buf, camera_tx_buf, BUFFER_SIZE);
    while (camera.available()) {
        for (int i=0;i<BUFFER_SIZE ;i++){
            data += (char)camera_rx_buf[i];
        }
        if(data.substring(0, 7) == "ObjectC"){
            int color = 0;
            color = data.substring(7, 8).toInt();
        }
    }
}