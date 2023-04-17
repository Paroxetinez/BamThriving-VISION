#include "threads.h"
#include <pthread.h>
#include "../serial/serial.h"


void Threads::T_serial(){

    Serial.port_init(ss);
    while (1) {

        tcflush(ss,TCIFLUSH);
        waitKey(1);
        Serial.port_RECV(ss);                              //串口接收

        while (serial_flag) {
        }
        Serial.port_SEND(ss,serial_pitch,serial_yaw,fire_mode);

//        double t=((double)getTickCount()-t__) / getTickFrequency();           //时间
//        t__ =static_cast<double>(getTickCount());
//        double FPS=1.0/t;
//        cout<<" - - - - - - - - - - - - FPS_serial : "<<FPS<<endl;

    }

}
