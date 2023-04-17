#include "threads.h"
#include"../configer.h"
#include <pthread.h>
#include "../Prediction/pnpsolve.h"
#include "../Detection/detect.h"


void Threads::image(){


#ifdef SAVE_VIDEO
    VideoWriter writer;
    for(int i=0;i<999;i++)
    {
        VideoCapture cap;
        cap.open("Video"+std::to_string(i)+".avi");
        if(!cap.isOpened())
        {
            writer.open("Video"+std::to_string(i)+".avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), frame_rate, Size(640, 480));
            break;
        }
    }
#endif
    while (true) {

        while(camera_count - image_count < 1)
        {
        }

        if(1){

            if(Switch==0)
                dahua.setCameraExposureTime(dahua.cameraSptr,8000,false);

            Switch=1;

            vector<Robot> robots;                  //所有识别到的机器人数据
            Robot robot;                           //当前目标数据!Serial.If_Buff
            robot.yaw=0;
            robot.pitch=0;

            /*
              识别部分:得到robots（RotatedRect、ROI）
            */
            detect detect(base,robots);

            /*
              数字识别部分：补充robots.number,不符合则删除
            */
            robots=bam_svm.output0(robots);

            judge_robotype(robots);


            double t=((double)getTickCount()-t_) / getTickFrequency();           //时间
            t_ =static_cast<double>(getTickCount());
            double FPS=1.0/t;
            cout<<"FPS : "<<FPS<<endl;


            if(robots.size())
            {
                pnpsolve anglesolve(robots,base);
                robot=select_robot(robots,robot_last);
                cout<<"pitch : "<<robot.pitch<<endl;
                cout<<"yaw : "<<robot.yaw<<endl;
                cout<<"distance : "<<robot.distance<<endl;
            }
            else{
                base.flag=0;
                if(robot_last.distance>1)
                    this->count++;
                if(this->count>count_flash){
                    this->count=0;
                    robot_last.distance=0;
                }
            }

            Mat src;
            src=show_final(robot,robot_last,FPS);
    #ifdef SHOW_FINAL
            imshow("FINAL_IMG",src);
    #endif
    #ifdef SAVE_VIDEO
            writer<<src;
    #endif


            cout<<"________________________Serial debug______________________"<<endl;
            printf("get_yaw :%lf\n",Serial.getYaw);
            printf("get_pitch:%lf\n",Serial.getPitch);
            cout<<"get_buff : "<<Serial.If_Buff<<endl;
            cout<<"BULLED_SPEED : "<<Serial.getSpeed<<endl;



            robot.pitch+=Serial.getPitch;       //相对转绝对
            robot.yaw+=Serial.getYaw;
            cout<<"absolute pitch : "<<robot.pitch<<endl;
            cout<<"absolute yaw : "<<robot.yaw<<endl;
            cout<<"absolute distance : "<<robot.distance<<endl;

            if(!base.flag){

                cout<<"no pre_pitch  "<<serial_p<<endl;
                cout<<"no pre_yaw  "<<Serial.getYaw<<endl;

                if(this->count>0&&this->count<this->count_flash&&0){
                    cout<<"no  debug informations____________"<<endl;
                    pre.prediction(pre.pre_robot_last,pre.pre_robot,1,t,Serial.getSpeed);
                    serial_pitch=Serial.getPitch+gravity_pitch;
                    serial_yaw=pre.pre_robot.yaw;
                }
                else{
                    serial_pitch=Serial.getPitch;
                    serial_yaw=Serial.getYaw;
                    final_x=robot.roborect.center.x;
                    final_y=robot.roborect.center.y;
                    fire_mode =NoFind;
                }
                image_count++;
                continue;
            }

            int flag=0;
            if(robot.robonum==robot_last.robonum&&fabs(robot.yaw-robot_last.yaw)<0.8){
                flag=1;
            }

            Robot robot_trans=robot;               //装甲数据转移

            pre.prediction(robot_last,robot,flag,t,Serial.getSpeed);



            cout<<"predict  debug informations____________"<<endl;
            cout<<"pre_pitch  "<<robot.pitch<<endl;
            cout<<"pre_yaw  "<<robot.yaw<<endl;


            serial_pitch=robot.pitch;
            serial_yaw=robot.yaw;
            fire_mode=NOFire;

            robot_last=robot_trans;


        }
        else{

            if(Switch==1)
                dahua.setCameraExposureTime(dahua.cameraSptr,7500,false);

            Switch=0;
            this->count=0;

            vector<Robot> buff_rects(1);                                    //风车装甲模块初始化
            Robot buff_rect;
            buff_rects[0].robonum=-1;
            buff_rects[0].robot_type=BUFF;
            Buff.buff_flag=0;

            Mat src=base.src;

#ifdef SAVE_VIDEO
            writer<<src;
#endif

            Point2f center=Point2f(src.rows/2,src.cols/2);

            if(Buff.find_R(src,base.enemy_team,center,base.model_R)||1){
                 Buff.findSquares(src,base.enemy_team,center);
                 buff_rects[0].roborect = RotatedRect(Buff.point_find,Size2f(26,26), 0);
            }
            cout<<"Serial   debug informations_______"<<endl;
            cout<<"get_pitch : "<<Serial.getPitch<<endl;
            cout<<"get_yaw : "<<Serial.getYaw<<endl;
            cout<<"get_color : "<<Serial.getColor<<endl;
            cout<<"get_buff : "<<Serial.If_Buff<<endl;
            cout<<"BULLED_SPEED : "<<Serial.getSpeed<<endl;


            double t=((double)getTickCount()-t_) / getTickFrequency();           //时间
            t_ =static_cast<double>(getTickCount());
            double FPS=1.0/t;
            cout<<"FPS : "<<FPS<<endl;
//            putText(base.src,"FPS:"+std::to_string(FPS),Point(10,20),1,1,Scalar(0,255,0),2,LINE_8);
//            imshow("debug",base.src);

            if(FPS<32){
                serial_pitch=Serial.getPitch;
                serial_yaw=Serial.getYaw;

                image_count++;
                continue;
            }

            if(Buff.buff_flag){
                pnpsolve anglesolve(buff_rects,base);
                buff_rect=buff_rects[0];
                cout<<"pitch : "<<buff_rect.pitch<<endl;
                cout<<"yaw : "<<buff_rect.yaw<<endl;
                cout<<"distance : "<<buff_rect.distance<<endl;

            }
            else{

                serial_pitch=Serial.getPitch;
                serial_yaw=Serial.getYaw;
                fire_mode=NoFind;
                image_count++;
                continue;
            }

            serial_flag=1;
            buff_rect.pitch+=Serial.getPitch;       //相对转绝对
            buff_rect.yaw+=Serial.getYaw;
            serial_flag=0;

            int flag=0;
            if(buff_rect.robonum==robot_last.robonum){
                flag=1;
            }
            Robot robot_trans=buff_rect;               //装甲数据转移


            cout<<"predict  debug informations____________"<<endl;
            cout<<"pre_pitch  "<<buff_rect.pitch<<endl;
            cout<<"pre_yaw  "<<buff_rect.yaw<<endl;


            serial_pitch=buff_rect.pitch;
            serial_yaw=buff_rect.yaw;
            fire_mode=NOFire;

            robot_last=robot_trans;

        }

        image_count++;
    }
}


