#ifndef BASE_H
#define BASE_H
#pragma once
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <mutex>
#include "serial/serial.h"
#include "dahua/dhua.h"

using namespace std;
using namespace cv;
using namespace cv::ml;


#define BIG 1
#define SMALL 0
#define BUFF 3

/*   全局宏定义，控制终端和图像信息输出         */

#define SHOW_FINAL                //显示最终图像
//#define SAVE_VIDEO             //保存最终图像为视频

//#define SHOW_GRAY                  //显示灰度图
//#define SHOW_SINGLE_THRESH            //显示灯条二值化图像
//#define DRAW_LIGHTS                //显示单灯条图像
//#define OUTPUT_SINGLE_information       //输出灯条信息
//#define SHOW_DOUBLE_LIGHTS              //显示双灯条匹配图像
//#define SHOW_ROBOTS_INFORMATIONS       //显示机器人信息
#define DRAW_ROI_RECT             //画ROI矩形
#define SHOW_ROI_AERA             //显示ROI区域

//#define SERIAL_OPEN             //串口开关

typedef struct base
{
    Ptr<KNearest> model_R = StatModel::load<KNearest>("/home/zhang/桌面/欣竹视觉_2023_框架/BamThriving-VISION/R.xml");

    Mat CAMERA_MATRIX = (Mat_<double>(3, 3) <<1.288341395548970e+03 , 0 , 3.478144530532289e+02 ,
                                  0 , 1.286551825922169e+03 , 2.430646717051290e+02 , 0 , 0 , 1);
    Mat DISTORTION_COEFF = (Mat_<double>(4, 1) <<-0.077012717292231 , 0.060457593355343 , 0 , 0);


    int enemy_team=0;           //enemy：0 for blue ,1 for red
    bool flag;                   //错误就跳过，减少运行错误和时间
    Mat src;
}Base;



typedef struct robot
{
    RotatedRect roborect;                     //机器人装甲板旋转矩形信息
    double robonum=0;                         //机器人序号
    int robot_type;                        //装甲版类型
    Mat ROI;                              //ROI
    double pitch=0;
    double yaw=0;
    double distance=0;
    /*   pnp平移向量结算坐标系     */
    double value_x=0;
    double value_y=0;
    double value_z=0;
//测试
    Mat src;
}Robot;



#endif // BASE_H
