#ifndef SVM_H
#define SVM_H
#pragma once
#include "cstdio"
#include "../configer.h"
#include <thread>
#include <sstream>
#include <chrono>
#include "../serial/serial.h"
#include "../buff/BUFF.h"
#include "../dahua/dhua.h"
#include "../Prediction/predictor.h"


class Bam_SVM
{


 public:
    Ptr<SVM>svm_hog=SVM::load("/home/zhang/桌面/欣竹视觉_2023_框架/BamThriving-VISION/xml/hog_grey.xml");
    Ptr<SVM>svm=SVM::load("/home/zhang/桌面/欣竹视觉_2023_框架/BamThriving-VISION/xml/svm_grey.xml");
    Mat trainingimg;
    vector<int> traininglabels;

void get_number(int num);
void get_xml();
void hog(int num);
void get_hog();
void output1();//自我测试1
void output2();//自我测试2
vector<Robot> output0(vector<Robot> robots);//实用函数，返回svm识别出的数字
private:
   void bam_svm();
   void digits();

};
//Bam_SVM bam_svm;


#endif // SVM_H
