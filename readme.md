#   西南民族大学 欣竹战队 视觉项目  

-------version: 2022.11.11--------


#    BamTriving-VISION ：

   -----装甲板识别
   |
   |
   |
   -----数字识别
   |
   |
   |
   -----Prediction:PNP结算和卡尔曼预测
   |
   |
   |
   -----Thread：图像获取线程（_some.cpp）和代码运行主线程，主线需要的一些函数（_some.cpp） 
   |
   |
   |
   -----serial：串口协议
   |
   |
   |
   -----configer.h：存放调试代码宏定义、robot结构体、base结构体存放全局变量
   |
   |
   |
   -----main.cpp：三个线程
   |
   |
   |
   -----build:代码产程文件
   |
   |
   |
   |----主线：图像获取后进行灯条识别、装甲板拟合、数字识别（只能识别不能检测），所有识别的装甲板信息储存   在robots里，识别条件符合后进行PNP结算，装甲板选择（Robot select_robot()--对装甲板进行评分判断，优先选择15count内数字相同的装甲板），识别错误或失帧电控信息不动返回，条件符进行预测，选取不同装甲板后重置卡尔曼，结束一帧。



tips：代码调试效果看biuld里视频，或者修改代码读取biuld未处理的视频文件。时间帧数（count、t_pre）处理为枚举场上事件进行修改，理解的时候可能有点困难。导入新项目的时候记得更改.pro文件里的部分路径和base里的.xml文件的路径（文件在项目的文件夹里）.



