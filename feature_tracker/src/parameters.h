#pragma once
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

extern int ROW;                                 //输入图像行数量
extern int COL;                                 //输入图像列数量
extern int FOCAL_LENGTH;                        //VINS使用相机的焦距
const int NUM_OF_CAM = 1;                       //VINS使用相机的数量（默认为单目）

extern std::string IMAGE_TOPIC;                 //图像话题名称
extern std::string IMU_TOPIC;                   //IMU话题名称
extern std::string FISHEYE_MASK;                //鱼眼相机Mask文件路径  
extern std::vector<std::string> CAM_NAMES;      //VINS使用相机的名称
extern int MAX_CNT;                             //最大特征点提取数量
extern int MIN_DIST;                            //同名特征点最小距离
extern int WINDOW_SIZE;                         //VINS滑窗大小
extern int FREQ;                                //VINS传入图像频率
extern double F_THRESHOLD;                      //Ransac参数
extern int SHOW_TRACK;                          //将跟踪图像作为话题发布出去
extern int STEREO_TRACK;                        //
extern int EQUALIZE;                            //是否对输入图像进行直方图均衡
extern int FISHEYE;                             //是否使用鱼眼相机
extern bool PUB_THIS_FRAME;                     //是否发布当前帧信息
/*读取config文件中的参数*/
void readParameters(ros::NodeHandle &n);
