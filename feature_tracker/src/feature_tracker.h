#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

/*判断图像特征点是否在指定图像边界内*/
bool inBorder(const cv::Point2f &pt);
/*根据光流跟踪状态重置跟踪特征点&索引*/
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
  public:
    /*默认构造函数*/
    FeatureTracker();
    /*读取图像&提取特征*/
    void readImage(const cv::Mat &_img);
    /*设置图像Mask*/
    void setMask();
    /*添加当前帧构造的特征点*/
    void addPoints();
    /*更新特征点ID*/
    bool updateID(unsigned int i);
    /*读取相机内参*/
    void readIntrinsicParameter(const string &calib_file);
    /*显示去畸变图像*/
    void showUndistortion();
    void showUndistortion(const string &name);
    /*使用Fundamental矩阵去除离群点*/
    void rejectWithF();
    /*获取去畸变特征点*/
    vector<cv::Point2f> undistortedPoints();
    
    /*图像mask*/
    cv::Mat mask;
    /*鱼眼镜头mask*/
    cv::Mat fisheye_mask;
    /*图像时序*/
    cv::Mat prev_img, cur_img, forw_img;
    /*当前帧提取的特征点*/
    vector<cv::Point2f> n_pts;
    /*特征点时序*/
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
    /*记录每个特征点的ID*/
    vector<int> ids;
    /*记录某个特征点被多少帧看见了*/
    vector<int> track_cnt;
    /*当前跟踪使用的相机模型*/
    camodocal::CameraPtr m_camera;
    /*静态成员变量，记录特征点的ID*/
    static int n_id;
};
