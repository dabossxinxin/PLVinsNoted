#include "feature_tracker.h"
/*初始化静态成员变量*/
int FeatureTracker::n_id = 0;
/*判断指定特征点是否在设定边界内*/
bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x &&
        img_x < COL - BORDER_SIZE &&
        BORDER_SIZE <= img_y && 
        img_y < ROW - BORDER_SIZE;
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}
/*默认构造函数*/
FeatureTracker::FeatureTracker()
{
}
/*设置图像mask*/
void FeatureTracker::setMask()
{
    /*判断VINS使用的相机是否为鱼眼相机*/
    if(FISHEYE) {
        mask = fisheye_mask.clone();
    }
    else {
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    }
    /*保存被长时间跟踪的特征点*/
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;
    for (unsigned int i = 0; i < forw_pts.size(); i++) {
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));
    }
    /*特征点按照被观测次数降序排列*/
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b) 
    {
        return a.first > b.first;
    });
    /*重新初始化当前帧特征点&索引&被观测次数*/
    forw_pts.clear();
    ids.clear();
    track_cnt.clear();
    /*重新组织当前特征点&索引&被观测次数*/
    for (auto &it : cnt_pts_id) {
        if (mask.at<uchar>(it.second.first) == 255) {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}
/*添加当前帧观测到的特征点*/
void FeatureTracker::addPoints()
{
    /*添加当前帧观测到的特征点并赋初值*/
    for (auto &p : n_pts) {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}
/*读取图像&提取特征*/
void FeatureTracker::readImage(const cv::Mat &_img)
{
    cv::Mat img;
    TicToc t_r;
    /*直方图均衡化*/
    if (EQUALIZE) {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else {
        img = _img;
    }
    /*当前图像为空说明系统刚刚运行*/
    if (forw_img.empty()) {
        prev_img = cur_img = forw_img = img;
    }
    /*当前图像非空说明系统已经运行*/
    else {
        forw_img = img;
    }
    /*初始化当前图像特征点*/
    forw_pts.clear();
    /*LK光流特征点跟踪*/
    if (cur_pts.size() > 0) {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        /*使用LK光流进行特征点跟踪*/
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);
        /*将跟踪成功但是在图像边界之外的点重置为失败*/
        for (int i = 0; i < int(forw_pts.size()); i++) {
            if (status[i] && !inBorder(forw_pts[i])) { 
                status[i] = 0;
            }
        }
        /*根据光流跟踪状态重置跟踪特征点&索引*/ 
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("Temporal optical flow costs: %fms", t_o.toc());
    }
    /*是否发布当前帧*/
    if (PUB_THIS_FRAME) {
        /*通过Fundamental矩阵剔除离群点*/
        rejectWithF();
        /*更新track_cnt*/
        for (auto &n : track_cnt) {
            n++;
        }
         /*将已经检测出特征点的区域设置mask，其他区域继续检测新的特征*/
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());
        /*使用OpenCV接口提取一些新的特征点*/
        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0) {
            /*检查mask参数的合理性*/
            if(mask.empty()){
                cout << "mask is empty " << endl;
            }
            if (mask.type() != CV_8UC1) {
                cout << "mask type wrong " << endl;
            }
            if (mask.size() != forw_img.size()) {
                cout << "wrong size " << endl;
            }
            cv::goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.1, MIN_DIST, mask);
        }
        else {
            n_pts.clear();
        }
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());
        /*将新特征点加入到forw_img中*/
        ROS_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());

        prev_img = forw_img;
        prev_pts = forw_pts;
    }
    cur_img = forw_img;
    cur_pts = forw_pts;
}

void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_prev_pts(prev_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < prev_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }
        vector<uchar> status;
        cv::findFundamentalMat(un_prev_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = prev_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}
/*使用静态成员变量更新当前帧提取的特征点ID*/
bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size()) {
        if (ids[i] == -1) {
            ids[i] = n_id++;
        }
        return true;
    }
    else {
        return false;
    }
}
/*读取相机内参*/
void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("Reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}
/*显示去畸变图像*/
void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(1);
}
/*显示去畸变图像*/
void FeatureTracker::showUndistortion()
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    cv::Mat undist_map1_, undist_map2_;

    m_camera->initUndistortRectifyMap(undist_map1_,undist_map2_);
    cv::remap(cur_img, undistortedImg, undist_map1_, undist_map2_, CV_INTER_LINEAR);

    cv::imshow("undist", undistortedImg);
    cv::waitKey(1);
}
/*去除特征点的畸变*/
vector<cv::Point2f> FeatureTracker::undistortedPoints()
{
    vector<cv::Point2f> un_pts;
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }

    return un_pts;
}
