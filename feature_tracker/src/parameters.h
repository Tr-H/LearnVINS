#pragma once
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
const int NUM_OF_CAM = 2;


extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;
extern cv::Mat cv_R, cv_T;
extern std::string DISTORTION_MODEL;


void readParameters(ros::NodeHandle &n);
void readParameters1(ros::NodeHandle &n);
void readParameters2(ros::NodeHandle &n);
