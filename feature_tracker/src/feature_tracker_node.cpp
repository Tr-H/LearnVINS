#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf;

ros::Publisher pub_img,pub_match;
ros::Publisher pub_restart;

FeatureTracker trackerData[NUM_OF_CAM];
// FeatureTracker trackerData[2];
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec();
        last_image_time = img_msg->header.stamp.toSec();
        return;
    }
    // detect unstable camera stream
    if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || img_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    last_image_time = img_msg->header.stamp.toSec();
    // frequency control
    if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image;
    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ROS_DEBUG("processing camera %d", i);
        if (i != 1 || !STEREO_TRACK) {
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), img_msg->header.stamp.toSec());
        } else {
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }

    if ( PUB_THIS_FRAME && STEREO_TRACK && trackerData[0].cur_pts.size() > 0)
    {
        pub_count++;
        r_status.clear();
        r_err.clear();
//         TicToc t_o;
//         cv::calcOpticalFlowPyrLK(trackerData[0].cur_img, trackerData[1].cur_img, trackerData[0].cur_pts, trackerData[1].cur_pts, r_status, r_err, cv::Size(21, 21), 3);
//         ROS_DEBUG("spatial optical flow costs: %fms", t_o.toc()); 
                
//         vector<cv::Point2f> ll, rr;
//         vector<int> idx;
//         for (unsigned int i = 0; i < r_status.size(); i++)  
//         {
//             if (r_status[i] && !inBorder(trackerData[1].cur_pts[i]))   
//                 r_status[i] = 0;

//             if (r_status[i])
//             {
// // ????
//                 idx.push_back(i);
//                 Eigen::Vector3d tmp_p;
//                 trackerData[0].m_camera->liftProjective(Eigen::Vector2d(trackerData[0].cur_pts[i].x, trackerData[0].cur_pts[i].y), tmp_p);
//                 tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
//                 tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
//                 ll.push_back(cv::Point2f(tmp_p.x(), tmp_p.y()));

//                 trackerData[1].m_camera->liftProjective(Eigen::Vector2d(trackerData[1].cur_pts[i].x, trackerData[1].cur_pts[i].y), tmp_p);
//                 tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
//                 tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
//                 rr.push_back(cv::Point2f(tmp_p.x(), tmp_p.y()));
//             }
//         }
//         if (ll.size() >= 8)
//         {
//             vector<uchar> status;
//             TicToc t_f;
//             cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, 1.0, 0.5, status);
//             ROS_DEBUG("find f cost: %f", t_f.toc());
//             int r_cnt = 0;
//             for (unsigned int i = 0; i < status.size(); i++)
//             {
//                 if (status[i] == 0)
//                     r_status[idx[i]] = 0;
//                 r_cnt += r_status[idx[i]];
//             }
//             ROS_INFO("NUM_cam1_pts: %d", r_cnt); // 88
//         }
        std::vector<cv::Point2f> cam1_pts_undistorted, cam1_pts_distorted;
        // 像素坐标转归一化平面坐标
        std::cout << cv_R << std::endl;
        trackerData[0].m_camera->_undistortPoints(trackerData[0].cur_pts, DISTORTION_MODEL, cam1_pts_undistorted, cv_R);
        trackerData[1].m_camera->_distortPoints(cam1_pts_undistorted, DISTORTION_MODEL, cam1_pts_distorted);

        cv::calcOpticalFlowPyrLK(trackerData[0].cur_img, trackerData[1].cur_img, trackerData[0].cur_pts, cam1_pts_distorted, r_status, r_err, 
                                 cv::Size(21, 21), 3, cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
        trackerData[1].cur_pts = cam1_pts_distorted;

        vector<int> idx;
        std::vector<cv::Point2f> ll, rr;
        for (unsigned int i = 0; i < r_status.size(); i++)  
        {
            if (r_status[i] && !inBorder(trackerData[1].cur_pts[i]))   
                r_status[i] = 0;

            if (r_status[i])
            {
                idx.push_back(i);
            }
        }
        ROS_INFO("NUM_cam1_points: %d", idx.size()); 
        trackerData[0].m_camera->_undistortPoints(trackerData[0].cur_pts, DISTORTION_MODEL, ll);
        trackerData[1].m_camera->_undistortPoints(trackerData[1].cur_pts, DISTORTION_MODEL, rr);
        reduceVector(ll, r_status);
        reduceVector(rr, r_status);
        if (ll.size() >= 8)
        {
            vector<uchar> status;
            cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, 1.0, 0.5, status);
            int r_cnt = 0;
            for (unsigned int i = 0; i < status.size(); i++)
            {
                if (status[i] == 0)
                    r_status[idx[i]] = 0;
                r_cnt += r_status[idx[i]];
            }
            ROS_INFO("NUM_cam1_RANSAC: %d", r_cnt); // 88
        }
    }

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < 1; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }
    trackerData[1].ids = trackerData[0].ids;
    trackerData[1].track_cnt = trackerData[0].track_cnt;

    reduceVector(trackerData[1].cur_pts, r_status);
    reduceVector(trackerData[1].ids, r_status);
    reduceVector(trackerData[1].track_cnt, r_status);



   if (PUB_THIS_FRAME)
   {
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::ChannelFloat32 velocity_y_of_point;

        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world";

        // vector<set<int>> hash_ids(NUM_OF_CAM);
        vector<set<int>> hash_ids(1);
        // for (int i = 0; i < NUM_OF_CAM; i++)
        for (int i = 0; i < 1; i++)
        {
            if (i != 1 || !STEREO_TRACK) {
                auto &un_pts = trackerData[i].cur_un_pts;
                auto &cur_pts = trackerData[i].cur_pts;
                auto &ids = trackerData[i].ids;
                auto &pts_velocity = trackerData[i].pts_velocity;
                for (unsigned int j = 0; j < ids.size(); j++)
                {
                    if (trackerData[i].track_cnt[j] > 1)
                    {
                        int p_id = ids[j];
                        hash_ids[i].insert(p_id);
                        // ROS_INFO("cam0_num_feature_pts, %d", hash_ids[i].size());
                        geometry_msgs::Point32 p;
                        p.x = un_pts[j].x;
                        p.y = un_pts[j].y;
                        p.z = 1;

                        feature_points->points.push_back(p);
                        id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                        u_of_point.values.push_back(cur_pts[j].x);
                        v_of_point.values.push_back(cur_pts[j].y);
                        velocity_x_of_point.values.push_back(pts_velocity[j].x);
                        velocity_y_of_point.values.push_back(pts_velocity[j].y);
                    }
                }
            // } else if (STEREO_TRACK) {
            } else if (0) {
                auto &r_un_pts = trackerData[1].cur_un_pts;
                auto &ids = trackerData[0].ids; 
                ROS_INFO("HELLO5: %d", trackerData[1].cur_un_pts.size());
                for (unsigned int j = 0; j < ids.size(); j++) {
                    ROS_INFO("HELLO6: %d %d", ids.size(), r_status.size()); // 150 150
                    if (r_status[j]) {
                        int p_id = ids[j];
                        hash_ids[i].insert(p_id);
                        geometry_msgs::Point32 p;
                        p.x = r_un_pts[j].x;
                        p.y = r_un_pts[j].y;
                        p.z = 1;

                        feature_points->points.push_back(p);
                        id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    }
                }
            }
        }
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);
        ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_img.publish(feature_points);

        if (SHOW_TRACK)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                // cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                    std::string temp_num = std::to_string(trackerData[i].ids[j]);
                    cv::putText(tmp_img,
                                temp_num.c_str(),
                                trackerData[i].cur_pts[j],
                                cv::FONT_HERSHEY_SIMPLEX,
                                0.3,
                                cv::Scalar(255 * (1 - len), 0, 255 * len),
                                1,
                                8,
                                false);
                    //draw speed line
                    /*
                    Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                    Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                    Vector3d tmp_prev_un_pts;
                    tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                    tmp_prev_un_pts.z() = 1;
                    Vector2d tmp_prev_uv;
                    trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                    cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                    */
                    //char name[10];
                    //sprintf(name, "%d", trackerData[i].ids[j]);
                    //cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                }
            }
            //cv::imshow("vis", stereo_img);
            //cv::waitKey(5)
            sensor_msgs::ImageConstPtr combine_image_ros_msg = cv_bridge::CvImage(img_msg->header, sensor_msgs::image_encodings::BGR8, stereo_img).toImageMsg();;
            pub_match.publish(combine_image_ros_msg);
        }
    }
    ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    // readParameters(n);
    readParameters1(n);
    readParameters2(n);

    // for (int i = 0; i < NUM_OF_CAM; i++)
    for (int i = 0; i < 2; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        // for (int i = 0; i < 2; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                ROS_INFO("load mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }

    ROS_INFO(" subscribe topic :%s", IMAGE_TOPIC.c_str());

    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);

    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000);
    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */
    ros::spin();
    return 0;
}


// new points velocity is 0, pub or not?
// track cnt > 1 pub?