#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

ros::Publisher pub_com_img;

void stereoCallback(
const sensor_msgs::ImageConstPtr& cam0_img,
const sensor_msgs::ImageConstPtr& cam1_img) {
    cv_bridge::CvImageConstPtr cam0_c_img = cv_bridge::toCvCopy(cam0_img, sensor_msgs::image_encodings::MONO8);
    cv_bridge::CvImageConstPtr cam1_c_img = cv_bridge::toCvCopy(cam1_img, sensor_msgs::image_encodings::MONO8);
    cv::Mat combine_image;
    cv::vconcat(cam0_c_img->image, cam1_c_img->image, combine_image);
    // cam0_c_img->image = combine_image;
    sensor_msgs::ImageConstPtr combine_image_ros_msg = cv_bridge::CvImage(cam0_img->header, sensor_msgs::image_encodings::MONO8,combine_image).toImageMsg();
    pub_com_img.publish(combine_image_ros_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "combine_image_node");
    ros::NodeHandle n("~");
    pub_com_img = n.advertise<sensor_msgs::Image>("/stereo_cam/image_raw",1000);
    // ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);
    message_filters::Subscriber<sensor_msgs::Image> cam0_img_sub;
    cam0_img_sub.subscribe(n, "/cam0/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::Image> cam1_img_sub;
    cam1_img_sub.subscribe(n, "/cam1/image_raw", 10);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> stereo_sub(10);
    stereo_sub.connectInput(cam0_img_sub, cam1_img_sub);
    stereo_sub.registerCallback(&stereoCallback);
    ros::spin();
}