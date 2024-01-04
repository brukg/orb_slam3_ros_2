#ifndef ORB_SLAM3_ROS_2__MONO_ORB_SLAM_NODE
#define ORB_SLAM3_ROS_2__MONO_ORB_SLAM_NODE
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
// #include <sensor_msgs/imu.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include "orb_slam3/System.hpp"

namespace orb_slam3_ros2
{
  class ORBSlamNode : public rclcpp::Node
  {
  public:
      // ORBSlamNode(ORB_SLAM3::System* pSLAM) : mpSLAM(pSLAM);
      explicit ORBSlamNode();
      ~ORBSlamNode();
      // void ORBSlamNodeORB(ORB_SLAM3::System* pSLAM);
      /*
      * @brief Callback function for the monocular camera topic.
      */
      void monoCallback(const sensor_msgs::msg::Image::SharedPtr msg);

      /*
      * @brief callback function for the stereo camera topic.
      */
      void stereoCallback(const sensor_msgs::msg::Image::SharedPtr msgLeft, const sensor_msgs::msg::Image::SharedPtr msgRight);

      /*
      * @brief callback function for stereo compressed camera topic.
      */
      void stereoCompressedCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msgLeft, const sensor_msgs::msg::CompressedImage::ConstSharedPtr msgRight);

      /*
      * @brief callback function for the RGBD camera topic.
      */
      void rgbdCallback(const sensor_msgs::msg::Image::SharedPtr msgLeft, const sensor_msgs::msg::Image::SharedPtr msgDepth);

      // /* 
      // * @brief callback function for the IMU topic.
      // */
      // void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  private:
      ORB_SLAM3::System *mpSLAM;
      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_mono, sub_stereo, sub_rgbd;
      // rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_stereo_compressed;
      message_filters::Subscriber<sensor_msgs::msg::CompressedImage> image_l_sub_ ;
      message_filters::Subscriber<sensor_msgs::msg::CompressedImage> image_r_sub_ ;
      std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::CompressedImage,sensor_msgs::msg::CompressedImage>> sync_;

      cv_bridge::CvImagePtr cv_ptr;
      std::string str_vocabulary_file, str_settings_file;
      bool visualization;
  };
}// namespace orb_slam3_ros2

#endif // ORB_SLAM3_ROS_2__MONO_ORB_SLAM_NODE