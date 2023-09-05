#ifndef ORB_SLAM3_ROS_2__MONO_ORB_SLAM_NODE
#define ORB_SLAM3_ROS_2__MONO_ORB_SLAM_NODE
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include<opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include "orb_slam3/System.hpp"

namespace orb_slam3_ros2
{
  class MonocularSlamNode : public rclcpp::Node
  {
  public:
      // MonocularSlamNode(ORB_SLAM3::System* pSLAM) : mpSLAM(pSLAM);
      explicit MonocularSlamNode();
      ~MonocularSlamNode();
      // void MonocularSlamNodeORB(ORB_SLAM3::System* pSLAM);
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
      void stereoCompressedCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msgLeft, const sensor_msgs::msg::CompressedImage::SharedPtr msgRight);

      /*
      * @brief callback function for the RGBD camera topic.
      */
      void rgbdCallback(const sensor_msgs::msg::Image::SharedPtr msgLeft, const sensor_msgs::msg::Image::SharedPtr msgDepth);

  private:
      ORB_SLAM3::System *mpSLAM;
      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_mono, sub_stereo, sub_rgbd;
      rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_stereo_compressed;
      cv_bridge::CvImagePtr cv_ptr;
      std::string str_vocabulary_file, str_settings_file;
      bool visualization;
  };
}// namespace orb_slam3_ros2

#endif // ORB_SLAM3_ROS_2__MONO_ORB_SLAM_NODE