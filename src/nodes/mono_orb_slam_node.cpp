#include "nodes/mono_orb_slam_node.hpp"
// using namespace std;

namespace orb_slam3_ros2
{
  MonocularSlamNode::MonocularSlamNode() 
  : Node("mono_orb_slam_node")
  { 
    declare_parameter("vocabulary_file", "/home/phoenix/ros2_ws/src/open-source/ORB_SLAM3/Vocabulary/ORBvoc.txt");
    declare_parameter("settings_file", "/home/phoenix/ros2_ws/src/orb_slam3_ros_2/params/ZED2i.yaml");
    declare_parameter("visualization", false);

    get_parameter("vocabulary_file", str_vocabulary_file);
    get_parameter("settings_file", str_settings_file);
    get_parameter("visualization", visualization);
    
    mpSLAM = new ORB_SLAM3::System(str_vocabulary_file, str_settings_file, ORB_SLAM3::System::MONOCULAR, visualization);

    sub_mono = this->create_subscription<sensor_msgs::msg::Image>
                        ("/camera/image_raw", 1, std::bind(&MonocularSlamNode::monoCallback, this, std::placeholders::_1));

    // sub_stereo_compressed = this->create_subscription<sensor_msgs::msg::CompressedImage>
    //                     ("/camera/image_raw/compressed", 1, std::bind(&MonocularSlamNode::stereoCompressedCallback, this, std::placeholders::_1, std::placeholders::_2));
                      
    // sub_stereo = this->create_subscription<sensor_msgs::msg::Image>
    //                     ("/camera/stereo/image_raw", 1, std::bind(&MonocularSlamNode::stereoCallback, this, std::placeholders::_1, std::placeholders::_2));
  };

  MonocularSlamNode::~MonocularSlamNode()
  {
    // Stop all threads
    mpSLAM->Shutdown();

    // Save camera trajectory
    mpSLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  };

  void MonocularSlamNode::monoCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  { 
    RCLCPP_INFO(rclcpp::get_logger("mono_orb::monoCallback"), "I heard: [%s]", msg->header.frame_id.c_str());
    // Copy the ros image message to cv::Mat.
    cv::Mat im;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        im = cv_ptr->image;
        float imageScale = mpSLAM->GetImageScale();
        int width = im.cols * imageScale;
        int height = im.rows * imageScale;
        cv::resize(im, im, cv::Size(width, height));
        // cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("mono_orb::monoCallback"), "cv_bridge exception: %s", e.what());
        // return;
    }
    double seconds = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    mpSLAM->TrackMonocular(im, seconds);
  }


  void MonocularSlamNode::stereoCallback(const sensor_msgs::msg::Image::SharedPtr msgLeft, const sensor_msgs::msg::Image::SharedPtr msgRight)
  {
    RCLCPP_INFO(rclcpp::get_logger("mono_orb::stereoCallback"), "I heard: [%s]", msgLeft->header.frame_id.c_str());
    // Copy the ros image message to cv::Mat.
    cv::Mat imLeft, imRight;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msgLeft, "bgr8");
        imLeft = cv_ptr->image;
        cv_ptr = cv_bridge::toCvCopy(msgRight, "bgr8");
        imRight = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("mono_orb::stereoCallback"), "cv_bridge exception: %s", e.what());
        // return;
    }
    double seconds = static_cast<double>(msgLeft->header.stamp.sec) + static_cast<double>(msgLeft->header.stamp.nanosec) * 1e-9;
    mpSLAM->TrackStereo(imLeft, imRight, seconds);
  }
  void MonocularSlamNode::stereoCompressedCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msgLeft, const sensor_msgs::msg::CompressedImage::SharedPtr msgRight)
  {
    RCLCPP_INFO(rclcpp::get_logger("mono_orb::stereoCompressedCallback"), "I heard: [%s]", msgLeft->header.frame_id.c_str());
    // Copy the ros image message to cv::Mat.
    cv::Mat imLeft, imRight;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msgLeft, "bgr8");
        imLeft = cv_ptr->image;
        cv_ptr = cv_bridge::toCvCopy(msgRight, "bgr8");
        imRight = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("mono_orb::stereoCompressedCallback"), "cv_bridge exception: %s", e.what());
        // return;
    }
    double seconds = static_cast<double>(msgLeft->header.stamp.sec) + static_cast<double>(msgLeft->header.stamp.nanosec) * 1e-9;
    mpSLAM->TrackStereo(imLeft, imRight, seconds);
  }
} 

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  rclcpp::spin(std::make_shared<orb_slam3_ros2::MonocularSlamNode>());

  
  rclcpp::shutdown();

  return 0;
}