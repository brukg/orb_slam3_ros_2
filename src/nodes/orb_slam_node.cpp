#include "nodes/orb_slam_node.hpp"
// using namespace std;

namespace orb_slam3_ros2
{
  ORBSlamNode::ORBSlamNode() 
  : Node("mono_orb_slam_node")
  { 
    declare_parameter("vocabulary_file", "~/params/vocabulary/ORBvoc.txt");
    declare_parameter("settings_file", "~/params/ZED2i.yaml");
    declare_parameter("visualization", false);

    get_parameter("vocabulary_file", str_vocabulary_file);
    get_parameter("settings_file", str_settings_file);
    get_parameter("visualization", visualization);
    
    mpSLAM = new ORB_SLAM3::System(str_vocabulary_file, str_settings_file, ORB_SLAM3::System::MONOCULAR, visualization);

    sub_mono = this->create_subscription<sensor_msgs::msg::Image>
                        ("/camera/image_raw", 1, std::bind(&ORBSlamNode::monoCallback, this, std::placeholders::_1));

    // sub_stereo_compressed = this->create_subscription<sensor_msgs::msg::CompressedImage>
    //                     ("/camera/image_raw/compressed", 1, std::bind(&ORBSlamNode::stereoCompressedCallback, this, std::placeholders::_1, std::placeholders::_2));
                      
    image_l_sub_.subscribe(this, "left/image_rect_gray/compressed");

    image_r_sub_.subscribe(this, "right/image_rect_gray/compressed");

    sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage>>(image_l_sub_, image_r_sub_, 5);
        
    sync_->registerCallback(std::bind(&ORBSlamNode::stereoCompressedCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    // sub_stereo = this->create_subscription<sensor_msgs::msg::Image>
    //                     ("/camera/stereo/image_raw", 1, std::bind(&ORBSlamNode::stereoCallback, this, std::placeholders::_1, std::placeholders::_2));

                
    // sub_rgbd = this->create_subscription<sensor_msgs::msg::Image>
    //                     ("/camera/depth/image_raw", 1, std::bind(&ORBSlamNode::rgbdCallback, this, std::placeholders::_1, std::placeholders::_2));

    // sub_imu = this->create_subscription<sensor_msgs::msg::Imu>
    //                     ("/imu", 1, std::bind(&ORBSlamNode::imuCallback, this, std::placeholders::_1));
    
    };

  ORBSlamNode::~ORBSlamNode()
  {
    // Stop all threads
    mpSLAM->Shutdown();

    // Save camera trajectory
    mpSLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  };

  void ORBSlamNode::monoCallback(const sensor_msgs::msg::Image::SharedPtr msg)
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


  void ORBSlamNode::stereoCallback(const sensor_msgs::msg::Image::SharedPtr msgLeft, const sensor_msgs::msg::Image::SharedPtr msgRight)
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

  void ORBSlamNode::stereoCompressedCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msgLeft, 
                                            const sensor_msgs::msg::CompressedImage::ConstSharedPtr msgRight)
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
  
  rclcpp::spin(std::make_shared<orb_slam3_ros2::ORBSlamNode>());

  
  rclcpp::shutdown();

  return 0;
}