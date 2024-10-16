#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.hpp>

class CompressImage : public rclcpp::Node
{
public:
  CompressImage()
      : Node("compress_image")
  {

    // TODO: declare parameters for the topic names
    // this->declare_parameter<std::string>("input_topic", "input");
    // this->get_parameter("input_topic", input_topic_);

    sub_raw_img_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", 10, std::bind(&CompressImage::raw_images_cb, this, std::placeholders::_1));

    pub_compressed_img_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/camera/compressed", 10);
  }

private:

  void raw_images_cb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert raw image to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    sensor_msgs::msg::CompressedImage img_msg;
    cv_ptr->toCompressedImageMsg(img_msg);

    pub_compressed_img_->publish(img_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_raw_img_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed_img_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_unique<CompressImage>());
  rclcpp::shutdown();
  return 0;
}
