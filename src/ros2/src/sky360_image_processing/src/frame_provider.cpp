#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

#include "sky360_camera/msg/image_info.hpp"
#include "sky360_camera/msg/camera_info.hpp"

#include <sky360lib/api/utils/profiler.hpp>

#include "parameter_node.hpp"
#include "image_utils.hpp"

class FrameProvider 
    : public ParameterNode
{
public:
    static std::shared_ptr<FrameProvider> create()
    {
        auto result = std::shared_ptr<FrameProvider>(new FrameProvider());
        result->init();
        return result;
    }
    
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr masked_publisher_;

    FrameProvider() 
        : ParameterNode("frame_provider_node")
    {
    }

    void init()
    {
        // Define the QoS profile for the subscriber
        rclcpp::QoS sub_qos_profile(2);
        sub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

        // Define the QoS profile for the publisher
        rclcpp::QoS pub_qos_profile(2);
        pub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

        image_subscription_ = create_subscription<sensor_msgs::msg::Image>("sky360/camera/all_sky/bayer", sub_qos_profile,
            std::bind(&FrameProvider::imageCallback, this, std::placeholders::_1));

        masked_publisher_ = create_publisher<sensor_msgs::msg::Image>("sky360/frames/all_sky/masked", pub_qos_profile);
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            auto pattern = msg->encoding;
            msg->encoding = msg->encoding != sensor_msgs::image_encodings::BGR8 ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8;

            auto bayer_img_bridge = cv_bridge::toCvShare(msg);
            if (!bayer_img_bridge->image.empty())
            {
                cv::Mat debayered_img;
                ImageUtils::debayer_image(bayer_img_bridge->image, debayered_img, pattern);

                cv::Mat the_img;
                if (false) // Resize frame
                {
                    uint32_t frame_width = debayered_img.size().width;
                    uint32_t frame_height = debayered_img.size().height;
                    double aspect_ratio = (double)debayered_img.size().width / (double)debayered_img.size().height;
                    frame_height = 1280;
                    frame_width = ((uint32_t)(aspect_ratio * (double)frame_height)) & 0xFFFFFFFE;
                    cv::resize(debayered_img, the_img, cv::Size(frame_width, frame_height));
                }
                else
                {
                    the_img = debayered_img;
                }

                auto the_image_msg = cv_bridge::CvImage(msg->header, the_img.channels() == 1 ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8, the_img).toImageMsg();
                masked_publisher_->publish(*the_image_msg);
            }
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(FrameProvider::create());
    rclcpp::shutdown();
    return 0;
}
