#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

#include "sky360_camera/msg/image_info.hpp"
#include "sky360_camera/msg/camera_info.hpp"

#include <sky360lib/api/utils/profiler.hpp>
#include <sky360lib/api/utils/ringbuf.h>

#include "parameter_node.hpp"
#include "image_utils.hpp"

class FrameSaver 
    : public ParameterNode
{
public:
    static std::shared_ptr<FrameSaver> create()
    {
        auto result = std::shared_ptr<FrameSaver>(new FrameSaver());
        result->init();
        return result;
    }
    
private:
    static const unsigned long NUMBER_OF_IMAGES_TO_STORE = 20 * 5;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    baudvine::RingBuf<cv::Mat, NUMBER_OF_IMAGES_TO_STORE> image_buffer_;

    FrameSaver() 
        : ParameterNode("frame_saver_node")
    {
    }

    void init()
    {
        // Define the QoS profile for the subscriber
        rclcpp::QoS sub_qos_profile(2);
        sub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

        image_subscription_ = create_subscription<sensor_msgs::msg::Image>("sky360/camera/all_sky/bayer", sub_qos_profile,
            std::bind(&FrameSaver::imageCallback, this, std::placeholders::_1));
    }

    void set_parameters_callback(const std::vector<rclcpp::Parameter> &params) override
    {
        (void)params;
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr image_msg)
    {
        try
        {
            cv::Mat debayered_img;
            ImageUtils::convert_image_msg(image_msg, debayered_img);

            image_buffer_.push_back(debayered_img);
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
    rclcpp::spin(FrameSaver::create());
    rclcpp::shutdown();
    return 0;
}
