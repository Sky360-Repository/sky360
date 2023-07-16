#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

#include "sky360_camera/msg/image_info.hpp"
#include "sky360_camera/msg/camera_info.hpp"
#include "sky360_camera/msg/bayer_format.hpp"

#include <sky360lib/api/utils/profiler.hpp>

#include "parameter_node.hpp"

class FrameProvider 
    : public ParameterNode
{
public:
    FrameProvider() 
        : ParameterNode("frame_provider_node")
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

        declare_parameters();
    }

protected:
    void set_parameters_callback(const std::vector<rclcpp::Parameter> &params) override
    {
        (void)params;
    }

    void declare_parameters() override
    {
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            if (enable_profiling_)
            {
                profiler_.start("Frame");
            }

            auto pattern = msg->encoding;
            msg->encoding = msg->encoding != sensor_msgs::image_encodings::BGR8 ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8;

            auto bayer_img_bridge = cv_bridge::toCvShare(msg);
            if (!bayer_img_bridge->image.empty())
            {
                cv::Mat debayered_img;
                debayer_image(bayer_img_bridge->image, debayered_img, pattern);

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

            if (enable_profiling_)
            {
                profiler_.stop("Frame");
                if (profiler_.get_data("Frame").duration_in_seconds() > 1.0)
                {
                    auto report = profiler_.report();
                    RCLCPP_INFO(get_logger(), report.c_str());
                    profiler_.reset();
                }
            }
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        }
    }

    static inline int convert_bayer_pattern(const std::string& _bayerFormat)
    {
        if (_bayerFormat == sky360_camera::msg::BayerFormat::BAYER_GB)
        {
            return cv::COLOR_BayerGR2BGR; //!< equivalent to GBRG Bayer pattern
        }
        else if (_bayerFormat == sky360_camera::msg::BayerFormat::BAYER_GR)
        {
            return cv::COLOR_BayerGB2BGR; //!< equivalent to GRBG Bayer pattern
        }
        else if (_bayerFormat == sky360_camera::msg::BayerFormat::BAYER_BG)
        {
            return cv::COLOR_BayerRG2BGR; //!< equivalent to GRBG Bayer pattern
        }
        else if (_bayerFormat == sky360_camera::msg::BayerFormat::BAYER_RG)
        {
            return cv::COLOR_BayerBG2BGR; //!< equivalent to GRBG Bayer pattern
        }
        return cv::COLOR_BayerGR2BGR;
    }

    void debayer_image(const cv::Mat &_image_in, cv::Mat &_image_out, const std::string& _bayerFormatStr) const
    {
        if (_bayerFormatStr != sensor_msgs::image_encodings::BGR8 
            && _bayerFormatStr != sensor_msgs::image_encodings::MONO8)
        {
            cv::cvtColor(_image_in, _image_out, convert_bayer_pattern(_bayerFormatStr));
        }
        else
        {
            _image_out = _image_in;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr masked_publisher_;

    sky360lib::utils::Profiler profiler_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameProvider>());
    rclcpp::shutdown();
    return 0;
}
