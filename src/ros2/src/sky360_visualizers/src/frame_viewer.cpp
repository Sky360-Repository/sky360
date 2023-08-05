#include <vector>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>

#include <sky360lib/api/utils/profiler.hpp>

#include "annotated_frame_creator.hpp"

#include "parameter_node.hpp"
#include "image_utils.hpp"

class FrameViewer
    : public ParameterNode
{
public:
    static std::shared_ptr<FrameViewer> create()
    {
        auto result = std::shared_ptr<FrameViewer>(new FrameViewer());
        result->init();
        return result;
    }

private:
    rclcpp::QoS sub_qos_profile_{2};
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    sky360lib::utils::Profiler profiler_;
    std::vector<std::string> topics_;
    int current_topic_;

    friend std::shared_ptr<FrameViewer> std::make_shared<FrameViewer>();

    FrameViewer() 
        : ParameterNode("frame_viewer_node")
        , current_topic_{0}
    {
        declare_node_parameters();
    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("topics", std::vector<std::string>({"sky360/camera/all_sky/bayer", "sky360/frames/all_sky/foreground_mask"})), 
                [this](const rclcpp::Parameter& param) {topics_ = param.as_string_array();}
            ),
        };
        add_action_parameters(params);
    }
    
    void init()
    {
        sub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        image_subscription_ = create_subscription<sensor_msgs::msg::Image>(topics_[current_topic_], sub_qos_profile_,
            std::bind(&FrameViewer::imageCallback, this, std::placeholders::_1));

        cv::namedWindow("Image Viewer", cv::WINDOW_NORMAL);
        cv::displayStatusBar("Image Viewer", topics_[current_topic_], 0);
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr image_msg)
    {
        try
        {
            cv::Mat debayered_img;
            ImageUtils::convert_image_msg(image_msg, debayered_img);

            cv::imshow("Image Viewer", debayered_img);
            int key = cv::waitKey(1);
            bool topic_change = false;
            switch (key)
            {
                case 81: current_topic_--; topic_change = true; break;
                case 83: current_topic_++; topic_change = true; break;
            }
            if (topic_change)
            {
                current_topic_ = current_topic_ < 0 ? topics_.size() - 1 : (current_topic_ >= (int)topics_.size() ? 0 : current_topic_);
                RCLCPP_INFO(get_logger(), "Changing topic to %s", topics_[current_topic_].c_str());
                image_subscription_ = create_subscription<sensor_msgs::msg::Image>(topics_[current_topic_], sub_qos_profile_,
                    std::bind(&FrameViewer::imageCallback, this, std::placeholders::_1));
                cv::displayStatusBar("Image Viewer", topics_[current_topic_], 0);
            }
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "CV bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(FrameViewer::create());
    rclcpp::shutdown();
    return 0;
}
