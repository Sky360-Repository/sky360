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

class FrameViewer
    : public ParameterNode
{
public:
    static std::shared_ptr<FrameViewer> Create()
    {
        auto result = std::shared_ptr<FrameViewer>(new FrameViewer());
        result->init();
        return result;
    }

protected:
    void set_parameters_callback(const std::vector<rclcpp::Parameter> &params) override
    {
        (void)params;
    }

    void declare_parameters() override
    {
        declare_parameter<std::vector<std::string>>("topics", {"sky360/frames/all_sky/masked", "sky360/frames/all_sky/foreground_mask"});

        get_parameter("topics", topics_);
    }

private:
    FrameViewer() 
        : ParameterNode("frame_viewer_node")
        , current_topic_{0}
    {
        declare_parameters();
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

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr masked_image_msg)
    {
        try
        {
            if (enable_profiling_)
            {
                profiler_.start("Frame");
            }

            cv::Mat frame = cv_bridge::toCvShare(masked_image_msg)->image;
            cv::imshow("Image Viewer", frame);
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
            RCLCPP_ERROR(get_logger(), "CV bridge exception: %s", e.what());
        }
    }

    rclcpp::QoS sub_qos_profile_{10};
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    sky360lib::utils::Profiler profiler_;
    std::vector<std::string> topics_;
    int current_topic_;

    friend std::shared_ptr<FrameViewer> std::make_shared<FrameViewer>();
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto annotatedFrameProvider = FrameViewer::Create();
    rclcpp::spin(annotatedFrameProvider);
    rclcpp::shutdown();
    return 0;
}
