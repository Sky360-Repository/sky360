#include <vector>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/bounding_box2_d_array.hpp>

#include <sky360lib/api/utils/profiler.hpp>

#include "annotated_frame_creator.hpp"

#include "parameter_node.hpp"

class FrameBBoxViewer
    : public ParameterNode
{
public:
    static std::shared_ptr<FrameBBoxViewer> Create()
    {
        auto result = std::shared_ptr<FrameBBoxViewer>(new FrameBBoxViewer());
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
    FrameBBoxViewer()
        : ParameterNode("frame_bbox_viewer_node"), current_topic_{0}
    {
        declare_parameters();
    }

    void init()
    {
        sub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);
        auto rmw_qos_profile = sub_qos_profile_.get_rmw_qos_profile();

        sub_image_.subscribe(this, topics_[current_topic_], rmw_qos_profile);
        sub_bbox_.subscribe(this, "sky360/detector/all_sky/bounding_boxes", rmw_qos_profile);

        time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, vision_msgs::msg::BoundingBox2DArray>>(sub_image_, sub_bbox_, 2);
        time_synchronizer_->registerCallback(&FrameBBoxViewer::imageCallback, this);

        cv::namedWindow("Image Viewer", cv::WINDOW_NORMAL);
        cv::displayStatusBar("Image Viewer", topics_[current_topic_], 0);
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr &image_msg, const vision_msgs::msg::BoundingBox2DArray::SharedPtr &bbox_msg)
    {
        try
        {
            if (enable_profiling_)
            {
                profiler_.start("Frame");
            }

            cv::Mat frame = cv_bridge::toCvShare(image_msg, image_msg->encoding)->image;

            for (const auto &bbox2D : bbox_msg->boxes)
            {
                auto bbox = cv::Rect(bbox2D.center.position.x - bbox2D.size_x / 2, bbox2D.center.position.y - bbox2D.size_y / 2, bbox2D.size_x, bbox2D.size_y);
                cv::rectangle(frame, bbox, cv::Scalar(255, 0, 255), 5, 1);
            }

            cv::imshow("Image Viewer", frame);
            int key = cv::waitKey(1);
            bool topic_change = false;
            switch (key)
            {
            case 81:
                current_topic_--;
                topic_change = true;
                break;
            case 83:
                current_topic_++;
                topic_change = true;
                break;
            }
            if (topic_change)
            {
                current_topic_ = current_topic_ < 0 ? topics_.size() - 1 : (current_topic_ >= (int)topics_.size() ? 0 : current_topic_);
                RCLCPP_INFO(get_logger(), "Changing topic to %s", topics_[current_topic_].c_str());

                sub_image_.unsubscribe();
                sub_image_.subscribe(this, topics_[current_topic_], sub_qos_profile_.get_rmw_qos_profile());

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
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_image_;
    message_filters::Subscriber<vision_msgs::msg::BoundingBox2DArray> sub_bbox_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, vision_msgs::msg::BoundingBox2DArray>> time_synchronizer_;

    sky360lib::utils::Profiler profiler_;
    std::vector<std::string> topics_;
    int current_topic_;

    friend std::shared_ptr<FrameBBoxViewer> std::make_shared<FrameBBoxViewer>();
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto annotatedFrameProvider = FrameBBoxViewer::Create();
    rclcpp::spin(annotatedFrameProvider);
    rclcpp::shutdown();
    return 0;
}
