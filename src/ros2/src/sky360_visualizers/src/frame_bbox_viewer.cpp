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
#include "image_utils.hpp"

class FrameBBoxViewer
    : public ParameterNode
{
public:
    static std::shared_ptr<FrameBBoxViewer> create()
    {
        auto result = std::shared_ptr<FrameBBoxViewer>(new FrameBBoxViewer());
        result->init();
        return result;
    }

private:
    rclcpp::QoS sub_qos_profile_{2};
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_image_;
    message_filters::Subscriber<vision_msgs::msg::BoundingBox2DArray> sub_bbox_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, vision_msgs::msg::BoundingBox2DArray>> time_synchronizer_;

    sky360lib::utils::Profiler profiler_;
    std::vector<std::string> topics_;
    int current_topic_;

    friend std::shared_ptr<FrameBBoxViewer> std::make_shared<FrameBBoxViewer>();

    FrameBBoxViewer()
        : ParameterNode("frame_bbox_viewer_node"), current_topic_{0}
    {
        declare_node_parameters();
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

    void set_parameters_callback(const std::vector<rclcpp::Parameter> &params) override
    {
        for (auto &param : params)
        {
            if (param.get_name() == "topics")
            {
                topics_ = param.as_string_array();
            }
        }
    }

    void declare_node_parameters()
    {
        declare_parameter<std::vector<std::string>>("topics", {"sky360/camera/all_sky/bayer", "sky360/frames/all_sky/foreground_mask"});
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr &image_msg, const vision_msgs::msg::BoundingBox2DArray::SharedPtr &bbox_msg)
    {
        try
        {
            cv::Mat debayered_img;
            ImageUtils::convert_image_msg(image_msg, debayered_img);

            for (const auto &bbox2D : bbox_msg->boxes)
            {
                auto bbox = cv::Rect(bbox2D.center.position.x - bbox2D.size_x / 2, bbox2D.center.position.y - bbox2D.size_y / 2, bbox2D.size_x, bbox2D.size_y);
                cv::rectangle(debayered_img, bbox, cv::Scalar(255, 0, 255), 5, 1);
            }

            cv::imshow("Image Viewer", debayered_img);
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
    rclcpp::spin(FrameBBoxViewer::create());
    rclcpp::shutdown();
    return 0;
}
