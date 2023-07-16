#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>
#include "sky360_interfaces/msg/tracking_state.hpp"
#include "sky360_interfaces/msg/track_detection_array.hpp"
#include "sky360_interfaces/msg/track_trajectory_array.hpp"

#include <sky360lib/api/utils/profiler.hpp>

#include "annotated_frame_creator.hpp"

#include "parameter_node.hpp"

class AnnotatedFrameProvider 
    : public ParameterNode
{
public:
    static std::shared_ptr<AnnotatedFrameProvider> Create()
    {
        auto result = std::shared_ptr<AnnotatedFrameProvider>(new AnnotatedFrameProvider());
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
    }

private:
    AnnotatedFrameProvider() 
        : ParameterNode("annotated_frame_provider_node")
        , annotated_frame_creator_(std::map<std::string, std::string>())
    {
        declare_parameters();
    }

    void init()
    {
        pub_annotated_frame_ = create_publisher<sensor_msgs::msg::Image>("sky360/frames/annotated", rclcpp::QoS(10));

        sub_masked_frame = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this->shared_from_this(), "sky360/frames/all_sky/masked");
        sub_tracking_state = std::make_shared<message_filters::Subscriber<sky360_interfaces::msg::TrackingState>>(this->shared_from_this(), "sky360/tracker/tracking_state");
        sub_tracker_detections = std::make_shared<message_filters::Subscriber<sky360_interfaces::msg::TrackDetectionArray>>(this->shared_from_this(), "sky360/tracker/detections");
        sub_tracker_trajectory = std::make_shared<message_filters::Subscriber<sky360_interfaces::msg::TrackTrajectoryArray>>(this->shared_from_this(), "sky360/tracker/trajectory");
        sub_tracker_prediction = std::make_shared<message_filters::Subscriber<sky360_interfaces::msg::TrackTrajectoryArray>>(this->shared_from_this(), "sky360/tracker/prediction");

        time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sky360_interfaces::msg::TrackingState, sky360_interfaces::msg::TrackDetectionArray, sky360_interfaces::msg::TrackTrajectoryArray, sky360_interfaces::msg::TrackTrajectoryArray>>(
            *sub_masked_frame, *sub_tracking_state, *sub_tracker_detections, *sub_tracker_trajectory, *sub_tracker_prediction, 10);
        time_synchronizer_->registerCallback(&AnnotatedFrameProvider::callback, this);
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr& masked_image_msg
        , const sky360_interfaces::msg::TrackingState::SharedPtr& tracking_state_msg
        , const sky360_interfaces::msg::TrackDetectionArray::SharedPtr& detections_msg
        , const sky360_interfaces::msg::TrackTrajectoryArray::SharedPtr& trajectory_msg
        , const sky360_interfaces::msg::TrackTrajectoryArray::SharedPtr& prediction_msg)
    {
        try
        {
            if (enable_profiling_)
            {
                profiler_.start("Frame");
            }

            auto masked_img_bridge = cv_bridge::toCvShare(masked_image_msg);

            auto annotated_frame = annotated_frame_creator_.create_frame(masked_img_bridge->image, *tracking_state_msg, *detections_msg, *trajectory_msg, *prediction_msg);

            auto annotated_frame_msg = cv_bridge::CvImage(masked_image_msg->header, sensor_msgs::image_encodings::BGR8, annotated_frame).toImageMsg();
            pub_annotated_frame_->publish(*annotated_frame_msg);

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

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_masked_frame;
    std::shared_ptr<message_filters::Subscriber<sky360_interfaces::msg::TrackingState>> sub_tracking_state;
    std::shared_ptr<message_filters::Subscriber<sky360_interfaces::msg::TrackDetectionArray>> sub_tracker_detections;
    std::shared_ptr<message_filters::Subscriber<sky360_interfaces::msg::TrackTrajectoryArray>> sub_tracker_trajectory;
    std::shared_ptr<message_filters::Subscriber<sky360_interfaces::msg::TrackTrajectoryArray>> sub_tracker_prediction;

    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sky360_interfaces::msg::TrackingState, sky360_interfaces::msg::TrackDetectionArray, sky360_interfaces::msg::TrackTrajectoryArray, sky360_interfaces::msg::TrackTrajectoryArray>> time_synchronizer_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_frame_;

    AnnotatedFrameCreator annotated_frame_creator_;
    sky360lib::utils::Profiler profiler_;

    friend std::shared_ptr<AnnotatedFrameProvider> std::make_shared<AnnotatedFrameProvider>();
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto annotatedFrameProvider = AnnotatedFrameProvider::Create();
    rclcpp::spin(annotatedFrameProvider);
    rclcpp::shutdown();
    return 0;
}
