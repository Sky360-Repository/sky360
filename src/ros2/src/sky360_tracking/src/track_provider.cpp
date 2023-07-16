#include <opencv2/opencv.hpp>

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/bounding_box2_d_array.hpp>
#include "sky360_interfaces/msg/track_detection_array.hpp"
#include "sky360_interfaces/msg/tracking_state.hpp"
#include "sky360_interfaces/msg/track_trajectory_array.hpp"

#include "sky360lib/api/utils/profiler.hpp"
#include "video_tracker.hpp"

#include "parameter_node.hpp"

class TrackProvider
    : public ParameterNode
{
public:
    static std::shared_ptr<TrackProvider> Create()
    {
        auto result = std::shared_ptr<TrackProvider>(new TrackProvider());
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
    TrackProvider()
        : ParameterNode("frame_provider_node"), video_tracker_(std::map<std::string, std::string>(), get_logger())
    {
        declare_parameters();
    }

    void init()
    {
        masked_frame_subscription_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this->shared_from_this(), "sky360/frames/all_sky/masked");
        detector_bounding_boxes_subscription_ = std::make_shared<message_filters::Subscriber<vision_msgs::msg::BoundingBox2DArray>>(this->shared_from_this(), "sky360/detector/all_sky/bounding_boxes");

        time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, vision_msgs::msg::BoundingBox2DArray>>(*masked_frame_subscription_, *detector_bounding_boxes_subscription_, 10);
        time_synchronizer_->registerCallback(&TrackProvider::callback, this);

        pub_tracker_tracking_state = create_publisher<sky360_interfaces::msg::TrackingState>("sky360/tracker/tracking_state", 10);
        pub_tracker_detects = create_publisher<sky360_interfaces::msg::TrackDetectionArray>("sky360/tracker/detections", 10);
        pub_tracker_trajectory = create_publisher<sky360_interfaces::msg::TrackTrajectoryArray>("sky360/tracker/trajectory", 10);
        pub_tracker_prediction = create_publisher<sky360_interfaces::msg::TrackTrajectoryArray>("sky360/tracker/prediction", 10);
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr &image_msg, const vision_msgs::msg::BoundingBox2DArray::SharedPtr &bounding_boxes_msg)
    {
        try
        {
            if (enable_profiling_)
            {
                profiler_.start("Frame");
            }

            cv_bridge::CvImagePtr masked_img_bridge = cv_bridge::toCvCopy(image_msg, image_msg->encoding);

            std::vector<cv::Rect> bboxes;
            for (const auto &bbox2D : bounding_boxes_msg->boxes)
            {
                bboxes.push_back(cv::Rect(bbox2D.center.position.x - bbox2D.size_x / 2, bbox2D.center.position.y - bbox2D.size_y / 2, bbox2D.size_x, bbox2D.size_y));
            }

            video_tracker_.update_trackers(bboxes, masked_img_bridge->image);

            publish_detect_array(image_msg->header);
            publish_trajectory_array(image_msg->header);
            publish_prediction_array(image_msg->header);
            publish_tracking_state(image_msg->header);

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

    void publish_detect_array(std_msgs::msg::Header &header)
    {
        sky360_interfaces::msg::TrackDetectionArray detection_array_msg;
        detection_array_msg.header = header;
        for (const auto &tracker : video_tracker_.get_live_trackers())
        {
            add_detects_to_msg(tracker, detection_array_msg);
        }
        pub_tracker_detects->publish(detection_array_msg);
    }

    void publish_trajectory_array(std_msgs::msg::Header &header)
    {
        sky360_interfaces::msg::TrackTrajectoryArray trajectory_array_msg;
        trajectory_array_msg.header = header;
        for (const auto &tracker : video_tracker_.get_live_trackers())
        {
            add_trajectories_to_msg(tracker, trajectory_array_msg);
        }
        pub_tracker_trajectory->publish(trajectory_array_msg);
    }

    void publish_prediction_array(std_msgs::msg::Header &header)
    {
        sky360_interfaces::msg::TrackTrajectoryArray prediction_array_msg;
        prediction_array_msg.header = header;
        for (const auto &tracker : video_tracker_.get_live_trackers())
        {
            add_predictions_to_msg(tracker, prediction_array_msg);
        }
        pub_tracker_prediction->publish(prediction_array_msg);
    }

    void publish_tracking_state(std_msgs::msg::Header &header)
    {
        sky360_interfaces::msg::TrackingState tracking_state_msg;
        tracking_state_msg.header = header;
        tracking_state_msg.trackable = video_tracker_.get_total_trackable_trackers();
        tracking_state_msg.alive = video_tracker_.get_total_live_trackers();
        tracking_state_msg.started = video_tracker_.get_total_trackers_started();
        tracking_state_msg.ended = video_tracker_.get_total_trackers_finished();
        pub_tracker_tracking_state->publish(tracking_state_msg);
    }

    void add_detects_to_msg(const Tracker &tracker, sky360_interfaces::msg::TrackDetectionArray &detection_2d_array_msg)
    {
        auto bbox = tracker.get_bbox();
        vision_msgs::msg::BoundingBox2D bbox_msg;
        bbox_msg.center.position.x = bbox.x + bbox.width / 2;
        bbox_msg.center.position.y = bbox.y + bbox.height / 2;
        bbox_msg.size_x = bbox.width;
        bbox_msg.size_y = bbox.height;

        sky360_interfaces::msg::TrackDetection detect_msg;
        detect_msg.id = tracker.get_id();
        detect_msg.state = (int)tracker.get_tracking_state();
        detect_msg.bbox = bbox_msg;

        detection_2d_array_msg.detections.push_back(detect_msg);
    }

    void add_trajectories_to_msg(const Tracker &tracker, sky360_interfaces::msg::TrackTrajectoryArray &trajectory_array_msg)
    {
        sky360_interfaces::msg::TrackTrajectory track_msg;
        track_msg.id = std::to_string(tracker.get_id()) + std::string("-") + std::to_string(tracker.get_tracking_state());

        for (const auto &center_point : tracker.get_center_points())
        {
            sky360_interfaces::msg::TrackPoint point;
            point.center.x = center_point.first.x;
            point.center.y = center_point.first.y;
            point.tracking_state = (int)center_point.second;
            track_msg.trajectory.push_back(point);
        }

        trajectory_array_msg.trajectories.push_back(track_msg);
    }

    void add_predictions_to_msg(const Tracker &tracker, sky360_interfaces::msg::TrackTrajectoryArray &prediction_array_msg)
    {
        sky360_interfaces::msg::TrackTrajectory track_msg;
        track_msg.id = std::to_string(tracker.get_id()) + std::string("-") + std::to_string(tracker.get_tracking_state());

        for (const auto &center_point : tracker.get_predictor_center_points())
        {
            sky360_interfaces::msg::TrackPoint point;
            point.center.x = center_point.x;
            point.center.y = center_point.y;
            track_msg.trajectory.push_back(point);
        }

        prediction_array_msg.trajectories.push_back(track_msg);
    }

    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> masked_frame_subscription_;
    std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::BoundingBox2DArray>> detector_bounding_boxes_subscription_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, vision_msgs::msg::BoundingBox2DArray>> time_synchronizer_;

    rclcpp::Publisher<sky360_interfaces::msg::TrackingState>::SharedPtr pub_tracker_tracking_state;
    rclcpp::Publisher<sky360_interfaces::msg::TrackDetectionArray>::SharedPtr pub_tracker_detects;
    rclcpp::Publisher<sky360_interfaces::msg::TrackTrajectoryArray>::SharedPtr pub_tracker_trajectory;
    rclcpp::Publisher<sky360_interfaces::msg::TrackTrajectoryArray>::SharedPtr pub_tracker_prediction;

    sky360lib::utils::Profiler profiler_;
    VideoTracker video_tracker_;

    friend std::shared_ptr<TrackProvider> std::make_shared<TrackProvider>();
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto trackProvider = TrackProvider::Create();
    rclcpp::spin(trackProvider);
    rclcpp::shutdown();
    return 0;
}
