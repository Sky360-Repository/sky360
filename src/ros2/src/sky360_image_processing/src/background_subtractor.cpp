#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <vision_msgs/msg/bounding_box2_d_array.hpp>

#include <sky360lib/api/bgs/bgs.hpp>
#include <sky360lib/api/bgs/WeightedMovingVariance/WeightedMovingVarianceUtils.hpp>
#include <sky360lib/api/blobs/connectedBlobDetection.hpp>
#include <sky360lib/api/utils/profiler.hpp>

#include "parameter_node.hpp"

class BackgroundSubtractor
    : public ParameterNode
{
public:
    BackgroundSubtractor()
        : ParameterNode("background_subtractor_node")
    {
        // Define the QoS profile for the subscriber
        rclcpp::QoS sub_qos_profile(2);
        sub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile); // TransientLocal);
        sub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

        // Define the QoS profile for the publisher
        rclcpp::QoS pub_qos_profile(2);
        pub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

        image_subscription_ = create_subscription<sensor_msgs::msg::Image>("sky360/camera/all_sky/bayer", sub_qos_profile,
                                                                                  std::bind(&BackgroundSubtractor::imageCallback, this, std::placeholders::_1));

        image_publisher_ = create_publisher<sensor_msgs::msg::Image>("sky360/frames/all_sky/foreground_mask", pub_qos_profile);
        detection_publisher_ = create_publisher<vision_msgs::msg::BoundingBox2DArray>("sky360/detector/all_sky/bounding_boxes", pub_qos_profile);

        bgsPtr = createBGS(WMV);

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

            msg->encoding = msg->encoding != sensor_msgs::image_encodings::BGR8 ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8;
            auto bayer_img_bridge = cv_bridge::toCvShare(msg);
            if (!bayer_img_bridge->image.empty())
            {
                cv::Mat gray_img{bayer_img_bridge->image};
                if (bayer_img_bridge->image.channels() > 1)
                {
                    cv::cvtColor(bayer_img_bridge->image, gray_img, cv::COLOR_BGR2GRAY);
                }

                if (enable_profiling_)
                {
                    profiler_.start("BGS");
                }
                cv::Mat mask;
                bgsPtr->apply(gray_img, mask);

                auto image_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, mask).toImageMsg();
                image_publisher_->publish(*image_msg);
                if (enable_profiling_)
                {
                    profiler_.stop("BGS");
                }

                if (enable_profiling_)
                {
                    profiler_.start("Blob");
                }
                std::vector<cv::Rect> bboxes;
                if (blob_detector_.detect(mask, bboxes))
                {
                    vision_msgs::msg::BoundingBox2DArray bbox2D_array;
                    bbox2D_array.header = msg->header;
                    add_bboxes(bbox2D_array, bboxes);

                    detection_publisher_->publish(bbox2D_array);
                }
                if (enable_profiling_)
                {
                    profiler_.stop("Blob");
                }
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

    void add_bboxes(vision_msgs::msg::BoundingBox2DArray &bbox2D_array, const std::vector<cv::Rect> &bboxes)
    {
        for (const auto &bbox : bboxes)
        {
            vision_msgs::msg::BoundingBox2D bbox2D;
            bbox2D.center.position.x = bbox.x + bbox.width / 2.0;
            bbox2D.center.position.y = bbox.y + bbox.height / 2.0;
            bbox2D.size_x = bbox.width;
            bbox2D.size_y = bbox.height;
            bbox2D_array.boxes.push_back(bbox2D);
        }
    }

    enum BGSType
    {
        Vibe,
        WMV
    };

    std::unique_ptr<sky360lib::bgs::CoreBgs> createBGS(BGSType _type)
    {
        switch (_type)
        {
        case BGSType::Vibe:
            return std::make_unique<sky360lib::bgs::Vibe>(sky360lib::bgs::VibeParams(50, 20, 2, 4));
        case BGSType::WMV:
            return std::make_unique<sky360lib::bgs::WeightedMovingVariance>(sky360lib::bgs::WMVParams(true, true, 25.0f, 0.5f, 0.3f, 0.2f));
        default:
            return nullptr;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<vision_msgs::msg::BoundingBox2DArray>::SharedPtr detection_publisher_;

    std::unique_ptr<sky360lib::bgs::CoreBgs> bgsPtr{nullptr};
    sky360lib::blobs::ConnectedBlobDetection blob_detector_{sky360lib::blobs::ConnectedBlobDetectionParams(7, 49, 40, 100)};
    sky360lib::utils::Profiler profiler_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BackgroundSubtractor>());
    rclcpp::shutdown();
    return 0;
}
