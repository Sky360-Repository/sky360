#include <chrono>

#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

#include "sky360_camera/msg/image_info.hpp"
#include "sky360_camera/msg/camera_info.hpp"
#include "sky360_camera/msg/bayer_format.hpp"

#include "parameter_node.hpp"

#include <sky360lib/api/camera/qhy_camera.hpp>
#include "sky360lib/api/utils/profiler.hpp"

class WebCameraPublisher
    : public ParameterNode
{
public:
    WebCameraPublisher()
        : ParameterNode("web_camera_publisher_node")
    {
        rclcpp::QoS qos_profile(2); // The depth of the publisher queue
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

        image_publisher_ = create_publisher<sensor_msgs::msg::Image>("sky360/camera/all_sky/bayer", qos_profile);
        image_info_publisher_ = create_publisher<sky360_camera::msg::ImageInfo>("sky360/camera/all_sky/image_info", qos_profile);
        camera_info_publisher_ = create_publisher<sky360_camera::msg::CameraInfo>("sky360/camera/all_sky/camera_info", qos_profile);

        declare_parameters();
    }

    void start_publishing()
    {
        open_camera();

        create_camera_info_msg();

        cv::Mat image;
        while (rclcpp::ok())
        {
            if (enable_profiling_)
            {
                profiler_.stop("Frame");
            }

            if (!video_capture_.read(image) && is_video_)
            {
                video_capture_.set(cv::CAP_PROP_POS_FRAMES, 0); // if video ends, loop back to start
                video_capture_.read(image);
            }

            if (false)
            {
                double aspect_ratio = (double)image.size().width / (double)image.size().height;
                uint32_t frame_height = 1280;
                uint32_t frame_width = (uint32_t)(aspect_ratio * (double)frame_height);
                cv::resize(image, image, cv::Size(frame_width, frame_height));
            }

            std_msgs::msg::Header header;
            header.stamp = now();
            header.frame_id = boost::uuids::to_string(uuid_generator_());

            auto image_msg = cv_bridge::CvImage(header, image.channels() == 1 ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8, image).toImageMsg();
            image_publisher_->publish(*image_msg);

            auto image_info_msg = generate_image_info(header, image);
            image_info_publisher_->publish(image_info_msg);

            publish_camera_info(header);

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

            rclcpp::spin_some(get_node_base_interface());
        }
    }

protected:
    void set_parameters_callback(const std::vector<rclcpp::Parameter> &params) override
    {
        for (auto &param : params)
        {
            if (param.get_name() == "is_video")
            {
                is_video_ = param.as_bool();
            }
            else if (param.get_name() == "camera_id")
            {
                camera_id_ = param.as_int();
            }
            else if (param.get_name() == "video_path")
            {
                video_path_ = param.as_string();
            }
        }
    }

    void declare_parameters() override
    {
        std::vector<rclcpp::Parameter> params = {
            rclcpp::Parameter("is_video", false),
            rclcpp::Parameter("camera_id", 0),
            rclcpp::Parameter("video_path", ""),
        };
        ParameterNode::declare_parameters(params);
    }

private:
    boost::uuids::random_generator uuid_generator_;
    cv::VideoCapture video_capture_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sky360_camera::msg::ImageInfo>::SharedPtr image_info_publisher_;
    rclcpp::Publisher<sky360_camera::msg::CameraInfo>::SharedPtr camera_info_publisher_;
    sky360_camera::msg::CameraInfo camera_info_msg_;
    sky360lib::utils::Profiler profiler_;
    bool is_video_;
    int camera_id_;
    std::string video_path_;

    inline void open_camera()
    {
        if (!is_video_)
        {
            camera_id_ = get_parameter("camera_id").get_value<rclcpp::ParameterType::PARAMETER_INTEGER>();
            RCLCPP_INFO(get_logger(), "Camera %d opening", camera_id_);
            video_capture_.open(camera_id_);
            setHighestResolution(video_capture_);
        }
        else
        {
            video_path_ = get_parameter("video_path").get_value<rclcpp::ParameterType::PARAMETER_STRING>();
            RCLCPP_INFO(get_logger(), "Video '%s' opening", video_path_.c_str());
            video_capture_.open(video_path_);
        }
    }

    inline bool setHighestResolution(cv::VideoCapture &cap)
    {
        std::vector<std::pair<int, int>> resolutions = {
            //{3840, 2160},  // 4K UHD
            //{2560, 1440}, // QHD, WQHD, 2K
            {1920, 1080}, // Full HD
            {1600, 900},  // HD+
            {1280, 720},  // HD
            {1024, 768},  // XGA
            {800, 600},   // SVGA
            {640, 480},   // VGA
            {320, 240}   // QVGA
        };

        std::vector<std::pair<int, int>> supportedResolutions;
        for (const auto &resolution : resolutions)
        {
            cap.set(cv::CAP_PROP_FRAME_WIDTH, resolution.first);
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, resolution.second);

            if (cap.get(cv::CAP_PROP_FRAME_WIDTH) == resolution.first &&
                cap.get(cv::CAP_PROP_FRAME_HEIGHT) == resolution.second)
            {
                RCLCPP_INFO(get_logger(), "Setting resolution for %d x %d", resolution.first, resolution.second);
                return true;
            }
        }

        return false;
    }

    inline sky360_camera::msg::ImageInfo generate_image_info(std_msgs::msg::Header &header, const cv::Mat& image)
    {
        sky360_camera::msg::ImageInfo image_info_msg;
        image_info_msg.header = header;

        image_info_msg.roi.start_x = 0;
        image_info_msg.roi.start_y = 0;
        image_info_msg.roi.width = image.size().width;
        image_info_msg.roi.height = image.size().height;
        image_info_msg.bpp = image.elemSize1() == 1 ? 8 : 16;
        image_info_msg.bayer_format = image.channels() == 1 ? sky360lib::camera::QhyCamera::BayerFormat::Mono : sky360lib::camera::QhyCamera::BayerFormat::Color;
        image_info_msg.exposure = 0;
        image_info_msg.gain = 0;
        image_info_msg.offset = 0;
        image_info_msg.white_balance.r = 0;
        image_info_msg.white_balance.g = 0;
        image_info_msg.white_balance.b = 0;
        image_info_msg.contrast = 0;
        image_info_msg.brightness = 0;
        image_info_msg.gamma = 1.0;
        image_info_msg.channels = image.channels();
        image_info_msg.bin_mode = 1;
        image_info_msg.current_temp = 0;
        image_info_msg.cool_enabled = false;
        image_info_msg.target_temp = 0;
        image_info_msg.auto_exposure = false;

        return image_info_msg;
    }

    inline void create_camera_info_msg()
    {
        camera_info_msg_.id = "web_camera_node";
        camera_info_msg_.model = is_video_ ? "video" : "web camera";
        camera_info_msg_.serial_num = is_video_ ? video_path_ : std::to_string(camera_id_);
        camera_info_msg_.overscan.start_x = 0;
        camera_info_msg_.overscan.start_y = 0;
        camera_info_msg_.overscan.width = 0;
        camera_info_msg_.overscan.height = 0;
        camera_info_msg_.effective.start_x = 0;
        camera_info_msg_.effective.start_y = 0;
        camera_info_msg_.effective.width = (uint32_t)video_capture_.get(cv::CAP_PROP_FRAME_WIDTH);
        camera_info_msg_.effective.height = (uint32_t)video_capture_.get(cv::CAP_PROP_FRAME_HEIGHT);
        camera_info_msg_.chip.width_mm = 0;
        camera_info_msg_.chip.height_mm = 0;
        camera_info_msg_.chip.pixel_width_um = 0;
        camera_info_msg_.chip.pixel_height_um = 0;
        camera_info_msg_.chip.max_image_width = (uint32_t)video_capture_.get(cv::CAP_PROP_FRAME_WIDTH);
        camera_info_msg_.chip.max_image_height = (uint32_t)video_capture_.get(cv::CAP_PROP_FRAME_HEIGHT);
        camera_info_msg_.chip.max_bpp = 8;
        camera_info_msg_.bayer_format = sky360lib::camera::QhyCamera::BayerFormat::Color;
        camera_info_msg_.is_color = true;
        camera_info_msg_.is_cool = false;
        camera_info_msg_.has_bin1x1_mode = true;
        camera_info_msg_.has_bin2x2_mode = false;
        camera_info_msg_.has_bin3x3_mode = false;
        camera_info_msg_.has_bin4x4_mode = false;
        camera_info_msg_.gain_limits.min = 0;
        camera_info_msg_.gain_limits.max = 0;
        camera_info_msg_.gain_limits.step = 0;
        camera_info_msg_.offset_limits.min = 0;
        camera_info_msg_.offset_limits.max = 0;
        camera_info_msg_.offset_limits.step = 0;
        camera_info_msg_.usb_traffic_limits.min = 0;
        camera_info_msg_.usb_traffic_limits.max = 0;
        camera_info_msg_.usb_traffic_limits.step = 0;
        camera_info_msg_.red_wb_limits.min = 0;
        camera_info_msg_.red_wb_limits.max = 0;
        camera_info_msg_.red_wb_limits.step = 0;
        camera_info_msg_.green_wb_limits.min = 0;
        camera_info_msg_.green_wb_limits.max = 0;
        camera_info_msg_.green_wb_limits.step = 0;
        camera_info_msg_.blue_wb_limits.min = 0;
        camera_info_msg_.blue_wb_limits.max = 0;
        camera_info_msg_.blue_wb_limits.step = 0;
        camera_info_msg_.temperature_limits.min = 0;
        camera_info_msg_.temperature_limits.max = 0;
        camera_info_msg_.temperature_limits.step = 0;
    }

    inline void publish_camera_info(std_msgs::msg::Header &header)
    {
        camera_info_msg_.header = header;

        camera_info_publisher_->publish(camera_info_msg_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto image_publisher = std::make_shared<WebCameraPublisher>();
    image_publisher->start_publishing();
    rclcpp::shutdown();
    return 0;
}
