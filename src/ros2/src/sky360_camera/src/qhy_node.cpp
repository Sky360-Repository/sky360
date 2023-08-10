#include <chrono>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "sky360_camera/msg/image_info.hpp"
#include "sky360_camera/msg/camera_info.hpp"

#include <sky360lib/api/camera/qhy_camera.hpp>
#include <sky360lib/api/utils/auto_exposure.hpp>
#include <sky360lib/api/utils/sub_sampler.hpp>
#include <sky360lib/api/utils/brightness_estimator.hpp>
#include <sky360lib/api/utils/profiler.hpp>

#include "parameter_node.hpp"

class QhyNode
    : public ParameterNode
{
public:
    static std::shared_ptr<QhyNode> create()
    {
        auto image_publisher = std::shared_ptr<QhyNode>(new QhyNode());
        image_publisher->init();
        return image_publisher;
    }
    
    void start_publishing()
    {
        cv::Mat image;
        while (rclcpp::ok())
        {
            if (enable_profiling_)
            {
                profiler_.start("Frame");
            }

            auto camera_params = qhy_camera_.get_camera_params();

            qhy_camera_.get_frame(image, false);

            if (auto_exposure_)
            {
                apply_auto_exposure(image, camera_params);
            }

            std_msgs::msg::Header header;
            header.stamp = now();
            header.frame_id = generate_uuid();

            auto image_msg = cv_bridge::CvImage(header, msg_bayer_format_str_, image).toImageMsg();
            auto borrow_msg = image_publisher_->borrow_loaned_message();
            borrow_msg.get() = *image_msg;
            image_publisher_->publish(std::move(borrow_msg));

            auto image_info_msg = generate_image_info(header, camera_params);
            image_info_publisher_->publish(image_info_msg);

            camera_info_msg_.header = header;
            camera_info_publisher_->publish(camera_info_msg_);

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

private:
    rclcpp::QoS qos_profile_{2}; // The depth of the publisher queue
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sky360_camera::msg::ImageInfo>::SharedPtr image_info_publisher_;
    rclcpp::Publisher<sky360_camera::msg::CameraInfo>::SharedPtr camera_info_publisher_;
    sky360_camera::msg::CameraInfo camera_info_msg_;
    sky360lib::camera::QhyCamera qhy_camera_;
    sky360lib::utils::SubSampler subSampler{50, 50};
    sky360lib::utils::BrightnessEstimator brightnessEstimator;
    sky360lib::utils::AutoExposure auto_exposure_control_{0.25, 180, 0.01, 100};
    sky360lib::utils::Profiler profiler_;
    std::string msg_bayer_format_str_;
    bool auto_exposure_;
    bool enable_profiling_;
    std::string image_publish_topic_;
    std::string image_info_publish_topic_;
    std::string camera_info_publish_topic_;
    std::string camera_id_;

    QhyNode()
        : ParameterNode("qhy_node")
    {
        qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        qhy_camera_.set_debug_info(false);
    }

    void init()
    {
        declare_node_parameters();
    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("image_publish_topic", "sky360/camera/all_sky/bayer"), 
                [this](const rclcpp::Parameter& param) {
                    image_publish_topic_ = param.as_string(); 
                    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(image_publish_topic_, qos_profile_);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("image_info_publish_topic", "sky360/camera/all_sky/image_info"), 
                [this](const rclcpp::Parameter& param) {
                    image_info_publish_topic_ = param.as_string(); 
                    image_info_publisher_ = create_publisher<sky360_camera::msg::ImageInfo>(image_info_publish_topic_, qos_profile_);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("camera_info_publish_topic", "sky360/camera/all_sky/camera_info"), 
                [this](const rclcpp::Parameter& param) {
                    camera_info_publish_topic_ = param.as_string(); 
                    camera_info_publisher_ = create_publisher<sky360_camera::msg::CameraInfo>(camera_info_publish_topic_, qos_profile_);
                }
            ),
            // camera_id should be declared before others so it will open the camera before setting the other configurations
            ParameterNode::ActionParam(
                rclcpp::Parameter("camera_id", ""), 
                [this](const rclcpp::Parameter& param) {
                    camera_id_ = param.as_string();
                    open_camera();
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("enable_profiling", false), 
                [this](const rclcpp::Parameter& param) {enable_profiling_ = param.as_bool();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("auto_exposure", true), 
                [this](const rclcpp::Parameter& param) {auto_exposure_ = param.as_bool();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("exposure", 2000), 
                [this](const rclcpp::Parameter& param) {
                    auto exposure = param.as_int();
                    qhy_camera_.set_control(sky360lib::camera::QhyCamera::ControlParam::Exposure, exposure);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("gain", 0), 
                [this](const rclcpp::Parameter& param) {
                    auto gain = param.as_int();
                    qhy_camera_.set_control(sky360lib::camera::QhyCamera::ControlParam::Gain, gain);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("offset", 0), 
                [this](const rclcpp::Parameter& param) {
                    auto offset = param.as_int();
                    qhy_camera_.set_control(sky360lib::camera::QhyCamera::ControlParam::Offset, offset);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("bpp", 8), 
                [this](const rclcpp::Parameter& param) {
                    auto bpp = param.as_int();
                    qhy_camera_.set_control(sky360lib::camera::QhyCamera::ControlParam::TransferBits, bpp);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("contrast", 0.0), 
                [this](const rclcpp::Parameter& param) {
                    auto contrast = param.as_double();
                    qhy_camera_.set_control(sky360lib::camera::QhyCamera::ControlParam::Contrast, contrast);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("brightness", 0.0), 
                [this](const rclcpp::Parameter& param) {
                    auto brightness = param.as_double();
                    qhy_camera_.set_control(sky360lib::camera::QhyCamera::ControlParam::Brightness, brightness);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("gamma", 1.0), 
                [this](const rclcpp::Parameter& param) {
                    auto gamma = param.as_double();
                    qhy_camera_.set_control(sky360lib::camera::QhyCamera::ControlParam::Gamma, gamma);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("bin", 1), 
                [this](const rclcpp::Parameter& param) {
                    auto bin = param.as_int();
                    qhy_camera_.set_bin_mode(sky360lib::camera::QhyCamera::BinMode(bin));
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("cooling", false), 
                [this](const rclcpp::Parameter& param) {
                    auto camera_params = qhy_camera_.get_camera_params();
                    auto cooling = param.as_bool();
                    qhy_camera_.set_cool_temp(camera_params.target_temp, cooling);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("target_temperature", 0.0), 
                [this](const rclcpp::Parameter& param) {
                    auto camera_params = qhy_camera_.get_camera_params();
                    auto target_temperature = param.as_double();
                    qhy_camera_.set_cool_temp(target_temperature, camera_params.cool_enabled);
                }
            ),
        };
        add_action_parameters(params);
    }

    inline void open_camera()
    {
        qhy_camera_.open(camera_id_);
        RCLCPP_INFO(get_logger(), qhy_camera_.get_camera_info()->to_string().c_str());
        create_camera_info_msg();
        msg_bayer_format_str_ = convert_bayer_pattern(qhy_camera_.get_camera_info()->bayer_format);

        // uint32_t x = ((uint32_t)qhy_camera_.get_camera_info()->chip.max_image_width - (uint32_t)qhy_camera_.get_camera_info()->chip.max_image_height) / 2;
        // uint32_t y = 0;
        // uint32_t width = qhy_camera_.get_camera_info()->chip.max_image_height;
        // uint32_t height = qhy_camera_.get_camera_info()->chip.max_image_height;
        // qhy_camera_.set_resolution(x, y, width, height);
    }

    inline void apply_auto_exposure(const cv::Mat &image, sky360lib::camera::QhyCamera::CameraParams &camera_params)
    {
        cv::Mat subSampledGray = subSampler.subSample(image);
        double msv = brightnessEstimator.estimateCurrentBrightness(subSampledGray);
        double exposure = (double)camera_params.exposure;
        double gain = (double)camera_params.gain;

        auto_exposure_control_.update(msv, exposure, gain);
        set_parameter({"exposure", (int)exposure});
        set_parameter({"gain", (int)gain});
    }

    inline sky360_camera::msg::ImageInfo generate_image_info(std_msgs::msg::Header &header, const sky360lib::camera::QhyCamera::CameraParams &camera_params)
    {
        sky360_camera::msg::ImageInfo image_info_msg;
        image_info_msg.header = header;

        image_info_msg.roi.start_x = camera_params.roi.start_x;
        image_info_msg.roi.start_y = camera_params.roi.start_y;
        image_info_msg.roi.width = camera_params.roi.width;
        image_info_msg.roi.height = camera_params.roi.height;
        image_info_msg.bpp = camera_params.bpp;
        image_info_msg.bayer_format = qhy_camera_.get_camera_info()->bayer_format;
        image_info_msg.exposure = camera_params.exposure;
        image_info_msg.gain = camera_params.gain;
        image_info_msg.offset = camera_params.offset;
        image_info_msg.white_balance.r = camera_params.red_white_balance;
        image_info_msg.white_balance.g = camera_params.green_white_balance;
        image_info_msg.white_balance.b = camera_params.blue_white_balance;
        image_info_msg.contrast = camera_params.contrast;
        image_info_msg.brightness = camera_params.brightness;
        image_info_msg.gamma = camera_params.gamma;
        image_info_msg.channels = camera_params.channels;
        image_info_msg.bin_mode = (uint32_t)camera_params.bin_mode;
        image_info_msg.current_temp = qhy_camera_.get_current_temp();
        image_info_msg.cool_enabled = camera_params.cool_enabled;
        image_info_msg.target_temp = camera_params.target_temp;
        image_info_msg.auto_exposure = auto_exposure_;

        return image_info_msg;
    }

    inline void create_camera_info_msg()
    {
        auto camera_info = qhy_camera_.get_camera_info();

        camera_info_msg_.id = camera_info->id;
        camera_info_msg_.model = camera_info->model;
        camera_info_msg_.serial_num = camera_info->serial_num;
        camera_info_msg_.overscan.start_x = camera_info->overscan.start_x;
        camera_info_msg_.overscan.start_y = camera_info->overscan.start_y;
        camera_info_msg_.overscan.width = camera_info->overscan.width;
        camera_info_msg_.overscan.height = camera_info->overscan.height;
        camera_info_msg_.effective.start_x = camera_info->effective.start_x;
        camera_info_msg_.effective.start_y = camera_info->effective.start_y;
        camera_info_msg_.effective.width = camera_info->effective.width;
        camera_info_msg_.effective.height = camera_info->effective.height;
        camera_info_msg_.chip.width_mm = camera_info->chip.width_mm;
        camera_info_msg_.chip.height_mm = camera_info->chip.height_mm;
        camera_info_msg_.chip.pixel_width_um = camera_info->chip.pixel_width_um;
        camera_info_msg_.chip.pixel_height_um = camera_info->chip.pixel_height_um;
        camera_info_msg_.chip.max_image_width = camera_info->chip.max_image_width;
        camera_info_msg_.chip.max_image_height = camera_info->chip.max_image_height;
        camera_info_msg_.chip.max_bpp = camera_info->chip.max_bpp;
        camera_info_msg_.bayer_format = camera_info->bayer_format;
        camera_info_msg_.is_color = camera_info->is_color;
        camera_info_msg_.is_cool = camera_info->is_cool;
        camera_info_msg_.has_bin1x1_mode = camera_info->has_bin1x1_mode;
        camera_info_msg_.has_bin2x2_mode = camera_info->has_bin2x2_mode;
        camera_info_msg_.has_bin3x3_mode = camera_info->has_bin3x3_mode;
        camera_info_msg_.has_bin4x4_mode = camera_info->has_bin4x4_mode;
        camera_info_msg_.gain_limits.min = camera_info->gain_limits.min;
        camera_info_msg_.gain_limits.max = camera_info->gain_limits.max;
        camera_info_msg_.gain_limits.step = camera_info->gain_limits.step;
        camera_info_msg_.offset_limits.min = camera_info->offset_limits.min;
        camera_info_msg_.offset_limits.max = camera_info->offset_limits.max;
        camera_info_msg_.offset_limits.step = camera_info->offset_limits.step;
        camera_info_msg_.usb_traffic_limits.min = camera_info->usb_traffic_limits.min;
        camera_info_msg_.usb_traffic_limits.max = camera_info->usb_traffic_limits.max;
        camera_info_msg_.usb_traffic_limits.step = camera_info->usb_traffic_limits.step;
        camera_info_msg_.red_wb_limits.min = camera_info->red_wb_limits.min;
        camera_info_msg_.red_wb_limits.max = camera_info->red_wb_limits.max;
        camera_info_msg_.red_wb_limits.step = camera_info->red_wb_limits.step;
        camera_info_msg_.green_wb_limits.min = camera_info->green_wb_limits.min;
        camera_info_msg_.green_wb_limits.max = camera_info->green_wb_limits.max;
        camera_info_msg_.green_wb_limits.step = camera_info->green_wb_limits.step;
        camera_info_msg_.blue_wb_limits.min = camera_info->blue_wb_limits.min;
        camera_info_msg_.blue_wb_limits.max = camera_info->blue_wb_limits.max;
        camera_info_msg_.blue_wb_limits.step = camera_info->blue_wb_limits.step;
        camera_info_msg_.temperature_limits.min = camera_info->temperature_limits.min;
        camera_info_msg_.temperature_limits.max = camera_info->temperature_limits.max;
        camera_info_msg_.temperature_limits.step = camera_info->temperature_limits.step;
    }

    static inline std::string convert_bayer_pattern(sky360lib::camera::QhyCamera::BayerFormat _bayerFormat)
    {
        switch (_bayerFormat)
        {
        case sky360lib::camera::QhyCamera::BayerFormat::BayerGB:
            return sensor_msgs::image_encodings::BAYER_GBRG8;
        case sky360lib::camera::QhyCamera::BayerFormat::BayerGR:
            return sensor_msgs::image_encodings::BAYER_GRBG8;
        case sky360lib::camera::QhyCamera::BayerFormat::BayerBG:
            return sensor_msgs::image_encodings::BAYER_BGGR8;
        case sky360lib::camera::QhyCamera::BayerFormat::BayerRG:
            return sensor_msgs::image_encodings::BAYER_RGGB8;
        default:
            return sensor_msgs::image_encodings::MONO8;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto image_publisher = QhyNode::create();
    image_publisher->start_publishing();
    rclcpp::shutdown();
    return 0;
}
