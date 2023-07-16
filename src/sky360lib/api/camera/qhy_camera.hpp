#pragma once

#include <string>
#include <stdint.h>

#include "qhyccd_wrap.h"
#include <opencv2/opencv.hpp>

namespace sky360lib::camera
{
    class QhyCamera
    {
    public:
        enum BinMode
        {
            Bin1x1 = 1,
            Bin2x2 = 2,
            Bin3x3 = 3,
            Bin4x4 = 4
        };
        enum StreamMode
        {
            SingleFrame = 0,
            LiveFrame = 1
        };
        enum BayerFormat
        {
            BayerGB = 1,
            BayerGR,
            BayerBG,
            BayerRG,
            Mono,
            Color
        };

        struct ParamLimits
        {
            double min;
            double max;
            double step;
        };

        struct AreaLimits
        {
            uint32_t start_x;
            uint32_t start_y;
            uint32_t width;
            uint32_t height;
        };

        struct ChipInfo
        {
            double width_mm;
            double height_mm;

            double pixel_width_um;
            double pixel_height_um;

            uint32_t max_image_width;
            uint32_t max_image_height;

            uint32_t max_bpp;
        };

        struct CameraInfo
        {
            std::string id;
            std::string model;
            std::string serial_num;

            AreaLimits overscan;
            AreaLimits effective;

            ChipInfo chip;

            BayerFormat bayer_format;
            bool is_color;

            bool is_cool;

            bool has_bin1x1_mode;
            bool has_bin2x2_mode;
            bool has_bin3x3_mode;
            bool has_bin4x4_mode;

            ParamLimits gain_limits;
            ParamLimits offset_limits;
            ParamLimits gamma_limits;
            ParamLimits usb_traffic_limits;
            ParamLimits red_wb_limits;
            ParamLimits green_wb_limits;
            ParamLimits blue_wb_limits;
            ParamLimits temperature_limits;

            std::string bayer_format_to_string() const;

            std::string to_string() const;
        };

        struct CameraParams
        {
            AreaLimits roi;

            bool apply_debayer;
            double red_white_balance;
            double green_white_balance;
            double blue_white_balance;

            uint32_t exposure;
            double contrast;
            double brightness;
            double gamma;

            StreamMode stream_mode; 

            uint32_t channels;
            uint32_t usb_traffic;
            uint32_t usb_speed;
            uint32_t gain;
            uint32_t offset;
            BinMode bin_mode;

            double target_temp;
            bool cool_enabled;

            uint32_t bpp;

            BayerFormat bayer_format;
        };

        enum ControlParam
        {
            Brightness = CONTROL_BRIGHTNESS, //!< image brightness
            Contrast = CONTROL_CONTRAST, //!< image contrast
            Exposure = CONTROL_EXPOSURE, //!< expose time (us)
            UsbTraffic = CONTROL_USBTRAFFIC, //!< hblank
            UsbSpeed = CONTROL_SPEED, //!< transfer speed
            Gain = CONTROL_GAIN, //!< camera gain
            Offset = CONTROL_OFFSET, //!< camera offset
            TransferBits = CONTROL_TRANSFERBIT, //!< image depth bits
            RedWB = CONTROL_WBR, //!< red of white balance
            BlueWB = CONTROL_WBB, //!< blue of white balance
            GreenWB = CONTROL_WBG, //!< the green of white balance
            Gamma = CONTROL_GAMMA, //!< screen gamma
            Channels = CONTROL_CHANNELS, //!< image channels
            Cooler = CONTROL_COOLER,
            ManualPwm = CONTROL_MANULPWM
        };

        QhyCamera();
        ~QhyCamera();

        void set_debug_info(bool _enable) { m_is_debug_info = _enable; };

        const std::map<std::string, CameraInfo>& get_cameras();

        bool get_frame(cv::Mat& _frame, bool _debayer);
        cv::Mat get_frame_ret(bool _debayer);
        double get_last_frame_capture_time() const { return m_last_frame_capture_time; }

        void debayer_image(const cv::Mat& _image_in, cv::Mat& _image_out) const;
        cv::Mat debayer_image_ret(const cv::Mat& _image_in) const;

        CameraInfo const* get_camera_info() const { return m_current_info; }
        const CameraParams& get_camera_params() const { return m_params; }

        bool open(const std::string& _camera_id);
        void close();

        double get_current_temp() const;

        bool set_cool_temp(double _target_temp, bool _enable);
        bool set_control(ControlParam _control_param, double _value, bool _force = false);
        bool set_debayer(bool _enable);
        bool set_bin_mode(BinMode _mode);
        bool set_resolution(uint32_t _startX, uint32_t _startY, uint32_t _width, uint32_t _height);
        bool set_stream_mode(StreamMode _mode);

    private:
        static const int DEFAULT_CAPTURE_RETRIES = 1000;

        std::string m_cam_id;
        std::string m_old_cam_id;
        qhyccd_handle *m_cam_handle{nullptr};
        std::unique_ptr<uint8_t[]> m_img_data{nullptr};
        std::map<std::string, CameraInfo> m_cameras;
        CameraParams m_params;
        CameraInfo* m_current_info;
        double m_last_frame_capture_time;

        bool m_is_debug_info{false};
        bool m_is_cam_init{false};
        bool m_is_cam_open{false};
        bool m_is_exposing{false};
        bool m_is_default_set{false};

        bool init();
        void release();

        bool check_force(ControlParam _control_param, double _value, bool _force);
        void change_internal_param(ControlParam _control_param, double _value);
        bool check_apply_direct_change(ControlParam _control_param);
        void apply_after_change(ControlParam _control_param, bool _wasOpen);
        bool get_frame();
        bool begin_exposing();
        bool end_exposing();
        bool set_control_low_level(ControlParam _controlParam, double _value);
        bool fill_camera_info(const std::string& _camId, CameraInfo &_ci);
        bool scan_cameras();
        bool alloc_buffer_memory();
        void set_default_params();
        inline bool get_single(uint32_t *_w, uint32_t *_h, uint32_t *_bpp, uint32_t *_channels, uint32_t *tries);
        inline bool get_live(uint32_t *_w, uint32_t *_h, uint32_t *_bpp, uint32_t *_channels, uint32_t *tries);
    };
}