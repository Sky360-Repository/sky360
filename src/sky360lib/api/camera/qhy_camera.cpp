#include "qhy_camera.hpp"

#include <thread>

namespace sky360lib::camera
{
    std::string QhyCamera::CameraInfo::bayer_format_to_string() const
    {
        switch (bayer_format)
        {
        case BayerGB:
            return "BayerGB";
        case BayerGR:
            return "BayerGR";
        case BayerBG:
            return "BayerBG";
        case BayerRG:
            return "BayerRG";
        case Mono:
            return "Mono";
        case Color:
            return "Color";
        }
        return "Unknown";
    }

    std::string QhyCamera::CameraInfo::to_string() const
    {
        std::stringstream toStr;
        toStr << "Camera model: " << model << ", Serial: " << serial_num << ", Id: " << id << std::endl;
        toStr << "Overscan  Area startX x startY: " << overscan.start_x << " x " << overscan.start_y
              << ", sizeX x sizeY : " << overscan.width << " x " << overscan.height << std::endl;
        toStr << "Effective Area startX x startY: " << effective.start_x << " x " << effective.start_y
              << ", sizeX x sizeY : " << effective.width << " x " << effective.height << std::endl;
        toStr << "Chip      Size width x height: " << chip.width_mm << " x " << chip.height_mm << " [mm]" << std::endl;
        toStr << "Pixel     Size width x height: " << chip.pixel_width_um << " x " << chip.pixel_height_um << " [um]" << std::endl;
        toStr << "Max Image Size width x height: " << chip.max_image_width << " x " << chip.max_image_height << std::endl;
        toStr << "Max Bits per Pixel: " << chip.max_bpp << std::endl;
        toStr << "Camera is color: " << (is_color ? "Yes" : "No") << ", Bayer Pattern: " << bayer_format_to_string() << std::endl;
        toStr << "Camera is cooled: " << (is_cool ? "Yes" : "No") << std::endl;
        toStr << "Available Bin modes:"
              << (has_bin1x1_mode ? " (1x1)" : "")
              << (has_bin2x2_mode ? " (2x2)" : "")
              << (has_bin3x3_mode ? " (3x3)" : "")
              << (has_bin4x4_mode ? " (4x4)" : "")
              << std::endl;
        toStr << "Gain Limits: Min: " << gain_limits.min << ", Max: " << gain_limits.max << ", Step: " << gain_limits.step << std::endl;
        toStr << "Offset Limits: Min: " << offset_limits.min << ", Max: " << offset_limits.max << ", Step: " << offset_limits.step << std::endl;
        toStr << "Gamma Limits: Min: " << gamma_limits.min << ", Max: " << gamma_limits.max << ", Step: " << gamma_limits.step << std::endl;
        toStr << "Usb Traffic Limits: Min: " << usb_traffic_limits.min << ", Max: " << usb_traffic_limits.max << ", Step: " << usb_traffic_limits.step << std::endl;
        toStr << "Temperature Limits: Min: " << temperature_limits.min << ", Max: " << temperature_limits.max << ", Step: " << temperature_limits.step << std::endl;
        return toStr.str();
    }

    QhyCamera::QhyCamera()
    {
        EnableQHYCCDMessage(false);
        EnableQHYCCDLogFile(false);
    }

    QhyCamera::~QhyCamera()
    {
        release();
    }

    bool QhyCamera::init()
    {
        if (!m_is_cam_init)
        {
            if (InitQHYCCDResource() != QHYCCD_SUCCESS)
            {
                std::cerr << "Cannot initialize SDK resources" << std::endl;
                m_is_cam_init = false;
                return false;
            }
            m_is_cam_init = true;
        }

        return m_is_cam_init;
    }

    void QhyCamera::release()
    {
        if (m_is_cam_open)
        {
            close();
        }

        uint32_t rc = ReleaseQHYCCDResource();
        if (QHYCCD_SUCCESS != rc)
        {
            std::cerr << "Cannot release SDK resources, error: " << rc << std::endl;
        }
        m_is_cam_init = false;
    }

    void QhyCamera::close()
    {
        if (m_is_cam_open)
        {
            if (m_params.stream_mode == SingleFrame)
            {
                CancelQHYCCDExposingAndReadout(m_cam_handle);
            }
            else
            {
                StopQHYCCDLive(m_cam_handle);
            }

            CloseQHYCCD(m_cam_handle);

            m_old_cam_id = m_cam_id;
            m_img_data = nullptr;
            m_cam_handle = nullptr;
            m_cam_id = "";
            m_is_cam_open = false;
            m_is_exposing = false;
        }
    }

    const std::map<std::string, QhyCamera::CameraInfo> &QhyCamera::get_cameras()
    {
        scan_cameras();
        return m_cameras;
    }

    bool QhyCamera::scan_cameras()
    {
        init();

        m_cameras.clear();

        uint32_t camCount = ScanQHYCCD();
        if (camCount <= 0)
        {
            std::cerr << "No QHYCCD camera found, please check USB or power." << std::endl;
            return false;
        }

        char camId[64];
        for (uint32_t i{0}; i < camCount; ++i)
        {
            uint32_t rc = GetQHYCCDId(i, camId);
            if (rc == QHYCCD_SUCCESS)
            {
                CameraInfo ci;
                if (fill_camera_info(camId, ci))
                {
                    m_cameras[camId] = ci;
                }
            }
        }

        if (m_cameras.size() == 0)
        {
            release();
            return false;
        }

        return true;
    }

    bool QhyCamera::fill_camera_info(const std::string& _camera_id, CameraInfo &_ci)
    {
        qhyccd_handle *camHandle = OpenQHYCCD((char *)_camera_id.c_str());
        if (camHandle == nullptr)
        {
            std::cerr << "OpenQHYCCD failure, camera id: " << _camera_id << std::endl;
            return false;
        }

        _ci.id = _camera_id;
        size_t posDash = _ci.id.find("-");
        _ci.model = _ci.id.substr(0, posDash);
        _ci.serial_num = _ci.id.substr(posDash + 1);

        uint32_t rc = GetQHYCCDOverScanArea(camHandle, &_ci.overscan.start_x, &_ci.overscan.start_y, 
                                &_ci.overscan.width, &_ci.overscan.height);
        if (rc != QHYCCD_SUCCESS)
        {
            std::cerr << "GetQHYCCDOverScanArea failure, camera id: " << _camera_id << ", error: " << rc << std::endl;
            return false;
        }

        rc = GetQHYCCDEffectiveArea(camHandle, &_ci.effective.start_x, &_ci.effective.start_y, 
                                &_ci.effective.width, &_ci.effective.height);
        if (rc != QHYCCD_SUCCESS)
        {
            std::cerr << "GetQHYCCDEffectiveArea failure, camera id: " << _camera_id << ", error: " << rc << std::endl;
            return false;
        }

        rc = GetQHYCCDChipInfo(camHandle, &_ci.chip.width_mm, &_ci.chip.height_mm, 
                            &_ci.chip.max_image_width, &_ci.chip.max_image_height,
                            &_ci.chip.pixel_width_um, &_ci.chip.pixel_height_um, 
                            &_ci.chip.max_bpp);
        if (rc != QHYCCD_SUCCESS)
        {
            std::cerr << "GetQHYCCDChipInfo failure," << std::endl;
            return false;
        }

        _ci.bayer_format = BayerFormat(IsQHYCCDControlAvailable(camHandle, CAM_COLOR));
        _ci.is_color = (_ci.bayer_format == BayerGB || _ci.bayer_format == BayerGR || _ci.bayer_format == BayerBG || _ci.bayer_format == BayerRG);

        _ci.is_cool = IsQHYCCDControlAvailable(camHandle, CONTROL_COOLER) == QHYCCD_SUCCESS;

        _ci.has_bin1x1_mode = IsQHYCCDControlAvailable(camHandle, CAM_BIN1X1MODE) == QHYCCD_SUCCESS;
        _ci.has_bin2x2_mode = IsQHYCCDControlAvailable(camHandle, CAM_BIN2X2MODE) == QHYCCD_SUCCESS;
        _ci.has_bin3x3_mode = IsQHYCCDControlAvailable(camHandle, CAM_BIN3X3MODE) == QHYCCD_SUCCESS;
        _ci.has_bin4x4_mode = IsQHYCCDControlAvailable(camHandle, CAM_BIN4X4MODE) == QHYCCD_SUCCESS;

        GetQHYCCDParamMinMaxStep(camHandle, CONTROL_GAIN, &_ci.gain_limits.min, &_ci.gain_limits.max, &_ci.gain_limits.step);
        GetQHYCCDParamMinMaxStep(camHandle, CONTROL_OFFSET, &_ci.offset_limits.min, &_ci.offset_limits.max, &_ci.offset_limits.step);
        GetQHYCCDParamMinMaxStep(camHandle, CONTROL_USBTRAFFIC, &_ci.usb_traffic_limits.min, &_ci.usb_traffic_limits.max, &_ci.usb_traffic_limits.step);
        GetQHYCCDParamMinMaxStep(camHandle, CONTROL_WBR, &_ci.red_wb_limits.min, &_ci.red_wb_limits.max, &_ci.red_wb_limits.step);
        GetQHYCCDParamMinMaxStep(camHandle, CONTROL_WBG, &_ci.green_wb_limits.min, &_ci.green_wb_limits.max, &_ci.green_wb_limits.step);
        GetQHYCCDParamMinMaxStep(camHandle, CONTROL_WBB, &_ci.blue_wb_limits.min, &_ci.blue_wb_limits.max, &_ci.blue_wb_limits.step);
        GetQHYCCDParamMinMaxStep(camHandle, CONTROL_COOLER, &_ci.temperature_limits.min, &_ci.temperature_limits.max, &_ci.temperature_limits.step);
        GetQHYCCDParamMinMaxStep(camHandle, CONTROL_GAMMA, &_ci.gamma_limits.min, &_ci.gamma_limits.max, &_ci.gamma_limits.step);

        rc = CloseQHYCCD(camHandle);
        if (rc != QHYCCD_SUCCESS)
        {
            std::cerr << "Close QHYCCD failure, error: " << rc << std::endl;
        }
        if (m_is_debug_info)
        {
            std::cout << _ci.to_string() << std::endl;
        }

        return true;
    }

    bool QhyCamera::open(const std::string& _camera_id)
    {
        if (!m_is_cam_init && !init())
        {
            return false;
        }
        if (!m_is_cam_open)
        {
            auto camera_id = _camera_id;
            if (camera_id.empty())
            {
                if (!scan_cameras())
                {
                    return false;
                }
                camera_id = m_cameras.begin()->second.id;
            }
            else if (camera_id != m_old_cam_id)
            {
                m_is_default_set = false;
            }
            m_cam_id = camera_id;
            m_current_info = &m_cameras[m_cam_id];

            m_cam_handle = OpenQHYCCD((char *)m_cam_id.c_str());
            if (m_cam_handle == nullptr)
            {
                m_cam_id = "";
                std::cerr << "Open QHYCCD failure." << std::endl;
                return false;
            }

            set_default_params();
            m_is_cam_open = true;
        }

        return m_is_cam_open;
    }

    double QhyCamera::get_current_temp() const
    {
        return GetQHYCCDParam(m_cam_handle, CONTROL_CURTEMP);
    }

    bool QhyCamera::set_cool_temp(double _target_temp, bool _enable)
    {
        if (!m_current_info->is_cool)
        {
            return false;
        }

        m_params.cool_enabled = _enable;
        if (!_enable)
        {
            set_control_low_level(ManualPwm, 0);
            return true;
        }

        if (!set_control_low_level(Cooler, _target_temp))
        {
            return false;
        }

        m_params.target_temp = _target_temp;

        return true;
    }

    void QhyCamera::set_default_params()
    {
        if (!m_is_default_set)
        {
            if (m_is_debug_info)
            {
                std::cout << "set_default_params: Set default params." << std::endl;
            }
            set_debayer(false);
            set_control(RedWB, 165.0, true);
            set_control(GreenWB, 128.0, true);
            set_control(BlueWB, 240.0, true);
            set_control(Exposure, 2000, true);
            set_stream_mode(LiveFrame);
            //set_stream_mode(SingleFrame);
            set_control(UsbTraffic, 3, true);
            set_control(UsbSpeed, 0, true);
            set_control(Gain, 0, true);
            set_control(Offset, 0, true);
            set_resolution(0, 0, m_current_info->chip.max_image_width, m_current_info->chip.max_image_height);
            set_control(TransferBits, 8, true);
            //set_control(TransferBits, 16, true);
            set_control(Channels, 1, true);
            set_bin_mode(Bin1x1);
            set_control(Contrast, 0.0, true);
            set_control(Brightness, 0.0, true);
            set_control(Gamma, 1.0, true);
            set_control(Cooler, 10.0, true);
            set_cool_temp(-5.0, false);

            m_is_default_set = true;
        }
        else
        {
            if (m_is_debug_info)
            {
                std::cout << "set_default_params: Set current params." << std::endl;
            }
            set_debayer(m_params.apply_debayer);
            set_control(RedWB, m_params.red_white_balance, true);
            set_control(GreenWB, m_params.green_white_balance, true);
            set_control(BlueWB, m_params.blue_white_balance, true);
            set_control(Exposure, m_params.exposure, true);
            set_stream_mode(m_params.stream_mode);
            set_control(UsbTraffic, m_params.usb_traffic, true);
            set_control(UsbSpeed, m_params.usb_speed, true);
            set_control(Gain, m_params.gain, true);
            set_control(Offset, m_params.offset, true);
            set_resolution(m_params.roi.start_x, m_params.roi.start_y, m_params.roi.width, m_params.roi.height);
            set_control(TransferBits, m_params.bpp, true);
            set_control(Channels, m_params.channels, true);
            set_bin_mode(m_params.bin_mode);
            set_control(Contrast, m_params.contrast, true);
            set_control(Brightness, m_params.brightness, true);
            set_control(Gamma, m_params.gamma, true);
            set_cool_temp(m_params.target_temp, m_params.cool_enabled);
        }
    }

    bool QhyCamera::set_debayer(bool _enable)
    {
        if (SetQHYCCDDebayerOnOff(m_cam_handle, _enable) != QHYCCD_SUCCESS)
        {
            std::cerr << "SetQHYCCDDebayerOnOff failure" << std::endl;
            return false;
        }
        alloc_buffer_memory();
        m_params.apply_debayer = _enable;
        m_params.bayer_format = m_params.bin_mode != Bin1x1 ? Mono : (m_params.apply_debayer ? Color : m_current_info->bayer_format);

        return true;
    }

    bool QhyCamera::set_bin_mode(BinMode _mode)
    {
        uint32_t rc = SetQHYCCDBinMode(m_cam_handle, (uint32_t)_mode, (uint32_t)_mode);
        if (rc != QHYCCD_SUCCESS)
        {
            std::cerr << "SetQHYCCDBinMode failure, error: " << rc << std::endl;
            return false;
        }
        uint32_t width = m_current_info->chip.max_image_width / (uint32_t)_mode;
        uint32_t height = m_current_info->chip.max_image_height / (uint32_t)_mode;
        m_params.bin_mode = _mode;
        m_params.bayer_format = m_params.bin_mode != Bin1x1 ? Mono : (m_params.apply_debayer ? Color : m_current_info->bayer_format);
        set_resolution(0, 0, width, height);

        return true;
    }

    bool QhyCamera::set_resolution(uint32_t _startX, uint32_t _startY, uint32_t _width, uint32_t _height)
    {
        uint32_t rc = SetQHYCCDResolution(m_cam_handle, _startX, _startY, _width, _height);
        if (rc != QHYCCD_SUCCESS)
        {
            std::cerr << "SetQHYCCDResolution failure, error: " << rc << std::endl;
            return false;
        }
        m_params.roi.start_x = _startX;
        m_params.roi.start_y = _startY;
        m_params.roi.width = _width;
        m_params.roi.height = _height;

        if (m_is_cam_open)
        {
            alloc_buffer_memory();
            close();
            open(m_old_cam_id);
        }

        return true;
    }

    bool QhyCamera::set_stream_mode(StreamMode _mode)
    {
        uint32_t rc = SetQHYCCDStreamMode(m_cam_handle, (uint8_t)_mode);
        if (rc != QHYCCD_SUCCESS)
        {
            std::cerr << "Error setting stream mode" << std::endl;
            return false;
        }
        m_params.stream_mode = _mode;

        rc = InitQHYCCD(m_cam_handle);
        if (rc != QHYCCD_SUCCESS)
        {
            std::cerr << "InitQHYCCD faililure" << std::endl;
            return false;
        }

        return true;
    }

    bool QhyCamera::alloc_buffer_memory()
    {
        uint32_t size = GetQHYCCDMemLength(m_cam_handle);
        if (size == 0)
        {
            std::cerr << "Cannot get memory for frame." << std::endl;
            return false;
        }
        m_img_data = std::make_unique_for_overwrite<uint8_t[]>(size);

        return true;
    }

    bool QhyCamera::set_control_low_level(ControlParam _control_param, double _value)
    {
        uint32_t rc = SetQHYCCDParam(m_cam_handle, (CONTROL_ID)_control_param, _value);
        if (rc != QHYCCD_SUCCESS)
        {
            std::cerr << "setControl failed: " << _control_param << std::endl;
            return false;
        }
        return true;
    }

    bool QhyCamera::set_control(ControlParam _control_param, double _value, bool _force)
    {
        if (m_is_debug_info)
        {
            std::cout << "set_control: Param: " << _control_param << ", value: " << _value << ", force: " << _force << std::endl;
        }
        uint32_t rc = IsQHYCCDControlAvailable(m_cam_handle, (CONTROL_ID)_control_param);
        if (rc == QHYCCD_SUCCESS)
        {
            auto change = check_force(_control_param, _value, _force);
            if (change)
            {
                auto apply_direct_change = check_apply_direct_change(_control_param);
                if (apply_direct_change)
                {
                    auto changed = set_control_low_level(_control_param, _value);
                    if (!changed) 
                    {
                        std::cerr << "Control could not change: " << _control_param << std::endl;
                        return false;
                    }
                }

                change_internal_param(_control_param, _value);

                apply_after_change(_control_param, apply_direct_change);
            }
        } 
        else if (m_is_debug_info)
        {
            std::cout << "Control not available to change: " << _control_param << std::endl;
        }

        return true;
    }

    bool QhyCamera::check_force(ControlParam _control_param, double _value, bool _force)
    {
        if (!_force)
        {
            double currentValue = 0.0;
            switch (_control_param)
            {
                case ControlParam::RedWB: currentValue = m_params.red_white_balance; break;
                case ControlParam::GreenWB: currentValue = m_params.green_white_balance; break;
                case ControlParam::BlueWB: currentValue = m_params.blue_white_balance; break;
                case ControlParam::Brightness: currentValue = m_params.brightness; break;
                case ControlParam::Channels: currentValue = (double)m_params.channels; break;
                case ControlParam::Contrast: currentValue = m_params.contrast; break;
                case ControlParam::Exposure: currentValue = (double)m_params.exposure; break;
                case ControlParam::UsbTraffic: currentValue = (double)m_params.usb_traffic; break;
                case ControlParam::UsbSpeed: currentValue = (double)m_params.usb_speed; break;
                case ControlParam::Gain: currentValue = (double)m_params.gain; break;
                case ControlParam::Offset: currentValue = (double)m_params.offset; break;
                case ControlParam::TransferBits: currentValue = (double)m_params.bpp; break;
                case ControlParam::Gamma: currentValue = m_params.gamma; break;
                case ControlParam::Cooler: currentValue = m_params.target_temp; break;
                default: return false;
            };
           return currentValue != _value;
        }

        return true;
    }

    void QhyCamera::change_internal_param(ControlParam _control_param, double _value)
    {
        switch (_control_param)
        {
            case ControlParam::RedWB: m_params.red_white_balance = _value; break;
            case ControlParam::GreenWB: m_params.green_white_balance = _value; break;
            case ControlParam::BlueWB: m_params.blue_white_balance = _value; break;
            case ControlParam::Brightness: m_params.brightness = (uint32_t)_value; break;
            case ControlParam::Channels: m_params.channels = (uint32_t)_value; break;
            case ControlParam::Contrast: m_params.contrast = _value; break;
            case ControlParam::Exposure: m_params.exposure = (uint32_t)_value; break;
            case ControlParam::UsbTraffic: m_params.usb_traffic = (uint32_t)_value; break;
            case ControlParam::UsbSpeed: m_params.usb_speed = (uint32_t)_value; break;
            case ControlParam::Gain: m_params.gain = (uint32_t)_value; break;
            case ControlParam::Offset: m_params.offset = (uint32_t)_value; break;
            case ControlParam::TransferBits: m_params.bpp = (uint32_t)_value; break;
            case ControlParam::Gamma: m_params.gamma = _value; break;
            case ControlParam::Cooler: m_params.target_temp = _value; break;
            default: break;
        };
    }

    bool QhyCamera::check_apply_direct_change(ControlParam _control_param)
    {
        if (m_is_cam_open)
        {
            switch (_control_param)
            {
                case ControlParam::TransferBits:
                    release();
                    return false;
                default:
                    break;
            };
        }
        return true;
    }

    void QhyCamera::apply_after_change(ControlParam _control_param, bool _apply_direct_change)
    {
        if (!_apply_direct_change)
        {
            switch (_control_param)
            {
                case ControlParam::TransferBits:
                    open(m_old_cam_id);
                    break;
                default:
                    break;
            };
        }
    }

    bool QhyCamera::get_frame(cv::Mat &_frame, bool _debayer)
    {
        auto frame_got = get_frame();
        if (!frame_got)
        {
            return false;
        }

        // int channels = (m_current_info->is_color && m_params.apply_debayer) ? 3 : 1;
        int channels = m_params.bayer_format == Color ? 3 : 1;
        int type = m_params.bpp == 16 ? CV_MAKETYPE(CV_16U, channels) : CV_MAKETYPE(CV_8U, channels);

        const cv::Mat imgQHY(m_params.roi.height, m_params.roi.width, type, (int8_t *)m_img_data.get());

        // if (_debayer && m_current_info->is_color && !m_params.apply_debayer)
        if (_debayer && m_params.bayer_format != Color && m_params.bayer_format != Mono)
        {
            debayer_image(imgQHY, _frame);
        }
        else
        {
            imgQHY.copyTo(_frame);
        }

        return true;
    }

    cv::Mat QhyCamera::get_frame_ret(bool _debayer)
    {
        cv::Mat returnFrame;
        get_frame(returnFrame, _debayer);
        return returnFrame;
    }

    static inline int convert_bayer_pattern(QhyCamera::BayerFormat _bayerFormat)
    {
        switch (_bayerFormat)
        {
        case QhyCamera::BayerGB:
            return cv::COLOR_BayerGR2BGR; //!< equivalent to GBRG Bayer pattern
        case QhyCamera::BayerGR:
            return cv::COLOR_BayerGB2BGR; //!< equivalent to GRBG Bayer pattern
        case QhyCamera::BayerBG:
            return cv::COLOR_BayerRG2BGR; //!< equivalent to BGGR Bayer pattern
        case QhyCamera::BayerRG:
            return cv::COLOR_BayerBG2BGR; //!< equivalent to RGGB Bayer pattern
        default:
            return cv::COLOR_BayerGR2BGR;
        }
    }

    void QhyCamera::debayer_image(const cv::Mat &_image_in, cv::Mat &_image_out) const
    {
        if (_image_in.channels() == 1 && m_params.bayer_format != Mono)
        {
            cv::cvtColor(_image_in, _image_out, convert_bayer_pattern(m_current_info->bayer_format));
        }
        else
        {
            _image_in.copyTo(_image_out);
        }
    }

    cv::Mat QhyCamera::debayer_image_ret(const cv::Mat& _image_in) const
    {
        cv::Mat imageOut;
        debayer_image(_image_in, imageOut);
        return imageOut;
    }

    bool QhyCamera::get_frame()
    {
        using fsec = std::chrono::duration<double>;
        uint32_t w, h, bpp, channels, tries;

        if (!m_is_exposing)
        {
            begin_exposing();
        }

        auto start = std::chrono::high_resolution_clock::now();

        if (m_params.stream_mode == SingleFrame)
        {
            if (!get_single(&w, &h, &bpp, &channels, &tries))
            {
                return false;
            }
        }
        else
        {
            if (!get_live(&w, &h, &bpp, &channels, &tries))
            {
                return false;
            }
        }

        auto stop = std::chrono::high_resolution_clock::now();
        fsec duration = (stop - start);
        m_last_frame_capture_time = duration.count();

        if (m_is_debug_info)
        {
            std::cout << "Gotframe: " << w << "x" << h << " pixels, " << bpp << "bpp, " << channels << " channels, tries: " << tries << std::endl;
        }

        return true;
    }

    inline bool QhyCamera::get_single(uint32_t *w, uint32_t *h, uint32_t *bpp, uint32_t *channels, uint32_t *tries)
    {
        *tries = 0;
        ExpQHYCCDSingleFrame(m_cam_handle);
        while (GetQHYCCDSingleFrame(m_cam_handle, w, h, bpp, channels, m_img_data.get()) != QHYCCD_SUCCESS)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
            if (++*tries > DEFAULT_CAPTURE_RETRIES)
            {
                std::cout << "retries: " << tries << ", aborting." << std::endl;
                return false;
            }
        }

        return true;
    }

    inline bool QhyCamera::get_live(uint32_t *w, uint32_t *h, uint32_t *bpp, uint32_t *channels, uint32_t *tries)
    {
        *tries = 0;
        while (GetQHYCCDLiveFrame(m_cam_handle, w, h, bpp, channels, m_img_data.get()) != QHYCCD_SUCCESS)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
            if (++*tries > DEFAULT_CAPTURE_RETRIES)
            {
                std::cout << "retries: " << tries << ", aborting." << std::endl;
                return false;
            }
        }
        return true;
    }

    bool QhyCamera::begin_exposing()
    {
        if (m_params.stream_mode == SingleFrame)
        {
            if (m_is_exposing)
            {
                CancelQHYCCDExposingAndReadout(m_cam_handle);
            }
            uint32_t rc = ExpQHYCCDSingleFrame(m_cam_handle);
            if (rc != QHYCCD_ERROR)
            {
                if (rc == QHYCCD_READ_DIRECTLY)
                {
                    std::this_thread::sleep_for(std::chrono::microseconds(10));
                }
            }
            else
            {
                std::cerr << "ExpQHYCCDSingleFrame failed: " << rc << std::endl;
                return false;
            }
        }
        else
        {
            if (m_is_exposing)
            {
                StopQHYCCDLive(m_cam_handle);
            }
            uint32_t rc = BeginQHYCCDLive(m_cam_handle);
            if (rc != QHYCCD_SUCCESS)
            {
                std::cerr << "ExpQHYCCDSingleFrame failed: " << rc << std::endl;
                return false;
            }
        }
        m_is_exposing = true;
        return m_is_exposing;
    }

    bool QhyCamera::end_exposing()
    {
        if (m_is_exposing)
        {
            if (m_params.stream_mode == SingleFrame)
            {
                CancelQHYCCDExposingAndReadout(m_cam_handle);
            }
            else
            {
                StopQHYCCDLive(m_cam_handle);
            }
            m_is_exposing = false;
        }
        return m_is_exposing;
    }
}