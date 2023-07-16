#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>

#include <Python.h>

#include "ndarray_converter.h"

#include "../../api/blobs/connectedBlobDetection.hpp"
#include "../../api/bgs/bgs.hpp"
#include "../../api//camera/qhy_camera.hpp"
#include "../../api/utils/auto_exposure.hpp"
#include "../../api/utils/brightness_estimator.hpp"
#include "../../api/utils/auto_white_balance.hpp"


namespace py = pybind11;
using namespace sky360lib::bgs;
using namespace sky360lib::blobs;
using namespace sky360lib::camera;
using namespace sky360lib::utils;

PYBIND11_MODULE(pysky360, m)
{
    NDArrayConverter::init_numpy();

    m.doc() = "python wrapper for sky360lib using pybind11";
    py::object version = py::cast("1.0.0");
    m.attr("__version__") = version;

    py::class_<WMVParams>(m, "WMVParams")
        .def(py::init<>())
        .def("getThreshold", &WMVParams::get_threshold)
        .def("getWeights", &WMVParams::get_weights)
        .def("getEnableWeight", &WMVParams::get_enable_weight)
        .def("getEnableThreshold", &WMVParams::get_enable_threshold)
        .def("setEnableWeight", &WMVParams::set_enable_weight)
        .def("setEnableThreshold", &WMVParams::set_enable_threshold)
        .def("setWeights", &WMVParams::set_weights)
        .def("setThreshold", &WMVParams::set_threshold)
        ;
    py::class_<VibeParams>(m, "VibeParams")
        .def(py::init<>())
        .def("getThreshold", &VibeParams::get_threshold)
        .def("getBGSamples", &VibeParams::get_bg_samples)
        .def("getRequiredBGSamples", &VibeParams::get_required_bg_samples)
        .def("getLearningRate", &VibeParams::get_learning_rate)
        .def("setThreshold", &VibeParams::set_threshold)
        .def("setBGSamples", &VibeParams::set_bg_samples)
        .def("setRequiredBGSamples", &VibeParams::set_required_bg_samples)
        .def("setLearningRate", &VibeParams::set_learning_rate)
        ;

    py::class_<Vibe>(m, "Vibe")
        .def(py::init<>())
        .def("apply", &Vibe::apply_ret)
        .def("getBackgroundImage", &Vibe::get_background_image)
        .def("getParameters", &Vibe::get_parameters, py::return_value_policy::reference);
    py::class_<WeightedMovingVariance>(m, "WeightedMovingVariance")
        .def(py::init<>())
        .def("apply", &WeightedMovingVariance::apply_ret)
        .def("getBackgroundImage", &WeightedMovingVariance::get_background_image)
        .def("getParameters", &WeightedMovingVariance::get_parameters, py::return_value_policy::reference);

    py::class_<ConnectedBlobDetection>(m, "ConnectedBlobDetection")
        .def(py::init<>())
        .def("detect", &ConnectedBlobDetection::detect_kp)
        .def("detectBB", &ConnectedBlobDetection::detect_ret)
        .def("setSizeThreshold", &ConnectedBlobDetection::set_size_threshold)
        .def("setAreaThreshold", &ConnectedBlobDetection::set_area_threshold)
        .def("setMinDistance", &ConnectedBlobDetection::set_min_distance);

    py::class_<QhyCamera>(m, "QHYCamera")
        .def(py::init<>())
        .def("setDebugInfo", &QhyCamera::set_debug_info)
        .def("getFrame", &QhyCamera::get_frame_ret)
        .def("debayerImage", &QhyCamera::debayer_image_ret)
        .def("getLastFrameCaptureTime", &QhyCamera::get_last_frame_capture_time)
        .def("getCameraInfo", &QhyCamera::get_camera_info)
        .def("getCameraParams", &QhyCamera::get_camera_params)
        .def("open", &QhyCamera::open)
        .def("close", &QhyCamera::close)
        .def("setControl", &QhyCamera::set_control)
        .def("setDebayer", &QhyCamera::set_debayer)
        .def("setBinMode", &QhyCamera::set_bin_mode)
        .def("setResolution", &QhyCamera::set_resolution)
        .def("setStreamMode", &QhyCamera::set_stream_mode);

    py::class_<QhyCamera::CameraInfo>(m, "QHYCamera.CameraInfo")
        .def(py::init<>())
        .def("bayerFormatToString", &QhyCamera::CameraInfo::bayer_format_to_string)
        .def("toString", &QhyCamera::CameraInfo::to_string);

    py::class_<QhyCamera::AreaLimits>(m, "QHYCamera.AreaLimits")
        .def(py::init<>())
        .def_readonly("start_x", &QhyCamera::AreaLimits::start_x)
        .def_readonly("start_y", &QhyCamera::AreaLimits::start_y)
        .def_readonly("width", &QhyCamera::AreaLimits::width)
        .def_readonly("height", &QhyCamera::AreaLimits::height)
        ;

    py::class_<QhyCamera::CameraParams>(m, "QHYCamera.CameraParams")
        .def(py::init<>())
        .def_readonly("roi", &QhyCamera::CameraParams::roi)
        .def_readonly("apply_debayer", &QhyCamera::CameraParams::apply_debayer)
        .def_readonly("red_white_balance", &QhyCamera::CameraParams::red_white_balance)
        .def_readonly("green_white_balance", &QhyCamera::CameraParams::green_white_balance)
        .def_readonly("blue_white_balance", &QhyCamera::CameraParams::blue_white_balance)
        .def_readonly("exposure", &QhyCamera::CameraParams::exposure)
        .def_readonly("contrast", &QhyCamera::CameraParams::contrast)
        .def_readonly("brightness", &QhyCamera::CameraParams::brightness)
        .def_readonly("gamma", &QhyCamera::CameraParams::gamma)
        .def_readonly("stream_mode", &QhyCamera::CameraParams::stream_mode)
        .def_readonly("channels", &QhyCamera::CameraParams::channels)
        .def_readonly("usb_traffic", &QhyCamera::CameraParams::usb_traffic)
        .def_readonly("usb_speed", &QhyCamera::CameraParams::usb_speed)
        .def_readonly("gain", &QhyCamera::CameraParams::gain)
        .def_readonly("offset", &QhyCamera::CameraParams::offset)
        .def_readonly("bin_mode", &QhyCamera::CameraParams::bin_mode)
        .def_readonly("target_temp", &QhyCamera::CameraParams::target_temp)
        .def_readonly("cool_enabled", &QhyCamera::CameraParams::cool_enabled)
        .def_readonly("bpp", &QhyCamera::CameraParams::bpp)
        ;

    py::enum_<QhyCamera::BinMode>(m, "BinMode")
        .value("Bin1x1", QhyCamera::BinMode::Bin1x1)
        .value("Bin2x2", QhyCamera::BinMode::Bin2x2)
        .value("Bin3x3", QhyCamera::BinMode::Bin3x3)
        .value("Bin4x4", QhyCamera::BinMode::Bin4x4)
        ;

    py::enum_<QhyCamera::StreamMode>(m, "StreamMode")
        .value("SingleFrame", QhyCamera::StreamMode::SingleFrame)
        .value("LiveFrame", QhyCamera::StreamMode::LiveFrame)
        ;

    py::enum_<QhyCamera::ControlParam>(m, "ControlParam")
        .value("Brightness", QhyCamera::ControlParam::Brightness)
        .value("Exposure", QhyCamera::ControlParam::Exposure)
        .value("Contrast", QhyCamera::ControlParam::Contrast)
        .value("UsbTraffic", QhyCamera::ControlParam::UsbTraffic)
        .value("UsbSpeed", QhyCamera::ControlParam::UsbSpeed)
        .value("Gain", QhyCamera::ControlParam::Gain)
        .value("Offset", QhyCamera::ControlParam::Offset)
        .value("TransferBits", QhyCamera::ControlParam::TransferBits)
        .value("RedWB", QhyCamera::ControlParam::RedWB)
        .value("GreenWB", QhyCamera::ControlParam::GreenWB)
        .value("BlueWB", QhyCamera::ControlParam::BlueWB)
        .value("Gamma", QhyCamera::ControlParam::Gamma)
        .value("Channels", QhyCamera::ControlParam::Channels)
        .export_values();

    py::class_<AutoExposure>(m, "AutoExposure")
        .def(py::init<double, double, double, double>())
        .def("update", &AutoExposure::update)
        .def("set_target_msv", &AutoExposure::set_target_msv)
        .def("get_target_msv", &AutoExposure::get_target_msv)
        .def("get_current_msv", &AutoExposure::get_current_msv)
        .def("is_day", &AutoExposure::is_day);


    py::class_<AutoWhiteBalance>(m, "AutoWhiteBalance")
        .def(py::init<>())
        .def("illumination_estimation", &AutoWhiteBalance::illumination_estimation)
        ;

    py::class_<WhiteBalanceValues>(m, "WhiteBalanceValues")
        .def(py::init<>())
        .def_readonly("red", &WhiteBalanceValues::red)
        .def_readonly("green", &WhiteBalanceValues::green)
        .def_readonly("blue", &WhiteBalanceValues::blue)
        ;

}