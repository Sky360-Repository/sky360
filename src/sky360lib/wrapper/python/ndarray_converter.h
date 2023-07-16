#ifndef __NDARRAY_CONVERTER_H__
#define __NDARRAY_CONVERTER_H__

#include <Python.h>
#include <opencv2/core/core.hpp>

class NDArrayConverter
{
public:
    // must call this first, or the other routines don't work!
    static bool init_numpy();

    static bool toMat(PyObject *o, cv::Mat &m);
    static PyObject *toNDArray(const cv::Mat &mat);
    static PyObject *toPy(const cv::KeyPoint &mat);
    static PyObject *toPy(const cv::Rect &mat);
};

//
// Define the type converter
//

#include <pybind11/pybind11.h>

namespace pybind11
{
    namespace detail
    {
        template <>
        struct type_caster<cv::Mat>
        {
        public:
            PYBIND11_TYPE_CASTER(cv::Mat, _("numpy.ndarray"));

            bool load(handle src, bool)
            {
                return NDArrayConverter::toMat(src.ptr(), value);
            }

            static handle cast(const cv::Mat &m, return_value_policy, handle)
            {
                return handle(NDArrayConverter::toNDArray(m));
            }
        };

        template <>
        struct type_caster<cv::KeyPoint>
        {
        public:
            PYBIND11_TYPE_CASTER(cv::KeyPoint, _("keypoint"));

            bool load(handle, bool)
            {
                return true;//NDArrayConverter::toMat(src.ptr(), value);
            }

            static handle cast(const cv::KeyPoint &m, return_value_policy, handle)
            {
                return handle(NDArrayConverter::toPy(m));
            }
        };

        template <>
        struct type_caster<cv::Rect>
        {
        public:
            PYBIND11_TYPE_CASTER(cv::KeyPoint, _("rect"));

            bool load(handle, bool)
            {
                return true;//NDArrayConverter::toMat(src.ptr(), value);
            }

            static handle cast(const cv::Rect &m, return_value_policy, handle)
            {
                return handle(NDArrayConverter::toPy(m));
            }
        };

    }
} // namespace pybind11::detail

#endif
