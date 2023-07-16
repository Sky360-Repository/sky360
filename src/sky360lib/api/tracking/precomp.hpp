#pragma once

#include <opencv2/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/hal/hal.hpp>

#include <opencv2/video/tracking.hpp>

#include "trackerModel.hpp"

// #include <opencv2/tracking.hpp>

// #include <opencv2/tracking/tracking_internals.hpp>

// namespace sky360lib::tracking
// {
//     namespace impl
//     {
//     }
//     using namespace impl;
//     //using namespace cv::detail::tracking;
// } // namespace

namespace sky360lib::tracking
{

    extern const float ColorNames[][10];

    /* Cholesky decomposition
     The function performs Cholesky decomposition <https://en.wikipedia.org/wiki/Cholesky_decomposition>.
     A - the Hermitian, positive-definite matrix,
     astep - size of row in A,
     asize - number of cols and rows in A,
     L - the lower triangular matrix, A = L*Lt.
    */

    template <typename _Tp>
    bool inline callHalCholesky(_Tp *L, size_t lstep, int lsize);

    template <>
    bool inline callHalCholesky<float>(float *L, size_t lstep, int lsize)
    {
        return cv::hal::Cholesky32f(L, lstep, lsize, NULL, 0, 0);
    }

    template <>
    bool inline callHalCholesky<double>(double *L, size_t lstep, int lsize)
    {
        return cv::hal::Cholesky64f(L, lstep, lsize, NULL, 0, 0);
    }

    template <typename _Tp>
    bool inline choleskyDecomposition(const _Tp *A, size_t astep, int asize, _Tp *L, size_t lstep)
    {
        bool success = false;

        astep /= sizeof(_Tp);
        lstep /= sizeof(_Tp);

        for (int i = 0; i < asize; i++)
            for (int j = 0; j <= i; j++)
                L[i * lstep + j] = A[i * astep + j];

        success = callHalCholesky(L, lstep * sizeof(_Tp), asize);

        if (success)
        {
            for (int i = 0; i < asize; i++)
                for (int j = i + 1; j < asize; j++)
                    L[i * lstep + j] = 0.0;
        }

        return success;
    }

}
