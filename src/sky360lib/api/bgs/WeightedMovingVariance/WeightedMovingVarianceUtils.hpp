#pragma once

#include "../CoreParameters.hpp"

namespace sky360lib::bgs
{
    class WMVParams 
        : public CoreParameters
    {
    public:
        static inline const bool DEFAULT_ENABLE_WEIGHT{true};
        static inline const bool DEFAULT_ENABLE_THRESHOLD{true};
        static inline const float DEFAULT_THRESHOLD_VALUE{30.0f};
        static inline const float DEFAULT_WEIGHTS[] = {0.5f, 0.3f, 0.2f};
        static inline const float ONE_THIRD{1.0f / 3.0f};

        WMVParams()
            : WMVParams(DEFAULT_ENABLE_WEIGHT, 
                DEFAULT_ENABLE_THRESHOLD, 
                DEFAULT_THRESHOLD_VALUE,
                DEFAULT_ENABLE_WEIGHT ? DEFAULT_WEIGHTS[0] : ONE_THIRD, 
                DEFAULT_ENABLE_WEIGHT ? DEFAULT_WEIGHTS[1] : ONE_THIRD, 
                DEFAULT_ENABLE_WEIGHT ? DEFAULT_WEIGHTS[2] : ONE_THIRD)
        {
        }

        WMVParams(bool _enableWeight,
                bool _enableThreshold,
                float _threshold,
                float _weight1,
                float _weight2,
                float _weight3)
            : CoreParameters()
            , weight{_enableWeight ? _weight1 : ONE_THIRD, 
                _enableWeight ? _weight2 : ONE_THIRD, 
                _enableWeight ? _weight3 : ONE_THIRD}
        {
            set_enable_threshold(_enableThreshold);
            set_enable_weight(_enableWeight);
            set_threshold(_threshold);
        }

        WMVParams(const WMVParams& _params)
            : CoreParameters()
            , weight{_params.weight[0], _params.weight[1], _params.weight[2]}
        {
            set_enable_threshold(_params.enable_threshold);
            set_enable_weight(_params.enable_weight);
            set_threshold(_params.threshold);
        }

        // WMVParams& operator=(const WMVParams& _params)
        // {
        //     enableWeight = _params.enableWeight;
        //     enableThreshold = _params.enableThreshold;
        //     threshold = _params.threshold;
        //     threshold16 = _params.threshold16;
        //     weight[0] = _params.weight[0];
        //     weight[1] = _params.weight[1];
        //     weight[2] = _params.weight[2];
        //     thresholdSquared = _params.thresholdSquared;
        //     thresholdSquared16 = _params.thresholdSquared16;
        //     setMap();
        //     return *this;
        // }

        float get_threshold() { return threshold; }
        float* get_weights() { return weight; }
        bool get_enable_weight() { return enable_weight; }
        bool get_enable_threshold() { return enable_threshold; }

        void set_enable_weight(bool value)
        { 
            enable_weight = value; 
        }
        void set_enable_threshold(bool value)
        { 
            enable_threshold = value; 
        }
        void set_weights(int _weight, float _value)
        {
            if (_weight >= 0 && _weight <= 3)
            {
                weight[_weight] = _value; 
            }
        }
        void set_threshold(float _value) 
        { 
            threshold = _value;
            threshold16 = threshold * 256.0f;
            threshold_squared = threshold * threshold;
            threshold_squared16 = threshold16 * threshold16;
        }

        friend class WeightedMovingVariance;
        friend class WeightedMovingVarianceCL;        

    protected:
        bool enable_weight;
        bool enable_threshold;
        float threshold;
        float threshold16;
        float weight[3];
        float threshold_squared;
        float threshold_squared16;
    };
}
