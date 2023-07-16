#pragma once

#include <opencv2/opencv.hpp>
#include "PID.h"
#include <functional> 
#include <limits>

namespace sky360lib::utils
{
    class AutoExposure 
    {
    public:
        AutoExposure(double targetBrightness, double kp, double ki, double kd)
            : m_target_msv(targetBrightness),
            m_min_target_msv(0.05),
            m_max_target_msv(0.25),
            m_max_exposure(60000),
            m_min_exposure(100), 
            m_max_gain(25),
            m_min_gain(0),
            m_gain_accumulator(0.0),
            m_is_night(false),
            m_pid_controller(kp, ki, kd, [this] { return m_current_msv; }, [](double){})
        {
            m_pid_controller.setTarget(m_target_msv);
            m_pid_controller.setMaxIntegralCumulation(500);
        }

        double get_target_msv() const { return m_target_msv; }
        double get_current_msv() const { return m_current_msv; }
        bool is_day() const { return !m_is_night; }

        void set_target_msv(double target_msv)
        {
            m_target_msv = target_msv;
            m_pid_controller.setTarget(m_target_msv);
        }

        void update(double msv, double& exposure, double& gain)
        {
            m_current_msv = msv;
            m_pid_controller.tick();
            double pidOutput = m_pid_controller.getOutput();
            double error = m_pid_controller.getError();

            // std::cout << "pidOutput: " << pidOutput << ", error: " << error << "max: " << m_pid_controller.getMaxIntegralCumulation() << std::endl;

            double widerErrorMargin = 0.04;
            double initialErrorMargin = 0.001;
            double currentErrorMargin = (std::abs(error) <= initialErrorMargin) ? widerErrorMargin : initialErrorMargin;
            double gain_weight = 0.3; 

            if (std::abs(error) > currentErrorMargin)
            {
                if (error > 0) // Light is decreasing
                {
                    if (exposure < m_max_exposure) // Adjust exposure first
                    {
                        exposure = std::clamp(exposure + pidOutput, m_min_exposure, m_max_exposure);
                    }
                    else if(m_target_msv > m_min_target_msv) // Then relax target
                    {
                        m_target_msv = std::clamp(m_target_msv - 0.001, m_min_target_msv, m_max_target_msv);
                        m_pid_controller.setTarget(m_target_msv);
                    }
                    else // Finally adjust gain
                    {
                        m_gain_accumulator += gain_weight * error; 

                        if (m_gain_accumulator >= 1.0) 
                        {
                            gain = std::clamp(gain + 1, m_min_gain, m_max_gain); 
                            m_gain_accumulator -= 1.0; 
                        }
                    }
                }
                else // Light is increasing
                {
                    if (gain > m_min_gain) // Reduce gain first
                    {
                        m_gain_accumulator -= gain_weight * std::abs(error); 

                        if (m_gain_accumulator <= -1.0) 
                        {
                            gain = std::clamp(gain - 1, m_min_gain, m_max_gain); 
                            m_gain_accumulator += 1.0; 
                        }
                    }
                    else if(m_target_msv < m_max_target_msv) // Increase target
                    {
                        m_target_msv = std::clamp(m_target_msv + 0.001, m_min_target_msv, m_max_target_msv);
                        m_pid_controller.setTarget(m_target_msv);
                    }
                    else // Finally decrease exposure
                    {
                        exposure = std::clamp(exposure + pidOutput, m_min_exposure, m_max_exposure);
                    }
                }
            }

            m_is_night = (exposure >= m_max_exposure) /*&& (gain > m_min_gain)*/;
        }

    private:
        double m_target_msv;
        double m_min_target_msv;
        double m_max_target_msv;
        double m_max_exposure;
        double m_min_exposure;
        double m_max_gain;
        double m_min_gain;
        double m_current_msv;
        double m_gain_accumulator;
        bool m_is_night;
        PIDController<double> m_pid_controller;
    };
    
}
