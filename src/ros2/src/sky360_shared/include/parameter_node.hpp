#pragma once
#ifndef __PARAMETER_NODE_H__
#define __PARAMETER_NODE_H__

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

class ParameterNode
    : public rclcpp::Node
{
public:
    ParameterNode(const std::string &node_name)
        : Node(node_name)
        , enable_profiling_(false)
    {
        declare_parameters();

        m_parameters_callback_handle = add_on_set_parameters_callback(std::bind(&ParameterNode::param_change_callback_method, this, std::placeholders::_1));
    }

    void declare_parameters(const std::vector<rclcpp::Parameter> &params)
    {
        for (const auto &param : params)
        {
            declare_parameter(param.get_name(), param.get_parameter_value());
        }
    }

protected:
    virtual void set_parameters_callback(const std::vector<rclcpp::Parameter> &parameters_to_set) = 0;

    virtual void declare_parameters()
    {
        std::vector<rclcpp::Parameter> params = {
            rclcpp::Parameter("enable_profiling", false)
        };
        declare_parameters(params);

        enable_profiling_ = get_parameter("enable_profiling").get_value<rclcpp::ParameterType::PARAMETER_BOOL>();
    }

    rcl_interfaces::msg::SetParametersResult param_change_callback_method(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &param : parameters)
        {
            if (param.get_name() == "enable_profiling")
            {
                enable_profiling_ = param.as_bool();
            }
        }
        set_parameters_callback(parameters);
        return result;
    }

    bool enable_profiling_;

private:
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_parameters_callback_handle;
};

#endif // __PARAMETER_NODE_H__