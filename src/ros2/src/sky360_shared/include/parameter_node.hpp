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
        : Node(node_name, *default_options())
    {
        m_parameters_callback_handle = add_on_set_parameters_callback(std::bind(&ParameterNode::param_change_callback_method, this, std::placeholders::_1));
    }

protected:
    virtual void set_parameters_callback(const std::vector<rclcpp::Parameter> &parameters_to_set) = 0;

    void declare_parameters(const std::vector<rclcpp::Parameter> &params)
    {
        for (const auto &param : params)
        {
            declare_parameter(param.get_name(), param.get_parameter_value());
        }
    }

private:
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_parameters_callback_handle;

    rcl_interfaces::msg::SetParametersResult param_change_callback_method(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        set_parameters_callback(parameters);
        return result;
    }

    static std::shared_ptr<rclcpp::NodeOptions> default_options()
    {
        auto options = std::make_shared<rclcpp::NodeOptions>();
        options->use_intra_process_comms(true);

        return options;
    }
};

#endif // __PARAMETER_NODE_H__