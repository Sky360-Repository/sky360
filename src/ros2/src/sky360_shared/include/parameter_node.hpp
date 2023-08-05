#pragma once
#ifndef __PARAMETER_NODE_H__
#define __PARAMETER_NODE_H__

#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

class ParameterNode
    : public rclcpp::Node
{
public:
    struct ActionParam 
    {
        rclcpp::Parameter parameter;
        std::function<void(const rclcpp::Parameter&)> action;

        ActionParam(const rclcpp::Parameter &_param,
                    std::function<void(const rclcpp::Parameter&)> _action) 
            : parameter(_param), action(_action) 
        {}
    };

    ParameterNode(const std::string &node_name)
        : Node(node_name, *default_options())
    {
        parameters_callback_handle_ = add_on_set_parameters_callback(std::bind(&ParameterNode::param_change_callback_method, this, std::placeholders::_1));
    }

protected:
    void add_action_parameters(const std::vector<ActionParam>& action_params)
    {
        for (auto & action_param : action_params)
        {
            parameters_map_.insert_or_assign(action_param.parameter.get_name(), action_param);
            declare_parameter(action_param.parameter.get_name(), action_param.parameter.get_parameter_value());
        }
    }

    void update_action_param(const rclcpp::Parameter &_param)
    {
        auto it = parameters_map_.find(_param.get_name());
        if (it != parameters_map_.end() && it->second.action != nullptr)
        {
            it->second.action(_param);
        }
    }

    static std::string generate_uuid()
    {
        boost::uuids::random_generator uuid_generator;
        return boost::uuids::to_string(uuid_generator());
    }

private:
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
    std::map<std::string, ActionParam> parameters_map_;

    rcl_interfaces::msg::SetParametersResult param_change_callback_method(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (auto &param : parameters)
        {
            update_action_param(param);
        }
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