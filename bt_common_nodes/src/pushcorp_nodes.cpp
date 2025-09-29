#include "pushcorp_nodes.hpp"

namespace bt_common_nodes 
{

bool SetSpindleEnable::setRequest(typename Request::SharedPtr& request)
{
  request->data = getBTInput<bool>(this, VELOCITY_ENABLE_INPUT_PORT_KEY);
  return true;
}

BT::NodeStatus SetSpindleEnable::onResponseReceived(const typename Response::SharedPtr& response)
{
  if (response->success)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

#ifdef HAS_CAPSEN_SUPPORT
bool SetSpindleVelocity::setRequest(typename Request::SharedPtr& request)
{
  request->rpm = getBTInput<int32_t>(this, VELOCITY_RPM_INPUT_PORT_KEY);
  return true;
}

BT::NodeStatus SetSpindleVelocity::onResponseReceived(const typename Response::SharedPtr& response)
{
  if (response->success)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("plg_bt"), "Set Spindle Velocity Failed");
    return BT::NodeStatus::FAILURE;
  }
}
#endif

BT::NodeStatus ListenForActualRPM::onTick(const typename std_msgs::msg::Int32::SharedPtr& last_msg)
{
  if (last_msg)
  {
    setOutput(ACTUAL_RPM_OUTPUT_PORT_KEY, last_msg->data);
    auto label_name = getBTInput<std::string>(this, RPM_LABEL_NAME_KEY);
    auto label = this->config().blackboard->get<QLabel*>(label_name);
    if (label) {
      label->setText(QString::number(last_msg->data));
    }
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

#ifdef HAS_PUSHCORP_SUPPORT
bool AFDSetControlMode::setRequest(typename Request::SharedPtr& request)
{
  auto mode = getBTInput<int>(this, CONTROL_MODE_INPUT_PORT_KEY);
  request->mode=mode;
  RCLCPP_INFO(logger(), "Request set for AFDSetControlMode to %d", mode);
  return true;
}

BT::NodeStatus AFDSetControlMode::onResponseReceived(const typename Response::SharedPtr& response)
{
  if (response->success)
  {
    RCLCPP_INFO(logger(), "AFD Control Mode set successfully");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_ERROR(logger(), "Failed to set AFD Control Mode");
    return BT::NodeStatus::FAILURE;
  }
}
#endif

#ifdef HAS_PUSHCORP_SUPPORT
bool AFDCommandForceValue::setRequest(typename Request::SharedPtr& request)
{
  request->value = getBTInput<int>(this, FORCE_VALUE_INPUT_PORT_KEY);
  RCLCPP_INFO(logger(), "Request set for AFDCommandForceValue to %f", request->value);
  return true;
}

BT::NodeStatus AFDCommandForceValue::onResponseReceived(const typename Response::SharedPtr& response)
{
  if (response->success)
  {
    RCLCPP_INFO(logger(), "AFD Command Force Value set successfully");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_ERROR(logger(), "Failed to set AFD Command Force Value");
    return BT::NodeStatus::FAILURE;
  }
}
#endif

#ifdef HAS_PUSHCORP_SUPPORT
BT::NodeStatus SubAFDCurrentForce::onTick(const typename pushcorp_msgs::msg::Value::SharedPtr& last_msg)
{
  if (last_msg)
  {
    auto gui_force_label_name = getBTInput<std::string>(this, CURRENT_FORCE_GUI_LABEL_PORT_KEY);
    auto label = this->config().blackboard->get<QLabel*>(gui_force_label_name);
    if (label) {
      label->setText(QString::number(last_msg->value));
    }
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}
#endif

} // namespace bt_common_nodes