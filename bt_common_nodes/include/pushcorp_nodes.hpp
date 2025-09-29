#pragma once

#include <atomic>

#include "bt_common_ros_headers.hpp"

// ROS Message Types
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "plg_msgs/srv/set_spindle_velocity.hpp"
#include "pushcorp_msgs/srv/set_control_mode.hpp"
#include "pushcorp_msgs/srv/command_value.hpp"
#include "pushcorp_msgs/msg/value.hpp"

namespace bt_common_nodes 
{

/**
 * @brief The SetVelocityEnable class
 * Service Node that enables or disables the spindle velocity.
 */
class SetSpindleEnable : public BT::RosServiceNode<std_srvs::srv::SetBool>
{
  public:
    inline static std::string VELOCITY_ENABLE_INPUT_PORT_KEY = "value";

    inline static BT::PortsList providedPorts()
    {
    return providedBasicPorts({ BT::InputPort<bool>(VELOCITY_ENABLE_INPUT_PORT_KEY) });
    }

    using RosServiceNode<std_srvs::srv::SetBool>::RosServiceNode;

    bool setRequest(typename Request::SharedPtr& request) override;
    BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

#ifdef HAS_CAPSEN_SUPPORT
/**
 * @brief The SetSpindleVelocity class
 * Service Node that sets the spindle velocity in RPM.
 */
class SetSpindleVelocity : public BT::RosServiceNode<plg_msgs::srv::SetSpindleVelocity>
{
  public:
    inline static std::string VELOCITY_RPM_INPUT_PORT_KEY = "rpm";

    inline static BT::PortsList providedPorts()
    {
    return providedBasicPorts({ BT::InputPort<int32_t>(VELOCITY_RPM_INPUT_PORT_KEY) });
    }

    using RosServiceNode<plg_msgs::srv::SetSpindleVelocity>::RosServiceNode;

    bool setRequest(typename Request::SharedPtr& request) override;
    BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};
#endif

/**
 * @brief The ListenForActualRPM class
 * Subscriber Node that listens for the actual RPM of the spindle and updates a GUI label.
 */
class ListenForActualRPM : public BT::RosTopicSubNode<std_msgs::msg::Int32>
{
  public:
    inline static std::string ACTUAL_RPM_OUTPUT_PORT_KEY = "actual_rpm";
    inline static std::string RPM_LABEL_NAME_KEY = "gui_label_name";
    inline static BT::PortsList providedPorts()
    {
        return providedBasicPorts({BT::OutputPort<int>(ACTUAL_RPM_OUTPUT_PORT_KEY),
        BT::InputPort<std::string>(RPM_LABEL_NAME_KEY)});
    }

    using RosTopicSubNode<std_msgs::msg::Int32>::RosTopicSubNode;

    BT::NodeStatus onTick(const typename std_msgs::msg::Int32::SharedPtr& last_msg) override;
}; 

#ifdef HAS_PUSHCORP_SUPPORT
/**
 * @brief The AFDSetControlMode class
 * Service Node that sets the control mode of the AFD (position or force control).
 */
class AFDSetControlMode : public BT::RosServiceNode<pushcorp_msgs::srv::SetControlMode>
{
public:
  inline static std::string CONTROL_MODE_INPUT_PORT_KEY = "control_mode";  // 0 - position, 1 - force
  inline static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<int>(CONTROL_MODE_INPUT_PORT_KEY) });
  }
  using RosServiceNode<pushcorp_msgs::srv::SetControlMode>::RosServiceNode;
  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};
#endif

#ifdef HAS_PUSHCORP_SUPPORT
/**
 * @brief The AFDCommandForceValue class
 * Service Node that commands a force value to the AFD.
 */
class AFDCommandForceValue : public BT::RosServiceNode<pushcorp_msgs::srv::CommandValue>
{
public:
  inline static std::string FORCE_VALUE_INPUT_PORT_KEY = "force_value";  // force value to be commanded
  inline static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<int>(FORCE_VALUE_INPUT_PORT_KEY) });
  }
  using RosServiceNode<pushcorp_msgs::srv::CommandValue>::RosServiceNode;
  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};
#endif

#ifdef HAS_PUSHCORP_SUPPORT
/**
 * @brief The SubAFDCurrentForce class
 * Subscriber Node that listens for the current force value from the AFD and updates a GUI label.
 */
class SubAFDCurrentForce : public BT::RosTopicSubNode<pushcorp_msgs::msg::Value>
{
public:
  inline static std::string CURRENT_FORCE_GUI_LABEL_PORT_KEY = "current_force_label";

  inline static BT::PortsList providedPorts()
  {
      return providedBasicPorts({ BT::InputPort<std::string>(CURRENT_FORCE_GUI_LABEL_PORT_KEY) });
  }
  using RosTopicSubNode<pushcorp_msgs::msg::Value>::RosTopicSubNode;
  BT::NodeStatus onTick(const typename pushcorp_msgs::msg::Value::SharedPtr& last_msg) override;
};
#endif
 
} // namespace bt_common_nodes