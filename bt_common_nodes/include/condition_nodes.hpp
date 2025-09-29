#pragma once

#include <atomic>

#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/condition_node.h"

#include "utils.hpp"

namespace bt_common_nodes 
{
/**
 * @brief Condition node that monitors the state of a Qt button.
 * @details A pointer to the monitored button (QAbstractButton*) is provided via the blackboard using the key defined in
 * the input port. When the button is clicked, the node returns failure the next time it is checked.
 */
class WaitForGuiInput : public BT::ConditionNode
{
  public:
    inline static std::string BUTTON_PORT_KEY = "button";
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>(BUTTON_PORT_KEY) };
    }

    WaitForGuiInput(const std::string& name, const BT::NodeConfig& config);

  protected:
    BT::NodeStatus tick() override;

    std::atomic_bool ok_;
};

/**
 * @brief Condition node that monitors the state of a Qt button.
 * @details A pointer to the monitored button (QAbstractButton*) is provided via the blackboard using the key defined in
 * the input port. Will only poll button clicking when the node is actively being ticked.
 */
class WaitForGuiInputOnTick : public BT::ConditionNode
{
  public:
    inline static std::string BUTTON_PORT_KEY = "button";
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>(BUTTON_PORT_KEY) };
    }

    WaitForGuiInputOnTick(const std::string& name, const BT::NodeConfig& config);

  protected:
    BT::NodeStatus tick() override;

    std::atomic_bool ok_;
    std::atomic_bool running_;
};

/**
 * @brief Condition node that checks if two integer input values are equal.
 * @details The two integer values are provided via the blackboard using the keys defined in the input ports.
 * If the values are equal, the node returns SUCCESS; otherwise, it returns FAILURE.
 */
class CheckIntValueEquality : public BT::ConditionNode
{
  public:
    inline static std::string VALUE1_PORT_KEY = "value_1";
    inline static std::string VALUE2_PORT_KEY = "value_2";

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<int>(VALUE1_PORT_KEY),
               BT::InputPort<int>(VALUE2_PORT_KEY) };
    }

    CheckIntValueEquality(const std::string& name, const BT::NodeConfig& config);
  protected:
    BT::NodeStatus tick() override;
};

/**
 * @brief Condition node that checks if a specified Behavior Tree variable matches an expected value.
 * @details The variable name and expected value are provided via the blackboard using the keys defined in the input ports.
 * If the current value of the variable matches the expected value, the node returns SUCCESS; otherwise, it returns FAILURE.
 */
class CheckBTStatus : public BT::ConditionNode
{
  public:
    inline static std::string STATUS_PORT_KEY = "bt_status";
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>(STATUS_PORT_KEY) };
    }

    CheckBTStatus(const std::string& name, const BT::NodeConfig& config);

  protected:
    BT::NodeStatus tick() override;
};

/**
 * @brief Condition node that checks if a specified Behavior Tree variable matches an expected value.
 * This is done as a STRING COMPARISON.
 * @details The variable name and expected value are provided via the blackboard using the keys defined in the input ports.
 * If the current value of the variable matches the expected value, the node returns SUCCESS; otherwise, it returns FAILURE.
 */
class CheckBTVariableValue : public BT::ConditionNode
{
  public:
    inline static std::string VARIABLE_NAME_PORT_KEY = "variable_name";
    inline static std::string EXPECTED_VALUE_PORT_KEY = "expected_value";
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>(VARIABLE_NAME_PORT_KEY),
                 BT::InputPort<std::string>(EXPECTED_VALUE_PORT_KEY) };
    }
    CheckBTVariableValue(const std::string& name, const BT::NodeConfig& config);
  protected:
    BT::NodeStatus tick() override;
}; 

} // namespace bt_common_nodes