#include "condition_nodes.hpp"

#include <QAbstractButton>
#include <QTabWidget>

namespace bt_common_nodes
{
// WaitForGuiInput
WaitForGuiInput::WaitForGuiInput(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config), ok_(true)
{
    auto button_key = getBTInput<std::string>(this, BUTTON_PORT_KEY);
    auto* button = this->config().blackboard->get<QAbstractButton*>(button_key);

    QObject::connect(button, &QAbstractButton::clicked, [this, button_key](const bool) { ok_ = false; });
}

BT::NodeStatus WaitForGuiInput::tick()
{
    if (!ok_)
    {
        ok_ = true;
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

WaitForSpinBoxValueChange::WaitForSpinBoxValueChange(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config), value_changed_(false)
{
    auto spin_box_key = getBTInput<std::string>(this, SPIN_BOX_PORT_KEY);
    auto* spin_box = this->config().blackboard->get<QAbstractSpinBox*>(spin_box_key);
    if (auto* spin = qobject_cast<QSpinBox*>(spin_box))
    {
        QObject::connect(spin, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), [this](int) { value_changed_ = true; });
    }
    else if (auto* dspin = qobject_cast<QDoubleSpinBox*>(spin_box))
    {
        QObject::connect(dspin, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [this](double) { value_changed_ = true; });
    }
    else
    {
        throw std::runtime_error("Widget is not a QSpinBox or QDoubleSpinBox");
    }
}

BT::NodeStatus WaitForSpinBoxValueChange::tick()
{
    if (value_changed_)
    {
        value_changed_ = false;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

WaitForCheckBoxValueChange::WaitForCheckBoxValueChange(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config), value_changed_(false)
{
    auto check_box_key = getBTInput<std::string>(this, CHECK_BOX_PORT_KEY);
    auto* check_box = this->config().blackboard->get<QCheckBox*>(check_box_key);
    QObject::connect(check_box, &QCheckBox::stateChanged, [this](int) { value_changed_ = true; });
}

BT::NodeStatus WaitForCheckBoxValueChange::tick()
{
    if (value_changed_)
    {
        value_changed_ = false;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

WaitForGuiInputOnTick::WaitForGuiInputOnTick(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config), ok_(true)
{
    auto button_key = getBTInput<std::string>(this, BUTTON_PORT_KEY);
    auto* button = this->config().blackboard->get<QAbstractButton*>(button_key);

    QObject::connect(button, &QAbstractButton::clicked, [this, button_key](const bool) {
        if (running_) {
            ok_ = false;
        }
    });
}

BT::NodeStatus WaitForGuiInputOnTick::tick()
{
    running_ = true;
    if (!ok_)
    {
        ok_ = true;
        running_ = false;
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

CheckIntValueEquality::CheckIntValueEquality(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
{}

BT::NodeStatus CheckIntValueEquality::tick()
{
    int value1 = getBTInput<int>(this, VALUE1_PORT_KEY);
    int value2 = getBTInput<int>(this, VALUE2_PORT_KEY);

    if (value1 == value2) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}    


CheckBTStatus::CheckBTStatus(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
{}
BT::NodeStatus CheckBTStatus::tick()
{
    std::string status = getBTInput<std::string>(this, STATUS_PORT_KEY);
    if (status == "OK")
    {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;

}

CheckBTVariableValue::CheckBTVariableValue(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
{}

BT::NodeStatus CheckBTVariableValue::tick()
{
    std::string variable_name = getBTInput<std::string>(this, VARIABLE_NAME_PORT_KEY);
    std::string expected_value = getBTInput<std::string>(this, EXPECTED_VALUE_PORT_KEY);

    std::string actual_value = config().blackboard->get<std::string>(variable_name);
    if (variable_name == "skip_motion_approval_bool")
        std::cout << "CheckBTVariableValue: variable '" << variable_name << "' has value '" << actual_value << "', expected '" << expected_value << "'" << std::endl;
    if (actual_value == expected_value)
    {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

} // namespace bt_common_nodes