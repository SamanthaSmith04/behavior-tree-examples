#pragma once

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/blackboard.h"
#include "rclcpp/node.hpp"
#include "QWidget"


namespace Ui {
  class ExampleGUI;
}
namespace example_widget 
{

class ExampleWidget : public QWidget
{

  Q_OBJECT

  public:
    explicit ExampleWidget(rclcpp::Node::SharedPtr ros_node, QWidget *parent = nullptr);

    ~ExampleWidget();

  private:
    rclcpp::Node::SharedPtr ros_node_;
    Ui::ExampleGUI* ui_;
    BT::Tree tree;
    BT::Blackboard::Ptr blackboard;
};

} // namespace example_widget