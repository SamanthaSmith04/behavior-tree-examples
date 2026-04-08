#include "example_widget.hpp"

// behavior tree common node headers
// #include <action_nodes.hpp>
#include <condition_nodes.hpp>
#include <bt_ros_nodes.hpp>
#include <qt_nodes.hpp>

// Your custom behavior tree nodes
#include <demo_bt_nodes.hpp>

#include <QTimer>

#include "ui_example_gui.h" // will always be "ui_" + <ui file name> + ".h"

namespace example_widget
{

ExampleWidget::ExampleWidget(rclcpp::Node::SharedPtr ros_node, QWidget *parent)
    : QWidget(parent), ros_node_(ros_node), ui_(new Ui::ExampleGUI)
{

  // find the file for the behavior tree configuration
  std::string package_path = ament_index_cpp::get_package_share_directory("bt_application");
  std::string gui_bt_file = package_path + "/config/example_bt.xml";  
 
  // Set up the QT UI components
  ui_->setupUi(this);

  // initialize the UI for the GUI
  blackboard = BT::Blackboard::create();

  // SET Blackboard values here
  blackboard->set("main_gui", static_cast<QWidget*>(this));
  blackboard->set("example_button", static_cast<QWidget*>(ui_->example_button));
  blackboard->set("example_text_edit", static_cast<QWidget*>(ui_->example_text_edit));
  blackboard->set("example_double_spin_box", static_cast<QWidget*>(ui_->example_double_spin_box));
  blackboard->set("example_check_box", static_cast<QWidget*>(ui_->example_check_box));
  blackboard->set("example_slider", static_cast<QWidget*>(ui_->example_slider));

  // Register custom nodes in the factory if any
  BT::BehaviorTreeFactory factory;
  // Example formatting to register a node:
  factory.registerNodeType<bt_common_nodes::WaitForGuiInputEvent>("WaitForGuiInputEvent");
  factory.registerNodeType<bt_common_nodes::WaitForGuiInputEventOnTick>("WaitForGuiInputEventOnTick");
  factory.registerNodeType<bt_common_nodes::GetValueFromGui>("GetValueFromGui");
  factory.registerNodeType<bt_common_nodes::SetGuiValue>("SetGuiValue");
  factory.registerNodeType<bt_common_nodes::OpenFileDialog>("OpenFileDialog");

  // Example formatting to register a ROS node:
  // auto ros_node_params = BT::RosNodeParams(ros_node, "topic_name"); 
  // factory.registerNodeType<ROSBTNodeName>("ROSBTNodeName", ros_node_params);

  // Create the Behavior Tree
  this->tree = factory.createTreeFromFile(gui_bt_file, blackboard);

  // Do any other GUI / BT setup here

  // Start ticking Behavior Tree
  QTimer *tree_timer = new QTimer(this);
  QObject::connect(tree_timer, &QTimer::timeout, [this]() {
      BT::NodeStatus status = this->tree.tickOnce();
  });
  tree_timer->start(10); // Tick every 10 ms

}

ExampleWidget::~ExampleWidget()
{
  delete ui_;
}

} // namespace example_widget