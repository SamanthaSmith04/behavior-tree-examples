#include "example_widget.hpp"

#include "ui_example_gui.h" // will always be "ui_" + <ui file name> + ".h"

namespace example_widget
{

ExampleWidget::ExampleWidget(rclcpp::Node::SharedPtr ros_node, QWidget *parent)
    : QWidget(parent), ros_node_(ros_node), ui_(new Ui::ExampleGUI)
{

  // find the file for the behavior tree configuration
  std::string package_path = ament_index_cpp::get_package_share_directory("bt_application");
  std::string gui_bt_file = package_path + "/config/example_bt.xml";  

  // initialize the UI for the GUI
  ui_->setupUi(this);
}

ExampleWidget::~ExampleWidget()
{
  delete ui_;
}

} // namespace example_widget