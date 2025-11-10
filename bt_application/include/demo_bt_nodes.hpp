#pragma once

#include <atomic>

// behavior tree common node headers import
// pulls in all the common behavior tree nodes for ROS2
#include <bt_common_ros_headers.hpp>


namespace example_bt_nodes 
{
  /**
   * Your custom behavior tree nodes can be defined here.
   * Remember to register them with the factory in the ExampleWidget class in example_widget.cpp
   * 
   */

   /**
      // Example of a custom synchronous action node
      class ExampleBTNode : public BT::SyncActionNode
      {
          public:
            inline static std::string ExampleInputKey = "example_input";
            inline static std::string ExampleOutputKey = "example_output";
            
            // Constructor
            ExampleBTNode(const std::string& name, const BT::NodeConfiguration& config)
              : BT::SyncActionNode(name, config) 
            {
              // Initialization code here
            }

            // This method is used to define the ports of the node
            static BT::PortsList providedPorts()
            {
              return{ BT::InputPort<std::string>(ExampleInputKey),
                      BT::OutputPort<std::string>(ExampleOutputKey) };
            }

            // This is the main method that is called when the node is ticked
            BT::NodeStatus tick() override
            {
              // Node execution code here
              return BT::NodeStatus::SUCCESS;
            }
        };
    */

}
