#pragma once

#include "bt_common_ros_headers.hpp"

// ROS Message Types
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

#ifdef HAS_MOTOMAN_SUPPORT
#include "motoros2_interfaces/srv/queue_traj_point.hpp"
#include "motoros2_interfaces/msg/queue_result_enum.hpp"
#include "motoros2_interfaces/srv/start_point_queue_mode.hpp"
#include "motoros2_interfaces/srv/read_single_io.hpp"
#include "motoros2_interfaces/srv/write_single_io.hpp"
#endif

#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "yaml_utils.hpp"

#include <QLabel>
#include <QSpinBox>


namespace bt_common_nodes 
{

/**
 * @brief Condition node for spinning the BT ROS node to keep it accessible for parameter updates, service calls, etc.
 */
class RosSpinnerNode : public BT::ConditionNode
{
public:
  inline static BT::PortsList providedPorts()
  {
      return {};
  }

  explicit RosSpinnerNode(const std::string& instance_name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

protected:
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;
};

/**
 * @brief The ExecuteMotionPlan class
 * Action Node that sends a JointTrajectory goal to a FollowJointTrajectory action server
 * and monitors the execution status.
 */
class ExecuteMotionPlan : public BT::RosActionNode<control_msgs::action::FollowJointTrajectory>
{
public:
  inline static std::string JOINT_TRAJECTORY_INPUT_PORT_KEY = "joint_trajectory";

  
  inline static BT::PortsList providedPorts()
  {
  return providedBasicPorts({ BT::InputPort<trajectory_msgs::msg::JointTrajectory>(JOINT_TRAJECTORY_INPUT_PORT_KEY)});
  }

  using RosActionNode<control_msgs::action::FollowJointTrajectory>::RosActionNode;

  bool setGoal(Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& response) override;
  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
};

/**
 * @brief The ReplaceStartState class
 * Subscriber Node that listens for the latest JointState message and replaces
 * the start state of a given JointTrajectory with the latest joint positions.
 */
class ReplaceStartState : public BT::RosTopicSubNode<sensor_msgs::msg::JointState>
{

public:
  inline static std::string MOTION_PLAN_PORT_KEY = "motion_plan";
  
  inline static BT::PortsList providedPorts()
  {
      return providedBasicPorts({ BT::BidirectionalPort<trajectory_msgs::msg::JointTrajectory>(MOTION_PLAN_PORT_KEY) });
  }

  using RosTopicSubNode<sensor_msgs::msg::JointState>::RosTopicSubNode;

  BT::NodeStatus onTick(const typename sensor_msgs::msg::JointState::SharedPtr& last_msg) override;
};

/**
 * @brief The CheckAtDestination class
 * Subscriber Node that listens for the latest JointState message and checks
 * if the robot has reached the final position of a given JointTrajectory.
 */
class CheckAtDestination : public BT::RosTopicSubNode<sensor_msgs::msg::JointState>
{
  public:
  inline static std::string MOTION_PLAN_PORT_KEY = "joint_trajectory";
  
  inline static BT::PortsList providedPorts()
  {
      return providedBasicPorts({ BT::BidirectionalPort<trajectory_msgs::msg::JointTrajectory>(MOTION_PLAN_PORT_KEY) });
  }

  using RosTopicSubNode<sensor_msgs::msg::JointState>::RosTopicSubNode;

  BT::NodeStatus onTick(const typename sensor_msgs::msg::JointState::SharedPtr& last_msg) override;
};

/**
 * @brief The ListenForTrajectory class
 * Subscriber Node that listens for JointTrajectory messages on a specified topic
 * and outputs the latest received message to a blackboard port.
 */
class ListenForTrajectory : public BT::RosTopicSubNode<trajectory_msgs::msg::JointTrajectory>
{
public:
  inline static std::string JOINT_TRAJECTORY_INPUT_PORT_KEY = "joint_trajectory";
  inline static std::string NUM_POINTS_OUTPUT_PORT_KEY = "num_points";
  
  inline static BT::PortsList providedPorts()
  {
      return providedBasicPorts({ BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(JOINT_TRAJECTORY_INPUT_PORT_KEY),
                                  BT::OutputPort<int>(NUM_POINTS_OUTPUT_PORT_KEY)});
  }

  using RosTopicSubNode<trajectory_msgs::msg::JointTrajectory>::RosTopicSubNode;

  BT::NodeStatus onTick(const typename trajectory_msgs::msg::JointTrajectory::SharedPtr& last_msg) override;
};

#ifdef HAS_PCL_SUPPORT
/**
 * @brief The ListenForPointCloud class
 * Subscriber Node that listens for PointCloud2 messages on a specified topic
 * and outputs the latest received message to a blackboard port.
 */
class ListenForPointCloud : public BT::RosTopicSubNode<sensor_msgs::msg::PointCloud2>
{
public:
  inline static std::string POINT_CLOUD_INPUT_PORT_KEY = "point_cloud";

  inline static BT::PortsList providedPorts()
  {
      return providedBasicPorts({BT::OutputPort<sensor_msgs::msg::PointCloud2>(POINT_CLOUD_INPUT_PORT_KEY)});
  }

  using RosTopicSubNode<sensor_msgs::msg::PointCloud2>::RosTopicSubNode;

  BT::NodeStatus onTick(const typename sensor_msgs::msg::PointCloud2::SharedPtr& last_msg) override;
};
#endif

#ifdef HAS_MOTOMAN_SUPPORT
/**
 * @brief The QueueTrajPoint class
 * Service Node that queues the next JointTrajectoryPoint from a queue into the motoros2 controller
 * using the QueueTrajPoint service.
 */
class QueueTrajPoint : public BT::RosServiceNode<motoros2_interfaces::srv::QueueTrajPoint>
{
public:
  inline static std::string POINT_QUEUE_INPUT_PORT_KEY = "point_queue";
  inline static std::string JOINT_TRAJECTORY_INPUT_PORT_KEY = "joint_trajectory";

  
  inline static BT::PortsList providedPorts()
  {
      return providedBasicPorts({ BT::InputPort<std::shared_ptr<std::queue<trajectory_msgs::msg::JointTrajectoryPoint>>>(POINT_QUEUE_INPUT_PORT_KEY),
                                  BT::InputPort<trajectory_msgs::msg::JointTrajectory>(JOINT_TRAJECTORY_INPUT_PORT_KEY)});
  }

  using RosServiceNode<motoros2_interfaces::srv::QueueTrajPoint>::RosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};
#endif

#ifdef HAS_MOTOMAN_SUPPORT
/**
 * @brief The StartPointQueueMode class
 * Service Node that starts the point queue execution mode in the motoros2 controller.
 */
class StartPointQueueMode : public BT::RosServiceNode<motoros2_interfaces::srv::StartPointQueueMode>
{
public:

  inline static BT::PortsList providedPorts()
  {
  return providedBasicPorts({});
  }

  using RosServiceNode<motoros2_interfaces::srv::StartPointQueueMode>::RosServiceNode;
  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};
#endif

#ifdef HAS_MOTOMAN_SUPPORT
/**
 * @brief Reads a single IO point from the motoros2 controller.
 * Service Node that reads the value of a specified IO point using the ReadSingleIO service.
 * Outputs the read value to a blackboard port.
 */
class ReadSingleIO : public BT::RosServiceNode<motoros2_interfaces::srv::ReadSingleIO>
{
public:
  inline static std::string IO_NAME_INPUT_PORT_KEY = "io_address";
  inline static std::string IO_VALUE_OUTPUT_PORT_KEY = "io_value";

  inline static BT::PortsList providedPorts()
  {
      return providedBasicPorts({ BT::InputPort<int>(IO_NAME_INPUT_PORT_KEY),
                                  BT::OutputPort<int>(IO_VALUE_OUTPUT_PORT_KEY)});
  }
  using RosServiceNode<motoros2_interfaces::srv::ReadSingleIO>::RosServiceNode;
  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

/**
 * @brief Writes a single IO point to the motoros2 controller.
 * Service Node that writes a specified value to an IO point using the WriteSingleIO service.
 */
class WriteSingleIO : public BT::RosServiceNode<motoros2_interfaces::srv::WriteSingleIO>
{
public:
  inline static std::string IO_NAME_INPUT_PORT_KEY = "io_address";
  inline static std::string IO_VALUE_INPUT_PORT_KEY = "io_value"; 
  inline static BT::PortsList providedPorts()
  {
      return providedBasicPorts({ BT::InputPort<int>(IO_NAME_INPUT_PORT_KEY),
                                  BT::InputPort<int>(IO_VALUE_INPUT_PORT_KEY)});
  }
  using RosServiceNode<motoros2_interfaces::srv::WriteSingleIO>::RosServiceNode;
  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};  
#endif

/**
 * @brief The StopTrajMode class
 * Service Node that stops any ongoing trajectory execution in the motoros2 controller.
 */
class StopTrajMode : public BT::RosServiceNode<std_srvs::srv::Trigger>
{
public:

  inline static BT::PortsList providedPorts()
  {
  return providedBasicPorts({});
  }

  using RosServiceNode<std_srvs::srv::Trigger>::RosServiceNode;
  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

/**
 * @brief The PubTrajectoryPreview class
 * Publisher Node that publishes a JointTrajectory message to a specified topic for visualization or monitoring.
 */
class PubTrajectoryPreview : public BT::RosTopicPubNode<trajectory_msgs::msg::JointTrajectory>
{
public:
  inline static std::string TRAJECTORY_KEY = "joint_trajectory";

  inline static BT::PortsList providedPorts()
  {
      return providedBasicPorts({ BT::InputPort<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_KEY) });
  }

  PubTrajectoryPreview(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params);
      
  bool setMessage(trajectory_msgs::msg::JointTrajectory& msg) override;

  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> plan_;
};

} // namespace bt_common_nodes