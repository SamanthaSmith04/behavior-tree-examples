#pragma once

#include <atomic>

#include "bt_common_ros_headers.hpp"

// ROS Message Types
#include "plg_msgs/msg/path_parameters.hpp"
#include "plg_msgs/msg/tool_path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "plg_msgs/srv/generate_grind_motion_plan.hpp"
#include "plg_msgs/srv/generate_path_plan.hpp"
#include "plg_msgs/srv/transfer_scan_data.hpp"

#include "capsen_planning/srv/generate_trajectory.hpp"
#include "capsen_planning/msg/grinding_command.hpp"

namespace bt_common_nodes 
{

#ifdef HAS_PCL_SUPPORT
/**
 * @brief The RequestPointCloud class
 * Service Node that requests a point cloud from the scanning service.
 */
class RequestPointCloud : public BT::RosServiceNode<plg_msgs::srv::TransferScanData>
{
public:
  inline static std::string POINT_CLOUD_OUTPUT_PORT_KEY = "point_cloud";
  inline static std::string REQUEST_INPUT_PORT_KEY = "request";
  
  inline static BT::PortsList providedPorts()
  {
  return providedBasicPorts({ BT::OutputPort<sensor_msgs::msg::PointCloud2>(POINT_CLOUD_OUTPUT_PORT_KEY)});
  }
  
  using RosServiceNode<plg_msgs::srv::TransferScanData>::RosServiceNode;
  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};
#endif 

#ifdef HAS_CAPSEN_SUPPORT
/**
 * @brief The RequestPathPlan class
 * Service Node that requests a grinding path plan based on point cloud data and path parameters.
 */
class RequestPathPlan : public BT::RosServiceNode<plg_msgs::srv::GeneratePathPlan>
{
public:
  inline static std::string FLASH_POINT_CLOUD_INPUT_PORT_KEY = "flash_point_cloud";
  inline static std::string PATH_PARAMETERS_INPUT_PORT_KEY = "path_parameters";
  inline static std::string EXPECTED_PART_POINT_CLOUD_INPUT_PORT_KEY = "expected_part_point_cloud";
  inline static std::string TOOL_PATH_OUTPUT_PORT_KEY = "tool_path";
  
  inline static BT::PortsList providedPorts()
  {
  return providedBasicPorts({ BT::InputPort<sensor_msgs::msg::PointCloud2>(FLASH_POINT_CLOUD_INPUT_PORT_KEY),
                              BT::InputPort<plg_msgs::msg::PathParameters>(PATH_PARAMETERS_INPUT_PORT_KEY),
                              BT::InputPort<sensor_msgs::msg::PointCloud2>(EXPECTED_PART_POINT_CLOUD_INPUT_PORT_KEY),
                              BT::OutputPort<plg_msgs::msg::ToolPath>(TOOL_PATH_OUTPUT_PORT_KEY) });
  }

  using RosServiceNode<plg_msgs::srv::GeneratePathPlan>::RosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};
#endif

#ifdef HAS_CAPSEN_SUPPORT
/**
 * @brief The RequestMotionPlan class
 * Service Node that requests a motion plan from the CapSen planning service based on the current joint states and tool path.
 */
class RequestMotionPlan : public BT::RosServiceNode<capsen_planning::srv::GenerateTrajectory>
{
public:
  inline static std::string CURRENT_JOINT_STATES_INPUT_PORT_KEY = "start_joint_state";
  inline static std::string TOOL_PATH_INPUT_PORT_KEY = "tool_path";
  inline static std::string JOINT_TRAJECTORY_OUTPUT_PORT_KEY = "joint_trajectory";
  
  inline static BT::PortsList providedPorts()
  {
  return providedBasicPorts({ BT::InputPort<plg_msgs::msg::ToolPath>(TOOL_PATH_INPUT_PORT_KEY),
                              BT::InputPort<sensor_msgs::msg::JointState>(CURRENT_JOINT_STATES_INPUT_PORT_KEY),
                              BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(JOINT_TRAJECTORY_OUTPUT_PORT_KEY) });
  }

  using RosServiceNode<capsen_planning::srv::GenerateTrajectory>::RosServiceNode;

  bool setRequest(typename Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};
#endif

#ifdef HAS_CAPSEN_SUPPORT
/**
 * @brief The EnterModeType class
 * Publisher Node that sends a command to enter a specific grinding mode.
 */
class EnterModeType : public BT::RosTopicPubNode<std_msgs::msg::Int32>
{
public:
  inline static std::string INT_TYPE_PORT_KEY = "mode_type";
  inline static BT::PortsList providedPorts()
  {
      return providedBasicPorts({ BT::InputPort<std::string>(INT_TYPE_PORT_KEY) });
  }
  EnterModeType(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params);
      
  bool setMessage(std_msgs::msg::Int32& msg) override;
};
#endif

class GetCapsenLogMessage : public BT::RosTopicSubNode<std_msgs::msg::String>
{
  public:
    inline static std::string LOG_MESSAGE_OUTPUT_PORT_KEY = "message";
    inline static BT::PortsList providedPorts()
    {
        return providedBasicPorts({ BT::OutputPort<std::string>(LOG_MESSAGE_OUTPUT_PORT_KEY) });
    }
    using BT::RosTopicSubNode<std_msgs::msg::String>::RosTopicSubNode;

    BT::NodeStatus onTick(const typename std_msgs::msg::String::SharedPtr& last_msg) override;
};

#ifdef HAS_CAPSEN_SUPPORT
class LoadTPPFromYAML : public BT::SyncActionNode
{
  public:
    LoadTPPFromYAML(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;

    inline static std::string TPP_FILE_PATH_PORT_KEY = "file_path";
    inline static std::string TPP_OUTPUT_PORT_KEY = "tpp";
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>(TPP_FILE_PATH_PORT_KEY), BT::OutputPort<plg_msgs::msg::ToolPath>(TPP_OUTPUT_PORT_KEY)};
    }
};

class SaveTPPToYAML : public BT::SyncActionNode
{
  public:
    SaveTPPToYAML(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;

    inline static std::string TPP_FILE_PATH_PORT_KEY = "file_path";
    inline static std::string TPP_INPUT_PORT_KEY = "tpp";
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>(TPP_FILE_PATH_PORT_KEY), BT::InputPort<plg_msgs::msg::ToolPath>(TPP_INPUT_PORT_KEY)};
    }
};
#endif

#ifdef HAS_CAPSEN_SUPPORT
class PubTPP : public BT::RosTopicPubNode<geometry_msgs::msg::PoseArray>
{
  public:
    inline static std::string TPP_INPUT_PORT_KEY = "tool_path";
    
    inline static BT::PortsList providedPorts()
    {
        return providedBasicPorts({ BT::InputPort<plg_msgs::msg::ToolPath>(TPP_INPUT_PORT_KEY)});
    }

    PubTPP(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params);
        
    bool setMessage(geometry_msgs::msg::PoseArray& msg) override;
};
#endif

#ifdef HAS_CAPSEN_SUPPORT
class GetNumberSegments : public BT::SyncActionNode
{
  public:
    inline static std::string TPP_INPUT_PORT_KEY = "tool_path";
    inline static std::string NUM_SEGMENTS_OUTPUT_PORT_KEY = "num_segments";
    GetNumberSegments(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;
    inline static BT::PortsList providedPorts()
    {
        return { BT::InputPort<plg_msgs::msg::ToolPath>(TPP_INPUT_PORT_KEY), BT::OutputPort<int>(NUM_SEGMENTS_OUTPUT_PORT_KEY) };
    }
};
#endif

} // namespace bt_common_nodes