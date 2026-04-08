/* Each of these BehaviorTree "Action" Nodes is effectivley a placeholder
*  for functionality that is not yet defined (e.g., interfacing with a
*  3D object viewer).
*/

#pragma once

#include <atomic>

#include "bt_common_ros_headers.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <QSpinBox>
#include <QPushButton>
#include <QCheckBox>
#include <QTextEdit>
#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QFileDialog>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#ifdef HAS_PCL_SUPPORT
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#endif
#include <fstream> 

#include "yaml_utils.hpp"
#include "utils.hpp"

namespace bt_common_nodes 
{

/**
 * @brief The ProcessTraj class
 * Action Node that takes a JointTrajectory message as input,
 * splits it into individual JointTrajectoryPoint messages,
 * and outputs a queue of these points for sequential processing.
 */
class ProcessTraj : public BT::SyncActionNode
{
  public:
  inline static std::string JOINT_TRAJECTORY_INPUT_PORT_KEY = "joint_trajectory";
  inline static std::string POINT_QUEUE_OUTPUT_PORT_KEY = "point_queue";
    ProcessTraj(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<trajectory_msgs::msg::JointTrajectory>(JOINT_TRAJECTORY_INPUT_PORT_KEY), 
                BT::OutputPort<std::shared_ptr<std::queue<trajectory_msgs::msg::JointTrajectoryPoint>>>(POINT_QUEUE_OUTPUT_PORT_KEY)};
    }
};

class ModuloOperator : public BT::SyncActionNode
{
  public:
    inline static std::string DIVIDEND_INPUT_PORT_KEY = "dividend";
    inline static std::string DIVISOR_INPUT_PORT_KEY = "divisor";
    inline static std::string REMAINDER_OUTPUT_PORT_KEY = "remainder";
    ModuloOperator(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<int>(DIVIDEND_INPUT_PORT_KEY),
                BT::InputPort<int>(DIVISOR_INPUT_PORT_KEY),
                BT::OutputPort<int>(REMAINDER_OUTPUT_PORT_KEY)};
    }
};


/**
 * @brief The SetBlackboardString class
 * Action Node that sets a string value in the blackboard.
 * It takes two input ports: 'key' for the blackboard key and 'value' for the string to set.
 */
class SetBlackboardString : public BT::SyncActionNode
{
  inline static std::string KEY_PORT = "key";
  inline static std::string VALUE_PORT = "value";
  public:
    SetBlackboardString(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>(KEY_PORT), BT::InputPort<std::string>(VALUE_PORT)};
    }
};


/**
 * @brief The GetComboBoxIndex class
 * Action Node that retrieves the current index from a specified QComboBox GUI element.
 * Note: This node assumes that the QComboBox pointer is stored in the
 * blackboard with the key provided in the input port.
 */
class GetComboBoxIndex : public BT::SyncActionNode
{
  public:
  inline static std::string COMBO_BOX_PORT_KEY = "combo_box";
  inline static std::string COMBO_BOX_INDEX_OUTPUT_PORT_KEY = "combo_box_index";
    GetComboBoxIndex(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>(COMBO_BOX_PORT_KEY), BT::OutputPort<int>(COMBO_BOX_INDEX_OUTPUT_PORT_KEY)};
    }
};


class ClearTrajectory : public BT::SyncActionNode
{
  public:
    inline static std::string TRAJECTORY_PORT_KEY = "trajectory_key";
    ClearTrajectory(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;
    inline static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>(TRAJECTORY_PORT_KEY) };
    }
};

class LoadMotionPlanFromYAML : public BT::SyncActionNode
{
  public:
    LoadMotionPlanFromYAML(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;

    inline static std::string TRAJ_FILE_PATH_PORT_KEY = "file_path";
    inline static std::string TRAJ_OUTPUT_PORT_KEY = "joint_trajectory";
    inline static std::string TRAJ_NUM_POINTS_OUTPUT_PORT_KEY = "num_points";
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>(TRAJ_FILE_PATH_PORT_KEY),
                BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(TRAJ_OUTPUT_PORT_KEY),
                BT::OutputPort<int>(TRAJ_NUM_POINTS_OUTPUT_PORT_KEY)};
    }
};

class SaveMotionPlanToYAML : public BT::SyncActionNode
{
  public:
    SaveMotionPlanToYAML(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;

    inline static std::string TRAJ_FILE_PATH_PORT_KEY = "file_path";
    inline static std::string TRAJ_INPUT_PORT_KEY = "joint_trajectory";
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>(TRAJ_FILE_PATH_PORT_KEY), BT::InputPort<trajectory_msgs::msg::JointTrajectory>(TRAJ_INPUT_PORT_KEY)};
    }
};


#ifdef HAS_PCL_SUPPORT
class LoadPCDFile : public BT::SyncActionNode
{
  public:
    LoadPCDFile(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;

    inline static std::string PCD_FILE_PATH_PORT_KEY = "file_path";
    inline static std::string POINT_CLOUD_OUTPUT_PORT_KEY = "point_cloud";
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>(PCD_FILE_PATH_PORT_KEY), BT::OutputPort<sensor_msgs::msg::PointCloud2>(POINT_CLOUD_OUTPUT_PORT_KEY)};
    }
};

class SavePCDFile : public BT::SyncActionNode
{
  public:
    SavePCDFile(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;

    inline static std::string PCD_FILE_PATH_PORT_KEY = "file_path";
    inline static std::string POINT_CLOUD_INPUT_PORT_KEY = "point_cloud";
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>(PCD_FILE_PATH_PORT_KEY), BT::InputPort<sensor_msgs::msg::PointCloud2>(POINT_CLOUD_INPUT_PORT_KEY)};
    }
};
#endif


#ifdef HAS_PCL_SUPPORT
class PubPointCloud2 : public BT::RosTopicPubNode<sensor_msgs::msg::PointCloud2>
{
  public:
    inline static std::string POINT_CLOUD_INPUT_PORT_KEY = "flash_point_cloud";
    
    inline static BT::PortsList providedPorts()
    {
        return providedBasicPorts({ BT::InputPort<sensor_msgs::msg::PointCloud2>(POINT_CLOUD_INPUT_PORT_KEY)});
    }

    PubPointCloud2(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params);
        
    bool setMessage(sensor_msgs::msg::PointCloud2& msg) override;
};
#endif

class GetCurrentJointState : public BT::RosTopicSubNode<sensor_msgs::msg::JointState>
{
  public:
    inline static std::string CURRENT_JOINT_STATES_OUTPUT_PORT_KEY = "joint_state";
    
    inline static BT::PortsList providedPorts()
    {
        return providedBasicPorts({ BT::OutputPort<sensor_msgs::msg::JointState>(CURRENT_JOINT_STATES_OUTPUT_PORT_KEY)});
    }

    using BT::RosTopicSubNode<sensor_msgs::msg::JointState>::RosTopicSubNode;

    BT::NodeStatus onTick(const typename sensor_msgs::msg::JointState::SharedPtr& last_msg) override;
};

class ClearTrajectoryList : public BT::SyncActionNode
{
  public:
    inline static std::string TRAJECTORY_LIST_PORT_KEY = "trajectory_list";
    ClearTrajectoryList(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;
    inline static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::vector<trajectory_msgs::msg::JointTrajectory>>(TRAJECTORY_LIST_PORT_KEY) };
    }
};

class AddTrajectoryToList : public BT::SyncActionNode
{
  public:
    inline static std::string TRAJECTORY_LIST_PORT_KEY = "trajectory_list";
    inline static std::string TRAJECTORY_TO_ADD_PORT_KEY = "joint_trajectory";
    inline static std::string NUM_TRAJECTORIES_OUTPUT_PORT_KEY = "num_trajectories";
    AddTrajectoryToList(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;
    inline static BT::PortsList providedPorts()
    {
        return { BT::BidirectionalPort<std::vector<trajectory_msgs::msg::JointTrajectory>>(TRAJECTORY_LIST_PORT_KEY), BT::InputPort<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_TO_ADD_PORT_KEY), BT::OutputPort<int>(NUM_TRAJECTORIES_OUTPUT_PORT_KEY) };
    }
};

class GetTrajectoryAtIndex : public BT::SyncActionNode
{
  public:
    inline static std::string TRAJECTORY_LIST_PORT_KEY = "trajectory_list";
    inline static std::string TRAJECTORY_INDEX_PORT_KEY = "index";
    inline static std::string TRAJECTORY_AT_INDEX_OUTPUT_PORT_KEY = "joint_trajectory";
    inline static std::string TRAJECTORY_NUM_POINTS_OUTPUT_PORT_KEY = "num_points";
    GetTrajectoryAtIndex(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;
    inline static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::vector<trajectory_msgs::msg::JointTrajectory>>(TRAJECTORY_LIST_PORT_KEY), BT::InputPort<int>(TRAJECTORY_INDEX_PORT_KEY), BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_AT_INDEX_OUTPUT_PORT_KEY), BT::OutputPort<int>(TRAJECTORY_NUM_POINTS_OUTPUT_PORT_KEY) };
    }
};

class StoreLastJointState : public BT::SyncActionNode
{
  public:
    inline static std::string LAST_JOINT_TRAJ_PORT_KEY = "last_joint_trajectory";
    inline static std::string CURRENT_JOINT_STATE_PORT_KEY = "last_joint_state";
    StoreLastJointState(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;
    inline static BT::PortsList providedPorts()
    {
        return { BT::InputPort<trajectory_msgs::msg::JointTrajectory>(LAST_JOINT_TRAJ_PORT_KEY), BT::OutputPort<sensor_msgs::msg::JointState>(CURRENT_JOINT_STATE_PORT_KEY) };
    }
};


class LoadMotionPlanYAMLsFromDirectory : public BT::SyncActionNode
{
  public:
    LoadMotionPlanYAMLsFromDirectory(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;

    inline static std::string DIRECTORY_PATH_PORT_KEY = "directory_path";
    inline static std::string TRAJ_LIST_OUTPUT_PORT_KEY = "trajectory_list";
    inline static std::string NUM_TRAJECTORIES_OUTPUT_PORT_KEY = "num_trajectories";
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>(DIRECTORY_PATH_PORT_KEY), BT::OutputPort<std::vector<trajectory_msgs::msg::JointTrajectory>>(TRAJ_LIST_OUTPUT_PORT_KEY), BT::OutputPort<int>(NUM_TRAJECTORIES_OUTPUT_PORT_KEY)};
    }
};


class ConcatAllTrajectoryPlans : public BT::SyncActionNode
{
  public:
    inline static std::string TRAJ_LIST_INPUT_PORT_KEY = "trajectory_plan_list";
    inline static std::string TRAJ_OUTPUT_PORT_KEY = "combined_joint_trajectory";
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::vector<trajectory_msgs::msg::JointTrajectory>>(TRAJ_LIST_INPUT_PORT_KEY),
                BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(TRAJ_OUTPUT_PORT_KEY)};
    }
    ConcatAllTrajectoryPlans(const std::string& name, const BT::NodeConfig& config);
    BT::NodeStatus tick() override;
};

class PubMeshMarker : public BT::RosTopicPubNode<visualization_msgs::msg::Marker>
{
public:
  inline static std::string MESH_PATH_INPUT_PORT_KEY = "mesh_filepath";
  inline static std::string FRAME_ID_PORT_KEY = "frame_id";

  inline static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<std::string>(MESH_PATH_INPUT_PORT_KEY),
        BT::InputPort<std::string>(FRAME_ID_PORT_KEY)
    });
  }

  PubMeshMarker(const std::string& name,
                const BT::NodeConfig& conf,
                const BT::RosNodeParams& params);

  bool setMessage(visualization_msgs::msg::Marker& msg) override;
};


} // namespace bt_common_nodes

