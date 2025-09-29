#include "pl_grinding_nodes.hpp"

namespace bt_common_nodes 
{

#ifdef HAS_CAPSEN_SUPPORT
bool RequestPathPlan::setRequest(typename Request::SharedPtr& request)
{
  request->path_parameters = getBTInput<plg_msgs::msg::PathParameters>(this, PATH_PARAMETERS_INPUT_PORT_KEY);
  request->cloud_flash = getBTInput<sensor_msgs::msg::PointCloud2>(this, FLASH_POINT_CLOUD_INPUT_PORT_KEY);
  request->cloud_expected = getBTInput<sensor_msgs::msg::PointCloud2>(this, EXPECTED_PART_POINT_CLOUD_INPUT_PORT_KEY);
  return true;
}

BT::NodeStatus RequestPathPlan::onResponseReceived(const typename Response::SharedPtr& response)
{
  setOutput(TOOL_PATH_OUTPUT_PORT_KEY, response->tool_path);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus RequestPathPlan::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(rclcpp::get_logger("plg_bt"), "Request Path Plan Service Call Failed");
  RCLCPP_INFO(rclcpp::get_logger("plg_bt"), "Error code: %d", static_cast<int>(error));
  return BT::NodeStatus::FAILURE;
}
#endif

#ifdef HAS_PCL_SUPPORT
bool RequestPointCloud::setRequest(typename Request::SharedPtr& request)
{
  return true;
}

BT::NodeStatus RequestPointCloud::onResponseReceived(const typename Response::SharedPtr& response)
{
  setOutput(POINT_CLOUD_OUTPUT_PORT_KEY, response->scanned_cloud);

  return BT::NodeStatus::SUCCESS;
}
#endif

#ifdef HAS_CAPSEN_SUPPORT
bool RequestMotionPlan::setRequest(typename Request::SharedPtr& request)
{
  // Pack latest joint_state message into CapSen requeset format
  auto current_joint_states = getBTInput<sensor_msgs::msg::JointState>(this, CURRENT_JOINT_STATES_INPUT_PORT_KEY);
  for (const auto& posn : current_joint_states.position) { 
      request->start_joints.push_back(posn);
  }
  RCLCPP_INFO(rclcpp::get_logger("plg_bt"), "Updated current_joint_state data.");
  
  // Set TCP Velocity (meters per second)
  request->speed_mpers = 0.03; 

  // Get Toolpath from Blackboard
  auto tool_path = getBTInput<plg_msgs::msg::ToolPath>(this, TOOL_PATH_INPUT_PORT_KEY);
  int segment_index = this->config().blackboard->get<int>("segment_index");
  RCLCPP_INFO(rclcpp::get_logger("plg_bt"), "Planning motion for segment index: %d", segment_index);

  // Pack toolpath waypoints into requeset
  request->ramp_up_path = tool_path.path[segment_index].lead_in.poses;
  request->tf_path = tool_path.path[segment_index].segment.poses;
  request->ramp_down_path = tool_path.path[segment_index].lead_out.poses;
  request->fix_orient = true;

  // Define a home position
  // geometry_msgs::msg::Pose pose_home;
  // pose_home.position.x = 0.0;
  // pose_home.position.y = -1.5;
  // pose_home.position.z = 1.8;
  // pose_home.orientation.x = -0.707;
  // pose_home.orientation.y = 0.707;
  // pose_home.orientation.z = 0.0;
  // pose_home.orientation.w = 0.0;
  
  // Prepend a liftoff point before motion
  geometry_msgs::msg::Pose first_pose = tool_path.path[segment_index].lead_in.poses.front();
  for (float i = 0.0; i <= 0.15; i+=0.002) { 
      first_pose.position.z += 0.002; // meters
      request->ramp_up_path.insert(request->ramp_up_path.begin(), first_pose);
  }

  // Append home position to end of grinding path
  geometry_msgs::msg::Pose end_pose = tool_path.path[segment_index].lead_out.poses.back();
  for (float i = 0; i < 0.1; i+=0.002) { 
      end_pose.position.z += 0.002; // meters
      request->ramp_down_path.push_back(end_pose);
  }

  // request->ramp_up_path.push_front(first_pose);

  RCLCPP_INFO(rclcpp::get_logger("plg_bt"), "Sending motion planning request....");

  return true;
}

BT::NodeStatus RequestMotionPlan::onResponseReceived(const typename Response::SharedPtr& response)
{
  if (response->trajectory.points.size() == 0)
  {
      RCLCPP_ERROR(rclcpp::get_logger("plg_bt"), "Capsen Motion Plan Request returned failure: %s", response->error);
      return BT::NodeStatus::FAILURE;
  }
  else
  {
  }

  // Initialize new trajectory message
  auto combined = trajectory_msgs::msg::JointTrajectory();
  combined.joint_names = response->start_trajectory.joint_names;
  combined.header = response->start_trajectory.header;
  
  // Copy points from first trajectory
  for (const auto& pt : response->start_trajectory.points) {
      combined.points.push_back(pt);
  }
  
  // Get the last time from start_trajectory
  rclcpp::Duration last_time(response->start_trajectory.points.back().time_from_start);
  
  // Append points from second trajectory, adjusting time_from_start
  for (const auto& pt : response->trajectory.points) {
      auto new_pt = pt;
      new_pt.time_from_start = rclcpp::Duration(pt.time_from_start) + last_time;
      combined.points.push_back(new_pt);
  }

  // Ensure Effort Values are empty (this is a Capsen issue that causes a crash)
  for (auto& point : combined.points)
  {
      point.effort.clear();
  }
  
  RCLCPP_INFO(rclcpp::get_logger("plg_bt"), "Capsen Motion Plan was Successfull. Combined two-part trajectory into a single output trajectory with %zu points", combined.points.size());
  setOutput(JOINT_TRAJECTORY_OUTPUT_PORT_KEY, combined);
  
  config().blackboard->set("segment_index", this->config().blackboard->get<int>("segment_index") + 1);
  return BT::NodeStatus::SUCCESS;
}
#endif


#ifdef HAS_CAPSEN_SUPPORT
EnterModeType::EnterModeType(const std::string& name, const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
    : RosTopicPubNode<std_msgs::msg::Int32>(name, conf, params)
{
}

bool EnterModeType::setMessage(std_msgs::msg::Int32& msg) {
    auto type = getBTInput<int32_t>(this, INT_TYPE_PORT_KEY);

    // RCLCPP_INFO(logger(), "Setting request for EnterMode");
    msg.data=type;
    // RCLCPP_INFO(logger(), "Request set for EnterMode to %d", mode_index);
    return true;
}
#endif

BT::NodeStatus GetCapsenLogMessage::onTick(const typename std_msgs::msg::String::SharedPtr& last_msg)
{
  if (last_msg)
  {
    std::string log = last_msg->data;
    setOutput(LOG_MESSAGE_OUTPUT_PORT_KEY, log);

    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

#ifdef HAS_CAPSEN_SUPPORT
LoadTPPFromYAML::LoadTPPFromYAML(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}
BT::NodeStatus LoadTPPFromYAML::tick()
{
    auto tpp_file_path = getBTInput<std::string>(this, TPP_FILE_PATH_PORT_KEY);
    std::ifstream file(tpp_file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("LoadTPPFromYAML"), "Failed to open TPP file: %s", tpp_file_path.c_str());
        return BT::NodeStatus::FAILURE;
    }   
    YAML::Node yaml = YAML::Load(file);
    file.close();

    auto tpp_msg = std::make_shared<plg_msgs::msg::ToolPath>();
    auto path_msg = std::make_shared<plg_msgs::msg::PathSegment>();
    try {
        if (!YAML::convert<plg_msgs::msg::ToolPath>::decode(yaml, *tpp_msg)) {
            RCLCPP_ERROR(rclcpp::get_logger("LoadTPPFromYAML"), "Failed to decode TPP from YAML file: %s", tpp_file_path.c_str());
            return BT::NodeStatus::FAILURE;
        }
    }
    catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("LoadTPPFromYAML"), "YAML parsing error: %s", e.what());
        return BT::NodeStatus::FAILURE;
    }

    setOutput(TPP_OUTPUT_PORT_KEY, *tpp_msg);  // Dereference shared_ptr to get direct value

    // [DEBUG]
    // print out the loaded TPP message
    // RCLCPP_INFO(rclcpp::get_logger("LoadTPPFromYAML"),
    //             "Loaded TPP with %zu paths from file: %s",
    //             tpp_msg->path.size(), tpp_file_path.c_str());
    // for (size_t i = 0; i < tpp_msg->path.size(); ++i) {
    //     const auto& path = tpp_msg->path[i];
    //     RCLCPP_INFO(rclcpp::get_logger("LoadTPPFromYAML"),
    //                 "Path %zu: num_points=%zu",
    //                 i,
    //                 path.segment.poses.size());
    // }

    return BT::NodeStatus::SUCCESS;
}

SaveTPPToYAML::SaveTPPToYAML(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}
BT::NodeStatus SaveTPPToYAML::tick()
{
    auto session_folder = getBTInput<std::string>(this, TPP_FILE_PATH_PORT_KEY);
    auto file_path = session_folder + "/tpp_" + std::to_string(this->config().blackboard->get<int>("num_tpp_paths")) + ".yaml";
    this->config().blackboard->set("num_tpp_paths", this->config().blackboard->get<int>("num_tpp_paths") + 1);
    auto tpp = getBTInput<plg_msgs::msg::ToolPath>(this, TPP_INPUT_PORT_KEY);

    std::ofstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("SaveTPPToYAML"), "Failed to open TPP file for writing: %s", file_path.c_str());
        return BT::NodeStatus::FAILURE;
    }

    YAML::Node yaml;
    try {
        yaml = YAML::convert<plg_msgs::msg::ToolPath>::encode(tpp);
    }
    catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("SaveTPPToYAML"), "YAML encoding error: %s", e.what());
        return BT::NodeStatus::FAILURE;
    }

    file << yaml;
    file.close();

    RCLCPP_INFO(rclcpp::get_logger("SaveTPPToYAML"), "Saved TPP to file: %s", file_path.c_str());
    return BT::NodeStatus::SUCCESS;
}
#endif

#ifdef HAS_CAPSEN_SUPPORT
PubTPP::PubTPP(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params) :
    BT::RosTopicPubNode<geometry_msgs::msg::PoseArray>(name, conf, params)
{}

bool PubTPP::setMessage(geometry_msgs::msg::PoseArray& msg)
{
    auto tpp = getBTInput<plg_msgs::msg::ToolPath>(this, TPP_INPUT_PORT_KEY);
    msg.poses.clear();
    for (const auto& path_segment : tpp.path) {
        for (const auto& pose : path_segment.lead_in.poses) {
            msg.poses.push_back(pose);
        }
        for (const auto& pose : path_segment.segment.poses) {
            msg.poses.push_back(pose);
        }
        for (const auto& pose : path_segment.lead_out.poses) {
            msg.poses.push_back(pose);
        }
    }
    msg.header.frame_id = "world";  // Set appropriate frame
    msg.header.stamp = rclcpp::Clock().now();

    return true;
}
#endif

#ifdef HAS_CAPSEN_SUPPORT
GetNumberSegments::GetNumberSegments(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}

BT::NodeStatus GetNumberSegments::tick() 
{    
    auto tpp = getBTInput<plg_msgs::msg::ToolPath>(this, TPP_INPUT_PORT_KEY);
    int num_segments = static_cast<int>(tpp.path.size());
    RCLCPP_INFO(rclcpp::get_logger("GetNumberSegments"), "Number of segments in TPP: %d", num_segments);
    setOutput(NUM_SEGMENTS_OUTPUT_PORT_KEY, num_segments);
    return BT::NodeStatus::SUCCESS;
}
#endif

} // namespace bt_common_nodes