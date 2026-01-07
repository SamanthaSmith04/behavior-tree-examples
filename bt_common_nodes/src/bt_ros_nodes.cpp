#include "bt_ros_nodes.hpp"
#include <QComboBox>
#include <rclcpp/rclcpp.hpp>

namespace bt_common_nodes 
{
RosSpinnerNode::RosSpinnerNode(const std::string& instance_name, const BT::NodeConfig& config,
    rclcpp::Node::SharedPtr node)
: BT::ConditionNode(instance_name, config), node_(node)
{
}

BT::NodeStatus RosSpinnerNode::tick()
{
    rclcpp::spin_some(node_);
    return BT::NodeStatus::SUCCESS;
}


bool ExecuteMotionPlan::setGoal(Goal& goal)
{
    RCLCPP_INFO(logger(), "Setting JointTrajectory Goal for Robot Motion Execution");
    goal.trajectory = getBTInput<trajectory_msgs::msg::JointTrajectory>(this, JOINT_TRAJECTORY_INPUT_PORT_KEY);
    RCLCPP_INFO(logger(), "Goal Set");
    return true;
}

BT::NodeStatus ExecuteMotionPlan::onResultReceived(const WrappedResult& result)
{
    RCLCPP_INFO(logger(), "result recieved");
    if (result.result->error_code == control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL)
    {
        RCLCPP_INFO(logger(), "Motion plan executed successfully");
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_ERROR(logger(), "Failed to execute motion plan: %s", result.result->error_string.c_str());
    return BT::NodeStatus::FAILURE;
}


BT::NodeStatus ExecuteMotionPlan::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
    RCLCPP_INFO(logger(), "Goal Running");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ReplaceStartState::onTick(const typename sensor_msgs::msg::JointState::SharedPtr& last_msg)
{
    auto trajectory = getBTInput<trajectory_msgs::msg::JointTrajectory>(this, MOTION_PLAN_PORT_KEY);
    if (last_msg)
    {
        auto latest_joint_state_ = *last_msg;

        trajectory_msgs::msg::JointTrajectoryPoint start_point;
        start_point.positions.resize(trajectory.joint_names.size());
        start_point.velocities = std::vector<double>(start_point.positions.size(), 0);
        start_point.accelerations = std::vector<double>(start_point.positions.size(), 0);
        start_point.effort = std::vector<double>(start_point.positions.size(), 0);
        start_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
        // Find the index of the trajectory joint in the latest joint state message
        for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i)
        {
            const std::string& name = trajectory.joint_names[i];
            auto it = std::find(latest_joint_state_.name.begin(), latest_joint_state_.name.end(), name);
            if (it == latest_joint_state_.name.end())
                throw std::runtime_error("Failed to find joint '" + name + "' in latest joint state message");

            auto idx = std::distance(latest_joint_state_.name.begin(), it);
            start_point.positions[i] = latest_joint_state_.position[idx];
          }
        trajectory.points[0] = start_point;

        setOutput(MOTION_PLAN_PORT_KEY, trajectory);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus CheckAtDestination::onTick(const typename sensor_msgs::msg::JointState::SharedPtr& last_msg)
{
    RCLCPP_INFO(logger(), "Entering");
    if(last_msg){
        auto latest_joint_state_ = *last_msg;
        RCLCPP_INFO(logger(), "Received Joint State");
        auto plan = getBTInput<trajectory_msgs::msg::JointTrajectory>(this, MOTION_PLAN_PORT_KEY);
        auto destination = plan.points[plan.points.size() -1 ];

        for (std::size_t i = 0; i < plan.joint_names.size(); ++i)
        {
            const std::string& name = plan.joint_names[i];
            auto it = std::find(latest_joint_state_.name.begin(), latest_joint_state_.name.end(), name);
            if (it == latest_joint_state_.name.end())
                throw std::runtime_error("Failed to find joint '" + name + "' in latest joint state message");

            auto idx = std::distance(latest_joint_state_.name.begin(), it);
            float distance =std::abs(destination.positions[i] - latest_joint_state_.position[idx]);
            RCLCPP_INFO(logger(), "joint: %s", plan.joint_names[i].c_str());
            RCLCPP_INFO(logger(), "current: %f", latest_joint_state_.position[idx]);
            RCLCPP_INFO(logger(), "target: %f", destination.positions[i]);
            RCLCPP_INFO(logger(), "distance: %f", distance);
            if(distance >= 0.0001){
                return BT::NodeStatus::FAILURE;
            }
        }
        RCLCPP_INFO(logger(), "Robot already at destination");
        return BT::NodeStatus::SUCCESS;
    }
    //FIXME: How do we deal with no message recieved
    RCLCPP_WARN(logger(), "No message was recieved");
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ListenForTrajectory::onTick(const typename trajectory_msgs::msg::JointTrajectory::SharedPtr& last_msg)
{
    RCLCPP_INFO(logger(), "Listening ...");
    if (last_msg && last_msg->points.size() > 0)
    {
        RCLCPP_INFO(logger(), "Received trajectory message with %ld points", last_msg->points.size());
        // Check if the message is older than 100ms
        // If so, return failure
        // Otherwise, set the output port and return success
        rclcpp::Time now;
        // //Get current time and compare to message time
            if(auto node = node_.lock())
            {
                now = node->get_clock()->now();
            }
        rclcpp::Time msg_time = last_msg->header.stamp;

        // copy values from last msg into a new pointer
        auto new_trajectory = std::make_shared<trajectory_msgs::msg::JointTrajectory>(*last_msg);
        setOutput(JOINT_TRAJECTORY_INPUT_PORT_KEY, *new_trajectory);
        setOutput(NUM_POINTS_OUTPUT_PORT_KEY, static_cast<int>(last_msg->points.size()));
        last_msg->points.clear();

        RCLCPP_INFO(logger(), "Set output port with trajectory message");

        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

#ifdef HAS_PCL_SUPPORT
BT::NodeStatus ListenForPointCloud::onTick(const typename sensor_msgs::msg::PointCloud2::SharedPtr& last_msg)
{
    RCLCPP_INFO(logger(), "Listening for point cloud...");
    if (last_msg)
    {
        RCLCPP_INFO(logger(), "Received point cloud");
        setOutput(POINT_CLOUD_INPUT_PORT_KEY, *last_msg);
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_WARN(logger(), "No point cloud received");
    return BT::NodeStatus::FAILURE;
}
#endif

#ifdef HAS_MOTOMAN_SUPPORT
bool QueueTrajPoint::setRequest(typename Request::SharedPtr& request)
{
    auto trajectory = getBTInput<trajectory_msgs::msg::JointTrajectory>(this, JOINT_TRAJECTORY_INPUT_PORT_KEY);
    auto queue = getBTInput<std::shared_ptr<std::queue<trajectory_msgs::msg::JointTrajectoryPoint>>>(this, POINT_QUEUE_INPUT_PORT_KEY);
    if (queue->empty())
    {
        RCLCPP_ERROR(logger(), "Queue is empty");
        return false;
    }

    request->joint_names = trajectory.joint_names;
    request->point = queue->front();
    RCLCPP_INFO(logger(), "Set request");
    return true;
}

BT::NodeStatus QueueTrajPoint::onResponseReceived(const typename Response::SharedPtr& response)
{
    if (response->result_code.value == motoros2_interfaces::msg::QueueResultEnum::SUCCESS)
    {
        RCLCPP_INFO(logger(), "Queued trajectory point successfully");
        auto queue = getBTInput<std::shared_ptr<std::queue<trajectory_msgs::msg::JointTrajectoryPoint>>>(this, POINT_QUEUE_INPUT_PORT_KEY);
        queue->pop();
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_ERROR(logger(), "Failed to queue trajectory point: %s", response->message.c_str());
        return BT::NodeStatus::FAILURE;
    }
}
#endif



#ifdef HAS_MOTOMAN_SUPPORT
bool StartPointQueueMode::setRequest(typename Request::SharedPtr& request)
{
    RCLCPP_INFO(logger(), "Point queue mode started");
    return true;
}

BT::NodeStatus StartPointQueueMode::onResponseReceived(const typename Response::SharedPtr& response)
{
    RCLCPP_INFO(logger(), "Response received for StartPointQueueMode");
    return BT::NodeStatus::SUCCESS;
}
#endif

#ifdef HAS_MOTOMAN_SUPPORT
bool ReadSingleIO::setRequest(typename Request::SharedPtr& request)
{
    request->address = getBTInput<int>(this, IO_NAME_INPUT_PORT_KEY);
    // RCLCPP_INFO(logger(), "Attempting to read from IO address: %d", request->io_address);
    return true;
}
BT::NodeStatus ReadSingleIO::onResponseReceived(const typename Response::SharedPtr& response)
{
    if (!response->success)
    {
        RCLCPP_ERROR(logger(), "Failed to read IO! Error code: %d", response->result_code);
        return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(logger(), "Read IO: value = %d", response->value);
    setOutput(IO_VALUE_OUTPUT_PORT_KEY, response->value);
    return BT::NodeStatus::SUCCESS;
}

bool WriteSingleIO::setRequest(typename Request::SharedPtr& request)
{
    request->address = getBTInput<int>(this, IO_NAME_INPUT_PORT_KEY);
    request->value = getBTInput<int>(this, IO_VALUE_INPUT_PORT_KEY);
    // RCLCPP_INFO(logger(), "Writing value %d to IO address: %d", request->io_value, request->io_address);
    return true;
}
BT::NodeStatus WriteSingleIO::onResponseReceived(const typename Response::SharedPtr& response)
{
    if (!response->success)
    {
        RCLCPP_ERROR(logger(), "Failed to write IO! Error code: %d", response->result_code);
        return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(logger(), "Wrote IO");
    return BT::NodeStatus::SUCCESS;
}
#endif

bool StopTrajMode::setRequest(typename Request::SharedPtr& request)
{
    RCLCPP_INFO(logger(), "Trajectory mode stopped");
    return true;
}

BT::NodeStatus StopTrajMode::onResponseReceived(const typename Response::SharedPtr& response)
{
    RCLCPP_INFO(logger(), "Response received for StopTrajMode");
    return BT::NodeStatus::SUCCESS;
}

PubTrajectoryPreview::PubTrajectoryPreview(const std::string& name, const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
    : RosTopicPubNode<trajectory_msgs::msg::JointTrajectory>(name, conf, params)
{
}

bool PubTrajectoryPreview::setMessage(trajectory_msgs::msg::JointTrajectory& msg)
{
    // get trajectory from blackboard
    auto plan = getBTInput<trajectory_msgs::msg::JointTrajectory>(this, TRAJECTORY_KEY);
    plan_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>(plan); // copy by value into pointer
    msg = *plan_;
    std::cout << "Publishing trajectory with " << msg.points.size() << " points." << std::endl;
    return true;
}


}// namespace bt_common_nodes