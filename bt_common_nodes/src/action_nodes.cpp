#include "action_nodes.hpp"

namespace bt_common_nodes 
{

ProcessTraj::ProcessTraj(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
{
}

BT::NodeStatus ProcessTraj::tick() 
{
    auto trajectory = getBTInput<trajectory_msgs::msg::JointTrajectory>(this, JOINT_TRAJECTORY_INPUT_PORT_KEY);
    auto queue = std::make_shared<std::queue<trajectory_msgs::msg::JointTrajectoryPoint>>();
    for (const auto& point : trajectory.points) {
        queue->push(point);
    }
    setOutput(POINT_QUEUE_OUTPUT_PORT_KEY, queue);
    return BT::NodeStatus::SUCCESS;
}

SetBlackboardString::SetBlackboardString(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}

BT::NodeStatus SetBlackboardString::tick() 
{
    std::string value = getBTInput<std::string>(this, VALUE_PORT);
    std::string key = getBTInput<std::string>(this, KEY_PORT);
    this->config().blackboard->set(key, value);
    std::cout << "Set blackboard key: " << key << " to value: " << value << std::endl;
    return BT::NodeStatus::SUCCESS;
}

ResetSpinBoxValue::ResetSpinBoxValue(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config) 
{}

BT::NodeStatus ResetSpinBoxValue::tick()
{
    auto spin_box_name = getBTInput<std::string>(this, SPIN_BOX_NAME_PORT_KEY);
    auto spin_box = this->config().blackboard->get<QSpinBox*>(spin_box_name);
    spin_box->setValue(0);
    

    return BT::NodeStatus::SUCCESS;
}

GetValueFromSpinBox::GetValueFromSpinBox(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}

BT::NodeStatus GetValueFromSpinBox::tick() 
{
    auto spin_box_name = getBTInput<std::string>(this, SPIN_BOX_NAME_PORT_KEY);
    auto spin_box = this->config().blackboard->get<QSpinBox*>(spin_box_name);
    auto value = spin_box->value();

    setOutput(VALUE_OUTPUT_PORT_KEY, value);
    return BT::NodeStatus::SUCCESS;
}

GetValueFromCheckBox::GetValueFromCheckBox(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}

BT::NodeStatus GetValueFromCheckBox::tick() 
{
    auto check_box_name = getBTInput<std::string>(this, CHECK_BOX_NAME_PORT_KEY);
    auto check_box = this->config().blackboard->get<QCheckBox*>(check_box_name);
    auto value = check_box->isChecked();

    setOutput(VALUE_OUTPUT_PORT_KEY, value);
    return BT::NodeStatus::SUCCESS;
}

AddMsgToTextEdit::AddMsgToTextEdit(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}

BT::NodeStatus AddMsgToTextEdit::tick() 
{    
    auto textEditKey = getBTInput<std::string>(this, OUTPUT_BOX_PORT_KEY);
    auto textEdit = this->config().blackboard->get<QTextEdit*>(textEditKey);
    auto message = getBTInput<std::string>(this, MESSAGE_PORT_KEY);
    auto messageType = getBTInput<std::string>(this, "message_type");
    
    if (messageType == "error") {
        textEdit->setTextColor(Qt::red);
        message = "[ERROR] - " + message;
    }
    else if (messageType == "warning") {
        textEdit->setTextColor(Qt::darkYellow);
        message = "[WARNING] - " + message;
    }
    else { // default to info
        textEdit->setTextColor(Qt::black);
    }
    
    // Append the message with the selected color
    textEdit->append(QString::fromStdString(message));
    
    return BT::NodeStatus::SUCCESS;
}

GetComboBoxIndex::GetComboBoxIndex(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}

BT::NodeStatus GetComboBoxIndex::tick() 
{    
    auto combo_box_key = getBTInput<std::string>(this, COMBO_BOX_PORT_KEY);
    auto* combo_box = this->config().blackboard->get<QComboBox*>(combo_box_key);

    int mode_index = combo_box->currentIndex();
    RCLCPP_INFO(rclcpp::get_logger("GetComboBoxIndex"), "Selected combo box index: %d", mode_index);
    setOutput(COMBO_BOX_INDEX_OUTPUT_PORT_KEY, mode_index);
    return BT::NodeStatus::SUCCESS;
}

ClearTrajectory::ClearTrajectory(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}
BT::NodeStatus ClearTrajectory::tick() 
{
    auto trajectory_key = getBTInput<std::string>(this, TRAJECTORY_PORT_KEY);
    this->config().blackboard->set(trajectory_key, trajectory_msgs::msg::JointTrajectory());
    RCLCPP_INFO(rclcpp::get_logger("ClearTrajectory"), "Cleared trajectory at key: %s", trajectory_key.c_str());
    return BT::NodeStatus::SUCCESS;
}

LoadMotionPlanFromYAML::LoadMotionPlanFromYAML(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}
BT::NodeStatus LoadMotionPlanFromYAML::tick()
{
    auto motion_plan_file_path = getBTInput<std::string>(this, TRAJ_FILE_PATH_PORT_KEY);
    std::ifstream file(motion_plan_file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("LoadMotionPlanFromYAML"), "Failed to open motion plan file: %s", motion_plan_file_path.c_str());
        return BT::NodeStatus::FAILURE;
    }
    YAML::Node yaml = YAML::Load(file);
    file.close();
    auto traj_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    try {
        if(!YAML::convert<trajectory_msgs::msg::JointTrajectory>::decode(yaml, *traj_msg)) {
            RCLCPP_ERROR(rclcpp::get_logger("LoadMotionPlanFromYAML"), "Failed to decode trajectory from YAML file: %s", motion_plan_file_path.c_str());
            return BT::NodeStatus::FAILURE;
        }
    }
    catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("LoadMotionPlanFromYAML"), "YAML parsing error: %s", e.what());
        return BT::NodeStatus::FAILURE;
    }
    setOutput(TRAJ_NUM_POINTS_OUTPUT_PORT_KEY, static_cast<int>(traj_msg->points.size()));
    setOutput(TRAJ_OUTPUT_PORT_KEY, *traj_msg);  // Dereference shared_ptr to get direct value
    RCLCPP_INFO(rclcpp::get_logger("LoadMotionPlanFromYAML"),
                "Loaded trajectory with %zu points from file: %s",
                traj_msg->points.size(), motion_plan_file_path.c_str());
    return BT::NodeStatus::SUCCESS;
}

SaveMotionPlanToYAML::SaveMotionPlanToYAML(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}
BT::NodeStatus SaveMotionPlanToYAML::tick()
{
    auto session_folder = getBTInput<std::string>(this, TRAJ_FILE_PATH_PORT_KEY);
    auto subfolder_idx = config().blackboard->get<int>("num_motion_plans");

    auto folder_path = session_folder + "/motion_plan_" + std::to_string(subfolder_idx);
    if (!std::filesystem::exists(folder_path)) {
        std::filesystem::create_directories(folder_path);
    }
    std::cout << "Saving motion plan to folder: " << folder_path << std::endl;

    auto file_path = folder_path + "/joint_trajectory_" + std::to_string(this->config().blackboard->get<int>("num_traj_paths")) + ".yaml";
    this->config().blackboard->set("num_traj_paths", this->config().blackboard->get<int>("num_traj_paths") + 1);

    auto traj = getBTInput<trajectory_msgs::msg::JointTrajectory>(this, TRAJ_INPUT_PORT_KEY);

    std::ofstream file(file_path);
    if (!file.is_open()) {  
        RCLCPP_ERROR(rclcpp::get_logger("SaveMotionPlanToYAML"), "Failed to open trajectory file for writing: %s", file_path.c_str());
        return BT::NodeStatus::FAILURE;
    }
    YAML::Node yaml;
    try {
        yaml = YAML::convert<trajectory_msgs::msg::JointTrajectory>::encode(traj);
    }
    catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("SaveMotionPlanToYAML"), "YAML encoding error: %s", e.what());
        return BT::NodeStatus::FAILURE;
    }
    file << yaml;
    file.close();
    RCLCPP_INFO(rclcpp::get_logger("SaveMotionPlanToYAML"), "Saved trajectory to file: %s", file_path.c_str());
    return BT::NodeStatus::SUCCESS;
}


OpenFileDialog::OpenFileDialog(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}
BT::NodeStatus OpenFileDialog::tick()
{
    // open file dialog to select point cloud file
    auto main_gui = this->config().blackboard->get<QWidget*>("main_gui");
    if (!main_gui) {
        return BT::NodeStatus::FAILURE;
    }
    QString dialog_title = getBTInput<std::string>(this, DIALOG_TITLE_PORT_KEY).c_str();
    QString file_filter = getBTInput<std::string>(this, FILE_FILTER_PORT_KEY).c_str();

    // Set initial directory to ~/.aims
    QString initial_dir = QDir::homePath() + "/.aims/plg/";
    QString fileName = QFileDialog::getOpenFileName(main_gui, dialog_title, initial_dir, file_filter);
    if (fileName.isEmpty()) {
        return BT::NodeStatus::FAILURE;
    }

    setOutput(SELECTED_FILE_PATH_OUTPUT_PORT_KEY, fileName.toStdString());
    return BT::NodeStatus::SUCCESS;
}

#ifdef HAS_PCL_SUPPORT

LoadPCDFile::LoadPCDFile(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}
BT::NodeStatus LoadPCDFile::tick()
{
    auto pcd_file_path = getBTInput<std::string>(this, PCD_FILE_PATH_PORT_KEY);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud) == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("LoadPCDFile"), "Failed to load PCD file: %s", pcd_file_path.c_str());
        return BT::NodeStatus::FAILURE;
    }   
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg(new sensor_msgs::msg::PointCloud2());
    pcl::toROSMsg(*cloud, *cloud_msg);
    cloud_msg->header.frame_id = "world";  // Set appropriate frame

    setOutput(POINT_CLOUD_OUTPUT_PORT_KEY, *cloud_msg);  // Dereference shared_ptr to get direct value
    return BT::NodeStatus::SUCCESS;
}

SavePCDFile::SavePCDFile(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}
BT::NodeStatus SavePCDFile::tick()
{
    auto session_folder = getBTInput<std::string>(this, PCD_FILE_PATH_PORT_KEY);
    auto pcd_file_path = session_folder + "/scanned_cloud_" + std::to_string(this->config().blackboard->get<int>("num_saved_pcds")) + ".pcd";
    this->config().blackboard->set("num_saved_pcds", this->config().blackboard->get<int>("num_saved_pcds") + 1);
    auto cloud_msg = getBTInput<sensor_msgs::msg::PointCloud2>(this, POINT_CLOUD_INPUT_PORT_KEY);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud_msg, *cloud);

    if (pcl::io::savePCDFile<pcl::PointXYZ>(pcd_file_path, *cloud) == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("SavePCDFile"), "Failed to save PCD file: %s", pcd_file_path.c_str());
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("SavePCDFile"), "Saved PCD file: %s", pcd_file_path.c_str());
    return BT::NodeStatus::SUCCESS;
}
#endif

SetLabelColor::SetLabelColor(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}

BT::NodeStatus SetLabelColor::tick()
{
    auto label = this->config().blackboard->get<QLabel*>(getBTInput<std::string>(this, LABEL_PORT_KEY));
    auto color = getBTInput<std::string>(this, COLOR_PORT_KEY);

    if (!label) {
        RCLCPP_ERROR(rclcpp::get_logger("SetLabelColor"), "Label pointer is null");
        return BT::NodeStatus::FAILURE;
    }

    label->setStyleSheet(QString("QLabel { color : %1; }").arg(QString::fromStdString(color)));
    return BT::NodeStatus::SUCCESS;
}

#ifdef HAS_PCL_SUPPORT
PubPointCloud2::PubPointCloud2(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params) :
    BT::RosTopicPubNode<sensor_msgs::msg::PointCloud2>(name, conf, params)
{}

bool PubPointCloud2::setMessage(sensor_msgs::msg::PointCloud2& msg)
{
    // Convert PCL PointCloud to ROS PointCloud2 message
    msg = getBTInput<sensor_msgs::msg::PointCloud2>(this, POINT_CLOUD_INPUT_PORT_KEY);
    msg.header.frame_id = "world";  // Set appropriate frame
    msg.header.stamp = rclcpp::Clock().now();

    return true;
}
#endif

BT::NodeStatus GetCurrentJointState::onTick(const typename sensor_msgs::msg::JointState::SharedPtr& last_msg)
{
    if (last_msg)
    {
        setOutput(CURRENT_JOINT_STATES_OUTPUT_PORT_KEY, *last_msg);
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}

ClearTrajectoryList::ClearTrajectoryList(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}
BT::NodeStatus ClearTrajectoryList::tick()
{
    auto traj_list = getBTInput<std::vector<trajectory_msgs::msg::JointTrajectory>>(this, TRAJECTORY_LIST_PORT_KEY);
    traj_list.clear();
    RCLCPP_INFO(rclcpp::get_logger("ClearTrajectoryList"), "Cleared trajectory list");
    return BT::NodeStatus::SUCCESS;
}

AddTrajectoryToList::AddTrajectoryToList(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}
BT::NodeStatus AddTrajectoryToList::tick()
{
    auto traj_list = getBTInput<std::vector<trajectory_msgs::msg::JointTrajectory>>(this, TRAJECTORY_LIST_PORT_KEY);
    auto traj = getBTInput<trajectory_msgs::msg::JointTrajectory>(this, TRAJECTORY_TO_ADD_PORT_KEY);

    traj_list.push_back(traj);

    setOutput(TRAJECTORY_LIST_PORT_KEY, traj_list);
    setOutput(NUM_TRAJECTORIES_OUTPUT_PORT_KEY, static_cast<int>(traj_list.size()));

    return BT::NodeStatus::SUCCESS;
}

GetTrajectoryAtIndex::GetTrajectoryAtIndex(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}
BT::NodeStatus GetTrajectoryAtIndex::tick()
{
    auto traj_list = getBTInput<std::vector<trajectory_msgs::msg::JointTrajectory>>(this, TRAJECTORY_LIST_PORT_KEY);
    auto index = getBTInput<int>(this, TRAJECTORY_INDEX_PORT_KEY);

    if (index < 0 || index >= static_cast<int>(traj_list.size())) {
        RCLCPP_ERROR(rclcpp::get_logger("GetTrajectoryAtIndex"), "Index out of bounds: %d", index);
        return BT::NodeStatus::FAILURE;
    }

    setOutput(TRAJECTORY_AT_INDEX_OUTPUT_PORT_KEY, traj_list[index]);
    RCLCPP_INFO(rclcpp::get_logger("GetTrajectoryAtIndex"), "Retrieved trajectory at index %d from list", index);
    config().blackboard->set("trajectory_index", index + 1);

    setOutput(TRAJECTORY_NUM_POINTS_OUTPUT_PORT_KEY, static_cast<int>(traj_list[index].points.size()));

    return BT::NodeStatus::SUCCESS;
}

StoreLastJointState::StoreLastJointState(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}

BT::NodeStatus StoreLastJointState::tick()
{
    auto last_joint_traj = getBTInput<trajectory_msgs::msg::JointTrajectory>(this, LAST_JOINT_TRAJ_PORT_KEY);
    if (last_joint_traj.points.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("StoreLastJointState"), "Last joint trajectory has no points");
        return BT::NodeStatus::FAILURE;
    }
    auto last_point = last_joint_traj.points.back();
    // convert from JointTrajectoryPoint to JointState
    sensor_msgs::msg::JointState last_state;
    last_state.header = last_joint_traj.header;
    last_state.name = last_joint_traj.joint_names;
    last_state.position = last_point.positions;
    setOutput(CURRENT_JOINT_STATE_PORT_KEY, last_state);
    return BT::NodeStatus::SUCCESS;
}

LoadMotionPlanYAMLsFromDirectory::LoadMotionPlanYAMLsFromDirectory(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}

BT::NodeStatus LoadMotionPlanYAMLsFromDirectory::tick()
{
    auto directory_path = getBTInput<std::string>(this, DIRECTORY_PATH_PORT_KEY);
    std::vector<trajectory_msgs::msg::JointTrajectory> traj_list;

    if (!std::filesystem::exists(directory_path) || !std::filesystem::is_directory(directory_path)) {
        RCLCPP_ERROR(rclcpp::get_logger("LoadMotionPlanYAMLsFromDirectory"), "Invalid directory: %s", directory_path.c_str());
        return BT::NodeStatus::FAILURE;
    }

    // Collect and sort file paths to ensure proper ordering
    std::vector<std::filesystem::path> yaml_files;
    for (const auto& entry : std::filesystem::directory_iterator(directory_path)) {
        if (entry.path().extension() == ".yaml") {
            yaml_files.push_back(entry.path());
        }
    }
    
    // Sort files by filename to ensure proper order (jt_0.yaml, jt_1.yaml, etc.)
    std::sort(yaml_files.begin(), yaml_files.end(), [](const std::filesystem::path& a, const std::filesystem::path& b) {
        return a.filename().string() < b.filename().string();
    });

    // Process files in sorted order
    for (const auto& file_path : yaml_files) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_WARN(rclcpp::get_logger("LoadMotionPlanYAMLsFromDirectory"), "Failed to open file: %s", file_path.c_str());
            continue;
        }
        YAML::Node yaml = YAML::Load(file);
        file.close();
        trajectory_msgs::msg::JointTrajectory traj_msg;
        try {
            if(!YAML::convert<trajectory_msgs::msg::JointTrajectory>::decode(yaml, traj_msg)) {
                RCLCPP_WARN(rclcpp::get_logger("LoadMotionPlanYAMLsFromDirectory"), "Failed to decode trajectory from YAML file: %s", file_path.c_str());
                continue;
            }
        }
        catch (const YAML::Exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("LoadMotionPlanYAMLsFromDirectory"), "YAML parsing error in file %s: %s", file_path.c_str(), e.what());
            continue;
        }
        traj_list.push_back(traj_msg);
        RCLCPP_INFO(rclcpp::get_logger("LoadMotionPlanYAMLsFromDirectory"), "Loaded trajectory with %zu points from file: %s",
                    traj_msg.points.size(), file_path.c_str());
    }

    if (traj_list.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("LoadMotionPlanYAMLsFromDirectory"), "No valid trajectory files found in directory: %s", directory_path.c_str());
        return BT::NodeStatus::FAILURE;
    }

    setOutput(TRAJ_LIST_OUTPUT_PORT_KEY, traj_list);
    setOutput(NUM_TRAJECTORIES_OUTPUT_PORT_KEY, static_cast<int>(traj_list.size()));
    return BT::NodeStatus::SUCCESS;
}

OpenDirectoryDialog::OpenDirectoryDialog(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}

BT::NodeStatus OpenDirectoryDialog::tick()
{
    // open directory dialog to select a folder
    auto main_gui = this->config().blackboard->get<QWidget*>("main_gui");
    if (!main_gui) {
        return BT::NodeStatus::FAILURE;
    }
    QString dialog_title = getBTInput<std::string>(this, DIALOG_TITLE_PORT_KEY).c_str();

    // Set initial directory to ~/.aims
    QString initial_dir = QDir::homePath() + "/.aims/plg/";
    QString dir_path = QFileDialog::getExistingDirectory(main_gui, dialog_title, initial_dir);
    if (dir_path.isEmpty()) {
        return BT::NodeStatus::FAILURE;
    }

    setOutput(SELECTED_DIRECTORY_PATH_OUTPUT_PORT_KEY, dir_path.toStdString());
    return BT::NodeStatus::SUCCESS;
}

ConcatAllTrajectoryPlans::ConcatAllTrajectoryPlans(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{}

BT::NodeStatus ConcatAllTrajectoryPlans::tick() 
{
    auto traj_list = getBTInput<std::vector<trajectory_msgs::msg::JointTrajectory>>(this, TRAJ_LIST_INPUT_PORT_KEY);
    trajectory_msgs::msg::JointTrajectory combined_traj;
    combined_traj.joint_names = traj_list[0].joint_names; // Assume all trajectories have the same joint names
    
    // Keep track of cumulative time offset
    rclcpp::Duration time_offset = rclcpp::Duration::from_seconds(0.0);
    
    for (const auto& traj : traj_list) {
        for (const auto& point : traj.points) {
            auto adjusted_point = point;
            // Adjust time_from_start by adding the cumulative offset
            rclcpp::Duration point_time(point.time_from_start);
            rclcpp::Duration adjusted_time = point_time + time_offset;
            adjusted_point.time_from_start = adjusted_time;
            combined_traj.points.push_back(adjusted_point);
        }
        
        // Update offset for next trajectory - use the last point's time from current trajectory
        if (!traj.points.empty()) {
            rclcpp::Duration last_point_time(traj.points.back().time_from_start);
            time_offset = time_offset + last_point_time;
        }
    }
    
    setOutput(TRAJ_OUTPUT_PORT_KEY, combined_traj);
    return BT::NodeStatus::SUCCESS;
}

} // namespace bt_common_nodes