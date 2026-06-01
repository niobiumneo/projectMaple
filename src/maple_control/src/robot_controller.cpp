#include "maple_control/robot_controller.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <fstream>

#include <nlohmann/json.hpp>

namespace {
constexpr int kCommunicationsSuccess = COMM_SUCCESS;

std::string to_lower_copy(std::string value)
{
	std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
		return static_cast<char>(std::tolower(ch));
	});
	return value;
}
}  // namespace

RobotController::RobotController()
	: Node("robot_move_node")
{
	device_name_ = this->declare_parameter<std::string>("device", "/dev/ttyUSB0");
	motion_dir_ = this->declare_parameter<std::string>("motion_dir", "MotionLib");
	baudrate_ = this->declare_parameter<int>("baudrate", 1000000);
	protocol_version_ = this->declare_parameter<double>("protocol_version", 2.0);
	motor_count_ = this->declare_parameter<int>("motor_count", 12);
	first_motor_id_ = this->declare_parameter<int>("first_motor_id", 0);

	addr_torque_enable_ = this->declare_parameter<int>("addr_torque_enable", 64);
	addr_goal_position_ = this->declare_parameter<int>("addr_goal_position", 116);
	len_goal_position_ = this->declare_parameter<int>("len_goal_position", 4);
	addr_present_position_ = this->declare_parameter<int>("addr_present_position", 132);
	len_present_position_ = this->declare_parameter<int>("len_present_position", 4);
	addr_profile_velocity_ = this->declare_parameter<int>("addr_profile_velocity", 112);
	addr_profile_acceleration_ = this->declare_parameter<int>("addr_profile_acceleration", 108);

	torque_enable_ = this->declare_parameter<int>("torque_enable", 1);
	torque_disable_ = this->declare_parameter<int>("torque_disable", 0);

	port_handler_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());
	packet_handler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);
	group_sync_write_ = new dynamixel::GroupSyncWrite(
		port_handler_, packet_handler_, addr_goal_position_, len_goal_position_);
	group_sync_read_ = new dynamixel::GroupSyncRead(
		port_handler_, packet_handler_, addr_present_position_, len_present_position_);

	motion_sub_ = this->create_subscription<std_msgs::msg::String>(
		"/motion_command", 10,
		std::bind(&RobotController::on_motion_command, this, std::placeholders::_1));

	interaction_sub_ = this->create_subscription<std_msgs::msg::String>(
		"/interaction_control", 10,
		std::bind(&RobotController::on_interaction_control, this, std::placeholders::_1));

	if (!initialize_robot()) {
		RCLCPP_ERROR(this->get_logger(), "Robot initialization failed.");
	}

	RCLCPP_INFO(this->get_logger(), "Robot Controller Node has been started.");
}

RobotController::~RobotController()
{
	stop_event_.store(true);
	run_event_.store(true);

	if (motion_thread_.joinable()) {
		motion_thread_.join();
	}

	shutdown_robot();

	delete group_sync_write_;
	delete group_sync_read_;
	group_sync_write_ = nullptr;
	group_sync_read_ = nullptr;
}

void RobotController::on_motion_command(const std_msgs::msg::String::SharedPtr msg)
{
	const std::string motion_name = msg && !msg->data.empty() ? msg->data : "";
	if (motion_name.empty()) {
		return;
	}

	if (motion_in_progress_.exchange(true)) {
		RCLCPP_WARN(this->get_logger(), "Motion already running. Ignoring '%s'.", motion_name.c_str());
		return;
	}

	stop_event_.store(false);

	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		active_motion_ = motion_name;
		active_pose_index_ = 0;
	}

	if (motion_thread_.joinable()) {
		motion_thread_.join();
	}

	motion_thread_ = std::thread([this, motion_name]() {
		run_motion(motion_name);
		motion_in_progress_.store(false);
	});
}

void RobotController::on_interaction_control(const std_msgs::msg::String::SharedPtr msg)
{
	const std::string cmd = msg && !msg->data.empty() ? msg->data : "";
	const std::string cmd_lower = to_lower_copy(cmd);

	if (cmd_lower == "pause") {
		run_event_.store(false);
		RCLCPP_INFO(this->get_logger(), "robot_ctr: PAUSE received");
	} else if (cmd_lower == "resume") {
		run_event_.store(true);
		RCLCPP_INFO(this->get_logger(), "robot_ctr: RESUME received");
	} else if (cmd_lower == "stop") {
		stop_event_.store(true);
		run_event_.store(true);
		RCLCPP_INFO(this->get_logger(), "robot_ctr: STOP received");
	} else {
		RCLCPP_WARN(this->get_logger(), "robot_ctr: Unknown interaction_control command: '%s'", cmd.c_str());
	}
}

void RobotController::run_motion(const std::string &motion_name)
{
	MotionConfig config;
	if (!load_motion_config(motion_name, config)) {
		std::lock_guard<std::mutex> lock(state_mutex_);
		active_motion_.clear();
		active_pose_index_ = -1;
		return;
	}

	if (config.motors.empty()) {
		RCLCPP_WARN(this->get_logger(), "robot_ctr: Motion '%s' has no motors defined", motion_name.c_str());
		std::lock_guard<std::mutex> lock(state_mutex_);
		active_motion_.clear();
		active_pose_index_ = -1;
		return;
	}

	const int num_states = static_cast<int>(config.pause_durations.size()) + 1;
	int pose_index = 0;

	while (pose_index < num_states && rclcpp::ok()) {
		if (stop_event_.load()) {
			RCLCPP_INFO(this->get_logger(), "robot_ctr: Stop requested, aborting motion");
			break;
		}

		if (!wait_until_running()) {
			break;
		}

		{
			std::lock_guard<std::mutex> lock(state_mutex_);
			active_pose_index_ = pose_index;
		}

		RCLCPP_INFO(
			this->get_logger(), "robot_ctr: Moving to pose %d of %d", pose_index + 1, num_states);

		for (const auto &motor : config.motors) {
			if (stop_event_.load() || !rclcpp::ok()) {
				break;
			}

			if (pose_index >= static_cast<int>(motor.goal_positions.size())) {
				RCLCPP_ERROR(
					this->get_logger(),
					"robot_ctr: Pose index %d out of range for motor %d", pose_index, motor.id);
				stop_event_.store(true);
				break;
			}

			const int goal_position = motor_angle_to_value(motor.goal_positions[pose_index]);

			int dxl_error = 0;
			int dxl_comm_result = packet_handler_->write4ByteTxRx(
				port_handler_, motor.id, addr_profile_acceleration_, motor.profile_acceleration, &dxl_error);
			if (dxl_comm_result != kCommunicationsSuccess) {
				RCLCPP_WARN(this->get_logger(), "%s", packet_handler_->getTxRxResult(dxl_comm_result));
			} else if (dxl_error != 0) {
				RCLCPP_WARN(this->get_logger(), "%s", packet_handler_->getRxPacketError(dxl_error));
			}

			dxl_comm_result = packet_handler_->write4ByteTxRx(
				port_handler_, motor.id, addr_profile_velocity_, motor.profile_speed, &dxl_error);
			if (dxl_comm_result != kCommunicationsSuccess) {
				RCLCPP_WARN(this->get_logger(), "%s", packet_handler_->getTxRxResult(dxl_comm_result));
			} else if (dxl_error != 0) {
				RCLCPP_WARN(this->get_logger(), "%s", packet_handler_->getRxPacketError(dxl_error));
			}

			uint8_t param_goal_position[4] = {
				DXL_LOBYTE(DXL_LOWORD(goal_position)),
				DXL_HIBYTE(DXL_LOWORD(goal_position)),
				DXL_LOBYTE(DXL_HIWORD(goal_position)),
				DXL_HIBYTE(DXL_HIWORD(goal_position))};

			const bool add_param_result = group_sync_write_->addParam(motor.id, param_goal_position);
			if (!add_param_result) {
				RCLCPP_ERROR(this->get_logger(), "[ID:%03d] groupSyncWrite addparam failed", motor.id);
				group_sync_write_->clearParam();
				return;
			}
		}

		if (stop_event_.load() || !rclcpp::ok()) {
			group_sync_write_->clearParam();
			break;
		}

		int dxl_comm_result = group_sync_write_->txPacket();
		if (dxl_comm_result != kCommunicationsSuccess) {
			RCLCPP_WARN(this->get_logger(), "%s", packet_handler_->getTxRxResult(dxl_comm_result));
		}

		group_sync_write_->clearParam();

		if (pose_index < static_cast<int>(config.pause_durations.size())) {
			const double pause_duration = config.pause_durations[pose_index];
			RCLCPP_INFO(
				this->get_logger(), "robot_ctr: Waiting %.2fs after pose %d", pause_duration,
				pose_index + 1);

			const auto sleep_result = interruptible_sleep(pause_duration);
			if (sleep_result == SleepResult::Stop) {
				RCLCPP_INFO(this->get_logger(), "robot_ctr: Stop requested during wait, aborting motion");
				break;
			}
			if (sleep_result == SleepResult::Restart) {
				RCLCPP_INFO(this->get_logger(), "robot_ctr: Resumed after pause, repeating current pose");
				continue;
			}
		}

		++pose_index;
	}

	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		active_motion_.clear();
		active_pose_index_ = -1;
	}

	stop_event_.store(false);
	RCLCPP_INFO(this->get_logger(), "robot_ctr: Motion complete");
}

bool RobotController::wait_until_running()
{
	while (!run_event_.load()) {
		if (!rclcpp::ok()) {
			return false;
		}
		rclcpp::sleep_for(std::chrono::milliseconds(50));
	}
	return rclcpp::ok();
}

RobotController::SleepResult RobotController::interruptible_sleep(double seconds)
{
	if (seconds <= 0.0) {
		return SleepResult::Done;
	}

	auto start = std::chrono::steady_clock::now();
	while (true) {
		if (!rclcpp::ok() || stop_event_.load()) {
			return SleepResult::Stop;
		}

		if (!run_event_.load()) {
			if (!wait_until_running()) {
				return SleepResult::Stop;
			}
			return SleepResult::Restart;
		}

		const auto elapsed = std::chrono::steady_clock::now() - start;
		const double elapsed_sec = std::chrono::duration<double>(elapsed).count();
		if (elapsed_sec >= seconds) {
			return SleepResult::Done;
		}

		rclcpp::sleep_for(std::chrono::milliseconds(50));
	}
}

bool RobotController::initialize_robot()
{
	if (!port_handler_->openPort()) {
		RCLCPP_ERROR(this->get_logger(), "Failed to open the port %s", device_name_.c_str());
		return false;
	}

	if (!port_handler_->setBaudRate(baudrate_)) {
		RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate %d", baudrate_);
		return false;
	}

	for (int i = 0; i < motor_count_; ++i) {
		const int motor_id = first_motor_id_ + i;
		int dxl_error = 0;
		const int dxl_comm_result = packet_handler_->write1ByteTxRx(
			port_handler_, motor_id, addr_torque_enable_, torque_enable_, &dxl_error);

		if (dxl_comm_result != kCommunicationsSuccess) {
			RCLCPP_WARN(this->get_logger(), "%s", packet_handler_->getTxRxResult(dxl_comm_result));
		} else if (dxl_error != 0) {
			RCLCPP_WARN(this->get_logger(), "%s", packet_handler_->getRxPacketError(dxl_error));
		} else {
			RCLCPP_INFO(this->get_logger(), "Dynamixel#%d has been successfully connected", motor_id);
		}

		if (!group_sync_read_->addParam(motor_id)) {
			RCLCPP_ERROR(this->get_logger(), "[ID:%03d] groupSyncRead addparam failed", motor_id);
			return false;
		}
	}

	return true;
}

void RobotController::shutdown_robot()
{
	if (!port_handler_ || !packet_handler_) {
		return;
	}

	if (group_sync_read_) {
		group_sync_read_->clearParam();
	}

	for (int i = 0; i < motor_count_; ++i) {
		const int motor_id = first_motor_id_ + i;
		int dxl_error = 0;
		const int dxl_comm_result = packet_handler_->write1ByteTxRx(
			port_handler_, motor_id, addr_torque_enable_, torque_disable_, &dxl_error);

		if (dxl_comm_result != kCommunicationsSuccess) {
			RCLCPP_WARN(this->get_logger(), "%s", packet_handler_->getTxRxResult(dxl_comm_result));
		} else if (dxl_error != 0) {
			RCLCPP_WARN(this->get_logger(), "%s", packet_handler_->getRxPacketError(dxl_error));
		}
	}

	port_handler_->closePort();
}

bool RobotController::load_motion_config(const std::string &motion_name, MotionConfig &config)
{
	const std::string motion_path = motion_dir_ + "/" + motion_name + ".json";
	std::ifstream file(motion_path);
	if (!file.is_open()) {
		RCLCPP_ERROR(this->get_logger(), "robot_ctr: Failed to open motion config '%s'", motion_path.c_str());
		return false;
	}

	nlohmann::json json_data;
	try {
		file >> json_data;
	} catch (const std::exception &ex) {
		RCLCPP_ERROR(this->get_logger(), "robot_ctr: Failed to parse JSON '%s': %s", motion_path.c_str(), ex.what());
		return false;
	}

	config.motors.clear();
	config.pause_durations.clear();

	if (json_data.contains("pause_durations")) {
		config.pause_durations = json_data["pause_durations"].get<std::vector<double>>();
	}

	if (!json_data.contains("motors") || !json_data["motors"].is_array()) {
		RCLCPP_ERROR(this->get_logger(), "robot_ctr: Motion '%s' has invalid motors list", motion_name.c_str());
		return false;
	}

	for (const auto &motor_json : json_data["motors"]) {
		MotorConfig motor;
		motor.id = motor_json.value("id", -1);
		motor.profile_acceleration = motor_json.value("profile_acceleration", 0);
		motor.profile_speed = motor_json.value("profile_speed", 0);
		if (motor_json.contains("goal_positions")) {
			motor.goal_positions = motor_json["goal_positions"].get<std::vector<double>>();
		}

		if (motor.id < 0 || motor.goal_positions.empty()) {
			RCLCPP_WARN(this->get_logger(), "robot_ctr: Skipping invalid motor entry in '%s'", motion_name.c_str());
			continue;
		}

		config.motors.push_back(std::move(motor));
	}

	return !config.motors.empty();
}

int RobotController::motor_angle_to_value(double angle)
{
	return static_cast<int>((angle / 360.0) * 4095.0);
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RobotController>());
	rclcpp::shutdown();
	return 0;
}


