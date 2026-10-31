#include "swerve_test/dynamixel_sdk_interface.hpp"

#include <stdexcept>

DynamixelSdkInterface::DynamixelSdkInterface(
  const std::string & port,
  int baudrate,
  double protocol_version,
  const std::vector<uint8_t> & dxl_ids,
  int operating_mode,
  const std::string & profile,
  double profile_velocity,
  double profile_acceleration)
: port_(port),
  baudrate_(baudrate),
  protocol_version_(protocol_version),
  dxl_ids_(dxl_ids),
  operating_mode_(operating_mode),
  profile_(profile),
  profile_velocity_(profile_velocity),
  profile_acceleration_(profile_acceleration),
  logger_(rclcpp::get_logger("DynamixelSdkInterface"))
{
  initPort();
  initMotors();
  initBulkRead();
  group_bulk_write_ =
    std::make_unique<dynamixel::GroupBulkWrite>(port_handler_, packet_handler_);
  RCLCPP_INFO(logger_, "DynamixelSdkInterface initialized (%zu motors)", dxl_ids_.size());
}

DynamixelSdkInterface::~DynamixelSdkInterface()
{
  if (port_handler_ != nullptr) {
    // Best-effort: torque off on shutdown so the wheels can be turned by hand.
    uint8_t dxl_error = 0;
    for (auto id : dxl_ids_) {
      packet_handler_->write1ByteTxRx(
        port_handler_, id, addr_torque_enable_, torque_disable_value_, &dxl_error);
    }
    port_handler_->closePort();
  }
}

void DynamixelSdkInterface::initPort()
{
  port_handler_ = dynamixel::PortHandler::getPortHandler(port_.c_str());
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);

  if (!port_handler_->openPort()) {
    throw std::runtime_error("Failed to open port: " + port_);
  }
  if (!port_handler_->setBaudRate(baudrate_)) {
    throw std::runtime_error("Failed to set baudrate");
  }
  RCLCPP_INFO(logger_, "Opened port %s @ %d", port_.c_str(), baudrate_);
}

void DynamixelSdkInterface::initMotors()
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Torque OFF before configuring (mode/profile writes are EEPROM-area
  // adjacent and require torque off).
  for (auto id : dxl_ids_) {
    dxl_comm_result = packet_handler_->write1ByteTxRx(
      port_handler_, id, addr_torque_enable_, torque_disable_value_, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_WARN(logger_, "Torque OFF failed (ID=%d): %s",
        id, packet_handler_->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
      RCLCPP_WARN(logger_, "Torque OFF error (ID=%d): %s",
        id, packet_handler_->getRxPacketError(dxl_error));
    }
  }

  // Operating mode.
  for (auto id : dxl_ids_) {
    packet_handler_->write1ByteTxRx(
      port_handler_, id, addr_operating_mode_, operating_mode_, &dxl_error);
  }

  // Profile.
  if (profile_ == "Step") {
    for (auto id : dxl_ids_) {
      packet_handler_->write4ByteTxRx(port_handler_, id, addr_profile_acc_, 0, &dxl_error);
      packet_handler_->write4ByteTxRx(port_handler_, id, addr_profile_vel_, 0, &dxl_error);
    }
  } else if (profile_ == "Rectangle") {
    for (auto id : dxl_ids_) {
      packet_handler_->write4ByteTxRx(
        port_handler_, id, addr_profile_acc_,
        static_cast<uint32_t>(profile_velocity_), &dxl_error);
      packet_handler_->write4ByteTxRx(port_handler_, id, addr_profile_vel_, 0, &dxl_error);
    }
  } else if (profile_ == "Trapezoidal") {
    for (auto id : dxl_ids_) {
      packet_handler_->write4ByteTxRx(
        port_handler_, id, addr_profile_acc_,
        static_cast<uint32_t>(profile_acceleration_), &dxl_error);
      packet_handler_->write4ByteTxRx(
        port_handler_, id, addr_profile_vel_,
        static_cast<uint32_t>(profile_velocity_), &dxl_error);
    }
  } else {
    RCLCPP_WARN(logger_, "Unknown profile type: %s", profile_.c_str());
  }

  // Torque ON.
  for (auto id : dxl_ids_) {
    dxl_comm_result = packet_handler_->write1ByteTxRx(
      port_handler_, id, addr_torque_enable_, torque_enable_value_, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_WARN(logger_, "Torque ON failed (ID=%d): %s",
        id, packet_handler_->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
      RCLCPP_WARN(logger_, "Torque ON error (ID=%d): %s",
        id, packet_handler_->getRxPacketError(dxl_error));
    }
  }
}

void DynamixelSdkInterface::initBulkRead()
{
  group_bulk_read_ =
    std::make_unique<dynamixel::GroupBulkRead>(port_handler_, packet_handler_);

  for (auto id : dxl_ids_) {
    bool ok = group_bulk_read_->addParam(id, addr_goal_position_, len_present_all_);
    if (!ok) {
      RCLCPP_WARN(logger_, "GroupBulkRead addParam failed (ID=%d)", id);
    }
  }
  RCLCPP_INFO(logger_, "GroupBulkRead setup done for %zu motors", dxl_ids_.size());
}

bool DynamixelSdkInterface::readOnce(States & out_states)
{
  if (!group_bulk_read_) {
    RCLCPP_ERROR(logger_, "GroupBulkRead not initialized");
    return false;
  }

  int dxl_comm_result = group_bulk_read_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_WARN(logger_, "BulkRead txRxPacket failed: %s",
      packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  }

  const std::size_t n = dxl_ids_.size();
  out_states.goal_positions.assign(n, 0);
  out_states.positions.assign(n, 0);
  out_states.velocities.assign(n, 0);
  out_states.loads.assign(n, 0);

  for (std::size_t i = 0; i < n; ++i) {
    auto id = dxl_ids_[i];

    if (group_bulk_read_->isAvailable(id, addr_goal_position_, len_goal_position_)) {
      uint32_t raw = group_bulk_read_->getData(id, addr_goal_position_, len_goal_position_);
      out_states.goal_positions[i] = static_cast<int32_t>(raw);
    } else {
      RCLCPP_WARN(logger_, "GoalPosition not available (ID=%d)", id);
    }
    if (group_bulk_read_->isAvailable(id, addr_present_position_, len_present_position_)) {
      uint32_t raw = group_bulk_read_->getData(id, addr_present_position_, len_present_position_);
      out_states.positions[i] = static_cast<int32_t>(raw);
    } else {
      RCLCPP_WARN(logger_, "Position not available (ID=%d)", id);
    }
    if (group_bulk_read_->isAvailable(id, addr_present_velocity_, len_present_velocity_)) {
      uint32_t raw = group_bulk_read_->getData(id, addr_present_velocity_, len_present_velocity_);
      out_states.velocities[i] = static_cast<int32_t>(raw);
    } else {
      RCLCPP_WARN(logger_, "Velocity not available (ID=%d)", id);
    }
    if (group_bulk_read_->isAvailable(id, addr_present_load_, len_present_load_)) {
      uint32_t raw = group_bulk_read_->getData(id, addr_present_load_, len_present_load_);
      out_states.loads[i] = static_cast<int16_t>(static_cast<uint16_t>(raw));
    } else {
      RCLCPP_WARN(logger_, "Load not available (ID=%d)", id);
    }
  }
  return true;
}

bool DynamixelSdkInterface::writeGoalPositions(const std::vector<int32_t> & goals_unit)
{
  if (!group_bulk_write_) {
    RCLCPP_ERROR(logger_, "GroupBulkWrite not initialized");
    return false;
  }
  if (goals_unit.size() != dxl_ids_.size()) {
    RCLCPP_ERROR(logger_, "Goal size mismatch (got %zu, expected %zu)",
      goals_unit.size(), dxl_ids_.size());
    return false;
  }

  group_bulk_write_->clearParam();
  for (std::size_t i = 0; i < dxl_ids_.size(); ++i) {
    int32_t g = goals_unit[i];
    uint8_t param[4];
    param[0] = static_cast<uint8_t>(g & 0xFF);
    param[1] = static_cast<uint8_t>((g >> 8) & 0xFF);
    param[2] = static_cast<uint8_t>((g >> 16) & 0xFF);
    param[3] = static_cast<uint8_t>((g >> 24) & 0xFF);
    if (!group_bulk_write_->addParam(dxl_ids_[i], addr_goal_position_, len_goal_position_, param)) {
      RCLCPP_WARN(logger_, "GroupBulkWrite addParam failed (ID=%d)", dxl_ids_[i]);
    }
  }
  int dxl_comm_result = group_bulk_write_->txPacket();
  group_bulk_write_->clearParam();
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_WARN(logger_, "BulkWrite txPacket failed: %s",
      packet_handler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  return true;
}
