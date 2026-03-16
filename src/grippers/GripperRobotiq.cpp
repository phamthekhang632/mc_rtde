#include <mc_rtde/grippers/GripperRobotiq.h>

namespace mc_rtde
{

void GripperRobotiq::connect()
{
  ur_rtde_gripper_ = new ur_rtde::RobotiqGripper(ip_, port_, false);

  ur_rtde_gripper_->connect();
  ur_rtde_gripper_->activate();
  ur_rtde_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_NORMALIZED);
}

void GripperRobotiq::setPosition(const std::vector<double> & position)
{
  auto target = std::clamp(position[0], 0.0, 0.725);
  target = -target / 0.725 + 1.0; // maping [0, 0.725] to [1.0, 0.0]
  ur_rtde_gripper_->move(target);
}

void GripperRobotiq::setForce(const std::vector<double> & forces)
{
  // TODO
}

std::vector<double> GripperRobotiq::getPosition()
{
  return {ur_rtde_gripper_->getCurrentPosition()};
}

std::vector<double> GripperRobotiq::getForce()
{
  // TODO not supported yet
  return {};
}

int GripperRobotiq::getStatus()
{
  // TODO not supported yet
  return 0;
}

void GripperRobotiq::autoCalibrate()
{
  ur_rtde_gripper_->autoCalibrate();
}
} // namespace mc_rtde
