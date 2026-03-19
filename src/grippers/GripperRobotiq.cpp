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

void GripperRobotiq::setForce(const std::vector<double> & force)
{
  ur_rtde_gripper_->setForce(force[0]);
}

void GripperRobotiq::setSpeed(const std::vector<double> & speed)
{
  ur_rtde_gripper_->setSpeed(speed[0]);
}

std::vector<double> GripperRobotiq::getPosition()
{
  return {ur_rtde_gripper_->getCurrentPosition()};
}

std::vector<double> GripperRobotiq::getForce()
{
  return {(double)ur_rtde_gripper_->getVar("FOR") / 255.0};
}

std::vector<double> GripperRobotiq::getSpeed()
{
  return {(double)ur_rtde_gripper_->getVar("SPE") / 255.0};
}

/**
 * @brief Gets the current status of the Robotiq gripper.
 *
 * For more defails, check [ur_rtde APT](https://sdurobotics.gitlab.io/ur_rtde/api/api.html#robotiq-gripper-api)
 *
 * @param vars Accepted variable names:
 *
 * - `"ACT"` : activate (1 while activated, can be reset to clear fault status)
 *
 * - `"GTO"` : go to (will perform go to with the actions set in pos, for, spe)
 *
 * - `"ATR"` : auto-release (emergency slow move)
 *
 * - `"ARD"` : auto-release direction (open(1) or close(0) during auto-release)
 *
 * - `"FOR"` : force (0.0-1.0)
 *
 * - `"SPE"` : speed (0.0-1.0)
 *
 * - `"POS"` : position (0.0-1.0), 1.0 = open
 *
 * Read only:
 *
 * - `"STA"` : status (0 = is reset, 1 = activating, 3 = active)
 *
 * - `"PRE"` : position request (echo of last commanded position)
 *
 * - `"OBJ"` : object detection (0 = moving, 1 = outer grip, 2 = inner grip, 3 = no object at rest)
 *
 * - `"FLT"` : fault (0=ok, see manual for errors if not zero)
 *
 * @return Values in the same order as @p vars.
 * `"FOR"`, `"SPE"`, and `"POS"` is in
 * [UNIT_NORMALIZED](https://sdurobotics.gitlab.io/ur_rtde/api/api.html#_CPPv4N7ur_rtde14RobotiqGripper5eUnit15UNIT_NORMALIZEDE)
 */
std::vector<double> GripperRobotiq::getStatus(const std::vector<std::string> & vars)
{
  std::vector<double> values;
  values.reserve(vars.size());

  for(const auto & var : vars)
  {
    if(var == "FOR")
    {
      values.push_back(getForce()[0]);
    }
    else if(var == "SPE")
    {
      values.push_back(getSpeed()[0]);
    }
    else if(var == "POS")
    {
      values.push_back(getPosition()[0]);
    }
    else
    {
      values.push_back(static_cast<double>(ur_rtde_gripper_->getVar(var)));
    }
  }
  return values;
}

void GripperRobotiq::autoCalibrate()
{
  ur_rtde_gripper_->autoCalibrate();
}

} // namespace mc_rtde
