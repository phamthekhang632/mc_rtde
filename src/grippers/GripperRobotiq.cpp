#include <mc_rtde/grippers/GripperRobotiq.h>

namespace mc_rtde
{

void GripperRobotiq::connect()
{
  ur_rtde_gripper_ = new ur_rtde::RobotiqGripper(ip_, port_, false);

  ur_rtde_gripper_->connect();
  ur_rtde_gripper_->activate();
  ur_rtde_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_NORMALIZED);

  state_ = getPosition();
}

void GripperRobotiq::control()
{
  std::vector<double> last_sent_pos = command_;
  std::vector<double> last_target_pos = command_;
  std::vector<double> command = command_;

  const float steady_threshold = 0.001f;
  int stable_count = 0;
  const int stable_required = 5;

  auto vectorDiff = [](const std::vector<double> & a, const std::vector<double> & b, double threshold) -> bool
  {
    for(size_t i = 0; i < a.size(); i++)
    {
      if(std::abs(a[i] - b[i]) > threshold) return true;
    }
    return false;
  };

  if(vectorDiff(command, last_target_pos, steady_threshold))
  {
    stable_count = 0;
    last_target_pos = command;
  }
  else
  {
    stable_count++;
  }

  if(stable_count >= stable_required && vectorDiff(command, last_sent_pos, steady_threshold))
  {
    setPosition(command);
    last_sent_pos = command;
  }
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

int GripperRobotiq::getDOF()
{
  return dof;
}

// TODO: add mutex
void GripperRobotiq::setState(std::vector<double> state)
{
  state_ = state;
};

void GripperRobotiq::setCommand(std::vector<double> command)
{
  command_ = command;
};

std::vector<double> GripperRobotiq::getState()
{
  return state_;
}

std::vector<double> GripperRobotiq::getCommand()
{
  return command_;
}

} // namespace mc_rtde
