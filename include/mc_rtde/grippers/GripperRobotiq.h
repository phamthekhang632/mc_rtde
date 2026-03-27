#pragma once

#include <ur_rtde/robotiq_gripper.h>

#include <mc_rtde/grippers/GripperInterface.h>

namespace mc_rtde
{
struct GripperRobotiq : public GripperInterface
{
  GripperRobotiq(const std::string & ip, int port = 63352) : ip_(ip), port_(port) {};
  void connect() override;

  void setPosition(const std::vector<double> & position) override;
  void setForce(const std::vector<double> & force) override;
  void setSpeed(const std::vector<double> & speed) override;

  std::vector<double> getPosition() override;
  std::vector<double> getForce() override;
  std::vector<double> getSpeed() override;
  std::vector<double> getStatus(const std::vector<std::string> & vars) override;

  void setState(std::vector<double> state);
  void setCommand(std::vector<double> state);
  std::vector<double> getState();
  std::vector<double> getCommand();

  void autoCalibrate() override;

private:
  ur_rtde::RobotiqGripper * ur_rtde_gripper_;
  std::string ip_;
  int port_;
  std::vector<double> state_ = {0};
  std::vector<double> command_ = {0};

  // TODO: move gripper mutex to here
};

} // namespace mc_rtde
