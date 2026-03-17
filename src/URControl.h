#pragma once
#include <condition_variable>

#include <mc_control/mc_global_controller.h>

#include "ControlMode.h"
#include "URControlType.h"
#include "mc_rtde/grippers/GripperInterface.h"

#include <mc_rtc/logging.h>
#include <mc_rtde/DriverBridgeRTDE.h>
#include <mc_rtde/DriverBridgeURModernDriver.h>
#include <mc_rtde/grippers/GripperRobotiq.h>

namespace mc_rtde
{

template<ControlMode cm>
struct URControlLoop
{
  URControlLoop(Driver driver, const std::string & name, const std::string & ip, double cycle_s);

  void init(mc_control::MCGlobalController & controller);

  void attachGripper(const mc_control::Configuration & config);

  void updateSensors(mc_control::MCGlobalController & controller);

  void updateControl(mc_control::MCGlobalController & controller);

  void controlThread(mc_control::MCGlobalController & controller,
                     std::mutex & startM,
                     std::condition_variable & startCV,
                     bool & start,
                     bool & running);

  void gripperThread(const std::string & gripper_name,
                     std::mutex & startM,
                     std::condition_variable & startCV,
                     bool & start,
                     bool & running);

  std::unordered_map<std::string, std::shared_ptr<GripperInterface>> grippers()
  {
    return grippersInterfaces_;
  }

  void autoCalibrateGrippers()
  {
    for(auto & gripper : grippersInterfaces_) gripper.second->autoCalibrate();
  }

private:
  std::string name_;
  std::string ip_;
  rbd::MultiBodyConfig command_;
  URSensorInfo state_;

  std::vector<double> gripper_command_;
  std::vector<double> gripper_state_;

  mc_rtc::Logger logger_;
  size_t sensor_id_ = 0;
  size_t control_id_ = 0;
  size_t prev_control_id_ = 0;
  double delay_ = 0;
  double cycle_s_ = 0;

  std::unique_ptr<DriverBridge> driverBridge_{nullptr};
  std::unordered_map<std::string, std::shared_ptr<GripperInterface>> grippersInterfaces_;
  URControlType<cm> control_;

  // To protect against concurrent read/write between:
  // - the thread URControlLoop::controlThread
  // - the method URControlLoop::updateSensors called from the controller_run thread (before MCGlobalController::run)
  mutable std::mutex updateSensorsMutex_;
  mutable std::mutex updateControlMutex_;
  mutable std::mutex gripperSensorMutex_;
  mutable std::mutex gripperControlMutex_;

  std::vector<double> sensorsBuffer_ = std::vector<double>(6, 0.0);
};

template<ControlMode cm>
using URControlLoopPtr = std::unique_ptr<URControlLoop<cm>>;

template<ControlMode cm>
URControlLoop<cm>::URControlLoop(Driver driver, const std::string & name, const std::string & ip, double cycle_s)
: name_(name), ip_(ip), logger_(mc_rtc::Logger::Policy::THREADED, "/tmp", "mc-rtde-" + name_), cycle_s_(cycle_s)
{
  if(driver == Driver::ur_rtde)
  {
    driverBridge_ = std::make_unique<DriverBridgeRTDE>(ip);
  }
  else
  {
    driverBridge_ = std::make_unique<DriverBridgeURModernDriver>(ip, cycle_s_);
  }
}

template<ControlMode cm>
void URControlLoop<cm>::attachGripper(const mc_control::Configuration & gripper_config)
{
  if(gripper_config("type") == "robotiq")
  {
    int port = gripper_config("port", 63352);
    std::string name = gripper_config("name");
    if(!grippersInterfaces_.count(name))
    {
      auto gripper = std::make_shared<mc_rtde::GripperRobotiq>(ip_, port);
      gripper->connect(); // connect before storing
      grippersInterfaces_.try_emplace(name, std::move(gripper));
    }
    else
      mc_rtc::log::error_and_throw("Gripper {} is already attached to the robot", name);
  }
  else
  {
    mc_rtc::log::warning("Gripper type : {} is not supported", gripper_config("type"));
  }
}

template<ControlMode cm>
void URControlLoop<cm>::init(mc_control::MCGlobalController & controller)
{
  // No need for thread synchronization here as the URControlLoop::controlThread is not yet running

  driverBridge_->sync(); // ensures that we got the first data
  logger_.start(controller.current_controller(), cycle_s_);
  logger_.addLogEntry("sensor_id", [this] { return sensor_id_; });
  logger_.addLogEntry("prev_control_id", [this]() { return prev_control_id_; });
  logger_.addLogEntry("control_id", [this]() { return control_id_; });
  logger_.addLogEntry("delay", [this]() { return delay_; });

  auto & robot = controller.controller().robots().robot(name_);
  auto & real = controller.controller().realRobots().robot(name_);
  state_.qIn_ = driverBridge_->getActualQ();
  state_.torqIn_ = driverBridge_->getJointTorques();
  const auto & rjo = robot.refJointOrder();
  for(size_t i = 0; i < rjo.size(); ++i)
  {
    auto jIndex = robot.jointIndexInMBC(i);
    robot.mbc().q[jIndex][0] = state_.qIn_[i];
    robot.mbc().jointTorque[jIndex][0] = state_.torqIn_[i];
  }

  for(auto & gripper : grippersInterfaces_)
  {
    auto & gripper_robot = controller.robots().robot(gripper.first);
    gripper_state_ = gripper.second->getPosition();

    for(size_t i = 0; i < gripper_robot.refJointOrder().size(); i++)
    {
      auto jIndex = gripper_robot.jointIndexInMBC(i);
      gripper_robot.mbc().q[jIndex][0] = gripper_state_[i];
    }
  }

  updateSensors(controller);
  updateControl(controller);

  robot.forwardKinematics();
  real.mbc() = robot.mbc();
}

template<ControlMode cm>
void URControlLoop<cm>::updateSensors(mc_control::MCGlobalController & controller)
{
  auto & robot = controller.robots().robot(name_);
  using GC = mc_control::MCGlobalController;
  using set_sensor_t = void (GC::*)(const std::string &, const std::vector<double> &);
  auto updateSensor = [&controller, &robot, this](set_sensor_t set_sensor, const std::vector<double> & data)
  {
    assert(sensorsBuffer_.size() == 6);
    std::memcpy(sensorsBuffer_.data(), data.data(), 6 * sizeof(double));
    (controller.*set_sensor)(robot.name(), sensorsBuffer_);
  };
  {
    // protects against conurrent write from the URControlLoop::updatecontrol thread
    std::lock_guard<std::mutex> lock(updateSensorsMutex_);
    updateSensor(&GC::setEncoderValues, state_.qIn_);
    updateSensor(&GC::setEncoderVelocities, state_.dqIn_);
    updateSensor(&GC::setJointTorques, state_.torqIn_);
  }
}

template<ControlMode cm>
void URControlLoop<cm>::updateControl(mc_control::MCGlobalController & controller)
{
  std::lock_guard<std::mutex> lock(updateControlMutex_);
  // In the same thread as MCGlobalController::run, thus we don't need synchronization here
  auto & robot = controller.robots().robot(name_);
  command_ = robot.mbc();

  for(auto & gripper : grippersInterfaces_)
  {
    auto & gripper_robot = controller.robots().robot(gripper.first);
    std::lock_guard<std::mutex> glock(gripperControlMutex_);
    gripper_command_.clear();
    for(size_t i = 0; i < gripper_robot.refJointOrder().size(); i++)
    {
      auto jIndex = gripper_robot.jointIndexInMBC(i);
      gripper_command_.push_back(gripper_robot.mbc().q[jIndex][0]);
    }
  }

  control_id_++;
}

template<ControlMode cm>
void URControlLoop<cm>::controlThread(mc_control::MCGlobalController & controller,
                                      std::mutex & startM,
                                      std::condition_variable & startCV,
                                      bool & start,
                                      bool & running)
{
  {
    std::unique_lock<std::mutex> lock(startM);
    startCV.wait(lock, [&]() { return start; });
  }

  while(running)
  {
    driverBridge_->sync();
    {
      std::lock_guard<std::mutex> lock(updateSensorsMutex_);
      state_.qIn_ = driverBridge_->getActualQ();
      state_.torqIn_ = driverBridge_->getJointTorques();
      driverBridge_->setDataRead();
    }
    auto command = rbd::MultiBodyConfig{};
    {
      std::lock_guard<std::mutex> lock(updateControlMutex_);
      command = command_;
    }
    using namespace std::chrono;
    auto time_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    control_.control(*driverBridge_, controller.robots().robot(name_), command);
  }
}

template<ControlMode cm>
void URControlLoop<cm>::gripperThread(const std::string & grippe_name,
                                      std::mutex & startM,
                                      std::condition_variable & startCV,
                                      bool & start,
                                      bool & running)
{
  {
    std::unique_lock<std::mutex> lock(startM);
    startCV.wait(lock, [&]() { return start; });
  }

  float last_sent_pos;
  float last_target_pos;
  {
    std::lock_guard<std::mutex> lock(gripperControlMutex_);
    last_sent_pos = gripper_command_[0]; // FIXME don't use [0]
    last_target_pos = gripper_command_[0]; // FIXME don't use [0]
  }

  const float steady_threshold = 0.001f;
  int stable_count = 0;

  // TODO; change stable_required so that this function can be put in controlThread() ?
  const int stable_required = 5;

  while(running)
  {
    {
      std::lock_guard<std::mutex> lock(gripperSensorMutex_);
      gripper_state_ = grippersInterfaces_[grippe_name]->getPosition();
    }

    double gripper_command = 0.0f;
    {
      std::lock_guard<std::mutex> lock(gripperControlMutex_);
      gripper_command = gripper_command_[0]; // FIXME don't use [0]
    }

    if(std::abs(gripper_command - last_target_pos) > steady_threshold)
    {
      stable_count = 0;
      last_target_pos = gripper_command;
    }
    else
    {
      stable_count++;
    }

    if(stable_count >= stable_required && std::abs(gripper_command - last_sent_pos) > steady_threshold)
    {
      grippersInterfaces_[grippe_name]->setPosition({gripper_command}); // FIXME don't use {}
      last_sent_pos = gripper_command;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

} // namespace mc_rtde
