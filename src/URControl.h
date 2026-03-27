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
  URControlLoop(const std::string & name, const mc_control::Configuration & config, double cycle_s);

  void init(mc_control::MCGlobalController & controller);

  void attachGripper(const std::string tool_name, const mc_control::Configuration & tool_config);

  void setActiveRobot(mc_control::MCGlobalController & controller, std::string & active_name_);

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
  mc_control::Configuration config_;

  std::string name_;
  std::string ip_;
  rbd::MultiBodyConfig command_;
  URSensorInfo state_;

  std::vector<double> tools_command_;
  std::vector<double> tools_state_;

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
URControlLoop<cm>::URControlLoop(const std::string & name, const mc_control::Configuration & config, double cycle_s)
: name_(name), config_(config), logger_(mc_rtc::Logger::Policy::THREADED, "/tmp", "mc-rtde-" + name_), cycle_s_(cycle_s)
{
  ip_ = std::string(config_("ip"));
  std::string driverName = config_("driver", std::string{"ur_rtde"});
  Driver driver = (driverName == "ur_rtde") ? Driver::ur_rtde : Driver::ur_modern_driver;
  if(driver == Driver::ur_rtde)
  {
    driverBridge_ = std::make_unique<DriverBridgeRTDE>(ip_);
  }
  else
  {
    driverBridge_ = std::make_unique<DriverBridgeURModernDriver>(ip_, cycle_s_);
  }
}

template<ControlMode cm>
void URControlLoop<cm>::attachGripper(const std::string tool_name, const mc_control::Configuration & tool_config)
{
  if(tool_config("type") == "robotiq")
  {
    int port = tool_config("port", 63352);
    std::string ip = tool_config("ip", std::string(ip_));
    if(!grippersInterfaces_.count(tool_name))
    {
      auto gripper = std::make_shared<mc_rtde::GripperRobotiq>(ip, port);
      gripper->connect();
      grippersInterfaces_.try_emplace(tool_name, std::move(gripper));
    }
    else
      mc_rtc::log::error_and_throw("Gripper {} is already attached to the robot", tool_name);
  }
  else
  {
    mc_rtc::log::warning("Gripper type : {} is not supported", tool_config("type"));
  }
}

template<ControlMode cm>
void URControlLoop<cm>::init(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("init {}", name_);

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

  // for(auto & gripper : grippersInterfaces_)
  // {
  //   auto & gripper_robot = controller.robots().robot(gripper.first);
  //   gripper_state_ = gripper.second->getPosition();

  //   for(size_t i = 0; i < gripper_robot.refJointOrder().size(); i++)
  //   {
  //     auto jIndex = gripper_robot.jointIndexInMBC(i);
  //     gripper_robot.mbc().q[jIndex][0] = gripper_state_[i];
  //   }
  // }

  updateSensors(controller);
  updateControl(controller);

  robot.forwardKinematics();
  real.mbc() = robot.mbc();
}

template<ControlMode cm>
void URControlLoop<cm>::setActiveRobot(mc_control::MCGlobalController & controller, std::string & active_name)
{
  mc_rtc::log::info("[mc_rtde] New robot {} detected", active_name);

  name_ = active_name;
  auto & robot = controller.controller().robots().robot(name_);
  for(size_t i = 0; i < state_.qIn_.size(); ++i)
  {
    auto jIndex = robot.jointIndexInMBC(i);
    robot.mbc().q[jIndex][0] = state_.qIn_[i];
    robot.mbc().jointTorque[jIndex][0] = state_.torqIn_[i];
  }

  if(config_.has(active_name))
  {
    const mc_control::Configuration active_config = config_(active_name);
    for(const std::string & tool_name : active_config.keys())
    {
      const mc_control::Configuration tool_config = active_config(tool_name);
      if(!tool_config.has("type"))
      {
        mc_rtc::log::error_and_throw("Tool configuration must contain 'type'");
      }
      attachGripper(tool_name, tool_config);
    }
    // TODO: sync tools state
    for(auto & tool : grippersInterfaces_)
    {
      std::lock_guard<std::mutex> lock(gripperSensorMutex_);
      std::vector<double> s = tool.second->getState();
      tools_state_.insert(tools_state_.end(), s.begin(), s.end());
    }
  }
  else
  {
    mc_rtc::log::warning("[mc_rtde] Robot variation {} configuration is not available", active_name);
  };
}

template<ControlMode cm>
void URControlLoop<cm>::updateSensors(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("updateSensors {}", name_);

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

  // QUESTION: this code might not do what i need it to do. I need to put tools_state to mc_rtc
  // what happend when robot is ur5e_gripper?
  std::vector<double> tools_state;
  for(auto & tool : grippersInterfaces_)
  {
    std::lock_guard<std::mutex> lock(gripperSensorMutex_);
    std::vector<double> s = tool.second->getState();
    tools_state.insert(tools_state.end(), s.begin(), s.end());
  }
  tools_state_ = tools_state;

  size_t state_idx = 0;
  for(size_t i = 0; i < robot.mb().joints().size(); ++i)
  {
    const auto & j = robot.mb().joint(i);
    if(i < 6) continue;
    if(j.dof() == 1 && !j.isMimic())
    {
      if(state_idx < tools_state_.size())
      {
        std::lock_guard<std::mutex> lock(updateSensorsMutex_);
        robot.mbc().q[i][0] = tools_state_[state_idx];
        state_idx++;
      }
    }
  }
}

template<ControlMode cm>
void URControlLoop<cm>::updateControl(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("updateControl");
  {
    std::lock_guard<std::mutex> lock(updateControlMutex_);
    // In the same thread as MCGlobalController::run, thus we don't need synchronization here
    auto & robot = controller.robots().robot(name_);
    command_ = robot.mbc();
  }
  // for(auto & gripper : grippersInterfaces_)
  // {
  //   auto & gripper_robot = controller.robots().robot(gripper.first);
  //   std::lock_guard<std::mutex> glock(gripperControlMutex_);
  //   gripper_command_.clear();
  //   for(size_t i = 0; i < gripper_robot.refJointOrder().size(); i++)
  //   {
  //     auto jIndex = gripper_robot.jointIndexInMBC(i);
  //     gripper_command_.push_back(gripper_robot.mbc().q[jIndex][0]);
  //   }
  // }

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

// since command_ = robot.mbc(), gripperThread() should looks somewhat simpler (similar to controlThread)
// command_ should be pass directly to gripperthread similar to how command_ is passed into controlThread
// how to handle command_ should be implemented in GripperRobotiq.cpp (or sth), not here

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

  std::vector<double> last_sent_pos;
  std::vector<double> last_target_pos;
  {
    std::lock_guard<std::mutex> lock(gripperControlMutex_);
    last_sent_pos = tools_command_;
    last_target_pos = tools_command_;
  }

  const float steady_threshold = 0.001f;
  int stable_count = 0;
  const int stable_required = 5;

  while(running)
  {
    {
      std::lock_guard<std::mutex> lock(gripperSensorMutex_);
      tools_state_ = grippersInterfaces_[grippe_name]->getPosition();
    }

    std::vector<double> tools_command = {};
    {
      std::lock_guard<std::mutex> lock(gripperControlMutex_);
      tools_command = tools_command_;
    }

    auto vectorDiff = [](const std::vector<double> & a, const std::vector<double> & b, double threshold) -> bool
    {
      for(size_t i = 0; i < a.size(); i++)
      {
        if(std::abs(a[i] - b[i]) > threshold) return true;
      }
      return false;
    };

    if(vectorDiff(tools_command, last_target_pos, steady_threshold))
    {
      stable_count = 0;
      last_target_pos = tools_command;
    }
    else
    {
      stable_count++;
    }

    if(stable_count >= stable_required && vectorDiff(tools_command, last_sent_pos, steady_threshold))
    {
      grippersInterfaces_[grippe_name]->setPosition(tools_command);
      last_sent_pos = tools_command;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

} // namespace mc_rtde
