// One-arm trajectory execution and verification. C++17 / ROS 2 Jazzy.
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "davinci_motion/arm_model.hpp"

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using Action = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
using davinci_motion::ArmModel;
using davinci_motion::Joints;

namespace
{
volatile std::sig_atomic_t interrupted = 0;
void on_signal(int) {interrupted = 1;}
}  // namespace

class MotionTest : public rclcpp::Node
{
public:
  MotionTest()
  : Node("davinci_motion"),
    model_(declare_parameter<int>("dof", 3)),
    names_(model_.joint_names())
  {
    cycles_ = declare_parameter<int>("cycles", 3);
    require_vertical_ = declare_parameter<bool>("require_vertical", false);
    report_path_ = declare_parameter<std::string>("report_path", "davinci-motion-results.csv");
    if (cycles_ < 1 || cycles_ > 100) {
      throw std::invalid_argument("cycles must be between 1 and 100");
    }
    client_ = rclcpp_action::create_client<Action>(
      this, "/arm_controller/follow_joint_trajectory");
    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        if (msg->name.size() != msg->position.size()) {return;}
        std::unordered_map<std::string, double> values;
        for (std::size_t i = 0; i < msg->name.size(); ++i) {
          values[msg->name[i]] = msg->position[i];
        }
        Joints next;
        for (const auto & name : names_) {
          const auto it = values.find(name);
          if (it == values.end() || !std::isfinite(it->second)) {return;}
          next.push_back(it->second);
        }
        actual_ = next;
        received_at_ = Clock::now();
        received_ = true;
      });
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      "~/stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        stop_requested_ = true;
        response->success = true;
        response->message = "Stop requested. Check cancellation outcome and final report.";
      });
  }

  void attach(rclcpp::Executor * executor) {executor_ = executor;}

  int run()
  {
    std::ofstream report(report_path_);
    if (!report) {
      RCLCPP_ERROR(get_logger(), "Cannot write report: %s", report_path_.c_str());
      return 1;
    }
    report << "# dof=" << model_.dof() << "\n";
    report << "# Arm motion only: no grasp, conveyor, or camera implementation.\n";
    report << "cycle,waypoint,wall_seconds,max_final_error_rad,result\n";
    int completed = 0;
    try {
      // Preflight the entire sequence before requesting any movement.
      const auto steps = model_.sequence(require_vertical_);
      wait_for([this]() {return client_->action_server_is_ready();}, 90.0,
        "Action server unavailable; check arm_controller is active", false);
      wait_for([this]() {return fresh_feedback();}, 10.0,
        "No fresh joint feedback", false);
      for (int cycle = 1; cycle <= cycles_; ++cycle) {
        for (const auto & step : steps) {
          execute(step, cycle, report);
        }
        ++completed;
        RCLCPP_INFO(get_logger(), "CYCLE %d/%d complete", completed, cycles_);
      }
      report << "# completed_cycles=" << completed << "\n# result=PASS\n";
      report.flush();
      if (!report) {throw std::runtime_error("Report write failed");}
      RCLCPP_INFO(get_logger(), "PASS: %d cycles; report %s", completed, report_path_.c_str());
      return 0;
    } catch (const std::exception & error) {
      const bool cancelled = cancel_active();
      report << "# completed_cycles=" << completed << "\n# result=FAIL\n";
      report << "# error=" << error.what() << "\n";
      report << "# cancellation_confirmed=" << (cancelled ? "true" : "false") << "\n";
      report.flush();
      RCLCPP_ERROR(get_logger(), "FAIL: %s", error.what());
      if (!cancelled) {
        RCLCPP_ERROR(get_logger(),
          "Stop could not be confirmed. Stop the simulator and inspect it before restarting.");
      }
      return stop_requested_ || interrupted ? 2 : 1;
    }
  }

private:
  bool fresh_feedback() const
  {
    return received_ &&
           std::chrono::duration<double>(Clock::now() - received_at_).count() < 0.75;
  }

  template<typename Predicate>
  void wait_for(Predicate predicate, double seconds, const std::string & message,
    bool monitor_feedback)
  {
    const auto end = Clock::now() + std::chrono::duration<double>(seconds);
    while (rclcpp::ok() && Clock::now() < end) {
      executor_->spin_some();
      if (stop_requested_ || interrupted) {throw std::runtime_error("Stop requested");}
      if (monitor_feedback && !fresh_feedback()) {
        throw std::runtime_error("Joint feedback watchdog expired");
      }
      if (predicate()) {return;}
      std::this_thread::sleep_for(10ms);
    }
    throw std::runtime_error(message);
  }

  static trajectory_msgs::msg::JointTrajectoryPoint point(const Joints & q, double time)
  {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = q;
    p.velocities.assign(q.size(), 0.0);
    p.accelerations.assign(q.size(), 0.0);
    p.time_from_start = rclcpp::Duration::from_seconds(time);
    return p;
  }

  void execute(const davinci_motion::Waypoint & step, int cycle, std::ofstream & report)
  {
    wait_for([this]() {return fresh_feedback();}, 5.0, "Missing feedback", false);
    model_.validate_path(actual_, step.positions);
    const double duration = ArmModel::duration(actual_, step.positions) + 0.1;
    Action::Goal goal;
    goal.trajectory.joint_names = names_;
    goal.trajectory.points = {point(actual_, 0.1), point(step.positions, duration)};
    goal.goal_time_tolerance = rclcpp::Duration::from_seconds(3.0);
    // Supplying p/v/a at both endpoints selects quintic interpolation in JTC.
    RCLCPP_INFO(get_logger(), "MOVE cycle=%d waypoint=%s", cycle, step.name.c_str());
    const auto start = Clock::now();
    result_ = {};
    active_goal_.reset();
    pending_ = client_->async_send_goal(goal);
    goal_outstanding_ = true;
    wait_for([this]() {return pending_.wait_for(0s) == std::future_status::ready;},
      10.0, "Goal acknowledgement timeout", true);
    active_goal_ = pending_.get();
    if (!active_goal_) {
      goal_outstanding_ = false;
      throw std::runtime_error("Controller rejected goal " + step.name);
    }
    result_ = client_->async_get_result(active_goal_);
    wait_for([this]() {return result_.wait_for(0s) == std::future_status::ready;},
      duration * 4 + 10, "Trajectory result timeout " + step.name, true);
    const auto result = result_.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
      result.result->error_code != Action::Result::SUCCESSFUL)
    {
      goal_outstanding_ = false;
      throw std::runtime_error(
              "Trajectory failed " + step.name + ": " +
              (result.result ? result.result->error_string : "no result body"));
    }
    goal_outstanding_ = false;
    active_goal_.reset();
    const auto result_at = Clock::now();
    wait_for([this, result_at]() {return received_at_ >= result_at;},
      3.0, "No new joint feedback after action completion", true);
    double max_error = 0;
    for (std::size_t i = 0; i < actual_.size(); ++i) {
      max_error = std::max(max_error, std::abs(actual_[i] - step.positions[i]));
    }
    if (max_error > 0.03) {
      throw std::runtime_error("Final joint error exceeds 0.03 rad at " + step.name);
    }
    const double seconds = std::chrono::duration<double>(Clock::now() - start).count();
    report << cycle << "," << step.name << "," << seconds << "," << max_error << ",PASS\n";
    report.flush();
    if (!report) {throw std::runtime_error("Report write failed");}
    RCLCPP_INFO(get_logger(), "DONE %s max_error=%.5f rad", step.name.c_str(), max_error);
  }

  bool cancel_active()
  {
    if (!goal_outstanding_) {return true;}
    try {
      // This test owns the arm action server exclusively while running.
      // Cancel-all also covers acknowledgement timeout with an unknown goal handle.
      auto cancelled = client_->async_cancel_all_goals();
      const auto deadline = Clock::now() + 3s;
      while (rclcpp::ok() && Clock::now() < deadline &&
        cancelled.wait_for(0s) != std::future_status::ready)
      {
        executor_->spin_some();
        std::this_thread::sleep_for(10ms);
      }
      if (cancelled.wait_for(0s) != std::future_status::ready) {return false;}
      const auto response = cancelled.get();
      if (!response || response->goals_canceling.empty()) {return false;}
      // Acknowledging cancellation is not the terminal action result.
      if (!active_goal_ && pending_.valid() &&
        pending_.wait_for(0s) == std::future_status::ready)
      {
        active_goal_ = pending_.get();
      }
      if (!active_goal_) {return false;}
      if (!result_.valid()) {result_ = client_->async_get_result(active_goal_);}
      const auto result_deadline = Clock::now() + 5s;
      while (rclcpp::ok() && Clock::now() < result_deadline &&
        result_.wait_for(0s) != std::future_status::ready)
      {
        executor_->spin_some();
        std::this_thread::sleep_for(10ms);
      }
      if (result_.wait_for(0s) != std::future_status::ready) {return false;}
      const auto terminal = result_.get();
      return terminal.code == rclcpp_action::ResultCode::CANCELED ||
             terminal.code == rclcpp_action::ResultCode::ABORTED ||
             terminal.code == rclcpp_action::ResultCode::SUCCEEDED;
    } catch (...) {
      return false;
    }
  }

  ArmModel model_;
  std::vector<std::string> names_;
  int cycles_;
  bool require_vertical_;
  std::string report_path_;
  rclcpp::Executor * executor_ = nullptr;
  rclcpp_action::Client<Action>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  Joints actual_;
  Clock::time_point received_at_{};
  bool received_ = false;
  bool stop_requested_ = false;
  bool goal_outstanding_ = false;
  GoalHandle::SharedPtr active_goal_;
  std::shared_future<GoalHandle::SharedPtr> pending_;
  std::shared_future<GoalHandle::WrappedResult> result_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  int code = 1;
  try {
    auto node = std::make_shared<MotionTest>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    node->attach(&executor);
    code = node->run();
    executor.remove_node(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("davinci_motion"), "%s", error.what());
  }
  rclcpp::shutdown();
  return code;
}
