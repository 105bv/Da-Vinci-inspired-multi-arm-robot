// C++17 kinematics and waypoint checks for the September one-arm milestone.
// Units: metres, radians, seconds. Dimensions match davinci_description/urdf/arm.urdf.xacro.
#ifndef DAVINCI_MOTION__ARM_MODEL_HPP_
#define DAVINCI_MOTION__ARM_MODEL_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>

namespace davinci_motion
{
constexpr double pi = 3.14159265358979323846;
constexpr double radians(double d) {return d * pi / 180.0;}
using Point = std::array<double, 3>;
using Joints = std::vector<double>;

struct Pose
{
  std::array<Point, 5> points;
  double pitch;
  const Point & tip() const {return points.back();}
};

struct Waypoint
{
  std::string name;
  Joints positions;
};

class ArmModel
{
public:
  static constexpr double base_height = 0.12;
  static constexpr double upper = 0.22;
  static constexpr double forearm = 0.20;
  static constexpr double tool = 0.06;

  explicit ArmModel(int dof) : dof_(dof)
  {
    if (dof != 3 && dof != 4) {
      throw std::invalid_argument("dof must be 3 or 4");
    }
  }

  int dof() const {return dof_;}

  std::vector<std::string> joint_names() const
  {
    std::vector<std::string> names{"base_yaw", "shoulder_pitch", "elbow_pitch"};
    if (dof_ == 4) {names.push_back("wrist_pitch");}
    return names;
  }

  Joints home() const
  {
    Joints q{0.0, radians(65), radians(-95)};
    if (dof_ == 4) {q.push_back(radians(-60));}
    return q;
  }

  static double distance(const Point & a, const Point & b)
  {
    return std::hypot(std::hypot(a[0] - b[0], a[1] - b[1]), a[2] - b[2]);
  }

  void validate(const Joints & q) const
  {
    constexpr std::array<double, 4> low{-170, 0, -150, -150};
    constexpr std::array<double, 4> high{170, 150, 0, 150};
    if (q.size() != static_cast<std::size_t>(dof_)) {
      throw std::invalid_argument("Wrong number of joints");
    }
    for (std::size_t i = 0; i < q.size(); ++i) {
      if (!std::isfinite(q[i]) || q[i] < radians(low[i]) - 1e-9 ||
        q[i] > radians(high[i]) + 1e-9)
      {
        throw std::invalid_argument("Invalid or out-of-limit joint " + std::to_string(i + 1));
      }
    }
  }

  Pose forward(const Joints & q) const
  {
    validate(q);
    Pose pose{};
    pose.points[0] = {0, 0, 0};
    pose.points[1] = {0, 0, base_height};
    double radius = 0, z = base_height, angle = 0;
    constexpr std::array<double, 3> lengths{upper, forearm, tool};
    for (std::size_t i = 0; i < lengths.size(); ++i) {
      angle += i + 1 < q.size() ? q[i + 1] : 0;
      radius += lengths[i] * std::cos(angle);
      z += lengths[i] * std::sin(angle);
      pose.points[i + 2] = {radius * std::cos(q[0]), radius * std::sin(q[0]), z};
    }
    pose.pitch = angle;
    return pose;
  }

  // Elbow-up analytic branch; 3 DOF controls position, 4 DOF also sets tool pitch.
  Joints inverse(const Point & p, double pitch = -pi / 2) const
  {
    for (double v : p) {
      if (!std::isfinite(v)) {throw std::invalid_argument("Target must be finite");}
    }
    if (!std::isfinite(pitch)) {throw std::invalid_argument("Pitch must be finite");}
    double r = std::hypot(p[0], p[1]), h = p[2] - base_height;
    if (r < 0.001) {throw std::invalid_argument("Target is at the yaw singularity");}
    const double yaw = std::atan2(p[1], p[0]);
    const double l2 = dof_ == 3 ? forearm + tool : forearm;
    if (dof_ == 4) {
      r -= tool * std::cos(pitch);
      h -= tool * std::sin(pitch);
    }
    const double c = (r * r + h * h - upper * upper - l2 * l2) / (2 * upper * l2);
    if (c < -1 - 1e-10 || c > 1 + 1e-10) {
      throw std::invalid_argument("Target is outside geometric reach");
    }
    const double elbow = -std::acos(std::clamp(c, -1.0, 1.0));
    const double shoulder = std::atan2(h, r) -
      std::atan2(l2 * std::sin(elbow), upper + l2 * std::cos(elbow));
    Joints q{yaw, shoulder, elbow};
    if (dof_ == 4) {q.push_back(pitch - shoulder - elbow);}
    validate_clearance(q);
    return q;
  }

  // Limited checks against the floor and pedestal. This is NOT collision-aware
  // planning: no mesh self-collision, objects, conveyor, or other-arm checks.
  void validate_clearance(const Joints & q) const
  {
    const auto pose = forward(q);
    for (std::size_t i = 2; i < pose.points.size(); ++i) {
      const double radius = i == 4 ? 0.013 : 0.020;
      if (std::min(pose.points[i - 1][2], pose.points[i][2]) < radius) {
        throw std::invalid_argument("Link intersects floor clearance");
      }
      if (i >= 3) {
        for (int j = 0; j <= 40; ++j) {
          const double t = j / 40.0;
          Point p{};
          for (std::size_t k = 0; k < 3; ++k) {
            p[k] = pose.points[i - 1][k] * (1 - t) + pose.points[i][k] * t;
          }
          if (p[2] < 0.10 + radius && std::hypot(p[0], p[1]) < 0.064 + radius) {
            throw std::invalid_argument("Distal link intersects pedestal clearance");
          }
        }
      }
    }
  }

  void validate_path(const Joints & a, const Joints & b) const
  {
    validate(a);
    validate(b);
    for (int step = 0; step <= 200; ++step) {
      const double u = step / 200.0;
      Joints q(a.size());
      for (std::size_t i = 0; i < q.size(); ++i) {q[i] = a[i] + (b[i] - a[i]) * u;}
      validate_clearance(q);
    }
  }

  std::vector<Waypoint> sequence(bool require_vertical = false) const
  {
    const std::vector<std::pair<std::string, Point>> targets{
      {"approach_pickup", {0.30, -0.14, 0.18}},
      {"lower_at_pickup", {0.30, -0.14, 0.035}},
      {"lift_from_pickup", {0.30, -0.14, 0.18}},
      {"transfer_to_placement", {-0.06, -0.31, 0.18}},
      {"lower_at_placement", {-0.06, -0.31, 0.035}},
      {"retreat_from_placement", {-0.06, -0.31, 0.18}}
    };
    std::vector<Waypoint> steps{{"home", home()}};
    for (const auto & target : targets) {
      auto q = inverse(target.second);
      if (require_vertical && std::abs(forward(q).pitch + pi / 2) > radians(5)) {
        throw std::invalid_argument(
                "3-DOF task cannot maintain vertical tool orientation; use 4 DOF");
      }
      validate_path(steps.back().positions, q);
      steps.push_back({target.first, q});
    }
    validate_path(steps.back().positions, home());
    steps.push_back({"return_home", home()});
    return steps;
  }

  static double duration(const Joints & a, const Joints & b)
  {
    if (a.size() != b.size() || a.empty()) {
      throw std::invalid_argument("Mismatched trajectory endpoints");
    }
    double delta = 0;
    for (std::size_t i = 0; i < a.size(); ++i) {
      if (!std::isfinite(a[i]) || !std::isfinite(b[i])) {
        throw std::invalid_argument("Non-finite trajectory endpoint");
      }
      delta = std::max(delta, std::abs(b[i] - a[i]));
    }
    // Quintic maxima: 1.875*d/T speed, approximately 5.774*d/T^2 acceleration.
    // Nominal command limits 1 rad/s, 3 rad/s^2, below model joint velocity limits.
    return std::max({2.5, 1.875 * delta, std::sqrt(5.774 * delta / 3.0)});
  }

private:
  int dof_;
};
}  // namespace davinci_motion
#endif  // DAVINCI_MOTION__ARM_MODEL_HPP_
