#include "drake/examples/humanoid_controller/dev/humanoid_simple_plan.h"

#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>

#include "robotlocomotion/robot_plan_t.hpp"
#include "drake/examples/humanoid_controller/lcm_custom_types/robotlocomotion/robot_plan_simple_t.hpp"

#include "drake/systems/controllers/qp_inverse_dynamics/lcm_utils.h"
#include "drake/systems/controllers/setpoint.h"

namespace drake {
namespace examples {
namespace humanoid_controller {

using systems::controllers::plan_eval::ContactState;
using systems::controllers::qp_inverse_dynamics::ParamSet;
using systems::controllers::qp_inverse_dynamics::QpInput;
using systems::controllers::qp_inverse_dynamics::RobotKinematicState;

template <typename T>
void HumanoidLocomotionPlan<T>::InitializeGenericPlanDerived(
    const RobotKinematicState<T>& robot_status, const ParamSet& paramset,
    const RigidBodyTreeAliasGroups<T>& alias_groups) {
  unused(paramset);

  // Knots are constant, the second time doesn't matter as long as it's larger.
  const std::vector<T> times = {robot_status.get_time(),
                                robot_status.get_time() + 1};

  // Current com q and v.
  Vector4<T> xcom;
  xcom << robot_status.get_com().template head<2>(),
      robot_status.get_com_velocity().template head<2>();
  // Set desired com q to current.
  MatrixX<T> com_d = robot_status.get_com().template head<2>();
  PiecewisePolynomial<T> zmp_d =
      PiecewisePolynomial<T>::ZeroOrderHold(times, {com_d, com_d});
  // Makes a zmp planner that stays still.
  zmp_planner_.Plan(zmp_d, xcom, zmp_height_);

  com_d_ = robot_status.get_com();

  // Assumes double support with both feet.
  ContactState double_support;
  double_support.insert(alias_groups.get_body("left_foot"));
  double_support.insert(alias_groups.get_body("right_foot"));
  this->UpdateContactState(double_support);

  // Sets body tracking trajectories for pelvis and torso.
  // TODO (kazu) read these bodies from config file
  // TODO (kazu) find where gains are read
  const std::vector<std::string> tracked_body_names = {"pelvis", "torso", "rightPalm", "leftPalm"};
  MatrixX<T> position;
  for (const auto& name : tracked_body_names) {
    const RigidBody<T>* body = alias_groups.get_body(name);
    Isometry3<T> body_pose = robot_status.get_robot().CalcBodyPoseInWorldFrame(
        robot_status.get_cache(), *body);
    position = body_pose.translation();
    PiecewisePolynomial<T> pos_traj =
        PiecewisePolynomial<T>::ZeroOrderHold(times, {position, position});
    PiecewiseQuaternionSlerp<T> rot_traj(
        times, {body_pose.linear(), body_pose.linear()});

    manipulation::PiecewiseCartesianTrajectory<T> body_traj(pos_traj, rot_traj);
    this->set_body_trajectory(body, body_traj);
  }
}

template <typename T>
void HumanoidLocomotionPlan<T>::UpdateQpInputGenericPlanDerived(
    const RobotKinematicState<T>& robot_status, const ParamSet& paramset,
    const RigidBodyTreeAliasGroups<T>& alias_groups, QpInput* qp_input) const {
  unused(paramset, alias_groups);

  // TODO get gains from paramset
//  Vector3<double> kp_com {40.0, 40.0, 20.0};
  Vector3<double> kp_com {20.0, 20.0, 40.0};
  Vector3<double> kd_com {12.0, 12.0, 10.0};
  Vector3<T> comdd_d = kp_com.cwiseProduct(com_d_ - robot_status.get_com()) +
      kd_com.cwiseProduct(Vector3<T>::Zero(3) - robot_status.get_com_velocity());

  // Zeros linear and angular momentum change.
  qp_input->mutable_desired_centroidal_momentum_dot()
      .mutable_values()
      .setZero();
  // Only sets the xy dimensions of the linear momentum change.
  qp_input->mutable_desired_centroidal_momentum_dot()
      .mutable_values()
      .segment<3>(3) = robot_status.get_robot().getMass() * comdd_d;
}

template <typename T>
void HumanoidLocomotionPlan<T>::HandlePlanGenericPlanDerived(
    const RobotKinematicState<T>& robot_status, const ParamSet& paramset,
    const RigidBodyTreeAliasGroups<T>& alias_groups,
    const systems::AbstractValue& plan) {
  unused(paramset);

  const auto& msg = plan.GetValueOrThrow<robotlocomotion::robot_plan_simple_t>();

  if (msg.utime == last_handle_plan_time_) return;

  // Saves the time stamp.
  last_handle_plan_time_ = msg.utime;

  // knots for setting up the splines.
  int length = static_cast<int>(msg.plan.size());
  if (length < 1) {
    drake::log()->warn(
        "HumanoidLocomotionPlan::HandlePlanGenericPlanDerived: "
            "received plan has less than 1 knots.");
    return;
  }

  const RigidBodyTree<T>& robot = robot_status.get_robot();
  KinematicsCache<T> cache = robot.CreateKinematicsCache();

  const double time_now = robot_status.get_time();
  VectorX<T> q = this->get_dof_trajectory().get_position(time_now);
  VectorX<T> v = VectorX<T>::Zero(robot.get_num_velocities());

  // Set the first knot points to the current desired values.
  std::vector<T> times(1, time_now);
  std::vector<MatrixX<T>> dof_knots(1, q);
  std::unordered_map<const RigidBody<T>*, std::vector<Isometry3<T>>> body_knots;
  std::vector<const RigidBody<T>*> tracked_bodies = {
      alias_groups.get_body("pelvis"), alias_groups.get_body("torso"),
      alias_groups.get_body("rightPalm"), alias_groups.get_body("leftPalm")};
  for (const RigidBody<T>* body : tracked_bodies) {
    body_knots[body] = std::vector<Isometry3<T>>(
        1, this->get_body_trajectory(body).get_pose(time_now));
  }
  std::vector<T> com_times(1, time_now);
  std::vector<MatrixX<T>> com_knots(1, zmp_planner_.get_nominal_com(time_now));

  const manipulation::RobotStateLcmMessageTranslator translator(
      robot_status.get_robot());

  const auto& desired_body_poses = msg.desired_poses;
  auto find_desired_body_pose = [&](std::string name) {
    for(int i=0; i< static_cast<int>(desired_body_poses.size()); ++i){
      if(desired_body_poses[i].body_name == name) return i;
    }
    return -1;
  };


  for (const bot_core::robot_state_t& keyframe : msg.plan) {
    translator.DecodeMessageKinematics(keyframe, q, v);
    const double time = static_cast<double>(keyframe.utime) / 1e6;

    cache.initialize(q);
    robot.doKinematics(cache, false);

    times.push_back(time_now + time);
    com_times.push_back(time_now + time);
    dof_knots.push_back(q);

    for (auto& body_knots_pair : body_knots) {
      const RigidBody<T>* body = body_knots_pair.first;
      Isometry3<double> desired_transform;
      std::vector<Isometry3<T>>& knots = body_knots_pair.second;

      int found_idx = find_desired_body_pose(body->get_name());
      if(found_idx != -1) {
        std::cout << body->get_name() << std::endl;
        const auto& des_pose = desired_body_poses[found_idx].body_pose;
        Vector3<double> desired_pos(des_pose.pos[0], des_pose.pos[1], des_pose.pos[2]);
        desired_transform.translate(desired_pos);
      }
      else {
        desired_transform = robot.CalcBodyPoseInWorldFrame(cache, *body);
      }
      knots.push_back(desired_transform);
    }

    // Computes com.
    com_d_ = robot.centerOfMass(cache);
    int com_idx = find_desired_body_pose("com");
    if(com_idx != -1) {
      const auto& des_pose = desired_body_poses[com_idx].body_pose;
      com_d_ = {des_pose.pos[0], des_pose.pos[1], des_pose.pos[2]};
    }
  }

  // Generates dof trajectories.
  {
    MatrixX<T> zeros = MatrixX<T>::Zero(robot.get_num_positions(), 1);
    this->set_dof_trajectory(manipulation::PiecewiseCubicTrajectory<T>(
        PiecewisePolynomial<T>::Cubic(times, dof_knots, zeros, zeros)));
  }

  // Generates body trajectories.
  {
    for (const auto& body_knots_pair : body_knots) {
      const RigidBody<T>* body = body_knots_pair.first;
      const std::vector<Isometry3<T>>& knots = body_knots_pair.second;

      manipulation::PiecewiseCartesianTrajectory<T> body_traj =
          manipulation::PiecewiseCartesianTrajectory<
              T>::MakeCubicLinearWithEndLinearVelocity(times, knots,
                                                       Vector3<T>::Zero(),
                                                       Vector3<T>::Zero());

      this->set_body_trajectory(body, body_traj);
    }
  }
}

template <typename T>
bool HumanoidLocomotionPlan<T>::IsRigidBodyTreeCompatible(
    const RigidBodyTree<T>& robot) const {
  if (robot.get_num_bodies() < 2) return false;

  const RigidBody<T>& root_body = robot.get_body(1);
  const DrakeJoint& root_joint = root_body.getJoint();
  if (!root_joint.is_floating()) return false;

  if (root_joint.get_num_positions() != root_joint.get_num_velocities())
    return false;

  if (robot.get_num_positions() != robot.get_num_velocities()) return false;

  return true;
}

template <typename T>
bool HumanoidLocomotionPlan<T>::IsRigidBodyTreeAliasGroupsCompatible(
    const RigidBodyTreeAliasGroups<T>& alias_groups) const {
  if (VerifyRigidBodyTreeAliasGroups(alias_groups, "pelvis") &&
      VerifyRigidBodyTreeAliasGroups(alias_groups, "torso") &&
      VerifyRigidBodyTreeAliasGroups(alias_groups, "left_foot") &&
      VerifyRigidBodyTreeAliasGroups(alias_groups, "right_foot")) {
    return true;
  } else {
    return false;
  }
}

template class HumanoidLocomotionPlan<double>;

}  // namespace humanoid_controller
}  // namespace examples
}  // namespace drake
