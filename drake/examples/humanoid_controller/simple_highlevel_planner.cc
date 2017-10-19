#include "drake/examples/humanoid_controller/simple_highlevel_planner.h"

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "drake/systems/framework/leaf_system.h"

#include "drake/systems/controllers/qp_inverse_dynamics/robot_kinematic_state.h"
#include "drake/examples/valkyrie/valkyrie_constants.h"
#include "drake/manipulation/util/robot_state_msg_translator.h"

namespace drake {
namespace examples {
namespace humanoid_controller {
namespace simple_highlevel_planner {
namespace {

using systems::controllers::qp_inverse_dynamics::RobotKinematicState;

template <typename ValueType>
ValueType& get_mutable_value(systems::State<double>* state, int index) {
  DRAKE_DEMAND(state);
  return state->get_mutable_abstract_state()
      ->get_mutable_value(index)
      .GetMutableValue<ValueType>();
}

}  // namespace

SimpleHighlevelPlanner::SimpleHighlevelPlanner(const RigidBodyTree<double>* robot) :
  robot_(*robot) {
  input_port_index_kinematic_state_ = DeclareAbstractInputPort().get_index();
  output_port_index_plan_output_ =
      DeclareAbstractOutputPort(robot_plan_simple_t(),
                                &SimpleHighlevelPlanner::CopyOutPlanOutput)
          .get_index();

  set_name("HumanoidSimpleHighlevelPlanner");
  DeclarePeriodicUnrestrictedUpdate(0.2); // TODO remove this?
  abs_state_index_plan_output_ = DeclareAbstractState(
      systems::AbstractValue::Make<robot_plan_simple_t>(robot_plan_simple_t()));
}

// TODO(kazu) figure out what this does, along with get_mutable_value
void SimpleHighlevelPlanner::CopyOutPlanOutput(
    const systems::Context<double>& context,
    robot_plan_simple_t* output) const {
  robot_plan_simple_t& plan_output = *output;
  plan_output = context.get_abstract_state<robot_plan_simple_t>(abs_state_index_plan_output_);
}

void SimpleHighlevelPlanner::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
    systems::State<double>* state) const {
  // Inputs:
  const RobotKinematicState<double>* rs =
      EvalInputValue<RobotKinematicState<double>>(
          context, input_port_index_kinematic_state_);

  VectorX<double> q = examples::valkyrie::RPYValkyrieFixedPointState().head(
      examples::valkyrie::kRPYValkyrieDof);
  VectorX<double> v = VectorX<double>::Zero(robot_.get_num_velocities());

  const manipulation::RobotStateLcmMessageTranslator translator(robot_);

  // Make a plan
  robot_plan_simple_t& msg = get_mutable_value<robot_plan_simple_t>(state, abs_state_index_plan_output_);
  msg.utime = time(NULL);
  msg.num_states = 1;
  msg.plan.resize(msg.num_states);
  msg.plan_info.resize(msg.num_states, 1);

  // Add desired body/CoM poses
  msg.num_body_poses = 1;
  msg.desired_poses.resize(msg.num_body_poses);

  if(std::abs(rs->get_com()[2]-0.8) < 0.1)
    com_z_d_ = 0.96;
  else if(std::abs(rs->get_com()[2]-0.96) < 0.1)
    com_z_d_ = 0.8;

  msg.desired_poses[0].body_name = "com";
  bot_core::pose_t com_pose_des;
  com_pose_des.pos[0] = 0.0;
  com_pose_des.pos[1] = 0.0; // 0.07;
  com_pose_des.pos[2] = com_z_d_; //0.962;
  msg.desired_poses[0].body_pose = com_pose_des;

//  msg.desired_poses[1].body_name = "rightPalm";
//  bot_core::pose_t righthand_pose_des;
//  righthand_pose_des.pos[0] = 0.43;
//  righthand_pose_des.pos[1] = -0.38;
//  righthand_pose_des.pos[2] = 0.94;
//  msg.desired_poses[1].body_pose = righthand_pose_des;

//  msg.desired_poses[2].body_name = "leftPalm";
//  bot_core::pose_t lefthand_pose_des;
//  lefthand_pose_des.pos[0] = 0.43;
//  lefthand_pose_des.pos[1] = 0.38;
//  lefthand_pose_des.pos[2] = 0.94;
//  msg.desired_poses[2].body_pose = lefthand_pose_des;

  translator.InitializeMessage(&(msg.plan[0]));
  translator.EncodeMessageKinematics(q, v, &(msg.plan[0]));
  msg.plan[0].utime = 1e6;

}


}  // namespace simple_highlevel_planner
}  // namespace humanoid_controller
}  // namespace examples
}  // namespace drake
