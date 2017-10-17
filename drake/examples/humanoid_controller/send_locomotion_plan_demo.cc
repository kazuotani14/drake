/**
 * @brief This is a demo program that sends a full body manipulation plan
 * encoded as a robotlocomotion::robot_plan_t to ValkyrieController. The plan
 * has most of the joints set to the nominal configuration except the right
 * shoulder joint, which can be commanded from a single command line argument.
 */
#include <gflags/gflags.h>
#include "lcm/lcm-cpp.hpp"
#include "robotlocomotion/robot_plan_t.hpp"
#include "drake/examples/humanoid_controller/lcm_custom_types/robotlocomotion/robot_plan_custom_t.hpp"

#include "drake/examples/valkyrie/valkyrie_constants.h"
#include "drake/manipulation/util/robot_state_msg_translator.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

//DEFINE_double(r_shy_offset, 0,
//              "Right shoulder pitch offset [rad].");
//  q[10] += FLAGS_r_shy_offset;

using std::default_random_engine;

namespace drake {
namespace examples {
namespace humanoid_controller {
namespace {

void send_manip_message() {
  RigidBodyTree<double> robot;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      "drake/examples/valkyrie/urdf/urdf/"
      "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf",
      multibody::joints::kRollPitchYaw, &robot);

  DRAKE_DEMAND(examples::valkyrie::kRPYValkyrieDof ==
               robot.get_num_positions());
  VectorX<double> q = examples::valkyrie::RPYValkyrieFixedPointState().head(
      examples::valkyrie::kRPYValkyrieDof);
  VectorX<double> v = VectorX<double>::Zero(robot.get_num_velocities());

  const manipulation::RobotStateLcmMessageTranslator translator(robot);

  // There needs to be at least 1 knot point, the controller will insert its
  // current desired q to the beginning to make the desired trajectories.
  //
  // Some notes about the message:
  // 1. The first timestamp needs to be bigger than 0.
  // 2. msg.utime needs to be different for different messages. The controller
  // ignores messages with the same utime.
  // 3. Assumes the message is packaged with a RPY floating joint.
  // 4. Assumes that the given knot points are stable given the current actual
  // contact points.
  // 5. Pelvis z and orientation + torso orientation are tracked in Cartesian
  // mode (These can be changed by the gains in the configuration file). CoM
  // is controlled by a LQR like controller, where the desired is given by the
  // knot points specified here.
  robotlocomotion::robot_plan_custom_t msg{};
  msg.utime = time(NULL);
  msg.num_states = 1;
  msg.plan.resize(msg.num_states);
  msg.plan_info.resize(msg.num_states, 1);

  msg.num_body_poses = 3;
  msg.body_names.resize(msg.num_body_poses);
  msg.body_pose_des.resize(msg.num_body_poses);
  msg.body_names[0] = "com";
  bot_core::pose_t com_pose_des;
  com_pose_des.pos[0] = 0.0;
  com_pose_des.pos[1] = 0.07;
  msg.body_pose_des[0] = com_pose_des;

  msg.body_names[1] = "rightPalm";
  bot_core::pose_t righthand_pose_des;
  righthand_pose_des.pos[0] = 0.43;
  righthand_pose_des.pos[1] = -0.38;
  righthand_pose_des.pos[2] = 0.94;
  msg.body_pose_des[1] = righthand_pose_des;

  msg.body_names[2] = "leftPalm";
  bot_core::pose_t lefthand_pose_des;
  lefthand_pose_des.pos[0] = 0.43;
  lefthand_pose_des.pos[1] = 0.38;
  lefthand_pose_des.pos[2] = 0.94;
  msg.body_pose_des[2] = lefthand_pose_des;

  translator.InitializeMessage(&(msg.plan[0]));
  translator.EncodeMessageKinematics(q, v, &(msg.plan[0]));
  msg.plan[0].utime = 1e6;

  lcm::LCM lcm;
  lcm.publish("VALKYRIE_MANIP_PLAN", &msg);

  sleep(1);
}

}  // namespace
}  // namespace humanoid_controller
}  // namespace examples
}  // namespace drake

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::humanoid_controller::send_manip_message();
  return 0;
}
