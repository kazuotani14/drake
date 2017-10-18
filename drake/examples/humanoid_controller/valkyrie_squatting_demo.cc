#include "drake/examples/humanoid_controller/simple_highlevel_planner.h"

#include "drake/common/find_resource.h"
#include "drake/examples/valkyrie/valkyrie_constants.h"
#include "drake/manipulation/util/robot_state_msg_translator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_driven_loop.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/manipulation/util/robot_state_msg_translator.h"
#include "drake/examples/humanoid_controller/humanoid_status_translator_system.h"

namespace drake {
namespace examples {
namespace humanoid_controller {

/*
 * The system outputs plans for HumanoidPlanEval, which then sends commands to the QP controller
 * Assumes that HumanoidController (valkyrie_balancing_demo) is running
 * Based on valkyrie_balancing_demo code
 */
void run_planner() {
  const std::string kModelFileName = FindResourceOrThrow(
      "drake/examples/valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf");
  const std::string kAliasGroupPath = FindResourceOrThrow(
      "drake/examples/humanoid_controller/"
          "config/valkyrie.alias_groups");

  drake::lcm::DrakeLcm lcm;

  RigidBodyTree<double> robot;
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      kModelFileName, multibody::joints::kRollPitchYaw, &robot);

  // Build diagram with planner and humanoid status translator
  systems::DiagramBuilder<double> builder;

  auto simple_planner = builder.AddSystem<simple_highlevel_planner::SimpleHighlevelPlanner>(&robot);
  RobotStateMsgToHumanoidStatusSystem* msg_to_humanoid_status =
      builder.AddSystem(std::make_unique<RobotStateMsgToHumanoidStatusSystem>(
          &robot, kAliasGroupPath));

  // Set up input and output
  auto robot_state_subscriber = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<bot_core::robot_state_t>(
          "EST_ROBOT_STATE", &lcm));
  auto plan_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<robotlocomotion::robot_plan_simple_t>(
          "VALKYRIE_MANIP_PLAN", &lcm));

  // lcm -> robot state
  builder.Connect(robot_state_subscriber->get_output_port(0),
                  msg_to_humanoid_status->get_input_port());
  // robot state -> planner
  builder.Connect(msg_to_humanoid_status->get_output_port(),
                  simple_planner->get_input_port_kinematic_state());
  // plan -> lcm
  builder.Connect(simple_planner->get_output_port_plan_output(),
                  plan_publisher->get_input_port(0));

  auto diagram = builder.Build();

  // Make a Lcm driven loop that's blocked by robot_state_subscriber.
  systems::lcm::LcmDrivenLoop loop(
      *diagram, *robot_state_subscriber, nullptr, &lcm,
      std::make_unique<
          systems::lcm::UtimeMessageToSeconds<bot_core::robot_state_t>>());

  // start the loop
  loop.RunToSecondsAssumingInitialized();
}

}  // namespace humanoid_controller
}  // namespace examples
}  // end namespace drake

int main() {
  drake::examples::humanoid_controller::run_planner();
}
