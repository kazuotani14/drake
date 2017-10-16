// Implements a passive simulation of the Drake-compatible description of the
// PR2 robot. There is no controller, but the contact parameters and integrator
// parameters are set to support reliable gripping of objects if a controller
// is added.

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");

namespace drake {
namespace examples {
namespace human_model {

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::lcm::DrakeLcm lcm;

  auto tree_ptr = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
//            FindResourceOrThrow("drake/examples/acrobot/Acrobot.urdf"),
//      FindResourceOrThrow(
//          "drake/examples/valkyrie/urdf/urdf/"
//              "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf"),
  FindResourceOrThrow(
//          "drake/examples/human_model/models/new_human_model.urdf"),
      "drake/examples/human_model/models/icub/icub_with_transmission.urdf"),
      multibody::joints::kRollPitchYaw, tree_ptr.get());

  int n_actuators = tree_ptr->get_num_actuators();
  drake::log()->info("Number of bodies: {}", tree_ptr->get_num_bodies());
  drake::log()->info("Number of actuators: {}", n_actuators);
  drake::log()->info("Mass: {}", tree_ptr->getMass());

  multibody::AddFlatTerrainToWorld(tree_ptr.get(), 100., 10.);

  systems::DiagramBuilder<double> builder;
  auto plant = builder.AddSystem<systems::RigidBodyPlant<double>>(
      std::move(tree_ptr));
  plant->set_name("plant");
  const auto& tree = plant->get_rigid_body_tree();

  systems::DrakeVisualizer& viz_publisher =
      *builder.template AddSystem<systems::DrakeVisualizer>(
          plant->get_rigid_body_tree(), &lcm);
  viz_publisher.set_name("visualizer_publisher");
  builder.Connect(plant->state_output_port(),
                          viz_publisher.get_input_port(0));

  // Send the robot's actuators zeros in absence of a controller.
  std::cout << "Setting constant actuator torque" << std::endl;
  int n_torques = plant->actuator_command_input_port().size();
  VectorX<double> constant_torques = VectorX<double>::Constant(n_torques, 0.0);
  auto constant_torque_source =
      builder.AddSystem<systems::ConstantVectorSource<double>>(
          constant_torques);
  builder.Connect(constant_torque_source->get_output_port(),
                          plant->actuator_command_input_port());

  // Contact parameters
  const double kStiffness = 100000;
  const double kDissipation = 5.0;
  const double kStaticFriction = 0.9;
  const double kDynamicFriction = 0.5;
  const double kStictionSlipTolerance = 0.01;
  plant->set_normal_contact_parameters(kStiffness, kDissipation);
  plant->set_friction_contact_parameters(kStaticFriction, kDynamicFriction,
                                          kStictionSlipTolerance);

  // Set up contact force visualization
  auto contact_viz =
      builder.template AddSystem<systems::ContactResultsToLcmSystem<double>>(
          tree);
  contact_viz->set_name("contact_viz");
  auto contact_results_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", &lcm));
  contact_results_publisher->set_name("contact_results_publisher");

  // Contact results to lcm msg.
  builder.Connect(plant->contact_results_output_port(),
                  contact_viz->get_input_port(0));
  builder.Connect(contact_viz->get_output_port(0),
                  contact_results_publisher->get_input_port(0));

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);

  auto context = simulator.get_mutable_context();
  auto state = context->get_mutable_state();
  int state_size = state->get_continuous_state()->size();
  drake::log()->info("state vec size: {}", state_size);

  VectorX<double> x0 = VectorX<double>::Zero(state_size);

  // First 6 entries are floating-base
  x0[2] = 2.0;
//  x0[3] = 1.0;
  x0[4] = 1.0;

  simulator.get_mutable_context()
      ->get_mutable_continuous_state_vector()
      ->SetFromVector(x0);

  simulator.set_target_realtime_rate(1.0);
  lcm.StartReceiveThread();
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_sec);

  return 0;
}

}  // namespace human_model
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::human_model::DoMain(argc, argv);
}
