// Implements a passive simulation of the Drake-compatible description of the
// PR2 robot. There is no controller, but the contact parameters and integrator
// parameters are set to support reliable gripping of objects if a controller
// is added.

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");

namespace drake {
namespace examples {
namespace human_model {

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::lcm::DrakeLcm lcm;

  // Construct the tree for the human model.
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      //      FindResourceOrThrow("drake/examples/acrobot/Acrobot.urdf"),
      FindResourceOrThrow(
          "drake/examples/valkyrie/urdf/urdf/"
              "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf"),
//  FindResourceOrThrow(
//          "drake/examples/human_model/models/simplified_human_model.urdf"),
      multibody::joints::kRollPitchYaw, tree.get());

  const double terrain_size = 100;
  const double terrain_depth = 10;
  multibody::AddFlatTerrainToWorld(tree.get(), terrain_size, terrain_depth);

  int n_actuators = tree->get_num_actuators();

  drake::log()->info("Number of bodies: {}", tree->get_num_bodies());
  drake::log()->info("Number of actuators: {}", n_actuators);
  drake::log()->info("Mass: {}", tree->getMass());

  systems::DiagramBuilder<double> diagram_builder;
  auto plant = diagram_builder.AddSystem<systems::RigidBodyPlant<double>>(
      std::move(tree));
  plant->set_name("plant");

  systems::DrakeVisualizer& viz_publisher =
      *diagram_builder.template AddSystem<systems::DrakeVisualizer>(
          plant->get_rigid_body_tree(), &lcm);
  viz_publisher.set_name("visualizer_publisher");
  diagram_builder.Connect(plant->state_output_port(),
                          viz_publisher.get_input_port(0));

  // Send the robot's actuators zeros in absence of a controller.
  if (n_actuators) {
    std::cout << "Setting constant actuator torque" << std::endl;
    int n_torques = plant->actuator_command_input_port().size();
    VectorX<double> constant_torques = VectorX<double>::Constant(n_torques, 0.0);
    auto constant_torque_source =
        diagram_builder.AddSystem<systems::ConstantVectorSource<double>>(
            constant_torques);
    diagram_builder.Connect(constant_torque_source->get_output_port(),
                            plant->actuator_command_input_port());
  }

  // Set contact parameters that support gripping.
  const double kStaticFriction = 1;
  const double kDynamicFriction = 5e-1;
  const double kStictionSlipTolerance = 1e-3;
  plant->set_friction_contact_parameters(kStaticFriction, kDynamicFriction,
                                         kStictionSlipTolerance);

  const double kStiffness = 1000;
  const double kDissipation = 100;
  plant->set_normal_contact_parameters(kStiffness, kDissipation);

  auto diagram = diagram_builder.Build();
  systems::Simulator<double> simulator(*diagram);

  auto context = simulator.get_mutable_context();
  auto state = context->get_mutable_state();
  int state_size = state->get_continuous_state()->size();
  drake::log()->info("state vec size: {}", state_size);

  VectorX<double> x0 = VectorX<double>::Zero(state_size);

  // First 6 entries are floating-base
  x0[2] = 1.0;
  x0[4] = 1.0;

//  x0[5] = 0.5;

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
