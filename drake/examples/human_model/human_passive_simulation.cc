// Implements a passive simulation of the Drake-compatible description of the
// PR2 robot. There is no controller, but the contact parameters and integrator
// parameters are set to support reliable gripping of objects if a controller
// is added.

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

#include "drake/common/text_logging.h"

#include <ros/ros.h>

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");

namespace drake {
namespace examples {
namespace human_model {

int DoMain() {

  drake::lcm::DrakeLcm lcm;

  // Construct the tree for the human model.
  auto tree_ = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFile(
    //   FindResourceOrThrow("drake/examples/acrobot/Acrobot.urdf"),
      FindResourceOrThrow("drake/examples/human_model/models/human_model.urdf"),
      multibody::joints::kFixed,
      nullptr /* weld to frame */, tree_.get());

  // const double terrain_size = 100;
  // const double terrain_depth = 10;
  // multibody::AddFlatTerrainToWorld(tree_.get(), terrain_size, terrain_depth);

  drake::log()->info("Number of bodies: {}", tree_->get_num_bodies());
  drake::log()->info("Number of actuators: {}", tree_->get_num_actuators());

  systems::DiagramBuilder<double> diagram_builder;
  auto plant_ = diagram_builder.AddSystem<systems::RigidBodyPlant<double>>(
      std::move(tree_));
  plant_->set_name("plant_");

  systems::DrakeVisualizer& viz_publisher =
      *diagram_builder.template AddSystem<systems::DrakeVisualizer>(
          plant_->get_rigid_body_tree(), &lcm);
  viz_publisher.set_name("visualizer_publisher");
  diagram_builder.Connect(plant_->state_output_port(),
                          viz_publisher.get_input_port(0));

    // Send the robot's actuators zeros in abscence of a controller.
    // if(tree_->get_num_actuators() > 0) {
    //     auto constant_zero_source =
    //         diagram_builder.AddSystem<systems::ConstantVectorSource<double>>(
    //             VectorX<double>::Zero(plant_->actuator_command_input_port().size()));
    //     diagram_builder.Connect(constant_zero_source->get_output_port(),
    //                             plant_->actuator_command_input_port());
    // }

  // Set contact parameters that support gripping.
  const double kStaticFriction = 1;
  const double kDynamicFriction = 5e-1;
  const double kStictionSlipTolerance = 1e-3;
  plant_->set_friction_contact_parameters(kStaticFriction, kDynamicFriction,
                                          kStictionSlipTolerance);

  const double kStiffness = 1000;
  const double kDissipation = 100;
  plant_->set_normal_contact_parameters(kStiffness, kDissipation);

  // Create the simulator.
  std::unique_ptr<systems::Diagram<double>> diagram = diagram_builder.Build();
  systems::Simulator<double> simulator(*diagram);

  // auto context = simulator.get_mutable_context();

  // Set the initial joint positions to be something more interesting. Note that
  // the joint position order is the same as the order you get when you read the
  // URDF from top to bottom.
  // Eigen::VectorXd initial_joint_positions(num_actuators);
  // initial_joint_positions << 0, 0, 0, 0.3, 0, 0, -1.14, 1.11, -1.40, -2.11,
  //     -1.33, -1.12, 2.19, 0.2, 0.2, 0.2, 0.2, 2.1, 1.29, 0 - 0.15, 0, -0.1, 0,
  //     0.2, 0.2, 0.2, 0.2;
  //
  // for (int index = 0; index < num_actuators; index++) {
  //   plant_->set_position(simulator.get_mutable_context(), index,
  //                        initial_joint_positions[index]);
  // }

  simulator.set_target_realtime_rate(1.0);
  // lcm.StartReceiveThread();
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_sec);

  return 0;
}

}  // namespace human_model
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::human_model::DoMain();
}
