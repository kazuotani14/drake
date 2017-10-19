#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

#include "drake/examples/humanoid_controller/lcm_custom_types/robotlocomotion/robot_plan_simple_t.hpp"

using robotlocomotion::robot_plan_simple_t;

namespace drake {
namespace examples {
namespace humanoid_controller {
namespace simple_highlevel_planner {

/**
 * A system block for a simple high-level planner for humanoid robot.
 * "Simple" meaning it only gives pose setpoints for body links and CoM.
 */
class SimpleHighlevelPlanner : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleHighlevelPlanner)

  /**
   * Constructor for the inverse dynamics controller.
   * @param robot Pointer to a RigidBodyTree. Its lifespan must be longer
   * than this object.
   * @param dt Control cycle period.
   */
  SimpleHighlevelPlanner(const RigidBodyTree<double>* robot);

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                const std::vector<const systems::UnrestrictedUpdateEvent<double>*>& events,
                                systems::State<double>* state) const override;

  /**
   * Returns the input port for HumanoidStatus.
   */
  inline const systems::InputPortDescriptor<double>&
  get_input_port_kinematic_state() const {
    return get_input_port(input_port_index_kinematic_state_);
  }

  /**
   * Returns the output port for robotlocomotion::robot_plan_simple_t.
   */
  inline const systems::OutputPort<double>&
  get_output_port_plan_output() const {
    return get_output_port(output_port_index_plan_output_);
  }

 private:
  // Copies the plan state variable to the output argument.
  void CopyOutPlanOutput(const systems::Context<double>& context,
                       robot_plan_simple_t* output) const;

  const RigidBodyTree<double>& robot_;
  mutable double com_z_d_ {0.96};

  int input_port_index_kinematic_state_{0};
  int output_port_index_plan_output_{0};

  int abs_state_index_plan_output_{0};
};

}  // namespace simple_highlevel_planner
}  // namespace humanoid_controller
}  // namespace examples
}  // namespace drake
