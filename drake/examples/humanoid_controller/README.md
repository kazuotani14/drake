A demo of valkyrie executing a manipulation plan while actively maintaining
balance. This demo involves three executables:

1. Uncontrolled valkyrie simulation
    * drake/examples/valkyrie/valkyrie_simulation.cc
2. Controller
    * drake/examples/humanoid_controller/valkyrie_balancing_demo.cc
3. Dummy manipulation plan generator
    * drake/examples/humanoid_controller/send_manipulation_plan_demo.cc

The dummy plan generator takes 1 command line argument, and uses it to offset
the right shoulder pitch joint from its nominal position. The offset in in
radians. If no argument is supplied, it resets to the nominal posture.

All three components communicate using LCM messages. To properly run this demo,
you need to have Gurobi installed and have access to a fixed-license.
See [the Drake Bazel documentation](http://drake.mit.edu/bazel.html?highlight=gurobi)
for more details about building with Gurobi enabled.

To build: (assuming in drake's root directory)

    `$ bazel build --config gurobi //...`

To run the demo: (assuming in drake's root directory)

1. Launch the visualizer before the simulator:

    `$ ./bazel-bin/tools/drake_visualizer`

2. Start the QP controller before the simulator. By default, it maintains the configuration that it started in.

    `$ ./bazel-bin/drake/examples/humanoid_controller/valkyrie_balancing_demo`

3. Start the simulator:

    `$ ./bazel-bin/drake/examples/valkyrie/valkyrie_simulation`

NEW COMMAND (with tasks - kazu)

4. Send CoM and end effector "position tasks". See the code in `simple_highlevel_planer.cc` to see how the goal positions are set, and how the high level planner reacts to the robot's state.

    `$ ./bazel-bin/drake/examples/humanoid_controller/valkyrie_squatting_demo`


OLD COMMAND

4. Execute a dummy manipulation plan:

    `$ ./bazel-bin/drake/examples/humanoid_controller/send_manipulation_plan_demo -r_shy_offset -1`

Note that you can repeatedly send new plans, the controller will start executing
the new plan as soon as it receives it.

To-do

* add orientation to desired pose
* get CoM gains from paramset instead of hard-coded
* make contact sets and tracking bodies come from plan msg
* clean stuff up - figure out better organization
* remove all mentions of trajectories?
