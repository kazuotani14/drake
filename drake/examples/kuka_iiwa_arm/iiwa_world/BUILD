# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
)
load("//tools/lint:lint.bzl", "add_lint_tests")

drake_cc_library(
    name = "iiwa_wsg_diagram_factory",
    srcs = [
        "iiwa_wsg_diagram_factory.cc",
    ],
    hdrs = [
        "iiwa_wsg_diagram_factory.h",
    ],
    visibility = ["//drake/examples/kuka_iiwa_arm:__subpackages__"],
    deps = [
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/examples/kuka_iiwa_arm:oracular_state_estimator",
        "//drake/manipulation/schunk_wsg:schunk_wsg_constants",
        "//drake/manipulation/util:sim_diagram_builder",
        "//drake/manipulation/util:world_sim_tree_builder",
        "//drake/multibody/rigid_body_plant",
        "//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/controllers:pid_controller",
        "//drake/systems/framework:diagram",
        "//drake/systems/framework:diagram_builder",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:matrix_gain",
    ],
)

drake_cc_binary(
    name = "multi_arm_with_gripper_demo",
    srcs = ["multi_arm_with_gripper_demo.cc"],
    data = [
        "//drake/examples/kuka_iiwa_arm:models",
        "//drake/manipulation/models/iiwa_description:models",
        "//drake/manipulation/models/wsg_50_description:models",
    ],
    deps = [
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/lcm",
        "//drake/manipulation/schunk_wsg:schunk_wsg_constants",
        "//drake/manipulation/util:sim_diagram_builder",
        "//drake/manipulation/util:world_sim_tree_builder",
        "//drake/systems/analysis",
        "//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/controllers:pid_controller",
        "//drake/systems/primitives:trajectory_source",
        "@lcmtypes_bot2_core",
        "@lcmtypes_robotlocomotion",
    ],
)

add_lint_tests()
