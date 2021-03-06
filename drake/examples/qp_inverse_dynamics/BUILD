# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")
load("//tools:gurobi.bzl", "gurobi_test_tags")
load("//tools/lint:lint.bzl", "add_lint_tests")

package(
    default_visibility = [":__subpackages__"],
)

drake_cc_library(
    name = "manipulator_move_joint_plan_eval_system",
    srcs = ["manipulator_move_joint_plan_eval_system.cc"],
    hdrs = ["manipulator_move_joint_plan_eval_system.h"],
    deps = [
        "//drake/lcmtypes:plan_eval_debug_info",
        "//drake/systems/controllers/plan_eval:plan_eval_base_system",
    ],
)

drake_cc_library(
    name = "manipulator_joint_space_controller",
    srcs = ["manipulator_joint_space_controller.cc"],
    hdrs = ["manipulator_joint_space_controller.h"],
    deps = [
        ":manipulator_move_joint_plan_eval_system",
        "//drake/multibody/parsers",
        "//drake/systems/controllers:state_feedback_controller_interface",
        "//drake/systems/controllers/qp_inverse_dynamics:qp_inverse_dynamics_system",  # noqa
        "//drake/systems/controllers/qp_inverse_dynamics:qp_output_translator_system",  # noqa
        "//drake/systems/controllers/qp_inverse_dynamics:robot_kinematic_state_translator_system",  # noqa
        "//drake/systems/framework",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "manipulator_joint_space_controller_test",
    srcs = ["test/manipulator_joint_space_controller_test.cc"],
    data = [
        "//drake/manipulation/models/iiwa_description:models",
        "//drake/systems/controllers/qp_inverse_dynamics:test/iiwa.alias_groups",  # noqa
        "//drake/systems/controllers/qp_inverse_dynamics:test/iiwa.id_controller_config",  # noqa
    ],
    tags = gurobi_test_tags(),
    deps = [
        ":manipulator_joint_space_controller",
        "//drake/common:find_resource",
        "//drake/common/test_utilities:eigen_matrix_compare",
        "//drake/multibody/parsers",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/framework",
        "//drake/systems/primitives:constant_vector_source",
    ],
)

add_lint_tests()
