# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
)
load("//tools:gurobi.bzl", "gurobi_test_tags")
load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

exports_files(
    ["config/valkyrie.alias_groups"],
)

exports_files(
    ["config/valkyrie.id_controller_config"],
)

exports_files(
    ["config/iiwa.alias_groups"],
)

exports_files(
    ["config/iiwa.id_controller_config"],
)

drake_cc_library(
    name = "atlas_command_translator_system",
    srcs = ["atlas_command_translator_system.cc"],
    hdrs = ["atlas_command_translator_system.h"],
    deps = [
        "//drake/systems/controllers/qp_inverse_dynamics:qp_output_translator_system",  # noqa
    ],
)

drake_cc_library(
    name = "humanoid_status",
    srcs = ["humanoid_status.cc"],
    hdrs = ["humanoid_status.h"],
    deps = [
        "//drake/common:essential",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody:rigid_body_tree_alias_groups",
        "//drake/systems/controllers/qp_inverse_dynamics:robot_kinematic_state",  # noqa
        "//drake/systems/robotInterfaces:side",
    ],
)

drake_cc_library(
    name = "humanoid_manipulation_plan",
    srcs = [
        "dev/humanoid_manipulation_plan.cc",
    ],
    hdrs = [
        "dev/humanoid_manipulation_plan.h",
    ],
    deps = [
        "//drake/manipulation/util:robot_state_msg_translator",
        "//drake/systems/controllers:zmp_planner",
        "//drake/systems/controllers/plan_eval:generic_plan",
        "//drake/systems/controllers/qp_inverse_dynamics:lcm_utils",
        "//drake/util",
        "@lcmtypes_bot2_core",
        "@lcmtypes_robotlocomotion",
    ],
)

drake_cc_library(
    name = "humanoid_simple_plan",
    srcs = [
        "dev/humanoid_simple_plan.cc",
    ],
    hdrs = [
        "dev/humanoid_simple_plan.h",
        "lcm_custom_types/robotlocomotion/robot_plan_simple_t.hpp",
    ],
    deps = [
        "//drake/manipulation/util:robot_state_msg_translator",
        "//drake/systems/controllers:zmp_planner",
        "//drake/systems/controllers/plan_eval:generic_plan",
        "//drake/systems/controllers/qp_inverse_dynamics:lcm_utils",
        "//drake/util",
        "@lcmtypes_bot2_core",
        "@lcmtypes_robotlocomotion",
    ],
)

drake_cc_library(
    name = "simple_highlevel_planner",
    srcs = ["simple_highlevel_planner.cc"],
    hdrs = [
        "lcm_custom_types/robotlocomotion/robot_plan_simple_t.hpp",
        "simple_highlevel_planner.h",
    ],
    deps = [
        ":humanoid_simple_plan",
        "//drake/examples/valkyrie:valkyrie_constants",
        "//drake/manipulation/util:robot_state_msg_translator",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "humanoid_plan_eval_system",
    srcs = ["humanoid_plan_eval_system.cc"],
    hdrs = ["humanoid_plan_eval_system.h"],
    deps = [
        ":humanoid_manipulation_plan",
        ":humanoid_simple_plan",
        "//drake/systems/controllers/plan_eval:plan_eval_base_system",
    ],
)

drake_cc_library(
    name = "humanoid_status_translator_system",
    srcs = ["humanoid_status_translator_system.cc"],
    hdrs = ["humanoid_status_translator_system.h"],
    deps = [
        ":humanoid_status",
        "//drake/manipulation/util:robot_state_msg_translator",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/controllers/qp_inverse_dynamics:lcm_utils",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "humanoid_controller",
    hdrs = ["humanoid_controller.h"],
    deps = [
        ":atlas_command_translator_system",
        ":humanoid_plan_eval_system",
        ":humanoid_status_translator_system",
        "//drake/lcm",
        "//drake/multibody/parsers",
        "//drake/systems/controllers/qp_inverse_dynamics:qp_inverse_dynamics_system",  # noqa
        "//drake/systems/framework",
        "//drake/systems/lcm",
        "@lcmtypes_bot2_core",
    ],
)

drake_cc_binary(
    name = "send_manipulation_plan_demo",
    srcs = [
        "send_manipulation_plan_demo.cc",
    ],
    data = [
        "//drake/examples/valkyrie:models",
    ],
    deps = [
        "//drake/common:find_resource",
        "//drake/examples/valkyrie:valkyrie_constants",
        "//drake/manipulation/util:robot_state_msg_translator",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "@com_github_gflags_gflags//:gflags",
        "@lcmtypes_bot2_core",
        "@lcmtypes_robotlocomotion",
    ],
)

drake_cc_binary(
    name = "send_simple_plan_demo",
    srcs = [
        "send_simple_plan_demo.cc",
    ],
    data = [
        "//drake/examples/valkyrie:models",
    ],
    deps = [
        ":humanoid_simple_plan",
        "//drake/common:find_resource",
        "//drake/examples/valkyrie:valkyrie_constants",
        "//drake/manipulation/util:robot_state_msg_translator",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "@com_github_gflags_gflags//:gflags",
        "@lcmtypes_bot2_core",
        "@lcmtypes_robotlocomotion",
    ],
)

drake_cc_binary(
    name = "valkyrie_balancing_demo",
    srcs = [
        "valkyrie_balancing_demo.cc",
    ],
    data = [
        ":config/valkyrie.alias_groups",
        ":config/valkyrie.id_controller_config",
        "//drake/examples/valkyrie:models",
    ],
    tags = gurobi_test_tags(),
    deps = [
        ":humanoid_controller",
        "//drake/common:find_resource",
        "//drake/examples/valkyrie:valkyrie_constants",
        "//drake/systems/lcm:lcm_driven_loop",
    ],
)

drake_cc_binary(
    name = "valkyrie_squatting_demo",
    srcs = [
        "valkyrie_squatting_demo.cc",
    ],
    data = [
        ":config/valkyrie.alias_groups",
        ":config/valkyrie.id_controller_config",
        "//drake/examples/valkyrie:models",
    ],
    tags = gurobi_test_tags(),
    deps = [
        ":humanoid_simple_plan",
        ":humanoid_status_translator_system",
        ":simple_highlevel_planner",
        "//drake/common:find_resource",
        "//drake/examples/valkyrie:valkyrie_constants",
        "//drake/lcm",
        "//drake/multibody/parsers",
        "//drake/systems/framework",
        "//drake/systems/lcm",
        "//drake/systems/lcm:lcm_driven_loop",
        "@lcmtypes_bot2_core",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "atlas_command_translator_system_test",
    srcs = ["test/atlas_command_translator_system_test.cc"],
    data = [
        "//drake/examples/valkyrie:models",
    ],
    deps = [
        ":atlas_command_translator_system",
        "//drake/common:find_resource",
        "//drake/common/test_utilities:eigen_matrix_compare",
        "//drake/multibody/parsers",
        "//drake/systems/framework",
    ],
)

drake_cc_googletest(
    name = "humanoid_plan_eval_system_test",
    srcs = ["test/humanoid_plan_eval_system_test.cc"],
    data = [
        ":config/valkyrie.alias_groups",
        ":config/valkyrie.id_controller_config",
        "//drake/examples/valkyrie:models",
    ],
    tags = gurobi_test_tags(),
    deps = [
        ":humanoid_plan_eval_system",
        ":humanoid_status",
        "//drake/common:find_resource",
        "//drake/common/test_utilities:eigen_matrix_compare",
        "//drake/examples/valkyrie:valkyrie_constants",
        "//drake/multibody/parsers",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:setpoint",
        "//drake/systems/controllers/qp_inverse_dynamics:param_parser",
        "//drake/systems/controllers/qp_inverse_dynamics:qp_inverse_dynamics_system",  # noqa
        "//drake/systems/framework",
        "//drake/systems/primitives:constant_value_source",
    ],
)

add_lint_tests()
