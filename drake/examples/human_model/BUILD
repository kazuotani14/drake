# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
    "drake_cc_googletest",
)
load("//tools/lint:lint.bzl", "add_lint_tests")
load("//tools:install_data.bzl", "install_data")

package(default_visibility = ["//visibility:public"])

drake_cc_binary(
    name = "human_passive_simulation",
    srcs = ["human_passive_simulation.cc"],
    data = [":models"],
    deps = [
        "//drake/common:find_resource",
        "//drake/lcm",
        "//drake/lcmtypes:contact_info_for_viz",
        "//drake/lcmtypes:contact_results_for_viz",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:contact_results_to_lcm",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:semi_explicit_euler_integrator",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:diagram_builder",
        "//drake/systems/lcm",
        "//drake/systems/primitives:constant_vector_source",
        "@com_github_gflags_gflags//:gflags",
    ],
)

#
#  # === test/ ===
#
#  drake_cc_googletest(
#      name = "load_pr2_simplified_test",
#      srcs = ["test/load_pr2_simplified_test.cc"],
#      data = [":models"],
#      deps = [
#          "//drake/common:find_resource",
#          "//drake/multibody/parsers",
#      ],
#  )
#
#  install_data()
#
#  add_lint_tests()
