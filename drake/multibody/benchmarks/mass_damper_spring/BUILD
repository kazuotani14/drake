# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")
load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "mass_damper_spring_analytical_solution",
    srcs = ["mass_damper_spring_analytical_solution.cc"],
    hdrs = ["mass_damper_spring_analytical_solution.h"],
    deps = [
        "//drake/common:essential",
        "//drake/math:autodiff",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "mass_damper_spring_analytical_solution_test",
    deps = [
        ":mass_damper_spring_analytical_solution",
        "//drake/math:autodiff",
    ],
)

add_lint_tests()
