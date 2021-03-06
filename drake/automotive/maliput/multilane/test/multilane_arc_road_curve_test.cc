/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/arc_road_curve.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace maliput {
namespace multilane {

class MultilaneArcRoadCurveTest : public ::testing::Test {
 protected:
  const Vector2<double> kCenter{10.0, 10.0};
  const double kRadius{10.0};
  const double kTheta0{M_PI / 4.0};
  const double kTheta1{3.0 * M_PI / 4.0};
  const double kDTheta{kTheta1 - kTheta0};
  const CubicPolynomial zp;
  const double kRMin{-0.5 * kRadius};
  const double kRMax{ 0.5 * kRadius};
  const api::HBounds height_bounds{0.0, 10.0};
  const double kVeryExact{1e-12};
  const double kNoOffset{0.0};
};

// Checks ArcRoadCurve constructor constraints.
TEST_F(MultilaneArcRoadCurveTest, ConstructorTest) {
  EXPECT_THROW(ArcRoadCurve(kCenter, -kRadius, kTheta0, kDTheta, zp, zp),
               std::runtime_error);
  EXPECT_NO_THROW(ArcRoadCurve(kCenter, kRadius, kTheta0, kDTheta, zp, zp));
}

// Checks arc reference curve interpolations, derivatives, and lengths.
TEST_F(MultilaneArcRoadCurveTest, ArcGeometryTest) {
  const ArcRoadCurve dut(kCenter, kRadius, kTheta0, kDTheta, zp, zp);
  // Checks the length.
  EXPECT_NEAR(dut.p_scale(), kDTheta * kRadius, kVeryExact);
  EXPECT_NEAR(dut.s_from_p(1., kNoOffset), kDTheta * kRadius, kVeryExact);
  // Checks the evaluation of xy at different values over the reference curve.
  EXPECT_TRUE(CompareMatrices(
      dut.xy_of_p(0.0), kCenter + Vector2<double>(kRadius * std::cos(kTheta0),
                                                  kRadius * std::sin(kTheta0)),
      kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      dut.xy_of_p(0.5),
      kCenter + Vector2<double>(kRadius * std::cos(kTheta0 + kDTheta * 0.5),
                                kRadius * std::sin(kTheta0 + kDTheta * 0.5)),
      kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      dut.xy_of_p(1.0), kCenter + Vector2<double>(kRadius * std::cos(kTheta1),
                                                  kRadius * std::sin(kTheta1)),
      kVeryExact));
  // Checks the derivative of xy at different values over the reference curve.
  EXPECT_TRUE(
      CompareMatrices(dut.xy_dot_of_p(0.0),
                      Vector2<double>(-kRadius * std::sin(kTheta0) * kDTheta,
                                      kRadius * std::cos(kTheta0) * kDTheta),
                      kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      dut.xy_dot_of_p(0.5),
      Vector2<double>(-kRadius * std::sin(kTheta0 + 0.5 * kDTheta) * kDTheta,
                      kRadius * std::cos(kTheta0 + 0.5 * kDTheta) * kDTheta),
      kVeryExact));
  EXPECT_TRUE(
      CompareMatrices(dut.xy_dot_of_p(1.0),
                      Vector2<double>(-kRadius * std::sin(kTheta1) * kDTheta,
                                      kRadius * std::cos(kTheta1) * kDTheta),
                      kVeryExact));
  // Checks the heading at different values.
  EXPECT_NEAR(dut.heading_of_p(0.0), kTheta0 + M_PI / 2.0, kVeryExact);
  EXPECT_NEAR(dut.heading_of_p(0.5), kTheta0 + kDTheta / 2.0 + M_PI / 2.0,
              kVeryExact);
  EXPECT_NEAR(dut.heading_of_p(1.0), kTheta1 + M_PI / 2.0, kVeryExact);
  // Checks the heading derivative of p at different values.
  EXPECT_NEAR(dut.heading_dot_of_p(0.0), kDTheta, kVeryExact);
  EXPECT_NEAR(dut.heading_dot_of_p(0.5), kDTheta, kVeryExact);
  EXPECT_NEAR(dut.heading_dot_of_p(1.0), kDTheta, kVeryExact);
}

// Checks the validity of the surface for different lateral bounds and
// geometries. Test cases are disposed in the following way for each road curve
// and lateral bounds:
// - Inside the lateral bounds.
// - Inside but with smaller lateral bounds.
// - To the right of the bounds.
// - To the left of the bounds.
// - Larger than the radius bounds.
// - Bounds equal to the radius.
GTEST_TEST(MultilaneArcRoadCurve, IsValidTest) {
  const Vector2<double> kZeroCenter(0.0, 0.0);
  const double kRadius = 10.0;
  const double kInitTheta = 0.0;
  const double kEndTheta1 = M_PI / 2.0;
  const double kDTheta1 = kEndTheta1 - kInitTheta;
  const double kEndTheta2 = -M_PI / 2.0;
  const double kDTheta2 = kEndTheta2 - kInitTheta;
  const CubicPolynomial constant_superelevation(M_PI / 4.0, 0.0, 0.0, 0.0);
  const api::HBounds height_bounds(0.0, 10.0);
  const CubicPolynomial zp;

  // Checks over a flat arc surface.
  const ArcRoadCurve flat_arc_geometry(kZeroCenter, kRadius, kInitTheta,
                                       kDTheta1, zp, zp);
  EXPECT_TRUE(
      flat_arc_geometry.IsValid(-0.5 * kRadius, 0.5 * kRadius, height_bounds));
  EXPECT_TRUE(flat_arc_geometry.IsValid(-0.25 * kRadius, 0.25 * kRadius,
                                        height_bounds));
  EXPECT_TRUE(
      flat_arc_geometry.IsValid(0.25 * kRadius, 0.75 * kRadius, height_bounds));
  EXPECT_TRUE(flat_arc_geometry.IsValid(-0.75 * kRadius, -0.25 * kRadius,
                                        height_bounds));
  EXPECT_FALSE(
      flat_arc_geometry.IsValid(-1.5 * kRadius, 1.5 * kRadius, height_bounds));
  EXPECT_FALSE(flat_arc_geometry.IsValid(-kRadius, kRadius, height_bounds));

  // Checks over a right handed cone.
  const ArcRoadCurve right_handed_cone_geometry(
      kZeroCenter, kRadius, kInitTheta, kDTheta1, zp, constant_superelevation);
  EXPECT_TRUE(right_handed_cone_geometry.IsValid(-0.5 * kRadius, 0.5 * kRadius,
                                                 height_bounds));
  EXPECT_TRUE(right_handed_cone_geometry.IsValid(
      -0.25 * kRadius, 0.25 * kRadius, height_bounds));
  EXPECT_TRUE(right_handed_cone_geometry.IsValid(0.25 * kRadius, 0.75 * kRadius,
                                                 height_bounds));
  EXPECT_TRUE(right_handed_cone_geometry.IsValid(
      -0.75 * kRadius, -0.25 * kRadius, height_bounds));
  EXPECT_FALSE(right_handed_cone_geometry.IsValid(-1.5 * kRadius, 1.5 * kRadius,
                                                  height_bounds));
  EXPECT_TRUE(
      right_handed_cone_geometry.IsValid(-kRadius, kRadius, height_bounds));

  // Checks over a left handed cone.
  const ArcRoadCurve left_handed_cone_geometry(
      kZeroCenter, kRadius, kInitTheta, kDTheta2, zp, constant_superelevation);
  EXPECT_TRUE(left_handed_cone_geometry.IsValid(-0.5 * kRadius, 0.5 * kRadius,
                                                height_bounds));
  EXPECT_TRUE(left_handed_cone_geometry.IsValid(-0.25 * kRadius, 0.25 * kRadius,
                                                height_bounds));
  EXPECT_TRUE(left_handed_cone_geometry.IsValid(0.25 * kRadius, 0.75 * kRadius,
                                                height_bounds));
  EXPECT_TRUE(left_handed_cone_geometry.IsValid(
      -0.75 * kRadius, -0.25 * kRadius, height_bounds));
  EXPECT_FALSE(left_handed_cone_geometry.IsValid(-1.5 * kRadius, 1.5 * kRadius,
                                                 height_bounds));
  EXPECT_TRUE(
      left_handed_cone_geometry.IsValid(-kRadius, kRadius, height_bounds));
}

// Checks the ToCurve frame coordinate conversion for different points in world
// coordinates.
TEST_F(MultilaneArcRoadCurveTest, ToCurveFrameTest) {
  const ArcRoadCurve dut(kCenter, kRadius, kTheta0, kDTheta, zp, zp);
  // Checks points over the composed curve.
  EXPECT_TRUE(CompareMatrices(
      dut.ToCurveFrame(
          Vector3<double>(kCenter(0) + kRadius * std::cos(kTheta0),
                          kCenter(1) + kRadius * std::sin(kTheta0), 0.0),
          kRMin, kRMax, height_bounds),
      Vector3<double>(0.0, 0.0, 0.0), kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      dut.ToCurveFrame(
          Vector3<double>(
              kCenter(0) + kRadius * std::cos(kTheta0 + kDTheta / 2.0),
              kCenter(1) + kRadius * std::sin(kTheta0 + kDTheta / 2.0), 0.0),
          kRMin, kRMax, height_bounds),
      Vector3<double>(0.5, 0.0, 0.0), kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      dut.ToCurveFrame(
          Vector3<double>(kCenter(0) + kRadius * std::cos(kTheta1),
                          kCenter(1) + kRadius * std::sin(kTheta1), 0.0),
          kRMin, kRMax, height_bounds),
      Vector3<double>(1., 0.0, 0.0), kVeryExact));
  // Checks with lateral and vertical deviations.
  EXPECT_TRUE(CompareMatrices(
      dut.ToCurveFrame(
          Vector3<double>(
              kCenter(0) + (kRadius + 1.0) * std::cos(kTheta0 + M_PI / 8.0),
              kCenter(1) + (kRadius + 1.0) * std::sin(kTheta0 + M_PI / 8.0),
              6.0),
          kRMin, kRMax, height_bounds),
      Vector3<double>(0.25, -1.0, 6.0), kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      dut.ToCurveFrame(
          Vector3<double>(
              kCenter(0) +
                  (kRadius - 2.0) *
                      std::cos(kTheta0 + kDTheta / 2.0 + M_PI / 8.0),
              kCenter(1) +
                  (kRadius - 2.0) *
                      std::sin(kTheta0 + kDTheta / 2.0 + M_PI / 8.0),
              3.0),
          kRMin, kRMax, height_bounds),
      Vector3<double>(0.75, 2.0, 3.0),
      kVeryExact));
}

// Checks that p_scale(), p_from_s() and s_from_p() with constant superelevation
// polynomial and up to linear elevation polynomial behave properly.
TEST_F(MultilaneArcRoadCurveTest, OffsetTest) {
  const std::vector<double> r_vector{-0.5 * kRadius, 0.0, 0.5 * kRadius};
  const std::vector<double> p_vector{0., 0.1, 0.2, 0.5, 0.7, 1.0};

  // Checks for flat ArcRoadCurve.
  const ArcRoadCurve flat_dut(kCenter, kRadius, kTheta0, kDTheta, zp, zp);
  EXPECT_DOUBLE_EQ(flat_dut.p_scale(), kRadius * kDTheta);
  // Checks that functions throw when lateral offset is exceeded.
  EXPECT_THROW(flat_dut.p_from_s(0., kRadius), std::runtime_error);
  EXPECT_THROW(flat_dut.p_from_s(0., 2.0 * kRadius), std::runtime_error);
  EXPECT_THROW(flat_dut.s_from_p(0., kRadius), std::runtime_error);
  EXPECT_THROW(flat_dut.s_from_p(0., 2.0 * kRadius), std::runtime_error);
  // Evaluates inverse function for different path length and offset values.
  for (double r : r_vector) {
    for (double p : p_vector) {
      EXPECT_DOUBLE_EQ(flat_dut.p_from_s(p * (kRadius - r) * kDTheta, r), p);
    }
  }
  // Evaluates the path length integral for different offset values.
  for (double r : r_vector) {
    for (double p : p_vector) {
      EXPECT_DOUBLE_EQ(flat_dut.s_from_p(p, r), p * (kRadius - r) * kDTheta);
    }
  }

  // Checks for linear elevation applied to the ArcRoadCurve.
  const double slope = 10. / (kRadius * kDTheta);
  const CubicPolynomial linear_elevation(10., slope, 0., 0.);
  const ArcRoadCurve elevated_dut(kCenter, kRadius, kTheta0, kDTheta,
                                  linear_elevation, zp);
  EXPECT_DOUBLE_EQ(elevated_dut.p_scale(), kRadius * kDTheta);
  // Evaluates inverse function and path length integral for different values of
  // p and r lateral offsets.
  for (double r : r_vector) {
    for (double p : p_vector) {
      const double s = p * kRadius * kDTheta *
          std::sqrt(std::pow((kRadius - r) / kRadius, 2.) + slope * slope);
      EXPECT_DOUBLE_EQ(elevated_dut.p_from_s(s , r), p);
      EXPECT_DOUBLE_EQ(elevated_dut.s_from_p(p, r), s);
    }
  }
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
