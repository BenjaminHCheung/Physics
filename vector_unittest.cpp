#include "gtest/gtest.h"
#include "vector3d.h"

TEST(GivenTwoEqualVectors, WhenUsingIsEqualOperator_ReturnsTrue)
{
    Vector3d leftSide{Vector3d(1,2,3)};
    Vector3d rightSide{Vector3d(1,2,3)};
    EXPECT_TRUE(leftSide==rightSide);
}

TEST(GivenTwoNonEqualVectors, WhenUsingIsEqualOperator_ReturnsFalse)
{
    Vector3d leftSide{Vector3d(13,22,3.6)};
    Vector3d rightSide{Vector3d(1.123,2.25,334)};
    EXPECT_FALSE(leftSide == rightSide);
}

TEST(GivenTwoDifferentVectors, WhenUsingAddingOperatorVectors_ReturnsVectorOfCorrectValuesInXYZPositions)
{
    Vector3d vectorOne{Vector3d(4.5, 19, -24.5)};
    Vector3d vectorTwo{Vector3d(16, 19.4, 0)};
    Vector3d calculatedVector{vectorOne+vectorTwo};
    Vector3d expectedVector{Vector3d(20.5, 38.4, -24.5)};
    EXPECT_EQ(expectedVector.get_x_value(),calculatedVector.get_x_value());
    EXPECT_EQ(expectedVector.get_y_value(),calculatedVector.get_y_value());
    EXPECT_EQ(expectedVector.get_z_value(),calculatedVector.get_z_value());
}

TEST(GivenTwoDifferentVectors, WhenUsingSubtractingOperatorVectors_ReturnsVectorOfCorrectValuesInXYZPositions)
{
    Vector3d vectorOne{Vector3d(55.5, -4, 0)};
    Vector3d vectorTwo{Vector3d(50, -33.6, 13)};
    Vector3d calculatedVector{vectorOne-vectorTwo};
    Vector3d expectedVector{Vector3d(5.5, 29.6, -13)};
    EXPECT_EQ(expectedVector.get_x_value(),calculatedVector.get_x_value());
    EXPECT_EQ(expectedVector.get_y_value(),calculatedVector.get_y_value());
    EXPECT_EQ(expectedVector.get_z_value(),calculatedVector.get_z_value());
}

TEST(GivenScaler3dVector, WhenUsingMultiplicationOperatorAsDotProduct_ReturnsCorrectVectorValues)
{
    Vector3d scalerVector{Vector3d(-1,-2,3)};
    Vector3d baseVector{(Vector3d(4,5,6))};
    Vector3d calculatedVector(scalerVector*baseVector);
    Vector3d expectedVector{Vector3d(-4,-10,18)};
    EXPECT_EQ(expectedVector.get_x_value(),calculatedVector.get_x_value());
    EXPECT_EQ(expectedVector.get_y_value(),calculatedVector.get_y_value());
    EXPECT_EQ(expectedVector.get_z_value(),calculatedVector.get_z_value());
}

TEST(GivenScalerValueOfTypeDouble, WhenUsingMultiplicationOperator_ScalesEachValueInVectorCorrectly)
{
    double scaler{9.8};
    Vector3d baseVector{Vector3d(1,2,2)};
    Vector3d calculatedVector{baseVector*scaler};
    Vector3d expectedVector{Vector3d(9.8,19.6,19.6)};
    EXPECT_EQ(expectedVector.get_x_value(),calculatedVector.get_x_value());
    EXPECT_EQ(expectedVector.get_y_value(),calculatedVector.get_y_value());
    EXPECT_EQ(expectedVector.get_z_value(),calculatedVector.get_z_value());
}


TEST(GivenDivisionValueOfTypeDouble, WhenUsingDivisionOperatore_DividesEachValueInVectorCorrectly)
{
    double divider{9.8};
    Vector3d baseVector{Vector3d(9.8,19.6,9.8)};
    Vector3d calculatedVector{baseVector/divider};
    Vector3d expectedVector{Vector3d(1,2,1)};
    EXPECT_EQ(expectedVector.get_x_value(),calculatedVector.get_x_value());
    EXPECT_EQ(expectedVector.get_y_value(),calculatedVector.get_y_value());
    EXPECT_EQ(expectedVector.get_z_value(),calculatedVector.get_z_value());
}
