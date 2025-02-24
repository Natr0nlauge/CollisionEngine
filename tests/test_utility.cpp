#include <gtest/gtest.h>

//#include "sfml_utility.hpp"
#include "..\Collision2D\sfml_utility.cpp"
#include "..\Collision2D\Matrix2f.cpp"
        //
const float EPSILON = 1e-5f; // Tolerance for floating-point comparisons

// Helper function to compare floating-point values
#define FLOAT_EQ(a, b) (std::fabs((a) - (b)) < EPSILON)

// Helper function to compare vectors
bool VEC_EQ(sf::Vector2f v1, sf::Vector2f v2) {
    return FLOAT_EQ(v1.x, v2.x) && FLOAT_EQ(v1.y, v2.y);
}

// Test getVectorLength
TEST(SfuTest, VectorLength) {
    EXPECT_NEAR(sfu::getVectorLength({3, 4}), 5.0f, EPSILON);
    EXPECT_NEAR(sfu::getVectorLength({0, 0}), 0.0f, EPSILON);
}

// Test getVectorDirection (angle in radians)
TEST(SfuTest, VectorDirection) {
    EXPECT_NEAR(sfu::getVectorDirection({1, 0}), 0.0f, EPSILON);
    EXPECT_NEAR(sfu::getVectorDirection({0, 1}), 90.0f, EPSILON);
}

// Test scaleVector
TEST(SfuTest, ScaleVector) {
    sf::Vector2f result = sfu::scaleVector({1, 2}, 2);
    EXPECT_TRUE(VEC_EQ(result, {2, 4}));
}

// Test normalizeVector
TEST(SfuTest, NormalizeVector) {
    sf::Vector2f result = sfu::normalizeVector({3, 4});
    sf::Vector2f resultForZero = sfu::normalizeVector({0, 0});
    EXPECT_TRUE(VEC_EQ(result, {0.6f, 0.8f}));
    EXPECT_TRUE(VEC_EQ(resultForZero, {0.0f, 0.0f}));
}

// Test addVectors
TEST(SfuTest, AddVectors) {
    sf::Vector2f result = sfu::addVectors({1, 2}, {3, 4});
    EXPECT_TRUE(VEC_EQ(result, {4, 6}));
}

// Test subtractVectors
TEST(SfuTest, SubtractVectors) {
    sf::Vector2f result = sfu::subtractVectors({3, 4}, {1, 2});
    EXPECT_TRUE(VEC_EQ(result, {2, 2}));
}

// Test scalarProduct (dot product)
TEST(SfuTest, ScalarProduct) {
    EXPECT_NEAR(sfu::scalarProduct({1, 2}, {3, 4}), 11.0f, EPSILON);
}

// Test pseudoCrossProduct
TEST(SfuTest, PseudoCrossProduct) {
    EXPECT_NEAR(sfu::pseudoCrossProduct({1, 0}, {0, 1}), 1.0f, EPSILON);
}

// Test pseudoCrossProduct (overload)
TEST(SfuTest, PseudoCrossProductWithLength) {
    sf::Vector2f result = sfu::pseudoCrossProduct(2.0f, {1, 0});
    EXPECT_TRUE(VEC_EQ(result, {0, 2}));
}

// Test rotateVector
TEST(SfuTest, RotateVector) {
    sf::Vector2f result = sfu::rotateVector({1, 0}, 90.0f);
    EXPECT_NEAR(result.x, 0, EPSILON);
    EXPECT_NEAR(result.y, 1, EPSILON);
}

// Test transformPoint
TEST(SfuTest, TransformPoint) {
    sf::Vector2f result = sfu::transformPoint({1, 0}, {0, 0}, 90.0f);
    EXPECT_TRUE(VEC_EQ(result, {0, 1}));
}

// Test getRotationMatrix (assuming Matrix2f is implemented)
TEST(SfuTest, RotationMatrix) {
    sfu::Matrix2f matrix = sfu::getRotationMatrix(90.0f);
    // Verify rotation matrix values (assuming you have a way to access elements)
    EXPECT_NEAR(matrix.m[0][0], 0, EPSILON);
    EXPECT_NEAR(matrix.m[0][1], -1, EPSILON);
    EXPECT_NEAR(matrix.m[1][0], 1, EPSILON);
    EXPECT_NEAR(matrix.m[1][1], 0, EPSILON);
}

// Test printVectorCoords (not easily testable, but we can check for runtime errors)
TEST(SfuTest, PrintVectorCoords) {
    EXPECT_NO_FATAL_FAILURE(sfu::printVectorCoords({3, 4}));
}

// Main function for Google Test
//int main(int argc, char ** argv) {
//    ::testing::InitGoogleTest(&argc, argv);
//    return RUN_ALL_TESTS();
//}