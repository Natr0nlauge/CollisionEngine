
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "CollisionDetector.hpp"
#include "sfml_utility.hpp"
#include "Polygon.hpp"
#include <cmath>
#include <numbers>

const float EPSILON = 1e-5f; // Tolerance for floating-point comparisons

void EXPECT_NEAR_VECTOR(sf::Vector2f i_vec1, sf::Vector2f i_vec2) {
    EXPECT_NEAR(i_vec1.x, i_vec2.x, EPSILON);
    EXPECT_NEAR(i_vec1.y, i_vec2.y, EPSILON);
};
TEST(CollisionDetectorTest, instanceTest) {
    CollisionDetector & instance1 = CollisionDetector::getInstance();
    CollisionDetector & instance2 = CollisionDetector::getInstance();
    EXPECT_EQ(&instance1, &instance2);
}

// Test: VertexBasedBody vs VertexBasedBody
TEST(CollisionDetectorTest, EdgeOnEdgeCollision) {
    Polygon polygon1;
    Polygon polygon2;
    CollisionDetector & cd = CollisionDetector::getInstance();
    const float EDGE_LENGTH = 50.0f; // In pixels
    // subtract one to make sure bodies are intersecting
    const float HORIZONTAL_OFFSET = EDGE_LENGTH - 1; // In pixels
    polygon1.setPosition({0.0f, 0.0f});
    polygon2.setPosition({HORIZONTAL_OFFSET, 0.0f});
    
    CollisionEvent event = cd.generateCollisionEvent(&polygon1, &polygon2);

    EXPECT_NEAR(event.getMinSeparation(), -1.0f, EPSILON);
    EXPECT_NEAR_VECTOR(event.getCollisionGeometry().location, sf::Vector2f(HORIZONTAL_OFFSET/2, 0.0f));
    EXPECT_NEAR_VECTOR(event.getCollisionGeometry().normals[0], sf::Vector2f(1.0f, 0.0f));
    EXPECT_NEAR_VECTOR(event.getCollisionGeometry().normals[1], sf::Vector2f(-1.0f, 0.0f));

    // Move one body up -> The collision location should also move up (but only half the distance of the body)
    const float VERTICAL_OFFSET = 20.0f;
    polygon1.setPosition({0.0f, VERTICAL_OFFSET});
    event = cd.generateCollisionEvent(&polygon1, &polygon2);
    EXPECT_NEAR(event.getMinSeparation(), -1.0f, EPSILON);
    EXPECT_NEAR_VECTOR(event.getCollisionGeometry().location, sf::Vector2f(HORIZONTAL_OFFSET/2, VERTICAL_OFFSET/2));
    EXPECT_NEAR_VECTOR(event.getCollisionGeometry().normals[0], sf::Vector2f(1.0f, 0.0f));
    EXPECT_NEAR_VECTOR(event.getCollisionGeometry().normals[1], sf::Vector2f(-1.0f, 0.0f));
}

TEST(CollisionDetectorTest, CornerOnEdgeCollision) {
    Polygon polygon1;
    Polygon polygon2;
    CollisionDetector & cd = CollisionDetector::getInstance();
    const float EDGE_LENGTH = 50.0f; // In pixels
    // subtract one to make sure bodies are intersecting
    const float HORIZONTAL_OFFSET = EDGE_LENGTH-1; // In pixels
    const float ROTATIONAL_OFFSET = 45.0f; // In degrees
    const float ROTATED_DISTANCE = EDGE_LENGTH * sin(ROTATIONAL_OFFSET * sfu::PI / 180);
    polygon1.setPosition({0.0f, 0.0f});
    polygon1.setRotation(ROTATIONAL_OFFSET);
    polygon2.setPosition({ROTATED_DISTANCE + HORIZONTAL_OFFSET / 2, 0.0f});

    CollisionEvent event = cd.generateCollisionEvent(&polygon1, &polygon2);
    EXPECT_NEAR(event.getMinSeparation(), -0.5f, EPSILON);
    EXPECT_NEAR_VECTOR(event.getCollisionGeometry().location, sf::Vector2f(ROTATED_DISTANCE, 0.0f));
    EXPECT_NEAR_VECTOR(event.getCollisionGeometry().normals[0], sf::Vector2f(1.0f, 0.0f));
    EXPECT_NEAR_VECTOR(event.getCollisionGeometry().normals[1], sf::Vector2f(-1.0f, 0.0f));

    //Change rotations, so that the corner of polygon2 will now hit the edge of polygon1
    polygon1.setRotation(0.0f);
    polygon2.setRotation(ROTATIONAL_OFFSET);
    event = cd.generateCollisionEvent(&polygon1, &polygon2);
    EXPECT_NEAR_VECTOR(event.getCollisionGeometry().location, sf::Vector2f(HORIZONTAL_OFFSET/2, 0.0f));
    EXPECT_NEAR_VECTOR(event.getCollisionGeometry().normals[0], sf::Vector2f(1.0f, 0.0f));
    EXPECT_NEAR_VECTOR(event.getCollisionGeometry().normals[1], sf::Vector2f(-1.0f, 0.0f));
}




