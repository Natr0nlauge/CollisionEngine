#include "pch.h"
#include "..\Collision2D\CollisionDetector.hpp"

TEST(CollisionDetectorTest, instanceTest){
    CollisionDetector & instance1 = CollisionDetector::getInstance();
    CollisionDetector & instance2 = CollisionDetector::getInstance();
    EXPECT_EQ(&instance1,&instance2);
}

// Main function for Google Test
int main(int argc, char ** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}