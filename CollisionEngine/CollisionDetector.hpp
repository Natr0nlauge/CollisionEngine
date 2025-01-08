#pragma once

#include "Polygon.hpp"
#include "CollisionEvent.hpp"

//// TODO: think about making this a class
//struct collisionEvent_type { // Will be used to pass all the necessary information to the Collision Handler
//    RigidBody & rBody1;      // TODO: naming!
//    RigidBody & rBody2;
//    // TODO: should this stuff be initialized?
//    sf::Vector2f collLoc1; // In global coordinates
//    // TODO: Check if saving collision Location in local coordinates is more efficient
//    sf::Vector2f normal1;
//    sf::Vector2f normal2;
//
//    // Constructor
//    collisionEvent_type(RigidBody & rb1, RigidBody & rb2) : rBody1(rb1), rBody2(rb2) {}
//};

// Includes some data about the separation
struct separationData_type {
    float separation = std::numeric_limits<float>::lowest();
    std::vector<int> indexVec;
    sf::Vector2f normal;
};

class CollisionDetector {
  public:
    // TODO: should this return a reference?
    //  Singleton accessor
    static CollisionDetector * getInstance();
    // TODO: overloads for detectCollision
    ~CollisionDetector();

    // Public methods
    bool detectCollision(CollisionEvent & c_collisionEvent);
    // TODO make collision detection and writing of the collisionEvent seperate functions!

  private:
    // Singleton implementation
    static CollisionDetector * s_instance;
    CollisionDetector();

    // Deleted copy constructor and assignment operator
    CollisionDetector(const CollisionDetector &) = delete;
    CollisionDetector & operator=(const CollisionDetector &) = delete;

    //  Private methods
    separationData_type findMinSeparation(Polygon & i_body1, Polygon & i_body2);
};
