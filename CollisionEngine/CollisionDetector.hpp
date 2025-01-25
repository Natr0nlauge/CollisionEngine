#pragma once

#include "EdgeStructure.hpp"
#include "Circle.hpp"
#include "CollisionEvent.hpp"
#include <mutex>

// Includes some data about the separation
struct edgeStructureSeparationData_type {
    float separation = std::numeric_limits<float>::lowest();
    std::vector<int> indexVec; // TODO this might not be ideal
    sf::Vector2f normal;
};

struct circleSeparationData_type {
    float edgeSeparation = std::numeric_limits<float>::lowest();
    float cornerSeparation = std::numeric_limits<float>::max();
    int pointIndex = -1;
    int normalIndex = -1;
};



class CollisionDetector {
  public:
    //  Singleton accessor
    static CollisionDetector & getInstance();

    // Destructor
    ~CollisionDetector();

    // Public methods
    bool detectCollision(CollisionEvent & c_collisionEvent);

  private:
    // Singleton implementation
    static std::unique_ptr<CollisionDetector> s_instance;
    static std::mutex mtx;
    CollisionDetector();

    // Deleted copy constructor and assignment operator
    CollisionDetector(const CollisionDetector &) = delete;
    CollisionDetector & operator=(const CollisionDetector &) = delete;

    // Private methods
    edgeStructureSeparationData_type calculateMinEdgeStructureSeparation(EdgeStructure & i_body1, EdgeStructure & i_body2) const;
    circleSeparationData_type calculateMinCircleSeparation(EdgeStructure & i_edgeStructure, Circle & i_circle) const;
    sf::Vector2f findCenterOfContact(edgeStructureSeparationData_type & i_sepData1, edgeStructureSeparationData_type & i_sepData2, EdgeStructure & i_body1,
            EdgeStructure & i_body2);
    collisionGeometry_type determineCollisionGeometry(EdgeStructure * firstBody, EdgeStructure * secondBody);
    collisionGeometry_type determineCollisionGeometry(EdgeStructure * firstBody, Circle * secondBody);
    collisionGeometry_type determineCollisionGeometry(Circle * firstBody, EdgeStructure * secondBody);
    collisionGeometry_type determineCollisionGeometry(Circle * firstBody, Circle * secondBody);
    collisionGeometry_type determineEdgeAndCircleGeometry(EdgeStructure * firstBody, Circle * secondBody);

    // Private member variables
    const float MIN_SEP_EPSILON = 0.01; // This makes "Central collisions" possible
};
