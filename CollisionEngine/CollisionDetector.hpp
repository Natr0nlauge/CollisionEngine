#pragma once

#include "VertexBasedBody.hpp"
#include "Circle.hpp"
#include "CollisionEvent.hpp"
#include <mutex>

/**
 * @brief Stores data related to the separation between two VertexBasedBodies.
 *
 * This struct holds information about the separation distance,
 * the indices of the colliding corners, and the normal vector representing
 * the direction of separation.
 */
struct VertexBasedBodySeparation_type {
    /// Separation in pixels, as determined by the SAT algorithm. This is not necessarily the minimum distance between two bodies.
    float separation = std::numeric_limits<float>::lowest();
    /// Holds the index for the point with the smallest separation. If two points have a similarly small separation, holds both indices.
    std::vector<int> indexVec;
    /// Normal vector corresponding to the smallest separation.
    sf::Vector2f normal;
};

/**
 * @brief Stores data related to the separation between a VertexBasedBody and a Circle.
 *
 * This struct holds information about the separation distance,
 * the index of the colliding corner, and the normal vector representing
 * the direction of separation.
 */
struct circleSeparation_type {
    /// The separation according to SAT as determined when iterating through all edges.
    float edgeSeparation = std::numeric_limits<float>::lowest();
    /// The separation according to SAT as determined when iterating through all corner points.
    float cornerSeparation = std::numeric_limits<float>::max();
    /// The index of the point closest to the Circle.
    int pointIndex = -1;
    /// The index of the edge corresponding to the edge separation.
    int normalIndex = -1;
};

/**
 * @class CollisionDetector
 * @brief Singleton class that holds methods to detect Collisions between RigidBody objects and to determine the collision geometry.
 */
class CollisionDetector {
  public:
    //  Singleton accessor
    static CollisionDetector & getInstance();

    // Destructor
    ~CollisionDetector();

    // Public methods
    CollisionEvent generateCollisionEvent(RigidBody * i_firstBody, RigidBody * i_secondBody);

  private:
    // Singleton implementation
    static std::unique_ptr<CollisionDetector> s_instance;
    static std::mutex mtx;
    CollisionDetector();

    // Deleted copy constructor and assignment operator
    CollisionDetector(const CollisionDetector &) = delete;
    CollisionDetector & operator=(const CollisionDetector &) = delete;

    // Private methods
    VertexBasedBodySeparation_type calculateMinVertexBasedBodySeparation(VertexBasedBody & i_body1, VertexBasedBody & i_body2) const;
    circleSeparation_type calculateMinCircleSeparation(VertexBasedBody & i_VertexBasedBody, Circle & i_circle) const;
    sf::Vector2f findCenterOfContact(VertexBasedBodySeparation_type & i_sepData1, VertexBasedBodySeparation_type & i_sepData2,
            VertexBasedBody & i_body1, VertexBasedBody & i_body2);
    collisionGeometry_type determineCollisionGeometry(VertexBasedBody * i_firstBody, VertexBasedBody * i_secondBody);
    collisionGeometry_type determineCollisionGeometry(VertexBasedBody * i_firstBody, Circle * i_secondBody);
    collisionGeometry_type determineCollisionGeometry(Circle * i_firstBody, VertexBasedBody * i_secondBody);
    collisionGeometry_type determineCollisionGeometry(Circle * i_firstBody, Circle * i_secondBody);
    collisionGeometry_type determineVertexBodyAndCircleGeometry(VertexBasedBody * i_firstBody, Circle * i_secondBody);
    float computeMedian(const std::array<float, 4> & i_arr);

    // Private member variables
    /// This helps identifying edge-on-edge collisions.
    const float MIN_SEP_EPSILON = 0.01;
};
