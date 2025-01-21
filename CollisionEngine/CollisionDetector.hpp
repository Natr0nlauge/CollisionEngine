#pragma once

#include "Polygon.hpp"
#include "Circle.hpp"
#include "CollisionEvent.hpp"
#include <mutex>

// Includes some data about the separation
struct separationData_type {
    float separation = std::numeric_limits<float>::lowest();
    std::vector<int> indexVec; // TODO this might not be ideal
    sf::Vector2f normal;
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
    //  Private methods
    separationData_type findMinPolygonSeparation(Polygon & i_body1, Polygon & i_body2) const;
    sf::Vector2f findCenterOfContact(separationData_type & i_sepData1, separationData_type & i_sepData2, Polygon & i_body1,
            Polygon & i_body2);
    bool detectPolygonCollision(CollisionEvent & c_collisionEvent);
    bool detectPolygonAndCircleCollision(CollisionEvent & c_collisionEvent);
    bool detectCircleCollision(CollisionEvent & c_collisionEvent);

    // Private member variables
    const float MIN_SEP_EPSILON = 0.01;
};
