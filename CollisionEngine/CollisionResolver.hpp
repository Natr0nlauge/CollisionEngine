#pragma once

#include "CollisionDetector.hpp"
#include <mutex>


class CollisionResolver {
  public:
    // Singleton accessor
	static CollisionResolver* getInstance();

	// Destructor
	~CollisionResolver();

	// Public methods
    void handleCollision(collisionEvent_type & c_collEvent);



private:
	// Singleton implementation
	CollisionResolver();
	static CollisionResolver* s_instance;
    static std::mutex mtx;

	// Deleted copy constructor and assignment operator
    CollisionResolver(const CollisionResolver &) = delete;
    CollisionResolver & operator=(const CollisionResolver &) = delete;
	
	// Private methods
    sf::Vector2f computeRelativePosition(const sf::Vector2f collLoc, const sf::Vector2f bodyPosition);
    float calculateContactVelocity(const collisionEvent_type & i_collEvent, sf::Vector2f relativePosition1, sf::Vector2f relativePosition2);
    float calculateDeltaVelPerUnitImpulse(const collisionEvent_type & i_collEvent, sf::Vector2f relativePosition1,
            sf::Vector2f relativePosition2);
    void handleCollision(collisionEvent_type & c_collEvent, sf::Vector2f relativePosition1, sf::Vector2f relativePosition2,
            float i_impulseContactLengthX, float contactTransformationAngle);
};

