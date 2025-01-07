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
	void handleCollision(collisionEvent_type& colEvent);



private:
	// Singleton implementation
	CollisionResolver();
	static CollisionResolver* s_instance;
    static std::mutex mtx;

	// Deleted copy constructor and assignment operator
    CollisionResolver(const CollisionResolver &) = delete;
    CollisionResolver & operator=(const CollisionResolver &) = delete;
};

