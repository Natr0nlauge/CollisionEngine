#pragma once

#include "CollisionDetector.hpp"
#include "sfmlUtility.hpp"

class CollisionResolver
{
public:
	static CollisionResolver* getInstance();
	~CollisionResolver();
	void handleCollision(collisionEvent& colEvent);



private:
	CollisionResolver();
	static CollisionResolver* s_instance;
};

