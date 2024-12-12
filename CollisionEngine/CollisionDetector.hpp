#pragma once

#include "Polygon.hpp"


struct collisionEvent { //will be used to pass all the necessary information to the Collision Handler
	RigidBody* pBody1;
	RigidBody* pBody2;
};
//TODO check coding standard

class CollisionDetector
{
public:
	static CollisionDetector* getInstance();
	bool detectCollision(Polygon& Body1, Polygon& Body2); 
	//TODO: Give this function a way to write collision Events (for example into a vector)
	//TODO: overloads for detectCollision
	~CollisionDetector();



private:
	static CollisionDetector* s_instance;
	CollisionDetector();
};

