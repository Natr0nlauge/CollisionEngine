#pragma once

#include "Polygon.hpp"


struct collisionEvent { //will be used to pass all the necessary information to the Collision Handler
	RigidBody* pBody1;
	RigidBody* pBody2;
};

struct basicCollisionData {
	float separation;
	std::vector<int> indexVec;
};


class CollisionDetector
{
public:
	static CollisionDetector* getInstance();
	//TODO: Give this function a way to write collision Events (for example into a vector)
	//TODO: overloads for detectCollision
	~CollisionDetector();
	bool detectCollision(Polygon& i_body1, Polygon& i_body2, sf::Vector2f& o_collLoc);



private:
	static CollisionDetector* s_instance;
	CollisionDetector();
	basicCollisionData findMinSeparation(Polygon& i_body1, Polygon& i_body2/*, std::vector<int>& o_collIndexVec, sf::Vector2f& o_collLoc*/);
	sf::Vector2f findCenterOfContact(const std::array<float, 4>& i_xValues, const std::array<float, 4>& i_yValues);
	float computeMedian(const std::array<float, 4>& i_arr);
	int incrIndex(int i_index, int max_index);
	int decrIndex(int i_index, int max_index);
};

