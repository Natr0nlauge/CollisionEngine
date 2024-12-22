#pragma once

#include "Polygon.hpp"

// 
struct collisionEvent { // Will be used to pass all the necessary information to the Collision Handler
	RigidBody& rBody1;
	RigidBody& rBody2;
	sf::Vector2f collLoc1; // In global coordinates
	//TODO: Check if saving collision Location in local coordinates is more efficient
	sf::Vector2f normal1;
	sf::Vector2f normal2;

	// Constructor
	collisionEvent(RigidBody& rb1, RigidBody& rb2) : rBody1(rb1), rBody2(rb2) {
	}
};

// Includes some data about the separation
struct basicSeparationData {
	float separation = std::numeric_limits<float>::lowest();
	std::vector<int> indexVec;
	sf::Vector2f normal;
};


class CollisionDetector
{
public:
	static CollisionDetector* getInstance();
	//TODO: Give this function a way to write collision Events (for example into a vector)
	//TODO: overloads for detectCollision
	~CollisionDetector();
	bool detectCollision(collisionEvent& c_collisionEvent);



private:
	static CollisionDetector* s_instance;
	CollisionDetector();
	basicSeparationData findMinSeparation(Polygon& i_body1, Polygon& i_body2);
	sf::Vector2f findCenterOfContact(const std::array<float, 4>& i_xValues, const std::array<float, 4>& i_yValues);
	float computeMedian(const std::array<float, 4>& i_arr);
	int incrIndex(int i_index, int max_index);
	int decrIndex(int i_index, int max_index);
};

