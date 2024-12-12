#include "CollisionDetector.hpp"


CollisionDetector* CollisionDetector::s_instance = nullptr;

CollisionDetector* CollisionDetector::getInstance() {
	if (s_instance == nullptr) {
		//std::lock_guard<std::mutex> lock(mtx);
		if (s_instance == nullptr) {
			s_instance = new CollisionDetector();
		}
	}
	return s_instance;
}

CollisionDetector::CollisionDetector(){}

CollisionDetector::~CollisionDetector()
{
}


bool CollisionDetector::detectCollision(Polygon& Body1, Polygon& Body2) {
	
	
	
	return true;
}


float findMinSeparation(Polygon& Body1, Polygon& Body2) {
	float separation = FLT_MIN;

	//Loop through all vertices for Body1 and get normal vector
	/*for (sf::Vector2f va : Body1->getPoints()) {
		sf::Vector2f normal = Body1->getNormal
	}*/


	return separation;
}
