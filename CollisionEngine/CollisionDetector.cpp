#include "CollisionDetector.hpp"
#include <iostream>


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


bool CollisionDetector::detectCollision(Polygon& i_body1, Polygon& i_body2, sf::Vector2f& o_collLoc1, sf::Vector2f& o_collLoc2) {
	std::cout << findMinSeparation(i_body1, i_body2, o_collLoc1) << ", " <<  findMinSeparation(i_body2, i_body1, o_collLoc2) << "\n";
	return findMinSeparation(i_body1, i_body2, o_collLoc1) <= 0 && findMinSeparation(i_body2, i_body1, o_collLoc2) <= 0;
}


float CollisionDetector::findMinSeparation(Polygon& i_body1, Polygon& i_body2, sf::Vector2f& o_collLoc) {
	float separation = std::numeric_limits<float>::lowest();
	int i = 0;
	int j = 0;
	int minIndex = 0;
	int minIndex2 = 0;
	//Loop through all vertices for Body1 and get normal vector
	for (i = 0; i < i_body1.getPointCount(); i++) {
		sf::Vector2f normal = i_body1.getGlobalNormal(i);
		float minSep = std::numeric_limits<float>::max();

		for (j = 0; j < i_body2.getPointCount(); j++) {
			sf::Vector2f pointConnector = sf::Vector2f(i_body2.getGlobalPoint(j).x - i_body1.getGlobalPoint(i).x, i_body2.getGlobalPoint(j).y - i_body1.getGlobalPoint(i).y);
			float dotProd = normal.x * pointConnector.x + normal.y * pointConnector.y;
			if (dotProd <= minSep) {
				minSep = std::min(minSep, dotProd);
				minIndex = j;
			}
		}

		if (minSep > separation) {
			separation = minSep;
			minIndex2 = minIndex;
			
		}
	}
	if (separation < 0) {
		o_collLoc = i_body2.getGlobalPoint(minIndex2);
	}
	return separation;
}
