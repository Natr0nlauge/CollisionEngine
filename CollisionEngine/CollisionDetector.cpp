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


bool CollisionDetector::detectCollision(Polygon& i_body1, Polygon& i_body2) {
	std::cout << findMinSeparation(i_body1, i_body2) << ", " <<  findMinSeparation(i_body2, i_body1) << "\n";
	return findMinSeparation(i_body1, i_body2) <= 0 && findMinSeparation(i_body2, i_body1) <= 0;
}


float CollisionDetector::findMinSeparation(Polygon& i_body1, Polygon& i_body2) {
	float separation = std::numeric_limits<float>::lowest();

	//Loop through all vertices for Body1 and get normal vector
	for (int i = 0; i < i_body1.getPointCount(); i++) {
		sf::Vector2f normal = i_body1.getGlobalNormal(i);
		float minSep = std::numeric_limits<float>::max();

		for (int j = 0; j < i_body2.getPointCount(); j++) {
			sf::Vector2f pointConnector = sf::Vector2f(i_body2.getGlobalPoint(j).x - i_body1.getGlobalPoint(i).x, i_body2.getGlobalPoint(j).y - i_body1.getGlobalPoint(i).y);
			float dotProd = normal.x * pointConnector.x + normal.y * pointConnector.y;
			minSep = std::min(minSep, dotProd);
		}

		if (minSep > separation)
			separation = minSep;
	}
	return separation;
}
