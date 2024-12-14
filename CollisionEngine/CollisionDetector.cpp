#include "CollisionDetector.hpp"
#include <iostream> 
#include <array>
#include <algorithm>

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

//TODO: use struct instead of output parameters
//TODO: Maybe switch names for collIndexes
bool CollisionDetector::detectCollision(Polygon& i_body1, Polygon& i_body2, sf::Vector2f& o_collLoc) {
	
	int collIndex1 = 0;
	int collIndex2 = 0;
	float minSep1 = findMinSeparation(i_body1, i_body2, collIndex1);
	float minSep2 = findMinSeparation(i_body2, i_body1, collIndex2);
	sf::Vector2f collLoc1 = i_body2.getGlobalPoint(collIndex1);
	sf::Vector2f collLoc2 = i_body1.getGlobalPoint(collIndex2);

	std::cout << minSep1 << ", " << minSep2 << "\n";
	if (minSep1 <= 0 && minSep2 <= 0) {

		//TODO: Change the conditions. In some cases edge-to-edge collisions may not be recognized
		if (minSep1 < minSep2) {
			o_collLoc = collLoc2;
			return true;
		}
		else if (minSep1 > minSep2) {
			o_collLoc = collLoc1;
			return true;
		}
		else /*if minSep1 == minSep2 */ {
			//TODO: check if this is still operational after modifying the Polygon definition
			
			int collIndex1Decr = 0;
			int collIndex2Decr = 0;
			//TODO: write a designated function for this, it is needed a few times in the project
			//avoid index out-of-bounds
			//problem: Indic
			/*if (collIndex1 != 0) {
				int collIndex1Decr = collIndex1-1;
			}
			else {
				int collIndex1Decr = i_body2.getPointCount()-1;
			}

			if (collIndex2 != i_body1.getPointCount() - 1) {
				int collIndex2Decr = collIndex2+1;
			}
			else {
				int collIndex2Decr = 0;
			}*/


			std::array<float, 4>xValues = {i_body2.getGlobalPoint(collIndex1).x,
					i_body2.getGlobalPoint(collIndex1Decr).x,
					i_body1.getGlobalPoint(collIndex2).x,
					i_body1.getGlobalPoint(collIndex2Decr).x};
			std::array<float, 4>yValues = { i_body2.getGlobalPoint(collIndex1).y,
					i_body2.getGlobalPoint(collIndex1Decr).y,
					i_body1.getGlobalPoint(collIndex2).y,
					i_body1.getGlobalPoint(collIndex2Decr).y};
			//o_collLoc = findCenterOfContact(xValues, yValues);
			o_collLoc = collLoc1;
			return true;
		}
	}
	else {
		return false;
	}
	
	
}

//TODO: use struct instead of output parameters
//TODO: fix variable names
float CollisionDetector::findMinSeparation(Polygon& i_body1, Polygon& i_body2, int& o_collIndex) {
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
	if (separation <= 0) {
		//o_collLoc = i_body2.getGlobalPoint(minIndex2);
		o_collIndex = minIndex2;
	}
	return separation;
}

sf::Vector2f CollisionDetector::findCenterOfContact(const std::array<float, 4>& i_xValues, const std::array<float, 4>& i_yValues)
{
	return sf::Vector2f(computeMedian(i_xValues),computeMedian(i_yValues));
}

float CollisionDetector::computeMedian(const std::array<float, 4>& i_arr) {
	// Make a copy of the array because we need to sort it
	std::array<float, 4> sortedArr = i_arr;
	std::sort(sortedArr.begin(), sortedArr.end());

	// Compute and return the median (average of the two middle elements)
	return (sortedArr[1] + sortedArr[2]) / 2.0f;
}
