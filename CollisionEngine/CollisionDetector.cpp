#include "CollisionDetector.hpp"
#include <iostream> 
#include <array>
#include <algorithm>
#include <vector>
#include <cmath>

const float minSepEpsilon = 0.5; //TODO adjust this (?)

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
//TODO: Maybe switch names for collIndexes (index1 is currently for body 2...)
bool CollisionDetector::detectCollision(Polygon& i_body1, Polygon& i_body2, sf::Vector2f& o_collLoc) {
	
	std::vector<int> collIndexVec1;
	std::vector<int> collIndexVec2;
	float minSep1 = findMinSeparation(i_body1, i_body2, collIndexVec1);
	float minSep2 = findMinSeparation(i_body2, i_body1, collIndexVec2);
	sf::Vector2f collLoc1 = i_body2.getGlobalPoint(collIndexVec1[0]);
	sf::Vector2f collLoc2 = i_body1.getGlobalPoint(collIndexVec2[0]);

	std::cout << minSep1 << ", " << minSep2 << "\n";
	if (minSep1 <= 0 && minSep2 <= 0) {

		//TODO: Change the conditions. In some cases edge-to-edge collisions may not be recognized
		 if (collIndexVec1.size()>1 && collIndexVec2.size() > 1) {
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


			std::array<float, 4>xValues = {i_body2.getGlobalPoint(collIndexVec1[0]).x,
					i_body2.getGlobalPoint(collIndexVec1[1]).x,
					i_body1.getGlobalPoint(collIndexVec2[0]).x,
					i_body1.getGlobalPoint(collIndexVec2[1]).x};
			std::array<float, 4>yValues = { i_body2.getGlobalPoint(collIndexVec1[0]).y,
					i_body2.getGlobalPoint(collIndexVec1[1]).y,
					i_body1.getGlobalPoint(collIndexVec2[0]).y,
					i_body1.getGlobalPoint(collIndexVec2[1]).y};
			o_collLoc = findCenterOfContact(xValues, yValues);
			//o_collLoc = collLoc1;
			return true;
		}else if (minSep1 < minSep2 /*- minSepEpsilon*/) {
			o_collLoc = collLoc2;
			return true;
		}
		else/* if (minSep1 > minSep2 + minSepEpsilon)*/ {
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
float CollisionDetector::findMinSeparation(Polygon& i_body1, Polygon& i_body2, std::vector<int>& o_collIndexVec) {
	float separation = std::numeric_limits<float>::lowest();
	int i = 0;
	int j = 0;
	int minIndex = 0;
	int minIndex_ = 0;
	int minIndex2 = 0;
	std::vector<std::pair<float, int>> minSepValues;
	std::vector<std::pair<float, int>> minSepValues2;
	std::vector<int> preliminaryCollIndexVec;
	std::vector<int> preliminaryCollIndexVec2;

	//Loop through all vertices for Body1 and get normal vector
	for (i = 0; i < i_body1.getPointCount(); i++) {
		sf::Vector2f normal = i_body1.getGlobalNormal(i);
		float minSep = std::numeric_limits<float>::max();

		for (j = 0; j < i_body2.getPointCount(); j++) {
			sf::Vector2f pointConnector = sf::Vector2f(i_body2.getGlobalPoint(j).x - i_body1.getGlobalPoint(i).x, i_body2.getGlobalPoint(j).y - i_body1.getGlobalPoint(i).y);
			float dotProd = normal.x * pointConnector.x + normal.y * pointConnector.y;
			if (dotProd < minSep-minSepEpsilon) {
				//minSep = std::min(minSep, dotProd);
				minSep = dotProd;
				minIndex = j;
				//minSepValues.clear();
				minSepValues.emplace_back(dotProd, j);
				preliminaryCollIndexVec.clear();
				preliminaryCollIndexVec.push_back(j);
			}
			else if (fabs(minSep-dotProd) <= minSepEpsilon) {
				minSepValues.emplace_back(dotProd, j);
				preliminaryCollIndexVec.push_back(j);
			}
			
		}

		if (minSep >= separation) {
			separation = minSep;
			minIndex2 = minIndex;
			minSepValues2 = minSepValues;
			//minSepValues.clear();
			preliminaryCollIndexVec2 = preliminaryCollIndexVec;
		}
		/*else {
			minSepValues.clear();
			//minSepValues2.clear();
		}*/
	}
	if (separation <= 0) {
		//if any j indices have the same minSep value (and all other minSep values are larger)->save and return the indices

		o_collIndexVec = preliminaryCollIndexVec2;
		//o_collIndexVec.push_back(minIndex2);
	}
	else {
		o_collIndexVec.push_back(0); //TODO: check why this is actually necessary?
	}
	std::cout << o_collIndexVec.size() << ", ";
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

int CollisionDetector::incrIndex(int i_index, int i_pointCount)
{
	int newIndex = i_pointCount -1;
	if (i_index != 0) {
		int newIndex = i_index - 1;
	}
	return newIndex;
	
}

int CollisionDetector::decrIndex(int i_index, int i_pointCount)
{	
	int newIndex = 0;
	if (i_index != i_pointCount -1){
		int newIndex = i_index + 1;
	}
	return newIndex;
}

/*std::vector<int>& findIndices(std::vector<std::pair<float, int>> minSepValues) {

}*/