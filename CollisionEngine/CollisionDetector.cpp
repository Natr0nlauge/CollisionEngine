#include "CollisionDetector.hpp"
#include <iostream> 
#include <array>
#include <algorithm>
#include <vector>
#include <cmath>

const float minSepEpsilon = 0.1;



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

CollisionDetector::CollisionDetector() {}

CollisionDetector::~CollisionDetector()
{
}

//TODO: use a struct to transfer data to the collision handler
bool CollisionDetector::detectCollision(Polygon& i_body1, Polygon& i_body2, sf::Vector2f& o_collLoc) {

	basicCollisionData collData2 = findMinSeparation(i_body1, i_body2/*, collIndexVec2*/);
	basicCollisionData collData1 = findMinSeparation(i_body2, i_body1/*, collIndexVec1*/);

	std::cout << collData1.separation << ", " << collData2.separation << "\n";

	// If both minimum seperations are smaller than 0, it indicates a collision
	if (collData1.separation <= 0 && collData2.separation <= 0) /*(collIndexVec2.size()>0 && collIndexVec1.size()>0)*/ {
		// Two values in each vector indicate an edge-to-edge collision
		if (collData1.indexVec.size() > 1 && collData2.indexVec.size() > 1) {
			// Get global coordinates of the colliding edges' vertices
			std::array<float, 4>xValues = { i_body2.getGlobalPoint(collData2.indexVec[0]).x,
					i_body2.getGlobalPoint(collData2.indexVec[1]).x,
					i_body1.getGlobalPoint(collData1.indexVec[0]).x,
					i_body1.getGlobalPoint(collData1.indexVec[1]).x };
			std::array<float, 4>yValues = { i_body2.getGlobalPoint(collData2.indexVec[0]).y,
					i_body2.getGlobalPoint(collData2.indexVec[1]).y,
					i_body1.getGlobalPoint(collData1.indexVec[0]).y,
					i_body1.getGlobalPoint(collData1.indexVec[1]).y };
			// Output center point of contact area
			o_collLoc = findCenterOfContact(xValues, yValues);
		}
		else if (collData2.separation < collData1.separation) {
			// Vertex of body 1 hits edge of body 2
			o_collLoc = i_body1.getGlobalPoint(collData1.indexVec[0]);
		}
		else/*if (collData2.separation > collData1.separation)*/ {
			// Vertex of body 2 hits edge of body 1
			o_collLoc = i_body2.getGlobalPoint(collData2.indexVec[0]);

		}
		return true;
	}
	else {
		return false;
	}


}


//TODO: Output collision normal
basicCollisionData CollisionDetector::findMinSeparation(Polygon& i_body1, Polygon& i_body2/*, std::vector<int>& o_collIndexVec*/) {

	// minSepValues are really useful for debugging!
	std::vector<std::pair<float, int>> minSepValues;
	std::vector<std::pair<float, int>> minSepValues2;
	basicCollisionData collData;
	
	std::vector<int> preliminaryCollIndexVec;
	std::vector<int> preliminaryCollIndexVec2;

	// Loop through all vertices for Body1 and get normal vector
	for (int i = 0; i < i_body1.getPointCount(); i++) {
		sf::Vector2f normal = i_body1.getGlobalNormal(i);
		float minSep = std::numeric_limits<float>::max();

		for (int j = 0; j < i_body2.getPointCount(); j++) {
			// Calculate dot product for each normal and for each connecting line between vertices
			sf::Vector2f pointConnector = sf::Vector2f(i_body2.getGlobalPoint(j).x - i_body1.getGlobalPoint(i).x, i_body2.getGlobalPoint(j).y - i_body1.getGlobalPoint(i).y);
			float dotProd = normal.x * pointConnector.x + normal.y * pointConnector.y;
			// Find minimum value of all possible dot products (for each normal vector)
			if (dotProd < minSep - minSepEpsilon) {
				//minSep = std::min(minSep, dotProd);
				minSep = dotProd;
				minSepValues.clear();
				minSepValues.emplace_back(dotProd, j);
				preliminaryCollIndexVec.clear(); //previous indices are irrelevant
				preliminaryCollIndexVec.push_back(j); //save index
			}
			// If a minSep value is sufficiently close to a previous one, save another index (edge-to-edge-collision possible)
			// Tolerance needs to be larger here to correctly account for angle deviations
			else if (fabs(minSep - dotProd) <= 4*minSepEpsilon) { 
				minSepValues.emplace_back(dotProd, j);
				preliminaryCollIndexVec.push_back(j);
			}

		}

		// The maximum minSep value for all normal vectors (for all i values) is the minimal seperation
		if (minSep > collData.separation) {
			collData.separation = minSep;
			minSepValues2 = minSepValues;
			preliminaryCollIndexVec2 = preliminaryCollIndexVec;
		}
	}
	if (collData.separation < 0) {
		//minSepValues2 = minSepValues;
		minSepValues.clear();
	}

	collData.indexVec = preliminaryCollIndexVec2; //output
	std::cout << collData.indexVec.size() << ", ";
	return collData;
}

sf::Vector2f CollisionDetector::findCenterOfContact(const std::array<float, 4>& i_xValues, const std::array<float, 4>& i_yValues)
{
	return sf::Vector2f(computeMedian(i_xValues), computeMedian(i_yValues));
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
	int newIndex = i_pointCount - 1;
	if (i_index != 0) {
		int newIndex = i_index - 1;
	}
	return newIndex;

}

int CollisionDetector::decrIndex(int i_index, int i_pointCount)
{
	int newIndex = 0;
	if (i_index != i_pointCount - 1) {
		int newIndex = i_index + 1;
	}
	return newIndex;
}
