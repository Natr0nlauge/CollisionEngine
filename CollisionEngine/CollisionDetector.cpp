#include "CollisionDetector.hpp"
#include <iostream> 
#include <array>
#include <algorithm>
#include <vector>
#include <cmath>
#include "sfmlUtility.hpp"

CollisionDetector* CollisionDetector::s_instance = nullptr;

const float minSepEpsilon = 0.01;

static void assignNormals(collisionEvent& o_collisionEvent, basicSeparationData& i_collData) {
	o_collisionEvent.normal1 = i_collData.normal;
	o_collisionEvent.normal2.x = -i_collData.normal.x;
	o_collisionEvent.normal2.y = -i_collData.normal.y;
}

// TODO add this to some utility module
static float computeMedian(const std::array<float, 4>& i_arr) {
	// Make a copy of the array because we need to sort it
	std::array<float, 4> sortedArr = i_arr;
	std::sort(sortedArr.begin(), sortedArr.end());

	// Compute and return the median (average of the two middle elements)
	return (sortedArr[1] + sortedArr[2]) / 2.0f;
}

// Find global coordinates of the colliding edges' vertices
static sf::Vector2f findCenterOfContact(basicSeparationData& i_sepData1, basicSeparationData& i_sepData2, Polygon& i_body1, Polygon& i_body2) {
	const int numberOfPoints = 4;
	sf::Vector2f vertices[numberOfPoints];
	std::array<float, numberOfPoints> xValues = {0.0f, 0.0f, 0.0f, 0.0f}; 
	std::array<float, numberOfPoints> yValues = {0.0f, 0.0f, 0.0f, 0.0f};

	vertices[0] = i_body2.getGlobalPoint(i_sepData2.indexVec[0]);
	vertices[1] = i_body2.getGlobalPoint(i_sepData2.indexVec[1]);
	vertices[2] = i_body1.getGlobalPoint(i_sepData1.indexVec[0]);
	vertices[3] = i_body1.getGlobalPoint(i_sepData1.indexVec[1]);

	for (int i=0; i < numberOfPoints; i++) {
		xValues[i] = vertices[i].x;
		yValues[i] = vertices[i].y;
	}

	return sf::Vector2f(computeMedian(xValues), computeMedian(yValues));
}





CollisionDetector* CollisionDetector::getInstance() {
	if (s_instance == nullptr) {
		//TODO: check thread safety
		//std::lock_guard<std::mutex> lock(mtx);
		if (s_instance == nullptr) {
			s_instance = new CollisionDetector();
		}
	}
	return s_instance;
}



CollisionDetector::CollisionDetector() {
}

CollisionDetector::~CollisionDetector() {
}

//TODO: case differentiation polygon and circle
// Detects a collision between two bodies and writes results to a collisionEvent
bool CollisionDetector::detectCollision(collisionEvent& c_collisionEvent) {
	Polygon & body1 = static_cast<Polygon&>(c_collisionEvent.rBody1);
	Polygon & body2 = static_cast<Polygon&>(c_collisionEvent.rBody2);

	basicSeparationData collData2 = findMinSeparation(body1, body2);
	basicSeparationData collData1 = findMinSeparation(body2, body1);
	//std::cout << collData1.separation << ", " << collData2.separation << "\n";
	// If both minimum seperations are smaller than 0, it indicates a collision
	if (collData1.separation <= 0 && collData2.separation <= 0) /*(collIndexVec2.size()>0 && collIndexVec1.size()>0)*/ {
		// Two values in each vector indicate an edge-to-edge collision
		if (collData1.indexVec.size() > 1 && collData2.indexVec.size() > 1) {
			collData1.normal = sfu::scaleVector(collData1.normal, -1); //make sure that normal has the correct direction
			c_collisionEvent.collLoc1 = findCenterOfContact(collData1, collData2, body1, body2);
			assignNormals(c_collisionEvent,  collData1); //TODO this causes errors!
		}
		else if (collData2.separation < collData1.separation) {
			// Vertex of body 1 hits edge of body 2
			// TODO check if normals have the correct direction
			// Assign location
			c_collisionEvent.collLoc1 = body1.getGlobalPoint(collData1.indexVec[0]);
			collData1.normal = sfu::scaleVector(collData1.normal, -1); //make sure that normal has the correct direction
			assignNormals(c_collisionEvent, collData1);
		}
		else/*if (collData2.separation > collData1.separation)*/ {
			// Vertex of body 2 hits edge of body 1
			// Assign location
			c_collisionEvent.collLoc1 = body2.getGlobalPoint(collData2.indexVec[0]);
			//collData2.normal = sfu::scaleVector(collData2.normal, -1);
			assignNormals(c_collisionEvent,  collData2);
		}
		//std::cout << "Position in CollisionDetector: " << c_collisionEvent.collLoc1.x << ", " << c_collisionEvent.collLoc1.y << "\n";
		return true;
	}
	else {
		return false;
	}


}



//TODO: use some sub-functions here to make it easier to read
basicSeparationData CollisionDetector::findMinSeparation(Polygon& i_body1, Polygon& i_body2/*, std::vector<int>& o_collIndexVec*/) {

	// minSepValues are really useful for debugging!
	std::vector<std::pair<float, int>> minSepValues;
	std::vector<std::pair<float, int>> minSepValues2;
	basicSeparationData collData;
	int normalIndex = 0;
	
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
			normalIndex = i;
			collData.normal = normal;
		}
	}
	if (collData.separation < 0) {
		//minSepValues2 = minSepValues;
		minSepValues.clear();
		//std::cout << collData.normal.x << ", " << collData.normal.y << "\n";
	}

	collData.indexVec = preliminaryCollIndexVec2; //output
	//std::cout << collData.indexVec.size() << ", ";
	return collData;
}


//int CollisionDetector::incrIndex(int i_index, int i_pointCount)
//{
//	int newIndex = i_pointCount - 1;
//	if (i_index != 0) {
//		int newIndex = i_index - 1;
//	}
//	return newIndex;
//
//}
//
//int CollisionDetector::decrIndex(int i_index, int i_pointCount)
//{
//	int newIndex = 0;
//	if (i_index != i_pointCount - 1) {
//		int newIndex = i_index + 1;
//	}
//	return newIndex;
//}
