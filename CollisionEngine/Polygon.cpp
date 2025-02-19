#include "Polygon.hpp"
#include "sfml_utility.hpp"
#include <iostream>

/**
 * @brief Constructor.
 * @param i_inverseMass The inverse mass of the body. Put in zero for an immovable body.
 * @param i_vertices A vector holding the coordinates of the Polygon's corners. Define them in counter-clockwise order.
 * 
 * @note Polygon has to be convex to make the SAT algorithm work correctly.
 */
Polygon::Polygon(float i_inverseMass, std::vector<sf::Vector2f> i_vertices) : VertexBasedBody(i_inverseMass, i_vertices) {
    
}

Polygon::~Polygon() {}


/**
 * @brief Calculates the separation of a single point from the polygon using a simplified SAT algorithm.
 * @param i_point The point coordinates in global coordinates.
 * @return Struct holding the separation and associated data.
 */
pointSeparationData_type Polygon::calculateMinPointSeparation(sf::Vector2f i_point) {
    pointSeparationData_type separationData;
    sf::Vector2f normal = sf::Vector2f();
    for (int i = 0; i < getNormalCount(); i++) {
        sf::Vector2f testVertex = getGlobalPoint(i);
        sf::Vector2f testNormal = getGlobalNormal(i);
        sf::Vector2f pointConnector = sfu::subtractVectors(i_point, testVertex);
        // std::cout << testPoint.x << ", " << testPoint.y << "\n";
        float newSeparation = sfu::scalarProduct(testNormal, pointConnector);
        // Edge separation is the maximum value of all possible edge separations
        if (newSeparation > separationData.separation) {
            // minSep = std::min(minSep, dotProd);
            separationData.separation = newSeparation; // previous indices are irrelevant
            separationData.index = i;
            separationData.normal = testNormal;
        }
    }

    return separationData;
}

int Polygon::getNormalCount() {
    return getPointCount();
}
