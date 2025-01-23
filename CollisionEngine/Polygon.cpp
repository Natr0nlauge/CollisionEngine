#include "Polygon.hpp"
#include "sfml_utility.hpp"
#include <iostream>

Polygon::Polygon(float i_inverseMass, std::vector<sf::Vector2f> i_vertices) : EdgeStructure(i_inverseMass, i_vertices) {
    
}

Polygon::~Polygon() {}



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
