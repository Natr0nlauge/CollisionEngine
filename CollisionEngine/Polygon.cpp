#include "Polygon.hpp"
#include "sfml_utility.hpp"
#include <iostream>

Polygon::Polygon(float i_inverseMass, std::vector<sf::Vector2f> i_vertices) : EdgeStructure(i_inverseMass, i_vertices) {
    
}

Polygon::~Polygon() {}

sf::Vector2f Polygon::getNormal(int i_index) {
    // TODO use utility functions
    sf::Vector2f edge;
    // subtract vertices
    if (i_index == (m_points.size() - 1)) {
        edge = sf::Vector2f(m_points[i_index].x - m_points[0].x, m_points[i_index].y - m_points[0].y);
    } else {
        edge = sf::Vector2f(m_points[i_index].x - m_points[i_index + 1].x, m_points[i_index].y - m_points[i_index + 1].y);
    }
    float vectorLength = sqrt(pow(edge.x, 2) + pow(edge.y, 2));
    sf::Vector2f normal = sf::Vector2f(edge.y / vectorLength, -edge.x / vectorLength);
    return normal;
}

pointSeparationData_type Polygon::calculateMinPointSeparation(sf::Vector2f i_point) {
    pointSeparationData_type separationData;
    sf::Vector2f normal = sf::Vector2f();
    for (int i = 0; i < getPointCount(); i++) {
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