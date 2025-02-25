#include "BoundaryElement.hpp"
#include "sfml_utility.hpp"
#include <iostream>

BoundaryElement::BoundaryElement(float i_length)
    : m_length(i_length), VertexBasedBody(0.0f, {sf::Vector2f(-i_length / 2, 0.0f), sf::Vector2f(i_length / 2, 0.0f)}) {}

BoundaryElement::~BoundaryElement() {}

sf::Vertex * BoundaryElement::getVertexArray() {
    m_vertexArray[0] = sf::Vertex(getGlobalPoint(0));
    m_vertexArray[1] = sf::Vertex(getGlobalPoint(1));
    return m_vertexArray;
}

/**
 * @brief Calculates the separation of any point to the center of the BoundaryElement. This is used to check if a calculated collision point
 * is actually inside the BoundaryElement.
 * @param i_point The point to test in global coordinates.
 * @return Struct holding the separation and associated data.
 */
pointSeparationData_type BoundaryElement::calculateMinPointSeparation(sf::Vector2f i_point) {
    pointSeparationData_type separationData;

    // We only have a single normal vector, so iteration won't be required.
    sf::Vector2f testVertex = getPosition();
    sf::Vector2f testNormal = getGlobalNormal(0);
    sf::Vector2f pointConnector = sfu::subtractVectors(i_point, testVertex);

    separationData.separation = sfu::getVectorLength(pointConnector) - m_length / 2; // previous indices are irrelevant
    separationData.index = 0;
    separationData.normal = testNormal;

    return separationData;
}

int BoundaryElement::getNormalCount() {
    // There is only one normal vector
    return 1;
}

sf::Vector2f BoundaryElement::getNormal(int i_index) {
    return sf::Vector2f({0.0f,1.0f});
}
