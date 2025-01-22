#include "BoundaryElement.hpp"
#include "sfml_utility.hpp"
#include <iostream>

BoundaryElement::BoundaryElement(float i_length)
    : m_length(i_length), EdgeStructure(0.0f, {sf::Vector2f(-i_length / 2, 0.0f), sf::Vector2f(i_length / 2, 0.0f)}) {}

BoundaryElement::~BoundaryElement() {}

sf::Vertex * BoundaryElement::getVertexArray() {
    m_vertexArray[0] = sf::Vertex(getGlobalPoint(0));
    m_vertexArray[1] = sf::Vertex(getGlobalPoint(1));
    return m_vertexArray;
}

pointSeparationData_type BoundaryElement::calculateMinPointSeparation(sf::Vector2f i_point) {
    pointSeparationData_type separationData;
    sf::Vector2f normal = sf::Vector2f();

    sf::Vector2f testVertex = getPosition();
    sf::Vector2f testNormal = getGlobalNormal(0);
    sf::Vector2f pointConnector = sfu::subtractVectors(i_point, testVertex);

    separationData.separation = sfu::getVectorLength(pointConnector)-m_length/2; // previous indices are irrelevant
    separationData.index = 0;
    separationData.normal = testNormal;

    return separationData;
}

sf::Vector2f BoundaryElement::getNormal(int i_index) {
    // Index doesn't matter in this case
    // TODO use utility functions
    sf::Vector2f edge;
    // subtract vertices
    // if (i_index == (m_points.size() - 1)) {
    edge = sf::Vector2f(m_points[1].x - m_points[0].x, m_points[1].y - m_points[0].y);
    //} else {
    // edge = sf::Vector2f(m_points[i_index].x - m_points[i_index + 1].x, m_points[i_index].y - m_points[i_index + 1].y);
    //}
    float vectorLength = sqrt(pow(edge.x, 2) + pow(edge.y, 2));
    sf::Vector2f normal = sf::Vector2f(edge.y / vectorLength, -edge.x / vectorLength);
    return normal;
}