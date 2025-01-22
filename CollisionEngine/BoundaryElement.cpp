#include "BoundaryElement.hpp"
#include "sfml_utility.hpp"

BoundaryElement::BoundaryElement(float i_length) : Polygon(0.0f, {sf::Vector2f(-i_length/2, 0.0f), sf::Vector2f(i_length/2, 0.0f)}) {

}

BoundaryElement::~BoundaryElement() {}

sf::Vertex * BoundaryElement::getVertexArray() {
    m_vertexArray[0] = sf::Vertex(getGlobalPoint(0));
    m_vertexArray[1] = sf::Vertex(getGlobalPoint(1));
    return m_vertexArray;
}

pointSeparationData_type BoundaryElement::calculateMinPointSeparation(sf::Vector2f i_point) {
    pointSeparationData_type separationData;
    sf::Vector2f normal = sf::Vector2f();
    for (int i = 0; i < getPointCount(); i++) {
        sf::Vector2f testVertex = getGlobalPoint(i);
        sf::Vector2f testNormal = getGlobalNormal(i);
        sf::Vector2f pointConnector = sfu::subtractVectors(i_point, testVertex);
        // std::cout << testPoint.x << ", " << testPoint.y << "\n";
        float newSeparation = sfu::scalarProduct(testNormal, pointConnector);
        // Edge separation is the maximum value of all possible edge separations
        //separationData.separation = std::numeric_limits<float>::max();
        if (newSeparation > separationData.separation) {
            // minSep = std::min(minSep, dotProd);
            separationData.separation = newSeparation; // previous indices are irrelevant
            separationData.index = i;
            separationData.normal = testNormal;
        }
    }

    return separationData;
}

sf::Vector2f BoundaryElement::getNormal(int i_index) {
    // TODO use utility functions
    sf::Vector2f edge;
    // subtract vertices

        edge = sf::Vector2f(m_points[i_index].x - m_points[0].x, m_points[i_index].y - m_points[0].y);
   
    float vectorLength = sqrt(pow(edge.x, 2) + pow(edge.y, 2));
    sf::Vector2f normal = sf::Vector2f(edge.y / vectorLength, -edge.x / vectorLength);
    return sf::Vector2f(5.0f,0.0f);
}
