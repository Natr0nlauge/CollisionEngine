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
 * @brief Return a normal vector corresponding to an edge.
 * @param i_index The index of the edge.
 * @return The normal vector in body coordinates.
 */
sf::Vector2f Polygon::getNormal(int i_index) {
    // subtract vertices
    int incrIndex = (i_index + 1) % m_points.size();
    // if (i_index == (m_points.size() - 1)) {
    //     edge = sf::Vector2f(m_points[i_index].x - m_points[0].x, m_points[i_index].y - m_points[0].y);
    // } else {
    sf::Vector2f edge = sfu::subtractVectors(m_points[i_index], m_points[incrIndex]);
    // edge = sf::Vector2f(m_points[i_index].x - m_points[i_index + 1].x, m_points[i_index].y - m_points[i_index + 1].y);
    // }
    float vectorLength = (float)sqrt(pow(edge.x, 2) + pow(edge.y, 2));
    sf::Vector2f normal = sfu::rotateVector(edge, -90.0f);
    normal = sfu::normalizeVector(normal);
    return normal;
}

/**
 * @brief Calculates the separation of a single point from the polygon using a simplified SAT algorithm.
 * @param i_point The point coordinates in global coordinates.
 * @return Struct holding the separation and associated data.
 */
pointSeparationData_type Polygon::calculateMinPointSeparation(sf::Vector2f i_point) {
    pointSeparationData_type separationData;
    sf::Vector2f normal = sf::Vector2f();
    // Iterate through all normal vectors of the polygon
    for (int i = 0; i < getNormalCount(); i++) {
        // Determine signed distance to each edge
        sf::Vector2f testVertex = getGlobalPoint(i);
        sf::Vector2f testNormal = getGlobalNormal(i);
        sf::Vector2f pointConnector = sfu::subtractVectors(i_point, testVertex);
        float newSeparation = sfu::scalarProduct(testNormal, pointConnector);
        // Final edge separation is the maximum value of all possible edge separations
        if (newSeparation > separationData.separation) {
            // Assign results
            separationData.separation = newSeparation;
            separationData.index = i;
            separationData.normal = testNormal;
        }
    }

    return separationData;
}

size_t Polygon::getNormalCount() {
    return getPointCount();
}
