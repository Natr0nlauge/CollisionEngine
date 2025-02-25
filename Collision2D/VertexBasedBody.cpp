#include "VertexBasedBody.hpp"
#include "sfml_utility.hpp"

/**
 * @brief Constructor.
 * @param i_inverseMass The inverse mass of the body. Put in zero for an immovable body.
 * @param i_vertices A vector holding the coordinates of the vertices (2 vertices for BoundaryElement, 3 or more vertices for Polygon).
 */
VertexBasedBody::VertexBasedBody(float i_inverseMass, std::vector<sf::Vector2f> i_vertices) : RigidBody(i_inverseMass) {
    m_points = i_vertices;
    calculateAndSetArea();
    setOrigin(sf::Vector2f(0.0f, 0.0f));

    // Redefine all the vertices, so the center of mass is at {0,0}
    sf::Vector2f com = calculateCenterOfMass();
    for (size_t i = 0; i < getPointCount(); ++i) {
        sf::Vector2f & current = m_points[i];
        current.x -= com.x;
        current.y -= com.y;
    }
    m_inverseMomentOfInertia = calculateInverseMomentOfInertia();
}

// Destructor
VertexBasedBody::~VertexBasedBody() {}

// Returns all points of the body
std::vector<sf::Vector2f> VertexBasedBody::getPoints() {
    return m_points;
}

// Returns a point according to the index in global coordinates
sf::Vector2f VertexBasedBody::getGlobalPoint(int i_index) {
    sf::Vector2f transformedPoint = transformPointToGlobal(m_points[i_index]);
    return transformedPoint;
}

/**
 * @brief Return a normal vector corresponding to an edge in global coordinates.
 * @param i_index The index of the edge.
 * @return The normal vector in global coordinates.
 */
sf::Vector2f VertexBasedBody::getGlobalNormal(int i_index) {
    sf::Vector2f normal;
    normal = transformVectorToGlobal(getNormal(i_index));

    return normal;
}

/**
 * @brief Calculate the signed area of the polygon based on the specified vertices using the shoelace formula.
 * @return The signed area given in pixel^2.
 */
float VertexBasedBody::calculateSignedArea() {
    size_t numberOfVertices = m_points.size();

    if (numberOfVertices < 3) {
        return 0.0f; // A polygon must have at least 3 vertices
    }
    float area = 0.0f;

    for (size_t i = 0; i < numberOfVertices; i++) {
        // Current vertex
        const sf::Vector2f & current = m_points[i];
        // Next vertex (wrapping around at the end)
        const sf::Vector2f & next = m_points[(i + 1) % numberOfVertices];

        // Add cross-product to the area sum
        area += current.x * next.y - current.y * next.x;
    }

    return area / 2.0f;
}

/**
 * @brief Calculate the covered area of the body based on the specified vertices and set the m_area member variable.
 */
void VertexBasedBody::calculateAndSetArea() {
    m_area = std::abs(calculateSignedArea());
}

/**
 * @brief Calculate the inverse moment of inertia of the body based on the specified vertices.
 * @return The inverse moment of inertia measured in 1/(mass unit * pixel^2).
 */
float VertexBasedBody::calculateInverseMomentOfInertia() {
    if (m_points.size() < 3) {
        return 0.0f; // A polygon needs at least 3 points
    }
    float numerator = 0.0f;
    float denominator = 0.0f;
    float inverseDensity = calculateInverseDensity();
    float inverseMass = getInverseMass();
    sf::Vector2f centroid = calculateCenterOfMass();
    std::size_t n = m_points.size();

    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n; // Next vertex index (wrap around)
        float xi = m_points[i].x - centroid.x;
        float yi = m_points[i].y - centroid.y;
        float xj = m_points[j].x - centroid.x;
        float yj = m_points[j].y - centroid.y;

        double commonTerm = std::abs(xi * yj - xj * yi);
        numerator += (xi * xi + xi * xj + xj * xj + yi * yi + yi * yj + yj * yj) * commonTerm;
        denominator += commonTerm;
    }

    float moi = numerator / (denominator * 6.0f);
    float invMoi = inverseMass / std::abs(moi);

    return invMoi;
}

/**
 * @brief Calculate the center of mass of the body based on the specified vertices.
 * @return The coordinates of the center of mass relative to {0.0,0.0}.
 */
sf::Vector2f VertexBasedBody::calculateCenterOfMass() {
    size_t numberOfVertices = m_points.size();
    if (numberOfVertices < 3) {
        return sf::Vector2f(0.0f, 0.0f); // A polygon must have at least 3 vertices
    }

    float signedArea = calculateSignedArea();
    float cx = 0.0f, cy = 0.0f;

    for (size_t i = 0; i < numberOfVertices; ++i) {
        const sf::Vector2f & current = m_points[i];
        const sf::Vector2f & next = m_points[(i + 1) % numberOfVertices];

        float crossProduct = current.x * next.y - next.x * current.y;
        cx += (current.x + next.x) * crossProduct;
        cy += (current.y + next.y) * crossProduct;
    }

    cx /= (6.0f * signedArea);
    cy /= (6.0f * signedArea);

    return sf::Vector2f(cx, cy);
}
