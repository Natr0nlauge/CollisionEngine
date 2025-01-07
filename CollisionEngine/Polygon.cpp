#include "Polygon.hpp"
#include <iostream>

Polygon::Polygon(float i_inverseMass, std::vector<sf::Vector2f> i_vertices) : m_points(i_vertices), RigidBody(i_inverseMass) {
    m_area = calculateArea();
    // setOrigin(calculateCenterOfMass());
    setOrigin(sf::Vector2f(0.0f, 0.0f));

    // Redifine all the vertices, so the center of mass is at {0,0}
    sf::Vector2f com = calculateCenterOfMass();
    for (size_t i = 0; i < getPointCount(); ++i) {
        sf::Vector2f & current = m_points[i];
        current.x -= com.x;
        current.y -= com.y;
    }
    m_inverseMomentOfInertia = calculateInverseMomentOfInertia();
}

Polygon::~Polygon() {}

std::size_t Polygon::getPointCount() const {
    return m_points.size();
}

sf::Vector2f Polygon::getPoint(std::size_t i_index) const {
    if (i_index < m_points.size()) {
        return m_points[i_index];
    }
    return sf::Vector2f(0.f, 0.f); // Fallback (shouldn't happen if used correctly)
}

std::vector<sf::Vector2f> Polygon::getPoints() {
    return m_points;
}

sf::Vector2f Polygon::getNormal(int i_index) {
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

sf::Vector2f Polygon::getGlobalPoint(int i_index) {
    sf::Vector2f transformedPoint = transformPointToGlobal(m_points[i_index]);
    return transformedPoint;
}

sf::Vector2f Polygon::getGlobalNormal(int i_index) {
    sf::Vector2f normal;
    normal = transformVectorToGlobal(getNormal(i_index));

    return normal;
}

// Calculate polygon are using shoelace formula
float Polygon::calculateSignedArea() {
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

    // std::cout << area / 2.0f;
    return area / 2.0f;
}

float Polygon::calculateArea() {
    return std::abs(calculateSignedArea());
}

float Polygon::calculateInverseMomentOfInertia() {
    if (m_points.size() < 3)
        return 0.0f; // A polygon needs at least 3 points

    float numerator = 0.0f;
    float denominator = 0.0f;
    float inverseDensity = calculateInverseDensity();
    float inverseMass = getInverseMass();
    sf::Vector2f centroid = calculateCenterOfMass(); // Precompute the centroid

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
    // moi = std::abs(moi) * density / 12.0f; // Divide by 12 and include density
    // std::cout << moi << "\n";
    return invMoi;
}

// Calculate Center of Mass (Centroid)
sf::Vector2f Polygon::calculateCenterOfMass() {
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

    // return { cx, cy };
    // std::cout << cx << ", " << cy << "\n";
    return sf::Vector2f(cx, cy);
}
