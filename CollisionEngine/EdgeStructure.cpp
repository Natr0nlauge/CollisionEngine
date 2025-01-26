#include "EdgeStructure.hpp"
#include "sfml_utility.hpp"

// Constructor
EdgeStructure::EdgeStructure(float i_inverseMass, std::vector<sf::Vector2f> i_vertices) : RigidBody(i_inverseMass) {
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

sf::Vector2f EdgeStructure::getNormal(int i_index) {
    // subtract vertices
    int incrIndex = (i_index+1) % m_points.size();
    //if (i_index == (m_points.size() - 1)) {
    //    edge = sf::Vector2f(m_points[i_index].x - m_points[0].x, m_points[i_index].y - m_points[0].y);
    //} else {
    sf::Vector2f edge = sfu::subtractVectors(m_points[i_index], m_points[incrIndex]);
    //edge = sf::Vector2f(m_points[i_index].x - m_points[i_index + 1].x, m_points[i_index].y - m_points[i_index + 1].y);
    //}
    float vectorLength = sqrt(pow(edge.x, 2) + pow(edge.y, 2));
    sf::Vector2f normal = sfu::rotateVector(edge,-90.0f);
    normal = sfu::normalizeVector(normal);
    return normal;
}

// Destructor
EdgeStructure::~EdgeStructure() {}

// Returns all points of the body
std::vector<sf::Vector2f> EdgeStructure::getPoints() {
    return m_points;
}

// Returns a point according to the index in global coordinates
sf::Vector2f EdgeStructure::getGlobalPoint(int i_index) {
    sf::Vector2f transformedPoint = transformPointToGlobal(m_points[i_index]);
    return transformedPoint;
}

// Returns a normal vector according to the index in global coordinates
sf::Vector2f EdgeStructure::getGlobalNormal(int i_index) {
    sf::Vector2f normal;
    normal = transformVectorToGlobal(getNormal(i_index));

    return normal;
}



// Calculate polygon are using shoelace formula
float EdgeStructure::calculateSignedArea() {
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

void EdgeStructure::calculateAndSetArea() {
    m_area = std::abs(calculateSignedArea());
}

float EdgeStructure::calculateInverseMomentOfInertia() {
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

// Calculate Center of Mass (Centroid)
sf::Vector2f EdgeStructure::calculateCenterOfMass() {
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

