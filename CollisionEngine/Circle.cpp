#include "Circle.hpp"
#include "sfml_utility.hpp"

/**
 * @brief Constructor.
 *
 * @param i_inverseMass Inverse mass. Put in zero for an immovable body.
 * @param i_radius Radius in pixels.
 * @param i_resolution The amount of rendered vertices. Irrelevant for physics.
 */
Circle::Circle(float i_inverseMass, float i_radius, int i_resolution)
    : m_radius(i_radius), m_resolution(i_resolution), RigidBody(i_inverseMass) {
    calculateAndSetArea();
    calculatePoints();
    // setOrigin(calculateCenterOfMass());
    setOrigin(sf::Vector2f(0.0f, 0.0f));
    m_inverseMomentOfInertia = calculateInverseMomentOfInertia();
}

Circle::~Circle() {}

float Circle::getRadius() const {
    return m_radius;
}

/**
 * @brief Calculate the covered area of the body based on the specified radius and set the m_area member variable.
 */
void Circle::calculateAndSetArea() {
    m_area = sfu::PI * m_radius * m_radius;
}

/**
 * @brief Calculate the inverse moment of inertia of the body based on the specified radius.
 * @return The inverse moment of inertia in 1/(mass unit * pixel^2)
 */
float Circle::calculateInverseMomentOfInertia() {
    return 2 * m_inverseMass * 1 / (m_radius * m_radius);
}

/**
 * @brief The center of mass of the Circle is always in the middle.
 * @return {0.0f,0.0f}
 */
sf::Vector2f Circle::calculateCenterOfMass() {
    return sf::Vector2f(0.0f, 0.0f);
}

/**
 * @brief Calculates the coordinates of the points needed for rendering and writes them to m_points (they depend on radius an friction).
 */
void Circle::calculatePoints() {
    float angle = 0.0f;
    for (int i = 0; i < m_resolution; i++) {
        sf::Vector2f newPoint = calculatePoint(angle);
        m_points.push_back(newPoint);
        angle += 2 * sfu::PI / m_resolution;
    }
}

/**
 * @brief Calculates the points needed for rendering and writes them to m_points (they depend on radius an friction).
 */
sf::Vector2f Circle::calculatePoint(float angle) const {
    sf::Vector2f newPoint = sf::Vector2f(cos(angle), sin(angle));
    return sfu::scaleVector(newPoint, m_radius);
}
