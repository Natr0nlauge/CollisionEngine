#include "Circle.hpp"
#include "sfml_utility.hpp"

Circle::Circle(float i_inverseMass, float i_radius, int i_resolution) : m_radius(i_radius), m_resolution(i_resolution), RigidBody(i_inverseMass) {
    calculateArea();
    calculatePoints();
    // setOrigin(calculateCenterOfMass());
    setOrigin(sf::Vector2f(0.0f, 0.0f));
    m_inverseMomentOfInertia = calculateInverseMomentOfInertia();
}

Circle::~Circle() {}

float Circle::getRadius() const {
    return m_radius;
}

void Circle::calculateArea() {
    m_area = sfu::PI * m_radius * m_radius;
}

float Circle::calculateInverseMomentOfInertia() {
    return 2 * m_inverseMass * 1 / (m_radius * m_radius);
}

sf::Vector2f Circle::calculateCenterOfMass() {
    return sf::Vector2f(0.0f, 0.0f);
}

void Circle::calculatePoints() {
    float angle = 0.0f;
    for (int i = 0; i < m_resolution; i++) {
        sf::Vector2f newPoint = calculatePoint(angle);
        m_points.push_back(newPoint);
        angle += 2 * sfu::PI / m_resolution;
    }
}

sf::Vector2f Circle::calculatePoint(float angle) const {
    sf::Vector2f newPoint = sf::Vector2f(cos(angle), sin(angle));
    return sfu::scaleVector(newPoint, m_radius);
}
