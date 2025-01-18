#include "RigidBody.hpp"
#include "sfml_utility.hpp"

const float PI = 3.14159265358979323846;

RigidBody::RigidBody(float i_inverseMass) : Shape(), m_inverseMass(i_inverseMass) {}

RigidBody::~RigidBody() {}

std::size_t RigidBody::getPointCount() const {
    return m_points.size();
}

sf::Vector2f RigidBody::getPoint(std::size_t i_index) const {
    if (i_index < m_points.size()) {
        return m_points[i_index];
    }
    return sf::Vector2f(0.f, 0.f); // Fallback (shouldn't happen if used correctly)
}

float RigidBody::getInverseMass() const {
    return m_inverseMass;
}

float RigidBody::getInverseMomentOfInertia() const {
    return m_inverseMomentOfInertia;
}

sf::Vector2f RigidBody::getVelocity() const {
    return m_velocity;
}

// In degrees per second
float RigidBody::getAngularVelocity() const {
    return m_angularVelocity;
}

void RigidBody::setVelocity(sf::Vector2f i_newVel) {
    m_velocity = i_newVel;
}

// In degrees per second
void RigidBody::setAngularVelocity(float i_newAngVel) {
    m_angularVelocity = i_newAngVel;
}

void RigidBody::updatePositionAndAngle(float i_dT) {
    move(m_velocity.x * i_dT, m_velocity.y * i_dT);
    rotate(m_angularVelocity * i_dT);
}

// Local to global
sf::Vector2f RigidBody::transformPointToGlobal(sf::Vector2f i_localPoint) {
    // Body position will be the new origin
    sf::Vector2f localOrigin = getPosition();
    float angle = getRotation();
    return sfu::transformPoint(i_localPoint, localOrigin, angle);
}

// Local to global
sf::Vector2f RigidBody::transformVectorToGlobal(sf::Vector2f i_localVector) {
    // Multiply with rotation matrix
    float angle = getRotation();
    return sfu::rotateVector(i_localVector, angle);
}

float RigidBody::calculateInverseDensity() const {
    return m_area * m_inverseMass;
}
