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

float RigidBody::getRestitutionCoefficient() {
    return m_restitutionCoefficient;
}

float RigidBody::getFrictionCoefficient() {
    return m_frictionCoefficient;
}

void RigidBody::setVelocity(sf::Vector2f i_newVel) {
    m_velocity = i_newVel;
}

// In degrees per second
void RigidBody::setAngularVelocity(float i_newAngVel) {
    m_angularVelocity = i_newAngVel;
}

void RigidBody::setRestitutionCoefficient(float i_restitutionCoefficient) {
    m_restitutionCoefficient = i_restitutionCoefficient;
}

void RigidBody::setFrictionCoefficient(float i_frictionCoefficient) {
    m_frictionCoefficient = i_frictionCoefficient;
}

void RigidBody::updatePositionAndAngle(float i_dT) {
    move(m_velocity.x * i_dT, m_velocity.y * i_dT);
    rotate(m_angularVelocity * i_dT);
    m_velocity = sfu::scaleVector(m_velocity, 1.0f - m_frictionCoefficient);
    m_angularVelocity = m_angularVelocity * (1.0f - m_frictionCoefficient);
}

void RigidBody::applyImpulse(sf::Vector2f i_relativePosition, sf::Vector2f i_impulse) {

    sf::Vector2f velocityChange = sfu::scaleVector(i_impulse, m_inverseMass);

    float impulsiveTorque = sfu::pseudoCrossProduct(i_relativePosition, i_impulse);
    float angularVelocityChange = m_inverseMomentOfInertia * impulsiveTorque;

    sf::Vector2f newVel = sfu::addVectors(m_velocity, velocityChange);
    float newAngVel = m_angularVelocity + angularVelocityChange * 180 / sfu::PI;
    setVelocity(newVel);
    setAngularVelocity(newAngVel);
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
