#include "RigidBody.hpp"
#include "sfml_utility.hpp"

RigidBody::RigidBody(float i_inverseMass) : Shape(), m_inverseMass(i_inverseMass) {}

RigidBody::~RigidBody() {}

std::size_t RigidBody::getPointCount() const {
    return m_points.size();
}

/**
 * @brief Required by SFML to render bodies.
 * @param i_index The point index.
 * @return Position of a corner in body coordinates-
 */
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
    return m_timeNormalizedFrictionCoefficient;
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

/**
 * @brief Set the friction coefficient for body movement (Doesn't influence collisions, only translational and rotational movement.)
 * @param i_frictionCoefficient The friction coefficient.
 */
void RigidBody::setFrictionCoefficient(float i_frictionCoefficient) {
    m_timeNormalizedFrictionCoefficient = i_frictionCoefficient;
}

/**
 * @brief Move the body one time step ahead with the current velocity and angular velocity. Updates the position and rotation and applies
 * movement friction.
 * 
 * @param i_dT Time increment.
 */
void RigidBody::updateBody(float i_dT) {
    move(m_velocity.x * i_dT, m_velocity.y * i_dT);
    rotate(m_angularVelocity * i_dT);
    m_velocity = sfu::scaleVector(m_velocity, 1.0f - m_timeNormalizedFrictionCoefficient * i_dT);
    m_angularVelocity = m_angularVelocity * (1.0f - m_timeNormalizedFrictionCoefficient * i_dT);
}

/**
 * @brief Apply an impulse to the body.
 * 
 * @param i_relativePosition The position of the impulse relative to the center of gravity.
 * @param i_impulse The impulse to apply.
 */
void RigidBody::applyImpulse(sf::Vector2f i_relativePosition, sf::Vector2f i_impulse) {

    sf::Vector2f velocityChange = sfu::scaleVector(i_impulse, m_inverseMass);

    float impulsiveTorque = sfu::pseudoCrossProduct(i_relativePosition, i_impulse);
    float angularVelocityChange = m_inverseMomentOfInertia * impulsiveTorque;

    sf::Vector2f newVel = sfu::addVectors(m_velocity, velocityChange);
    float newAngVel = m_angularVelocity + angularVelocityChange * 180 / sfu::PI;
    setVelocity(newVel);
    setAngularVelocity(newAngVel);
}

/**
 * @brief Transform a point from body coordinates to global coordinates.
 * @param i_localPoint The point to transform in body coordinates.
 * @return The point in global coordinates.
 */
sf::Vector2f RigidBody::transformPointToGlobal(sf::Vector2f i_localPoint) {
    // Body position will be the new origin
    sf::Vector2f localOrigin = getPosition();
    float angle = getRotation();
    return sfu::transformPoint(i_localPoint, localOrigin, angle);
}

/**
 * @brief Transform a point from body coordinates to global coordinates.
 * @param i_localPoint The point to transform in global coordinates.
 * @return The point in body coordinates.
 */
sf::Vector2f RigidBody::transformVectorToGlobal(sf::Vector2f i_localVector) {
    // Multiply with rotation matrix
    float angle = getRotation();
    return sfu::rotateVector(i_localVector, angle);
}

/**
 * @brief Calculate the inverse density of the body.
 * 
 * @note Calculate area first.
 * 
 * @return The density.
 */
float RigidBody::calculateInverseDensity() const {
    return m_area * m_inverseMass;
}
