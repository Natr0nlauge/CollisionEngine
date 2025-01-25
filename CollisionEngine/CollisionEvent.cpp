#include "CollisionEvent.hpp"
#include "sfml_utility.hpp"
#include <iostream>

const float RESTITUTION = 1.0f; // depends on material; 1.0 for collisions without losses

const int BODIES_PER_COLLISION = 2;

// Constructor
CollisionEvent::CollisionEvent(RigidBody * i_rb1, RigidBody * i_rb2) : m_collisionPartners{i_rb1, i_rb2} {}

CollisionEvent::~CollisionEvent() {}

void CollisionEvent::resolve() {
    float contactTransformationAngle = sfu::getVectorDirection(m_collisionGeometry.normals[0]); // only one is needed;

    // Get relative position in global coordinates for body 1 and 2

    sf::Vector2f relativePositions[2] = {computeRelativePosition(m_collisionGeometry.location, m_collisionPartners[0]->getPosition()),
            computeRelativePosition(m_collisionGeometry.location, m_collisionPartners[1]->getPosition())};

    float contactVel = calculateContactVelocity(relativePositions);

    std::cout << contactVel << "\n";

    if (contactVel > 0) { // Avoid bodies getting stuck inside each other
        float desiredDeltaVel = -contactVel * (1 + RESTITUTION);

        float deltaVelPerUnitImpulse = calculateDeltaVelPerUnitImpulse(relativePositions);

        float impulseContactX = desiredDeltaVel / deltaVelPerUnitImpulse;

        handleCollision(relativePositions, impulseContactX, contactTransformationAngle);
    }
}

RigidBody * CollisionEvent::getCollisionPartner(int i_index) const {
    return m_collisionPartners[i_index];
}

collisionGeometry_type CollisionEvent::getCollisionGeometry() const {
    return m_collisionGeometry;
}

void CollisionEvent::setCollisionGeometry(collisionGeometry_type i_collisionGeometry) {
    m_collisionGeometry = i_collisionGeometry;
}

sf::Vector2f CollisionEvent::computeRelativePosition(sf::Vector2f i_collLoc, sf::Vector2f i_bodyPosition) {
    return sfu::subtractVectors(i_collLoc, i_bodyPosition);
}

float CollisionEvent::calculateContactVelocity(sf::Vector2f * i_relativePositions) const {

    float projectedClosingVelocity = 0.0f;

    for (int i = 0; i < BODIES_PER_COLLISION; i++) {
        float angVel = m_collisionPartners[i]->getAngularVelocity() * sfu::PI / 180;
        sf::Vector2f tranVel = m_collisionPartners[i]->getVelocity();
        sf::Vector2f closingVel = sfu::pseudoCrossProduct2(angVel, i_relativePositions[i]);
        closingVel = sfu::addVectors(closingVel, tranVel);
        projectedClosingVelocity += sfu::scalarProduct(closingVel, m_collisionGeometry.normals[i]);
    }
    // std::cout << "Normal in CollisionResolver: " << c_collEvent.normal2.x << ", " << c_collEvent.normal2.y << "\n";
    // std::cout << projectedClosingVelocity << "\n";
    return projectedClosingVelocity;
}

float CollisionEvent::calculateDeltaVelPerUnitImpulse(sf::Vector2f * i_relativePositions) const {
    // Implement the three equations, see p. 338
    float deltaVel = 0.0f;

    for (int i = 0; i < BODIES_PER_COLLISION; i++) {
        float torquePerUnitImpulse = sfu::pseudoCrossProduct(i_relativePositions[i], m_collisionGeometry.normals[i]);
        float angVelPerUnitImpulse = m_collisionPartners[i]->getInverseMomentOfInertia() * torquePerUnitImpulse;
        sf::Vector2f velocityPerUnitImpulse1 = sfu::pseudoCrossProduct2(angVelPerUnitImpulse, i_relativePositions[i]);
        deltaVel += sfu::scalarProduct(velocityPerUnitImpulse1, m_collisionGeometry.normals[i]);
        deltaVel += m_collisionPartners[i]->getInverseMass();
    }

    return deltaVel;
}

void CollisionEvent::handleCollision(sf::Vector2f * i_relativePosition, float i_impulseContactX, float i_contactTransformationAngle) {

    sf::Vector2f impulseContact = sf::Vector2f(i_impulseContactX, 0.0f);

    // In global coordinates
    sf::Vector2f impulse[2];
    impulse[0] = sfu::rotateVector(impulseContact, i_contactTransformationAngle);
    impulse[1] = sfu::scaleVector(impulse[0], -1.0f);
    // sf::Vector2f impulse2 = sfu::scaleVector(impulse1, -1.0f); // Newton's third law

    for (int i = 0; i < BODIES_PER_COLLISION; i++) {

        applyImpulse(m_collisionPartners[i], i_relativePosition[i], impulse[i]);
    }
    // std::cout << newVel1.x << ", " << newVel1.y << ", " << newVel2.x << ", " << newVel2.y << "\n";
}

void CollisionEvent::applyImpulse(RigidBody * c_collisionPartner, sf::Vector2f i_relativePosition, sf::Vector2f i_impulse) {

    sf::Vector2f velocityChange = sfu::scaleVector(i_impulse, c_collisionPartner->getInverseMass());

    float impulsiveTorque = sfu::pseudoCrossProduct(i_relativePosition, i_impulse);
    float angularVelocityChange = c_collisionPartner->getInverseMomentOfInertia() * impulsiveTorque;

    sf::Vector2f newVel = sfu::addVectors(c_collisionPartner->getVelocity(), velocityChange);
    float newAngVel = c_collisionPartner->getAngularVelocity() + angularVelocityChange * 180 / sfu::PI;
    c_collisionPartner->setVelocity(newVel);
    c_collisionPartner->setAngularVelocity(newAngVel);
}
