#include "CollisionEvent.hpp"
#include "sfml_utility.hpp"
#include <iostream>

const int BODIES_PER_COLLISION = 2;

// Constructor
CollisionEvent::CollisionEvent(RigidBody * i_rb1, RigidBody * i_rb2, const collisionGeometry_type & i_cg)
    : m_collisionPartners{i_rb1, i_rb2}, m_collisionGeometry(i_cg) {}

CollisionEvent::~CollisionEvent() {}

/**
 * @brief Resolve the collision and apply the translational and rotational velocities to the respective bodies.
 */
void CollisionEvent::resolve() {
    float contactTransformationAngle = sfu::getVectorDirection(m_collisionGeometry.normals[0]); // only one is needed;

    // Get relative position in global coordinates for body 1 and 2
    std::array<sf::Vector2f, 2> relativePositions = {
            computeRelativePosition(m_collisionGeometry.location, m_collisionPartners[0]->getPosition()),
            computeRelativePosition(m_collisionGeometry.location, m_collisionPartners[1]->getPosition())};

    float contactVel = calculateContactSpeed(relativePositions);

    if (contactVel > 0) { // Avoid bodies getting stuck inside each other
        float restitutionCoefficient =
                m_collisionPartners[0]->getRestitutionCoefficient() * m_collisionPartners[1]->getRestitutionCoefficient();

        float desiredDeltaVel = -contactVel * (1 + restitutionCoefficient);

        float deltaVelPerUnitImpulse = 1.0f; // arbitrary value !=0

        try {
            deltaVelPerUnitImpulse = calculateDeltaVelPerUnitImpulse(relativePositions);
        } catch (const std::runtime_error & e) {
            // TODO use subfunction
            std::cerr << e.what() << "-> Forcing the bodies away from each other." << "\n";
            for (RigidBody * colPar : m_collisionPartners) {
                sf::Vector2f vel = colPar->getVelocity();
                float angVel = colPar->getAngularVelocity();
                sf::Vector2f newVel = sfu::scaleVector(vel, -1.0f);
                float newAngVel = -1.0f;
                colPar->setVelocity(newVel);
                colPar->setAngularVelocity(newAngVel);
            }
        }
        float impulseContactX = desiredDeltaVel / deltaVelPerUnitImpulse;

        handleImpulse(relativePositions, impulseContactX, contactTransformationAngle);
    }
}

collisionGeometry_type CollisionEvent::getCollisionGeometry() const {
    return m_collisionGeometry;
}

float CollisionEvent::getMinSeparation() const {
    return m_collisionGeometry.minSeparation;
}

/**
 * @brief Calculates the relative position of the collision location to the body location.
 * @param i_collLoc The collision location in global coordinates.
 * @param i_bodyPosition The position of the body's center of mass.
 * @return The relative collision location in global coordinates.
 */
sf::Vector2f CollisionEvent::computeRelativePosition(sf::Vector2f i_collLoc, sf::Vector2f i_bodyPosition) {
    return sfu::subtractVectors(i_collLoc, i_bodyPosition);
}

/**
 * @brief Calculates the speed at which the bodies are colliding in the collision point (considering translational and rotational body
 * movement)
 * @param i_relativePositions Array containing the collision location in relation to the center of mass of the respective body (in global
 * coordinates)
 * @return The collision speed (the component of the local relative velocity vector parallel to the collision normal)
 */
float CollisionEvent::calculateContactSpeed(std::array<sf::Vector2f, 2> i_relativePositions) const {

    float projectedClosingVelocity = 0.0f;

    for (int i = 0; i < BODIES_PER_COLLISION; i++) {
        float angVel = m_collisionPartners[i]->getAngularVelocity() * sfu::PI / 180;
        sf::Vector2f tranVel = m_collisionPartners[i]->getVelocity();
        sf::Vector2f closingVel = sfu::pseudoCrossProduct(angVel, i_relativePositions[i]);
        closingVel = sfu::addVectors(closingVel, tranVel);
        projectedClosingVelocity += sfu::scalarProduct(closingVel, m_collisionGeometry.normals[i]);
    }
    // std::cout << "Normal in CollisionResolver: " << c_collEvent.normal2.x << ", " << c_collEvent.normal2.y << "\n";
    // std::cout << projectedClosingVelocity << "\n";
    return projectedClosingVelocity;
}

float CollisionEvent::calculateDeltaVelPerUnitImpulse(std::array<sf::Vector2f, 2> i_relativePositions) const {
    // Implement the three equations, see p. 338
    float deltaVel = 0.0f;

    for (int i = 0; i < BODIES_PER_COLLISION; i++) {
        float torquePerUnitImpulse = sfu::pseudoCrossProduct(i_relativePositions[i], m_collisionGeometry.normals[i]);
        float angVelPerUnitImpulse = m_collisionPartners[i]->getInverseMomentOfInertia() * torquePerUnitImpulse;
        sf::Vector2f velocityPerUnitImpulse1 = sfu::pseudoCrossProduct(angVelPerUnitImpulse, i_relativePositions[i]);
        deltaVel += sfu::scalarProduct(velocityPerUnitImpulse1, m_collisionGeometry.normals[i]);
        deltaVel += m_collisionPartners[i]->getInverseMass();
    }
    if (deltaVel <= 0) {
        throw std::runtime_error("The collision is impossible! Maybe two bodies with inverseMass = 0 have collided?");
    }
    return deltaVel;
}

/**
 * @brief Determines the impulse each body receives.
 * @param i_relativePosition Array containing the collision location in relation to the center of mass of the respective body (in global
 * coordinates)
 * @param i_impulseContactX The impulse in contact coordinates, parallel to the contact x axis (parallel to the collision normal)
 * @param i_contactTransformationAngle
 *
 */
void CollisionEvent::handleImpulse(std::array<sf::Vector2f, 2> i_relativePosition, float i_impulseContactX,
        float i_contactTransformationAngle) {

    sf::Vector2f impulseContact = sf::Vector2f(i_impulseContactX, 0.0f);

    // In global coordinates
    sf::Vector2f impulse[2];
    impulse[0] = sfu::rotateVector(impulseContact, i_contactTransformationAngle);
    impulse[1] = sfu::scaleVector(impulse[0], -1.0f); // Newton's third law

    for (int i = 0; i < BODIES_PER_COLLISION; i++) {

        m_collisionPartners[i]->applyImpulse(i_relativePosition[i], impulse[i]);
    }
}
