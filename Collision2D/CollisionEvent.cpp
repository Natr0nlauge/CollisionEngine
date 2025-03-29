#include "CollisionEvent.hpp"
#include "sfml_utility.hpp"
#include <iostream>

const int BODIES_PER_COLLISION = 2;

// Constructor
CollisionEvent::CollisionEvent(RigidBody * i_rb1, RigidBody * i_rb2, const collisionGeometry & i_cg)
    : m_collisionPartners{i_rb1, i_rb2}, m_collisionGeometry(i_cg) {}

CollisionEvent::~CollisionEvent() {}

/**
 * @brief Resolve the collision and apply the translational and rotational velocities to the respective bodies.
 */
void CollisionEvent::resolve() {
    float contactTransformationAngle = sfu::getVectorDirection(m_collisionGeometry.normals[0]); // only one is needed;

    // Get relative collision location in global coordinates for body 1 and 2
    std::array<sf::Vector2f, 2> relativePositions = {
            computeRelativePosition(m_collisionGeometry.location, m_collisionPartners[0]->getPosition()),
            computeRelativePosition(m_collisionGeometry.location, m_collisionPartners[1]->getPosition())};

    float contactSpeed = calculateContactSpeed(relativePositions);

    // Check contact speed to avoid bodies getting stuck inside each other
    if (contactSpeed > 0) { 
        // Restitution coefficient determines how much bodies will "bounce back"
        float restitutionCoefficient =
                m_collisionPartners[0]->getRestitutionCoefficient() * m_collisionPartners[1]->getRestitutionCoefficient();
        float desiredDeltaVel = -contactSpeed * (1 + restitutionCoefficient);

        float deltaVelPerUnitImpulse = 1.0f; // arbitrary value !=0

        try {
            deltaVelPerUnitImpulse = calculateDeltaVelPerUnitImpulse(relativePositions); // throws error if two bodies with infinite mass collide
        } catch (const std::runtime_error & e) {
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
        // Calculate and handle impulse vector
        float impulseContactX = desiredDeltaVel / deltaVelPerUnitImpulse;
        handleImpulse(relativePositions, impulseContactX, contactTransformationAngle);
    }
}

collisionGeometry CollisionEvent::getCollisionGeometry() const {
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
 * @brief Calculates the speed at which the bodies are colliding in the contact point (considering translational and rotational body
 * movement)
 * @param i_relativePositions Array containing the collision location in relation to the center of mass of the respective body (in global
 * coordinates)
 * @return The collision speed (the component of the local relative velocity vector parallel to the collision normal)
 */
float CollisionEvent::calculateContactSpeed(std::array<sf::Vector2f, 2> i_relativePositions) const {
    float projectedClosingSpeed = 0.0f;
    for (int i = 0; i < BODIES_PER_COLLISION; i++) {
        float angVel = m_collisionPartners[i]->getAngularVelocity() * sfu::PI / 180; // in rad/s
        sf::Vector2f tranVel = m_collisionPartners[i]->getVelocity(); // in pixel/s
        // Local velocity vector considering angular and translational velocity
        sf::Vector2f closingVel = sfu::pseudoCrossProduct(angVel, i_relativePositions[i]);
        closingVel = sfu::addVectors(closingVel, tranVel);
        // Project velocity to collision normal vector (lateral component is ignored as we assume frictionless collisions)
        projectedClosingSpeed += sfu::scalarProduct(closingVel, m_collisionGeometry.normals[i]);
    }
    return projectedClosingSpeed;
}

/**
 * @brief Calculates how much the local velocity will change if a unit impulse applies. Depends on mass and moment of inertia of BOTH bodies.
 * @param i_relativePositions Array holding the relative collision location for each body in global coordinates.
 * @return The velocity change per unit impulse.
 */
float CollisionEvent::calculateDeltaVelPerUnitImpulse(std::array<sf::Vector2f, 2> i_relativePositions) const {
    float deltaVel = 0.0f;

    for (int i = 0; i < BODIES_PER_COLLISION; i++) {
        // Rotational component
        float torquePerUnitImpulse = sfu::pseudoCrossProduct(i_relativePositions[i], m_collisionGeometry.normals[i]);
        float angVelPerUnitImpulse = m_collisionPartners[i]->getInverseMomentOfInertia() * torquePerUnitImpulse;
        // Translational component
        sf::Vector2f velocityPerUnitImpulse1 = sfu::pseudoCrossProduct(angVelPerUnitImpulse, i_relativePositions[i]);
        // Combine rotational and translational components
        deltaVel += sfu::scalarProduct(velocityPerUnitImpulse1, m_collisionGeometry.normals[i]);
        deltaVel += m_collisionPartners[i]->getInverseMass();
    }
    if (deltaVel <= 0) {
        // This would mean infinite inertia
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
