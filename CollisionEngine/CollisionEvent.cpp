#include "CollisionEvent.hpp"
#include "sfml_utility.hpp"

float RESTITUTION = 1.0f; // depends on material; 1.0 for collisions without losses
// TODO define this for every body seperately

void CollisionEvent::resolve() {
    float contactTransformationAngle = sfu::getVectorDirection(m_contactNormal); // only one is needed;

    // Get relative position in global coordinates for body 1 and 2
    sf::Vector2f relativePosition1 = computeRelativePosition(m_collisionLocation, m_collisionPartners[0]->getPosition());
    sf::Vector2f relativePosition2 = computeRelativePosition(m_collisionLocation, m_collisionPartners[1]->getPosition());

    float contactVel = calculateContactVelocity(relativePosition1, relativePosition2);

    if (contactVel > 0) { // Avoid bodies getting stuck inside each other
        float desiredDeltaVel = -contactVel * (1 + RESTITUTION);

        float deltaVelPerUnitImpulse = calculateDeltaVelPerUnitImpulse(relativePosition1, relativePosition2);

        float impulseContactX = desiredDeltaVel / deltaVelPerUnitImpulse;

        handleCollision(relativePosition1, relativePosition2, impulseContactX, contactTransformationAngle);

    }
}

sf::Vector2f CollisionEvent::computeRelativePosition(const sf::Vector2f i_collLoc, const sf::Vector2f i_bodyPosition) {
    return sfu::subtractVectors(i_collLoc, i_bodyPosition);
}
float CollisionEvent::calculateContactVelocity(sf::Vector2f i_relativePosition1, sf::Vector2f i_relativePosition2) {
    // Calculate closing velocity at contact point
    float angVel1 = m_collisionPartners[0]->getAngularVelocity() * sfu::PI / 180;
    float angVel2 = m_collisionPartners[1]->getAngularVelocity() * sfu::PI / 180;
    sf::Vector2f tranVel1 = m_collisionPartners[0]->getVelocity();
    sf::Vector2f tranVel2 = m_collisionPartners[1]->getVelocity();
    // simplification of cross product
    sf::Vector2f closingVel1 = sfu::pseudoCrossProduct2(angVel1, i_relativePosition1);
    sf::Vector2f closingVel2 = sfu::pseudoCrossProduct2(angVel2, i_relativePosition2);
    closingVel1 = sfu::addVectors(closingVel1, tranVel1);
    closingVel2 = sfu::addVectors(closingVel2, tranVel2);

    sf::Vector2f closingVel = sfu::subtractVectors(closingVel1, closingVel2);

    // Projected closing velocity - Are objects moving towards each other or not?
    float closingVel1Proj = sfu::scalarProduct(closingVel1, m_contactNormal);
    float closingVel2Proj = sfu::scalarProduct(closingVel2, sfu::scaleVector(m_contactNormal, -1.0f));
    // std::cout << "Normal in CollisionResolver: " << c_collEvent.normal2.x << ", " << c_collEvent.normal2.y << "\n";
    return closingVel1Proj + closingVel2Proj;
    // std::cout << contactVel << "\n";
}

float CollisionEvent::calculateDeltaVelPerUnitImpulse(sf::Vector2f i_relativePosition1, sf::Vector2f i_relativePosition2) {
    // Implement the three equations, see p. 338
    float torquePerUnitImpulse1 = sfu::pseudoCrossProduct(i_relativePosition1, m_contactNormal);
    float torquePerUnitImpulse2 = sfu::pseudoCrossProduct(i_relativePosition2, sfu::scaleVector(m_contactNormal, -1.0f));
    float angVelPerUnitImpulse1 = m_collisionPartners[0]->getInverseMomentOfInertia() * torquePerUnitImpulse1;
    float angVelPerUnitImpulse2 = m_collisionPartners[1]->getInverseMomentOfInertia() * torquePerUnitImpulse2;
    sf::Vector2f velocityPerUnitImpulse1 = sfu::pseudoCrossProduct2(angVelPerUnitImpulse1, i_relativePosition1);
    sf::Vector2f velocityPerUnitImpulse2 = sfu::pseudoCrossProduct2(angVelPerUnitImpulse2, i_relativePosition2);

    // Total velocity change per unit impulse
    // Splitting into summands for debugging
    float deltaVelSummand1 = sfu::scalarProduct(velocityPerUnitImpulse1, m_contactNormal);
    float deltaVelSummand2 = sfu::scalarProduct(velocityPerUnitImpulse2, sfu::scaleVector(m_contactNormal, -1.0f));

    float deltaVelSummand3 = m_collisionPartners[0]->getInverseMass();
    float deltaVelSummand4 = m_collisionPartners[1]->getInverseMass();

    return deltaVelSummand1 + deltaVelSummand2 + deltaVelSummand3 + deltaVelSummand4;
}

void CollisionEvent::handleCollision(sf::Vector2f i_relativePosition1, sf::Vector2f i_relativePosition2, float i_impulseContactX,
        float i_contactTransformationAngle) {

sf::Vector2f impulseContact = sf::Vector2f(i_impulseContactX, 0.0f);

    // In global coordinates
    sf::Vector2f impulse1 = sfu::rotateVector(impulseContact, i_contactTransformationAngle);
    sf::Vector2f impulse2 = sfu::scaleVector(impulse1, -1.0f); // Newton's third law

    sf::Vector2f velocityChange1 = sfu::scaleVector(impulse1, m_collisionPartners[0]->getInverseMass());
    sf::Vector2f velocityChange2 = sfu::scaleVector(impulse2, m_collisionPartners[1]->getInverseMass());

    float impulsiveTorque1 = sfu::pseudoCrossProduct(i_relativePosition1, impulse1);
    float impulsiveTorque2 = sfu::pseudoCrossProduct(i_relativePosition2, impulse2);
    float angularVelocityChange1 = m_collisionPartners[0]->getInverseMomentOfInertia() * impulsiveTorque1;
    float angularVelocityChange2 = m_collisionPartners[1]->getInverseMomentOfInertia() * impulsiveTorque2;

    sf::Vector2f newVel1 = sfu::addVectors(m_collisionPartners[0]->getVelocity(), velocityChange1);
    float newAngVel1 = m_collisionPartners[0]->getAngularVelocity() + angularVelocityChange1 * 180 / sfu::PI;
    sf::Vector2f newVel2 = sfu::addVectors(m_collisionPartners[1]->getVelocity(), velocityChange2);
    float newAngVel2 = m_collisionPartners[1]->getAngularVelocity() + angularVelocityChange2 * 180 / sfu::PI;
    m_collisionPartners[0]->setVelocity(newVel1);
    m_collisionPartners[1]->setVelocity(newVel2);
    m_collisionPartners[0]->setAngularVelocity(newAngVel1);
    m_collisionPartners[1]->setAngularVelocity(newAngVel2);
    // std::cout << newVel1.x << ", " << newVel1.y << ", " << newVel2.x << ", " << newVel2.y << "\n";


}


