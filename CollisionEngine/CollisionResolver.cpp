#include "CollisionResolver.hpp"
#include <iostream>
#include "sfml_utility.hpp"

float RESTITUTION = 1.0f; // depends on material

CollisionResolver * CollisionResolver::s_instance = nullptr;
std::mutex CollisionResolver::mtx;

CollisionResolver * CollisionResolver::getInstance() {
    if (s_instance == nullptr) {
        std::lock_guard<std::mutex> lock(mtx);
        if (s_instance == nullptr) {
            s_instance = new CollisionResolver();
        }
    }
    return s_instance;
}

CollisionResolver::~CollisionResolver() {}

CollisionResolver::CollisionResolver() {}



void CollisionResolver::handleCollision(collisionEvent_type & c_collEvent) {

    float contactTransformationAngle = sfu::getVectorDirection(c_collEvent.normal1); // only one is needed;

    // Get relative position in global coordinates for body 1 and 2
    sf::Vector2f relativePosition1 = computeRelativePosition(c_collEvent.collLoc1, c_collEvent.rBody1.getPosition());
    sf::Vector2f relativePosition2 = computeRelativePosition(c_collEvent.collLoc1, c_collEvent.rBody2.getPosition());
    
    float contactVel = calculateContactVelocity(c_collEvent, relativePosition1, relativePosition2);

    if (contactVel > 0) {         // Avoid bodies getting stuck inside each other
        float desiredDeltaVel = -contactVel * (1 + RESTITUTION);

        float deltaVelPerUnitImpulse = calculateDeltaVelPerUnitImpulse(c_collEvent, relativePosition1, relativePosition2);

        float impulseContactX = desiredDeltaVel / deltaVelPerUnitImpulse;

        handleCollision(c_collEvent, relativePosition1, relativePosition2, impulseContactX, contactTransformationAngle);
        
    } // if contactVel>0
} // end of handleCollision

sf::Vector2f CollisionResolver::computeRelativePosition(const sf::Vector2f i_collLoc, const sf::Vector2f i_bodyPosition) {
    return sfu::subtractVectors(i_collLoc, i_bodyPosition);
}


float CollisionResolver::calculateContactVelocity(const collisionEvent_type & i_collEvent, sf::Vector2f i_relativePosition1,
        sf::Vector2f i_relativePosition2) {

    // Calculate closing velocity at contact point
    float angVel1 = i_collEvent.rBody1.getAngularVelocity() * sfu::PI / 180;
    float angVel2 = i_collEvent.rBody2.getAngularVelocity() * sfu::PI / 180;
    sf::Vector2f tranVel1 = i_collEvent.rBody1.getVelocity();
    sf::Vector2f tranVel2 = i_collEvent.rBody2.getVelocity();
    // simplification of cross product
    sf::Vector2f closingVel1 = sfu::pseudoCrossProduct2(angVel1, i_relativePosition1);
    sf::Vector2f closingVel2 = sfu::pseudoCrossProduct2(angVel2, i_relativePosition2);
    closingVel1 = sfu::addVectors(closingVel1, tranVel1);
    closingVel2 = sfu::addVectors(closingVel2, tranVel2);

    sf::Vector2f closingVel = sfu::subtractVectors(closingVel1, closingVel2);

    // Projected closing velocity - Are objects moving towards each other or not?
    float closingVel1Proj = sfu::scalarProduct(closingVel1, i_collEvent.normal1);
    float closingVel2Proj = sfu::scalarProduct(closingVel2, i_collEvent.normal2);
    // std::cout << "Normal in CollisionResolver: " << c_collEvent.normal2.x << ", " << c_collEvent.normal2.y << "\n";
    return closingVel1Proj + closingVel2Proj;
    // std::cout << contactVel << "\n";
}

float CollisionResolver::calculateDeltaVelPerUnitImpulse(const collisionEvent_type & i_collEvent, sf::Vector2f relativePosition1,
        sf::Vector2f relativePosition2) {

    // Implement the three equations, see p. 338
    float torquePerUnitImpulse1 = sfu::pseudoCrossProduct(relativePosition1, i_collEvent.normal1);
    float torquePerUnitImpulse2 = sfu::pseudoCrossProduct(relativePosition2, i_collEvent.normal2);
    float angVelPerUnitImpulse1 = i_collEvent.rBody1.getInverseMomentOfInertia() * torquePerUnitImpulse1;
    float angVelPerUnitImpulse2 = i_collEvent.rBody2.getInverseMomentOfInertia() * torquePerUnitImpulse2;
    sf::Vector2f velocityPerUnitImpulse1 = sfu::pseudoCrossProduct2(angVelPerUnitImpulse1, relativePosition1);
    sf::Vector2f velocityPerUnitImpulse2 = sfu::pseudoCrossProduct2(angVelPerUnitImpulse2, relativePosition2);

    // Total velocity change per unit impulse
    // Splitting into summands for debugging
    float deltaVelSummand1 = sfu::scalarProduct(velocityPerUnitImpulse1, i_collEvent.normal1);
    float deltaVelSummand2 = sfu::scalarProduct(velocityPerUnitImpulse2, i_collEvent.normal2);

    float deltaVelSummand3 = i_collEvent.rBody1.getInverseMass();
    float deltaVelSummand4 = i_collEvent.rBody2.getInverseMass();

    return deltaVelSummand1 + deltaVelSummand2 + deltaVelSummand3 + deltaVelSummand4;
}

void CollisionResolver::handleCollision(collisionEvent_type & c_collEvent, sf::Vector2f i_relativePosition1, sf::Vector2f i_relativePosition2,
        float i_impulseContactX, float i_contactTransformationAngle) {

    sf::Vector2f impulseContact = sf::Vector2f(i_impulseContactX, 0.0f);

    // In global coordinates
    sf::Vector2f impulse1 = sfu::rotateVector(impulseContact, i_contactTransformationAngle);
    sf::Vector2f impulse2 = sfu::scaleVector(impulse1, -1.0f); // Newton's third law

    sf::Vector2f velocityChange1 = sfu::scaleVector(impulse1, c_collEvent.rBody1.getInverseMass());
    sf::Vector2f velocityChange2 = sfu::scaleVector(impulse2, c_collEvent.rBody2.getInverseMass());

    float impulsiveTorque1 = sfu::pseudoCrossProduct(i_relativePosition1, impulse1);
    float impulsiveTorque2 = sfu::pseudoCrossProduct(i_relativePosition2, impulse2);
    float angularVelocityChange1 = c_collEvent.rBody1.getInverseMomentOfInertia() * impulsiveTorque1;
    float angularVelocityChange2 = c_collEvent.rBody2.getInverseMomentOfInertia() * impulsiveTorque2;

    sf::Vector2f newVel1 = sfu::addVectors(c_collEvent.rBody1.getVelocity(), velocityChange1);
    float newAngVel1 = c_collEvent.rBody1.getAngularVelocity() + angularVelocityChange1 * 180 / sfu::PI;
    sf::Vector2f newVel2 = sfu::addVectors(c_collEvent.rBody2.getVelocity(), velocityChange2);
    float newAngVel2 = c_collEvent.rBody2.getAngularVelocity() + angularVelocityChange2 * 180 / sfu::PI;
    c_collEvent.rBody1.setVelocity(newVel1);
    c_collEvent.rBody2.setVelocity(newVel2);
    c_collEvent.rBody1.setAngularVelocity(newAngVel1);
    c_collEvent.rBody2.setAngularVelocity(newAngVel2);
    // std::cout << newVel1.x << ", " << newVel1.y << ", " << newVel2.x << ", " << newVel2.y << "\n";
}
