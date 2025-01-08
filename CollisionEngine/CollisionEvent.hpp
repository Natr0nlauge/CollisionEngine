#pragma once
#include "RigidBody.hpp"

class CollisionEvent {

  public:
    // Constructor
    CollisionEvent(RigidBody * i_rb1, RigidBody * i_rb2) : m_collisionPartners{i_rb1, i_rb2} {}

    // Public members
    RigidBody * m_collisionPartners[2];
    sf::Vector2f m_collisionLocation;
    sf::Vector2f m_contactNormal; // Per definition: The normal of collision partner 1

    // Public methods
    void resolve();

  private:
      // TODO make inputs arrays wherever possible
    // Private methods
    sf::Vector2f computeRelativePosition(const sf::Vector2f i_collLoc, const sf::Vector2f i_bodyPosition);
    float calculateContactVelocity(sf::Vector2f i_relativePosition1, sf::Vector2f i_relativePosition2);
    float calculateDeltaVelPerUnitImpulse(sf::Vector2f i_relativePosition1, sf::Vector2f i_relativePosition2);
    void handleCollision(sf::Vector2f i_relativePosition1, sf::Vector2f i_relativePosition2, float i_impulseContactX,
            float i_contactTransformationAngle);
};
