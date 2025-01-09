#pragma once
#include "RigidBody.hpp"

class CollisionEvent {

  public:
    // Constructor
    CollisionEvent(RigidBody * i_rb1, RigidBody * i_rb2) : m_collisionPartners{i_rb1, i_rb2} {}

    // Public members
    RigidBody * m_collisionPartners[2];
    sf::Vector2f m_collisionLocation; // In global coordinates
    sf::Vector2f m_contactNormals[2];

    // Public methods
    void resolve();

  private:
      // TODO make inputs arrays wherever possible
    // Private methods
    sf::Vector2f computeRelativePosition(sf::Vector2f i_collLoc, sf::Vector2f i_bodyPosition);
    float calculateContactVelocity(sf::Vector2f * i_relativePositions) const;
    float calculateDeltaVelPerUnitImpulse(sf::Vector2f * i_relativePositions) const;
    void handleCollision(sf::Vector2f * i_relativePosition, float i_impulseContactX,
            float i_contactTransformationAngle);
    void applyImpulse(RigidBody * c_collisionPartner, sf::Vector2f i_relativePosition, sf::Vector2f i_impulse);
};
