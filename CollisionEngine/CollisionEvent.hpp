#pragma once
#include "RigidBody.hpp"
#include <algorithm>

// holds
struct collisionGeometry_type {
    float minSeparation = std::numeric_limits<float>::lowest(); 
    sf::Vector2f location = sf::Vector2f();                          // In global coordinates
    sf::Vector2f normals[2] = {sf::Vector2f(), sf::Vector2f()}; // In global coordinates
};

class CollisionEvent {

  public:
    // Constructor
    CollisionEvent(RigidBody * i_rb1, RigidBody * i_rb2, const collisionGeometry_type & i_cg);

    // Destructor
    ~CollisionEvent();

    // Public methods
    void resolve();
    RigidBody * getCollisionPartner(int i_index) const;
    collisionGeometry_type getCollisionGeometry() const;
    float getMinSeparation() const;
    //void setCollisionGeometry(collisionGeometry_type i_collisionGeometry);

  private:
    // Private methods
    sf::Vector2f computeRelativePosition(sf::Vector2f i_collLoc, sf::Vector2f i_bodyPosition);
    float calculateContactVelocity(sf::Vector2f * i_relativePositions) const;
    float calculateDeltaVelPerUnitImpulse(sf::Vector2f * i_relativePositions) const;
    void handleCollision(sf::Vector2f * i_relativePosition, float i_impulseContactX, float i_contactTransformationAngle);

    // Private members
    RigidBody * m_collisionPartners[2];
    collisionGeometry_type m_collisionGeometry;
};
