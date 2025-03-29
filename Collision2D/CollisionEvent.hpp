#pragma once
#include "RigidBody.hpp"
#include <algorithm>
#include <array>

/**
 * @brief Holds all gemetrical parameters necessary for resolving the collision.
 *
 * This struct contains the separation, collision location in global coordinates
 *
 * @note It is only considered a collision, if minSeparation<=0.
 */
struct collisionGeometry {
    /// The separation of two bodies according to SAT algorithm (this is not necessarily the actual minimum distance!)
    float minSeparation = std::numeric_limits<float>::max();
    /// Collision location in global coordinates. If there is no collision, this is {0,0}
    sf::Vector2f location = sf::Vector2f();
    /// Contains the normalized collision normal vectors in global coordinates. Note that normals[0] is just normals[1] multiplied by -1.
    sf::Vector2f normals[2] = {sf::Vector2f(), sf::Vector2f()};
};

/**
 * @class CollisionEvent
 * @brief Holds all needed data for modeling a collision and provides methods for resolving it.
 */
class CollisionEvent {

  public:
    CollisionEvent(RigidBody * i_rb1, RigidBody * i_rb2, const collisionGeometry & i_cg);

    // Destructor
    ~CollisionEvent();

    // Public methods
    void resolve();
    collisionGeometry getCollisionGeometry() const;
    float getMinSeparation() const;

  private:
    // Private methods
    sf::Vector2f computeRelativePosition(sf::Vector2f i_collLoc, sf::Vector2f i_bodyPosition);
    float calculateContactSpeed(std::array<sf::Vector2f, 2> i_relativePositions) const;
    float calculateDeltaVelPerUnitImpulse(std::array<sf::Vector2f, 2> i_relativePositions) const;
    void handleImpulse(std::array<sf::Vector2f, 2> i_relativePosition, float i_impulseContactX, float i_contactTransformationAngle);

    // Private members
    std::array<RigidBody *, 2> m_collisionPartners = {nullptr, nullptr};
    collisionGeometry m_collisionGeometry;
};
