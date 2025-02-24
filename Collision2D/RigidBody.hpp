#pragma once

#include <SFML/Graphics.hpp>

/**
 * @class RigidBody
 * @brief Abstract class which describes the physical behaviour of the simulated bodies. This does not include the geometry of the bodies,
 * which is implemented in the subclasses.
 */
class RigidBody : public sf::Shape {
  public:
    RigidBody(float i_mass);
    virtual ~RigidBody();

    // Getters
    float getInverseMass() const;
    float getInverseMomentOfInertia() const;
    sf::Vector2f getVelocity() const;
    float getAngularVelocity() const;
    float getRestitutionCoefficient() const;
    float getFrictionCoefficient() const;

    // Setters
    void setVelocity(sf::Vector2f i_newVel);
    void setAngularVelocity(float i_newAngVel);
    void setRestitutionCoefficient(float i_restitutionCoefficient);
    void setFrictionCoefficient(float i_frictionCoefficient);

    // Public methods
    void updateBody(float i_dT);
    void applyImpulse(sf::Vector2f i_relativePosition, sf::Vector2f i_impulse);

    // Override inherited methods
    std::size_t getPointCount() const override;
    sf::Vector2f getPoint(std::size_t index) const override;

  protected:
    // Utility methods
    sf::Vector2f transformPointToGlobal(sf::Vector2f i_localPoint);
    sf::Vector2f transformVectorToGlobal(sf::Vector2f i_localVector);
    float calculateInverseDensity() const;

    // Pure virtual methods
    virtual void calculateAndSetArea() = 0;
    virtual float calculateInverseMomentOfInertia() = 0;
    virtual sf::Vector2f calculateCenterOfMass() = 0;

    // Member variables
    /// Mass is inverted to save division operations and to easily depict infinite translational inertia (in this case, the variable will be
    /// zero). Choose any mass unit.
    float m_inverseMass;
    /// Moment of inertia is inverted to save division operations and to easily depict infinite rotational inertia (in this case, the
    /// variable will be zero). Measured in 1/(mass unit * pixel^2)
    float m_inverseMomentOfInertia = 0.0f;
    /// The area covered by the body. Measured in pixel^2
    float m_area = 0.0f;
    /// In pixels per second.
    sf::Vector2f m_velocity = sf::Vector2f(0.0f, 0.0f);
    /// In degrees per second. Clockwise is positive!
    float m_angularVelocity = 0.0f;
    /// The restitution coefficient of a collision is determined by taking this variable from both bodies and multiplying them. This is not
    /// necessarily realistic.
    float m_restitutionCoefficient = 1.0f;
    /// Friction coefficient divided by the time per frame (This is necessary to frame rate interacting with friction). Only affects
    /// movement, not collisions. Measured in 1/s.
    float m_timeNormalizedFrictionCoefficient = 0.005f * 120.0f;
    /// The corners of the rendered body. Should be set in subclasses.
    std::vector<sf::Vector2f> m_points;
};
