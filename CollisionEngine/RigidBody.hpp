#pragma once

#include <SFML/Graphics.hpp>

class RigidBody : public sf::Shape {
  public:
    RigidBody(float i_mass);
    ~RigidBody();

    // Getters
    float getInverseMass() const;
    float getInverseMomentOfInertia() const;
    sf::Vector2f getVelocity() const;
    float getAngularVelocity() const;

    // Setters
    void setVelocity(sf::Vector2f i_newVel);
    void setAngularVelocity(float i_newAngVel);

    // Public methods
    void updatePositionAndAngle(float i_dT);

    // Override inherited methods
    std::size_t getPointCount() const override;
    sf::Vector2f getPoint(std::size_t index) const override;

  protected:
    // Utility methods
    sf::Vector2f transformPointToGlobal(sf::Vector2f i_localPoint);
    sf::Vector2f transformVectorToGlobal(sf::Vector2f i_localVector);
    float calculateInverseDensity() const;

    

    // Pure virtual methods
    virtual void calculateArea() = 0;
    virtual float calculateInverseMomentOfInertia() = 0;
    virtual sf::Vector2f calculateCenterOfMass() = 0;

    // Member variables
    float m_inverseMass; // inverse mass is more useful ("infinite" mass for immovable objects is easier to implement)
    float m_inverseMomentOfInertia = 0.0f;
    float m_area = 0.0f; // area of 0.0f indicates that it is not yet calculated (this should be avoided)
    sf::Vector2f m_velocity = sf::Vector2f(0.0f, 0.0f);
    float m_angularVelocity = 0.0f;
    std::vector<sf::Vector2f> m_points;
};
