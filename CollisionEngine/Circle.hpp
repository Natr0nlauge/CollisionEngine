#pragma once

#include "RigidBody.hpp"

class Circle : public RigidBody {
  public:
    // Constructor and destructor
    Circle(float i_inverseMass = 0.1, float i_radius = 25.0f, int i_resolution = 12);
    ~Circle();

    // Getters
    float getRadius() const;

  private:
    // Override virtual methods
    void calculateAndSetArea() override;
    float calculateInverseMomentOfInertia() override;
    sf::Vector2f calculateCenterOfMass() override;
    void calculatePoints();
    sf::Vector2f calculatePoint(float angle) const;

    // Private member variables:
    float m_radius;
    // std::vector<sf::Vector2f> m_points;
    int m_resolution;
};
