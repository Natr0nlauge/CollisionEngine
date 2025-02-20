#pragma once

#include "RigidBody.hpp"

/**
 * @class Circle
 * @brief Circular RigidBody.
 *
 * This is displayed as a Polygon with a lot of corners so it looks "smooth". To avoid iterating through a lot of corners and edges, Circle
 * is a seperate class. Its only geometrical parameter is m_radius (m_resolution only influences appearance but not behaviour).
 *
 */
class Circle : public RigidBody {
  public:
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

    // Private member variables
    /// Radius in pixels.
    float m_radius;
    /// How many vertices the circular shape will have (only changes appearance, doesn't influence the physical behaviour).
    int m_resolution;
};
