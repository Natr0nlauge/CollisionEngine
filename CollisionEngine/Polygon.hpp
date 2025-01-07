#pragma once

#include "RigidBody.hpp"

class Polygon : public RigidBody {
  public:
    // Constructor
    Polygon(float i_inverseMass, std::vector<sf::Vector2f> vertices);

    // Destructor
    ~Polygon();

    // Getters
    std::size_t getPointCount() const;
    sf::Vector2f getPoint(std::size_t index) const;
    std::vector<sf::Vector2f> getPoints();
    sf::Vector2f getNormal(int i_index);
    sf::Vector2f getGlobalPoint(int i_index);
    sf::Vector2f getGlobalNormal(int i_index);

  private:
    // Utility methods
    float calculateSignedArea();
    float calculateArea() override;
    float calculateInverseMomentOfInertia();
    sf::Vector2f calculateCenterOfMass();

    // Member variables
    std::vector<sf::Vector2f> m_points = {sf::Vector2f(25.0f, 25.0f), sf::Vector2f(-25.0f, 25.0f), sf::Vector2f(-50.0f, 0.0f),
            sf::Vector2f(-25.0f, -25.0f), sf::Vector2f(25.0f, -25.0f)};
};
