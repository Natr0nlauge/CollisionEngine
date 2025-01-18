#pragma once

#include "RigidBody.hpp"

class Polygon : public RigidBody {
  public:
    // Constructor
    Polygon(float i_inverseMass = 0.1, std::vector<sf::Vector2f> i_vertices = {sf::Vector2f(25.0f, 25.0f), sf::Vector2f(-25.0f, 25.0f),
                                               sf::Vector2f(-50.0f, 0.0f), sf::Vector2f(-25.0f, -25.0f), sf::Vector2f(25.0f, -25.0f)});

    // Destructor
    ~Polygon();

    // Getters
    std::vector<sf::Vector2f> getPoints();
    sf::Vector2f getNormal(int i_index);
    sf::Vector2f getGlobalPoint(int i_index);
    sf::Vector2f getGlobalNormal(int i_index);

  private:
    // Utility methods
    float calculateSignedArea();

    // Override virtual methods
    void calculateArea() override;
    float calculateInverseMomentOfInertia() override;
    sf::Vector2f calculateCenterOfMass() override;

};
