#pragma once

#include "RigidBody.hpp"

struct pointSeparationData_type {
    float separation = std::numeric_limits<float>::lowest();
    int index = -1;
    sf::Vector2f normal;
};

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
    pointSeparationData_type calculateMinPointSeparation(sf::Vector2f i_point);

  private:
    // Utility methods
    float calculateSignedArea();

    // Override virtual methods
    void calculateArea() override;
    float calculateInverseMomentOfInertia() override;
    sf::Vector2f calculateCenterOfMass() override;

};
