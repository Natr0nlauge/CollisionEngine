#pragma once

#include "RigidBody.hpp"

struct pointSeparationData_type {
    float separation = std::numeric_limits<float>::lowest();
    int index = -1;
    sf::Vector2f normal;
};

class EdgeStructure : public RigidBody{
  public:
    // Constructor
    EdgeStructure(float i_inverseMass, std::vector<sf::Vector2f> i_vertices);

    // Destructor
    ~EdgeStructure();

    // Getters
    std::vector<sf::Vector2f> getPoints();
    sf::Vector2f getNormal(int i_index);
    virtual int getNormalCount() = 0;
    sf::Vector2f getGlobalPoint(int i_index);
    sf::Vector2f getGlobalNormal(int i_index);
    virtual pointSeparationData_type calculateMinPointSeparation(sf::Vector2f i_point) = 0;

  protected:
    // Utility methods
    float calculateSignedArea();

    // Override virtual methods
    void calculateAndSetArea() override;
    float calculateInverseMomentOfInertia() override;
    sf::Vector2f calculateCenterOfMass() override;
};
