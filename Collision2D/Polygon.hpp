#pragma once

#include "VertexBasedBody.hpp"


/**
 * @brief (Convex) Polygonal RigidBody.
 * 
 * @note Simulation is limited to convex bodies, in order to make the SAT algorithm work correctly.
 * 
 */
class Polygon : public VertexBasedBody {
  public:
    // Constructor
    Polygon(float i_inverseMass = 0.1, std::vector<sf::Vector2f> i_vertices = {sf::Vector2f(25.0f, -25.0f), sf::Vector2f(-25.0f, -25.0f),
                                               sf::Vector2f(-25.0f, 25.0f), sf::Vector2f(25.0f, 25.0f)});

    // Destructor
    ~Polygon();
    
    // Public methods
    pointSeparationData_type calculateMinPointSeparation(sf::Vector2f i_point) override;
    int getNormalCount() override;
};
