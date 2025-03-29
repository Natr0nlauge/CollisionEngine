#pragma once

#include "RigidBody.hpp"

/**
 * @brief Stores data related to the separation between a VertexBasedBody and a single point.
 *
 * This struct holds information about the separation distance,
 * the index of the relevant edge, and the normal vector representing
 * the direction of separation.
 */
struct pointSeparationData_type {
    float separation = std::numeric_limits<float>::lowest(); ///< Separation according to SAT in pixels.
    int index = -1;                                          ///< Index of the edge corresponding to the separation
    sf::Vector2f normal = sf::Vector2f();                    ///< Normal vector corresponding to the separation
};

/**
 * @class VertexBasedBody
 * @brief Represents a RigidBody based on vertices with straight edges.
 *
 * This class models a RigidBody with vertices and straight edges as opposed to a  round body/circle. This is either a BoundaryElement (2
 * vertices) or a polygon (3 or more vertices).
 */
class VertexBasedBody : public RigidBody {
  public:
    // Constructor
    VertexBasedBody(float i_inverseMass, std::vector<sf::Vector2f> i_vertices);

    // Destructor
    virtual ~VertexBasedBody();

    // Getters
    std::vector<sf::Vector2f> getPoints();
    virtual sf::Vector2f getNormal(int i_index) = 0;
    virtual size_t getNormalCount() = 0;
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
