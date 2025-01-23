#pragma once
#include "Polygon.hpp"

class BoundaryElement : public EdgeStructure {
  public:
    // Constructor
    BoundaryElement(float i_length);

    // Destructor
    ~BoundaryElement();

    // Public methods
    sf::Vertex * getVertexArray() ;
    pointSeparationData_type calculateMinPointSeparation(sf::Vector2f i_point) override;
    int getNormalCount() override;

  private:
    sf::Vertex m_vertexArray[2]; // Start and end point
    float m_length;
};
