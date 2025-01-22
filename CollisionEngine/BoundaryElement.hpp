#pragma once
#include "Polygon.hpp"

class BoundaryElement : public Polygon {
  public:
    // Constructor
    BoundaryElement(float i_length);

    // Destructor
    ~BoundaryElement();

    // Public methods
    sf::Vertex * getVertexArray() ;
    pointSeparationData_type calculateMinPointSeparation(sf::Vector2f i_point) ;
    sf::Vector2f getNormal(int i_index);

  private:
    sf::Vertex m_vertexArray[2]; // Start and end point
};
