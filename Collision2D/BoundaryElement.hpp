#pragma once
#include "Polygon.hpp"

/**
 * @class BoundaryElement
 * @brief A barrier intended for placing at the outer Boundaries of the simulation area.
 * 
 * @note The Boundary will work in one direction only. If the normal is facing the wrong way, the Simulation will not work correctly.
 *
 * By using boundaries, the bodies will bounce back instead of floating away to infinity. In comparison to a Polygon, it only has one edge
 * and two vertices, so collision detection will be more efficient. It is modeled as having infinite mass and moment of inertia.
 */
class BoundaryElement : public VertexBasedBody {
  public:
    /**
     * @brief Create a boundary element. Adjust position and rotation after creation.
     * 
     * @attention Make sure the normal is facing the right way after positioning the BoundaryElement!
     * 
     * @param i_length Length of the element.
     */
    BoundaryElement(float i_length);

    // Destructor
    ~BoundaryElement();

    // Public methods
    sf::Vertex * getVertexArray();
    pointSeparationData_type calculateMinPointSeparation(sf::Vector2f i_point) override;
    size_t getNormalCount() override;

  private:
    sf::Vector2f getNormal(int i_index) override;
    /// Array containing start and end point in body coordinates
    sf::Vertex m_vertexArray[2];
    /// Length of the element in pixels
    float m_length;
};
