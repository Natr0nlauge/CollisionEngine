#pragma once

#include "sfml/Graphics.hpp"

namespace sfu {
/**
* @class Matrix2f
 * @brief A matrix compatible with sf::Vector2f.
 */
class Matrix2f {
  public:
    float m[2][2];

    Matrix2f();
    Matrix2f(float m11, float m12, float m21, float m22);

    sf::Vector2f multiply(sf::Vector2f i_vector) const;
    // float getDeterminant();
    // Matrix2f getInverse();
};

} // namespace sfu