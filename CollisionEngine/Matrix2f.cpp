#include "Matrix2f.hpp"

sfu::Matrix2f::Matrix2f() : m{{1, 0}, {0, 1}} {};
sfu::Matrix2f::Matrix2f(float m11, float m12, float m21, float m22) : m{{m11, m12}, {m21, m22}} {};

// Multiply with vector
sf::Vector2f sfu::Matrix2f::multiply(sf::Vector2f i_vector) {
    sf::Vector2f outputVec = sf::Vector2f();

    outputVec.x = m[0][0] * i_vector.x + m[0][1] * i_vector.y;
    outputVec.y = m[1][0] * i_vector.x + m[1][1] * i_vector.y;

    return outputVec;
}

// float sfu::Matrix2f::getDeterminant()
//{
//     return m[0][0] * m[1][1] - m[0][1] * m[1][0];
// }
//
// sfu::Matrix2f sfu::Matrix2f::getInverse()
//{
//     float determinant = getDeterminant();
//     if (std::abs(determinant) < 1e-9) { // Check for near-zero determinant
//         throw std::runtime_error("Matrix is singular and cannot be inverted.");
//     }
//
//     float invDet = 1 / determinant;
//
//     return sfu::Matrix2f(
//         m[1][1] * invDet, -m[0][1] * invDet,
//         -m[1][0] * invDet, m[0][0] * invDet
//     );
// }
