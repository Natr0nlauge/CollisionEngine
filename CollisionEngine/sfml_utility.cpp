#include "sfml_utility.hpp"
//#include "CollisionDetector.hpp"
#include <array>
#include <iostream>

/**
 * @brief Construct the 2D direct cosine matrix.
 * @param i_angle The angle in degrees.
 * @return The rotation matrix.
 */
sfu::Matrix2f sfu::getRotationMatrix(float i_angle) {
    i_angle = i_angle * PI / 180;
    float cosA = std::cos(i_angle);
    float sinA = std::sin(i_angle);
    return Matrix2f(cosA, -sinA, sinA, cosA);
}

/**
 * @brief Calculate the length of a 2D vector.
 * @param i_vector The vector.
 * @return The length in pixels.
 */
float sfu::getVectorLength(sf::Vector2f i_vector) {
    return sqrt(i_vector.x * i_vector.x + i_vector.y * i_vector.y);
}

/**
 * @brief Calculate the direction of a 2D vector.
 * @param i_vector The vector.
 * @return The angle in degrees.
 */
float sfu::getVectorDirection(sf::Vector2f i_vector) {
    return std::atan2(i_vector.y, i_vector.x) * 180 / PI;
}

/**
 * @brief Multiply a 2D vector with a scalar value.
 * @param i_vector The vector.
 * @param factor The factor to multiply the vector with.
 * @return The scaled vector.
 */
sf::Vector2f sfu::scaleVector(sf::Vector2f i_vector, float factor) {
    return sf::Vector2f(i_vector.x * factor, i_vector.y * factor);
}

/**
 * @brief Normalize a 2D vector to unit length.
 * @param i_vector The vector.
 * @return The normalized vector.
 */
sf::Vector2f sfu::normalizeVector(sf::Vector2f i_vector) {
    float vectorLength = getVectorLength(i_vector);
    if (vectorLength != 0) {
        return scaleVector(i_vector, 1 / vectorLength);
    } else {
        return sf::Vector2f(0.0f, 0.0f);
    }
}

/**
 * @brief Add two 2D vectors.
 * @param i_vec1 The first vector.
 * @param i_vec2 The second vector.
 * @return The resulting vector.
 */
sf::Vector2f sfu::addVectors(sf::Vector2f i_vec1, sf::Vector2f i_vec2) {
    return sf::Vector2f(i_vec1.x + i_vec2.x, i_vec1.y + i_vec2.y);
}

/**
 * @brief Subtract two 2D vectors.
 * @param i_vec1 The first vector.
 * @param i_vec2 The second vector.
 * @return The resulting vector.
 */
sf::Vector2f sfu::subtractVectors(sf::Vector2f i_vec1, sf::Vector2f i_vec2) {
    return sf::Vector2f(i_vec1.x - i_vec2.x, i_vec1.y - i_vec2.y);
}

/**
 * @brief Calculate the scalar product of two 2D vectors.
 * @param i_vec1 The first vector.
 * @param i_vec2 The second vector.
 * @return The scalar product.
 */
float sfu::scalarProduct(sf::Vector2f i_vec1, sf::Vector2f i_vec2) {
    return i_vec1.x * i_vec2.x + i_vec1.y * i_vec2.y;
}

/**
 * @brief Calculate the pseudo cross product for two vectors.
 * @param i_vec1 The first vector.
 * @param i_vec2 The second vector.
 * @return The z-Component of the resulting vector.
 */
float sfu::pseudoCrossProduct(sf::Vector2f i_vec1, sf::Vector2f i_vec2) {
    return i_vec1.x * i_vec2.y - i_vec1.y * i_vec2.x;
}

/**
 * @brief Calculate the pseudo cross product for two vectors.
 * @param i_length1 The z-Component of the first vector.
 * @param i_vec2 The second vector.
 * @return The resulting vector.
 */
sf::Vector2f sfu::pseudoCrossProduct(float i_length1, sf::Vector2f i_vec2) {
    return sf::Vector2f(-i_length1 * i_vec2.y, i_length1 * i_vec2.x);
}

/**
 * @brief Rotate a vector using multiplication with a DCM.
 * @param i_vector The vector to rotate.
 * @param i_angle The rotation angle.
 * @return The rotated vector.
 */
sf::Vector2f sfu::rotateVector(sf::Vector2f i_vector, float i_angle) {
    // multiply with rotation matrix
    return sfu::getRotationMatrix(i_angle).multiply(i_vector);
}

/**
 * @brief Transform a point to a different coordinate system.
 * @param i_vector The coordinates of the point to transform.
 * @param i_origin The origin of the new coordinate system given in coordinates of the old coordinate system.
 * @param i_angle The angle of the new coordinate system relative to the old coordinate system.
 * @return The coordinates of the transformed point.
 */
sf::Vector2f sfu::transformPoint(sf::Vector2f i_vector, sf::Vector2f i_origin, float i_angle) {
    return sfu::addVectors(rotateVector(i_vector, i_angle), i_origin);
}

/**
 * @brief Prints the x- and y-Coordinate of a 2D vector into the console. Intended mainly for debugging.
 * @param i_vector The vector to print.
 */
void sfu::printVectorCoords(sf::Vector2f i_vector) {
    std::cout << i_vector.x << ", " << i_vector.y << "\n";
}

