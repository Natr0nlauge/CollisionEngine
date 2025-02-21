#pragma once

#include "sfml/Graphics.hpp"
#include "Matrix2f.hpp"

/**
 * @brief Some utility functions to interact with sf::Vector2f.
 */
namespace sfu {
const float PI = 3.14159265358979323846;

sfu::Matrix2f getRotationMatrix(float angle);

float getVectorLength(sf::Vector2f i_vector);

float getVectorDirection(sf::Vector2f i_vector);

sf::Vector2f scaleVector(sf::Vector2f i_vector, float factor);

sf::Vector2f normalizeVector(sf::Vector2f i_vector);

sf::Vector2f addVectors(sf::Vector2f i_vec1, sf::Vector2f i_vec2);

sf::Vector2f subtractVectors(sf::Vector2f vec1, sf::Vector2f vec2);

float scalarProduct(sf::Vector2f i_vec1, sf::Vector2f i_vec2);

float pseudoCrossProduct(sf::Vector2f i_vec1, sf::Vector2f i_vec2);

sf::Vector2f pseudoCrossProduct(float i_length1, sf::Vector2f i_vec2);

sf::Vector2f rotateVector(sf::Vector2f i_vector, float i_angle);

sf::Vector2f transformPoint(sf::Vector2f i_vector, sf::Vector2f i_origin, float i_angle);

void printVectorCoords(sf::Vector2f i_vector);

} // namespace sfu