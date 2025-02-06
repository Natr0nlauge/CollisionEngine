#include "sfml_utility.hpp"
#include "CollisionDetector.hpp"
#include <array>



float sfu::computeMedian(const std::array<float, 4> & i_arr) {
    // Make a copy of the array because we need to sort it
    std::array<float, 4> sortedArr = i_arr;
    std::sort(sortedArr.begin(), sortedArr.end());

    // Compute and return the median (average of the two middle elements)
    return (sortedArr[1] + sortedArr[2]) / 2.0f;
}

sfu::Matrix2f sfu::getRotationMatrix(float i_angle) {
    i_angle = i_angle * PI / 180;
    float cosA = std::cos(i_angle);
    float sinA = std::sin(i_angle);
    return Matrix2f(cosA, -sinA, sinA, cosA);
}

float sfu::getVectorLength(sf::Vector2f i_vector) {
    return sqrt(i_vector.x * i_vector.x + i_vector.y * i_vector.y);
}

float sfu::getVectorDirection(sf::Vector2f i_vector) {
    return std::atan2(i_vector.y, i_vector.x) * 180 / PI;
}

sf::Vector2f sfu::scaleVector(sf::Vector2f i_vector, float factor) {
    return sf::Vector2f(i_vector.x * factor, i_vector.y * factor);
}

sf::Vector2f sfu::normalizeVector(sf::Vector2f i_vector) {
    float vectorLength = getVectorLength(i_vector);
    if (vectorLength != 0) {
        return scaleVector(i_vector, 1 / getVectorLength(i_vector));
    } else {
        return sf::Vector2f(0.0f, 0.0f);
    }

}

sf::Vector2f sfu::addVectors(sf::Vector2f i_vec1, sf::Vector2f i_vec2) {
    return sf::Vector2f(i_vec1.x + i_vec2.x, i_vec1.y + i_vec2.y);
}

// vec1 - vec2
sf::Vector2f sfu::subtractVectors(sf::Vector2f i_vec1, sf::Vector2f i_vec2) {
    return sf::Vector2f(i_vec1.x - i_vec2.x, i_vec1.y - i_vec2.y);
}

float sfu::scalarProduct(sf::Vector2f i_vec1, sf::Vector2f i_vec2) {
    return i_vec1.x * i_vec2.x + i_vec1.y * i_vec2.y;
}

float sfu::pseudoCrossProduct(sf::Vector2f i_vec1, sf::Vector2f i_vec2) {
    return i_vec1.x * i_vec2.y - i_vec1.y * i_vec2.x;
}

sf::Vector2f sfu::pseudoCrossProduct(float i_length1, sf::Vector2f i_vec2) {
    return sf::Vector2f(-i_length1 * i_vec2.y, i_length1 * i_vec2.x);
}

sf::Vector2f sfu::rotateVector(sf::Vector2f i_vector, float i_angle) {
    // multiply with rotation matrix
    return sfu::getRotationMatrix(i_angle).multiply(i_vector);
}

sf::Vector2f sfu::transformPoint(sf::Vector2f i_vector, sf::Vector2f i_origin, float i_angle) {
    return sfu::addVectors(rotateVector(i_vector, i_angle), i_origin);
}



