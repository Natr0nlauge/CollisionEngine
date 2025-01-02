#include "sfmlUtility.h"


float getVectorDirection(sf::Vector2f i_vector) {
	return std::atan2(i_vector.y, i_vector.x) * 180 / 3.14159265358979323846;
}