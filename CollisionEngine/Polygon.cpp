#include "Polygon.hpp"

Polygon::Polygon(){
}

Polygon::Polygon(std::vector<sf::Vector2f> i_vertices) : m_points(i_vertices){
}

Polygon::~Polygon(){
}


//placeholder
sf::Vector2f Polygon::getPoint(std::size_t i_index) const {
	if (i_index < m_points.size()) {
		return m_points[i_index];
	}
	return sf::Vector2f(0.f, 0.f); // Fallback (shouldn't happen if used correctly)
}

//placeholder
std::size_t Polygon::getPointCount() const {
	return m_points.size();
}

std::vector<sf::Vector2f> Polygon::getPoints() {
	return m_points;
}
