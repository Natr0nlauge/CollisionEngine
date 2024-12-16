#include "Polygon.hpp"

sf::Vector2f Polygon::getNormal(int i_index)
{
	//std::vector<sf::Vector2f> normals;
	//for (int i = 0; i < m_points.size(); i++) {
	sf::Vector2f edge;
	//subtract vertices
	if (i_index == (m_points.size() - 1)) {
		edge = sf::Vector2f(m_points[i_index].x - m_points[0].x, m_points[i_index].y - m_points[0].y);
	}
	else
	{
		edge = sf::Vector2f(m_points[i_index].x - m_points[i_index + 1].x, m_points[i_index].y - m_points[i_index + 1].y);
	}
	float vectorLength = sqrt(pow(edge.x, 2) + pow(edge.y, 2));
	sf::Vector2f normal = sf::Vector2f(edge.y / vectorLength, -edge.x / vectorLength);
	//normals.push_back(normal);

	return normal;
}

sf::Vector2f Polygon::getGlobalNormal(int i_index)
{
	sf::Vector2f normal;
	normal = transformVectorToGlobal(getNormal(i_index));

	return normal;
}

Polygon::Polygon() {
}

Polygon::Polygon(std::vector<sf::Vector2f> i_vertices) : m_points(i_vertices) {
}

Polygon::~Polygon() {
}


sf::Vector2f Polygon::getPoint(std::size_t i_index) const {
	if (i_index < m_points.size()) {
		return m_points[i_index];
	}
	return sf::Vector2f(0.f, 0.f); // Fallback (shouldn't happen if used correctly)
}

std::size_t Polygon::getPointCount() const {
	return m_points.size();
}

std::vector<sf::Vector2f> Polygon::getPoints() {
	return m_points;
}

sf::Vector2f Polygon::getGlobalPoint(int i_index)
{
		sf::Vector2f transformedPoint = transformPointToGlobal(m_points[i_index]);
	
	return transformedPoint;
}
