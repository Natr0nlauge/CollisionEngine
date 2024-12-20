#include "Polygon.hpp"
#include <iostream>

sf::Vector2f Polygon::getNormal(int i_index)
{
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

	return normal;
}

sf::Vector2f Polygon::getGlobalNormal(int i_index)
{
	sf::Vector2f normal;
	normal = transformVectorToGlobal(getNormal(i_index));

	return normal;
}

/*Polygon::Polygon(float i_mass) : RigidBody(i_mass) {
}*/

Polygon::Polygon(float i_mass, std::vector<sf::Vector2f> i_vertices) : m_points(i_vertices), RigidBody(i_mass) {
	m_area = calculateArea();
	//setOrigin(calculateCenterOfMass());
	setOrigin(sf::Vector2f(0.0f,0.0f));

	// Redifine all the vertices, so the center of mass is at {0,0}
	sf::Vector2f com = calculateCenterOfMass();
	for (size_t i = 0; i < getPointCount(); ++i) {
		sf::Vector2f& current = m_points[i];
		current.x -= com.x;
		current.y -= com.y;
	}

	m_momentOfInertia = calculateMomentOfInertia();

}

Polygon::~Polygon() {
}

// Calculate polygon are using shoelace formula
float Polygon::calculateSignedArea() {
	size_t numberOfVertices = m_points.size();

	if (numberOfVertices < 3) {
		return 0.0f; // A polygon must have at least 3 vertices
	}
	float area = 0.0f;

	for (size_t i = 0; i < numberOfVertices; i++) {
		// Current vertex
		const sf::Vector2f& current = m_points[i];
		// Next vertex (wrapping around at the end)
		const sf::Vector2f& next = m_points[(i + 1) % numberOfVertices];

		// Add cross-product to the area sum
		area += current.x * next.y - current.y * next.x;
	}

	//std::cout << area / 2.0f;
	return area/2.0f;
}


float Polygon::calculateArea() {
	return std::abs(calculateSignedArea());
}

float Polygon::calculateMomentOfInertia()
{
	if (m_points.size() < 3)
		return 0.0f; // A polygon needs at least 3 points.

	float moi = 0.0f;
	float density = calculateDensity(); // Assuming you have a density method.
	sf::Vector2f centroid = calculateCenterOfMass(); // Precompute the centroid.

	std::size_t n = m_points.size();
	for (std::size_t i = 0; i < n; ++i)
	{
		std::size_t j = (i + 1) % n; // Wrap around to the first vertex.

		sf::Vector2f p1 = m_points[i] - centroid;
		sf::Vector2f p2 = m_points[j] - centroid;

		float cross = p1.x * p2.y - p2.x * p1.y;

		float term = (p1.x * p1.x + p1.x * p2.x + p2.x * p2.x) +
			(p1.y * p1.y + p1.y * p2.y + p2.y * p2.y);

		moi += cross * term;
	}

	moi = std::abs(moi) * density / 12.0f; // Divide by 12 and include density.
	//std::cout << moi << "\n";
	return moi;
}

// Calculate Center of Mass (Centroid)
sf::Vector2f Polygon::calculateCenterOfMass()
{
	size_t numberOfVertices = m_points.size();
	if (numberOfVertices < 3) {
		return sf::Vector2f(0.0f, 0.0f); // A polygon must have at least 3 vertices
	}

	float signedArea = calculateSignedArea();
	float cx = 0.0f, cy = 0.0f;

	for (size_t i = 0; i < numberOfVertices; ++i) {
		const sf::Vector2f& current = m_points[i];
		const sf::Vector2f& next = m_points[(i + 1) % numberOfVertices];

		float crossProduct = current.x * next.y - next.x * current.y;
		cx += (current.x + next.x) * crossProduct;
		cy += (current.y + next.y) * crossProduct;
	}

	cx /= (6.0f * signedArea);
	cy /= (6.0f * signedArea);

	//return { cx, cy };
	//std::cout << cx << ", " << cy << "\n";
	return sf::Vector2f(cx, cy);
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

sf::Vector2f calculateCenterOfMass() {
	return sf::Vector2f(0.0f,0.0f);
}
