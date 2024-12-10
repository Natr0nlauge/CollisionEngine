#include "RigidBody.hpp"



RigidBody::RigidBody() : Shape() {

}

RigidBody::~RigidBody() {

}






//placeholder
sf::Vector2f Polygon::getPoint(std::size_t index) const {
    if (index < m_points.size()) {
        return m_points[index];
    }
    return sf::Vector2f(0.f, 0.f); // Fallback (shouldn't happen if used correctly)
}

//placeholder
std::size_t Polygon::getPointCount() const {
	return m_points.size();
}

Ball::Ball() {

}

Ball::~Ball() {

}

Polygon::Polygon()
{
}

Polygon::~Polygon()
{
}
